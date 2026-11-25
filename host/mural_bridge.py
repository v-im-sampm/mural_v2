#!/usr/bin/env python3
"""Mural v2 bridge: run Mural's web UI on a PC and drive an Arduino Uno + CNC Shield.

The upstream Mural firmware runs on an ESP32 and serves its own web UI. An Uno
can't do that (no WiFi, 2 KB RAM), so this script takes over everything except
the real-time stepping:

  browser  --HTTP-->  mural_bridge.py (this file)  --USB serial-->  Uno (firmware/mural_uno)
           same API as the ESP32 firmware             M/P/J/Z/W protocol

It implements the same HTTP endpoints and setup phases as the ESP32 firmware, so
the unmodified Mural UI (built with scripts/build_ui.sh) works against it.

Usage:
  python3 mural_bridge.py --port /dev/ttyACM0      (Linux)
  python3 mural_bridge.py --port COM3              (Windows)
  python3 mural_bridge.py --simulate               (no hardware, for trying the UI)
Then open http://localhost:8080
"""

import argparse
import email.parser
import email.policy
import json
import math
import mimetypes
import os
import signal
import sys
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

from kinematics import (MOVE_SPEED_STEPS, PRINT_SPEED_STEPS, SAFE_X_FRACTION,
                        SAFE_Y_FRACTION, Geometry)

HERE = Path(__file__).resolve().parent
DATA_DIR = Path.home() / ".mural-uno"
SETTINGS_FILE = DATA_DIR / "settings.json"
COMMANDS_FILE = DATA_DIR / "commands"

PEN_UP_ANGLE = 90
PEN_CAL_STEP_MM = 100.0
INTERPOLATION_INCREMENT_MM = 1.0


def log(*args):
    print(time.strftime("%H:%M:%S"), *args, flush=True)


# ---------------------------------------------------------------------------
# Links to the motion controller
# ---------------------------------------------------------------------------

class LinkError(Exception):
    pass


class SerialLink:
    """Talks to firmware/mural_uno over USB serial."""

    def __init__(self, port, baud=115200):
        try:
            import serial  # pyserial
        except ImportError:
            sys.exit("pyserial is missing: run  pip install pyserial")
        self.ser = serial.Serial(port, baud, timeout=0.5)
        self.lock = threading.Lock()
        log(f"Opened {port}, waiting for the Uno to boot...")
        deadline = time.time() + 6
        while time.time() < deadline:
            line = self.ser.readline().decode(errors="replace").strip()
            if line:
                log("Uno:", line)
            if "ready" in line:
                break
        else:
            log("No banner from the Uno (already running?). Checking with V...")
        reply = self.cmd("V", timeout=3)
        if not reply or "MuralUno" not in reply[0]:
            raise LinkError(f"Unexpected reply to V: {reply!r}. Is firmware/mural_uno flashed?")

    def cmd(self, line, timeout=None):
        """Send one command and wait for ok. Returns the extra lines before ok."""
        with self.lock:
            self.ser.write((line + "\n").encode())
            extra = []
            start = time.time()
            while True:
                raw = self.ser.readline()
                if not raw:
                    if timeout is not None and time.time() - start > timeout:
                        raise LinkError(f"Timeout waiting for reply to {line!r}")
                    continue
                text = raw.decode(errors="replace").strip()
                if text == "ok":
                    return extra
                if text.startswith("err"):
                    raise LinkError(f"{line!r} -> {text}")
                if text.startswith("#"):
                    log("Uno:", text)
                    continue
                extra.append(text)

    def estop(self):
        self.ser.write(b"!")
        self.ser.flush()

    def status(self):
        state, left, right, queued = self.cmd("?", timeout=3)[0].split()
        return state, int(left), int(right), int(queued)


class SimLink:
    """Pretends to be the Uno, with realistic timing, so the UI can be tried without hardware."""

    def __init__(self):
        self.lock = threading.Lock()
        self.left = self.right = 0
        self.busy_until = time.time()
        self.jog = (0, 0)
        self.jog_since = None
        self.jog_speed = PRINT_SPEED_STEPS
        log("Simulation mode: no hardware attached")

    def _settle_jog(self):
        if self.jog_since is not None:
            steps = int((time.time() - self.jog_since) * self.jog_speed)
            self.left += self.jog[0] * steps
            self.right += self.jog[1] * steps
            self.jog_since = time.time()

    def cmd(self, line, timeout=None):
        with self.lock:
            parts = line.split()
            op, args = parts[0].upper(), [int(p) for p in parts[1:]]
            now = time.time()
            if op == "M":
                left, right, speed = args
                duration = max(abs(left - self.left), abs(right - self.right)) / max(speed, 1)
                self.left, self.right = left, right
                # Emulate the firmware's small queue: don't run more than ~1 s ahead.
                if self.busy_until - now > 1.0:
                    time.sleep(self.busy_until - now - 1.0)
                self.busy_until = max(self.busy_until, time.time()) + duration
            elif op == "P":
                self.busy_until = max(self.busy_until, now) + 0.6
            elif op == "W":
                time.sleep(max(0.0, self.busy_until - now))
            elif op == "J":
                self._settle_jog()
                self.jog = (args[0], args[1])
                self.jog_speed = args[2] if len(args) > 2 else PRINT_SPEED_STEPS
                self.jog_since = now if any(self.jog) else None
            elif op == "Z":
                self.left, self.right = args
            elif op == "?":
                self._settle_jog()
                state = "Jog" if any(self.jog) else ("Run" if self.busy_until > now else "Idle")
                return [f"{state} {self.left} {self.right} 0"]
            elif op == "V":
                return ["MuralUno sim"]
            return []

    def estop(self):
        self.busy_until = time.time()
        self.jog, self.jog_since = (0, 0), None
        log("Sim: emergency stop")

    def status(self):
        state, left, right, queued = self.cmd("?")[0].split()
        return state, int(left), int(right), int(queued)


# ---------------------------------------------------------------------------
# The machine: Mural's setup phases + drawing runner
# ---------------------------------------------------------------------------

class PhaseError(Exception):
    pass


class Machine:
    def __init__(self, link, microsteps):
        self.link = link
        self.lock = threading.RLock()
        settings = self._load_settings()
        self.geo = Geometry(pulley_diameter=settings.get("pulleyDiameter", Geometry.pulley_diameter),
                            steps_per_rotation=200 * microsteps)
        self.left = self.right = 0      # last commanded absolute motor positions (steps)
        self.x = self.y = None          # pen position in image coordinates (mm)
        self.homed = False
        self.started_homing = False
        self.moving = False
        self.pen_distance = None
        self.jog = [0, 0]
        self.progress = None
        self.stop_requested = False
        self.runner_thread = None
        self.phase = "SetTopDistance"

    # --- settings ----------------------------------------------------------

    @staticmethod
    def _load_settings():
        try:
            return json.loads(SETTINGS_FILE.read_text())
        except (OSError, ValueError):
            return {}

    def _save_settings(self):
        DATA_DIR.mkdir(parents=True, exist_ok=True)
        SETTINGS_FILE.write_text(json.dumps({"pulleyDiameter": self.geo.pulley_diameter}, indent=2))

    # --- state -------------------------------------------------------------

    def state(self):
        g = self.geo
        home_x, home_y = g.home_coordinates()
        state = {
            "phase": self.phase,
            "moving": self.moving,
            "startedHoming": self.started_homing,
            "topDistance": g.top_distance,
            "pulleyDiameter": g.pulley_diameter,
            "safeWidth": g.width if g.top_distance != -1 else -1,
            "safeXFraction": SAFE_X_FRACTION,
            "safeYFraction": SAFE_Y_FRACTION,
            "homeX": home_x,
            "homeY": home_y,
            "freeKb": 1024 * 1024,
        }
        if self.progress is not None:
            state["progress"] = self.progress
        return state

    def require(self, *phases):
        if self.phase not in phases:
            raise PhaseError("Request is not supported by the current server phase")

    def reset(self):
        self.phase = "SetTopDistance"
        self.homed = False
        self.started_homing = False
        self.progress = None

    # --- low-level motion ---------------------------------------------------

    def _sync_position(self):
        _, self.left, self.right, _ = self.link.status()

    def _move_steps(self, left, right, speed):
        """Queue a straight move in motor space; returns its duration in seconds."""
        duration = max(abs(left - self.left), abs(right - self.right)) / speed
        self.link.cmd(f"M {left} {right} {int(speed)}")
        self.left, self.right = left, right
        return duration

    def _linear_travel(self, x, y, speed):
        """Mural's Movement::beginLinearTravel: one un-interpolated move to (x, y)."""
        if self.geo.top_distance == -1 or not self.homed:
            raise PhaseError("not ready")
        if x < 0 or (x - 1) > self.geo.width:
            raise PhaseError(f"Invalid x {x}")
        if y < 0:
            raise PhaseError(f"Invalid y {y}")
        left, right = self.geo.belt_lengths_steps(x, y)
        self.x, self.y = x, y
        return self._move_steps(left, right, speed)

    def _wait_then(self, callback=None):
        """Mark as moving until the Uno reports the queue drained, then run callback."""
        self.moving = True

        def waiter():
            try:
                self.link.cmd("W")
            finally:
                with self.lock:
                    self.moving = False
                    if callback:
                        callback()

        threading.Thread(target=waiter, daemon=True).start()

    # --- endpoints ----------------------------------------------------------

    def set_top_distance(self, distance):
        self.require("SetTopDistance")
        self.geo.set_top_distance(int(float(distance)))
        log(f"Top distance (pin to pin) set to {self.geo.top_distance} mm, "
            f"drawing width {self.geo.width:.0f} mm")
        self.phase = "SvgSelect"

    def set_pulley_diameter(self, diameter):
        self.require("SetTopDistance")
        self.geo.set_pulley_diameter(float(diameter))
        self._save_settings()
        log(f"Pulley diameter set to {self.geo.pulley_diameter:.5f} mm (saved)")

    def set_servo(self, angle):
        self.require("SetTopDistance", "PenCalibration")
        self.link.cmd(f"S {int(float(angle))}")

    def esteps_calibration(self, distance):
        self.require("SetTopDistance")
        if self.moving:
            raise PhaseError("busy")
        self._sync_position()
        steps = self.geo.mm_to_steps(int(float(distance)))
        log(f"E-steps calibration: extending both belts {distance} mm ({steps} steps)")
        self._move_steps(self.left + steps, self.right + steps, MOVE_SPEED_STEPS)
        self._wait_then()

    def motor_command(self, command):
        if self.phase == "PenCalibration":
            return self._pen_calibration_move(command)
        self.require("SetTopDistance", "RetractBelts")
        table = {"l-ret": (0, -1), "l-ext": (0, 1), "l-0": (0, 0),
                 "r-ret": (1, -1), "r-ext": (1, 1), "r-0": (1, 0)}
        if command not in table:
            raise PhaseError("Unsupported command")
        side, direction = table[command]
        self.jog[side] = direction
        self.link.cmd(f"J {self.jog[0]} {self.jog[1]} {PRINT_SPEED_STEPS}")
        if not any(self.jog):
            self._sync_position()
        return "OK"

    def _pen_calibration_move(self, command):
        if command not in ("raise", "lower"):
            raise PhaseError("Unsupported command")
        if self.moving:
            return "0"
        new_y = self.y + (-PEN_CAL_STEP_MM if command == "raise" else PEN_CAL_STEP_MM)
        if new_y < 0:
            return "0"
        log(f"Pen calibration move to {self.x:.1f}, {new_y:.1f}")
        t = self._linear_travel(self.x, new_y, PRINT_SPEED_STEPS)
        self._wait_then()
        return str(int(math.ceil(t)) + 1)

    def done_with_phase(self):
        if self.phase == "RetractBelts":
            if any(self.jog):
                self.jog = [0, 0]
                self.link.cmd("J 0 0")
            self.phase = "ExtendToHome"
        elif self.phase == "BeginDrawing":
            self.reset()
        else:
            raise PhaseError("Request is not supported by the current server phase")

    def extend_to_home(self):
        self.require("ExtendToHome")
        # Belts are fully retracted against the stop screws: that is our origin.
        offset = self.geo.homed_steps_offset()
        self.link.cmd(f"Z {offset} {offset}")
        self.left = self.right = offset
        self.homed = True
        self.started_homing = True
        self.geo.gamma_last = 0.0
        home_x, home_y = self.geo.home_coordinates()
        log(f"Extending to home ({home_x:.0f}, {home_y:.0f})")
        t = self._linear_travel(home_x, home_y, MOVE_SPEED_STEPS)

        def to_pen_calibration():
            if self.phase == "ExtendToHome":
                self.phase = "PenCalibration"

        self._wait_then(to_pen_calibration)
        return str(int(math.ceil(t)) + 1)

    def set_pen_distance(self, angle):
        self.require("PenCalibration")
        self.pen_distance = int(float(angle))
        log(f"Pen-down angle set to {self.pen_distance}")
        self.link.cmd(f"P {PEN_UP_ANGLE}")
        self.link.cmd("W")
        self.phase = "BeginDrawing"

    def start_drawing(self, speed):
        self.require("BeginDrawing")
        if self.runner_thread and self.runner_thread.is_alive():
            raise PhaseError("Already drawing")
        speed = max(300, min(3000, int(float(speed or PRINT_SPEED_STEPS))))
        self.stop_requested = False
        self.runner_thread = threading.Thread(target=self._run_drawing, args=(speed,), daemon=True)
        self.runner_thread.start()

    def stop_drawing(self):
        log("Stop requested")
        self.stop_requested = True
        self.link.estop()

    # --- drawing -----------------------------------------------------------

    def _run_drawing(self, speed):
        try:
            self._draw(speed)
        except Exception as e:  # report and leave the machine in a sane state
            log(f"Drawing aborted: {e}. Position is no longer known: redo the setup screens.")
            try:
                self.link.cmd(f"S {PEN_UP_ANGLE}")
            except Exception:
                pass
            with self.lock:
                self.reset()
        finally:
            self.moving = False

    def _draw(self, speed):
        with open(COMMANDS_FILE) as f:
            lines = [ln.strip() for ln in f if ln.strip()]
        if not lines or not lines[0].startswith("d") or not lines[1].startswith("h"):
            raise PhaseError("Bad commands file (missing d/h header)")
        total = float(lines[0][1:]) or 1.0
        done = 0.0
        self.progress = 0
        self.moving = True
        log(f"Drawing started at {speed} steps/s, {total:.0f} mm to travel")
        home = self.geo.home_coordinates()

        for line in lines[2:] + [f"{home[0]} {home[1]}"]:
            if self.stop_requested:
                raise PhaseError("stopped by user")
            if line.startswith("p"):
                angle = self.pen_distance if line[1:2] == "1" else PEN_UP_ANGLE
                self.link.cmd(f"P {angle}")
                continue
            tx, ty = (float(v) for v in line.split()[:2])
            start = (self.x, self.y)
            # Mural's InterpolatingMovementTask: straight lines in 1 mm increments.
            while (self.x, self.y) != (tx, ty):
                if self.stop_requested:
                    raise PhaseError("stopped by user")
                dist = math.hypot(tx - self.x, ty - self.y)
                if dist <= INTERPOLATION_INCREMENT_MM:
                    nx, ny = tx, ty
                else:
                    k = INTERPOLATION_INCREMENT_MM / dist
                    nx, ny = self.x + k * (tx - self.x), self.y + k * (ty - self.y)
                self._linear_travel(nx, ny, speed)
            done += math.hypot(tx - start[0], ty - start[1])
            pct = min(100, int(done / total * 100))
            if pct != self.progress:
                self.progress = pct
                log(f"Progress: {pct}%")

        self.link.cmd("W")
        log("Drawing finished. Start again from the first screen for the next one.")
        with self.lock:
            self.reset()


# ---------------------------------------------------------------------------
# HTTP server (same endpoints as the ESP32 firmware)
# ---------------------------------------------------------------------------

NO_UI_PAGE = """<!doctype html><meta charset=utf-8><title>Mural bridge</title>
<body style="font-family:sans-serif;max-width:40em;margin:2em auto">
<h1>Mural bridge is running</h1>
<p>The Mural web UI hasn't been built yet. Run <code>scripts/build_ui.sh</code>
(needs Node.js 18+) and restart the bridge, or pass <code>--www</code> pointing at a built UI.</p>
<p>Current state: <a href="/getState">/getState</a></p>"""


def make_handler(machine, www_dir):
    class Handler(BaseHTTPRequestHandler):
        server_version = "MuralBridge/1.0"

        def log_message(self, fmt, *args):
            pass  # keep the console readable; Machine logs what matters

        # --- helpers ---
        def _send(self, status, body, ctype="text/plain"):
            data = body.encode() if isinstance(body, str) else body
            self.send_response(status)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            self.wfile.write(data)

        def _send_state(self):
            self._send(200, json.dumps(machine.state()), "application/json")

        def _form(self):
            length = int(self.headers.get("Content-Length") or 0)
            body = self.rfile.read(length) if length else b""
            return body, {k: v[0] for k, v in parse_qs(body.decode(errors="replace")).items()}

        @staticmethod
        def _param(form, name):
            if name in form:
                return form[name]
            if form:
                return next(iter(form.values()))  # firmware uses getParam(0)
            raise PhaseError(f"missing parameter {name}")

        # --- GET ---
        def do_GET(self):
            path = urlparse(self.path).path
            if path == "/getState":
                return self._send_state()
            if path == "/downloadCommands":
                try:
                    return self._send(200, COMMANDS_FILE.read_bytes())
                except OSError:
                    return self._send(404, "No commands uploaded")
            self._serve_static(path)

        def _serve_static(self, path):
            if www_dir is None or not (www_dir / "index.html").exists():
                return self._send(200, NO_UI_PAGE, "text/html")
            rel = path.lstrip("/") or "index.html"
            target = (www_dir / rel).resolve()
            if www_dir not in target.parents and target != www_dir:
                return self._send(404, "Not found")
            if target.is_dir():
                target = target / "index.html"
            if not target.is_file():
                return self._send(404, "Not found")
            ctype = mimetypes.guess_type(str(target))[0] or "application/octet-stream"
            if target.suffix == ".wasm":
                ctype = "application/wasm"
            self._send(200, target.read_bytes(), ctype)

        # --- POST ---
        def do_POST(self):
            url = urlparse(self.path)
            try:
                if url.path == "/uploadCommands":
                    return self._upload(url)
                _, form = self._form()
                with machine.lock:
                    self._dispatch(url.path, form)
            except PhaseError as e:
                self._send(400, str(e))
            except LinkError as e:
                log(f"Controller error: {e}")
                self._send(500, f"Controller error: {e}")

        def _dispatch(self, path, form):
            m = machine
            if path == "/setTopDistance":
                m.set_top_distance(self._param(form, "distance"))
                return self._send_state()
            if path == "/setPulleyDiameter":
                m.set_pulley_diameter(self._param(form, "diameter"))
                return self._send(200, "OK")
            if path == "/setServo":
                m.set_servo(self._param(form, "angle"))
                return self._send(200, "OK")
            if path == "/estepsCalibration":
                m.esteps_calibration(self._param(form, "distance"))
                return self._send(200, "OK")
            if path == "/command":
                return self._send(200, m.motor_command(self._param(form, "command")))
            if path == "/doneWithPhase":
                m.done_with_phase()
                return self._send_state()
            if path == "/extendToHome":
                return self._send(200, m.extend_to_home())
            if path == "/setPenDistance":
                m.set_pen_distance(self._param(form, "angle"))
                return self._send_state()
            if path == "/run":
                m.start_drawing(form.get("speed"))
                return self._send(200, "OK")
            if path == "/stop":  # bridge-only extra: abort a drawing
                m.stop_drawing()
                return self._send(200, "OK")
            self._send(404, "Not found")

        def _upload(self, url):
            query = {k: v[0] for k, v in parse_qs(url.query).items()}
            body = self.rfile.read(int(self.headers.get("Content-Length") or 0))
            ctype = self.headers.get("Content-Type", "")
            msg = email.parser.BytesParser(policy=email.policy.HTTP).parsebytes(
                b"Content-Type: " + ctype.encode() + b"\r\n\r\n" + body)
            chunk = None
            for part in msg.iter_parts() if msg.is_multipart() else []:
                if part.get_param("name", header="content-disposition") == "commands":
                    chunk = part.get_payload(decode=True)
            if chunk is None:
                raise PhaseError("no 'commands' part in upload")
            with machine.lock:
                machine.require("SvgSelect")
                offset = int(query.get("offset", 0))
                DATA_DIR.mkdir(parents=True, exist_ok=True)
                mode = "wb" if offset == 0 else "r+b"
                with open(COMMANDS_FILE, mode) as f:
                    f.seek(offset)
                    f.write(chunk)
                    f.truncate()
                if query.get("final") == "true":
                    log(f"Drawing uploaded ({COMMANDS_FILE.stat().st_size} bytes)")
                    machine.phase = "RetractBelts"
                self._send_state()

    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--port", help="Uno serial port, e.g. /dev/ttyACM0, /dev/cu.usbmodem1101 or COM3")
    src.add_argument("--simulate", action="store_true", help="run without hardware")
    ap.add_argument("--http-port", type=int, default=8080)
    ap.add_argument("--listen", default="127.0.0.1",
                    help="address to listen on; use 0.0.0.0 to control Mural from a phone on your network")
    ap.add_argument("--www", type=Path, default=HERE / "www", help="directory with the built Mural UI")
    ap.add_argument("--microsteps", type=int, default=8, help="must match the jumpers under the drivers")
    args = ap.parse_args()

    link = SimLink() if args.simulate else SerialLink(args.port)
    machine = Machine(link, args.microsteps)
    www = args.www.resolve() if args.www else None
    if not (www and (www / "index.html").exists()):
        log(f"WARNING: no built UI in {www}. Run scripts/build_ui.sh first.")

    server = ThreadingHTTPServer((args.listen, args.http_port), make_handler(machine, www))

    def shutdown(*_):
        log("Shutting down: stopping motors, pen up")
        try:
            link.estop()
            link.cmd(f"S {PEN_UP_ANGLE}", timeout=2)
        except Exception:
            pass
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, shutdown)
    host = "localhost" if args.listen in ("127.0.0.1", "0.0.0.0") else args.listen
    log(f"Mural bridge ready: open http://{host}:{args.http_port}  (Ctrl+C stops the robot)")
    server.serve_forever()


if __name__ == "__main__":
    main()
