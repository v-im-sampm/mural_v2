# Mural v2 on an Arduino Uno + CNC Shield

This runs a [Mural v2](https://getmural.me) wall-drawing robot with an **Arduino Uno + CNC Shield V3**
instead of the ESP32 "TMC2209 Pen/Laser FluidNC" controller from the official BOM.

## Why this needs a PC

Stock Mural firmware runs entirely on an ESP32. That chip hosts the WiFi web UI,
stores the drawing on an SD card, does the kinematics, and drives the motors. An Uno
has no WiFi and only 2 KB of RAM, so it can't run that firmware. This project splits
the job in two:

```
 Browser (Mural UI) ──HTTP──▶ host/mural_bridge.py on your PC ──USB cable──▶ Uno + CNC Shield ──▶ motors, servo
                               • serves the stock Mural UI                     • firmware/mural_uno
                               • Mural's kinematic model (exact port)          • coordinated stepping
                               • setup phases, drawing runner                  • pen servo
```

You get the **same UI and the same setup flow** as stock Mural, and the same motion math.
The trade-off is that **the robot stays tethered to a computer by a USB cable** while it draws.

> If you'd rather have stock Mural (wireless, standalone, official support), the ESP32
> FluidNC pen/laser board is the only missing part and costs about $20–30. Everything else
> you already have carries over.

---

## 1. Extra parts

On top of the [Mural v2 BOM](https://github.com/nikivanov/mural/blob/main/BOM.md):

| Part | Notes |
|---|---|
| Arduino Uno (R3 or compatible) | |
| CNC Shield V3 | the common red/blue "grbl" shield |
| 2 × stepper driver modules | A4988, DRV8825, or TMC2209 (standalone/step-dir). TMC2209 runs quietest, closest to stock Mural |
| 5 V buck converter (e.g. Mini-360 or LM2596 module) | powers the servo from the 12 V supply |
| USB-A to USB-B cable (≤ 5 m) | PC to Uno. Use an *active* USB extension if you need more length |

**Not used:** the 0.91" OLED, the micro-SD card, and the Dupont wires for the FluidNC board.
**Still used:** both pancake steppers, the pulleys, belt, MG90S servo, USB-C PD trigger, 100 W USB-C supply, and the 10 ft USB-C cable (power only now).

## 2. Wiring

| From | To |
|---|---|
| USB-C PD trigger, **set to 12 V** | CNC Shield blue screw terminal (+ / −). **Check the polarity: reversing it kills the drivers.** |
| PD trigger 12 V | 5 V buck converter input. **Set the buck output to 5.0 V with a multimeter before connecting the servo.** |
| Left belt motor | **X** motor header |
| Right belt motor | **Y** motor header |
| Servo signal (orange) | **Z+ / Z− end-stop** header, signal pin (= Arduino **D11**) |
| Servo + (red) | buck converter 5 V out |
| Servo − (brown) | buck converter GND **and** CNC Shield GND (common ground is required) |
| Uno USB-B | your PC |

The Uno gets its power from USB. Don't run the servo from the Uno's 5 V pin: servo current spikes can reset the board in the middle of a drawing.

## 3. Driver setup (on the bench, before hanging it up)

Every move in Mural assumes **1/8 microstepping** (1600 steps/rev). Set the jumpers under each driver:

| Driver | Jumpers under the driver (M0 M1 M2) |
|---|---|
| A4988 | **M0 + M1 on**, M2 off |
| DRV8825 | **M0 + M1 on**, M2 off |
| TMC2209 standalone | **no jumpers** (MS1 = MS2 = low → 1/8 on most modules; check yours) |

If you use a different setting, start the bridge with `--microsteps N` to match.

**Current limit.** The pancake motors are 1 A parts; aim for about 0.7–0.8 A. Adjust the potentiometer on each driver while measuring between the pot and GND, with motors unplugged and 12 V applied:

- **A4988:** Vref = I × 8 × R<sub>sense</sub>. With R100 (0.1 Ω) sense resistors, use **≈ 0.6 V**. With R050 resistors, use ≈ 0.3 V.
- **DRV8825:** Vref = I ÷ 2 → **≈ 0.4 V**.
- **TMC2209:** follow the module's datasheet (often around **1.0 V** for 0.7 A RMS).

Other checks:
- Plug each driver in with its **EN pin next to the shield's EN label**. A driver plugged in backwards is destroyed at power-on.
- Never plug or unplug a motor while 12 V is on.
- If a motor buzzes but doesn't turn, one of its coil pairs is wired wrong. Check the pairs with a multimeter: a coil pair shows a few Ω between its two wires.

## 4. Flash the Uno

1. Open `firmware/mural_uno/mural_uno.ino` in the Arduino IDE (it uses only the built-in Servo library).
2. Board: **Arduino Uno**. Select the port and click **Upload**.
3. Open the Serial Monitor at **115200** baud, line ending "Newline". You should see `MuralUno 1.0 ready`. Typing `V` should answer `ok`.

Pin choices and motor direction are set in `firmware/mural_uno/config.h`.

## 5. Set up the PC bridge

You need Python 3.9+, git, and Node.js 18+ (Node is used once, to build the UI).

```bash
pip install -r host/requirements.txt
python3 scripts/build_ui.py          # downloads upstream Mural and builds its web UI into host/www
```

Run it (close the Arduino Serial Monitor first, because only one program can hold the port):

```bash
python3 host/mural_bridge.py --port /dev/ttyACM0       # Linux
python3 host/mural_bridge.py --port /dev/cu.usbmodem1101   # macOS
python3 host/mural_bridge.py --port COM3               # Windows
```

Then open **http://localhost:8080**. To control it from your phone, add `--listen 0.0.0.0` and browse to `http://<pc-ip>:8080`.

To try the UI without any hardware: `python3 host/mural_bridge.py --simulate`.

## 6. Drawing

The screens are the same as in the [official Mural guide](https://getmural.me):

1. **Set top distance:** enter the pin-to-pin distance in mm.
2. **Choose an image:** SVG or raster. It's converted in your browser and uploaded to the bridge.
3. **Retract belts:** hold the retract buttons until each belt is pulled all the way in against its stop screw.
   **First-time direction check:** "retract" must *wind the belt in*. If a side runs the wrong way,
   flip `INVERT_LEFT_DIR` / `INVERT_RIGHT_DIR` in `config.h` and re-upload.
4. **Extend to home:** hang Mural on the pins. It lowers itself to the home position.
5. **Pen calibration:** find the servo angle where the marker just touches the wall.
6. **Begin drawing.** Watch progress in the bridge's terminal.

**Stopping:** press **Ctrl+C** in the bridge terminal. That stops the motors immediately and lifts the pen.
You can also run `curl -X POST http://localhost:8080/stop`.
After a stop, the robot's position is no longer known, so redo the setup screens.

**Calibration:** the UI's tools menu has Mural's belt-length (e-steps) calibration. The pulley diameter it produces is saved in `~/.mural-uno/settings.json`.

## 7. Things that differ from stock Mural

- **Weight.** Mural's kinematics assume a **0.55 kg** robot. The Uno and shield weigh more and may also hang in a different spot.
  Weigh your finished robot and set `MASS_BOT` in `host/kinematics.py` to match. It affects how the model compensates for tilt and belt stretch.
- **Mounting.** The v2 STLs have a mount for the FluidNC board, not for an Uno (68.6 × 53.4 mm, holes on the standard Uno pattern).
  You'll need an adapter plate. Keep the weight centred between the pulleys.
- **No OLED or SD card.** Status appears in the bridge terminal, and the drawing file is stored on the PC in `~/.mural-uno/commands`.
- **Noise.** A4988 and DRV8825 drivers are noticeably louder than the TMC2209 drivers on the stock board.

## Files

| Path | What it is |
|---|---|
| `firmware/mural_uno/` | Uno sketch: serial protocol, coordinated stepping on Timer2, servo. The protocol is documented at the top of the `.ino` |
| `host/mural_bridge.py` | PC bridge: HTTP API compatible with Mural's UI, setup phases, drawing runner, simulation mode |
| `host/kinematics.py` | Port of Mural's kinematic model (`Movement::getBeltLengths`) |
| `scripts/build_ui.py` | Fetches and builds the upstream UI at a pinned commit |

Credits: Mural by Nik Ivanov and contributors, [github.com/nikivanov/mural](https://github.com/nikivanov/mural),
licensed CC BY-NC 4.0. `host/kinematics.py` and the bridge's phase logic are adapted from it and carry the same non-commercial terms.
