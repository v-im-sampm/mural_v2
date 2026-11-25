#!/usr/bin/env python3
"""Fetch the official Mural firmware and patch it for an ESP32 Uno board + CNC Shield V3.

    python3 scripts/prepare_firmware.py

Creates ./mural-firmware: the upstream Mural repo at a pinned commit, with
patches/mural-d1r32.patch applied. Build and flash it from that folder with
PlatformIO (see README.md):

    pio run -e d1r32 -t upload      # firmware
    pio run -e d1r32 -t uploadfs    # web UI (built automatically, needs Node.js)

Upstream Mural (https://github.com/nikivanov/mural) is licensed CC BY-NC 4.0,
so it is downloaded rather than copied into this repository.
"""

import subprocess
import sys
from pathlib import Path

UPSTREAM = "https://github.com/nikivanov/mural.git"
COMMIT = "1eceffa1f14c30d761a06b45510a966253782e82"  # 2026-11-12, the version the patch was made against

ROOT = Path(__file__).resolve().parent.parent
DEST = ROOT / "mural-firmware"
PATCH = ROOT / "patches" / "mural-d1r32.patch"


def git(*args):
    print("+ git", " ".join(args), flush=True)
    subprocess.run(["git", *args], cwd=DEST, check=True)


def main():
    if not (DEST / ".git").exists():
        subprocess.run(["git", "clone", UPSTREAM, str(DEST)], check=True)
    git("fetch", "origin")
    git("checkout", "--force", COMMIT)
    git("clean", "-fdq", "src")  # drop a previously applied src/board.h
    git("apply", str(PATCH))

    for pkg in ("worker", "ui"):  # the firmware build also builds the web UI
        npm = "npm.cmd" if sys.platform == "win32" else "npm"
        print(f"+ {npm} install (in {pkg})", flush=True)
        subprocess.run([npm, "install", "--no-audit", "--no-fund"], cwd=DEST / pkg, check=True)

    print(f"\nReady: {DEST}\nNext: cd mural-firmware && pio run -e d1r32 -t upload && pio run -e d1r32 -t uploadfs")


if __name__ == "__main__":
    main()
