#!/usr/bin/env python3
"""Download the upstream Mural web UI and build it into host/www for mural_bridge.py.

Requires git and Node.js 18+ (npm) on your PATH. Works on Windows, macOS and Linux:
    python3 scripts/build_ui.py

The UI is not copied into this repository because upstream Mural is licensed
CC BY-NC 4.0 (https://github.com/nikivanov/mural). It is built locally instead,
from a pinned commit that mural_bridge.py was written against.
"""

import shutil
import subprocess
import sys
from pathlib import Path

UPSTREAM = "https://github.com/nikivanov/mural.git"
COMMIT = "1eceffa1f14c30d761a06b45510a966253782e82"  # 2026-11-12

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "host" / ".upstream"
WWW = ROOT / "host" / "www"
NPM = "npm.cmd" if sys.platform == "win32" else "npm"


def run(cmd, cwd):
    print("+", " ".join(cmd), f"(in {cwd})", flush=True)
    subprocess.run(cmd, cwd=cwd, check=True)


def main():
    if not (SRC / ".git").exists():
        run(["git", "clone", UPSTREAM, str(SRC)], ROOT)
    run(["git", "fetch", "origin"], SRC)
    run(["git", "checkout", "--force", COMMIT], SRC)

    # Same steps as upstream build.py, minus the gzip/LittleFS packaging.
    worker = SRC / "worker"
    run([NPM, "install", "--no-audit", "--no-fund"], worker)
    run([NPM, "run", "build"], worker)

    ui = SRC / "ui"
    run([NPM, "ci", "--no-audit", "--no-fund"], ui)
    run([NPM, "run", "build"], ui)

    built = SRC / "data" / "www"
    (built / "worker").mkdir(parents=True, exist_ok=True)
    packed = worker / "dist_packed"
    shutil.copy(packed / "main.js", built / "worker" / "worker.js")
    shutil.copy(packed / "oneline.js", built / "worker" / "oneline.js")
    shutil.copy(packed / "oneline.wasm", built / "worker" / "oneline.wasm")

    if WWW.exists():
        shutil.rmtree(WWW)
    shutil.copytree(built, WWW)
    print(f"\nDone: Mural UI built into {WWW}")


if __name__ == "__main__":
    main()
