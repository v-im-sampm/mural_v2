# Mural v2 on an ESP32 "Uno" board + Arduino CNC Shield

This runs the **official [Mural v2](https://getmural.me) firmware** on an ESP32 board in Arduino Uno form factor
(Wemos **D1 R32** and its clones) with an **Arduino CNC Shield V3**, instead of the
"TMC2209 Pen/Laser FluidNC" controller from the Mural BOM.

The CNC Shield is wired differently from the official controller, so the firmware needs a small patch
([`patches/mural-d1r32.patch`](patches/mural-d1r32.patch)):

- **Pins** remapped to the CNC Shield. All pin numbers live in one new file, `src/board.h`.
- **Driver UART setup disabled.** The stock board sets the TMC2209 current and microstepping over a serial line the
  shield doesn't have. On the shield you set them with jumpers and the driver's potentiometer.
  The UART pins (GPIO16/17) are also X-DIR and Z-STEP on the shield, so leaving that code in would break the X motor.
- **OLED made optional.** Stock firmware crashes without it. The IP address is now also printed on the USB serial port.

Everything else is unchanged: the same WiFi web UI, setup screens, kinematics, and drawing pipeline.

---

## 1. Check your board

Look at the silkscreen next to the Uno headers. On a D1 R32 the pin labels include the ESP32 GPIO number,
for example **D2 → IO26, D3 → IO25, D8 → IO12, D9 → IO13**. The full mapping this patch assumes is:

| Uno pin | D2 | D3 | D4 | D5 | D6 | D7 | D8 | D9 | D10 | D11 | D12 | D13 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| **GPIO** | 26 | 25 | 17 | 16 | 27 | 14 | 12 | 13 | 5 | 23 | 19 | 18 |

If your board's numbers differ, edit `src/board.h` in the prepared firmware (step 5).

## 2. Extra parts

On top of the [Mural v2 BOM](https://github.com/nikivanov/mural/blob/main/BOM.md):

| Part | Why |
|---|---|
| 2 × stepper driver modules | **DRV8825** (recommended: guaranteed to accept the ESP32's 3.3 V signals), TMC2209 standalone (quietest), or A4988 |
| Micro-SD card **module** (SPI breakout, 5 V type with regulator) | The FluidNC board has an SD slot built in; the shield doesn't. **The firmware won't run without a card.** |
| 5 V buck converter (Mini-360, LM2596…), set to 5.0 V | Powers the servo and SD module from 12 V |
| DC barrel plug (5.5 × 2.1 mm) with leads | Powers the ESP32 board from the PD trigger |
| Female-to-female and male-to-female Dupont wires | |

## 3. Wiring

**Power** (USB-C PD trigger **set to 12 V**):

| From | To |
|---|---|
| PD trigger 12 V | CNC Shield blue screw terminal. **Check polarity: reversed power kills the drivers.** |
| PD trigger 12 V | ESP32 board's **DC barrel jack** (accepts 7–12 V). This powers the ESP32 and the shield's 5 V logic. |
| PD trigger 12 V | 5 V buck converter input. **Set the buck to 5.0 V with a multimeter before connecting anything to it.** |
| Buck GND | CNC Shield GND (screw terminal −). A common ground is required. |

**Signals:**

| Mural part | Where on the CNC Shield | GPIO |
|---|---|---|
| Left belt motor | **X** motor header | 26 / 16 |
| Right belt motor | **Y** motor header | 25 / 27 |
| Servo signal (orange) | **X+ or X−** end-stop header, signal pin (D9) | 13 |
| Servo red / brown | buck 5 V / GND | |
| SD module **CS** | **Y+ or Y−** end-stop header, signal pin (D10) | 5 |
| SD module **MOSI** | **Z+ or Z−** end-stop header, signal pin (D11) | 23 |
| SD module **MISO** | **SpnEn** pin (D12) | 19 |
| SD module **SCK** | **SpnDir** pin (D13) | 18 |
| SD module VCC / GND | buck 5 V / GND | |
| OLED SDA *(optional)* | **STEP** hole of the empty **Z driver socket** (D4) | 17 |
| OLED SCL *(optional)* | **DIR** hole of the empty **Z driver socket** (D7) | 14 |
| OLED VCC / GND *(optional)* | 3.3 V (the D1 R32's 3V3 pin) / GND | |

Notes:
- Each end-stop header is a pair of pins: one is the signal, the other is GND. "+" and "−" share the same signal line.
- **Leave the Z driver socket empty**: its STEP and DIR pins are borrowed for the OLED.
- Without the OLED, find Mural at **http://mural.local**, or read the IP address from the serial monitor (115200 baud).
- The ESP32 is not 5 V-tolerant. Power the OLED from 3.3 V.
  If you can only reach 5 V, check that the module's I²C pull-ups go to its own 3.3 V regulator, or skip the OLED.

## 4. Driver setup (on the bench)

Mural assumes **1/8 microstepping**. Set the jumpers under each driver:

| Driver | Jumpers (M0 M1 M2) |
|---|---|
| DRV8825 | **M0 + M1 on**, M2 off |
| A4988 | **M0 + M1 on**, M2 off |
| TMC2209 standalone | **none** (= 1/8 on most modules; check your module's table) |

**Current.** The pancake motors are 1 A parts; stock Mural runs them at about 0.7 A RMS. With motors unplugged and
12 V on, measure between the driver's potentiometer and GND, then adjust:

- **DRV8825:** Vref = I ÷ 2 → **≈ 0.4 V**
- **A4988:** Vref = I × 8 × R<sub>sense</sub> → **≈ 0.6 V** with R100 resistors (≈ 0.3 V with R050)
- **TMC2209:** per the module's datasheet (often **≈ 1.0 V**)

Other checks:
- Seat each driver with its **EN pin next to the shield's EN label**. A driver plugged in backwards is destroyed at power-on.
- Never plug or unplug a motor while 12 V is on.

## 5. Build and flash the firmware

You need [PlatformIO](https://platformio.org/install) (the VS Code extension or `pip install platformio`),
[Node.js 18+](https://nodejs.org), and git.

```bash
python3 scripts/prepare_firmware.py   # downloads Mural at a pinned commit, applies the patch, installs UI deps
cd mural-firmware
```

Connect the ESP32 board to your PC by USB (12 V can stay off for this), then:

```bash
pio run -e d1r32 -t upload      # firmware
pio run -e d1r32 -t uploadfs    # web UI (built automatically from ui/ and worker/)
pio device monitor -b 115200    # optional: watch it boot
```

**First boot:** Mural opens a WiFi hotspot named **"Mural"**. Join it, pick your home WiFi in the page that opens,
and Mural restarts. Then browse to **http://mural.local** (or the IP shown on the OLED or serial monitor).
From here on, follow the [official Mural guide](https://getmural.me).

**First-time direction check:** on the *Retract belts* screen, "retract" must **wind the belt in**.
If a side runs the wrong way, flip `BOARD_INVERT_LEFT` / `BOARD_INVERT_RIGHT` in
`mural-firmware/src/board.h` and upload again. Swapping one coil pair of that motor's cable also works.

## 6. Differences from stock Mural to watch for

- **Weight.** Mural's kinematics assume the robot weighs **0.55 kg** (`mass_bot` in `src/movement.h`).
  The Uno-size board, shield and SD module change that. Weigh your finished robot and update the value.
- **Mounting.** The v2 STLs have a mount for the FluidNC board, not for a 68.6 × 53.4 mm Uno-format board.
  You'll need an adapter. Keep the weight centred between the pulleys.
- **Noise.** DRV8825 and A4988 drivers are louder than TMC2209s.

## Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| Board boot-loops or reports `flash read err` **only with the shield attached** | GPIO12 (D8, the shield's EN line) is an ESP32 boot-strapping pin. If the shield pulls it high at reset, the flash gets the wrong voltage. Last-resort permanent fix: `espefuse.py --port <port> set_flash_voltage 3.3V` (safe for ESP32-WROOM-32 modules, irreversible). |
| OLED shows **"NO CARD"** or serial says no card, then hangs | SD module wiring (CS/MOSI/MISO/SCK above) or power. Use a 5 V module powered from the buck. The card is formatted automatically if needed. |
| Motor buzzes but doesn't turn | A coil pair is swapped in the motor cable, or the current is too low |
| Motors stutter or skip at random with A4988/TMC2209 | Those drivers expect ~3.5 V logic when the shield runs them at 5 V, and the ESP32 only gives 3.3 V. Use DRV8825s. |
| Can't reach http://mural.local | Use the IP address from the OLED or serial monitor (Android and some Windows setups don't resolve `.local`) |

## What's in this repo

| Path | What it is |
|---|---|
| `patches/mural-d1r32.patch` | The firmware changes, against upstream commit `1eceffa` |
| `scripts/prepare_firmware.py` | Clones upstream Mural, applies the patch, installs the UI's npm dependencies |

Credits: Mural by Nik Ivanov and contributors, [github.com/nikivanov/mural](https://github.com/nikivanov/mural),
licensed CC BY-NC 4.0. The patch is a derivative work under the same non-commercial terms.
