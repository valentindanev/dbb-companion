# DBB Companion

**DBB Companion** is the open-source onboard-computer firmware for the **Danevi Bait Boats** platform — an ESP32 companion computer that bridges dual sonar and a MAVLink flight controller (ArduPilot / ArduRover) over Wi-Fi, with a built-in web dashboard and dual-slot OTA updates.

> ⚠️ **Early / work-in-progress.** This is the open *base*. It already works as a sonar + MAVLink Wi-Fi bridge, but hardening (RAM/task cleanup) is ongoing — expect changes.

## What it does
- **Dual sonar → MAVLink `DISTANCE_SENSOR`** — a hardwired UART sonar *and* a Deeper CHIRP+ Wi-Fi sonar (boot-time source selection), published to the flight controller.
- **MAVLink 2 Wi-Fi bridge** to an ArduPilot / ArduRover flight controller (plus a transparent passthrough mode).
- **Web dashboard** (AP or STA) — settings, live stats, sonar debug, logs.
- **Dual-slot A/B OTA** for the app and the web UI, with a baked-in recovery portal.
- **Persistent sonar log** with HTTP download.

## Hardware
- ESP32 (classic) or ESP32-S3. Reference target: **ESP32-S3-WROOM-1 (N16R8)**.
- A flight controller running **ArduPilot / ArduRover** with MAVLink 2 on a spare UART.
- Optional: a hardwired UART sonar and/or a Deeper CHIRP+.

## Build (ESP-IDF v5.4.x)
```sh
idf.py set-target esp32        # or esp32s3
idf.py build
idf.py -p <PORT> flash monitor
```
The small web UI under `frontend/` is built automatically (via npm) during the CMake build.

## Architecture: open base + optional brain
This repository is the **open base**. It exposes a small hook interface (`main/dbb_brain.h`) backed by **weak no-op stubs** (`main/dbb_brain_stub.c`). Out of the box those hooks do nothing, so you get a plain, fully-working sonar + MAVLink bridge.

An optional **brain** component can override those hooks at link time to add autonomous behaviour. If a sibling folder `../dbb-brain-private/components/` is present at build time, its *strong* symbols replace the weak stubs (see the top-level `CMakeLists.txt`). Anyone can write their own brain against the same interface.

## Credits & license
- This firmware is **Apache-2.0** — see [`LICENSE`](LICENSE) and [`NOTICE`](NOTICE).
- Built on **[DroneBridge for ESP32](https://github.com/DroneBridge/ESP32)** (Apache-2.0) by Wolfgang Christl — the Wi-Fi transport, parameter/settings system, serial↔MAVLink bridge, and REST scaffolding.
- MAVLink via **[fastMAVLink](https://github.com/olliw42/fastmavlink)** (MIT).
- The dual-sonar stack, persistent log, A/B OTA, and web dashboard are original work by **Valentin Danev**.

---
*Danevi Bait Boats · DBB Companion*
