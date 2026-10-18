# K230-RUNNER

A simple CanMV K230 runner game built with MicroPython and LVGL.

This repository contains the game source and project documentation.
The repository intentionally keeps bundled font binaries out of Git history to stay lightweight.

## Game Features

- Tap to jump
- Tap again in mid-air for double jump
- Dodge low obstacles, tall walls, and flying birds
- Collect power-ups:
  - Shield
  - Magnet
  - Rocket
- Start/title screen
- Game-over screen with score, best score, rank, and session stats

## Hardware / Environment

Tested on:

- CanMV K230 / K230D
- CanMV firmware based on MicroPython
- 640x480 ST7701 display
- Touch panel input

Default font resource path in code:

- A:/sdcard/CanMV Sample/Fonts/lv_font_simsun_16_cjk.fnt

## Repository Structure

- man.py - main game entry point
- main.py - optional device default entry point
- Fonts/README.md - notes about required font assets
- docs/AUTHENTICATION.md - notes for pushing over HTTPS
- LICENSE - MIT license

## How to Run

1. Copy the project to your CanMV storage.
2. Copy required font assets into the matching path on the device.
3. Run man.py, or use main.py if your device boots from that entry point.

## Notes

- APP/ and Photos/ are not tracked in this repository.
- This project is intended for the CanMV / MicroPython environment on K230.

## License

MIT
