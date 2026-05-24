# Building and flashing instructions

### 1. Clone this repo

```bash
git clone https://github.com/ArisMorgens/Crazyflie-Lighthouse-Wand.git
cd Crazyflie-Lighthouse-Wand
```

### 2. Initialize the firmware submodule

```bash
git submodule update --init --recursive --depth 1 firmware/crazyflie-firmware
```

This fetches `crazyflie-firmware @ 2026.04` into `firmware/crazyflie-firmware/`, which provides the build system and platform support needed to compile the app.

### 3. Set up the Python environment

This project uses [uv](https://docs.astral.sh/uv/) for Python dependency management.

```bash
cd firmware
uv sync
```

This creates a `.venv` with `cfloader` and the other required packages installed.

### 4. Build and flash the custom firmware app

Modify the [drones_config.yaml](../firmware/config/drones_config.yaml) defining the URI and app type for each drone. Then, use the `flash_all.py` script that handles building and flashing with automatic app selection:

```bash
# Flash all drones
uv run flash_all.py --all

# Flash specific drones
uv run flash_all.py --ids 1 2 3

# Flash a range
python flash_all.py --range 1-6
```

The script automatically:
- Builds firmware with correct app flags
- Detects when app type changes and triggers smart rebuilds
- Handles different platforms (cf2, cf21bl, bolt)
- Flashes via radio with warm boot
- Shows progress bars
