# heatsoak

Adaptive heatsoak for Klipper. Detects thermal steady state from heater
PWM output instead of waiting a fixed time.

## Why

`M190` / `TEMPERATURE_WAIT` return as soon as the bed *sensor* reads target
temperature — not when the surrounding frame, gantry, and chamber have caught up.

As the thermal mass equilibrates, the PID controller draws less power to hold
temperature. This plugin watches that power drop and exits when it settles, so cold prints wait as long as they need to and warm ones exit early.

## Installation

```bash
git clone https://github.com/Ofir-T/heatsoak.git ~/heatsoak
cd ~/heatsoak
./install.sh
```

This symlinks `heatsoak.py` into `~/klipper/klippy/extras/` and restarts
the `klipper` service. Override with `KLIPPER_PATH=...` if Klipper lives
elsewhere.

### Moonraker update manager (optional)

Add to `moonraker.conf` so updates flow through Moonraker:

```ini
[update_manager heatsoak]
type: git_repo
path: ~/heatsoak
origin: https://github.com/Ofir-T/heatsoak.git
primary_branch: main
managed_services: klipper
install_script: install.sh
```

## Configuration

Add a minimal section to `printer.cfg` to get started:

```ini
[heatsoak]
heater: heater_bed
```

Everything else has safe defaults. Run `HEATSOAK_CALIBRATE` (see [Tuning](#tuning))
to fill in the threshold values, then `SAVE_CONFIG`.

Full reference — all fields except `heater` are optional:

```ini
[heatsoak]
heater: heater_bed              # required: which heater to watch
window_size: 30                 # sliding window length (samples)
sample_interval: 2.0            # seconds between samples
min_duration: 0                 # minimum wait regardless of detector (sec)
max_duration: 1800              # hard cap, stops waiting and prints a message (sec)
log_path: ~/printer_data/logs/heatsoak/    # CSV trace per run; empty to disable
# set by HEATSOAK_CALIBRATE + SAVE_CONFIG:
slope_threshold: 0.005
residual_threshold: 0.02
relative_slope_threshold: 0.0
steady_state_power: 0.0
```

## Usage

In your print-start macro, after `M190`:

```gcode
M140 S60
M190 S60
HEATSOAK_WAIT
G28
```

For prints where only bed surface temperature matters, `MODE=quick` exits at the
inflection point of the power decay — when the bed surface is at temperature but
the frame and gantry are still catching up (~90s into a 60°C cold start vs ~430s
for full equilibration):

```gcode
HEATSOAK_WAIT MODE=quick
```

Per-call overrides:

```gcode
HEATSOAK_WAIT MIN_DURATION=120 MAX_DURATION=600
HEATSOAK_WAIT MODE=quick MAX_DURATION=300
```

Console output:

```
// heatsoak: starting (target=60.0C start=25.0C signal=integral mode=full min=0s max=1800s)
// heatsoak: t=10s power=0.561 slope=-0.01842 resid=0.0041 n=5
...
// heatsoak: steady state reached at t=428s (power=0.062)
```

```
// heatsoak: starting (target=60.0C start=25.0C signal=integral mode=quick min=0s max=1800s)
...
// heatsoak: tail entry at t=97s (power=0.561), bed surface at temp
```

## Tuning

The defaults are reasonable starting points, but every printer has its own
thermal characteristics. For best results, run the calibration command after
setting your target temperature:

```gcode
M140 S60
HEATSOAK_CALIBRATE
; or override directly:
HEATSOAK_CALIBRATE TARGET=70 DURATION=2400
```

`TARGET` defaults to the heater's current target; `DURATION` defaults to
`max_duration` from config. **Minimum recommended duration is 900 s** (15 min);
1800 s is sufficient for most printers. If your bed is large or slow to equalize,
use `DURATION=2400` or longer. This runs a full observation without applying any
threshold checks, analyzes the tail, and stages the recommended values for
`SAVE_CONFIG`:

```
// heatsoak calibrate: analyzed tail of 150 samples (last 300s of run)
// heatsoak calibrate: tail max|slope|=0.000048, max resid=0.0042, avg power=0.165
// heatsoak calibrate: recommended slope_threshold: 0.00010
// heatsoak calibrate: recommended residual_threshold: 0.0084
// heatsoak calibrate: recommended relative_slope_threshold: 0.00120
// heatsoak calibrate: recommended steady_state_power: 0.1650
// heatsoak calibrate: run SAVE_CONFIG to persist these values
```

Then run `SAVE_CONFIG` to write them to `printer.cfg` and restart Klipper.

If the bed was still settling at the end of the run the values are **not** staged
and you'll see a warning — re-run with a longer `DURATION`, then `SAVE_CONFIG`.

### Manual tuning (fallback)

1. Run `HEATSOAK_WAIT MAX_DURATION=300` and watch the console output.
2. Note the `slope` and `resid` values after the bed has visibly settled
   (Mainsail/Fluidd graphs help confirm).
3. Set `slope_threshold` to ~2× the observed settled slope.
4. Set `residual_threshold` to ~2× the observed settled residual.
5. Iterate if the detector trips too early or too late.

## Development

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install pytest
pytest
```

The `SteadyStateDetector` class is pure Python with no Klipper imports and
is covered by the unit tests in `tests/`.

## License

GPL-3.0, matching Klipper itself. See [LICENSE](LICENSE).
