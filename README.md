# heatsoak

Adaptive heatsoak for Klipper. Detects thermal steady state from heater
PWM output instead of waiting a fixed time.

## Why

`M190` / `TEMPERATURE_WAIT` return as soon as the bed *sensor* reads target
temperature. But a single thermistor under one spot of the bed only tells
you that *this* spot is at temperature, not the rest of the plate, the
gantry, or the chamber.

The usual workaround is a constant wait, which has to be long enough for
cold prints and is therefore wasteful for warm ones.

Heater power tells you more: as the surrounding material catches up,
the PID controller needs less energy to hold temperature, so PWM duty cycle
drops and then settles at whatever it takes to offset heat loss. That
settled value is a better proxy for full equalization than the temperature
reading alone.

This plugin samples the heater's PWM duty cycle over a sliding window and
declares steady state when:

  - the fitted slope of power vs. time is below a threshold (signal is flat)
  - the std-dev of the residuals around that fit is below a threshold (oscillation is bounded)

Slope catches slow drift; residual variance catches loud PID oscillation.

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

Add to `printer.cfg`:

```ini
[heatsoak]
heater: heater_bed              # which heater to watch
window_size: 15                 # sliding window length (samples); also the
                                # number of samples that must accumulate before
                                # a steady-state decision can be made
sample_interval: 2.0            # seconds between samples
slope_threshold: 0.005          # max rate of power change (PWM/sec) considered flat
residual_threshold: 0.02        # max std-dev of residuals around the fit (PWM)
min_duration: 0                 # minimum wait regardless of detector (sec)
max_duration: 1800              # hard cap, stops waiting and prints a message (sec)
log_path: ~/printer_data/logs/heatsoak/    # CSV trace per run; empty to disable
```

All fields optional; defaults above. `min_duration: 0` means an already-hot
bed exits the loop as soon as the detector has enough samples.

## Usage

In your print-start macro, after `M190`:

```gcode
M140 S60
M190 S60
HEATSOAK_WAIT
G28
```

Optional per-call overrides:

```gcode
HEATSOAK_WAIT MIN_DURATION=120 MAX_DURATION=600
```

Console output:

```
// heatsoak: starting (target=60.0C min=0s max=1800s)
// heatsoak: t=10s power=0.142 slope=-0.00614 resid=0.0036 n=5
// heatsoak: t=20s power=0.118 slope=-0.00422 resid=0.0028 n=10
...
// heatsoak: steady state reached at t=128s (power=0.062)
```

## Tuning

The defaults are reasonable starting points, but every printer has its own
thermal characteristics. For best results on your specific machine, use the
calibration command:

```gcode
M140 S60
M190 S60
HEATSOAK_CALIBRATE             ; default 20 min observation
; or:
HEATSOAK_CALIBRATE DURATION=1800   ; 30 min for slower beds
```

This runs a long observation **without applying any threshold check** — it
samples for the full duration regardless of what the signal looks like. At the
end it analyzes the tail of the trace (presumed steady state) and prints
recommended threshold values:

```
// heatsoak calibrate: analyzed tail of 150 samples (last 300s of run)
// heatsoak calibrate: tail max|slope|=0.000048, max resid=0.0042, avg power=0.165
// heatsoak calibrate: recommended slope_threshold: 0.00010
// heatsoak calibrate: recommended residual_threshold: 0.0084
```

Copy those values into your `[heatsoak]` config block.

The calibration also writes a CSV trace (`cal_<from>Cto<to>C_<unix>.csv`),
distinguishable from normal run traces. Useful for offline analysis if you
want to see the full curve.

### Manual tuning (fallback)

If you want to tune without running calibration:

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
