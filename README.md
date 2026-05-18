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
window_size: 15                 # sliding window length (samples)
sample_interval: 2.0            # seconds between samples
slope_threshold: 0.005          # max rate of power change (PWM/sec) considered flat
residual_threshold: 0.02        # max std-dev of residuals around the fit (PWM)
min_samples: 5                  # samples required before steady-state can be declared
min_duration: 0                 # minimum wait regardless of detector (sec)
max_duration: 1800              # hard cap, stops waiting and prints a message (sec)
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

To tune the thresholds for your printer:

1. Run a print with `HEATSOAK_WAIT MAX_DURATION=300` and watch the console
   output. The 300-second ceiling lets the command return even if the default
   thresholds don't yet match your printer.
2. Record `slope`, `resid`, and `power` after the bed has visibly settled
   (you can use Mainsail/Fluidd graphs to confirm).
3. Set `slope_threshold` to ~2x the observed settled slope.
4. Set `residual_threshold` to ~2x the observed settled residual.
5. Repeat once or twice if the detector trips too early or too late.

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
