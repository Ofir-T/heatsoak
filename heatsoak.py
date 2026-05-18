# Detect thermal steady state of a heater (adaptive heatsoak).
#
# Copyright (c) 2026 Ofir Temelman <ofirtemelman@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

from collections import deque
from math import sqrt

class SteadyStateDetector:
    def __init__(self, window_size, slope_threshold, residual_threshold, min_samples):
        """
        Initializes the steady-state detector.

        :param window_size: Max samples to keep; older samples drop off the back
        :param slope_threshold: Max |slope| (PWM/sec) considered flat
        :param residual_threshold: Max std-dev of residuals around the fitted line
        :param min_samples: Minimum samples before is_stable() can return True
        """
        self.window_size = window_size
        self.slope_threshold = slope_threshold
        self.residual_threshold = residual_threshold
        self.min_samples = min_samples
        self.samples = deque(maxlen=window_size)

    def add_sample(self, timestamp, power):
        """
        Append a (time, power) sample to the window.
        Once the window is full, the oldest sample is dropped automatically.

        :param timestamp: Monotonic time of the reading, in seconds
        :param power: Heater PWM duty cycle at the reading time, in [0.0, 1.0]
        """
        self.samples.append((timestamp, power))

    def is_stable(self) -> bool:
        """
        Return True when the window has at least min_samples and
        both the fitted slope and the residual std-dev are below their thresholds.
        False otherwise.
        """
        if len(self.samples) < self.min_samples:
            return False

        status = self.get_status()
        slope = status['slope']
        residual_std = status['residual_std']

        return (abs(slope) < self.slope_threshold) and (residual_std < self.residual_threshold)

    def reset(self):
        """Clear all stored samples."""
        self.samples.clear()

    def get_status(self):
        """
        Snapshot of detector state as a dict:

          - 'sample_count': number of samples currently in the window
          - 'slope':        fitted slope (PWM/sec), or None if n < 2
          - 'residual_std': residual std-dev, or None if n < 3
        """
        n = len(self.samples)
        if n < 2:
            return {'sample_count': n, 'slope': None, 'residual_std': None}
        slope, intercept = _fit_line(self.samples)
        if n < 3:
            return {'sample_count': n, 'slope': slope, 'residual_std': None}
        residual_std = _residual_stdev(slope, intercept, self.samples)
        return {'sample_count': n, 'slope': slope, 'residual_std': residual_std}

def _fit_line(points):
    """
    Fit a line to a collection of (x, y) points using least squares.

    :param points: Iterable of (x, y) pairs; requires len >= 2 and at least two distinct x values
    :returns: (slope, intercept) of the fitted line
    """
    n = len(points)
    x_values, y_values = zip(*points)

    x_mean = sum(x_values) / n
    y_mean = sum(y_values) / n

    numerator = sum([(x - x_mean)*(y - y_mean) for x,y in points])
    denominator = sum([(x - x_mean)** 2 for x,y in points])

    slope = numerator / denominator
    intercept = y_mean - (slope * x_mean)

    return slope, intercept

def _residual_stdev(slope, intercept, points):
    """
    Compute the standard deviation of residuals around a fitted line.
    Uses the (n - 2) denominator since two degrees of freedom were spent fitting slope and intercept.

    :param slope: Slope of the fitted line
    :param intercept: Intercept of the fitted line
    :param points: Iterable of (x, y) pairs used to fit the line; requires len >= 3
    :returns: Standard deviation of the residuals
    """
    n = len(points)
    residuals = [y - (slope * x + intercept) for x, y in points]
    var = sum(r ** 2 for r in residuals) / (n - 2)
    std = sqrt(var)

    return std

class Heatsoak:
    """
    Klipper extra that exposes the HEATSOAK_WAIT G-code command.
    Reads config, owns a SteadyStateDetector, and drives the detection loop
    by sampling heater power until the detector reports steady state.
    """
    def __init__(self, config):
        self.printer = config.get_printer()

        self.heater_name = config.get('heater', 'heater_bed')
        self.window_size = config.getint('window_size', 15, minval=4)
        self.sample_interval = config.getfloat('sample_interval', 2.0, above=0)
        self.slope_threshold = config.getfloat('slope_threshold', 0.005, above=0)
        self.residual_threshold = config.getfloat('residual_threshold', 0.02, above=0)
        self.min_samples = min(config.getint('min_samples', 5, minval=3), self.window_size)
        self.min_duration = config.getfloat('min_duration', 0.0, minval=0) # allow for hot printers to start quickly
        self.max_duration = config.getfloat('max_duration', 1800, minval=self.min_duration)

        self.detector = SteadyStateDetector(self.window_size,self.slope_threshold,
                                            self.residual_threshold, self.min_samples)

        # find heater after entire config was initialized
        self.heater = None
        self.printer.register_event_handler("klippy:connect", self._handle_connect)

        # register gcode command
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('HEATSOAK_WAIT', self.cmd_HEATSOAK_WAIT,
                               desc=self.cmd_HEATSOAK_WAIT_help)

    def _handle_connect(self):
        pheaters = self.printer.lookup_object('heaters')
        self.heater = pheaters.lookup_heater(self.heater_name)

    cmd_HEATSOAK_WAIT_help = "Wait until heater reaches thermal steady state"
    def cmd_HEATSOAK_WAIT(self, gcmd):
        min_duration = gcmd.get_float('MIN_DURATION', self.min_duration, minval=0.)
        max_duration = gcmd.get_float('MAX_DURATION', self.max_duration, above=min_duration)

        target = self.heater.target_temp
        if target <= 0:
            raise gcmd.error(f'Heater {self.heater_name} has no target temperature, skipping heatsoak')
        gcmd.respond_info(f"heatsoak: starting (target={target:.1f}C min={min_duration:.0f}s max={max_duration:.0f}s)")

        self.detector.reset()
        reactor = self.printer.get_reactor()
        start_time = reactor.monotonic()
        eventtime = start_time

        # wait for heater to "reach" target temp
        while not self.printer.is_shutdown() and self.heater.check_busy(eventtime):
            eventtime = reactor.pause(eventtime + 1.)

        # perform heatsoak
        while not self.printer.is_shutdown():
            eventtime = reactor.pause(eventtime + self.sample_interval) # yield until the next sample window
            elapsed = eventtime - start_time
            heater_status = self.heater.get_status(eventtime)
            power = heater_status['power']

            if elapsed > max_duration: # timeout
                gcmd.respond_info(f"heatsoak: max_duration {max_duration:.0f}s exceeded, proceeding anyway (last power={power:.3f})")
                return

            # next sample
            self.detector.add_sample(eventtime, power)
            if elapsed >= min_duration and self.detector.is_stable():
                gcmd.respond_info(f"heatsoak: steady state reached at t={elapsed:.0f}s (power={power:.3f})")
                return

            # progress update
            det = self.detector.get_status()
            slope = det['slope'] if det['slope'] is not None else 0.
            resid = det['residual_std'] if det['residual_std'] is not None else 0.
            n = det['sample_count']
            gcmd.respond_info(f"heatsoak: t={elapsed:.0f}s power={power:.3f} slope={slope:.5f} resid={resid:.4f} n={n}")

def load_config(config):
    return Heatsoak(config)