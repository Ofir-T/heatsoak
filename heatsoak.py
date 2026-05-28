# Detect thermal steady state of a heater (adaptive heatsoak).
#
# Copyright (c) 2026 Ofir Temelman <ofirtemelman@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import os
import time
from collections import deque
from math import sqrt

class SteadyStateDetector:
    def __init__(self, window_size, slope_threshold, residual_threshold):
        """
        Initializes the steady-state detector.

        :param window_size: Number of samples that must fill the window before
                            is_stable() can return True. Also caps memory.
        :param slope_threshold: Max |slope| (PWM/sec) considered flat
        :param residual_threshold: Max std-dev of residuals around the fitted line
        """
        self.window_size = window_size
        self.slope_threshold = slope_threshold
        self.residual_threshold = residual_threshold
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
        Return True when the signal is genuinely constant across the window.

        Three checks must pass; if any fails, the signal is still moving:
          1. residual_std below threshold (signal is quiet around its trend)
          2. each half-window's slope below threshold (signal is flat in both halves)
          3. the two halves' slopes don't differ by more than threshold
             (no curvature - the slope itself isn't trending)

        The third check is what distinguishes "constant" from "small but still
        curving toward an asymptote". A signal in slow exponential decay can
        satisfy a small full-window slope while still showing different slopes
        in its first vs second half - this check rejects that case.

        Requires a full window before any True is possible.
        """
        if len(self.samples) < self.window_size:
            return False

        # Check 1: residual on the full-window linear fit
        status = self.get_status()
        if status['residual_std'] >= self.residual_threshold:
            return False

        # Checks 2 and 3: split-window slope analysis
        samples = list(self.samples)
        mid = len(samples) // 2
        slope_first, _ = _fit_line(samples[:mid])
        slope_second, _ = _fit_line(samples[mid:])

        if abs(slope_first) >= self.slope_threshold:
            return False
        if abs(slope_second) >= self.slope_threshold:
            return False
        if abs(slope_first - slope_second) >= self.slope_threshold:
            return False

        return True

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
        self.min_duration = config.getfloat('min_duration', 0.0, minval=0) # allow for hot printers to start quickly
        self.max_duration = config.getfloat('max_duration', 1800, minval=self.min_duration)
        self.log_path = config.get('log_path', '~/printer_data/logs/heatsoak/')
        if self.log_path:
            self.log_path = os.path.expanduser(self.log_path)

        self.detector = SteadyStateDetector(self.window_size, self.slope_threshold,
                                            self.residual_threshold)

        # find heater after entire config was initialized
        self.heater = None
        self.printer.register_event_handler("klippy:connect", self._handle_connect)

        # register gcode commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('HEATSOAK_WAIT', self.cmd_HEATSOAK_WAIT,
                               desc=self.cmd_HEATSOAK_WAIT_help)
        gcode.register_command('HEATSOAK_CALIBRATE', self.cmd_HEATSOAK_CALIBRATE,
                               desc=self.cmd_HEATSOAK_CALIBRATE_help)

    def _handle_connect(self):
        pheaters = self.printer.lookup_object('heaters')
        self.heater = pheaters.lookup_heater(self.heater_name)
        # PID controllers expose prev_temp_integ + Ki; bang-bang does not.
        # When available, Ki * prev_temp_integ is the steady-state component
        # of the PID output - a much cleaner signal than raw PWM snapshots.
        self.use_integral = hasattr(self.heater.control, 'prev_temp_integ') \
                            and hasattr(self.heater.control, 'Ki')

    def _read_signal(self):
        """Return the value fed to the detector.
        For PID: Ki * prev_temp_integ (low-pass-filtered effective power).
        For bang-bang or unknown: raw PWM duty cycle.
        Read under heater.lock to avoid torn reads during a PID update."""
        with self.heater.lock:
            if self.use_integral:
                ctrl = self.heater.control
                return ctrl.Ki * ctrl.prev_temp_integ
            return self.heater.last_pwm_value

    cmd_HEATSOAK_WAIT_help = "Wait until heater reaches thermal steady state"
    def cmd_HEATSOAK_WAIT(self, gcmd):
        min_duration = gcmd.get_float('MIN_DURATION', self.min_duration, minval=0.)
        max_duration = gcmd.get_float('MAX_DURATION', self.max_duration, above=min_duration)

        target = self.heater.target_temp
        if target <= 0:
            raise gcmd.error(f'Heater {self.heater_name} has no target temperature, skipping heatsoak')

        reactor = self.printer.get_reactor()
        start_status = self.heater.get_status(reactor.monotonic())
        start_temp = start_status['temperature']
        signal_source = 'integral' if self.use_integral else 'pwm'

        gcmd.respond_info(f"heatsoak: starting (target={target:.1f}C, start={start_temp:.1f}C, signal={signal_source}, min={min_duration:.0f}s max={max_duration:.0f}s)")

        csv_file = None
        try:
            if self.log_path:
                os.makedirs(self.log_path, exist_ok=True)
                filename = os.path.join(self.log_path, f"run_{int(start_temp)}Cto{int(target)}C_{int(time.time())}.csv")
                csv_file = open(filename, 'w')
                csv_file.write(f"# start_temp={start_temp:.2f},target={target:.1f},signal={signal_source},start_unix={int(time.time())}\n")
                csv_file.write("elapsed_s,unix_time,temp,target,power,slope,residual_std,sample_count\n")
                gcmd.respond_info(f"heatsoak: logging to {filename}")

            self.detector.reset()
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
                power = self._read_signal()

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

                # csv log
                if csv_file is not None:
                    slope_str = f"{det['slope']:.6f}" if det['slope'] is not None else ""
                    resid_str = f"{det['residual_std']:.6f}" if det['residual_std'] is not None else ""
                    csv_file.write(
                        f"{elapsed:.2f},{time.time():.2f},"
                        f"{heater_status['temperature']:.2f},{heater_status['target']:.1f},"
                        f"{power:.4f},{slope_str},{resid_str},{det['sample_count']}\n"
                    )
        finally:
            if csv_file is not None:
                csv_file.close()

    cmd_HEATSOAK_CALIBRATE_help = (
        "Observe heater for DURATION seconds to characterize steady state "
        "and suggest threshold values"
    )
    def cmd_HEATSOAK_CALIBRATE(self, gcmd):
        duration = gcmd.get_float('DURATION', self.max_duration, above=60.)

        target = self.heater.target_temp
        if target <= 0:
            raise gcmd.error(f'Heater {self.heater_name} has no target temperature, skipping calibration')

        reactor = self.printer.get_reactor()
        start_status = self.heater.get_status(reactor.monotonic())
        start_temp = start_status['temperature']
        signal_source = 'integral' if self.use_integral else 'pwm'

        gcmd.respond_info(f"heatsoak calibrate: starting (target={target:.1f}C, start={start_temp:.1f}C, signal={signal_source}, duration={duration:.0f}s)")

        records = []  # (elapsed, power, slope, residual_std)
        csv_file = None
        try:
            if self.log_path:
                os.makedirs(self.log_path, exist_ok=True)
                filename = os.path.join(self.log_path, f"cal_{int(start_temp)}Cto{int(target)}C_{int(time.time())}.csv")
                csv_file = open(filename, 'w')
                csv_file.write(f"# CALIBRATION start_temp={start_temp:.2f},target={target:.1f},signal={signal_source},duration={duration:.0f},start_unix={int(time.time())}\n")
                csv_file.write("elapsed_s,unix_time,temp,target,power,slope,residual_std,sample_count\n")
                gcmd.respond_info(f"heatsoak calibrate: logging to {filename}")

            self.detector.reset()
            start_time = reactor.monotonic()
            eventtime = start_time

            # wait for heater to reach target temp before starting observation
            while not self.printer.is_shutdown() and self.heater.check_busy(eventtime):
                eventtime = reactor.pause(eventtime + 1.)

            # observation loop - runs to duration, no threshold checks
            while not self.printer.is_shutdown():
                eventtime = reactor.pause(eventtime + self.sample_interval)
                elapsed = eventtime - start_time
                if elapsed > duration:
                    break
                heater_status = self.heater.get_status(eventtime)
                power = self._read_signal()
                self.detector.add_sample(eventtime, power)
                det = self.detector.get_status()
                records.append((elapsed, power, det['slope'], det['residual_std']))

                # progress update
                slope_disp = f"{det['slope']:.5f}" if det['slope'] is not None else "n/a"
                resid_disp = f"{det['residual_std']:.4f}" if det['residual_std'] is not None else "n/a"
                gcmd.respond_info(f"calibrate: t={elapsed:.0f}s power={power:.3f} slope={slope_disp} resid={resid_disp} n={det['sample_count']}")

                # csv log
                if csv_file is not None:
                    slope_csv = f"{det['slope']:.6f}" if det['slope'] is not None else ""
                    resid_csv = f"{det['residual_std']:.6f}" if det['residual_std'] is not None else ""
                    csv_file.write(
                        f"{elapsed:.2f},{time.time():.2f},"
                        f"{heater_status['temperature']:.2f},{heater_status['target']:.1f},"
                        f"{power:.4f},{slope_csv},{resid_csv},{det['sample_count']}\n"
                    )

            # analysis: characterize the tail (presumed steady state)
            if self.printer.is_shutdown() or not records:
                gcmd.respond_info("heatsoak calibrate: no samples collected, nothing to analyze")
                return

            total_time = records[-1][0]
            tail_window_s = max(300., total_time * 0.25)
            cutoff = total_time - tail_window_s
            tail = [r for r in records
                    if r[0] >= cutoff and r[2] is not None and r[3] is not None]

            if len(tail) < 3:
                gcmd.respond_info("heatsoak calibrate: not enough valid tail samples for analysis")
                return

            max_slope = max(abs(r[2]) for r in tail)
            max_resid = max(r[3] for r in tail)
            avg_power = sum(r[1] for r in tail) / len(tail)
            rec_slope = max_slope * 2.
            rec_resid = max_resid * 2.

            # Sanity check: split the tail in half and compare. If the first
            # half had significantly more drift than the second, the bed was
            # still settling - the "tail" isn't yet a real steady-state
            # noise floor and the recommendation will be too lenient.
            mid = len(tail) // 2
            first_half_max_slope = max(abs(r[2]) for r in tail[:mid]) if mid > 0 else 0.
            second_half_max_slope = max(abs(r[2]) for r in tail[mid:]) if mid < len(tail) else 0.
            tail_still_trending = (
                first_half_max_slope > second_half_max_slope * 1.5
                and first_half_max_slope > 0
            )

            if csv_file is not None:
                csv_file.write(
                    f"# ANALYSIS tail_samples={len(tail)},tail_window_s={tail_window_s:.0f},"
                    f"max_abs_slope={max_slope:.6f},max_residual_std={max_resid:.6f},"
                    f"avg_power={avg_power:.4f},"
                    f"first_half_max_slope={first_half_max_slope:.6f},"
                    f"second_half_max_slope={second_half_max_slope:.6f},"
                    f"tail_still_trending={int(tail_still_trending)},"
                    f"recommended_slope_threshold={rec_slope:.6f},"
                    f"recommended_residual_threshold={rec_resid:.6f}\n"
                )

            gcmd.respond_info("---")
            gcmd.respond_info(f"heatsoak calibrate: analyzed tail of {len(tail)} samples (last {tail_window_s:.0f}s of run)")
            gcmd.respond_info(f"heatsoak calibrate: tail max|slope|={max_slope:.6f}, max resid={max_resid:.5f}, avg power={avg_power:.3f}")
            if tail_still_trending:
                gcmd.respond_info(
                    f"heatsoak calibrate: WARNING - slope still decaying through end of run "
                    f"(first-half max {first_half_max_slope:.6f}, second-half max {second_half_max_slope:.6f}). "
                    f"Recommended thresholds may be too lenient. Re-run with longer DURATION."
                )
            gcmd.respond_info(f"heatsoak calibrate: recommended slope_threshold: {rec_slope:.5f}")
            gcmd.respond_info(f"heatsoak calibrate: recommended residual_threshold: {rec_resid:.4f}")
            gcmd.respond_info("---")
        finally:
            if csv_file is not None:
                csv_file.close()

def load_config(config):
    return Heatsoak(config)