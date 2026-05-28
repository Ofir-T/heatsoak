import pytest
from heatsoak import SteadyStateDetector


# Default thresholds tight enough that only genuinely flat signals pass.
# window_size=4 keeps the True-expecting tests below short while still
# requiring a full window for is_stable() to return True.
def make_detector(window_size=4):
    return SteadyStateDetector(
        window_size=window_size,
        slope_threshold=0.001,
        residual_threshold=0.01,
    )


# --- Construction & windowing ---

def test_window_evicts_old_samples():
    d = SteadyStateDetector(window_size=3, slope_threshold=0.001,
                            residual_threshold=0.01)
    for t in range(5):
        d.add_sample(t, 0.1 * t)
    assert len(d.samples) == 3
    # Oldest two samples (t=0, t=1) should have dropped off; t=2 is now the front.
    assert d.samples[0] == (2, pytest.approx(0.2))


# --- is_stable decisions ---

def test_flat_signal_is_stable():
    d = make_detector()
    for t in range(5):
        d.add_sample(t, 0.1)
    assert d.is_stable() is True


def test_linear_decay_not_stable():
    d = make_detector()
    for t, p in [(0, 1.0), (1, 0.8), (2, 0.6), (3, 0.4)]:
        d.add_sample(t, p)
    assert d.is_stable() is False


def test_small_oscillation_with_zero_trend_is_stable():
    d = make_detector(window_size=6)
    # Alternating +/-0.005 around 0.105: slope ~0, residual well under 0.01.
    for t, p in [(0, 0.10), (1, 0.11), (2, 0.10), (3, 0.11), (4, 0.10), (5, 0.11)]:
        d.add_sample(t, p)
    assert d.is_stable() is True


def test_large_oscillation_not_stable():
    d = make_detector()
    # Symmetric arrangement: slope is exactly 0 but residuals are large.
    # slope-only would pass this; residual check rejects it.
    for t, p in [(0, 0.0), (1, 1.0), (2, 1.0), (3, 0.0)]:
        d.add_sample(t, p)
    assert d.is_stable() is False


def test_partial_window_returns_false():
    # Window not yet full: must return False regardless of how flat the data is.
    d = make_detector(window_size=4)
    d.add_sample(0, 0.1)
    d.add_sample(1, 0.1)
    assert d.is_stable() is False


def test_split_window_catches_curving_signal():
    # Both half-window slopes are below the absolute threshold, but they
    # differ enough that the signal is clearly still curving. The split-window
    # check rejects this case; the previous full-window absolute-slope check
    # would have passed it.
    d = SteadyStateDetector(window_size=8, slope_threshold=0.01,
                            residual_threshold=0.02)
    # First half slope: +0.009 (rising). Second half slope: -0.005 (falling).
    # |first - second| = 0.014, above slope_threshold=0.01.
    samples = [(0, 0.500), (1, 0.509), (2, 0.518), (3, 0.527),
               (4, 0.527), (5, 0.522), (6, 0.517), (7, 0.512)]
    for t, p in samples:
        d.add_sample(t, p)
    assert d.is_stable() is False


# --- get_status semantics ---

def test_get_status_with_empty_window():
    d = make_detector()
    assert d.get_status() == {'sample_count': 0, 'slope': None, 'residual_std': None}


def test_get_status_with_two_samples_has_slope_but_no_resid():
    d = make_detector()
    d.add_sample(0, 0.5)
    d.add_sample(1, 0.4)
    status = d.get_status()
    assert status['sample_count'] == 2
    assert status['slope'] == pytest.approx(-0.1)
    assert status['residual_std'] is None


def test_get_status_with_three_samples_has_both():
    d = make_detector()
    for t, p in [(0, 1.0), (1, 0.8), (2, 0.6)]:
        d.add_sample(t, p)
    status = d.get_status()
    assert status['sample_count'] == 3
    assert status['slope'] == pytest.approx(-0.2)
    assert status['residual_std'] == pytest.approx(0.0, abs=1e-9)


# --- relative slope check ---

def test_relative_slope_rejects_mid_decay():
    # slope=-0.0015 passes absolute threshold=0.002, but |slope/power|=0.006 fails relative=0.002
    d = SteadyStateDetector(window_size=4, slope_threshold=0.002,
                            residual_threshold=0.01, relative_slope_threshold=0.002)
    for t, p in [(0, 0.253), (1, 0.2515), (2, 0.250), (3, 0.2485)]:
        d.add_sample(t, p)
    assert d.is_stable() is False


def test_relative_slope_passes_genuine_steady_state():
    # slope≈0, power≈0.097: |slope/power| ≈ 0, well below threshold
    d = SteadyStateDetector(window_size=4, slope_threshold=0.002,
                            residual_threshold=0.01, relative_slope_threshold=0.002)
    for t in range(4):
        d.add_sample(t, 0.097)
    assert d.is_stable() is True


def test_relative_slope_disabled_when_zero():
    # Same mid-decay signal; with relative_slope_threshold=0, check is skipped
    d = SteadyStateDetector(window_size=4, slope_threshold=0.002,
                            residual_threshold=0.01, relative_slope_threshold=0.0)
    for t, p in [(0, 0.253), (1, 0.2515), (2, 0.250), (3, 0.2485)]:
        d.add_sample(t, p)
    assert d.is_stable() is True


# --- reset ---

def test_reset_clears_samples_and_status():
    d = make_detector()
    for t in range(5):
        d.add_sample(t, 0.1)
    d.reset()
    assert len(d.samples) == 0
    assert d.get_status() == {'sample_count': 0, 'slope': None, 'residual_std': None}
