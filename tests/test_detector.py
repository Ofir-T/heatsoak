import pytest
from heatsoak import SteadyStateDetector


# Default thresholds tight enough that only genuinely flat signals pass.
def make_detector(min_samples=4):
    return SteadyStateDetector(
        window_size=10,
        slope_threshold=0.001,
        residual_threshold=0.01,
        min_samples=min_samples,
    )


# --- Construction & windowing ---

def test_window_evicts_old_samples():
    d = SteadyStateDetector(window_size=3, slope_threshold=0.001,
                            residual_threshold=0.01, min_samples=2)
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
    d = make_detector()
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


def test_below_min_samples_returns_false():
    d = make_detector(min_samples=4)
    d.add_sample(0, 0.1)
    d.add_sample(1, 0.1)
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


# --- reset ---

def test_reset_clears_samples_and_status():
    d = make_detector()
    for t in range(5):
        d.add_sample(t, 0.1)
    d.reset()
    assert len(d.samples) == 0
    assert d.get_status() == {'sample_count': 0, 'slope': None, 'residual_std': None}
