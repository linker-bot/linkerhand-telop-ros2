import pytest
import numpy as np
from linkerhand_retarget.linkerhand.filter import (
    LCFilter,
    MultiChannelLCFilter,
    AdaptiveLCFilter,
    KalmanFilter,
    MultiChannelKalmanFilter,
    AdaptiveKalmanFilter,
    SavitzkyGolayFilter,
    MultiChannelSavitzkyGolayFilter,
    AdaptiveSavitzkyGolayFilter,
    apply_lc_filter,
)


class TestLCFilter:
    def test_initialization(self):
        f = LCFilter(alpha=0.5, initial_value=1.0)
        assert f.alpha == 0.5
        assert f.filtered_value == 1.0

    def test_invalid_alpha(self):
        with pytest.raises(ValueError):
            LCFilter(alpha=0)
        with pytest.raises(ValueError):
            LCFilter(alpha=1.5)

    def test_update(self):
        f = LCFilter(alpha=0.5)
        result = f.update(10.0)
        assert result == 5.0  # 0.5 * 10 + 0.5 * 0

    def test_update_chain(self):
        f = LCFilter(alpha=0.5)
        f.update(10.0)  # 5.0
        result = f.update(20.0)  # 0.5 * 20 + 0.5 * 5 = 12.5
        assert result == 12.5

    def test_update_array(self):
        f = LCFilter(alpha=0.5)
        result = f.update_array([10.0, 20.0, 30.0])
        assert len(result) == 3

    def test_reset(self):
        f = LCFilter(alpha=0.5, initial_value=5.0)
        f.update(10.0)
        f.reset(initial_value=0.0)
        assert f.filtered_value == 0.0
        assert len(f.history_raw) == 0


class TestMultiChannelLCFilter:
    def test_initialization(self):
        f = MultiChannelLCFilter(num_channels=3, alpha=0.5)
        assert f.num_channels == 3
        assert len(f.filters) == 3

    def test_invalid_channels(self):
        with pytest.raises(ValueError):
            MultiChannelLCFilter(num_channels=3, initial_values=[1.0, 2.0])

    def test_update(self):
        f = MultiChannelLCFilter(num_channels=3, alpha=0.5)
        result = f.update([10.0, 20.0, 30.0])
        assert result == [5.0, 10.0, 15.0]

    def test_update_channel(self):
        f = MultiChannelLCFilter(num_channels=3, alpha=0.5)
        result = f.update_channel(1, 20.0)
        assert result == 10.0

    def test_invalid_channel_index(self):
        f = MultiChannelLCFilter(num_channels=3)
        with pytest.raises(ValueError):
            f.update_channel(5, 10.0)


class TestAdaptiveLCFilter:
    def test_initialization(self):
        f = AdaptiveLCFilter(alpha_min=0.05, alpha_max=0.3)
        assert f.alpha_min == 0.05
        assert f.alpha_max == 0.3

    def test_adaptive_update_fast_change(self):
        f = AdaptiveLCFilter(alpha_min=0.05, alpha_max=0.3, change_threshold=0.1)
        f.update(0.0)  # initial
        result = f.update(10.0)  # large change, should use alpha_max
        assert f.alpha == 0.3

    def test_adaptive_update_slow_change(self):
        f = AdaptiveLCFilter(alpha_min=0.05, alpha_max=0.3, change_threshold=0.1)
        f.update(0.0)  # initial
        f.update(0.01)  # small change
        result = f.update(0.02)  # small change
        assert f.alpha == 0.05


class TestKalmanFilter:
    def test_initialization(self):
        kf = KalmanFilter(process_variance=1e-5, measurement_variance=0.1)
        assert kf.process_variance == 1e-5
        assert kf.measurement_variance == 0.1

    def test_update(self):
        kf = KalmanFilter(process_variance=1e-5, measurement_variance=0.1)
        result = kf.update(10.0)
        assert result > 0 and result < 10.0

    def test_update_batch(self):
        kf = KalmanFilter(process_variance=1e-5, measurement_variance=0.1)
        result = kf.update_batch([10.0, 20.0, 30.0])
        assert len(result) == 3

    def test_reset(self):
        kf = KalmanFilter(process_variance=1e-5, measurement_variance=0.1)
        kf.update(10.0)
        kf.reset(initial_value=0.0)
        assert kf.x_hat == 0.0


class TestMultiChannelKalmanFilter:
    def test_initialization(self):
        mkf = MultiChannelKalmanFilter(num_channels=3)
        assert mkf.num_channels == 3
        assert len(mkf.filters) == 3

    def test_update(self):
        mkf = MultiChannelKalmanFilter(num_channels=3)
        result = mkf.update([10.0, 20.0, 30.0])
        assert len(result) == 3

    def test_update_channel(self):
        mkf = MultiChannelKalmanFilter(num_channels=3)
        result = mkf.update_channel(1, 20.0)
        assert result > 0

    def test_invalid_channel(self):
        mkf = MultiChannelKalmanFilter(num_channels=3)
        with pytest.raises(ValueError):
            mkf.update_channel(5, 10.0)


class TestAdaptiveKalmanFilter:
    def test_initialization(self):
        akf = AdaptiveKalmanFilter(
            min_process_variance=1e-6,
            max_process_variance=1e-3,
        )
        assert akf.min_process_variance == 1e-6
        assert akf.max_process_variance == 1e-3


class TestSavitzkyGolayFilter:
    def test_initialization(self):
        sgf = SavitzkyGolayFilter(window_length=7, polyorder=2)
        assert sgf.window_length == 7
        assert sgf.polyorder == 2

    def test_invalid_window_length(self):
        with pytest.raises(ValueError):
            SavitzkyGolayFilter(window_length=6)  # even number

    def test_window_less_than_polyorder(self):
        with pytest.raises(ValueError):
            SavitzkyGolayFilter(window_length=3, polyorder=4)

    def test_update(self):
        sgf = SavitzkyGolayFilter(window_length=7, polyorder=2)
        result = sgf.update(10.0)
        assert isinstance(result, float)

    def test_buffer_not_full(self):
        sgf = SavitzkyGolayFilter(window_length=7, polyorder=2)
        for i in range(3):
            result = sgf.update(float(i))
            assert result == float(i)  # returns original when buffer not full

    def test_reset(self):
        sgf = SavitzkyGolayFilter(window_length=7, polyorder=2)
        sgf.update(10.0)
        sgf.reset()
        assert len(sgf.buffer) == 0


class TestMultiChannelSavitzkyGolayFilter:
    def test_initialization(self):
        msgf = MultiChannelSavitzkyGolayFilter(num_channels=3)
        assert msgf.num_channels == 3

    def test_update(self):
        msgf = MultiChannelSavitzkyGolayFilter(num_channels=3)
        result = msgf.update([10.0, 20.0, 30.0])
        assert len(result) == 3


class TestAdaptiveSavitzkyGolayFilter:
    def test_initialization(self):
        asgf = AdaptiveSavitzkyGolayFilter(min_window=5, max_window=13)
        assert asgf.min_window == 5
        assert asgf.max_window == 13

    def test_update(self):
        asgf = AdaptiveSavitzkyGolayFilter(min_window=5, max_window=13)
        result = asgf.update(10.0)
        assert isinstance(result, float)


class TestApplyLCFilter:
    def test_empty_list(self):
        result = apply_lc_filter([])
        assert result == []

    def test_single_value(self):
        result = apply_lc_filter([5.0], alpha=0.5)
        assert result == [5.0]

    def test_multiple_values(self):
        result = apply_lc_filter([10.0, 20.0, 30.0], alpha=0.5)
        assert len(result) == 3

    def test_invalid_alpha(self):
        with pytest.raises(ValueError):
            apply_lc_filter([1.0, 2.0], alpha=0)
