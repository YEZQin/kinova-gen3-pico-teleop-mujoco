import importlib

import numpy as np
import pytest


def pid_class():
    spec = importlib.util.find_spec('kinova_teleop.cartesian_pid')
    assert spec is not None, 'Cartesian PID implementation is required'
    return importlib.import_module('kinova_teleop.cartesian_pid').CartesianPID


def test_ideal_kinematic_step_reaches_ninety_percent_near_point_seven_seconds():
    pid = pid_class()()
    position = np.zeros(3)
    target = np.array([.01, 0., 0.])
    reached = None
    for step in range(200):
        velocity = pid.update(target, position, step * .01)
        assert np.linalg.norm(velocity) <= .05 + 1e-12
        position = position + velocity * .01
        if position[0] >= .009 and reached is None:
            reached = (step + 1) * .01
    assert .60 <= reached <= .80
    assert abs(position[0] - .01) < .0002


def test_saturation_does_not_wind_up_and_reset_clears_history():
    pid = pid_class()(kp=3.3, ki=1., kd=0.)
    for step in range(1000):
        velocity = pid.update([1., 1., 1.], [0., 0., 0.], step * .01)
        assert np.linalg.norm(velocity) <= .05 + 1e-12
    np.testing.assert_allclose(pid.update([0., 0., 0.], [0., 0., 0.], 10.), 0.)
    pid.update([.001, 0., 0.], [0., 0., 0.], 10.01)
    pid.reset()
    np.testing.assert_allclose(pid.update([0., 0., 0.], [0., 0., 0.], 0.), 0.)


def test_derivative_is_filtered_measurement_not_target_kick():
    pid = pid_class()(kp=0., ki=0., kd=1.)
    np.testing.assert_allclose(pid.update([0., 0., 0.], [0., 0., 0.], 0.), 0.)
    np.testing.assert_allclose(pid.update([1., 0., 0.], [0., 0., 0.], .01), 0.)
    output = pid.update([1., 0., 0.], [.001, 0., 0.], .02)
    assert -.05 < output[0] < 0.


@pytest.mark.parametrize('kwargs', [{'kp': -1}, {'ki': np.nan}, {'kd': np.inf}, {'max_speed': 0}, {'max_speed': .051}])
def test_invalid_parameters_fail_closed(kwargs):
    with pytest.raises(ValueError):
        pid_class()(**kwargs)


@pytest.mark.parametrize('target,measured,now', [([0, 0], [0, 0, 0], 0), ([np.nan, 0, 0], [0, 0, 0], 0), ([0, 0, 0], [np.inf, 0, 0], 0), ([0, 0, 0], [0, 0, 0], np.nan)])
def test_invalid_samples_fail_closed(target, measured, now):
    with pytest.raises(ValueError):
        pid_class()().update(target, measured, now)


def test_nonincreasing_clock_is_rejected_without_corrupting_state():
    pid = pid_class()()
    pid.update([.01, 0, 0], [0, 0, 0], 1.)
    for now in [1., .5]:
        with pytest.raises(ValueError):
            pid.update([.01, 0, 0], [0, 0, 0], now)
    assert np.isfinite(pid.update([.01, 0, 0], [0, 0, 0], 1.01)).all()
