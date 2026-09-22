import threading

import numpy as np
import pytest

from kinova_teleop.cartesian_pid import CartesianPID
import kinova_teleop.kortex_backend as backend_module
from test_kortex_backend import (
    _backend, _begin_pose_control, _Clock, _Connection, _feedback,
    _ManualThread, _target, _velocity,
)


@pytest.fixture(autouse=True)
def manual_watchdog(monkeypatch):
    monkeypatch.setattr(backend_module, '_watchdog_thread_factory', _ManualThread)


def test_optional_pid_changes_translation_but_default_remains_proportional():
    for pid, expected in [(None, .01), (CartesianPID(), .033)]:
        connection = _Connection(_feedback())
        backend = _backend(connection, translation_pid=pid)
        _begin_pose_control(backend)
        assert backend.command_pose(_target(position=(.01, 0, 0))).accepted
        np.testing.assert_allclose(_velocity(connection.base.sent[-1]), [expected, 0, 0, 0, 0, 0])
        backend.close()


@pytest.mark.parametrize('stop_method', ['hold', 'close'])
def test_stop_paths_clear_integral(stop_method):
    pid = CartesianPID(kp=0, ki=1, kd=0)
    connection = _Connection(_feedback())
    clock = _Clock()
    backend = _backend(connection, translation_pid=pid, monotonic=clock)
    _begin_pose_control(backend)
    backend.command_pose(_target(position=(.01, 0, 0)))
    clock.now = .1
    backend.command_pose(_target(position=(.01, 0, 0)))
    assert connection.base.sent[-1].twist.linear_x > 0
    getattr(backend, stop_method)()
    np.testing.assert_allclose(pid.update([0, 0, 0], [0, 0, 0], 0), 0)
    backend.close()


def test_begin_control_clears_prior_pid_state():
    pid = CartesianPID(kp=0, ki=1, kd=0)
    backend = _backend(_Connection(_feedback()), translation_pid=pid)
    pid.update([.01, 0, 0], [0, 0, 0], 0)
    pid.update([.01, 0, 0], [0, 0, 0], .1)
    backend.begin_control()
    np.testing.assert_allclose(pid.update([0, 0, 0], [0, 0, 0], 0), 0)
    backend.close()


def test_pid_cap_cannot_exceed_backend_cap():
    with pytest.raises(ValueError, match='PID'):
        _backend(_Connection(_feedback()), translation_pid=CartesianPID(), max_linear_speed=.02)


def test_feedback_returning_after_stop_does_not_repopulate_pid(monkeypatch):
    pid = CartesianPID(kp=0, ki=1, kd=0)
    backend = _backend(_Connection(_feedback()), translation_pid=pid)
    _begin_pose_control(backend)
    entered, release = threading.Event(), threading.Event()

    def delayed_pose():
        entered.set()
        assert release.wait(1)
        return _target()

    monkeypatch.setattr(backend, 'current_pose', delayed_pose)
    results = []
    worker = threading.Thread(target=lambda: results.append(backend.command_pose(_target(position=(.01, 0, 0)))))
    worker.start()
    assert entered.wait(1)
    backend.hold()
    release.set()
    worker.join(1)
    assert not worker.is_alive()
    assert not results[0].accepted
    # A stale PID update would establish a timestamp and reject this first sample.
    np.testing.assert_allclose(pid.update([0, 0, 0], [0, 0, 0], 0), 0)
    backend.close()
