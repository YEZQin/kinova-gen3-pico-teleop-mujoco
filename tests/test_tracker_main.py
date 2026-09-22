import io
from types import SimpleNamespace

import numpy as np
import pytest

from kinova_teleop import tracker_main as cli
from kinova_teleop.pose_mapping import Pose


def test_default_is_ideal_benchmark_without_hardware(monkeypatch, capsys):
    monkeypatch.setattr(cli, 'hardware_backend', lambda *a: pytest.fail('hardware accessed'))
    assert cli.main([]) == 0
    assert 'ideal_10mm_step_t90_s=0.675' in capsys.readouterr().out


@pytest.mark.parametrize('arguments', [
    ['--scale', 'nan'], ['--control-hz', '41'], ['--max-linear-speed', '.1'],
    ['--kp', '-1'], ['--samples', '0'], ['--enable-hardware'],
    ['--rotation', *(['0'] * 9)],
])
def test_bad_args_fail_before_sdk(arguments, monkeypatch):
    monkeypatch.setattr(cli, 'hardware_backend', lambda *a: pytest.fail('hardware accessed'))
    assert cli.main(arguments) == 2


def test_keyboard_pause_reanchor_and_exit(monkeypatch):
    calls = []
    position = iter([1., 1.001, 2., 3., 3.001])
    source = SimpleNamespace(read=lambda: SimpleNamespace(
        valid=True, position=np.array([next(position), 0., 0.]), timestamp_ns=1))
    backend = SimpleNamespace(
        begin_control=lambda: calls.append('begin'),
        current_pose=lambda: Pose(np.array([.4, 0., .3]), np.array([1., 0., 0., 0.])),
        command_pose=lambda target: calls.append(target.position.copy()) or SimpleNamespace(accepted=True),
        hold=lambda: calls.append('stop'),
    )
    keys = iter([None, None, ' ', ' ', None, '\x1b'])
    monkeypatch.setattr(cli, 'key', lambda: next(keys))
    monkeypatch.setattr(cli.time, 'sleep', lambda _: None)
    cli.run_follow(cli.parser().parse_args([]), source, backend, np.eye(3), io.StringIO())
    assert calls[0] == 'begin' and calls[2:4] == ['stop', 'begin']
    np.testing.assert_allclose(calls[1], [.401, 0., .3])
    np.testing.assert_allclose(calls[4], [.401, 0., .3])


def test_check_input_cleanup_failure_returns_error(monkeypatch):
    from kinova_teleop import motion_tracker
    class Source:
        def __init__(self, *_): pass
        def read(self):
            return SimpleNamespace(valid=True, timestamp_ns=1, position=[0., 0., 0.])
        def close(self): raise RuntimeError('close failed')
    monkeypatch.setattr(motion_tracker, 'MotionTrackerInput', Source)
    assert cli.main(['--check-input', '--tracker-serial', 'test', '--samples', '1']) == 2


@pytest.mark.parametrize('ready', [True, False])
def test_hardware_readonly_admission_before_backend(monkeypatch, ready):
    from kinova_teleop import hardware_profile, kortex_transport, kortex_backend, preflight
    from kinova_teleop.cartesian_pid import CartesianPID
    calls = []
    class Connection:
        def __init__(self, _): calls.append('construct')
        def connect(self, **kwargs): calls.append(('connect', kwargs))
        def close(self, **kwargs): calls.append(('close', kwargs)); return True
    def report(*_):
        calls.append('inspect')
        return None
    def require(_):
        if not ready: raise ValueError('wrong robot')
    def create_backend(connection, **kwargs):
        calls.append('backend')
        assert isinstance(kwargs['translation_pid'], CartesianPID)
        return SimpleNamespace(current_pose=lambda: Pose(np.array([.4, 0., .3]), np.array([1., 0., 0., 0.])))
    monkeypatch.setattr(hardware_profile, 'validate_kortex_runtime', lambda: None)
    monkeypatch.setattr(cli.getpass, 'getpass', lambda _: 'test-only')
    monkeypatch.setattr(kortex_transport, 'KortexConnection', Connection)
    monkeypatch.setattr(kortex_backend, 'KortexBackend', create_backend)
    monkeypatch.setattr(preflight, 'run_kortex_readonly_preflight', report)
    monkeypatch.setattr(preflight, 'require_live_kortex_ready', require)
    args = cli.parser().parse_args(['--robot-ip', '192.168.1.10', '--workspace-min', '0', '-.2', '.1',
                                    '--workspace-max', '.8', '.2', '.5'])
    if ready:
        cli.hardware_backend(args, CartesianPID())
        assert calls.index(('close', {'send_stop': False})) < calls.index('backend')
    else:
        with pytest.raises(ValueError, match='wrong robot'):
            cli.hardware_backend(args, CartesianPID())
        assert 'backend' not in calls and calls.count('construct') == 1


@pytest.mark.parametrize('interrupt', [False, True])
@pytest.mark.parametrize('close_ok', [False, True])
def test_hardware_exit_always_closes_robot_before_tracker(monkeypatch, tmp_path, interrupt, close_ok):
    from kinova_teleop import motion_tracker
    calls = []
    class Source:
        def __init__(self, *_): pass
        def close(self): calls.append('tracker_close')
    class Backend:
        def close(self): calls.append('robot_stop_close'); return close_ok
    def follow(*_):
        if interrupt: raise KeyboardInterrupt
        # Normal return from run_follow is its Esc exit path.
    monkeypatch.setattr(cli, 'validate', lambda _: np.eye(3))
    monkeypatch.setattr(motion_tracker, 'MotionTrackerInput', Source)
    monkeypatch.setattr(cli, 'wait_tracker', lambda *_: None)
    monkeypatch.setattr(cli, 'hardware_backend', lambda *_: Backend())
    monkeypatch.setattr(cli, 'run_follow', follow)
    code = cli.main(['--enable-hardware', '--tracker-serial', 'test', '--log', str(tmp_path/'run.csv')])
    assert calls == ['robot_stop_close', 'tracker_close']
    assert code == (0 if close_ok else 2)
