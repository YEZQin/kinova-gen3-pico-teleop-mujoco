"""Receive actual Motion packets from the Windows SDK, including timestamp-less v1.1.1.

Arrival timestamps measure transport freshness, not sensor acquisition age.
No robot API or device-control API is called by this module.
"""
import ctypes
import importlib.util
import json
import math
from pathlib import Path
import threading
import time


class MotionPacketBuffer:
    def __init__(self):
        self._lock = threading.Lock()
        self._frame = None

    def invalidate(self):
        with self._lock:
            self._frame = None

    def ingest(self, raw, arrival_ns):
        try:
            outer = json.loads(raw)
            value = outer.get('value', {})
            if isinstance(value, str):
                value = json.loads(value)
            if not isinstance(value, dict):
                raise ValueError('value must be an object')
            if 'Motion' not in value:
                return False
            motion = value['Motion']
            joints = motion['joints']
            count = motion.get('len', len(joints))
            if isinstance(count, bool) or not isinstance(count, int) or count != len(joints):
                raise ValueError('Motion len disagrees with cached joint array')
            if not isinstance(joints, list) or not 1 <= len(joints) <= 3:
                raise ValueError('invalid tracker count')
            serials, poses = [], []
            for joint in joints:
                serial = joint['sn']
                pose = tuple(float(v) for v in joint['p'].split(','))
                if (not isinstance(serial, str) or not serial or serial in serials
                        or len(pose) != 7 or not all(math.isfinite(v) for v in pose)
                        or not any(pose[3:])):
                    raise ValueError('invalid tracker data')
                serials.append(serial)
                poses.append(pose)
            stamp = motion.get('timeStampNs')
            if stamp is None:  # Only absent fields permit the legacy arrival clock.
                stamp, source = arrival_ns, 'host_packet_arrival'
            else:
                if isinstance(stamp, bool) or not isinstance(stamp, int) or stamp <= 0:
                    raise ValueError('invalid device timestamp')
                source = 'device_motion'
            frame = dict(timestamp_ns=stamp, received_ns=arrival_ns,
                         time_source=source, serials=tuple(serials), poses=tuple(poses))
            with self._lock:
                self._frame = frame
            return True
        except (ValueError, TypeError, KeyError, AttributeError):
            self.invalidate()
            return False

    def snapshot(self):
        with self._lock:
            if self._frame is None:
                raise ValueError('No valid Motion packet received')
            return dict(self._frame)


class _DeviceState(ctypes.Structure):
    _fields_ = [('device', ctypes.c_char * 32), ('data', ctypes.c_char * 16352)]


class PacketMotionSdk:
    """Own one native receive-only connection; callback copies data before return."""
    def __init__(self, dll_path=None, serial=None):
        self._path = dll_path
        self._buffer = MotionPacketBuffer()
        self._dll = None
        self._callback = None
        self._closing = False
        self._device = None
        self._serial = serial
        self._receive_lock = threading.Lock()

    def init(self):
        if self._dll is not None:
            return
        path = self._path
        if path is None:
            spec = importlib.util.find_spec('xrobotoolkit_sdk')
            if spec is None or not spec.origin:
                raise RuntimeError('XRoboToolkit Windows binding not found; run build_tracker_binding.ps1')
            path = Path(spec.origin).parent / 'PXREARobotSDK.dll'
        dll = ctypes.CDLL(str(path))
        callback_type = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_int,
                                        ctypes.c_int, ctypes.c_void_p)
        self._callback = callback_type(self._receive)
        dll.PXREAInit.argtypes = [ctypes.c_void_p, callback_type, ctypes.c_uint]
        dll.PXREAInit.restype = ctypes.c_int
        dll.PXREADeinit.argtypes = []
        dll.PXREADeinit.restype = ctypes.c_int
        code = dll.PXREAInit(None, self._callback, 0xffffffff)
        if code != 0:
            dll.PXREADeinit()
            raise RuntimeError(f'XR SDK init failed ({code})')
        self._dll = dll

    def _receive(self, context, kind, status, pointer):
        if self._closing:
            return
        if kind in (1 << 3, 1 << 5):  # server disconnected or headset missing
            self._buffer.invalidate()
            return
        if kind != 1 << 25 or not pointer:
            return
        try:
            message = ctypes.cast(pointer, ctypes.POINTER(_DeviceState)).contents
            device = bytes(message.device)
            with self._receive_lock:
                if self._device is not None and device != self._device:
                    return
                if self._buffer.ingest(bytes(message.data), time.monotonic_ns()):
                    if self._serial is not None and self._serial not in self._buffer.snapshot()['serials']:
                        self._buffer.invalidate()
                        return
                    self._device = device
        except Exception:
            self._buffer.invalidate()

    def get_motion_snapshot(self):
        return self._buffer.snapshot()

    def close(self):
        self._closing = True
        self._buffer.invalidate()
        if self._dll is not None:
            dll, self._dll = self._dll, None
            if dll.PXREADeinit() != 0:
                raise RuntimeError('XR SDK cleanup failed')
