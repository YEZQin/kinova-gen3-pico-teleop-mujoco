# PICO UDP Kinova Gen3 Teleoperation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a stable L20-derived PICO Unity/OpenXR UDP path that lets the left controller command the MuJoCo Kinova Gen3 end effector in relative 6DoF while Grip is held, with a simulator-independent backend boundary for future hardware work.

**Architecture:** A fixed binary protocol and L20-style UDP discovery adapter produce validated `ControllerSample` values on native Windows. A release-gated Grip state machine maps relative controller motion to an end-effector pose, while a generic controller commands a tested `MuJoCoBackend` through `EndEffectorTargetBackend`; the PICO Unity application, build script, and launcher remain separate from the L20 repository.

**Tech Stack:** Python 3.10+, pytest, NumPy, MuJoCo 3.3+, Unity 2022.3.62f3c1, Unity XR Management 4.4.0, Unity OpenXR 1.8.2, PICO Unity OpenXR SDK commit `3aa3e62bff41df618529eeb60ff02c29a515dafe`, C#, PowerShell 5.1+, Android ARM64/IL2CPP, UDP.

## Global Constraints

- Only the PICO left controller is supported.
- Grip is the only control input: activate at `> 0.9`, release at `< 0.8`, and require a release after startup or recovery before activation.
- Control is full relative 6DoF; default translation scale is `0.5`, orientation scale is 1:1.
- UDP discovery uses port `15031`, `KINOVA_DISCOVER_V1`, and `KINOVA_READY_V1`.
- Control packets are exactly 56 little-endian bytes with magic `KINVPICO` and protocol version `1`.
- Input becomes invalid after `0.2 s` without a new valid tracked frame.
- Use `kinova_gen3_mujoco/teleop_scene.xml`; never use `kinova/kinova.urdf` for Gen3.
- Native Windows is the supported runtime. The recommended path must not require WSL, XRoboToolkit, ADB Reverse, `adb tcpip`, or `adb connect`.
- The Unity/PICO versions listed in the Tech Stack are pinned; PICO OS must be 5.13.0 or newer.
- The L20 repository at `D:\yezq\26.7.11_L20_controll` is a read-only reference and must remain unchanged.
- Current production behavior is MuJoCo-only. Do not add a `KortexBackend` implementation or any physical-robot command path.
- Every production behavior is preceded by a failing test, and each RED failure must be observed for the expected reason.
- Existing Kinova tests must continue to pass after every task.

---

## File Structure

### Python runtime

- `kinova_teleop/pico_protocol.py`: fixed binary packet schema, encoding/decoding, modular sequence arithmetic.
- `kinova_teleop/pico_receiver.py`: UDP bind, discovery reply, and one-datagram receive.
- `kinova_teleop/pico_udp_input.py`: source locking, health counters, stale watchdog, and `ControllerSample` adapter.
- `kinova_teleop/pose_mapping.py`: release-gated Grip state machine and relative 6DoF mapping.
- `kinova_teleop/backend.py`: backend protocol, result type, and MuJoCo implementation.
- `kinova_teleop/teleop_controller.py`: backend-independent control loop.
- `kinova_teleop/main.py`: input/backend factories, diagnostics, CLI, and Viewer wiring.

### PICO Unity application

- `pico_unity_bridge/Assets/KinovaPico/`: packet, UDP bridge, bootstrap/status display, editor configuration, and EditMode tests.
- `pico_unity_bridge/Packages/`: pinned L20-compatible package manifest and lock.
- `pico_unity_bridge/ProjectSettings/`: L20-compatible Unity/OpenXR Android settings.

### Windows workflow

- `scripts/build_pico_udp_bridge.ps1`: deterministic Unity tests and APK build, optional ADB install.
- `scripts/start_pico_udp_teleop.ps1`: optional ADB app launch, UDP preflight, and MuJoCo launch.
- `docs/pico-udp-quickstart.md`: one-time setup, daily use, diagnostics, and safety behavior.

---

### Task 1: Fixed PICO Controller Protocol

**Files:**
- Create: `kinova_teleop/pico_protocol.py`
- Create: `tests/test_pico_protocol.py`

**Interfaces:**
- Produces:
  - `PICO_DISCOVER: bytes`
  - `PICO_READY: bytes`
  - `PICO_MAGIC: bytes`
  - `PICO_PACKET_SIZE: int`
  - `PicoControllerFrame`
  - `encode_pico_packet(frame: PicoControllerFrame) -> bytes`
  - `decode_pico_packet(payload: bytes, *, received_at: float, source: tuple[str, int]) -> PicoControllerFrame`
  - `sequence_delta(candidate: int, previous: int) -> int`
  - `sequence_is_newer(candidate: int, previous: int) -> bool`

- [ ] **Step 1: Write the failing protocol tests**

Create `tests/test_pico_protocol.py` with focused tests:

```python
import math
import struct

import pytest

from kinova_teleop.pico_protocol import (
    PICO_DISCOVER,
    PICO_PACKET_SIZE,
    PICO_READY,
    PicoControllerFrame,
    decode_pico_packet,
    encode_pico_packet,
    sequence_delta,
    sequence_is_newer,
)


def frame(**overrides) -> PicoControllerFrame:
    values = {
        "sequence": 42,
        "tracked": True,
        "source_time_us": 1_234_567,
        "position": (0.1, 1.2, -0.3),
        "quaternion_xyzw": (0.0, 0.0, 0.0, 1.0),
        "grip": 0.75,
        "received_at": 0.0,
        "source": ("192.168.1.4", 34000),
    }
    values.update(overrides)
    return PicoControllerFrame(**values)


def test_protocol_constants_and_exact_layout() -> None:
    payload = encode_pico_packet(frame())
    assert PICO_DISCOVER == b"KINOVA_DISCOVER_V1"
    assert PICO_READY == b"KINOVA_READY_V1"
    assert PICO_PACKET_SIZE == 56
    assert len(payload) == 56
    assert payload[:8] == b"KINVPICO"
    assert payload[8] == 1
    assert struct.unpack_from("<I", payload, 12)[0] == 42
    assert struct.unpack_from("<Q", payload, 16)[0] == 1_234_567


def test_round_trip_preserves_controller_values_and_receive_metadata() -> None:
    decoded = decode_pico_packet(
        encode_pico_packet(frame()),
        received_at=9.5,
        source=("10.0.0.3", 4567),
    )
    assert decoded.sequence == 42
    assert decoded.tracked
    assert decoded.position == pytest.approx((0.1, 1.2, -0.3))
    assert decoded.quaternion_xyzw == pytest.approx((0.0, 0.0, 0.0, 1.0))
    assert decoded.grip == pytest.approx(0.75)
    assert decoded.received_at == 9.5
    assert decoded.source == ("10.0.0.3", 4567)


@pytest.mark.parametrize(
    ("mutator", "message"),
    [
        (lambda data: data[:-1], "56 bytes"),
        (lambda data: b"BADMAGIC" + data[8:], "magic"),
        (lambda data: data[:8] + b"\x02" + data[9:], "version"),
        (lambda data: data[:10] + b"\x01\x00" + data[12:], "reserved"),
        (lambda data: data[:9] + b"\x80" + data[10:], "flags"),
    ],
)
def test_decode_rejects_invalid_envelope(mutator, message: str) -> None:
    with pytest.raises(ValueError, match=message):
        decode_pico_packet(
            mutator(encode_pico_packet(frame())),
            received_at=1.0,
            source=("127.0.0.1", 1),
        )


@pytest.mark.parametrize(
    "overrides",
    [
        {"position": (math.nan, 0.0, 0.0)},
        {"quaternion_xyzw": (0.0, 0.0, 0.0, 0.0)},
        {"grip": math.inf},
        {"grip": -0.01},
        {"grip": 1.01},
    ],
)
def test_encode_rejects_invalid_numeric_values(overrides) -> None:
    with pytest.raises(ValueError):
        encode_pico_packet(frame(**overrides))


def test_modular_sequence_arithmetic_accepts_wrap_and_rejects_old_values() -> None:
    assert sequence_delta(0, 0xFFFFFFFF) == 1
    assert sequence_is_newer(0, 0xFFFFFFFF)
    assert sequence_delta(8, 5) == 3
    assert not sequence_is_newer(5, 5)
    assert not sequence_is_newer(4, 5)
```

- [ ] **Step 2: Run the protocol tests and confirm RED**

Run:

```powershell
.\.venv\Scripts\python.exe -m pytest tests/test_pico_protocol.py -q
```

Expected: collection fails with `ModuleNotFoundError: kinova_teleop.pico_protocol`.

- [ ] **Step 3: Implement the immutable protocol and validation**

Create `kinova_teleop/pico_protocol.py` around one explicit struct:

```python
from __future__ import annotations

from dataclasses import dataclass
import math
import struct

PICO_DISCOVER = b"KINOVA_DISCOVER_V1"
PICO_READY = b"KINOVA_READY_V1"
PICO_MAGIC = b"KINVPICO"
PICO_VERSION = 1
TRACKED_FLAG = 0x01
_PACKET = struct.Struct("<8sBBHIQ8f")
PICO_PACKET_SIZE = _PACKET.size


@dataclass(frozen=True)
class PicoControllerFrame:
    sequence: int
    tracked: bool
    source_time_us: int
    position: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]
    grip: float
    received_at: float
    source: tuple[str, int]


def sequence_delta(candidate: int, previous: int) -> int:
    return (int(candidate) - int(previous)) & 0xFFFFFFFF


def sequence_is_newer(candidate: int, previous: int) -> bool:
    delta = sequence_delta(candidate, previous)
    return 0 < delta < 0x80000000
```

Add `_validate_frame_values`, `encode_pico_packet`, and
`decode_pico_packet`. Validate unsigned ranges, shapes, finite values,
Grip `[0,1]`, nonzero quaternion norm, exact packet size, exact magic/version,
reserved zero, and undefined flag bits zero. Normalize the decoded quaternion
before returning it. For `tracked=False`, encode zero position, identity
quaternion, and zero Grip regardless of supplied pose values.

- [ ] **Step 4: Run focused and regression tests**

```powershell
.\.venv\Scripts\python.exe -m pytest tests/test_pico_protocol.py -q
.\.venv\Scripts\python.exe -m pytest -q
```

Expected: protocol tests and all pre-existing tests pass.

- [ ] **Step 5: Commit the protocol**

```powershell
git add kinova_teleop/pico_protocol.py tests/test_pico_protocol.py
git commit -m "feat: define PICO controller UDP protocol"
```

---

### Task 2: UDP Receiver, Source Lock, and Input Watchdog

**Files:**
- Create: `kinova_teleop/pico_receiver.py`
- Create: `kinova_teleop/pico_udp_input.py`
- Create: `tests/test_pico_receiver.py`
- Create: `tests/test_pico_udp_input.py`

**Interfaces:**
- Consumes: Task 1 protocol constants, decoder, frame, and sequence helpers.
- Produces:
  - `PicoUdpReceiver(host: str = "0.0.0.0", port: int = 15031)`
  - `PicoUdpReceiver.receive_once(timeout: float | None = None) -> PicoControllerFrame | None`
  - `PicoStreamHealth` immutable snapshot
  - `PicoUdpInput(receiver=None, *, stale_after=0.2, monotonic=time.monotonic)`
  - `PicoUdpInput.read() -> ControllerSample`
  - `PicoUdpInput.health() -> PicoStreamHealth`

- [ ] **Step 1: Write failing receiver tests**

Create `tests/test_pico_receiver.py`:

```python
import socket

import pytest

from kinova_teleop.pico_protocol import (
    PICO_DISCOVER,
    PICO_READY,
    encode_pico_packet,
)
from kinova_teleop.pico_receiver import PicoUdpReceiver

from kinova_teleop.pico_protocol import PicoControllerFrame


def make_frame(**overrides) -> PicoControllerFrame:
    values = {
        "sequence": 42,
        "tracked": True,
        "source_time_us": 1_234_567,
        "position": (0.1, 1.2, -0.3),
        "quaternion_xyzw": (0.0, 0.0, 0.0, 1.0),
        "grip": 0.75,
        "received_at": 0.0,
        "source": ("192.168.1.4", 34000),
    }
    values.update(overrides)
    return PicoControllerFrame(**values)


def test_receiver_replies_to_discovery_and_does_not_return_a_frame() -> None:
    with PicoUdpReceiver("127.0.0.1", 0) as receiver:
        address = receiver.address
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            sender.settimeout(1.0)
            sender.sendto(PICO_DISCOVER, address)
            assert receiver.receive_once(timeout=1.0) is None
            reply, source = sender.recvfrom(128)
    assert reply == PICO_READY
    assert source == address


def test_receiver_decodes_binary_frame_with_local_source_and_time() -> None:
    with PicoUdpReceiver("127.0.0.1", 0, monotonic=lambda: 7.0) as receiver:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            sender.sendto(encode_pico_packet(make_frame()), receiver.address)
            sender_port = sender.getsockname()[1]
            result = receiver.receive_once(timeout=1.0)
    assert result is not None
    assert result.received_at == 7.0
    assert result.source == ("127.0.0.1", sender_port)


def test_receiver_ignores_unrelated_ascii_and_propagates_bad_protocol() -> None:
    with PicoUdpReceiver("127.0.0.1", 0) as receiver:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            sender.sendto(b"unrelated", receiver.address)
            assert receiver.receive_once(timeout=1.0) is None
            sender.sendto(b"KINVPICO", receiver.address)
            with pytest.raises(ValueError, match="56 bytes"):
                receiver.receive_once(timeout=1.0)
```

- [ ] **Step 2: Write failing source-lock and watchdog tests**

Create `tests/test_pico_udp_input.py` with an injectable receiver:

```python
from collections import deque

from kinova_teleop.pico_udp_input import PicoUdpInput

from kinova_teleop.pico_protocol import PicoControllerFrame


def make_frame(**overrides) -> PicoControllerFrame:
    values = {
        "sequence": 42,
        "tracked": True,
        "source_time_us": 1_234_567,
        "position": (0.1, 1.2, -0.3),
        "quaternion_xyzw": (0.0, 0.0, 0.0, 1.0),
        "grip": 0.75,
        "received_at": 0.0,
        "source": ("192.168.1.4", 34000),
    }
    values.update(overrides)
    return PicoControllerFrame(**values)


class FakeReceiver:
    def __init__(self, items=()):
        self.items = deque(items)
        self.closed = False

    def receive_once(self, *, timeout=0.0):
        assert timeout == 0.0
        if not self.items:
            raise BlockingIOError
        item = self.items.popleft()
        if isinstance(item, BaseException):
            raise item
        return item

    def close(self):
        self.closed = True


def test_input_drains_to_newest_frame_and_maps_timestamp_to_nanoseconds() -> None:
    receiver = FakeReceiver(
        [
            make_frame(sequence=1, source_time_us=10, received_at=5.0),
            make_frame(sequence=2, source_time_us=20, received_at=5.01),
        ]
    )
    source = PicoUdpInput(receiver=receiver, monotonic=lambda: 5.01)
    sample = source.read()
    assert sample.valid
    assert sample.timestamp_ns == 20_000
    assert sample.grip == 0.75


def test_input_allows_packet_gap_and_counts_loss() -> None:
    receiver = FakeReceiver(
        [
            make_frame(sequence=5, received_at=5.0),
            make_frame(sequence=8, received_at=5.01),
        ]
    )
    source = PicoUdpInput(receiver=receiver, monotonic=lambda: 5.01)
    assert source.read().valid
    assert source.health().dropped == 2


def test_duplicate_or_out_of_order_frame_invalidates_current_input() -> None:
    receiver = FakeReceiver(
        [
            make_frame(sequence=5, received_at=5.0),
            make_frame(sequence=5, received_at=5.01),
        ]
    )
    source = PicoUdpInput(receiver=receiver, monotonic=lambda: 5.01)
    assert not source.read().valid
    assert source.health().rejected == 1


def test_other_source_is_ignored_until_active_source_is_stale() -> None:
    first = make_frame(
        sequence=1,
        position=(1.0, 0.0, 0.0),
        received_at=5.0,
        source=("10.0.0.2", 3000),
    )
    other = make_frame(
        sequence=1,
        position=(2.0, 0.0, 0.0),
        received_at=5.05,
        source=("10.0.0.3", 4000),
    )
    receiver = FakeReceiver([first, other])
    now = [5.05]
    source = PicoUdpInput(receiver=receiver, monotonic=lambda: now[0])
    assert source.read().position.tolist() == list(first.position)
    assert source.health().foreign == 1
    now[0] = 5.21
    receiver.items.append(
        make_frame(sequence=2, received_at=5.21, source=("10.0.0.3", 4000))
    )
    assert source.read().valid
    assert source.health().active_source == ("10.0.0.3", 4000)


def test_untracked_or_stale_stream_returns_invalid_sample() -> None:
    now = [5.0]
    receiver = FakeReceiver([make_frame(sequence=1, received_at=5.0)])
    source = PicoUdpInput(receiver=receiver, monotonic=lambda: now[0])
    assert source.read().valid
    now[0] = 5.201
    assert not source.read().valid
    receiver.items.append(
        make_frame(sequence=2, tracked=False, received_at=5.202)
    )
    now[0] = 5.202
    assert not source.read().valid
```

- [ ] **Step 3: Run both files and confirm RED**

```powershell
.\.venv\Scripts\python.exe -m pytest `
  tests/test_pico_receiver.py tests/test_pico_udp_input.py -q
```

Expected: imports fail because both production modules are absent.

- [ ] **Step 4: Implement the L20-style UDP receiver**

Create `kinova_teleop/pico_receiver.py` with one socket and no worker thread:

```python
class PicoUdpReceiver:
    def __init__(self, host="0.0.0.0", port=15031, *, monotonic=time.monotonic):
        self._clock = monotonic
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind((host, port))
        self._closed = False

    @property
    def address(self) -> tuple[str, int]:
        host, port = self._socket.getsockname()
        return str(host), int(port)

    def receive_once(self, *, timeout=None):
        self._socket.settimeout(timeout)
        payload, source = self._socket.recvfrom(512)
        endpoint = str(source[0]), int(source[1])
        if payload == PICO_DISCOVER:
            self._socket.sendto(PICO_READY, endpoint)
            return None
        if not payload.startswith(PICO_MAGIC):
            return None
        return decode_pico_packet(
            payload, received_at=self._clock(), source=endpoint
        )
```

Add idempotent `close`, `__enter__`, and `__exit__`. Preserve
`BlockingIOError` for `timeout=0.0`, `socket.timeout` for positive timeouts,
and bind errors for the caller.

- [ ] **Step 5: Implement source locking and the local stale watchdog**

Create `kinova_teleop/pico_udp_input.py` with:

```python
@dataclass(frozen=True)
class PicoStreamHealth:
    active_source: tuple[str, int] | None
    accepted: int
    rejected: int
    dropped: int
    foreign: int
    stale: bool
    last_error: str


class PicoUdpInput:
    def __init__(
        self,
        receiver: PicoUdpReceiver | None = None,
        *,
        host: str = "0.0.0.0",
        port: int = 15031,
        stale_after: float = 0.2,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None: ...

    def read(self) -> ControllerSample: ...
    def health(self) -> PicoStreamHealth: ...
    def close(self) -> None: ...
```

On each `read`, drain `receive_once(timeout=0.0)` until
`BlockingIOError`. The first valid source becomes active. Ignore foreign
sources until `now - last_valid_received_at > stale_after`; then reset
sequence state and permit the next source. Accept modularly newer sequence
values, add `delta - 1` to `dropped`, and invalidate on active-source
duplicates, reverse order, decode errors, or untracked frames. Return an
invalid zero/identity `ControllerSample` for stale or invalid state. For a
valid frame, return NumPy copies and `timestamp_ns=source_time_us * 1000`.

- [ ] **Step 6: Run focused and complete Python tests**

```powershell
.\.venv\Scripts\python.exe -m pytest `
  tests/test_pico_receiver.py tests/test_pico_udp_input.py -q
.\.venv\Scripts\python.exe -m pytest -q
```

Expected: all pass without network threads left running.

- [ ] **Step 7: Commit the UDP input path**

```powershell
git add kinova_teleop/pico_receiver.py kinova_teleop/pico_udp_input.py `
  tests/test_pico_receiver.py tests/test_pico_udp_input.py
git commit -m "feat: receive PICO controller UDP input"
```

---

### Task 3: Release-Gated Grip State Machine

**Files:**
- Modify: `kinova_teleop/pose_mapping.py:26-33`
- Modify: `kinova_teleop/pose_mapping.py:222-330`
- Modify: `tests/test_pose_mapping.py:20-200`

**Interfaces:**
- Consumes: unchanged `ControllerSample` shape.
- Produces:
  - `ClutchState` enum with `WAITING_FOR_RELEASE`, `READY`, `ACTIVE`
  - `MappingConfig.grip_press_threshold: float = 0.9`
  - `MappingConfig.grip_release_threshold: float = 0.8`
  - `MappingOutput.clutch_state: ClutchState`

- [ ] **Step 1: Add failing startup/recovery safety tests**

Append tests that express the new safety contract before changing old tests:

```python
from kinova_teleop.pose_mapping import ClutchState


def test_startup_pressed_grip_waits_for_release_before_activation() -> None:
    mapper = RelativePoseMapper(unfiltered_config())
    ee = identity_pose()
    pressed = mapper.update(
        sample([0, 0, 0], grip=1.0, stamp=1, received=1.0),
        ee,
        now=1.0,
    )
    released = mapper.update(
        sample([0, 0, 0], grip=0.0, stamp=2, received=1.01),
        ee,
        now=1.01,
    )
    activated = mapper.update(
        sample([0, 0, 0], grip=1.0, stamp=3, received=1.02),
        ee,
        now=1.02,
    )
    assert pressed.clutch_state is ClutchState.WAITING_FOR_RELEASE
    assert not pressed.active
    assert released.clutch_state is ClutchState.READY
    assert activated.activated and activated.active


def test_grip_hysteresis_avoids_chatter() -> None:
    mapper = RelativePoseMapper(unfiltered_config())
    ee = identity_pose()
    mapper.update(sample([0, 0, 0], grip=0.0, stamp=1), ee, now=0.0)
    mapper.update(sample([0, 0, 0], grip=1.0, stamp=2), ee, now=0.01)
    middle = mapper.update(
        sample([0, 0, 0], grip=0.85, stamp=3), ee, now=0.02
    )
    released = mapper.update(
        sample([0, 0, 0], grip=0.79, stamp=4), ee, now=0.03
    )
    assert middle.active
    assert released.clutch_state is ClutchState.READY
    assert not released.active


def test_invalid_input_requires_new_release_before_reclutch() -> None:
    mapper = RelativePoseMapper(unfiltered_config())
    ee = identity_pose()
    mapper.update(sample([0, 0, 0], grip=0.0, stamp=1), ee, now=0.0)
    mapper.update(sample([0, 0, 0], grip=1.0, stamp=2), ee, now=0.01)
    mapper.update(
        sample([0, 0, 0], grip=1.0, stamp=3, valid=False),
        ee,
        now=0.02,
    )
    still_pressed = mapper.update(
        sample([1, 0, 0], grip=1.0, stamp=4), ee, now=0.03
    )
    assert still_pressed.clutch_state is ClutchState.WAITING_FOR_RELEASE
    assert not still_pressed.active
```

- [ ] **Step 2: Run new tests and confirm RED for the missing state**

```powershell
.\.venv\Scripts\python.exe -m pytest tests/test_pose_mapping.py `
  -k "startup_pressed or hysteresis or requires_new_release" -q
```

Expected: collection fails because `ClutchState` does not exist.

- [ ] **Step 3: Implement the minimal state machine**

Add:

```python
from enum import Enum


class ClutchState(str, Enum):
    WAITING_FOR_RELEASE = "waiting_for_release"
    READY = "ready"
    ACTIVE = "active"


@dataclass(frozen=True)
class MappingConfig:
    translation_scale: float = 1.0
    grip_press_threshold: float = 0.9
    grip_release_threshold: float = 0.8
    stale_timeout: float = 0.2
    filter_time_constant: float = 0.05
    max_position_step: float = 0.02
    max_rotation_step: float = 0.15
```

Replace the boolean as authoritative state with `self.clutch_state`.
`reset` and invalid/stale input enter `WAITING_FOR_RELEASE`. In that state,
only Grip `< grip_release_threshold` moves to `READY`. `READY` activates only
at Grip `> grip_press_threshold`. `ACTIVE` remains active through the
hysteresis band and moves to `READY` below the release threshold.
`MappingOutput.active` remains for callers and equals
`clutch_state is ACTIVE`.

- [ ] **Step 4: Update existing mapping tests to perform the required release**

Add a helper:

```python
def release_then_press(
    mapper: RelativePoseMapper,
    ee: Pose,
    *,
    start: float = 1.0,
) -> None:
    mapper.update(
        sample([0, 0, 0], grip=0.0, stamp=1, received=start),
        ee,
        now=start,
    )
    mapper.update(
        sample([0, 0, 0], grip=1.0, stamp=2, received=start + 0.01),
        ee,
        now=start + 0.01,
    )
```

Use this helper in existing tests that previously activated with their first
pressed sample. Preserve distinct increasing timestamps for subsequent
motion. Keep the activation no-jump assertion by checking the second helper
update or by spelling out release/press inside that test.

- [ ] **Step 5: Run mapping and full regression tests**

```powershell
.\.venv\Scripts\python.exe -m pytest tests/test_pose_mapping.py -q
.\.venv\Scripts\python.exe -m pytest -q
```

Expected: all release-gate, 6DoF, stale, hold, filter, and previous tests pass.

- [ ] **Step 6: Commit the safety state machine**

```powershell
git add kinova_teleop/pose_mapping.py tests/test_pose_mapping.py
git commit -m "feat: gate Grip control on safe release"
```

---

### Task 4: Simulator-Independent End-Effector Backend

**Files:**
- Create: `kinova_teleop/backend.py`
- Modify: `kinova_teleop/teleop_controller.py`
- Create: `tests/test_backend.py`
- Modify: `tests/test_teleop_controller.py`

**Interfaces:**
- Consumes: `Pose`, model contract, existing `DampedLeastSquaresIK`.
- Produces:
  - `BackendResult`
  - `EndEffectorTargetBackend` protocol
  - `MuJoCoBackend`
  - `TeleopController(config, source, backend)`

- [ ] **Step 1: Write failing backend contract tests**

Create `tests/test_backend.py`:

```python
import numpy as np

from kinova_teleop.backend import MuJoCoBackend
from kinova_teleop.pose_mapping import Pose


def test_mujoco_backend_reports_current_pose_and_finite_state(
    teleop_model_path,
) -> None:
    backend = MuJoCoBackend(teleop_model_path, control_hz=100.0)
    try:
        pose = backend.current_pose()
        assert pose.position.shape == (3,)
        assert pose.quaternion.shape == (4,)
        assert backend.state_is_finite()
    finally:
        backend.close()


def test_mujoco_backend_rejects_nonconverged_ik_without_changing_controls(
    teleop_model_path,
) -> None:
    backend = MuJoCoBackend(teleop_model_path, control_hz=100.0)
    backend.begin_control()
    held = backend.data.ctrl.copy()

    class FailedIK:
        def solve(self, *_args, **_kwargs):
            from kinova_teleop.ik_solver import IKResult
            return IKResult(np.ones(7), False, 0.1, 0.2, 40)

    backend.ik = FailedIK()
    result = backend.command_pose(backend.current_pose())
    assert not result.accepted and not result.converged
    np.testing.assert_allclose(backend.data.ctrl, held)
    backend.close()
```

- [ ] **Step 2: Write a failing generic-controller test with a fake backend**

In `tests/test_teleop_controller.py`, define:

```python
class RecordingBackend:
    def __init__(self):
        self.pose = Pose(
            np.array([0.0, 0.0, 0.5]),
            np.array([1.0, 0.0, 0.0, 0.0]),
        )
        self.begin_calls = 0
        self.commands = []
        self.holds = 0
        self.steps = 0
        self.closed = False

    def current_pose(self):
        return self.pose

    def begin_control(self):
        self.begin_calls += 1

    def command_pose(self, target):
        from kinova_teleop.backend import BackendResult
        self.commands.append(target)
        return BackendResult(True, True, 0.0, 0.0, "")

    def hold(self):
        self.holds += 1

    def step(self):
        self.steps += 1

    def close(self):
        self.closed = True
```

Test a released sample, a pressed activation sample, a moved sample, and a
released sample. Assert `begin_control` occurs once, only active frames call
`command_pose`, inactive frames call `hold`, every frame calls `step`, and
`close` closes both source and backend.

- [ ] **Step 3: Run backend/controller tests and confirm RED**

```powershell
.\.venv\Scripts\python.exe -m pytest `
  tests/test_backend.py tests/test_teleop_controller.py -q
```

Expected: backend module import fails and the old controller constructor does
not accept an injected backend.

- [ ] **Step 4: Extract `MuJoCoBackend` without changing IK behavior**

Create:

```python
@dataclass(frozen=True)
class BackendResult:
    accepted: bool
    converged: bool
    position_error: float
    rotation_error: float
    reason: str


class EndEffectorTargetBackend(Protocol):
    def current_pose(self) -> Pose: ...
    def begin_control(self) -> None: ...
    def command_pose(self, target: Pose) -> BackendResult: ...
    def hold(self) -> None: ...
    def step(self) -> None: ...
    def close(self) -> None: ...
```

Move model load/validation, `MjData`, keyframe reset, site pose, joint
extraction, actuator writes, target marker, IK, posture reference,
`last_valid_target`, substeps, and finite-state checks from
`TeleopController` into `MuJoCoBackend`. Its constructor is:

```python
def __init__(
    self,
    model_path: Path,
    *,
    control_hz: float = 100.0,
) -> None:
```

`begin_control` captures current joint positions. `command_pose` updates the
target marker, runs IK, writes controls only for converged finite results,
and returns a reason such as `"IK did not converge"` on rejection. `hold`
reapplies `last_valid_target`. `step` performs the computed MuJoCo substeps
and finite checks. Keep `model`, `data`, and `contract` readable for
diagnostic tests. Add `state_is_finite()` and `launch_viewer()` as
MuJoCo-specific conveniences outside the generic protocol; track and close
the launched Viewer in `close`.

- [ ] **Step 5: Refactor `TeleopController` to use only the protocol**

Change `TeleopConfig` to contain only control-loop/mapping values:

```python
@dataclass(frozen=True)
class TeleopConfig:
    control_hz: float = 100.0
    realtime: bool = True
    translation_scale: float = 0.5
    stale_timeout: float = 0.2
```

Change the constructor to:

```python
def __init__(
    self,
    config: TeleopConfig,
    source: XrInputSource,
    backend: EndEffectorTargetBackend,
) -> None:
```

Initialize the mapper from `backend.current_pose()`. In `step_once`, call
`backend.begin_control()` on `mapping.activated`; call `command_pose` only
while active, otherwise call `hold`; always call `backend.step`. Convert the
`BackendResult` into existing `StepDiagnostics`. Make `close` idempotently
close source and backend. Keep `run` generic:

```python
def run(
    self,
    *,
    max_steps: int | None = None,
    should_continue: Callable[[], bool] | None = None,
    on_step: Callable[[], None] | None = None,
) -> None:
```

Viewer callbacks are supplied by the CLI only when a MuJoCo Viewer is open.

- [ ] **Step 6: Update existing integration tests to construct the backend**

Replace old construction with:

```python
backend = MuJoCoBackend(teleop_model_path, control_hz=100.0)
controller = TeleopController(
    TeleopConfig(realtime=False),
    source,
    backend,
)
```

Move direct MuJoCo assertions from `controller.data` to `backend.data`.
Keep a 500-step finite integration test and the target-marker test.

- [ ] **Step 7: Run backend, controller, and full tests**

```powershell
.\.venv\Scripts\python.exe -m pytest `
  tests/test_backend.py tests/test_teleop_controller.py -q
.\.venv\Scripts\python.exe -m pytest -q
```

Expected: all pass; `TeleopController` can be tested with no MuJoCo model.

- [ ] **Step 8: Commit the backend boundary**

```powershell
git add kinova_teleop/backend.py kinova_teleop/teleop_controller.py `
  tests/test_backend.py tests/test_teleop_controller.py
git commit -m "refactor: isolate end-effector backend"
```

---

### Task 5: Native-Windows `pico-udp` CLI and Preflight

**Files:**
- Modify: `kinova_teleop/main.py`
- Modify: `tests/test_cli.py`

**Interfaces:**
- Consumes: `PicoUdpInput`, `MuJoCoBackend`, generic `TeleopController`.
- Produces:
  - `create_input(args) -> XrInputSource`
  - CLI `--input pico-udp|xrobotoolkit`
  - CLI `--pico-host`, `--pico-port`, `--check-input`, `--check-timeout`
  - compatibility alias `--check-xr`

- [ ] **Step 1: Add failing parser and factory tests**

Append:

```python
def test_pico_udp_is_default_native_input() -> None:
    args = build_parser().parse_args([])
    assert args.input == "pico-udp"
    assert args.pico_host == "0.0.0.0"
    assert args.pico_port == 15031


def test_create_input_selects_pico_udp(monkeypatch) -> None:
    created = []
    monkeypatch.setattr(
        "kinova_teleop.main.PicoUdpInput",
        lambda **kwargs: created.append(kwargs) or object(),
    )
    args = build_parser().parse_args(["--input", "pico-udp"])
    from kinova_teleop.main import create_input
    create_input(args)
    assert created == [{"host": "0.0.0.0", "port": 15031, "stale_after": 0.2}]


def test_dry_run_never_constructs_network_input(monkeypatch) -> None:
    monkeypatch.setattr(
        "kinova_teleop.main.PicoUdpInput",
        lambda **_kwargs: (_ for _ in ()).throw(AssertionError("network")),
    )
    assert main(["--dry-run", "--headless", "--steps", "20"]) == 0
```

- [ ] **Step 2: Add failing preflight behavior tests**

Use a scripted source with duplicate reads, then 20 distinct valid samples.
Assert `--check-input --samples 20` counts distinct increasing timestamps,
prints position/quaternion/Grip, requires at least one Grip `< 0.8`, closes
the source, and returns `2` on timeout or if all valid samples remain pressed.
Test `--check-xr` as an alias that uses the selected input rather than
hardcoding XRoboToolkit.

- [ ] **Step 3: Run CLI tests and confirm RED**

```powershell
.\.venv\Scripts\python.exe -m pytest tests/test_cli.py -q
```

Expected: parser rejects the new arguments and `create_input` is missing.

- [ ] **Step 4: Implement input and backend factories**

Add parser arguments:

```python
parser.add_argument(
    "--input",
    choices=("pico-udp", "xrobotoolkit"),
    default="pico-udp",
)
parser.add_argument("--pico-host", default="0.0.0.0")
parser.add_argument("--pico-port", type=int, default=15031)
parser.add_argument("--check-input", action="store_true")
parser.add_argument("--check-xr", action="store_true", help=argparse.SUPPRESS)
parser.add_argument("--check-timeout", type=float, default=15.0)
```

Implement:

```python
def create_input(args):
    if args.dry_run:
        return DryRunXrInput(control_hz=args.control_hz)
    if args.input == "pico-udp":
        return PicoUdpInput(
            host=args.pico_host,
            port=args.pico_port,
            stale_after=args.stale_timeout,
        )
    return SdkXrInput()
```

Build `MuJoCoBackend(args.model, control_hz=args.control_hz)` only outside
input-check mode. Pass it to the generic `TeleopController`. For interactive
mode, call `viewer = backend.launch_viewer()` and pass
`viewer.is_running`/`viewer.sync` callbacks. Determine success through
`backend.state_is_finite()`, not direct controller MuJoCo fields.

- [ ] **Step 5: Implement finite-sample preflight**

The preflight loop:

- waits until `check_timeout`;
- ignores invalid and duplicate-timestamp reads while continuing to wait;
- prints each new valid sample;
- counts exactly `samples` distinct timestamps;
- records whether any accepted sample has Grip `< 0.8`;
- returns `2` with actionable text for timeout or missing release;
- closes the source in `finally`.

Sleep `0.01 s` between polls so the loop does not busy-spin.

- [ ] **Step 6: Run CLI, full tests, and native dry-run**

```powershell
.\.venv\Scripts\python.exe -m pytest tests/test_cli.py -q
.\.venv\Scripts\python.exe -m pytest -q
.\.venv\Scripts\python.exe -m kinova_teleop.main `
  --dry-run --headless --steps 2000
```

Expected: tests pass and final line contains
`completed steps=2000 finite_state=true`.

- [ ] **Step 7: Commit native CLI integration**

```powershell
git add kinova_teleop/main.py tests/test_cli.py
git commit -m "feat: add native PICO UDP teleop CLI"
```

---

### Task 6: L20-Derived PICO Unity/OpenXR Application

**Files:**
- Create: `pico_unity_bridge/Packages/manifest.json`
- Create: `pico_unity_bridge/Packages/packages-lock.json`
- Create: `pico_unity_bridge/ProjectSettings/*`
- Create: `pico_unity_bridge/Assets/XR/*`
- Create: `pico_unity_bridge/Assets/Resources/*`
- Create: `pico_unity_bridge/Assets/Plugins/Android/AndroidManifest.xml`
- Create: `pico_unity_bridge/Assets/KinovaPico/KinovaControllerPacket.cs`
- Create: `pico_unity_bridge/Assets/KinovaPico/KinovaDiscoveryState.cs`
- Create: `pico_unity_bridge/Assets/KinovaPico/KinovaControllerUdpBridge.cs`
- Create: `pico_unity_bridge/Assets/KinovaPico/KinovaBridgeBootstrap.cs`
- Create: `pico_unity_bridge/Assets/KinovaPico/Editor/BuildKinovaPicoBridge.cs`
- Create: `pico_unity_bridge/Assets/KinovaPico/Tests/KinovaControllerPacketTests.cs`
- Create: `pico_unity_bridge/Assets/KinovaPico/Tests/KinovaDiscoveryStateTests.cs`
- Create: corresponding `.asmdef` and `.meta` files
- Create: `tests/test_unity_project_contract.py`

**Interfaces:**
- Consumes: Task 1 byte layout and the approved fixed Unity/PICO versions.
- Produces: Android package `com.yezqin.kinovapicobridge` and UDP v1 packets.

- [ ] **Step 1: Write failing Python project-contract tests**

Create tests that read repository files and assert:

```python
def test_unity_manifest_pins_l20_verified_openxr_stack() -> None:
    manifest = json.loads((ROOT / "pico_unity_bridge/Packages/manifest.json").read_text())
    deps = manifest["dependencies"]
    assert deps["com.unity.xr.management"] == "4.4.0"
    assert deps["com.unity.xr.openxr"] == "1.8.2"
    assert deps["com.unity.xr.openxr.picoxr"].endswith(
        "#3aa3e62bff41df618529eeb60ff02c29a515dafe"
    )


def test_bridge_contract_is_udp_not_old_transport() -> None:
    text = "\n".join(
        path.read_text(encoding="utf-8")
        for path in (ROOT / "pico_unity_bridge/Assets").rglob("*.cs")
    )
    assert "KINOVA_DISCOVER_V1" in text
    assert "KINOVA_READY_V1" in text
    assert "15031" in text
    assert "XRNode.LeftHand" in text
    assert "CommonUsages.grip" in text
    for forbidden in (
        "XRoboToolkit", "adb reverse", "adb tcpip", "TcpListener"
    ):
        assert forbidden.lower() not in text.lower()
```

Also assert Unity version `2022.3.62f3c1`, package name, INTERNET permission,
ARM64/IL2CPP editor settings, independent Kinova namespace, and no edits or
relative references into the L20 repository.

- [ ] **Step 2: Run the contract test and confirm RED**

```powershell
.\.venv\Scripts\python.exe -m pytest tests/test_unity_project_contract.py -q
```

Expected: the new Unity project is absent.

- [ ] **Step 3: Copy only the verified L20 Unity platform configuration**

Copy these read-only reference directories from
`D:\yezq\26.7.11_L20_controll\pico_unity_bridge` into the new Kinova
`pico_unity_bridge`:

```powershell
New-Item -ItemType Directory -Force '.\pico_unity_bridge\Assets' | Out-Null
Copy-Item -Recurse -Force `
  'D:\yezq\26.7.11_L20_controll\pico_unity_bridge\ProjectSettings' `
  '.\pico_unity_bridge\ProjectSettings'
Copy-Item -Recurse -Force `
  'D:\yezq\26.7.11_L20_controll\pico_unity_bridge\Packages' `
  '.\pico_unity_bridge\Packages'
Copy-Item -Recurse -Force `
  'D:\yezq\26.7.11_L20_controll\pico_unity_bridge\Assets\XR' `
  '.\pico_unity_bridge\Assets\XR'
Copy-Item -Recurse -Force `
  'D:\yezq\26.7.11_L20_controll\pico_unity_bridge\Assets\Resources' `
  '.\pico_unity_bridge\Assets\Resources'
```

Verify `manifest.json` retains the exact pinned dependency. Do not copy
`Assets/L20Pico`; the Kinova runtime is new and independently named.

- [ ] **Step 4: Write failing C# packet tests**

`KinovaControllerPacketTests.cs` must verify:

- output length 56;
- bytes 0-8, version, tracked flag, reserved, sequence, timestamp;
- little-endian position/quaternion/Grip values;
- Grip is clamped by the sampling layer before packet creation;
- packet creation rejects non-finite pose, zero quaternion, out-of-range Grip;
- untracked packet encodes zero position, identity xyzw quaternion, zero Grip;
- sequence wraps under `unchecked`.

Include one canonical fixture matching Task 1:

```csharp
CollectionAssert.AreEqual(
    new byte[] { (byte)'K', (byte)'I', (byte)'N', (byte)'V',
                 (byte)'P', (byte)'I', (byte)'C', (byte)'O' },
    payload.Take(8).ToArray());
Assert.AreEqual(56, payload.Length);
Assert.AreEqual(42u, BitConverter.ToUInt32(payload, 12));
Assert.AreEqual(1234567ul, BitConverter.ToUInt64(payload, 16));
```

- [ ] **Step 5: Implement the exact C# packet writer**

Before implementation, run the new EditMode packet tests:

```powershell
New-Item -ItemType Directory -Force .\artifacts | Out-Null
& 'D:\Program Files\Unity 2023.2.20f1c1\Editor\2022.3.62f3c1\Editor\Unity.exe' `
  -batchmode -nographics -quit `
  -projectPath "$PWD\pico_unity_bridge" `
  -runTests -testPlatform EditMode `
  -testFilter Yezqin.KinovaPico.Tests.KinovaControllerPacketTests `
  -testResults "$PWD\artifacts\pico-packet-red.xml" `
  -logFile "$PWD\artifacts\pico-packet-red.log"
```

Expected: tests fail because `KinovaControllerPacket` is missing.

Core layout:

```csharp
public const int PacketSize = 56;
public const byte Version = 1;
public const byte TrackedFlag = 1;

public static byte[] Encode(
    uint sequence,
    ulong sourceTimeUs,
    bool tracked,
    Vector3 position,
    Quaternion rotation,
    float grip)
{
    Validate(tracked, position, rotation, grip);
    using (var stream = new MemoryStream(PacketSize))
    using (var writer = new BinaryWriter(stream, Encoding.UTF8, true))
    {
        writer.Write(Magic);
        writer.Write(Version);
        writer.Write((byte)(tracked ? TrackedFlag : 0));
        writer.Write((ushort)0);
        writer.Write(sequence);
        writer.Write(sourceTimeUs);
        Vector3 p = tracked ? position : Vector3.zero;
        Quaternion q = tracked ? rotation : Quaternion.identity;
        float g = tracked ? grip : 0f;
        writer.Write(p.x); writer.Write(p.y); writer.Write(p.z);
        writer.Write(q.x); writer.Write(q.y); writer.Write(q.z); writer.Write(q.w);
        writer.Write(g);
        writer.Flush();
        return stream.ToArray();
    }
}
```

- [ ] **Step 6: Run packet tests GREEN**

```powershell
& 'D:\Program Files\Unity 2023.2.20f1c1\Editor\2022.3.62f3c1\Editor\Unity.exe' `
  -batchmode -nographics -quit `
  -projectPath "$PWD\pico_unity_bridge" `
  -runTests -testPlatform EditMode `
  -testFilter Yezqin.KinovaPico.Tests.KinovaControllerPacketTests `
  -testResults "$PWD\artifacts\pico-packet-green.xml" `
  -logFile "$PWD\artifacts\pico-packet-green.log"
```

Expected: packet suite passes.

- [ ] **Step 7: Write and observe failing UDP discovery state tests**

Create `KinovaDiscoveryStateTests.cs` for a pure state object. Assert:

```csharp
[Test]
public void ReadyEndpointStopsDiscoveryUntilNetworkFailure()
{
    var state = new KinovaDiscoveryState(1.0);
    Assert.IsTrue(state.DiscoveryDue(0.0));
    state.MarkDiscoverySent(0.0);
    Assert.IsFalse(state.DiscoveryDue(0.5));
    state.AcceptReady(new IPEndPoint(IPAddress.Parse("192.168.1.10"), 15031));
    Assert.IsFalse(state.DiscoveryDue(2.0));
    state.MarkNetworkFailure();
    Assert.IsTrue(state.DiscoveryDue(2.0));
}
```

Also test that `Reset` clears the endpoint, null endpoints are rejected, and
the discovery interval must be positive. Run only this fixture and confirm
RED because `KinovaDiscoveryState` is absent.

- [ ] **Step 8: Adapt the L20 UDP bridge state machine**

Use `PicoHandUdpBridge.cs` only as a behavioral reference and implement
`KinovaControllerUdpBridge` in namespace `Yezqin.KinovaPico`.

Required constants and input:

```csharp
public const int ProtocolPort = 15031;
public const string DiscoverMessage = "KINOVA_DISCOVER_V1";
public const string ReadyMessage = "KINOVA_READY_V1";

static bool TryReadLeftController(
    out Vector3 position,
    out Quaternion rotation,
    out float grip)
{
    InputDevice device = InputDevices.GetDeviceAtXRNode(XRNode.LeftHand);
    bool isTracked = false;
    position = Vector3.zero;
    rotation = Quaternion.identity;
    grip = 0f;
    bool hasTracking = device.isValid
        && device.TryGetFeatureValue(CommonUsages.isTracked, out isTracked);
    bool hasPosition = device.isValid
        && device.TryGetFeatureValue(CommonUsages.devicePosition, out position);
    bool hasRotation = device.isValid
        && device.TryGetFeatureValue(CommonUsages.deviceRotation, out rotation);
    bool hasGrip = device.isValid
        && device.TryGetFeatureValue(CommonUsages.grip, out grip);
    if (!(hasTracking && isTracked && hasPosition && hasRotation && hasGrip))
    {
        position = Vector3.zero;
        rotation = Quaternion.identity;
        grip = 0f;
        return false;
    }
    grip = Mathf.Clamp01(grip);
    return true;
}
```

Clamp Grip with `Mathf.Clamp01`. Generate `sourceTimeUs` from
`Time.realtimeSinceStartupAsDouble`. Increment sequence in `unchecked`.
Use the L20 limits of one discovery per second and at most 16 READY datagrams
per frame. On send failure, clear `_hostEndpoint` and rediscover. Recreate
the socket after app resume. Keep all Unity XR reads on the main thread.
Put discovery timing and endpoint reset behavior in the tested
`KinovaDiscoveryState`; keep socket I/O and Unity lifecycle in the
MonoBehaviour.

- [ ] **Step 9: Add bootstrap, status display, and editor configuration**

The bootstrap creates the bridge before scene load and a simple XR camera
plus TextMesh after scene load when absent. Display:

```text
Kinova PICO UDP bridge
host: searching | {hostAddress}:15031
tracking: tracked | untracked
grip: 0.000
sequence: {sequence}
rate: {packetsPerSecond}
```

Implement `BuildKinovaPicoBridge.cs` by adapting the L20 editor
configuration to package
`com.yezqin.kinovapicobridge`, application name `Kinova PICO Bridge`, ARM64,
IL2CPP, and the same OpenXR PICO controller profiles. Configure a generated
minimal scene so no binary scene asset is hand-maintained. Expose the exact
batch entry point
`Yezqin.KinovaPico.Editor.BuildKinovaPicoBridge.Build`.

- [ ] **Step 10: Run Python contract and all Unity EditMode tests**

```powershell
.\.venv\Scripts\python.exe -m pytest tests/test_unity_project_contract.py -q
New-Item -ItemType Directory -Force .\artifacts | Out-Null
& 'D:\Program Files\Unity 2023.2.20f1c1\Editor\2022.3.62f3c1\Editor\Unity.exe' `
  -batchmode -nographics -quit `
  -projectPath "$PWD\pico_unity_bridge" `
  -runTests -testPlatform EditMode `
  -testResults "$PWD\artifacts\pico-unity-editmode.xml" `
  -logFile "$PWD\artifacts\pico-unity-editmode.log"
```

Expected: Python contract passes and Unity XML reports zero failed tests.

- [ ] **Step 11: Commit the Unity project**

```powershell
git add pico_unity_bridge tests/test_unity_project_contract.py
git commit -m "feat: add PICO UDP controller bridge app"
```

---

### Task 7: Reproducible Build and Stable Daily Launcher

**Files:**
- Create: `scripts/build_pico_udp_bridge.ps1`
- Create: `scripts/start_pico_udp_teleop.ps1`
- Create: `tests/test_pico_udp_scripts.py`
- Modify: `.gitignore`

**Interfaces:**
- Consumes: Task 5 CLI and Task 6 Unity editor build method.
- Produces:
  - `artifacts/kinova-pico-udp-bridge.apk`
  - optional ADB install/start
  - daily native-Windows preflight and Viewer launch

- [ ] **Step 1: Write failing PowerShell dry-run/contract tests**

Create tests that invoke both scripts with `-DryRun` and assert exact
commands. Required assertions:

```python
assert "2022.3.62f3c1" in build.stdout
assert "runTests" in build.stdout
assert "BuildKinovaPicoBridge.Build" in build.stdout
assert "kinova-pico-udp-bridge.apk" in build.stdout

assert "com.yezqin.kinovapicobridge" in start.stdout
assert "--input pico-udp" in start.stdout
assert "--check-input" in start.stdout
assert "--samples 20" in start.stdout
assert "--scale 0.5" in start.stdout
for forbidden in (
    "wsl.exe", "xrobotoolkit", "reverse tcp", "adb tcpip", "adb connect"
):
    assert forbidden not in start.stdout.lower()
```

Add static assertions that `.gitignore` covers `artifacts/`,
`pico_unity_bridge/Library/`, `Temp/`, `Logs/`, `obj/`, and `UserSettings/`.

- [ ] **Step 2: Run script tests and confirm RED**

```powershell
.\.venv\Scripts\python.exe -m pytest tests/test_pico_udp_scripts.py -q
```

Expected: scripts are missing.

- [ ] **Step 3: Implement deterministic Unity build/install**

`build_pico_udp_bridge.ps1` parameters:

```powershell
param(
    [string]$UnityPath =
      'D:\Program Files\Unity 2023.2.20f1c1\Editor\2022.3.62f3c1\Editor\Unity.exe',
    [switch]$Install,
    [switch]$DryRun
)
```

Resolve repository paths from `$PSScriptRoot`. Run EditMode tests first,
inspect process exit code and test XML, then call:

```powershell
& $UnityPath -batchmode -nographics -quit `
  -projectPath $projectPath `
  -executeMethod Yezqin.KinovaPico.Editor.BuildKinovaPicoBridge.Build `
  -logFile $buildLog
```

Require a nonempty APK. For `-Install`, require exactly one authorized
physical USB device from `C:\adb\adb.exe devices -l`, then run
`C:\adb\adb.exe -d install -r
.\artifacts\kinova-pico-udp-bridge.apk`. `-DryRun` prints commands without
starting Unity or ADB.

- [ ] **Step 4: Implement automatic-or-manual daily launch**

`start_pico_udp_teleop.ps1` parameters:

```powershell
param(
    [double]$Scale = 0.5,
    [int]$Samples = 20,
    [double]$CheckTimeout = 30.0,
    [switch]$ManualPicoStart,
    [switch]$DryRun
)
```

Resolve `.\.venv\Scripts\python.exe`, falling back to `python.exe`.
Unless `-ManualPicoStart`:

1. If `C:\adb\adb.exe` exists and exactly one authorized USB device is
   present, run:

   ```powershell
   C:\adb\adb.exe -d shell monkey `
     -p com.yezqin.kinovapicobridge `
     -c android.intent.category.LAUNCHER 1
   ```

2. If ADB is absent, offline, unauthorized, or has no physical USB PICO,
   print a manual-start instruction and continue. Multiple authorized USB
   devices are an error rather than an arbitrary selection.

Run preflight:

```powershell
python -m kinova_teleop.main `
  --input pico-udp `
  --check-input `
  --samples $Samples `
  --check-timeout $CheckTimeout
```

Only after exit code zero, run:

```powershell
python -m kinova_teleop.main --input pico-udp --scale $Scale
```

On preflight failure, print checks for same LAN/VLAN, AP isolation, VPN,
left-controller wake/tracking, Grip release, and UDP 15031 firewall. Do not
create a firewall rule automatically.

- [ ] **Step 5: Run script tests and inspect dry-run commands**

```powershell
.\.venv\Scripts\python.exe -m pytest tests/test_pico_udp_scripts.py -q
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File .\scripts\build_pico_udp_bridge.ps1 -DryRun
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File .\scripts\start_pico_udp_teleop.ps1 -DryRun
```

Expected: tests pass; no Unity, ADB, network, or MuJoCo process is started.

- [ ] **Step 6: Commit the Windows workflow**

```powershell
git add .gitignore scripts/build_pico_udp_bridge.ps1 `
  scripts/start_pico_udp_teleop.ps1 tests/test_pico_udp_scripts.py
git commit -m "feat: add PICO UDP build and launch workflow"
```

---

### Task 8: Documentation, Full Verification, and PICO/MuJoCo Acceptance

**Files:**
- Rewrite: `README.md`
- Create: `docs/pico-udp-quickstart.md`
- Create: `tests/test_pico_udp_docs.py`
- Modify only when a failing automated or live validation test demonstrates a defect.

**Interfaces:**
- Documents the one-time build/install, automatic/manual app start, daily
  command, health output, safety behavior, and explicit absence of real
  Kinova control.

- [ ] **Step 1: Write failing documentation contract tests**

Test both documents for:

- Windows native Python;
- Unity `2022.3.62f3c1`;
- fixed SDK commit;
- `build_pico_udp_bridge.ps1 -Install`;
- `start_pico_udp_teleop.ps1`;
- manual PICO app start fallback;
- UDP 15031 and automatic discovery;
- left controller only;
- release-first, Grip `>0.9`, release `<0.8`;
- relative 6DoF and default scale `0.5`;
- invalid/stale/untracked hold behavior;
- Gen3 MJCF and JACO2 URDF warning;
- no WSL/XRoboToolkit requirement on the recommended path;
- no Kortex/physical Kinova implementation.

- [ ] **Step 2: Run documentation tests and confirm RED**

```powershell
.\.venv\Scripts\python.exe -m pytest tests/test_pico_udp_docs.py -q
```

Expected: current XRoboToolkit-first README does not satisfy the new stable
workflow contract.

- [ ] **Step 3: Write the focused quick-start and update README**

Lead with:

```powershell
.\scripts\build_pico_udp_bridge.ps1 -Install
.\scripts\start_pico_udp_teleop.ps1
```

Document manual app launch, Windows firewall prompt, `--check-input`,
expected live output, Grip instructions, scale tuning, tracking/network
faults, Ctrl+C/Viewer exit, and dry-run:

```powershell
.\.venv\Scripts\python.exe -m kinova_teleop.main `
  --dry-run --headless --steps 5000
```

Move the old XRoboToolkit instructions to a clearly labeled compatibility
section rather than presenting them as the default.

- [ ] **Step 4: Run all native automated verification**

```powershell
.\.venv\Scripts\python.exe -m pip install -e ".[dev]"
.\.venv\Scripts\python.exe -m pytest -q
.\.venv\Scripts\python.exe -m kinova_teleop.main `
  --dry-run --headless --steps 5000
git diff --check
```

Expected: zero failed tests; dry-run reports
`completed steps=5000 finite_state=true`; no whitespace errors.

- [ ] **Step 5: Build and inspect the Android APK**

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File .\scripts\build_pico_udp_bridge.ps1
Get-Item .\artifacts\kinova-pico-udp-bridge.apk
& 'C:\adb\adb.exe' devices -l
& 'C:\adb\adb.exe' -d install -r `
  .\artifacts\kinova-pico-udp-bridge.apk
& 'C:\adb\adb.exe' -d shell pm path `
  com.yezqin.kinovapicobridge
```

Expected: nonempty APK, A9210 authorized, install returns `Success`, package
path exists.

- [ ] **Step 6: Validate 100 live PICO controller frames**

Start the app through ADB or manually, then run:

```powershell
.\.venv\Scripts\python.exe -m kinova_teleop.main `
  --input pico-udp --check-input --samples 100 --check-timeout 60
```

With the user moving/rotating the left controller and pressing/releasing
Grip, record that:

- 100 distinct sequence/timestamp samples arrive;
- position and quaternion values change;
- quaternion remains normalized and finite;
- Grip crosses above 0.9 and below 0.8;
- health output reports no persistent reorder or stale condition.

If physical interaction is not performed, report this step as pending rather
than treating static samples as success.

- [ ] **Step 7: Perform MuJoCo end-to-end acceptance**

Run:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File .\scripts\start_pico_udp_teleop.ps1
```

Record manual results:

1. released Grip does not move the arm;
2. pressing Grip causes no target jump;
3. held Grip maps controller translation and rotation to full relative 6DoF;
4. release holds the last target;
5. app pause/network loss holds the target;
6. recovery while still pressed cannot move the arm;
7. release then press resumes safely.

Do not mention or test physical Kinova output.

- [ ] **Step 8: Commit documentation and validation corrections**

```powershell
git add README.md docs/pico-udp-quickstart.md tests/test_pico_udp_docs.py
git commit -m "docs: add stable PICO UDP teleop guide"
```

Any production correction discovered during live validation must first gain
its own failing regression test and be committed separately before this
documentation-only commit.

- [ ] **Step 9: Final repository audit**

```powershell
git status --short
git diff --check HEAD~1..HEAD
git ls-files | Select-String -Pattern `
  'artifacts/|Library/|Temp/|Logs/|UserSettings/|\\.apk$'
git log --oneline --decorate -12
```

Expected: clean worktree, no generated Unity/APK artifacts tracked, and one
reviewable commit per task.

---

## Plan Self-Review Checklist

- Every approved design requirement is assigned to a task.
- The wire type is consistently named `PicoControllerFrame`.
- The input adapter is consistently named `PicoUdpInput`.
- The generic backend is consistently named `EndEffectorTargetBackend`.
- The simulator backend is consistently named `MuJoCoBackend`.
- The new CLI input value is consistently `pico-udp`.
- All paths use UDP 15031 and the exact v1 magic/discovery strings.
- No implementation task creates a Kortex backend or physical robot output.
- Each production change has an explicit preceding RED step.
