# Kinova Gen3 PICO Teleoperation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a tested WSL2 application that maps the PICO left controller's relative 6DoF motion to the `pinch_site` of a single Kinova Gen3 arm in MuJoCo.

**Architecture:** A lazy-loaded XR input adapter produces device-independent controller samples. A stateful pose mapper implements Grip clutching, coordinate conversion, filtering, and stale-data handling. A MuJoCo-native damped least-squares solver computes safe 7DoF joint targets, and a controller coordinates those units with the model, position actuators, dry-run input, and Viewer.

**Tech Stack:** Python 3.10+, NumPy, MuJoCo 3.3+, pytest, XRoboToolkit PC Service and `xrobotoolkit_sdk`.

## Global Constraints

- Target environment is WSL2 Ubuntu 22.04 with WSLg.
- PICO input uses the left controller only.
- End-effector control is relative full-pose 6DoF.
- The controlled robot is the single Kinova Gen3 7DoF arm in `kinova_gen3_mujoco/scene.xml`.
- The model has no gripper and no gripper behavior may be added.
- Grip is a hold-to-enable clutch with threshold `0.9`; release or stale data holds the last valid actuator target.
- XR data older than `0.2` seconds is stale.
- Production Python code must be preceded by a test that fails for the intended missing behavior.
- `.upstream`, `.venv`, the obsolete `kinova/` JACO2 model, local SDK settings, and generated caches must not be published.
- The GitHub repository is private and named `YEZQin/kinova-gen3-pico-teleop-mujoco`.

---

### Task 1: Repository Hygiene, Packaging, and Model Contract

**Files:**
- Create: `.gitignore`
- Create: `pyproject.toml`
- Create: `kinova_teleop/__init__.py`
- Create: `kinova_teleop/model_contract.py`
- Create: `tests/conftest.py`
- Create: `tests/test_model_contract.py`
- Create: `LICENSE`
- Modify: `requirements-mujoco.txt`

**Interfaces:**
- Consumes: `kinova_gen3_mujoco/scene.xml`.
- Produces: `ModelContract`, `load_and_validate_model(path: Path) -> tuple[mujoco.MjModel, ModelContract]`, and shared pytest model fixtures.

- [ ] **Step 1: Add repository ignore rules**

```gitignore
.venv/
.upstream/
kinova/
__pycache__/
*.py[cod]
.pytest_cache/
.coverage
htmlcov/
build/
dist/
*.egg-info/
PXREASetting.ini
setting.ini
logs/
```

- [ ] **Step 2: Add package metadata and dependencies**

```toml
[build-system]
requires = ["setuptools>=69", "wheel"]
build-backend = "setuptools.build_meta"

[project]
name = "kinova-gen3-pico-teleop"
version = "0.1.0"
description = "PICO left-controller teleoperation for a Kinova Gen3 arm in MuJoCo"
readme = "README.md"
requires-python = ">=3.10"
license = {text = "MIT"}
dependencies = [
    "mujoco>=3.3",
    "numpy>=1.26",
]

[project.optional-dependencies]
dev = ["pytest>=8.0"]

[project.scripts]
kinova-pico-teleop = "kinova_teleop.main:main"

[tool.pytest.ini_options]
testpaths = ["tests"]
addopts = "-ra"
```

Update `requirements-mujoco.txt` to:

```text
mujoco>=3.3
numpy>=1.26
pytest>=8.0
```

- [ ] **Step 3: Write the failing model contract tests**

```python
from pathlib import Path

import mujoco
import pytest

from kinova_teleop.model_contract import load_and_validate_model


def test_gen3_scene_satisfies_teleop_contract(model_path: Path) -> None:
    model, contract = load_and_validate_model(model_path)
    assert model.nq == 7
    assert contract.site_name == "pinch_site"
    assert contract.joint_names == tuple(f"joint_{i}" for i in range(1, 8))
    assert len(contract.qpos_addresses) == 7
    assert len(contract.dof_addresses) == 7
    assert len(contract.actuator_ids) == 7
    assert contract.home_key_id == model.key("home").id


def test_contract_rejects_missing_model(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError, match="MuJoCo model not found"):
        load_and_validate_model(tmp_path / "missing.xml")


def test_contract_rejects_model_without_pinch_site(tmp_path: Path) -> None:
    xml = tmp_path / "bad.xml"
    xml.write_text("<mujoco><worldbody><body/></worldbody></mujoco>", encoding="utf-8")
    with pytest.raises(ValueError, match="pinch_site"):
        load_and_validate_model(xml)
```

`tests/conftest.py`:

```python
from pathlib import Path

import pytest


@pytest.fixture
def model_path() -> Path:
    return Path(__file__).resolve().parents[1] / "kinova_gen3_mujoco" / "scene.xml"
```

- [ ] **Step 4: Run the tests and confirm RED**

Run:

```powershell
.\.venv\Scripts\python.exe -m pip install -e .[dev]
.\.venv\Scripts\python.exe -m pytest tests/test_model_contract.py -q
```

Expected: collection fails with `ModuleNotFoundError: No module named 'kinova_teleop.model_contract'`.

- [ ] **Step 5: Implement the model contract**

```python
from dataclasses import dataclass
from pathlib import Path

import mujoco


JOINT_NAMES = tuple(f"joint_{i}" for i in range(1, 8))
SITE_NAME = "pinch_site"
HOME_KEY = "home"


@dataclass(frozen=True)
class ModelContract:
    site_name: str
    site_id: int
    joint_names: tuple[str, ...]
    joint_ids: tuple[int, ...]
    qpos_addresses: tuple[int, ...]
    dof_addresses: tuple[int, ...]
    actuator_ids: tuple[int, ...]
    home_key_id: int


def _require_id(model: mujoco.MjModel, obj: mujoco.mjtObj, name: str) -> int:
    obj_id = mujoco.mj_name2id(model, obj, name)
    if obj_id < 0:
        raise ValueError(f"Required MuJoCo object '{name}' is missing")
    return obj_id


def load_and_validate_model(path: Path) -> tuple[mujoco.MjModel, ModelContract]:
    path = Path(path).resolve()
    if not path.is_file():
        raise FileNotFoundError(f"MuJoCo model not found: {path}")
    model = mujoco.MjModel.from_xml_path(str(path))
    site_id = _require_id(model, mujoco.mjtObj.mjOBJ_SITE, SITE_NAME)
    home_key_id = _require_id(model, mujoco.mjtObj.mjOBJ_KEY, HOME_KEY)
    joint_ids = tuple(_require_id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in JOINT_NAMES)
    qpos_addresses = tuple(int(model.jnt_qposadr[joint_id]) for joint_id in joint_ids)
    dof_addresses = tuple(int(model.jnt_dofadr[joint_id]) for joint_id in joint_ids)
    actuator_ids = []
    for joint_id, name in zip(joint_ids, JOINT_NAMES):
        matches = [
            actuator_id
            for actuator_id in range(model.nu)
            if int(model.actuator_trnid[actuator_id, 0]) == joint_id
        ]
        if len(matches) != 1:
            raise ValueError(f"Joint '{name}' must have exactly one actuator")
        actuator_ids.append(matches[0])
    return model, ModelContract(
        site_name=SITE_NAME,
        site_id=site_id,
        joint_names=JOINT_NAMES,
        joint_ids=joint_ids,
        qpos_addresses=qpos_addresses,
        dof_addresses=dof_addresses,
        actuator_ids=tuple(actuator_ids),
        home_key_id=home_key_id,
    )
```

- [ ] **Step 6: Run tests and confirm GREEN**

Run:

```powershell
.\.venv\Scripts\python.exe -m pytest tests/test_model_contract.py -q
```

Expected: `3 passed`.

- [ ] **Step 7: Add the MIT license and commit**

Use the standard MIT license text with copyright `2026 YEZQin`.

```powershell
git add .gitignore pyproject.toml requirements-mujoco.txt LICENSE kinova_teleop tests
git commit -m "build: scaffold teleoperation package"
```

---

### Task 2: Pose Math, Coordinate Conversion, Grip Clutch, and Filtering

**Files:**
- Create: `kinova_teleop/pose_mapping.py`
- Create: `tests/test_pose_mapping.py`

**Interfaces:**
- Consumes: device-independent `ControllerSample`.
- Produces: `Pose`, quaternion helpers, `MappingConfig`, `MappingOutput`, and `RelativePoseMapper.update`.

- [ ] **Step 1: Write failing quaternion and coordinate tests**

```python
import numpy as np

from kinova_teleop.pose_mapping import (
    PICO_TO_WORLD,
    Pose,
    quat_from_axis_angle,
    quat_to_matrix,
    transform_controller_pose,
)


def test_pico_translation_uses_documented_basis() -> None:
    pose = transform_controller_pose(
        np.array([1.0, 2.0, 3.0]),
        np.array([0.0, 0.0, 0.0, 1.0]),
    )
    np.testing.assert_allclose(pose.position, PICO_TO_WORLD @ [1.0, 2.0, 3.0])


def test_controller_orientation_uses_basis_conjugation() -> None:
    q_xyzw = np.roll(quat_from_axis_angle([0.0, 1.0, 0.0], np.pi / 2), -1)
    pose = transform_controller_pose(np.zeros(3), q_xyzw)
    expected = PICO_TO_WORLD @ quat_to_matrix(np.roll(q_xyzw, 1)) @ PICO_TO_WORLD.T
    np.testing.assert_allclose(quat_to_matrix(pose.quaternion), expected, atol=1e-8)
```

- [ ] **Step 2: Run coordinate tests and confirm RED**

Run:

```powershell
.\.venv\Scripts\python.exe -m pytest tests/test_pose_mapping.py -q
```

Expected: import fails because `pose_mapping.py` does not exist.

- [ ] **Step 3: Implement quaternion primitives and coordinate conversion**

Implement:

```python
PICO_TO_WORLD = np.array(
    [[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
    dtype=np.float64,
)


@dataclass(frozen=True)
class Pose:
    position: np.ndarray
    quaternion: np.ndarray  # wxyz


def normalize_quat(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-12:
        raise ValueError("Quaternion must contain four finite values with non-zero norm")
    q = q / np.linalg.norm(q)
    return -q if q[0] < 0.0 else q


def quat_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = normalize_quat(left)
    w2, x2, y2, z2 = normalize_quat(right)
    return normalize_quat(np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ]))


def transform_controller_pose(position: np.ndarray, quat_xyzw: np.ndarray) -> Pose:
    q_world = matrix_to_quat(
        PICO_TO_WORLD @ quat_to_matrix(np.roll(normalize_quat(np.roll(quat_xyzw, 1)), 0)) @ PICO_TO_WORLD.T
    )
    return Pose(PICO_TO_WORLD @ np.asarray(position, dtype=np.float64), q_world)
```

Also implement `quat_conjugate`, `quat_to_matrix`, `matrix_to_quat`, `quat_from_axis_angle`,
`quat_to_rotvec`, `slerp`, and vector-norm clipping without SciPy.

- [ ] **Step 4: Add failing clutch and stale-data tests**

```python
def sample(position, quaternion=(0.0, 0.0, 0.0, 1.0), grip=0.0, stamp=1, received=0.0, valid=True):
    return ControllerSample(
        position=np.asarray(position, dtype=float),
        quaternion_xyzw=np.asarray(quaternion, dtype=float),
        grip=grip,
        timestamp_ns=stamp,
        received_monotonic=received,
        valid=valid,
    )


def test_grip_rising_edge_has_no_target_jump() -> None:
    mapper = RelativePoseMapper(MappingConfig(filter_time_constant=0.0))
    ee = Pose(np.array([0.2, -0.1, 0.5]), np.array([1.0, 0.0, 0.0, 0.0]))
    out = mapper.update(sample([1, 2, 3], grip=1.0, received=10.0), ee, now=10.0)
    assert out.activated
    np.testing.assert_allclose(out.target.position, ee.position)
    np.testing.assert_allclose(out.target.quaternion, ee.quaternion)


def test_held_grip_maps_full_relative_pose() -> None:
    mapper = RelativePoseMapper(MappingConfig(filter_time_constant=0.0, translation_scale=1.0))
    ee = Pose(np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]))
    mapper.update(sample([0, 0, 0], grip=1.0, stamp=1, received=1.0), ee, now=1.0)
    moved_q = np.roll(quat_from_axis_angle([0, 0, 1], 0.2), -1)
    out = mapper.update(sample([0.1, 0, 0], moved_q, 1.0, 2, 1.01), ee, now=1.01)
    np.testing.assert_allclose(out.target.position, PICO_TO_WORLD @ [0.1, 0, 0])
    assert np.linalg.norm(quat_to_rotvec(out.target.quaternion)) == pytest.approx(0.2)


def test_release_and_stale_input_deactivate_and_hold() -> None:
    mapper = RelativePoseMapper(MappingConfig(filter_time_constant=0.0, stale_timeout=0.2))
    ee = Pose(np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]))
    mapper.update(sample([0, 0, 0], grip=1.0, stamp=1, received=1.0), ee, now=1.0)
    moved = mapper.update(sample([0.1, 0, 0], grip=1.0, stamp=2, received=1.01), ee, now=1.01)
    released = mapper.update(sample([9, 9, 9], grip=0.0, stamp=3, received=1.02), ee, now=1.02)
    assert not released.active
    np.testing.assert_allclose(released.target.position, moved.target.position)
    stale = mapper.update(sample([9, 9, 9], grip=1.0, stamp=3, received=1.02), ee, now=1.30)
    assert not stale.active
    assert stale.stale
```

- [ ] **Step 5: Run clutch tests and confirm RED**

Expected: missing `RelativePoseMapper` and related dataclasses.

- [ ] **Step 6: Implement stateful mapping**

Implement exact public shapes:

```python
@dataclass(frozen=True)
class MappingConfig:
    translation_scale: float = 1.0
    grip_threshold: float = 0.9
    stale_timeout: float = 0.2
    filter_time_constant: float = 0.05
    max_position_step: float = 0.02
    max_rotation_step: float = 0.15


@dataclass(frozen=True)
class MappingOutput:
    target: Pose
    active: bool
    activated: bool
    deactivated: bool
    stale: bool


```

Implement `RelativePoseMapper.__init__(config)`, `reset(ee_pose)`, and
`update(sample, ee_pose, now) -> MappingOutput`. The implementation must capture references on the
Grip rising edge, use relative position and `R_current R_reference.T` orientation, low-pass the
desired pose, clip each filtered step, retain the last target on release, and clear controller
references on release or stale input.

- [ ] **Step 7: Run tests and commit**

```powershell
.\.venv\Scripts\python.exe -m pytest tests/test_pose_mapping.py -q
git add kinova_teleop/pose_mapping.py tests/test_pose_mapping.py
git commit -m "feat: add relative 6DoF pose mapping"
```

Expected: all pose mapping tests pass.

---

### Task 3: XR Input Adapter and Deterministic Dry-Run Source

**Files:**
- Create: `kinova_teleop/xr_input.py`
- Create: `tests/test_xr_input.py`

**Interfaces:**
- Produces: `ControllerSample`, `XrInputSource`, `SdkXrInput`, and `DryRunXrInput`.
- Consumed by: `RelativePoseMapper` and `TeleopController`.

- [ ] **Step 1: Write failing adapter tests**

```python
import numpy as np
import pytest

from kinova_teleop.xr_input import DryRunXrInput, SdkXrInput


class FakeSdk:
    def __init__(self):
        self.closed = False
    def init(self): pass
    def get_left_controller_pose(self): return np.array([1, 2, 3, 0, 0, 0, 1], dtype=float)
    def get_left_grip(self): return 0.95
    def get_time_stamp_ns(self): return 123
    def close(self): self.closed = True


def test_sdk_adapter_reads_left_controller() -> None:
    sdk = FakeSdk()
    source = SdkXrInput(sdk=sdk, monotonic=lambda: 5.0)
    sample = source.read()
    np.testing.assert_allclose(sample.position, [1, 2, 3])
    np.testing.assert_allclose(sample.quaternion_xyzw, [0, 0, 0, 1])
    assert sample.grip == pytest.approx(0.95)
    assert sample.timestamp_ns == 123
    assert sample.received_monotonic == 5.0
    source.close()
    assert sdk.closed


def test_sdk_import_error_is_actionable(monkeypatch) -> None:
    monkeypatch.setattr("importlib.import_module", lambda _: (_ for _ in ()).throw(ModuleNotFoundError()))
    with pytest.raises(RuntimeError, match="xrobotoolkit_sdk"):
        SdkXrInput()


def test_dry_run_is_deterministic_and_exercises_grip() -> None:
    source = DryRunXrInput(control_hz=100.0)
    samples = [source.read() for _ in range(150)]
    assert samples[0].timestamp_ns < samples[-1].timestamp_ns
    assert any(sample.grip > 0.9 for sample in samples)
    assert all(sample.position.shape == (3,) for sample in samples)
```

- [ ] **Step 2: Run tests and confirm RED**

Expected: import fails because `xr_input.py` does not exist.

- [ ] **Step 3: Implement the input abstraction**

```python
@dataclass(frozen=True)
class ControllerSample:
    position: np.ndarray
    quaternion_xyzw: np.ndarray
    grip: float
    timestamp_ns: int
    received_monotonic: float
    valid: bool = True


class XrInputSource(Protocol):
    def read(self) -> ControllerSample:
        raise NotImplementedError
    def close(self) -> None:
        raise NotImplementedError


class SdkXrInput:
    def __init__(self, sdk=None, monotonic=time.monotonic):
        if sdk is None:
            try:
                sdk = importlib.import_module("xrobotoolkit_sdk")
            except (ImportError, OSError) as exc:
                raise RuntimeError(
                    "xrobotoolkit_sdk is unavailable. Follow README.md to build it in WSL."
                ) from exc
        self._sdk = sdk
        self._monotonic = monotonic
        self._closed = False
        self._sdk.init()

    def read(self) -> ControllerSample:
        raw = np.asarray(self._sdk.get_left_controller_pose(), dtype=np.float64)
        valid = raw.shape == (7,) and np.isfinite(raw).all()
        return ControllerSample(
            position=raw[:3].copy() if valid else np.zeros(3),
            quaternion_xyzw=raw[3:].copy() if valid else np.array([0.0, 0.0, 0.0, 1.0]),
            grip=float(self._sdk.get_left_grip()),
            timestamp_ns=int(self._sdk.get_time_stamp_ns()),
            received_monotonic=self._monotonic(),
            valid=valid,
        )

    def close(self) -> None:
        if not self._closed:
            self._sdk.close()
            self._closed = True
```

`DryRunXrInput` increments an integer step, emits a small sinusoidal translation and rotation,
uses Grip `1.0` after the first 10 samples, and generates timestamps from `step / control_hz`.

- [ ] **Step 4: Run tests and commit**

```powershell
.\.venv\Scripts\python.exe -m pytest tests/test_xr_input.py -q
git add kinova_teleop/xr_input.py tests/test_xr_input.py
git commit -m "feat: add XR and dry-run input adapters"
```

---

### Task 4: MuJoCo-Native Damped Least-Squares IK

**Files:**
- Create: `kinova_teleop/ik_solver.py`
- Create: `tests/test_ik_solver.py`

**Interfaces:**
- Consumes: validated `MjModel`, `ModelContract`, current 7-vector, target `Pose`, posture reference.
- Produces: `IKConfig`, `IKResult`, and `DampedLeastSquaresIK.solve`.

- [ ] **Step 1: Write failing reachable-target and limit tests**

```python
import mujoco
import numpy as np

from kinova_teleop.ik_solver import DampedLeastSquaresIK, IKConfig
from kinova_teleop.model_contract import load_and_validate_model
from kinova_teleop.pose_mapping import Pose, matrix_to_quat


def home_state(model, contract):
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, contract.home_key_id)
    mujoco.mj_forward(model, data)
    q = data.qpos[list(contract.qpos_addresses)].copy()
    pose = Pose(data.site_xpos[contract.site_id].copy(), matrix_to_quat(data.site_xmat[contract.site_id].reshape(3, 3)))
    return q, pose


def test_ik_converges_for_reachable_6dof_target(model_path) -> None:
    model, contract = load_and_validate_model(model_path)
    q, pose = home_state(model, contract)
    target = Pose(pose.position + np.array([0.015, -0.01, 0.01]), pose.quaternion)
    result = DampedLeastSquaresIK(model, contract, IKConfig()).solve(q, target, q)
    assert result.converged
    assert result.position_error < 1e-3
    assert result.rotation_error < 1e-2
    assert np.isfinite(result.qpos).all()


def test_ik_respects_limited_joints_and_stays_finite(model_path) -> None:
    model, contract = load_and_validate_model(model_path)
    q, pose = home_state(model, contract)
    target = Pose(pose.position + np.array([0.5, 0.5, 0.5]), pose.quaternion)
    result = DampedLeastSquaresIK(model, contract, IKConfig(max_iterations=10)).solve(q, target, q)
    assert np.isfinite(result.qpos).all()
    for joint_id, value in zip(contract.joint_ids, result.qpos):
        if model.jnt_limited[joint_id]:
            low, high = model.jnt_range[joint_id]
            assert low <= value <= high
```

- [ ] **Step 2: Run tests and confirm RED**

Expected: import fails because `ik_solver.py` does not exist.

- [ ] **Step 3: Implement IK solver**

```python
@dataclass(frozen=True)
class IKConfig:
    max_iterations: int = 40
    position_tolerance: float = 1e-3
    rotation_tolerance: float = 1e-2
    position_gain: float = 1.0
    rotation_gain: float = 0.7
    base_damping: float = 1e-3
    singular_value_threshold: float = 0.08
    singular_damping_gain: float = 0.2
    nullspace_gain: float = 0.03
    max_joint_step: float = 0.08


@dataclass(frozen=True)
class IKResult:
    qpos: np.ndarray
    converged: bool
    position_error: float
    rotation_error: float
    iterations: int


class DampedLeastSquaresIK:
    def __init__(self, model: mujoco.MjModel, contract: ModelContract, config: IKConfig):
        self.model = model
        self.contract = contract
        self.config = config
        self.data = mujoco.MjData(model)

    def solve(self, q_start: np.ndarray, target: Pose, posture_reference: np.ndarray) -> IKResult:
        q = np.asarray(q_start, dtype=np.float64).copy()
        posture_reference = np.asarray(posture_reference, dtype=np.float64)
        for iteration in range(1, self.config.max_iterations + 1):
            self.data.qpos[list(self.contract.qpos_addresses)] = q
            mujoco.mj_fwdPosition(self.model, self.data)
            position = self.data.site_xpos[self.contract.site_id].copy()
            current_q = matrix_to_quat(self.data.site_xmat[self.contract.site_id].reshape(3, 3))
            position_error = target.position - position
            rotation_error = quat_to_rotvec(quat_multiply(target.quaternion, quat_conjugate(current_q)))
            if (
                np.linalg.norm(position_error) < self.config.position_tolerance
                and np.linalg.norm(rotation_error) < self.config.rotation_tolerance
            ):
                return IKResult(q, True, float(np.linalg.norm(position_error)), float(np.linalg.norm(rotation_error)), iteration)
            jacp = np.zeros((3, self.model.nv))
            jacr = np.zeros((3, self.model.nv))
            mujoco.mj_jacSite(self.model, self.data, jacp, jacr, self.contract.site_id)
            J = np.vstack((jacp[:, self.contract.dof_addresses], jacr[:, self.contract.dof_addresses]))
            e = np.concatenate((self.config.position_gain * position_error, self.config.rotation_gain * rotation_error))
            singular_values = np.linalg.svd(J, compute_uv=False)
            sigma_min = float(singular_values[-1])
            damping = self.config.base_damping + self.config.singular_damping_gain * max(
                0.0, self.config.singular_value_threshold - sigma_min
            ) ** 2
            pinv = J.T @ np.linalg.solve(J @ J.T + damping * damping * np.eye(6), np.eye(6))
            null_projector = np.eye(7) - pinv @ J
            dq = pinv @ e + null_projector @ (
                self.config.nullspace_gain * nearest_angle_delta(posture_reference, q, self.model, self.contract)
            )
            dq = np.clip(dq, -self.config.max_joint_step, self.config.max_joint_step)
            q = self._apply_limits(q + dq, q_start)
        return IKResult(q, False, float(np.linalg.norm(position_error)), float(np.linalg.norm(rotation_error)), self.config.max_iterations)
```

Implement `_apply_limits` and `nearest_angle_delta` so limited joints use `jnt_range`, while
unlimited hinge joints use `reference + atan2(sin(value-reference), cos(value-reference))`.

- [ ] **Step 4: Run IK tests and confirm GREEN**

```powershell
.\.venv\Scripts\python.exe -m pytest tests/test_ik_solver.py -q
```

- [ ] **Step 5: Add a rotation and near-singularity regression test**

Add a reachable `0.05` rad target rotation and assert orientation error decreases below `1e-2`.
Set a known extended configuration, solve a small target, and assert all result fields remain finite.

- [ ] **Step 6: Run all tests and commit**

```powershell
.\.venv\Scripts\python.exe -m pytest -q
git add kinova_teleop/ik_solver.py tests/test_ik_solver.py
git commit -m "feat: add MuJoCo Jacobian IK"
```

---

### Task 5: Teleoperation Controller, CLI, Headless Dry Run, and Target Marker

**Files:**
- Create: `kinova_teleop/teleop_controller.py`
- Create: `kinova_teleop/main.py`
- Create: `kinova_gen3_mujoco/teleop_scene.xml`
- Create: `tests/test_teleop_controller.py`
- Create: `tests/test_cli.py`

**Interfaces:**
- Consumes: `XrInputSource`, `RelativePoseMapper`, `DampedLeastSquaresIK`, model contract.
- Produces: `TeleopConfig`, `StepDiagnostics`, `TeleopController.step_once()`, `TeleopController.run()`, and CLI `main()`.

- [ ] **Step 1: Add a teleoperation scene with a mocap marker**

```xml
<mujoco model="gen3 teleoperation scene">
  <include file="scene.xml"/>
  <worldbody>
    <body name="teleop_target" mocap="true" pos="0 0 0.5">
      <site name="teleop_target_site" type="sphere" size="0.015" rgba="1 0.2 0.1 0.8"/>
      <geom type="box" size="0.04 0.002 0.002" rgba="1 0 0 0.8" contype="0" conaffinity="0"/>
      <geom type="box" size="0.002 0.04 0.002" rgba="0 1 0 0.8" contype="0" conaffinity="0"/>
      <geom type="box" size="0.002 0.002 0.04" rgba="0.2 0.4 1 0.8" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>
```

- [ ] **Step 2: Write failing controller integration tests**

```python
import mujoco
import numpy as np

from kinova_teleop.teleop_controller import TeleopConfig, TeleopController
from kinova_teleop.xr_input import DryRunXrInput


def test_headless_dry_run_stays_finite(teleop_model_path) -> None:
    source = DryRunXrInput(control_hz=100.0)
    controller = TeleopController(
        TeleopConfig(model_path=teleop_model_path, realtime=False),
        source,
    )
    try:
        controller.run(headless=True, max_steps=2000)
        assert np.isfinite(controller.data.qpos).all()
        assert np.isfinite(controller.data.qvel).all()
        assert np.isfinite(controller.data.ctrl).all()
        assert controller.steps == 2000
    finally:
        controller.close()


def test_release_holds_last_actuator_target(teleop_model_path) -> None:
    samples = [
        ControllerSample(
            position=np.array([0.0, 0.0, 0.0]),
            quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
            grip=1.0,
            timestamp_ns=1,
            received_monotonic=1.0,
        ),
        ControllerSample(
            position=np.array([0.01, 0.0, 0.0]),
            quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
            grip=1.0,
            timestamp_ns=2,
            received_monotonic=1.01,
        ),
        ControllerSample(
            position=np.array([1.0, 1.0, 1.0]),
            quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
            grip=0.0,
            timestamp_ns=3,
            received_monotonic=1.02,
        ),
        ControllerSample(
            position=np.array([2.0, 2.0, 2.0]),
            quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
            grip=0.0,
            timestamp_ns=4,
            received_monotonic=1.03,
        ),
    ]
    source = ScriptedInput(samples)
    controller = TeleopController(TeleopConfig(model_path=teleop_model_path, realtime=False), source)
    for _ in range(active_count):
        controller.step_once()
    held = controller.data.ctrl.copy()
    for _ in range(released_count):
        controller.step_once()
    np.testing.assert_allclose(controller.data.ctrl, held)
```

Add a `teleop_model_path` fixture pointing to `teleop_scene.xml`. The scripted samples must use
explicit numeric positions, unit quaternions, increasing timestamps, and Grip `1.0` then `0.0`.

- [ ] **Step 3: Run controller tests and confirm RED**

Expected: import fails because `teleop_controller.py` does not exist.

- [ ] **Step 4: Implement the controller**

```python
@dataclass(frozen=True)
class TeleopConfig:
    model_path: Path
    control_hz: float = 100.0
    realtime: bool = True
    translation_scale: float = 1.0
    stale_timeout: float = 0.2


@dataclass(frozen=True)
class StepDiagnostics:
    active: bool
    stale: bool
    ik_converged: bool
    position_error: float
    rotation_error: float


class TeleopController:
    def __init__(self, config: TeleopConfig, source: XrInputSource):
        self.config = config
        self.source = source
        self.model, self.contract = load_and_validate_model(config.model_path)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.contract.home_key_id)
        mujoco.mj_forward(self.model, self.data)
        self.mapper = RelativePoseMapper(MappingConfig(
            translation_scale=config.translation_scale,
            stale_timeout=config.stale_timeout,
        ))
        self.ik = DampedLeastSquaresIK(self.model, self.contract, IKConfig())
        self.posture_reference = self._joint_positions()
        self.last_valid_target = self.posture_reference.copy()
        self._set_controls(self.last_valid_target)
        self.steps = 0
        self._closed = False

    def _joint_positions(self) -> np.ndarray:
        return self.data.qpos[list(self.contract.qpos_addresses)].copy()

    def _site_pose(self) -> Pose:
        return Pose(
            self.data.site_xpos[self.contract.site_id].copy(),
            matrix_to_quat(self.data.site_xmat[self.contract.site_id].reshape(3, 3)),
        )

    def _set_controls(self, q_target: np.ndarray) -> None:
        for actuator_id, value in zip(self.contract.actuator_ids, q_target):
            self.data.ctrl[actuator_id] = value

```

`step_once` reads one sample, updates the mapper, captures posture reference on activation, solves
only while active, retains the last valid actuator target otherwise, updates the mocap marker if it
exists, performs `round((1/control_hz)/model.opt.timestep)` simulation substeps, and rejects
non-finite results.

`run` uses `mujoco.viewer.launch_passive` when not headless, stops on Viewer close, and sleeps to
the control deadline only when `realtime=True`. It must close the source in `finally`.

- [ ] **Step 5: Write failing CLI tests**

```python
from kinova_teleop.main import main


def test_dry_run_headless_cli_returns_success(teleop_model_path) -> None:
    code = main([
        "--model", str(teleop_model_path),
        "--dry-run",
        "--headless",
        "--steps", "100",
    ])
    assert code == 0


def test_check_xr_reports_missing_sdk(monkeypatch, capsys) -> None:
    monkeypatch.setattr(
        "kinova_teleop.main.SdkXrInput",
        lambda: (_ for _ in ()).throw(RuntimeError("xrobotoolkit_sdk is unavailable")),
    )
    assert main(["--check-xr", "--samples", "1"]) == 2
    assert "xrobotoolkit_sdk" in capsys.readouterr().err
```

- [ ] **Step 6: Implement CLI**

Use `argparse` with the exact flags from the design:

Expose `build_parser() -> argparse.ArgumentParser`,
`check_xr(samples: int | None) -> int`, and
`main(argv: Sequence[str] | None = None) -> int`. The module guard must be:

```python
if __name__ == "__main__":
    raise SystemExit(main())
```

The default model is `kinova_gen3_mujoco/teleop_scene.xml`; `--controller` accepts only `left`;
`--headless` requires a positive `--steps`; errors print to stderr and return `2`.

- [ ] **Step 7: Run controller and CLI tests**

```powershell
.\.venv\Scripts\python.exe -m pytest tests/test_teleop_controller.py tests/test_cli.py -q
.\.venv\Scripts\python.exe -m kinova_teleop.main --dry-run --headless --steps 2000
```

Expected: tests pass and the command exits `0` with finite-state summary.

- [ ] **Step 8: Commit**

```powershell
git add kinova_teleop kinova_gen3_mujoco/teleop_scene.xml tests
git commit -m "feat: add Gen3 teleoperation runtime"
```

---

### Task 6: WSL/PICO Operations Guide

**Files:**
- Create: `README.md`
- Modify: `kinova_gen3_mujoco/README_CN.md`

**Interfaces:**
- Documents: install, configuration, connection checks, dry run, real run, controls, troubleshooting, tests, and scope.

- [ ] **Step 1: Write the README with exact setup commands**

README must include:

```bash
wsl --install -d Ubuntu-22.04
sudo apt update
sudo apt install -y python3-venv python3-dev build-essential cmake git libgl1-mesa-dev libglfw3
cd /mnt/d/yezq/26.7.24_Kinova_controll
python3 -m venv .venv-wsl
source .venv-wsl/bin/activate
python -m pip install --upgrade pip
python -m pip install -e ".[dev]"
```

XR SDK build:

```bash
mkdir -p .local-deps
git clone https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind.git \
  .local-deps/XRoboToolkit-PC-Service-Pybind
cd .local-deps/XRoboToolkit-PC-Service-Pybind
git checkout c64ccf6acd577a333e03b66fafe8efeeceb511b1
bash setup_ubuntu.sh
cd /mnt/d/yezq/26.7.24_Kinova_controll
```

Windows `setting.ini`:

```ini
[Service]
listenAddr=0.0.0.0
listenPort=60061
```

WSL client generation:

```bash
WINDOWS_HOST_IP="$(ip route show default | awk '{print $3}')"
cat > PXREASetting.ini <<EOF
[Client]
connectAddr=${WINDOWS_HOST_IP}
connectPort=60061
EOF
nc -vz "${WINDOWS_HOST_IP}" 60061
```

Document the startup sequence: PICO app/device connection, Windows PC Service, WSL
`--check-xr`, dry run, then normal Viewer. Include the exact commands:

```bash
python -m kinova_teleop.main --check-xr --samples 100
python -m kinova_teleop.main --dry-run --headless --steps 2000
python -m kinova_teleop.main --model kinova_gen3_mujoco/teleop_scene.xml
```

- [ ] **Step 2: Document controls and troubleshooting**

Explicitly document:

- Hold left Grip above `0.9` to move.
- Release Grip to hold and re-clutch.
- Full relative position and orientation tracking.
- `Ctrl+C` or Viewer close exits.
- No gripper behavior.
- Firewall and `nc` checks.
- `xrobotoolkit_sdk` import/DLL/shared-library errors.
- WSLg `DISPLAY`/Viewer checks.
- Stale timestamps and incorrect direction checks.
- How to reduce `--scale`.

- [ ] **Step 3: Verify README commands and links**

Run:

```powershell
rg -n \"setting.ini|PXREASetting.ini|60061|--check-xr|--dry-run|left Grip|WSLg|pytest\" README.md
.\.venv\Scripts\python.exe -m kinova_teleop.main --help
```

Expected: every required topic is found and CLI help exits `0`.

- [ ] **Step 4: Commit**

```powershell
git add README.md kinova_gen3_mujoco/README_CN.md
git commit -m "docs: add WSL PICO operation guide"
```

---

### Task 7: Full Verification, WSL Smoke Test, and GitHub Publication

**Files:**
- Modify only if verification exposes a tested defect.

**Interfaces:**
- Proves: package integrity, tests, headless simulation, WSL installability, Git scope, and remote publication.

- [ ] **Step 1: Run fresh Windows verification**

```powershell
.\.venv\Scripts\python.exe -m pip install -e .[dev]
.\.venv\Scripts\python.exe -m pytest -q
.\.venv\Scripts\python.exe -m kinova_teleop.main --dry-run --headless --steps 5000
```

Expected: zero test failures, command exit `0`, and finite-state summary.

- [ ] **Step 2: Run WSL verification**

```powershell
wsl.exe -d Ubuntu-22.04 bash -lc "cd /mnt/d/yezq/26.7.24_Kinova_controll && \
python3 -m venv .venv-wsl && source .venv-wsl/bin/activate && \
python -m pip install --upgrade pip && python -m pip install -e '.[dev]' && \
python -m pytest -q && \
python -m kinova_teleop.main --dry-run --headless --steps 2000"
```

Expected: zero test failures and headless dry run exits `0`.

- [ ] **Step 3: Audit repository scope**

```powershell
git status -sb
git ls-files
git diff --check
```

Confirm `.venv`, `.venv-wsl`, `.upstream`, `kinova/`, `PXREASetting.ini`, caches, and secrets
are absent. Confirm the Gen3 MJCF, STL assets, license, code, tests, specs, plan, and README are present.

- [ ] **Step 4: Commit any final tracked documentation state**

```powershell
git add -- README.md LICENSE pyproject.toml requirements-mujoco.txt kinova_teleop tests kinova_gen3_mujoco docs .gitignore
git commit -m "feat: complete PICO Gen3 MuJoCo teleoperation"
```

Skip this commit only if `git status --short` is empty.

- [ ] **Step 5: Create the private GitHub repository and push**

First verify authentication:

```powershell
gh auth status
```

Create and push:

```powershell
gh repo create YEZQin/kinova-gen3-pico-teleop-mujoco `
  --private `
  --source . `
  --remote origin `
  --push `
  --description "PICO left-controller teleoperation for a Kinova Gen3 arm in MuJoCo"
```

If the repository already exists, stop and inspect it before changing any remote.

- [ ] **Step 6: Verify the remote**

```powershell
git remote -v
git status -sb
gh repo view YEZQin/kinova-gen3-pico-teleop-mujoco --json nameWithOwner,visibility,url,defaultBranchRef
```

Expected: `visibility` is `PRIVATE`, default branch is `main`, local `main` tracks
`origin/main`, and the working tree is clean.
