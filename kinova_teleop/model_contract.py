"""Validation helpers for the Kinova Gen3 MuJoCo model."""

from dataclasses import dataclass
from pathlib import Path

import mujoco

JOINT_NAMES = tuple(f"joint_{index}" for index in range(1, 8))
SITE_NAME = "pinch_site"
HOME_KEY = "home"


@dataclass(frozen=True)
class ModelContract:
    """Resolved IDs and addresses required by the teleoperation controller."""

    site_name: str
    site_id: int
    joint_names: tuple[str, ...]
    joint_ids: tuple[int, ...]
    qpos_addresses: tuple[int, ...]
    dof_addresses: tuple[int, ...]
    actuator_ids: tuple[int, ...]
    home_key_id: int


def _require_id(model: mujoco.MjModel, object_type: mujoco.mjtObj, name: str) -> int:
    object_id = mujoco.mj_name2id(model, object_type, name)
    if object_id < 0:
        raise ValueError(f"Required MuJoCo object '{name}' is missing")
    return object_id


def load_and_validate_model(path: Path) -> tuple[mujoco.MjModel, ModelContract]:
    """Load an MJCF file and resolve the fixed Gen3 teleoperation contract."""

    resolved_path = Path(path).resolve()
    if not resolved_path.is_file():
        raise FileNotFoundError(f"MuJoCo model not found: {resolved_path}")

    model = mujoco.MjModel.from_xml_path(str(resolved_path))
    site_id = _require_id(model, mujoco.mjtObj.mjOBJ_SITE, SITE_NAME)
    home_key_id = _require_id(model, mujoco.mjtObj.mjOBJ_KEY, HOME_KEY)
    joint_ids = tuple(
        _require_id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in JOINT_NAMES
    )
    qpos_addresses = tuple(int(model.jnt_qposadr[joint_id]) for joint_id in joint_ids)
    dof_addresses = tuple(int(model.jnt_dofadr[joint_id]) for joint_id in joint_ids)

    actuator_ids: list[int] = []
    for joint_id, joint_name in zip(joint_ids, JOINT_NAMES):
        matches = [
            actuator_id
            for actuator_id in range(model.nu)
            if int(model.actuator_trnid[actuator_id, 0]) == joint_id
        ]
        if len(matches) != 1:
            raise ValueError(f"Joint '{joint_name}' must have exactly one actuator")
        actuator_ids.append(matches[0])

    contract = ModelContract(
        site_name=SITE_NAME,
        site_id=site_id,
        joint_names=JOINT_NAMES,
        joint_ids=joint_ids,
        qpos_addresses=qpos_addresses,
        dof_addresses=dof_addresses,
        actuator_ids=tuple(actuator_ids),
        home_key_id=home_key_id,
    )
    return model, contract
