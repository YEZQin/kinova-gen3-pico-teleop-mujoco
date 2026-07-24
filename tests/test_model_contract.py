from pathlib import Path

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
