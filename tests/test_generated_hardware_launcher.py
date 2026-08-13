from __future__ import annotations

from pathlib import Path


def test_prepare_script_only_hands_password_to_the_readonly_t0_child() -> None:
    """Moving secret handling into package generation would persist credentials."""
    script = Path("scripts/prepare_gen3_hardware.ps1").read_text(encoding="utf-8")

    assert "Read-Host -AsSecureString" in script
    assert "KINOVA_PASSWORD" in script
    assert "finally" in script
    assert "package" not in script.split("Read-Host -AsSecureString", 1)[0]


def test_generated_launcher_validates_offline_before_pico_or_password() -> None:
    """Reordering the gates could connect or disclose a secret for an invalid package."""
    script = Path("scripts/start_generated_gen3_teleop.ps1").read_text(encoding="utf-8")

    offline_gate = script.index("--validate-motion-package")
    pico_gate = script.index("--check-input")
    password_gate = script.index("Read-Host -AsSecureString")
    assert offline_gate < pico_gate < password_gate
    assert "HARDWARE-READY" in script
    assert "Type MOVE" not in script
    assert "--translation-only" in script
    assert "--expanded-translation-envelope" in script
    assert "--responsive-translation-profile" in script
    assert "--recover-stale-input" in script
    assert "'--scale','0.8'" in script
    assert "'--control-hz','40'" in script
    assert "--gripper" not in script
    assert "finally" in script
