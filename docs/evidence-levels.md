# Evidence levels

| Level | Meaning | What it does not establish |
| --- | --- | --- |
| Automated / offline verified | Unit and integration tests, manifest checks, package validation, and deterministic MuJoCo finite run passed. | A connected headset, robot, physical stopping distance, or safe workspace. |
| Installation-local PICO gate | This operator observed fresh, tracked PICO left-controller samples and a Grip release on this installation. | Kortex connectivity or arm movement. |
| Read-only T0 | This robot/session produced a current read-only Kortex observation with expected model, firmware, state, and finite TCP pose. | Approval for motion or completed physical checks. |
| Onsite observed | An operator recorded the specified guarded physical trial with the same robot and code revision. | Future-session performance or unattended operation. |

The public target is Gen3 L53 7 DoF, firmware `2.8.0-5`, Kortex 2.8. The tuned advanced gripper profile has installation-local onsite observation at core control commit `b40eb57` and remains automatically checked/offline verified in this branch. That observation is not hardware-validated as a universal safety claim; every new robot, workspace, calibration, payload, firmware, or publication follow-up commit needs fresh onsite evidence before making a stronger hardware claim.
