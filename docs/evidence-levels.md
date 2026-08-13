# Evidence levels

| Level | Meaning | What it does not establish |
| --- | --- | --- |
| Automated / offline verified | Unit and integration tests, manifest checks, package validation, and deterministic MuJoCo finite run passed. | A connected headset, robot, physical stopping distance, or safe workspace. |
| Installation-local PICO gate | This operator observed fresh, tracked PICO left-controller samples and a Grip release on this installation. | Kortex connectivity or arm movement. |
| Read-only T0 | This robot/session produced a current read-only Kortex observation with expected model, firmware, state, and finite TCP pose. | Approval for motion or completed physical checks. |
| Onsite observed | An operator recorded the specified guarded physical trial with the same robot and code revision. | Future-session performance or unattended operation. |

The public target is Gen3 L53 7 DoF, firmware `2.8.0-5`, Kortex 2.8. The final expanded/asymmetric profile is **offline verified and not hardware-validated**. It must receive new onsite evidence before any hardware claim.
