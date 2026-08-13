# PICO UDP Bridge details

Follow the ordered [root README](../README.md) for installation and hardware gates. The included Unity OpenXR bridge uses the PICO left controller and UDP discovery on port `15031`; it needs no fixed PICO address.

Use PICO 4 or PICO 4 Ultra on the same trusted LAN/VLAN as the PC. Permit only inbound UDP 15031 for Python on the trusted Windows network profile if prompted. Do not use public networks, broad firewall rules, AP/client isolation, VPNs that block local traffic, `adb tcpip`, `adb connect`, or ADB reverse. The scripts do not change firewall rules.

The expected APK is a `v0.2.0-rc.1` release asset. `bootstrap_public_teleop.ps1 -InstallApk` requires exactly one authorized ADB device and verifies the manifest before installation. If building locally, use Unity `2022.3.62f3c1` with Android tooling and `scripts/build_pico_udp_bridge.ps1`; no Unity installation is redistributed here.

Start **Kinova PICO Bridge** on the headset and pass the PICO gate before simulation or hardware work. For failures, check that the app is foreground, the left controller is awake and tracked, Grip is released, and the PC/headset network can carry UDP discovery. A release below `0.8` followed by a press above `0.9` is required for a new clutch session.
