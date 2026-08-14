# PICO UDP Bridge details

Follow the ordered [root README](../README.md) for installation and hardware gates. The included Unity OpenXR bridge uses the PICO left controller and UDP discovery on port `15031`; it needs no fixed PICO address.

Use PICO 4 or PICO 4 Ultra on the same trusted LAN/VLAN as the PC. Permit only inbound UDP 15031 for Python on the trusted Windows network profile if prompted. Do not use public networks, broad firewall rules, AP/client isolation, VPNs that block local traffic, `adb tcpip`, `adb connect`, or ADB reverse. The scripts do not change firewall rules.

The APK is intentionally not redistributed because the pinned PICO SDK terms do not provide an open-source redistribution license. Follow the root README to use Unity `2022.3.62f3c1` and the tracked `scripts/build_pico_udp_bridge.ps1` to build it locally. The script can install only the newly built artifact to exactly one authorized ADB device and does not use wireless ADB. The official pinned SDK [source](https://github.com/Pico-Developer/PICO-Unity-OpenXR-SDK/tree/3aa3e62bff41df618529eeb60ff02c29a515dafe) and [license](https://github.com/Pico-Developer/PICO-Unity-OpenXR-SDK/blob/3aa3e62bff41df618529eeb60ff02c29a515dafe/LICENSE.md) remain upstream.

Start **Kinova PICO Bridge** on the headset and pass the PICO gate before simulation or hardware work. For failures, check that the app is foreground, the left controller is awake and tracked, Grip is released, and the PC/headset network can carry UDP discovery. A release below `0.8` followed by a press above `0.9` is required for a new clutch session.
