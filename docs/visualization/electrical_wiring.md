# DRON Physical Electrical & Wiring Specification

## 1. UART Telemetry Harness (Pixhawk <-> Raspberry Pi 5)

This harness carries the high-speed serial telemetry (`MicroXRCEAgent` at 921600 baud) for PX4 flight control and visual-inertial SLAM (Kimera-VIO).

### Wire Color Code & Extension Transition
| Original Lead Color | Signal Function | Harness / Splice Color | Destination | Notes |
|---|---|---|---|---|
| **Green** | **RX** (Receive) | 🟤 **Brown** | Pi **Pin 8 (GPIO 14, TXD)** | Connected to transmit on the other end |
| **Yellow** | **TX** (Transmit) | 🟡 **Yellow** | Pi **Pin 10 (GPIO 15, RXD)** | Connected to receive on the other end |
| **Black** | **GND** (Ground) | ⚫ **Black** | Pi **Pin 6 (GND)** | Shared common ground plane |
| **Red** | **+5V** (Power) | 🔴 **Red** | ⚠️ Power Line | Ensure power isolation rules |

> ⚠️ **Power Rule:** Do not power the Pi 5 from the Pixhawk TELEM port. The Pi 5 draws up to 5A at 5V under full vision processing load, which exceeds the Pixhawk internal regulator. The Red (+5V) wire should remain isolated unless connected to an approved external BEC.

---

## 2. Sensor Interconnect Summary

| Sensor | Interface | Physical Connection | Driver / Device Node |
|---|---|---|---|
| **Camera 0 (Left)** | Dual MIPI CSI-2 | 15-to-22 pin CSI ribbon to Pi 5 CAM0 | `libcamera` (`i2c@88000`) |
| **Camera 1 (Right)** | Dual MIPI CSI-2 | 15-to-22 pin CSI ribbon to Pi 5 CAM1 | `libcamera` (`i2c@80000`) |
| **MI48 Thermal** | USB 2.0 | Micro-USB to Pi 5 USB-A 3.0 port | `/dev/ttyACM0` (SenXor SDK) |
| **Pixhawk 6C** | UART Serial | `TELEM2` port to Pi 40-pin GPIO header | `/dev/ttyAMA0` (`MicroXRCEAgent`) |
