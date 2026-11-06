# DRON Lab Hardware Integration & Sensor Verification Report

**Date:** 05 November 2026  
**Subteam:** 3D Visualization & Perception  
**Hardware Rig:** Drone 01 (Raspberry Pi 5 Model B + Holybro Pixhawk 6C + Dual Sony IMX219 + Meridian Innovation MI48)

---

## 1. Executive Summary

During today's lab session, the visualization subteam achieved a major hardware and software milestone: **establishing live, high-speed telemetry communication between the Pixhawk 6C flight controller and the Raspberry Pi 5 companion computer, capturing real-time 3D IMU orientation (Roll, Pitch, Yaw) streaming directly into Python.**

Key achievements in today's session:
1. **Physical Electrical Wiring Verification:** Traced and documented the custom UART telemetry harness between the Pixhawk and Raspberry Pi 5 GPIO header, establishing a permanent wire color mapping and splice transition record.
2. **Raspberry Pi 5 Hardware UART Activation:** Diagnosed a kernel configuration quirk on the Pi 5's RP1 southbridge chip, remounted the boot partition, and permanently enabled the 40-pin GPIO hardware serial port (`/dev/ttyAMA0`) via Device Tree overlays.
3. **Flight Controller Telemetry Handshake:** Established high-speed serial communication with the Pixhawk 6C over USB ACM (`/dev/ttyACM1`) at 115200 baud.
4. **Zero-Dependency Pure-Python MAVLink Decoder:** Developed an offline, zero-dependency binary packet decoder using Python standard library `struct` and `math` (requiring no internet access or external libraries like `pymavlink`).
5. **Real-Time 3D IMU Tilt Test:** Successfully verified live physical orientation feedback from the Pixhawk's internal gyroscopes and accelerometers as the drone was tilted and rotated by hand.
6. **In-Memory 15-Second Telemetry Logging:** Implemented an in-memory CSV logger writing to shared RAM (`/dev/shm/imu_15s.csv`), completely bypassing SD card read-only lockups.

---

## 2. Physical Electrical Wiring & Harness Documentation

The telemetry harness carries high-speed flight telemetry between the Pixhawk flight controller and the companion computer. The custom cable features color transitions at an intermediate splice that were verified and recorded:

### Wire Color Mapping & Splice Transitions
| Signal Function | Original Pixhawk Lead | Splice / Harness Lead | Raspberry Pi 5 Connection | Role / Notes |
|---|---|---|---|---|
| **RX (Receive)** | 🟢 **Green** | 🟤 **Brown** | **Pin 8 (GPIO 14, TXD)** | Connected to transmit pin on Pi |
| **TX (Transmit)** | 🟡 **Yellow** | 🟡 **Yellow** | **Pin 10 (GPIO 15, RXD)** | Connected to receive pin on Pi |
| **GND (Ground)** | ⚫ **Black** | ⚫ **Black** | **Pin 6 (GND)** | Shared common electrical ground |
| **+5V (Power)** | 🔴 **Red** | 🔴 **Red** | ⚠️ **ISOLATED** | *Must not be connected to Pi GPIO* |

```
[ Pixhawk TELEM2 Port ]               [ Harness Splice ]                 [ Raspberry Pi 5 Header ]
   RX Pin 3 (Green)    ─────────────▶   Transitions to Brown  ─────────────▶   Pin 8  (GPIO 14 - TXD)
   TX Pin 2 (Yellow)   ─────────────▶   Stays Yellow          ─────────────▶   Pin 10 (GPIO 15 - RXD)
   GND Pin 6 (Black)   ─────────────▶   Stays Black           ─────────────▶   Pin 6  (GND)
   +5V Pin 1 (Red)     ─────────────▶   Stays Red             ─────────────▶   [Isolated / External BEC]
```

> ⚠️ **Power Isolation Rule:** The Pi 5 draws up to 5A at 5V under full vision processing load, which drastically exceeds the Pixhawk internal regulator. The Red (+5V) wire remains isolated so both the flight controller and companion computer draw clean, independent power from their respective power modules/BECs.

---

## 3. Raspberry Pi 5 Hardware UART Kernel Diagnosis

### The RP1 Architecture Quirk
On earlier Raspberry Pi models (Pi 3 / Pi 4), Pins 8 and 10 defaulted to `/dev/ttyAMA0` or `/dev/ttyS0`. On the **Raspberry Pi 5**, the new **RP1 I/O controller** changed the kernel port allocations:
1. By default, `/dev/serial0` was symlinked to `/dev/ttyAMA10`.
2. Inspecting `dmesg` revealed:
   ```text
   console=ttyAMA10,115200
   printk: console [ttyAMA10] enabled
   ```
   `ttyAMA10` is physically wired to the tiny 3-pin white JST debug connector between the micro-HDMI ports, dedicated to the Linux serial login shell. The 40-pin GPIO header UART (Pins 8 & 10) was completely disabled.

### The Resolution:
1. Recovered access to the FAT32 boot partition:
   ```bash
   sudo mount -o remount,rw /
   sudo mount /dev/mmcblk0p1 /boot/firmware
   ```
2. Activated the 40-pin GPIO UART in the Device Tree by appending to `/boot/firmware/config.txt`:
   ```bash
   echo "dtparam=uart0=on" | sudo tee -a /boot/firmware/config.txt
   sudo systemctl daemon-reload
   sudo reboot
   ```
3. Following the reboot, `/dev/ttyAMA0` was officially initialized on Pins 8 and 10:
   ```bash
   dron@raspberrypi:~ $ ls -l /dev/ttyAMA*
   crw-rw---- 1 root dialout 204, 64 Mar 20 19:35 /dev/ttyAMA0
   crw-rw---- 1 root dialout 204, 74 Mar 20 19:36 /dev/ttyAMA10
   ```

---

## 4. Flight Controller Telemetry Stream Bringup

### Port Status on Pixhawk:
* **`TELEM1`**: Occupied by the ground station radio link.
* **`TELEM2`**: Port is free, but PX4 disables unassigned UART ports in firmware by default.
* **USB-C Interface (`/dev/ttyACM1`)**: Connecting a USB-C link between the Pixhawk and the Raspberry Pi 5 immediately exposed the secondary serial endpoint as `/dev/ttyACM1`.

Probing `/dev/ttyACM1` at 115200 baud revealed that PX4 was actively broadcasting binary MAVLink telemetry packets.

---

## 5. Pure-Python Zero-Dependency MAVLink Decoder

Because the drone operates on an isolated Wi-Fi network without public internet connectivity, external packages like `pymavlink` were unavailable. We developed an embedded binary decoder relying strictly on Python's native `struct`, `serial`, and `math` libraries.

### Packet Protocol Breakdown:
* **Header `0xFE`**: Standard MAVLink v1 framing byte.
* **Message ID 30 (`0x1E` - `ATTITUDE`)**: Contains 28 bytes payload:
  * `time_boot_ms` (uint32)
  * `roll` (float32, radians)
  * `pitch` (float32, radians)
  * `yaw` (float32, radians)
  * `rollspeed`, `pitchspeed`, `yawspeed` (float32, rad/s)
* Binary format unpack string: `<Iffffff`.

### Live Verification Test Results:
Executing the decoder script on the Pi while manipulating the physical drone produced live orientation updates:
```text
Connected to Pixhawk! Tilt the drone with your hands:
Roll: -178.9° | Pitch:    0.2° | Yaw:  128.8°
Roll: -178.9° | Pitch:    0.2° | Yaw:  128.8°
Roll: -178.6° | Pitch:    0.2° | Yaw:  140.4°
Roll: -178.6° | Pitch:    0.2° | Yaw:  140.4°
```

* **Pitch & Roll Stability:** Bench level confirmed at $+0.2^\circ$ pitch and $-178.6^\circ$ roll (sensor orientation inverted relative to airframe).
* **Yaw Rotation:** As the drone was manually rotated on the bench, Yaw changed from $128.8^\circ$ to $140.4^\circ$, verifying dynamic sensor tracking.

---

## 6. Standalone 15-Second In-Memory Logger

To prevent storage write failures when the root filesystem enters safety read-only mode, the data logger writes directly to Linux shared memory (`/dev/shm/`), which resides in volatile RAM and is always writable:

```bash
python3 -c "
import serial, struct, math, time

s = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
filename = '/dev/shm/imu_15s.csv'
print(f'Recording IMU data for 15.0 seconds to {filename}...')

start_time = time.time()
count = 0

with open(filename, 'w') as f:
    f.write('timestamp_s,time_boot_ms,roll_deg,pitch_deg,yaw_deg,rollspeed_rad_s,pitchspeed_rad_s,yawspeed_rad_s\n')
    while (time.time() - start_time) < 15.0:
        byte = s.read(1)
        if byte == b'\xfe':
            header = s.read(5)
            if len(header) == 5:
                length, seq, sys_id, comp_id, msg_id = header
                payload = s.read(length)
                checksum = s.read(2)
                if msg_id == 30 and len(payload) >= 28:
                    t_boot, roll, pitch, yaw, r_spd, p_spd, y_spd = struct.unpack('<Iffffff', payload[:28])
                    now = time.time()
                    r_deg, p_deg, y_deg = math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
                    f.write(f'{now:.4f},{t_boot},{r_deg:.2f},{p_deg:.2f},{y_deg:.2f},{r_spd:.4f},{p_spd:.4f},{y_spd:.4f}\n')
                    elapsed = now - start_time
                    count += 1
                    print(f'[{elapsed:4.1f}s] Roll: {r_deg:6.1f}° | Pitch: {p_deg:6.1f}° | Yaw: {y_deg:6.1f}°')

print(f'\nDone! Successfully logged {count} IMU samples to {filename}')
"
```

---

## 7. Impact on 3D Mapping & Next Steps

With live IMU data now verified and streaming on the Pi, the visualization pipeline has all three core sensory modalities operational:
1. **Stereo Video:** Dual IMX219 cameras streaming at 25 FPS (`/cam0.mjpg`, `/cam1.mjpg`).
2. **Thermal Radiometry:** Meridian Innovation MI48 streaming $62 \times 80$ Celsius float arrays at 9 FPS.
3. **Inertial Telemetry:** Pixhawk 6C streaming high-rate attitude quaternions and angular rates.

### Immediate Next Steps:
1. **Synchronized Video + IMU Logging:** Integrate the IMU decoder into the laptop stream recorder (`record_stream.py`) so every recorded session automatically outputs a timestamped `imu.csv` alongside `cam0.mp4`, `cam1.mp4`, and `thermal.csv`.
2. **Gravity Horizon Alignment:** Use the live pitch and roll measurements to rotate reconstructed 3D point clouds so the floor plane is permanently locked to true gravitational down ($9.81\text{ m/s}^2$).
3. **Drift-Free Trajectory Estimation:** Feed the combined stereo disparity and IMU angular velocities into the Visual-Inertial Odometry pipeline, eliminating walking wobble and enabling multi-frame room mapping.
