# MetaBoard™

[![Buy Now](https://github.com/user-attachments/assets/0c43f235-db9e-4589-929c-abc4d3690541)](https://www.paypal.com/ncp/payment/BA7CCF7549GLA)

# MetaBoard
### Main ICs

- Micro: [nRF53240](https://www.nordicsemi.com/Products/nRF5340) in the [MDBT53](https://www.raytac.com/download/index.php?index_id=60) module
- PMIC (Power manangement IC): [nPM1100](https://www.nordicsemi.com/Products/nPM1100)
- IMU: [BNO085](https://www.ceva-ip.com/wp-content/uploads/2019/10/BNO080_085-Datasheet.pdf)
- Microphone: [MP34DT05-A](https://www.st.com/resource/en/datasheet/mp34dt05-a.pdf)
- Capacitive Touch sensor: [IQS7222A001QNR](https://www.azoteq.com/images/stories/pdf/iqs7222a_datasheet.pdf)
- Haptic Feedback driver: [DRV2605YZFR](https://www.ti.com/lit/ds/symlink/drv2605.pdf)

![_newWhatsApp Image 2024-11-21 at 10 35 25 PM](https://github.com/user-attachments/assets/a0e07bbb-69da-4184-be8f-96bc5452a5a3)

### Inputs/Outputs

- RGB Status LED
- PWR LED (optional, requires solder bridge)
- 2x Buttons
- Wake Button
- Sleep trigger (only available as test pad for production use)
- Magnetic USB connector
- Tag-Connect Programming power
- 10 pin IDC programming port (optional)
- 8 pin FPC connector for 5x touch inputs

### Notes:

- The board includes a 10K NTC thermistor for instances where a battery is used that does not contain an NTC. If the battery includes an NTC, the trace MUST be cut (T_CUT).
- The Power LED is unconnected. Solder the solder bridge next to QR code to enable.
- The Device can be placed into a hardware sleep mode by bridging the two SLP test pads. This is also possible by toggling P0.05

# Firmware

Build with nRF Connect SDK 2.4.2 using the VSCode extension. The app is setup as a standalone application. Choose the ```nrf5340dk_nrf5340_cpuapp``` board target and the ```metaboard.overlay``` in your build settings to build for production hardware.

Overlays for other targets have been provided but not recently tested. 

**NEVER** enable DCDC for metabow targets, it WILL brick the board.

The current firmware is designed to output 13 floating-point values via BLE: 4 values representing a quaternion and 3 values each for raw accelerometer, gyroscope, and magnetometer data. The firmware can be modified to stream additional sensor fusion values and support improved audio encoding. Firmware-level integration of the Touch IC and haptic transducer is still pending.

# Unboxing

https://github.com/user-attachments/assets/5d7addd9-e810-4c3d-97cc-f49c3b631e52

The MetaBoard Development Kit includes:
- (1) MetaBoard
- (1) MetaBox
- (1) LiPo Battery
- (1) Magentic 4-pin Charger
- (1) Touch IC Flexboard
- (1) Haptic Capacitor 

# MetaBox

# MetaBoard Bridge

<img width="789" alt="Screenshot 2025-02-03 at 10 55 24" src="https://github.com/user-attachments/assets/d68e97e9-04f0-47c4-a3f2-478378f89a9d" />
<br>
<br>

A Python application that bridges Metabow Bluetooth devices with OSC (Open Sound Control), providing real-time audio processing and recording capabilities.

###  Features

- **Device Management**: connect to multiple Metabow devices simultaneously via Bluetooth LE
- **Audio Processing**: real-time audio capture and processing with configurable gain, noise gate, and noise reduction
- **OSC Integration**: route processed audio and motion data to multiple OSC destinations
- **Recording**: save processed audio as WAV files with timestamp-based naming
- **Monitoring**: real-time audio levels and latency monitoring via GUI

### Requirements

- Python 3.x
- Dependencies: `bleak`, `python-osc`, `numpy`, `tkinter`

### Quick Start

1. Install dependencies ```pip install bleak python-osc numpy```
   
3. Run the application ```python metabow_bridge.py```

5. In the GUI:
- **Add OSC destinations** (specify ports)
- **Scan and connect** to Metabow devices
- **Adjust audio** processing settings if needed
- **Start recording** (optional)

### Technical Notes 
 
- Optimized for MP34DT05-A PDM microphone
- Bluetooth protocol: PCM audio + motion data (13 floats) + status flag
- OSC messages: /metabow/pcm (audio) and /metabow/motion (sensor data)
