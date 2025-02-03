# The MetaBoard™

### Main ICs

- Micro: [nRF53240](https://www.nordicsemi.com/Products/nRF5340) in the MDBT53 module](https://www.raytac.com/download/index.php?index_id=60)
- PMIC (Power manangement IC): [nPM1100](https://www.nordicsemi.com/Products/nPM1100)
- IMU: [BNO085](https://www.ceva-ip.com/wp-content/uploads/2019/10/BNO080_085-Datasheet.pdf)
- Microphone: [MP34DT05-A](https://www.st.com/resource/en/datasheet/mp34dt05-a.pdf)
- Capacitive Touch sensor: [IQS7222A001QNR](https://www.azoteq.com/images/stories/pdf/iqs7222a_datasheet.pdf)
- Haptic Feedback driver: [DRV2605YZFR](https://www.ti.com/lit/ds/symlink/drv2605.pdf)

![WhatsApp Image 2024-11-21 at 10 35 25 PM](https://github.com/user-attachments/assets/1c10d5c2-fe00-4740-94f0-c228e55fa3ba)

### Inputs/Outputs

- RGB Status LED
- PWR LED (optional, requires solder bridge)
- 2x Buttons
- Wake Button
- Sleep trigger (Only available as test pad for production use)
- Magnetic USB connector
- Tag-Connect Programming power
- 10 pin IDC programming port (optional)
- 8 pin FPC connector for 5x touch inputs

### Notes:

- The board includes a 10K NTC thermistor for instances where a battery is used that does not contain an NTC. If the battery includes an NTC, the trace MUST be cut (T_CUT).
- The Power LED is unconnected. Solder the solder bridge next to QR code to enable.
- The Device can be placed into a hardware sleep mode by bridging the two SLP test pads. This is also possible by toggling P0.05

# Unboxing

# The MetaBox

# The MetaBoard Bridge

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


