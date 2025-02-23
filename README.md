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

<img width="100%" alt="Screenshot 2025-02-03 at 10 55 24" src="https://github.com/user-attachments/assets/d68e97e9-04f0-47c4-a3f2-478378f89a9d" />
<br>
<br>

A Python application that bridges Metabow Bluetooth devices with OSC (Open Sound Control), providing real-time audio processing and recording capabilities.

A Python application that bridges Metabow Bluetooth devices with OSC (Open Sound Control), providing comprehensive audio processing, recording capabilities, and motion data routing.

### 1. Features

- **Device Management**: connect to multiple Metabow devices simultaneously via Bluetooth LE
- **Audio Processing**: real-time audio capture and processing with configurable gain, noise gate, and noise reduction
- **OSC Integration**: 
  - Flexible routing system for audio and motion data
  - Support for individual routes and bundled messages
  - Customizable OSC paths and message bundling
  - Multiple OSC destinations with independent routing configurations
- **Recording**: WAV file recording with timestamp-based naming
- **Monitoring**: 
  - Real-time audio levels (peak and noise floor)
  - Latency monitoring (average, peak, and buffer)
  - Comprehensive logging system
  - Route discovery and monitoring
- **Virtual Audio Output**: integration with VB-Cable for virtual audio device routing (Note: This feature is currently in development and not fully functional)

### 2. Requirements

#### 2.1 Core Dependencies
- `Python 3.x`
- `bleak`
- `python-osc`
- `numpy`
- `tkinter`
- `pyaudio`
- `sounddevice`

#### 2.2 Optional Dependencies
- VB-Cable (for virtual audio routing - feature in development)

### 3. Quick Start

1. Install dependencies `pip install bleak python-osc numpy pyaudio sounddevice`
   
3. Run the application ```python metabow_bridge.py```

5. In the GUI:
   - **Add OSC destinations** (specify ports)
   - **Scan and connect** to Metabow devices
   - **Configure routing:**
     - Add individual routes for specific data
     - Create bundles for combined messages
     - Customize OSC paths as needed
   - **Adjust audio** processing settings if needed
   - **Start recording** (optional)

### 4. Technical Notes 

**4.1 Audio System**
- Optimized for MP34DT05-A PDM microphone
- Configurable audio processing parameters
- Real-time audio monitoring and statistics

**4.2 Data Protocol**
- Bluetooth protocol combines PCM audio data with motion data
- Motion data includes 13 float values:
  - Quaternion (i, j, k, r)
  - Accelerometer (x, y, z)
  - Gyroscope (x, y, z)
  - Magnetometer (x, y, z)
- Status flag for data validation

**4.3 OSC Implementation**
- Flexible routing system with support for:
  - Individual routes (/metabow/audio, /metabow/motion/*)
  - Custom path mapping
  - Message bundling for combined data
  - Multiple independent destinations
- Real-time route discovery and management
 
### 5. GUI Sections
**5.1 Bluetooth Devices**
- Device scanning
- Connection management

**5.2 OSC Routing**
- Destination management
- Route configuration
- Bundle creation

**5.3 Audio Controls**
- Processing parameters
- Recording controls
- Virtual output (in development)

**5.4 Monitoring**
- Audio levels
- Latency metrics
- System logs

### 6. Contributing
We welcome contributions! Please feel free to submit a Pull Request.
