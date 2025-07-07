# MetaBoard™

[![Buy Now](https://github.com/user-attachments/assets/0c43f235-db9e-4589-929c-abc4d3690541)](https://www.paypal.com/ncp/payment/BA7CCF7549GLA)

# MetaBoard

![Board_mount-ezgif com-video-to-gif-converter](https://github.com/user-attachments/assets/a8ff25df-45c3-48f4-9fb2-3c85edcdc59c)

### Main ICs

- Micro: [nRF53240](https://www.nordicsemi.com/Products/nRF5340) in the [MDBT53](https://www.raytac.com/download/index.php?index_id=60) module
- PMIC (Power manangement IC): [nPM1100](https://www.nordicsemi.com/Products/nPM1100)
- IMU: [BNO086](https://www.ceva-ip.com/wp-content/uploads/2019/10/BNO080_085-Datasheet.pdf)
- Microphone: [MP34DT05-A](https://www.st.com/resource/en/datasheet/mp34dt05-a.pdf)
- Capacitive Touch sensor: [IQS7222A001QNR](https://www.azoteq.com/images/stories/pdf/iqs7222a_datasheet.pdf)
- Haptic Feedback driver: [DRV2605YZFR](https://www.ti.com/lit/ds/symlink/drv2605.pdf)

![_newWhatsApp Image 2024-11-21 at 10 35 25 PM](https://github.com/user-attachments/assets/a0e07bbb-69da-4184-be8f-96bc5452a5a3)

### Inputs/Outputs

- RGB Status LED
- PWR/SOC LED
- 2x Programmable Buttons
- Wake, RST, and Sleep Buttons
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

![MetaBox-ezgif com-video-to-gif-converter](https://github.com/user-attachments/assets/fb4cad53-14b2-437c-afcd-14f95e22bbc0)


# MetaBoard Bridge

<img width="100%" alt="Screenshot 2025-06-10 at 10 44 19" src="https://github.com/user-attachments/assets/bca1c253-9071-40ee-8135-0b5504e1d6cd" />

<br>A Python application that bridges Metabow Bluetooth devices with OSC (Open Sound Control), providing comprehensive audio processing, motion data routing, real-time feature extraction, and advanced IMU calibration capabilities.

## Features

### Device Management
- **Bluetooth LE Connection**: connect to multiple Metabow devices simultaneously.
- **Real-time Status Monitoring**: RSSI signal strength, connection quality, and data packet tracking.
- **Connection Recovery**: automatic reconnection handling and failure tracking.

### Advanced Audio Processing
- **Real-time Audio Capture**: high-quality audio processing with configurable parameters.
- **Audio Feature Extraction**: 
  - MFCC (Mel-frequency Cepstral Coefficients)
  - Spectral features (centroid, bandwidth, contrast, rolloff)
  - Chroma features for harmonic analysis
  - Tonnetz for tonal analysis
  - Custom bow force estimation for violin applications
- **Audio Enhancement**: configurable gain, noise gate, and noise reduction.
- **Virtual Audio Output**: VB-Cable integration for routing to audio software.

### Motion Data Processing
- **IMU Data Smoothing**: multiple filtering algorithms including:
  - Moving Average: simple averaging filter that reduces noise by averaging recent samples over a configurable window.
  - Exponential Moving Average (EMA): weighted average that gives more importance to recent samples while maintaining responsiveness.
  - Kalman Filter: optimal estimation filter that uses prediction and measurement models to minimize estimation error.
  - Savitzky-Golay Filter: polynomial smoothing filter that preserves signal features while reducing noise.
  - Median Filter: non-linear filter that replaces each sample with the median of neighboring samples to remove outliers.
  - Gaussian Filter: applies Gaussian weighting to smooth data while preserving important signal characteristics.
- **Axis Calibration**: 3D visualization and calibration tools for proper IMU orientation mapping.
- **Real-time 3D Visualization**: interactive 3D model display with STL file support.

### OSC Integration
- **Flexible Routing System**: individual routes and bundled messages with customizable paths.
- **Multiple Destinations**: independent routing configurations for different applications.
- **Route Discovery**: automatic detection and management of available data streams.
- **Data Logging**: comprehensive JSON export of all OSC data with timestamps.

### System Monitoring
- **Performance Tracking**: real-time CPU usage, memory monitoring, and processing latency.
- **Comprehensive Logging**: floating log window with exportable logs.
- **Audio Monitoring**: peak levels, noise floor, and latency metrics.

## Requirements

### Core Dependencies
```bash
pip install bleak python-osc numpy tkinter pyaudio sounddevice
```

### Advanced Features Dependencies
```bash
pip install scipy scikit-learn librosa matplotlib psutil
```

### 3D Visualization (Optional)
```bash
pip install numpy-stl matplotlib
```

### System Requirements
- **VB-Cable** (for virtual audio routing) - Download from [VB-Audio.com](https://vb-audio.com/Cable/)
- **Python 3.8+** recommended for optimal performance

## Quick Start

1. **Install Dependencies**
   ```bash
   pip install bleak python-osc numpy pyaudio sounddevice scipy scikit-learn librosa matplotlib psutil numpy-stl
   ```

2. **Run the Application**
   ```bash
   python testing_file_floating_calibration_NIME_CPU.py
   ```

3. **Basic Setup**
   - **Add OSC Destinations**: specify target ports (e.g., 6448 for Wekinator).
   - **Scan and Connect**: find and connect to Metabow devices.
   - **Configure Audio Features**: enable desired audio feature extraction.
   - **Set up IMU Processing**: configure axis calibration and smoothing if needed.

4. **Advanced Configuration**
   - **Audio Features**: open "Feature Extraction" to configure MFCC, spectral features, etc.
   - **IMU Calibration**: use "Axis Calibration" for 3D visualization and orientation mapping.
   - **Data Logging**: enable "Save JSON" to capture all OSC data for analysis.

## Technical Architecture

### Audio Processing Pipeline
- **Input**: MP34DT05-A PDM microphone data via Bluetooth.
- **Processing**: real-time feature extraction using librosa and scikit-learn.
- **Output**: individual features or bundled data via OSC.
- **Features Available**:
  - `mfcc`: Mel-frequency cepstral coefficients (13 coefficients)
  - `spectral_centroid`: Brightness indicator
  - `spectral_bandwidth`: Spectral width measure
  - `spectral_contrast`: Harmonic vs. noise ratio
  - `spectral_rolloff`: High-frequency content measure
  - `chroma_*`: Harmonic content analysis (12 bins)
  - `tonnetz`: Tonal space analysis (6 dimensions)
  - `bow_force_*`: Custom violin bow pressure estimation

### Motion Data Processing
- **Raw Data**: 13-component IMU data (quaternion, accelerometer, gyroscope, magnetometer).
- **Calibration**: 3x3 transformation matrices for axis remapping.
- **Smoothing**: configurable filters (Kalman, EMA, Gaussian, etc.).
- **Visualization**: real-time 3D rendering with STL model support.

### OSC Data Structure
```
/metabow/audio                   # Raw PCM audio data
/metabow/audio/mfcc              # MFCC features (array)
/metabow/audio/spectral_centroid # Spectral centroid (float)
/metabow/audio/bow_force_rms     # Bow force estimation (float)
/metabow/motion/quaternion_*     # Quaternion components
/metabow/motion/accelerometer_*  # Accelerometer data
/metabow/motion/gyroscope_*      # Gyroscope data
/metabow/motion/magnetometer_*   # Magnetometer data
```

### Data Logging Format
```json
{
  "metadata": {
    "export_timestamp": 1641234567.89,
    "total_entries": 15000,
    "duration_seconds": 30.5,
    "data_types": ["motion", "audio_feature", "bundle"]
  },
  "data": [
    {
      "timestamp": 1641234567.89,
      "osc_path": "/metabow/audio/mfcc",
      "value": [1.2, -0.5, 0.8, ...],
      "data_type": "audio_feature"
    }
  ]
}
```

## GUI Interface

### Bluetooth Devices Panel
- **Device Control**: scan, connect, disconnect multiple devices.
- **Status Monitoring**: real-time RSSI, signal quality, and data rates.
- **Configuration**: access to IMU smoothing, axis calibration, and audio features.

### OSC Routing Panel
- **Destination Management**: add/remove OSC targets with port configuration.
- **Route Configuration**: individual data stream routing with custom paths.
- **Bundle Management**: combine multiple data streams into single OSC messages.
- **Real-time Monitoring**: view active routes and data flow.

### Audio Controls Panel
- **Virtual Output**: VB-Cable integration for audio software routing.
- **Recording**: WAV file capture with timestamp naming.
- **Processing Controls**: cain, gate threshold, noise reduction.
- **Monitoring**: peak levels, noise floor, latency metrics.

### System Performance Panel
- **CPU Monitoring**: real-time usage tracking and performance warnings.
- **Memory Usage**: process memory consumption and system resources.
- **Data Logging**: JSON export controls and buffer status.

## Advanced Features

### IMU Axis Calibration
- **3D Visualization**: interactive matplotlib-based 3D display.
- **STL Model Support**: Load custom 3D models for visualization.
- **Manual Rotation**: Apply offset rotations for proper orientation.
- **Calibration Presets**: Save and load calibration configurations.

### Audio Feature Extraction
- **Configurable Parameters**: adjust frame sizes, hop lengths, and feature parameters.
- **Real-time Processing**: low-latency feature extraction suitable for live performance.
- **Machine Learning Ready**: features formatted for direct use with Wekinator or custom ML models.

### Performance Optimization
- **CPU Monitoring**: real-time performance tracking and optimization warnings.
- **Processing Statistics**: detailed latency and throughput metrics.
- **Memory Management**: efficient buffering and resource cleanup.

## Use Cases

### Music Performance
- **Real-time Control**: use motion and audio features to control live audio processing.
- **Gesture Recognition**: train ML models on combined motion and audio data.
- **Interactive Systems**: create responsive musical instruments and installations.

### Research Applications
- **Data Collection**: comprehensive logging of multimodal sensor data.
- **Analysis**: JSON export for offline analysis and machine learning.
- **Prototyping**: rapid development of sensor-based interactive systems.

### Educational Projects
- **NIME (New Interfaces for Musical Expression)**: complete toolkit for digital music interface research.
- **Sensor Fusion**: explore combination of audio and motion data.
- **Real-time Systems**: learn about low-latency sensor processing.

## Troubleshooting

### Common Issues
- **VB-Cable Not Detected**: install VB-Cable from official website and restart application.
- **High CPU Usage**: reduce audio feature extraction frame rate or disable unused features.
- **Connection Drops**: check Bluetooth signal strength and reduce distance to device.

### Performance Optimization
- **Reduce Latency**: lower audio buffer sizes and disable unnecessary features.
- **Memory Usage**: adjust data logging buffer sizes and clear logs regularly.
- **3D Visualization**: use lower-resolution STL models for better performance.

## Contributing

We welcome contributions! Please feel free to submit Pull Requests for:
- Additional audio features and analysis algorithms
- Enhanced 3D visualization capabilities
- Performance optimizations
- Documentation improvements
- Bug fixes and stability improvements

## License

This project is open source. Please check the license file for specific terms and conditions.


# Bootloading
### Prerequisites

- snRF Connect app installed on your mobile device
- Programming hardware for initial firmware flashing
- MetaBoard PCB
  
### Initial Setup

#### 1. Flash the Base Firmware
- Erase the device memory completely
- Flash the `merged_domains.hex` file to your MetaBoard

### Performing OTA Updates

#### 2. Transfer Update Package
- Copy the `dfu_application.zip` file to your mobile device

#### 3. Connect & Update

- Open the nRF Connect app on your mobile device
- Scan and connect to your MetaBoard device
- Select the DFU option
- Choose the `dfu_application.zip` file
- Follow the on-screen instructions to complete the update

### Troubleshooting
If you encounter any issues during the update process:

- Ensure the device is sufficiently charged
- Check that you're using the correct update package
- Verify the device is in range and has a stable connection
- Restart the nRF Connect app and try again

### Demo Video
A demonstration video is available showing the complete update process using the nRF Connect app.

Note: Remember that for initial programming, the board should be powered through the programming header, not through the USB magnetic connector. When using the nRF Connect SDK, ensure you're building with version 2.4.2 as specified in the main firmware documentation.

