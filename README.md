# MetaBoard_launch
Specs and box opening instructions for MetaBoard users


**The MetaBoard**

![WhatsApp Image 2024-11-21 at 10 35 25 PM](https://github.com/user-attachments/assets/1c10d5c2-fe00-4740-94f0-c228e55fa3ba)


**The MetaBox**

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

1. Install dependencies:
```bash
pip install bleak python-osc numpy


