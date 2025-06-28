#!/usr/bin/env python3
"""
MetaBow Micro - Clean Sequoia-Aware BLE Management
Properly structured and working implementation - FIXED
"""

import sys
import asyncio
import time
import struct
import os
import threading
from datetime import datetime
import gc
import platform

# BLE imports
from bleak import BleakClient, BleakScanner

# OSC and file output
try:
    from pythonosc import udp_client
    OSC_AVAILABLE = True
except ImportError:
    OSC_AVAILABLE = False

# UI imports
import tkinter as tk
from tkinter import messagebox

# Global BLE loop
BLE_LOOP = None
BLE_THREAD = None

def setup_ble_loop():
    """Setup dedicated BLE event loop"""
    global BLE_LOOP, BLE_THREAD
    
    def run_loop():
        global BLE_LOOP
        BLE_LOOP = asyncio.new_event_loop()
        asyncio.set_event_loop(BLE_LOOP)
        try:
            BLE_LOOP.run_forever()
        except:
            pass
    
    if BLE_THREAD is None or not BLE_THREAD.is_alive():
        BLE_THREAD = threading.Thread(target=run_loop, daemon=True)
        BLE_THREAD.start()
        time.sleep(0.1)

def run_in_ble_loop(coro):
    """Run coroutine in BLE loop"""
    global BLE_LOOP
    if BLE_LOOP is None:
        setup_ble_loop()
    
    future = asyncio.run_coroutine_threadsafe(coro, BLE_LOOP)
    return future.result(timeout=30)

def detect_sequoia():
    """Detect if running macOS Sequoia"""
    try:
        if platform.system() == 'Darwin':
            version = platform.mac_ver()[0]
            if version:
                major = int(version.split('.')[0])
                return major >= 15, version
    except:
        pass
    return False, None

class BLEConnection:
    """Simple BLE connection with platform-aware settings"""
    
    def __init__(self, client, rx_char, tx_char, log_callback):
        self.client = client
        self.rx_char = rx_char
        self.tx_char = tx_char
        self.log = log_callback
        
        # Detect platform and apply settings
        self.is_sequoia, self.version = detect_sequoia()
        self._apply_platform_settings()
        
        # State
        self.message_count = 0
        self.last_data_time = time.time()
        self._active = True
        self.last_packet_time = 0
        
        # Data queue
        self.data_queue = []
        self.queue_lock = threading.Lock()
        
        # Stats
        self.total_messages = 0
        self.processed_messages = 0
        self.dropped_messages = 0
        
        # File output
        self.audio_recording_enabled = False
        self.binary_file = None
        
        # Setup
        self._setup_osc()
        self._start_processor()
        self._setup_downsampling()
        
        if self.is_sequoia:
            self.log(f"🔴 SEQUOIA {self.version} - Ultra-conservative mode")
        else:
            self.log(f"✅ Platform optimized settings applied")
    
    def _apply_platform_settings(self):
        """Apply platform-specific settings"""
        if self.is_sequoia:
            # Ultra-conservative for Sequoia
            self.throttle_delay = 0.005  # 5ms
            self.max_queue_size = 3
            self.drop_ratio = 8  # Keep 1 in 8
            self.batch_size = 1
            self.gc_interval = 1
        elif platform.system() == 'Darwin':
            # Standard macOS
            self.throttle_delay = 0.015  # 15ms
            self.max_queue_size = 10
            self.drop_ratio = 3
            self.batch_size = 3
            self.gc_interval = 3
        elif platform.system() == 'Windows':
            # Windows
            self.throttle_delay = 0.05  # 50ms
            self.max_queue_size = 5
            self.drop_ratio = 9
            self.batch_size = 2
            self.gc_interval = 2
        else:
            # Linux
            self.throttle_delay = 0.01  # 10ms
            self.max_queue_size = 20
            self.drop_ratio = 2
            self.batch_size = 5
            self.gc_interval = 5
    
    def _setup_osc(self):
        """Setup OSC client"""
        if OSC_AVAILABLE:
            try:
                self.osc = udp_client.SimpleUDPClient("127.0.0.1", 8888)
                self.log("OSC connected to port 8888")
            except Exception as e:
                self.log(f"OSC setup failed: {e}")
                self.osc = None
        else:
            self.osc = None
    
    def _setup_downsampling(self):
        """Setup message downsampling"""
        self.original_callback = self.data_callback
        self.message_counter = 0
        
        def downsampling_callback(sender, data):
            self.total_messages += 1
            self.message_counter += 1
            
            # Keep every nth message based on drop ratio
            if self.message_counter % self.drop_ratio == 0:
                self.processed_messages += 1
                return self.original_callback(sender, data)
            else:
                self.dropped_messages += 1
                return  # Drop message
        
        self.data_callback = downsampling_callback
        
        strategy = "ultra_aggressive" if self.is_sequoia else "aggressive"
        self.log(f"Downsampling: {strategy} (1 in {self.drop_ratio} messages)")
    
    def _start_processor(self):
        """Start background processor"""
        self.processor_active = True
        self.last_gc_time = time.time()
        
        def processor():
            while self.processor_active and self._active:
                try:
                    # Process queued data
                    data_to_process = []
                    with self.queue_lock:
                        if self.data_queue:
                            data_to_process = self.data_queue[:self.batch_size]
                            self.data_queue = self.data_queue[self.batch_size:]
                    
                    for data, timestamp in data_to_process:
                        self._process_packet(data, timestamp)
                    
                    # Periodic maintenance
                    current_time = time.time()
                    if current_time - self.last_gc_time > self.gc_interval:
                        gc.collect()
                        self.last_gc_time = current_time
                    
                    # Sleep
                    sleep_time = self.throttle_delay / 2
                    time.sleep(max(0.001, sleep_time))
                
                except Exception as e:
                    self.log(f"Processor error: {e}")
                    time.sleep(0.01)
        
        self.processor_thread = threading.Thread(target=processor, daemon=True)
        self.processor_thread.start()
    
    async def start(self):
        """Start BLE notifications"""
        try:
            await self.client.start_notify(self.tx_char, self.data_callback)
            await asyncio.sleep(1)
            self.log("BLE notifications started")
        except Exception as e:
            self.log(f"Failed to start notifications: {e}")
            raise
    
    def data_callback(self, sender, data):
        """BLE data callback with throttling"""
        if not self._active:
            return
        
        try:
            current_time = time.time()
            
            # Throttling
            if current_time - self.last_packet_time < self.throttle_delay:
                return  # Drop packet
            self.last_packet_time = current_time
            
            # Queue management
            with self.queue_lock:
                if len(self.data_queue) >= self.max_queue_size:
                    # Drop old packets
                    dropped = len(self.data_queue) // 2
                    self.data_queue = self.data_queue[dropped:]
                
                self.data_queue.append((data, current_time))
        
        except Exception as e:
            self.log(f"Callback error: {e}")
    
    def _process_packet(self, data, timestamp):
        """Process individual packet"""
        try:
            self.message_count += 1
            self.last_data_time = timestamp
            
            # Log progress
            if self.message_count % 1000 == 0:
                stats = self.get_stats()
                indicator = "🔴 " if self.is_sequoia else ""
                self.log(f"{indicator}Messages: {self.message_count}")
                self.log(f"  Drop rate: {stats['drop_rate']:.1f}%")
                self.log(f"  Effective: {stats['effective_rate']:.1f} msg/s")
            
            # Process data
            data_len = len(data)
            if data_len < 54:
                return
            
            # Audio recording
            if self.audio_recording_enabled and self.binary_file:
                try:
                    imu_data_len = 52
                    flag_size = 1
                    audio_end = data_len - imu_data_len - flag_size
                    if audio_end > 0:
                        audio_data = data[0:audio_end:2]
                        self.binary_file.write(audio_data)
                        
                        if self.message_count % 200 == 0:
                            self.binary_file.flush()
                except Exception as e:
                    self.log(f"File write error: {e}")
            
            # Motion data (IMU)
            if data_len > 0 and data[-1] == 1:
                try:
                    imu_data_len = 52
                    flag_size = 1
                    motion_start = data_len - imu_data_len - flag_size
                    
                    if motion_start >= 0:
                        motion_data = data[motion_start:motion_start + imu_data_len]
                        if len(motion_data) == imu_data_len:
                            motion_floats = list(struct.unpack('13f', motion_data))
                            if self.osc:
                                self.osc.send_message("/motion", motion_floats)
                except:
                    pass  # Ignore motion processing errors
        
        except Exception as e:
            self.log(f"Packet processing error: {e}")
    
    def setup_file_output(self):
        """Setup audio recording"""
        try:
            if platform.system() == "Darwin":
                output_dir = os.path.expanduser("~/Documents/MetaBow_Data")
            else:
                output_dir = os.path.expanduser("~/MetaBow_Data")
            
            os.makedirs(output_dir, exist_ok=True)
            filename = f'pcm_{int(time.time())}.bin'
            filepath = os.path.join(output_dir, filename)
            
            self.binary_file = open(filepath, 'wb', buffering=8192)
            self.log(f"Recording started: {filename}")
        except Exception as e:
            self.log(f"Recording setup failed: {e}")
            self.binary_file = None
    
    def stop_file_output(self):
        """Stop audio recording"""
        if self.binary_file:
            try:
                self.binary_file.flush()
                self.binary_file.close()
                self.log("Recording stopped")
            except:
                pass
            self.binary_file = None
    
    def toggle_audio_recording(self):
        """Toggle audio recording"""
        self.audio_recording_enabled = not self.audio_recording_enabled
        if self.audio_recording_enabled:
            self.setup_file_output()
        else:
            self.stop_file_output()
    
    async def soft_reset(self):
        """Gentle reset without disconnecting"""
        try:
            if not self.client or not self.client.is_connected:
                return "Error: Not connected"
            
            indicator = "🔴 " if self.is_sequoia else ""
            self.log(f"{indicator}Performing gentle reset...")
            
            # Clear queue
            with self.queue_lock:
                dropped = len(self.data_queue)
                self.data_queue.clear()
                if dropped > 0:
                    self.log(f"Cleared {dropped} queued packets")
            
            # Reset counters
            old_count = self.message_count
            self.message_count = 0
            self.last_data_time = time.time()
            
            # Reset downsampling counters
            self.total_messages = 0
            self.processed_messages = 0
            self.dropped_messages = 0
            self.message_counter = 0
            
            gc.collect()
            
            self.log(f"{indicator}Reset complete (was {old_count} messages)")
            return "Success"
        
        except Exception as e:
            self.log(f"Reset failed: {e}")
            return f"Error: {e}"
    
    async def send_reset_command(self):
        """Send reset command to device"""
        try:
            if not self.client or not self.client.is_connected:
                return "Error: Not connected"
            
            commands = [b'RESET\n', b'RST\n', b'\x00\x01', b'R']
            
            for cmd in commands:
                try:
                    await self.client.write_gatt_char(self.rx_char, cmd)
                    self.log(f"Sent reset command: {cmd}")
                    await asyncio.sleep(0.1)
                except:
                    continue
            
            return "Success"
        
        except Exception as e:
            return f"Error: {e}"
    
    def get_stats(self):
        """Get connection statistics"""
        if self.total_messages > 0:
            drop_rate = (self.dropped_messages / self.total_messages) * 100
            effective_rate = (self.processed_messages / self.total_messages) * 170  # 170 = input rate
        else:
            drop_rate = 0
            effective_rate = 0
        
        strategy = "ultra_aggressive" if self.is_sequoia else "aggressive"
        
        return {
            'total': self.total_messages,
            'processed': self.processed_messages,
            'dropped': self.dropped_messages,
            'drop_rate': drop_rate,
            'effective_rate': effective_rate,
            'strategy': strategy,
            'message_count': self.message_count,
            'queue_size': len(self.data_queue),
            'throttle_ms': self.throttle_delay * 1000,
            'time_since_data': time.time() - self.last_data_time
        }
    
    def close(self):
        """Clean shutdown"""
        self._active = False
        self.processor_active = False
        
        if hasattr(self, 'processor_thread') and self.processor_thread.is_alive():
            self.processor_thread.join(timeout=1)
        
        with self.queue_lock:
            self.data_queue.clear()
        
        if self.binary_file:
            try:
                self.binary_file.flush()
                self.binary_file.close()
            except:
                pass
        
        gc.collect()

class MetaBowApp:
    """Main MetaBow application"""
    
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("MetaBow - Sequoia-Aware BLE")
        self.root.geometry("550x650")
        
        # Detect Sequoia
        self.is_sequoia, self.version = detect_sequoia()
        
        # State
        self.client = None
        self.connection = None
        self.is_connected = False
        self.devices = []
        self.selected_device = None
        self._shutting_down = False
        self.reset_in_progress = False
        
        # Settings based on platform
        if self.is_sequoia:
            self.reset_thresholds = [500, 1000, 2000]
            self.stall_timeout = 4
            self.connection_timeout = 40.0
        elif platform.system() == 'Darwin':
            self.reset_thresholds = [2000, 5000, 10000]
            self.stall_timeout = 3
            self.connection_timeout = 25.0
        else:
            self.reset_thresholds = [1500, 4000, 8000]
            self.stall_timeout = 4
            self.connection_timeout = 30.0
        
        self.current_threshold_index = 0
        self.last_reset_count = 0
        self.connection_start_time = None
        
        # Device names to scan for
        self.device_names = ["metabow", "metabow_ota"]
        
        # Setup
        setup_ble_loop()
        self.create_ui()
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.update_status()
    
    def create_ui(self):
        """Create user interface"""
        # Title
        title = "MetaBow - Sequoia-Aware BLE"
        if self.is_sequoia:
            title += " 🔴"
        tk.Label(self.root, text=title, font=("Arial", 16, "bold")).pack(pady=10)
        
        # Platform info
        if self.is_sequoia:
            platform_text = f"🔴 SEQUOIA {self.version} - Ultra-conservative mode"
            platform_color = "red"
        else:
            platform_text = f"✅ {platform.system()} - Optimized settings"
            platform_color = "blue"
        
        tk.Label(self.root, text=platform_text, font=("Arial", 10), fg=platform_color).pack()
        
        # OSC status
        osc_text = "OSC: Available" if OSC_AVAILABLE else "OSC: Disabled"
        osc_color = "green" if OSC_AVAILABLE else "orange"
        tk.Label(self.root, text=osc_text, font=("Arial", 8), fg=osc_color).pack()
        
        # Status
        self.status_label = tk.Label(self.root, text="Status: Ready", 
                                   fg="red" if self.is_sequoia else "blue", 
                                   font=("Arial", 12))
        self.status_label.pack(pady=10)
        
        # Stats frame
        stats_frame = tk.Frame(self.root, relief=tk.RIDGE, bd=1)
        stats_frame.pack(pady=10, padx=20, fill=tk.X)
        
        tk.Label(stats_frame, text="Connection Statistics", font=("Arial", 10, "bold")).pack()
        
        self.timer_label = tk.Label(stats_frame, text="Time: --:--:--", font=("Arial", 9))
        self.timer_label.pack()
        
        self.messages_label = tk.Label(stats_frame, text="Messages: 0", font=("Arial", 9))
        self.messages_label.pack()
        
        self.strategy_label = tk.Label(stats_frame, text="Strategy: None", font=("Arial", 9))
        self.strategy_label.pack()
        
        self.droprate_label = tk.Label(stats_frame, text="Drop Rate: 0%", font=("Arial", 9))
        self.droprate_label.pack()
        
        self.effective_label = tk.Label(stats_frame, text="Effective: 0 msg/s", font=("Arial", 9))
        self.effective_label.pack()
        
        self.queue_label = tk.Label(stats_frame, text="Queue: 0", font=("Arial", 9))
        self.queue_label.pack()
        
        self.reset_label = tk.Label(stats_frame, text=f"Next Reset: {self.reset_thresholds[0]}", font=("Arial", 9))
        self.reset_label.pack()
        
        # Controls frame
        controls_frame = tk.Frame(self.root)
        controls_frame.pack(pady=10)
        
        # Audio recording
        self.audio_var = tk.BooleanVar(value=False)
        self.audio_check = tk.Checkbutton(controls_frame, text="Record Audio", 
                                        variable=self.audio_var, command=self.toggle_audio)
        self.audio_check.pack()
        
        # Reset buttons
        button_frame = tk.Frame(controls_frame)
        button_frame.pack(pady=5)
        
        self.gentle_reset_btn = tk.Button(button_frame, text="Gentle Reset", 
                                        command=self.gentle_reset, state=tk.DISABLED,
                                        bg="#28a745", fg="white")
        self.gentle_reset_btn.pack(side=tk.LEFT, padx=2)
        
        self.cmd_reset_btn = tk.Button(button_frame, text="Send Reset", 
                                     command=self.send_reset, state=tk.DISABLED,
                                     bg="#6f42c1", fg="white")
        self.cmd_reset_btn.pack(side=tk.LEFT, padx=2)
        
        self.emergency_btn = tk.Button(button_frame, text="Emergency", 
                                     command=self.emergency_reset, state=tk.DISABLED,
                                     bg="#dc3545", fg="white")
        self.emergency_btn.pack(side=tk.LEFT, padx=2)
        
        # Scan button
        scan_color = "#6610f2" if self.is_sequoia else "#007bff"
        tk.Button(self.root, text="Scan for Devices", command=self.scan_devices,
                 bg=scan_color, fg="white", font=("Arial", 12)).pack(pady=10)
        
        # Device list
        tk.Label(self.root, text="Devices:", font=("Arial", 10, "bold")).pack()
        self.device_listbox = tk.Listbox(self.root, height=3)
        self.device_listbox.pack(pady=5, fill=tk.X, padx=20)
        self.device_listbox.bind('<<ListboxSelect>>', self.on_device_select)
        
        # Connection buttons
        conn_frame = tk.Frame(self.root)
        conn_frame.pack(pady=10)
        
        connect_color = "#dc3545" if self.is_sequoia else "#28a745"
        self.connect_btn = tk.Button(conn_frame, text="Connect", 
                                   command=self.connect_device, state=tk.DISABLED,
                                   bg=connect_color, fg="white", font=("Arial", 11))
        self.connect_btn.pack(side=tk.LEFT, padx=5)
        
        self.disconnect_btn = tk.Button(conn_frame, text="Disconnect", 
                                      command=self.disconnect_device, state=tk.DISABLED,
                                      bg="#dc3545", fg="white", font=("Arial", 11))
        self.disconnect_btn.pack(side=tk.LEFT, padx=5)
        
        # Log area
        tk.Label(self.root, text="Log:", font=("Arial", 10, "bold")).pack(pady=(15, 0))
        self.log_text = tk.Text(self.root, height=6, font=("Courier", 8), wrap=tk.WORD)
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=20, pady=5)
        
        # Initial log messages
        self.log("MetaBow Sequoia-Aware BLE Management ready!")
        if self.is_sequoia:
            self.log(f"🔴 SEQUOIA {self.version} detected - Ultra-conservative mode")
            self.log(f"   Reset thresholds: {self.reset_thresholds}")
        else:
            self.log(f"✅ {platform.system()} optimized settings")
        
        if OSC_AVAILABLE:
            self.log("OSC enabled - motion data sent to port 8888")
        self.log("Ready to scan for MetaBow devices")
    
    def log(self, message):
        """Add log message"""
        def add_log():
            timestamp = datetime.now().strftime("%H:%M:%S")
            log_entry = f"[{timestamp}] {message}\n"
            try:
                self.log_text.insert(tk.END, log_entry)
                self.log_text.see(tk.END)
                self.root.update_idletasks()
            except:
                pass
            print(log_entry.strip())
        
        if threading.current_thread() == threading.main_thread():
            add_log()
        else:
            try:
                self.root.after(0, add_log)
            except:
                pass
    
    def scan_devices(self):
        """Scan for BLE devices"""
        indicator = "🔴 " if self.is_sequoia else ""
        self.log(f"{indicator}Scanning for MetaBow devices...")
        
        self.device_listbox.delete(0, tk.END)
        self.device_listbox.insert(tk.END, "Scanning...")
        
        def run_scan():
            try:
                devices = run_in_ble_loop(self.async_scan())
                self.root.after(0, self.update_device_list, devices)
            except Exception as e:
                error = "Scan timeout" if "timeout" in str(e).lower() else str(e)
                self.root.after(0, self.log, f"Scan failed: {error}")
                self.root.after(0, self.update_device_list, [])
        
        threading.Thread(target=run_scan, daemon=True).start()
    
    async def async_scan(self):
        """Async device scan"""
        timeout = 15.0 if self.is_sequoia else 10.0
        devices = await BleakScanner.discover(timeout=timeout)
        
        metabow_devices = []
        for device in devices:
            if device.name:
                name_lower = device.name.lower()
                for target in self.device_names:
                    if target.lower() in name_lower:
                        metabow_devices.append(device)
                        break
        
        return metabow_devices
    
    def update_device_list(self, devices):
        """Update device list"""
        self.devices = devices
        self.device_listbox.delete(0, tk.END)
        
        if devices:
            for device in devices:
                display = f"{device.name} ({device.address})"
                self.device_listbox.insert(tk.END, display)
            
            indicator = "🔴 " if self.is_sequoia else ""
            self.log(f"{indicator}Found {len(devices)} MetaBow device(s)")
        else:
            self.device_listbox.insert(tk.END, "No MetaBow devices found")
            self.log("No devices found")
    
    def on_device_select(self, event):
        """Handle device selection"""
        selection = self.device_listbox.curselection()
        if selection and self.devices and selection[0] < len(self.devices):
            self.selected_device = self.devices[selection[0]]
            self.connect_btn.config(state=tk.NORMAL if not self.is_connected else tk.DISABLED)
            self.log(f"Selected: {self.selected_device.name}")
    
    def connect_device(self):
        """Connect to selected device"""
        if not self.selected_device:
            messagebox.showwarning("No Device", "Please select a device")
            return
        
        indicator = "🔴 " if self.is_sequoia else ""
        self.log(f"{indicator}Connecting to {self.selected_device.name}...")
        
        self.connect_btn.config(state=tk.DISABLED)
        self.status_label.config(text="Status: Connecting...", fg="orange")
        
        def run_connect():
            try:
                result = run_in_ble_loop(self.async_connect())
                self.root.after(0, self.handle_connect_result, result)
            except Exception as e:
                self.root.after(0, self.handle_connect_result, f"Error: {e}")
        
        threading.Thread(target=run_connect, daemon=True).start()
    
    async def async_connect(self):
        """Async connection"""
        try:
            # Cleanup existing client
            if self.client:
                try:
                    if self.client.is_connected:
                        await self.client.disconnect()
                except:
                    pass
                self.client = None
            
            gc.collect()
            
            # Create new client
            self.client = BleakClient(self.selected_device.address, timeout=self.connection_timeout)
            await self.client.connect()
            await asyncio.sleep(1)
            
            if not self.client.is_connected:
                return "Failed to establish connection"
            
            # Get services
            try:
                services = await self.client.get_services()
            except:
                services = self.client.services
            
            # Find UART service
            uart_service = None
            uart_uuid = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
            
            for service in services:
                if str(service.uuid).lower() == uart_uuid.lower():
                    uart_service = service
                    break
            
            if not uart_service:
                return "UART service not found"
            
            # Find characteristics
            tx_char = None
            rx_char = None
            tx_uuid = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
            rx_uuid = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
            
            for char in uart_service.characteristics:
                char_uuid = str(char.uuid).lower()
                if char_uuid == tx_uuid.lower():
                    tx_char = char
                elif char_uuid == rx_uuid.lower():
                    rx_char = char
            
            if not tx_char or not rx_char:
                return "Required characteristics not found"
            
            # Create connection
            self.connection = BLEConnection(self.client, rx_char, tx_char, self.log)
            await self.connection.start()
            
            return "Success"
        
        except Exception as e:
            if "timeout" in str(e).lower():
                return "Connection timeout"
            return str(e)
    
    def handle_connect_result(self, result):
        """Handle connection result"""
        if result == "Success":
            self.is_connected = True
            self.connection_start_time = time.time()
            
            status_color = "red" if self.is_sequoia else "green"
            self.status_label.config(text="Status: Connected", fg=status_color)
            self.connect_btn.config(state=tk.DISABLED)
            self.disconnect_btn.config(state=tk.NORMAL)
            
            # Enable reset buttons
            self.gentle_reset_btn.config(state=tk.NORMAL)
            self.cmd_reset_btn.config(state=tk.NORMAL)
            self.emergency_btn.config(state=tk.NORMAL)
            
            indicator = "🔴 " if self.is_sequoia else ""
            self.log(f"{indicator}Connected successfully!")
            
            if self.connection:
                stats = self.connection.get_stats()
                self.log(f"Strategy: {stats['strategy']}")
            
            self.start_monitoring()
        else:
            self.is_connected = False
            self.status_label.config(text="Status: Failed", fg="red")
            self.connect_btn.config(state=tk.NORMAL if self.selected_device else tk.DISABLED)
            
            indicator = "🔴 " if self.is_sequoia else ""
            self.log(f"{indicator}Connection failed: {result}")
    
    def disconnect_device(self):
        """Disconnect from device"""
        indicator = "🔴 " if self.is_sequoia else ""
        self.log(f"{indicator}Disconnecting...")
        
        self.is_connected = False
        
        def run_disconnect():
            try:
                if self.connection:
                    self.connection.close()
                
                if self.client:
                    try:
                        if self.client.is_connected:
                            run_in_ble_loop(self.client.disconnect())
                    except Exception as e:
                        self.root.after(0, self.log, f"Disconnect warning: {e}")
                
                self.connection = None
                self.client = None
                self.root.after(0, self.handle_disconnect)
            except Exception as e:
                self.root.after(0, self.log, f"Disconnect error: {e}")
                self.root.after(0, self.handle_disconnect)
        
        threading.Thread(target=run_disconnect, daemon=True).start()
    
    def handle_disconnect(self):
        """Handle disconnection cleanup"""
        if self._shutting_down:
            return
        
        self.is_connected = False
        self.connection_start_time = None
        
        self.status_label.config(text="Status: Disconnected", fg="red")
        self.connect_btn.config(state=tk.NORMAL if self.selected_device else tk.DISABLED)
        self.disconnect_btn.config(state=tk.DISABLED)
        
        # Disable reset buttons
        self.gentle_reset_btn.config(state=tk.DISABLED)
        self.cmd_reset_btn.config(state=tk.DISABLED)
        self.emergency_btn.config(state=tk.DISABLED)
        
        indicator = "🔴 " if self.is_sequoia else ""
        self.log(f"{indicator}Disconnected")
    
    def start_monitoring(self):
        """Start connection monitoring"""
        def monitor():
            try:
                while self.is_connected and not self._shutting_down:
                    if self.reset_in_progress:
                        time.sleep(0.5)
                        continue
                    
                    if self.client and not self.client.is_connected:
                        if not self.reset_in_progress:
                            self.log("BLE client disconnected")
                            break
                        else:
                            time.sleep(0.5)
                            continue
                    
                    if self.connection:
                        stats = self.connection.get_stats()
                        message_count = stats['message_count']
                        time_since_data = stats['time_since_data']
                        
                        # Check for reset threshold
                        current_threshold = self.get_current_threshold()
                        if (current_threshold and 
                            message_count >= current_threshold and 
                            message_count > self.last_reset_count + 50 and
                            not self.reset_in_progress):
                            
                            self.last_reset_count = message_count
                            self.current_threshold_index += 1
                            
                            indicator = "🔴 " if self.is_sequoia else ""
                            self.root.after(0, self.log, f"{indicator}Preventive reset at {message_count} messages")
                            self.root.after(0, self.gentle_reset_and_continue)
                            # Wait for reset to complete, then continue monitoring
                            while self.reset_in_progress and self.is_connected:
                                time.sleep(0.5)
                            continue  # Continue monitoring instead of breaking
                        
                        # Check for data stall
                        if (time_since_data > self.stall_timeout and 
                            message_count > 0 and 
                            not self.reset_in_progress):
                            
                            indicator = "🔴 " if self.is_sequoia else ""
                            self.root.after(0, self.log, f"{indicator}Data stall detected - gentle reset")
                            self.root.after(0, self.gentle_reset_and_continue)
                            # Wait for reset to complete, then continue monitoring
                            while self.reset_in_progress and self.is_connected:
                                time.sleep(0.5)
                            continue  # Continue monitoring instead of breaking
                    
                    sleep_time = 0.2 if self.is_sequoia else 0.3
                    time.sleep(sleep_time)
                
                # Connection lost
                if (self.is_connected and not self._shutting_down and not self.reset_in_progress):
                    self.root.after(0, self.handle_disconnect)
            
            except Exception as e:
                print(f"Monitor error: {e}")
                # Restart monitoring if there was an error but we're still connected
                if self.is_connected and not self._shutting_down:
                    time.sleep(1)
                    self.start_monitoring()
        
        threading.Thread(target=monitor, daemon=True).start()
    
    def get_current_threshold(self):
        """Get current reset threshold"""
        if self.current_threshold_index < len(self.reset_thresholds):
            return self.reset_thresholds[self.current_threshold_index]
        return None
    
    def gentle_reset(self):
        """Perform gentle reset"""
        if not self.is_connected or not self.connection:
            return
        
        self.reset_in_progress = True
        
        indicator = "🔴 " if self.is_sequoia else ""
        self.log(f"{indicator}Performing gentle reset...")
        
        def run_reset():
            try:
                result = run_in_ble_loop(self.connection.soft_reset())
                self.root.after(0, self.handle_reset_result, result)
            except Exception as e:
                self.root.after(0, self.handle_reset_result, f"Error: {e}")
        
        threading.Thread(target=run_reset, daemon=True).start()
    
    def gentle_reset_and_continue(self):
        """Perform gentle reset without stopping monitoring"""
        if not self.is_connected or not self.connection:
            return
        
        self.reset_in_progress = True
        
        indicator = "🔴 " if self.is_sequoia else ""
        self.log(f"{indicator}Performing gentle reset...")
        
        def run_reset():
            try:
                result = run_in_ble_loop(self.connection.soft_reset())
                self.root.after(0, self.handle_reset_result_and_continue, result)
            except Exception as e:
                self.root.after(0, self.handle_reset_result_and_continue, f"Error: {e}")
        
        threading.Thread(target=run_reset, daemon=True).start()
    
    def send_reset(self):
        """Send reset command to device"""
        if not self.is_connected or not self.connection:
            return
        
        self.log("Sending reset command to device...")
        
        def run_cmd_reset():
            try:
                result = run_in_ble_loop(self.connection.send_reset_command())
                self.root.after(0, self.handle_cmd_reset_result, result)
            except Exception as e:
                self.root.after(0, self.handle_cmd_reset_result, f"Error: {e}")
        
        threading.Thread(target=run_cmd_reset, daemon=True).start()
    
    def handle_reset_result(self, result):
        """Handle gentle reset result"""
        self.reset_in_progress = False
        
        indicator = "🔴 " if self.is_sequoia else ""
        
        if result == "Success":
            self.log(f"{indicator}Gentle reset successful!")
            self.last_reset_count = 0
        else:
            self.log(f"{indicator}Gentle reset failed: {result}")
    
    def handle_reset_result_and_continue(self, result):
        """Handle gentle reset result and ensure monitoring continues"""
        self.reset_in_progress = False
        
        indicator = "🔴 " if self.is_sequoia else ""
        
        if result == "Success":
            self.log(f"{indicator}Gentle reset successful! Continuing...")
            self.last_reset_count = 0
        else:
            self.log(f"{indicator}Gentle reset failed: {result}")
            # If reset failed, we might need to check connection status
            if self.client and not self.client.is_connected:
                self.handle_disconnect()
                return
        
        # Reset completed, monitoring will continue automatically
    
    def handle_cmd_reset_result(self, result):
        """Handle command reset result"""
        if result == "Success":
            self.log("Reset command sent successfully")
        else:
            self.log(f"Reset command failed: {result}")
    
    def emergency_reset(self):
        """Perform emergency reset with reconnection"""
        if not self.is_connected:
            return
        
        indicator = "🔴 " if self.is_sequoia else ""
        self.log(f"{indicator}EMERGENCY RESET - reconnecting...")
        
        def run_emergency():
            try:
                # Force disconnect
                if self.connection:
                    self.connection.close()
                
                if self.client:
                    try:
                        if self.client.is_connected:
                            run_in_ble_loop(self.client.disconnect())
                    except:
                        pass
                
                self.connection = None
                self.client = None
                
                # Wait for recovery
                wait_time = 8 if self.is_sequoia else 5
                self.root.after(0, self.log, f"Waiting {wait_time}s for device recovery...")
                time.sleep(wait_time)
                
                gc.collect()
                
                # Reset counters
                self.current_threshold_index = 0
                self.last_reset_count = 0
                
                # Reconnect
                if self.selected_device:
                    result = run_in_ble_loop(self.async_connect())
                    self.root.after(0, self.handle_emergency_result, result)
                else:
                    self.root.after(0, self.handle_disconnect)
            
            except Exception as e:
                self.root.after(0, self.log, f"Emergency reset error: {e}")
                self.root.after(0, self.handle_disconnect)
        
        threading.Thread(target=run_emergency, daemon=True).start()
    
    def handle_emergency_result(self, result):
        """Handle emergency reset result"""
        indicator = "🔴 " if self.is_sequoia else ""
        
        if result == "Success":
            self.log(f"{indicator}Emergency reset successful!")
            self.handle_connect_result(result)
        else:
            self.log(f"{indicator}Emergency reset failed: {result}")
            self.handle_disconnect()
    
    def toggle_audio(self):
        """Toggle audio recording - FIXED METHOD"""
        if not self.is_connected or not self.connection:
            messagebox.showwarning("Not Connected", "Please connect to a device first")
            self.audio_var.set(False)  # Reset checkbox
            return
        
        try:
            enabled = self.audio_var.get()
            self.connection.toggle_audio_recording()
            
            if enabled:
                self.log("Audio recording enabled")
            else:
                self.log("Audio recording disabled")
        except Exception as e:
            self.log(f"Audio toggle error: {e}")
            self.audio_var.set(False)  # Reset checkbox on error
    
    def update_status(self):
        """Update status display"""
        if self._shutting_down:
            return
        
        try:
            # Connection timer
            if self.connection_start_time and self.is_connected:
                elapsed = time.time() - self.connection_start_time
                timer_text = f"Time: {self.format_duration(elapsed)}"
                timer_color = "red" if self.is_sequoia else "green"
                self.timer_label.config(text=timer_text, fg=timer_color)
            else:
                self.timer_label.config(text="Time: --:--:--", fg="black")
            
            # Connection stats
            if self.is_connected and self.connection:
                try:
                    if self.reset_in_progress:
                        indicator = "🔴 " if self.is_sequoia else ""
                        self.messages_label.config(text=f"Messages: {indicator}RESETTING...")
                        self.strategy_label.config(text="Strategy: RESETTING...")
                        self.droprate_label.config(text="Drop Rate: RESETTING...")
                        self.effective_label.config(text="Effective: RESETTING...")
                        self.queue_label.config(text="Queue: RESETTING...")
                        self.reset_label.config(text="Reset in progress...")
                        
                        self.root.after(200, self.update_status)
                        return
                    
                    # Get current stats
                    stats = self.connection.get_stats()
                    
                    indicator = "🔴 " if self.is_sequoia else ""
                    
                    self.messages_label.config(text=f"Messages: {indicator}{stats['message_count']}")
                    
                    strategy_text = f"Strategy: {stats['strategy']}"
                    if self.is_sequoia:
                        strategy_text = f"🔴 {strategy_text}"
                    self.strategy_label.config(text=strategy_text)
                    
                    drop_text = f"Drop Rate: {stats['drop_rate']:.1f}%"
                    if self.is_sequoia and stats['drop_rate'] > 80:
                        drop_text = f"🔴 {drop_text}"
                    self.droprate_label.config(text=drop_text)
                    
                    effective_text = f"Effective: {stats['effective_rate']:.1f} msg/s"
                    if self.is_sequoia:
                        effective_text = f"🔴 {effective_text}"
                    self.effective_label.config(text=effective_text)
                    
                    self.queue_label.config(text=f"Queue: {stats['queue_size']}")
                    
                    # Reset countdown
                    current_threshold = self.get_current_threshold()
                    if current_threshold and stats['message_count'] > 0:
                        remaining = max(0, current_threshold - stats['message_count'])
                        reset_text = f"Next Reset: {remaining} msgs"
                    else:
                        reset_text = f"Next Reset: {self.reset_thresholds[0]}"
                    
                    if self.is_sequoia:
                        reset_text = f"🔴 {reset_text}"
                    self.reset_label.config(text=reset_text)
                
                except Exception as e:
                    if not self.reset_in_progress:
                        print(f"Status update error: {e}")
            else:
                # Not connected
                self.messages_label.config(text="Messages: 0")
                self.strategy_label.config(text="Strategy: Not connected")
                self.droprate_label.config(text="Drop Rate: 0%")
                self.effective_label.config(text="Effective: 0 msg/s")
                self.queue_label.config(text="Queue: 0")
                
                reset_text = f"Next Reset: {self.reset_thresholds[0]}"
                if self.is_sequoia:
                    reset_text = f"🔴 {reset_text}"
                self.reset_label.config(text=reset_text)
            
            self.root.update_idletasks()
            self.root.after(500, self.update_status)
        
        except Exception as e:
            print(f"Critical status update error: {e}")
            self.root.after(1000, self.update_status)
    
    def format_duration(self, seconds):
        """Format duration as HH:MM:SS"""
        hours = int(seconds // 3600)
        minutes = int((seconds % 3600) // 60)
        secs = int(seconds % 60)
        return f"{hours:02d}:{minutes:02d}:{secs:02d}"
    
    def on_closing(self):
        """Handle application closing"""
        self._shutting_down = True
        
        indicator = "🔴 " if self.is_sequoia else ""
        self.log(f"{indicator}Shutting down...")
        
        if self.is_connected:
            self.disconnect_device()
            time.sleep(0.5)
        
        self.root.destroy()
    
    def run(self):
        """Run the application"""
        self.root.mainloop()


def check_dependencies():
    """Check required dependencies"""
    print("MetaBow - Sequoia-Aware BLE Management")
    print(f"Python: {sys.version.split()[0]}")
    print(f"Platform: {platform.system()}")
    
    # Sequoia detection
    is_sequoia, version = detect_sequoia()
    if is_sequoia:
        print(f"🔴 SEQUOIA DETECTED: macOS {version}")
        print("   Ultra-conservative BLE settings will be applied")
    elif platform.system() == 'Darwin':
        print(f"✅ macOS {version} - Standard BLE settings")
    
    # Check bleak
    try:
        import bleak
        version = getattr(bleak, '__version__', 'unknown')
        print(f"Bleak: {version}")
    except ImportError:
        print("ERROR: bleak not installed")
        print("Install: pip3 install bleak")
        return False
    
    # Check python-osc
    global OSC_AVAILABLE
    try:
        import pythonosc
        print("python-osc: available")
        OSC_AVAILABLE = True
    except ImportError:
        print("python-osc: not available (OSC disabled)")
        OSC_AVAILABLE = False
    
    return True


if __name__ == "__main__":
    if not check_dependencies():
        sys.exit(1)
    
    print("\nStarting MetaBow Sequoia-aware BLE management...")
    print("Features:")
    print("- Automatic Sequoia detection and ultra-conservative settings")
    print("- Platform-aware throttling and downsampling") 
    print("- Smart reset strategies")
    print("- Real-time statistics")
    
    app = MetaBowApp()
    app.run()