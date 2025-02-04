#!/usr/bin/env python3

import asyncio
import struct
import tkinter as tk
from tkinter import ttk, simpledialog, filedialog
from tkinter.messagebox import showerror, askyesno, showinfo
from pythonosc import udp_client
from bleak import BleakScanner, BleakClient
from bleak.exc import BleakError
import wave
import time
from datetime import datetime
import os
import numpy as np
from dataclasses import dataclass, field
from typing import List, Any
import time

@dataclass
class OSCRouteTemplate:
    path: str
    data_type: str
    last_seen: float = field(default_factory=time.time)
    sample_value: Any = None

@dataclass
class OSCRoute:
    path: str
    data_type: str
    enabled: bool = True

class OSCRouteManager:
    """Manages discovered OSC routes"""
    def __init__(self):
        self.discovered_routes = {}  # path -> OSCRouteTemplate
        self.discovery_callbacks = []  # Callbacks for when new routes are discovered

    def register_discovery_callback(self, callback):
        """Register a callback to be notified when new routes are discovered"""
        self.discovery_callbacks.append(callback)

    def update_route(self, path: str, data_type: str, sample_value: Any = None):
        """Update or add a route based on received data"""
        if path not in self.discovered_routes:
            self.discovered_routes[path] = OSCRouteTemplate(path, data_type, sample_value=sample_value)
            # Notify callbacks of new route
            for callback in self.discovery_callbacks:
                callback(path, data_type)
        else:
            self.discovered_routes[path].last_seen = time.time()
            self.discovered_routes[path].sample_value = sample_value

    def get_available_routes(self):
        """Get list of discovered routes"""
        return list(self.discovered_routes.values())

class OSCDestination:
    def __init__(self, port):
        self.port = port
        self.name = f"Local Port {port}"
        self.client = udp_client.SimpleUDPClient("127.0.0.1", port)
        self.routes = []  # Active routes for this destination

    def add_route(self, template: OSCRouteTemplate):
        """Add a route from a template"""
        route = OSCRoute(template.path, template.data_type)
        # Check if route already exists
        if not any(r.path == route.path for r in self.routes):
            self.routes.append(route)
            return True
        return False

    def remove_route(self, index: int):
        """Remove a route by index"""
        if 0 <= index < len(self.routes):
            self.routes.pop(index)

    def toggle_route(self, index: int):
        """Toggle route enabled state"""
        if 0 <= index < len(self.routes):
            self.routes[index].enabled = not self.routes[index].enabled

class Window:
    def __init__(self, loop):
        self.root = tk.Tk()
        self.root.title("Metabow OSC Bridge")
        self.root.geometry("800x600")
        self.root.protocol("WM_DELETE_WINDOW", self.on_exit)
        self.loop = loop

        # Initialize managers and recorders
        self.route_manager = OSCRouteManager()
        self.audio_recorder = AudioRecorder(loop)
        self.osc_destinations = []

        # Initialize state variables
        self.is_destroyed = False
        self.IMU_devices = {}
        self.selected_devices = []
        self.device_name = "metabow"
        self.clients = []
        self.scanner = None

        # Create UI components first
        self.create_main_frames()

        # Bind selection events
        self.bind_selection_events()

        # Start monitoring after UI is created
        self.start_route_monitoring()
        self.start_level_monitoring()
        self.update_latency_display()

    def bind_selection_events(self):
        """Bind selection events for route management"""
        self.dest_listbox.bind('<<ListboxSelect>>', self.on_destination_select)
        self.available_routes_listbox.bind('<<ListboxSelect>>', self.on_available_route_select)
        self.route_listbox.bind('<<ListboxSelect>>', self.on_active_route_select)

    # Main section creation methods
    def create_main_frames(self):
        self.main_frame = ttk.Frame(self.root)
        self.main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        self.devices_frame = ttk.LabelFrame(self.main_frame, text="Bluetooth Devices")
        self.devices_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.create_devices_section()

        self.routing_frame = ttk.LabelFrame(self.main_frame, text="OSC Routing")
        self.routing_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.create_routing_section()

        self.audio_frame = ttk.LabelFrame(self.root, text="Audio Controls")
        self.audio_frame.pack(fill=tk.X, padx=10, pady=5)
        self.create_audio_section()

        self.logs_frame = ttk.LabelFrame(self.root, text="Logs")
        self.logs_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        self.create_logs_section()

    def create_logs_section(self):
        """Creates the logging section of the UI"""
        # Create log text widget with scrollbar
        log_frame = ttk.Frame(self.logs_frame)
        log_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Add vertical scrollbar
        scrollbar = ttk.Scrollbar(log_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Create text widget for logs
        self.log_text = tk.Text(log_frame, height=8, wrap=tk.WORD, 
                               yscrollcommand=scrollbar.set)
        self.log_text.pack(fill=tk.BOTH, expand=True)
        
        # Configure scrollbar to work with text widget
        scrollbar.config(command=self.log_text.yview)
        
        # Add timestamp for initialization
        self.log_message("Application started")

    def create_audio_section(self):
        """Creates the audio control section of the UI"""
        # Create main controls frame
        controls_frame = ttk.Frame(self.audio_frame)
        controls_frame.pack(fill=tk.X, padx=5, pady=5)

        # Recording controls
        record_frame = ttk.LabelFrame(controls_frame, text="Recording")
        record_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

        self.record_button = ttk.Button(record_frame, text="Start Recording",
                                      command=self.toggle_recording, state=tk.DISABLED)
        self.record_button.pack(side=tk.LEFT, padx=5, pady=5)

        self.recording_label = ttk.Label(record_frame, text="Not Recording")
        self.recording_label.pack(side=tk.LEFT, padx=5, pady=5)

        # Audio processing controls
        processing_frame = ttk.LabelFrame(controls_frame, text="Processing")
        processing_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

        # Gain control
        gain_frame = ttk.Frame(processing_frame)
        gain_frame.pack(fill=tk.X, padx=5, pady=2)
        ttk.Label(gain_frame, text="Gain:").pack(side=tk.LEFT)
        self.gain_var = tk.DoubleVar(value=0.5)
        ttk.Scale(gain_frame, from_=0, to=2, variable=self.gain_var,
                 command=self.update_audio_settings).pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.gain_value_label = ttk.Label(gain_frame, text="0.5")
        self.gain_value_label.pack(side=tk.LEFT, padx=5)

        # Gate threshold control
        gate_frame = ttk.Frame(processing_frame)
        gate_frame.pack(fill=tk.X, padx=5, pady=2)
        ttk.Label(gate_frame, text="Gate:").pack(side=tk.LEFT)
        self.gate_var = tk.IntVar(value=200)
        ttk.Scale(gate_frame, from_=0, to=1000, variable=self.gate_var,
                 command=self.update_audio_settings).pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.gate_value_label = ttk.Label(gate_frame, text="200")
        self.gate_value_label.pack(side=tk.LEFT, padx=5)

        # Noise reduction control
        reduction_frame = ttk.Frame(processing_frame)
        reduction_frame.pack(fill=tk.X, padx=5, pady=2)
        ttk.Label(reduction_frame, text="Reduction:").pack(side=tk.LEFT)
        self.reduction_var = tk.DoubleVar(value=0.5)
        ttk.Scale(reduction_frame, from_=0, to=1, variable=self.reduction_var,
                 command=self.update_audio_settings).pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.reduction_value_label = ttk.Label(reduction_frame, text="0.5")
        self.reduction_value_label.pack(side=tk.LEFT, padx=5)

        # Meters frame
        meters_frame = ttk.LabelFrame(controls_frame, text="Meters")
        meters_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

        # Peak level meter
        peak_frame = ttk.Frame(meters_frame)
        peak_frame.pack(fill=tk.X, padx=5, pady=2)
        ttk.Label(peak_frame, text="Peak:").pack(side=tk.LEFT)
        self.peak_level_bar = ttk.Progressbar(peak_frame, length=100, mode='determinate')
        self.peak_level_bar.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

        # Noise floor meter
        noise_frame = ttk.Frame(meters_frame)
        noise_frame.pack(fill=tk.X, padx=5, pady=2)
        ttk.Label(noise_frame, text="Noise:").pack(side=tk.LEFT)
        self.noise_floor_bar = ttk.Progressbar(noise_frame, length=100, mode='determinate')
        self.noise_floor_bar.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

        # Latency frame
        latency_frame = ttk.LabelFrame(controls_frame, text="Latency")
        latency_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

        # Average latency
        avg_frame = ttk.Frame(latency_frame)
        avg_frame.pack(fill=tk.X, padx=5, pady=2)
        ttk.Label(avg_frame, text="Avg:").pack(side=tk.LEFT)
        self.avg_latency_label = ttk.Label(avg_frame, text="0.0 ms")
        self.avg_latency_label.pack(side=tk.LEFT, padx=5)

        # Peak latency
        peak_latency_frame = ttk.Frame(latency_frame)
        peak_latency_frame.pack(fill=tk.X, padx=5, pady=2)
        ttk.Label(peak_latency_frame, text="Peak:").pack(side=tk.LEFT)
        self.peak_latency_label = ttk.Label(peak_latency_frame, text="0.0 ms")
        self.peak_latency_label.pack(side=tk.LEFT, padx=5)

        # Buffer latency
        buffer_frame = ttk.Frame(latency_frame)
        buffer_frame.pack(fill=tk.X, padx=5, pady=2)
        ttk.Label(buffer_frame, text="Buffer:").pack(side=tk.LEFT)
        self.buffer_latency_label = ttk.Label(buffer_frame, text="0.0 ms")
        self.buffer_latency_label.pack(side=tk.LEFT, padx=5)

    def create_devices_section(self):
        button_frame = ttk.Frame(self.devices_frame)
        button_frame.pack(fill=tk.X, padx=5, pady=5)

        self.scan_button = ttk.Button(button_frame, text="Scan Devices", 
                                    command=lambda: self.loop.create_task(self.start_scan()))
        self.scan_button.pack(side=tk.LEFT, padx=2)

        self.connect_button = ttk.Button(button_frame, text="Connect", 
                                       command=lambda: self.loop.create_task(self.connect()),
                                       state=tk.DISABLED)
        self.connect_button.pack(side=tk.LEFT, padx=2)

        self.disconnect_button = ttk.Button(button_frame, text="Disconnect", 
                                          command=lambda: self.loop.create_task(self.disconnect()),
                                          state=tk.DISABLED)
        self.disconnect_button.pack(side=tk.LEFT, padx=2)

        self.device_listbox = tk.Listbox(self.devices_frame, selectmode=tk.EXTENDED)
        self.device_listbox.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.device_listbox.bind('<<ListboxSelect>>', self.on_device_select)

    def create_routing_section(self):
        """Creates the routing section with both available and active routes"""
        dest_frame = ttk.Frame(self.routing_frame)
        dest_frame.pack(fill=tk.X, padx=5, pady=5)

        ttk.Button(dest_frame, text="Add Port", command=self.add_osc_destination).pack(side=tk.LEFT, padx=2)
        ttk.Button(dest_frame, text="Remove Port", command=self.remove_osc_destination).pack(side=tk.LEFT, padx=2)

        # Create three-panel layout
        lists_frame = ttk.Frame(self.routing_frame)
        lists_frame.pack(fill=tk.BOTH, expand=True)

        # Left panel - Destinations
        dest_list_frame = ttk.LabelFrame(lists_frame, text="Destinations")
        dest_list_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.dest_listbox = tk.Listbox(dest_list_frame)
        self.dest_listbox.pack(fill=tk.BOTH, expand=True)

        # Middle panel - Available Routes
        available_routes_frame = ttk.LabelFrame(lists_frame, text="Available Routes")
        available_routes_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Add route info display
        self.route_info_text = tk.Text(available_routes_frame, height=3, width=30)
        self.route_info_text.pack(fill=tk.X, padx=5, pady=5)
        self.route_info_text.config(state=tk.DISABLED)
        
        self.available_routes_listbox = tk.Listbox(available_routes_frame)
        self.available_routes_listbox.pack(fill=tk.BOTH, expand=True)

        # Add route selection buttons
        route_buttons_frame = ttk.Frame(available_routes_frame)
        route_buttons_frame.pack(fill=tk.X, pady=5)
        ttk.Button(route_buttons_frame, text="Add →", 
                  command=self.add_selected_route).pack(side=tk.LEFT, padx=2)
        ttk.Button(route_buttons_frame, text="← Remove", 
                  command=self.remove_selected_route).pack(side=tk.LEFT, padx=2)

        # Right panel - Active Routes
        active_routes_frame = ttk.LabelFrame(lists_frame, text="Active Routes")
        active_routes_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.route_listbox = tk.Listbox(active_routes_frame)
        self.route_listbox.pack(fill=tk.BOTH, expand=True)

        # Route enable/disable checkbox
        self.route_enabled_var = tk.BooleanVar(value=True)
        self.route_enabled_check = ttk.Checkbutton(
            active_routes_frame,
            text="Enabled",
            variable=self.route_enabled_var,
            command=self.toggle_selected_route
        )
        self.route_enabled_check.pack(pady=5)

    # Route management methods
    def add_selected_route(self):
        """Add selected available route to active routes"""
        dest_sel = self.dest_listbox.curselection()
        if not dest_sel:
            showerror("Error", "Please select a destination first")
            return
        
        route_sel = self.available_routes_listbox.curselection()
        if not route_sel:
            showerror("Error", "Please select a route to add")
            return
        
        try:
            dest = self.osc_destinations[dest_sel[0]]
            route_template = self.route_manager.get_available_routes()[route_sel[0]]
            if dest.add_route(route_template):
                self.log_message(f"Added route {route_template.path}")
                self.update_route_list(dest_sel[0])
        except Exception as e:
            self.log_message(f"Error adding route: {e}")
            showerror("Error", f"Failed to add route: {e}")

    def remove_selected_route(self):
        """Remove selected active route"""
        dest_sel = self.dest_listbox.curselection()
        if not dest_sel:
            showerror("Error", "Please select a destination first")
            return
        
        route_sel = self.route_listbox.curselection()
        if not route_sel:
            showerror("Error", "Please select a route to remove")
            return

        try:
            dest = self.osc_destinations[dest_sel[0]]
            dest.remove_route(route_sel[0])
            self.update_route_list(dest_sel[0])
            self.log_message("Removed route")
        except Exception as e:
            self.log_message(f"Error removing route: {e}")

    def toggle_selected_route(self):
        """Toggle enabled state of selected route"""
        dest_sel = self.dest_listbox.curselection()
        route_sel = self.route_listbox.curselection()
        
        if not dest_sel or not route_sel:
            return

        try:
            dest = self.osc_destinations[dest_sel[0]]
            dest.toggle_route(route_sel[0])
            self.update_route_list(dest_sel[0])
        except Exception as e:
            self.log_message(f"Error toggling route: {e}")

    def update_route_list(self, dest_index):
        """Updates the active routes list for the selected destination"""
        try:
            self.route_listbox.delete(0, tk.END)
            if 0 <= dest_index < len(self.osc_destinations):
                dest = self.osc_destinations[dest_index]
                for route in dest.routes:
                    status = "✓" if route.enabled else "✗"
                    self.route_listbox.insert(tk.END, f"{status} {route.path} ({route.data_type})")
        except Exception as e:
            self.log_message(f"Error updating route list: {e}")

    # Selection event handlers
    def on_available_route_select(self, event):
        """Show details about the selected available route"""
        selected = self.available_routes_listbox.curselection()
        if not selected:
            return

        try:
            route = self.route_manager.get_available_routes()[selected[0]]
            self.route_info_text.config(state=tk.NORMAL)
            self.route_info_text.delete(1.0, tk.END)
            self.route_info_text.insert(tk.END, 
                f"Path: {route.path}\n"
                f"Type: {route.data_type}\n"
                f"Last seen: {time.strftime('%H:%M:%S', time.localtime(route.last_seen))}")
            if route.sample_value is not None:
                self.route_info_text.insert(tk.END, f"\nSample value: {route.sample_value}")
            self.route_info_text.config(state=tk.DISABLED)
        except Exception as e:
            self.log_message(f"Error showing route details: {e}")

    def on_active_route_select(self, event):
        """Update checkbox state when an active route is selected"""
        selected = self.route_listbox.curselection()
        dest_sel = self.dest_listbox.curselection()
        if selected and dest_sel:
            try:
                dest = self.osc_destinations[dest_sel[0]]
                route = dest.routes[selected[0]]
                self.route_enabled_var.set(route.enabled)
            except Exception as e:
                self.log_message(f"Error updating route state: {e}")

    def on_destination_select(self, event):
        """Called when a destination is selected"""
        selected = self.dest_listbox.curselection()
        if selected:
            self.update_route_list(selected[0])

    def on_device_select(self, event):
        """Called when a device is selected"""
        selected = self.device_listbox.curselection()
        self.connect_button.state(['!disabled'] if selected else ['disabled'])

    # Route monitoring
    def start_route_monitoring(self):
        """Start periodic updates of available routes list"""
        def update_routes():
            if not self.is_destroyed:
                try:
                    # Store current selections
                    current_selection = self.available_routes_listbox.curselection()
                    selected_index = current_selection[0] if current_selection else None
                    
                    # Update list
                    self.available_routes_listbox.delete(0, tk.END)
                    for route in self.route_manager.get_available_routes():
                        self.available_routes_listbox.insert(tk.END, 
                            f"{route.path} ({route.data_type})")
                    
                    # Restore selection
                    if selected_index is not None:
                        if selected_index < self.available_routes_listbox.size():
                            self.available_routes_listbox.selection_set(selected_index)
                    
                    # Schedule next update
                    self.root.after(1000, update_routes)  # Update every second
                except Exception as e:
                    self.log_message(f"Error updating routes: {e}")
                    self.root.after(1000, update_routes)  # Keep trying even if there's an error

        # Start the update loop
        update_routes()

    # OSC destination management
    def add_osc_destination(self):
        """Add a new OSC destination"""
        port = simpledialog.askinteger("Add Local Destination", "Enter port number:")
        if port:
            try:
                dest = OSCDestination(port)
                self.osc_destinations.append(dest)
                self.dest_listbox.insert(tk.END, dest.name)
                self.log_message(f"Added OSC destination on port {port}")
            except Exception as e:
                self.log_message(f"Error adding destination: {e}")
                showerror("Error", f"Failed to create OSC destination: {e}")

    def remove_osc_destination(self):
        """Remove selected OSC destination"""
        try:
            selected = self.dest_listbox.curselection()[0]
            self.dest_listbox.delete(selected)
            del self.osc_destinations[selected]
            self.log_message("Removed OSC destination")
        except IndexError:
            showerror("Error", "Please select a destination to remove")
        except Exception as e:
            self.log_message(f"Error removing destination: {e}")

    # Device management
    async def start_scan(self):
        """Start scanning for Bluetooth devices"""
        try:
            self.device_listbox.delete(0, tk.END)
            self.IMU_devices.clear()
            self.scan_button.state(['disabled'])
            
            async def device_detected(device, _):
                if (device.name and 
                    device.name.lower() == self.device_name.lower() and 
                    device.address not in self.IMU_devices):
                    
                    self.IMU_devices[device.address] = device
                    self.root.after(0, lambda: 
                        self.device_listbox.insert(tk.END, 
                            f"{device.name} ({device.address})"))

            self.scanner = BleakScanner(detection_callback=device_detected)
            await self.scanner.start()
            await asyncio.sleep(10)
            await self.scanner.stop()
            
        except BleakError as e:
            showerror("Bluetooth Error", f"Bluetooth error: {e}")
        except Exception as e:
            showerror("Error", f"Scan error: {e}")
        finally:
            self.scan_button.state(['!disabled'])

    async def connect(self):
        """Connect to selected devices"""
        selected_indices = self.device_listbox.curselection()
        if not selected_indices:
            showerror("Connection Error", "No device selected")
            return

        if not self.osc_destinations:
            showerror("Routing Error", "No OSC destinations configured")
            return

        try:
            self.clients = []
            for index in selected_indices:
                address = list(self.IMU_devices.keys())[index]
                device = self.IMU_devices[address]
                
                client = BleakClient(device)
                await client.connect()
                
                if client.is_connected:
                    self.clients.append(client)
                    self.log_message(f"Connected to {address}")
                    await client.start_notify(
                        "6e400003-b5a3-f393-e0a9-e50e24dcca9e", 
                        self.handle_notification
                    )
                    
            if self.clients:
                self.connect_button.state(['disabled'])
                self.disconnect_button.state(['!disabled'])
                self.device_listbox.config(state=tk.DISABLED)
                self.record_button.state(['!disabled'])
                
        except Exception as e:
            showerror("Connection Error", f"Failed to connect: {e}")

    async def disconnect(self):
        """Disconnect from all devices"""
        if self.audio_recorder.recording:
            self.toggle_recording()
            
        try:
            for client in self.clients:
                if client.is_connected:
                    await client.disconnect()
            self.clients.clear()
            self.log_message("All devices disconnected")
            
            self.disconnect_button.state(['disabled'])
            self.connect_button.state(['!disabled'])
            self.device_listbox.config(state=tk.NORMAL)
            self.record_button.state(['disabled'])
            
        except Exception as e:
            self.log_message(f"Disconnection error: {e}")

    def handle_notification(self, sender, data):
        """Handle notifications from connected devices"""
        try:
            decoded_data = self.decode_data(data)
            if decoded_data:
                # Register routes with the route manager when data is received
                if decoded_data['pcm_data']:
                    self.route_manager.update_route(
                        path="/metabow/audio",
                        data_type="pcm",
                        sample_value=len(decoded_data['pcm_data'])
                    )
                
                if decoded_data['motion_data']:
                    self.route_manager.update_route(
                        path="/metabow/motion",
                        data_type="motion",
                        sample_value=decoded_data['motion_data']
                    )

                # Write to WAV file if recording
                if decoded_data['pcm_data'] and self.audio_recorder.recording:
                    self.audio_recorder.write_frames(decoded_data['pcm_data'])
                
                # Route data through configured OSC destinations
                for dest in self.osc_destinations:
                    for route in dest.routes:
                        if not route.enabled:
                            continue
                            
                        data_key = f"{route.data_type}_data"
                        if data_key in decoded_data and decoded_data[data_key]:
                            try:
                                dest.client.send_message(route.path, decoded_data[data_key])
                            except Exception as e:
                                self.log_message(f"Error sending {route.data_type} to {route.path}: {e}")
                                
        except Exception as e:
            self.log_message(f"Notification handling error: {e}")

    def decode_data(self, data):
        """Decode data received from device"""
        try:
            imu_data_len = 13 * 4
            flag_size = 1
            pcm_chunk_size = 2
            
            flag = data[-1]
            data_len = len(data)
            
            pcm_data = []
            for i in range(0, data_len - imu_data_len - flag_size, pcm_chunk_size):
                pcm_value = int.from_bytes(
                    data[i:i+pcm_chunk_size],
                    byteorder='little',
                    signed=True
                )
                pcm_data.append(pcm_value)
            
            motion_floats = []
            if flag == 1:
                motion_start = data_len - imu_data_len - flag_size
                motion_end = data_len - flag_size
                motion_floats = [
                    struct.unpack('f', data[i:i+4])[0] 
                    for i in range(motion_start, motion_end, 4)
                ]
            
            return {
                'pcm_data': pcm_data,
                'motion_data': motion_floats,
                'flag': flag
            }
                
        except Exception as e:
            self.log_message(f"Data decoding error: {e}")
            return None

    # Audio recording
    def toggle_recording(self):
        """Toggle audio recording state"""
        if not self.audio_recorder.recording:
            directory = filedialog.askdirectory(
                title="Choose Recording Save Location",
                initialdir=os.path.expanduser("~/Documents")
            )
            if directory:
                try:
                    filename = self.audio_recorder.start_recording(directory)
                    self.record_button.configure(text="Stop Recording")
                    self.recording_label.configure(text=f"Recording to: {os.path.basename(filename)}")
                    self.log_message(f"Started recording to {filename}")
                except Exception as e:
                    showerror("Recording Error", f"Failed to start recording: {e}")
        else:
            try:
                filename = self.audio_recorder.stop_recording()
                self.record_button.configure(text="Start Recording")
                self.recording_label.configure(text="Not Recording")
                if filename:
                    self.log_message(f"Stopped recording. Saved to {filename}")
            except Exception as e:
                showerror("Recording Error", f"Failed to stop recording: {e}")

    # Monitoring methods
    def start_level_monitoring(self):
        """Start audio level monitoring"""
        def update_meters():
            if not self.is_destroyed:
                peak_db = 20 * np.log10(max(1e-6, self.audio_recorder.peak_level / 32767))
                peak_percent = min(100, max(0, (peak_db + 60) * 1.66))
                self.peak_level_bar['value'] = peak_percent

                noise_db = 20 * np.log10(max(1e-6, self.audio_recorder.noise_floor / 32767))
                noise_percent = min(100, max(0, (noise_db + 60) * 1.66))
                self.noise_floor_bar['value'] = noise_percent

                self.root.after(100, update_meters)
        update_meters()

    def update_latency_display(self):
        """Update latency display"""
        if not self.is_destroyed:
            self.avg_latency_label.configure(
                text=f"{self.audio_recorder.avg_latency:.1f} ms")
            self.peak_latency_label.configure(
                text=f"{self.audio_recorder.peak_latency:.1f} ms")
            self.buffer_latency_label.configure(
                text=f"{self.audio_recorder.buffer_latency:.1f} ms")
            self.root.after(100, self.update_latency_display)

    def update_audio_settings(self, *args):
        """Update audio processing settings"""
        try:
            self.audio_recorder.gain = self.gain_var.get()
            self.audio_recorder.gate_threshold = self.gate_var.get()
            self.audio_recorder.noise_reduction = self.reduction_var.get()
            
            self.gain_value_label.configure(text=f"{self.gain_var.get():.1f}")
            self.gate_value_label.configure(text=f"{self.gate_var.get()}")
            self.reduction_value_label.configure(text=f"{self.reduction_var.get():.1f}")
            
            self.log_message(f"Updated audio settings - Gain: {self.audio_recorder.gain}, "
                           f"Gate: {self.audio_recorder.gate_threshold}, "
                           f"Reduction: {self.audio_recorder.noise_reduction}")
        except tk.TclError:
            pass

    # Logging
    def log_message(self, message):
        """Log a message to the UI"""
        if not self.is_destroyed:
            self.root.after(0, lambda: self._safe_log(message))

    def _safe_log(self, message):
        """Safely log a message to the text widget"""
        try:
            self.log_text.insert(tk.END, f"{datetime.now().strftime('%H:%M:%S')}: {message}\n")
            self.log_text.see(tk.END)
        except tk.TclError:
            pass

    # Application lifecycle
    def on_exit(self):
        """Handle application exit"""
        if askyesno("Exit", "Do you want to quit the application?"):
            if self.audio_recorder.recording:
                self.toggle_recording()
            self.is_destroyed = True
            self.loop.create_task(self.cleanup())
            self.root.quit()

    async def cleanup(self):
        """Clean up resources"""
        if self.scanner:
            await self.scanner.stop()
        await self.disconnect()

    async def run(self):
        """Main application loop"""
        try:
            while not self.is_destroyed:
                self.root.update()
                await asyncio.sleep(0.1)
        except Exception as e:
            self.log_message(f"Error in main loop: {e}")
        finally:
            await self.cleanup()

class AudioRecorder:
    def __init__(self, loop, channels=1, sample_width=2, framerate=16000):
        self.loop = loop
        self.channels = channels
        self.sample_width = sample_width
        self.framerate = framerate
        self.recording = False
        self.wave_file = None
        self.filename = None
        
        # PDM and SNR specifications for MP34DT05-A
        self.pdm_scale = 32768.0
        self.sensitivity_db = -26
        self.sensitivity_scale = pow(10, self.sensitivity_db / 20)
        self.snr_db = 64
        self.snr_linear = pow(10, self.snr_db / 20)
        
        # Audio processing parameters
        self.dc_offset = 0
        self.dc_alpha = 0.995
        self.buffer_size = int(self.snr_linear)
        self.sample_buffer = []
        
        self.gain = 0.5
        self.gate_threshold = 200
        self.noise_reduction = 0.5

        # Real-time statistics
        self.peak_level = 0
        self.noise_floor = 0

        # Latency tracking
        self.processing_times = []
        self.max_processing_times = 100
        self.avg_latency = 0
        self.peak_latency = 0
        self.buffer_latency = 0

    def update_latency_stats(self, processing_time):
        self.processing_times.append(processing_time)
        if len(self.processing_times) > self.max_processing_times:
            self.processing_times.pop(0)
        
        self.avg_latency = sum(self.processing_times) / len(self.processing_times)
        self.peak_latency = max(self.processing_times)
        samples_in_buffer = len(self.sample_buffer)
        self.buffer_latency = (samples_in_buffer / self.framerate) * 1000

    def process_audio(self, pcm_data):
        start_time = time.time()

        if not pcm_data:
            return pcm_data

        samples = np.array(pcm_data, dtype=np.float32)
        
        if len(self.sample_buffer) < self.buffer_size:
            self.sample_buffer.extend(pcm_data)
        else:
            self.sample_buffer = self.sample_buffer[len(pcm_data):] + pcm_data
        
        self.dc_offset = self.dc_alpha * self.dc_offset + (1 - self.dc_alpha) * np.mean(samples)
        samples = samples - self.dc_offset
        
        samples = samples * self.sensitivity_scale * self.gain
        
        if len(self.sample_buffer) >= self.buffer_size:
            self.noise_floor = np.percentile(np.abs(self.sample_buffer), 15)
        
        noise_mask = np.abs(samples) > (self.noise_floor * self.gate_threshold / 100)
        samples = samples * noise_mask
        
        samples = samples * (1 - self.noise_reduction * (1 - noise_mask))
        
        clip_mask = np.abs(samples) > 32767
        samples[clip_mask] = np.sign(samples[clip_mask]) * (32767 - (32767 - np.abs(samples[clip_mask])) / 3)
        
        self.peak_level = max(self.peak_level * 0.95, np.max(np.abs(samples)))
        
        processed_data = samples.astype(np.int16).tolist()

        processing_time = (time.time() - start_time) * 1000
        self.update_latency_stats(processing_time)

        return processed_data

    def write_frames(self, pcm_data):
        if self.recording and self.wave_file and pcm_data:
            try:
                processed_data = self.process_audio(pcm_data)
                frames = b''.join(val.to_bytes(2, 'little', signed=True) 
                                for val in processed_data)
                self.wave_file.writeframes(frames)
            except Exception as e:
                print(f"Error writing audio frames: {e}")

    def start_recording(self, directory=None):
        if directory is None:
            directory = os.path.expanduser("~/Documents")
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = os.path.join(directory, f"metabow_recording_{timestamp}.wav")
        
        self.wave_file = wave.open(self.filename, 'wb')
        self.wave_file.setnchannels(self.channels)
        self.wave_file.setsampwidth(self.sample_width)
        self.wave_file.setframerate(self.framerate)
        self.recording = True
        
        self.dc_offset = 0
        self.sample_buffer = []
        self.peak_level = 0
        self.noise_floor = 0
        
        return self.filename

    def stop_recording(self):
        if self.recording and self.wave_file:
            self.wave_file.close()
            self.wave_file = None
            self.recording = False
            return self.filename
        return None

def main():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    
    window = Window(loop)
    try:
        loop.run_until_complete(window.run())
    except KeyboardInterrupt:
        print("\nApplication interrupted by user")
    except Exception as e:
        print(f"Application error: {e}")
    finally:
        loop.close()

if __name__ == '__main__':
    main()