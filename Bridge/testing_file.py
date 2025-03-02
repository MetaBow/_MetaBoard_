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
import sounddevice as sd
import queue
import wave
import pyaudio
import queue
import subprocess
import platform
import logging
import shutil


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
    custom_path: str = None  # New field for custom path

    @property
    def effective_path(self):
        """Returns the custom path if set, otherwise returns the default path"""
        return self.custom_path if self.custom_path else self.path

class OSCRouteManager:
    """Manages discovered OSC routes"""
    def __init__(self):
        self.discovered_routes = {}  # path -> OSCRouteTemplate
        self.discovery_callbacks = []  # Callbacks for when new routes are discovered
        print("DEBUG: OSCRouteManager initialized")

    def register_discovery_callback(self, callback):
        """Register a callback to be notified when new routes are discovered"""
        self.discovery_callbacks.append(callback)
        print(f"DEBUG: Registered new discovery callback, total callbacks: {len(self.discovery_callbacks)}")

    def update_route(self, path: str, data_type: str, sample_value: Any = None):
        """Update or add a route based on received data"""
        print(f"DEBUG: Attempting to update route {path} ({data_type})")  # Debug print
        
        if path not in self.discovered_routes:
            print(f"DEBUG: Creating new route: {path}")  # Debug print
            self.discovered_routes[path] = OSCRouteTemplate(path, data_type, sample_value=sample_value)
            # Notify callbacks of new route
            for callback in self.discovery_callbacks:
                callback(path, data_type)
        else:
            print(f"DEBUG: Updating existing route: {path}")  # Debug print
            self.discovered_routes[path].last_seen = time.time()
            self.discovered_routes[path].sample_value = sample_value

    def get_available_routes(self):
        """Get list of discovered routes"""
        routes = list(self.discovered_routes.values())
        print(f"DEBUG: Returning {len(routes)} available routes")  # Debug print
        return routes

@dataclass
class OSCBundle:
    """A bundle that combines multiple OSC routes into a single message"""
    name: str
    path: str
    enabled: bool = True
    routes: List[OSCRoute] = field(default_factory=list)

class OSCDestination:
    def __init__(self, port):
        self.port = port
        self.name = f"Local Port {port}"
        self.client = udp_client.SimpleUDPClient("127.0.0.1", port)
        self.routes = []  # Individual routes
        self.bundles = [] # Bundles for combined messages

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
    def add_bundle(self, name: str, path: str) -> OSCBundle:
        """Create a new bundle"""
        bundle = OSCBundle(name=name, path=path)
        self.bundles.append(bundle)
        return bundle

    def remove_bundle(self, bundle_index: int):
        """Remove a bundle by index"""
        if 0 <= bundle_index < len(self.bundles):
            self.bundles.pop(bundle_index)

    def add_route_to_bundle(self, bundle: OSCBundle, route: OSCRoute) -> bool:
        """Add a route to a bundle if not already present"""
        if route not in bundle.routes:
            bundle.routes.append(route)
            return True
        return False

    def remove_route_from_bundle(self, bundle: OSCBundle, route_index: int):
        """Remove a route from a bundle"""
        if 0 <= route_index < len(bundle.routes):
            bundle.routes.pop(route_index)

    def get_bundle_values(self, bundle: OSCBundle, decoded_data: dict) -> List[float]:
        """Get all values for a bundle's routes from decoded data"""
        values = []
        motion_paths = [
            "quaternion_i", "quaternion_j", "quaternion_k", "quaternion_r",
            "accelerometer_x", "accelerometer_y", "accelerometer_z",
            "gyroscope_x", "gyroscope_y", "gyroscope_z",
            "magnetometer_x", "magnetometer_y", "magnetometer_z"
        ]
        
        print(f"DEBUG Bundle Value Collection:")
        print(f"  Bundle: {bundle.name}")
        print(f"  Enabled: {bundle.enabled}")
        print(f"  Total Routes: {len(bundle.routes)}")
        
        for route in bundle.routes:
            print(f"  Checking Route: {route.path}")
            print(f"    Enabled: {route.enabled}")
            
            if route.enabled and route.path.startswith("/metabow/motion/"):
                try:
                    # Extract motion component name
                    motion_component = route.path.split('/')[-1]
                    print(f"    Motion Component: {motion_component}")
                    
                    # Find index in motion paths list
                    motion_idx = motion_paths.index(motion_component)
                    print(f"    Motion Index: {motion_idx}")
                    
                    # Add value to bundle values
                    if decoded_data['motion_data']:
                        value = decoded_data['motion_data'][motion_idx]
                        values.append(value)
                        print(f"    Added Value: {value}")
                except Exception as e:
                    print(f"Error getting bundle value for {route.path}: {e}")
        
        print(f"  Total Bundle Values: {values}")
        return values

    def send_bundle_message(self, bundle: OSCBundle, values: List[float]):
        """Send a combined OSC message with all bundle values"""
        if bundle.enabled and values:
            try:
                print(f"DEBUG Bundle Sending:")
                print(f"  Port: {self.port}")
                print(f"  Path: {bundle.path}")
                print(f"  Values: {values}")
                print(f"  Num Values: {len(values)}")
                
                # Additional diagnostic information
                for idx, value in enumerate(values):
                    print(f"    Value {idx}: {value}")
                
                self.client.send_message(bundle.path, values)
                print("  Bundle message sent successfully")
            except Exception as e:
                print(f"Error sending bundle message: {e}")
                import traceback
                traceback.print_exc()

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
        """Bind selection events for route and bundle management"""
        self.dest_listbox.bind('<<ListboxSelect>>', self.on_destination_select)
        self.available_routes_listbox.bind('<<ListboxSelect>>', self.on_available_route_select)
        self.route_listbox.bind('<<ListboxSelect>>', self.on_active_route_select)
        self.bundle_listbox.bind('<<ListboxSelect>>', self.on_bundle_select)

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
        # Create main controls frame first
        controls_frame = ttk.Frame(self.audio_frame)
        controls_frame.pack(fill=tk.X, padx=5, pady=5)

        # Then create virtual output frame
        virtual_frame = ttk.LabelFrame(controls_frame, text="Virtual Output")
        virtual_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

        self.virtual_output_button = ttk.Button(
            virtual_frame, 
            text="Enable Virtual Output",
            command=self.toggle_virtual_output
        )
        self.virtual_output_button.pack(side=tk.LEFT, padx=5, pady=5)

        self.virtual_output_label = ttk.Label(virtual_frame, text="Disabled")
        self.virtual_output_label.pack(side=tk.LEFT, padx=5, pady=5)

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

    def toggle_virtual_output(self):
        """Toggle virtual audio output"""
        try:
            success = self.audio_recorder.toggle_virtual_output()
            if success:
                new_text = "Disable Virtual Output" if self.audio_recorder.virtual_output_enabled else "Enable Virtual Output"
                new_status = "Enabled" if self.audio_recorder.virtual_output_enabled else "Disabled"
                self.virtual_output_button.configure(text=new_text)
                self.virtual_output_label.configure(text=new_status)
                self.log_message(f"Virtual output {new_status.lower()}")
            else:
                self.log_message("Failed to toggle virtual output")
                showerror("Error", "Failed to toggle virtual output")
        except Exception as e:
            self.log_message(f"Error toggling virtual output: {e}")
            showerror("Error", f"Failed to toggle virtual output: {e}")

        # Update button text
        new_text = "Disable Virtual Output" if self.audio_recorder.virtual_output_enabled else "Enable Virtual Output"
        self.virtual_output_button.configure(text=new_text)


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
        # Set exportselection=0 to maintain selection
        self.dest_listbox = tk.Listbox(dest_list_frame, exportselection=0)
        self.dest_listbox.pack(fill=tk.BOTH, expand=True)

        # Middle panel - Available Routes
        available_routes_frame = ttk.LabelFrame(lists_frame, text="Available Routes")
        available_routes_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Add route info display
        self.route_info_text = tk.Text(available_routes_frame, height=3, width=30)
        self.route_info_text.pack(fill=tk.X, padx=5, pady=5)
        self.route_info_text.config(state=tk.DISABLED)
        
        # Set exportselection=0 to maintain selection
        self.available_routes_listbox = tk.Listbox(available_routes_frame, exportselection=0)
        self.available_routes_listbox.pack(fill=tk.BOTH, expand=True)

        # Route controls for Available Routes
        available_route_controls = ttk.Frame(available_routes_frame)
        available_route_controls.pack(fill=tk.X, pady=5)
        
        ttk.Button(available_route_controls, text="Add Route", 
                   command=self.add_selected_route).pack(side=tk.LEFT, padx=2)

        # Right panel - Active Routes
        active_routes_frame = ttk.LabelFrame(lists_frame, text="Active Routes")
        active_routes_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Add route control buttons frame
        route_controls = ttk.Frame(active_routes_frame)
        route_controls.pack(fill=tk.X, pady=5)
        
        ttk.Button(route_controls, text="Remove Route", 
                   command=self.remove_selected_route).pack(side=tk.LEFT, padx=2)
        ttk.Button(route_controls, text="Edit Path", 
                   command=self.edit_selected_route_path).pack(side=tk.LEFT, padx=2)
        ttk.Button(route_controls, text="Reset Path", 
                   command=self.reset_selected_route_path).pack(side=tk.LEFT, padx=2)
        
        # Set exportselection=0 to maintain selection
        self.route_listbox = tk.Listbox(active_routes_frame, exportselection=0)
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

        # Bundle Management section (rest of the code remains the same)
        bundle_frame = ttk.LabelFrame(self.routing_frame, text="Bundle Management")
        bundle_frame.pack(fill=tk.X, padx=5, pady=5)

        # Bundle controls
        bundle_controls = ttk.Frame(bundle_frame)
        bundle_controls.pack(fill=tk.X, pady=2)

        ttk.Button(bundle_controls, text="Create Bundle", 
                   command=self.create_bundle).pack(side=tk.LEFT, padx=2)
        ttk.Button(bundle_controls, text="Delete Bundle", 
                   command=self.delete_bundle).pack(side=tk.LEFT, padx=2)
        ttk.Button(bundle_controls, text="Add Selected to Bundle", 
                   command=self.add_to_bundle).pack(side=tk.LEFT, padx=2)
        ttk.Button(bundle_controls, text="Remove from Bundle", 
                   command=self.remove_from_bundle).pack(side=tk.LEFT, padx=2)

        # Bundle list and routes split view
        bundle_list_frame = ttk.Frame(bundle_frame)
        bundle_list_frame.pack(fill=tk.BOTH, expand=True)

        # Bundle list (left side)
        bundle_list_subframe = ttk.Frame(bundle_list_frame)
        bundle_list_subframe.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        ttk.Label(bundle_list_subframe, text="Available Bundles").pack(fill=tk.X)
        # Set exportselection=0 to maintain selection
        self.bundle_listbox = tk.Listbox(bundle_list_subframe, height=6, exportselection=0)
        self.bundle_listbox.pack(fill=tk.BOTH, expand=True)

        # Bundle routes (right side)
        bundle_routes_subframe = ttk.Frame(bundle_list_frame)
        bundle_routes_subframe.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        ttk.Label(bundle_routes_subframe, text="Bundle Routes").pack(fill=tk.X)
        # Set exportselection=0 to maintain selection
        self.bundle_routes_listbox = tk.Listbox(bundle_routes_subframe, height=6, exportselection=0)
        self.bundle_routes_listbox.pack(fill=tk.BOTH, expand=True)

        # Bundle enable/disable checkbox
        self.bundle_enabled_var = tk.BooleanVar(value=True)
        self.bundle_enabled_check = ttk.Checkbutton(
            bundle_frame,
            text="Bundle Enabled",
            variable=self.bundle_enabled_var,
            command=self.toggle_selected_bundle
        )
        self.bundle_enabled_check.pack(pady=2)

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

    def edit_selected_route_path(self):
        """Edit the path of the selected route"""
        dest_sel = self.dest_listbox.curselection()
        route_sel = self.route_listbox.curselection()
        
        if not dest_sel or not route_sel:
            showerror("Error", "Please select a route to edit")
            return
            
        try:
            dest = self.osc_destinations[dest_sel[0]]
            route = dest.routes[route_sel[0]]
            
            # Show dialog with current path
            current_path = route.effective_path
            new_path = simpledialog.askstring(
                "Edit OSC Path",
                "Enter new OSC path:",
                initialvalue=current_path
            )
            
            if new_path:
                # Validate OSC path format
                if not new_path.startswith('/'):
                    new_path = '/' + new_path
                    
                route.custom_path = new_path
                self.update_route_list(dest_sel[0])
                self.log_message(f"Updated route path from {route.path} to {new_path}")
                
        except Exception as e:
            self.log_message(f"Error editing route path: {e}")
            showerror("Error", f"Failed to edit route path: {e}")

    def reset_selected_route_path(self):
        """Reset the path of the selected route to its default"""
        dest_sel = self.dest_listbox.curselection()
        route_sel = self.route_listbox.curselection()
        
        if not dest_sel or not route_sel:
            showerror("Error", "Please select a route to reset")
            return
            
        try:
            dest = self.osc_destinations[dest_sel[0]]
            route = dest.routes[route_sel[0]]
            
            if route.custom_path:
                old_path = route.custom_path
                route.custom_path = None
                self.update_route_list(dest_sel[0])
                self.log_message(f"Reset route path from {old_path} to {route.path}")
            
        except Exception as e:
            self.log_message(f"Error resetting route path: {e}")

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
                    path_display = route.effective_path
                    if route.custom_path:
                        path_display += f" (default: {route.path})"
                    self.route_listbox.insert(tk.END, f"{status} {path_display} ({route.data_type})")
        except Exception as e:
            self.log_message(f"Error updating route list: {e}")

    def create_bundle(self):
        """Create a new OSC bundle"""
        dest_sel = self.dest_listbox.curselection()
        if not dest_sel:
            showerror("Error", "Please select a destination first")
            return

        try:
            bundle_name = simpledialog.askstring(
                "Create Bundle", 
                "Enter bundle name:",
                initialvalue="New Bundle"
            )
            if not bundle_name:
                return

            bundle_path = simpledialog.askstring(
                "Create Bundle",
                "Enter OSC path for bundled data:",
                initialvalue="/wekinator/input"
            )
            if not bundle_path:
                return

            if not bundle_path.startswith('/'):
                bundle_path = '/' + bundle_path

            dest = self.osc_destinations[dest_sel[0]]
            
            # Create the bundle
            bundle = dest.add_bundle(bundle_name, bundle_path)
            
            # Prompt user to add routes to bundle
            self.add_to_bundle()
            
            # Explicitly log bundle details
            print(f"Bundle Created:")
            print(f"  Name: {bundle_name}")
            print(f"  Path: {bundle_path}")
            print(f"  Routes: {len(bundle.routes)}")
            for route in bundle.routes:
                print(f"    Route: {route.path} (Enabled: {route.enabled})")
            
            self.update_route_list(dest_sel[0])
            self.update_bundle_list(dest_sel[0])
            self.log_message(f"Created bundle: {bundle_name} ({bundle_path})")

        except Exception as e:
            self.log_message(f"Error creating bundle: {e}")
            showerror("Error", f"Failed to create bundle: {e}")

    def delete_bundle(self):
        """Delete the selected bundle"""
        dest_sel = self.dest_listbox.curselection()
        bundle_sel = self.bundle_listbox.curselection()
        
        if not dest_sel or not bundle_sel:
            showerror("Error", "Please select a bundle to delete")
            return
            
        try:
            dest = self.osc_destinations[dest_sel[0]]
            dest.remove_bundle(bundle_sel[0])
            self.update_bundle_list(dest_sel[0])
            self.update_bundle_routes_list()
            self.log_message("Deleted bundle")
        except Exception as e:
            self.log_message(f"Error deleting bundle: {e}")

    def add_to_bundle(self):
        """Add selected route to selected bundle"""
        dest_sel = self.dest_listbox.curselection()
        route_sel = self.route_listbox.curselection()
        bundle_sel = self.bundle_listbox.curselection()
        
        if not all([dest_sel, route_sel, bundle_sel]):
            showerror("Error", "Please select a destination, route, and bundle")
            return
            
        try:
            dest = self.osc_destinations[dest_sel[0]]
            bundle = dest.bundles[bundle_sel[0]]
            route = dest.routes[route_sel[0]]
            
            if dest.add_route_to_bundle(bundle, route):
                self.update_bundle_list(dest_sel[0])
                self.update_bundle_routes_list()
                self.log_message(f"Added route {route.path} to bundle {bundle.name}")
            else:
                self.log_message("Route already in bundle")
                
        except Exception as e:
            self.log_message(f"Error adding route to bundle: {e}")

    def remove_from_bundle(self):
        """Remove selected route from the bundle"""
        dest_sel = self.dest_listbox.curselection()
        bundle_sel = self.bundle_listbox.curselection()
        route_sel = self.bundle_routes_listbox.curselection()
        
        if not all([dest_sel, bundle_sel, route_sel]):
            showerror("Error", "Please select a bundle and route to remove")
            return
            
        try:
            dest = self.osc_destinations[dest_sel[0]]
            bundle = dest.bundles[bundle_sel[0]]
            
            dest.remove_route_from_bundle(bundle, route_sel[0])
            self.update_bundle_list(dest_sel[0])
            self.update_bundle_routes_list()
            self.log_message("Removed route from bundle")
            
        except Exception as e:
            self.log_message(f"Error removing route from bundle: {e}")

    def toggle_selected_bundle(self):
        """Toggle the selected bundle's enabled state"""
        dest_sel = self.dest_listbox.curselection()
        bundle_sel = self.bundle_listbox.curselection()
        
        if not dest_sel or not bundle_sel:
            return
            
        try:
            dest = self.osc_destinations[dest_sel[0]]
            dest.bundles[bundle_sel[0]].enabled = self.bundle_enabled_var.get()
            self.update_bundle_list(dest_sel[0])
        except Exception as e:
            self.log_message(f"Error toggling bundle: {e}")

    def update_bundle_list(self, dest_index):
        """Update the bundle listbox"""
        try:
            self.bundle_listbox.delete(0, tk.END)
            if 0 <= dest_index < len(self.osc_destinations):
                dest = self.osc_destinations[dest_index]
                for bundle in dest.bundles:
                    status = "✓" if bundle.enabled else "✗"
                    route_count = len(bundle.routes)
                    self.bundle_listbox.insert(tk.END, 
                        f"{status} {bundle.name} ({bundle.path}) [{route_count} routes]")
        except Exception as e:
            self.log_message(f"Error updating bundle list: {e}")

    def update_bundle_routes_list(self):
        """Update the list of routes in the selected bundle"""
        dest_sel = self.dest_listbox.curselection()
        bundle_sel = self.bundle_listbox.curselection()
        
        self.bundle_routes_listbox.delete(0, tk.END)
        
        if dest_sel and bundle_sel:
            try:
                dest = self.osc_destinations[dest_sel[0]]
                bundle = dest.bundles[bundle_sel[0]]
                
                for route in bundle.routes:
                    status = "✓" if route.enabled else "✗"
                    self.bundle_routes_listbox.insert(tk.END, 
                        f"{status} {route.path}")
            except Exception as e:
                self.log_message(f"Error updating bundle routes: {e}")

    def on_bundle_select(self, event):
        """Update bundle routes list when a bundle is selected"""
        self.update_bundle_routes_list()
        
        # Update bundle enable checkbox
        bundle_sel = self.bundle_listbox.curselection()
        dest_sel = self.dest_listbox.curselection()
        if bundle_sel and dest_sel:
            try:
                dest = self.osc_destinations[dest_sel[0]]
                bundle = dest.bundles[bundle_sel[0]]
                self.bundle_enabled_var.set(bundle.enabled)
            except Exception as e:
                self.log_message(f"Error updating bundle state: {e}")

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
                    # Get available routes and log count
                    available_routes = self.route_manager.get_available_routes()
                    self.log_message(f"DEBUG: Found {len(available_routes)} available routes")
                    
                    # Store current selections
                    current_selection = self.available_routes_listbox.curselection()
                    selected_index = current_selection[0] if current_selection else None
                    
                    # Get current and new items
                    current_items = self.available_routes_listbox.get(0, tk.END)
                    new_items = [f"{route.path} ({route.data_type})" for route in available_routes]
                    
                    # Update if content has changed
                    if list(current_items) != new_items:
                        self.available_routes_listbox.delete(0, tk.END)
                        for route in available_routes:
                            item_text = f"{route.path} ({route.data_type})"
                            self.available_routes_listbox.insert(tk.END, item_text)
                            self.log_message(f"DEBUG: Listed route {route.path}")
                    
                    # Restore selection
                    if selected_index is not None:
                        if selected_index < self.available_routes_listbox.size():
                            self.available_routes_listbox.selection_set(selected_index)
                    
                    # Schedule next update
                    self.root.after(1000, update_routes)
                    
                except Exception as e:
                    self.log_message(f"Error updating routes: {e}")
                    self.root.after(1000, update_routes)

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
        """Handle notifications and route data to individual routes and bundles"""
        try:
            decoded_data = self.decode_data(data)
            if not decoded_data:
                return

            # Handle Audio Data
            if decoded_data['pcm_data']:
                # Register audio route
                self.route_manager.update_route(
                    path="/metabow/audio",
                    data_type="pcm",
                    sample_value=len(decoded_data['pcm_data'])
                )
                
                # Handle recording if active
                if self.audio_recorder.recording:
                    self.audio_recorder.write_frames(decoded_data['pcm_data'])

            # Handle Motion Data
            if decoded_data['motion_data'] and len(decoded_data['motion_data']) == 13:
                motion_paths = [
                    "quaternion_i", "quaternion_j", "quaternion_k", "quaternion_r",
                    "accelerometer_x", "accelerometer_y", "accelerometer_z",
                    "gyroscope_x", "gyroscope_y", "gyroscope_z",
                    "magnetometer_x", "magnetometer_y", "magnetometer_z"
                ]
                
                # Register individual motion routes
                for idx, path_suffix in enumerate(motion_paths):
                    self.route_manager.update_route(
                        path=f"/metabow/motion/{path_suffix}",
                        data_type="float",
                        sample_value=decoded_data['motion_data'][idx]
                    )

            # Route data through OSC destinations
            for dest in self.osc_destinations:
                # Handle individual routes
                for route in dest.routes:
                    if not route.enabled:
                        continue
                        
                    effective_path = route.effective_path
                        
                    # Handle audio data
                    if route.path == "/metabow/audio" and decoded_data['pcm_data']:
                        dest.client.send_message(effective_path, decoded_data['pcm_data'])
                    
                    # Handle individual motion data
                    if route.path.startswith("/metabow/motion/") and decoded_data['motion_data']:
                        try:
                            motion_component = route.path.split('/')[-1]
                            motion_idx = motion_paths.index(motion_component)
                            dest.client.send_message(effective_path, decoded_data['motion_data'][motion_idx])
                        except Exception as e:
                            print(f"Error sending motion data: {e}")

                # Handle bundles
                for bundle in dest.bundles:
                    if not bundle.enabled:
                        continue
                        
                    # Get combined values for all routes in the bundle
                    bundle_values = dest.get_bundle_values(bundle, decoded_data)
                    
                    # Send combined message if we have values
                    if bundle_values:
                        dest.send_bundle_message(bundle, bundle_values)
                            
        except Exception as e:
            print(f"Notification handling error: {e}")
            import traceback
            traceback.print_exc()

    def decode_data(self, data):
        try:
            imu_data_len = 13 * 4
            flag_size = 1
            pcm_chunk_size = 2
    
            flag = data[-1]
            data_len = len(data)
    
            print(f"Total data length: {data_len}")
            print(f"Flag: {flag}")
            print(f"IMU data length: {imu_data_len}")
            print(f"Flag size: {flag_size}")
    
            pcm_data = []
            for i in range(0, data_len - imu_data_len - flag_size, pcm_chunk_size):
                pcm_value = int.from_bytes(
                    data[i:i+pcm_chunk_size],
                    byteorder='little',
                    signed=True
                )
                pcm_data.append(pcm_value)
    
            # Enhanced PCM data diagnostics
            if pcm_data:
                print("PCM Data Statistics:")
                print(f"  Total samples: {len(pcm_data)}")
                print(f"  Min value: {min(pcm_data)}")
                print(f"  Max value: {max(pcm_data)}")
                print(f"  Mean value: {sum(pcm_data) / len(pcm_data):.2f}")
            
                # Range checking
                sample_range = max(pcm_data) - min(pcm_data)
                print(f"  Sample range: {sample_range}")
            
                # RMS calculation
                rms = np.sqrt(sum(x*x for x in pcm_data) / len(pcm_data))
                print(f"  RMS value: {rms:.2f}")
    
            print(f"First 10 PCM samples: {pcm_data[:10]}")
    
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
            print(f"Detailed data decoding error: {e}")
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
        if askyesno("Exit", "Do you want to quit the application?"):
            if self.audio_recorder.recording:
                self.toggle_recording()
        
            # Cleanup audio resources
            try:
                self.audio_recorder.stop_recording()
            except Exception as e:
                print(f"Error during audio cleanup: {e}")
        
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
        
        # PyAudio setup
        self.pya = pyaudio.PyAudio()
        self.stream = None
        
        # Initialize virtual stream as None
        self.virtual_stream = None
        self.virtual_output_enabled = False
        
        # Recording state
        self.recording = False
        self.wave_file = None
        self.filename = None
        
        # Real-time statistics
        self.peak_level = 0
        self.noise_floor = 0
        
        # Latency tracking
        self.processing_times = []
        self.max_processing_times = 100
        self.avg_latency = 0
        self.peak_latency = 0
        self.buffer_latency = 0

        # After existing initialization
        self.initialize_virtual_audio_device()

    def toggle_virtual_output(self):
        """Toggle virtual audio output with VB-Cable compatibility"""
        try:
            import sounddevice as sd
            import numpy as np
            
            # Log all current devices for debugging
            print("\n--- Available Audio Devices ---")
            devices = sd.query_devices()
            for i, device in enumerate(devices):
                print(f"Device {i}: {device['name']}")
                print(f"  Max Input Channels: {device['max_input_channels']}")
                print(f"  Max Output Channels: {device['max_output_channels']}")
                print(f"  Default Samplerate: {device['default_samplerate']}")
            
            # If not currently enabled, try to create and start a stream
            if not self.virtual_output_enabled:
                try:
                    # Find VB-Cable device index
                    device_index = None
                    for i, device in enumerate(devices):
                        if 'VB-Cable' in device['name'] and device['max_output_channels'] > 0:
                            device_index = i
                            print(f"Found VB-Cable at index {i}")
                            break
                    
                    if device_index is None:
                        print("ERROR: No VB-Cable output device found")
                        return False
                    
                    # Get device details
                    selected_device = devices[device_index]
                    print(f"Selected VB-Cable Device Details:")
                    print(f"  Name: {selected_device['name']}")
                    print(f"  Max Output Channels: {selected_device['max_output_channels']}")
                    print(f"  Default Samplerate: {selected_device['default_samplerate']}")
                    
                    # Create output stream with VB-Cable's minimum settings
                    self.virtual_stream = sd.OutputStream(
                        device=device_index,
                        samplerate=44100,       # VB-Cable minimum samplerate
                        channels=1,             # Mono (minimum channels)
                        dtype='float32'         # 32-bit float (required)
                    )
                    
                    # Start the stream
                    self.virtual_stream.start()
                    
                    self.virtual_output_enabled = True
                    print(f"Virtual output stream created successfully on VB-Cable (device {device_index})")
                    print(f"Converting from {self.framerate}Hz to 44100Hz for VB-Cable compatibility")
                    return True
                    
                except Exception as e:
                    print(f"ERROR in virtual output creation: {e}")
                    import traceback
                    traceback.print_exc()
                    return False
            
            # If already enabled, stop the stream
            else:
                if self.virtual_stream:
                    self.virtual_stream.stop()
                    self.virtual_stream.close()
                    self.virtual_stream = None
                self.virtual_output_enabled = False
                print("Virtual output stream stopped")
                return True
        
        except Exception as e:
            print(f"ERROR in toggle_virtual_output: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def start_recording(self, directory=None):
        """Start recording with basic WAV file setup"""
        if directory is None:
            directory = os.path.expanduser("~/Documents")
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = os.path.join(directory, f"metabow_recording_{timestamp}.wav")
        
        # Open WAV file for writing
        self.wave_file = wave.open(self.filename, 'wb')
        self.wave_file.setnchannels(self.channels)
        self.wave_file.setsampwidth(self.sample_width)
        self.wave_file.setframerate(self.framerate)
        
        # Reset recording state
        self.recording = True
        
        print(f"Started recording to {self.filename}")
        return self.filename

    def write_frames(self, pcm_data):
        """Process incoming audio data for VB-Cable on macOS"""
        if pcm_data:
            try:
                # Convert to numpy array for processing
                samples = np.array(pcm_data, dtype=np.int16)
                
                # Handle virtual output streaming (if enabled)
                if self.virtual_output_enabled and self.virtual_stream:
                    try:
                        # 1. Convert from int16 to float32 with explicit casting
                        float_samples = (samples.astype(np.float32) / 32767.0).astype(np.float32)
                        
                        # Increase gain to compensate for VB-Cable characteristics
                        gain = 4.0  # Adjust as needed
                        float_samples = np.clip(float_samples * gain, -0.95, 0.95)
                        
                        # 2. Adapt to VB-Cable format
                        # Get saved output settings
                        output_rate = getattr(self, 'output_samplerate', 44100)
                        output_channels = getattr(self, 'output_channels', 2)
                        
                        # Resample if needed
                        if self.framerate != output_rate:
                            # Resample using linear interpolation
                            ratio = output_rate / self.framerate
                            num_output_samples = int(len(float_samples) * ratio)
                            resampled = np.zeros(num_output_samples, dtype=np.float32)
                            
                            for i in range(num_output_samples):
                                src_idx_float = i / ratio
                                src_idx_int = int(src_idx_float)
                                fraction = src_idx_float - src_idx_int
                                
                                if src_idx_int < len(float_samples) - 1:
                                    resampled[i] = float_samples[src_idx_int] * (1 - fraction) + \
                                                float_samples[src_idx_int + 1] * fraction
                                else:
                                    resampled[i] = float_samples[-1]
                        else:
                            resampled = float_samples
                        
                        # Convert to stereo if needed
                        if output_channels == 2 and len(resampled.shape) == 1:
                            stereo_samples = np.column_stack((resampled, resampled))
                        else:
                            stereo_samples = resampled
                        
                        # Log diagnostics occasionally
                        if np.random.random() < 0.01:  # ~1% of frames
                            print(f"Audio frame: {len(pcm_data)} → {len(stereo_samples)} samples")
                            print(f"  Level: min={np.min(stereo_samples):.3f}, max={np.max(stereo_samples):.3f}")
                            print(f"  RMS: {np.sqrt(np.mean(stereo_samples**2)):.3f}")
                        
                        # Write to VB-Cable stream
                        self.virtual_stream.write(stereo_samples)
                        
                    except Exception as e:
                        print(f"Error streaming to VB-Cable: {e}")
                        import traceback
                        traceback.print_exc()
                
                # Handle recording separately (unchanged)
                if self.recording and self.wave_file:
                    self.wave_file.writeframes(samples.tobytes())
                
                # Update metrics (unchanged)
                current_peak = np.max(np.abs(samples))
                self.peak_level = max(self.peak_level * 0.95, current_peak)
                self.noise_floor = np.percentile(np.abs(samples), 15)
                
                # Calculate latency (unchanged)
                start_time = time.time()
                processing_time = (time.time() - start_time) * 1000
                self.processing_times.append(processing_time)
                if len(self.processing_times) > self.max_processing_times:
                    self.processing_times.pop(0)
                
                self.avg_latency = np.mean(self.processing_times) if self.processing_times else 0
                self.peak_latency = np.max(self.processing_times) if self.processing_times else 0
                self.buffer_latency = (len(pcm_data) / self.framerate) * 1000
                
            except Exception as e:
                print(f"Error processing audio: {e}")
                import traceback
                traceback.print_exc()

    def stop_recording(self):
        """Stop recording and close the WAV file"""
        if self.recording:
            self.recording = False
            if self.wave_file:
                self.wave_file.close()
                self.wave_file = None
            return self.filename
        return None

    def cleanup(self):
        """Clean up resources"""
        if self.virtual_stream:
            self.virtual_stream.stop()
            self.virtual_stream.close()
        if self.wave_file:
            self.wave_file.close()
        if self.pya:
            self.pya.terminate()

    def initialize_virtual_audio_device(self):
        """
        Initialize virtual audio device management using VB-Cable
        """
        self.device_manager = VirtualAudioDeviceManager()
        
        # Check for VB-Cable on startup
        device_result = self.device_manager.create_virtual_device()
        
        if device_result['success']:
            print(f"VB-Cable virtual audio device detected: {device_result['device_name']}")
            print(f"Ready to stream audio at {device_result['sample_rate']}Hz with {device_result['channels']} channels")
        else:
            print(f"VB-Cable not detected: {device_result.get('error', 'Unknown error')}")
            if 'instructions' in device_result:
                print(device_result['instructions'])

class VirtualAudioDeviceManager:
    def __init__(self):
        self.os_name = platform.system()
        self.logger = logging.getLogger(__name__)
        logging.basicConfig(level=logging.INFO)
        
    def create_virtual_device(self):
        """
        Check for VB-Cable virtual audio device
        """
        try:
            import sounddevice as sd
            
            # Check if VB-Cable is available
            devices = sd.query_devices()
            vb_cable_devices = []
            
            for i, device in enumerate(devices):
                if 'VB-Cable' in device['name']:
                    vb_cable_devices.append((i, device))
                    
            if vb_cable_devices:
                idx, device = vb_cable_devices[0]
                self.logger.info(f"Found VB-Cable: {device['name']} at index {idx}")
                return {
                    'success': True,
                    'device_name': device['name'],
                    'device_index': idx,
                    'channels': device['max_output_channels'],
                    'sample_rate': device['default_samplerate'],
                    'platform': self.os_name
                }
            else:
                self.logger.warning("VB-Cable not found. Please install VB-Cable.")
                return {
                    'success': False,
                    'error': "VB-Cable not found",
                    'instructions': "Please install VB-Cable from https://vb-audio.com/Cable/"
                }
                
        except Exception as e:
            self.logger.error(f"Error checking for VB-Cable: {e}")
            return {
                'success': False,
                'error': str(e)
            }
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
        import traceback
        traceback.print_exc()
    finally:
        loop.close()

if __name__ == '__main__':
    main()
