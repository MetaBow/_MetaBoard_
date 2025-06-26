#!/usr/bin/env python3
"""
MetaBow Micro - Aggressive Anti-Stall Version
Implements multiple strategies to prevent device stalls:
- Flow control and throttling
- Frequent preventive resets
- Back-pressure management
- Connection parameter optimization

EXACT MINIMAL FIXES:
1. Fixed incomplete soft_reset() return statement (line 388)
2. Added missing send_reset_command() method after pause_and_clear()
3. Added missing get_avg_processing_time() method in FlowControlledConnection
4. Added missing reset_in_progress initialization in AntiStallApp.__init__()
"""

import sys
import asyncio
import time
import struct
import os
import threading
from datetime import datetime
import weakref
import gc

# BLE imports
from bleak import BleakClient, BleakScanner

# OSC and file output
try:
    from pythonosc import udp_client
    OSC_AVAILABLE = True
except ImportError:
    OSC_AVAILABLE = False
    print("WARNING: python-osc not available, OSC features disabled")

# UI imports
import tkinter as tk
from tkinter import messagebox

# Global event loop for BLE operations
BLE_LOOP = None
BLE_THREAD = None

def setup_ble_loop():
    """Setup dedicated event loop for BLE operations"""
    global BLE_LOOP, BLE_THREAD
    
    def run_ble_loop():
        global BLE_LOOP
        BLE_LOOP = asyncio.new_event_loop()
        asyncio.set_event_loop(BLE_LOOP)
        try:
            BLE_LOOP.run_forever()
        except:
            pass
    
    if BLE_THREAD is None or not BLE_THREAD.is_alive():
        BLE_THREAD = threading.Thread(target=run_ble_loop, daemon=True)
        BLE_THREAD.start()
        time.sleep(0.1)

def run_in_ble_loop(coro):
    """Run coroutine in the dedicated BLE loop"""
    global BLE_LOOP
    if BLE_LOOP is None:
        setup_ble_loop()
    
    future = asyncio.run_coroutine_threadsafe(coro, BLE_LOOP)
    return future.result(timeout=30)

class FlowControlledConnection:
    """Connection with aggressive flow control and throttling"""
    def __init__(self, client, rx_char, tx_char, log_callback):
        self.client = client
        self.rx_char = rx_char
        self.tx_char = tx_char
        self.log_callback = log_callback
        self.message_count = 0
        self.last_data_time = time.time()
        self._active = True
        
        # Aggressive flow control settings
        self.throttle_enabled = True
        self.throttle_delay = 0.0001  # 100 microseconds between packets
        self.last_packet_time = 0
        
        # Small buffers to prevent buildup
        self.data_queue = []
        self.max_queue_size = 50  # Very small queue
        self.queue_lock = threading.Lock()
        
        # Back-pressure detection
        self.processing_time_samples = []
        self.max_samples = 100
        self.avg_processing_time = 0
        self.slow_processing_threshold = 0.001  # 1ms per packet
        
        # Memory management
        self.last_gc_time = time.time()
        self.gc_interval = 10  # Very frequent GC
        self.bytes_processed = 0
        
        # Performance monitoring
        self.packet_rate_window = []
        self.rate_window_size = 1000
        self.last_rate_log = time.time()
        
        # File output with optional recording
        self.audio_recording_enabled = False  # Start with recording disabled
        self.binary_file = None
        
        self.setup_osc()
        
        # Start background processor with verification
        self.processor_active = True
        self.processor_thread = threading.Thread(target=self._throttled_processor, daemon=True)
        self.processor_thread.start()
        print(f"DEBUG: Background processor thread started, is_alive: {self.processor_thread.is_alive()}")

    def setup_file_output(self):
        """Setup file output when recording is enabled"""
        if sys.platform == "darwin":
            output_dir = os.path.expanduser("~/Documents/MetaBow_Data")
        else:
            output_dir = os.path.expanduser("~/MetaBow_Data")
        
        os.makedirs(output_dir, exist_ok=True)
        pcm_file_path = os.path.join(output_dir, 'pcm_' + str(int(time.time())) + '.bin')
        
        try:
            # Open with smaller buffer
            self.binary_file = open(pcm_file_path, 'wb', buffering=8192)
            self.log_callback("Audio recording started: " + pcm_file_path)
        except Exception as e:
            self.log_callback("File creation failed: " + str(e))
            self.binary_file = None

    def stop_file_output(self):
        """Stop file output and close file"""
        if self.binary_file:
            try:
                self.binary_file.flush()
                self.binary_file.close()
                self.log_callback("Audio recording stopped and file saved")
            except:
                pass
            self.binary_file = None

    def toggle_audio_recording(self):
        """Toggle audio recording on/off"""
        self.audio_recording_enabled = not self.audio_recording_enabled
        
        if self.audio_recording_enabled:
            self.setup_file_output()
        else:
            self.stop_file_output()

    def setup_osc(self):
        """Setup OSC with error handling"""
        if OSC_AVAILABLE:
            try:
                self.osc = udp_client.SimpleUDPClient("127.0.0.1", 8888)
                self.log_callback("OSC client connected to port 8888")
            except Exception as e:
                self.log_callback("OSC setup failed: " + str(e))
                self.osc = None
        else:
            self.osc = None

    async def start(self):
        """Start notifications with enhanced debugging"""
        if self._active:
            try:
                print("DEBUG: About to start notifications...")
                await self.client.start_notify(self.tx_char, self.data_callback)
                print("DEBUG: start_notify() call completed successfully")
                
                # Test if notifications are actually working by waiting a bit
                await asyncio.sleep(2)
                print(f"DEBUG: After 2 seconds, callback count: {getattr(self, '_callback_count', 0)}")
                
                self.log_callback('Notifications enabled with flow control')
            except Exception as e:
                print(f"DEBUG: start_notify() failed with error: {e}")
                self.log_callback(f"Failed to start notifications: {e}")
                raise

    def data_callback(self, sender, data):
        """Ultra-lightweight callback with throttling"""
        if not self._active:
            return
            
        try:
            current_time = time.time()
            
            # Implement throttling at callback level
            if self.throttle_enabled:
                time_since_last = current_time - self.last_packet_time
                if time_since_last < self.throttle_delay:
                    # Drop packet if we're going too fast
                    return
                self.last_packet_time = current_time
            
            # Very aggressive queue management
            with self.queue_lock:
                if len(self.data_queue) >= self.max_queue_size:
                    # Drop multiple old packets
                    dropped = min(10, len(self.data_queue) // 2)
                    self.data_queue = self.data_queue[dropped:]
                
                self.data_queue.append((data, current_time))
                
        except Exception as e:
            self.log_callback(f"Callback error: {e}")

    def _throttled_processor(self):
        """Background processor with aggressive throttling"""
        print("DEBUG: Processor thread started")
        while self.processor_active and self._active:
            try:
                start_time = time.time()
                
                # Process data in small batches
                data_to_process = []
                with self.queue_lock:
                    if self.data_queue:
                        # Very small batches
                        batch_size = min(5, len(self.data_queue))
                        data_to_process = self.data_queue[:batch_size]
                        self.data_queue = self.data_queue[batch_size:]
                
                # Debug output every 10 iterations
                if hasattr(self, '_processor_iterations'):
                    self._processor_iterations += 1
                else:
                    self._processor_iterations = 0
                
                if self._processor_iterations % 1000 == 0 and len(data_to_process) > 0:
                    print(f"DEBUG Processor: Processing batch of {len(data_to_process)}, total messages: {self.message_count}")
                
                # Process the batch
                for data, timestamp in data_to_process:
                    self._process_single_packet(data, timestamp)
                
                # Track processing time for back-pressure
                processing_time = time.time() - start_time
                self._update_processing_stats(processing_time)
                
                # Adaptive throttling based on processing time
                if self.avg_processing_time > self.slow_processing_threshold:
                    self.throttle_delay = min(0.001, self.throttle_delay * 1.1)  # Increase throttling
                else:
                    self.throttle_delay = max(0.00005, self.throttle_delay * 0.99)  # Decrease throttling
                
                # Memory management
                self._periodic_maintenance()
                
                # Throttled sleep
                sleep_time = max(0.0001, self.throttle_delay)
                time.sleep(sleep_time)
                
            except Exception as e:
                self.log_callback(f"Processor error: {e}")
                print(f"DEBUG Processor Error: {e}")
                time.sleep(0.01)
        
        print("DEBUG: Processor thread ended")

    def _process_single_packet(self, data, timestamp):
        """Optimized packet processing"""
        try:
            self.message_count += 1
            self.last_data_time = timestamp
            self.bytes_processed += len(data)
            
            # Log progress less frequently but with more detail
            if self.message_count % 2000 == 0:
                rate = self._calculate_packet_rate()
                queue_size = len(self.data_queue)
                self.log_callback(f"Message {self.message_count}: {rate:.1f} pkt/s, throttle: {self.throttle_delay*1000:.2f}ms, queue: {queue_size}")
            elif self.message_count % 500 == 0:
                # Quick status check every 500 messages
                print(f"Quick: {self.message_count} msgs, throttle: {self.throttle_delay*1000:.2f}ms")
            
            # Optimized data processing
            data_len = len(data)
            if data_len < 54:  # Minimum expected size
                return
                
            imu_data_len = 52  # 13 * 4 bytes
            flag_size = 1
            
            # Write audio data only if recording is enabled
            if self.audio_recording_enabled and self.binary_file and not self.binary_file.closed:
                try:
                    audio_end = data_len - imu_data_len - flag_size
                    if audio_end > 0:
                        # Write every other byte more efficiently
                        audio_data = data[0:audio_end:2]
                        self.binary_file.write(audio_data)
                        
                        # More frequent flushing to prevent buffer buildup
                        if self.message_count % 100 == 0:
                            self.binary_file.flush()
                            
                except Exception as e:
                    self.log_callback(f"File write error: {e}")
            
            # Optimized motion data processing
            if data_len > 0 and data[-1] == 1:
                try:
                    motion_start = data_len - imu_data_len - flag_size
                    if motion_start >= 0 and motion_start + imu_data_len <= data_len - flag_size:
                        motion_data = data[motion_start:motion_start + imu_data_len]
                        if len(motion_data) == imu_data_len:
                            motion_floats = list(struct.unpack('13f', motion_data))
                            
                            if self.osc:
                                self.osc.send_message("/motion", motion_floats)
                                
                except Exception as e:
                    pass  # Silently ignore motion errors to reduce overhead
                    
        except Exception as e:
            self.log_callback(f"Packet processing error: {e}")

    def _update_processing_stats(self, processing_time):
        """Update processing time statistics"""
        self.processing_time_samples.append(processing_time)
        if len(self.processing_time_samples) > self.max_samples:
            self.processing_time_samples.pop(0)
        
        self.avg_processing_time = sum(self.processing_time_samples) / len(self.processing_time_samples)

    def _calculate_packet_rate(self):
        """Calculate recent packet rate"""
        current_time = time.time()
        self.packet_rate_window.append(current_time)
        
        # Keep only recent timestamps
        cutoff = current_time - 1.0  # Last second
        self.packet_rate_window = [t for t in self.packet_rate_window if t > cutoff]
        
        return len(self.packet_rate_window)

    def _periodic_maintenance(self):
        """Aggressive maintenance"""
        current_time = time.time()
        
        # Very frequent garbage collection
        if current_time - self.last_gc_time > self.gc_interval:
            gc.collect()
            self.last_gc_time = current_time

    async def soft_reset(self):
        """Ultra-gentle reset without touching notifications"""
        try:
            if not self.client or not self.client.is_connected:
                return "Error: Not connected"
            
            self.log_callback("Attempting gentle reset (memory/queue only)...")
            
            # DON'T touch notifications - just reset internal state
            # Clear any queued data
            with self.queue_lock:
                dropped_count = len(self.data_queue)
                self.data_queue.clear()
                if dropped_count > 0:
                    self.log_callback(f"Cleared {dropped_count} queued packets")
            
            # Force garbage collection
            gc.collect()
            
            # Reset processing statistics
            self.processing_time_samples.clear()
            self.avg_processing_time = 0
            self.packet_rate_window.clear()
            
            # Reset counters but keep notifications and connection alive
            old_count = self.message_count
            self.message_count = 0
            self.last_data_time = time.time()
            self.bytes_processed = 0
            
            # Reset throttling to initial state
            self.throttle_delay = 0.0001
            
            self.log_callback(f"Gentle reset completed - cleared {old_count} messages from memory")
            return "Success"
            
        except Exception as e:
            self.log_callback(f"Gentle reset failed: {e}")
            return f"Error: {e}"  # FIX 1: Added missing return statement
            
    async def pause_and_clear(self):
        """Pause processing briefly and clear buffers"""
        try:
            if not self.client or not self.client.is_connected:
                return "Error: Not connected"
            
            self.log_callback("Pausing processing to clear buffers...")
            
            # Temporarily pause processing by increasing throttle dramatically
            old_throttle = self.throttle_delay
            self.throttle_delay = 0.01  # 10ms delay = very slow processing
            
            # Wait for queue to drain
            await asyncio.sleep(2)
            
            # Clear any remaining queued data
            with self.queue_lock:
                dropped_count = len(self.data_queue)
                self.data_queue.clear()
                if dropped_count > 0:
                    self.log_callback(f"Cleared {dropped_count} remaining packets")
            
            # Force garbage collection
            gc.collect()
            
            # Reset throttling
            self.throttle_delay = old_throttle
            
            self.log_callback("Processing pause completed - buffers cleared")
            return "Success"
            
        except Exception as e:
            self.log_callback(f"Pause and clear failed: {e}")
            return f"Error: {e}"

    # FIX 2: Add missing send_reset_command method
    async def send_reset_command(self):
        """Send reset command to device via RX characteristic"""
        try:
            if not self.client or not self.client.is_connected:
                return "Error: Not connected"
            
            # Try common reset commands
            reset_commands = [
                b'RESET\n',
                b'RST\n', 
                b'\x00\x01',  # Binary reset
                b'R',
            ]
            
            for cmd in reset_commands:
                try:
                    await self.client.write_gatt_char(self.rx_char, cmd)
                    self.log_callback(f"Sent reset command: {cmd}")
                    await asyncio.sleep(0.1)
                except:
                    continue
            
            return "Success"
            
        except Exception as e:
            return f"Error: {e}"

    def close(self):
        """Clean shutdown"""
        self._active = False
        self.processor_active = False
        
        # Wait for processor
        if hasattr(self, 'processor_thread') and self.processor_thread.is_alive():
            self.processor_thread.join(timeout=1)
        
        # Clear queue
        with self.queue_lock:
            self.data_queue.clear()
        
        # Clear any remaining data and close file if recording
        if hasattr(self, 'binary_file') and self.binary_file:
            try:
                self.binary_file.flush()
                self.binary_file.close()
                self.log_callback("Audio recording stopped and file saved")
            except:
                pass
        
        gc.collect()

    def get_message_count(self):
        return self.message_count
    
    def time_since_last_data(self):
        return time.time() - self.last_data_time
    
    def get_queue_size(self):
        with self.queue_lock:
            return len(self.data_queue)
    
    def get_throttle_delay(self):
        return self.throttle_delay
    
    # FIX 3: Add missing get_avg_processing_time method
    def get_avg_processing_time(self):
        return self.avg_processing_time
    
    async def force_disconnect(self):
        """Force disconnect with cleanup"""
        try:
            if self.client and self.client.is_connected:
                try:
                    await self.client.stop_notify(self.tx_char)
                    await asyncio.sleep(0.1)
                except:
                    pass
                
                await self.client.disconnect()
                await asyncio.sleep(0.5)
        except Exception as e:
            print(f"Force disconnect error: {e}")


class AntiStallApp:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("MetaBow Micro - Anti-Stall Version")
        self.root.geometry("550x600")
        
        # State management
        self.client = None
        self.connection = None
        self.is_connected = False
        self.devices = []
        self.selected_device = None
        self._shutting_down = False
        
        # FIX 4: Add missing reset_in_progress initialization
        self.reset_in_progress = False
        
        # Aggressive anti-stall strategy
        self.connection_start_time = None
        self.auto_reconnect_enabled = True
        self.reconnect_attempts = 0
        self.max_reconnect_attempts = 2  # Fast cycling
        self.stall_timeout = 5  # Very short timeout
        self.reconnecting = False
        
        # Much more conservative thresholds to prevent device stress
        self.reset_thresholds = [50000, 75000, 100000]  # Much higher thresholds
        self.current_threshold_index = 0
        self.last_reset_count = 0
        
        # Adaptive throttling - increase throttling as we approach thresholds
        self.adaptive_throttling = True
        self.base_throttle_delay = 0.0001  # Start with very light throttling
        self.max_throttle_delay = 0.002    # Maximum 2ms throttling
        
        # Device management
        self.device_names = ["metabow", "metabow_ota"]
        
        # Setup BLE loop
        setup_ble_loop()
        
        # Create UI
        self.create_ui()
        
        # Handle window close
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Start status updates
        self.update_status()

    def try_gentle_reset(self):
        """Try ultra-gentle reset that doesn't touch notifications"""
        if not self.is_connected or not self.connection:
            return
        
        # IMPORTANT: Set reset_in_progress BEFORE starting reset to prevent monitor interference
        self.reset_in_progress = True
        self._reset_start_time = time.time()  # Track reset duration
        # Give monitoring thread a moment to see the flag
        time.sleep(0.1)
        self.log("Trying gentle reset (memory only, no notification changes)...")
        
        def run_gentle_reset():
            try:
                result = run_in_ble_loop(self.connection.soft_reset())
                self.root.after(0, self.handle_gentle_reset_result, result)
            except Exception as e:
                self.root.after(0, self.handle_gentle_reset_result, f"Error: {e}")
        
        threading.Thread(target=run_gentle_reset, daemon=True).start()

    def handle_gentle_reset_result(self, result):
        """Handle gentle reset result"""
        # Resume monitoring after reset
        self.reset_in_progress = False
        
        if result == "Success":
            self.log("Gentle reset successful - connection maintained, memory cleared!")
            # Reset threshold tracking for this session
            self.last_reset_count = 0  # Allow immediate next reset if needed
            if self.connection:
                # Don't reset the message count since we want to track cumulative
                pass
            # IMPORTANT: Force immediate UI update after reset
            self.root.after(0, self.force_ui_update)
        else:
            self.log(f"Gentle reset failed: {result} - trying pause & clear...")
            self.try_pause_clear()

    def force_ui_update(self):
        """Force an immediate UI update after reset"""
        try:
            if self.connection and self.is_connected and not self.reset_in_progress:
                # Get fresh stats immediately
                count = self.connection.get_message_count()
                queue_size = self.connection.get_queue_size()
                throttle_delay = self.connection.get_throttle_delay() * 1000
                proc_time = self.connection.get_avg_processing_time() * 1000
                
                # Update UI immediately
                self.data_label.config(text=f"Messages: {count}")
                self.queue_label.config(text=f"Queue: {queue_size}")
                self.throttle_label.config(text=f"Throttle: {throttle_delay:.2f}ms")
                self.processing_label.config(text=f"Proc Time: {proc_time:.2f}ms")
                
                # Update reset countdown
                current_threshold = self.get_current_threshold()
                if current_threshold and count > 0:
                    remaining = max(0, current_threshold - count)
                    self.reset_label.config(text=f"Next Reset: {remaining} msgs")
                else:
                    self.reset_label.config(text="Next Reset: 50000")
                
                print(f"DEBUG: Forced UI update after reset - Count: {count}, Queue: {queue_size}")
        except Exception as e:
            print(f"DEBUG: Error in force_ui_update: {e}")

    def try_pause_clear(self):
        """Try pause and clear approach"""
        if not self.is_connected or not self.connection:
            return
            
        self.log("Trying pause & clear (slow processing briefly)...")
        
        def run_pause_clear():
            try:
                result = run_in_ble_loop(self.connection.pause_and_clear())
                self.root.after(0, self.handle_pause_clear_result, result)
            except Exception as e:
                self.root.after(0, self.handle_pause_clear_result, f"Error: {e}")
        
        threading.Thread(target=run_pause_clear, daemon=True).start()

    def handle_pause_clear_result(self, result):
        """Handle pause clear result"""
        if result == "Success":
            self.log("Pause & clear successful - buffers cleared!")
        else:
            self.log(f"Pause & clear failed: {result} - trying emergency reset...")
            self.emergency_reset()

    def send_device_reset_command(self):
        """Send reset command to device"""
        if not self.is_connected:
            return
            
        self.log("Sending reset command to device...")
        
        def run_command_reset():
            try:
                if self.connection:
                    result = run_in_ble_loop(self.connection.send_reset_command())
                    self.root.after(0, self.handle_command_reset_result, result)
                else:
                    self.root.after(0, self.log, "No connection available for reset command")
            except Exception as e:
                self.root.after(0, self.log, f"Reset command error: {e}")
        
        threading.Thread(target=run_command_reset, daemon=True).start()

    def handle_command_reset_result(self, result):
        """Handle command reset result"""
        if result == "Success":
            self.log("Reset command sent successfully")
        else:
            self.log(f"Reset command failed: {result}")

    def emergency_reset(self):
        """Emergency reset with longer recovery time"""
        if not self.is_connected:
            return
            
        self.log("EMERGENCY RESET - forcing complete device recovery...")
        
        def run_emergency_reset():
            try:
                # More aggressive disconnect with proper error handling
                if self.connection:
                    self.connection.close()
                
                if self.client:
                    try:
                        if self.connection:
                            run_in_ble_loop(self.connection.force_disconnect())
                        else:
                            # Fallback if connection is None
                            run_in_ble_loop(self.client.disconnect())
                    except Exception as disconnect_error:
                        self.root.after(0, self.log, f"Disconnect warning: {disconnect_error}")
                
                # Clear references
                self.connection = None
                self.client = None
                
                # Much longer wait for device to fully recover
                self.log("Waiting 10 seconds for device recovery...")
                time.sleep(10)
                
                # Force garbage collection
                gc.collect()
                
                # Reset threshold tracking
                self.current_threshold_index = 0
                self.last_reset_count = 0
                
                # Reconnect
                if self.selected_device:
                    result = run_in_ble_loop(self.async_connect(self.selected_device))
                    self.root.after(0, self.handle_reset_result, result)
                else:
                    self.root.after(0, self.handle_disconnect)
                    
            except Exception as e:
                self.root.after(0, self.log, f"Emergency reset error: {e}")
                self.root.after(0, self.handle_disconnect)
        
        threading.Thread(target=run_emergency_reset, daemon=True).start()

    def handle_reset_result(self, result):
        """Handle reset result"""
        if result == "Success":
            self.log("Reset successful - connection restored!")
            self.handle_connect_result(result)
        else:
            self.log(f"Reset failed: {result}")
            self.handle_disconnect()

    def toggle_audio_recording_ui(self):
        """Handle UI toggle for audio recording"""
        if not self.is_connected or not self.connection:
            return
            
        enabled = self.audio_recording_var.get()
        self.connection.audio_recording_enabled = enabled
        
        if enabled:
            self.connection.setup_file_output()
            self.log("Audio recording enabled")
        else:
            self.connection.stop_file_output()
            self.log("Audio recording disabled")

    def try_soft_reset(self):
        """Try soft reset first, fallback to hard reset"""
        if not self.is_connected or not self.connection:
            return
            
        self.log("Trying soft reset (notifications restart)...")
        
        def run_soft_reset():
            try:
                result = run_in_ble_loop(self.connection.soft_reset())
                self.root.after(0, self.handle_soft_reset_result, result)
            except Exception as e:
                self.root.after(0, self.handle_soft_reset_result, f"Error: {e}")
        
        threading.Thread(target=run_soft_reset, daemon=True).start()

    def handle_soft_reset_result(self, result):
        """Handle soft reset result"""
        if result == "Success":
            self.log("Soft reset successful - connection maintained!")
            # Reset threshold tracking for this session
            self.last_reset_count = self.connection.get_message_count() if self.connection else 0
            # Reset the message counter in connection to start fresh
            if self.connection:
                self.connection.message_count = 0
                self.connection.last_data_time = time.time()
        else:
            self.log(f"Soft reset failed: {result} - trying hard reset...")
            self.emergency_reset()

    def get_current_threshold(self):
        """Get current reset threshold"""
        if self.current_threshold_index < len(self.reset_thresholds):
            return self.reset_thresholds[self.current_threshold_index]
        return None

    def start_monitoring(self):
        """Enhanced monitoring with multi-stage reset"""
        def monitor():
            try:
                while self.is_connected and not self._shutting_down:
                    # Skip monitoring if reset is in progress
                    if self.reset_in_progress:
                        print("DEBUG: Monitoring paused - reset in progress")
                        time.sleep(0.5)
                        continue
                    
                    # IMPORTANT: Double-check reset_in_progress before any disconnection logic
                    if self.reset_in_progress:
                        time.sleep(0.5)
                        continue
                    
                    # Check if client is still connected at BLE level
                    if self.client and not self.client.is_connected:
                        # FINAL CHECK: Don't disconnect if reset is in progress
                        if not self.reset_in_progress:
                            self.log("BLE client disconnected - stopping monitor")
                            break
                        else:
                            print("DEBUG: BLE client disconnected during reset - ignoring")
                            time.sleep(0.5)
                            continue
                    
                    if self.connection:
                        message_count = self.connection.get_message_count()
                        time_since_data = self.connection.time_since_last_data()
                        
                        # Multi-stage preventive reset with better logic and debug
                        current_threshold = self.get_current_threshold()
                        if message_count % 5000 == 0:  # Debug every 5k messages
                            print(f"DEBUG Reset Check: Count={message_count}, Threshold={current_threshold}")
                            print(f"  last_reset_count={self.last_reset_count}")
                            print(f"  auto_reconnect_enabled={self.auto_reconnect_var.get()}")
                            print(f"  reconnecting={self.reconnecting}")
                            print(f"  reset_in_progress={self.reset_in_progress}")
                            print(f"  time_since_data={time_since_data:.1f}s (stall_timeout={self.stall_timeout}s)")
                        
                        # Also debug when message count doesn't change for a while
                        if not hasattr(self, '_last_message_count'):
                            self._last_message_count = message_count
                            self._stuck_count_start = time.time()
                        elif message_count == self._last_message_count:
                            stuck_time = time.time() - self._stuck_count_start
                            if stuck_time > 10:  # Debug if stuck for more than 10 seconds
                                print(f"DEBUG: Message count stuck at {message_count} for {stuck_time:.1f}s")
                                print(f"  time_since_data={time_since_data:.1f}s")
                                self._stuck_count_start = time.time()  # Reset timer to avoid spam
                        else:
                            self._last_message_count = message_count
                            self._stuck_count_start = time.time()
                        
                        if (current_threshold and 
                            message_count >= current_threshold and 
                            message_count > self.last_reset_count + 100 and  # Prevent rapid resets
                            self.auto_reconnect_var.get() and 
                            not self.reconnecting and
                            not self.reset_in_progress):  # Don't reset if already resetting
                            
                            self.last_reset_count = message_count
                            self.current_threshold_index += 1
                            
                            print(f"DEBUG: Triggering reset at {message_count} messages, threshold was {current_threshold}")
                            self.root.after(0, self.log, f"Preventive reset #{self.current_threshold_index} at {message_count} messages")
                            
                            # Try gentle reset first for first few attempts
                            if self.current_threshold_index <= 2:  # First 2 resets try gentle reset
                                self.root.after(0, self.try_gentle_reset)
                            else:
                                self.root.after(0, self.emergency_reset)  # Use hard reset for later attempts
                            break
                        
                        # Regular stall detection - but not during resets
                        if (time_since_data > self.stall_timeout and 
                            message_count > 0 and 
                            not self.reconnecting and 
                            not self.reset_in_progress and  # Don't trigger stall detection during reset
                            self.auto_reconnect_var.get()):
                            
                            print(f"DEBUG: Data stalled for {int(time_since_data)}s (timeout: {self.stall_timeout}s)")
                            print(f"DEBUG: Message count stuck at {message_count}, triggering gentle reset")
                            self.root.after(0, self.log, f"Data stalled for {int(time_since_data)}s - trying gentle reset")
                            # Try gentle reset first for data stalls instead of reconnection
                            self.root.after(0, self.try_gentle_reset)
                            break
                    
                    time.sleep(0.5)  # More frequent monitoring
                    
                # Connection lost - BUT only if not during reset
                if (self.is_connected and not self._shutting_down and 
                    not self.reconnecting and not self.reset_in_progress):
                    print("DEBUG: Monitor detected connection loss")
                    self.root.after(0, self.handle_disconnect)
            except Exception as e:
                print(f"DEBUG: Monitor thread error: {e}")
        
        threading.Thread(target=monitor, daemon=True).start()

    def update_status(self):
        """Enhanced status updates with flow control info"""
        if self._shutting_down:
            return
            
        try:
            # Basic connectivity test - force a simple update to verify UI works
            current_time = time.time()
            test_timestamp = datetime.now().strftime("%H:%M:%S")
            
            # Update connection timer FIRST to test if basic UI updates work
            if self.connection_start_time and self.is_connected:
                elapsed = current_time - self.connection_start_time
                timer_text = f"Connection Time: {self.format_duration(elapsed)}"
                # IMPORTANT: Always update timer, even during reset
                self.timer_label.config(text=timer_text, fg="green", bg="white")
                
                # Test if this basic update works
                if hasattr(self, '_timer_test_counter'):
                    self._timer_test_counter += 1
                else:
                    self._timer_test_counter = 0
                
                if self._timer_test_counter % 10 == 0:
                    print(f"DEBUG: Timer updated to {timer_text}")
            else:
                self.timer_label.config(text="Connection Time: --:--:--", fg="black", bg="lightgray")
            
            # Debug connection state
            if hasattr(self, '_debug_counter'):
                self._debug_counter += 1
            else:
                self._debug_counter = 0
            
            if self._debug_counter % 10 == 0:  # Every 5 seconds
                print(f"DEBUG UI Update [{test_timestamp}]:")
                print(f"  is_connected: {self.is_connected}")
                print(f"  connection exists: {self.connection is not None}")
                print(f"  client exists: {self.client is not None}")
                
                # Test basic method calls with more detailed error handling
                if self.connection:
                    try:
                        # Test each method individually
                        print(f"  Testing get_message_count()...")
                        count = self.connection.get_message_count()
                        print(f"  message_count: {count} ✓")
                        
                        print(f"  Testing get_queue_size()...")
                        queue_size = self.connection.get_queue_size()
                        print(f"  queue_size: {queue_size} ✓")
                        
                        print(f"  Testing get_throttle_delay()...")
                        throttle = self.connection.get_throttle_delay()
                        print(f"  throttle_delay: {throttle} ✓")
                        
                        print(f"  Testing get_avg_processing_time()...")
                        proc_time = self.connection.get_avg_processing_time()
                        print(f"  avg_processing_time: {proc_time} ✓")
                        
                        print(f"  All method calls successful!")
                    except AttributeError as e:
                        print(f"  ATTRIBUTE ERROR: {e}")
                        print(f"  Connection object type: {type(self.connection)}")
                        print(f"  Available methods: {[m for m in dir(self.connection) if not m.startswith('_')]}")
                    except Exception as e:
                        print(f"  OTHER ERROR calling methods: {e}")
                        import traceback
                        traceback.print_exc()
                else:
                    print(f"  connection is None - cannot call methods")
            
            # FORCE button enabling with simple state changes only
            if self.is_connected:
                # Just enable/disable, don't mess with colors
                self.gentle_reset_btn.config(state=tk.NORMAL)
                self.pause_clear_btn.config(state=tk.NORMAL) 
                self.cmd_reset_btn.config(state=tk.NORMAL)
                self.emergency_reset_btn.config(state=tk.NORMAL)
                
                if self._debug_counter % 10 == 0:
                    print(f"  FORCED buttons to NORMAL state")
            
            # Simplified UI updates with minimal color changes
            connection_valid = self.connection is not None
            is_connected = self.is_connected
            
            if is_connected and connection_valid:
                try:
                    # IMPORTANT: Skip status updates during reset to prevent false disconnections
                    if self.reset_in_progress:
                        # Show reset status instead of trying to get live stats
                        reset_time = time.time() - getattr(self, '_reset_start_time', time.time())
                        self.data_label.config(text=f"Messages: RESETTING... ({reset_time:.1f}s)")
                        self.queue_label.config(text="Queue: RESETTING...")
                        self.throttle_label.config(text="Throttle: RESETTING...")
                        self.processing_label.config(text="Proc Time: RESETTING...")
                        self.reset_label.config(text="Reset in progress...")
                        return  # Don't try to get connection stats during reset
                    
                    count = self.connection.get_message_count()
                    time_since = self.connection.time_since_last_data()
                    queue_size = self.connection.get_queue_size()
                    throttle_delay = self.connection.get_throttle_delay() * 1000  # Convert to ms
                    proc_time = self.connection.get_avg_processing_time() * 1000  # Convert to ms
                    
                    if self._debug_counter % 10 == 0:
                        print(f"  Retrieved stats: Count={count}, Queue={queue_size}, Throttle={throttle_delay:.2f}")
                    
                    # Simple text updates without color changes to avoid readability issues
                    self.data_label.config(text=f"Messages: {count}")
                    self.queue_label.config(text=f"Queue: {queue_size}")
                    self.throttle_label.config(text=f"Throttle: {throttle_delay:.2f}ms")
                    self.processing_label.config(text=f"Proc Time: {proc_time:.2f}ms")
                    
                    # Reset countdown
                    current_threshold = self.get_current_threshold()
                    if current_threshold and count > 0:
                        remaining = max(0, current_threshold - count)
                        self.reset_label.config(text=f"Next Reset: {remaining} msgs")
                    else:
                        self.reset_label.config(text="Next Reset: 50000")
                    
                    if self._debug_counter % 10 == 0:
                        print(f"  Updated UI labels successfully")
                    
                except Exception as e:
                    # CRITICAL FIX: Don't trigger disconnection during reset!
                    if self.reset_in_progress:
                        print(f"DEBUG: Ignoring connection error during reset: {e}")
                        # Show reset status instead of error
                        self.data_label.config(text="Messages: RESETTING...")
                        self.queue_label.config(text="Queue: RESETTING...")
                        self.throttle_label.config(text="Throttle: RESETTING...")
                        self.processing_label.config(text="Proc Time: RESETTING...")
                        return  # Don't show errors or trigger disconnect during reset
                    
                    print(f"ERROR in connected status update: {e}")
                    import traceback
                    traceback.print_exc()
                    # Show error in UI but DON'T trigger disconnection
                    self.data_label.config(text="Messages: ERROR")
                    self.queue_label.config(text="Queue: ERROR")
                    self.throttle_label.config(text="Throttle: ERROR")
                    self.processing_label.config(text="Proc Time: ERROR")
            else:
                # Not connected - show disconnected state
                if self._debug_counter % 10 == 0:
                    print(f"  Not connected - showing disconnected state")
                
                self.data_label.config(text=f"Messages: 0 [{test_timestamp}]")
                self.queue_label.config(text="Queue: 0")
                self.throttle_label.config(text="Throttle: 0.00ms")
                self.processing_label.config(text="Proc Time: 0.00ms")
                self.reset_label.config(text="Next Reset: 50000")
                
                # Disable buttons with gray appearance
                self.gentle_reset_btn.config(state=tk.DISABLED, bg="gray")
                self.pause_clear_btn.config(state=tk.DISABLED, bg="gray")
                self.cmd_reset_btn.config(state=tk.DISABLED, bg="gray")
                self.emergency_reset_btn.config(state=tk.DISABLED, bg="gray")
            
            # Force UI refresh
            self.root.update_idletasks()
            
            # Schedule next update
            self.root.after(500, self.update_status)
            
        except Exception as e:
            print(f"CRITICAL ERROR in update_status: {e}")
            import traceback
            traceback.print_exc()
            # Try to continue
            self.root.after(1000, self.update_status)

    def create_ui(self):
        """Enhanced UI with flow control monitoring"""
        # Title
        tk.Label(self.root, text="MetaBow Micro", font=("Arial", 16, "bold")).pack(pady=10)
        tk.Label(self.root, text="Anti-Stall Version with Aggressive Flow Control", font=("Arial", 9)).pack()
        
        # System info
        sys_info = f"Python: {sys.version.split()[0]} | Platform: {sys.platform}"
        tk.Label(self.root, text=sys_info, font=("Arial", 8), fg="gray").pack()
        
        # OSC status
        osc_status = "OSC: Available" if OSC_AVAILABLE else "OSC: Disabled"
        osc_color = "green" if OSC_AVAILABLE else "orange"
        tk.Label(self.root, text=osc_status, font=("Arial", 8), fg=osc_color).pack()
        
        # Status section
        self.status_label = tk.Label(self.root, text="Status: Ready", fg="blue", font=("Arial", 12))
        self.status_label.pack(pady=10)
        
        # Enhanced stats section
        stats_frame = tk.Frame(self.root)
        stats_frame.pack(pady=5)
        
        self.timer_label = tk.Label(stats_frame, text="Connection Time: --:--:--", font=("Arial", 10))
        self.timer_label.pack()
        
        self.data_label = tk.Label(stats_frame, text="Messages: 0", font=("Arial", 10))
        self.data_label.pack()
        
        self.queue_label = tk.Label(stats_frame, text="Queue: 0", font=("Arial", 10))
        self.queue_label.pack()
        
        self.throttle_label = tk.Label(stats_frame, text="Throttle: 0.00ms", font=("Arial", 10))
        self.throttle_label.pack()
        
        self.processing_label = tk.Label(stats_frame, text="Proc Time: 0.00ms", font=("Arial", 10))
        self.processing_label.pack()
        
        # Next reset indicator
        self.reset_label = tk.Label(stats_frame, text="Next Reset: 15000", font=("Arial", 10), fg="orange")
        self.reset_label.pack()
        
        # Controls section
        controls_frame = tk.Frame(self.root)
        controls_frame.pack(pady=10)
        
        # Auto-reconnect with multiple resets
        self.auto_reconnect_var = tk.BooleanVar(value=True)
        self.auto_reconnect_check = tk.Checkbutton(
            controls_frame, 
            text="Auto-reconnect + multi-stage reset", 
            variable=self.auto_reconnect_var,
            font=("Arial", 10)
        )
        self.auto_reconnect_check.pack()
        
        # Audio recording toggle
        audio_frame = tk.Frame(controls_frame)
        audio_frame.pack(pady=5)
        
        tk.Label(audio_frame, text="Audio Recording:", font=("Arial", 9)).pack(side=tk.LEFT)
        self.audio_recording_var = tk.BooleanVar(value=False)
        self.audio_recording_check = tk.Checkbutton(
            audio_frame, 
            text="Enable PCM file recording", 
            variable=self.audio_recording_var,
            command=self.toggle_audio_recording_ui,
            font=("Arial", 9)
        )
        self.audio_recording_check.pack(side=tk.LEFT)
        
        # Manual reset buttons
        reset_buttons_frame = tk.Frame(controls_frame)
        reset_buttons_frame.pack(pady=5)
        
        self.gentle_reset_btn = tk.Button(reset_buttons_frame, text="Gentle Reset", 
                                        command=self.try_gentle_reset, state=tk.DISABLED,
                                        bg="#28a745", fg="black", font=("Arial", 9))
        self.gentle_reset_btn.pack(side=tk.LEFT, padx=2)
        
        self.pause_clear_btn = tk.Button(reset_buttons_frame, text="Pause & Clear", 
                                       command=self.try_pause_clear, state=tk.DISABLED,
                                       bg="#17a2b8", fg="black", font=("Arial", 9))
        self.pause_clear_btn.pack(side=tk.LEFT, padx=2)
        
        self.cmd_reset_btn = tk.Button(reset_buttons_frame, text="Send Reset Cmd", 
                                     command=self.send_device_reset_command, state=tk.DISABLED,
                                     bg="#6f42c1", fg="black", font=("Arial", 9))
        self.cmd_reset_btn.pack(side=tk.LEFT, padx=2)
        
        self.emergency_reset_btn = tk.Button(reset_buttons_frame, text="Emergency Reset", 
                                           command=self.emergency_reset, state=tk.DISABLED,
                                           bg="#ff6b6b", fg="black", font=("Arial", 9))
        self.emergency_reset_btn.pack(side=tk.LEFT, padx=2)
        
        # Scan button
        tk.Button(self.root, text="Scan for Devices", command=self.scan_devices, 
                 font=("Arial", 12), bg="#007bff", fg="black").pack(pady=10)
        
        # Device list
        tk.Label(self.root, text="Devices:", font=("Arial", 10, "bold")).pack()
        self.device_listbox = tk.Listbox(self.root, height=3, font=("Arial", 10))
        self.device_listbox.pack(pady=5, fill=tk.X, padx=20)
        self.device_listbox.bind('<<ListboxSelect>>', self.on_device_select)
        
        # Connection buttons
        button_frame = tk.Frame(self.root)
        button_frame.pack(pady=10)
        
        self.connect_btn = tk.Button(button_frame, text="Connect", 
                                   command=self.connect_device, state=tk.DISABLED,
                                   bg="#28a745", fg="black", font=("Arial", 11))
        self.connect_btn.pack(side=tk.LEFT, padx=5)
        
        self.disconnect_btn = tk.Button(button_frame, text="Disconnect", 
                                      command=self.disconnect_device, state=tk.DISABLED,
                                      bg="#dc3545", fg="black", font=("Arial", 11))
        self.disconnect_btn.pack(side=tk.LEFT, padx=5)
        
        # Log area
        tk.Label(self.root, text="Log:", font=("Arial", 10, "bold")).pack(pady=(15, 0))
        self.log_text = tk.Text(self.root, height=8, font=("Courier", 9), wrap=tk.WORD)
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=20, pady=5)
        
        # Initial log
        self.log("Anti-stall version ready! (EXACT MINIMAL FIXES APPLIED)")
        self.log("Features: Conservative throttling, gentle resets, optional audio recording")
        self.log("Reset options: Gentle (memory only), Pause & Clear, Command, Emergency")
        self.log("TESTING: Very high thresholds: 50k, 75k, 100k messages - manual reset only")
        self.log("Audio recording: DISABLED by default (enable with checkbox if needed)")
        if OSC_AVAILABLE:
            self.log("OSC enabled - motion data will be sent to port 8888")

    def format_duration(self, seconds):
        if seconds is None:
            return "--:--:--"
        hours = int(seconds // 3600)
        minutes = int((seconds % 3600) // 60)
        secs = int(seconds % 60)
        return f"{hours:02d}:{minutes:02d}:{secs:02d}"

    def log(self, message):
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
        """Scan for devices"""
        self.log("Scanning for MetaBow devices...")
        self.device_listbox.delete(0, tk.END)
        self.device_listbox.insert(tk.END, "Scanning...")
        
        def run_scan():
            try:
                devices = run_in_ble_loop(self.async_scan())
                self.root.after(0, self.update_device_list, devices)
            except Exception as e:
                error_msg = str(e)
                if "timeout" in error_msg.lower():
                    error_msg = "Scan timeout - check Bluetooth"
                self.root.after(0, self.log, f"Scan failed: {error_msg}")
                self.root.after(0, self.update_device_list, [])
        
        threading.Thread(target=run_scan, daemon=True).start()

    async def async_scan(self):
        """Async device scan"""
        try:
            devices = await BleakScanner.discover(timeout=10.0)
            metabow_devices = []
            
            for device in devices:
                if device.name:
                    device_name_lower = device.name.lower()
                    for target_name in self.device_names:
                        if target_name.lower() in device_name_lower:
                            metabow_devices.append(device)
                            break
            
            return metabow_devices
            
        except Exception as e:
            print(f"Scan error: {e}")
            raise

    def update_device_list(self, devices):
        """Update device list"""
        self.devices = devices
        self.device_listbox.delete(0, tk.END)
        
        if devices:
            for device in devices:
                display_name = f"{device.name} ({device.address})"
                self.device_listbox.insert(tk.END, display_name)
            self.log(f"Found {len(devices)} MetaBow device(s)")
        else:
            self.device_listbox.insert(tk.END, "No MetaBow devices found")
            self.log("No devices found")

    def on_device_select(self, event):
        """Device selection handler"""
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
        
        self.log(f"Connecting to {self.selected_device.name}...")
        self.connect_btn.config(state=tk.DISABLED)
        self.status_label.config(text="Status: Connecting...", fg="orange")
        self.reconnecting = False
        self.reconnect_attempts = 0
        
        # Reset threshold tracking
        self.current_threshold_index = 0
        self.last_reset_count = 0
        
        def run_connect():
            try:
                result = run_in_ble_loop(self.async_connect(self.selected_device))
                self.root.after(0, self.handle_connect_result, result)
            except Exception as e:
                self.root.after(0, self.handle_connect_result, f"Error: {e}")
        
        threading.Thread(target=run_connect, daemon=True).start()

    async def async_connect(self, device):
        """Enhanced connection with aggressive cleanup"""
        try:
            # Force cleanup of any existing client
            if self.client:
                try:
                    if self.client.is_connected:
                        await self.client.disconnect()
                except:
                    pass
                self.client = None
            
            # Force garbage collection before new connection
            gc.collect()
            
            # Create new client with shorter timeout for faster failure detection
            self.client = BleakClient(device.address, timeout=20.0)
            
            # Connect
            await self.client.connect()
            await asyncio.sleep(1.0)
            
            if not self.client.is_connected:
                return "Error: Failed to establish connection"
            
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
                return "Error: UART service not found"
            
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
                return "Error: Required characteristics not found"
            
            # Create flow-controlled connection
            self.connection = FlowControlledConnection(
                self.client, rx_char, tx_char, self.log
            )
            
            # Start notifications
            await self.connection.start()
            
            return "Success"
            
        except Exception as e:
            error_msg = str(e)
            if "timeout" in error_msg.lower():
                return "Error: Connection timeout"
            elif "failed to connect" in error_msg.lower():
                return "Error: Connection failed"
            else:
                return f"Error: {error_msg}"

    def handle_connect_result(self, result):
        """Handle connection result with enhanced debugging"""
        if self._shutting_down:
            return
            
        if result == "Success":
            self.is_connected = True
            self.connection_start_time = time.time()
            self.status_label.config(text="Status: Connected", fg="green")
            self.connect_btn.config(state=tk.DISABLED)
            self.disconnect_btn.config(state=tk.NORMAL)
            self.log("Connected successfully! Flow control active.")
            
            # IMMEDIATELY enable reset buttons upon connection
            self.gentle_reset_btn.config(state=tk.NORMAL)
            self.pause_clear_btn.config(state=tk.NORMAL)
            self.cmd_reset_btn.config(state=tk.NORMAL)
            self.emergency_reset_btn.config(state=tk.NORMAL)
            
            # Debug connection state
            print(f"DEBUG: Connection established - Client: {self.client is not None}")
            print(f"DEBUG: Connection object: {self.connection is not None}")
            if self.connection:
                print(f"DEBUG: Processor thread alive: {self.connection.processor_thread.is_alive()}")
                print(f"DEBUG: Message count method: {hasattr(self.connection, 'get_message_count')}")
            print(f"DEBUG: Buttons manually enabled")
            
            self.start_monitoring()
        else:
            self.is_connected = False
            self.connection_start_time = None
            self.status_label.config(text="Status: Failed", fg="red")
            self.connect_btn.config(state=tk.NORMAL if self.selected_device else tk.DISABLED)
            self.log(f"Connection failed: {result}")

    def attempt_reconnect(self):
        """Fast reconnection attempt"""
        if not self.selected_device or self.reconnecting or self._shutting_down:
            return
        
        if not self.auto_reconnect_var.get():
            self.log("Auto-reconnect disabled")
            return
        
        if self.reconnect_attempts >= self.max_reconnect_attempts:
            self.log("Max reconnect attempts reached")
            return
        
        self.reconnecting = True
        self.reconnect_attempts += 1
        
        self.log(f"Fast reconnect attempt #{self.reconnect_attempts}/{self.max_reconnect_attempts}...")
        self.status_label.config(text=f"Status: Reconnecting... ({self.reconnect_attempts}/{self.max_reconnect_attempts})", fg="orange")
        
        def run_reconnect():
            try:
                # Quick cleanup with proper error handling
                if self.connection:
                    self.connection.close()
                
                if self.client:
                    try:
                        if self.connection:
                            run_in_ble_loop(self.connection.force_disconnect())
                        else:
                            # Fallback if connection is None
                            run_in_ble_loop(self.client.disconnect())
                    except Exception as disconnect_error:
                        self.root.after(0, self.log, f"Disconnect warning: {disconnect_error}")
                
                # Clear references
                self.connection = None
                self.client = None
                
                # Shorter wait for faster recovery
                time.sleep(2)
                
                # Force garbage collection
                gc.collect()
                
                # Attempt reconnection
                result = run_in_ble_loop(self.async_connect(self.selected_device))
                self.root.after(0, self.handle_reconnect_result, result)
                
            except Exception as e:
                self.root.after(0, self.handle_reconnect_result, f"Error: {e}")
        
        threading.Thread(target=run_reconnect, daemon=True).start()

    def handle_reconnect_result(self, result):
        """Handle reconnection result"""
        self.reconnecting = False
        
        if result == "Success":
            self.log("Fast reconnection successful!")
            self.reconnect_attempts = 0
            # Don't reset threshold tracking on reconnect
            self.handle_connect_result(result)
        else:
            self.log(f"Reconnection attempt {self.reconnect_attempts} failed: {result}")
            
            if self.reconnect_attempts < self.max_reconnect_attempts:
                self.log("Will retry in 3 seconds...")
                self.root.after(3000, self.attempt_reconnect)
            else:
                self.log("Fast reconnect failed - trying emergency reset")
                self.emergency_reset()

    def disconnect_device(self):
        """Disconnect device"""
        self.log("Disconnecting...")
        self.is_connected = False
        self.reconnecting = False
        
        def run_disconnect():
            try:
                if self.connection:
                    self.connection.close()
                
                if self.client:
                    try:
                        if self.client.is_connected:
                            run_in_ble_loop(self.client.disconnect())
                    except Exception as disconnect_error:
                        self.root.after(0, self.log, f"Disconnect warning: {disconnect_error}")
                
                # Clear references
                self.connection = None
                self.client = None
                
                self.root.after(0, self.handle_disconnect)
            except Exception as e:
                self.root.after(0, self.log, f"Disconnect error: {e}")
                self.root.after(0, self.handle_disconnect)
        
        threading.Thread(target=run_disconnect, daemon=True).start()

    def handle_disconnect(self):
        """Handle disconnection"""
        if self._shutting_down:
            return
        
        # CRITICAL DEBUG: Find out what's calling disconnect during reset
        if self.reset_in_progress:
            import traceback
            print("ERROR: handle_disconnect() called during reset! Call stack:")
            traceback.print_stack()
            print("DEBUG: Ignoring disconnect call during reset")
            return  # Don't actually disconnect during reset!
            
        self.is_connected = False
        self.connection_start_time = None
        if self.connection:
            self.connection.close()
            self.connection = None
        self.status_label.config(text="Status: Disconnected", fg="red")
        self.connect_btn.config(state=tk.NORMAL if self.selected_device else tk.DISABLED)
        self.disconnect_btn.config(state=tk.DISABLED)
        self.log("Disconnected")

    def on_closing(self):
        """Handle app closing"""
        self._shutting_down = True
        self.log("Shutting down...")
        
        if self.is_connected:
            self.disconnect_device()
            time.sleep(0.5)
        
        self.root.destroy()

    def run(self):
        """Start the application"""
        self.root.mainloop()


def check_dependencies():
    """Check for required dependencies"""
    print("MetaBow Micro - Anti-Stall Edition (EXACT MINIMAL FIXES)")
    print(f"Python version: {sys.version.split()[0]}")
    print(f"Platform: {sys.platform}")
    
    # Check bleak
    try:
        import bleak
        version = getattr(bleak, '__version__', 'unknown')
        print(f"Bleak version: {version}")
    except ImportError:
        print("ERROR: bleak not installed.")
        print("Install with: pip3 install bleak")
        return False
    
    # Check python-osc
    global OSC_AVAILABLE
    try:
        import pythonosc
        print("python-osc: available")
        OSC_AVAILABLE = True
    except ImportError:
        print("python-osc: not installed (OSC features disabled)")
        OSC_AVAILABLE = False
    
    return True


if __name__ == "__main__":
    if not check_dependencies():
        sys.exit(1)
    
    print("\nStarting anti-stall application... (EXACT MINIMAL FIXES)")
    print("Features: Multi-stage reset, flow control, aggressive throttling")
    print("FIXES: 1) soft_reset return, 2) send_reset_command method, 3) get_avg_processing_time, 4) reset_in_progress init")
    app = AntiStallApp()
    app.run()