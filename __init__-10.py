"""
MetaBow v16 - ADPCM Audio Implementation
- Motion data: /metabow/motion (16 float32 IMU values including raw acceleration)
- Battery data: /metabow/battery/percentage 
- Audio data: /metabow/audio (ADPCM decoded to PCM)
- ADPCM audio recording to WAV and raw files
- All data sent to both ports 8888 and 8889
- Sequoia-aware BLE management with platform optimizations
- NEW: 114-byte packet format with linear + raw acceleration
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
import numpy as np
import wave

# BLE imports
from bleak import BleakClient, BleakScanner

# OSC and file output
try:
    from pythonosc import udp_client
    OSC_AVAILABLE = True
except ImportError:
    OSC_AVAILABLE = False

# ADPCM Decoder Implementation
class ADPCMDecoder:
    """IMA ADPCM decoder matching firmware implementation"""
    
    # ADPCM index adaptation table
    INDEX_TABLE = [
        -1, -1, -1, -1, 2, 4, 6, 8,
        -1, -1, -1, -1, 2, 4, 6, 8
    ]
    
    # ADPCM step size table
    STEP_TABLE = [
        7, 8, 9, 10, 11, 12, 13, 14, 16, 17,
        19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
        50, 55, 60, 66, 73, 80, 88, 97, 107, 118,
        130, 143, 157, 173, 190, 209, 230, 253, 279, 307,
        337, 371, 408, 449, 494, 544, 598, 658, 724, 796,
        876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066,
        2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358,
        5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899,
        15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
    ]
    
    def __init__(self, sample_rate: int = 16000):
        """Initialize ADPCM decoder"""
        self.sample_rate = sample_rate
        self.reset()
    
    def reset(self):
        """Reset decoder state"""
        self.predicted_sample = 0
        self.step_index = 0
    
    def _clamp(self, v, lo, hi):
        """Clamp value to range"""
        return lo if v < lo else (hi if v > hi else v)
    
    def decode_sample(self, adpcm_nibble: int) -> int:
        """Decode a single 4-bit ADPCM sample to 16-bit PCM"""
        # Get current step size
        step = self.STEP_TABLE[self.step_index]
        
        # Calculate difference (matching firmware)
        diff = 0
        if adpcm_nibble & 4:
            diff += step
        if adpcm_nibble & 2:
            diff += step >> 1
        if adpcm_nibble & 1:
            diff += step >> 2
        diff += step >> 3
        
        # Apply sign
        if adpcm_nibble & 8:
            self.predicted_sample -= diff
        else:
            self.predicted_sample += diff
        
        # Clamp to 16-bit range
        self.predicted_sample = self._clamp(self.predicted_sample, -32768, 32767)
        
        # Update step index
        self.step_index += self.INDEX_TABLE[adpcm_nibble & 0x0F]
        self.step_index = self._clamp(self.step_index, 0, 88)
        
        return self.predicted_sample
    
    def decode(self, adpcm_data: bytes) -> list:
        """
        Decode ADPCM compressed data to PCM samples.
        Each byte contains two samples: high nibble (first), low nibble (second).
        State persists across calls - no reset between packets.
        """
        out = []
        for byte_val in adpcm_data:
            # Decode high nibble first (bits 4-7) - matches firmware
            high_nibble = (byte_val >> 4) & 0x0F
            out.append(self.decode_sample(high_nibble))
            
            # Then decode low nibble (bits 0-3)
            low_nibble = byte_val & 0x0F
            out.append(self.decode_sample(low_nibble))
        
        return out
    
    def get_stats(self):
        """Get decoder statistics"""
        return {
            'sample_rate': self.sample_rate,
            'compression_ratio': 4.0,
            'current_predicted': self.predicted_sample,
            'current_step_index': self.step_index
        }
    
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
    """BLE connection with complete OSC output and ADPCM audio recording"""
    
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
        
        # ADPCM audio recording - STREAMING MODE (no per-packet reset)
        self.audio_recording_enabled = False
        self.binary_file = None
        self.wav_file = None
        self.adpcm_decoder = ADPCMDecoder(sample_rate=16000)
        
        # CRITICAL: Must be False for streaming ADPCM
        # self.use_packet_reset = False  # Changed from True
        # self.use_low_nibble_first = False
        # self.debug_pcm_samples = True
        # self.skip_header_bytes = 0
        
        # OSC message printing
        self.osc_print_enabled = False
        self.osc_print_counter = 0
        self.motion_osc_counter = 0
        self.battery_osc_counter = 0
        self.audio_osc_counter = 0
        
        # Setup
        self._setup_osc()
        self._start_processor()
        # self._setup_downsampling()
        
        if self.is_sequoia:
            self.log(f"🔴 SEQUOIA {self.version} - Ultra-conservative mode")
        else:
            self.log(f"✅ Platform optimized settings applied")
        
        self.log("✅ ADPCM decoder initialized for 16kHz audio")
    
    def _apply_platform_settings(self):
        """Apply platform-specific settings"""
        if self.is_sequoia:
            # Ultra-conservative for Sequoia
            self.throttle_delay = 0.005  # 5ms
            self.max_queue_size = 3
            # self.drop_ratio = 8  # Keep 1 in 8
            self.batch_size = 1
            self.gc_interval = 1
        elif platform.system() == 'Darwin':
            # Standard macOS
            self.throttle_delay = 0.015  # 15ms
            self.max_queue_size = 10
            # self.drop_ratio = 3
            self.batch_size = 3
            self.gc_interval = 3
        elif platform.system() == 'Windows':
            # Windows
            self.throttle_delay = 0.05  # 50ms
            self.max_queue_size = 5
            # self.drop_ratio = 9
            self.batch_size = 2
            self.gc_interval = 2
        else:
            # Linux
            self.throttle_delay = 0.01  # 10ms
            self.max_queue_size = 20
            # self.drop_ratio = 2
            self.batch_size = 5
            self.gc_interval = 5
    
    def _setup_osc(self):
        """Setup OSC clients for both ports 8888 and 8889"""
        self.osc_clients = []
        
        if OSC_AVAILABLE:
            # Setup port 8888
            try:
                osc_8888 = udp_client.SimpleUDPClient("127.0.0.1", 8888)
                self.osc_clients.append(('8888', osc_8888))
                self.log("OSC connected to port 8888")
            except Exception as e:
                self.log(f"OSC setup failed for port 8888: {e}")
            
            # Setup port 8889
            try:
                osc_8889 = udp_client.SimpleUDPClient("127.0.0.1", 8889)
                self.osc_clients.append(('8889', osc_8889))
                self.log("OSC connected to port 8889")
            except Exception as e:
                self.log(f"OSC setup failed for port 8889: {e}")
            
            if self.osc_clients:
                self.log("OSC ready for motion, battery, and audio data")
            else:
                self.log("No OSC clients available")
        else:
            self.log("OSC library not available")
    
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
            # if current_time - self.last_packet_time < self.throttle_delay:
                # return  # Drop packet
            # self.last_packet_time = current_time
            
            # Queue management
            with self.queue_lock:
                if len(self.data_queue) >= self.max_queue_size:
                    # Drop old packets
                    dropped = len(self.data_queue) // 2
                    self.data_queue = self.data_queue[dropped:]
                
                self.data_queue.append((data, current_time))
        
        except Exception as e:
            self.log(f"Callback error: {e}")
    
    def _decode_adpcm_audio(self, adpcm_data):
        """Decode ADPCM audio to PCM - streaming mode"""
        try:
            if not adpcm_data or len(adpcm_data) == 0:
                return None
            
            # Decode ADPCM to PCM samples (returns list of int16)
            pcm_samples = self.adpcm_decoder.decode(adpcm_data)
            
            # Debug logging
            if self.message_count % 200 == 0:
                sample_min, sample_max = min(pcm_samples), max(pcm_samples)
                sample_range = sample_max - sample_min
                
                self.log(f"ADPCM: {len(adpcm_data)}B → {len(pcm_samples)} samples")
                self.log(f"  Range: [{sample_min}, {sample_max}] span={sample_range}")
                self.log(f"  State: pred={self.adpcm_decoder.predicted_sample}, step={self.adpcm_decoder.step_index}")
                
                if sample_range < 100:
                    self.log("  WARNING: Very low range - possible silence")
                elif sample_range > 1000:
                    self.log("  Good: Active audio signal")
            
            # Pack samples to bytes - CRITICAL: Use struct.pack exactly like working script
            pcm_bytes = struct.pack('<' + 'h' * len(pcm_samples), *pcm_samples)
            return pcm_bytes
            
        except Exception as e:
            if self.message_count % 1000 == 0:
                self.log(f"ADPCM decode error: {e}")
            return None
    
    def _send_motion_osc_structured(self, imu_floats):
        """Send structured motion data with proper IMU breakdown"""
        for port_name, osc_client in self.osc_clients:
            try:
                # Send complete motion vector (all 13 floats)
                osc_client.send_message("/metabow/motion", imu_floats)
                
                # Send structured IMU data (assuming standard IMU format)
                if len(imu_floats) >= 13:
                    # Quaternion (w, x, y, z) - floats 0-3
                    osc_client.send_message("/metabow/motion/quaternion", imu_floats[0:4])
                    
                    # Acceleration (x, y, z) - floats 4-6  
                    osc_client.send_message("/metabow/motion/acceleration", imu_floats[4:7])
                    
                    # Gyroscope (x, y, z) - floats 7-9
                    osc_client.send_message("/metabow/motion/gyroscope", imu_floats[7:10])
                    
                    # Magnetometer (x, y, z) - floats 10-12
                    osc_client.send_message("/metabow/motion/magnetometer", imu_floats[10:13])
                
                # Print OSC message if enabled
                if self.osc_print_enabled:
                    self.motion_osc_counter += 1
                    if self.motion_osc_counter % 10 == 0:
                        if len(imu_floats) >= 7:
                            quat_str = f"Q=({imu_floats[0]:.2f},{imu_floats[1]:.2f},{imu_floats[2]:.2f},{imu_floats[3]:.2f})"
                            accel_str = f"A=({imu_floats[4]:.2f},{imu_floats[5]:.2f},{imu_floats[6]:.2f})"
                            self.log(f"OSC[{port_name}] Motion: {quat_str} {accel_str}")
                        
            except Exception as e:
                self.log(f"Motion OSC error on port {port_name}: {e}")

    def _process_packet(self, data, timestamp):
        """Process 114-byte packet with linear + raw acceleration"""
        try:
            self.message_count += 1
            self.last_data_time = timestamp
            
            # Debug for first few packets
            if self.message_count in [10, 50, 100, 200, 500]:
                self.log(f"DEBUG {self.message_count}: packet len={len(data)}")
            
            # Log progress
            if self.message_count % 1000 == 0:
                stats = self.get_stats()
                indicator = "🔴 " if self.is_sequoia else ""
                self.log(f"{indicator}Messages: {self.message_count}")
                self.log(f"  Drop rate: {stats['drop_rate']:.1f}%")
                self.log(f"  Effective: {stats['effective_rate']:.1f} msg/s")
            
            # Validate packet size (should be 114 bytes)
            data_len = len(data)
            if data_len != 114:
                if self.message_count % 100 == 0:
                    self.log(f"Unexpected packet size: {data_len} bytes (expected 114)")
                return
            
            # Parse 114-byte packet format:
            # [0..44] (45 B) ADPCM audio
            # [45..108] (64 B) IMU bundle = 16 floats (little-endian):
            #   [0-3] Quaternion (I, J, K, R)
            #   [4-6] Linear Accel (m/s², gravity removed)
            #   [7-9] Gyroscope (rad/s)
            #   [10-12] Magnetometer (µT)
            #   [13-15] Raw Accel (m/s², gravity included)
            # [109] (1 B) IMU present flag
            # [110..113] (4 B) Battery SoC as float
            
            # Extract ADPCM audio data (45 bytes)
            adpcm_audio_data = data[0:45]
            
            # Extract IMU data (64 bytes = 16 floats)
            try:
                imu_flag = data[109]
                if imu_flag == 1:
                    # Parse 16 float32 values (64 bytes)
                    imu_floats = list(struct.unpack('<16f', data[45:109]))
                    
                    # Validate IMU data
                    if all(abs(x) < 1e10 and not (x != x) for x in imu_floats):  # Not NaN or huge
                        self._send_motion_osc_structured(imu_floats)
                        
                        if self.message_count % 100 == 0:
                            self.log(f"IMU SUCCESS: 16 floats parsed (incl. raw accel)")
                            self.log(f"  [0-3] Quat: {[f'{x:.3f}' for x in imu_floats[0:4]]}")
                            self.log(f"  [4-6] Lin.Accel: {[f'{x:.3f}' for x in imu_floats[4:7]]}")
                            self.log(f"  [13-15] Raw Accel: {[f'{x:.3f}' for x in imu_floats[13:16]]}")
                    else:
                        if self.message_count % 1000 == 0:
                            self.log("IMU validation failed: invalid float values")
                else:
                    if self.message_count % 1000 == 0:
                        self.log(f"IMU not present (flag={imu_flag})")
                        
            except Exception as e:
                if self.message_count % 1000 == 0:
                    self.log(f"IMU parsing error: {e}")
            
            # Extract battery data (4 bytes)
            try:
                battery_soc = struct.unpack('<f', data[110:114])[0]
                if self.message_count % 500 == 0:
                    self.log(f"Battery SoC: {battery_soc:.2f}%")
                
                if 0 <= battery_soc <= 100 or battery_soc == 0.0:
                    self._send_battery_osc(battery_soc)
                    
            except Exception as e:
                if self.message_count % 1000 == 0:
                    self.log(f"Battery parsing error: {e}")
            
            # Process ADPCM audio
            try:
                if adpcm_audio_data:
                    self._send_audio_osc(adpcm_audio_data)
                    
                    # Record audio if enabled
                    if self.audio_recording_enabled and (self.binary_file or self.wav_file):
                        if self.binary_file:
                            self.binary_file.write(adpcm_audio_data)
                            if self.message_count % 200 == 0:
                                self.binary_file.flush()
                        
                        if self.wav_file:
                            pcm_data = self._decode_adpcm_audio(adpcm_audio_data)
                            if pcm_data:
                                self.wav_file.writeframes(pcm_data)
                                if self.message_count % 200 == 0:
                                    self.wav_file._file.flush()
                    
            except Exception as e:
                if self.message_count % 1000 == 0:
                    self.log(f"Audio processing error: {e}")
        
        except Exception as e:
            self.log(f"Packet processing error: {e}")
    
    def _send_battery_osc(self, battery_percentage):
        """Send battery data to OSC ports"""
        for port_name, osc_client in self.osc_clients:
            try:
                osc_client.send_message("/metabow/battery/percentage", [battery_percentage])
                
                # Print OSC message if enabled
                if self.osc_print_enabled:
                    self.battery_osc_counter += 1
                    if self.battery_osc_counter % 50 == 0:
                        self.log(f"OSC[{port_name}] /metabow/battery/percentage: {battery_percentage:.1f}%")
                        
            except Exception as e:
                self.log(f"Battery OSC error on port {port_name}: {e}")
    
    def _send_audio_osc(self, audio_data):
        """Send audio data to OSC ports"""
        for port_name, osc_client in self.osc_clients:
            try:
                # For ADPCM, send the compressed data as normalized floats
                audio_floats = [float(b) / 255.0 for b in audio_data[::2]]  # Downsample
                
                osc_client.send_message("/metabow/audio", audio_floats)
                
                # Print OSC message if enabled
                if self.osc_print_enabled:
                    self.audio_osc_counter += 1
                    if self.audio_osc_counter % 100 == 0:
                        self.log(f"OSC[{port_name}] /metabow/audio: {len(audio_floats)} samples")
                        
            except Exception as e:
                self.log(f"Audio OSC error on port {port_name}: {e}")
    
    def toggle_osc_print(self):
        """Toggle OSC message printing"""
        self.osc_print_enabled = not self.osc_print_enabled
        
        # Reset all counters
        self.osc_print_counter = 0
        self.motion_osc_counter = 0
        self.battery_osc_counter = 0
        self.audio_osc_counter = 0
        
        if self.osc_print_enabled:
            self.log("OSC message printing ENABLED")
            self.log("  Motion: every 10th message")
            self.log("  Battery: every 50th message") 
            self.log("  Audio: every 100th message")
        else:
            self.log("OSC message printing DISABLED")
        
        return self.osc_print_enabled
    
    def setup_file_output(self):
        """Setup ADPCM audio recording"""
        try:
            if platform.system() == "Darwin":
                output_dir = os.path.expanduser("~/Documents/MetaBow_Data")
            else:
                output_dir = os.path.expanduser("~/MetaBow_Data")
            
            os.makedirs(output_dir, exist_ok=True)
            timestamp = int(time.time())
            
            self.log(f"Recording to: {output_dir}")
            
            # Setup raw ADPCM file recording
            adpcm_filename = f'adpcm_audio_{timestamp}.bin'
            adpcm_filepath = os.path.join(output_dir, adpcm_filename)
            self.binary_file = open(adpcm_filepath, 'wb', buffering=8192)
            self.log(f"ADPCM file: {adpcm_filename}")
            
            # Setup WAV file recording (decoded ADPCM)
            try:
                wav_filename = f'decoded_audio_{timestamp}.wav'
                wav_filepath = os.path.join(output_dir, wav_filename)
                
                self.wav_file = wave.open(wav_filepath, 'wb')
                self.wav_file.setnchannels(1)  # Mono
                self.wav_file.setsampwidth(2)  # 16-bit samples
                self.wav_file.setframerate(self.adpcm_decoder.sample_rate)
                
                self.log(f"✅ WAV file: {wav_filename}")
                self.log(f"   Audio format: {self.adpcm_decoder.sample_rate}Hz, 1 channel, 16-bit")
                self.log("   ADPCM decoder ready - creating both ADPCM and WAV files")
                
            except Exception as e:
                self.log(f"WAV setup failed: {e}")
                self.wav_file = None
                self.log("Recording raw ADPCM data only")
                
        except Exception as e:
            self.log(f"Recording setup failed: {e}")
            self.binary_file = None
            self.wav_file = None
    
    def stop_file_output(self):
        """Stop audio recording"""
        if self.binary_file:
            try:
                self.binary_file.flush()
                self.binary_file.close()
                self.log("ADPCM recording stopped")
            except:
                pass
            self.binary_file = None
            
        if self.wav_file:
            try:
                self.wav_file.close()
                self.log("WAV recording stopped")
            except:
                pass
            self.wav_file = None
    
    def toggle_audio_recording(self):
        """Toggle audio recording"""
        self.audio_recording_enabled = not self.audio_recording_enabled
        if self.audio_recording_enabled:
            # Reset decoder at start of NEW recording session
            self.adpcm_decoder.reset()
            self.setup_file_output()
            self.log("Audio recording started - decoder state reset")
        else:
            self.stop_file_output()
            self.log("Audio recording stopped")
    
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
            
            # Reset OSC print counters
            self.osc_print_counter = 0
            self.motion_osc_counter = 0
            self.battery_osc_counter = 0
            self.audio_osc_counter = 0
            
            # Reset ADPCM decoder state
            # self.adpcm_decoder.reset()
            
            gc.collect()
            
            self.log(f"{indicator}Reset complete (was {old_count} messages)")
            return "Success"
        
        except Exception as e:
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
            'time_since_data': time.time() - self.last_data_time,
            'osc_print_enabled': self.osc_print_enabled
        }
    
    def get_adpcm_stats(self):
        """Get ADPCM decoder statistics"""
        return self.adpcm_decoder.get_stats()
    
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
        
        if self.wav_file:
            try:
                self.wav_file.close()
            except:
                pass
        
        gc.collect()

class MetaBowApp:
    """Main MetaBow application"""
    
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("MetaBow v16 - ADPCM Audio Implementation")
        self.root.geometry("550x700")
        
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
        self.monitor_thread = None
        self.monitor_active = False
        
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
        title = "MetaBow v16 - ADPCM Audio Implementation"
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
        if OSC_AVAILABLE:
            osc_text = "OSC: /metabow/motion + /metabow/battery/percentage + /metabow/audio"
            osc_color = "green"
        else:
            osc_text = "OSC: Disabled"
            osc_color = "orange"
        tk.Label(self.root, text=osc_text, font=("Arial", 8), fg=osc_color).pack()
        tk.Label(self.root, text="Ports: 8888 & 8889", font=("Arial", 8), fg=osc_color).pack()
        
        # ADPCM status
        tk.Label(self.root, text="ADPCM Decoder: Built-in IMA 4-bit decoder", font=("Arial", 8), fg="green").pack()
        tk.Label(self.root, text="Audio: 16kHz, 4:1 compression ratio", font=("Arial", 8), fg="green").pack()
        
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
        
        # OSC status label
        self.osc_status_label = tk.Label(stats_frame, text="OSC Print: OFF", font=("Arial", 9), fg="gray")
        self.osc_status_label.pack()
        
        # Controls frame
        controls_frame = tk.Frame(self.root)
        controls_frame.pack(pady=10)
        
        # Audio recording
        self.audio_var = tk.BooleanVar(value=False)
        self.audio_check = tk.Checkbutton(controls_frame, text="Record Audio (ADPCM + WAV)", 
                                        variable=self.audio_var, command=self.toggle_audio)
        self.audio_check.pack()
        
        # OSC printing checkbox
        self.osc_print_var = tk.BooleanVar(value=False)
        self.osc_print_check = tk.Checkbutton(controls_frame, text="Print OSC Messages", 
                                            variable=self.osc_print_var, command=self.toggle_osc_print)
        self.osc_print_check.pack()
        
        # ADPCM Debug button
        self.adpcm_debug_btn = tk.Button(controls_frame, text="ADPCM Debug Mode", 
                                       command=self.toggle_adpcm_debugging, state=tk.DISABLED,
                                       bg="#ffc107", fg="black")
        self.adpcm_debug_btn.pack(pady=2)
        
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
        self.log("MetaBow v16 ADPCM Audio Implementation ready!")
        if self.is_sequoia:
            self.log(f"🔴 SEQUOIA {self.version} detected - Ultra-conservative mode")
            self.log(f"   Reset thresholds: {self.reset_thresholds}")
        else:
            self.log(f"✅ {platform.system()} optimized settings")
        
        if OSC_AVAILABLE:
            self.log("OSC enabled - sending to ports 8888 & 8889:")
            self.log("  /metabow/motion - IMU motion data (13 floats)")
            self.log("  /metabow/battery/percentage - Battery level")
            self.log("  /metabow/audio - ADPCM audio data")
        
        self.log("ADPCM decoder ready - 16kHz, 4:1 compression")
        self.log("Packet format: 114 bytes (45B audio + 64B IMU + 1B flag + 4B battery)")
        self.log("ADPCM: Streaming mode (continuous state across packets)")
        self.log("Ready to scan for MetaBow devices")
    
    def toggle_audio(self):
        """Toggle audio recording"""
        if not self.is_connected or not self.connection:
            messagebox.showwarning("Not Connected", "Please connect to a device first")
            self.audio_var.set(False)
            return
        
        try:
            self.connection.toggle_audio_recording()
            enabled = self.audio_var.get()
            
            if enabled:
                self.log("ADPCM audio recording enabled")
                self.log("Recording both raw ADPCM (.bin) and decoded WAV files")
            else:
                self.log("Audio recording disabled")
        except Exception as e:
            self.log(f"Audio toggle error: {e}")
            self.audio_var.set(False)
    
    def toggle_osc_print(self):
        """Toggle OSC message printing"""
        if not self.is_connected or not self.connection:
            messagebox.showwarning("Not Connected", "Please connect to a device first")
            self.osc_print_var.set(False)
            return
        
        try:
            enabled = self.connection.toggle_osc_print()
            self.osc_status_label.config(text=f"OSC Print: {'ON' if enabled else 'OFF'}", 
                                       fg="green" if enabled else "gray")
        except Exception as e:
            self.log(f"OSC print toggle error: {e}")
            self.osc_print_var.set(False)
    
    def toggle_adpcm_debugging(self):
        """Toggle ADPCM debugging - streaming mode only"""
        if not self.is_connected or not self.connection:
            messagebox.showwarning("Not Connected", "Please connect first")
            return
        
        # Only toggle debug output, not decoding mode
        self.connection.debug_pcm_samples = not self.connection.debug_pcm_samples
        
        if self.connection.debug_pcm_samples:
            self.log("ADPCM Debug: Detailed logging ENABLED")
        else:
            self.log("ADPCM Debug: Detailed logging DISABLED")
    
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
            self.adpcm_debug_btn.config(state=tk.NORMAL)
            
            indicator = "🔴 " if self.is_sequoia else ""
            self.log(f"{indicator}Connected successfully!")
            self.log("Expecting 114-byte packets with linear + raw acceleration")
            self.log("Sending OSC data to ports 8888 & 8889")
            
            if self.connection:
                stats = self.connection.get_stats()
                self.log(f"Strategy: {stats['strategy']}")
                self.log("Move device to generate data...")
            
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
        
        self.stop_monitoring()
        
        def run_disconnect():
            try:
                if self.connection:
                    self.connection.close()
                    self.connection = None
                
                if self.client and self.client.is_connected:
                    run_in_ble_loop(self.client.disconnect())
                
                self.root.after(0, self.handle_disconnect_result, "Success")
            except Exception as e:
                self.root.after(0, self.handle_disconnect_result, f"Error: {e}")
        
        threading.Thread(target=run_disconnect, daemon=True).start()
    
    def handle_disconnect_result(self, result):
        """Handle disconnect result"""
        self.is_connected = False
        self.connection_start_time = None
        
        self.status_label.config(text="Status: Disconnected", fg="red")
        self.connect_btn.config(state=tk.NORMAL if self.selected_device else tk.DISABLED)
        self.disconnect_btn.config(state=tk.DISABLED)
        
        # Disable reset buttons
        self.gentle_reset_btn.config(state=tk.DISABLED)
        self.cmd_reset_btn.config(state=tk.DISABLED)
        self.emergency_btn.config(state=tk.DISABLED)
        self.adpcm_debug_btn.config(state=tk.DISABLED)
        
        # Reset audio recording checkbox
        self.audio_var.set(False)
        self.osc_print_var.set(False)
        self.osc_status_label.config(text="OSC Print: OFF", fg="gray")
        
        indicator = "🔴 " if self.is_sequoia else ""
        if result == "Success":
            self.log(f"{indicator}Disconnected successfully")
        else:
            self.log(f"{indicator}Disconnect: {result}")
    
    def start_monitoring(self):
        """Start connection monitoring"""
        self.monitor_active = True
        
        def monitor():
            while self.monitor_active and self.is_connected:
                try:
                    if self.connection:
                        stats = self.connection.get_stats()
                        
                        # DISABLED: Auto-reset based on message count
                        # if not self.reset_in_progress:
                        #     threshold = self.reset_thresholds[self.current_threshold_index]
                        #     if stats['message_count'] >= threshold:
                        #         self.log(f"Auto-reset triggered at {threshold} messages")
                        #         self.gentle_reset()
                        
                        # DISABLED: Data stall detection
                        # if stats['time_since_data'] > self.stall_timeout:
                        #     if not self.reset_in_progress:
                        #         self.log("Data stall detected - triggering reset")
                        #         self.gentle_reset()
                    
                    time.sleep(1)
                
                except Exception as e:
                    self.log(f"Monitor error: {e}")
                    time.sleep(1)
        
        self.monitor_thread = threading.Thread(target=monitor, daemon=True)
        self.monitor_thread.start()
    
    def stop_monitoring(self):
        """Stop connection monitoring"""
        self.monitor_active = False
        if self.monitor_thread and self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=1)
    
    def gentle_reset(self):
        """Perform gentle reset"""
        if self.reset_in_progress or not self.is_connected:
            return
        
        self.reset_in_progress = True
        indicator = "🔴 " if self.is_sequoia else ""
        self.log(f"{indicator}Initiating gentle reset...")
        
        def run_reset():
            try:
                result = run_in_ble_loop(self.connection.soft_reset())
                self.root.after(0, self.handle_reset_result, result, "gentle")
            except Exception as e:
                self.root.after(0, self.handle_reset_result, f"Error: {e}", "gentle")
        
        threading.Thread(target=run_reset, daemon=True).start()
    
    def send_reset(self):
        """Send reset command to device"""
        if self.reset_in_progress or not self.is_connected:
            return
        
        self.reset_in_progress = True
        indicator = "🔴 " if self.is_sequoia else ""
        self.log(f"{indicator}Sending reset command...")
        
        def run_reset():
            try:
                result = run_in_ble_loop(self.connection.send_reset_command())
                self.root.after(0, self.handle_reset_result, result, "command")
            except Exception as e:
                self.root.after(0, self.handle_reset_result, f"Error: {e}", "command")
        
        threading.Thread(target=run_reset, daemon=True).start()
    
    def emergency_reset(self):
        """Perform emergency reset (disconnect and reconnect)"""
        if self.reset_in_progress:
            return
        
        self.reset_in_progress = True
        indicator = "🔴 " if self.is_sequoia else ""
        self.log(f"{indicator}Emergency reset - disconnecting...")
        
        def run_emergency():
            try:
                # Disconnect
                if self.connection:
                    self.connection.close()
                if self.client and self.client.is_connected:
                    run_in_ble_loop(self.client.disconnect())
                
                time.sleep(2)  # Wait before reconnecting
                
                # Reconnect
                result = run_in_ble_loop(self.async_connect())
                self.root.after(0, self.handle_reset_result, result, "emergency")
                
            except Exception as e:
                self.root.after(0, self.handle_reset_result, f"Error: {e}", "emergency")
        
        threading.Thread(target=run_emergency, daemon=True).start()
    
    def handle_reset_result(self, result, reset_type):
        """Handle reset result"""
        self.reset_in_progress = False
        
        indicator = "🔴 " if self.is_sequoia else ""
        
        if result == "Success" or (reset_type == "emergency" and result == "Success"):
            self.log(f"{indicator}{reset_type.title()} reset successful")
            
            # Update reset threshold for next auto-reset
            if reset_type == "gentle":
                self.current_threshold_index = (self.current_threshold_index + 1) % len(self.reset_thresholds)
                next_threshold = self.reset_thresholds[self.current_threshold_index]
                self.reset_label.config(text=f"Next Reset: {next_threshold}")
                
                if self.connection:
                    stats = self.connection.get_stats()
                    self.last_reset_count = stats['message_count']
            
            # Re-enable connection for emergency reset
            if reset_type == "emergency":
                self.handle_connect_result("Success")
        else:
            self.log(f"{indicator}{reset_type.title()} reset failed: {result}")
    
    def update_status(self):
        """Update UI status displays"""
        try:
            if self.is_connected and self.connection:
                stats = self.connection.get_stats()
                
                # Update timer
                if self.connection_start_time:
                    elapsed = time.time() - self.connection_start_time
                    hours = int(elapsed // 3600)
                    minutes = int((elapsed % 3600) // 60)
                    seconds = int(elapsed % 60)
                    self.timer_label.config(text=f"Time: {hours:02d}:{minutes:02d}:{seconds:02d}")
                
                # Update statistics
                self.messages_label.config(text=f"Messages: {stats['message_count']}")
                self.strategy_label.config(text=f"Strategy: {stats['strategy']}")
                self.droprate_label.config(text=f"Drop Rate: {stats['drop_rate']:.1f}%")
                self.effective_label.config(text=f"Effective: {stats['effective_rate']:.1f} msg/s")
                self.queue_label.config(text=f"Queue: {stats['queue_size']}")
            
        except Exception as e:
            pass  # Ignore UI update errors
        
        # Schedule next update
        self.root.after(1000, self.update_status)
    
    def on_closing(self):
        """Handle application closing"""
        self._shutting_down = True
        
        indicator = "🔴 " if self.is_sequoia else ""
        self.log(f"{indicator}Shutting down...")
        
        self.stop_monitoring()
        
        if self.is_connected:
            try:
                if self.connection:
                    self.connection.close()
                if self.client and self.client.is_connected:
                    run_in_ble_loop(self.client.disconnect())
            except:
                pass
        
        self.root.destroy()
    
    def run(self):
        """Run the application"""
        self.root.mainloop()

if __name__ == "__main__":
    print("MetaBow v16 - ADPCM Audio Implementation")
    print(f"Python: {sys.version.split()[0]}")
    print(f"Platform: {platform.system()}")
    
    # Sequoia detection
    is_sequoia, version = detect_sequoia()
    if is_sequoia:
        print(f"🔴 SEQUOIA DETECTED: macOS {version}")
        print("   Ultra-conservative BLE settings will be applied")
    elif platform.system() == 'Darwin':
        print(f"✅ macOS {version} - Standard BLE settings")
    
    # Check dependencies
    try:
        import bleak
        print(f"Bleak: {getattr(bleak, '__version__', 'unknown')}")
    except ImportError:
        print("ERROR: bleak not installed")
        print("Install: pip3 install bleak")
        sys.exit(1)
    
    try:
        import pythonosc
        print("python-osc: available")
    except ImportError:
        print("python-osc: not available (OSC disabled)")
    
    try:
        import numpy
        print(f"NumPy: {numpy.__version__}")
    except ImportError:
        print("ERROR: numpy not installed")
        print("Install: pip3 install numpy")
        sys.exit(1)
    
    print("\nStarting MetaBow v16 ADPCM Audio Implementation...")
    print("Features:")
    print("- ADPCM 4-bit audio decoding (16kHz, 4:1 compression)")
    print("- 114-byte packet format support (linear + raw acceleration)")
    print("- 16-float IMU data parsing (quaternion, linear accel, gyro, mag, raw accel)")
    print("- Real-time WAV file creation from ADPCM")
    print("- Crash-safe audio processing")
    print("- Complete OSC data streaming to ports 8888 & 8889")
    print("- Platform-aware BLE optimization")
    
    app = MetaBowApp()
    app.run()#!/usr/bin/env python3