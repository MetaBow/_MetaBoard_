#!/usr/bin/env python3

import numpy as np
import librosa
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import List, Dict, Any, Optional
import tkinter as tk
from tkinter import ttk

# ===============================
# AUDIO FEATURE EXTRACTION CLASSES
# ===============================

@dataclass
class AudioFeatureConfig:
    """Configuration for individual audio features"""
    name: str
    display_name: str
    category: str
    enabled: bool = False
    parameters: Dict[str, Any] = field(default_factory=dict)
    last_value: Any = None
    osc_path: str = ""
    
    def __post_init__(self):
        if not self.osc_path:
            self.osc_path = f"/metabow/audio/{self.name}"

class RealTimeAudioFeatureExtractor:
    """Real-time audio feature extraction using librosa"""
    
    def __init__(self, sample_rate=44100, frame_size=2048, hop_length=512):
        self.sample_rate = sample_rate
        self.frame_size = frame_size
        self.hop_length = hop_length
        self.buffer_size = frame_size * 4  # Keep 4 frames worth of data
        
        # Audio buffer for real-time processing
        self.audio_buffer = deque(maxlen=self.buffer_size)
        self.processing_enabled = False
        
        # Feature configurations
        self.feature_configs = self._initialize_feature_configs()
        
        # Processing statistics
        self.features_processed = 0
        self.processing_time_ms = 0
        self.last_processing_time = 0
        
        # Threading for real-time processing
        self.processing_thread = None
        self.stop_processing = False
        
        print("Real-time audio feature extractor initialized")
    
    def _initialize_feature_configs(self) -> Dict[str, AudioFeatureConfig]:
        """Initialize all available audio feature configurations"""
        configs = {}
        
        # 1. Spectral Features
        spectral_features = [
            ("mfcc", "MFCC", {"n_mfcc": 13}),
            ("spectral_centroid", "Spectral Centroid", {}),
            ("spectral_bandwidth", "Spectral Bandwidth", {}),
            ("spectral_contrast", "Spectral Contrast", {"n_bands": 6}),
            ("spectral_flatness", "Spectral Flatness", {}),
            ("spectral_rolloff", "Spectral Rolloff", {"roll_percent": 0.85}),
        ]
        
        for name, display_name, params in spectral_features:
            configs[name] = AudioFeatureConfig(
                name=name,
                display_name=display_name,
                category="Spectral",
                parameters=params
            )
        
        # 2. Pitch and Tuning Features
        pitch_features = [
            ("pyin", "PYIN F0 Estimation", {"fmin": 80.0, "fmax": 2000.0}),
            ("piptrack", "Pitch Tracking", {"threshold": 0.1}),
            ("estimate_tuning", "Tuning Deviation", {}),
        ]
        
        for name, display_name, params in pitch_features:
            configs[name] = AudioFeatureConfig(
                name=name,
                display_name=display_name,
                category="Pitch",
                parameters=params
            )
        
        # 3. Temporal & Rhythm Features
        temporal_features = [
            ("onset_detect", "Onset Detection", {"units": "time"}),
            ("tempo", "Tempo Analysis", {}),
            ("zero_crossing_rate", "Zero Crossing Rate", {}),
            ("rms", "RMS Energy", {"frame_length": 2048}),
        ]
        
        for name, display_name, params in temporal_features:
            configs[name] = AudioFeatureConfig(
                name=name,
                display_name=display_name,
                category="Temporal",
                parameters=params
            )
        
        # 4. Chroma Features
        chroma_features = [
            ("chroma_stft", "Chroma STFT", {"n_chroma": 12}),
            ("chroma_cqt", "Chroma CQT", {"n_chroma": 12}),
            ("chroma_cens", "Chroma CENS", {"n_chroma": 12}),
        ]
        
        for name, display_name, params in chroma_features:
            configs[name] = AudioFeatureConfig(
                name=name,
                display_name=display_name,
                category="Chroma",
                parameters=params
            )
        
        # 5. Advanced Features
        advanced_features = [
            ("tonnetz", "Tonnetz", {}),
            ("harmonics", "Harmonic Analysis", {"margin": 8}),
        ]
        
        for name, display_name, params in advanced_features:
            configs[name] = AudioFeatureConfig(
                name=name,
                display_name=display_name,
                category="Advanced",
                parameters=params
            )
        
        return configs
    
    def set_enabled(self, enabled: bool):
        """Enable or disable feature extraction"""
        self.processing_enabled = enabled
        
        if enabled and not self.processing_thread:
            self.start_processing_thread()
        elif not enabled and self.processing_thread:
            self.stop_processing_thread()
        
        print(f"Audio feature extraction {'enabled' if enabled else 'disabled'}")
    
    def start_processing_thread(self):
        """Start the real-time processing thread"""
        if self.processing_thread and self.processing_thread.is_alive():
            return
        
        self.stop_processing = False
        self.processing_thread = threading.Thread(target=self._processing_loop, daemon=True)
        self.processing_thread.start()
        print("Audio feature processing thread started")
    
    def stop_processing_thread(self):
        """Stop the real-time processing thread"""
        self.stop_processing = True
        if self.processing_thread:
            self.processing_thread.join(timeout=1.0)
            self.processing_thread = None
        print("Audio feature processing thread stopped")
    
    def add_audio_data(self, audio_samples: np.ndarray):
        """Add new audio data to the processing buffer"""
        if not self.processing_enabled:
            return
        
        # Ensure audio is float32 and normalized
        if audio_samples.dtype != np.float32:
            audio_samples = audio_samples.astype(np.float32)
        
        # Normalize if needed
        if np.max(np.abs(audio_samples)) > 1.0:
            audio_samples = audio_samples / 32767.0
        
        # Add to buffer
        self.audio_buffer.extend(audio_samples)
    
    def _processing_loop(self):
        """Main processing loop for real-time feature extraction"""
        while not self.stop_processing and self.processing_enabled:
            try:
                if len(self.audio_buffer) >= self.frame_size:
                    # Extract a frame for processing
                    frame = np.array(list(self.audio_buffer)[-self.frame_size:])
                    
                    # Process enabled features
                    start_time = time.time()
                    self._extract_features(frame)
                    self.processing_time_ms = (time.time() - start_time) * 1000
                    self.last_processing_time = time.time()
                    self.features_processed += 1
                
                # Control processing rate (avoid overloading)
                time.sleep(0.01)  # 100 Hz max processing rate
                
            except Exception as e:
                print(f"Error in feature processing loop: {e}")
                time.sleep(0.1)
    
    def _extract_features(self, audio_frame: np.ndarray):
        """Extract enabled features from audio frame"""
        try:
            for feature_name, config in self.feature_configs.items():
                if not config.enabled:
                    continue
                
                try:
                    feature_value = self._extract_single_feature(audio_frame, feature_name, config)
                    config.last_value = feature_value
                    
                except Exception as e:
                    print(f"Error extracting {feature_name}: {e}")
                    config.last_value = None
        
        except Exception as e:
            print(f"Error in feature extraction: {e}")
    
    def _extract_single_feature(self, audio_frame: np.ndarray, feature_name: str, config: AudioFeatureConfig):
        """Extract a single feature from audio frame"""
        params = config.parameters
        
        try:
            # Spectral Features
            if feature_name == "mfcc":
                mfccs = librosa.feature.mfcc(
                    y=audio_frame, 
                    sr=self.sample_rate,
                    n_mfcc=params.get("n_mfcc", 13),
                    hop_length=self.hop_length
                )
                return np.mean(mfccs, axis=1).tolist()
            
            elif feature_name == "spectral_centroid":
                centroid = librosa.feature.spectral_centroid(
                    y=audio_frame,
                    sr=self.sample_rate,
                    hop_length=self.hop_length
                )
                return float(np.mean(centroid))
            
            elif feature_name == "spectral_bandwidth":
                bandwidth = librosa.feature.spectral_bandwidth(
                    y=audio_frame,
                    sr=self.sample_rate,
                    hop_length=self.hop_length
                )
                return float(np.mean(bandwidth))
            
            elif feature_name == "spectral_contrast":
                contrast = librosa.feature.spectral_contrast(
                    y=audio_frame,
                    sr=self.sample_rate,
                    n_bands=params.get("n_bands", 6),
                    hop_length=self.hop_length
                )
                return np.mean(contrast, axis=1).tolist()
            
            elif feature_name == "spectral_flatness":
                flatness = librosa.feature.spectral_flatness(
                    y=audio_frame,
                    hop_length=self.hop_length
                )
                return float(np.mean(flatness))
            
            elif feature_name == "spectral_rolloff":
                rolloff = librosa.feature.spectral_rolloff(
                    y=audio_frame,
                    sr=self.sample_rate,
                    roll_percent=params.get("roll_percent", 0.85),
                    hop_length=self.hop_length
                )
                return float(np.mean(rolloff))
            
            # Pitch Features
            elif feature_name == "pyin":
                f0, _, _ = librosa.pyin(
                    audio_frame,
                    fmin=params.get("fmin", 80.0),
                    fmax=params.get("fmax", 2000.0),
                    sr=self.sample_rate
                )
                valid_f0 = f0[~np.isnan(f0)]
                return float(np.median(valid_f0)) if len(valid_f0) > 0 else 0.0
            
            elif feature_name == "piptrack":
                pitches, magnitudes = librosa.piptrack(
                    y=audio_frame,
                    sr=self.sample_rate,
                    threshold=params.get("threshold", 0.1),
                    hop_length=self.hop_length
                )
                pitch_idx = np.argmax(magnitudes, axis=0)
                frame_pitches = []
                for t in range(pitches.shape[1]):
                    pitch = pitches[pitch_idx[t], t]
                    frame_pitches.append(pitch if pitch > 0 else 0.0)
                return float(np.mean(frame_pitches))
            
            elif feature_name == "estimate_tuning":
                tuning = librosa.estimate_tuning(y=audio_frame, sr=self.sample_rate)
                return float(tuning)
            
            # Temporal Features
            elif feature_name == "onset_detect":
                onsets = librosa.onset.onset_detect(
                    y=audio_frame,
                    sr=self.sample_rate,
                    units=params.get("units", "time"),
                    hop_length=self.hop_length
                )
                return len(onsets)
            
            elif feature_name == "tempo":
                tempo, _ = librosa.beat.beat_track(
                    y=audio_frame,
                    sr=self.sample_rate,
                    hop_length=self.hop_length
                )
                return float(tempo)
            
            elif feature_name == "zero_crossing_rate":
                zcr = librosa.feature.zero_crossing_rate(
                    audio_frame,
                    hop_length=self.hop_length
                )
                return float(np.mean(zcr))
            
            elif feature_name == "rms":
                rms = librosa.feature.rms(
                    y=audio_frame,
                    frame_length=params.get("frame_length", 2048),
                    hop_length=self.hop_length
                )
                return float(np.mean(rms))
            
            # Chroma Features
            elif feature_name == "chroma_stft":
                chroma = librosa.feature.chroma_stft(
                    y=audio_frame,
                    sr=self.sample_rate,
                    n_chroma=params.get("n_chroma", 12),
                    hop_length=self.hop_length
                )
                return np.mean(chroma, axis=1).tolist()
            
            elif feature_name == "chroma_cqt":
                chroma = librosa.feature.chroma_cqt(
                    y=audio_frame,
                    sr=self.sample_rate,
                    n_chroma=params.get("n_chroma", 12),
                    hop_length=self.hop_length
                )
                return np.mean(chroma, axis=1).tolist()
            
            elif feature_name == "chroma_cens":
                chroma = librosa.feature.chroma_cens(
                    y=audio_frame,
                    sr=self.sample_rate,
                    n_chroma=params.get("n_chroma", 12),
                    hop_length=self.hop_length
                )
                return np.mean(chroma, axis=1).tolist()
            
            # Advanced Features
            elif feature_name == "tonnetz":
                tonnetz = librosa.feature.tonnetz(
                    y=audio_frame,
                    sr=self.sample_rate
                )
                return np.mean(tonnetz, axis=1).tolist()
            
            elif feature_name == "harmonics":
                harmonics, percussive = librosa.effects.hpss(
                    audio_frame,
                    margin=params.get("margin", 8)
                )
                harmonic_energy = np.sum(harmonics**2)
                total_energy = np.sum(audio_frame**2)
                return float(harmonic_energy / (total_energy + 1e-10))
            
            else:
                return None
                
        except Exception as e:
            print(f"Error extracting {feature_name}: {e}")
            return None
    
    def get_enabled_features(self) -> List[str]:
        """Get list of currently enabled features"""
        return [name for name, config in self.feature_configs.items() if config.enabled]
    
    def set_feature_enabled(self, feature_name: str, enabled: bool):
        """Enable or disable a specific feature"""
        if feature_name in self.feature_configs:
            self.feature_configs[feature_name].enabled = enabled
            print(f"Feature {feature_name} {'enabled' if enabled else 'disabled'}")
    
    def get_feature_value(self, feature_name: str):
        """Get the latest value for a feature"""
        if feature_name in self.feature_configs:
            return self.feature_configs[feature_name].last_value
        return None
    
    def get_all_feature_values(self) -> Dict[str, Any]:
        """Get all current feature values"""
        return {
            name: config.last_value 
            for name, config in self.feature_configs.items() 
            if config.enabled and config.last_value is not None
        }
    
    def get_processing_stats(self) -> Dict[str, Any]:
        """Get processing statistics"""
        return {
            "features_processed": self.features_processed,
            "processing_time_ms": self.processing_time_ms,
            "enabled_features": len(self.get_enabled_features()),
            "buffer_size": len(self.audio_buffer),
            "processing_rate_hz": 1.0 / max(0.001, time.time() - self.last_processing_time) if self.last_processing_time > 0 else 0
        }


# ===============================
# AUDIO FEATURE CONFIGURATION WINDOW
# ===============================

class AudioFeatureConfigWindow:
    """Configuration window for audio feature extraction"""
    
    def __init__(self, parent, feature_extractor: RealTimeAudioFeatureExtractor):
        self.parent = parent
        self.feature_extractor = feature_extractor
        self.window = None
        
        # UI variables
        self.enabled_var = tk.BooleanVar(value=feature_extractor.processing_enabled)
        self.feature_vars = {}
        self.stats_text = None
        self.update_stats_job = None
        
        # Initialize feature checkboxes
        for feature_name in feature_extractor.feature_configs:
            self.feature_vars[feature_name] = tk.BooleanVar(value=False)
    
    def show(self):
        """Show the configuration window"""
        if self.window and self.window.winfo_exists():
            self.window.lift()
            return
        
        self.window = tk.Toplevel(self.parent)
        self.window.title("Audio Feature Extraction Configuration")
        self.window.geometry("800x700")
        self.window.resizable(True, True)
        
        # Make window modal
        self.window.transient(self.parent)
        self.window.grab_set()
        
        self.create_widgets()
        self.start_stats_update()
        
        # Handle window close
        self.window.protocol("WM_DELETE_WINDOW", self.on_close)
    
    def create_widgets(self):
        """Create all widgets for the configuration window"""
        main_frame = ttk.Frame(self.window)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Enable/Disable Section
        enable_frame = ttk.LabelFrame(main_frame, text="Feature Extraction Control")
        enable_frame.pack(fill=tk.X, pady=5)
        
        ttk.Checkbutton(
            enable_frame, 
            text="Enable Real-time Audio Feature Extraction",
            variable=self.enabled_var,
            command=self.on_enable_change
        ).pack(anchor=tk.W, padx=10, pady=5)
        
        # Feature Selection Section
        features_frame = ttk.LabelFrame(main_frame, text="Feature Selection")
        features_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # Create notebook for categorized features
        notebook = ttk.Notebook(features_frame)
        notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Group features by category
        categories = {}
        for feature_name, config in self.feature_extractor.feature_configs.items():
            category = config.category
            if category not in categories:
                categories[category] = []
            categories[category].append((feature_name, config))
        
        # Create tabs for each category
        for category_name, features in categories.items():
            category_frame = ttk.Frame(notebook)
            notebook.add(category_frame, text=category_name)
            
            # Add features to frame
            for feature_name, config in features:
                feature_frame = ttk.Frame(category_frame)
                feature_frame.pack(fill=tk.X, padx=5, pady=2)
                
                ttk.Checkbutton(
                    feature_frame,
                    text=config.display_name,
                    variable=self.feature_vars[feature_name],
                    command=lambda fn=feature_name: self.on_feature_toggle(fn)
                ).pack(side=tk.LEFT, anchor=tk.W)
                
                # Show OSC path
                ttk.Label(
                    feature_frame,
                    text=f"OSC: {config.osc_path}",
                    font=('Courier', 8),
                    foreground='gray'
                ).pack(side=tk.RIGHT, anchor=tk.E)
        
        # Quick selection buttons
        quick_frame = ttk.Frame(features_frame)
        quick_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Button(quick_frame, text="Select All", command=self.select_all_features).pack(side=tk.LEFT, padx=5)
        ttk.Button(quick_frame, text="Clear All", command=self.clear_all_features).pack(side=tk.LEFT, padx=5)
        ttk.Button(quick_frame, text="Violin Preset", command=self.select_violin_preset).pack(side=tk.LEFT, padx=5)
        ttk.Button(quick_frame, text="Basic Preset", command=self.select_basic_preset).pack(side=tk.LEFT, padx=5)
        
        # Statistics Section
        stats_frame = ttk.LabelFrame(main_frame, text="Real-time Processing Statistics")
        stats_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        self.stats_text = tk.Text(stats_frame, height=12, wrap=tk.WORD, font=('Courier', 9))
        self.stats_text.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Action Buttons
        button_frame = ttk.Frame(main_frame)
        button_frame.pack(fill=tk.X, pady=10)
        
        ttk.Button(button_frame, text="Close", command=self.on_close).pack(side=tk.RIGHT, padx=5)
    
    # Event handlers
    def on_enable_change(self):
        """Handle enable/disable change"""
        self.feature_extractor.set_enabled(self.enabled_var.get())
    
    def on_feature_toggle(self, feature_name):
        """Handle individual feature toggle"""
        enabled = self.feature_vars[feature_name].get()
        self.feature_extractor.set_feature_enabled(feature_name, enabled)
    
    def select_all_features(self):
        """Enable all features"""
        for feature_name, var in self.feature_vars.items():
            var.set(True)
            self.feature_extractor.set_feature_enabled(feature_name, True)
    
    def clear_all_features(self):
        """Disable all features"""
        for feature_name, var in self.feature_vars.items():
            var.set(False)
            self.feature_extractor.set_feature_enabled(feature_name, False)
    
    def select_violin_preset(self):
        """Select features most relevant for violin analysis"""
        violin_features = [
            "mfcc", "spectral_centroid", "spectral_contrast", "spectral_flatness",
            "pyin", "estimate_tuning", "rms", "zero_crossing_rate",
            "chroma_stft", "onset_detect", "harmonics"
        ]
        
        self.clear_all_features()
        
        for feature_name in violin_features:
            if feature_name in self.feature_vars:
                self.feature_vars[feature_name].set(True)
                self.feature_extractor.set_feature_enabled(feature_name, True)
    
    def select_basic_preset(self):
        """Select basic features for general audio analysis"""
        basic_features = [
            "spectral_centroid", "rms", "zero_crossing_rate", 
            "pyin", "mfcc", "chroma_stft"
        ]
        
        self.clear_all_features()
        
        for feature_name in basic_features:
            if feature_name in self.feature_vars:
                self.feature_vars[feature_name].set(True)
                self.feature_extractor.set_feature_enabled(feature_name, True)
    
    def start_stats_update(self):
        """Start periodic statistics updates"""
        self.update_stats()
    
    def update_stats(self):
        """Update statistics display"""
        if not self.window or not self.window.winfo_exists():
            return
        
        try:
            self.stats_text.delete(1.0, tk.END)
            
            stats = self.feature_extractor.get_processing_stats()
            self.stats_text.insert(tk.END, f"Processing Status: {'Active' if self.feature_extractor.processing_enabled else 'Inactive'}\n")
            self.stats_text.insert(tk.END, f"Features Processed: {stats['features_processed']}\n")
            self.stats_text.insert(tk.END, f"Processing Time: {stats['processing_time_ms']:.2f} ms\n")
            self.stats_text.insert(tk.END, f"Enabled Features: {stats['enabled_features']}\n")
            self.stats_text.insert(tk.END, f"Buffer Size: {stats['buffer_size']}\n")
            self.stats_text.insert(tk.END, f"Processing Rate: {stats['processing_rate_hz']:.1f} Hz\n\n")
            
            # Current feature values
            feature_values = self.feature_extractor.get_all_feature_values()
            if feature_values:
                self.stats_text.insert(tk.END, "Current Feature Values:\n")
                self.stats_text.insert(tk.END, "-" * 40 + "\n")
                
                for feature_name, value in feature_values.items():
                    config = self.feature_extractor.feature_configs[feature_name]
                    
                    if isinstance(value, list):
                        if len(value) <= 3:
                            value_str = f"[{', '.join(f'{v:.3f}' for v in value)}]"
                        else:
                            value_str = f"[{value[0]:.3f}, ..., {value[-1]:.3f}] ({len(value)} values)"
                    elif isinstance(value, (int, float)):
                        value_str = f"{value:.4f}"
                    else:
                        value_str = str(value)
                    
                    self.stats_text.insert(tk.END, f"{config.display_name}: {value_str}\n")
            
            # Schedule next update
            self.update_stats_job = self.window.after(1000, self.update_stats)
            
        except Exception as e:
            print(f"Error updating feature stats: {e}")
    
    def on_close(self):
        """Handle window close"""
        if self.update_stats_job:
            self.window.after_cancel(self.update_stats_job)
        
        self.window.grab_release()
        self.window.destroy()
        self.window = None