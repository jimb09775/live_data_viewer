import numpy as np
import serial
import threading
from queue import Queue, Empty
import time
import pyqtgraph as pg
from PyQt5 import QtWidgets, QtCore
import sys
import csv
from datetime import datetime


# --- Constants ---
NUM_POINTS = 400

POINT_SIZE = 6   # 2 bytes index + 4 bytes amplitude (single points only)
HEADER_BYTE = 0xAA
TRAILER_BYTE = 0xA0
LOOPBACK_HEADER = 0xCC  # Loopback data marker


RF_FACTOR = 1.48 


START = (0* 0.0075) / RF_FACTOR
STEP = 0.0075 / RF_FACTOR


# --- Threshold LUT Loader ---
def load_threshold_lut(h_file="threshold_lut.h", c_file="threshold_lut.c"):
    """
    Parse threshold_lut.h for start/step metadata and threshold_lut.c for the
    amplitude array.  Returns (x_meters, y_amplitudes) ready for set_threshold_line().

    Falls back to empty lists if either file is missing or malformed.
    """
    import re, os

    # ---- parse header for start_mm and step_mm ----
    start_mm = None
    step_mm  = None
    try:
        with open(h_file, "r") as f:
            header_text = f.read()
        m = re.search(r"#define\s+THRESHOLD_LUT_START_MM\s+([\d.eE+\-]+)f?", header_text)
        if m:
            start_mm = float(m.group(1))
        m = re.search(r"#define\s+THRESHOLD_LUT_STEP_MM\s+([\d.eE+\-]+)f?", header_text)
        if m:
            step_mm = float(m.group(1))
    except FileNotFoundError:
        print(f"⚠️  threshold LUT: '{h_file}' not found — threshold line disabled")
        return None

    if start_mm is None or step_mm is None:
        print("⚠️  threshold LUT: could not parse START_MM / STEP_MM from header")
        return None

    # ---- parse .c file for array values (also returns raw int array for numpy use) ----
    try:
        with open(c_file, "r") as f:
            c_text = f.read()
    except FileNotFoundError:
        print(f"⚠️  threshold LUT: '{c_file}' not found — threshold line disabled")
        return None

    # Extract everything between the opening { and closing };
    m = re.search(r"g_threshold_lut\[.*?\]\s*=\s*\{(.*?)\};", c_text, re.DOTALL)
    if not m:
        print("⚠️  threshold LUT: could not locate array body in .c file")
        return None

    # Strip C comments and extract all integers
    body = re.sub(r"/\*.*?\*/", "", m.group(1), flags=re.DOTALL)
    values = [int(v.rstrip("U")) for v in re.findall(r"\b\d+U?\b", body)]

    if not values:
        print("⚠️  threshold LUT: no values found in array body")
        return None

    n = len(values)
    lut_arr = np.array(values, dtype=np.float32)

    print(f"✅  threshold LUT loaded: {n} points, amp {min(values)}–{max(values)}")
    return lut_arr

FIXED_GRAPH_Y = True  # Set to True to fix Y axis, False for auto-scaling
Y_LIMITS = (0, 200000)  # Used if FIXED_GRAPH_Y is True 

# Crossing detection — skip near-field clutter below this distance (match CONFIG_MIN_RANGE_MM)
MIN_DETECTION_RANGE_MM = 100.0

# Loopback display settings
LOOPBACK_START_POINT = -15 # CONFIG_LOOPBACK_START_POINT
LOOPBACK_NUM_POINTS = 6 # CONFIG_LOOPBACK_NUM_POINTS
LOOPBACK_STEP = 1  # CONFIG_LOOPBACK_STEP
LOOPBACK_Y_LIMITS = (0, 300)  # Loopback typically has higher amplitudes


# ---  High-Performance Serial Worker ---
class SerialReader:
    """Multi-threaded, bulk-reading, ultra-optimized serial worker"""
    
    def __init__(self, port: str, baudrate: int, data_queue: Queue):
        self.port = port
        self.baudrate = baudrate
        self.data_queue = data_queue
        self.raw_queue = Queue(maxsize=5000)
        self.running = True
        
        self.bytes_read = 0
        self.packets_processed = 0
        self.errors = 0
        
    def start(self):
        self.reader_thread = threading.Thread(target=self.__serial_reader, daemon=True)
        self.processor_thread = threading.Thread(target=self.__packet_processor, daemon=True)
        self.reader_thread.start()
        self.processor_thread.start()
        print(f"🚀 Serial Reader started: {self.port} @ {self.baudrate}")
        return self.reader_thread, self.processor_thread
    
    def __serial_reader(self):
        try:
            ser = serial.Serial(
                self.port,
                self.baudrate,
                timeout=0.002,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
            
            ser.set_buffer_size(rx_size=2147483647, tx_size=1024)
            ser.reset_input_buffer()
            print("⚡ Serial connected with 64KB buffer")
            
            while self.running:
                try:
                    available = ser.in_waiting
                    if available > 0:
                        data = ser.read(min(available, 8192))
                        if data:
                            self.bytes_read += len(data)
                            self.raw_queue.put_nowait(data)
                    else:
                        time.sleep(0.00005)
                except Exception:
                    self.errors += 1
                    time.sleep(0.001)
        except Exception as e:
            print(f"❌ Serial reader error: {e}")
        finally:
            if 'ser' in locals():
                ser.close()
                print("📴 Serial port closed")
    
    def __packet_processor(self):
        buffer = bytearray()
        HEADER = HEADER_BYTE
        TRAILER = TRAILER_BYTE
        MAX_POINTS = NUM_POINTS
        METADATA_HEADER = 0xBB  # New header for temp/divisor metadata
        LOOPBACK_HDR = LOOPBACK_HEADER  # 0xCC for loopback data
        
        while self.running:
            try:
                raw_data = self.raw_queue.get(timeout=0.005)
                buffer.extend(raw_data)
                
                # Use 7 here so 7-byte metadata packets ([0xBB]..[0xA0]) are processed
                while len(buffer) >= 7:
                    # Find the first valid header byte
                    header_pos = -1
                    header_type = None
                    
                    for i in range(len(buffer)):
                        if buffer[i] == HEADER:
                            header_pos = i
                            header_type = 'amplitude'
                            break
                        elif buffer[i] == METADATA_HEADER:
                            header_pos = i
                            header_type = 'metadata'
                            break
                        elif buffer[i] == LOOPBACK_HDR:
                            header_pos = i
                            header_type = 'loopback'
                            break
                    
                    if header_pos == -1:
                        buffer.clear()
                        break
                    
                    if header_pos > 0:
                        buffer = buffer[header_pos:]
                    
                    # Process based on header type
                    if header_type == 'metadata':
                        if len(buffer) < 7:
                            break
                        # Metadata packet: [0xBB][temp_h][temp_l][div_h][div_l][chk][0xA0]
                        if buffer[6] == TRAILER:
                            # Validate simple XOR checksum used by sender
                            chk = buffer[5]
                            computed = buffer[1] ^ buffer[2] ^ buffer[3] ^ buffer[4]
                            if chk == computed:
                                temp = (buffer[1] << 8) | buffer[2]
                                if int(temp) > 150:
                                    temp -= 65536
                                divisor = (buffer[3] << 8) | buffer[4]
                                try:
                                    self.data_queue.put_nowait(('metadata', temp, divisor))
                                    self.packets_processed += 1
                                except:
                                    pass
                                buffer = buffer[7:]
                            else:
                                # Checksum mismatch: count error and skip this header byte
                                self.errors += 1
                                buffer = buffer[1:]
                        else:
                            buffer = buffer[1:]
                    
                    elif header_type == 'loopback':
                        if len(buffer) < 10:
                            break
                        # Loopback packet: [0xCC][idx_l][idx_h][amp(4)][len_l][len_h][0xA0]
                        if buffer[9] == TRAILER:
                            idx = buffer[1] | (buffer[2] << 8)
                            amp = (buffer[3] |
                                   (buffer[4] << 8) |
                                   (buffer[5] << 16) |
                                   (buffer[6] << 24))
                            # Interpret as signed 32-bit integer
                            if amp >= 0x80000000:
                                amp -= 0x100000000
                            total_len = buffer[7] | (buffer[8] << 8)
                            if 0 <= idx < LOOPBACK_NUM_POINTS:
                                try:
                                    self.data_queue.put_nowait(('loopback', idx, amp, total_len))
                                    self.packets_processed += 1
                                except:
                                    pass
                            buffer = buffer[10:]
                        else:
                            buffer = buffer[1:]
                    
                    elif header_type == 'amplitude':
                        if len(buffer) < 8:
                            break
                        if buffer[7] == TRAILER:
                            idx = buffer[1] | (buffer[2] << 8)
                            amp = (buffer[3] |
                                   (buffer[4] << 8) |
                                   (buffer[5] << 16) |
                                   (buffer[6] << 24))
                            # Interpret as signed 32-bit integer
                            if amp >= 0x80000000:
                                amp -= 0x100000000
                            if 0 <= idx < MAX_POINTS:
                                try:
                                    self.data_queue.put_nowait((idx, amp))
                                    self.packets_processed += 1
                                except:
                                    pass
                            buffer = buffer[8:]
                        else:
                            buffer = buffer[1:]
                    else:
                        buffer = buffer[1:]
            except Empty:
                continue
            except Exception:
                self.errors += 1
    
    def stop(self):
        self.running = False
    
    def get_performance_stats(self):
        return {
            'bytes_read': self.bytes_read,
            'packets_processed': self.packets_processed,
            'errors': self.errors,
            'raw_queue_size': self.raw_queue.qsize(),
            'data_queue_size': self.data_queue.qsize()
        }


# ---  High-Performance GUI ---
class LiveDataViewer(QtWidgets.QMainWindow):
    def __init__(self, port="COM5", baud=2000000, debug=False, csv_mode=False, csv_file=None):
        super().__init__()
        self.port = port
        self.baud = baud
        self.debug = debug
        self.csv_mode = csv_mode
        self.csv_file = csv_file
        self.csv_writer = None
        self.csv_filehandle = None
        self.sweep_count = 0
        self.csv_logging_active = False

        self.data_queue = Queue(maxsize=10000)
        self.display_data = np.zeros(NUM_POINTS, dtype=np.float32)
        self.display_temp = 0
        self.display_divisor = 0
        
        # Loopback data array
        self.loopback_data = np.zeros(LOOPBACK_NUM_POINTS, dtype=np.float32)
        self.loopback_range_values = (np.arange(LOOPBACK_NUM_POINTS, dtype=np.float32) * 
                                       (LOOPBACK_STEP * 0.0025 ) + 
                                       (LOOPBACK_START_POINT * 0.0025))
        self.loopback_sweep_count = 0
        self.loopback_packets_received = 0
        self.loopback_last_idx = -1
        
        # Temperature and divisor tracking
        self.current_temp = 0
        self.current_divisor = 0
        self.temp_history = []
        self.divisor_history = []
        self.temp_timestamps = []

        self.range_values = (np.arange(NUM_POINTS, dtype=np.float32) * STEP + START) 

        self.init__ui()
        
        if self.csv_mode:
            self.start_csv_logging()

        self.serial_reader = SerialReader(port, baud, self.data_queue)
        self.reader_threads = self.serial_reader.start()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._update_plot)
        self.timer.start(3)
        
        self.update_counter = 0
        self.points_received = 0
        self.last_status_update = time.time()
        self.fps_counter = 0
        self.last_fps_time = time.time()

    def init__ui(self):
        self.setWindowTitle(f"🚀 Live Data Viewer w/ Thresholds - {self.port}")
        self.setGeometry(100, 100, 1200, 800)

        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        layout = QtWidgets.QVBoxLayout(central_widget)

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setLabel('left', 'Amplitude')
        self.plot_widget.setLabel('bottom', 'Range (m)     T: --   D: --')
        self.plot_widget.showGrid(x=True, y=True)
        self.plot_widget.setMouseEnabled(x=True, y=True)
        self.plot_widget.setBackground("w")
        self.plot_widget.setClipToView(True)
        self.plot_widget.setDownsampling(mode='subsample')
        self.plot_widget.setAutoVisible(y=True)
        if FIXED_GRAPH_Y:
            self.plot_widget.setYRange(Y_LIMITS[0], Y_LIMITS[1])
        else:   
            self.plot_widget.enableAutoRange(axis='y', enable=True)

        self.curve = self.plot_widget.plot(
            self.range_values,
            self.display_data,
            pen=pg.mkPen(color='black', width=1.5),
            antialias=False,
            skipFiniteCheck=True,
            autoDownsample=True
        )

        self.threshold_x = []
        self.threshold_y = []
        self.threshold_lut_arr = None   # numpy array for crossing detection, set after load
        self.threshold_curve = self.plot_widget.plot(
            self.threshold_x,
            self.threshold_y,
            pen=pg.mkPen(color='red', width=2, style=QtCore.Qt.DashLine),
            antialias=False
        )
        self.threshold_visible = True

        # --- Threshold crossing vertical line ---
        self.crossing_line = pg.InfiniteLine(
            pos=0.0,
            angle=90,
            pen=pg.mkPen(color='#ff6600', width=2, style=QtCore.Qt.SolidLine),
            movable=False,
            label='',
        )
        self.crossing_line.setVisible(False)   # hidden until a crossing is found
        self.plot_widget.addItem(self.crossing_line)

        self.crossing_label = pg.TextItem(text="", color='#ff6600', anchor=(0, 1))
        self.plot_widget.addItem(self.crossing_label)
        
        # Add text items for temp and divisor display


        layout.addWidget(self.plot_widget)
        
        # --- Loopback Plot Widget ---
        self.loopback_plot_widget = pg.PlotWidget()
        self.loopback_plot_widget.setLabel('left', 'Loopback Amplitude')
        self.loopback_plot_widget.setLabel('bottom', 'Range (m)')
        self.loopback_plot_widget.showGrid(x=True, y=True)
        self.loopback_plot_widget.setMouseEnabled(x=True, y=True)
        self.loopback_plot_widget.setBackground("#f0f0ff")
        self.loopback_plot_widget.setClipToView(True)
        self.loopback_plot_widget.setYRange(LOOPBACK_Y_LIMITS[0], LOOPBACK_Y_LIMITS[1])
        self.loopback_plot_widget.setMaximumHeight(200)
        
        self.loopback_curve = self.loopback_plot_widget.plot(
            self.loopback_range_values,
            self.loopback_data,
            pen=pg.mkPen(color='blue', width=1.5),
            antialias=False,
            skipFiniteCheck=True
        )
        
        # Add loopback sweep count text
        self.loopback_text = pg.TextItem(text="Loopback: --", color=(0, 0, 200), anchor=(0, 1))
        self.loopback_text.setPos(self.loopback_range_values[0], LOOPBACK_Y_LIMITS[1] * 0.9)
        self.loopback_plot_widget.addItem(self.loopback_text)
        
        layout.addWidget(self.loopback_plot_widget)
        
        self.init__controls(layout)
        self.status_bar = self.statusBar()
        self.status_bar.showMessage(f"🚀 MODE: {self.port} @ {self.baud} baud")

    def init__controls(self, layout):
        control_layout = QtWidgets.QHBoxLayout()
        
        self.reset_button = QtWidgets.QPushButton("🔄 Reset Display")
        self.reset_button.clicked.connect(self.reset_display_data)
        self.reset_button.setStyleSheet("QPushButton { background-color: #ff4444; color: white; font-weight: bold; }")
        control_layout.addWidget(self.reset_button)
        
        self.stats_button = QtWidgets.QPushButton("📊 Performance Stats")
        self.stats_button.clicked.connect(self.show_performance_stats)
        self.stats_button.setStyleSheet("QPushButton { background-color: #4444ff; color: white; font-weight: bold; }")
        control_layout.addWidget(self.stats_button)

        # --- Toggle Threshold Button ---
        self.toggle_thresh_button = QtWidgets.QPushButton("⚙️ Toggle Threshold")
        self.toggle_thresh_button.clicked.connect(self.toggle_threshold)
        self.toggle_thresh_button.setStyleSheet("QPushButton { background-color: #00aa00; color: white; font-weight: bold; }")
        control_layout.addWidget(self.toggle_thresh_button)
        
        # --- CSV Logging Button ---
        self.csv_button = QtWidgets.QPushButton("💾 Start CSV Log")
        self.csv_button.clicked.connect(self.toggle_csv_logging)
        self.csv_button.setStyleSheet("QPushButton { background-color: #ff8800; color: white; font-weight: bold; }")
        control_layout.addWidget(self.csv_button)

        control_layout.addStretch()
        layout.addLayout(control_layout)

    def reset_display_data(self):
        self.display_data.fill(0)
        self.curve.setData(self.range_values, self.display_data)
        self.status_bar.showMessage("🔄 Display data reset")

    def show_performance_stats(self):
        stats = self.serial_reader.get_performance_stats()
        msg = (f"📊 STATS: {stats['packets_processed']} packets | "
               f"{stats['bytes_read']} bytes | "
               f"{stats['errors']} errors | "
               f"Raw Q: {stats['raw_queue_size']} | "
               f"Data Q: {stats['data_queue_size']}")
        print(msg)
        self.status_bar.showMessage(msg)

    def set_threshold_line(self, x_data, y_data):
        """Update the threshold line data."""
        if len(x_data) == len(y_data):
            self.threshold_x = np.array(x_data, dtype=np.float32)
            self.threshold_y = np.array(y_data, dtype=np.float64)
            self.threshold_curve.setData(self.threshold_x, self.threshold_y)
        else:
            print("❌ Threshold arrays must be same length")

    def toggle_threshold(self):
        """Show/hide threshold line"""
        self.threshold_visible = not self.threshold_visible
        self.threshold_curve.setVisible(self.threshold_visible)
        self.status_bar.showMessage("👁 Threshold: " + ("Visible" if self.threshold_visible else "Hidden"))
    
    def start_csv_logging(self):
        """Start CSV logging to file"""
        if self.csv_logging_active:
            return
        
        if not self.csv_file:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.csv_file = f"sweep_data_{timestamp}.csv"
        
        try:
            self.csv_filehandle = open(self.csv_file, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_filehandle)
            
            # Write header
            header = ['sweep_num', 'timestamp', 'temperature', 'divisor'] + [f'range_{i}_{self.range_values[i]:.4f}m' for i in range(NUM_POINTS)]
            self.csv_writer.writerow(header)
            self.csv_filehandle.flush()
            
            self.csv_logging_active = True
            self.csv_button.setText("💾 Stop CSV Log")
            self.csv_button.setStyleSheet("QPushButton { background-color: #00aa00; color: white; font-weight: bold; }")
            self.status_bar.showMessage(f"💾 CSV logging started: {self.csv_file}")
            print(f"💾 CSV logging started: {self.csv_file}")
        except Exception as e:
            print(f"❌ Failed to start CSV logging: {e}")
            self.status_bar.showMessage(f"❌ CSV logging failed: {e}")
    
    def stop_csv_logging(self):
        """Stop CSV logging and close file"""
        if not self.csv_logging_active:
            return
        
        self.csv_logging_active = False
        if self.csv_filehandle:
            self.csv_filehandle.close()
            self.csv_filehandle = None
            self.csv_writer = None
        
        self.csv_button.setText("💾 Start CSV Log")
        self.csv_button.setStyleSheet("QPushButton { background-color: #ff8800; color: white; font-weight: bold; }")
        self.status_bar.showMessage(f"💾 CSV logging stopped: {self.sweep_count} sweeps saved")
        print(f"💾 CSV logging stopped: {self.sweep_count} sweeps saved to {self.csv_file}")
    
    def toggle_csv_logging(self):
        """Toggle CSV logging on/off"""
        if self.csv_logging_active:
            self.stop_csv_logging()
        else:
            self.start_csv_logging()
    
    def log_sweep_to_csv(self):
        """Log current sweep data to CSV file"""
        if not self.csv_logging_active or not self.csv_writer:
            return
        
        
        
        try:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            row = [self.sweep_count, timestamp, self.current_temp, self.current_divisor] + self.display_data.tolist()
            self.csv_writer.writerow(row)
            
            # Flush every 10 sweeps
            if self.sweep_count % 10 == 0:
                self.csv_filehandle.flush()
            
            self.sweep_count += 1
        except Exception as e:
            print(f"❌ CSV write error: {e}")
            self.stop_csv_logging()

    def _update_plot(self):
        data_changed = False
        loopback_changed = False
        points_processed = 0
        batch_count = 0
        max_batch = 200
        sweep_complete = False
        loopback_complete = False
        metadata_updated = False
        
        while not self.data_queue.empty() and batch_count < max_batch:
            try:
                packet = self.data_queue.get_nowait()
                batch_count += 1
                
                # Check if it's metadata, loopback, or amplitude data
                if isinstance(packet, tuple) and len(packet) == 3 and packet[0] == 'metadata':
                    _, temp, divisor = packet
                    self.current_temp = temp
                    self.current_divisor = divisor

                    self.temp_history.append(temp)
                    self.divisor_history.append(divisor)
                    self.temp_timestamps.append(time.time())
                    
                    # Keep only last 1000 values
                    if len(self.temp_history) > 1000:
                        self.temp_history.pop(0)
                        self.divisor_history.pop(0)
                        self.temp_timestamps.pop(0)
                    
                    metadata_updated = True
                
                elif isinstance(packet, tuple) and len(packet) == 4 and packet[0] == 'loopback':
                    _, idx, amplitude, total_len = packet
                    self.loopback_packets_received += 1
                    self.loopback_last_idx = idx
                    if 0 <= idx < LOOPBACK_NUM_POINTS:
                        self.loopback_data[idx] = float(amplitude)
                        loopback_changed = True
                        
                        # Check if we got the last point (loopback sweep complete)
                        if idx == total_len - 1:
                            loopback_complete = True
                            self.loopback_sweep_count += 1
                    
                elif isinstance(packet, tuple) and len(packet) == 2:
                    idx, amplitude = packet
                    if 0 <= idx < NUM_POINTS:
                        self.display_data[idx] = float(amplitude)
                        points_processed += 1
                        data_changed = True
                        self.points_received += 1
                        
                        # Check if we got the last point (sweep complete)
                        if idx == NUM_POINTS - 1:
                            sweep_complete = True
            except:
                break
        
        # Update temp/divisor display
        if metadata_updated:
            self.plot_widget.setLabel('bottom', f'Range (m)     T: {self.current_temp}   D: {self.current_divisor}')
        
        # Update loopback plot
        if loopback_changed:
            self.loopback_curve.setData(self.loopback_range_values, self.loopback_data)
            if loopback_complete:
                self.loopback_text.setText(f"Loopback #{self.loopback_sweep_count} | Max: {np.max(self.loopback_data):.0f}")

        if data_changed:
            self.curve.setData(self.range_values, self.display_data)

            # --- Threshold crossing detection ---
            if self.threshold_lut_arr is not None:
                lut = self.threshold_lut_arr
                n   = min(len(self.display_data), len(lut))
                # Skip near-field clutter — only search from MIN_DETECTION_RANGE_MM onward
                min_idx = int(np.searchsorted(self.range_values[:n] * 1000.0,
                                              MIN_DETECTION_RANGE_MM))
                # Find first index where amplitude exceeds the LUT threshold
                crossings = np.where(self.display_data[min_idx:n] > lut[min_idx:n])[0]
                if crossings.size > 0:
                    cross_idx = min_idx + crossings[0]
                    cross_x   = float(self.range_values[cross_idx])
                    self.crossing_line.setValue(cross_x)
                    self.crossing_line.setVisible(True)
                    self.crossing_label.setText(f"  {cross_x*1000:.1f} mm")
                    # Pin label to top of Y range so it doesn't drift with data
                    y_top = Y_LIMITS[1] if FIXED_GRAPH_Y else float(np.max(self.display_data)) * 1.05
                    self.crossing_label.setPos(cross_x, y_top)
                else:
                    self.crossing_line.setVisible(False)
                    self.crossing_label.setText("")
            self.update_counter += 1
            self.fps_counter += 1
            
            # Log to CSV if a complete sweep was received
            if sweep_complete and self.csv_logging_active:
                self.log_sweep_to_csv()
            
            current_time = time.time()
            if current_time - self.last_status_update >= 0.2:
                fps = self.fps_counter / (current_time - self.last_fps_time)
                rate = self.points_received / (current_time - self.last_status_update)
                non_zero_count = np.count_nonzero(self.display_data)
                data_max = np.max(self.display_data)
                csv_status = f" | CSV:{self.sweep_count}" if self.csv_logging_active else ""
                temp_status = f" | T:{self.current_temp}" if self.current_temp > 0 else ""
                div_status = f" | D:{self.current_divisor}" if self.current_divisor > 0 else ""
                self.status_bar.showMessage(
                    f"🚀 : Max:{data_max:.0f} | Active:{non_zero_count} | "
                    f"Rate:{rate:.0f}pts/s | FPS:{fps:.0f} | Updates:{self.update_counter}{temp_status}{div_status} | LB_pkts:{self.loopback_packets_received} LB_idx:{self.loopback_last_idx}{csv_status}"
                )
                self.last_status_update = current_time
                self.points_received = 0
                self.fps_counter = 0
                self.last_fps_time = current_time

    def closeEvent(self, event):
        if self.csv_logging_active:
            self.stop_csv_logging()
        self.serial_reader.stop()
        event.accept()


# ---  Application Entry Point ---
def _plot_loop(port="COM7", baud=2000000, debug=False, csv_mode=False, csv_file=None):
    try:
        import psutil
        import os
        process = psutil.Process(os.getpid())
        if hasattr(psutil, 'HIGH_PRIORITY_CLASS'):
            process.nice(psutil.HIGH_PRIORITY_CLASS)
        else:
            process.nice(-10)
        print("🚀 Process priority boosted to MAXIMUM")
    except ImportError:
        print("⚠️ Install psutil for maximum performance: pip install psutil")
    except Exception as e:
        print(f"⚠️ Could not boost priority: {e}")
    
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle('Fusion')
    app.setStyleSheet("""
        QMainWindow { background-color: #2b2b2b; color: #ffffff; }
        QStatusBar { background-color: #3c3c3c; color: #00ff00; font-weight: bold; }
        QPushButton { padding: 8px; border-radius: 4px; font-size: 12px; }
    """)

    viewer = LiveDataViewer(port=port, baud=baud, debug=debug, csv_mode=csv_mode, csv_file=csv_file)
    
    # --- Load threshold from LUT files ---
    # lut[i] maps directly to data[i] by index (per header comment), so use
    # range_values as the x-axis — this guarantees pixel-perfect alignment.
    threshold_lut_arr = load_threshold_lut()
    if threshold_lut_arr is not None:
        n = min(len(viewer.range_values), len(threshold_lut_arr))
        viewer.set_threshold_line(viewer.range_values[:n], threshold_lut_arr[:n])
        viewer.threshold_lut_arr = threshold_lut_arr

    viewer.show()

    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        print("🛑 Application interrupted by user")
        viewer.serial_reader.stop()
        app.quit()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='Live Data Viewer')
    parser.add_argument('--port', default='COM5', help='Serial port (default: COM5)')
    parser.add_argument('--baud', type=int, default=2000000, help='Baud rate (default: 2000000)')
    parser.add_argument('--debug', action='store_true', help='Enable debug output')
    parser.add_argument('--csv', action='store_true', help='Start CSV logging on launch')
    parser.add_argument('--csv-file', type=str, help='Custom CSV filename (default: auto-generated with timestamp)')
    args = parser.parse_args()
    _plot_loop(port=args.port, baud=args.baud, debug=args.debug, csv_mode=args.csv, csv_file=args.csv_file)
