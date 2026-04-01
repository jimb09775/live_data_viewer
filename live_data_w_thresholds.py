"""
live_data_w_thresholds.py
--------------------------
Live radar data viewer with integrated threshold LUT editor.

Workflow:
  1. Edit threshold_breakpoints.py (plain Python config file) OR use the
     in-app "Edit Breakpoints" table — either way the threshold line updates
     instantly without restarting.
  2. Watch the live radar plot with the threshold overlaid.
  3. Close the window → dialog asks: Write LUT files / Edit More / Exit.

threshold_breakpoints.py is auto-created with defaults on first run.
"""

import numpy as np
import serial
import threading
from queue import Queue, Empty
import time
import pyqtgraph as pg
from PyQt5 import QtWidgets, QtCore
import sys
import csv
import os
import re
import importlib.util
from datetime import datetime


# ─────────────────────────────────────────────────────────────────────────────
#  Constants
# ─────────────────────────────────────────────────────────────────────────────
NUM_POINTS   = 400
POINT_SIZE   = 6
HEADER_BYTE  = 0xAA
TRAILER_BYTE = 0xA0
LOOPBACK_HEADER = 0xCC

RF_FACTOR = 1.48
START     = (0 * 0.0075) / RF_FACTOR
STEP      = 0.0075 / RF_FACTOR

# Threshold LUT generation params (must match firmware config)
LUT_CONFIG_STEP        = 3       # CONFIG_STEP
LUT_CONFIG_START_POINT = 10      # CONFIG_START_POINT
LUT_RF_FACTOR          = 1.47    # sqrt(epsilon) for hydraulic oil at 66 GHz
LUT_BASE_STEP_M        = 0.0025  # A121 base step in metres

# Output file names
OUTPUT_H             = "threshold_lut.h"
OUTPUT_C             = "threshold_lut.c"
BREAKPOINTS_FILE     = "threshold_breakpoints.py"

FIXED_GRAPH_Y        = False
Y_LIMITS             = (0, 2000000)
MIN_DETECTION_RANGE_MM = 0.0

LOOPBACK_START_POINT = -15
LOOPBACK_NUM_POINTS  = 6
LOOPBACK_STEP        = 1
LOOPBACK_Y_LIMITS    = (0, 300)

# ─────────────────────────────────────────────────────────────────────────────
#  Default breakpoints — written to file on first run
# ─────────────────────────────────────────────────────────────────────────────
DEFAULT_BREAKPOINTS = [
    (17.01,  3000),
    (50.0,   3000),
    (100.0,  5000),
    (200.0,  8000),
    (400.0, 10000),
    (600.0, 12000),
    (800.0, 15000),
    (999.0, 20000),
]

# ─────────────────────────────────────────────────────────────────────────────
#  Breakpoints file I/O
# ─────────────────────────────────────────────────────────────────────────────
def _ensure_breakpoints_file():
    """Create threshold_breakpoints.py with defaults if it doesn't exist."""
    if not os.path.exists(BREAKPOINTS_FILE):
        _write_breakpoints_file(DEFAULT_BREAKPOINTS)
        print(f"✅ Created default breakpoints file: {BREAKPOINTS_FILE}")


def _write_breakpoints_file(breakpoints):
    """Write breakpoints list to threshold_breakpoints.py."""
    lines = [
        "# threshold_breakpoints.py",
        "# Edit this file to adjust threshold breakpoints.",
        "# Format: (distance_mm, amplitude)",
        "# The live viewer hot-reloads this file when you click 'Reload'.",
        "",
        "BREAKPOINTS = [",
    ]
    for mm, amp in breakpoints:
        lines.append(f"    ({mm:.2f}, {amp:.0f}),")
    lines.append("]")
    lines.append("")
    with open(BREAKPOINTS_FILE, "w") as f:
        f.write("\n".join(lines))


def _load_breakpoints_file():
    """Load breakpoints from threshold_breakpoints.py. Returns sorted list."""
    try:
        spec = importlib.util.spec_from_file_location("threshold_breakpoints", BREAKPOINTS_FILE)
        mod  = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        bps = sorted(mod.BREAKPOINTS, key=lambda x: x[0])
        print(f"✅ Loaded {len(bps)} breakpoints from {BREAKPOINTS_FILE}")
        return bps
    except Exception as e:
        print(f"⚠️  Could not load {BREAKPOINTS_FILE}: {e} — using defaults")
        return sorted(DEFAULT_BREAKPOINTS, key=lambda x: x[0])


# ─────────────────────────────────────────────────────────────────────────────
#  LUT computation
# ─────────────────────────────────────────────────────────────────────────────
def _compute_lut_distances():
    """Return the 400-point distance array used for LUT generation (mm).
    Index 0 = 0mm (our system zero = A121 start point)."""
    step   = LUT_CONFIG_STEP
    rf     = LUT_RF_FACTOR
    base   = LUT_BASE_STEP_M
    return np.array(
        [i * step * base / rf * 1000.0 for i in range(NUM_POINTS)],
        dtype=float
    )


def _interpolate_lut(breakpoints, distances_mm):
    """Linear interpolation between breakpoints, with linear extrapolation
    beyond the edges so breakpoints outside the LUT range still influence
    the slope (e.g. a 0mm breakpoint affects the value at the 51mm LUT start)."""
    xs  = np.array([bp[0] for bp in breakpoints], dtype=float)
    ys  = np.array([bp[1] for bp in breakpoints], dtype=float)
    # np.interp clamps at edges — use manual linear extrap beyond range
    lut_f = np.interp(distances_mm, xs, ys)
    # Left extrapolation: extend slope of first two breakpoints
    if distances_mm[0] < xs[0] and len(xs) >= 2:
        slope = (ys[1] - ys[0]) / (xs[1] - xs[0]) if xs[1] != xs[0] else 0
        mask  = distances_mm < xs[0]
        lut_f[mask] = ys[0] + slope * (distances_mm[mask] - xs[0])
    # Right extrapolation: extend slope of last two breakpoints
    if distances_mm[-1] > xs[-1] and len(xs) >= 2:
        slope = (ys[-1] - ys[-2]) / (xs[-1] - xs[-2]) if xs[-1] != xs[-2] else 0
        mask  = distances_mm > xs[-1]
        lut_f[mask] = ys[-1] + slope * (distances_mm[mask] - xs[-1])
    lut = np.round(np.clip(lut_f, 0, None)).astype(np.uint32)
    return lut


def _build_threshold_from_breakpoints(breakpoints):
    """Return (lut_arr_float32, distances_mm) for the current breakpoints."""
    distances_mm = _compute_lut_distances()
    lut          = _interpolate_lut(breakpoints, distances_mm)
    return lut.astype(np.float32), distances_mm


# ─────────────────────────────────────────────────────────────────────────────
#  Write LUT C/H files
# ─────────────────────────────────────────────────────────────────────────────
def write_lut_files(breakpoints):
    distances_mm = _compute_lut_distances()
    lut          = _interpolate_lut(breakpoints, distances_mm)

    step     = LUT_CONFIG_STEP
    rf       = LUT_RF_FACTOR
    step_mm  = step * LUT_BASE_STEP_M * 1000.0
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    bp_comment_lines = "\n".join(
        f" *    {bp[0]:8.2f} mm  ->  {bp[1]:.0f}" for bp in breakpoints
    )

    header = f"""\
/**
 * @file    threshold_lut.h
 * @brief   Amplitude threshold LUT — one entry per A121 radar point.
 *
 * Auto-generated by live_data_w_thresholds.py on {timestamp}
 * DO NOT EDIT MANUALLY — regenerate using the Python tool.
 *
 * Sensor config:
 *   CONFIG_STEP       = {step}
 *   Distance zero     = A121 start point (system zero)
 *   rf_factor         = {rf}
 *   Step size         = {step_mm:.4f} mm/point
 *   Distance range    = {distances_mm[0]:.2f} .. {distances_mm[-1]:.2f} mm
 *   Entries           = {NUM_POINTS}  ({NUM_POINTS*4} bytes in flash)
 *
 * Firmware indexing:
 *   lut[i] corresponds directly to data[i] from acc_algorithm.
 *
 * Breakpoints used:
{bp_comment_lines}
 */

#ifndef THRESHOLD_LUT_H
#define THRESHOLD_LUT_H

#include <stdint.h>

#define THRESHOLD_LUT_LEN        {NUM_POINTS}U
#define THRESHOLD_LUT_STEP_MM    {step_mm:.4f}f
#define THRESHOLD_LUT_START_MM   {distances_mm[0]:.4f}f
#define THRESHOLD_LUT_END_MM     {distances_mm[-1]:.4f}f

extern const uint32_t g_threshold_lut[THRESHOLD_LUT_LEN];

#endif /* THRESHOLD_LUT_H */
"""

    rows = []
    for i in range(0, NUM_POINTS, 10):
        chunk = lut[i:i + 10]
        dist  = distances_mm[i]
        values = ", ".join(f"{v:7d}U" for v in chunk)
        rows.append(f"    /* [{i:3d}] {dist:7.2f} mm */ {values},")
    array_body = "\n".join(rows).rstrip(",")

    source = f"""\
/**
 * @file    threshold_lut.c
 * @brief   Amplitude threshold LUT — stored once in flash.
 *
 * Auto-generated by live_data_w_thresholds.py on {timestamp}
 * DO NOT EDIT MANUALLY — regenerate using the Python tool.
 *
 * {NUM_POINTS} entries  |  CONFIG_STEP={step}, rf_factor={rf}
 * Step: {step_mm:.4f} mm/point  |  Range: {distances_mm[0]:.2f} .. {distances_mm[-1]:.2f} mm
 */

#include "threshold_lut.h"

const uint32_t g_threshold_lut[THRESHOLD_LUT_LEN] = {{
{array_body}
}};
"""

    with open(OUTPUT_H, "w") as f:
        f.write(header)
    with open(OUTPUT_C, "w") as f:
        f.write(source)

    print(f"✅ Written: {os.path.abspath(OUTPUT_H)}")
    print(f"✅ Written: {os.path.abspath(OUTPUT_C)}")
    print(f"   Entries : {NUM_POINTS}  ({NUM_POINTS*4} bytes)")
    print(f"   Range   : {distances_mm[0]:.2f} .. {distances_mm[-1]:.2f} mm")
    print(f"   Min/Max : {lut.min()} / {lut.max()}")
    return True


# ─────────────────────────────────────────────────────────────────────────────
#  Breakpoint Editor Dialog
# ─────────────────────────────────────────────────────────────────────────────
class BreakpointEditorDialog(QtWidgets.QDialog):
    """
    Inline table editor for threshold breakpoints.
    Changes are applied immediately to the live plot and saved to file.
    """
    def __init__(self, breakpoints, distances_range, parent=None):
        super().__init__(parent)
        self.setWindowTitle("✏️ Threshold Breakpoint Editor")
        self.setMinimumWidth(420)
        self.setMinimumHeight(500)
        self._breakpoints = [list(bp) for bp in breakpoints]  # mutable copy
        self._dist_min    = distances_range[0]
        self._dist_max    = distances_range[-1]

        layout = QtWidgets.QVBoxLayout(self)

        info = QtWidgets.QLabel(
            f"Distance range: 0.0 – {self._dist_max:.1f} mm\n"
            "Changes apply immediately to the live plot when you click Apply.\n"
            f"File: {os.path.abspath(BREAKPOINTS_FILE)}"
        )
        info.setStyleSheet("color: #555; font-size: 11px;")
        layout.addWidget(info)

        # Table
        self.table = QtWidgets.QTableWidget(len(self._breakpoints), 2)
        self.table.setHorizontalHeaderLabels(["Distance (mm)", "Amplitude"])
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.verticalHeader().setDefaultSectionSize(26)
        self.table.setAlternatingRowColors(True)
        self._populate_table()
        layout.addWidget(self.table)

        # Row buttons
        row_btn_layout = QtWidgets.QHBoxLayout()
        add_btn = QtWidgets.QPushButton("➕ Add Row")
        add_btn.clicked.connect(self._add_row)
        del_btn = QtWidgets.QPushButton("🗑 Delete Selected")
        del_btn.clicked.connect(self._delete_row)
        row_btn_layout.addWidget(add_btn)
        row_btn_layout.addWidget(del_btn)
        row_btn_layout.addStretch()
        layout.addLayout(row_btn_layout)

        # Action buttons
        btn_layout = QtWidgets.QHBoxLayout()
        apply_btn = QtWidgets.QPushButton("✅ Apply & Save")
        apply_btn.setStyleSheet("background-color: #2a7a2a; color: white; font-weight: bold; padding: 6px;")
        apply_btn.clicked.connect(self._apply)
        close_btn = QtWidgets.QPushButton("Close")
        close_btn.clicked.connect(self.reject)
        btn_layout.addWidget(apply_btn)
        btn_layout.addWidget(close_btn)
        layout.addLayout(btn_layout)

        self._error_label = QtWidgets.QLabel("")
        self._error_label.setStyleSheet("color: red;")
        layout.addWidget(self._error_label)

    def _populate_table(self):
        self.table.setRowCount(len(self._breakpoints))
        for row, (mm, amp) in enumerate(self._breakpoints):
            self.table.setItem(row, 0, QtWidgets.QTableWidgetItem(f"{mm:.2f}"))
            self.table.setItem(row, 1, QtWidgets.QTableWidgetItem(f"{amp:.0f}"))

    def _add_row(self):
        row = self.table.rowCount()
        self.table.insertRow(row)
        self.table.setItem(row, 0, QtWidgets.QTableWidgetItem("0.00"))
        self.table.setItem(row, 1, QtWidgets.QTableWidgetItem("5000"))

    def _delete_row(self):
        rows = sorted({idx.row() for idx in self.table.selectedIndexes()}, reverse=True)
        for r in rows:
            self.table.removeRow(r)

    def _read_table(self):
        """Parse table into sorted breakpoints list. Returns list or None on error."""
        bps = []
        for row in range(self.table.rowCount()):
            try:
                mm  = float(self.table.item(row, 0).text())
                amp = float(self.table.item(row, 1).text())
            except (ValueError, AttributeError):
                self._error_label.setText(f"❌ Row {row+1}: invalid number")
                return None
            if mm < 0 or mm > self._dist_max:
                self._error_label.setText(
                    f"❌ Row {row+1}: distance {mm:.1f} out of range "
                    f"(0.0–{self._dist_max:.1f} mm)"
                )
                return None
            if amp < 0:
                self._error_label.setText(f"❌ Row {row+1}: amplitude must be >= 0")
                return None
            bps.append((mm, amp))
        if len(bps) < 2:
            self._error_label.setText("❌ Need at least 2 breakpoints")
            return None
        bps.sort(key=lambda x: x[0])
        self._error_label.setText("")
        return bps

    def _apply(self):
        bps = self._read_table()
        if bps is None:
            return
        self._breakpoints = [list(bp) for bp in bps]
        _write_breakpoints_file(bps)
        # Signal parent to reload
        if self.parent():
            self.parent().reload_threshold(bps)
        self._error_label.setText(f"✅ Saved {len(bps)} breakpoints — threshold updated")

    def closeEvent(self, event):
        event.accept()  # Prevent propagating to parent main window

    def get_breakpoints(self):
        return [tuple(bp) for bp in self._breakpoints]


# ─────────────────────────────────────────────────────────────────────────────
#  On-close dialog
# ─────────────────────────────────────────────────────────────────────────────
class CloseDialog(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Exit Options")
        self.setModal(True)
        self.choice = None

        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(QtWidgets.QLabel("What would you like to do?"))

        write_btn = QtWidgets.QPushButton("💾  Write threshold_lut.h + threshold_lut.c")
        write_btn.setStyleSheet("padding: 10px; font-size: 13px; background-color: #2a7a2a; color: white; font-weight: bold;")
        write_btn.clicked.connect(lambda: self._pick("write"))
        layout.addWidget(write_btn)

        edit_btn = QtWidgets.QPushButton("✏️  Edit breakpoints more (go back)")
        edit_btn.setStyleSheet("padding: 10px; font-size: 13px;")
        edit_btn.clicked.connect(lambda: self._pick("edit"))
        layout.addWidget(edit_btn)

        exit_btn = QtWidgets.QPushButton("🚪  Exit without writing LUT files")
        exit_btn.setStyleSheet("padding: 10px; font-size: 13px; color: #aa0000;")
        exit_btn.clicked.connect(lambda: self._pick("exit"))
        layout.addWidget(exit_btn)

    def _pick(self, choice):
        self.choice = choice
        self.accept()


# ─────────────────────────────────────────────────────────────────────────────
#  Serial Worker (unchanged from original)
# ─────────────────────────────────────────────────────────────────────────────
class SerialReader:
    def __init__(self, port, baudrate, data_queue):
        self.port      = port
        self.baudrate  = baudrate
        self.data_queue = data_queue
        self.raw_queue = Queue(maxsize=5000)
        self.running   = True
        self.bytes_read = 0
        self.packets_processed = 0
        self.errors    = 0

    def start(self):
        self.reader_thread    = threading.Thread(target=self.__serial_reader, daemon=True)
        self.processor_thread = threading.Thread(target=self.__packet_processor, daemon=True)
        self.reader_thread.start()
        self.processor_thread.start()
        print(f"🚀 Serial Reader started: {self.port} @ {self.baudrate}")
        return self.reader_thread, self.processor_thread

    def __serial_reader(self):
        try:
            ser = serial.Serial(
                self.port, self.baudrate, timeout=0.002,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, xonxoff=False, rtscts=False, dsrdtr=False
            )
            try:
                ser.set_buffer_size(rx_size=2147483647, tx_size=1024)
            except AttributeError:
                pass  # not available on Mac
            ser.reset_input_buffer()
            print("⚡ Serial connected")
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
        HEADER   = HEADER_BYTE
        TRAILER  = TRAILER_BYTE
        MAX_PTS  = NUM_POINTS
        META_HDR = 0xBB
        LB_HDR   = LOOPBACK_HEADER

        while self.running:
            try:
                raw_data = self.raw_queue.get(timeout=0.005)
                buffer.extend(raw_data)

                while len(buffer) >= 7:
                    header_pos  = -1
                    header_type = None
                    for i in range(len(buffer)):
                        if buffer[i] == HEADER:
                            header_pos  = i; header_type = 'amplitude'; break
                        elif buffer[i] == META_HDR:
                            header_pos  = i; header_type = 'metadata';  break
                        elif buffer[i] == LB_HDR:
                            header_pos  = i; header_type = 'loopback';  break

                    if header_pos == -1:
                        buffer.clear(); break
                    if header_pos > 0:
                        buffer = buffer[header_pos:]

                    if header_type == 'metadata':
                        if len(buffer) < 7: break
                        if buffer[6] == TRAILER:
                            chk      = buffer[5]
                            computed = buffer[1] ^ buffer[2] ^ buffer[3] ^ buffer[4]
                            if chk == computed:
                                temp    = (buffer[1] << 8) | buffer[2]
                                if int(temp) > 150: temp -= 65536
                                divisor = (buffer[3] << 8) | buffer[4]
                                try:
                                    self.data_queue.put_nowait(('metadata', temp, divisor))
                                    self.packets_processed += 1
                                except: pass
                                buffer = buffer[7:]
                            else:
                                self.errors += 1; buffer = buffer[1:]
                        else:
                            buffer = buffer[1:]

                    elif header_type == 'loopback':
                        if len(buffer) < 10: break
                        if buffer[9] == TRAILER:
                            idx = buffer[1] | (buffer[2] << 8)
                            amp = (buffer[3] | (buffer[4] << 8) |
                                   (buffer[5] << 16) | (buffer[6] << 24))
                            if amp >= 0x80000000: amp -= 0x100000000
                            total_len = buffer[7] | (buffer[8] << 8)
                            if 0 <= idx < LOOPBACK_NUM_POINTS:
                                try:
                                    self.data_queue.put_nowait(('loopback', idx, amp, total_len))
                                    self.packets_processed += 1
                                except: pass
                            buffer = buffer[10:]
                        else:
                            buffer = buffer[1:]

                    elif header_type == 'amplitude':
                        if len(buffer) < 8: break
                        if buffer[7] == TRAILER:
                            idx = buffer[1] | (buffer[2] << 8)
                            amp = (buffer[3] | (buffer[4] << 8) |
                                   (buffer[5] << 16) | (buffer[6] << 24))
                            if amp >= 0x80000000: amp -= 0x100000000
                            if 0 <= idx < MAX_PTS:
                                try:
                                    self.data_queue.put_nowait((idx, amp))
                                    self.packets_processed += 1
                                except: pass
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
            'bytes_read':       self.bytes_read,
            'packets_processed': self.packets_processed,
            'errors':           self.errors,
            'raw_queue_size':   self.raw_queue.qsize(),
            'data_queue_size':  self.data_queue.qsize(),
        }


# ─────────────────────────────────────────────────────────────────────────────
#  Main GUI
# ─────────────────────────────────────────────────────────────────────────────
class LiveDataViewer(QtWidgets.QMainWindow):
    def __init__(self, port="COM5", baud=2000000, debug=False,
                 csv_mode=False, csv_file=None):
        super().__init__()
        self.port              = port
        self.baud              = baud
        self.debug             = debug
        self.csv_mode          = csv_mode
        self.csv_file          = csv_file
        self.csv_writer        = None
        self.csv_filehandle    = None
        self.sweep_count       = 0
        self.csv_logging_active = False

        self.data_queue   = Queue(maxsize=10000)
        self.display_data = np.zeros(NUM_POINTS, dtype=np.float32)
        self.current_temp     = 0
        self.current_divisor  = 0
        self.temp_history     = []
        self.divisor_history  = []
        self.temp_timestamps  = []

        self.loopback_data = np.zeros(LOOPBACK_NUM_POINTS, dtype=np.float32)
        self.loopback_range_values = (
            np.arange(LOOPBACK_NUM_POINTS, dtype=np.float32) *
            (LOOPBACK_STEP * 0.0025) +
            (LOOPBACK_START_POINT * 0.0025)
        )
        self.loopback_sweep_count     = 0
        self.loopback_packets_received = 0
        self.loopback_last_idx        = -1

        self.range_values = np.arange(NUM_POINTS, dtype=np.float32) * STEP + START

        # Load breakpoints
        _ensure_breakpoints_file()
        self.breakpoints = _load_breakpoints_file()
        self.lut_distances_mm = _compute_lut_distances()

        self._init_ui()

        if self.csv_mode:
            self.start_csv_logging()

        self.serial_reader  = SerialReader(port, baud, self.data_queue)
        self.reader_threads = self.serial_reader.start()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._update_plot)
        self.timer.start(3)

        self.update_counter      = 0
        self.points_received     = 0
        self.last_status_update  = time.time()
        self.fps_counter         = 0
        self.last_fps_time       = time.time()

        # Load initial threshold
        self.reload_threshold(self.breakpoints)

    # ── UI construction ───────────────────────────────────────────────────────
    def _init_ui(self):
        self.setWindowTitle(f"🚀 Live Data Viewer w/ Thresholds — {self.port}")
        self.setGeometry(100, 100, 1200, 800)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout  = QtWidgets.QVBoxLayout(central)

        # Main plot
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
            self.range_values, self.display_data,
            pen=pg.mkPen(color='black', width=1.5),
            antialias=False, skipFiniteCheck=True, autoDownsample=True
        )

        # Threshold curve
        self.threshold_x   = []
        self.threshold_y   = []
        self.threshold_lut_arr = None
        self.threshold_curve = self.plot_widget.plot(
            self.threshold_x, self.threshold_y,
            pen=pg.mkPen(color='red', width=2, style=QtCore.Qt.DashLine),
            antialias=False
        )
        self.threshold_visible = True

        # Breakpoint scatter markers on threshold
        self.bp_scatter = pg.ScatterPlotItem(
            symbol='o', size=10,
            pen=pg.mkPen(color='#cc0000', width=1.5),
            brush=pg.mkBrush('#ff4444')
        )
        self.plot_widget.addItem(self.bp_scatter)

        # Crossing line
        self.crossing_line = pg.InfiniteLine(
            pos=0.0, angle=90,
            pen=pg.mkPen(color='#ff6600', width=2, style=QtCore.Qt.SolidLine),
            movable=False
        )
        self.crossing_line.setVisible(False)
        self.plot_widget.addItem(self.crossing_line)

        self.crossing_label = pg.TextItem(text="", color='#ff6600', anchor=(0, 1))
        self.plot_widget.addItem(self.crossing_label)

        layout.addWidget(self.plot_widget)

        # Loopback plot
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
            self.loopback_range_values, self.loopback_data,
            pen=pg.mkPen(color='blue', width=1.5),
            antialias=False, skipFiniteCheck=True
        )
        self.loopback_text = pg.TextItem(text="Loopback: --", color=(0, 0, 200), anchor=(0, 1))
        self.loopback_text.setPos(self.loopback_range_values[0], LOOPBACK_Y_LIMITS[1] * 0.9)
        self.loopback_plot_widget.addItem(self.loopback_text)

        layout.addWidget(self.loopback_plot_widget)
        self._init_controls(layout)
        self.status_bar = self.statusBar()
        self.status_bar.showMessage(f"🚀 {self.port} @ {self.baud} baud")

    def _init_controls(self, layout):
        ctrl = QtWidgets.QHBoxLayout()

        self.reset_button = QtWidgets.QPushButton("🔄 Reset Display")
        self.reset_button.clicked.connect(self.reset_display_data)
        self.reset_button.setStyleSheet("background-color:#ff4444;color:white;font-weight:bold;padding:6px;")
        ctrl.addWidget(self.reset_button)

        self.stats_button = QtWidgets.QPushButton("📊 Stats")
        self.stats_button.clicked.connect(self.show_performance_stats)
        self.stats_button.setStyleSheet("background-color:#4444ff;color:white;font-weight:bold;padding:6px;")
        ctrl.addWidget(self.stats_button)

        self.toggle_thresh_button = QtWidgets.QPushButton("⚙️ Toggle Threshold")
        self.toggle_thresh_button.clicked.connect(self.toggle_threshold)
        self.toggle_thresh_button.setStyleSheet("background-color:#00aa00;color:white;font-weight:bold;padding:6px;")
        ctrl.addWidget(self.toggle_thresh_button)

        self.edit_bp_button = QtWidgets.QPushButton("✏️ Edit Breakpoints")
        self.edit_bp_button.clicked.connect(self.open_breakpoint_editor)
        self.edit_bp_button.setStyleSheet("background-color:#7a3fa0;color:white;font-weight:bold;padding:6px;")
        ctrl.addWidget(self.edit_bp_button)

        self.reload_bp_button = QtWidgets.QPushButton("🔃 Reload from File")
        self.reload_bp_button.clicked.connect(self.reload_from_file)
        self.reload_bp_button.setStyleSheet("background-color:#3a6080;color:white;font-weight:bold;padding:6px;")
        ctrl.addWidget(self.reload_bp_button)

        self.csv_button = QtWidgets.QPushButton("💾 Start CSV Log")
        self.csv_button.clicked.connect(self.toggle_csv_logging)
        self.csv_button.setStyleSheet("background-color:#ff8800;color:white;font-weight:bold;padding:6px;")
        ctrl.addWidget(self.csv_button)

        ctrl.addStretch()
        layout.addLayout(ctrl)

        # Breakpoint info label
        self.bp_info_label = QtWidgets.QLabel("")
        self.bp_info_label.setStyleSheet("color:#555;font-size:10px;padding:2px 4px;")
        layout.addWidget(self.bp_info_label)

    # ── Threshold management ──────────────────────────────────────────────────
    def reload_threshold(self, breakpoints):
        """Recompute LUT from breakpoints and update the plot immediately."""
        self.breakpoints = breakpoints
        lut_arr, distances_mm = _build_threshold_from_breakpoints(breakpoints)
        self.threshold_lut_arr = lut_arr

        # x-axis for plot: interpolate breakpoints onto the full display
        # range_values grid so the threshold line starts from x=0
        xs_bp = np.array([bp[0] / 1000.0 for bp in breakpoints], dtype=float)
        ys_bp = np.array([bp[1]           for bp in breakpoints], dtype=float)
        thresh_y = np.interp(self.range_values, xs_bp, ys_bp)
        # Left extrapolation beyond first breakpoint
        if len(xs_bp) >= 2 and self.range_values[0] < xs_bp[0]:
            slope = (ys_bp[1] - ys_bp[0]) / (xs_bp[1] - xs_bp[0]) if xs_bp[1] != xs_bp[0] else 0
            mask  = self.range_values < xs_bp[0]
            thresh_y[mask] = ys_bp[0] + slope * (self.range_values[mask] - xs_bp[0])
        thresh_y = np.clip(thresh_y, 0, None).astype(np.float32)
        self.threshold_x = self.range_values.copy()
        self.threshold_y = thresh_y.astype(np.float32)
        self.threshold_curve.setData(self.threshold_x, self.threshold_y)
        self.threshold_curve.setVisible(self.threshold_visible)

        # Scatter markers at breakpoint positions
        bp_x = np.array([bp[0] / 1000.0 for bp in breakpoints], dtype=np.float32)
        bp_y = np.array([bp[1]           for bp in breakpoints], dtype=np.float32)
        self.bp_scatter.setData(x=bp_x, y=bp_y)
        self.bp_scatter.setVisible(self.threshold_visible)

        # Update info label
        self.bp_info_label.setText(
            f"Threshold: {len(breakpoints)} breakpoints  |  "
            f"LUT range {distances_mm[0]:.1f}–{distances_mm[-1]:.1f} mm  |  "
            f"amp {int(lut_arr.min())}–{int(lut_arr.max())}  |  "
            f"file: {BREAKPOINTS_FILE}"
        )
        print(f"🔄 Threshold reloaded: {len(breakpoints)} breakpoints")

    def reload_from_file(self):
        """Re-read threshold_breakpoints.py from disk and update plot."""
        bps = _load_breakpoints_file()
        self.reload_threshold(bps)
        self.status_bar.showMessage(f"🔃 Breakpoints reloaded from {BREAKPOINTS_FILE}")

    def open_breakpoint_editor(self):
        dlg = BreakpointEditorDialog(
            self.breakpoints, self.lut_distances_mm, parent=self
        )
        dlg.exec_()
        # Sync back whatever was last applied
        self.breakpoints = _load_breakpoints_file()

    def set_threshold_line(self, x_data, y_data):
        if len(x_data) == len(y_data):
            self.threshold_x = np.array(x_data, dtype=np.float32)
            self.threshold_y = np.array(y_data, dtype=np.float64)
            self.threshold_curve.setData(self.threshold_x, self.threshold_y)
        else:
            print("❌ Threshold arrays must be same length")

    def toggle_threshold(self):
        self.threshold_visible = not self.threshold_visible
        self.threshold_curve.setVisible(self.threshold_visible)
        self.bp_scatter.setVisible(self.threshold_visible)
        self.status_bar.showMessage(
            "👁 Threshold: " + ("Visible" if self.threshold_visible else "Hidden")
        )

    # ── Plot update loop ──────────────────────────────────────────────────────
    def reset_display_data(self):
        self.display_data.fill(0)
        self.curve.setData(self.range_values, self.display_data)
        self.status_bar.showMessage("🔄 Display data reset")

    def show_performance_stats(self):
        stats = self.serial_reader.get_performance_stats()
        msg   = (f"📊 {stats['packets_processed']} packets | "
                 f"{stats['bytes_read']} bytes | "
                 f"{stats['errors']} errors | "
                 f"Raw Q:{stats['raw_queue_size']} Data Q:{stats['data_queue_size']}")
        print(msg)
        self.status_bar.showMessage(msg)

    def _update_plot(self):
        data_changed     = False
        loopback_changed = False
        points_processed = 0
        batch_count      = 0
        sweep_complete   = False
        loopback_complete = False
        metadata_updated = False

        while not self.data_queue.empty() and batch_count < 200:
            try:
                packet = self.data_queue.get_nowait()
                batch_count += 1

                if isinstance(packet, tuple) and len(packet) == 3 and packet[0] == 'metadata':
                    _, temp, divisor = packet
                    self.current_temp    = temp
                    self.current_divisor = divisor
                    self.temp_history.append(temp)
                    self.divisor_history.append(divisor)
                    self.temp_timestamps.append(time.time())
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
                        if idx == total_len - 1:
                            loopback_complete = True
                            self.loopback_sweep_count += 1

                elif isinstance(packet, tuple) and len(packet) == 2:
                    idx, amplitude = packet
                    if 0 <= idx < NUM_POINTS:
                        self.display_data[idx] = float(amplitude)
                        points_processed += 1
                        data_changed  = True
                        self.points_received += 1
                        if idx == NUM_POINTS - 1:
                            sweep_complete = True
            except:
                break

        if metadata_updated:
            self.plot_widget.setLabel(
                'bottom',
                f'Range (m)     T: {self.current_temp}   D: {self.current_divisor}'
            )

        if loopback_changed:
            self.loopback_curve.setData(self.loopback_range_values, self.loopback_data)
            if loopback_complete:
                self.loopback_text.setText(
                    f"Loopback #{self.loopback_sweep_count} | Max: {np.max(self.loopback_data):.0f}"
                )

        if data_changed:
            self.curve.setData(self.range_values, self.display_data)

            if self.threshold_lut_arr is not None and len(self.threshold_y) == len(self.range_values):
                # Use threshold_y — aligned to range_values, covers full plot including 0mm
                lut = self.threshold_y
                n   = min(len(self.display_data), len(lut))
                # Skip negative range values and anything below MIN_DETECTION_RANGE_MM
                min_mm  = max(MIN_DETECTION_RANGE_MM, 0.0)
                min_idx = int(np.searchsorted(self.range_values[:n] * 1000.0, min_mm))
                crossings = np.where(self.display_data[min_idx:n] > lut[min_idx:n])[0]
                if crossings.size > 0:
                    cross_idx   = min_idx + crossings[0]
                    cross_x     = float(self.range_values[cross_idx])
                    self.crossing_line.setValue(cross_x)
                    self.crossing_line.setVisible(True)
                    self.crossing_label.setText(f"  {cross_x*1000:.1f} mm")
                    y_top = Y_LIMITS[1] if FIXED_GRAPH_Y else float(np.max(self.display_data)) * 1.05
                    self.crossing_label.setPos(cross_x, y_top)
                else:
                    self.crossing_line.setVisible(False)
                    self.crossing_label.setText("")

            self.update_counter += 1
            self.fps_counter    += 1

            if sweep_complete and self.csv_logging_active:
                self.log_sweep_to_csv()

            now = time.time()
            if now - self.last_status_update >= 0.2:
                fps      = self.fps_counter / (now - self.last_fps_time)
                rate     = self.points_received / (now - self.last_status_update)
                nz       = np.count_nonzero(self.display_data)
                data_max = np.max(self.display_data)
                csv_s    = f" | CSV:{self.sweep_count}" if self.csv_logging_active else ""
                temp_s   = f" | T:{self.current_temp}"    if self.current_temp > 0    else ""
                div_s    = f" | D:{self.current_divisor}" if self.current_divisor > 0 else ""
                self.status_bar.showMessage(
                    f"🚀 Max:{data_max:.0f} | Active:{nz} | "
                    f"Rate:{rate:.0f}pts/s | FPS:{fps:.0f} | "
                    f"Updates:{self.update_counter}{temp_s}{div_s} | "
                    f"LB:{self.loopback_packets_received} idx:{self.loopback_last_idx}{csv_s}"
                )
                self.last_status_update  = now
                self.points_received     = 0
                self.fps_counter         = 0
                self.last_fps_time       = now

    # ── Close handler ─────────────────────────────────────────────────────────
    def closeEvent(self, event):
        if self.csv_logging_active:
            self.stop_csv_logging()
        self.timer.stop()

        while True:
            dlg = CloseDialog(self)
            dlg.exec_()
            choice = dlg.choice

            if choice == "write":
                ok = write_lut_files(self.breakpoints)
                if ok:
                    QtWidgets.QMessageBox.information(
                        self, "LUT Written",
                        f"Files written:\n  {os.path.abspath(OUTPUT_H)}\n  {os.path.abspath(OUTPUT_C)}\n\n"
                        f"{len(self.breakpoints)} breakpoints  |  {NUM_POINTS} entries"
                    )
                self.serial_reader.stop()
                event.accept()
                break

            elif choice == "edit":
                # Reopen breakpoint editor and don't close yet
                self.timer.start(3)     # resume plot while editing
                self.open_breakpoint_editor()
                self.timer.stop()
                # Loop back to show close dialog again
                continue

            else:  # "exit" or dialog closed with X
                self.serial_reader.stop()
                event.accept()
                break

    # ── CSV logging ───────────────────────────────────────────────────────────
    def start_csv_logging(self):
        if self.csv_logging_active:
            return
        if not self.csv_file:
            self.csv_file = f"sweep_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        try:
            self.csv_filehandle = open(self.csv_file, 'w', newline='')
            self.csv_writer     = csv.writer(self.csv_filehandle)
            header = (['sweep_num', 'timestamp', 'temperature', 'divisor'] +
                      [f'range_{i}_{self.range_values[i]:.4f}m' for i in range(NUM_POINTS)])
            self.csv_writer.writerow(header)
            self.csv_filehandle.flush()
            self.csv_logging_active = True
            self.csv_button.setText("💾 Stop CSV Log")
            self.csv_button.setStyleSheet("background-color:#00aa00;color:white;font-weight:bold;padding:6px;")
            self.status_bar.showMessage(f"💾 CSV logging: {self.csv_file}")
            print(f"💾 CSV logging started: {self.csv_file}")
        except Exception as e:
            print(f"❌ CSV start failed: {e}")

    def stop_csv_logging(self):
        if not self.csv_logging_active:
            return
        self.csv_logging_active = False
        if self.csv_filehandle:
            self.csv_filehandle.close()
            self.csv_filehandle = None
            self.csv_writer     = None
        self.csv_button.setText("💾 Start CSV Log")
        self.csv_button.setStyleSheet("background-color:#ff8800;color:white;font-weight:bold;padding:6px;")
        print(f"💾 CSV stopped: {self.sweep_count} sweeps → {self.csv_file}")

    def toggle_csv_logging(self):
        if self.csv_logging_active:
            self.stop_csv_logging()
        else:
            self.start_csv_logging()

    def log_sweep_to_csv(self):
        if not self.csv_logging_active or not self.csv_writer:
            return
        try:
            ts  = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            row = ([self.sweep_count, ts, self.current_temp, self.current_divisor] +
                   self.display_data.tolist())
            self.csv_writer.writerow(row)
            if self.sweep_count % 10 == 0:
                self.csv_filehandle.flush()
            self.sweep_count += 1
        except Exception as e:
            print(f"❌ CSV write error: {e}")
            self.stop_csv_logging()


# ─────────────────────────────────────────────────────────────────────────────
#  Entry Point
# ─────────────────────────────────────────────────────────────────────────────
def _plot_loop(port="COM7", baud=2000000, debug=False, csv_mode=False, csv_file=None):
    try:
        import psutil, os as _os
        p = psutil.Process(_os.getpid())
        p.nice(psutil.HIGH_PRIORITY_CLASS if hasattr(psutil, 'HIGH_PRIORITY_CLASS') else -10)
        print("🚀 Process priority boosted")
    except ImportError:
        print("⚠️  Install psutil for maximum performance")
    except Exception as e:
        print(f"⚠️  Could not boost priority: {e}")

    app = QtWidgets.QApplication(sys.argv)
    app.setStyle('Fusion')
    app.setStyleSheet("""
        QMainWindow  { background-color: #2b2b2b; color: #ffffff; }
        QStatusBar   { background-color: #3c3c3c; color: #00ff00; font-weight: bold; }
        QPushButton  { padding: 8px; border-radius: 4px; font-size: 12px; }
        QDialog      { background-color: #f8f8f8; }
        QTableWidget { font-size: 12px; }
    """)

    viewer = LiveDataViewer(port=port, baud=baud, debug=debug,
                            csv_mode=csv_mode, csv_file=csv_file)
    viewer.show()

    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        print("🛑 Interrupted")
        viewer.serial_reader.stop()
        app.quit()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='Live Data Viewer w/ Threshold Editor')
    parser.add_argument('--port',     default='COM5',    help='Serial port (default: COM5)')
    parser.add_argument('--baud',     type=int, default=2000000, help='Baud rate')
    parser.add_argument('--debug',    action='store_true')
    parser.add_argument('--csv',      action='store_true', help='Start CSV logging on launch')
    parser.add_argument('--csv-file', type=str,           help='Custom CSV filename')
    args = parser.parse_args()
    _plot_loop(port=args.port, baud=args.baud, debug=args.debug,
               csv_mode=args.csv, csv_file=args.csv_file)
