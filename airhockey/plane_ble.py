"""
XIAO Ball Studio v2.1 — Desktop application for autonomous IMU ball
- USB Live tab (wired debug)
- Ball Control tab (BLE: scan, connect, status, record, download, sleep)
- Playback tab (load CSV, timeline, 3D airplane replay)
"""

import sys
import asyncio
import struct
import queue
import numpy as np
import time
import serial
import serial.tools.list_ports
from datetime import datetime
from collections import deque
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QWidget,
                             QLabel, QHBoxLayout, QPushButton, QTabWidget,
                             QComboBox, QProgressBar, QFrame, QSplitter,
                             QSizePolicy, QFileDialog, QSlider, QSpinBox)
from PyQt5.QtCore import QThread, pyqtSignal, Qt, QTimer
from PyQt5.QtGui import QFont, QColor

from PyQt5.QtOpenGL import QGLWidget
from OpenGL.GL import *
from OpenGL.GLU import *

# pyqtgraph needs QApplication to exist
_app = QApplication.instance()
if _app is None:
    _app = QApplication(sys.argv)
import pyqtgraph as pg

# ═══════════════════════════════════════════════════
#  CONSTANTS
# ═══════════════════════════════════════════════════

UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
UART_RX_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
UART_TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

ACCEL_SCALE = 0.000488  # ±16g (LSM6DS3 library default): 0.488 mg/LSB
GYRO_SCALE  = 0.070     # ±2000 dps (LSM6DS3 library default): 70 mdps/LSB


# ═══════════════════════════════════════════════════
#  BLE SCAN WORKER
# ═══════════════════════════════════════════════════

class BLEScanWorker(QThread):
    devices_found = pyqtSignal(list)
    scan_error = pyqtSignal(str)

    def __init__(self, timeout=5.0):
        super().__init__()
        self.timeout = timeout

    def run(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            result = loop.run_until_complete(self._scan())
            self.devices_found.emit(result)
        except Exception as e:
            self.scan_error.emit(str(e))
        finally:
            loop.close()

    async def _scan(self):
        from bleak import BleakScanner
        devices = await BleakScanner.discover(timeout=self.timeout, return_adv=True)
        balls = []
        for addr, (device, adv) in devices.items():
            name = adv.local_name or device.name or ""
            if not any(kw in name.upper() for kw in ["BALL", "XIAO"]):
                continue
            battery = -1
            if adv.manufacturer_data:
                for cid, val in adv.manufacturer_data.items():
                    if len(val) >= 1:
                        battery = val[0]
            balls.append({
                "name": name, "address": device.address,
                "rssi": adv.rssi or -999, "battery": battery,
            })
        balls.sort(key=lambda x: x["rssi"], reverse=True)
        return balls


# ═══════════════════════════════════════════════════
#  BLE CONTROL WORKER (persistent connection)
# ═══════════════════════════════════════════════════

class BLEControlWorker(QThread):
    connected_changed = pyqtSignal(bool)
    status_received   = pyqtSignal(dict)
    recording_status  = pyqtSignal(str, int, int)   # "START"/"DONE", value, sample_rate_hz
    download_progress = pyqtSignal(int, int)
    download_finished = pyqtSignal(list)
    message_received  = pyqtSignal(str)
    error_signal      = pyqtSignal(str)

    def __init__(self, address):
        super().__init__()
        self.address = address
        self.running = True
        self.cmd_queue = queue.Queue()

        # Download state
        self._dl_active = False
        self._dl_header_received = False
        self._dl_expected = 0
        self._dl_blocks = {}
        self._dl_binbuf = bytearray()
        self._dl_txtbuf = ""
        self._dl_end = False
        self._dl_need_rdy = False

        # Text accumulator
        self._text_accum = ""

    def send_command(self, cmd):
        self.cmd_queue.put(cmd)

    def stop(self):
        self.running = False

    def run(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(self._main_loop())
        except Exception as e:
            self.error_signal.emit(f"BLE: {str(e)[:80]}")
        finally:
            self.connected_changed.emit(False)
            loop.close()

    async def _main_loop(self):
        from bleak import BleakScanner, BleakClient

        self.message_received.emit("Scanning...")
        device = await BleakScanner.find_device_by_address(self.address, timeout=10.0)
        if not device:
            self.error_signal.emit("Device not found")
            return

        self.message_received.emit("Connecting...")
        # Request large MTU to prevent status message BLE splitting
        try:
            ble_client = BleakClient(device, timeout=20.0, mtu_size=247)
        except TypeError:
            ble_client = BleakClient(device, timeout=20.0)
        async with ble_client as client:
            if not client.is_connected:
                self.error_signal.emit("Connection failed")
                return

            self.connected_changed.emit(True)
            self.message_received.emit("Connected!")
            self._client = client
            await asyncio.sleep(0.5)
            await client.start_notify(UART_TX_CHAR_UUID, self._on_notify)

            # Main command loop
            while self.running and client.is_connected:
                try:
                    cmd = self.cmd_queue.get_nowait()
                    await self._execute(client, cmd)
                except queue.Empty:
                    pass
                await asyncio.sleep(0.05)

            try:
                await client.stop_notify(UART_TX_CHAR_UUID)
            except:
                pass

    # ── BLE write helper ──

    async def _ble_write(self, client, data: bytes):
        """Write to BLE UART RX with response=False (NUS standard)"""
        try:
            await client.write_gatt_char(UART_RX_CHAR_UUID, data, response=False)
            return True
        except Exception as e:
            # Retry with response=True in case device uses Write With Response
            try:
                await client.write_gatt_char(UART_RX_CHAR_UUID, data, response=True)
                return True
            except Exception as e2:
                self.error_signal.emit(f"BLE write failed: {e2}")
                return False

    # ── Notification handler ──

    def _on_notify(self, sender, data: bytearray):
        # Debug: log raw BLE notification
        if not self._dl_active:
            try:
                txt_preview = data.decode('utf-8', errors='replace')[:40]
            except:
                txt_preview = "?"
            print(f"[BLE RAW] {len(data)}b: {txt_preview!r}")

        if self._dl_active:
            self._handle_download(data)
            return

        # Text mode
        try:
            text = data.decode('utf-8', errors='ignore')
            self._text_accum += text
            self._process_text()
        except:
            pass

    def _process_text(self):
        while '\n' in self._text_accum:
            line, self._text_accum = self._text_accum.split('\n', 1)
            line = line.strip('\r').strip()
            if not line:
                continue

            print(f"[BLE RX] {line}")

            if line.startswith("ST:"):
                self._parse_status(line)
            elif line.startswith("REC:START"):
                parts = line.split(",")
                dur = int(parts[1]) if len(parts) > 1 else 0
                hz  = int(parts[2]) if len(parts) > 2 else 100
                self.recording_status.emit("START", dur, hz)
            elif line.startswith("REC:DONE"):
                parts = line.split(",")
                cnt = int(parts[1]) if len(parts) > 1 else 0
                self.recording_status.emit("DONE", cnt, 0)
            elif line.startswith("ERR:"):
                self.error_signal.emit(line)
            elif line == "SLEEPING":
                self.message_received.emit("Ball going to sleep")
            elif line == "EMPTY":
                self.message_received.emit("No data on device")
                self.download_finished.emit([])
            else:
                self.message_received.emit(line)

    def _parse_status(self, line):
        try:
            parts = line[3:].split(",")
            # Status must have exactly 5 fields: state, bat%, voltage, samples, maxDur
            # If fewer, it's a partial BLE split — ignore it
            if len(parts) < 5:
                print(f"[STATUS] Ignoring partial: {line} ({len(parts)} fields)")
                return
            st = {
                "state": parts[0],
                "battery_pct": int(parts[1]),
                "battery_v": float(parts[2]),
                "samples": int(parts[3]),
                "max_duration": int(parts[4]),
            }
            self.status_received.emit(st)
        except Exception as e:
            print(f"[STATUS] Parse error: {e} in '{line}'")

    # ── Download protocol (proven from v1) ──

    def _reset_download(self):
        self._dl_active = True
        self._dl_header_received = False
        self._dl_expected = 0
        self._dl_blocks = {}
        self._dl_binbuf = bytearray()
        self._dl_txtbuf = ""
        self._dl_end = False
        self._dl_need_rdy = False

    def _handle_download(self, data):
        # Phase 1: waiting for header (text)
        if not self._dl_header_received:
            try:
                text = data.decode('utf-8', errors='ignore')
                self._dl_txtbuf += text

                if "HDR:" in self._dl_txtbuf and "\n" in self._dl_txtbuf:
                    line = self._dl_txtbuf.split("\n")[0]
                    if line.startswith("HDR:"):
                        parts = line[4:].split(",")
                        if len(parts) >= 2:
                            self._dl_expected = int(parts[0])
                            self._dl_header_received = True
                            self._dl_need_rdy = True
                            self.message_received.emit(f"Receiving {self._dl_expected} pts...")
                            print(f"[DL] Header: {self._dl_expected} pts")
                            self._dl_txtbuf = ""

                if "EMPTY" in self._dl_txtbuf:
                    self._dl_active = False
                    self.message_received.emit("No data on device")
                    self.download_finished.emit([])
                if "TIMEOUT" in self._dl_txtbuf:
                    self._dl_active = False
                    self.message_received.emit("Device timeout")
                    self.download_finished.emit([])
            except:
                pass
            return

        # Phase 2: binary data
        self._dl_binbuf.extend(data)

        # Check for END marker
        if b"END" in self._dl_binbuf:
            end_pos = self._dl_binbuf.find(b"END")
            self._dl_binbuf = self._dl_binbuf[:end_pos]
            self._dl_end = True

        # Parse 14-byte blocks
        while len(self._dl_binbuf) >= 14:
            blk = self._dl_binbuf[:14]
            self._dl_binbuf = self._dl_binbuf[14:]

            bnum = blk[0] | (blk[1] << 8)
            ax = struct.unpack('<h', blk[2:4])[0]
            ay = struct.unpack('<h', blk[4:6])[0]
            az = struct.unpack('<h', blk[6:8])[0]
            gx = struct.unpack('<h', blk[8:10])[0]
            gy = struct.unpack('<h', blk[10:12])[0]
            gz = struct.unpack('<h', blk[12:14])[0]

            if bnum not in self._dl_blocks:
                self._dl_blocks[bnum] = [ax, ay, az, gx, gy, gz]
                cur = len(self._dl_blocks)
                exp = max(self._dl_expected, 1)
                self.download_progress.emit(min(cur, exp), exp)

        if self._dl_end:
            self._finalize_download()

    def _finalize_download(self):
        self._dl_active = False
        if not self._dl_blocks:
            self.download_finished.emit([])
            return
        result = []
        for i in range(max(self._dl_blocks.keys()) + 1):
            if i in self._dl_blocks:
                result.append(self._dl_blocks[i])
            if len(result) >= self._dl_expected:
                break
        n = len(result)
        lost = self._dl_expected - n
        msg = f"Download: {n}/{self._dl_expected} pts"
        if lost > 0:
            msg += f" ({lost} lost)"
        self.message_received.emit(msg)
        print(f"[DL] {msg}")
        self.download_finished.emit(result)

    # ── Command execution ──

    async def _execute(self, client, cmd):
        try:
            if cmd == "STATUS":
                ok = await self._ble_write(client, b'S')
                if not ok:
                    self.error_signal.emit("Failed to send Status")

            elif cmd.startswith("RECORD:"):
                dur = cmd.split(":")[1]
                ok = await self._ble_write(client, f'R:{dur}'.encode())
                if not ok:
                    self.error_signal.emit("Failed to send Record")

            elif cmd == "DOWNLOAD":
                self._reset_download()
                ok = await self._ble_write(client, b'D')
                if not ok:
                    self._dl_active = False
                    self.error_signal.emit("Failed to send Download")
                    self.download_finished.emit([])
                    return

                # Wait for header
                t = 0
                while not self._dl_header_received and not self._dl_end and t < 40:
                    await asyncio.sleep(0.5)
                    t += 1

                if not self._dl_header_received:
                    self._dl_active = False
                    self.error_signal.emit("No header from device")
                    self.download_finished.emit([])
                    return

                # Send RDY
                print("[DL] Sending RDY")
                await self._ble_write(client, b'RDY')

                # Wait for all data
                last_count = 0
                stale = 0
                while not self._dl_end and stale < 120:
                    await asyncio.sleep(0.5)
                    cur = len(self._dl_blocks)
                    if cur > last_count:
                        last_count = cur
                        stale = 0
                    else:
                        stale += 1
                    if cur >= self._dl_expected:
                        break
                    if cur > 0 and stale > 60:
                        break

                # Finalize if END wasn't received
                if self._dl_active:
                    self._finalize_download()

            elif cmd == "SLEEP":
                self._text_accum = ""
                await self._ble_write(client, b'Z')
                # Do NOT disconnect from our side — let the ball disconnect
                # Wait up to 5 sec for ball to disconnect
                for _ in range(50):
                    if not client.is_connected:
                        break
                    await asyncio.sleep(0.1)
                self.running = False

            elif cmd == "DISCONNECT":
                self.running = False

        except Exception as e:
            self.error_signal.emit(f"Cmd error: {str(e)[:60]}")


# ═══════════════════════════════════════════════════
#  USB WORKER (unchanged from v1)
# ═══════════════════════════════════════════════════

class USBWorker(QThread):
    data_received = pyqtSignal(list)
    connection_lost = pyqtSignal()

    def __init__(self, port):
        super().__init__()
        self.port = port
        self.running = True

    def run(self):
        try:
            ser = serial.Serial(self.port, 115200, timeout=0.1)
            while self.running:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    try:
                        parts = [float(x) for x in line.split(',')]
                        if len(parts) >= 6:
                            self.data_received.emit(parts)
                    except ValueError:
                        pass
            ser.close()
        except Exception as e:
            print(f"USB Error: {e}")
            self.connection_lost.emit()


# ═══════════════════════════════════════════════════
#  WIDGETS
# ═══════════════════════════════════════════════════

class StatusIndicator(QFrame):
    def __init__(self, size=12):
        super().__init__()
        self.setFixedSize(size, size)
        self.set_status('off')

    def set_status(self, st):
        c = {'off':'#555','ok':'#4CAF50','active':'#2196F3','warning':'#FF9800','error':'#f44336'}.get(st,'#555')
        self.setStyleSheet(f"QFrame{{background:{c};border-radius:{self.width()//2}px;border:2px solid {c};}}")


class DataCard(QFrame):
    def __init__(self, title, value="--"):
        super().__init__()
        self.setFrameStyle(QFrame.StyledPanel)
        lo = QVBoxLayout(self); lo.setContentsMargins(12,8,12,8); lo.setSpacing(2)
        lo.addWidget(QLabel(title, styleSheet="color:#888;font-size:9pt;"))
        self.val = QLabel(value, styleSheet="color:#fff;font-size:16pt;font-weight:bold;")
        lo.addWidget(self.val)

    def set_value(self, v):
        self.val.setText(str(v))


class BatteryWidget(QFrame):
    def __init__(self):
        super().__init__()
        self.setFixedSize(140, 40)
        self.pct = -1; self.volt = 0.0

    def set_level(self, pct, volt=0.0):
        self.pct = pct; self.volt = volt; self.update()

    def paintEvent(self, event):
        from PyQt5.QtGui import QPainter, QBrush, QPen
        p = QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        bx, by, bw, bh = 4, 8, w-20, h-16

        p.setPen(QPen(QColor('#555'), 2)); p.setBrush(QBrush(QColor('#21262d')))
        p.drawRoundedRect(bx, by, bw, bh, 3, 3)
        p.setBrush(QBrush(QColor('#555'))); p.setPen(Qt.NoPen)
        p.drawRect(bx+bw, by+bh//4, 6, bh//2)

        if self.pct >= 0:
            fw = max(0, int((bw-4)*self.pct/100))
            clr = '#4CAF50' if self.pct > 50 else '#FF9800' if self.pct > 20 else '#f44336'
            p.setBrush(QBrush(QColor(clr))); p.drawRoundedRect(bx+2, by+2, fw, bh-4, 2, 2)
            p.setPen(QPen(QColor('#fff'))); p.setFont(QFont('Arial', 9, QFont.Bold))
            txt = f"{self.pct}%" + (f" {self.volt:.2f}V" if self.volt > 0 else "")
            p.drawText(bx, by, bw, bh, Qt.AlignCenter, txt)
        else:
            p.setPen(QPen(QColor('#666'))); p.setFont(QFont('Arial', 9))
            p.drawText(bx, by, bw, bh, Qt.AlignCenter, "N/A")
        p.end()


class AirplaneWidget(QGLWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.roll = self.pitch = self.yaw = 0.0
        self.setMinimumSize(200, 200)

    def set_orientation(self, r, p, y):
        self.roll, self.pitch, self.yaw = r, p, y; self.update()

    def initializeGL(self):
        glClearColor(0.05, 0.07, 0.09, 1.0)
        glEnable(GL_DEPTH_TEST); glEnable(GL_LIGHTING); glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL); glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        glLightfv(GL_LIGHT0, GL_POSITION, [5,5,10,1])
        glLightfv(GL_LIGHT0, GL_AMBIENT, [0.3,0.3,0.3,1])
        glLightfv(GL_LIGHT0, GL_DIFFUSE, [0.8,0.8,0.8,1])

    def resizeGL(self, w, h):
        glViewport(0,0,w,h); glMatrixMode(GL_PROJECTION); glLoadIdentity()
        gluPerspective(45, w/h if h>0 else 1, 0.1, 100); glMatrixMode(GL_MODELVIEW)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT); glLoadIdentity()
        gluLookAt(0,-6,3, 0,0,0, 0,0,1)
        # Grid
        glDisable(GL_LIGHTING); glColor3f(0.2,0.25,0.3); glBegin(GL_LINES)
        for i in range(-5,6):
            glVertex3f(i,-5,0); glVertex3f(i,5,0); glVertex3f(-5,i,0); glVertex3f(5,i,0)
        glEnd(); glEnable(GL_LIGHTING)
        # Rotate
        glRotatef(self.yaw,0,0,1); glRotatef(self.pitch,1,0,0); glRotatef(self.roll,0,1,0)
        # Airplane
        def cube():
            glBegin(GL_QUADS)
            for n,v in [((0,0,1),[(-0.5,-0.5,0.5),(0.5,-0.5,0.5),(0.5,0.5,0.5),(-0.5,0.5,0.5)]),
                        ((0,0,-1),[(-0.5,-0.5,-0.5),(-0.5,0.5,-0.5),(0.5,0.5,-0.5),(0.5,-0.5,-0.5)]),
                        ((0,1,0),[(-0.5,0.5,-0.5),(-0.5,0.5,0.5),(0.5,0.5,0.5),(0.5,0.5,-0.5)]),
                        ((0,-1,0),[(-0.5,-0.5,-0.5),(0.5,-0.5,-0.5),(0.5,-0.5,0.5),(-0.5,-0.5,0.5)]),
                        ((1,0,0),[(0.5,-0.5,-0.5),(0.5,0.5,-0.5),(0.5,0.5,0.5),(0.5,-0.5,0.5)]),
                        ((-1,0,0),[(-0.5,-0.5,-0.5),(-0.5,-0.5,0.5),(-0.5,0.5,0.5),(-0.5,0.5,-0.5)])]:
                glNormal3f(*n)
                for vt in v: glVertex3f(*vt)
            glEnd()
        glColor3f(0.2,0.5,1.0); glPushMatrix(); glScalef(0.3,1.5,0.25); cube(); glPopMatrix()
        glColor3f(0.4,0.7,1.0); glPushMatrix(); glTranslatef(0,1.2,0); glScalef(0.2,0.5,0.2); cube(); glPopMatrix()
        glColor3f(0.2,0.8,0.3); glPushMatrix(); glScalef(2.5,0.4,0.08); cube(); glPopMatrix()
        glColor3f(1.0,0.3,0.2); glPushMatrix(); glTranslatef(0,-1.3,0.3); glScalef(0.08,0.3,0.5); cube(); glPopMatrix()
        glColor3f(1.0,0.6,0.2); glPushMatrix(); glTranslatef(0,-1.3,0.1); glScalef(0.8,0.25,0.06); cube(); glPopMatrix()
        glDisable(GL_LIGHTING); glLineWidth(2); glBegin(GL_LINES)
        glColor3f(1,0,0); glVertex3f(0,0,0); glVertex3f(1.5,0,0)
        glColor3f(0,1,0); glVertex3f(0,0,0); glVertex3f(0,1.5,0)
        glColor3f(0,0,1); glVertex3f(0,0,0); glVertex3f(0,0,1.5)
        glEnd(); glLineWidth(1); glEnable(GL_LIGHTING)


# ═══════════════════════════════════════════════════
#  MAIN WINDOW
# ═══════════════════════════════════════════════════

class BallStudio(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("⚽ XIAO Ball Studio v2.1")
        self.resize(1350, 880)
        self._apply_style()

        # USB
        self.usb_thread = None
        self.live_data = {k: deque(maxlen=400) for k in ['ax','ay','az','roll','pitch','filtered_roll','total_g']}
        self.roll_fused = self.pitch_fused = self.yaw_fused = 0.0
        self.last_time = time.time()
        self.sample_count = 0
        self.usb_raw_data = []
        self.recording = False
        self.gyro_bias = [0.0,0.0,0.0]
        self.accel_scale = 1.0
        self.calibrating = False
        self.calib_samples = []
        self.calib_count = 100
        self.is_calibrated = False

        # BLE
        self.ble_worker = None
        self.ble_connected = False
        self.ball_status = {}
        self.ble_raw_data = []
        self.ble_sample_rate = 100  # default, updated from REC:START

        # Playback
        self.playback_data = None
        self.playback_index = 0
        self.playback_playing = False
        self.playback_speed = 1.0
        self.playback_timer = QTimer()
        self.playback_timer.timeout.connect(self._pb_tick)

        self._init_ui()

        # Status polling timer (when connected)
        self.status_poll_timer = QTimer()
        self.status_poll_timer.timeout.connect(self._poll_status)

        # Stats timer
        self.stats_timer = QTimer()
        self.stats_timer.timeout.connect(self._update_stats)
        self.stats_timer.start(500)

    # ═══════════════════════════════════════════════
    #  STYLE
    # ═══════════════════════════════════════════════

    def _apply_style(self):
        self.setStyleSheet("""
            QMainWindow{background:qlineargradient(x1:0,y1:0,x2:0,y2:1,stop:0 #0d1117,stop:1 #161b22);}
            QWidget{color:#c9d1d9;font-family:'Segoe UI',Arial;font-size:10pt;}
            QTabWidget::pane{border:1px solid #30363d;background:#0d1117;border-radius:8px;}
            QTabBar::tab{background:#21262d;color:#8b949e;padding:12px 28px;margin-right:4px;
                         border-top-left-radius:8px;border-top-right-radius:8px;font-weight:500;}
            QTabBar::tab:selected{background:#0d1117;color:#58a6ff;border-bottom:3px solid #58a6ff;}
            QTabBar::tab:hover:!selected{background:#30363d;color:#c9d1d9;}
            QPushButton{background:#238636;border:none;padding:10px 20px;color:white;
                        font-weight:600;border-radius:6px;min-width:80px;}
            QPushButton:hover{background:#2ea043;} QPushButton:pressed{background:#196c2e;}
            QPushButton:disabled{background:#21262d;color:#484f58;}
            QPushButton#stopBtn{background:#da3633;} QPushButton#stopBtn:hover{background:#f85149;}
            QPushButton#blueBtn{background:#1f6feb;} QPushButton#blueBtn:hover{background:#388bfd;}
            QPushButton#sleepBtn{background:#6e40c9;} QPushButton#sleepBtn:hover{background:#8957e5;}
            QComboBox{background:#21262d;border:1px solid #30363d;border-radius:6px;padding:8px 12px;}
            QComboBox:hover{border-color:#58a6ff;}
            QComboBox::drop-down{border:none;padding-right:8px;}
            QComboBox QAbstractItemView{background:#21262d;border:1px solid #30363d;selection-background-color:#388bfd;}
            QProgressBar{border:none;background:#21262d;border-radius:4px;height:8px;text-align:center;}
            QProgressBar::chunk{background:qlineargradient(x1:0,y1:0,x2:1,y2:0,stop:0 #238636,stop:1 #3fb950);border-radius:4px;}
            QLabel#statusLabel{color:#58a6ff;font-weight:600;font-size:11pt;}
            QFrame#card{background:#161b22;border:1px solid #30363d;border-radius:8px;}
            QSpinBox{background:#21262d;border:1px solid #30363d;border-radius:4px;padding:4px 8px;color:#c9d1d9;}
            QSlider::groove:horizontal{height:6px;background:#21262d;border-radius:3px;}
            QSlider::handle:horizontal{background:#58a6ff;width:16px;margin:-5px 0;border-radius:8px;}
            QSlider::sub-page:horizontal{background:#238636;border-radius:3px;}
        """)

    # ═══════════════════════════════════════════════
    #  UI
    # ═══════════════════════════════════════════════

    def _init_ui(self):
        central = QWidget(); self.setCentralWidget(central)
        lo = QVBoxLayout(central); lo.setContentsMargins(16,16,16,16); lo.setSpacing(12)

        # Header
        hdr = QHBoxLayout()
        t = QLabel("⚽ XIAO Ball Studio v2.1"); t.setStyleSheet("font-size:18pt;font-weight:bold;color:#f0f6fc;")
        hdr.addWidget(t); hdr.addStretch()
        self.card_samples = DataCard("SAMPLES","0"); self.card_fps = DataCard("FPS","--"); self.card_max_g = DataCard("MAX G","--")
        for c in [self.card_samples, self.card_fps, self.card_max_g]:
            c.setObjectName("card"); c.setFixedWidth(100); hdr.addWidget(c)
        lo.addLayout(hdr)

        # Tabs
        self.tabs = QTabWidget()
        self.tabs.addTab(self._usb_tab(), "📡  USB LIVE")
        self.tabs.addTab(self._ball_tab(), "🔴  BALL CONTROL")
        self.tabs.addTab(self._playback_tab(), "▶  PLAYBACK")
        lo.addWidget(self.tabs)

    # ─── USB TAB ───

    def _usb_tab(self):
        w = QWidget(); lo = QVBoxLayout(w); lo.setSpacing(12)
        ctrl = QFrame(); ctrl.setObjectName("card"); cl = QHBoxLayout(ctrl); cl.setContentsMargins(16,12,16,12)

        self.usb_ind = StatusIndicator(); cl.addWidget(self.usb_ind)
        cl.addWidget(QLabel("Port:"))
        self.combo_ports = QComboBox(); self.combo_ports.setMinimumWidth(150); cl.addWidget(self.combo_ports)
        self.btn_ports = QPushButton("↻"); self.btn_ports.setFixedWidth(36)
        self.btn_ports.setObjectName("blueBtn"); self.btn_ports.setToolTip("Refresh COM ports")
        self.btn_ports.clicked.connect(self._refresh_ports); cl.addWidget(self.btn_ports)
        self.btn_refresh = QPushButton("↻ Connect"); self.btn_refresh.setFixedWidth(120)
        self.btn_refresh.clicked.connect(self._usb_toggle); cl.addWidget(self.btn_refresh)
        self.btn_usb = QPushButton("▶ Start"); self.btn_usb.setFixedWidth(100)
        self.btn_usb.clicked.connect(self._usb_rec_toggle); self.btn_usb.setEnabled(False); cl.addWidget(self.btn_usb)
        self.btn_usb_save = QPushButton("💾 Save"); self.btn_usb_save.setFixedWidth(100)
        self.btn_usb_save.clicked.connect(self._usb_save); self.btn_usb_save.setEnabled(False); cl.addWidget(self.btn_usb_save)
        cl.addSpacing(10)
        self.usb_count = QLabel("0 samples"); self.usb_count.setStyleSheet("color:#8b949e;"); cl.addWidget(self.usb_count)
        cl.addStretch()
        self.usb_status = QLabel("Disconnected"); self.usb_status.setObjectName("statusLabel"); cl.addWidget(self.usb_status)
        lo.addWidget(ctrl)

        pg.setConfigOptions(background='#0d1117', foreground='#c9d1d9', antialias=True)
        self.plot_acc = pg.PlotWidget(); self.plot_acc.setTitle("Acceleration",color='#8b949e',size='11pt')
        self.plot_acc.setLabel('left','G'); self.plot_acc.showGrid(x=True,y=True,alpha=0.15)
        self.plot_acc.addLegend(offset=(10,10)); self.plot_acc.setYRange(-4,4)
        self.curve_ax = self.plot_acc.plot(pen=pg.mkPen('#f85149',width=2),name="X")
        self.curve_ay = self.plot_acc.plot(pen=pg.mkPen('#3fb950',width=2),name="Y")
        self.curve_az = self.plot_acc.plot(pen=pg.mkPen('#58a6ff',width=2),name="Z")

        self.airplane = AirplaneWidget()
        ac = QFrame(); ac.setObjectName("card"); al = QVBoxLayout(ac); al.setContentsMargins(8,8,8,8)
        al.addWidget(QLabel("3D Orientation",alignment=Qt.AlignCenter,styleSheet="color:#8b949e;font-size:11pt;"))
        al.addWidget(self.airplane)
        self.orient_lbl = QLabel("Roll: 0°  Pitch: 0°  Yaw: 0°",alignment=Qt.AlignCenter,
                                  styleSheet="color:#58a6ff;font-size:10pt;font-weight:bold;")
        al.addWidget(self.orient_lbl)

        sp = QSplitter(Qt.Horizontal); sp.addWidget(self.plot_acc); sp.addWidget(ac); sp.setSizes([600,400])
        lo.addWidget(sp)
        self._refresh_ports()
        return w

    # ─── BALL CONTROL TAB ───

    def _ball_tab(self):
        w = QWidget(); lo = QVBoxLayout(w); lo.setSpacing(10)

        # Connection bar
        f1 = QFrame(); f1.setObjectName("card"); c1 = QHBoxLayout(f1); c1.setContentsMargins(16,12,16,12)
        self.ble_ind = StatusIndicator(); c1.addWidget(self.ble_ind)
        self.ble_combo = QComboBox(); self.ble_combo.setMinimumWidth(200); c1.addWidget(self.ble_combo)
        self.btn_scan = QPushButton("🔍 Scan"); self.btn_scan.setFixedWidth(100)
        self.btn_scan.setObjectName("blueBtn"); self.btn_scan.clicked.connect(self._ble_scan); c1.addWidget(self.btn_scan)
        self.btn_conn = QPushButton("Connect"); self.btn_conn.setFixedWidth(120)
        self.btn_conn.clicked.connect(self._ble_toggle_conn); c1.addWidget(self.btn_conn)
        c1.addSpacing(10)
        self.bat_widget = BatteryWidget(); c1.addWidget(self.bat_widget)
        c1.addStretch()
        self.ble_status = QLabel("Disconnected"); self.ble_status.setObjectName("statusLabel"); c1.addWidget(self.ble_status)
        lo.addWidget(f1)

        # Control bar
        f2 = QFrame(); f2.setObjectName("card"); c2 = QHBoxLayout(f2); c2.setContentsMargins(16,12,16,12)

        c2.addWidget(QLabel("Duration:"))
        self.spin_dur = QSpinBox(); self.spin_dur.setRange(1,10); self.spin_dur.setValue(5)
        self.spin_dur.setSuffix(" sec"); self.spin_dur.setFixedWidth(90); c2.addWidget(self.spin_dur)

        self.btn_record = QPushButton("⏺ Record"); self.btn_record.setFixedWidth(110)
        self.btn_record.clicked.connect(self._ble_record); self.btn_record.setEnabled(False); c2.addWidget(self.btn_record)

        self.btn_dl = QPushButton("⬇ Download"); self.btn_dl.setFixedWidth(120)
        self.btn_dl.clicked.connect(self._ble_download); self.btn_dl.setEnabled(False); c2.addWidget(self.btn_dl)

        self.btn_ble_save = QPushButton("💾 Save CSV"); self.btn_ble_save.setFixedWidth(110)
        self.btn_ble_save.clicked.connect(self._ble_save); self.btn_ble_save.setEnabled(False); c2.addWidget(self.btn_ble_save)

        c2.addStretch()

        self.btn_status = QPushButton("📊 Status"); self.btn_status.setFixedWidth(100)
        self.btn_status.clicked.connect(self._ble_req_status); self.btn_status.setEnabled(False); c2.addWidget(self.btn_status)

        self.btn_sleep = QPushButton("💤 Sleep"); self.btn_sleep.setFixedWidth(90)
        self.btn_sleep.setObjectName("sleepBtn"); self.btn_sleep.clicked.connect(self._ble_sleep)
        self.btn_sleep.setEnabled(False); c2.addWidget(self.btn_sleep)
        lo.addWidget(f2)

        # Progress
        self.ble_prog = QProgressBar(); self.ble_prog.setRange(0,100); self.ble_prog.setValue(0)
        self.ble_prog.setTextVisible(False); self.ble_prog.setFixedHeight(6); lo.addWidget(self.ble_prog)

        # Info
        self.ble_info = QLabel("Ready"); self.ble_info.setStyleSheet("color:#8b949e;padding:4px;"); lo.addWidget(self.ble_info)

        # ── Playback bar (visible after download) ──
        self.ble_pb_frame = QFrame(); self.ble_pb_frame.setObjectName("card")
        pbl = QHBoxLayout(self.ble_pb_frame); pbl.setContentsMargins(16,8,16,8)

        self.btn_ble_play = QPushButton("▶ Play"); self.btn_ble_play.setFixedWidth(80)
        self.btn_ble_play.clicked.connect(self._ble_play); self.btn_ble_play.setEnabled(False)
        pbl.addWidget(self.btn_ble_play)

        self.btn_ble_pause = QPushButton("⏸"); self.btn_ble_pause.setFixedWidth(40)
        self.btn_ble_pause.clicked.connect(self._ble_pb_pause); self.btn_ble_pause.setEnabled(False)
        pbl.addWidget(self.btn_ble_pause)

        self.btn_ble_reset = QPushButton("⏹"); self.btn_ble_reset.setFixedWidth(40)
        self.btn_ble_reset.setObjectName("stopBtn")
        self.btn_ble_reset.clicked.connect(self._ble_pb_reset); self.btn_ble_reset.setEnabled(False)
        pbl.addWidget(self.btn_ble_reset)

        pbl.addSpacing(8)
        self.ble_pb_slider = QSlider(Qt.Horizontal)
        self.ble_pb_slider.setRange(0, 100); self.ble_pb_slider.setValue(0)
        self.ble_pb_slider.sliderMoved.connect(self._ble_pb_slider_moved)
        pbl.addWidget(self.ble_pb_slider)

        self.ble_pb_time = QLabel("0.00s / 0.00s")
        self.ble_pb_time.setStyleSheet("color:#58a6ff;font-weight:bold;min-width:120px;")
        pbl.addWidget(self.ble_pb_time)

        self.ble_pb_frame.setVisible(False)  # Hidden until data downloaded
        lo.addWidget(self.ble_pb_frame)

        # Playback timer
        self.ble_pb_timer = QTimer()
        self.ble_pb_timer.timeout.connect(self._ble_pb_tick)
        self.ble_pb_index = 0
        self.ble_pb_playing = False

        # Plots + 3D
        self.ble_plot = pg.PlotWidget(); self.ble_plot.setTitle("Acceleration (G)",color='#8b949e',size='11pt')
        self.ble_plot.setLabel('left','G'); self.ble_plot.setLabel('bottom','Time (s)')
        self.ble_plot.showGrid(x=True,y=True,alpha=0.15); self.ble_plot.addLegend(offset=(10,10))
        self.ble_c_ax = self.ble_plot.plot(pen=pg.mkPen('#f85149',width=2),name="X")
        self.ble_c_ay = self.ble_plot.plot(pen=pg.mkPen('#3fb950',width=2),name="Y")
        self.ble_c_az = self.ble_plot.plot(pen=pg.mkPen('#58a6ff',width=2),name="Z")
        self.ble_cursor = pg.InfiniteLine(pos=0, angle=90, pen=pg.mkPen('#f0883e', width=2, style=Qt.DashLine))
        self.ble_cursor.setVisible(False)
        self.ble_plot.addItem(self.ble_cursor)

        self.ble_plot_g = pg.PlotWidget(); self.ble_plot_g.setTitle("Gyroscope (°/s)",color='#8b949e',size='11pt')
        self.ble_plot_g.setLabel('left','°/s'); self.ble_plot_g.setLabel('bottom','Time (s)')
        self.ble_plot_g.showGrid(x=True,y=True,alpha=0.15); self.ble_plot_g.addLegend(offset=(10,10))
        self.ble_c_gx = self.ble_plot_g.plot(pen=pg.mkPen('#f85149',width=2),name="X")
        self.ble_c_gy = self.ble_plot_g.plot(pen=pg.mkPen('#3fb950',width=2),name="Y")
        self.ble_c_gz = self.ble_plot_g.plot(pen=pg.mkPen('#58a6ff',width=2),name="Z")
        self.ble_cursor_g = pg.InfiniteLine(pos=0, angle=90, pen=pg.mkPen('#f0883e', width=2, style=Qt.DashLine))
        self.ble_cursor_g.setVisible(False)
        self.ble_plot_g.addItem(self.ble_cursor_g)

        # Link X axes so zoom/pan syncs between accel and gyro
        self.ble_plot_g.setXLink(self.ble_plot)

        self.ble_airplane = AirplaneWidget()
        bac = QFrame(); bac.setObjectName("card"); bal = QVBoxLayout(bac); bal.setContentsMargins(8,8,8,8)
        bal.addWidget(QLabel("3D Orientation",alignment=Qt.AlignCenter,styleSheet="color:#8b949e;font-size:11pt;"))
        bal.addWidget(self.ble_airplane)
        self.ble_orient = QLabel("Roll: 0°  Pitch: 0°  Yaw: 0°",alignment=Qt.AlignCenter,
                                  styleSheet="color:#58a6ff;font-size:10pt;font-weight:bold;")
        bal.addWidget(self.ble_orient)

        plots_v = QSplitter(Qt.Vertical); plots_v.addWidget(self.ble_plot); plots_v.addWidget(self.ble_plot_g); plots_v.setSizes([250,250])
        sp = QSplitter(Qt.Horizontal); sp.addWidget(plots_v); sp.addWidget(bac); sp.setSizes([600,400])
        lo.addWidget(sp)
        return w

    # ─── PLAYBACK TAB ───

    def _playback_tab(self):
        w = QWidget(); lo = QVBoxLayout(w); lo.setSpacing(12)

        ctrl = QFrame(); ctrl.setObjectName("card"); cl = QHBoxLayout(ctrl); cl.setContentsMargins(16,12,16,12)
        self.btn_pb_load = QPushButton("📂 Load CSV"); self.btn_pb_load.setFixedWidth(120)
        self.btn_pb_load.clicked.connect(self._pb_load); cl.addWidget(self.btn_pb_load)
        cl.addSpacing(10)
        self.btn_pb_play = QPushButton("▶ Play"); self.btn_pb_play.setFixedWidth(80)
        self.btn_pb_play.clicked.connect(self._pb_play); self.btn_pb_play.setEnabled(False); cl.addWidget(self.btn_pb_play)
        self.btn_pb_pause = QPushButton("⏸"); self.btn_pb_pause.setFixedWidth(40)
        self.btn_pb_pause.clicked.connect(self._pb_pause); self.btn_pb_pause.setEnabled(False); cl.addWidget(self.btn_pb_pause)
        self.btn_pb_stop = QPushButton("⏹"); self.btn_pb_stop.setFixedWidth(40); self.btn_pb_stop.setObjectName("stopBtn")
        self.btn_pb_stop.clicked.connect(self._pb_stop); self.btn_pb_stop.setEnabled(False); cl.addWidget(self.btn_pb_stop)
        cl.addSpacing(10); cl.addWidget(QLabel("Speed:"))
        self.combo_speed = QComboBox(); self.combo_speed.setFixedWidth(80)
        for s in ["0.25x","0.5x","1x","2x","4x"]: self.combo_speed.addItem(s)
        self.combo_speed.setCurrentText("1x"); self.combo_speed.currentTextChanged.connect(self._pb_speed_changed)
        cl.addWidget(self.combo_speed); cl.addStretch()
        self.pb_time = QLabel("0.00s / 0.00s"); self.pb_time.setStyleSheet("color:#58a6ff;font-weight:bold;font-size:11pt;")
        cl.addWidget(self.pb_time)
        self.pb_info = QLabel("No data"); self.pb_info.setStyleSheet("color:#8b949e;"); cl.addWidget(self.pb_info)
        lo.addWidget(ctrl)

        self.pb_slider = QSlider(Qt.Horizontal); self.pb_slider.setRange(0,1000); self.pb_slider.setValue(0)
        self.pb_slider.sliderMoved.connect(self._pb_slider_moved); lo.addWidget(self.pb_slider)

        self.pb_plot_a = pg.PlotWidget(); self.pb_plot_a.setTitle("Acceleration (G)",color='#8b949e',size='11pt')
        self.pb_plot_a.setLabel('left','G'); self.pb_plot_a.setLabel('bottom','Time (s)')
        self.pb_plot_a.showGrid(x=True,y=True,alpha=0.15); self.pb_plot_a.addLegend(offset=(10,10))
        self.pb_ca = self.pb_plot_a.plot(pen=pg.mkPen('#f85149',width=2),name="X")
        self.pb_cb = self.pb_plot_a.plot(pen=pg.mkPen('#3fb950',width=2),name="Y")
        self.pb_cc = self.pb_plot_a.plot(pen=pg.mkPen('#58a6ff',width=2),name="Z")
        self.pb_cursor = pg.InfiniteLine(pos=0,angle=90,pen=pg.mkPen('#f0883e',width=2,style=Qt.DashLine))
        self.pb_plot_a.addItem(self.pb_cursor)

        self.pb_plot_g = pg.PlotWidget(); self.pb_plot_g.setTitle("Gyroscope (°/s)",color='#8b949e',size='11pt')
        self.pb_plot_g.setLabel('left','°/s'); self.pb_plot_g.setLabel('bottom','Time (s)')
        self.pb_plot_g.showGrid(x=True,y=True,alpha=0.15); self.pb_plot_g.addLegend(offset=(10,10))
        self.pb_gx = self.pb_plot_g.plot(pen=pg.mkPen('#f85149',width=2),name="X")
        self.pb_gy = self.pb_plot_g.plot(pen=pg.mkPen('#3fb950',width=2),name="Y")
        self.pb_gz = self.pb_plot_g.plot(pen=pg.mkPen('#58a6ff',width=2),name="Z")
        self.pb_cursor_g = pg.InfiniteLine(pos=0,angle=90,pen=pg.mkPen('#f0883e',width=2,style=Qt.DashLine))
        self.pb_plot_g.addItem(self.pb_cursor_g)

        self.pb_airplane = AirplaneWidget()
        pac = QFrame(); pac.setObjectName("card"); pal = QVBoxLayout(pac); pal.setContentsMargins(8,8,8,8)
        pal.addWidget(QLabel("3D Playback",alignment=Qt.AlignCenter,styleSheet="color:#8b949e;font-size:11pt;"))
        pal.addWidget(self.pb_airplane)
        self.pb_orient = QLabel("Roll: 0°  Pitch: 0°  Yaw: 0°",alignment=Qt.AlignCenter,
                                 styleSheet="color:#58a6ff;font-size:10pt;font-weight:bold;")
        pal.addWidget(self.pb_orient)

        plots = QSplitter(Qt.Vertical); plots.addWidget(self.pb_plot_a); plots.addWidget(self.pb_plot_g); plots.setSizes([250,250])
        sp = QSplitter(Qt.Horizontal); sp.addWidget(plots); sp.addWidget(pac); sp.setSizes([600,400])
        lo.addWidget(sp)
        return w

    # ═══════════════════════════════════════════════
    #  USB HANDLERS
    # ═══════════════════════════════════════════════

    def _refresh_ports(self):
        self.combo_ports.clear()
        for p in serial.tools.list_ports.comports():
            self.combo_ports.addItem(p.device)

    def _usb_toggle(self):
        if self.usb_thread:
            self.usb_thread.running = False; self.usb_thread.quit(); self.usb_thread.wait(1000)
            self.usb_thread = None
            self.btn_refresh.setText("↻ Connect"); self.btn_refresh.setObjectName(""); self.btn_refresh.style().polish(self.btn_refresh)
            self.btn_usb.setEnabled(False); self.btn_usb_save.setEnabled(len(self.usb_raw_data) > 0)
            self.usb_status.setText("Disconnected"); self.usb_ind.set_status('off'); self.recording = False
        else:
            port = self.combo_ports.currentText()
            if not port: return
            self._usb_reset()
            self.usb_thread = USBWorker(port)
            self.usb_thread.data_received.connect(self._usb_data)
            self.usb_thread.connection_lost.connect(lambda: (self._usb_toggle() if self.usb_thread else None,
                                                              self.usb_status.setText("Connection Lost"),
                                                              self.usb_ind.set_status('error')))
            self.usb_thread.start()
            self.btn_refresh.setText("⏹ Disconnect"); self.btn_refresh.setObjectName("stopBtn")
            self.btn_refresh.style().polish(self.btn_refresh)
            self.usb_ind.set_status('warning'); self.usb_status.setText("Calibrating...")
            self.calibrating = True; self.calib_samples = []

    def _usb_reset(self):
        self.live_data = {k: deque(maxlen=400) for k in self.live_data}
        self.usb_raw_data = []; self.sample_count = 0
        self.roll_fused = self.pitch_fused = self.yaw_fused = 0.0
        self.recording = False; self.last_time = time.time()
        for c in [self.curve_ax, self.curve_ay, self.curve_az]: c.setData([])
        self.card_samples.set_value("0"); self.card_max_g.set_value("--")
        self.usb_count.setText("0 samples"); self.btn_usb_save.setEnabled(False)
        self.btn_usb.setEnabled(False); self.btn_usb.setText("▶ Start")
        self.btn_usb.setObjectName(""); self.btn_usb.style().polish(self.btn_usb)

    def _usb_rec_toggle(self):
        if self.recording:
            self.recording = False; self.btn_usb.setText("▶ Start"); self.btn_usb.setObjectName("")
            self.btn_usb.style().polish(self.btn_usb)
            self.usb_status.setText(f"Stopped: {len(self.usb_raw_data)} samples")
            self.btn_usb_save.setEnabled(len(self.usb_raw_data) > 0)
        else:
            self.recording = True; self.usb_raw_data = []; self.sample_count = 0
            self.usb_record_start = time.time()
            self.btn_usb.setText("■ Stop"); self.btn_usb.setObjectName("stopBtn")
            self.btn_usb.style().polish(self.btn_usb)
            self.usb_status.setText("Recording..."); self.btn_usb_save.setEnabled(False)

    def _usb_data(self, data):
        ax_r,ay_r,az_r = data[0],data[1],data[2]
        gx_r = data[3] if len(data)>3 else 0
        gy_r = data[4] if len(data)>4 else 0
        gz_r = data[5] if len(data)>5 else 0

        if self.calibrating:
            self.calib_samples.append([ax_r,ay_r,az_r,gx_r,gy_r,gz_r])
            rem = self.calib_count - len(self.calib_samples)
            self.usb_status.setText(f"Calibrating... {rem}")
            if len(self.calib_samples) >= self.calib_count: self._finish_calib()
            return
        if not self.recording: return

        gx = gx_r - self.gyro_bias[0]; gy = gy_r - self.gyro_bias[1]; gz = gz_r - self.gyro_bias[2]
        ax = ax_r*self.accel_scale; ay = ay_r*self.accel_scale; az = az_r*self.accel_scale
        ct = time.time(); dt = ct - self.last_time; self.last_time = ct

        ra = np.arctan2(ay,az)*57.2958; pa = np.arctan2(-ax,np.sqrt(ay*ay+az*az))*57.2958
        if not self.live_data['roll']:
            self.roll_fused=ra; self.pitch_fused=pa; self.yaw_fused=0.0
        else:
            if abs(ra-self.live_data['roll'][-1])>180: self.roll_fused=ra
            else:
                self.roll_fused = 0.96*(self.roll_fused+gx*dt)+0.04*ra
                while self.roll_fused>180: self.roll_fused-=360
                while self.roll_fused<-180: self.roll_fused+=360
            if abs(pa-self.live_data['pitch'][-1])>180: self.pitch_fused=pa
            else:
                self.pitch_fused = 0.96*(self.pitch_fused+gy*dt)+0.04*pa
                while self.pitch_fused>180: self.pitch_fused-=360
                while self.pitch_fused<-180: self.pitch_fused+=360
            self.yaw_fused += gz*dt
            while self.yaw_fused>180: self.yaw_fused-=360
            while self.yaw_fused<-180: self.yaw_fused+=360

        tg = np.sqrt(ax*ax+ay*ay+az*az); self.sample_count += 1
        ts = int((ct-self.usb_record_start)*1000)
        self.usb_raw_data.append([ts,ax,ay,az,gx,gy,gz,self.roll_fused,self.pitch_fused,self.yaw_fused])
        for k,v in zip(['ax','ay','az','roll','pitch','filtered_roll','total_g'],
                        [ax,ay,az,ra,pa,self.roll_fused,tg]):
            self.live_data[k].append(v)
        self.curve_ax.setData(list(self.live_data['ax']))
        self.curve_ay.setData(list(self.live_data['ay']))
        self.curve_az.setData(list(self.live_data['az']))
        self.airplane.set_orientation(self.roll_fused,-self.pitch_fused,self.yaw_fused)
        self.orient_lbl.setText(f"Roll:{self.roll_fused:.1f}° Pitch:{self.pitch_fused:.1f}° Yaw:{self.yaw_fused:.1f}°")
        if self.sample_count%50==0: self.usb_count.setText(f"{self.sample_count} samples")
        self.usb_ind.set_status('active')

    def _finish_calib(self):
        self.calibrating = False; s = np.array(self.calib_samples)
        self.gyro_bias = [np.mean(s[:,3]),np.mean(s[:,4]),np.mean(s[:,5])]
        gm = np.sqrt(np.mean(s[:,0])**2+np.mean(s[:,1])**2+np.mean(s[:,2])**2)
        self.accel_scale = 1.0/gm if gm>0.1 else 1.0
        ac = np.mean(s[:,0])*self.accel_scale; bc = np.mean(s[:,1])*self.accel_scale; cc = np.mean(s[:,2])*self.accel_scale
        ir = np.arctan2(bc,cc)*57.2958; ip = np.arctan2(-ac,np.sqrt(bc**2+cc**2))*57.2958
        self.roll_fused=self.pitch_fused=self.yaw_fused=0.0; self.last_time=time.time()
        self.is_calibrated=True; self.btn_usb.setEnabled(True); self.usb_ind.set_status('ok')
        self.usb_status.setText(f"Ready | Roll={ir:.1f}° Pitch={ip:.1f}°")

    def _update_stats(self):
        self.card_samples.set_value(str(self.sample_count))
        if self.usb_thread and self.live_data['total_g']:
            self.card_max_g.set_value(f"{max(self.live_data['total_g']):.1f}")
            self.card_fps.set_value("50"); self.usb_ind.set_status('ok')

    def _usb_save(self):
        if not self.usb_raw_data: return
        fn, _ = QFileDialog.getSaveFileName(self,"Save USB Data",f"usb_{datetime.now():%Y%m%d_%H%M%S}.csv","CSV (*.csv)")
        if fn:
            try:
                arr = np.array(self.usb_raw_data)
                np.savetxt(fn, arr, delimiter=',', header='timestamp_ms,ax,ay,az,gx,gy,gz,roll,pitch,yaw',
                           comments='', fmt=['%d']+['%.6f']*6+['%.2f']*3)
                self.usb_status.setText(f"Saved: {len(self.usb_raw_data)} samples")
            except Exception as e:
                self.usb_status.setText(f"Error: {e}")

    # ═══════════════════════════════════════════════
    #  BLE HANDLERS
    # ═══════════════════════════════════════════════

    def _ble_scan(self):
        self.btn_scan.setEnabled(False); self.ble_status.setText("Scanning...")
        self.ble_ind.set_status('warning'); self.ble_combo.clear()
        self._scanner = BLEScanWorker(5.0)
        self._scanner.devices_found.connect(self._ble_scan_done)
        self._scanner.scan_error.connect(lambda e: (self.btn_scan.setEnabled(True),
                                                     self.ble_status.setText(f"Scan error: {e}"),
                                                     self.ble_ind.set_status('error')))
        self._scanner.start()

    def _ble_scan_done(self, devs):
        self.btn_scan.setEnabled(True)
        if not devs:
            self.ble_status.setText("No balls found"); self.ble_ind.set_status('warning'); return
        self._scan_devs = devs
        for d in devs:
            bat = f" {d['battery']}%" if d['battery']>=0 else ""
            self.ble_combo.addItem(f"{d['name']} ({d['address'][-5:]}){bat} [{d['rssi']}dBm]", d['address'])
        self.ble_status.setText(f"Found {len(devs)} device(s)"); self.ble_ind.set_status('ok')

    def _ble_toggle_conn(self):
        if self.ble_connected:
            if self.ble_worker: self.ble_worker.send_command("DISCONNECT")
        else:
            idx = self.ble_combo.currentIndex()
            if idx < 0: return
            addr = self.ble_combo.itemData(idx)
            if not addr: return
            self.ble_worker = BLEControlWorker(addr)
            self.ble_worker.connected_changed.connect(self._ble_conn_changed)
            self.ble_worker.status_received.connect(self._ble_on_status)
            self.ble_worker.recording_status.connect(self._ble_on_rec)
            self.ble_worker.download_progress.connect(self._ble_on_dl_prog)
            self.ble_worker.download_finished.connect(self._ble_on_dl_done)
            self.ble_worker.message_received.connect(self._ble_on_msg)
            self.ble_worker.error_signal.connect(self._ble_on_err)
            self.ble_worker.start()
            self.btn_conn.setEnabled(False); self.ble_status.setText("Connecting..."); self.ble_ind.set_status('warning')

    def _ble_conn_changed(self, connected):
        self.ble_connected = connected; self.btn_conn.setEnabled(True)
        btns = [self.btn_record, self.btn_dl, self.btn_status, self.btn_sleep]
        if connected:
            self.btn_conn.setText("Disconnect"); self.btn_conn.setObjectName("stopBtn")
            self.btn_conn.style().polish(self.btn_conn)
            self.ble_ind.set_status('ok'); self.ble_status.setText("Connected")
            for b in btns: b.setEnabled(True)
            self.btn_dl.setEnabled(False)  # Enable only after status shows data
            # Auto-request status after connect
            QTimer.singleShot(500, self._ble_req_status)
            # Start polling status every 3s
            self.status_poll_timer.start(3000)
        else:
            self.btn_conn.setText("Connect"); self.btn_conn.setObjectName("")
            self.btn_conn.style().polish(self.btn_conn)
            self.ble_ind.set_status('off'); self.ble_status.setText("Disconnected")
            for b in btns: b.setEnabled(False)
            self.btn_ble_save.setEnabled(False)
            self.status_poll_timer.stop()
            self.ble_worker = None

    def _poll_status(self):
        """Auto-poll status while connected"""
        if self.ble_worker and self.ble_connected:
            self.ble_worker.send_command("STATUS")

    def _ble_on_status(self, st):
        self.ball_status = st
        state = st.get('state', '?')
        bp = st.get('battery_pct', -1)
        bv = st.get('battery_v', 0.0)
        samples = st.get('samples', 0)
        maxd = st.get('max_duration', 10)

        self.bat_widget.set_level(bp, bv)
        self.ble_status.setText(f"{state} | {samples} samples in memory")
        self.ble_ind.set_status('ok')

        # Enable/disable buttons based on state
        self.btn_record.setEnabled(state in ('IDLE', 'READY'))
        self.btn_dl.setEnabled(samples > 0 and state == 'READY')
        self.spin_dur.setMaximum(maxd)

        self.ble_info.setText(f"Status: {state}, Battery: {bp}% ({bv:.2f}V), Samples: {samples}")

    def _ble_on_rec(self, status, val, hz):
        if status == "START":
            if hz > 0:
                self.ble_sample_rate = hz
            self.ble_info.setText(f"⏺ Recording {val}s @ {self.ble_sample_rate}Hz...")
            self.ble_ind.set_status('active')
            self.btn_record.setEnabled(False)
            self.btn_dl.setEnabled(False)
        elif status == "DONE":
            self.ble_info.setText(f"✓ Recording done: {val} samples")
            self.ble_ind.set_status('ok')
            self.btn_record.setEnabled(True)
            self.btn_dl.setEnabled(True)
            # Resume polling and request fresh status
            self.status_poll_timer.start(3000)
            QTimer.singleShot(300, self._ble_req_status)

    def _ble_on_dl_prog(self, cur, total):
        self.ble_prog.setMaximum(max(total,1)); self.ble_prog.setValue(cur)
        pct = int(100*cur/max(total,1))
        self.ble_info.setText(f"Downloading... {cur}/{total} ({pct}%)")

    def _ble_on_dl_done(self, data):
        self.btn_dl.setEnabled(True)
        self.status_poll_timer.start(3000)  # Resume polling
        if not data:
            self.ble_info.setText("No data received"); self.ble_ind.set_status('warning'); return

        # Process data: raw int16 → physical units (no auto-calibration)
        dt = 1.0 / self.ble_sample_rate

        t_arr, ax_arr, ay_arr, az_arr = [],[],[],[]
        gx_arr, gy_arr, gz_arr = [],[],[]
        roll_arr, pitch_arr, yaw_arr = [],[],[]
        fr = fp = fy = 0.0

        for i, row in enumerate(data):
            if len(row)<6: continue
            t = i*dt; t_arr.append(t)
            fax=row[0]*ACCEL_SCALE; fay=row[1]*ACCEL_SCALE; faz=row[2]*ACCEL_SCALE
            fgx=row[3]*GYRO_SCALE;  fgy=row[4]*GYRO_SCALE;  fgz=row[5]*GYRO_SCALE
            ax_arr.append(fax); ay_arr.append(fay); az_arr.append(faz)
            gx_arr.append(fgx); gy_arr.append(fgy); gz_arr.append(fgz)

            # Orientation for 3D playback only (not saved to CSV)
            ra = np.arctan2(fay, faz) * 57.2958
            pa = np.arctan2(-fax, np.sqrt(fay*fay + faz*faz)) * 57.2958
            if i == 0:
                fr = ra; fp = pa; fy = 0.0
            else:
                fr = 0.96*(fr + fgx*dt) + 0.04*ra
                fp = 0.96*(fp + fgy*dt) + 0.04*pa
                fy += fgz*dt
            roll_arr.append(fr); pitch_arr.append(fp); yaw_arr.append(fy)

        if ax_arr:
            # ble_raw_data: 7 columns for CSV, orientation stored separately for playback
            self.ble_raw_data = list(zip(t_arr,ax_arr,ay_arr,az_arr,gx_arr,gy_arr,gz_arr))
            self.ble_orientation = list(zip(roll_arr, pitch_arr, yaw_arr))
            self.ble_c_ax.setData(t_arr,ax_arr); self.ble_c_ay.setData(t_arr,ay_arr); self.ble_c_az.setData(t_arr,az_arr)
            self.ble_c_gx.setData(t_arr,gx_arr); self.ble_c_gy.setData(t_arr,gy_arr); self.ble_c_gz.setData(t_arr,gz_arr)
            self.ble_info.setText(f"✓ Downloaded: {len(ax_arr)} pts, {t_arr[-1]:.1f}s @ {self.ble_sample_rate}Hz (raw)")
            self.ble_ind.set_status('ok'); self.ble_prog.setValue(self.ble_prog.maximum())
            self.btn_ble_save.setEnabled(True)
            self.ble_airplane.set_orientation(roll_arr[-1], -pitch_arr[-1], yaw_arr[-1])
            self.ble_orient.setText(f"Roll:{roll_arr[-1]:.1f}° Pitch:{pitch_arr[-1]:.1f}° Yaw:{yaw_arr[-1]:.1f}°")

            # Enable playback
            self.ble_pb_frame.setVisible(True)
            self.btn_ble_play.setEnabled(True)
            self.btn_ble_reset.setEnabled(True)
            self.btn_ble_pause.setEnabled(True)
            self.ble_pb_slider.setRange(0, len(t_arr) - 1)
            self.ble_pb_slider.setValue(0)
            self.ble_pb_time.setText(f"0.00s / {t_arr[-1]:.2f}s")
            self.ble_pb_index = 0
            self.ble_cursor.setVisible(True)
            self.ble_cursor.setValue(0)
            self.ble_cursor_g.setVisible(True)
            self.ble_cursor_g.setValue(0)

    def _ble_on_msg(self, msg):
        self.ble_info.setText(msg); print(f"[BLE MSG] {msg}")

    def _ble_on_err(self, err):
        self.ble_info.setText(f"⚠ {err}"); self.ble_ind.set_status('error')
        print(f"[BLE ERR] {err}")

    def _ble_req_status(self):
        if self.ble_worker and self.ble_connected:
            self.ble_worker.send_command("STATUS")

    def _ble_record(self):
        if self.ble_worker and self.ble_connected:
            dur = self.spin_dur.value()
            self.ble_info.setText(f"Sending record command ({dur}s)...")
            self.status_poll_timer.stop()  # Don't poll during recording
            self.ble_worker.send_command(f"RECORD:{dur}")
            self.ble_prog.setValue(0)

    def _ble_download(self):
        if self.ble_worker and self.ble_connected:
            self.btn_dl.setEnabled(False); self.ble_prog.setValue(0)
            self.ble_raw_data = []; self.ble_c_ax.setData([]); self.ble_c_ay.setData([]); self.ble_c_az.setData([])
            self.ble_c_gx.setData([]); self.ble_c_gy.setData([]); self.ble_c_gz.setData([])
            self.ble_info.setText("Starting download...")
            self.status_poll_timer.stop()  # CRITICAL: don't poll during download
            self.ble_worker.send_command("DOWNLOAD")

    def _ble_sleep(self):
        if self.ble_worker and self.ble_connected:
            self.status_poll_timer.stop()
            self.ble_info.setText("Sending sleep command...")
            self.ble_worker.send_command("SLEEP")

    def _ble_save(self):
        if not self.ble_raw_data: return
        fn, _ = QFileDialog.getSaveFileName(self,"Save Ball Data",f"ball_{datetime.now():%Y%m%d_%H%M%S}.csv","CSV (*.csv)")
        if fn:
            with open(fn,'w') as f:
                f.write("time_s,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps\n")
                for r in self.ble_raw_data:
                    f.write(f"{r[0]:.3f},{r[1]:.6f},{r[2]:.6f},{r[3]:.6f},{r[4]:.2f},{r[5]:.2f},{r[6]:.2f}\n")
            self.ble_info.setText(f"Saved: {fn}")

    # ── BLE tab playback ──

    def _ble_play(self):
        if not self.ble_raw_data: return
        self.ble_pb_playing = True
        self._ble_pb_start_time = time.time()
        self._ble_pb_start_index = self.ble_pb_index
        self.ble_pb_timer.start(30)  # ~33 fps, enough for smooth cursor
        self.btn_ble_play.setEnabled(False)
        self.btn_ble_pause.setEnabled(True)

    def _ble_pb_pause(self):
        self.ble_pb_playing = False
        self.ble_pb_timer.stop()
        self.btn_ble_play.setEnabled(True)

    def _ble_pb_reset(self):
        self.ble_pb_playing = False
        self.ble_pb_timer.stop()
        self.ble_pb_index = 0
        self.ble_pb_slider.setValue(0)
        self.btn_ble_play.setEnabled(True)
        if self.ble_raw_data:
            r = self.ble_raw_data[0]
            self.ble_cursor.setValue(r[0])
            self.ble_cursor_g.setValue(r[0])
            self.ble_pb_time.setText(f"0.00s / {self.ble_raw_data[-1][0]:.2f}s")
            if hasattr(self, 'ble_orientation') and self.ble_orientation:
                ro, pi, ya = self.ble_orientation[0]
                self.ble_airplane.set_orientation(ro, -pi, ya)
                self.ble_orient.setText(f"Roll:{ro:.1f}° Pitch:{pi:.1f}° Yaw:{ya:.1f}°")

    def _ble_pb_slider_moved(self, val):
        if not self.ble_raw_data: return
        self.ble_pb_index = val
        self._ble_pb_update()

    def _ble_pb_tick(self):
        if not self.ble_raw_data: return
        elapsed = time.time() - self._ble_pb_start_time
        start_t = self.ble_raw_data[self._ble_pb_start_index][0]
        target_t = start_t + elapsed
        # Find index closest to target_t
        while (self.ble_pb_index < len(self.ble_raw_data) - 1
               and self.ble_raw_data[self.ble_pb_index][0] < target_t):
            self.ble_pb_index += 1
        if self.ble_pb_index >= len(self.ble_raw_data) - 1:
            self.ble_pb_index = len(self.ble_raw_data) - 1
            self._ble_pb_pause()
        self._ble_pb_update()

    def _ble_pb_update(self):
        if not self.ble_raw_data: return
        i = min(self.ble_pb_index, len(self.ble_raw_data) - 1)
        r = self.ble_raw_data[i]
        t = r[0]; total = self.ble_raw_data[-1][0]

        self.ble_pb_slider.blockSignals(True)
        self.ble_pb_slider.setValue(i)
        self.ble_pb_slider.blockSignals(False)

        self.ble_cursor.setValue(t)
        self.ble_cursor_g.setValue(t)
        self.ble_pb_time.setText(f"{t:.2f}s / {total:.2f}s")

        if hasattr(self, 'ble_orientation') and i < len(self.ble_orientation):
            ro, pi, ya = self.ble_orientation[i]
            self.ble_airplane.set_orientation(ro, -pi, ya)
            self.ble_orient.setText(f"Roll:{ro:.1f}° Pitch:{pi:.1f}° Yaw:{ya:.1f}°")

    # ═══════════════════════════════════════════════
    #  PLAYBACK HANDLERS
    # ═══════════════════════════════════════════════

    def _pb_load(self):
        fn, _ = QFileDialog.getOpenFileName(self,"Load CSV","","CSV (*.csv);;All (*)")
        if not fn: return
        try:
            raw = np.genfromtxt(fn,delimiter=',',skip_header=1,filling_values=0.0)
            if raw.ndim<2 or raw.shape[1]<7:
                self.pb_info.setText("Need 7+ columns"); return

            tc = raw[:,0]; axc=raw[:,1]; ayc=raw[:,2]; azc=raw[:,3]
            gxc=raw[:,4]; gyc=raw[:,5]; gzc=raw[:,6]

            # Normalize time to seconds starting at 0
            if tc[-1] > 500:  # Timestamps in ms (USB saves 0, 20, 40...)
                tc = (tc - tc[0]) / 1000.0
            else:
                tc = tc - tc[0]

            if raw.shape[1]>=10:
                rc=raw[:,7]; pc=raw[:,8]; yc=raw[:,9]
            else:
                rc=np.zeros(len(tc)); pc=np.zeros(len(tc)); yc=np.zeros(len(tc))
                fr=fp=fy=0.0
                for i in range(len(tc)):
                    dt = (tc[i]-tc[i-1]) if i>0 else 0.01
                    ra=np.arctan2(ayc[i],azc[i])*57.2958; pa=np.arctan2(-axc[i],np.sqrt(ayc[i]**2+azc[i]**2))*57.2958
                    if i==0: fr=ra; fp=pa
                    else: fr=0.96*(fr+gxc[i]*dt)+0.04*ra; fp=0.96*(fp+gyc[i]*dt)+0.04*pa; fy+=gzc[i]*dt
                    rc[i]=fr; pc[i]=fp; yc[i]=fy

            self.playback_data = {'time':tc,'ax':axc,'ay':ayc,'az':azc,'gx':gxc,'gy':gyc,'gz':gzc,'roll':rc,'pitch':pc,'yaw':yc}
            self.playback_index = 0

            self.pb_ca.setData(tc,axc); self.pb_cb.setData(tc,ayc); self.pb_cc.setData(tc,azc)
            self.pb_gx.setData(tc,gxc); self.pb_gy.setData(tc,gyc); self.pb_gz.setData(tc,gzc)
            self.pb_slider.setRange(0,len(tc)-1)
            self.pb_time.setText(f"0.00s / {tc[-1]:.2f}s")
            self.pb_info.setText(f"Loaded: {len(tc)} pts, {tc[-1]:.1f}s")
            for b in [self.btn_pb_play,self.btn_pb_stop,self.btn_pb_pause]: b.setEnabled(True)
            self.pb_airplane.set_orientation(rc[0],-pc[0],yc[0])
        except Exception as e:
            self.pb_info.setText(f"Load error: {e}")

    def _pb_play(self):
        if not self.playback_data: return
        self.playback_playing = True
        dt = np.mean(np.diff(self.playback_data['time'][:50]))
        interval = max(5, int(dt*1000/self.playback_speed))
        self.playback_timer.start(interval); self.btn_pb_play.setEnabled(False); self.btn_pb_pause.setEnabled(True)

    def _pb_pause(self):
        self.playback_playing = False; self.playback_timer.stop(); self.btn_pb_play.setEnabled(True)

    def _pb_stop(self):
        self.playback_playing = False; self.playback_timer.stop(); self.playback_index = 0
        self.pb_slider.setValue(0); self.btn_pb_play.setEnabled(True)
        if self.playback_data:
            d = self.playback_data
            self.pb_cursor.setValue(0); self.pb_cursor_g.setValue(0)
            self.pb_airplane.set_orientation(d['roll'][0],-d['pitch'][0],d['yaw'][0])
            self.pb_time.setText(f"0.00s / {d['time'][-1]:.2f}s")

    def _pb_speed_changed(self, txt):
        self.playback_speed = float(txt.replace('x',''))
        if self.playback_playing:
            self.playback_timer.stop()
            dt = np.mean(np.diff(self.playback_data['time'][:50]))
            self.playback_timer.start(max(5,int(dt*1000/self.playback_speed)))

    def _pb_slider_moved(self, val):
        if not self.playback_data: return
        self.playback_index = val; self._pb_update()

    def _pb_tick(self):
        if not self.playback_data: return
        self.playback_index += 1
        if self.playback_index >= len(self.playback_data['time']):
            self._pb_pause(); self.playback_index = len(self.playback_data['time'])-1
        self._pb_update()

    def _pb_update(self):
        d = self.playback_data; i = min(self.playback_index, len(d['time'])-1)
        t = d['time'][i]; total = d['time'][-1]
        self.pb_slider.blockSignals(True); self.pb_slider.setValue(i); self.pb_slider.blockSignals(False)
        self.pb_cursor.setValue(t); self.pb_cursor_g.setValue(t)
        self.pb_time.setText(f"{t:.2f}s / {total:.2f}s")
        self.pb_airplane.set_orientation(d['roll'][i],-d['pitch'][i],d['yaw'][i])
        self.pb_orient.setText(f"Roll:{d['roll'][i]:.1f}° Pitch:{d['pitch'][i]:.1f}° Yaw:{d['yaw'][i]:.1f}°")


# ═══════════════════════════════════════════════════
if __name__ == "__main__":
    app = QApplication.instance() or QApplication(sys.argv)
    app.setStyle('Fusion')
    window = BallStudio()
    window.show()
    sys.exit(app.exec_())