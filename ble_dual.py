"""
XIAO Ball Studio v3.1 — Desktop application for autonomous IMU ball
- USB Live tab (wired debug)
- Ball Control tab (BLE: scan, connect, status, record, download, sleep)
  * Supports TWO simultaneous BLE devices
  * Individual: connect, disconnect, status
  * Group: record, download, sleep (applies to all active devices)
  * Left side: Device 1 graphs (accel + gyro)
  * Right side: Device 2 graphs (accel + gyro)
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
                             QSizePolicy, QFileDialog, QSlider, QSpinBox,
                             QGroupBox, QGridLayout)
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
            print(f"[SCAN] Found: '{name}' @ {device.address}")  # Debug
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

    def __init__(self, address, device_id=0):
        super().__init__()
        self.address = address
        self.device_id = device_id
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

    async def _ble_write(self, client, data: bytes):
        try:
            await client.write_gatt_char(UART_RX_CHAR_UUID, data, response=False)
            return True
        except Exception as e:
            try:
                await client.write_gatt_char(UART_RX_CHAR_UUID, data, response=True)
                return True
            except Exception as e2:
                self.error_signal.emit(f"BLE write failed: {e2}")
                return False

    def _on_notify(self, sender, data: bytearray):
        if not self._dl_active:
            try:
                txt_preview = data.decode('utf-8', errors='replace')[:40]
            except:
                txt_preview = "?"
            print(f"[BLE{self.device_id} RAW] {len(data)}b: {txt_preview!r}")

        if self._dl_active:
            self._handle_download(data)
            return

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

            print(f"[BLE{self.device_id} RX] {line}")

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
                            print(f"[DL{self.device_id}] Header: {self._dl_expected} pts")
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

        self._dl_binbuf.extend(data)

        if b"END" in self._dl_binbuf:
            end_pos = self._dl_binbuf.find(b"END")
            self._dl_binbuf = self._dl_binbuf[:end_pos]
            self._dl_end = True

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
        print(f"[DL{self.device_id}] {msg}")
        self.download_finished.emit(result)

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

                t = 0
                while not self._dl_header_received and not self._dl_end and t < 40:
                    await asyncio.sleep(0.5)
                    t += 1

                if not self._dl_header_received:
                    self._dl_active = False
                    self.error_signal.emit("No header from device")
                    self.download_finished.emit([])
                    return

                print(f"[DL{self.device_id}] Sending RDY")
                await self._ble_write(client, b'RDY')

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

                if self._dl_active:
                    self._finalize_download()

            elif cmd == "SLEEP":
                self._text_accum = ""
                await self._ble_write(client, b'Z')
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
#  USB WORKER
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
        self.setFixedSize(120, 32)
        self.pct = -1; self.volt = 0.0

    def set_level(self, pct, volt=0.0):
        self.pct = pct; self.volt = volt; self.update()

    def paintEvent(self, event):
        from PyQt5.QtGui import QPainter, QBrush, QPen
        p = QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        bx, by, bw, bh = 4, 6, w-20, h-12

        p.setPen(QPen(QColor('#555'), 2)); p.setBrush(QBrush(QColor('#21262d')))
        p.drawRoundedRect(bx, by, bw, bh, 3, 3)
        p.setBrush(QBrush(QColor('#555'))); p.setPen(Qt.NoPen)
        p.drawRect(bx+bw, by+bh//4, 5, bh//2)

        if self.pct >= 0:
            fw = max(0, int((bw-4)*self.pct/100))
            clr = '#4CAF50' if self.pct > 50 else '#FF9800' if self.pct > 20 else '#f44336'
            p.setBrush(QBrush(QColor(clr))); p.drawRoundedRect(bx+2, by+2, fw, bh-4, 2, 2)
            p.setPen(QPen(QColor('#fff'))); p.setFont(QFont('Arial', 8, QFont.Bold))
            txt = f"{self.pct}%"
            p.drawText(bx, by, bw, bh, Qt.AlignCenter, txt)
        else:
            p.setPen(QPen(QColor('#666'))); p.setFont(QFont('Arial', 8))
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
        glDisable(GL_LIGHTING); glColor3f(0.2,0.25,0.3); glBegin(GL_LINES)
        for i in range(-5,6):
            glVertex3f(i,-5,0); glVertex3f(i,5,0); glVertex3f(-5,i,0); glVertex3f(5,i,0)
        glEnd(); glEnable(GL_LIGHTING)
        glRotatef(self.yaw,0,0,1); glRotatef(self.pitch,1,0,0); glRotatef(self.roll,0,1,0)
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
#  DEVICE PANEL - Individual device control widget
# ═══════════════════════════════════════════════════

class DevicePanel(QFrame):
    """Compact panel for controlling a single BLE device"""
    
    connection_changed = pyqtSignal(int, bool)
    status_updated = pyqtSignal(int, dict)
    data_downloaded = pyqtSignal(int)  # device_id when download completes
    
    def __init__(self, device_id, title="Device"):
        super().__init__()
        self.setObjectName("card")
        self.device_id = device_id
        self.title = title
        self.ble_worker = None
        self.ble_connected = False
        self.ball_status = {}
        self.ble_sample_rate = 100
        self.ble_raw_data = []
        
        self._init_ui()
        
        self.status_poll_timer = QTimer()
        self.status_poll_timer.timeout.connect(self._poll_status)
        
    def _init_ui(self):
        lo = QVBoxLayout(self)
        lo.setContentsMargins(12, 8, 12, 8)
        lo.setSpacing(6)
        
        # Title row
        title_row = QHBoxLayout()
        self.ind = StatusIndicator()
        title_row.addWidget(self.ind)
        title_lbl = QLabel(self.title)
        title_lbl.setStyleSheet("color:#58a6ff;font-weight:bold;font-size:11pt;")
        title_row.addWidget(title_lbl)
        title_row.addStretch()
        self.bat_widget = BatteryWidget()
        title_row.addWidget(self.bat_widget)
        lo.addLayout(title_row)
        
        # Connection row
        conn_row = QHBoxLayout()
        self.combo = QComboBox()
        self.combo.setMinimumWidth(160)
        conn_row.addWidget(self.combo)
        
        self.btn_conn = QPushButton("Connect")
        self.btn_conn.setFixedWidth(90)
        self.btn_conn.clicked.connect(self._toggle_conn)
        conn_row.addWidget(self.btn_conn)
        
        self.btn_status = QPushButton("📊")
        self.btn_status.setFixedWidth(36)
        self.btn_status.setToolTip("Request Status")
        self.btn_status.clicked.connect(self._req_status)
        self.btn_status.setEnabled(False)
        conn_row.addWidget(self.btn_status)
        lo.addLayout(conn_row)
        
        # Status label
        self.lbl_status = QLabel("Disconnected")
        self.lbl_status.setStyleSheet("color:#8b949e;font-size:9pt;")
        lo.addWidget(self.lbl_status)
        
        # Progress bar
        self.prog = QProgressBar()
        self.prog.setRange(0, 100)
        self.prog.setValue(0)
        self.prog.setTextVisible(False)
        self.prog.setFixedHeight(4)
        lo.addWidget(self.prog)
        
    def set_devices(self, devices):
        self.combo.clear()
        for d in devices:
            bat = f" {d['battery']}%" if d['battery'] >= 0 else ""
            self.combo.addItem(
                f"{d['name']} ({d['address'][-5:]}){bat}",
                d['address']
            )
            
    def _toggle_conn(self):
        if self.ble_connected:
            if self.ble_worker:
                self.ble_worker.send_command("DISCONNECT")
        else:
            idx = self.combo.currentIndex()
            if idx < 0:
                return
            addr = self.combo.itemData(idx)
            if not addr:
                return
                
            self.ble_worker = BLEControlWorker(addr, self.device_id)
            self.ble_worker.connected_changed.connect(self._on_conn_changed)
            self.ble_worker.status_received.connect(self._on_status)
            self.ble_worker.recording_status.connect(self._on_rec)
            self.ble_worker.download_progress.connect(self._on_dl_prog)
            self.ble_worker.download_finished.connect(self._on_dl_done)
            self.ble_worker.message_received.connect(self._on_msg)
            self.ble_worker.error_signal.connect(self._on_err)
            self.ble_worker.start()
            self.btn_conn.setEnabled(False)
            self.lbl_status.setText("Connecting...")
            self.ind.set_status('warning')
            
    def _on_conn_changed(self, connected):
        self.ble_connected = connected
        self.btn_conn.setEnabled(True)
        
        if connected:
            self.btn_conn.setText("Disconnect")
            self.btn_conn.setObjectName("stopBtn")
            self.btn_conn.style().polish(self.btn_conn)
            self.ind.set_status('ok')
            self.lbl_status.setText("Connected")
            self.btn_status.setEnabled(True)
            QTimer.singleShot(500, self._req_status)
            self.status_poll_timer.start(3000)
        else:
            self.btn_conn.setText("Connect")
            self.btn_conn.setObjectName("")
            self.btn_conn.style().polish(self.btn_conn)
            self.ind.set_status('off')
            self.lbl_status.setText("Disconnected")
            self.btn_status.setEnabled(False)
            self.status_poll_timer.stop()
            self.ble_worker = None
            
        self.connection_changed.emit(self.device_id, connected)
        
    def _poll_status(self):
        if self.ble_worker and self.ble_connected:
            self.ble_worker.send_command("STATUS")
            
    def _req_status(self):
        if self.ble_worker and self.ble_connected:
            self.ble_worker.send_command("STATUS")
            
    def _on_status(self, st):
        self.ball_status = st
        state = st.get('state', '?')
        bp = st.get('battery_pct', -1)
        bv = st.get('battery_v', 0.0)
        samples = st.get('samples', 0)
        
        self.bat_widget.set_level(bp, bv)
        self.lbl_status.setText(f"{state} | {samples} samples | {bv:.2f}V")
        self.ind.set_status('ok')
        
        self.status_updated.emit(self.device_id, st)
        
    def _on_rec(self, status, val, hz):
        if status == "START":
            if hz > 0:
                self.ble_sample_rate = hz
            self.lbl_status.setText(f"⏺ Recording {val}s @ {self.ble_sample_rate}Hz...")
            self.ind.set_status('active')
        elif status == "DONE":
            self.lbl_status.setText(f"✓ Done: {val} samples")
            self.ind.set_status('ok')
            self.status_poll_timer.start(3000)
            QTimer.singleShot(300, self._req_status)
            
    def _on_dl_prog(self, cur, total):
        self.prog.setMaximum(max(total, 1))
        self.prog.setValue(cur)
        pct = int(100 * cur / max(total, 1))
        self.lbl_status.setText(f"Downloading... {pct}%")
        
    def _on_dl_done(self, data):
        self.status_poll_timer.start(3000)
        if not data:
            self.lbl_status.setText("No data")
            self.ind.set_status('warning')
            self.ble_raw_data = []
            self.data_downloaded.emit(self.device_id)
            return
            
        dt = 1.0 / self.ble_sample_rate
        
        t_arr, ax_arr, ay_arr, az_arr = [], [], [], []
        gx_arr, gy_arr, gz_arr = [], [], []
        
        for i, row in enumerate(data):
            if len(row) < 6:
                continue
            t = i * dt
            t_arr.append(t)
            ax_arr.append(row[0] * ACCEL_SCALE)
            ay_arr.append(row[1] * ACCEL_SCALE)
            az_arr.append(row[2] * ACCEL_SCALE)
            gx_arr.append(row[3] * GYRO_SCALE)
            gy_arr.append(row[4] * GYRO_SCALE)
            gz_arr.append(row[5] * GYRO_SCALE)
            
        if ax_arr:
            self.ble_raw_data = list(zip(t_arr, ax_arr, ay_arr, az_arr, gx_arr, gy_arr, gz_arr))
            self.lbl_status.setText(f"✓ {len(ax_arr)} pts, {t_arr[-1]:.1f}s")
            self.ind.set_status('ok')
            self.prog.setValue(self.prog.maximum())
            
        self.data_downloaded.emit(self.device_id)
            
    def _on_msg(self, msg):
        self.lbl_status.setText(msg)
        print(f"[BLE{self.device_id} MSG] {msg}")
        
    def _on_err(self, err):
        self.lbl_status.setText(f"⚠ {err}")
        self.ind.set_status('error')
        print(f"[BLE{self.device_id} ERR] {err}")
        
    def send_command(self, cmd):
        if self.ble_worker and self.ble_connected:
            if cmd.startswith("RECORD"):
                self.status_poll_timer.stop()
            elif cmd == "DOWNLOAD":
                self.status_poll_timer.stop()
                self.prog.setValue(0)
                self.ble_raw_data = []
            elif cmd == "SLEEP":
                self.status_poll_timer.stop()
            self.ble_worker.send_command(cmd)
            return True
        return False
        
    def is_connected(self):
        return self.ble_connected
        
    def has_data(self):
        return self.ball_status.get('samples', 0) > 0
        
    def get_data(self):
        return self.ble_raw_data, self.ble_sample_rate


# ═══════════════════════════════════════════════════
#  MAIN WINDOW
# ═══════════════════════════════════════════════════

class BallStudio(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("⚽ XIAO Ball Studio v3.1 (Dual Device)")
        self.resize(1400, 850)
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

        self._scan_devs = []

        self._init_ui()

        self.stats_timer = QTimer()
        self.stats_timer.timeout.connect(self._update_stats)
        self.stats_timer.start(500)

    def _apply_style(self):
        self.setStyleSheet("""
            QMainWindow{background:qlineargradient(x1:0,y1:0,x2:0,y2:1,stop:0 #0d1117,stop:1 #161b22);}
            QWidget{color:#c9d1d9;font-family:'Segoe UI',Arial;font-size:10pt;}
            QTabWidget::pane{border:1px solid #30363d;background:#0d1117;border-radius:8px;}
            QTabBar::tab{background:#21262d;color:#8b949e;padding:12px 28px;margin-right:4px;
                         border-top-left-radius:8px;border-top-right-radius:8px;font-weight:500;}
            QTabBar::tab:selected{background:#0d1117;color:#f0f6fc;border-bottom:2px solid #238636;}
            QTabBar::tab:hover:!selected{background:#30363d;color:#c9d1d9;}
            QPushButton{background:#238636;border:none;padding:8px 16px;color:white;
                        font-weight:600;border-radius:6px;min-width:70px;}
            QPushButton:hover{background:#2ea043;} QPushButton:pressed{background:#196c2e;}
            QPushButton:disabled{background:#21262d;color:#484f58;}
            QPushButton#stopBtn{background:#da3633;} QPushButton#stopBtn:hover{background:#f85149;}
            QPushButton#blueBtn{background:#1f6feb;} QPushButton#blueBtn:hover{background:#388bfd;}
            QPushButton#sleepBtn{background:#6e40c9;} QPushButton#sleepBtn:hover{background:#8957e5;}
            QComboBox{background:#21262d;border:1px solid #30363d;border-radius:6px;padding:6px 10px;}
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

    def _init_ui(self):
        central = QWidget(); self.setCentralWidget(central)
        lo = QVBoxLayout(central); lo.setContentsMargins(12,12,12,12); lo.setSpacing(10)

        # Header
        hdr = QHBoxLayout()
        t = QLabel("⚽ XIAO Ball Studio v3.1"); t.setStyleSheet("font-size:16pt;font-weight:bold;color:#f0f6fc;")
        hdr.addWidget(t); hdr.addStretch()
        self.card_samples = DataCard("SAMPLES","0"); self.card_fps = DataCard("FPS","--"); self.card_max_g = DataCard("MAX G","--")
        for c in [self.card_samples, self.card_fps, self.card_max_g]:
            c.setObjectName("card"); c.setFixedWidth(100); hdr.addWidget(c)
        lo.addLayout(hdr)

        # Tabs
        self.tabs = QTabWidget()
        self.tabs.addTab(self._usb_tab(), "📡  USB LIVE")
        self.tabs.addTab(self._ball_tab(), "🔴  BALL CONTROL (×2)")
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

    # ─── BALL CONTROL TAB (DUAL DEVICE) ───

    def _ball_tab(self):
        w = QWidget()
        lo = QVBoxLayout(w)
        lo.setSpacing(8)

        # Top control bar: Scan + Group controls
        top_frame = QFrame()
        top_frame.setObjectName("card")
        top_lo = QHBoxLayout(top_frame)
        top_lo.setContentsMargins(12, 10, 12, 10)
        
        self.btn_scan = QPushButton("🔍 Scan")
        self.btn_scan.setFixedWidth(100)
        self.btn_scan.setObjectName("blueBtn")
        self.btn_scan.clicked.connect(self._ble_scan)
        top_lo.addWidget(self.btn_scan)
        
        self.scan_status = QLabel("Ready")
        self.scan_status.setStyleSheet("color:#8b949e;")
        top_lo.addWidget(self.scan_status)
        
        top_lo.addSpacing(20)
        top_lo.addWidget(QLabel("Duration:"))
        self.spin_dur = QSpinBox()
        self.spin_dur.setRange(1, 10)
        self.spin_dur.setValue(5)
        self.spin_dur.setSuffix("s")
        self.spin_dur.setFixedWidth(70)
        top_lo.addWidget(self.spin_dur)

        self.btn_record_all = QPushButton("⏺ Record All")
        self.btn_record_all.setFixedWidth(110)
        self.btn_record_all.clicked.connect(self._ble_record_all)
        self.btn_record_all.setEnabled(False)
        top_lo.addWidget(self.btn_record_all)

        self.btn_dl_all = QPushButton("⬇ Download All")
        self.btn_dl_all.setFixedWidth(120)
        self.btn_dl_all.clicked.connect(self._ble_download_all)
        self.btn_dl_all.setEnabled(False)
        top_lo.addWidget(self.btn_dl_all)

        self.btn_save_all = QPushButton("💾 Save All")
        self.btn_save_all.setFixedWidth(100)
        self.btn_save_all.clicked.connect(self._ble_save_all)
        self.btn_save_all.setEnabled(False)
        top_lo.addWidget(self.btn_save_all)

        top_lo.addStretch()

        self.btn_sleep_all = QPushButton("💤 Sleep All")
        self.btn_sleep_all.setFixedWidth(100)
        self.btn_sleep_all.setObjectName("sleepBtn")
        self.btn_sleep_all.clicked.connect(self._ble_sleep_all)
        self.btn_sleep_all.setEnabled(False)
        top_lo.addWidget(self.btn_sleep_all)

        lo.addWidget(top_frame)

        # Device panels row
        panels_lo = QHBoxLayout()
        panels_lo.setSpacing(10)
        
        self.device_panels = []
        for i in range(2):
            panel = DevicePanel(i, f"Device {i+1}")
            panel.connection_changed.connect(self._on_device_conn_changed)
            panel.status_updated.connect(self._on_device_status)
            panel.data_downloaded.connect(self._on_device_download)
            self.device_panels.append(panel)
            panels_lo.addWidget(panel)
            
        lo.addLayout(panels_lo)

        # Main content: Left (Device 1 graphs) | Right (Device 2 graphs)
        main_splitter = QSplitter(Qt.Horizontal)
        
        # Device 1 graphs (left side)
        left_widget = QWidget()
        left_lo = QVBoxLayout(left_widget)
        left_lo.setContentsMargins(0, 0, 0, 0)
        left_lo.setSpacing(4)
        
        lbl1 = QLabel("Device 1")
        lbl1.setStyleSheet("color:#58a6ff;font-weight:bold;font-size:12pt;")
        lbl1.setAlignment(Qt.AlignCenter)
        left_lo.addWidget(lbl1)
        
        self.plot_acc_1 = pg.PlotWidget()
        self.plot_acc_1.setTitle("Acceleration (G)", color='#8b949e', size='10pt')
        self.plot_acc_1.setLabel('left', 'G')
        self.plot_acc_1.showGrid(x=True, y=True, alpha=0.15)
        self.plot_acc_1.addLegend(offset=(10, 5))
        self.curve_ax_1 = self.plot_acc_1.plot(pen=pg.mkPen('#f85149', width=2), name="X")
        self.curve_ay_1 = self.plot_acc_1.plot(pen=pg.mkPen('#3fb950', width=2), name="Y")
        self.curve_az_1 = self.plot_acc_1.plot(pen=pg.mkPen('#58a6ff', width=2), name="Z")
        left_lo.addWidget(self.plot_acc_1)
        
        self.plot_gyro_1 = pg.PlotWidget()
        self.plot_gyro_1.setTitle("Gyroscope (°/s)", color='#8b949e', size='10pt')
        self.plot_gyro_1.setLabel('left', '°/s')
        self.plot_gyro_1.setLabel('bottom', 'Time (s)')
        self.plot_gyro_1.showGrid(x=True, y=True, alpha=0.15)
        self.plot_gyro_1.addLegend(offset=(10, 5))
        self.curve_gx_1 = self.plot_gyro_1.plot(pen=pg.mkPen('#f85149', width=2), name="X")
        self.curve_gy_1 = self.plot_gyro_1.plot(pen=pg.mkPen('#3fb950', width=2), name="Y")
        self.curve_gz_1 = self.plot_gyro_1.plot(pen=pg.mkPen('#58a6ff', width=2), name="Z")
        self.plot_gyro_1.setXLink(self.plot_acc_1)
        left_lo.addWidget(self.plot_gyro_1)
        
        main_splitter.addWidget(left_widget)
        
        # Device 2 graphs (right side)
        right_widget = QWidget()
        right_lo = QVBoxLayout(right_widget)
        right_lo.setContentsMargins(0, 0, 0, 0)
        right_lo.setSpacing(4)
        
        lbl2 = QLabel("Device 2")
        lbl2.setStyleSheet("color:#f0883e;font-weight:bold;font-size:12pt;")
        lbl2.setAlignment(Qt.AlignCenter)
        right_lo.addWidget(lbl2)
        
        self.plot_acc_2 = pg.PlotWidget()
        self.plot_acc_2.setTitle("Acceleration (G)", color='#8b949e', size='10pt')
        self.plot_acc_2.setLabel('left', 'G')
        self.plot_acc_2.showGrid(x=True, y=True, alpha=0.15)
        self.plot_acc_2.addLegend(offset=(10, 5))
        self.curve_ax_2 = self.plot_acc_2.plot(pen=pg.mkPen('#f85149', width=2), name="X")
        self.curve_ay_2 = self.plot_acc_2.plot(pen=pg.mkPen('#3fb950', width=2), name="Y")
        self.curve_az_2 = self.plot_acc_2.plot(pen=pg.mkPen('#58a6ff', width=2), name="Z")
        right_lo.addWidget(self.plot_acc_2)
        
        self.plot_gyro_2 = pg.PlotWidget()
        self.plot_gyro_2.setTitle("Gyroscope (°/s)", color='#8b949e', size='10pt')
        self.plot_gyro_2.setLabel('left', '°/s')
        self.plot_gyro_2.setLabel('bottom', 'Time (s)')
        self.plot_gyro_2.showGrid(x=True, y=True, alpha=0.15)
        self.plot_gyro_2.addLegend(offset=(10, 5))
        self.curve_gx_2 = self.plot_gyro_2.plot(pen=pg.mkPen('#f85149', width=2), name="X")
        self.curve_gy_2 = self.plot_gyro_2.plot(pen=pg.mkPen('#3fb950', width=2), name="Y")
        self.curve_gz_2 = self.plot_gyro_2.plot(pen=pg.mkPen('#58a6ff', width=2), name="Z")
        self.plot_gyro_2.setXLink(self.plot_acc_2)
        right_lo.addWidget(self.plot_gyro_2)
        
        main_splitter.addWidget(right_widget)
        main_splitter.setSizes([500, 500])
        
        lo.addWidget(main_splitter)
        
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
            self.usb_thread.running = False
            self.usb_thread.quit()
            self.usb_thread.wait(1000)
            self.usb_thread = None
            self.usb_ind.set_status('off')
            self.usb_status.setText("Disconnected")
            self.btn_refresh.setText("↻ Connect")
            self.btn_usb.setEnabled(False)
        else:
            port = self.combo_ports.currentText()
            if not port:
                return
            self.usb_thread = USBWorker(port)
            self.usb_thread.data_received.connect(self._usb_data)
            self.usb_thread.connection_lost.connect(self._usb_lost)
            self.usb_thread.start()
            self.usb_ind.set_status('warning')
            self.usb_status.setText("Connecting...")
            self.btn_refresh.setText("Disconnect")
            self.calibrating = True
            self.calib_samples = []
            self.usb_status.setText(f"Calibrating... {self.calib_count}")

    def _usb_lost(self):
        self.usb_thread = None
        self.usb_ind.set_status('error')
        self.usb_status.setText("Connection lost")
        self.btn_refresh.setText("↻ Connect")
        self.usb_count.setText("0 samples")
        self.btn_usb_save.setEnabled(False)
        self.btn_usb.setEnabled(False)
        self.btn_usb.setText("▶ Start")
        self.btn_usb.setObjectName("")
        self.btn_usb.style().polish(self.btn_usb)

    def _usb_rec_toggle(self):
        if self.recording:
            self.recording = False
            self.btn_usb.setText("▶ Start")
            self.btn_usb.setObjectName("")
            self.btn_usb.style().polish(self.btn_usb)
            self.usb_status.setText(f"Stopped: {len(self.usb_raw_data)} samples")
            self.btn_usb_save.setEnabled(len(self.usb_raw_data) > 0)
        else:
            self.recording = True
            self.usb_raw_data = []
            self.sample_count = 0
            self.usb_record_start = time.time()
            self.btn_usb.setText("■ Stop")
            self.btn_usb.setObjectName("stopBtn")
            self.btn_usb.style().polish(self.btn_usb)
            self.usb_status.setText("Recording...")
            self.btn_usb_save.setEnabled(False)

    def _usb_data(self, data):
        ax_r, ay_r, az_r = data[0], data[1], data[2]
        gx_r = data[3] if len(data) > 3 else 0
        gy_r = data[4] if len(data) > 4 else 0
        gz_r = data[5] if len(data) > 5 else 0

        if self.calibrating:
            self.calib_samples.append([ax_r, ay_r, az_r, gx_r, gy_r, gz_r])
            rem = self.calib_count - len(self.calib_samples)
            self.usb_status.setText(f"Calibrating... {rem}")
            if len(self.calib_samples) >= self.calib_count:
                self._finish_calib()
            return
        if not self.recording:
            return

        gx = gx_r - self.gyro_bias[0]
        gy = gy_r - self.gyro_bias[1]
        gz = gz_r - self.gyro_bias[2]
        ax = ax_r * self.accel_scale
        ay = ay_r * self.accel_scale
        az = az_r * self.accel_scale
        ct = time.time()
        dt = ct - self.last_time
        self.last_time = ct

        ra = np.arctan2(ay, az) * 57.2958
        pa = np.arctan2(-ax, np.sqrt(ay*ay + az*az)) * 57.2958
        if not self.live_data['roll']:
            self.roll_fused = ra
            self.pitch_fused = pa
            self.yaw_fused = 0.0
        else:
            if abs(ra - self.live_data['roll'][-1]) > 180:
                self.roll_fused = ra
            else:
                self.roll_fused = 0.96 * (self.roll_fused + gx*dt) + 0.04*ra
                while self.roll_fused > 180:
                    self.roll_fused -= 360
                while self.roll_fused < -180:
                    self.roll_fused += 360
            if abs(pa - self.live_data['pitch'][-1]) > 180:
                self.pitch_fused = pa
            else:
                self.pitch_fused = 0.96 * (self.pitch_fused + gy*dt) + 0.04*pa
                while self.pitch_fused > 180:
                    self.pitch_fused -= 360
                while self.pitch_fused < -180:
                    self.pitch_fused += 360
            self.yaw_fused += gz * dt
            while self.yaw_fused > 180:
                self.yaw_fused -= 360
            while self.yaw_fused < -180:
                self.yaw_fused += 360

        tg = np.sqrt(ax*ax + ay*ay + az*az)
        self.sample_count += 1
        ts = int((ct - self.usb_record_start) * 1000)
        self.usb_raw_data.append([ts, ax, ay, az, gx, gy, gz, self.roll_fused, self.pitch_fused, self.yaw_fused])
        for k, v in zip(['ax','ay','az','roll','pitch','filtered_roll','total_g'],
                        [ax, ay, az, ra, pa, self.roll_fused, tg]):
            self.live_data[k].append(v)
        self.curve_ax.setData(list(self.live_data['ax']))
        self.curve_ay.setData(list(self.live_data['ay']))
        self.curve_az.setData(list(self.live_data['az']))
        self.airplane.set_orientation(self.roll_fused, -self.pitch_fused, self.yaw_fused)
        self.orient_lbl.setText(f"Roll:{self.roll_fused:.1f}° Pitch:{self.pitch_fused:.1f}° Yaw:{self.yaw_fused:.1f}°")
        if self.sample_count % 50 == 0:
            self.usb_count.setText(f"{self.sample_count} samples")
        self.usb_ind.set_status('active')

    def _finish_calib(self):
        self.calibrating = False
        s = np.array(self.calib_samples)
        self.gyro_bias = [np.mean(s[:,3]), np.mean(s[:,4]), np.mean(s[:,5])]
        gm = np.sqrt(np.mean(s[:,0])**2 + np.mean(s[:,1])**2 + np.mean(s[:,2])**2)
        self.accel_scale = 1.0 / gm if gm > 0.1 else 1.0
        ac = np.mean(s[:,0]) * self.accel_scale
        bc = np.mean(s[:,1]) * self.accel_scale
        cc = np.mean(s[:,2]) * self.accel_scale
        ir = np.arctan2(bc, cc) * 57.2958
        ip = np.arctan2(-ac, np.sqrt(bc**2 + cc**2)) * 57.2958
        self.roll_fused = self.pitch_fused = self.yaw_fused = 0.0
        self.last_time = time.time()
        self.is_calibrated = True
        self.btn_usb.setEnabled(True)
        self.usb_ind.set_status('ok')
        self.usb_status.setText(f"Ready | Roll={ir:.1f}° Pitch={ip:.1f}°")

    def _update_stats(self):
        self.card_samples.set_value(str(self.sample_count))
        if self.usb_thread and self.live_data['total_g']:
            self.card_max_g.set_value(f"{max(self.live_data['total_g']):.1f}")
            self.card_fps.set_value("50")
            self.usb_ind.set_status('ok')

    def _usb_save(self):
        if not self.usb_raw_data:
            return
        fn, _ = QFileDialog.getSaveFileName(self, "Save USB Data", f"usb_{datetime.now():%Y%m%d_%H%M%S}.csv", "CSV (*.csv)")
        if fn:
            try:
                arr = np.array(self.usb_raw_data)
                np.savetxt(fn, arr, delimiter=',', header='timestamp_ms,ax,ay,az,gx,gy,gz,roll,pitch,yaw',
                           comments='', fmt=['%d'] + ['%.6f']*6 + ['%.2f']*3)
                self.usb_status.setText(f"Saved: {len(self.usb_raw_data)} samples")
            except Exception as e:
                self.usb_status.setText(f"Error: {e}")

    # ═══════════════════════════════════════════════
    #  BLE HANDLERS (DUAL DEVICE)
    # ═══════════════════════════════════════════════

    def _ble_scan(self):
        self.btn_scan.setEnabled(False)
        self.scan_status.setText("Scanning...")
        self._scanner = BLEScanWorker(5.0)
        self._scanner.devices_found.connect(self._ble_scan_done)
        self._scanner.scan_error.connect(lambda e: (
            self.btn_scan.setEnabled(True),
            self.scan_status.setText(f"Error: {e}")
        ))
        self._scanner.start()

    def _ble_scan_done(self, devs):
        self.btn_scan.setEnabled(True)
        if not devs:
            self.scan_status.setText("No devices found")
            return
        self._scan_devs = devs
        self.scan_status.setText(f"Found {len(devs)} device(s)")
        
        for panel in self.device_panels:
            panel.set_devices(devs)

    def _on_device_conn_changed(self, device_id, connected):
        self._update_group_buttons()

    def _on_device_status(self, device_id, status):
        self._update_group_buttons()

    def _on_device_download(self, device_id):
        """Called when a device finishes downloading"""
        self._update_plots()
        self._update_group_buttons()

    def _update_group_buttons(self):
        active = [p for p in self.device_panels if p.is_connected()]
        has_data = any(p.has_data() for p in active)
        has_downloaded = any(len(p.ble_raw_data) > 0 for p in self.device_panels)
        
        self.btn_record_all.setEnabled(len(active) > 0)
        self.btn_dl_all.setEnabled(has_data)
        self.btn_save_all.setEnabled(has_downloaded)
        self.btn_sleep_all.setEnabled(len(active) > 0)

    def _update_plots(self):
        """Update all plots with current data from device panels"""
        # Device 1
        panel1 = self.device_panels[0]
        if panel1.ble_raw_data:
            data = panel1.ble_raw_data
            t = [r[0] for r in data]
            ax = [r[1] for r in data]
            ay = [r[2] for r in data]
            az = [r[3] for r in data]
            gx = [r[4] for r in data]
            gy = [r[5] for r in data]
            gz = [r[6] for r in data]
            
            self.curve_ax_1.setData(t, ax)
            self.curve_ay_1.setData(t, ay)
            self.curve_az_1.setData(t, az)
            self.curve_gx_1.setData(t, gx)
            self.curve_gy_1.setData(t, gy)
            self.curve_gz_1.setData(t, gz)
        else:
            self.curve_ax_1.setData([])
            self.curve_ay_1.setData([])
            self.curve_az_1.setData([])
            self.curve_gx_1.setData([])
            self.curve_gy_1.setData([])
            self.curve_gz_1.setData([])
            
        # Device 2
        panel2 = self.device_panels[1]
        if panel2.ble_raw_data:
            data = panel2.ble_raw_data
            t = [r[0] for r in data]
            ax = [r[1] for r in data]
            ay = [r[2] for r in data]
            az = [r[3] for r in data]
            gx = [r[4] for r in data]
            gy = [r[5] for r in data]
            gz = [r[6] for r in data]
            
            self.curve_ax_2.setData(t, ax)
            self.curve_ay_2.setData(t, ay)
            self.curve_az_2.setData(t, az)
            self.curve_gx_2.setData(t, gx)
            self.curve_gy_2.setData(t, gy)
            self.curve_gz_2.setData(t, gz)
        else:
            self.curve_ax_2.setData([])
            self.curve_ay_2.setData([])
            self.curve_az_2.setData([])
            self.curve_gx_2.setData([])
            self.curve_gy_2.setData([])
            self.curve_gz_2.setData([])

    def _ble_record_all(self):
        dur = self.spin_dur.value()
        active = [p for p in self.device_panels if p.is_connected()]
        for panel in active:
            panel.send_command(f"RECORD:{dur}")
        self.scan_status.setText(f"Recording {dur}s on {len(active)} device(s)...")

    def _ble_download_all(self):
        active = [p for p in self.device_panels if p.is_connected() and p.has_data()]
        
        # Clear plots
        self._update_plots()
        
        for panel in active:
            panel.send_command("DOWNLOAD")
        self.scan_status.setText(f"Downloading from {len(active)} device(s)...")

    def _ble_save_all(self):
        # Get data from both devices
        data1 = self.device_panels[0].ble_raw_data
        data2 = self.device_panels[1].ble_raw_data
        
        if not data1 and not data2:
            self.scan_status.setText("No data to save")
            return
        
        fn, _ = QFileDialog.getSaveFileName(
            self,
            "Save Combined Data",
            f"balls_{datetime.now():%Y%m%d_%H%M%S}.csv",
            "CSV (*.csv)"
        )
        if not fn:
            return
            
        # Determine max length and align by time
        len1 = len(data1) if data1 else 0
        len2 = len(data2) if data2 else 0
        max_len = max(len1, len2)
        
        with open(fn, 'w') as f:
            # Header: time + 6 cols device1 + 6 cols device2 = 13 columns
            f.write("time_s,ax1_g,ay1_g,az1_g,gx1_dps,gy1_dps,gz1_dps,ax2_g,ay2_g,az2_g,gx2_dps,gy2_dps,gz2_dps\n")
            
            for i in range(max_len):
                # Get time from whichever device has data at this index
                if i < len1:
                    t = data1[i][0]
                elif i < len2:
                    t = data2[i][0]
                else:
                    t = 0.0
                
                # Device 1 data (or zeros if no data)
                if i < len1:
                    d1 = data1[i]
                    ax1, ay1, az1 = d1[1], d1[2], d1[3]
                    gx1, gy1, gz1 = d1[4], d1[5], d1[6]
                else:
                    ax1 = ay1 = az1 = gx1 = gy1 = gz1 = 0.0
                
                # Device 2 data (or zeros if no data)
                if i < len2:
                    d2 = data2[i]
                    ax2, ay2, az2 = d2[1], d2[2], d2[3]
                    gx2, gy2, gz2 = d2[4], d2[5], d2[6]
                else:
                    ax2 = ay2 = az2 = gx2 = gy2 = gz2 = 0.0
                
                f.write(f"{t:.3f},{ax1:.6f},{ay1:.6f},{az1:.6f},{gx1:.2f},{gy1:.2f},{gz1:.2f},"
                        f"{ax2:.6f},{ay2:.6f},{az2:.6f},{gx2:.2f},{gy2:.2f},{gz2:.2f}\n")
        
        self.scan_status.setText(f"Saved: {max_len} rows to {fn.split('/')[-1]}")

    def _ble_sleep_all(self):
        active = [p for p in self.device_panels if p.is_connected()]
        for panel in active:
            panel.send_command("SLEEP")
        self.scan_status.setText(f"Sleep sent to {len(active)} device(s)")


# ═══════════════════════════════════════════════════
if __name__ == "__main__":
    app = QApplication.instance() or QApplication(sys.argv)
    app.setStyle('Fusion')
    window = BallStudio()
    window.show()
    sys.exit(app.exec_())
