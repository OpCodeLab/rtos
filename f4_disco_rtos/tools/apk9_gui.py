"""APK9 GUI: simple safety lab (NORMAL / SAFE) - water heater plant + 4 faults.

    pip install pyqt5 pyserial
    python apk9_gui.py

The GUI is the physical world: it computes the water temperature from the
heater power and the safety relay reported by the STM32 and sends the sensor
value every 100 ms. Each fault button creates one of the 4 errors.
Choose "Simulator (no board)" to run without hardware.
"""
import html
import queue
import re
import sys
import threading
import time
from collections import deque

import serial
import serial.tools.list_ports
from PyQt5.QtCore import QPointF, QRectF, Qt, QThread, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QGridLayout, QGroupBox, QHBoxLayout, QLabel,
    QMainWindow, QPushButton, QTextEdit, QVBoxLayout, QWidget,
)

SOF, RSP_FLAG, FRAME_TIMEOUT_S = 0xAA, 0x80, 0.05
CMD_PING, CMD_SENSOR, CMD_RESET, CMD_INJECT, CMD_GET_MODE = 0x01, 0x60, 0x62, 0x63, 0x64
EVT_MODE, EVT_ERROR, EVT_HEATER, EVT_RELAY, EVT_BOOT = 0x70, 0x71, 0x72, 0x73, 0x74
NORMAL, SAFE = 0, 1
MODES = ["NORMAL", "SAFE"]
MODE_COLOURS = ["#2E7D32", "#C62828"]
ERRORS = {0x01: "ERR_SENSOR (value invalid)", 0x02: "ERR_TIMEOUT (no sample)",
          0x04: "ERR_OVERTEMP (T > 90 C)", 0x08: "ERR_ALIVE (ControlTask stopped)"}
SET_POINT = 40
HAZARD_C = 95
ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")
SIM_PORT = "__sim__"


# ============================================================ board twin ===
class SimBoard:
    """Python twin of apk9.c: same 3 tasks, same 4 errors, same 2 modes."""

    def __init__(self):
        self.out = bytearray()
        self.lock = threading.Lock()
        self.frames = queue.Queue(8)
        self.rx, self.cmd, self.last = 0, 0, 0.0
        self.running = True
        self._boot()
        for fn in (self._link_layer, self._control_task, self._safety_task):
            threading.Thread(target=fn, daemon=True).start()

    def _boot(self):
        self.mode = NORMAL
        self.sensor, self.sensor_t, self.sensor_seen = 0xFF, 0.0, False
        self.reset_req, self.deadlock = False, False
        self.power_req, self.alive = 0, 0

    # pyserial-like API
    def write(self, data):
        for b in bytes(data):
            now = time.perf_counter()
            if self.rx and now - self.last > FRAME_TIMEOUT_S:
                self.rx = 0
            self.last = now
            if self.rx == 0:
                self.rx = 1 if b == SOF else 0
            elif self.rx == 1:
                self.cmd, self.rx = b, 2
            else:
                try:
                    self.frames.put_nowait((self.cmd, b))
                except queue.Full:
                    pass
                self.rx = 0

    def read(self, n):
        time.sleep(0.005)
        with self.lock:
            chunk = bytes(self.out[:n])
            del self.out[:n]
        return chunk

    def close(self):
        self.running = False

    def _send(self, cmd, data, log=None):
        with self.lock:
            if log:
                self.out += f"\x1b[36m{log}\x1b[0m\r\n".encode()
            self.out += bytes([SOF, cmd & 0xFF, data & 0xFF])

    def _link_layer(self):
        while self.running:
            cmd, data = self.frames.get()
            if cmd == CMD_SENSOR:
                self.sensor, self.sensor_t, self.sensor_seen = data, time.monotonic(), True
            elif cmd == CMD_PING:
                self._send(cmd | RSP_FLAG, data)
            elif cmd == CMD_RESET:
                if self.mode == SAFE:
                    self.reset_req = True
                else:
                    self._send(cmd | RSP_FLAG, 0xE6)
            elif cmd == CMD_INJECT:
                self.deadlock = data == 1
                self._send(cmd | RSP_FLAG, 0)
            elif cmd == CMD_GET_MODE:
                self._send(cmd | RSP_FLAG, self.mode)
            else:
                self._send(cmd | RSP_FLAG, 0xE1)

    def _control_task(self):
        power = 0
        while self.running:
            time.sleep(0.1)
            if self.deadlock:
                continue                           # blocked forever: no heartbeat
            self.alive += 1
            t = self.sensor
            if t > 120:
                power = 0
            elif t + 2 <= SET_POINT:
                power = 100
            elif t >= SET_POINT:
                power = 0
            self.power_req = power

    def _safety_task(self):
        self._send(EVT_BOOT, 0, " RTOS LAB APK9 (simple safety: NORMAL / SAFE) STARTING")
        self._send(EVT_MODE, NORMAL)
        last_alive, alive_t = self.alive, time.monotonic()
        heater, relay = None, None
        while self.running:
            time.sleep(0.05)
            now = time.monotonic()
            err = 0
            if self.sensor_seen:
                if self.sensor > 120:
                    err |= 0x01
                elif self.sensor > 90:
                    err |= 0x04
                if now - self.sensor_t > 0.5:
                    err |= 0x02
            if self.alive != last_alive:
                last_alive, alive_t = self.alive, now
            elif now - alive_t > 0.3:
                err |= 0x08
            if self.mode == NORMAL and not err:
                out = (self.power_req, 1)
            else:
                out = (0, 0)
                if self.mode == NORMAL:
                    self.mode = SAFE
                    self._send(EVT_HEATER, 0)
                    self._send(EVT_RELAY, 0)
                    heater, relay = 0, 0
                    self._send(EVT_MODE, SAFE)
                    self._send(EVT_ERROR, err, f"SAFE: errors 0x{err:02X}")
                elif self.reset_req:
                    self.reset_req = False
                    fresh = self.sensor_seen and now - self.sensor_t < 0.5
                    if fresh and self.sensor < 60:
                        self._send(CMD_RESET | RSP_FLAG, 0, "Reset accepted: restart")
                        time.sleep(0.3)                       # NVIC_SystemReset()
                        self._boot()
                        self._send(EVT_BOOT, 0, " RTOS LAB APK9 (simple safety: NORMAL / SAFE) STARTING")
                        self._send(EVT_MODE, NORMAL)
                        last_alive, alive_t, heater, relay = self.alive, time.monotonic(), None, None
                        continue
                    self._send(CMD_RESET | RSP_FLAG, 0xE6, "Reset refused: sensor not valid or T >= 60 C")
            if out[0] != heater:
                heater = out[0]
                self._send(EVT_HEATER, heater)
            if out[1] != relay:
                relay = out[1]
                self._send(EVT_RELAY, relay)


# ============================================================ serial I/O ===
class SerialReader(QThread):
    frame_received = pyqtSignal(int, int)
    text_received = pyqtSignal(str)
    error = pyqtSignal(str)

    def __init__(self, port):
        super().__init__()
        self.port, self.running = port, True

    def run(self):
        state, cmd, last, line = 0, 0, 0.0, bytearray()
        while self.running:
            try:
                chunk = self.port.read(64)
            except Exception as exc:
                self.error.emit(str(exc))
                return
            for b in chunk:
                now = time.monotonic()
                if state and now - last > FRAME_TIMEOUT_S:
                    state = 0
                last = now
                if state == 0:
                    if b == SOF:
                        state = 1
                    elif b == 0x0A:
                        t = ANSI_RE.sub("", line.decode(errors="replace")).strip()
                        if t:
                            self.text_received.emit(t)
                        line.clear()
                    elif b != 0x0D:
                        line.append(b)
                elif state == 1:
                    cmd, state = b, 2
                else:
                    self.frame_received.emit(cmd, b)
                    state = 0

    def stop(self):
        self.running = False
        self.wait(500)


# ================================================================= chart ===
class Chart(QWidget):
    """Water temperature over the last 60 s, with the mode under it."""

    def __init__(self):
        super().__init__()
        self.setMinimumHeight(260)
        self.data = deque(maxlen=600)              # (temp, mode, effective power)

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        l, r, t, b = 44, 10, 10, 34
        pw, ph = w - l - r, h - t - b
        p.fillRect(self.rect(), QColor("#FFFFFF"))
        y = lambda v: t + ph * (1 - min(max(v, 0), 120) / 120)
        p.setFont(QFont("Arial", 8))
        for v in range(0, 121, 20):
            p.setPen(QColor("#EEEEEE"))
            p.drawLine(l, int(y(v)), l + pw, int(y(v)))
            p.setPen(QColor("#777777"))
            p.drawText(QRectF(0, y(v) - 8, l - 6, 16), Qt.AlignRight | Qt.AlignVCenter, f"{v}")
        for v, txt, col, style in ((SET_POINT, f"set point {SET_POINT} C", "#2E7D32", Qt.DashLine),
                                   (90, "detection 90 C", "#C62828", Qt.DashLine),
                                   (HAZARD_C, f"hazard {HAZARD_C} C", "#880E4F", Qt.DotLine)):
            p.setPen(QPen(QColor(col), 1, style))
            p.drawLine(l, int(y(v)), l + pw, int(y(v)))
            p.drawText(l + (4 if v != HAZARD_C else 160), int(y(v)) - 3, txt)
        n = len(self.data)
        if n:
            dx = pw / 599
            x0 = l + pw - (n - 1) * dx
            for i, (_, m, pw_) in enumerate(self.data):
                c = QColor(MODE_COLOURS[m])
                c.setAlpha(170)
                p.fillRect(QRectF(x0 + i * dx, t + ph + 4, dx + 1, 8), c)
                if pw_:
                    p.fillRect(QRectF(x0 + i * dx, t + ph + 14, dx + 1, 8), QColor(30, 136, 229, 180))
            p.setPen(QPen(QColor("#212121"), 2))
            p.drawPolyline(QPolygonF([QPointF(x0 + i * dx, y(v)) for i, (v, _, _) in enumerate(self.data)]))
        p.setPen(QColor("#777777"))
        p.drawText(QRectF(l, t + ph + 22, pw, 12), Qt.AlignLeft, "mode (green NORMAL / red SAFE)  /  heater ON (blue)   - last 60 s")


# ============================================================ main window ===
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("APK9 - Simple safety: NORMAL / SAFE")
        self.resize(1150, 780)
        self.port = None
        self.reader = None
        self.temp, self.heater, self.relay, self.mode = 20.0, 0, 1, NORMAL

        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)
        left = QVBoxLayout()
        panel = QWidget()
        panel.setLayout(left)
        panel.setMaximumWidth(300)
        root.addWidget(panel)

        box = QGroupBox("Connection")
        g = QGridLayout(box)
        self.port_combo = QComboBox()
        refresh = QPushButton("Refresh")
        refresh.clicked.connect(self.refresh_ports)
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        g.addWidget(self.port_combo, 0, 0, 1, 2)
        g.addWidget(refresh, 1, 0)
        g.addWidget(self.connect_btn, 1, 1)
        left.addWidget(box)

        box = QGroupBox("Faults: one button = one error")
        v = QVBoxLayout(box)
        self.f_wire = QCheckBox("Sensor wire cut  -> ERR_SENSOR")
        self.f_cable = QCheckBox("Cable lost  -> ERR_TIMEOUT")
        self.f_stuck = QCheckBox("Heater stuck ON  -> ERR_OVERTEMP")
        for c in (self.f_wire, self.f_cable, self.f_stuck):
            c.toggled.connect(lambda on, c=c: self.log(f"FAULT {'ON ' if on else 'OFF'}: {c.text()}", "#AD1457"))
            v.addWidget(c)
        dead = QPushButton("SW bug: deadlock  -> ERR_ALIVE")
        dead.clicked.connect(lambda: (self.log("FAULT: deadlock injected", "#AD1457"), self.send(CMD_INJECT, 1)))
        v.addWidget(dead)
        left.addWidget(box)

        box = QGroupBox("Leave SAFE")
        v = QVBoxLayout(box)
        reset = QPushButton("RESET request")
        reset.setMinimumHeight(40)
        reset.setStyleSheet("font-weight:bold;")
        reset.clicked.connect(lambda: self.send(CMD_RESET, 0))
        v.addWidget(reset)
        v.addWidget(QLabel("<small>Accepted only in SAFE, with a valid<br>sensor value below 60 C.<br>"
                           "Then the board restarts in NORMAL.</small>"))
        left.addWidget(box)
        left.addStretch()
        self.cmd_widgets = [dead, reset]

        right = QVBoxLayout()
        root.addLayout(right, 1)
        top = QHBoxLayout()
        self.mode_label = QLabel()
        self.mode_label.setAlignment(Qt.AlignCenter)
        self.mode_label.setMinimumHeight(80)
        top.addWidget(self.mode_label, 2)
        self.values = QLabel()
        self.values.setStyleSheet("font-size:14px; padding:6px;")
        top.addWidget(self.values, 1)
        right.addLayout(top)
        self.chart = Chart()
        right.addWidget(self.chart, 2)
        self.errors = QLabel("Errors: none")
        self.errors.setStyleSheet("font-size:14px; padding:4px;")
        right.addWidget(self.errors)
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setFont(QFont("Consolas", 9))
        right.addWidget(self.log_view, 1)

        self.plant = QTimer(self)
        self.plant.timeout.connect(self.plant_step)
        self.plant.start(100)
        self.refresh_ports()
        self._set_connected(False)
        self.render()

    # ------------------------------------------------------- connection
    def refresh_ports(self):
        self.port_combo.clear()
        self.port_combo.addItem("Simulator (no board)", SIM_PORT)
        for p in serial.tools.list_ports.comports():
            self.port_combo.addItem(f"{p.device} - {p.description}", p.device)
        if self.port_combo.count() > 1:
            self.port_combo.setCurrentIndex(1)

    def toggle_connection(self):
        if self.port:
            self.disconnect_port()
            return
        name = self.port_combo.currentData()
        try:
            self.port = SimBoard() if name == SIM_PORT else serial.Serial(name, 115200, timeout=0.05)
        except serial.SerialException as exc:
            self.log(f"Cannot open {name}: {exc}", "red")
            return
        self.reader = SerialReader(self.port)
        self.reader.frame_received.connect(self.on_frame)
        self.reader.text_received.connect(lambda t: self.log(f"    [board] {t}", "gray"))
        self.reader.error.connect(lambda m: (self.log(f"Serial error: {m}", "red"), self.disconnect_port()))
        self.reader.start()
        self.log(f"Connected to {self.port_combo.currentText()}", "gray")
        self._set_connected(True)
        self.send(CMD_GET_MODE, 0)

    def disconnect_port(self):
        if self.reader:
            self.reader.stop()
            self.reader = None
        if self.port:
            self.port.close()
            self.port = None
        self.log("Disconnected", "gray")
        self._set_connected(False)

    def _set_connected(self, on):
        self.connect_btn.setText("Disconnect" if on else "Connect")
        self.port_combo.setEnabled(not on)
        for w in self.cmd_widgets:
            w.setEnabled(on)

    def closeEvent(self, e):
        if self.port:
            self.disconnect_port()
        e.accept()

    # ------------------------------------------------------ plant model
    def plant_step(self):
        """Water: heating 0.5 C / 100 ms at 100 %, losses 0.5 % of (T - 20)."""
        power = (100 if self.f_stuck.isChecked() else self.heater) if self.relay else 0
        self.temp += 0.5 * power / 100 - 0.005 * (self.temp - 20)
        self.chart.data.append((self.temp, self.mode, power))
        self.chart.update()
        self.render(power)
        if not self.port or self.f_cable.isChecked():
            return
        value = 0xFF if self.f_wire.isChecked() else int(round(self.temp))
        self.port.write(bytes([SOF, CMD_SENSOR, value]))

    # -------------------------------------------------------------- TX/RX
    def send(self, cmd, data):
        if self.port:
            self.port.write(bytes([SOF, cmd, data]))
            self.log(f"TX  AA {cmd:02X} {data:02X}", "#1565C0")

    def on_frame(self, cmd, data):
        raw = f"AA {cmd:02X} {data:02X}"
        if cmd & RSP_FLAG:
            req = cmd & 0x7F
            if req == CMD_GET_MODE:
                self.mode = SAFE if data else NORMAL
                self.log(f"RX  {raw}   GET_MODE -> {MODES[self.mode]}", "#2E7D32")
            elif req == CMD_RESET:
                self.log(f"RX  {raw}   RESET -> {'accepted' if data == 0 else 'REFUSED (T >= 60 C or no valid sensor)'}",
                         "#2E7D32" if data == 0 else "red")
            else:
                self.log(f"RX  {raw}   answer 0x{data:02X}", "#2E7D32")
        elif cmd == EVT_MODE:
            self.mode = SAFE if data else NORMAL
            self.log(f"RX  {raw}   EVT_MODE {MODES[self.mode]}", MODE_COLOURS[self.mode])
            if self.mode == NORMAL:
                self.errors.setText("Errors: none")
        elif cmd == EVT_ERROR:
            names = [n for bit, n in ERRORS.items() if data & bit]
            self.errors.setText("Errors: " + ", ".join(names))
            self.errors.setStyleSheet("font-size:14px; padding:4px; color:#C62828; font-weight:bold;")
            self.log(f"RX  {raw}   EVT_ERROR {', '.join(names)}", "red")
        elif cmd == EVT_HEATER:
            self.heater = data
            self.log(f"RX  {raw}   EVT_HEATER {data} %", "#6A1B9A")
        elif cmd == EVT_RELAY:
            self.relay = data
            self.log(f"RX  {raw}   EVT_RELAY {'CLOSED' if data else 'OPEN'}", "#6A1B9A")
        elif cmd == EVT_BOOT:
            self.heater, self.relay = 0, 1
            self.errors.setStyleSheet("font-size:14px; padding:4px;")
            self.log(f"RX  {raw}   EVT_BOOT (board restarted)", "#1565C0")
        self.render()

    def render(self, power=None):
        m = self.mode
        sub = "heater follows the ControlTask" if m == NORMAL else "heater OFF, relay OPEN - waiting for RESET"
        self.mode_label.setText(f"<div style='font-size:28px; font-weight:bold'>{MODES[m]}</div>"
                                f"<div style='font-size:12px'>{sub}</div>")
        self.mode_label.setStyleSheet(f"background:{MODE_COLOURS[m]}; color:white; border-radius:8px;")
        eff = power if power is not None else self.heater
        hazard = eff > 0 and self.temp > HAZARD_C
        self.values.setText(
            f"Water: <b>{self.temp:5.1f} C</b> (set point {SET_POINT} C)<br>Heater command: <b>{self.heater} %</b><br>"
            f"Safety relay: <b>{'CLOSED' if self.relay else 'OPEN'}</b><br>Effective power: <b>{eff} %</b><br>"
            + (f"<span style='color:#C62828'><b>FAILURE: heating above {HAZARD_C} C!</b></span>" if hazard else
               "<span style='color:#2E7D32'>no failure</span>"))

    def log(self, text, colour="black"):
        self.log_view.append(f'<span style="color:{colour}; white-space:pre">{time.strftime("%H:%M:%S")}  {html.escape(text)}</span>')


def main():
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
