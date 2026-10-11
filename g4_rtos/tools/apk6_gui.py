"""APK6 GUI: washing machine front panel talking to the STM32 (or to a built-in simulator).

    pip install pyqt5 pyserial
    python apk6_gui.py

Pick "Simulator (no board)" in the port list to try it without hardware; the
"Press B1" button then plays the role of the board's user button (door).
"""
import html
import math
import re
import sys
import time

import serial
import serial.tools.list_ports
from PyQt5.QtCore import QRectF, Qt, QThread, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QPainter, QPen
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QFormLayout, QGridLayout, QGroupBox, QHBoxLayout,
    QLabel, QMainWindow, QProgressBar, QPushButton, QSplitter, QTextEdit, QVBoxLayout, QWidget,
)

from apk6_proto import (
    CMD_CFG_PROGRAM, CMD_CFG_SPIN, CMD_CFG_TEMP, CMD_GET_STATUS, CMD_NAMES, CMD_PING,
    CMD_START_CYCLE, EVT_DONE, EVT_DOOR, EVT_ERROR, EVT_NAMES, EVT_PROGRESS, EVT_STEP,
    FRAME_TIMEOUT_S, PROGRAMS, RSP_FLAG, SOF, SPINS, STATE_NAMES, STATUS_NAMES, STEPS,
    ST_OK, STATUS_ERROR_FLAG, IDLE_STATE, READY_STATE, RUN_STATE, TEMPS, SimBoard,
)

ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")
SIM_PORT = "__sim__"
# What the GUI shows (PAUSED = RUN_STATE + sensor ERROR_STATE, DONE = READY after a cycle)
V_IDLE, V_READY, V_RUN, V_PAUSED, V_DONE = range(5)


class SerialReader(QThread):
    """Same SOF / CMD / DATA state machine as the firmware."""

    frame_received = pyqtSignal(int, int)
    text_received = pyqtSignal(str)
    error = pyqtSignal(str)

    def __init__(self, port):
        super().__init__()
        self.port = port
        self.running = True

    def run(self):
        state, cmd, last, line = 0, 0, 0.0, bytearray()
        while self.running:
            try:
                chunk = self.port.read(64)
            except Exception as exc:
                self.error.emit(str(exc))
                return
            for byte in chunk:
                now = time.monotonic()
                if state and now - last > FRAME_TIMEOUT_S:
                    state = 0
                last = now
                if state == 0:
                    if byte == SOF:
                        state = 1
                    elif byte == 0x0A:
                        text = ANSI_RE.sub("", line.decode(errors="replace")).strip()
                        if text:
                            self.text_received.emit(text)
                        line.clear()
                    elif byte != 0x0D:
                        line.append(byte)
                elif state == 1:
                    cmd, state = byte, 2
                else:
                    self.frame_received.emit(cmd, byte)
                    state = 0

    def stop(self):
        self.running = False
        self.wait(500)


class MachineView(QWidget):
    """Front of the washing machine: display, door window, rotating drum, water."""

    def __init__(self):
        super().__init__()
        self.setMinimumSize(340, 340)
        self.angle = 0.0
        self.state = V_IDLE
        self.door_open = False
        self.step = 0
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._spin)
        self.timer.start(40)

    def _spin(self):
        if self.state == V_RUN:
            speed = {1: 0, 2: 2, 3: 6, 4: 6, 5: 2, 6: 25}.get(self.step, 4)
            self.angle = (self.angle + speed) % 360
            self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        bw, bh = min(w - 20, 260), min(h - 20, 320)
        x0, y0 = (w - bw) / 2, (h - bh) / 2

        # body + control panel
        p.setPen(QPen(QColor("#9E9E9E"), 2))
        p.setBrush(QColor("#FAFAFA"))
        p.drawRoundedRect(QRectF(x0, y0, bw, bh), 14, 14)
        p.setBrush(QColor("#ECEFF1"))
        p.drawRoundedRect(QRectF(x0 + 10, y0 + 10, bw - 20, 44), 8, 8)
        p.setBrush(QColor("#263238"))
        p.drawRoundedRect(QRectF(x0 + 20, y0 + 18, bw * 0.55, 28), 4, 4)
        p.setPen(QColor("#80CBC4"))
        p.setFont(QFont("Consolas", 10, QFont.Bold))
        text = {V_IDLE: "CONFIG?", V_READY: "READY", V_RUN: STEPS[self.step - 1].upper() if self.step else "RUN",
                V_PAUSED: "DOOR!", V_DONE: "END"}.get(self.state, "")
        p.drawText(QRectF(x0 + 20, y0 + 18, bw * 0.55, 28), Qt.AlignCenter, text)
        led = {V_RUN: "#43A047", V_PAUSED: "#E53935", V_DONE: "#1E88E5"}.get(self.state, "#BDBDBD")
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(led))
        p.drawEllipse(QRectF(x0 + bw - 46, y0 + 22, 20, 20))

        # door window
        cx, cy = x0 + bw / 2, y0 + 60 + (bh - 60) / 2
        r = min(bw, bh - 60) * 0.38
        p.setPen(QPen(QColor("#78909C"), 8))
        p.setBrush(QColor("#CFD8DC"))
        p.drawEllipse(QRectF(cx - r, cy - r, 2 * r, 2 * r))

        # water level depends on the step
        level = {1: 0.35, 2: 0.45, 3: 0.45, 4: 0.4, 5: 0.1, 6: 0.0}.get(self.step, 0) if self.state in (V_RUN, V_PAUSED) else 0
        if level:
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(66, 165, 245, 150))
            p.drawChord(QRectF(cx - r + 4, cy - r + 4, 2 * r - 8, 2 * r - 8),
                        int(-90 * 16 - level * 180 * 16), int(level * 360 * 16))

        # drum paddles
        p.setPen(QPen(QColor("#546E7A"), 6, Qt.SolidLine, Qt.RoundCap))
        for k in range(3):
            a = math.radians(self.angle + k * 120)
            p.drawLine(int(cx + math.cos(a) * r * 0.25), int(cy + math.sin(a) * r * 0.25),
                       int(cx + math.cos(a) * r * 0.75), int(cy + math.sin(a) * r * 0.75))

        # open door swings to the left
        if self.door_open:
            p.setPen(QPen(QColor("#E53935"), 4))
            p.setBrush(QColor(255, 235, 238, 200))
            p.drawEllipse(QRectF(cx - r * 1.45, cy - r * 0.95, r * 0.45, 1.9 * r))
            p.setPen(QColor("#C62828"))
            p.setFont(QFont("Arial", 11, QFont.Bold))
            p.drawText(QRectF(x0, cy + r + 4, bw, 24), Qt.AlignCenter, "DOOR OPEN")


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("APK6 - Washing machine (STM32 + FreeRTOS)")
        self.resize(1150, 760)
        self.port = None
        self.reader = None
        self.state = V_IDLE

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        top = QHBoxLayout()
        root.addLayout(top, 3)

        left_panel = QWidget()
        left_panel.setMaximumWidth(320)
        left = QVBoxLayout(left_panel)
        left.setContentsMargins(0, 0, 0, 0)
        left.addWidget(self._build_connection())
        left.addWidget(self._build_config())
        left.addWidget(self._build_cycle())
        left.addStretch()
        top.addWidget(left_panel, 0)

        self.machine = MachineView()
        top.addWidget(self.machine, 1)
        top.addWidget(self._build_status(), 1)
        root.addWidget(self._build_log(), 2)

        self._set_connected(False)
        self._render_state()

    # -------------------------------------------------------------- UI ---
    def _build_connection(self):
        box = QGroupBox("Connection")
        lay = QGridLayout(box)
        self.port_combo = QComboBox()
        refresh = QPushButton("Refresh")
        refresh.clicked.connect(self.refresh_ports)
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        lay.addWidget(self.port_combo, 0, 0, 1, 2)
        lay.addWidget(refresh, 1, 0)
        lay.addWidget(self.connect_btn, 1, 1)
        self.refresh_ports()
        return box

    def _build_config(self):
        box = QGroupBox("Configuration")
        lay = QFormLayout(box)
        self.program = QComboBox()
        self.program.addItems(PROGRAMS)
        self.temp = QComboBox()
        self.temp.addItems([f"{t} °C" for t in TEMPS])
        self.temp.setCurrentIndex(2)
        self.spin = QComboBox()
        self.spin.addItems([f"{s * 100} rpm" for s in SPINS])
        self.spin.setCurrentIndex(3)
        self.send_cfg = QPushButton("Send configuration")
        self.send_cfg.clicked.connect(self.send_configuration)
        lay.addRow("Program", self.program)
        lay.addRow("Temperature", self.temp)
        lay.addRow("Spin", self.spin)
        lay.addRow(self.send_cfg)
        self.cfg_label = QLabel("Not configured")
        self.cfg_label.setStyleSheet("color:#C62828;")
        lay.addRow(self.cfg_label)
        return box

    def _build_cycle(self):
        box = QGroupBox("Cycle")
        lay = QVBoxLayout(box)
        self.start_btn = QPushButton("START CYCLE")
        self.start_btn.setMinimumHeight(44)
        self.start_btn.setStyleSheet("font-size:15px; font-weight:bold;")
        self.start_btn.clicked.connect(lambda: self.send_frame(CMD_START_CYCLE, 0))
        status_btn = QPushButton("Get status")
        status_btn.clicked.connect(lambda: self.send_frame(CMD_GET_STATUS, 0))
        self.door_btn = QPushButton("Press B1 (door) - simulator")
        self.door_btn.clicked.connect(self.sim_press)
        self.gui_guard = QCheckBox("GUI blocks invalid actions")
        self.gui_guard.setToolTip("Unticked: every click is sent, so you can watch the STM32 refuse it")
        self.gui_guard.toggled.connect(self._render_state)
        lay.addWidget(self.start_btn)
        lay.addWidget(status_btn)
        lay.addWidget(self.door_btn)
        lay.addWidget(self.gui_guard)
        self.cmd_buttons = [self.send_cfg, self.start_btn, status_btn]
        return box

    def _build_status(self):
        box = QGroupBox("Machine status")
        lay = QVBoxLayout(box)
        self.state_label = QLabel()
        self.state_label.setAlignment(Qt.AlignCenter)
        self.state_label.setStyleSheet("font-size:20px; font-weight:bold; padding:6px; border-radius:6px;")
        lay.addWidget(self.state_label)
        self.banner = QLabel("⚠  DOOR OPEN — cycle suspended\nPress B1 again to close the door")
        self.banner.setAlignment(Qt.AlignCenter)
        self.banner.setStyleSheet("background:#C62828; color:white; font-size:14px; font-weight:bold; padding:10px; border-radius:6px;")
        self.banner.hide()
        lay.addWidget(self.banner)

        self.step_labels = []
        grid = QGridLayout()
        for i, name in enumerate(STEPS):
            lbl = QLabel(f"{i + 1}. {name}")
            lbl.setStyleSheet("padding:4px;")
            self.step_labels.append(lbl)
            grid.addWidget(lbl, i // 2, i % 2)
        lay.addLayout(grid)
        lay.addWidget(QLabel("Current step"))
        self.step_bar = QProgressBar()
        lay.addWidget(self.step_bar)
        lay.addWidget(QLabel("Whole cycle"))
        self.total_bar = QProgressBar()
        lay.addWidget(self.total_bar)
        self.door_label = QLabel()
        lay.addWidget(self.door_label)
        self.msg = QLabel("")
        self.msg.setWordWrap(True)
        lay.addWidget(self.msg)
        lay.addStretch()
        self.step = 0
        self.door_open = False
        self.configured = False
        return box

    def _build_log(self):
        box = QGroupBox("Log")
        lay = QVBoxLayout(box)
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setFont(QFont("Consolas", 9))
        row = QHBoxLayout()
        self.show_progress = QCheckBox("Show EVT_PROGRESS")
        self.show_board = QCheckBox("Show board printf text")
        self.show_board.setChecked(True)
        clear = QPushButton("Clear")
        clear.clicked.connect(self.log_view.clear)
        row.addWidget(self.show_progress)
        row.addWidget(self.show_board)
        row.addStretch()
        row.addWidget(clear)
        lay.addWidget(self.log_view)
        lay.addLayout(row)
        return box

    # ------------------------------------------------------ connection ---
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
            self.port = None
            return
        self.reader = SerialReader(self.port)
        self.reader.frame_received.connect(self.on_frame)
        self.reader.text_received.connect(self.on_text)
        self.reader.error.connect(self.on_error)
        self.reader.start()
        self.log(f"Connected to {self.port_combo.currentText()}", "gray")
        self._set_connected(True)
        self.send_frame(CMD_GET_STATUS, 0)

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
        for b in self.cmd_buttons:
            b.setEnabled(on)
        self.door_btn.setVisible(isinstance(self.port, SimBoard))
        if on:
            self._render_state()

    def closeEvent(self, event):
        if self.port:
            self.disconnect_port()
        event.accept()

    # -------------------------------------------------------------- TX ---
    def send_frame(self, cmd, data):
        if not self.port:
            return
        self.port.write(bytes([SOF, cmd, data]))
        self.log(f"TX  {SOF:02X} {cmd:02X} {data:02X}   {CMD_NAMES.get(cmd, '?')} data={data}", "#1565C0")

    def send_configuration(self):
        self.send_frame(CMD_CFG_PROGRAM, self.program.currentIndex())
        self.send_frame(CMD_CFG_TEMP, TEMPS[self.temp.currentIndex()])
        self.send_frame(CMD_CFG_SPIN, SPINS[self.spin.currentIndex()])

    def sim_press(self):
        if isinstance(self.port, SimBoard):
            self.log("B1 pressed (simulated)", "gray")
            self.port.press_button()

    # -------------------------------------------------------------- RX ---
    def on_frame(self, cmd, data):
        raw = f"{SOF:02X} {cmd:02X} {data:02X}"
        if cmd & RSP_FLAG:
            self._on_response(raw, cmd & 0x7F, data)
        elif cmd in EVT_NAMES:
            self._on_event(raw, cmd, data)
        else:
            self.log(f"RX  {raw}   unexpected frame", "red")

    def _on_response(self, raw, req, data):
        name = CMD_NAMES.get(req, f"0x{req:02X}")
        if req == CMD_PING:
            self.log(f"RX  {raw}   PING echo {data}", "#2E7D32")
            return
        if req == CMD_GET_STATUS:
            sys_state, error = data & 0x0F, bool(data & STATUS_ERROR_FLAG)
            sensor = "ERROR_STATE" if error else "NO_ERROR_STATE"
            self.log(f"RX  {raw}   GET_STATUS -> {STATE_NAMES.get(sys_state, sys_state)}, sensor {sensor}", "#2E7D32")
            self.door_open = error
            self.configured = sys_state != IDLE_STATE
            self.state = {IDLE_STATE: V_IDLE, READY_STATE: V_READY}.get(sys_state, V_PAUSED if error else V_RUN)
            self._render_state()
            return
        status = STATUS_NAMES.get(data, f"0x{data:02X}")
        ok = data == ST_OK
        self.log(f"RX  {raw}   {name} -> {status}", "#2E7D32" if ok else "red")
        if req == CMD_START_CYCLE:
            if ok:
                self.state, self.step = V_RUN, 0
                self._flash("Cycle started", "#2E7D32")
            else:
                why = {0xE3: "a cycle is already running",
                       0xE4: "send a configuration first",
                       0xE5: "close the door first"}.get(data, status)
                self._flash(f"START refused: {why}", "#C62828")
        elif req == CMD_CFG_SPIN and ok:
            self.configured = True
            if self.state == V_IDLE:
                self.state = V_READY
            self.cfg_label.setText(f"Configured: {self.program.currentText()}, "
                                   f"{self.temp.currentText()}, {self.spin.currentText()}")
            self.cfg_label.setStyleSheet("color:#2E7D32;")
        elif not ok and data == 0xE3:
            self._flash("Configuration refused: cycle running", "#C62828")
        self._render_state()

    def _on_event(self, raw, evt, data):
        if evt == EVT_PROGRESS:
            self.step_bar.setValue(data)
            if self.step:
                self.total_bar.setValue(((self.step - 1) * 100 + data) // len(STEPS))
            if self.show_progress.isChecked():
                self.log(f"RX  {raw}   EVT_PROGRESS {data} %", "#6A1B9A")
            return
        if evt == EVT_STEP:
            self.step, self.state = data, V_RUN
            self.step_bar.setValue(0)
            self.total_bar.setValue((data - 1) * 100 // len(STEPS))
            text = f"EVT_STEP {data} ({STEPS[data - 1] if 1 <= data <= len(STEPS) else '?'})"
        elif evt == EVT_DONE:
            self.state = V_DONE
            self.step_bar.setValue(100)
            self.total_bar.setValue(100)
            text = "EVT_DONE - cycle finished"
            self._flash("Cycle finished", "#1565C0")
        elif evt == EVT_DOOR:
            self.door_open = bool(data)
            if not data and self.state == V_PAUSED:
                self.state = V_RUN
            text = f"EVT_DOOR {'OPEN' if data else 'CLOSED'}"
        else:  # EVT_ERROR
            self.state = V_PAUSED
            text = f"EVT_ERROR 0x{data:02X} {'DOOR OPEN' if data == 1 else ''}"
        self.log(f"RX  {raw}   {text}", "red" if evt == EVT_ERROR else "#6A1B9A")
        self._render_state()

    def on_text(self, text):
        if self.show_board.isChecked():
            self.log(f"    [board] {text}", "gray")

    def on_error(self, msg):
        self.log(f"Serial error: {msg}", "red")
        self.disconnect_port()

    # ---------------------------------------------------------- render ---
    def _render_state(self):
        colours = {V_IDLE: ("IDLE - send configuration", "#ECEFF1", "#37474F"),
                   V_READY: ("READY - press START", "#FFF9C4", "#F57F17"), V_RUN: ("RUNNING", "#C8E6C9", "#1B5E20"),
                   V_PAUSED: ("PAUSED", "#FFCDD2", "#B71C1C"), V_DONE: ("DONE - cycle finished", "#BBDEFB", "#0D47A1")}
        text, bg, fg = colours.get(self.state, ("?", "#EEE", "#000"))
        self.state_label.setText(text)
        self.state_label.setStyleSheet(f"font-size:20px; font-weight:bold; padding:6px; border-radius:6px; background:{bg}; color:{fg};")
        self.banner.setVisible(self.state == V_PAUSED)
        self.door_label.setText(f"Door: {'OPEN' if self.door_open else 'closed'}")
        self.door_label.setStyleSheet("color:#C62828; font-weight:bold;" if self.door_open else "color:#2E7D32;")
        for i, lbl in enumerate(self.step_labels):
            n = i + 1
            if self.state == V_DONE or (self.step and n < self.step):
                lbl.setStyleSheet("padding:4px; color:#2E7D32;")
                lbl.setText(f"✔ {n}. {STEPS[i]}")
            elif n == self.step and self.state in (V_RUN, V_PAUSED):
                lbl.setStyleSheet("padding:4px; background:#FFF59D; font-weight:bold; border-radius:4px;")
                lbl.setText(f"▶ {n}. {STEPS[i]}")
            else:
                lbl.setStyleSheet("padding:4px; color:#757575;")
                lbl.setText(f"   {n}. {STEPS[i]}")
        self.machine.state, self.machine.step, self.machine.door_open = self.state, self.step, self.door_open
        self.machine.update()

        if self.port and self.gui_guard.isChecked():
            busy = self.state in (V_RUN, V_PAUSED)
            self.send_cfg.setEnabled(not busy)
            self.start_btn.setEnabled(self.state in (V_READY, V_DONE) and not self.door_open)
        elif self.port:
            self.send_cfg.setEnabled(True)
            self.start_btn.setEnabled(True)

    def _flash(self, text, colour):
        self.msg.setText(text)
        self.msg.setStyleSheet(f"color:{colour}; font-weight:bold;")

    def log(self, text, colour="black"):
        self.log_view.append(f'<span style="color:{colour}; white-space:pre">{time.strftime("%H:%M:%S")}  {html.escape(text)}</span>')


def main():
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
