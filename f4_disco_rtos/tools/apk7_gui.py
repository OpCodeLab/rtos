"""APK7 GUI: Button 1 / Button 2 -> event group on the STM32 -> 5 s timer.

    pip install pyqt5 pyserial
    python apk7_gui.py

Choose "Simulator (no board)" in the port list to try it without hardware.
"""
import html
import queue
import re
import sys
import threading
import time

import serial
import serial.tools.list_ports
from PyQt5.QtCore import Qt, QThread, QTimer, pyqtSignal
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QGridLayout, QGroupBox, QHBoxLayout, QLabel,
    QMainWindow, QProgressBar, QPushButton, QTextEdit, QVBoxLayout, QWidget,
)

SOF = 0xAA
RSP_FLAG = 0x80
FRAME_TIMEOUT_S = 0.05

CMD_PING = 0x01
CMD_BUTTON_1 = 0x40
CMD_BUTTON_2 = 0x41
EVT_TIMER_STARTED = 0x50
EVT_TIMER_EXPIRED = 0x51

NAMES = {CMD_PING: "PING", CMD_BUTTON_1: "BUTTON_1", CMD_BUTTON_2: "BUTTON_2",
         EVT_TIMER_STARTED: "EVT_TIMER_STARTED", EVT_TIMER_EXPIRED: "EVT_TIMER_EXPIRED"}
STATUS = {0x00: "OK", 0xE1: "BAD_CMD"}
ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")
SIM_PORT = "__sim__"


class SimBoard:
    """Software twin of apk7.c (event group = a set of bits + a condition variable)."""

    def __init__(self, duration_s=5):
        self.duration = duration_s
        self.out = bytearray()
        self.lock = threading.Lock()
        self.frames = queue.Queue(8)
        self.bits = 0
        self.cond = threading.Condition()
        self.rx, self.cmd, self.last = 0, 0, 0.0
        for fn in (self._link_layer, self._app_task):
            threading.Thread(target=fn, daemon=True).start()

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
        pass

    def _send(self, cmd, data, log=None):
        with self.lock:
            if log:
                self.out += f"\x1b[36m{log}\x1b[0m\r\n".encode()
            self.out += bytes([SOF, cmd & 0xFF, data & 0xFF])

    def _set_bits(self, mask):
        with self.cond:
            self.bits |= mask
            self.cond.notify_all()

    def _wait_all(self, mask):
        with self.cond:
            self.cond.wait_for(lambda: self.bits & mask == mask)
            self.bits &= ~mask

    def _link_layer(self):
        while True:
            cmd, data = self.frames.get()
            log = f"RX frame CMD=0x{cmd:02X} DATA=0x{data:02X}"
            if cmd == CMD_PING:
                self._send(cmd | RSP_FLAG, data, log)
            elif cmd in (CMD_BUTTON_1, CMD_BUTTON_2):
                self._set_bits(1 if cmd == CMD_BUTTON_1 else 2)
                self._send(cmd | RSP_FLAG, 0x00, log)
            else:
                self._send(cmd | RSP_FLAG, 0xE1, log)

    def _app_task(self):
        while True:
            self._wait_all(0b011)
            threading.Timer(self.duration, self._set_bits, args=(0b100,)).start()
            self._send(EVT_TIMER_STARTED, self.duration, f"Button 1 + Button 2 received: timer started ({self.duration} s)")
            self._wait_all(0b100)
            self._send(EVT_TIMER_EXPIRED, 0, "Timer expired")


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
                        text = ANSI_RE.sub("", line.decode(errors="replace")).strip()
                        if text:
                            self.text_received.emit(text)
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


class Lamp(QLabel):
    def __init__(self, text):
        super().__init__(text)
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumHeight(48)
        self.set(False)

    def set(self, on, colour="#43A047"):
        bg, fg = (colour, "white") if on else ("#ECEFF1", "#78909C")
        self.setStyleSheet(f"background:{bg}; color:{fg}; border-radius:8px; font-size:14px; font-weight:bold;")


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("APK7 - Event group + timer")
        self.resize(900, 640)
        self.port = None
        self.reader = None
        self.duration = 0
        self.t0 = 0.0
        self.got1 = self.got2 = False      # mirror of EV_BUTTON_1 / EV_BUTTON_2

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        # connection
        conn = QHBoxLayout()
        self.port_combo = QComboBox()
        refresh = QPushButton("Refresh")
        refresh.clicked.connect(self.refresh_ports)
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        conn.addWidget(QLabel("Port"))
        conn.addWidget(self.port_combo, 1)
        conn.addWidget(refresh)
        conn.addWidget(self.connect_btn)
        root.addLayout(conn)

        # buttons + status
        mid = QHBoxLayout()
        cmd_box = QGroupBox("Send")
        cl = QVBoxLayout(cmd_box)
        self.b1 = QPushButton("BUTTON 1")
        self.b2 = QPushButton("BUTTON 2")
        for b, c in ((self.b1, CMD_BUTTON_1), (self.b2, CMD_BUTTON_2)):
            b.setMinimumHeight(60)
            b.setStyleSheet("font-size:18px; font-weight:bold;")
            b.clicked.connect(lambda _, cmd=c: self.send_frame(cmd, 0))
            cl.addWidget(b)
        self.ping = QPushButton("PING")
        self.ping.clicked.connect(lambda: self.send_frame(CMD_PING, 0x55))
        cl.addWidget(self.ping)
        mid.addWidget(cmd_box, 1)

        st_box = QGroupBox("STM32 event group")
        gl = QGridLayout(st_box)
        self.lamp1 = Lamp("EV_BUTTON_1")
        self.lamp2 = Lamp("EV_BUTTON_2")
        self.lamp_t = Lamp("TIMER")
        gl.addWidget(self.lamp1, 0, 0)
        gl.addWidget(self.lamp2, 0, 1)
        gl.addWidget(self.lamp_t, 1, 0, 1, 2)
        self.bar = QProgressBar()
        self.bar.setFormat("%v ms")
        gl.addWidget(self.bar, 2, 0, 1, 2)
        self.status = QLabel("Press BUTTON 1 and BUTTON 2 (any order)")
        self.status.setAlignment(Qt.AlignCenter)
        self.status.setStyleSheet("font-size:13px;")
        gl.addWidget(self.status, 3, 0, 1, 2)
        mid.addWidget(st_box, 2)
        root.addLayout(mid)

        # log
        log_box = QGroupBox("Log")
        ll = QVBoxLayout(log_box)
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setFont(QFont("Consolas", 9))
        row = QHBoxLayout()
        self.show_board = QCheckBox("Show board printf text")
        self.show_board.setChecked(True)
        clear = QPushButton("Clear")
        clear.clicked.connect(self.log_view.clear)
        row.addWidget(self.show_board)
        row.addStretch()
        row.addWidget(clear)
        ll.addWidget(self.log_view)
        ll.addLayout(row)
        root.addWidget(log_box, 1)

        self.tick = QTimer(self)
        self.tick.timeout.connect(self._countdown)
        self.refresh_ports()
        self._set_connected(False)

    # ---------------------------------------------------------- connection
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
        self.reader.text_received.connect(self.on_text)
        self.reader.error.connect(self.on_error)
        self.reader.start()
        self.log(f"Connected to {self.port_combo.currentText()}", "gray")
        self._set_connected(True)

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
        for b in (self.b1, self.b2, self.ping):
            b.setEnabled(on)

    def closeEvent(self, event):
        if self.port:
            self.disconnect_port()
        event.accept()

    # ------------------------------------------------------------- TX / RX
    def send_frame(self, cmd, data):
        if self.port:
            self.port.write(bytes([SOF, cmd, data]))
            self.log(f"TX  AA {cmd:02X} {data:02X}   {NAMES.get(cmd, '?')}", "#1565C0")

    def on_frame(self, cmd, data):
        raw = f"AA {cmd:02X} {data:02X}"
        if cmd & RSP_FLAG:
            req = cmd & 0x7F
            ok = data == 0x00 or req == CMD_PING
            self.log(f"RX  {raw}   {NAMES.get(req, hex(req))} -> {data if req == CMD_PING else STATUS.get(data, hex(data))}",
                     "#2E7D32" if ok else "red")
            if req == CMD_BUTTON_1 and ok:
                self.got1 = True
                self.lamp1.set(True)
            elif req == CMD_BUTTON_2 and ok:
                self.got2 = True
                self.lamp2.set(True, "#EF6C00")
            if req in (CMD_BUTTON_1, CMD_BUTTON_2) and not self.tick.isActive() and self.got1 != self.got2:
                self.status.setText(f"Waiting for BUTTON {2 if self.got1 else 1} ...")
        elif cmd == EVT_TIMER_STARTED:
            self.log(f"RX  {raw}   EVT_TIMER_STARTED {data} s", "#6A1B9A")
            self.got1 = self.got2 = False
            self.lamp1.set(False)
            self.lamp2.set(False)
            self.lamp_t.set(True, "#1E88E5")
            self.duration = data
            self.bar.setRange(0, data * 1000)
            self.bar.setValue(data * 1000)
            self.status.setText(f"Both buttons received: timer running ({data} s)")
            self.t0 = time.monotonic()
            self.tick.start(50)
        elif cmd == EVT_TIMER_EXPIRED:
            self.log(f"RX  {raw}   EVT_TIMER_EXPIRED", "#6A1B9A")
            self.tick.stop()
            self.bar.setValue(0)
            self.lamp_t.set(False)
            self.status.setText("Timer expired. Press BUTTON 1 and BUTTON 2 again")
        else:
            self.log(f"RX  {raw}   unexpected frame", "red")

    def _countdown(self):
        left = max(0.0, self.duration - (time.monotonic() - self.t0))
        self.bar.setValue(int(left * 1000))

    def on_text(self, text):
        if self.show_board.isChecked():
            self.log(f"    [board] {text}", "gray")

    def on_error(self, msg):
        self.log(f"Serial error: {msg}", "red")
        self.disconnect_port()

    def log(self, text, colour="black"):
        self.log_view.append(f'<span style="color:{colour}; white-space:pre">{time.strftime("%H:%M:%S")}  {html.escape(text)}</span>')


def main():
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
