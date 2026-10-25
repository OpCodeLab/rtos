"""APK14 GUI: the PC is a sensor with a self test; the STM32 checks it before starting.

    pip install pyqt5 pyserial
    python apk14_gui.py

Start-up (the GUI answers automatically, like a real sensor):
    STM32 -> PC   AA 60 cc        AUTOTEST_REQ, cc = challenge
    PC -> STM32   AA 61 ss ee     ss = sensor state (0 = OK), ee = cc ^ 0xFF
    STM32 -> PC   AA 62 rr        STATUS: 0 = NORMAL, else SAFE + error bits
NORMAL: the GUI streams 1 sample / ms (AA 50 LO HI) and draws the spectrum (AA 90 ...).
SAFE  : nothing runs on the board; "RESET board" (AA 63 00) restarts it and its AutoTest.
"""
import html
import math
import struct
import sys
import threading
import time
from collections import deque

import serial
import serial.tools.list_ports
from PyQt5.QtCore import QPointF, QRectF, QThread, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QGridLayout, QGroupBox, QHBoxLayout,
    QLabel, QPushButton, QSpinBox, QTextEdit, QVBoxLayout, QWidget,
)

SOF = 0xAA
CMD_SAMPLE, AUTOTEST_REQ, AUTOTEST_RSP, EVT_STATUS, CMD_RESET, EVT_FFT = 0x50, 0x60, 0x61, 0x62, 0x63, 0x90
FS_HZ, FFT_N = 1000, 256
BINS = FFT_N // 2
FRAME_LEN = {AUTOTEST_REQ: 3, EVT_STATUS: 3, EVT_FFT: 2 + 2 * BINS}

SENSOR_FAULTS = [(0x01, "POWER", "supply out of range"), (0x02, "STUCK", "output does not move"),
                 (0x04, "RANGE", "output saturated"), (0x08, "CALIB", "calibration data corrupted")]
ERRORS = {0x01: "POWER", 0x02: "STUCK", 0x04: "RANGE", 0x08: "CALIB",
          0x20: "LOST (no sample 500 ms)", 0x40: "LINK (bad echo)", 0x80: "NO ANSWER"}


def error_names(bits):
    return ", ".join(n for b, n in ERRORS.items() if bits & b) or "none"


# ============================================================ the sensor ===
class Sensor:
    """State of the simulated sensor, read by the serial thread (no lock needed: plain values)."""

    def __init__(self):
        self.state = 0              # error bits reported in the AutoTest answer
        self.mute = False           # does not answer the AutoTest
        self.bad_echo = False       # answers with a wrong echo
        self.disconnected = False   # stops sending samples (run-time fault)
        self.tones = [(50.0, 1000)]


class Streamer(threading.Thread):
    """Sends the samples that are due every ~1 ms: sample n is at time n / fs."""

    def __init__(self, write, sensor, on_samples):
        super().__init__(daemon=True)
        self.write, self.sensor, self.on_samples = write, sensor, on_samples
        self.running, self.n = True, 0

    def run(self):
        t0 = time.perf_counter()
        while self.running:
            due = int((time.perf_counter() - t0) * FS_HZ)
            values = []
            while self.n < due:
                t = self.n / FS_HZ
                v = sum(a * math.sin(2 * math.pi * f * t) for f, a in self.sensor.tones)
                values.append(max(-32768, min(32767, round(v))))
                self.n += 1
            if values and not self.sensor.disconnected:
                self.write(b"".join(bytes([SOF, CMD_SAMPLE]) + struct.pack("<h", v) for v in values))
                self.on_samples(values)
            time.sleep(0.001)


# ======================================================= receiver thread ===
class Receiver(QThread):
    """Parses the board frames. The AutoTest answer is sent from HERE, at once (board timeout 500 ms)."""
    status = pyqtSignal(int)
    spectrum = pyqtSignal(list)
    log = pyqtSignal(str, str)

    def __init__(self, port, write, sensor):
        super().__init__()
        self.port, self.write, self.sensor, self.running = port, write, sensor, True

    def answer_autotest(self, challenge):
        s = self.sensor
        if s.mute:
            self.log.emit(f"RX  AA 60 {challenge:02X}  AUTOTEST_REQ  -> sensor MUTE, no answer", "#AD1457")
            return
        echo = (challenge ^ 0xFF) ^ (0x0F if s.bad_echo else 0)
        self.write(bytes([SOF, AUTOTEST_RSP, s.state, echo]))
        self.log.emit(f"RX  AA 60 {challenge:02X}  AUTOTEST_REQ", "#1565C0")
        self.log.emit(f"TX  AA 61 {s.state:02X} {echo:02X}  AUTOTEST_RSP state={error_names(s.state)}"
                      + ("  (echo CORRUPTED)" if s.bad_echo else ""), "#1565C0")

    def run(self):
        buf, line = bytearray(), bytearray()
        while self.running:
            try:
                buf += self.port.read(512)
            except Exception as exc:
                self.log.emit(f"Serial error: {exc}", "red")
                return
            while buf:
                if buf[0] != SOF:                                  # text from LOG_INFO / printf
                    b = buf.pop(0)
                    if b == 0x0A:
                        t = bytes(line).decode(errors="replace")
                        t = "".join(c for c in t if c.isprintable()).replace("[0m", "")
                        for col in ("[31m", "[32m", "[33m", "[36m", "[37m"):
                            t = t.replace(col, "")
                        if t.strip():
                            self.log.emit(f"    [board] {t.strip()}", "gray")
                        line.clear()
                    elif b != 0x0D:
                        line.append(b)
                    continue
                if len(buf) < 2:
                    break
                n = FRAME_LEN.get(buf[1])
                if n is None:                                      # not a frame start
                    buf.pop(0)
                    continue
                if len(buf) < n:
                    break                                          # wait for the rest
                frame, buf = bytes(buf[:n]), buf[n:]
                if frame[1] == AUTOTEST_REQ:
                    self.answer_autotest(frame[2])
                elif frame[1] == EVT_STATUS:
                    self.status.emit(frame[2])
                else:
                    self.spectrum.emit(list(struct.unpack(f"<{BINS}H", frame[2:])))


# ================================================================= plot ===
class Plot(QWidget):
    def __init__(self, title, bars):
        super().__init__()
        self.title, self.bars, self.data = title, bars, []
        self.setMinimumHeight(190)

    def paintEvent(self, _):
        p = QPainter(self)
        w, h = self.width() - 20, self.height() - 40
        p.fillRect(self.rect(), QColor("white"))
        p.drawText(10, 15, self.title)
        if not self.data:
            return
        top = max(max(abs(v) for v in self.data), 1)
        if self.bars:
            bw = w / len(self.data)
            for k, m in enumerate(self.data):
                p.fillRect(QRectF(10 + k * bw, 25 + h * (1 - m / top), max(bw - 1, 1), h * m / top),
                           QColor("#FF8F00"))
            for f in range(0, FS_HZ // 2 + 1, 50):
                p.drawText(int(10 + w * f / (FS_HZ / 2)) - 12, h + 38, f"{f}")
        elif len(self.data) > 1:
            dx = w / (len(self.data) - 1)
            p.setPen(QPen(QColor("#212121"), 1))
            p.drawPolyline(QPolygonF([QPointF(10 + i * dx, 25 + h / 2 * (1 - v / top))
                                      for i, v in enumerate(self.data)]))


# ============================================================ main window ===
class Window(QWidget):
    samples_sent = pyqtSignal(list)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("APK14 - sensor AutoTest + SAFE mode")
        self.resize(1200, 820)
        self.port = self.receiver = self.streamer = None
        self.tx_lock = threading.Lock()
        self.sensor = Sensor()
        self.last = deque(maxlen=FFT_N)

        left = QVBoxLayout()
        box = QGroupBox("Connection")
        g = QGridLayout(box)
        self.ports = QComboBox()
        self.ports.addItems([p.device for p in serial.tools.list_ports.comports()])
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        self.reset_btn = QPushButton("RESET board (new AutoTest)")
        self.reset_btn.clicked.connect(self.reset_board)
        g.addWidget(self.ports, 0, 0)
        g.addWidget(self.connect_btn, 0, 1)
        g.addWidget(self.reset_btn, 1, 0, 1, 2)
        left.addWidget(box)

        box = QGroupBox("Sensor state (answer to the AutoTest)")
        v = QVBoxLayout(box)
        self.fault_boxes = []
        for bit, name, desc in SENSOR_FAULTS:
            c = QCheckBox(f"{name} - {desc}")
            c.toggled.connect(self.update_sensor)
            v.addWidget(c)
            self.fault_boxes.append((bit, c))
        self.mute = QCheckBox("Sensor does not answer  -> NO ANSWER")
        self.bad_echo = QCheckBox("Answer corrupted (bad echo)  -> LINK")
        for c in (self.mute, self.bad_echo):
            c.toggled.connect(self.update_sensor)
            v.addWidget(c)
        self.state_label = QLabel()
        v.addWidget(self.state_label)
        left.addWidget(box)

        box = QGroupBox("Run-time fault")
        v = QVBoxLayout(box)
        self.disconnect = QCheckBox("Disconnect sensor (stop samples)  -> LOST")
        self.disconnect.toggled.connect(self.update_sensor)
        v.addWidget(self.disconnect)
        left.addWidget(box)

        box = QGroupBox("Signal")
        g = QGridLayout(box)
        self.on, self.freq, self.amp = [], [], []
        for i, (f, a) in enumerate(((50, 1000), (120, 600), (300, 400))):
            on = QCheckBox(f"sine {i + 1}")
            on.setChecked(i == 0)
            fr = QDoubleSpinBox(suffix=" Hz", maximum=FS_HZ / 2, value=f)
            am = QSpinBox(maximum=10000, singleStep=100, value=a)
            for sig in (on.toggled, fr.valueChanged, am.valueChanged):
                sig.connect(self.update_sensor)
            g.addWidget(on, i, 0)
            g.addWidget(fr, i, 1)
            g.addWidget(am, i, 2)
            self.on.append(on)
            self.freq.append(fr)
            self.amp.append(am)
        left.addWidget(box)
        left.addStretch()

        right = QVBoxLayout()
        self.mode = QLabel()
        self.mode.setMinimumHeight(80)
        right.addWidget(self.mode)
        self.signal_plot = Plot("Signal sent (last 256 samples)", bars=False)
        self.fft_plot = Plot("FFT from the STM32 (0 .. 500 Hz)", bars=True)
        right.addWidget(self.signal_plot)
        right.addWidget(self.fft_plot)
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setFont(QFont("Consolas", 9))
        right.addWidget(self.log_view)

        root = QHBoxLayout(self)
        panel = QWidget()
        panel.setLayout(left)
        panel.setMaximumWidth(380)
        root.addWidget(panel)
        root.addLayout(right, 1)

        self.samples_sent.connect(self.last.extend)
        timer = QTimer(self)
        timer.timeout.connect(self.redraw_signal)
        timer.start(50)
        self.update_sensor()
        self.show_mode(None)
        self.reset_btn.setEnabled(False)

    # ------------------------------------------------------------ sensor
    def update_sensor(self, *_):
        s = self.sensor
        s.state = sum(bit for bit, c in self.fault_boxes if c.isChecked())
        s.mute, s.bad_echo = self.mute.isChecked(), self.bad_echo.isChecked()
        s.disconnected = self.disconnect.isChecked()
        s.tones = [(self.freq[i].value(), self.amp[i].value()) for i in range(3) if self.on[i].isChecked()]
        ok = s.state == 0 and not s.mute and not s.bad_echo
        self.state_label.setText(f"<b>Sensor answer: {'OK' if ok else 'FAULT'}</b>  (state = 0x{s.state:02X})")
        self.state_label.setStyleSheet(f"color: {'#2E7D32' if ok else '#C62828'}; font-size: 13px;")

    # -------------------------------------------------------- connection
    def write(self, data):
        with self.tx_lock:
            if self.port:
                self.port.write(data)

    def toggle_connection(self):
        if self.port:
            self.stop_stream()
            self.receiver.running = False
            self.receiver.wait(500)
            self.port.close()
            self.port = self.receiver = None
            self.connect_btn.setText("Connect")
            self.reset_btn.setEnabled(False)
            self.show_mode(None)
            return
        try:
            self.port = serial.Serial(self.ports.currentText(), 115200, timeout=0.02)
        except serial.SerialException as exc:
            self.log(f"Cannot open port: {exc}", "red")
            return
        self.receiver = Receiver(self.port, self.write, self.sensor)
        self.receiver.status.connect(self.on_status)
        self.receiver.spectrum.connect(self.on_spectrum)
        self.receiver.log.connect(self.log)
        self.receiver.start()
        self.connect_btn.setText("Disconnect")
        self.reset_btn.setEnabled(True)
        self.log("Connected. Press RESET board to run the AutoTest.", "gray")

    def reset_board(self):
        self.stop_stream()
        self.show_mode(None)
        self.write(bytes([SOF, CMD_RESET, 0]))
        self.log("TX  AA 63 00  RESET -> the board restarts and runs its AutoTest", "#1565C0")

    def stop_stream(self):
        if self.streamer:
            self.streamer.running = False
            self.streamer.join(0.5)
            self.streamer = None

    # ---------------------------------------------------------------- RX
    def on_status(self, err):
        if err != self.last_status:                            # SAFE repeats it every 1 s: log changes only
            self.log(f"RX  AA 62 {err:02X}  STATUS -> {'NORMAL' if err == 0 else 'SAFE: ' + error_names(err)}",
                     "#2E7D32" if err == 0 else "#C62828")
        self.last_status = err
        self.show_mode(err)
        if err == 0 and not self.streamer:                     # self test passed: the sensor starts
            self.streamer = Streamer(self.write, self.sensor, self.samples_sent.emit)
            self.streamer.start()
            self.log("Sensor streaming started: 1 sample / ms", "#1565C0")
        elif err != 0:
            self.stop_stream()

    def on_spectrum(self, mags):
        self.fft_plot.data = mags
        self.fft_plot.update()

    def redraw_signal(self):
        self.signal_plot.data = list(self.last)
        self.signal_plot.update()

    def show_mode(self, err):
        if err is None:
            self.last_status = None
            txt, sub, col = "WAITING", "connect, then RESET board to run the AutoTest", "#757575"
        elif err == 0:
            txt, sub, col = "NORMAL", "AutoTest passed - FFT running", "#2E7D32"
        else:
            txt, sub, col = "SAFE", f"errors: {error_names(err)} - fix the sensor, then RESET board", "#C62828"
        self.mode.setText(f"<div style='font-size:28px; font-weight:bold'>{txt}</div>"
                          f"<div style='font-size:13px'>{html.escape(sub)}</div>")
        self.mode.setStyleSheet(f"background:{col}; color:white; border-radius:8px; padding:8px;")

    def log(self, text, colour="black"):
        self.log_view.append(f'<span style="color:{colour}; white-space:pre">{time.strftime("%H:%M:%S")}  '
                             f'{html.escape(text)}</span>')

    def closeEvent(self, e):
        if self.port:
            self.toggle_connection()
        e.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = Window()
    win.show()
    sys.exit(app.exec_())
