"""APK12 GUI: the PC is a sensor, the STM32 computes the FFT with CMSIS-DSP.

    pip install pyqt5 pyserial
    python apk12_gui.py

Sends 1 sample every 1 ms (fs = 1000 Hz):   AA 50 LO HI
Receives the spectrum every 256 ms:         AA 90 <128 x uint16>
and a timing line:   TIME fill=256012 wake=7 fft=412 tx=22410 job=22823 us
"""
import math
import re
import struct
import sys
import threading
import time
from collections import deque

import serial
import serial.tools.list_ports
from PyQt5.QtCore import QPointF, QRectF, Qt, QThread, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QGridLayout, QHBoxLayout,
    QLabel, QPushButton, QSpinBox, QVBoxLayout, QWidget,
)

SOF, CMD_SAMPLE, EVT_FFT = 0xAA, 0x50, 0x90
FS_HZ, FFT_N = 1000, 256
BINS = FFT_N // 2
TIME_RE = re.compile(rb"TIME fill=(\d+) wake=(\d+) fft=(\d+) tx=(\d+) job=(\d+) us")


# ======================================================= sensor (PC -> STM32) ===
class Sensor(threading.Thread):
    """Sends the samples that are due every ~1 ms: sample n is at time n / fs."""

    def __init__(self, port, get_tones, on_samples):
        super().__init__(daemon=True)
        self.port, self.get_tones, self.on_samples = port, get_tones, on_samples
        self.running, self.n = True, 0

    def run(self):
        t0 = time.perf_counter()
        while self.running:
            due = int((time.perf_counter() - t0) * FS_HZ)
            values = []
            while self.n < due:
                t = self.n / FS_HZ
                v = sum(a * math.sin(2 * math.pi * f * t) for f, a in self.get_tones())
                values.append(max(-32768, min(32767, round(v))))
                self.n += 1
            if values:
                self.port.write(b"".join(bytes([SOF, CMD_SAMPLE]) + struct.pack("<h", v) for v in values))
                self.on_samples(values)
            time.sleep(0.001)


# ======================================================= receiver (STM32 -> PC) ===
class Receiver(QThread):
    spectrum = pyqtSignal(list)
    timing = pyqtSignal(dict)

    def __init__(self, port):
        super().__init__()
        self.port, self.running = port, True

    def run(self):
        buf = bytearray()
        while self.running:
            buf += self.port.read(512)
            while True:
                t = TIME_RE.search(buf)                        # text line sent after each frame
                if t and (buf.find(bytes([SOF, EVT_FFT])) < 0 or t.start() < buf.find(bytes([SOF, EVT_FFT]))):
                    self.timing.emit(dict(zip(("fill", "wake", "fft", "tx", "job"), map(int, t.groups()))))
                    del buf[:t.end()]
                    continue
                i = buf.find(bytes([SOF, EVT_FFT]))
                if i < 0:
                    del buf[:-100]                             # keep a possible SOF / partial line
                    break
                if len(buf) < i + 2 + 2 * BINS:
                    del buf[:i]                                # wait for the rest
                    break
                mags = struct.unpack(f"<{BINS}H", buf[i + 2:i + 2 + 2 * BINS])
                del buf[:i + 2 + 2 * BINS]
                self.spectrum.emit(list(mags))


# ================================================================= plot ===
class Plot(QWidget):
    def __init__(self, title, bars):
        super().__init__()
        self.title, self.bars, self.data = title, bars, []
        self.setMinimumHeight(250)

    def paintEvent(self, _):
        p = QPainter(self)
        w, h = self.width() - 20, self.height() - 40
        p.fillRect(self.rect(), QColor("white"))
        p.drawText(10, 15, self.title)
        if not self.data:
            return
        top = max(max(abs(v) for v in self.data), 1)
        if self.bars:                                          # spectrum: 0 .. 500 Hz
            bw = w / len(self.data)
            for k, m in enumerate(self.data):
                p.fillRect(QRectF(10 + k * bw, 25 + h * (1 - m / top), max(bw - 1, 1), h * m / top),
                           QColor("#FF8F00"))
            for f in range(0, FS_HZ // 2 + 1, 50):
                p.drawText(int(10 + w * f / (FS_HZ / 2)) - 12, h + 38, f"{f}")
        else:                                                  # signal: -top .. +top
            dx = w / (len(self.data) - 1)
            p.setPen(QPen(QColor("#212121"), 1))
            p.drawPolyline(QPolygonF([QPointF(10 + i * dx, 25 + h / 2 * (1 - v / top))
                                      for i, v in enumerate(self.data)]))


# ============================================================ main window ===
class Window(QWidget):
    samples_sent = pyqtSignal(list)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("APK12 - double buffer + CMSIS-DSP FFT")
        self.resize(1000, 750)
        self.port = self.sensor = self.receiver = None
        self.tones = []
        self.last = deque(maxlen=FFT_N)

        top = QGridLayout()
        self.ports = QComboBox()
        self.ports.addItems([p.device for p in serial.tools.list_ports.comports()])
        self.start_btn = QPushButton("Start")
        self.start_btn.clicked.connect(self.start_stop)
        top.addWidget(QLabel("Port"), 0, 0)
        top.addWidget(self.ports, 0, 1)
        top.addWidget(self.start_btn, 0, 2)
        self.on, self.freq, self.amp = [], [], []
        for i, (f, a) in enumerate(((50, 1000), (120, 600), (300, 400))):
            on = QCheckBox(f"sine {i + 1}")
            on.setChecked(i == 0)
            fr = QDoubleSpinBox(suffix=" Hz", maximum=FS_HZ / 2, value=f)
            am = QSpinBox(maximum=10000, singleStep=100, value=a)
            for wdg, sig in ((on, on.toggled), (fr, fr.valueChanged), (am, am.valueChanged)):
                sig.connect(self.update_tones)
            top.addWidget(on, 1 + i, 0)
            top.addWidget(fr, 1 + i, 1)
            top.addWidget(am, 1 + i, 2)
            self.on.append(on)
            self.freq.append(fr)
            self.amp.append(am)

        self.signal_plot = Plot("Signal sent (last 256 samples = 256 ms)", bars=False)
        self.fft_plot = Plot("FFT from the STM32 (0 .. 500 Hz, 3.9 Hz per bin)", bars=True)
        self.peaks = QLabel("Peaks: -")
        self.timing = QLabel("Timing: -")
        self.timing.setStyleSheet("font-family: Consolas; font-size: 13px;")
        self.job_max = 0
        layout = QVBoxLayout(self)
        row = QHBoxLayout()
        row.addLayout(top)
        row.addStretch()
        layout.addLayout(row)
        layout.addWidget(self.signal_plot)
        layout.addWidget(self.fft_plot)
        layout.addWidget(self.peaks)
        layout.addWidget(self.timing)

        self.samples_sent.connect(self.last.extend)
        timer = QTimer(self)
        timer.timeout.connect(self.redraw_signal)
        timer.start(50)
        self.update_tones()

    def update_tones(self, *_):
        self.tones = [(self.freq[i].value(), self.amp[i].value()) for i in range(3) if self.on[i].isChecked()]

    def start_stop(self):
        if self.sensor:
            self.sensor.running = self.receiver.running = False
            self.sensor.join(0.5)
            self.receiver.wait(500)
            self.port.close()
            self.sensor = self.receiver = self.port = None
            self.start_btn.setText("Start")
            return
        self.port = serial.Serial(self.ports.currentText(), 115200, timeout=0.02)
        self.receiver = Receiver(self.port)
        self.receiver.spectrum.connect(self.show_fft)
        self.receiver.timing.connect(self.show_timing)
        self.job_max = 0
        self.receiver.start()
        self.sensor = Sensor(self.port, lambda: self.tones, self.samples_sent.emit)
        self.sensor.start()
        self.start_btn.setText("Stop")

    def redraw_signal(self):
        self.signal_plot.data = list(self.last)
        self.signal_plot.update()

    def show_timing(self, t):
        """Times measured on the STM32 with the DWT cycle counter (us)."""
        self.job_max = max(self.job_max, t["job"])
        load = 100.0 * t["job"] / t["fill"] if t["fill"] else 0.0
        self.timing.setText(
            f"STM32 timing:  fill half = {t['fill'] / 1000:7.2f} ms   |   FftTask job = {t['job'] / 1000:6.2f} ms "
            f"(max {self.job_max / 1000:.2f})  =  fft {t['fft']} us + tx {t['tx'] / 1000:.2f} ms   |   "
            f"wake-up {t['wake']} us   |   load {load:.1f} %   |   margin {(t['fill'] - t['job']) / 1000:.1f} ms")

    def show_fft(self, mags):
        self.fft_plot.data = mags
        self.fft_plot.update()
        top = max(mags) or 1
        peaks = [k for k in range(1, BINS - 1)
                 if mags[k] > top * 0.1 and mags[k] > mags[k - 1] and mags[k] >= mags[k + 1]]
        self.peaks.setText("Peaks: " + "   ".join(f"{k * FS_HZ / FFT_N:.1f} Hz ({mags[k]})" for k in peaks))


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = Window()
    win.show()
    sys.exit(app.exec_())
