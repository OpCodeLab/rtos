"""APK15 GUI: the PC simulates TWO sensors, the STM32 computes one FFT per sensor.

    pip install pyqt5 pyserial
    python apk15_gui.py

Every 1 ms (fs = 1000 Hz) sends one sample of each sensor:
    AA 51 LO HI   sensor 1
    AA 52 LO HI   sensor 2
Receives one spectrum per sensor every 256 ms:
    AA 91 <128 x uint16>   from FftSensor1
    AA 92 <128 x uint16>   from FftSensor2
"""
import math
import struct
import sys
import threading
import time
from collections import deque

import serial
import serial.tools.list_ports
from PyQt5.QtCore import QPointF, QRectF, QThread, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QGridLayout, QGroupBox, QHBoxLayout,
    QLabel, QPushButton, QSpinBox, QVBoxLayout, QWidget,
)

SOF = 0xAA
CMD_SAMPLE = (0x51, 0x52)                                      # sensor 1, sensor 2
EVT_FFT = (0x91, 0x92)
FS_HZ, FFT_N = 1000, 256
BINS = FFT_N // 2
FRAME_LEN = 2 + 2 * BINS
COLORS = ("#FF8F00", "#1E88E5")
DEFAULT_TONES = (((50, 1000), (120, 600), (300, 400)),         # sensor 1: 50 Hz on
                 ((80, 800), (200, 800), (420, 500)))          # sensor 2: 200 Hz on


# ======================================================= sensors (PC -> STM32) ===
class Sensors(threading.Thread):
    """Sends the samples that are due every ~1 ms: sample n of BOTH sensors at time n / fs."""

    def __init__(self, port, get_tones, on_samples):
        super().__init__(daemon=True)
        self.port, self.get_tones, self.on_samples = port, get_tones, on_samples
        self.running, self.n = True, 0

    def run(self):
        t0 = time.perf_counter()
        while self.running:
            due = int((time.perf_counter() - t0) * FS_HZ)
            frames, values = bytearray(), ([], [])
            while self.n < due:
                t = self.n / FS_HZ
                for s in range(2):
                    v = sum(a * math.sin(2 * math.pi * f * t) for f, a in self.get_tones(s))
                    v = max(-32768, min(32767, round(v)))
                    frames += bytes([SOF, CMD_SAMPLE[s]]) + struct.pack("<h", v)
                    values[s].append(v)
                self.n += 1
            if frames:
                self.port.write(frames)
                self.on_samples(0, values[0])
                self.on_samples(1, values[1])
            time.sleep(0.001)


# ======================================================= receiver (STM32 -> PC) ===
class Receiver(QThread):
    spectrum = pyqtSignal(int, list)                           # sensor index, magnitudes

    def __init__(self, port):
        super().__init__()
        self.port, self.running = port, True

    def run(self):
        buf = bytearray()
        while self.running:
            buf += self.port.read(512)
            while True:
                hits = [(i, s) for s in range(2) if (i := buf.find(bytes([SOF, EVT_FFT[s]]))) >= 0]
                if not hits:
                    del buf[:-1]                               # keep a possible SOF
                    break
                i, s = min(hits)                               # first frame in the buffer
                if len(buf) < i + FRAME_LEN:
                    del buf[:i]                                # wait for the rest
                    break
                mags = struct.unpack(f"<{BINS}H", buf[i + 2:i + FRAME_LEN])
                del buf[:i + FRAME_LEN]
                self.spectrum.emit(s, list(mags))


# ================================================================= plot ===
class Plot(QWidget):
    def __init__(self, title, bars, color):
        super().__init__()
        self.title, self.bars, self.color, self.data = title, bars, QColor(color), []
        self.setMinimumHeight(220)

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
                p.fillRect(QRectF(10 + k * bw, 25 + h * (1 - m / top), max(bw - 1, 1), h * m / top), self.color)
            for f in range(0, FS_HZ // 2 + 1, 100):
                p.drawText(int(10 + w * f / (FS_HZ / 2)) - 12, h + 38, f"{f}")
        else:                                                  # signal: -top .. +top
            dx = w / (len(self.data) - 1)
            p.setPen(QPen(QColor("#212121"), 1))
            p.drawPolyline(QPolygonF([QPointF(10 + i * dx, 25 + h / 2 * (1 - v / top))
                                      for i, v in enumerate(self.data)]))


# ======================================================== one sensor column ===
class SensorPanel(QGroupBox):
    def __init__(self, index):
        super().__init__(f"Sensor {index + 1}   (AA {CMD_SAMPLE[index]:02X} -> FftSensor{index + 1} -> AA {EVT_FFT[index]:02X})")
        self.tones, self.frames = [], 0
        self.last = deque(maxlen=FFT_N)

        grid = QGridLayout()
        self.on, self.freq, self.amp = [], [], []
        for i, (f, a) in enumerate(DEFAULT_TONES[index]):
            on = QCheckBox(f"sine {i + 1}")
            on.setChecked(i == index)
            fr = QDoubleSpinBox(suffix=" Hz", maximum=FS_HZ / 2, value=f)
            am = QSpinBox(maximum=10000, singleStep=100, value=a)
            for sig in (on.toggled, fr.valueChanged, am.valueChanged):
                sig.connect(self.update_tones)
            grid.addWidget(on, i, 0)
            grid.addWidget(fr, i, 1)
            grid.addWidget(am, i, 2)
            self.on.append(on)
            self.freq.append(fr)
            self.amp.append(am)

        self.signal_plot = Plot("Signal sent (last 256 samples = 256 ms)", False, COLORS[index])
        self.fft_plot = Plot(f"FFT from FftSensor{index + 1} (0 .. 500 Hz, 3.9 Hz per bin)", True, COLORS[index])
        self.peaks = QLabel("Peaks: -")
        self.peaks.setWordWrap(True)
        layout = QVBoxLayout(self)
        layout.addLayout(grid)
        layout.addWidget(self.signal_plot)
        layout.addWidget(self.fft_plot)
        layout.addWidget(self.peaks)
        self.update_tones()

    def update_tones(self, *_):
        self.tones = [(self.freq[i].value(), self.amp[i].value()) for i in range(3) if self.on[i].isChecked()]

    def redraw_signal(self):
        self.signal_plot.data = list(self.last)
        self.signal_plot.update()

    def show_fft(self, mags):
        self.frames += 1
        self.fft_plot.data = mags
        self.fft_plot.update()
        top = max(mags) or 1
        peaks = [k for k in range(1, BINS - 1)
                 if mags[k] > top * 0.1 and mags[k] > mags[k - 1] and mags[k] >= mags[k + 1]]
        self.peaks.setText(f"[{self.frames} spectra]  Peaks: "
                           + "   ".join(f"{k * FS_HZ / FFT_N:.1f} Hz ({mags[k]})" for k in peaks))


# ============================================================ main window ===
class Window(QWidget):
    samples_sent = pyqtSignal(int, list)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("APK15 - two sensors, two FFT tasks")
        self.resize(1400, 800)
        self.port = self.sensors = self.receiver = None

        top = QHBoxLayout()
        self.ports = QComboBox()
        self.ports.addItems([p.device for p in serial.tools.list_ports.comports()])
        self.start_btn = QPushButton("Start")
        self.start_btn.clicked.connect(self.start_stop)
        top.addWidget(QLabel("Port"))
        top.addWidget(self.ports)
        top.addWidget(self.start_btn)
        top.addStretch()

        self.panels = [SensorPanel(0), SensorPanel(1)]
        cols = QHBoxLayout()
        for panel in self.panels:
            cols.addWidget(panel)
        layout = QVBoxLayout(self)
        layout.addLayout(top)
        layout.addLayout(cols)

        self.samples_sent.connect(lambda s, values: self.panels[s].last.extend(values))
        timer = QTimer(self)
        timer.timeout.connect(lambda: [panel.redraw_signal() for panel in self.panels])
        timer.start(50)

    def start_stop(self):
        if self.sensors:
            self.sensors.running = self.receiver.running = False
            self.sensors.join(0.5)
            self.receiver.wait(500)
            self.port.close()
            self.sensors = self.receiver = self.port = None
            self.start_btn.setText("Start")
            return
        self.port = serial.Serial(self.ports.currentText(), 115200, timeout=0.02)
        self.receiver = Receiver(self.port)
        self.receiver.spectrum.connect(lambda s, mags: self.panels[s].show_fft(mags))
        self.receiver.start()
        self.sensors = Sensors(self.port, lambda s: self.panels[s].tones, self.samples_sent.emit)
        self.sensors.start()
        self.start_btn.setText("Stop")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = Window()
    win.show()
    sys.exit(app.exec_())
