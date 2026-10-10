"""APK5 GUI: send SOF-CMD-DATA frames to the STM32 and show what comes back.

Frame (3 bytes):  SOF(0xAA) | CMD | DATA
Response:         SOF(0xAA) | CMD|0x80 | STATUS/DATA

The board also prints ASCII log text on the same UART. The receiver runs the
same state machine as the firmware (WAIT_SOF / WAIT_CMD / WAIT_DATA); bytes
seen while waiting for SOF are collected as board log text.

Usage:
    pip install pyqt5 pyserial
    python apk5_gui.py
"""
import html
import re
import sys
import time

import serial
import serial.tools.list_ports
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QGridLayout, QGroupBox, QHBoxLayout,
    QLabel, QLineEdit, QMainWindow, QPushButton, QSpinBox, QTextEdit,
    QVBoxLayout, QWidget,
)

SOF = 0xAA
RSP_FLAG = 0x80
FRAME_TIMEOUT_S = 0.05      # same inter-byte timeout as the firmware

CMD_PING = 0x01
CMD_LED_SET = 0x02
CMD_LED_TOGGLE = 0x03
CMD_GET_BUTTON = 0x04

CMD_NAMES = {
    CMD_PING: "PING",
    CMD_LED_SET: "LED_SET",
    CMD_LED_TOGGLE: "LED_TOGGLE",
    CMD_GET_BUTTON: "GET_BUTTON",
}
STATUS = {0x00: "OK", 0xE1: "BAD_CMD", 0xE2: "BAD_DATA"}

# LED index -> (pin, colour on the F4 Discovery board)
LEDS = [("PD12", "green"), ("PD13", "orange"), ("PD14", "red"), ("PD15", "blue")]

ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")

WAIT_SOF, WAIT_CMD, WAIT_DATA = range(3)


class SerialReader(QThread):
    """Reads the UART and splits the stream into response frames and log text."""

    frame_received = pyqtSignal(int, int)   # cmd, data
    text_received = pyqtSignal(str)         # one board log line
    error = pyqtSignal(str)

    def __init__(self, ser):
        super().__init__()
        self.ser = ser
        self.running = True

    def run(self):
        state = WAIT_SOF
        cmd = 0
        last = 0.0
        line = bytearray()

        while self.running:
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except serial.SerialException as exc:
                self.error.emit(str(exc))
                return
            for byte in chunk:
                now = time.monotonic()
                if state != WAIT_SOF and now - last > FRAME_TIMEOUT_S:
                    state = WAIT_SOF
                last = now

                if state == WAIT_SOF:
                    if byte == SOF:
                        state = WAIT_CMD
                    elif byte == 0x0A:
                        text = ANSI_RE.sub("", line.decode(errors="replace")).strip()
                        if text:
                            self.text_received.emit(text)
                        line.clear()
                    elif byte != 0x0D:
                        line.append(byte)
                elif state == WAIT_CMD:
                    cmd = byte
                    state = WAIT_DATA
                else:
                    self.frame_received.emit(cmd, byte)
                    state = WAIT_SOF

    def stop(self):
        self.running = False
        self.wait(500)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("APK5 - STM32 UART protocol")
        self.resize(820, 600)
        self.ser = None
        self.reader = None

        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)

        controls = QVBoxLayout()
        root.addLayout(controls, 0)
        root.addWidget(self._build_log(), 1)

        controls.addWidget(self._build_connection())
        controls.addWidget(self._build_ping())
        controls.addWidget(self._build_leds())
        controls.addWidget(self._build_button())
        controls.addWidget(self._build_custom())
        controls.addStretch()

        self._set_connected(False)

    # ---------------------------------------------------------------- UI ---
    def _build_connection(self):
        box = QGroupBox("Connection")
        lay = QGridLayout(box)
        self.port_combo = QComboBox()
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["115200", "57600", "38400", "19200", "9600"])
        refresh = QPushButton("Refresh")
        refresh.clicked.connect(self.refresh_ports)
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)

        lay.addWidget(QLabel("Port"), 0, 0)
        lay.addWidget(self.port_combo, 0, 1)
        lay.addWidget(refresh, 0, 2)
        lay.addWidget(QLabel("Baud"), 1, 0)
        lay.addWidget(self.baud_combo, 1, 1)
        lay.addWidget(self.connect_btn, 1, 2)
        self.refresh_ports()
        return box

    def _build_ping(self):
        box = QGroupBox("PING (0x01)")
        lay = QHBoxLayout(box)
        self.ping_data = QSpinBox()
        self.ping_data.setRange(0, 255)
        self.ping_data.setDisplayIntegerBase(16)
        self.ping_data.setPrefix("0x")
        self.ping_data.setValue(0x55)
        btn = QPushButton("Send PING")
        btn.clicked.connect(lambda: self.send_frame(CMD_PING, self.ping_data.value()))
        lay.addWidget(QLabel("Data"))
        lay.addWidget(self.ping_data)
        lay.addWidget(btn)
        self.cmd_buttons = [btn]
        return box

    def _build_leds(self):
        box = QGroupBox("LEDs")
        lay = QGridLayout(box)
        self.led_checks = []
        for i, (pin, colour) in enumerate(LEDS):
            chk = QCheckBox(f"LED{i} {pin}")
            chk.setStyleSheet(f"QCheckBox {{ color: {colour}; font-weight: bold; }}")
            self.led_checks.append(chk)
            lay.addWidget(chk, 0, i)

            tog = QPushButton(f"Toggle {i}")
            tog.clicked.connect(lambda _, idx=i: self.send_frame(CMD_LED_TOGGLE, idx))
            lay.addWidget(tog, 1, i)
            self.cmd_buttons.append(tog)

        set_btn = QPushButton("LED_SET (0x02) from checkboxes")
        set_btn.clicked.connect(self.send_led_set)
        all_on = QPushButton("All ON")
        all_on.clicked.connect(lambda: self.send_frame(CMD_LED_SET, 0x0F))
        all_off = QPushButton("All OFF")
        all_off.clicked.connect(lambda: self.send_frame(CMD_LED_SET, 0x00))
        lay.addWidget(set_btn, 2, 0, 1, 2)
        lay.addWidget(all_on, 2, 2)
        lay.addWidget(all_off, 2, 3)
        self.cmd_buttons += [set_btn, all_on, all_off]
        return box

    def _build_button(self):
        box = QGroupBox("User button (0x04)")
        lay = QHBoxLayout(box)
        btn = QPushButton("Read button")
        btn.clicked.connect(lambda: self.send_frame(CMD_GET_BUTTON, 0))
        self.button_state = QLabel("?")
        self.button_state.setStyleSheet("font-weight: bold;")
        lay.addWidget(btn)
        lay.addWidget(self.button_state)
        lay.addStretch()
        self.cmd_buttons.append(btn)
        return box

    def _build_custom(self):
        box = QGroupBox("Custom frame (hex)")
        lay = QHBoxLayout(box)
        self.custom_cmd = QLineEdit("42")
        self.custom_data = QLineEdit("00")
        for edit in (self.custom_cmd, self.custom_data):
            edit.setMaxLength(2)
            edit.setFixedWidth(40)
            edit.setInputMask("HH")
        btn = QPushButton("Send")
        btn.clicked.connect(self.send_custom)
        lay.addWidget(QLabel("AA |"))
        lay.addWidget(self.custom_cmd)
        lay.addWidget(QLabel("|"))
        lay.addWidget(self.custom_data)
        lay.addWidget(btn)
        lay.addStretch()
        self.cmd_buttons.append(btn)
        return box

    def _build_log(self):
        box = QGroupBox("Log")
        lay = QVBoxLayout(box)
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setFont(QFont("Consolas", 9))
        self.show_board_log = QCheckBox("Show board printf text")
        self.show_board_log.setChecked(True)
        clear = QPushButton("Clear")
        clear.clicked.connect(self.log_view.clear)
        bottom = QHBoxLayout()
        bottom.addWidget(self.show_board_log)
        bottom.addStretch()
        bottom.addWidget(clear)
        lay.addWidget(self.log_view)
        lay.addLayout(bottom)
        return box

    # -------------------------------------------------------- connection ---
    def refresh_ports(self):
        self.port_combo.clear()
        for p in serial.tools.list_ports.comports():
            self.port_combo.addItem(f"{p.device} - {p.description}", p.device)

    def toggle_connection(self):
        if self.ser:
            self.disconnect_port()
            return
        port = self.port_combo.currentData()
        if not port:
            self.log("No serial port selected", "red")
            return
        try:
            self.ser = serial.Serial(port, int(self.baud_combo.currentText()), timeout=0.1)
        except serial.SerialException as exc:
            self.log(f"Cannot open {port}: {exc}", "red")
            self.ser = None
            return
        self.reader = SerialReader(self.ser)
        self.reader.frame_received.connect(self.on_frame)
        self.reader.text_received.connect(self.on_text)
        self.reader.error.connect(self.on_error)
        self.reader.start()
        self.log(f"Connected to {port}", "gray")
        self._set_connected(True)

    def disconnect_port(self):
        if self.reader:
            self.reader.stop()
            self.reader = None
        if self.ser:
            self.ser.close()
            self.ser = None
        self.log("Disconnected", "gray")
        self._set_connected(False)

    def _set_connected(self, connected):
        self.connect_btn.setText("Disconnect" if connected else "Connect")
        self.port_combo.setEnabled(not connected)
        self.baud_combo.setEnabled(not connected)
        for btn in self.cmd_buttons:
            btn.setEnabled(connected)

    def closeEvent(self, event):
        if self.ser:
            self.disconnect_port()
        event.accept()

    # ------------------------------------------------------------- TX -----
    def send_frame(self, cmd, data):
        if not self.ser:
            return
        frame = bytes([SOF, cmd & 0xFF, data & 0xFF])
        try:
            self.ser.write(frame)
        except serial.SerialException as exc:
            self.on_error(str(exc))
            return
        name = CMD_NAMES.get(cmd, "UNKNOWN")
        self.log(f"TX  {frame.hex(' ').upper()}   {name} data=0x{data:02X}", "#1565c0")

    def send_led_set(self):
        mask = sum(1 << i for i, chk in enumerate(self.led_checks) if chk.isChecked())
        self.send_frame(CMD_LED_SET, mask)

    def send_custom(self):
        try:
            cmd = int(self.custom_cmd.text() or "0", 16)
            data = int(self.custom_data.text() or "0", 16)
        except ValueError:
            self.log("Invalid hex value", "red")
            return
        self.send_frame(cmd, data)

    # ------------------------------------------------------------- RX -----
    def on_frame(self, cmd, data):
        raw = f"{SOF:02X} {cmd:02X} {data:02X}"
        if not cmd & RSP_FLAG:
            self.log(f"RX  {raw}   not a response (bit7 of CMD not set)", "red")
            return

        req = cmd & ~RSP_FLAG
        name = CMD_NAMES.get(req, f"0x{req:02X}")
        if req == CMD_PING:
            ok = data == self.ping_data.value()
            text = f"PING echo=0x{data:02X}" + ("" if ok else " (mismatch!)")
        elif req == CMD_GET_BUTTON:
            state = "PRESSED" if data else "released"
            self.button_state.setText(state)
            text = f"GET_BUTTON -> {state}"
        else:
            text = f"{name} -> {STATUS.get(data, f'0x{data:02X}')}"

        colour = "red" if data in (0xE1, 0xE2) and req != CMD_PING else "#2e7d32"
        self.log(f"RX  {raw}   {text}", colour)

    def on_text(self, text):
        if self.show_board_log.isChecked():
            self.log(f"    [board] {text}", "gray")

    def on_error(self, msg):
        self.log(f"Serial error: {msg}", "red")
        self.disconnect_port()

    def log(self, text, colour="black"):
        stamp = time.strftime("%H:%M:%S")
        self.log_view.append(
            f'<span style="color:{colour}; white-space:pre">{stamp}  {html.escape(text)}</span>'
        )


def main():
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
