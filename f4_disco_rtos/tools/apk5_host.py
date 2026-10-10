"""APK5 host: talks to the STM32 over UART using the SOF-CMD-DATA protocol.

Frame (3 bytes):  SOF(0xAA) | CMD | DATA
Response:         SOF(0xAA) | CMD|0x80 | STATUS/DATA

The board also prints ASCII log text on the same UART, so the reader skips
everything until it sees SOF (0xAA never appears in ASCII text).

Usage:
    pip install pyserial
    python apk5_host.py COM5
"""
import sys
import time

import serial

SOF = 0xAA
RSP_FLAG = 0x80

CMD_PING = 0x01
CMD_LED_SET = 0x02
CMD_LED_TOGGLE = 0x03
CMD_GET_BUTTON = 0x04

STATUS = {0x00: "OK", 0xE1: "BAD_CMD", 0xE2: "BAD_DATA"}


class Link:
    def __init__(self, port, baud=115200, timeout=0.5):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self.log = bytearray()

    def send(self, cmd, data=0):
        self.ser.write(bytes([SOF, cmd & 0xFF, data & 0xFF]))

    def receive(self):
        """Return (cmd, data) of the next response frame, or None on timeout."""
        deadline = time.monotonic() + self.ser.timeout
        while time.monotonic() < deadline:
            b = self.ser.read(1)
            if not b:
                continue
            if b[0] != SOF:
                self._log_byte(b[0])
                continue
            rest = self.ser.read(2)
            if len(rest) == 2:
                return rest[0], rest[1]
        return None

    def request(self, cmd, data=0):
        self.send(cmd, data)
        rsp = self.receive()
        if rsp is None:
            raise TimeoutError(f"no response to CMD 0x{cmd:02X}")
        rsp_cmd, rsp_data = rsp
        if rsp_cmd != (cmd | RSP_FLAG):
            raise ValueError(f"unexpected response CMD 0x{rsp_cmd:02X}")
        return rsp_data

    def _log_byte(self, byte):
        # Echo the board's log lines so they stay visible on the PC side.
        if byte == 0x0A:
            print("  [board]", self.log.decode(errors="replace").strip())
            self.log.clear()
        else:
            self.log.append(byte)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM5"
    link = Link(port)
    time.sleep(0.2)
    link.ser.reset_input_buffer()

    print("PING 0x55 ->", hex(link.request(CMD_PING, 0x55)))

    for mask in (0x1, 0x2, 0x4, 0x8, 0xF, 0x0):
        st = link.request(CMD_LED_SET, mask)
        print(f"LED_SET 0x{mask:X} ->", STATUS.get(st, hex(st)))
        time.sleep(0.3)

    for i in range(4):
        st = link.request(CMD_LED_TOGGLE, i)
        print(f"LED_TOGGLE {i} ->", STATUS.get(st, hex(st)))

    st = link.request(CMD_LED_TOGGLE, 7)
    print("LED_TOGGLE 7 (invalid) ->", STATUS.get(st, hex(st)))

    st = link.request(0x42, 0)
    print("CMD 0x42 (unknown) ->", STATUS.get(st, hex(st)))

    print("BUTTON ->", "pressed" if link.request(CMD_GET_BUTTON) else "released")


if __name__ == "__main__":
    main()
