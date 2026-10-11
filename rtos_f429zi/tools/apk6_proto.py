"""APK6 protocol constants and a software model of the STM32 firmware (for use without a board)."""
import queue
import threading
import time

SOF = 0xAA
RSP_FLAG = 0x80
FRAME_TIMEOUT_S = 0.05

# Requests (host -> STM32), answered with CMD | 0x80
CMD_PING = 0x01
CMD_CFG_PROGRAM = 0x10
CMD_CFG_TEMP = 0x11
CMD_CFG_SPIN = 0x12
CMD_START_CYCLE = 0x20
CMD_GET_STATUS = 0x21

# Events (STM32 -> host)
EVT_STEP = 0x30
EVT_PROGRESS = 0x31
EVT_DONE = 0x32
EVT_DOOR = 0x33
EVT_ERROR = 0x34

ST_OK = 0x00
ST_BAD_CMD = 0xE1
ST_BAD_DATA = 0xE2
ST_BUSY = 0xE3
ST_NOT_CONFIG = 0xE4
ST_DOOR_OPEN = 0xE5

ERR_DOOR_OPEN = 0x01

# SystemTask states (GET_STATUS bits 0..3) and SensorTask states (GET_STATUS bit 7)
IDLE_STATE, READY_STATE, RUN_STATE = 0, 1, 2
NO_ERROR_STATE, ERROR_STATE = 0, 1
STATUS_ERROR_FLAG = 0x80

CMD_NAMES = {
    CMD_PING: "PING", CMD_CFG_PROGRAM: "CFG_PROGRAM", CMD_CFG_TEMP: "CFG_TEMP",
    CMD_CFG_SPIN: "CFG_SPIN", CMD_START_CYCLE: "START_CYCLE", CMD_GET_STATUS: "GET_STATUS",
}
EVT_NAMES = {
    EVT_STEP: "EVT_STEP", EVT_PROGRESS: "EVT_PROGRESS", EVT_DONE: "EVT_DONE",
    EVT_DOOR: "EVT_DOOR", EVT_ERROR: "EVT_ERROR",
}
STATUS_NAMES = {
    ST_OK: "OK", ST_BAD_CMD: "BAD_CMD", ST_BAD_DATA: "BAD_DATA", ST_BUSY: "BUSY (cycle running)",
    ST_NOT_CONFIG: "NOT_CONFIGURED", ST_DOOR_OPEN: "DOOR_OPEN",
}
STATE_NAMES = {IDLE_STATE: "IDLE_STATE", READY_STATE: "READY_STATE", RUN_STATE: "RUN_STATE"}

PROGRAMS = ["Cotton", "Synthetic", "Wool", "Quick"]
TEMPS = [20, 30, 40, 60, 90]
SPINS = [4, 6, 8, 10, 12, 14]          # x 100 rpm
STEPS = ["Fill", "Heat", "Wash", "Rinse", "Drain", "Spin"]
STEP_TIME_S = 10.0


class SimBoard:
    """Software twin of apk6.c with a pyserial-like API (write / read / close).

    press_button() plays the role of the user button B1 (door sensor).
    """

    def __init__(self, step_time=STEP_TIME_S):
        self.step_time = step_time
        self.out = bytearray()
        self.lock = threading.Lock()
        self.frames = queue.Queue(8)       # QueueFrame
        self.cmds = queue.Queue(4)         # QueueCmd
        self.state = IDLE_STATE
        self.sensor = NO_ERROR_STATE
        self.suspended_by_sensor = False
        self.not_suspended = threading.Event()
        self.not_suspended.set()
        self.rx_state, self.rx_cmd, self.last = 0, 0, 0.0
        self.alive = True
        for fn in (self._link_layer, self._system_task):
            threading.Thread(target=fn, daemon=True).start()

    # ---- pyserial-like API
    def write(self, data):
        for byte in bytes(data):
            self._isr(byte)

    def read(self, n):
        time.sleep(0.005)
        with self.lock:
            chunk = bytes(self.out[:n])
            del self.out[:n]
        return chunk

    def close(self):
        self.alive = False

    # ---- firmware model
    def _send(self, cmd, data):
        with self.lock:
            self.out += bytes([SOF, cmd & 0xFF, data & 0xFF])

    def _log(self, text):
        with self.lock:
            self.out += f"\x1b[36m{text}\x1b[0m\r\n".encode()

    def _isr(self, byte):
        now = time.perf_counter()
        if self.rx_state and now - self.last > FRAME_TIMEOUT_S:
            self.rx_state = 0
        self.last = now
        if self.rx_state == 0:
            self.rx_state = 1 if byte == SOF else 0
        elif self.rx_state == 1:
            self.rx_cmd, self.rx_state = byte, 2
        else:
            try:
                self.frames.put_nowait((self.rx_cmd, byte))
            except queue.Full:
                pass
            self.rx_state = 0

    def _link_layer(self):
        while self.alive:
            cmd, data = self.frames.get()
            self._log(f"RX frame CMD=0x{cmd:02X} DATA=0x{data:02X}")
            if cmd == CMD_PING:
                self._send(cmd | RSP_FLAG, data)
            elif cmd == CMD_GET_STATUS:
                self._send(cmd | RSP_FLAG, self.state | (STATUS_ERROR_FLAG if self.sensor == ERROR_STATE else 0))
            elif cmd in (CMD_CFG_PROGRAM, CMD_CFG_TEMP, CMD_CFG_SPIN, CMD_START_CYCLE):
                if self.state == RUN_STATE:
                    self._send(cmd | RSP_FLAG, ST_BUSY)
                else:
                    try:
                        self.cmds.put_nowait((cmd, data))
                    except queue.Full:
                        self._send(cmd | RSP_FLAG, ST_BUSY)
            else:
                self._send(cmd | RSP_FLAG, ST_BAD_CMD)

    def _system_task(self):
        cfg, mask = {}, 0
        while self.alive:
            if self.state == RUN_STATE:
                self._run_cycle()
                self.state = READY_STATE
                self._log("Cycle done -> READY_STATE")
                self._send(EVT_DONE, 0)
                continue
            cmd, data = self.cmds.get()
            if cmd == CMD_START_CYCLE:
                if self.state == IDLE_STATE:
                    self._send(cmd | RSP_FLAG, ST_NOT_CONFIG)
                elif self.sensor == ERROR_STATE:
                    self._send(cmd | RSP_FLAG, ST_DOOR_OPEN)
                else:
                    self._send(cmd | RSP_FLAG, ST_OK)
                    self._log(f"Cycle start: {PROGRAMS[cfg['p']]}, {cfg['t']}C, {cfg['s'] * 100} rpm -> RUN_STATE")
                    self.state = RUN_STATE
                continue
            status = ST_OK
            if cmd == CMD_CFG_PROGRAM and data <= 3:
                cfg["p"], mask = data, mask | 1
            elif cmd == CMD_CFG_TEMP and data in TEMPS:
                cfg["t"], mask = data, mask | 2
            elif cmd == CMD_CFG_SPIN and 4 <= data <= 14:
                cfg["s"], mask = data, mask | 4
            else:
                status = ST_BAD_DATA
            self._send(cmd | RSP_FLAG, status)
            if self.state == IDLE_STATE and mask == 0x07:
                self._log("Configured -> READY_STATE")
                self.state = READY_STATE

    def _run_cycle(self):
        tick, ticks = 0.1, int(round(self.step_time / 0.1))
        for step in range(1, len(STEPS) + 1):
            self._send(EVT_STEP, step)
            self._log(f"Step {step}: {STEPS[step - 1]}")
            for t in range(1, ticks + 1):
                time.sleep(tick)
                self.not_suspended.wait()          # vTaskSuspend / vTaskResume
                while not self.cmds.empty():
                    cmd, _ = self.cmds.get_nowait()
                    self._send(cmd | RSP_FLAG, ST_BUSY)
                if t % 2 == 0:
                    self._send(EVT_PROGRESS, t * 100 // ticks)

    def press_button(self):
        """SensorTask state machine for one debounced press of B1."""
        if self.sensor == NO_ERROR_STATE:                 # door closed -> open
            if self.state == RUN_STATE:
                self.not_suspended.clear()                # vTaskSuspend(SystemTask)
                self.suspended_by_sensor = True
            self._send(EVT_DOOR, 1)
            if self.suspended_by_sensor:
                self._send(EVT_ERROR, ERR_DOOR_OPEN)
            self.sensor = ERROR_STATE
        else:                                             # door open -> closed
            self.sensor = NO_ERROR_STATE
            self._send(EVT_DOOR, 0)
            if self.suspended_by_sensor:
                self.suspended_by_sensor = False
                self.not_suspended.set()                  # vTaskResume(SystemTask)
