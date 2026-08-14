"""Thread-safe serial connection to the Pico firmware."""

from __future__ import annotations

import threading
import time
from typing import Callable

from .protocol import FirmwareProtocol, RobotTelemetry


class SerialRobot:
    """Own the serial port, latest telemetry, and firmware command pacing."""

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        timeout: float = 0.05,
        protocol: FirmwareProtocol | None = None,
        serial_factory: Callable[..., object] | None = None,
        line_callback: Callable[[str], None] | None = None,
    ) -> None:
        self.port = port
        self.baudrate = int(baudrate)
        self.timeout = float(timeout)
        self.protocol = protocol or FirmwareProtocol()
        self._serial_factory = serial_factory
        self._line_callback = line_callback
        self._serial = None
        self._reader_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._write_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._latest: RobotTelemetry | None = None
        self._latest_monotonic = 0.0
        self._last_error: Exception | None = None

    @property
    def is_connected(self) -> bool:
        return self._serial is not None and bool(getattr(self._serial, "is_open", True))

    @property
    def last_error(self) -> Exception | None:
        return self._last_error

    def connect(self) -> None:
        if self.is_connected:
            return
        if self._serial_factory is None:
            import serial

            factory = serial.Serial
        else:
            factory = self._serial_factory
        self._serial = factory(self.port, self.baudrate, timeout=self.timeout)
        self._stop_event.clear()
        self._last_error = None
        with self._state_lock:
            self._latest = None
            self._latest_monotonic = 0.0
        self._reader_thread = threading.Thread(target=self._read_loop, name="pico-robot-reader", daemon=True)
        self._reader_thread.start()

    def disconnect(self) -> None:
        self._stop_event.set()
        serial_port = self._serial
        self._serial = None
        if serial_port is not None:
            try:
                serial_port.close()
            except Exception:
                pass
        if self._reader_thread is not None and self._reader_thread is not threading.current_thread():
            self._reader_thread.join(timeout=max(0.2, self.timeout * 3.0))
        self._reader_thread = None

    def latest_telemetry(self) -> tuple[RobotTelemetry | None, float]:
        """Return the latest record and its monotonic receive timestamp."""
        with self._state_lock:
            return self._latest, self._latest_monotonic

    def send(self, command: str) -> None:
        payload = self.protocol.encode(command)
        if len(payload) - 1 > self.protocol.max_command_bytes:
            raise ValueError(f"command exceeds firmware limit: {command!r}")
        with self._write_lock:
            if not self.is_connected:
                raise RuntimeError("robot serial port is not connected")
            self._serial.write(payload)
            self._serial.flush()

    def send_configuration(self, positions_rad, velocity_rad_s: float) -> None:
        self.send(self.protocol.configuration_command(positions_rad, velocity_rad_s))

    def hold(self) -> bool:
        telemetry, _ = self.latest_telemetry()
        if telemetry is None:
            return False
        self.send_configuration(telemetry.positions_rad, velocity_rad_s=math_radians(10.0))
        return True

    def _read_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                serial_port = self._serial
                if serial_port is None:
                    return
                line = serial_port.readline()
                if not line:
                    continue
                telemetry = self.protocol.parse_telemetry(line)
                if telemetry is not None:
                    with self._state_lock:
                        self._latest = telemetry
                        self._latest_monotonic = time.monotonic()
                elif self._line_callback is not None:
                    if isinstance(line, bytes):
                        text = line.decode("ascii", errors="replace").strip()
                    else:
                        text = str(line).strip()
                    if text:
                        try:
                            self._line_callback(text)
                        except Exception:
                            # Diagnostics must never terminate serial reception.
                            pass
            except Exception as exc:
                if not self._stop_event.is_set():
                    self._last_error = exc
                serial_port = self._serial
                self._serial = None
                if serial_port is not None:
                    try:
                        serial_port.close()
                    except Exception:
                        pass
                return


def math_radians(degrees: float) -> float:
    # Kept local so this module remains tiny and easy to mock in tests.
    return float(degrees) * 0.017453292519943295
