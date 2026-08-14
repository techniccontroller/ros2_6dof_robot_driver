import math
import queue
import time

from pico_6dof_robot_driver.serial_robot import SerialRobot


class FakeSerial:
    def __init__(self, port, baudrate, timeout):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.is_open = True
        self.lines = queue.Queue()
        self.writes = []

    def readline(self):
        try:
            return self.lines.get(timeout=self.timeout)
        except queue.Empty:
            return b""

    def write(self, payload):
        self.writes.append(payload)

    def flush(self):
        pass

    def close(self):
        self.is_open = False


def test_reader_keeps_latest_telemetry_and_writer_terminates_lines():
    ports = []

    def factory(*args, **kwargs):
        port = FakeSerial(*args, **kwargs)
        ports.append(port)
        return port

    robot = SerialRobot("TEST", timeout=0.01, serial_factory=factory)
    robot.connect()
    ports[0].lines.put(b'{"robot_data":{"config":[0,90,0,0,0,0]}}\n')
    deadline = time.monotonic() + 0.5
    telemetry = None
    while telemetry is None and time.monotonic() < deadline:
        telemetry, _ = robot.latest_telemetry()
        time.sleep(0.005)
    robot.send("GRIP_OPEN")
    robot.disconnect()

    assert telemetry is not None
    assert telemetry.positions_rad[1] == math.pi / 2
    assert ports[0].writes == [b"GRIP_OPEN\n"]
    assert not robot.is_connected


def test_reader_forwards_non_telemetry_firmware_lines():
    ports = []
    received_lines = []

    def factory(*args, **kwargs):
        port = FakeSerial(*args, **kwargs)
        ports.append(port)
        return port

    robot = SerialRobot(
        "TEST",
        timeout=0.01,
        serial_factory=factory,
        line_callback=received_lines.append,
    )
    robot.connect()
    ports[0].lines.put(b"VEL_CONFIG is set\n")
    deadline = time.monotonic() + 0.5
    while not received_lines and time.monotonic() < deadline:
        time.sleep(0.005)
    robot.disconnect()

    assert received_lines == ["VEL_CONFIG is set"]
