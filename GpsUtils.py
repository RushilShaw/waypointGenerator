import serial
import time
import pathlib
import serial.tools.list_ports


class ClawGPSSimulator:
    GPS_DEVICE_NAME = "Silicon Labs CP210x USB to UART Bridge"
    BAUD = 115200
    PORT: serial.Serial

    def __init__(self, com_port=None):
        if com_port is None:
            com_port = self.detect_device_port()
        self.PORT = serial.Serial(port=com_port, baudrate=self.BAUD)

    def detect_device_port(self):
        for port in serial.tools.list_ports.comports():
            if port.description.startswith(self.GPS_DEVICE_NAME):
                return port.name

    def stream_file(self, command_filepath: pathlib.Path, *, stream_delay_seconds=0.5):
        with (
            open(command_filepath, 'r') as file,
            self.PORT as ser
        ):
            time.sleep(1)
            for line in file.readline():
                ser.write(line.encode())
                time.sleep(stream_delay_seconds)

    def stream_list_of_commands(self, commands: list[str],  *, stream_delay_seconds=0.5):
        with self.PORT as ser:
            time.sleep(1)
            for command in commands:
                ser.write(command.encode())
                time.sleep(stream_delay_seconds)

    def send_command(self, command: str):
        with self.PORT as ser:
            time.sleep(1)
            ser.flushInput()
            ser.write(command.encode())
            bytes_to_read = ser.inWaiting()
            response_encoded = ser.read(bytes_to_read)
        response_decoded = response_encoded.decode()
        return response_decoded

