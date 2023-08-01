import serial
import time
import pathlib
import serial.tools.list_ports


class ClawGPSSimulator:
    GPS_DEVICE_NAME = "Silicon Labs CP210x USB to UART Bridge"
    BAUD = 115200
    PORT: serial.Serial
    PORT_INITIALIZATION_DELAY_SECONDS = 1.0

    def __init__(self, port_name=None):
        if port_name is None:
            port_name = self.detect_device_port()
        self.PORT = serial.Serial(port=port_name, baudrate=self.BAUD)

    def detect_device_port(self):
        for port in serial.tools.list_ports.comports():
            if port.description.startswith(self.GPS_DEVICE_NAME):
                return port.name

    def stream_file(self, command_filepath: pathlib.Path, *, stream_delay_seconds=0.5):
        with (
            open(command_filepath, 'r') as file,
            self.PORT as ser
        ):
            time.sleep(self.PORT_INITIALIZATION_DELAY_SECONDS)
            for line in file.readlines():
                encoded_command = f"{line.strip()}\r".encode()
                ser.write(encoded_command)
                time.sleep(stream_delay_seconds)

    def stream_list_of_commands(self, commands: list[str],  *, stream_delay_seconds=0.5):
        with self.PORT as ser:
            time.sleep(self.PORT_INITIALIZATION_DELAY_SECONDS)
            for command in commands:
                encoded_command = f"{command}\r".encode()
                ser.write(encoded_command)
                time.sleep(stream_delay_seconds)

    def send_command(self, command: str):
        with self.PORT as ser:
            time.sleep(self.PORT_INITIALIZATION_DELAY_SECONDS)
            ser.flushInput()
            encoded_command = f"{command}\r".encode()
            ser.write(encoded_command)
            bytes_to_read = ser.inWaiting()
            response_encoded = ser.read(bytes_to_read)
        response_decoded = response_encoded.decode()
        return response_decoded

