import serial
import serial.tools.list_ports
import time


FILE_NAME = "./output.txt"
GPS_DEVICE_NAME = "Silicon Labs CP210x USB to UART Bridge"
BAUD = 115200
STREAM_DELAY = 0.5


def stream_file(file_name, port, baud, stream_delay=0.5):
    with (
        open(file_name, 'r') as file,
        serial.Serial(port, baudrate=baud) as ser
    ):
        time.sleep(1)
        for line in file.readline():
            ser.write(line.encode())
            time.sleep(stream_delay)


def get_serial_port_by_device_name(device_name):
    for port in serial.tools.list_ports.comports():
        if port.description.startswith(device_name):
            return port.name


def main():
    gps_device_port = get_serial_port_by_device_name(GPS_DEVICE_NAME)
    stream_file(FILE_NAME, gps_device_port, BAUD, STREAM_DELAY)


if __name__ == '__main__':
    main()
