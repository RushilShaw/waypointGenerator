import GpsUtils
import pathlib

COMMANDS_FILE = pathlib.Path("commands.txt")


def main():
    gps = GpsUtils.ClawGPSSimulator()
    gps.stream_file(COMMANDS_FILE)
    start_simulation_command = "SIM:COM START"
    gps.send_command(start_simulation_command)


if __name__ == '__main__':
    main()
