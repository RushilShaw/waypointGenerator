import GpsUtils
import pathlib


def main():
    initialization_commands = [
        "SIM:COM STOP",
        "SIM:POS:MOTION:ZEROIZE"
    ]
    command_file = pathlib.Path("commands.txt")
    final_commands = [
        "SIM:POS:MOTION:READ 1",
        "SIM:POS:MODE MOTION",
        "SIM:COM START"
    ]

    gps = GpsUtils.ClawGPSSimulator()

    gps.stream_list_of_commands(initialization_commands)
    gps.stream_file(command_file)
    gps.stream_list_of_commands(final_commands)


if __name__ == '__main__':
    main()
