import LaunchBoxProtocol as lbp
from serial.tools import list_ports
import struct
import sys
import math

BAUD = 38400

HELP = """This is a simple interface for configuring the SPR electronics.
Supported commands are:
help: print this help page
quit: quits the interface
list: print the list of available keys
get {keyname}: gets a key from the SRP board. See the list command for a list of keys
set {keyname} {value}: sets a key on the SRP board. See the list command for a list of keys"""


# calculation constants
REF_VOLT = 1.275
TIME_DELTA = 1/50
RESOLUTION = 5 / 256 # (5 V-ref) / (8-bit of the ADC)
VDIV = 2 # Voltage divider

PACKET_NAMES = {
    "min_deploy_time"      : 0x00,
    "max_deploy_time"      : 0x01,
    "measured_deploy_time" : 0x02,
    "battery_voltage"      : 0x03,
    "battery_empty_limit"  : 0x04,
    "deploy_mode"          : 0x05,
    "servo_closed_position": 0x06,
    "servo_open_position"  : 0x07,
    "servo_position"       : 0x08,
    "address"              : 0x09
}

PACKET_NUMBERS = {num: name for name, num in PACKET_NAMES.items()}

PACKET_SIZE = {
    0x00: "<H",
    0x01: "<H",
    0x02: "<H",
    0x03: "<B",
    0x04: "<B",
    0x05: "<B",
    0x06: "<B",
    0x07: "<B",
    0x08: "<B",
    0x09: "<B"
}

# parsing functions

def val_int(value):
    try:
        return int(value)
    except ValueError:
        raise SyntaxError("Parameter has to be an integer")

def val_float(value):
    try:
        return float(value)
    except ValueError:
        raise SyntaxError("Parameter has to be a number")

def parse_time(seconds):
    seconds = val_float(seconds)
    return int(seconds / TIME_DELTA)

def revparse_time(ticks):
    return ticks * TIME_DELTA

def parse_voltage(voltage):
    voltage = val_float(voltage) - 0.4
    return int(voltage / VDIV / RESOLUTION)

def revparse_voltage(value):
    if not value:
        return "Unknown"
    return value * RESOLUTION * VDIV + 0.4

def parse_deploy_mode(mode):
    try:
        return val_int(mode)
    except SyntaxError:
        if mode == "servo":
            return 1
        elif mode == "pyro":
            return 0
        else:
            raise SyntaxError("Unknown mode {}".format(mode))

def revparse_deploy_mode(mode):
    return "servo" if mode else "pyro"

PARSE_FUN = {
    0x00: (parse_time, revparse_time),
    0x01: (parse_time, revparse_time),
    0x02: (parse_time, revparse_time),
    0x03: (parse_voltage, revparse_voltage),
    0x04: (parse_voltage, revparse_voltage),
    0x05: (parse_deploy_mode, revparse_deploy_mode),
}


class CommsPort(lbp.CommsPortSerial):
    # Just parse them immediately in the other thread
    def PacketParserHandler(self, data):
        self.rxPacket(data)


class PacketHandler:
    def __init__(self, device):
        self.device = device
        device.setAsynchronousPacketHandler(self.AsynchronousPacketHandler)
        device.setCommandPacketHandler(self.CommandPacketHandler)
        device.setFillIdentificationData(self.FillIdentificationDataHandler)
        device.setReplyPacketHandler(self.ReplyPacketHandler)
        device.setFillStatusData(self.FillStatusDataHandler)

    def send_user_input(self, s):
        # The string should start with "get" or "set"
        command, *parameters = s.split()
        if command == "get":
            if len(parameters) != 1:
                raise SyntaxError("Incorrect amount of parameters. Expected 1 got {}".format(len(parameters)))

            if parameters[0] not in PACKET_NAMES:
                raise SyntaxError("Unknown variable name {}".format(parameters[0]))

            code = PACKET_NAMES[parameters[0]]
            self.device.write(code + 0x10, b"", Flags=lbp.Comms.FLAGS_COMMAND)

        elif command == "set":
            if len(parameters) != 2:
                raise SyntaxError("Incorrect amount of parameters. Expected 2 got {}".format(len(parameters)))

            if parameters[0] not in PACKET_NAMES:
                raise SyntaxError("Unknown variable name {}".format(parameters[0]))

            code = PACKET_NAMES[parameters[0]]

            value = PARSE_FUN.get(code, (val_int, int))[0](parameters[1])
            value = struct.pack(PACKET_SIZE[code], value)

            self.device.write(code + 0x20, value, Flags=lbp.Comms.FLAGS_COMMAND)

        else:
            raise SyntaxError("Unknown command {}".format(command))

    def AsynchronousPacketHandler(self, source, sequence, command, data):
        print('\nAsynchronous/Broadcast packet')
        print('Source: ' + hex(source))
        print('Sequence: ' + hex(sequence))
        print('Command: ' + hex(command))
        print('Data: ' + str(bytes(data)))

    def CommandPacketHandler(self, source, sequence, command, data):
        print('\Command packet (NACK returned)')
        print('Source: ' + hex(source))
        print('Sequence: ' + hex(sequence))
        print('Command: ' + hex(command))
        print('Data: ' + str(bytes(data)))
        self.device._TxBuffer.clear()
        return False

    def ReplyPacketHandler(self, source, sequence, command, data):
        if command - 0x10 in PACKET_NUMBERS:
            command -= 0x10

        elif command - 0x20 in PACKET_NUMBERS:
            command -= 0x20

        else:
            print('\nReply packet')
            print('Source: ' + hex(source))
            print('Sequence: ' + hex(sequence))
            print('Command: ' + hex(command))
            print('Data: ' + str(bytes(data)))
            return

        name = PACKET_NUMBERS[command]
        value, = struct.unpack(PACKET_SIZE[command], data)
        value = PARSE_FUN.get(command, (val_int, int))[1](value)
        print("{} is {}".format(name, value))

    def FillIdentificationDataHandler(self, source, sequence, command, data):
        print('\nIdentification data request (empty packet returned)')
        print('Source: ' + hex(source))
        print('Sequence: ' + hex(sequence))
        print('Command: ' + hex(command))
        print('Data: ' + str(bytes(data)))
        self.device._TxBuffer.clear()

    def FillStatusDataHandler(self, source, sequence, command, data):
        print('\Status data request (empty packet returned)')
        print('Source: ' + hex(source))
        print('Sequence: ' + hex(sequence))
        print('Command: ' + hex(command))
        print('Data: ' + str(bytes(data)))
        self.device._TxBuffer.clear()


def main():
    comports = [i.device for i in list_ports.comports()]
    if not len(comports):
        sys.exit("No available comports")

    elif len(comports) > 1:
        print("Choose a port. Available ports are: {}".format(", ".join(comports)))
        name = input()
        if name not in comports:
            exit("Invalid port")

    else:
        name = comports[0]

    port = CommsPort()
    port.enable(name, BAUD)

    device = lbp.CommsDevice(port)

    handler = PacketHandler(device)

    print("Type a command, or type help")
    while True:
        s = input("").strip()
        if not s:
            continue

        elif s.strip() == "quit":
            break

        elif s == "help":
            print(HELP)
            continue

        elif s.strip() == "list":
            print(", ".join(PACKET_NAMES))
            continue

        try:
            handler.send_user_input(s)

        except SyntaxError as e:
            print("\n".join(e.args))

    port.disable()


if __name__ == "__main__":
    main()