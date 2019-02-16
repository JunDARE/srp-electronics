'''Launchbox protocol for python3
This module contains a python implementation of the launchbox protocol and TEZTER.
TEZTER is a simple console aplication that displays incoming packets and can optionally broadcast the ansi encoded string 'TEZT' with command 0xFF. Note that Z is one of the characters escaped by the protocol and escaping is therefore tested with that string. To start TEZTER just run the module file.
'''

# Author:  Aaron M. de Windt
# Date:    20150125
# Delft Aerospace Rocket Engineering
#
# Translated from C++ code written by Jeroen van Straten. 
#
# Communications library for UART based networks between rockets and
# PCs, designed specifically to enable routing devices such as the ones
# in Gabriel v2 and Lucifer v2.
#
# PHYSICAL LAYER IMPLEMENTATION
#  - UART, RS232 or RS485 using LPC UART peripheral modules.
#  - UART config: 115200bps, 8 bits, no parity, 1 stop bit
#
# DATA LINK LAYER IMPLEMENTATION
#  - Handles error detection, drops packets of which the checksum
#    fails.
#  - Frame format:
# 
#      0x55  Start byte
#      Data  Data, may be of any length as long as the buffers can
#            handle it.
#      CRC   CRC over the contents of the data section.
#      0x5A  Stop byte
#    
#    To ensure uniqueness of 0x55 and 0x5A, the following escape
#    sequences are applied to the data and CRC fields:
#
#      0x50  -  0x50 0xAF
#      0x55  -  0x50 0xAA
#      0x5A  -  0x50 0xA5
#
#    Or, in other words, these are the ones complement of the original
#    bytes prepended with 0x50.
#
# NETWORK LAYER IMPLEMENTATION
#  - Packet format:
#      
#      Byte 0
#        Bit 7:6  Flags
#        Bit 5:0  6-bit source address
#
#      Byte 1
#        Bit 7:6  2-bit sequence number for pipelining
#        Bit 5:0  6-bit destination address (ignored in broadcast
#                 packets)
#
#      Byte 2     Message ID
#
#      Rest       Data portion of the packet
#
#  - Flags:
#
#      00  Command packet. A device must always reply to a command
#          packet. In the reply packet, the Flags must be 01, the
#          sequence number should equal the received sequence number,
#          and the command byte must equal either the received command
#          byte (ACK) or 0x01 if the command is not supported (NACK).
#          If no reply is received, the sending device may resend the
#          command to retry.
#      01  Reply packet. Sent in reply to a command packet, see above.
#      10  Asynchronous packet. Similar to a command packet, but no
#          reply is expected. Useful for streaming data or when
#          delivery notifications are sent by higher level protocols.
#      11  Broadcast packet. Identical to an asynchronous packet, but
#          routers will relay these packets to all ports, ignoring the
#          destination byte. Loopback is supressed, however.
#
#  - Source address: defines the source of the packet, should the
#    recipient wish to reply to it. This field should be set to 0x3F
#    (all ones) by any end device when transmitting, any routers along
#    the way will set the address to the actual source address. This
#    way, the end devices need never know what address they have, only
#    the routers have to know. When two end devices are connected to
#    each other, the source and destination addresses simply remain
#    0x3F, as there is no need for addressing in such a case.
#
#  - Sequence number: may be used to identify replies to commands when
#    up to four commands are sent at once, before receiving the reply
#    to them (pipelining/sliding window).
#
#  - Destination address: the address which the packet is destined for.
#    Ignored for broadcast packets. If the destination address is 0x3F
#    and the command ID is 0x02 (identification), the router must reply
#    instead of routing the packet, if there is a router in the
#    connection.
#
#  - Message ID: this byte defines the packet and the meaning of the
#    data. Messages 0x00 to 0x0F are reserved, the rest can be used
#    freely. The reserved packets defined so far are as follows.
#
#      0x00  Used to be RESEND in earlier protocols, the functionality
#            of this command is now replaced by timeouts. As such, this
#            packet is unused. Data field is ignored.
#
#      0x01  NACK. Sent in reply to unknown/unimplemented commands.
#            Data field is ignored.
#
#      0x02  Identification. Must be supported by all devices. Used
#            to figure out which device is connected to a port. Data
#            field of command packet is ignored.
#            
#            This message ID may be used in all packet types. If it
#            is received as a command, the reply is sent using a
#            normal reply packet. If the packet type is asynchronous
#            or broadcast, the reply is sent by means of an 
#            asynchronous packet with message ID 0x03. Refer to
#            packet 0x03 for the reply syntax.
#
#      0x03  Identification reply. Must be supported by all devices.
#            Asynchronous reply to asynchronous packet 0x02. Data
#            syntax:
#              Byte 0
#                Bit 7:4  Electronics system type (eg. cansat, stratos)
#                         If 0b1111, byte 2 is an extended identifier.
#                Bit 3:0  Major version number.
#              Byte 1
#                Bit 7:1  Minor version number.
#                Bit 0    Stable bit. If 0, software should give a
#                         warning when connecting.
#              Byte 2     System type if first 4 bits are ones,
#                         otherwise byte is omitted.
#              Rest       ASCII string with additional information
#                         (usually a build date).
#
#      0x04  Network discovery. All devices should reply to this with
#            a NACK, or, if it is sent asynchronously, they should not
#            reply at all. In stead, the first router along the line
#            should reply with a list of addresses which the sending
#            device can communicate with. If it is sent asynchronously,
#            the reply should be sent with asynchronous message 0x05.
#
#      0x05  Network discovery reply. Sent by routers upon reception
#            of asynchronous message 0x04. Contains a list of
#            available addresses.
#
#      0x06  Status. All devices should support this command. If sent
#            asynchronously, devices should reply asynchronously using
#            message 0x07. For reply syntax, see message 0x07.
#
#      0x07  Status reply. Asynchronous/broadcast packet only. Sent
#            in reply to asynchronous status command, or whenever the
#            device wants. The syntax is as follows:
#              Byte 0
#                0x0-  Standard device, not launchable.
#                0x10  Rocket, status OK, safe.
#                0x11  Rocket, status OK, armed.
#                0x12  Rocket, status warning, safe.
#                0x13  Rocket, status warning, armed.
#                0x14  Rocket, status error, safe.
#                0x15  Rocket, status error, armed.
#                0x1-  Rocket, reserved state
#                0x--  Reserved
#              Rest    Up to 16 ASCII characters representing a
#                      friendly version of the state.
#
#      0x08  Window size. May be sent synchronously only. Command
#            data is ignored, reply data should be 1 byte ranging from
#            1 to 4 representing the maximum amount of pipelining
#            permissible. If a device replies with a NACK, it should
#            be assumed the window size is 1.
#
#      0x0-  Reserved. Returns NACK if synchronous, ignored if
#            asynchronous.
#


from abc import ABCMeta, abstractmethod
import serial
import queue
import copy
import threading

class Comms(object):
    @classmethod
    def NameFromCode(cls, Code):
        for name, code in cls.__dict__.items():
            if code == Code:
                return "Comms." + name
        return "Unknown Comms Code"

    ADDRESS_MASK                      = 0x3F
    FLAGS_MASK                        = 0xC0
                                        
    ADDRESS_UNKNOWN                   = 0x3F
                                        
    FLAGS_COMMAND                     = 0x00
    FLAGS_REPLY                       = 0x40
    FLAGS_NOTIFICATION                = 0x80
    FLAGS_BROADCAST                   = 0xC0
                                        
    COMMAND_RESEND                    = 0x00
    COMMAND_NACK                      = 0x01
    COMMAND_IDENTIFY                  = 0x02
    COMMAND_IDENTIFY_REPLY            = 0x03
    COMMAND_NETWORK_DISCOVERY         = 0x04
    COMMAND_NETWORK_DISCOVERY_REPLY   = 0x05
    COMMAND_STATUS                    = 0x06
    COMMAND_STATUS_REPLY              = 0x07
    COMMAND_WINDOW_SIZE               = 0x08
    COMMAND_TEZT					  = 0x09 # NOT standard Launchbox protocol
    COMMAND_TEZT_REPLY				  = 0x0A # NOT standard Launchbox protocol
    COMMAND_VARIABLE                  = 0x0B # NOT Standard
                                        
    PACKET_START                      = 0x55
    PACKET_END                        = 0x5A
    PACKET_ESCAPE                     = 0x50
                                        
    DEVICE_STATUS_STANDARD            = 0x00
    DEVICE_STATUS_ROCKET              = 0x10
                                        
    DEVICE_STATUS_SAFE                = 0x00
    DEVICE_STATUS_ARMED               = 0x01
                                        
    DEVICE_STATUS_OK                  = 0x00
    DEVICE_STATUS_WARNING             = 0x02
    DEVICE_STATUS_ERROR               = 0x04

    RXS_idle                          = 0
    RXS_receiving                     = 1
    RXS_escaping                      = 2

    @staticmethod
    def crc8(b, crc):
        b2 = b
        if (b < 0):
            b2 = b + 256
        for i in range(8):
            odd = ((b2^crc) & 1) == 1
            crc >>= 1
            b2 >>= 1
            if (odd):
                crc ^= 0x8C
        return crc

class CommsListener(object, metaclass=ABCMeta):
    """Interface for a class which accepts packets as an alternative to using a simple callback function."""
    @abstractmethod
    def handlePacket(self, sender, data):
        pass

class CommsPort(object, metaclass=ABCMeta):
    """Interface for a communications port. Implementations of this interface should override update() and txPacket()."""
    def setListener(self, listener):
        """(aka setRxHandler) Sets the listning device asosiated with the port."""
        self.listener = listener;

    @abstractmethod
    def update(self):
        pass

    @abstractmethod
    def txPacket(self, data):
        pass

    @abstractmethod
    def enable(self, name, baudrate):
        pass

    @abstractmethod
    def disable(self):
        pass

    @abstractmethod
    def reset(self):
        pass

    @abstractmethod
    def update(self):
        pass

    def rxPacket(self, data):
        self.listener.handlePacket(self, data)

class CommsPortSerial(CommsPort):
    counter_tx = 0

    def __init__(self):
        self.PacketParser = CommsPacketParser()
        self.PacketParser.setPacketHandler(self.PacketParserHandler)

        self.PacketBuilder = CommsPacketBuilder()

        self.ser = None
        self.q = queue.Queue()   
        
        self.thread = None    

    def enable(self, name, baudrate=38400, WriteTimeout=1):
        self.ser = serial.Serial(name, baudrate, timeout=1, writeTimeout=WriteTimeout)        
        self.thread = threading.Thread(target=self.ThreadTarget)
        self.thread.start()

    def disable(self):
        self.ser.close()

    def reset(self):
        self.PacketBuilder.Reset()
        self.PacketParser.Reset()
        self.disable()
        self.enable()

    def update(self):        
        try:
             self.rxPacket(self.q.get(False))
        except queue.Empty as e:
            pass
        
    def txPacket(self, data):
        try:
            self.PacketBuilder.AddData(data)
            bytes_package = self.PacketBuilder.GetPacket()
            #print("Packge send: ", bytes_package)
            bytes = self.ser.write(bytes_package)
            self.counter_tx += 1;            
        except serial.SerialTimeoutException:
            print("Warning: Data transmit timed-out.")
        except:
            print("Unexpected error:", sys.exc_info()[0])
            raise
        
    def ThreadTarget(self):
        b = None
        while self.ser.isOpen():    
            #print("read thread")        
            b = self.ser.read()
            if b:
                self.PacketParser.parseByte(b)
    
    def PacketParserHandler(self, data):
        #raise Exception("skdbhfsjdhfbjhrbfr")
        self.q.put(data)

class CommsDevice(CommsListener):
    """Provides network layer functionality for end devices."""
    def __init__(self, port):
        if not isinstance(port, CommsPort):
            raise TypeError("port is not a CommsPort")

        port.setListener(self)
        self.port = port;

        # Handlers
        self._CommandPacketHandler      = None
        self._AsynchronousPacketHandler = None
        self._ReplyPacketHandler        = None
        self._FillIdentificationData    = None
        self._FillStatusData            = None        

        self.TxHeader = bytearray([0,0,0]);
        self._TxBuffer = bytearray();

    # Rx
    def setCommandPacketHandler(self, handler):
        self._CommandPacketHandler = handler;

    def setAsynchronousPacketHandler(self, handler):
        self._AsynchronousPacketHandler = handler
        
    def setReplyPacketHandler(self, handler):
        self._ReplyPacketHandler        = handler

    def setFillIdentificationData(self, handler):
        self._FillIdentificationData    = handler    
       
    def setFillStatusData(self, handler):
        self._FillStatusData            = handler

    def CommandPacketHandler(self, source, sequence, command, data):
        if self._CommandPacketHandler:
            return self._CommandPacketHandler(source, sequence, command, data)

    def AsynchronousPacketHandler(self, source, sequence, command, data):
        if self._AsynchronousPacketHandler:
            self._AsynchronousPacketHandler(source, sequence, command, data)

    def ReplyPacketHandler(self, source, sequence, command, data):
        if self._ReplyPacketHandler:
            self._ReplyPacketHandler(source, sequence, command, data)

    def FillIdentificationData(self, source, sequence, command, data):
        if self._FillIdentificationData:
            self._FillIdentificationData(source, sequence, command, data)
        else:
            self._TxBuffer.clear()
        return 1

    def FillStatusData(self, source, sequence, command, data):
        if self._FillStatusData:
            self._FillStatusData(source, sequence, command, data)
        else:
            self._TxBuffer.clear()
        return 1

    def handlePacket(self, sender, Packet):
        source   = Packet[0] & Comms.ADDRESS_MASK
        flags    = Packet[0] & Comms.FLAGS_MASK
        sequence = Packet[1] & Comms.FLAGS_MASK
        command  = Packet[2]
        data     = Packet[3:]

        if flags == Comms.FLAGS_COMMAND:            
            if self._internalCommandPacketHandler(source, sequence, command, data):
                self.TxCommand = command               
            else:
                self.TxCommand = Comms.COMMAND_NACK
                self._TxBuffer.clear()
            self.TxDest = source
            self.TxFlags = Comms.FLAGS_REPLY
            self.TxSequence = sequence
            self.transmit()
        elif flags == Comms.FLAGS_REPLY:
            self.ReplyPacketHandler(source, sequence, command, data)
        elif flags == Comms.FLAGS_NOTIFICATION or \
             flags == Comms.FLAGS_BROADCAST:
            self._internalAsynchronousPacketHandler(source, sequence, command, data)

    def _internalCommandPacketHandler(self, source, sequence, command, data):
        if   command == Comms.COMMAND_RESEND or \
             command == Comms.COMMAND_NACK or \
             command == Comms.COMMAND_IDENTIFY_REPLY or \
             command == Comms.COMMAND_NETWORK_DISCOVERY or \
             command == Comms.COMMAND_NETWORK_DISCOVERY_REPLY or \
             command == Comms.COMMAND_STATUS_REPLY:
            return 0
        elif command == Comms.COMMAND_IDENTIFY:
            return self.FillIdentificationData(source, sequence, command, data)
        elif command == Comms.COMMAND_STATUS:
            return self.FillStatusData(source, sequence, command, data)
        elif command == Comms.COMMAND_WINDOW_SIZE:
            self.TxBuffer = Comms.COMMAND_WINDOW_SIZE
        else:
            return self.CommandPacketHandler(source, sequence, command, data)

    def _internalAsynchronousPacketHandler(self, source, sequence, command, data):
        if command == Comms.COMMAND_RESEND or \
           command == Comms.COMMAND_NACK or \
           command == Comms.COMMAND_NETWORK_DISCOVERY or \
           command == Comms.COMMAND_WINDOW_SIZE:
            return
        elif command == Comms.COMMAND_IDENTIFY:
            self.setTxDest(source)
            self.setTxFlags(Comms.FLAGS_NOTIFICATION)
            self.setTxSequence(sequence)
            self.setTxCommand(Comms.COMMAND_IDENTIFY_REPLY)
            self.FillIdentificationData()
            self.transmit()
            return
        elif command == Comms.COMMAND_STATUS:
            self.setTxDest(source)
            self.setTxFlags(Comms.FLAGS_NOTIFICATION)
            self.setTxSequence(sequence)
            self.setTxCommand(Comms.COMMAND_STATUS_REPLY)
            self.FillStatusData()
            self.transmit()
        else:
            self.AsynchronousPacketHandler(source, sequence, command, data)
    # Tx
    def write(self, Command, Data, Flags=Comms.FLAGS_NOTIFICATION, Destination=Comms.ADDRESS_UNKNOWN, Sequence=0x00):
        # setTxBuffer
        if isinstance(Data, bytearray):
            self._TxBuffer = Data
        elif isinstance(Data, bytes):
            self._TxBuffer = bytearray(Data)

        # setTxCommand
        self.TxHeader[2] = Command

        # setTxDest
        self.TxHeader[1] = (self.TxHeader[1] & Comms.FLAGS_MASK) | (Destination & Comms.ADDRESS_MASK);

        # setTxFlags
        self.TxHeader[0] = (Flags & Comms.FLAGS_MASK) | (Comms.ADDRESS_UNKNOWN & Comms.ADDRESS_MASK);

        # setTxSequence
        self.TxHeader[1] = ((Sequence << 6) & Comms.FLAGS_MASK) | (self.TxHeader[1] & Comms.ADDRESS_MASK);

        # transmit
        self.port.txPacket(self.TxHeader.copy() + self._TxBuffer.copy())

    @property
    def TxBuffer(self):
        return self._TxBuffer
    @TxBuffer.setter
    def TxBuffer(self, buffer):
        if isinstance(buffer, bytearray):
            self._TxBuffer = buffer
        elif isinstance(buffer, bytes):
            self._TxBuffer = bytearray(buffer)        
    
    @property
    def TxCommand(self):
        return self.TxHeader[2]
    @TxCommand.setter
    def TxCommand(self, data):
        self.TxHeader[2] = data

    @property
    def TxDest(self):
        return self.TxHeader[1] & Comms.ADDRESS_MASK
    @TxDest.setter
    def TxDest(self, dest):
        self.TxHeader[1] = (self.TxHeader[1] & Comms.FLAGS_MASK) | (dest & Comms.ADDRESS_MASK);

    @property
    def TxFlags(self):
        return self.TxHeader[0] & Comms.FLAGS_MASK
    @TxFlags.setter
    def TxFlags(self,  flags):
        self.TxHeader[0] = (flags & Comms.FLAGS_MASK) | (Comms.ADDRESS_UNKNOWN & Comms.ADDRESS_MASK);

    @property
    def TxSequence(self):
        return (self.TxHeader[1] & Comms.FLAGS_MASK) >> 6
    @TxSequence.setter
    def TxSequence(self, sequence):
        self.TxHeader[1] = ((sequence << 6) & Comms.FLAGS_MASK) | (self.TxHeader[1] & Comms.ADDRESS_MASK);

    def transmit(self):
        self.port.txPacket(self.TxHeader.copy() + self._TxBuffer.copy())

class CommsPacketBuilder(object):
    """Builds Launch box protocol packets."""
    def __init__(self):
        self._buffer = bytearray((Comms.PACKET_START,));
        self.crc = 0;

    def Reset(self):
        self._buffer.clear()
        self._buffer.append(Comms.PACKET_START)
        self.crc = 0

    def AddData(self, Data):
        if not (isinstance(Data, bytearray) or isinstance(Data, bytes)):
            raise TypeError()

        for b in iter(Data):
            self.AddChar(b)

    def AddChar(self, b):
        if (b == Comms.PACKET_START or \
            b == Comms.PACKET_END or \
            b == Comms.PACKET_ESCAPE):
            self._buffer.append(Comms.PACKET_ESCAPE)
            self._buffer.append((~b)&0xFF)
        else:
            self._buffer.append(b)
        self.crc = Comms.crc8(b,self.crc)

    def GetPacket(self):
        self.AddChar(self.crc)
        self._buffer.append(Comms.PACKET_END);
        A = copy.deepcopy(self._buffer) 
        self.Reset()
        return A

class CommsPacketParser(object):
    """Reads and procceses data."""
    
    def __init__(self):
        self._PacketHandler = None
        self._buffer = bytearray()
        self.rxCrc = 0
        self.rxState = Comms.RXS_idle

    def Reset(self):
        self._buffer.clear()
        
    def parseByte(self, b):
        b = b[0]
        if b == Comms.PACKET_START:
            self._buffer.clear();
            self.rxCrc = 0
            self.rxState = Comms.RXS_receiving
            return

        if self.rxState == Comms.RXS_idle:
            return

        if b == Comms.PACKET_END:
            if not self.rxCrc:
                self._PacketHandler(self._buffer[:-1])
            self.rxState = Comms.RXS_idle

        if b == Comms.PACKET_ESCAPE:
            self.rxState = Comms.RXS_escaping
            return

        if self.rxState == Comms.RXS_escaping:
            self.rxState = Comms.RXS_receiving
            b = (~b)&0xFF

        self._buffer.append(b)
        self.rxCrc = Comms.crc8(b, self.rxCrc)

    def parseData(self, data):
        for b in data:
            self.parseByte(bytes((b,)))

    def setPacketHandler(self, handler):
        ''' handler function must have one parameter'''
        self._PacketHandler = handler
        self.Reset()


if __name__ == "__main__":
    print('''Welcome to the Launchbox protocol TEZTER.
This program will listen for all incoming launchbox protocol packeges.''')
    # The Z is not a typo. It's one of the characters escaped by the 
    # protocol and is there to test if it's being escaped proporly.

    def CreateCommSerialPort():
        name = input('Port name?\n')
        baud = input('Baudrate? [38400]\n')
        if baud == '':
            baud = 38400;
        elif baud.isdigit():
            baud=int(baud)
        else:
            raise TypeError('Baud rate must be an integer.')
        port = CommsPortSerial()
        port.enable(name, baud)
        return port

    tezt = None
    psblnp = {'y':True, 'n':False, '': False}
    while not tezt in psblnp:
        tezt = input('Boadcast TEZT(y/N)?\n')
    tezt = psblnp[tezt]

    def AsynchronousPacketHandler(source, sequence, command, data):
        print('\nAsynchronous/Broadcast packet')
        print('Source: ' + hex(source))
        print('Sequence: ' + hex(sequence))
        print('Command: ' + hex(command))
        print('Data: ' + str(bytes(data)))

    def CommandPacketHandler(source, sequence, command, data):
        print('\Command packet (NACK returned)')
        print('Source: ' + hex(source))
        print('Sequence: ' + hex(sequence))
        print('Command: ' + hex(command))
        print('Data: ' + str(bytes(data)))
        device._TxBuffer.clear()
        return False

    def ReplyPacketHandler(source, sequence, command, data):
        print('\nReply packet')
        print('Source: ' + hex(source))
        print('Sequence: ' + hex(sequence))
        print('Command: ' + hex(command))
        print('Data: ' + str(bytes(data)))

    def FillIdentificationDataHandler(source, sequence, command, data):
        print('\nIdentification data request (empty packet returned)')
        print('Source: ' + hex(source))
        print('Sequence: ' + hex(sequence))
        print('Command: ' + hex(command))
        print('Data: ' + str(bytes(data)))
        device._TxBuffer.clear()

    def FillStatusDataHandler(source, sequence, command, data):
        print('\Status data request (empty packet returned)')
        print('Source: ' + hex(source))
        print('Sequence: ' + hex(sequence))
        print('Command: ' + hex(command))
        print('Data: ' + str(bytes(data)))
        device._TxBuffer.clear()

    port = CreateCommSerialPort()
    device = CommsDevice(port)
    
    device.setAsynchronousPacketHandler(AsynchronousPacketHandler)
    device.setCommandPacketHandler(CommandPacketHandler)
    device.setFillIdentificationData(FillIdentificationDataHandler)
    device.setReplyPacketHandler(ReplyPacketHandler)
    device.setFillStatusData(FillStatusDataHandler)

    print('Listening...')
    import time
    t = time.process_time()
    while True:
        device.port.update();
        if time.process_time() - t >= 1 and tezt:
            t = time.process_time()
            print('\nTEZT send')  
            
            device.write(Comms.COMMAND_TEZT, b'TE\x5AT', Comms.FLAGS_BROADCAST)
