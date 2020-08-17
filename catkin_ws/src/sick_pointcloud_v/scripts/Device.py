#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
This module handles the device connection. It allows to connect
to a V3SCamera device. The device is connected via two channels -
one for the control commands and one for streaming.

Copyright note: Redistribution and use in source, with or without modification, are permitted.

created: 2015-01-19
updated: 2018-02-02

author: Uwe Hahne, Torben Peichl
SICK AG, Waldkirch
email: techsupport0905@sick.de
Last commit: $Date: 2018-08-20 15:39:47 +0200 (Mo, 20 Aug 2018) $
Last editor: $Author: dedekst $

Version: "$Revision: 19101 $"

"""

import socket, sys, logging
import binascii
import struct
import hashlib
import time
import re
import zlib
import base64

# ----------------------------------------------------------------------------------------------------------------------

# device settings
TCP_PORT_BLOBSERVER = 2114  # standard port for data stream
# You have to use SOPAS ET in order to change these settings for the device

# constants
START_STX = b'\x02\x02\x02\x02' # start sequence of the CoLa protocol
PAYLOAD_OFFSET = 8 # payload starts after an offset of 8 bytes
FRAGMENT_SIZE = 1024  # tcp fragment size of the data stream

### BEGIN REMOVE FOR PUBLIC VERSION
# needed for internal flash access
TOFSPI_FLASH_CHIP_SIZE = 0x200000
TOFSPI_FLASH_SECTOR_SIZE = 0x10000
TOFSPI_FLASH_FILE_BASE_ADDR = 0x100000
### END REMOVE FOR PUBLIC VERSION

USERLEVEL_OPERATOR = 1
USERLEVEL_MAINTENANCE = 2
USERLEVEL_AUTH_CLIENT = 3
USERLEVEL_SERVICE = 4

### BEGIN REMOVE FOR PUBLIC VERSION
# additional internal user levels
USERLEVEL_SICKSERVICE = 5
USERLEVEL_PRODUCTION = 6
USERLEVEL_DEVELOPER = 7
### END REMOVE FOR PUBLIC VERSION

TRANSPORTPROTOCOLAPI_TCP = 0
TRANSPORTPROTOCOLAPI_UDP = 1

ACQUISITIONMODE_NORMAL = 0
ACQUISITIONMODE_HDR = 1
ACQUISITIONMODE_HIGHSPEED = 2

POWERMODE_STREAMING_STANDBY = 5 # Device is in a stand-by mode, that keeps streaming data albeit without usable data
POWERMODE_ACTIVE = 6 # Device is up and running

# SRT errors using the list from http://plone.sickcn.net/div08/tools/sopas-et/sopas-errorcodes
COLA_ERROR_MSG = {
  0x0001 : "access denied",
  0x0002 : "unknown method",
  0x0003 : "unknown variable",
  0x0004 : "local condition failed",
  0x0005 : "invalid data",
  0x0006 : "unknown command",
  0x0007 : "parameter/return value buffer overflow",
  0x0008 : "parameter/return value buffer underflow",
  0x0009 : "parameter type error",
  0x000A : "variable write access denied",
  0x000B : "unknown command for nameserver",
  0x000C : "unknown CoLa command",
  0x000D : "method server busy",
  0x000E : "flex array/string out of bounds",
  0x000F : "unknown event",
  0x0010 : "CoLaA value overflow",
  0x0011 : "invalid character in CoLaA packet",
  0x0012 : "OsAI no message",
  0x0013 : "OsAI no answer message",
  0x0014 : "Internal error, e.g. AppSpace SRT method does not return a retval",
  0x0015 : "HubAddress corrupted",
  0x0016 : "HubAddress decoding",
  0x0017 : "HubAddress address exceeded",
  0x0018 : "HubAddress blank expected",
  0x0019 : "AsyncMethods are suppressed",
  0x001A : "reserved",
  0x001B : "reserved",
  0x001C : "reserved",
  0x001D : "reserved",
  0x001E : "reserved",
  0x001F : "reserved",
  0x0020 : "ComplexArrays are not supported",
  0x0021 : "no ressources for new session",
  0x0022 : "unknown session ID",
  0x0023 : "cannot connect",
  0x0024 : "invalid port ID",
  0x0025 : "scan already active",
  0x0026 : "out of timers",
  0x0027 : "reserved",
  0x0028 : "reserved",
  0x0029 : "reserved",
  0x002A : "reserved",
  0x002B : "reserved",
  0x002C : "reserved",
  0x002D : "reserved",
  0x002E : "reserved",
  0x002F : "reserved",
  0x0030 : "reserved",
  0x0031 : "reserved",
  0x0032 : "reserved",
  0x0033 : "reserved",
  0x0034 : "reserved",
  0x0035 : "reserved",
  0x0036 : "reserved",
  0x0037 : "reserved",
  0x0038 : "reserved",
  0x0039 : "reserved",
  0x003A : "reserved",
  0x003B : "reserved",
  0x003C : "reserved",
  0x003D : "reserved",
  0x003E : "reserved",
  0x003F : "reserved",
  # SRTpp errors
  0x0040 : "CID node error",
  0x0041 : "CID leaf error",
  0x0042 : "CID struct error",
  0x0043 : "CID type select error",
  0x0044 : "CID array error",
  0x0045 : "SRTpp processor error",
  0x0046 : "SRTpp repository error",
  0x0047 : "SRT factory error",
  0x0048 : "SRT factory xml error",
  0x0049 : "IXML parser error",
  0x004A : "addressing by index not supported by SRTpp",
  0x004C : "no ICID method handler registered",
  0x004D : "method handler expected parameter which wasn't provided",
  0x004E : "method handler expected return value which wasn't provided",
  0x004F : "CID enum error",
  0x0050 : "can't acquire ClientID",
  0x0051 : "CID VirtualMemory / CIDBankSwitching error",
  0x0052 : "CIDCplxLeaf unknown buffer",
  0x0053 : "CIDCplxLeaf out of buffer"
}

# ----------------------------------------------------------------------------------------------------------------------

def chksum_cola(str_value):
    """ Calculate CoLa checksum.
    The checksum is built by exclusive ORing all bytes beginning after the
    length indication. The checksum is one byte and it is placed at the end of
    the frame.
    """
    chksum = 0
    for x in str_value:
        chksum ^= ord(x)
    return chksum

def calculatePasswordHash(strPassword):
    m = hashlib.md5() #use new hashlib
    m.update(strPassword.encode('utf8'))
    dig = m.digest()
    dig = [ord(x) for x in dig] # convert bytes to int
    # 128 bit to 32 bit by XOR
    byte0 = dig[0] ^ dig[4] ^ dig[8]  ^ dig[12]
    byte1 = dig[1] ^ dig[5] ^ dig[9]  ^ dig[13]
    byte2 = dig[2] ^ dig[6] ^ dig[10] ^ dig[14]
    byte3 = dig[3] ^ dig[7] ^ dig[11] ^ dig[15]
    retValue = byte0 | (byte1 << 8) | (byte2 << 16) | (byte3 << 24)
    return retValue

def encode_framing_cola_a(payload):
    """ the ascii framing used to serialize the commands """
    return '\x02' + payload + '\x03'

def encode_framing(payload):
    """ the binary framing used to serialize the commands """
    return START_STX + struct.pack('>I', len(payload)) + payload + chr(chksum_cola(payload))

def addMessageLayer(payload):
    """ the binary framing used to serialize the commands """
    # checksum is omitted in CoLa 2
    # HubCntr and NoC are inserted as 0
    return START_STX + struct.pack('>IBB', len(payload) + 2, 0, 0) + payload

def to_hex(str_value):
    """ just to produce a readable output of the device responses """
    return ' '.join(x.encode('hex') for x in str_value)

def to_ascii(str_value):
    """ just to produce a readable output of the device responses """
    return binascii.b2a_qp(str_value)

STRUCT_FLEX16 = struct.Struct('>H')

def pack_flexstring(s):
    """ packs and return a cola flexstring (with 16bit length) for a given string """
    s = bytes(s)
    return STRUCT_FLEX16.pack(len(s)) + s

def unpack_flexstring_from(buf, offset=0):
    """ unpacks a cola flexstring (with 16bit length) and returns the string and the length taken"""
    length, = STRUCT_FLEX16.unpack_from(buf, offset)
    start = offset + STRUCT_FLEX16.size
    end = start + length
    s = buf[start:end].tobytes()
    return s, end

def unpack_flexstring_from_cola_a(buf, offset=0):
    end = buf[offset:].tobytes().find(' ')
    if end < 0:
        raise RuntimeError('Cannot identify string length correctly')
    strlen = int(buf[offset:offset+end].tobytes())
    offset += (end + 1) # +1 for separating space char
    s = buf[offset:offset+strlen].tobytes()
    offset += (strlen + 1) # +1 for separating space char
    return s, offset

class Control:
    """ all methods that use the control channel (sopas) """

    _COLAB_HEADER = struct.Struct('>ccc')
    _COLA2_HEADER = struct.Struct('>IHcc')

    COLA_A = 'COLA_A'
    COLA_B = 'COLA_B'
    COLA_2 = 'COLA_2'

    DEF_PORT_COLA_B = 2112
    DEF_PORT_COLA_2 = 2122

    def __init__(self, ipAddress='192.168.1.10', tcpPort=None, protocol=None, timeout=5):
        self.ipAddress = ipAddress
        self.timeout = timeout
        if not tcpPort:
            self.tcpPort = self.DEF_PORT_COLA_B  # default
        else:
            self.tcpPort = tcpPort
        if not protocol:
            # best guess depending on port
            if self.tcpPort == self.DEF_PORT_COLA_B:
                self.protocol = self.COLA_B
            elif self.tcpPort == self.DEF_PORT_COLA_2:
                self.protocol = self.COLA_2
            else:
                raise RuntimeError("ERROR Device.Control(): please spcifiy either {!r} or {!r} as protocol".format(self.COLA_B, self.COLA_2))
        else:
            self.protocol = protocol
        self.sessionId = -1
        self.reqId = 0
        # must be divided to take into account place for base64 encoding
        self.maxFileBufferLength = int(16384 / 1.334) # DevTool supports 32768, for compatibility stay with 16k

        logging.info("Control() ip: {}, port: {}, protocol: {}".format(self.ipAddress, str(self.tcpPort), self.protocol))

    def open(self):
        """ establish the control channel to the device """
        logging.info("Connecting to device...")
        self.sock_sopas = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock_sopas.settimeout(self.timeout)
        self.sock_sopas.connect((self.ipAddress, self.tcpPort))
        if self.protocol == self.COLA_2:
            self.getSession()
        logging.info("done.")

    def close(self):
        """ close device control channel """
        logging.info("Closing device connection...")
        self.sock_sopas.close()
        logging.info("done.")


    def _raiseColaError(self, errorCode):
        errorText = COLA_ERROR_MSG.get(errorCode, "unknown")
        raise RuntimeError("Error: received FA response, code: 0x%04X (%s)" % (errorCode, errorText))
    
    def recvResponse(self, extra_bytes):
        """ Receives and parses a response from the device"""
        if self.protocol == self.COLA_A:
            # check fist byte is stx
            rxbuf = self.sock_sopas.recv(1)
            if '\x02' != rxbuf:
                raise RuntimeError("Could not find start of framing")
            
            while rxbuf.find('\x03') < 0:
                rxbuf += self.sock_sopas.recv(1024)
            pos_etx = rxbuf.find('\x03')
            payload = memoryview(rxbuf[1:pos_etx])
            
            if logging.getLogger().isEnabledFor(logging.DEBUG):
                logging.debug("received %i bytes payload"%len(payload))
                logging.debug("payload is: %s" % to_hex(payload))
            
            return payload
        else:
            header = self.sock_sopas.recv(8)  # minimum header
            if START_STX != header[:4]:
                raise RuntimeError("Could not find start of framing")
            payloadLength, = struct.unpack_from('>I', header, 4)
            payloadLength += extra_bytes
    
            if logging.getLogger().isEnabledFor(logging.DEBUG):
                logging.debug("received header (8 bytes): %s" % to_hex(header))
                logging.debug("length of %i bytes expected"%payloadLength)
    
            toread = payloadLength
            data = bytearray(toread)
            view = memoryview(data)
            while toread:
                nBytes = self.sock_sopas.recv_into(view, toread)
                if nBytes==0:
                    # premature end of connection
                    raise RuntimeError("received {} but requested {} bytes".format(len(data)-len(view), payloadLength))
                view = view[nBytes:]
                toread -= nBytes
    
            payload = memoryview(data)
    
            if logging.getLogger().isEnabledFor(logging.DEBUG):
                logging.debug("received %i bytes payload"%len(payload))
                logging.debug("payload is: %s" % to_hex(payload))
    
            return payload

    def sendToDevice(self, message, extra_bytes):
        """ Sends a given message to the device and return the response """
        logging.debug("Sending %d bytes to device: %s" % (len(message), to_hex(message)))
        self.sock_sopas.send(message)
        time.sleep(0.001) # at least the header should be receivable
        return self.recvResponse(extra_bytes)
    
    def sendCoLaA(self, cmd, mode, payload):
        msg = self._COLAB_HEADER.pack('s', cmd, mode) + ' ' + payload
        msg = encode_framing_cola_a(msg)
        # add one byte for checksum, see cola spec
        payload = self.sendToDevice(msg, extra_bytes=1)

        startS, cmd, mode = self._COLAB_HEADER.unpack_from(payload)
        payload = payload[self._COLAB_HEADER.size:]
        if startS != 's':
            raise RuntimeError("malformed response packet, preceeding 's' missing, got {!r} instead".format(startS))
        return cmd, mode, payload

    def sendCoLaB(self, cmd, mode, payload):
        msg = self._COLAB_HEADER.pack('s', cmd, mode) + payload
        msg = encode_framing(msg)
        # add one byte for checksum, see cola spec
        payload = self.sendToDevice(msg, extra_bytes=1)

        # CoLaB has a checksum
        payload_cs = ord(payload[-1])
        payload = payload[:-1] # one byte for checksum, see cola spec
        checksum = chksum_cola(payload)
        if checksum != payload_cs:
            raise RuntimeError("Wrong telegram checksum. Expected: 0x{:02X}, received: 0x{:02X}".format(checksum, payload_cs))
        startS, cmd, mode = self._COLAB_HEADER.unpack_from(payload)
        payload = payload[self._COLAB_HEADER.size:]
        if startS != 's':
            raise RuntimeError("malformed response packet, preceeding 's' missing, got {!r} instead".format(startS))
        return cmd, mode, payload

    def sendCoLa2(self, sessionId, cmd, mode, payload):
        reqId = self.reqId
        self.reqId += 1
        msg = self._COLA2_HEADER.pack(sessionId, reqId, cmd, mode) + payload
        msg = addMessageLayer(msg)
        payload = self.sendToDevice(msg, extra_bytes=0)
        # skip HubCtrl und NoC
        payload = payload[2:]
        sessionId, recvReqId, cmd, mode = self._COLA2_HEADER.unpack_from(payload)
        payload = payload[self._COLA2_HEADER.size:]
        if recvReqId != reqId:
            raise RuntimeError("unexpected response; request ids {} expected, but got {}".format(recvReqId, reqId))
        return sessionId, cmd, mode, payload

    def getSession(self):
         # request cola2 session from device
        self.sessionId = -1
        clientID = b'pythonDevice'
        sessionTimeout = 30
        payload = struct.pack('>BH', sessionTimeout, len(clientID)) + clientID
        sessionId, cmd, mode, data = self.sendCoLa2(0, 'O', 'x', payload)

        if cmd != 'O' or mode != 'A':
            if cmd == 'F' and mode == 'A':
                errorNumber, = struct.unpack_from('>H', data)
                self._raiseColaError(errorNumber)                
            raise RuntimeError("failed to create session, invalid command {!r} and mode {!r}".format(cmd, mode))
        if sessionId == 0:
            raise RuntimeError("failed to create session, sessionId was 0")

        self.sessionId = sessionId

    def sendCommand(self, cmd, name, payload=None):
        if not payload:
            payload = bytes()

        payload = bytes(name) + bytes(' ') + bytes(payload)

        if self.protocol == self.COLA_B:
            recvCmd, recvMode, payload = self.sendCoLaB(cmd=cmd, mode='N', 
                                                        payload=payload)
        elif self.protocol == self.COLA_2:
            sessionId = self.sessionId
            if sessionId <= 0:
                raise RuntimeError("Error: cannot send telegram with session id {}!".format(sessionId))
            recvSessionId, recvCmd, recvMode, payload = self.sendCoLa2(sessionId=sessionId, 
                                                                       cmd=cmd, mode='N', 
                                                                       payload=payload)
        elif self.protocol == self.COLA_A:
            payload = payload.rstrip(' ')
            recvCmd, recvMode, payload = self.sendCoLaA(cmd=cmd, mode='N', 
                                                        payload=payload)
        else:
            raise RuntimeError("Error: cannot send command, unknown protocol: {}".format(self.protocol))

        # expected response command code
        if cmd == 'M':
            # synchronous methods returns AN on success
            expectedCmd = 'A'
            expectedMode = 'N'
        else:
            expectedCmd = cmd
            expectedMode = 'A'

        if recvCmd != expectedCmd:
            if recvCmd == 'F':
                errorNumber, = struct.unpack_from('>H', payload)
                self._raiseColaError(errorNumber)
            else:
                raise RuntimeError("unexpcted response packet, expected command {!r}; got {!r}".format(expectedCmd, recvCmd))

        if recvMode != expectedMode:
            raise RuntimeError("invalid response packet, expected answer; got: {!r}{!r}".format(recvCmd, recvMode))

        # check for space between mode and name
        if payload[0] != ' ':
            raise RuntimeError("malformed package, expected space after mode, but got {}{}{!r}".format(recvCmd, recvMode, payload[0]))
        payload = payload[1:]

        # check if received name matches, maximum name length assumed to be 128
        nameEndIdx = payload[:128].tobytes().find(' ')

        if nameEndIdx == 0:
            raise RuntimeError("malformed package, got empty name {!r}{!r}".format(recvCmd, recvMode))

        if nameEndIdx > 0:
            recvName = payload[:nameEndIdx].tobytes()
            payload = payload[nameEndIdx+1:]
        else:
            recvName = payload.tobytes()
            payload = bytes()

        if recvName != name:
            raise RuntimeError("cmd name {!r} and response name {!r} differ".format(name, recvName))

        return payload

    def reboot(self):
        """ reboot device """
        logging.info("Rebooting device...")
        self.sendCommand('M', 'mSCreboot')
        logging.info("done.")

    def readVariable(self, name):
        """ returns data from a variable """
        return self.sendCommand(b'R', name)

    def writeVariable(self, name, data=None):
        """ write data to a variable """
        self.sendCommand(b'W', name, data)

    def invokeMethod(self, name, data=None):
        """ Invoke method. """
        logging.info("Invoking %s method..." % name)
        rx = self.sendCommand(b'M', name, data)
        logging.info("... done.")
        return rx

    def initStream(self):
        """ Tells the device that there is a streaming channel by invoking a
        method named GetBlobClientConfig.
        """
        self.invokeMethod(b'GetBlobClientConfig')

    def startStream(self):
        """ Start streaming the data by calling the "PLAYSTART" method on the
        device and sending a "Blob request" afterwards.
        """
        self.invokeMethod(b'PLAYSTART')

    def stopStream(self):
        """ Stops the data stream. """
        self.invokeMethod(b'PLAYSTOP')

    def singleStep(self):
        """ Triggers one image. """
        self.invokeMethod(b'PLAYNEXT')

    ''' Activate polar 2D data reduction '''
    def activatePolar2DReduction(self):
        self.writeVariable(b'enPolar', struct.pack('B',1))

    ''' Deactivate polar 2D data reduction '''
    def deactivatePolar2DReduction(self):
        self.writeVariable(b'enPolar', struct.pack('B',0))

    ''' Activate Cartesian data reduction '''
    def activateCartesianReduction(self):
        self.writeVariable(b'enCart', struct.pack('B',1))

    ''' Deactivate Cartesian data reduction '''
    def deactivateCartesianReduction(self):
        self.writeVariable(b'enCart', struct.pack('B',0))

    def waitForReductionParamsApplied(self):
        counter = 0
        while True:
            time.sleep(1)
            counter = counter + 1
            rx = self.readVariable(b'applyingParams')
            break_condition = (ord(rx[-1]) != 1) or counter > 5
            if break_condition:
                break

    ''' Enable depth map data channel '''
    def enableDepthMapDataTransfer(self):
        self.writeVariable(b'enDepthAPI', struct.pack('B',1))

    ''' Disables depth map data channel. '''
    def disableDepthMapDataTransfer(self):
        self.writeVariable(b'enDepthAPI', struct.pack('B',0))

    ''' Enable polar 2D data channel '''
    def enablePolar2DDataTransfer(self):
        self.writeVariable(b'enPolarAPI', struct.pack('B',1))

    ''' Disables polar 2D data channel. '''
    def disablePolar2DDataTransfer(self):
        self.writeVariable(b'enPolarAPI', struct.pack('B',0))

    ''' Enable Cartesian data channel '''
    def enableCartesianDataTransfer(self):
        self.writeVariable(b'enHeightAPI', struct.pack('B',1))

    ''' Disables Cartesian data channel. '''
    def disableCartesianDataTransfer(self):
        self.writeVariable(b'enHeightAPI', struct.pack('B',0))

    def applySettings(self):
        self.invokeMethod(b'DeviceReInit')

    def setPowerMode(self, newPowerMode):
        """ Set the device power mode. """
        self.invokeMethod(b'SetPwrMod', struct.pack('>B', newPowerMode))

    def getPowerMode(self):
        """ Get the current device power mode. """
        rx = self.readVariable(b'CurPwrMode')
        return ord(rx[-1])

    def setIntegrationTimeUs(self, newIntegrationTime):
        """ Set the device integration time in microseconds. """
        self.writeVariable(b'integrationTimeUs',struct.pack('>I',newIntegrationTime))

    def getIntegrationTimeUs(self):
        """ Get the current device integration time in microseconds. """
        rx = self.readVariable(b'integrationTimeUs')
        intTime = struct.unpack('>I', rx)
        return intTime[0]

    def setIntegrationTimeUsColor(self, newIntegrationTime):
        """ Set the device integration time in microseconds. """
        self.writeVariable(b'integrationTimeUsColor',struct.pack('>I',newIntegrationTime))

    def getIntegrationTimeUsColor(self):
        """ Get the current device integration time in microseconds. """
        rx = self.readVariable(b'integrationTimeUsColor')
        intTime = struct.unpack('>I', rx)
        return intTime[0]

    def setAcquisitionMode(self, newAcquisitionMode):
        """ Set the acquisition mode. """
        self.writeVariable(b'acquisitionMode', struct.pack('>B',newAcquisitionMode))

    def getAcquisitionMode(self):
        """ Get the current acquisition mode. """
        rx = self.readVariable(b'acquisitionMode')
        return ord(rx[-1])

    def setNonAmbiguityMode(self, newNonAmbiguityMode):
        """ Set the device NonAmbiguity mode. """
        self.writeVariable(b'nareMode', struct.pack('>B',newNonAmbiguityMode))

    def getNonAmbiguityMode(self):
        """ Get the current NonAmbiguity mode. """
        rx = self.readVariable(b'nareMode')
        return ord(rx[-1])

    def login(self, newUserLevel, password):
        """ Logs in into the device with a given user level """
        pwHash = calculatePasswordHash(password)
        rx = self.invokeMethod(b'SetAccessMode', struct.pack('>BI', newUserLevel, pwHash))
        if ord(rx[-1]) != 1: # check the return byte for success
            raise RuntimeError("Fail to login as user level %s with password %s" % (newUserLevel, password))

    def logout(self):
        self.invokeMethod(b'Run')

    def getUserLevel(self):
        rx = self.invokeMethod(b'GetAccessMode')
        return ord(rx[-1])

    def getIdent(self):
        """ Returns the device Name and Version identifier """
        rx = self.readVariable(b'DeviceIdent')
        if self.protocol == self.COLA_A:
            offset = 0
            deviceName, offset = unpack_flexstring_from_cola_a(rx, offset)
            deviceVersion, offset = unpack_flexstring_from_cola_a(rx, offset)
        else:
            # expect to receive "[16bit length]Name [16bit length]Version"
            offset = 0
            deviceName, offset = unpack_flexstring_from(rx, offset)
            deviceVersion, offset = unpack_flexstring_from(rx, offset)
        return  (deviceName, deviceVersion)

### BEGIN REMOVE FOR PUBLIC VERSION
# Additional methods for internal tests (flash access)

    def readFlashScl(self, startAdr, length):
        """ Do not use this private function! Use "readFlash" instead! """
        logging.debug("readFlashScl: %d bytes @ 0x%0X" % (length, startAdr))
        self.validateFlashAccess(startAdr, length)
        if length > 255:
            raise RuntimeError('Length exeeds its limit, only read a maximum of 255 bytes at once')
        rx = self.invokeMethod("ReadFlash", struct.pack('>IH', startAdr, length))
        buf, _ = unpack_flexstring_from(rx)
        if len(buf) != length:
            raise RuntimeError('Less bytes returned %d than requested %d') % (len(buf), length)
        return buf

    def writeFlashScl(self, startAdr, data):
        """ Do not use this private function! Use "writeFlash" instead! """
        logging.debug("writeFlashScl: %d bytes @ 0x%0X" % (len(data), startAdr))
        self.validateFlashAccess(startAdr, len(data))
        if len(data) > 255:
            raise RuntimeError('Length exeeds its limit, only write a maximum of 255 bytes at once')
        rx = self.invokeMethod("WriteFlash", struct.pack('>IH', startAdr, len(data)) + data)
        success, = struct.unpack('>B', rx)
        if success == 0:
            raise RuntimeError("FATAL ERROR: failed to program flash")
        return

    def readFlash(self, startAdr, length):
        """ Read <length> bytes from FPGA flash """
        logging.debug("readFromFlash: %d bytes @ 0x%0X" % (length, startAdr))
        self.validateFlashAccess(startAdr, length)
        if length < 256:
            logging.info("readFlash: simple SCL call")
            return  self.readFlashScl(startAdr, length)
        else:
            logging.info("readFlash: looped calls")
            bytesRead = 0
            data = ""
            while bytesRead < length:
                numBytes = min(255, length - bytesRead)
                logging.debug(("readFlash : scl(%d, %d) # bytesRead=%d") % (startAdr + bytesRead, numBytes, bytesRead))
                data = data + self.readFlashScl(startAdr + bytesRead, numBytes)
                bytesRead = bytesRead + numBytes
            return data

    def writeFlash(self, startAdr, data):
        """ Write <data> into FPGA flash """
        logging.debug("writeFlash: %d bytes @ 0x%0X" % (len(data), startAdr))
        self.validateFlashAccess(startAdr, len(data))
        if len(data) < 256:
            logging.info("writeFlash: simple SCL call")
            self.writeFlashScl(startAdr, data)
        else:
            logging.info("writeFlash: looped calls")
            bytesWritten = 0
            while bytesWritten < len(data):
                numBytes = min(255, len(data) - bytesWritten)
                logging.debug(("writeFlash : scl(%d, %d) # bytesWritten=%d") % (startAdr + bytesWritten, numBytes, bytesWritten))
                self.writeFlashScl(startAdr + bytesWritten, data[bytesWritten:bytesWritten+numBytes])
                bytesWritten = bytesWritten + numBytes
        return

    def eraseAndWriteFlash(self, startAdr, data):
        """ Handle flash write on a filesystem level """
        logging.debug("eraseAndWriteFlash: %d bytes @ 0x%0X" % (len(data), startAdr))
        self.validateFlashAccess(startAdr, len(data))
        endAdr = startAdr + len(data) - 1
        startSector = int(startAdr / TOFSPI_FLASH_SECTOR_SIZE)
        endSector = int(endAdr / TOFSPI_FLASH_SECTOR_SIZE)
        lenSectorRange = (endSector-startSector+1) * TOFSPI_FLASH_SECTOR_SIZE
        # read affected sector range
        buf = self.readFlash(startSector * TOFSPI_FLASH_SECTOR_SIZE, lenSectorRange)
        bufOffset = (startAdr - (startSector * TOFSPI_FLASH_SECTOR_SIZE))
        # just in case again check sizes
        if (bufOffset +  len(data)) > len(buf) :
            logging.error("ERROR: unexpected size missmatch")
            return
        # insert the new data into the buffer
        buf = buf[0:bufOffset] + data + buf[bufOffset+len(data):]
        # erase affected sectors
        for i in range(startSector, endSector + 1):
            logging.debug("delSec %d" % i)
            self.invokeMethod("EraseSector", struct.pack('>B', i))
        # rewrite from buffer (which now contains new data)
        self.writeFlash(startSector * TOFSPI_FLASH_SECTOR_SIZE, buf)

    def validateFlashAccess(self, startAdr, length):
        """ This validates <starAdr> and <length> to fit into the spi chip size """
        endAdr = startAdr + length - 1
        if startAdr < 0 or startAdr >= TOFSPI_FLASH_CHIP_SIZE :
            raise RuntimeError('Start address (%d) must match within the cip size (%u)') % (startAdr, TOFSPI_FLASH_CHIP_SIZE)
        if endAdr >= TOFSPI_FLASH_CHIP_SIZE:
            raise RuntimeError('Address + Size = %d but chip size is %u') % (endAdr, TOFSPI_FLASH_CHIP_SIZE)


    def setPlayFilePath(self, dir, fileName):
        """ Returns the device Name and Version identifier """
        self.writeVariable(b'plyPth', pack_flexstring(dir) + pack_flexstring(fileName))
        
    def getPlayFilePath(self):
        """ Returns the device Name and Version identifier """
        rx = self.readVariable("plyPth")
        offset = 0
        path, offset = unpack_flexstring_from(rx, offset)
        filename, offset = unpack_flexstring_from(rx, offset)
        return  (path, filename)

    def readFile(self, destFile):
        """ read file from target (example stdfs:///media/ram/foobar.txt) """
        # see: http://plone.sickcn.net/div08/EDP2.0/bundles/edpbase/doxygen/page_file_filesystem_sopas_interface.html

        # try to open destFile
        rx = self.invokeMethod("mFSAcc", pack_flexstring(destFile + "?open&mode=rb") + pack_flexstring("") + struct.pack('>I', 0))
        offset = 0
        retUrl, offset = unpack_flexstring_from(rx, offset)
        if retUrl == '?err':
            raise RuntimeError('Failed to open file %s on the device: %s')%(destFile, retUrl)

        # fetch fileId and size
        fileId = re.search('(fileid=)(\d+)', retUrl).group(2)
        rx = self.invokeMethod("mFSAcc", pack_flexstring(destFile + "?size") + pack_flexstring("") + struct.pack('>I', 0))
        try:
            fileLen = int(re.search('(size=)(\d+)', rx).group(2))
        except:
            raise RuntimeError('Failed get file length for: %s returned: %s')%(destFile, rx[2:])

        # fetch data
        paramUrl = "?read&fileid=" + fileId + "&len=" + str(self.maxFileBufferLength)
        retUrl = ""
        outbuf = []
        while retUrl != '?eof':
            rx = self.invokeMethod("mFSAcc", pack_flexstring(paramUrl) + pack_flexstring("") + struct.pack('>I', 0))
            offset = 0
            retUrl, offset = unpack_flexstring_from(rx, offset)
            retBuf, offset = unpack_flexstring_from(rx, offset)
            retCrc, = struct.unpack_from('>i',rx, offset)
            if retUrl == '?err':
                # try to close
                rx = self.invokeMethod("mFSAcc", pack_flexstring("?close&fileid=" + fileId) + pack_flexstring("") + struct.pack('>I', 0))
                # then raise error
                raise RuntimeError('Error while writing file %s, fileId: %s'%(destFile,fileId))
            retBufDec = base64.b64decode(retBuf)
            retBufCrc = zlib.crc32(retBufDec)
            if retBufCrc != retCrc:
                # try to close
                rx = self.invokeMethod("mFSAcc", pack_flexstring("?close&fileid=" + fileId) + pack_flexstring("") + struct.pack('>I', 0))
                # then raise error
                raise RuntimeError('Checksum error! retCRC=0x%04X <-> retBufCrc=0x%04X')%(retCrc, retBufCrc)
            outbuf.append(retBufDec)
        # join list of received chunks
        outbuf = ''.join(outbuf)
        if len(outbuf) != fileLen:
            # try to close
            rx = self.invokeMethod("mFSAcc", pack_flexstring("?close&fileid=" + fileId) + pack_flexstring("") + struct.pack('>I', 0))
            # then raise error
            raise RuntimeError('Unexpected amount of bytes received: %u for file length: %u')%(len(outbuf), fileLen)

        # close file
        rx = self.invokeMethod("mFSAcc", pack_flexstring("?close&fileid=" + fileId) + pack_flexstring("") + struct.pack('>I', 0))
        offset = 0
        retUrl, offset = unpack_flexstring_from(rx, offset)
        if retUrl == '?err':
            raise RuntimeError('Error while closing file %s, fileId: %s'%(destFile,fileId))

        return outbuf

    def writeFile(self, srcFile, destFile):
        """ writes a local file to the device (example stdfs:///media/ram/foobar.txt) """
        # see: http://plone.sickcn.net/div08/EDP2.0/bundles/edpbase/doxygen/page_file_filesystem_sopas_interface.html

        fpIn = open(srcFile, "rb")
        data = fpIn.read()
        fpIn.close()

        # try to open destFile
        rx = self.invokeMethod("mFSAcc", pack_flexstring(destFile + "?open&mode=wb") + pack_flexstring("") + struct.pack('>I', 0))
        offset = 0
        retUrl, offset = unpack_flexstring_from(rx, offset)
        if retUrl == '?err':
            raise RuntimeError('Failed to create file %s on the device: %s')%(destFile, retUrl)

        # fetch fileId and transmit data
        fileId = re.search('(fileid=)(\d+)', retUrl).group(2)
        bytesWritten = 0
        paramUrl = "?write&fileid=" + fileId
        while bytesWritten < len(data):
            paramBuffer = data[bytesWritten:bytesWritten + self.maxFileBufferLength]
            paramCRC = struct.pack('>i', zlib.crc32(paramBuffer))
            rx = self.invokeMethod("mFSAcc", pack_flexstring(paramUrl) + pack_flexstring(base64.b64encode(paramBuffer)) + paramCRC)
            offset = 0
            retUrl, offset = unpack_flexstring_from(rx, offset)
            if retUrl == '?err':
                # try to close
                rx = self.invokeMethod("mFSAcc", pack_flexstring("?close&fileid=" + fileId) + pack_flexstring("") + struct.pack('>I', 0))
                # then raise error
                raise RuntimeError('Error while writing file %s, fileId: %s'%(destFile,fileId))
            bytesWritten += len(paramBuffer)
            time.sleep(0.00005) # 50us settle time

        # close file
        rx = self.invokeMethod("mFSAcc", pack_flexstring("?close&fileid=" + fileId) + pack_flexstring("") + struct.pack('>I', 0))
        offset = 0
        retUrl, offset = unpack_flexstring_from(rx, offset)
        if retUrl == '?err':
            raise RuntimeError('Error while closing file %s, fileId: %s'%(destFile,fileId))

### END REMOVE FOR PUBLIC VERSION

''' All methods that use the streaming channel. '''
class Streaming:
    def __init__(self, ipAddress='192.168.1.10',tcpPort=TCP_PORT_BLOBSERVER):
        self.ipAddress = ipAddress
        self.tcpPort = tcpPort

    ''' Opens the streaming channel. '''
    def openStream(self):
        logging.info("Opening streaming socket..."),
        self.sock_stream = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock_stream.settimeout(5)
        try:
            self.sock_stream.connect((self.ipAddress, self.tcpPort))
        except socket.error as err:
            logging.error("Error on connecting to %s:%d: %s" % (self.ipAddress, self.tcpPort, err))
            sys.exit(2)
        logging.info("...done.")

    def closeStream(self):
        """ Closes the streaming channel. """
        logging.info("Closing streaming connection..."),
        self.sock_stream.close()
        logging.info("...done.")

    def sendBlobRequest(self):
        """ Sending a blob request. """
        MSG_BLREQ_TX = b'BlbReq'

        logging.debug("Sending BlbReq: %s" % (to_hex(MSG_BLREQ_TX)))
        self.sock_stream.send(MSG_BLREQ_TX)

    def getFrame(self):
        """ Receives the raw data frame from the device via the streaming channel."""
        logging.debug('Reading image from stream...')
        keepRunning = True

        BLOB_HEAD_LEN = 11
        header = self.sock_stream.recv(BLOB_HEAD_LEN)  # minimum header
        frameAcqStart = time.clock()
        logging.debug("len(header) = %d dump: %s" % (len(header),to_hex(header)))
        if len(header) < BLOB_HEAD_LEN:
            raise RuntimeError("Uh, not enough bytes for BLOB_HEAD_LEN, only %s" % (len(header)))

        # check if the header content is as expected
        (magicword, pkgLength, protocolVersion, packetType) = \
            struct.unpack('>IIHB', header)
        if magicword != 0x02020202:
            logging.error("Unknown magic word: %0x" % (magicword))
            keepRunning = False
        if protocolVersion != 0x0001:
            logging.error("Unknown protocol version: %0x" % (protocolVersion))
            keepRunning = False
        if packetType != 0x62:
            logging.error("Unknown packet type: %0x" % (packetType))
            keepRunning = False

        if not keepRunning:
            raise RuntimeError('something is wrong with the buffer')

        # -3 for protocolVersion and packetType already received
        # +1 for checksum
        toread = pkgLength - 3 + 1
        logging.debug("pkgLength: %d" % (pkgLength))
        logging.debug("toread: %d" % (toread))

        data = bytearray(len(header) + toread)
        view = memoryview(data)
        view[:len(header)] = header
        view = view[len(header):]
        while toread:
            nBytes = self.sock_stream.recv_into(view, toread)
            if nBytes==0:
                # premature end of connection
                raise RuntimeError("received {} but requested {} bytes".format(len(data)-len(view), pkgLength))
            view = view[nBytes:]
            toread -= nBytes

        self.frame = str(data)

        frameAcqStop = time.clock()
        logging.info("Receiving took %0.1f ms"%((frameAcqStop-frameAcqStart)*1000))
        # full frame should be received now
        logging.debug("...done.")
