"""
This module handles CID Types unpacking and the packing

To create the own CIDTypes simply inherit the class CIDType, 
set the byteorder and the subtypes.

Note: It is crucial that the contructor parameters are named as
the class members, for the pack method uses them to determine 
which value is mapped to which field
"THY SHALL NOT ALTER THE NAME OF THE MEMEBER!" (11th Commandment)

Refer to: https://docs.python.org/2/library/struct.html

author: Georg Fischer <georg.fischer@sick.de>
SICK AG, Waldkirch
email: techsupport0905@sick.de
Last commit: $Date:  $
Last editor: $Author:  $

Version: "$Revision:  $"
"""
import struct
import binascii
import inspect

TYPE_SIZES = {
    "c" : 1, # char
    "b" : 1, # signed char
    "B" : 1, # unsigned char
    "?" : 1, # 	_Bool
    "h" : 2, # short
    "H" : 2, # unsigned short
    "i" : 4, # int
    "I" : 4, # unsigned int
    "l" : 4, # long
    "L" : 4, # unsigned long
    "f" : 4, # float
    "q" : 8, # long long
    "Q" : 8, # unsigned long long
    "d" : 8, # double
}

def packParameters(params):
    """
    Returns a byte array with the given parameters
    """
    buf = bytes("")

    for param in params:
        if type(param) == memoryview:
            buf += bytes(param.tobytes())

        elif issubclass(type(param), CIDType):
            buf += bytes(param.pack().tobytes())

        else:
            buf += bytes(param)

    return buf


class PackException(Exception):
    pass

class CIDType(object):
    """
    Every implemented type contains a field __byteorder__ which specifies the byte order of that type 
    and a field __subtypes__ which specifies the contained subtypes
    """

    __byteorder__ = ">"
    __subtypes__ = []

    def pack(self, buffer = None):

        # first create a buffer (if none is given)
        if buffer == None:
            size = self.getSize()
            buffer = memoryview(bytearray([0] * size))

        # get the constructor arguments
        argNames = inspect.getargspec(type(self).__init__).args[1:]

        if len(argNames) != len(self.__subtypes__):
            raise PackException("Number of arguments not matching with given subtypes")

        # iterate through the subtypes and pack them into the buffer
        members = inspect.getmembers(self)
        offset = 0
        for i in range(len(self.__subtypes__)):
            t = self.__subtypes__[i]

            # get the member value
            searchResult = ([item for item in members if item[0] == argNames[i]])
            value = searchResult[0][1]

            if type(t) == str:
                fmt = self.__byteorder__ + t
                struct.pack_into(fmt, buffer, offset, value)
                # update the offset
                offset += TYPE_SIZES[t]
            elif type(t) == type: 
                if issubclass(t, CIDType):
                    value.pack(buffer[offset:])
                    offset += value.getSize()

        return buffer

    @classmethod
    def getSize(cls):

        size = 0
        for item in cls.__subtypes__:

            if type(item) == str:
                size += TYPE_SIZES[item]

            elif type(item) == type:
                if issubclass(item, CIDType):
                    size += item.getSize()

        return size

    @classmethod
    def unpack(cls, payload):
        
        startPointer = 0
        stopPointer = 0
        data = []
        unpackList = []

        for i in range(len(cls.__subtypes__)):
            item = cls.__subtypes__[i]
            if type(item) == str:
                unpackList.append(item)

                stopPointer += TYPE_SIZES[item]

            elif type(item) == type:

                if len(unpackList) > 0:
                    fmt = cls.__byteorder__ + "".join(unpackList)
                    stopPointer += startPointer
                    data.extend(list(struct.unpack_from(fmt, payload[startPointer:stopPointer])))
                    unpackList = []
                    startPointer = stopPointer
                    stopPointer = 0

                if issubclass(item, CIDType):
                    length, obj  = item.unpack(payload[startPointer:])
                    startPointer += length
                    data.append(obj)   

        if len(unpackList) > 0:
            fmt = cls.__byteorder__ + "".join(unpackList)
            stopPointer += startPointer
            data.extend(list(struct.unpack_from(fmt, payload[startPointer:stopPointer])))
            startPointer = stopPointer

        # returns the stopPointer (which is in fact the size) and a object of type cls
        return stopPointer, cls(*data)

class IntelType(object):
    __byteorder__ = "<"

class CIDArray(CIDType):

    __length__ = 0
    __type__ = None

    def __init__(self, data):
        self.data = data
        self.index = 0

    #### Interation interface ####
    def __iter__(self):
        return self

    def next(self):

        if self.index == len(self.data):
            raise StopIteration
        
        self.index += 1
        return self.data[self.index - 1]

    #### Array interface ####
    def __len__(self):
        return len(self.data)

    def __getitem__(self, key):
        return self.data[key]

    def __setitem__(self, key, value):
        self.data[key] = value

    #### Custom pack method ####
    def pack(self, buffer = None):

        if buffer == None:
            size = self.getSize()
            buffer = memoryview(bytearray([0] * size))
        
        if type(self.__type__) == str:
            fmt = self.__byteorder__ + (self.__type__ * self.__length__)
            struct.pack_into(fmt, buffer, 0, *self.data)

        elif type(self.__type__) == type:            
            if issubclass(self.__type__, CIDType):
                offset = 0
                typesize = self.__type__.getSize()
                for i in range(self.__length__):
                    self.data[i].pack(buffer[offset:])
                    offset += typesize

        return buffer

    #### Custom size method ####
    @classmethod
    def getSize(cls):

        if type(cls.__type__) == str:
            fmt = (cls.__type__ * cls.__length__)
            return struct.calcsize(fmt)
        elif type(cls.__type__) == type:            
            if issubclass(cls.__type__, CIDType):
                return cls.__length__ * cls.__type__.getSize()
        return 0

    #### Custom unpack method ####
    @staticmethod
    def unpackWithClass(cls, payload):
        """
        Because classes like FlexArray need to inject their own class def,
        this static method is provided which does not get the class def injected by 
        invocation, in contrast to the classmethod unpack()
        """
        if type(cls.__type__) == str:
            fmt = cls.__byteorder__ + (cls.__type__ * cls.__length__)
            size = struct.calcsize(fmt)
            data = struct.unpack_from(fmt, payload[:size])

            return size, cls(list(data))

        elif type(cls.__type__) == type:            
            if issubclass(cls.__type__, CIDType):
                
                data = []
                size = 0
                for _ in range(cls.__length__):
                    subSize, subData = cls.__type__.unpack(payload[size:])
                    data.append(subData)
                    size += subSize

                return size, cls(data)
        
        return 0, cls([])

    @classmethod
    def unpack(cls, payload):
        return cls.unpackWithClass(cls, payload)      
        

class FlexArray(CIDArray):
    """
    TODO: packing
    """
    __lengthFieldSize__ = "H"
    
    def __init__(self, data):
        self.data = data

    @classmethod
    def unpack(cls, payload):

        fmt = cls.__byteorder__ + cls.__lengthFieldSize__

        # unpack always returns a tupel => [0] at the end
        length = struct.unpack_from(fmt, payload)[0]

        cls.__length__ = length
        fieldSize = TYPE_SIZES[cls.__lengthFieldSize__]
        
        return CIDArray.unpackWithClass(cls, payload[fieldSize:])


class FlexArray32(FlexArray):
    pass

class FlexArray64(FlexArray):
    __lengthFieldSize__ = "I"


class FlexString(CIDType):
    _headstruct = struct.Struct('>H')
    
    # the size of the length field
    __lengthFieldSize__ = "H"

    def __init__(self, length, string):
        self.length = length
        self.string = string
        
    @classmethod
    def sencode(cls, s):
        return cls._headstruct.pack(len(s)) + bytes(s)

    @classmethod
    def unpack(cls, payload):

        # first unpack the length
        lengthFieldSize = TYPE_SIZES[cls.__lengthFieldSize__]
        length = struct.unpack_from(
            cls.__byteorder__ + cls.__lengthFieldSize__, payload[:lengthFieldSize])[0]

        # then unpack the string
        string = struct.unpack_from(
            cls.__byteorder__ + str(length) + "s", 
            payload[lengthFieldSize:(lengthFieldSize+length)])[0]
        lengthSize = struct.calcsize(cls.__lengthFieldSize__)
        return length+lengthSize, cls(length, string)

class FlexString32(FlexString):
    pass

class FlexString64(FlexString):
    __lengthFieldSize__ = "I"

class ErrTimeType(CIDType):
    __subtypes__ = ["H", "L", "L"]

    def __init__(self, PwrOnCnt, OpSecs, TimeOccur):
        self.PwrOnCnt = PwrOnCnt
        self.OpSecs = OpSecs
        self.TimeOccur = TimeOccur

class ErrStructType(CIDType):
    __subtypes__ = ["I", "I", ErrTimeType, ErrTimeType, "H", "H", FlexString32]
    def __init__(self, ErrorId, ErrorState, FirstTime, LastTime, NumberOccurance, ErrReserved, ExtInfo):
        self.ErrorId = ErrorId
        self.ErrorState = ErrorState
        self.FirstTime = FirstTime
        self.LastTime = LastTime
        self.NumberOccurance = NumberOccurance
        self.ErrReserved = ErrReserved
        self.ExtInfo = ExtInfo

class EMsgWarning(CIDArray):
    __length__ = 25
    __type__ = ErrStructType

class EMsgError(CIDArray):
    __length__ = 10
    __type__ = ErrStructType

class V3SElectricalMonitoring(CIDType):

    __subtypes__ = ["f", "f", "f", "f"]

    def __init__(self, LEDsCurrent, OperationVoltage, MinimalVoltage, MaximalVoltage):
        self.LEDsCurrent = LEDsCurrent
        self.OperationVoltage = OperationVoltage
        self.MinimalVoltage = MinimalVoltage
        self.MaximalVoltage = MaximalVoltage

class IOConfig(CIDType):

    DIRECTION_INPUT = 0
    DIRECTION_OUTPUT = 1

    PUSHPULLMODE_OPENDRAIN = 0
    PUSHPULLMODE_PUSHPULL = 1

    NPNORPNPMODE_PNP = 0
    NPNORPNPMODE_NPN = 1

    INPUTREACTION_RISING_EDGE = 0
    INPUTREACTION_FALLING_EDGE = 1
    INPUTREACTION_BOTH = 2

    NOTIFICATIONMODE_POLLING = 0
    NOTIFICATIONMODE_IRQ = 1

    EXTERNALTRIGGER_DISABLED = 0
    EXTERNALTRIGGER_ENABLED = 1

    __subtypes__ = ["B", "B", "B", "B", "B", "B", "B"]

    def __init__(self, Direction = DIRECTION_INPUT, PushPullMode = PUSHPULLMODE_OPENDRAIN,
        NPNorPNPMode = NPNORPNPMODE_PNP, InputReaction = INPUTREACTION_RISING_EDGE,
        NotificationMode = NOTIFICATIONMODE_POLLING, SoftwareFilterSettings = 16,
        ExternalTrigger = EXTERNALTRIGGER_DISABLED):

        self.Direction = Direction
        self.PushPullMode = PushPullMode
        self.NPNorPNPMode = NPNorPNPMode
        self.InputReaction = InputReaction
        self.NotificationMode = NotificationMode
        self.SoftwareFilterSettings = SoftwareFilterSettings
        self.ExternalTrigger = ExternalTrigger

class V3SIOsState(CIDType):

    __subtypes__ = ["b", "b", "b", "b", "b", "b"]

    def __init__(self, INOUT1, INOUT2, INOUT3, INOUT4, SENS_IN1, SENS_IN2):

        self.INOUT1 = INOUT1
        self.INOUT2 = INOUT2
        self.INOUT3 = INOUT3
        self.INOUT4 = INOUT4
        self.SENS_IN1 = SENS_IN1
        self.SENS_IN2 = SENS_IN2

class IOFunctionType(CIDType):
    NoFunction              = 0
    SteadyLow               = 1
    SteadyHigh              = 2
    DeviceStatus            = 3
    DataQualityCheck        = 4
    TemperatureWarning      = 5
    DONTUSE_PollutionWarning= 6
    Trigger                 = 7
    DONTUSE_UserStart       = 8

class DoutPinError(CIDType):

    __subtypes__ = ['I']

    def __init__(self, Mask):
        self.Mask = Mask

    def hasError(self, pin):
        """
        Checks if pin n has an error. Starts with pin 0
        """
        return bool(self.Mask & (0b1 << pin))


class V3STemperature(CIDType):
    # specify the type of the member variables
    __subtypes__ = ["B", "B", "B", "B", "B", "B", "B", "B"]

    def __init__(self, 
                 QorIQ, NetX, GigabitEth, Ambient, TempSensor, IOController, Illumination, ImagerBoard):

        self.QorIQ = QorIQ
        self.NetX = NetX
        self.GigabitEth = GigabitEth
        self.Ambient = Ambient
        self.TempSensor = TempSensor
        self.IOController = IOController
        self.Illumination = Illumination
        self.ImagerBoard = ImagerBoard

class Vector3(CIDType):
    __subtypes__ = ["f", "f", "f"]
    
    def __init__(self, X, Y, Z):
        self.X = X
        self.Y = Y
        self.Z = Z

class RotationVector3i(CIDType):
    __subtypes__ = ["h", "h", "h"]
    
    def __init__(self, X, Y, Z):
        self.X = X
        self.Y = Y
        self.Z = Z

class RotationVector3f(CIDType):
    __subtypes__ = ["f", "f", "f"]
    
    def __init__(self, X, Y, Z):
        self.X = X
        self.Y = Y
        self.Z = Z

class BoundingBoxLReal(CIDType):
    __subtypes__ = ["d", "d", "d", "d", "d", "d",]
    
    def __init__(self, X_lower, X_upper, Y_lower, Y_upper, Z_lower, Z_upper):
        self.X_lower = X_lower
        self.X_upper = X_upper
        self.Y_lower = Y_lower
        self.Y_upper = Y_upper
        self.Z_lower = Z_lower
        self.Z_upper = Z_upper

class RangeMm(CIDType):
    __subtypes__ = ["d", "d"]
    
    def __init__(self, lower, upper):
        self.lower = lower
        self.lower = lower

class DetectionResults(CIDType):
    __subtypes__ = [Vector3,Vector3,Vector3,"?"]

    def __init__(self,leftPocket,rightPocket,center,palletFound):
        self.leftPocket = leftPocket
        self.rightPocket = rightPocket
        self.center = center
        self.palletFound = palletFound
