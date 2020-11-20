'''
Created on Feb 22, 2014

@author: waltj
'''
from numbers import Number
import numpy as np
import os
import glob
import sys
import simplekml
import ctypes as ct
import pylib.pose as pose
import pylib.filterTools as ft

# Profiling code 
import time as systime


# Set Reference LLA (deg, deg, m) used for NED - Salem, UT   
refLla = np.r_[ 40.0557114, -111.6585476, 1426.77 ]
gpsWeek = 0

# Set Reference latitude, longitude, height above ellipsoid (deg, deg, m) used for NED calculations
def setRefLla(lla):
    global refLla
    refLla = lla

def setGpsWeek(week): 
    global gpsWeek
    # Search for a valid GPS week
    size = np.shape(week)
    if size and size[0] > 1:
#         if week[0]:
#             week = week[0]
#         else:
#             week = week[-1]
        week = np.max(week)   

    if week > gpsWeek:
        gpsWeek = week

# import time

# Default run behavior
# execfile("..\INS_logger\IsParseLoggerDat.py")


class sChuckHdr(ct.Structure):
    _pack_ = 1
    _fields_ = [('marker',          ct.c_uint32),
                ('version',         ct.c_uint16),
                ('classification',  ct.c_uint16),
                ('name',            ct.c_char * 4),
                ('invName',         ct.c_char * 4),
                ('dataSize',        ct.c_uint32),
                ('invDataSize',     ct.c_uint32),
                ('grpNum',          ct.c_uint32),
                ('serialNum',       ct.c_uint32),
                ('pHandle',         ct.c_uint32),
                ('reserved',        ct.c_uint32)]

class sDataHdr(ct.Structure):
    _pack_ = 1
    _fields_ = [('id',              ct.c_uint32),
                ('size',            ct.c_uint32),
                ('offset',          ct.c_uint32)]

class sDevInfo(ct.Structure):
    _pack_ = 1
    _fields_ = [('reserved',        ct.c_uint32),
                ('serialNumber',    ct.c_uint32),
                ('hardwareVer',     ct.c_uint8 * 4),
                ('firmwareVer',     ct.c_uint8 * 4),
                ('build',           ct.c_uint32),
                ('commVer',         ct.c_uint8 * 4),
                ('repoRevision',    ct.c_uint32),
                ('manufacturer',    ct.c_uint8 * 24),
                ]

class sImu(ct.Structure):
    _pack_ = 1
    _fields_ = [('time',            ct.c_double),
                ('pqr',             ct.c_float * 3),
                ('acc',             ct.c_float * 3),
                ('mag',             ct.c_float * 3),
                ('mslBar',          ct.c_float)]

class sIns1(ct.Structure):
    _pack_ = 1
    _fields_ = [('week',            ct.c_uint32),
                ('time',            ct.c_double),
                ('iStatus',         ct.c_uint32),
                ('hStatus',         ct.c_uint32),
                ('euler',           ct.c_float * 3),
                ('uvw',             ct.c_float * 3),
                ('lla',             ct.c_double * 3),
                ('ned',             ct.c_float * 3),
                ]

class sIns2(ct.Structure):
    _pack_ = 1
    _fields_ = [('week',            ct.c_uint32),
                ('time',            ct.c_double),
                ('iStatus',         ct.c_uint32),
                ('hStatus',         ct.c_uint32),
                ('q',               ct.c_float * 4),
                ('uvw',             ct.c_float * 3),
                ('lla',             ct.c_double * 3),
                ]

class sInsRes(ct.Structure):
    _pack_ = 1
    _fields_ = [('timeMs',          ct.c_uint32),
                ('x_dot.lla',       ct.c_double * 3),
                ('x_dot.uvw',       ct.c_float * 3),
                ('x_dot.q',         ct.c_float * 4),
                ('ned_dot',         ct.c_float * 3),
                ('accCoriolis',     ct.c_float * 3),
                ('geoMagNed',       ct.c_float * 3),
                ('magRefNed',       ct.c_float * 3),
                ('magYawOffset',    ct.c_float),
                ]

class sInsDev1(ct.Structure):
    _pack_ = 1
    _fields_ = [('week',            ct.c_uint32),
                ('time',            ct.c_double),
                ('iStatus',         ct.c_uint32),
                ('hStatus',         ct.c_uint32),
                ('euler',           ct.c_float * 3),
                ('uvw',             ct.c_float * 3),
                ('lla',             ct.c_double * 3),
                ('ned',             ct.c_float * 3),
                ('eulerErr',        ct.c_float * 3),
                ('uvwErr',          ct.c_float * 3),
                ('nedErr',          ct.c_float * 3),
                ]

class sGpsPos(ct.Structure):
    _pack_ = 1
    _fields_ = [('week',            ct.c_uint32),
                ('timeMs',          ct.c_uint32),
                ('satsUsed',        ct.c_uint32),
                ('cno',             ct.c_uint32),
                ('lla',             ct.c_double * 3),
                ('mMSL',            ct.c_float),
                ('hAcc',            ct.c_float),
                ('vAcc',            ct.c_float),
                ('dop',             ct.c_float),
                ]

class sGpsVel(ct.Structure):
    _pack_ = 1
    _fields_ = [('timeMs',          ct.c_uint32),
                ('ned',             ct.c_float * 3),
                ('s2D',             ct.c_float),
                ('s3D',             ct.c_float),
                ('sAcc',            ct.c_float),
                ('course',          ct.c_float),
                ('cAcc',            ct.c_float),
                ]

class sGps(ct.Structure):
    _pack_ = 1
    _fields_ = [('pos',             sGpsPos),
                ('vel',             sGpsVel),
                ('rxps',            ct.c_uint32),
                ('timeOffset',      ct.c_double),
                ]

class sStateVars(ct.Structure):
    _pack_ = 1
    _fields_ = [('lla',             ct.c_double * 3),
                ('uvw',             ct.c_float * 3),
                ('q',               ct.c_float * 4),
                ]

class sInsMisc(ct.Structure):
    _pack_ = 1
    _fields_ = [('time',            ct.c_double),
                ('timeMs',          ct.c_uint32),
                ('x',               sStateVars),
                ('theta',           ct.c_float * 3),
                ('ned',             ct.c_float * 3),
                ('dcm',             ct.c_float * 9),
                ('pqr',             ct.c_float * 3),
                ('acc',             ct.c_float * 3),
                ('mag',             ct.c_float * 3),
                ('mslBar',          ct.c_float),
                ]

class sSysParams(ct.Structure):
    _pack_ = 1
    _fields_ = [('timeMs',          ct.c_uint32),
                ('iStatus',         ct.c_uint32),
                ('hStatus',         ct.c_uint32),
                ('alignAttDetect',  ct.c_float),
                ('alignAttError',   ct.c_float),
                ('alignVelError',   ct.c_float),
                ('alignPosError',   ct.c_float),
                ('insDtMs',         ct.c_uint32),
                ('ftf0',            ct.c_float),
                ('magInclination',  ct.c_float),
                ('magDeclination',  ct.c_float),
                ('genFaultCode',    ct.c_uint32),
                ]

class sInsParams(ct.Structure):
    _pack_ = 1
    _fields_ = [('timeMs',          ct.c_uint32),
                ('debugI4',         ct.c_int32 * 4),
                ('debugF3',         ct.c_float * 3),
                ('debugV3',         ct.c_float * 3),
                ('debugV3g',        ct.c_float * 3),
                ('sensorState',     ct.c_uint32),
                ('obsK.psi',        ct.c_float),
                ('obsK.vel',        ct.c_float),
                ('obsK.pos',        ct.c_float),
                ('theta',           ct.c_float * 3),
                ('magTimeMs',       ct.c_uint32),                
                ('vMagTiltCmp',     ct.c_float * 3),
                ('magHeading',      ct.c_float),
                ('yawErrMag',       ct.c_float),
                ('pqrEffIF',        ct.c_float * 3),
                ('pqrEff',          ct.c_float * 3),
                ('uvwEff',          ct.c_float * 3),
                ]

class sObsParams(ct.Structure):
    _pack_ = 1
    _fields_ = [('timeMs',          ct.c_uint32),
                
                ('uvw.timeMs',      ct.c_uint32),
                ('uvw.ref',         ct.c_float * 3),
                ('uvw.refValid',    ct.c_uint32),
                ('uvw.ins',         ct.c_float * 3),

                ('accNed.timeMs',   ct.c_uint32),
                ('accNed.ref',      ct.c_float * 3),
                ('accNed.refValid', ct.c_uint32),
                ('accNed.ins',      ct.c_float * 3),

                ('velNed.timeMs',   ct.c_uint32),
                ('velNed.ref',      ct.c_double * 3),
                ('velNed.refValid', ct.c_uint32),
                ('velNed.ins',      ct.c_double * 3),

                ('lla.timeMs',      ct.c_uint32),
                ('lla.ref',         ct.c_float * 3),
                ('lla.refValid',    ct.c_uint32),
                ('lla.ins',         ct.c_float * 3),
                
                ('mslBar',          ct.c_float),
                ('mslBarDot',       ct.c_float),
                ('uvwErr',          ct.c_float * 3),
                ('nedErr',          ct.c_float * 3),
                ]

class sDebugArray(ct.Structure):
    _pack_ = 1
    _fields_ = [('i',               ct.c_uint32 * 9),
                ('f',               ct.c_float * 9),
                ('lf',              ct.c_double * 3),
                ]

class sSensorBias(ct.Structure):
    _pack_ = 1
    _fields_ = [('timeMs',          ct.c_uint32),
                ('pqr',             ct.c_float * 3),
                ('acc',             ct.c_float * 3),
                ('mslBar',          ct.c_float),
                ]

class sIo(ct.Structure):
    _pack_ = 1
    _fields_ = [('timeMs',          ct.c_uint32),
                ('gpioStatus',      ct.c_uint32),
                ]

class sIoServos(ct.Structure):
    _pack_ = 1
    _fields_ = [('ch',              ct.c_uint32 * 8),
                ]

#     def getdict(self):
#             dict((f, getattr(self, f)) for f, _ in self._fields_)

def getdict(struct):
    return dict((field, getattr(struct, field)) for field, _ in struct._fields_)

class cCtypeStructures:   
    def devInfo(self):
        return ('devInfo', sDevInfo)         
    def imu(self):
        return ('imu', sImu)
    def ins1(self):
        return ('ins1', sIns1) 
    def ins2(self):
        return ('ins2', sIns2)   
    def insMisc(self):
        return ('insMisc', sInsMisc)
    def sysParams(self):
        return ('sysParams', sSysParams)
    def gpsPos(self):
        return ('gpsPos', sGpsPos)   
    def gpsVel(self):
        return ('gpsVel', sGpsVel)   
    def gps(self):
        return ('gps', sGps)   
    def insResources(self):
        return ('insResources', sInsRes)      
    def sensorBias(self):
        return ('sensorBias', sSensorBias)
    def insParams(self):
        return ('insParams', sInsParams)
    def obsParams(self):
        return ('obsParams', sObsParams)
    def debugArray(self):
        return ('debugArray', sDebugArray)
    def insDev1(self):
        return ('insDev1', sInsDev1)
#     def io(self):
#         return ('io', sIo)   

    def __init__(self):
        # map the inputs to the function blocks
        self.structList = { 
            1  : self.devInfo,
            2  : self.imu,      # INS Input
            3  : self.ins1,
            4  : self.ins2,
            5  : self.gps,
#             6  : self.config,
#             7  : self.asciiBCastPeriod,
            8  : self.insMisc,
            9  : self.sysParams,
#             10 : self.sysSensors,
#             11 : self.flashConfig,
#             12 : self.gpsRssi,
            13 : self.gpsPos,
            14 : self.gpsVel,
#             15 : self.io,
#             16 : self.ioServosPwm,
#             17 : self.ioServosPpm,
#             18 : self.magnetometerCal,
            19 : self.insResources,
            27 : self.sensorBias,
            30 : self.insParams,
            31 : self.obsParams,
            39 : self.debugArray,
            47 : self.insDev1,
        }
          
    def get(self, dHdr):
        try:
            func = self.structList.get(dHdr.id)
            return func()
        except:
            return (None, None)


# Empty class/dictionary
class cObj:
    def __init__(self):
#         self.res = []
        return

                     
def vector3( _v, name ):
    return np.c_[ _v[name+'[0]'].T, _v[name+'[1]'].T, _v[name+'[2]'].T ]
   
def vector4( _v, name ):
    return np.c_[ _v[name+'[0]'].T, _v[name+'[1]'].T, _v[name+'[2]'].T, _v[name+'[3]'].T ]


class cDevice:
    def __init__(self, index, directory, serialNumber, refIns=None ):        
        global refLla
        
        # Profiling
        timeStart = systime.time()
        self.loadTime = 0
        self.unknownId = {}
        self.structs = cCtypeStructures()
        self.directory = directory
        self.serialNumber = serialNumber
        self.rdat = {}      # Raw data in python list format
        self.data = {}      # data in numpy format
        self.index = index  # index in all serial numbers
        self.refLla = refLla
#         self.version = []
#         self.units = []

        if refIns!=None:
            print "#%2d Opening: Ref INS %s" % (index,directory)
    
            fileMask = "LOG_REF_INS*.dat"
            # Use first file in directory if not defined
        else:
            print "#%2d Opening: %s %s" %(index,serialNumber,directory)
            
            fileMask = "LOG_"+serialNumber+"*.dat"

        if not os.path.isdir(directory):
            print "Directory doesn't exist!"
            sys.exit()
        os.chdir(directory)
        self.fileNames = glob.glob(fileMask)
        
        if not self.fileNames:
#             print "   *****   Files not found!  Check directory name and serial number.   *****   "
            raise Exception('Load Error: .dat files not found.')
        
        self.parse()

        # Profiling
        self.loadTime = systime.time() - timeStart
        print "Load time: %.2fs" % (self.loadTime)
    
    
    def parse(self):
        self.curTime = np.r_[0]
                
        # Iterate over files to concatenate data
        for fileName in self.fileNames:
            self.__parseFile(fileName)
        
        # Convert lists (rdat) to numpy arrays (data)
        self.data = self.__toNumpy(self.rdat)
        

    def appendRDat(self, rdat, name, obj):
        dct = getdict( obj )

#         print "Adding: ", name, " ", dct.keys()
#         print "Adding: ", name

#         if 'pos' in dct.keys():
#         if 'gps' == name:
#             j = 1

        if not name in rdat.keys():

            # Create dictionary
            rd = rdat[name] = {}
            #             print ' ' + name + ' (Created)'

            # Is a number or list of numbers NOT a dictionary (structure)
            rd['isNum'] = {}
                               
            for key in dct.keys():
                rd['isNum'][key] = isinstance(dct[key], Number) or isinstance(dct[key], ct.Array)
                
                # Add empty list for numbers or lists or numbers
#                 if rd[dictKey].notDict:
                if rd['isNum'][key]:
                    rd[key] = []
#                     print ' ' + ' ' + key + ' (Created)'

#         print name
        
        # Append new data
        rd = rdat[name]
        for key in dct.keys():

            # Append number or arrays
            if rd['isNum'][key]:
#                 print ' ' + ' ' + key
                try:
                    size = len(dct[key])
                except:
                    size = 0
    
                                    
                if size == 0:
                    rd[key].append(dct[key])
                else:
                    fList = []
                    for i in range(size):
                        fList.append(dct[key][i])
                    rd[key].append(fList)
            # Append dictionary (nested struct)
            else:
                self.appendRDat(rd, key, dct[key])
                
                
    def __parseFile(self, filename):

#         print 'parsing: '+filename
        with open(filename, 'rb') as f:

            cHdr = sChuckHdr()

            # Read Chunk Header
            dHdrSize = ct.sizeof(sDataHdr)
            while f.readinto(cHdr) and cHdr.marker == 0xFC05EA32:
                    
                # Read Chunk Data
                cDat = bytearray(f.read(cHdr.dataSize))

                # Parse Chunk Data
                n = 0
                while n < cHdr.dataSize:
                    # Data Header
                    dHdr = (sDataHdr).from_buffer(cDat,n)
                    n += dHdrSize
                    
#                     print "Found id,size: ", dHdr.id, " ", dHdr.size

#                     if dHdr.id == 1:
#                         j = 1
                    
                    (sName, sType) = self.structs.get(dHdr)

                    # Error check data ID
                    if sType == None:
                        if sType not in self.unknownId.keys():
                            self.unknownId[sType] = 1
                            print "Unknown data id: ", dHdr.id, " size: ", dHdr.size
                        (ct.c_ubyte*dHdr.size).from_buffer(cDat,n)
                        n += dHdr.size
                        continue

                    # Error check data size
                    if ct.sizeof(sType) != dHdr.size:
                        print "Size mismatch, data id: ", dHdr.id, " size: ", dHdr.size
                        (ct.c_ubyte*dHdr.size).from_buffer(cDat,n)
                        n += dHdr.size
                        continue

                    # Add ctypes structures to data dictionary
                    self.appendRDat(self.rdat, sName, (sType).from_buffer(cDat,n) )
                    n += dHdr.size


    # Recursive function that converts dictionaries and lists to numpy arrays                    
    def __toNumpy(self, rdat, depth=-1):
        depth += 1
              
        # Dictionary                  
        if type(rdat) is dict:
            data = {}
            for key in rdat.keys():
#                 print depth, " ", key
#                 if key == 'pos':
#                     depth += 1
                data[key] = self.__toNumpy(rdat[key], depth)                
                    
        # List
        elif type(rdat) is list:
            data = []
            # List of dictionaries (array or structs)
            if type(rdat[0]) is dict:
                for obj in rdat:
                    data.append( self.__toNumpy(obj, depth) )
            # 1D or 2D List of numbers
            elif type(rdat[0]) is list or isinstance(rdat[0], Number):
                # Convert list to numpy arrays
                data = np.array(rdat)
            # List of Objects (nested structs)
            else:
#                 if rdat[0] == 
                
                for obj in rdat:
                    data.append( self.__toNumpy(obj.__dict__, depth) )
                
        else:
            # Add specific value
            data = np.array(rdat)
            
        return data


class cDevices:
    def __init__(self):        
        self.devices = []        
        self.loadTime = 0       # Profiling

    # Load data for to be viewed.  If the "selection.txt" file is found, the line by line contents 
    # of selection.txt specify an additional subdirectory and list of serial numbers to be loaded.  
    # If serial numbers are not specified, either in selection.txt or in the loadData() parameter, 
    # then all serial numbers and files are read. 
    #   directory       Directory data is loaded from.  If not specified, the current directory is used.
    #   serialNumbers   Device serial numbers to load.  If not specified, all serial numbers and files found are loaded.
    #   startDev        First index of found devices (serial numbers) to load.
    #   devCount        Number of devices (serial numbers) to load.
    def loadData(self, directory=None, serialNumbers=None, postProcessed=None, refIns=None, startDev=0, devCount=-1):        

        # Profiling
        self.loadTime = 0
        timeLoadStart = systime.time()

        # We don't support reference INS right now        
        if refIns != None:
            raise Exception('refIns not supported right now.')

        if directory!=None:
            # Convert backslash to forward slash (Windows to Linux)
            directory = directory.replace('\\','/')

            # Automatically open logs specified in "selection.txt" 
            os.chdir(directory)

        # Use selection file if it exists
        selectionFileName = 'selection.txt'
        if os.path.exists(selectionFileName):
            with open(selectionFileName) as f:
                lines = f.read().splitlines()
        
            # Convert backslash to forward slash (Windows to Linux)
            directory += lines[0].replace('\\','/')

            # Read serial numbers from selection.txt
            serialNumbers=lines[1:]
        
        if postProcessed==1 :
            directory += '/post_processed'
                
        # Add all devices in directory
        if serialNumbers == None or serialNumbers == []:            
            # Find list of serial numbers from files in directory
            files = os.listdir(directory)
            serNums = []
            for str in files:
                if str.find('.dat') != -1:
                    str = str.replace('.dat', '')
                    if str.find('LOG_SN') != -1:
                        str = str[4:11]
                        if not str in serNums:
                            serNums.append(str)
                    elif str.find('LOG_P') != -1:
                        str = str.replace('LOG_', '')
                        str = str[:str.find('_')]
                        if not str in serNums:
                            serNums.append(str)
            serialNumbers = serNums

        count = len(serialNumbers)
        
        # Validate serial numbers
        if count <= 0:
            print "No files found..."
            return
           
        # Find size and last index 
        if devCount > 0 and devCount < count:
            count = devCount
        endIndex = min(startDev+count, len(serialNumbers))
        
#         print "Start Index: ", startDev, " End Index: ", endIndex
                  
        # Add devices
        for i in range(startDev, endIndex):
            device = cDevice(i, directory, serialNumbers[i], refIns)                    
            self.devices.append(device)
            
        # Profiling
        self.loadTime = systime.time() - timeLoadStart
        print "Total load time: %.2fs" % (self.loadTime)


def gpsTimeToUTC(gpsWeek, gpsSOW, leapSecs = 14):
    # Search for a valid GPS week
    size = np.shape(gpsWeek)
    if size and size[0] > 1:
#         if gpsWeek[0] == 0:
#             gpsWeek = gpsWeek[-1]
        # Use the largest value for the week
        gpsWeek = np.max(gpsWeek)   
    
    secsInWeek = 604800 
#     secsInDay = 86400 
    gpsEpoch = (1980, 1, 6, 0, 0, 0)  # (year, month, day, hh, mm, ss)     
#     secFract = gpsSOW % 1 
    
    epochTuple = gpsEpoch + (-1, -1, 0)  
    t0 = systime.mktime(epochTuple) - systime.timezone  #mktime is localtime, correct for UTC 
    tdiff = (gpsWeek * secsInWeek) + gpsSOW - leapSecs 
    t = t0 + tdiff
    return t


class cSIMPLE:
    def __init__(self, _v):
        self.v = _v

        
class cIMU:
    def __init__(self, _v):
        global gpsWeek
        self.v = _v
        self.__flt      = cObj()
        self.__flt.pqr  = None
        self.__flt.acc  = None
        self.__flt.pqrNoBias = None
        self.__flt.accNoBias = None
        self.__flt.barNoBias = None
        self.cornerFreqHz = 60
#         self.cornerFreqHz = 30
#         self.cornerFreqHz = 15
        
        self.time = gpsTimeToUTC(gpsWeek, self.v['time'])

    def fltAcc(self):    
        if self.__flt.acc == None:
            self.__flt.acc = ft.lpfNoDelay(self.v['acc'], self.cornerFreqHz, time=self.v['time'])
        return self.__flt.acc
    
    def fltPqr(self):
        if self.__flt.pqr == None:
            self.__flt.pqr = ft.lpfNoDelay(self.v['pqr'], self.cornerFreqHz, time=self.v['time'])
        return self.__flt.pqr
    
    def fltPqrNoBias(self):
        if 'pqrNoBias' in self.v.dtype.names and self.__flt.pqrNoBias==None:
            self.__flt.pqrNoBias = ft.lpfNoDelay(self.v['pqrNoBias'], self.cornerFreqHz, time=self.v['time'])
        return self.__flt.pqrNoBias
    
    def fltAccNoBias(self):
        if 'accNoBias' in self.v.dtype.names and self.__flt.accNoBias==None:
            self.__flt.accNoBias = ft.lpfNoDelay(self.v['accNoBias'], self.cornerFreqHz, time=self.v['time'])
        return self.__flt.accNoBias

    def fltBarNoBias(self):            
        if 'mslBarNoBias' in self.v.dtype.names and self.__flt.barNoBias==None:
            self.__flt.mslBarNoBias = ft.lpfNoDelay(self.v['mslBarNoBias'], self.cornerFreqHz, time=self.v['time'])
        return self.__flt.mslBarNoBias

#         self.mslBar = ft.smooth(self.v['mslBar']+72, delta=200)
#         self.mslBarDot = ft.derivative(self.v['time'], self.mslBar, delta=10)
#         self.mslBarDotLpf = ft.lpfNoDelay(self.mslBarDot, cornerFreqHz=0.5, time = self.v['time'])
        
class cINS:
    def __init__(self, _v):
        self.v = _v
        self.__velNED   = None
        self.__course   = None
        self.__ned      = None
        self.__size     = np.shape(self.v['time'])[0]
        self.time       = gpsTimeToUTC(self.v['week'], self.v['time'])
        
        if not 'euler' in self.v.dtype.names and 'q' in self.v.dtype.names:
            self.v['euler'] = pose.quat2eulerArray(self.v['q'])

        if 'euler' in self.v.dtype.names and not 'q' in self.v.dtype.names:
            self.v['q'] = pose.euler2quatArray(self.v['euler'])

        
    # Velocity vector in inertial frame
    def velNed(self):
        if self.__velNED == None:
            self.__velNED = np.zeros(np.shape(self.v['uvw']))
            
            for i in range(0, self.__size):
                DCM = pose.eulerDCM(self.v['euler'][i,:])
                velNED = np.dot(DCM.T, self.v['uvw'][i,:])      # body to inertial frame
                self.__velNED[i,:] = velNED
                    
        return self.__velNED

    def course(self):
        if self.__course == None:
            self.__course = np.arctan2( self.velNED[:,1], self.velNED[:,0] )
        return self.__course        

    def ned(self):
        global refLla
        if self.__ned == None:
            self.__ned = pose.lla2ned(refLla, self.v['lla'])
        return self.__ned
        
    def set(self, time):
        self.time = time
                
    def speed2D(self):
        return np.sqrt( np.square(self.v['uvw'][:,0]) +
                        np.square(self.v['uvw'][:,1]) )

    def speed3D(self):
        return np.sqrt( np.square(self.v['uvw'][:,0]) +
                        np.square(self.v['uvw'][:,1]) +
                        np.square(self.v['uvw'][:,2]) )


class cRIMU:
    def __init__(self, _v, 
                 accBias = np.r_[0,0,0], 
                 pqrBias = np.r_[0,0,0], 
                 rotate = np.r_[0,0,0]):
        self.v = _v
        self.cornerFreqHz = 30
        self.__flt = cObj()
        self.__flt.pqr  = None
        self.__flt.acc  = None

        if accBias[0]!=0 or accBias[1]!=0 or accBias[2]!=0:
            self.v['acc'] += accBias

        if pqrBias[0]!=0 or pqrBias[1]!=0 or pqrBias[2]!=0:
            self.v['pqr'] += pqrBias

        if rotate[0]!=0 or rotate[1]!=0 or rotate[2]!=0:
            self.v['acc'] = pose.vectorRotateInertialToBody2(self.v['acc'], rotate)
            self.v['pqr'] = pose.vectorRotateInertialToBody2(self.v['pqr'], rotate)

    def fltPqr(self):
        if self.__flt.pqr==None:
            self.__flt.pqr = ft.lpfNoDelay(self.v['pqr'], self.cornerFreqHz, time=self.v['time'])
        return self.__flt.pqr

    def fltAcc(self):
        if self.__flt.acc==None:
            self.__flt.acc = ft.lpfNoDelay(self.v['acc'], self.cornerFreqHz, time=self.v['time'])
        return self.__flt.acc

        
class cRINS:
    def __init__(self, _v, rotate = np.r_[0,0,0]):
        global refLla
        self.v = _v

        self.__ned = None
        self.__nedDotDot = None
        self.__uvw = None
        self.__rotate = rotate

#         self.v['nedDot'] = ft.smooth(self.v['nedDot'], delta=10)
#         self.v['euler'] = ft.smooth(self.v['euler'], delta=10)
        if self.__rotate[0]!=0 or self.__rotate[1]!=0 or self.__rotate[2]!=0:
            self.v['euler'][:,0] += self.__rotate[0]
            self.v['euler'][:,1] += self.__rotate[1]
            self.v['euler'][:,2] += self.__rotate[2]

    def ned(self):
        if self.__ned==None:
            self.__ned = pose.lla2ned(refLla, self.v['lla'])
        return self.__ned

    def nedDotDot(self):
        if self.__nedDotDot==None:
            self.__nedDotDot = ft.derivative(self.v['time'], self.v['nedDot'], delta=2)
            self.__nedDotDot[:,2] -= 9.80665
            cornerFreqHz = 10
            self.__nedDotDot = ft.lpfNoDelay(self.__nedDotDot, cornerFreqHz, time=self.v['time'])
        return self.__nedDotDot
    
    def uvw(self):
        if self.__uvw==None:
            self.__uvw = pose.vectorRotateInertialToBody(self.v['nedDot'], self.v['euler'])
            if self.__rotate[0]!=0 or self.__rotate[1]!=0 or self.__rotate[2]!=0:
                self.uvw = pose.vectorRotateInertialToBody2(self.uvw, self.__rotate)                
        return self.__uvw
        
        
class cRGPS:
    def __init__(self, _v):
        global refLla
        
        self.v = _v
        self.__ned = None
        self.__acc = cObj()
        self.__acc.ned = None

    def ned(self):
        if self.__ned==None:
            self.__ned = pose.lla2ned(refLla, self.v['lla'])
        return self.__ned
    
    def accNed(self):
        if self.__acc.ned == None:
        # Create Accelerations from GPS velocities
#             self.__acc.ned = ft.meanDerivative(self.v['time'], self.v['vel.ned'], 5, 3)
            self.__acc.ned = ft.meanDerivative(self.v['time'], self.v['vel.ned'], 2, 2)
        return self.__acc.ned


class cGPS:
    def __init__(self, _v):
        global refLla
        self.pos = None
        self.vel = None
        self.acc = None
             
        if 'pos' in _v:
            self.pos = cGpsPos(_v['pos'])
             
        if 'vel' in _v:
            self.vel = cGpsVel(_v['vel'])
            

class cGpsVel:
    def __init__(self, _v):
        self.v = _v
#         self.time = _v['timeMs'] * 0.001
        self.time       = gpsTimeToUTC(self.v['week'], (_v['timeMs'] * 0.001))
        self.__acc = None
        self.v['course'] = pose.unwrapAngle(self.v['course'])
        
    def acc(self):
        if self.__acc==None:
            self.__acc = cObj()        
            self.__acc.time = self.time 
    #         self.__acc.ned = ft.meanDerivative(self.vel.time, self.v['ned'], 5, 3)
            self.__acc.ned = ft.meanDerivative(self.time, self.v['ned'], 2, 2)
        return self.__acc 

class cGpsAcc:
    def __init__(self, _v):
        self.v = _v
#         self.time = _v['timeMs'] * 0.001
        self.time       = gpsTimeToUTC(self.v['week'], (_v['timeMs'] * 0.001))

class cGpsPos:
    def __init__(self, _v):
        global refLla
        self.v = _v
#         self.time = _v['timeMs'] * 0.001
        self.time       = gpsTimeToUTC(self.v['week'], (_v['timeMs'] * 0.001))
        self.ned = pose.lla2ned(refLla, _v['lla'])
                   

class cBias:
    def __init__(self, _v):
        global gpsWeek
        self.v = _v
#         self.time = _v['timeMs'] * 0.001
        self.time       = gpsTimeToUTC(gpsWeek, (_v['timeMs'] * 0.001))

class cInsRes:
    def __init__(self, _v):
        global gpsWeek
        self.v = _v
        self.time       = gpsTimeToUTC(gpsWeek, (_v['timeMs'] * 0.001))

class cDevInfo:
    def __init__(self, _v):
        self.v = _v                              
                          
class cSysParams:
    def __init__(self, _v):
        global gpsWeek
        self.v = _v

        if 'time' in _v:
#             self.time               = _v['time']
            self.time       = gpsTimeToUTC(gpsWeek, _v['time'])
        if 'timeMs' in _v:
#             self.time               = (_v['timeMs']) * 0.001
            self.time       = gpsTimeToUTC(gpsWeek, (_v['timeMs'] * 0.001))

        #####   INS Status   #####
        self.aligning = cObj()
        self.aligned = cObj()
        self.aligned.coarse = cObj()
        
        self.aligned.coarse.att     = (_v['iStatus'] >> 0) & 1      # 0-3
        self.aligned.coarse.vel     = (_v['iStatus'] >> 1) & 1
        self.aligned.coarse.pos     = (_v['iStatus'] >> 2) & 1
        
        self.aligned.att            = (_v['iStatus'] >> 4) & 1      # 4-7
        self.aligned.vel            = (_v['iStatus'] >> 5) & 1
        self.aligned.pos            = (_v['iStatus'] >> 6) & 1
        self.aligned.attFine        = (_v['iStatus'] >> 7) & 1
        
        self.aligning.attGps        = (_v['iStatus'] >> 8) & 1      # 8-11
        self.aligning.vel           = (_v['iStatus'] >> 9) & 1
        self.aligning.pos           = (_v['iStatus'] >> 10) & 1        
        self.aligning.attMag        = (_v['iStatus'] >> 11) & 1
 
        self.startupAlignedDynamic  = (_v['iStatus'] >> 12) & 1     
        self.gpsPosValid            = (_v['iStatus'] >> 13) & 1     # 12-15
        self.gpsVelValid            = (_v['iStatus'] >> 14) & 1 
        self.gpsAccValid            = (_v['iStatus'] >> 15) & 1 

        self.insAcc2D               = (_v['iStatus'] >> 16) & 1     # 16-19 
        self.refAcc2D               = (_v['iStatus'] >> 17) & 1
        self.refVel2D               = (_v['iStatus'] >> 18) & 1
        
        self.startupAlignedStatic   = (_v['iStatus'] >> 20) & 1     # 20-23
#         self.startupAlignedDynamic  = (_v['iStatus'] >> 21) & 1     
        self.magCal                 = (_v['iStatus'] >> 22) & 1     
        
        self.biasEstPqr             = (_v['iStatus'] >> 24) & 1     # 24-27
        self.biasEstAcc             = (_v['iStatus'] >> 25) & 1
        self.biasEstBar             = (_v['iStatus'] >> 26) & 1
        self.biasEstPqrStable       = (_v['iStatus'] >> 27) & 1

        # 28-31

        #####   Hardware Status   #####
        self.motionGyrSig           = (_v['hStatus'] >> 0) & 1      # 0-3
        self.motionAccSig           = (_v['hStatus'] >> 1) & 1
        self.motionGyrDev           = (_v['hStatus'] >> 2) & 1
        self.motionAccDev           = (_v['hStatus'] >> 3) & 1
        
        # 4-7

        self.saturationGyr1         = (_v['hStatus'] >> 8) & 1      # 8-11
        self.saturationAcc1         = (_v['hStatus'] >> 9) & 1
        self.saturationMag1         = (_v['hStatus'] >> 10) & 1
        self.saturationBaro         = (_v['hStatus'] >> 11) & 1

        self.saturationGyr2         = (_v['hStatus'] >> 12) & 1      # 12-15
        self.saturationAcc2         = (_v['hStatus'] >> 13) & 1
        self.saturationMag2         = (_v['hStatus'] >> 14) & 1

        self.errComTxLimited        = (_v['hStatus'] >> 16) & 1     # 16-19
        self.errComRxOverrun        = (_v['hStatus'] >> 17) & 1
        self.errGpsTxLimited        = (_v['hStatus'] >> 18) & 1
        self.errGpsRxOverrun        = (_v['hStatus'] >> 19) & 1

        self.errAutobaudFault       = (_v['hStatus'] >> 20) & 1     # 20-23
        self.errComReadFault        = (_v['hStatus'] >> 21) & 1

        self.autobaudDetected       = (_v['hStatus'] >> 24) & 1     # 24-27

        self.faultWatchdogReset     = (_v['hStatus'] >> 28) & 1     # 28-31
        self.faultBODReset          = (_v['hStatus'] >> 29) & 1
        self.faultPORReset          = (_v['hStatus'] >> 30) & 1
        self.faultCPUErrReset       = (_v['hStatus'] >> 31) & 1
        
        # 28-31

class cObsParams:
    def __init__(self, _v):
        global refLla
        self.v = _v
        self.accNed = cObj()
        self.velNed = cObj()
        self.lla = cObj()
        self.uvw = cObj()

#         self.time           = _v['timeMs'] * 0.001
#         self.accNed.time    = _v['accNed.timeMs'] * 0.001
#         self.velNed.time    = _v['velNed.timeMs'] * 0.001
#         self.lla.time       = _v['lla.timeMs'] * 0.001
#         self.uvw.time       = _v['uvw.timeMs'] * 0.001
        self.time           = gpsTimeToUTC(gpsWeek, (_v['timeMs'] * 0.001))
        self.accNed.time    = gpsTimeToUTC(gpsWeek, (_v['accNed.timeMs'] * 0.001))
        self.velNed.time    = gpsTimeToUTC(gpsWeek, (_v['velNed.timeMs'] * 0.001))
        self.lla.time       = gpsTimeToUTC(gpsWeek, (_v['lla.timeMs'] * 0.001))
        self.uvw.time       = gpsTimeToUTC(gpsWeek, (_v['uvw.timeMs'] * 0.001))
                        
        self.accNed.refHdg = np.arctan2( self.v['accNed.ref'][:,1], self.v['accNed.ref'][:,0] ) 
        self.accNed.insHdg = np.arctan2( self.v['accNed.ins'][:,1], self.v['accNed.ins'][:,0] )

        self.lla.refNed     = pose.lla2ned(refLla, _v['lla.ref'])
        self.lla.insNed     = pose.lla2ned(refLla, _v['lla.ins'])
        
#         self.v['mslBar'] += 86;
                          
class cInsParams:
    def __init__(self, _v):
        global gpsWeek
        self.v = _v

#         self.time           = _v['timeMs'] * 0.001
        self.time           = gpsTimeToUTC(gpsWeek, (_v['timeMs'] * 0.001))
        if 'magTimeMs' in _v:
#             self.magTime          = (_v['magTimeMs']) * 0.001
            self.magTime    = gpsTimeToUTC(gpsWeek, (_v['magTimeMs'] * 0.001))



def lla2kml(time, lla, serialNumber, kmlFileName="log.kml", **kwargs ):
    kml = simplekml.Kml()
    
    color = kwargs.pop('color', simplekml.Color.yellow)
    altitudeMode = kwargs.pop('altitudeMode', simplekml.constants.AltitudeMode.absolute)
    timeStep = kwargs.pop('timeStep', 0)

    latLon = []
    tNext = 0
    lNext = 0
    for i in range( 0, np.shape(lla)[0]):
        latLon.append( (lla[i,1], lla[i,0], lla[i,2]) )

        # Add timestamp
        if timeStep:
#             if timeStep == -1:
#                 pt = kml.newpoint(name="%.1f" % time[i], coords=[latLon[i]])
#                 pt.style.iconstyle.color = color
#                 pt.style.iconstyle.scale = 0.5
#                 pt.style.labelstyle.scale = 0.7
            if time[i] >= tNext:
                tNext += timeStep
#                 round(tNext, timeStep)
                if time[i] >= lNext:
                    if timeStep > lNext:
                        lNext += timeStep
                    else:
                        lNext += 1
                    pt = kml.newpoint(name="%.2f" % time[i], coords=[latLon[i]])
                else:
                    pt = kml.newpoint(coords=[latLon[i]])
                pt.style.iconstyle.color = color
                pt.style.iconstyle.scale = 0.4
                pt.style.labelstyle.scale = 0.6
                pt.altitudemode = altitudeMode

    # Add path    
    ls = kml.newlinestring(name="Tracks", description=serialNumber+" tracks", coords=latLon)
    
    # Style
    ls.extrude = 1 
    ls.altitudemode = altitudeMode
    ls.style.linestyle.width = 2
    ls.style.linestyle.color = color
    kml.save(kmlFileName)
    
    return kmlFileName

