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

from numpy import uint8 as u8
from numpy import uint16 as u16
from numpy import uint32 as u32
from numpy import int32  as i32
from numpy import float32 as f32
from numpy import int64 as i64
from numpy import float64 as f64
import datetime

# Set Reference LLA (deg, deg, m) used for NED - Salem, UT
refLla = np.r_[40.0557114, -111.6585476, 1426.77]
gpsWeek = 0
showUtcTime = 0


# Set Reference latitude, longitude, height above ellipsoid (deg, deg, m) used for NED calculations
def setRefLla(lla):
    global refLla
    refLla = lla


def setShowUtcTime(show):
    global showUtcTime
    showUtcTime = show

WEEK_TIME = []
def setGpsWeek(week):
    global gpsWeek
    global WEEK_TIME
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

    GPS_start_Time = datetime.datetime.strptime('6/Jan/1980', "%d/%b/%Y")
    WEEK_TIME = GPS_start_Time + (datetime.timedelta(weeks=int(week)))

def getTimeFromTowMs(ms):
    global WEEK_TIME
    return [WEEK_TIME + datetime.timedelta(milliseconds=int(i)) for i in ms]

def getTimeFromTow(s):
    global WEEK_TIME
    return [WEEK_TIME + datetime.timedelta(seconds=float(i)) for i in s]

def getTimeFromGTime(gtime):
    GPS_start_Time = datetime.datetime.strptime('1/Jan/1970', "%d/%b/%Y")
    return [GPS_start_Time + datetime.timedelta(seconds=float(t['time'] + t['sec'])) for t in gtime]

# import time

# Default run behavior
# execfile("..\INS_logger\IsParseLoggerDat.py")


#     def getdict(self):
#             dict((f, getattr(self, f)) for f, _ in self._fields_)


# Empty class/dictionary
class cObj:
    def __init__(self):
        #         self.res = []
        return


class cDataType:
    def __init__(self, name='', dtype=0):
        self.name = name
        self.dtype = dtype

    def set(self, name, dtype):
        self.name = name
        self.dtype = dtype


#     def nameID(self, did, name ):
#         self.id = did
#         self.name = name
#
#     def dType(self, dtype):
#         self.dtype = dtype

def vector3(_v, name):
    return np.c_[_v[name + '[0]'].T, _v[name + '[1]'].T, _v[name + '[2]'].T]


def vector4(_v, name):
    return np.c_[_v[name + '[0]'].T, _v[name + '[1]'].T, _v[name + '[2]'].T, _v[name + '[3]'].T]


RAW_DATA_OBS = 1,
RAW_DATA_EPH = 2,
RAW_DATA_GEPH = 3,
RAW_DATA_SBAS = 4,
RAW_DATA_STA = 5,
RAW_DATA_RTK_SOL = 123

dtypeGpsRaw = np.dtype([
    ('dataSerNum', u32),
    ('receiverIndex', u8),
    ('type', u8),
    ('count', u8),
    ('reserved', u8)
])

dtypeGtime = np.dtype([
    ('time', i64),
    ('sec', f64)])

dtypeEph = np.dtype([
    ('sat', i32),
    ('iode', i32),
    ('iodc', i32),
    ('sva', i32),
    ('svh', i32),
    ('week', i32),
    ('code', i32),
    ('flag', i32),
    ('toe', dtypeGtime),
    ('toc', dtypeGtime),
    ('ttr', dtypeGtime),
    ('A', f64),
    ('e', f64),
    ('i0', f64),
    ('OMG0', f64),
    ('omg', f64),
    ('M0', f64),
    ('deln', f64),
    ('OMGd', f64),
    ('idot', f64),
    ('crc', f64),
    ('crs', f64),
    ('cuc', f64),
    ('cus', f64),
    ('cic', f64),
    ('cis', f64),
    ('toes', f64),
    ('fit', f64),
    ('f0', f64),
    ('f1', f64),
    ('f2', f64),
    ('tgd', (f64, 4)),
    ('Adot', f64),
    ('ndot', f64),
])

dtypeGEph = np.dtype([
    ('sat', i32),
    ('iode', i32),
    ('frq', i32),
    ('svh', i32),
    ('sva', i32),
    ('age', i32),
    ('toe', dtypeGtime),
    ('tof', dtypeGtime),
    ('pos', (f64, 3)),
    ('vel', (f64, 3)),
    ('acc', (f64, 3)),
    ('taun', f64),
    ('gamn', f64),
    ('dtaun', f64)
])

dtypeSbas = np.dtype([
    ('week', i32),
    ('tow', i32),
    ('prn', i32),
    ('msg', (u8, 29)),
    ('reserved', (u8, 3)),
])

dtypeSta = np.dtype([
    ('deltype', i32),
    ('pos', (f32, 3)),
    ('delta', (f32, 3)),
    ('hgt', f32),
    ('stationId', i32),
])

dtypeObsD = np.dtype([
    ('time', dtypeGtime),
    ('sat', u8),
    ('rcv', u8),
    ('SNR', u8),
    ('LLI', u8),
    ('code', u8),
    ('qualL', u8),
    ('qualP', u8),
    ('reserved', u8),
    ('L', f64),
    ('P', f64),
    ('D', f32)
])


class cDevice:
    def __init__(self, index, directory, serialNumber, refIns=None):
        global refLla
        self.unknownDidDisplayed = {}
        self.serialNumber = serialNumber

        self.dtCnkHdr = np.dtype([
            ('marker', u32),
            ('version', u16),
            ('classification', u16),
            ('name', np.dtype((str, 4))),
            ('invName', np.dtype((str, 4))),
            ('dataSize', u32),
            ('invDataSize', u32),
            ('grpNum', u32),
            ('devSerialNum', u32),
            ('pHandle', u32),
            ('reserved', u32),
        ])

        self.dtCnkSubHdr = np.dtype([
            ('dHdr', [('id', u32),
                      ('size', u32),
                      ('offset', u32), ]),
            ('dCount', u32),
        ])

        # Data info
        self.DID_COUNT = 73
        self.di = [cDataType() for i in range(self.DID_COUNT)]

        self.di[1].set('devInfo', np.dtype([
            ('dataSerNum', u32),  # Indicates serial order in time
            ('reserved', u32),
            ('serialNumber', u32),
            ('hardwareVer', (u8, 4)),
            ('firmwareVer', (u8, 4)),
            ('build', u32),
            ('commVer', (u8, 4)),
            ('repoRevision', f32),
            ('manufacturer', np.dtype((str, 24))),
            ('buildDate', (u8, 4)),
            ('buildTime', (u8, 4)),
            ('addInfo', np.dtype((str, 24))),
        ]))

        dtypeImu = np.dtype([
            ('pqr', (f32, 3)),
            ('acc', (f32, 3)),
        ])

        # 2   'crashInfo'

        self.di[3].set('preintegratedImu', np.dtype([
            ('dataSerNum', u32),  # Indicates serial order in time
            ('time', f64),
            ('theta1', (f32, 3)),
            ('theta2', (f32, 3)),
            ('vel1', (f32, 3)),
            ('vel2', (f32, 3)),
            ('dt', f32),
        ]))

        self.di[4].set('ins1', np.dtype([
            ('dataSerNum', u32),  # Indicates serial order in time
            ('week', u32),
            ('tow', f64),
            ('iStatus', u32),
            ('hStatus', u32),
            ('euler', (f32, 3)),
            ('uvw', (f32, 3)),
            ('lla', (f64, 3)),
            ('ned', (f32, 3)),
        ]))

        self.di[5].set('ins2', np.dtype([
            ('dataSerNum', u32),  # Indicates serial order in time
            ('week', u32),
            ('tow', f64),
            ('iStatus', u32),
            ('hStatus', u32),
            ('q', (f32, 4)),
            ('uvw', (f32, 3)),
            ('lla', (f64, 3)),
        ]))

        dtypeGpsPos = np.dtype([
            ('dataSerNum', u32),  # Indicates serial order in time
            ('week', u32),
            ('timeOfWeekMs', u32),
            ('status', u32),
            ('ecef', (f64, 3)),
            ('lla', (f64, 3)),
            ('hMSL', f32),
            ('hAcc', f32),
            ('vAcc', f32),
            ('pDop', f32),
            ('cnoMean', f32),
            ('towOffset', f64)
        ])

        self.di[6].set('gps1UbxPos', dtypeGpsPos)

        # 7  'config'
        # 8  'asciiBCastPeriod'

        dtStartVars = np.dtype([
            ('lla', (f64, 3)),
            ('uvw', (f32, 3)),
            ('q', (f32, 4)),
        ])

        self.di[9].set('insMisc', np.dtype([
            ('dataSerNum', u32),  # Indicates serial order in time
            ('tow', f64),
            ('towMs', u32),
            ('x', dtStartVars),
            ('theta', (f32, 3)),
            ('ned', (f32, 3)),
            ('dcm', (f32, 9)),
            ('pqr', (f32, 3)),
            ('acc', (f32, 3)),
            ('mag', (f32, 3)),
            ('mslBar', f32),
        ]))

        self.di[10].set('sysParams', np.dtype([
            ('dataSerNum', u32),  # Indicates serial order in time
            ('towMs', u32),
            ('iStatus', u32),
            ('hStatus', u32),
            ('imuTemp', f32),
            ('baroTemp', f32),
            ('mcuTemp', f32),
            ('reserved1', f32),
            ('sampleDtMs', u32),
            ('insDtMs', u32),
            ('reserved2', (f32, 4)),
            ('genFaultcode', u32),
        ]))

        # 11 'sysSensors'

        self.di[12].set('flashConfig', np.dtype([
            ('dataSerNum', u32),  # Indicates serial order in ti
            ('size', u32),
            ('checksum', u32),
            ('key', u32),
            ('startupSampleDtMs', u32),
            ('startupNavDtMs', u32),
            ('ser0BaudRate', u32),
            ('ser1BaudRate', u32),
            ('insRotation', (f32, 3)),
            ('insOffset', (f32, 3)),
            ('gps1AntOffset', (f32, 3)),
            ('insDynModel', u32),
            ('sysCfgBits', u32),
            ('refLla', (f64, 3)),
            ('lastLla', (f64, 3)),
            ('lastLlaTimeOfWeekMs', u32),
            ('lastLlaWeek', u32),
            ('lastLlaUpdateDistance', f32),
            ('ioConfig', u32),
            ('cBrdConfig', u32),
            ('gps2AntOffset', (f32, 3)),
            ('zeroVelRotation', (f32, 3)),
            ('zeroVelOffset', (f32, 3)),
            ('magInclination', f32),
            ('magDeclination', f32),
            ('gpsTimeSyncPulsePeriodMs', u32),
            ('startupGPSDtMs', u32),
            ('RTKCfgBits', u32),
            ('reserved', u32),
        ]))

        self.di[13].set('gps1Pos', dtypeGpsPos)
        self.di[14].set('gps2Pos', dtypeGpsPos)

        # 15 'gps1Cno'
        # 16 'gps2Cno'
        # 17 'gps2Version'
        # 18 'gps2Version'
        # 19 'magCal'

        self.di[20].set('insResources', np.dtype([
            ('dataSerNum', u32),  # Indicates serial order in time
            ('towMs', u32),
            ('x_dot', dtStartVars),
            ('magYawOffset', f32),
        ]))

        self.di[21].set('gps1RtkPosRel', np.dtype([
            ('dataSerNum', u32),  # Indicates serial order in time
            ('timeOfWeekMs', u32 ),
            ('differentialAge', f32 ),
            ('arRatio', f32 ),
            ('vectorToBase', (f32, 3)),
            ('distanceToBase', f32 ),
            ('headingToBase', f32 ),
        ]))

        self.di[22].set('gps1RtkPosMisc', np.dtype([
            ('dataSerNum', u32),  # Indicates serial order in time
            ('timeOfWeekMs', u32),
            ('accuracyPos', (f32, 3)),
            ('accuracyCov', (f32, 3)),
            ('arThreshold', f32),
            ('gDop', f32),
            ('hDop', f32),
            ('vDop', f32),
            ('baseLla', (f64, 3)),
            ('cycleSlipCount', u32),
            ('roverGpsObservationCount', u32),
            ('baseGpsObservationCount', u32),
            ('roverGlonassObservationCount', u32),
            ('baseGlonassObservationCount', u32),
            ('roverGalileoObservationCount', u32),
            ('baseGalileoObservationCount', u32),
            ('roverBeidouObservationCount', u32),
            ('baseBeidouObservationCount', u32),
            ('roverQzsObservationCount', u32),
            ('baseQzsObservationCount', u32),
            ('roverGpsEphemerisCount', u32),
            ('baseGpsEphemerisCount', u32),
            ('roverGlonassEphemerisCount', u32),
            ('baseGlonassEphemerisCount', u32),
            ('roverGalileoEphemerisCount', u32),
            ('baseGalileoEphemerisCount', u32),
            ('roverBeidouEphemerisCount', u32),
            ('baseBeidouEphemerisCount', u32),
            ('roverQzsEphemerisCount', u32),
            ('baseQzsEphemerisCount', u32),
            ('roverSbasCount', u32),
            ('baseSbasCount', u32),
            ('baseAntennaCount', u32),
            ('ionUtcAlmCount', u32)
             ]))

        # 23 'Feature Bits'

        dtypeSensorsMpuWTemp = np.dtype([
            ('pqr', (f32, 3)),
            ('acc', (f32, 3)),
            ('mag', (f32, 3)),
            ('temp', f32),
        ])

        self.di[24].set('sensorsIs1', np.dtype([
            ('dataSerNum', u32),  # Indicates serial order in time
            ('mpu', (dtypeSensorsMpuWTemp, 2)),
        ]))

        # 25 'Sensor IS2'
        # 26 'Sensor TC Bias'

        self.di[27].set('sensorBias', np.dtype([
            ('dataSerNum', u32),  # Indicates serial order in time
            ('towMs', u32),
            ('pqr', (f32, 3)),
            ('acc', (f32, 3)),
            ('mslBar', f32),
            ('magI', (f32, 3)),
            ('magB', (f32, 3)),
        ]))

        # 28 'Sensor ADC'
        # 29 'SCOMP'

        dtypeGpsVel = np.dtype([
            ('dataSerNum', u32),  # Indicates serial order in time
            ('timeOfWeekMs', u32),
            ('velEcef', (f32, 3)),
            ('sAcc', f32)
        ])

        self.di[30].set('gps1Vel', dtypeGpsVel)
        self.di[31].set('gps2Vel', dtypeGpsVel)

        # 32 'HDW params'
        # 33-37 Flash
        # 38 'RTOS Info'

        self.di[39].set('debugArray', np.dtype([
            ('dataSerNum', u32),  # Indicates serial order in time
            ('i', (i32, 9)),
            ('f', (f32, 9)),
            ('lf', (f64, 3)),
        ]))

        self.di[47].set('insDev1', np.dtype([
            ('dataSerNum', u32),  # Indicates serial order in time
            ('week', u32),
            ('tow', f64),
            ('iStatus', u32),
            ('hStatus', u32),
            ('euler', (f32, 3)),
            ('uvw', (f32, 3)),
            ('lla', (f64, 3)),
            ('ned', (f32, 3)),
            ('eulerErr', (f32, 3)),
            ('uvwErr', (f32, 3)),
            ('nedErr', (f32, 3)),
        ]))

        self.di[48].set('ekfStates', np.dtype([
            ('dataSerNum', u32),  # Indicates serial order in time
            ('time', f64),
            ('qe2b', (f32, 4)),
            ('ve', (f32, 3)),
            ('ecef', (f64, 3)),
            ('biasPqr', (f32, 3)),
            ('biasAcc', (f32, 3)),
            ('biasBaro', f32),
            ('magDec', f32),
            ('magInc', f32),
        ]))

        # 49 'EKF Covariance'
        # 50 'EKF Innovations'
        # 51 'EKF Innovations Var'

        self.di[52].set('magnetometer1', np.dtype([
            ('dataSerNum', u32),  # Indicates serial order in time
            ('time', f64),
            ('mag', (f32, 3)),
        ]))

        self.di[53].set('barometer', np.dtype([
            ('dataSerNum', u32),  # Indicates serial order in time
            ('time', f64),
            ('bar', f32),
            ('mslBar', f32),
            ('barTemp', f32),
            ('humidity', f32),
        ]))

        self.di[54].set('gps1RtkPos', dtypeGpsPos)

        self.di[55].set('gps1RtkCmpRel', np.dtype([
            ('dataSerNum', u32),  # Indicates serial order in time
            ('timeOfWeekMs', u32 ),
            ('differentialAge', f32 ),
            ('arRatio', f32 ),
            ('vectorToBase', (f32, 3)),
            ('distanceToBase', f32 ),
            ('headingToBase', f32 ),
        ]))

        self.di[56].set('gpsVersion', np.dtype([
            ('dataSerNum', u32),  # Indicates serial order in time
            ('swVersion', np.dtype((str, 30))),
            ('hwVersion', np.dtype((str, 10))),
            ('extension', np.dtype((str, 30))),
            ('reserved', (u32, 2)),
        ]))

        # 57 'Communications Loopback'

        self.di[58].set('dualImu', np.dtype([
            ('dataSerNum', u32),  # Indicates serial order in time
            ('time', f64),
            ('I', (dtypeImu, 2)),
        ]))

        self.di[59].set('inl2MagObs', np.dtype([
            ('dataSerNum', u32),  # Indicates serial order in time
            ('towMs', u32),
            ('Ncal_samples', u32),
            ('ready', u32),
            ('calibrated', u32),
            ('auto-recal', u32),
            ('outlier', u32),
            ('magHeading', f32),
            ('insHeading', f32),
            ('magInsHdgDelta', f32),
            ('nis', f32),
            ('nis_threshold', f32),
            ('Wcal', (f32, 9)),
            ('activeCalSet', u32),
            ('magHeadingOffset', f32),
        ]))

        # 60 - Raw GPS Ephemeris and Observation from Base
        self.di[60].set('GPSBaseRaw', dtypeGpsRaw)

        # 61 - RTK Options
        # 62 - Internal User page Info
        # 63 - Manufacturing Info
        # 64 - Self Test
        # 65 - INS - 3 - ECEF Position & Quaternions NED
        # 66 - INS - 4 - ECEF Position & Quaternions ECEF

        self.di[67].set('inl2Variance', np.dtype([
            ('dataSerNum', u32),  # Indicates serial order in time
            ('towMs', u32),
            ('PxyxNED', (f32, 3)),
            ('PvelNED', (f32, 3)),
            ('PattNED', (f32, 3)),
            ('PABias', (f32, 3)),
            ('PWBias', (f32, 3)),
            ('PBaroBias', f32),
            ('PDeclination', f32),
        ]))

        # 68 - Strobe input time

        self.di[69].set('GPS1Raw', dtypeGpsRaw)
        self.di[70].set('GPS2Raw', dtypeGpsRaw)

        self.di[91].set('gps1RtkCmpMisc', np.dtype([
            ('dataSerNum', u32),  # Indicates serial order in time
            ('timeOfWeekMs', u32),
            ('accuracyPos', (f32, 3)),
            ('accuracyCov', (f32, 3)),
            ('arThreshold', f32),
            ('gDop', f32),
            ('hDop', f32),
            ('vDop', f32),
            ('baseLla', (f64, 3)),
            ('cycleSlipCount', u32),
            ('roverGpsObservationCount', u32),
            ('baseGpsObservationCount', u32),
            ('roverGlonassObservationCount', u32),
            ('baseGlonassObservationCount', u32),
            ('roverGalileoObservationCount', u32),
            ('baseGalileoObservationCount', u32),
            ('roverBeidouObservationCount', u32),
            ('baseBeidouObservationCount', u32),
            ('roverQzsObservationCount', u32),
            ('baseQzsObservationCount', u32),
            ('roverGpsEphemerisCount', u32),
            ('baseGpsEphemerisCount', u32),
            ('roverGlonassEphemerisCount', u32),
            ('baseGlonassEphemerisCount', u32),
            ('roverGalileoEphemerisCount', u32),
            ('baseGalileoEphemerisCount', u32),
            ('roverBeidouEphemerisCount', u32),
            ('baseBeidouEphemerisCount', u32),
            ('roverQzsEphemerisCount', u32),
            ('baseQzsEphemerisCount', u32),
            ('roverSbasCount', u32),
            ('baseSbasCount', u32),
            ('baseAntennaCount', u32),
            ('ionUtcAlmCount', u32)
        ]))

        # Profiling
        timeStart = systime.time()
        self.loadTime = 0
        self.unknownId = {}
        self.directory = directory
        self.serialNumber = serialNumber
        self.rdat = {}  # Raw data in python list format
        self.data = {}  # data in numpy format
        self.index = index  # index in all serial numbers
        self.refLla = refLla
        #         self.version = []
        #         self.units = []

        if refIns is not None:
            print("#%2d Opening: Ref INS %s" % (index, directory))

            fileMask = "LOG_REF_INS*.dat"
            # Use first file in directory if not defined
        else:
            print("#%2d Opening: %s %s" % (index, serialNumber, directory))

            fileMask = "LOG_" + serialNumber + "*.sdat"

        if not os.path.isdir(directory):
            print("Directory doesn't exist!")
            sys.exit()
        os.chdir(directory)
        self.fileNames = glob.glob(fileMask)

        if not self.fileNames:
            #             print("   *****   Files not found!  Check directory name and serial number.   *****   ")
            raise Exception('Load Error: .sdat files not found.')

        self.parse()
        self.clean()

        # Profiling
        self.loadTime = systime.time() - timeStart
        print("Load time: %.2fs" % (self.loadTime))

    def clean(self):
        for key, item in self.data.iteritems():
            if not isinstance(item, np.ndarray):
                continue

            for field in ['towMs', 'timeOfWeekMs', 'tow']:
                if field in item.dtype.names:
                    if (np.diff(item[field].astype(np.int64)) < 0).any():
                        idx = np.argmin(np.diff(item[field].astype(np.int64)))
                        print("\033[93m" + "Time went backwards in ", key, r"!!!, removing all data " + ("before" if idx < len(item[field])/2.0 else "after") + "\033[0m")
                        if idx < len(item[field])/2.0:
                            self.data[key] = item[idx +1:]
                        else:
                            self.data[key] = item[:idx]
                    ms_multiplier = 1000.0 if 'Ms' in field else 1.0
                    if (np.diff(item[field]) > 3600 * ms_multiplier).any():
                        print("\033[93m" + "greater than 1 minute gap in ", key, " data, assuming GPS fix was acquired during data set, and chopping data"+ "\033[0m")
                        idx = np.argmax(np.diff(item[field])) + 1
                        self.data[key] = item[idx:]

    def parse(self):
        self.curTime = np.r_[0]
        self.raw_gps_keys = []

        # Iterate over files to concatenate data
        self.fileNames.sort()
        for fileName in self.fileNames:
            print(fileName)
            self.__parseFile(fileName)
        # set the raw GPS dictionary as a datatype
        for name in self.raw_gps_keys:
            for key, item in self.data[name].iteritems():
                self.data[name][key] = np.array(item)
        if 'ins2' in self.data.keys():
            setGpsWeek(self.data['ins2']['week'][0])


    def parse_raw_gps(self, f, did, dati, sHdr, cHdr):

        valid_types = [1, 2, 3, 4, 5, 6, 123]
        valid_receiver_indexes = [1, 2, 3, 4]

        if dati.name not in self.raw_gps_keys:
            self.raw_gps_keys.append(dati.name)

        buf = np.fromfile(f, np.uint8, count=cHdr['dataSize'])
        for i in range(sHdr['dCount']):
            pointer = 0
            hdr_size = np.dtype(dtypeGpsRaw).itemsize
            gps_raw_header = buf[pointer:pointer + hdr_size].view(dtypeGpsRaw)
            pointer += hdr_size

            # Pull in the header data
            try:
                type = gps_raw_header['type'][0]
                count = gps_raw_header['count'][0]
                receiverIndex = gps_raw_header['receiverIndex'][0]
                assert (type in valid_types and receiverIndex in valid_receiver_indexes)
                if dati.name not in self.data.keys():
                    self.data[dati.name] = {'dataSerNum': [gps_raw_header['dataSerNum'][0]],
                                            'receiverIndex': [receiverIndex],
                                            'type': [type],
                                            'count': [count],
                                            'corrupt_data': 0}
                else:
                    self.data[dati.name]['dataSerNum'].append(gps_raw_header['dataSerNum'][0])
                    self.data[dati.name]['receiverIndex'].append(gps_raw_header['receiverIndex'][0])
                    self.data[dati.name]['type'].append(type)
                    self.data[dati.name]['count'].append(count)
            except:
                print("invalid raw gps header: type=", type, "count = ", count, "receiverIndex = ", receiverIndex)
                self.data[dati.name]['corrupt_data'] += 1
                continue

            if type == RAW_DATA_OBS:
                try:
                    bytes_in_payload = np.dtype(dtypeObsD).itemsize * count
                    obs = buf[pointer:pointer + bytes_in_payload].view(dtypeObsD)
                    pointer += bytes_in_payload
                    if 'obs' not in self.data[dati.name]:
                        self.data[dati.name]['obs'] = np.rec.array(obs)
                    else:
                        self.data[dati.name]['obs'] = np.hstack((self.data[dati.name]['obs'], np.rec.array(obs)))
                except:
                    print("badly formed raw gps data - DID: %d type: Obs, count: %d, actual: %f" %
                          (did, count, (len(buf) - 8) / (float(np.dtype(dtypeObsD).itemsize))))
                    self.data[dati.name]['corrupt_data'] += 1
                    continue

    def __parseFile(self, filename):
        with open(filename, 'rb') as f:

            while 1:
                # Read and validate chunk header
                cHdr = np.fromfile(f, dtype=self.dtCnkHdr, count=1)

                count = cHdr['dataSize']

                if np.shape(cHdr)[0] == 0 or cHdr['marker'][0] != 0xFC05EA32:
                    #                     print( "Done parsing data!" )
                    break

                # Read chunk sub header
                sHdr = np.fromfile(f, dtype=self.dtCnkSubHdr, count=1)

                # Find ID
                did = sHdr['dHdr']['id'][0]
                dsize = sHdr['dHdr']['size'][0]
                #                 if did == 6:
                #                     print( "DID: ",did )
                if did >= self.DID_COUNT:
                    if did not in self.unknownDidDisplayed.keys():
                        self.unknownDidDisplayed[did] = True
                        print("==============================================================================")
                        print(" - ERROR - Data ID " + str(did) + " out of range " + str(
                            self.DID_COUNT) + ". Please add missing DID definitions to ISToolsDataSorted.pyx.")
                        print("==============================================================================")
                    did = 0
                    self.unknownDidDisplayed[did] = True
                    systime.sleep(0.5)

                dati = self.di[did]

                if dati.dtype:
                    if dsize == (dati.dtype.itemsize - 4):
                        # Known data type
                        #                     print("Found id: ", did)
                        cDat = np.fromfile(f, dati.dtype, count=sHdr['dCount'])

                        if dati.name in self.data.keys():
                            # Append
                            #                     self.data[dati.name].append(cDat)
                            self.data[dati.name] = np.concatenate([self.data[dati.name], cDat])
                        else:
                            # Create
                            self.data[dati.name] = cDat

                    # Handle Raw data differently (because it changes sizes and carries multiple messages)
                    elif dati.dtype == dtypeGpsRaw:
                        self.parse_raw_gps(f, did, dati, sHdr, cHdr)

                    else:
                        # Mismatched data size
                        print("==============================================================================")
                        print(" - ERROR - Data ID", did, "(" + dati.name + ") mismatched size.  Read", dsize, "expected", dati.dtype.itemsize - 4)
                        print("==============================================================================")
                        # systime.sleep(0.5)
                        #                         sys.exit()
                        cDat = np.fromfile(f, np.uint8, count=cHdr['dataSize'][0])
                else:
                    # Unknown data type
                    if did not in self.unknownDidDisplayed.keys():
                        self.unknownDidDisplayed[did] = True
                        print("Undefined DID: ", did)
                    cDat = np.fromfile(f, np.uint8, count=cHdr['dataSize'][0])


class cDevices:
    def __init__(self):
        self.devices = []
        self.loadTime = 0  # Profiling

    # Load data to be viewed.  If the "selection.txt" file is found, the line by line contents
    # of selection.txt specify an additional subdirectory and list of serial numbers to be loaded.
    # If serial numbers are not specified, either in selection.txt or in the loadData() parameter,
    # then all serial numbers and files are read.
    #   directory       Directory data is loaded from.  If not specified, the current directory is used.  If no data found, use latest data sub directory.
    #   serialNumbers   Device serial numbers to load.  If not specified, all serial numbers and files found are loaded.
    #   startDev        First index of found devices (serial numbers) to load.
    #   devCount        Number of devices (serial numbers) to load.
    def loadData(self, directory=None, serialNumbers=None, refIns=None, startDev=0, devCount=-1):

        # Profiling
        self.loadTime = 0
        timeLoadStart = systime.time()

        # We don't support reference INS right now
        if refIns != None:
            raise Exception('refIns not supported right now.')

        if directory is not None:
            # Convert backslash to forward slash (Windows to Linux)
            directory = directory.replace('\\', '/')

            if '~' in directory:
                pass

            # Automatically open logs specified in "selection.txt"
            os.chdir(directory)

        # Use selection file if it exists
        selectionFileName = 'selection.txt'
        if os.path.exists(selectionFileName):
            with open(selectionFileName) as f:
                lines = f.read().splitlines()

            # Convert backslash to forward slash (Windows to Linux)
            directory += lines[0].replace('\\', '/')

            # Read serial numbers from selection.txt
            serialNumbers = []
            for serNum in lines[1:]:
                # Stop if we find a blank line
                if serNum == '':
                    break
                serialNumbers.append(serNum)

        # If current directory has NO data, use newest sub directory containing data.
        files = os.listdir(directory)
        if not any(".sdat" in s for s in files):
            dirName = None
            dirTime = 0
            for fname in files:
                # Has data log directory name format
                if len(fname) >= 15 and fname[0:2] == '20' and fname[8:9] == '_':
                    dTime = int(fname[0:8] + fname[9:15])
                    # Is latest
                    if dTime > dirTime:
                        dirTime = dTime
                        dirName = fname

            if dirName != None:
                directory += dirName

        # Print directory
        print("Loading Data: ", directory)

        # Add all devices in directory
        if serialNumbers is None or serialNumbers == []:
            # Find list of serial numbers from files in directory
            files = os.listdir(directory)
            serNums = []
            for str in files:
                if str.find('.sdat') != -1:
                    str = str.replace('.sdat', '')
                    if str.find('LOG_SN') != -1:
                        str = str[4:11]
                        if not str in serNums:
                            serNums.append(str)
                    elif str.find('LOG_PR') != -1:
                        str = str.replace('LOG_', '')
                        str = str[:str.find('_')]
                        if not str in serNums:
                            serNums.append(str)
            serialNumbers = serNums

        count = len(serialNumbers)

        # Validate serial numbers
        if count <= 0:
            raise Exception('Load Error: .sdat files not found.')

        # Find size and last index
        if devCount > 0 and devCount < count:
            count = devCount
        endIndex = min(startDev + count, len(serialNumbers))

        #         print ("Start Index: ", startDev, " End Index: ", endIndex)

        # Add devices
        for i in range(startDev, endIndex):
            device = cDevice(i, directory, serialNumbers[i], refIns)
            self.devices.append(device)

        # Profiling
        self.loadTime = systime.time() - timeLoadStart
        print("Total load time: %.2fs" % (self.loadTime))


def gpsTimeToUTC(gpsWeek, gpsSOW, leapSecs=14):
    global showUtcTime
    if showUtcTime == 0:
        return gpsSOW

    # Search for a valid GPS week
    size = np.shape(gpsWeek)
    if size and size[0] > 1:
        #         if gpsWeek[0] == 0:
        #             gpsWeek = gpsWeek[-1]
        # Use the largest value for the week
        gpsWeek = np.max(gpsWeek)

    if gpsWeek == 0:
        return gpsSOW

    secsInWeek = 604800
    #     secsInDay = 86400
    gpsEpoch = (1980, 1, 6, 0, 0, 0)  # (year, month, day, hh, mm, ss)
    #     secFract = gpsSOW % 1

    epochTuple = gpsEpoch + (-1, -1, 0)
    t0 = systime.mktime(epochTuple) - systime.timezone  # mktime is localtime, correct for UTC
    tdiff = (gpsWeek * secsInWeek) + gpsSOW - leapSecs
    t = t0 + tdiff
    return t


def join_struct_arrays(arrays):
    sizes = np.array([a.itemsize for a in arrays])
    offsets = np.r_[0, sizes.cumsum()]
    n = len(arrays[0])
    joint = np.empty((n, offsets[-1]), dtype=np.uint8)
    for a, size, offset in zip(arrays, sizes, offsets):
        joint[:, offset:offset + size] = a.view(np.uint8).reshape(n, size)
    dtype = sum((a.dtype.descr for a in arrays), [])
    return joint.ravel().view(dtype)


# Join list of structured numpy arrays into one
def join_struct_arrays2(arrays):
    newdtype = sum((a.dtype.descr for a in arrays), [])
    newrecarray = np.empty(len(arrays[0]), dtype=newdtype)
    for a in arrays:
        for name in a.dtype.names:
            newrecarray[name] = a[name]
    return newrecarray


class cSIMPLE:
    def __init__(self, _v):
        self.v = _v


class cIMU:
    def __init__(self, _v):
        global gpsWeek
        self.v = _v
        self.__flt = cObj()
        self.__flt.pqr = None
        self.__flt.acc = None
        self.__flt.pqrNoBias = None
        self.__flt.accNoBias = None
        self.__flt.barNoBias = None
        self.cornerFreqHz = 60
        #         self.cornerFreqHz = 30
        #         self.cornerFreqHz = 15

        self.time = gpsTimeToUTC(gpsWeek, self.v['time'])

        self.i = [cObj(), cObj()]
        for j in range(0, 2):
            self.i[j].pqr = None
            self.i[j].acc = None

        # Dual IMU
        if 'I' in self.v.dtype.names:
            self.i[0].pqr = self.v['I']['pqr'][:, 0, :]
            self.i[1].pqr = self.v['I']['pqr'][:, 1, :]
            self.i[0].acc = self.v['I']['acc'][:, 0, :]
            self.i[1].acc = self.v['I']['acc'][:, 1, :]

        # Preintegrated IMU
        if 'theta1' in self.v.dtype.names and 'theta2' in self.v.dtype.names:
            divDt = 1.0 / self.v['dt']
            self.i[0].pqr = self.v['theta1']
            self.i[1].pqr = self.v['theta2']
            self.i[0].acc = self.v['vel1']
            self.i[1].acc = self.v['vel2']

            for i in range(0, 2):
                for a in range(0, 3):
                    self.i[i].pqr[:, a] *= divDt
                    self.i[i].acc[:, a] *= divDt

    def fltAcc(self):
        if self.__flt.acc is None:
            self.__flt.acc = ft.lpfNoDelay(self.v['acc'], self.cornerFreqHz, time=self.v['time'])
        return self.__flt.acc

    def fltPqr(self):
        if self.__flt.pqr is None:
            self.__flt.pqr = ft.lpfNoDelay(self.v['pqr'], self.cornerFreqHz, time=self.v['time'])
        return self.__flt.pqr

    def fltPqrNoBias(self):
        if 'pqrNoBias' in self.v.dtype.names and self.__flt.pqrNoBias is None:
            self.__flt.pqrNoBias = ft.lpfNoDelay(self.v['pqrNoBias'], self.cornerFreqHz, time=self.v['time'])
        return self.__flt.pqrNoBias

    def fltAccNoBias(self):
        if 'accNoBias' in self.v.dtype.names and self.__flt.accNoBias is None:
            self.__flt.accNoBias = ft.lpfNoDelay(self.v['accNoBias'], self.cornerFreqHz, time=self.v['time'])
        return self.__flt.accNoBias

    def fltBarNoBias(self):
        if 'mslBarNoBias' in self.v.dtype.names and self.__flt.barNoBias is None:
            self.__flt.mslBarNoBias = ft.lpfNoDelay(self.v['mslBarNoBias'], self.cornerFreqHz, time=self.v['time'])
        return self.__flt.mslBarNoBias


#         self.mslBar = ft.smooth(self.v['mslBar']+72, delta=200)
#         self.mslBarDot = ft.derivative(self.v['time'], self.mslBar, delta=10)
#         self.mslBarDotLpf = ft.lpfNoDelay(self.mslBarDot, cornerFreqHz=0.5, time = self.v['time'])

class cINS:
    def __init__(self, _v):
        # self.v = _v
        self.v = _v[:-1]    # Throw out last element
        self.__velNED = None
        self.__course = None
        self.__ecef = None
        self.__ned = None
        self.__istatus = None
        self.__hstatus = None
        self.__size = np.shape(self.v['tow'])[0]
        self.time = gpsTimeToUTC(self.v['week'], self.v['tow'])

        if not 'euler' in self.v.dtype.names and 'q' in self.v.dtype.names:
            #             self.v['euler'] = pose.quat2eulerArray(self.v['q'])
            #             self.euler = pose.quat2eulerArray(self.v['q'])
            dtypeeuler = np.dtype([('euler', (np.float, 3))])
            e = pose.quat2eulerArray(self.v['q'])
            euler = np.ndarray(np.shape(e)[0], dtype=dtypeeuler, buffer=e)
            self.v = join_struct_arrays2([self.v, euler])

        if not 'q' in self.v.dtype.names and 'euler' in self.v.dtype.names:
            #             self.v['q'] = pose.euler2quatArray(self.v['euler'])
            #             self.q = pose.euler2quatArray(self.v['euler'])
            dtypeq = np.dtype([('q', (np.float, 4))])
            q = pose.euler2quatArray(self.v['euler'])
            quat = np.ndarray(np.shape(q)[0], dtype=dtypeq, buffer=q)
            self.v = join_struct_arrays2([self.v, quat])

    # Velocity vector in inertial frame
    def velNed(self):
        if self.__velNED is None:
            self.__velNED = np.zeros(np.shape(self.v['uvw']))

            for i in range(0, self.__size):
                DCM = pose.eulerDCM(self.v['euler'][i, :])
                velNED = np.dot(DCM.T, self.v['uvw'][i, :])  # body to inertial frame
                self.__velNED[i, :] = velNED

        return self.__velNED

    def course(self):
        if self.__course is None:
            self.__course = np.arctan2(self.velNED[:, 1], self.velNED[:, 0])
        return self.__course

    def ned(self):
        global refLla
        if self.__ned is None:
            self.__ned = pose.lla2ned(refLla, self.v['lla'])
        return self.__ned

    def ecef(self):
        if self.__ecef is None:
            self.__ecef = pose.lla2ecef(self.v['lla'])
        return self.__ecef

    def set(self, time):
        self.time = time

    def speed2D(self):
        return np.sqrt(np.square(self.v['uvw'][:, 0]) +
                       np.square(self.v['uvw'][:, 1]))

    def speed3D(self):
        return np.sqrt(np.square(self.v['uvw'][:, 0]) +
                       np.square(self.v['uvw'][:, 1]) +
                       np.square(self.v['uvw'][:, 2]))

    def iStatus(self):
        if self.__istatus is None:
            self.__istatus = insStatus(self.v['iStatus'])
        return self.__istatus

    def hStatus(self):
        if self.__hstatus is None:
            self.__hstatus = hdwStatus(self.v['hStatus'])
        return self.__hstatus


class cRIMU:
    def __init__(self, _v,
                 accBias=np.r_[0, 0, 0],
                 pqrBias=np.r_[0, 0, 0],
                 rotate=np.r_[0, 0, 0]):
        self.v = _v
        self.cornerFreqHz = 30
        self.__flt = cObj()
        self.__flt.pqr = None
        self.__flt.acc = None

        if accBias[0] != 0 or accBias[1] != 0 or accBias[2] != 0:
            self.v['acc'] += accBias

        if pqrBias[0] != 0 or pqrBias[1] != 0 or pqrBias[2] != 0:
            self.v['pqr'] += pqrBias

        if rotate[0] != 0 or rotate[1] != 0 or rotate[2] != 0:
            self.v['acc'] = pose.vectorRotateInertialToBody2(self.v['acc'], rotate)
            self.v['pqr'] = pose.vectorRotateInertialToBody2(self.v['pqr'], rotate)

    def fltPqr(self):
        if self.__flt.pqr is None:
            self.__flt.pqr = ft.lpfNoDelay(self.v['pqr'], self.cornerFreqHz, time=self.v['time'])
        return self.__flt.pqr

    def fltAcc(self):
        if self.__flt.acc is None:
            self.__flt.acc = ft.lpfNoDelay(self.v['acc'], self.cornerFreqHz, time=self.v['time'])
        return self.__flt.acc


class cRINS:
    def __init__(self, _v, rotate=np.r_[0, 0, 0]):
        global refLla
        self.v = _v

        self.__ned = None
        self.__nedDotDot = None
        self.__uvw = None
        self.__rotate = rotate

        #         self.v['nedDot'] = ft.smooth(self.v['nedDot'], delta=10)
        #         self.v['euler'] = ft.smooth(self.v['euler'], delta=10)
        if self.__rotate[0] != 0 or self.__rotate[1] != 0 or self.__rotate[2] != 0:
            self.v['euler'][:, 0] += self.__rotate[0]
            self.v['euler'][:, 1] += self.__rotate[1]
            self.v['euler'][:, 2] += self.__rotate[2]

    def ned(self):
        if self.__ned is None:
            self.__ned = pose.lla2ned(refLla, self.v['lla'])
        return self.__ned

    def nedDotDot(self):
        if self.__nedDotDot is None:
            self.__nedDotDot = ft.derivative(self.v['time'], self.v['nedDot'], delta=2)
            self.__nedDotDot[:, 2] -= 9.80665
            cornerFreqHz = 10
            self.__nedDotDot = ft.lpfNoDelay(self.__nedDotDot, cornerFreqHz, time=self.v['time'])
        return self.__nedDotDot

    def uvw(self):
        if self.__uvw is None:
            self.__uvw = pose.vectorRotateInertialToBody(self.v['nedDot'], self.v['euler'])
            if self.__rotate[0] != 0 or self.__rotate[1] != 0 or self.__rotate[2] != 0:
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
        if self.__ned is None:
            self.__ned = pose.lla2ned(refLla, self.v['lla'])
        return self.__ned

    def accNed(self):
        if self.__acc.ned is None:
            # Create Accelerations from GPS velocities
            #             self.__acc.ned = ft.meanDerivative(self.v['time'], self.v['vel.ned'], 5, 3)
            self.__acc.ned = ft.meanDerivative(self.v['time'], self.v['vel.ned'], 2, 2)
        return self.__acc.ned


class cGPS:
    def __init__(self, _v):
        global refLla
        global refLla
        global gpsWeek
        self.v = _v

        self.time = gpsTimeToUTC(self.v['week'], (_v['timeOfWeekMs'] * 0.001))
        self.ned = pose.lla2ned(refLla, _v['lla'])

        self.satsUsed = (_v['status'] >> 0) & 0xFF
        self.fixType = (_v['status'] >> 8) & 0xFF
        self.rtkMode = (_v['status'] >> 20) & 0x01

        # self.vectorToBase = _v['vectorToBase']
        # self.distanceToBase = _v['distanceToBase']


class cGPSRaw:
    def __init__(self, _v):
        self.count = _v['count']
        self.type = _v['type']
        self.receiverIndex = _v['receiverIndex']
        self.corruptCount = int(_v['corrupt_data'])

        if 'obs' in _v.keys():
            self.obs = _v['obs']
            try:
                self.obstime = np.array([np.datetime64(int(np.round((t['time'] + t['sec'])*1000000)), 'us') for t in _v['obs']['time']])
            except OverflowError as e:
                debug = 1


class cRTKMisc:
    def __init__(self, _v):
        self.v = _v
        self.time = gpsTimeToUTC(_v['week'], (_v['timeOfWeekMs'] * 0.001))

        self.slipCounter = _v['cycleSlipCount']
        self.arThreshold = _v['arThreshold']
        self.baseLla = _v['baseLla']
        self.heading = _v['rtkCompassHeading']


class cGpsVel:
    def __init__(self, _v):
        global gpsWeek
        self.v = _v

    def acc(self):
        if self.__acc is None:
            self.__acc = cObj()
            self.__acc.time = self.time
            #         self.__acc.ned = ft.meanDerivative(self.vel.time, self.v['ned'], 5, 3)
            self.__acc.ned = ft.meanDerivative(self.time, self.v['ned'], 2, 2)
        return self.__acc


class cGpsAcc:
    def __init__(self, _v):
        self.v = _v
        #         self.time = _v['timeMs'] * 0.001
        self.time = gpsTimeToUTC(self.v['week'], (_v['timeOfWeekMs'] * 0.001))


class cBias:
    def __init__(self, _v):
        global gpsWeek
        self.v = _v
        #         self.time = _v['timeMs'] * 0.001
        self.time = gpsTimeToUTC(gpsWeek, (_v['towMs'] * 0.001))


class cInsRes:
    def __init__(self, _v):
        global gpsWeek
        self.v = _v
        self.time = gpsTimeToUTC(gpsWeek, (_v['towMs'] * 0.001))


class cDevInfo:
    def __init__(self, _v):
        self.v = _v


class cSysParams:
    def __init__(self, _v):
        global gpsWeek
        self.v = _v
        self.__istatus = None
        self.__hstatus = None

        if 'tow' in _v.dtype.names:
            #             self.time               = _v['time']
            self.time = gpsTimeToUTC(gpsWeek, _v['tow'])
        if 'towMs' in _v.dtype.names:
            #             self.time               = (_v['timeMs']) * 0.001
            self.time = gpsTimeToUTC(gpsWeek, (_v['towMs'] * 0.001))

    def iStatus(self):
        if self.__istatus is None:
            self.__istatus = insStatus(self.v['iStatus'])
        return self.__istatus

    def hStatus(self):
        if self.__istatus is None:
            self.__hstatus = hdwStatus(self.v['hStatus'])
        return self.__hstatus


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
        self.time = gpsTimeToUTC(gpsWeek, (_v['towMs'] * 0.001))
        self.accNed.time = gpsTimeToUTC(gpsWeek, (_v['accNed']['towMs'] * 0.001))
        self.velNed.time = gpsTimeToUTC(gpsWeek, (_v['velNed']['towMs'] * 0.001))
        self.lla.time = gpsTimeToUTC(gpsWeek, (_v['lla']['towMs'] * 0.001))

        self.accNed.refHdg = np.arctan2(self.v['accNed']['ref'][:, 1], self.v['accNed']['ref'][:, 0])
        self.accNed.insHdg = np.arctan2(self.v['accNed']['ins'][:, 1], self.v['accNed']['ins'][:, 0])

        self.lla.refNed = pose.lla2ned(refLla, _v['lla']['ref'])
        self.lla.insNed = pose.lla2ned(refLla, _v['lla']['ins'])


#         self.v['mslBar'] += 86;

class cInsParams:
    def __init__(self, _v):
        global gpsWeek
        self.v = _v

        #         self.time           = _v['timeMs'] * 0.001
        self.time = gpsTimeToUTC(gpsWeek, (_v['towMs'] * 0.001))
        if 'magTowMs' in _v.dtype.names:
            #             self.magTime          = (_v['magTowMs']) * 0.001
            self.magTime = gpsTimeToUTC(gpsWeek, (_v['magTowMs'] * 0.001))


def lla2kml(time, lla, serialNumber, kmlFileName="log.kml", **kwargs):
    kml = simplekml.Kml()

    color = kwargs.pop('color', simplekml.Color.yellow)
    altitudeMode = kwargs.pop('altitudeMode', simplekml.constants.AltitudeMode.absolute)
    timeStep = kwargs.pop('timeStep', 0)

    latLon = []
    tNext = 0
    lNext = 0
    for i in range(0, np.shape(lla)[0]):
        latLon.append((lla[i, 1], lla[i, 0], lla[i, 2]))

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
    ls = kml.newlinestring(name="Tracks", description=serialNumber + " tracks", coords=latLon)

    # Style
    ls.extrude = 1
    ls.altitudemode = altitudeMode
    ls.style.linestyle.width = 2
    ls.style.linestyle.color = color
    kml.save(kmlFileName)

    return kmlFileName


#####   INS Status   #####
def insStatus(istatus):
    result = cObj()

    result.align = cObj()
    result.align.coarse = cObj()
    result.align.good = cObj()
    result.align.fine = cObj()

    # 0-3
    result.align.coarse.att = (istatus >> 0) & 1
    result.align.coarse.vel = (istatus >> 1) & 1
    result.align.coarse.pos = (istatus >> 2) & 1

    # 4-7
    result.align.good.att = (istatus >> 4) & 1
    result.align.good.vel = (istatus >> 5) & 1
    result.align.good.pos = (istatus >> 6) & 1
    result.align.fine.att = (istatus >> 7) & 1

    # 8-11
    result.usingGps = (istatus >> 8) & 1
    result.usingMag = (istatus >> 11) & 1

    # 12-15
    result.navMode = (istatus >> 12) & 1

    # 16-23
    result.solutionStatus = (istatus >> 16) & 0x7

    # 20-23
    result.magActiveCalSet = (istatus >> 20) & 1
    result.magRecalibrating = (istatus >> 22) & 1
    result.magInterOrBadCal = ((istatus >> 23) & 1) != 1

    # 24-27

    # 28-31
    result.rtosTaskPeriodOverrun = (istatus >> 29) & 1
    result.generalFault = (istatus >> 31) & 1

    return result


#####   Hardware Status   #####
def hdwStatus(hstatus):
    result = cObj()

    # 0-3
    result.motionGyrSig = (hstatus >> 0) & 0x1
    result.motionAccSig = (hstatus >> 1) & 0x1
    result.motionGyrDev = (hstatus >> 2) & 0x1
    result.motionAccDev = (hstatus >> 3) & 0x1

    # 4-7
    result.satellite_rx = (hstatus >> 4) & 0x1

    # 8-11
    result.saturationGyr = (hstatus >> 8) & 0x1
    result.saturationAcc = (hstatus >> 9) & 0x1
    result.saturationMag = (hstatus >> 10) & 0x1
    result.saturationBaro = (hstatus >> 11) & 0x1

    # 12-15
    result.saturationHistory = (hstatus >> 12) & 0x1

    # 16-19
    result.errComTxLimited = (hstatus >> 16) & 0x1
    result.errComRxOverrun = (hstatus >> 17) & 0x1
    result.errGpsTxLimited = (hstatus >> 18) & 0x1
    result.errGpsRxOverrun = (hstatus >> 19) & 0x1

    # 20-23
    result.comParseErrCount = (hstatus >> 20) & 0xF

    # 24-27
    result.selfTestFault = (hstatus >> 24) & 0x1
    result.errTemperature = (hstatus >> 25) & 0x1

    # 28-31
    result.faultWatchdogReset = (hstatus >> 28) & 0x1
    result.faultBODReset = (hstatus >> 29) & 0x1
    result.faultPORReset = (hstatus >> 30) & 0x1
    result.faultCPUErrReset = (hstatus >> 31) & 0x1

    return result

