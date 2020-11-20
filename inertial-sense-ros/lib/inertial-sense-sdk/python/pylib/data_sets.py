from enum import Enum, IntEnum

# Everything in this file comes from data_sets.h.
# This file needs to be kept in sync with data_sets.h

DID_NULL                        = 0
DID_DEV_INFO                    = 1
DID_SYS_FAULT                  = 2
DID_PREINTEGRATED_IMU           = 3
DID_INS_1                       = 4
DID_INS_2                       = 5
DID_GPS1_UBX_POS                = 6
DID_CONFIG                      = 7
DID_ASCII_BCAST_PERIOD          = 8
DID_RMC                         = 9
DID_SYS_PARAMS                  = 10
DID_SYS_SENSORS                 = 11
DID_FLASH_CONFIG                = 12
DID_GPS1_POS                    = 13
DID_GPS2_POS                    = 14
DID_GPS1_SAT                    = 15
DID_GPS2_SAT                    = 16
DID_GPS1_VERSION                = 17
DID_GPS2_VERSION                = 18
DID_MAG_CAL                     = 19
DID_INTERNAL_DIAGNOSTIC         = 20
DID_GPS1_RTK_POS_REL            = 21
DID_GPS1_RTK_POS_MISC           = 22
DID_FEATURE_BITS                = 23
DID_SENSORS_IS1                 = 24
DID_SENSORS_IS2                 = 25
DID_SENSORS_TC_BIAS             = 26
DID_IO                          = 27
DID_SENSORS_ADC                 = 28
DID_SCOMP                       = 29
DID_GPS1_VEL                    = 30
DID_GPS2_VEL                    = 31
DID_HDW_PARAMS                  = 32
DID_NVR_MANAGE_USERPAGE         = 33
DID_NVR_USERPAGE_SN             = 34
DID_NVR_USERPAGE_G0             = 35
DID_NVR_USERPAGE_G1             = 36
DID_DEBUG_STRING                = 37
DID_RTOS_INFO                   = 38
DID_DEBUG_ARRAY                 = 39
DID_SENSORS_CAL1                = 40
DID_SENSORS_CAL2                = 41
DID_CAL_SC                      = 42
DID_CAL_SC1                     = 43
DID_CAL_SC2                     = 44
DID_SYS_SENSORS_SIGMA           = 45
DID_SENSORS_ADC_SIGMA           = 46
DID_INS_DEV_1                   = 47
DID_INL2_STATES                 = 48
DID_INL2_COVARIANCE_LD          = 49
DID_INL2_STATUS                 = 50
DID_INL2_MISC                   = 51
DID_MAGNETOMETER_1              = 52
DID_BAROMETER                   = 53
DID_GPS1_RTK_POS                = 54
DID_UNUSED_55                   = 55
DID_COMMUNICATIONS_LOOPBACK     = 56
DID_DUAL_IMU_RAW                = 57
DID_DUAL_IMU                    = 58
DID_INL2_MAG_OBS_INFO           = 59
DID_GPS_BASE_RAW                = 60
DID_GPS_RTK_OPT                 = 61
DID_NVR_USERPAGE_INTERNAL       = 62
DID_MANUFACTURING_INFO          = 63
DID_BIT                         = 64
DID_INS_3                       = 65
DID_INS_4                       = 66
DID_INL2_VARIANCE               = 67
DID_STROBE_IN_TIME              = 68
DID_GPS1_RAW                    = 69
DID_GPS2_RAW                    = 70
DID_WHEEL_ENCODER               = 71
DID_DIAGNOSTIC_MESSAGE          = 72
DID_SURVEY_IN                   = 73
DID_EVB2                        = 74
DID_PORT_MONITOR                = 75
DID_RTK_STATE                   = 76
DID_RTK_PHASE_RESIDUAL          = 77
DID_RTK_CODE_RESIDUAL           = 78
DID_RTK_DEBUG                   = 79
DID_EVB_STATUS                  = 80
DID_EVB_CONFIG                  = 81
DID_EVB_DEBUG_ARRAY             = 82
DID_EVB_RTOS_INFO               = 83
DID_DUAL_IMU_RAW_MAG            = 84
DID_DUAL_IMU_MAG                = 85
DID_PREINTEGRATED_IMU_MAG       = 86
DID_WHEEL_CONFIG                = 87
DID_POSITION_MEASUREMENT        = 88
DID_RTK_DEBUG_2                 = 89
DID_CAN_CONFIG                  = 90
DID_GPS1_RTK_CMP_REL            = 91
DID_GPS1_RTK_CMP_MISC           = 92

NUM_DIDS                        = 96


did_name_lookup = {
 DID_NULL : "null",
 DID_DEV_INFO : "devInfo",
 DID_SYS_FAULT : "sysFault",
 DID_PREINTEGRATED_IMU : "preintegratedImu",
 DID_INS_1 : "ins1",
 DID_INS_2 : "ins2",
 DID_GPS1_UBX_POS : "gps1UbxPos",
 DID_CONFIG : "config",
 DID_ASCII_BCAST_PERIOD : "asciiBcastPeriod",
 DID_RMC : "rmc",
 DID_SYS_PARAMS : "sysParams",
 DID_SYS_SENSORS : "sysSensors",
 DID_FLASH_CONFIG : "flashConfig",
 DID_GPS1_POS : "gps1Pos",
 DID_GPS2_POS : "gps2Pos",
 DID_GPS1_SAT : "gps1Sat",
 DID_GPS2_SAT : "gps2Sat",
 DID_GPS1_VERSION : "gps1Version",
 DID_GPS2_VERSION : "gps2Version",
 DID_MAG_CAL : "magCal",
 DID_INTERNAL_DIAGNOSTIC : "internalDiagnostic",
 DID_GPS1_RTK_POS_REL : "gps1RtkPosRel",
 DID_GPS1_RTK_CMP_REL : "gps1RtkCmpRel",
 DID_GPS1_RTK_POS_MISC: "gps1RtkPosMisc",
 DID_GPS1_RTK_CMP_MISC: "gps1RtkCmpMisc",
 DID_FEATURE_BITS : "featureBits",
 DID_SENSORS_IS1 : "sensorsIs1",
 DID_SENSORS_IS2 : "sensorsIs2",
 DID_SENSORS_TC_BIAS : "sensorsTcBias",
 DID_IO : "io",
 DID_SENSORS_ADC : "sensorsAdc",
 DID_SCOMP : "scomp",
 DID_GPS1_VEL : "gps1Vel",
 DID_GPS2_VEL : "gps2Vel",
 DID_HDW_PARAMS : "hdwParams",
 DID_NVR_MANAGE_USERPAGE : "nvrManageUserpage",
 DID_NVR_USERPAGE_SN : "nvrUserpageSn",
 DID_NVR_USERPAGE_G0 : "nvrUserpageG0",
 DID_NVR_USERPAGE_G1 : "nvrUserpageG1",
 DID_DEBUG_STRING : "debugString",
 DID_RTOS_INFO : "rtosInfo",
 DID_DEBUG_ARRAY : "debugArray",
 DID_SENSORS_CAL1 : "sensorsCal1",
 DID_SENSORS_CAL2 : "sensorsCal2",
 DID_CAL_SC : "calSc",
 DID_CAL_SC1 : "calSc1",
 DID_CAL_SC2 : "calSc2",
 DID_SYS_SENSORS_SIGMA : "sysSensorsSigma",
 DID_SENSORS_ADC_SIGMA : "sensorsAdcSigma",
 DID_INS_DEV_1 : "insDev1",
 DID_INL2_STATES : "inl2States",
 DID_INL2_COVARIANCE_LD : "inl2CovarianceLd",
 DID_INL2_STATUS : "inl2Status",
 DID_INL2_MISC : "inl2Misc",
 DID_MAGNETOMETER_1 : "magnetometer1",
 DID_BAROMETER : "barometer",
 DID_GPS1_RTK_POS : "gps1RtkPos",
 DID_COMMUNICATIONS_LOOPBACK : "communicationsLoopback",
 DID_DUAL_IMU_RAW : "dualImuRaw",
 DID_DUAL_IMU : "dualImu",
 DID_INL2_MAG_OBS_INFO : "inl2MagObsInfo",
 DID_GPS_BASE_RAW : "gpsBaseRaw",
 DID_GPS_RTK_OPT : "gpsRtkOpt",
 DID_NVR_USERPAGE_INTERNAL : "nvrUserpageInternal",
 DID_MANUFACTURING_INFO : "manufacturingInfo",
 DID_BIT : "bit",
 DID_INS_3 : "ins3",
 DID_INS_4 : "ins4",
 DID_INL2_VARIANCE : "inl2Variance",
 DID_STROBE_IN_TIME : "strobeInTime",
 DID_GPS1_RAW : "gps1Raw",
 DID_GPS2_RAW : "gps2Raw",
 DID_WHEEL_ENCODER : "wheelEncoder",
 DID_DIAGNOSTIC_MESSAGE : "diagnosticMessage",
 DID_SURVEY_IN : "surveyIn",
 DID_EVB2 : "evb2",
 DID_PORT_MONITOR : "portMonitor",
 DID_RTK_STATE : "rtkState",
 DID_RTK_PHASE_RESIDUAL : "rtkPhaseResidual",
 DID_RTK_CODE_RESIDUAL : "rtkCodeResidual",
 DID_RTK_DEBUG : "rtkDebug",
 DID_EVB_STATUS : "evbStatus",
 DID_EVB_CONFIG : "evbConfig",
 DID_EVB_DEBUG_ARRAY : "evbDebugArray",
 DID_EVB_RTOS_INFO : "evbRtosInfo",
 DID_RTK_DEBUG_2: "rtkDebug2",
 DID_CAN_CONFIG: "canconfig",
}

class eGpsNavFixStatus(Enum):
    """ Navigation Fix Type """
    NAV_FIX_STATUS_NONE                = 0x00000000
    NAV_FIX_STATUS_3D                  = 0x00000001
    NAV_FIX_STATUS_RTK_FLOAT           = 0x00000002
    NAV_FIX_STATUS_RTK_FIX             = 0x00000003


# This function is based on the INS_STATUS_NAV_FIX_STATUS macro in data_sets.h
    @staticmethod
    def from_ins_status(insStatus):
        INS_STATUS_NAV_FIX_STATUS_MASK   = 0x07000000
        INS_STATUS_NAV_FIX_STATUS_OFFSET = 24
        return eNavFixStatus((insStatus&INS_STATUS_NAV_FIX_STATUS_MASK)>>INS_STATUS_NAV_FIX_STATUS_OFFSET)

    def __str__(self):
        return self.name


class eRtosTask(Enum):
    """
     RTOS task types
    """
    TASK_SAMPLE         = 0 # Sample task
    TASK_NAV            = 1 # Nav task
    TASK_COMMUNICATIONS = 2 # Communications task
    TASK_MAINTENANCE    = 3 # Maintenance task
    TASK_IDLE           = 4 # Idle task
    TASK_TIMER          = 5 # Timer task
    RTOS_NUM_TASKS      = 6 # Number of RTOS tasks - KEEP LAST

class eRawDataType(Enum):
    raw_data_type_observation = 1
    raw_data_type_ephemeris = 2
    raw_data_type_glonass_ephemeris = 3
    raw_data_type_sbas = 4
    raw_data_type_base_station_antenna_position = 5
    raw_data_type_ionosphere_model_utc_alm = 6


# This is part of eInsStatusFlags from data_sets.h
class eInsStatusRtkBase(Enum):
    """RTK base portion of INS status"""
    INS_STATUS_RTK_BASE_ERR_NO_OBSERV   = 0x08000000 # GPS base NO observations received (i.e. RTK differential corrections)
    INS_STATUS_RTK_BASE_ERR_NO_POSITION	= 0x10000000 # GPS base NO position received
    INS_STATUS_RTK_BASE_POSITION_MOVING	= 0x20000000 # GPS base position is moving

    @staticmethod
    def from_ins_status(insStatus):
        INS_STATUS_RTK_BASE_MASK = 0x38300000 # GPS base mask
        status = insStatus & INS_STATUS_RTK_BASE_MASK
        if status:
            return eInsStatusRtkBase
        return None


class SysFaultStatus(Enum):
    # Normal Resets
    HARDWARE_RESET                                   = 0x00000000
    USER_RESET                                       = 0x00000001
    ENABLE_BOOTLOADER                                = 0x00000002

    # General Errors
    SOFT_RESET                      = 0x00000010
    FLASH_MIGRATION_EVENT           = 0x00000020
    FLASH_MIGRATION_COMPLETED       = 0x00000040

    # Critical Errors
    HARD_FAULT                      = 0x00010000
    USAGE_FAULT                     = 0x00020000
    MEM_MANGE                       = 0x00040000
    BUS_FAULT                       = 0x00080000
    MALLOC_FAILED                   = 0x00100000
    STACK_OVERFLOW                  = 0x00200000
    INVALID_CODE_OPERATION          = 0x00400000
    FLASH_MIGRATION_MARKER_UPDATED  = 0x00800000

    def is_general_error(self):
        MASK_GENERAL_ERROR = 0xFFFFFFF0
        return self.value & MASK_GENERAL_ERROR

    def is_critical_error(self):
        MASK_CRITICAL_ERROR = 0xFFFF0000
        return self.value & MASK_CRITICAL_ERROR



class eConfigSystem(IntEnum):
    CFG_SYS_CMD_SAVE_PERSISTENT_MESSAGES            = 1
    CFG_SYS_CMD_ENABLE_BOOTLOADER_AND_RESET         = 2
    CFG_SYS_CMD_ENABLE_SENSOR_STATS                 = 3
    CFG_SYS_CMD_ENABLE_RTOS_STATS                   = 4
    CFG_SYS_CMD_ENABLE_GPS_LOW_LEVEL_CONFIG         = 10
    CFG_SYS_CMD_CLEAR_GPS_ASSIST_FROM_FLASH         = 97
    CFG_SYS_CMD_SAVE_GPS_ASSIST_TO_FLASH_RESET      = 98
    CFG_SYS_CMD_SOFTWARE_RESET                      = 99
