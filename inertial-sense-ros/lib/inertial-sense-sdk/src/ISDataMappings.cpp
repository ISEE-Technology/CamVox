/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <map>
#include <vector>
#include <set>
#include <algorithm>
#include <stdlib.h>
#include <stddef.h>
#include <inttypes.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>

#include "ISDataMappings.h"
#include "DataJSON.h"
#include "ISUtilities.h"
#include "ISConstants.h"
#include "data_sets.h"

#ifdef USE_IS_INTERNAL
#include "../../libs/IS_internal.h"
#endif

#if PLATFORM_IS_EMBEDDED
cISDataMappings* cISDataMappings::s_map;
#else
cISDataMappings cISDataMappings::s_map;
#endif

const unsigned char g_asciiToLowerMap[256] =
{
	0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,
	41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,'a','b','c','d','e','f','g','h','i','j','k','l','m','n','o','p','q',
	'r','s','t','u','v','w','x','y','z',91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,
	119,120,121,122,123,124,125,126,127,128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,143,144,145,146,147,148,149,150,151,152,153,
	154,155,156,157,158,159,160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,179,180,181,182,183,184,185,186,187,188,
	189,190,191,192,193,194,195,196,197,198,199,200,201,202,203,204,205,206,207,208,209,210,211,212,213,214,215,216,217,218,219,220,221,222,223,
	224,225,226,227,228,229,230,231,232,233,234,235,236,237,238,239,240,241,242,243,244,245,246,247,248,249,250,251,252,253,254,255
};

CONST_EXPRESSION uint32_t s_eDataTypeSizes[DataTypeCount] =
{
	(uint32_t)sizeof(int8_t),
	(uint32_t)sizeof(uint8_t),
	(uint32_t)sizeof(int16_t),
	(uint32_t)sizeof(uint16_t),
	(uint32_t)sizeof(int32_t),
	(uint32_t)sizeof(uint32_t),
	(uint32_t)sizeof(int64_t),
	(uint32_t)sizeof(uint64_t),
	(uint32_t)sizeof(float),
	(uint32_t)sizeof(double),
    (uint32_t)0, // string, must be set to actual size by caller
    (uint32_t)0  // binary, must be set to actual size by caller
};

inline uint32_t GetDataTypeSize(eDataType dataType)
{
	if (dataType >= 0 && dataType < DataTypeCount)
	{
		return s_eDataTypeSizes[dataType];
	}
	return 0;
}

#if CPP11_IS_ENABLED

// dataSize can be 0 for default size, must be set for string type
#define ADD_MAP_NO_VALIDATION(map, totalSize, name, member, dataSize, dataType, memberType) (map)[std::string(name)] = { (uint32_t)offsetof(MAP_TYPE, member), (uint32_t)(dataSize == 0 ? sizeof(memberType) : dataSize), dataType, name }; totalSize += sizeof(memberType);

// note when passing member type for arrays, it must be a reference, i.e. float&
#define ADD_MAP(map, totalSize, name, member, dataSize, dataType, memberType) \
    ADD_MAP_NO_VALIDATION(map, totalSize, name, member, dataSize, dataType, memberType); \
    static_assert(is_same<decltype(MAP_TYPE::member), memberType>::value, "Member type is an unexpected type"); \
	static_assert((uint32_t)(dataSize == 0 ? sizeof(memberType) : dataSize) == sizeof(memberType), "Member type is an unexpected size"); \
    static_assert((uint32_t)(dataSize == 0 ? sizeof(memberType) : dataSize) == sizeof(MAP_TYPE::member), "Member type is an unexpected size"); \
	static_assert(s_eDataTypeSizes[dataType] == 0 || (uint32_t)(dataSize == 0 ? sizeof(memberType) : dataSize) == s_eDataTypeSizes[dataType], "Data type size does not match member size");
#define ASSERT_SIZE(s) assert(s == sizeof(MAP_TYPE))

#else

#define ADD_MAP_NO_VALIDATION(map, totalSize, name, member, dataSize, dataType, memberType) (map)[std::string(name)] = { (uint32_t)offsetof(MAP_TYPE, member), (uint32_t)(dataSize == 0 ? sizeof(memberType) : dataSize), dataType, name }; totalSize += sizeof(memberType);
#define ADD_MAP(map, totalSize, name, member, dataSize, dataType, memberType) ADD_MAP_NO_VALIDATION(map, totalSize, name, member, dataSize, dataType, memberType)
#define ASSERT_SIZE(s) // not supported on VS < 2015

#endif

static void PopulateSizeMappings(uint32_t sizeMap[DID_COUNT])
{
    //memset(sizeMap, 0, sizeof(sizeMap));
    // NOTE: If you use the line above, the compiler issues this warning:
    //       "‘sizeof’ on array function parameter ‘sizeMap’ will return size of ‘uint32_t* {aka unsigned int*}’"
    //       So, we use `sizeof(uint32_t) * DID_COUNT` instead to get the correct size.
	memset(sizeMap, 0, sizeof(uint32_t) * DID_COUNT);

	sizeMap[DID_DEV_INFO] = sizeof(dev_info_t);
	sizeMap[DID_MAGNETOMETER_1] = sizeof(magnetometer_t);
	sizeMap[DID_MAGNETOMETER_2] = sizeof(magnetometer_t);
	sizeMap[DID_BAROMETER] = sizeof(barometer_t);
	sizeMap[DID_PREINTEGRATED_IMU] = sizeof(preintegrated_imu_t);
    sizeMap[DID_WHEEL_ENCODER] = sizeof(wheel_encoder_t);
    sizeMap[DID_SYS_CMD] = sizeof(system_command_t);
	sizeMap[DID_INS_1] = sizeof(ins_1_t);
	sizeMap[DID_INS_2] = sizeof(ins_2_t);
	sizeMap[DID_INS_3] = sizeof(ins_3_t);
	sizeMap[DID_INS_4] = sizeof(ins_4_t);
	sizeMap[DID_GPS1_POS] = sizeof(gps_pos_t);
	sizeMap[DID_GPS1_UBX_POS] = sizeof(gps_pos_t);
	sizeMap[DID_GPS1_VEL] = sizeof(gps_vel_t);
	sizeMap[DID_GPS2_POS] = sizeof(gps_pos_t);
	sizeMap[DID_GPS2_VEL] = sizeof(gps_vel_t);
	sizeMap[DID_GPS1_SAT] = sizeof(gps_sat_t);
	sizeMap[DID_GPS2_SAT] = sizeof(gps_sat_t);
	sizeMap[DID_GPS1_VERSION] = sizeof(gps_version_t);
	sizeMap[DID_GPS2_VERSION] = sizeof(gps_version_t);
	sizeMap[DID_GPS1_RTK_POS] = sizeof(gps_pos_t);
	sizeMap[DID_GPS1_RTK_POS_REL] = sizeof(gps_rtk_rel_t);
	sizeMap[DID_GPS1_RTK_POS_MISC] = sizeof(gps_rtk_misc_t);
	sizeMap[DID_GPS2_RTK_CMP_REL] = sizeof(gps_rtk_rel_t);
	sizeMap[DID_GPS2_RTK_CMP_MISC] = sizeof(gps_rtk_misc_t);
	sizeMap[DID_SYS_PARAMS] = sizeof(sys_params_t);
	sizeMap[DID_SYS_SENSORS] = sizeof(sys_sensors_t);
	sizeMap[DID_FLASH_CONFIG] = sizeof(nvm_flash_cfg_t);
	sizeMap[DID_DUAL_IMU] = sizeof(dual_imu_t);
    sizeMap[DID_DUAL_IMU_RAW] = sizeof(dual_imu_t);
	sizeMap[DID_GPS_BASE_RAW] = sizeof(gps_raw_t);
	sizeMap[DID_STROBE_IN_TIME] = sizeof(strobe_in_time_t);
	sizeMap[DID_RTOS_INFO] = sizeof(rtos_info_t);
	sizeMap[DID_DUAL_IMU_RAW_MAG] = sizeof(imu_mag_t);
	sizeMap[DID_CAN_CONFIG] = sizeof(can_config_t);


#ifdef USE_IS_INTERNAL

	sizeMap[DID_SENSORS_IS1] = sizeof(sensors_w_temp_t);
	sizeMap[DID_SENSORS_IS2] = sizeof(sensors_w_temp_t);
	sizeMap[DID_SENSORS_TC_BIAS] = sizeof(sensors_t);
	sizeMap[DID_SCOMP] = sizeof(sensor_compensation_t);
	sizeMap[DID_DEBUG_ARRAY] = sizeof(debug_array_t);
    sizeMap[DID_RTK_DEBUG] = sizeof(rtk_debug_t);
//     sizeMap[DID_RTK_STATE] = sizeof(rtk_state_t);
    sizeMap[DID_RTK_CODE_RESIDUAL] = sizeof(rtk_residual_t);
    sizeMap[DID_RTK_PHASE_RESIDUAL] = sizeof(rtk_residual_t);
	sizeMap[DID_NVR_USERPAGE_G0] = sizeof(nvm_group_0_t);
	sizeMap[DID_NVR_USERPAGE_G1] = sizeof(nvm_group_1_t);
	sizeMap[DID_INL2_STATES] = sizeof(inl2_states_t);
	sizeMap[DID_INL2_STATUS] = sizeof(inl2_status_t);
	sizeMap[DID_INL2_MISC] = sizeof(inl2_misc_t);
	sizeMap[DID_INL2_MAG_OBS_INFO] = sizeof(inl2_mag_obs_info_t);
	sizeMap[DID_DUAL_IMU_MAG] = sizeof(imu_mag_t);
	sizeMap[DID_PREINTEGRATED_IMU_MAG] = sizeof(pimu_mag_t);
	sizeMap[DID_SENSORS_ADC] = sizeof(sys_sensors_adc_t);
	sizeMap[DID_RTK_DEBUG_2] = sizeof(rtk_debug_2_t);

#endif

}

static void PopulateTimestampField(uint32_t id, const data_info_t** timestamps, map_name_to_info_t mappings[DID_COUNT])
{
	static const string timestampFields[] = { "time", "timeOfWeek", "timeOfWeekMs", "seconds" };
	const map_name_to_info_t& offsetMap = mappings[id];

	if (offsetMap.size() != 0)
	{
		for (size_t i = 0; i < _ARRAY_ELEMENT_COUNT(timestampFields); i++)
		{
			map_name_to_info_t::const_iterator timestampField = offsetMap.find(timestampFields[i]);
			if (timestampField != offsetMap.end())
			{
				timestamps[id] = (const data_info_t*)&timestampField->second;
				return;
			}
		}
	}

	timestamps[id] = NULLPTR; // ensure value is not garbage
}

static void PopulateDeviceInfoMappings(map_name_to_info_t mappings[DID_COUNT])
{
	typedef dev_info_t MAP_TYPE;
	map_name_to_info_t& m = mappings[DID_DEV_INFO];
	uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "reserved", reserved, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "serialNumber", serialNumber, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "hardwareVer[0]", hardwareVer[0], 0, DataTypeUInt8, uint8_t&);
    ADD_MAP(m, totalSize, "hardwareVer[1]", hardwareVer[1], 0, DataTypeUInt8, uint8_t&);
    ADD_MAP(m, totalSize, "hardwareVer[2]", hardwareVer[2], 0, DataTypeUInt8, uint8_t&);
    ADD_MAP(m, totalSize, "hardwareVer[3]", hardwareVer[3], 0, DataTypeUInt8, uint8_t&);
    ADD_MAP(m, totalSize, "firmwareVer[0]", firmwareVer[0], 0, DataTypeUInt8, uint8_t&);
    ADD_MAP(m, totalSize, "firmwareVer[1]", firmwareVer[1], 0, DataTypeUInt8, uint8_t&);
    ADD_MAP(m, totalSize, "firmwareVer[2]", firmwareVer[2], 0, DataTypeUInt8, uint8_t&);
    ADD_MAP(m, totalSize, "firmwareVer[3]", firmwareVer[3], 0, DataTypeUInt8, uint8_t&);
    ADD_MAP(m, totalSize, "buildNumber", buildNumber, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "protocolVer[0]", protocolVer[0], 0, DataTypeUInt8, uint8_t&);
    ADD_MAP(m, totalSize, "protocolVer[1]", protocolVer[1], 0, DataTypeUInt8, uint8_t&);
    ADD_MAP(m, totalSize, "protocolVer[2]", protocolVer[2], 0, DataTypeUInt8, uint8_t&);
    ADD_MAP(m, totalSize, "protocolVer[3]", protocolVer[3], 0, DataTypeUInt8, uint8_t&);
    ADD_MAP(m, totalSize, "repoRevision", repoRevision, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "manufacturer", manufacturer, DEVINFO_MANUFACTURER_STRLEN, DataTypeString, char[DEVINFO_MANUFACTURER_STRLEN]);
    ADD_MAP(m, totalSize, "buildDate[0]", buildDate[0], 0, DataTypeUInt8, uint8_t&);
    ADD_MAP(m, totalSize, "buildDate[1]", buildDate[1], 0, DataTypeUInt8, uint8_t&);
    ADD_MAP(m, totalSize, "buildDate[2]", buildDate[2], 0, DataTypeUInt8, uint8_t&);
    ADD_MAP(m, totalSize, "buildDate[3]", buildDate[3], 0, DataTypeUInt8, uint8_t&);
    ADD_MAP(m, totalSize, "buildTime[0]", buildTime[0], 0, DataTypeUInt8, uint8_t&);
    ADD_MAP(m, totalSize, "buildTime[1]", buildTime[1], 0, DataTypeUInt8, uint8_t&);
    ADD_MAP(m, totalSize, "buildTime[2]", buildTime[2], 0, DataTypeUInt8, uint8_t&);
    ADD_MAP(m, totalSize, "buildTime[3]", buildTime[3], 0, DataTypeUInt8, uint8_t&);
    ADD_MAP(m, totalSize, "addInfo", addInfo, DEVINFO_ADDINFO_STRLEN, DataTypeString, char[DEVINFO_ADDINFO_STRLEN]);

    ASSERT_SIZE(totalSize);
}

static void PopulateIMUMappings(map_name_to_info_t mappings[DID_COUNT], uint32_t dataId)
{
	typedef dual_imu_t MAP_TYPE;
	map_name_to_info_t& m = mappings[dataId];
	uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "time", time, 0, DataTypeDouble, double);
    ADD_MAP(m, totalSize, "pqr1[0]", I[0].pqr[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr1[1]", I[0].pqr[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr1[2]", I[0].pqr[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc1[0]", I[0].acc[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc1[1]", I[0].acc[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc1[2]", I[0].acc[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr2[0]", I[1].pqr[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr2[1]", I[1].pqr[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr2[2]", I[1].pqr[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc2[0]", I[1].acc[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc2[1]", I[1].acc[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc2[2]", I[1].acc[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "status", status, 0, DataTypeUInt32, uint32_t);

    ASSERT_SIZE(totalSize);
}

static void PopulateSysParamsMappings(map_name_to_info_t mappings[DID_COUNT])
{
	typedef sys_params_t MAP_TYPE;
	map_name_to_info_t& m = mappings[DID_SYS_PARAMS];
	uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "timeOfWeekMs", timeOfWeekMs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "insStatus", insStatus, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "hdwStatus", hdwStatus, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "imuTemp", imuTemp, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "baroTemp", baroTemp, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "mcuTemp", mcuTemp, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "reserved1", reserved1, 0, DataTypeFloat, float);
	ADD_MAP(m, totalSize, "imuPeriodMs", imuPeriodMs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "navPeriodMs", navPeriodMs, 0, DataTypeUInt32, uint32_t);
	ADD_MAP(m, totalSize, "sensorTruePeriod", sensorTruePeriod, 0, DataTypeDouble, double);
	ADD_MAP(m, totalSize, "reserved2[0]", reserved2[0], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "reserved2[1]", reserved2[1], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "genFaultCode", genFaultCode, 0, DataTypeUInt32, uint32_t);

    ASSERT_SIZE(totalSize);
}

static void PopulateSysSensorsMappings(map_name_to_info_t mappings[DID_COUNT])
{
	typedef sys_sensors_t MAP_TYPE;
	map_name_to_info_t& m = mappings[DID_SYS_SENSORS];
	uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "time", time, 0, DataTypeDouble, double);
    ADD_MAP(m, totalSize, "temp", temp, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "pqr[0]", pqr[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr[1]", pqr[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr[2]", pqr[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc[0]", acc[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc[1]", acc[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc[2]", acc[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag[0]", mag[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag[1]", mag[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag[2]", mag[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "bar", bar, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "barTemp", barTemp, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "mslBar", mslBar, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "humidity", humidity, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "vin", vin, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "ana1", ana1, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "ana3", ana1, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "ana4", ana1, 0, DataTypeFloat, float);

    ASSERT_SIZE(totalSize);
}

static void PopulateINS1Mappings(map_name_to_info_t mappings[DID_COUNT])
{
	typedef ins_1_t MAP_TYPE;
	map_name_to_info_t& m = mappings[DID_INS_1];
	uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "week", week, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "timeOfWeek", timeOfWeek, 0, DataTypeDouble, double);
    ADD_MAP(m, totalSize, "insStatus", insStatus, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "hdwStatus", hdwStatus, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "theta[0]", theta[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "theta[1]", theta[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "theta[2]", theta[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "uvw[0]", uvw[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "uvw[1]", uvw[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "uvw[2]", uvw[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "lla[0]", lla[0], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "lla[1]", lla[1], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "lla[2]", lla[2], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "ned[0]", ned[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "ned[1]", ned[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "ned[2]", ned[2], 0, DataTypeFloat, float&);

    ASSERT_SIZE(totalSize);
}

static void PopulateINS2Mappings(map_name_to_info_t mappings[DID_COUNT], uint32_t did)
{
	typedef ins_2_t MAP_TYPE;
	map_name_to_info_t& m = mappings[did];
	uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "week", week, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "timeOfWeek", timeOfWeek, 0, DataTypeDouble, double);
    ADD_MAP(m, totalSize, "insStatus", insStatus, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "hdwStatus", hdwStatus, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "qn2b[0]", qn2b[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "qn2b[1]", qn2b[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "qn2b[2]", qn2b[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "qn2b[3]", qn2b[3], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "uvw[0]", uvw[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "uvw[1]", uvw[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "uvw[2]", uvw[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "lla[0]", lla[0], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "lla[1]", lla[1], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "lla[2]", lla[2], 0, DataTypeDouble, double&);

    ASSERT_SIZE(totalSize);
}

static void PopulateINS4Mappings(map_name_to_info_t mappings[DID_COUNT])
{
	typedef ins_4_t MAP_TYPE;
	map_name_to_info_t& m = mappings[DID_INS_4];
	uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "week", week, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "timeOfWeek", timeOfWeek, 0, DataTypeDouble, double);
    ADD_MAP(m, totalSize, "insStatus", insStatus, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "hdwStatus", hdwStatus, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "qe2b[0]", qe2b[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "qe2b[1]", qe2b[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "qe2b[2]", qe2b[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "qe2b[3]", qe2b[3], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "ve[0]", ve[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "ve[1]", ve[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "ve[2]", ve[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "ecef[0]", ecef[0], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "ecef[1]", ecef[1], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "ecef[2]", ecef[2], 0, DataTypeDouble, double&);

    ASSERT_SIZE(totalSize);
}

static void PopulateGpsPosMappings(map_name_to_info_t mappings[DID_COUNT], uint32_t id)
{
	typedef gps_pos_t MAP_TYPE;
	map_name_to_info_t& m = mappings[id];
	uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "week", week, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "timeOfWeekMs", timeOfWeekMs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "status", status, 0, DataTypeUInt32, uint32_t);
	ADD_MAP(m, totalSize, "ecef[0]", ecef[0], 0, DataTypeDouble, double&);
	ADD_MAP(m, totalSize, "ecef[1]", ecef[1], 0, DataTypeDouble, double&);
	ADD_MAP(m, totalSize, "ecef[2]", ecef[2], 0, DataTypeDouble, double&);
	ADD_MAP(m, totalSize, "lla[0]", lla[0], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "lla[1]", lla[1], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "lla[2]", lla[2], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "hMSL", hMSL, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "hAcc", hAcc, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "vAcc", vAcc, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "pDop", pDop, 0, DataTypeFloat, float);
	ADD_MAP(m, totalSize, "cnoMean", cnoMean, 0, DataTypeFloat, float);
	ADD_MAP(m, totalSize, "towOffset", towOffset, 0, DataTypeDouble, double);
	ADD_MAP(m, totalSize, "leapS", leapS, 0, DataTypeUInt8, uint8_t);
	ADD_MAP(m, totalSize, "reserved[0]", reserved[0], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "reserved[1]", reserved[1], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "reserved[2]", reserved[2], 0, DataTypeUInt8, uint8_t&);

    ASSERT_SIZE(totalSize);
}

static void PopulateGpsVelMappings(map_name_to_info_t mappings[DID_COUNT], uint32_t id)
{
	typedef gps_vel_t MAP_TYPE;
	map_name_to_info_t& m = mappings[id];
	uint32_t totalSize = 0;
	ADD_MAP(m, totalSize, "timeOfWeekMs", timeOfWeekMs, 0, DataTypeUInt32, uint32_t);
	ADD_MAP(m, totalSize, "vel[0]", vel[0], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "vel[1]", vel[1], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "vel[2]", vel[2], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "sAcc", sAcc, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "status", status, 0, DataTypeUInt32, uint32_t);

	ASSERT_SIZE(totalSize);
}

static void PopulateGPSCNOMappings(map_name_to_info_t mappings[DID_COUNT], uint32_t id)
{
	typedef gps_sat_t MAP_TYPE;
	map_name_to_info_t& m = mappings[id];
	uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "timeOfWeekMs", timeOfWeekMs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "numSats", numSats, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[0]", sat[0].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[1]", sat[1].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[2]", sat[2].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[3]", sat[3].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[4]", sat[4].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[5]", sat[5].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[6]", sat[6].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[7]", sat[7].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[8]", sat[8].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[9]", sat[9].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[10]", sat[10].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[11]", sat[11].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[12]", sat[12].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[13]", sat[13].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[14]", sat[14].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[15]", sat[15].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[16]", sat[16].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[17]", sat[17].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[18]", sat[18].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[19]", sat[19].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[20]", sat[20].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[21]", sat[21].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[22]", sat[22].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[23]", sat[23].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[24]", sat[24].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[25]", sat[25].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[26]", sat[26].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[27]", sat[27].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[28]", sat[28].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[29]", sat[29].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[30]", sat[30].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[31]", sat[31].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[32]", sat[32].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[33]", sat[33].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[34]", sat[34].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[35]", sat[35].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[36]", sat[36].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[37]", sat[37].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[38]", sat[38].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[39]", sat[39].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[40]", sat[40].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[41]", sat[41].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[42]", sat[42].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[43]", sat[43].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[44]", sat[44].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[45]", sat[45].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[46]", sat[46].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[47]", sat[47].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[48]", sat[48].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "svId[49]", sat[49].svId, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[0]", sat[0].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[1]", sat[1].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[2]", sat[2].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[3]", sat[3].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[4]", sat[4].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[5]", sat[5].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[6]", sat[6].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[7]", sat[7].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[8]", sat[8].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[9]", sat[9].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[10]", sat[10].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[11]", sat[11].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[12]", sat[12].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[13]", sat[13].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[14]", sat[14].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[15]", sat[15].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[16]", sat[16].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[17]", sat[17].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[18]", sat[18].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[19]", sat[19].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[20]", sat[20].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[21]", sat[21].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[22]", sat[22].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[23]", sat[23].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[24]", sat[24].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[25]", sat[25].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[26]", sat[26].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[27]", sat[27].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[28]", sat[28].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[29]", sat[29].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[30]", sat[30].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[31]", sat[31].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[32]", sat[32].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[33]", sat[33].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[34]", sat[34].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[35]", sat[35].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[36]", sat[36].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[37]", sat[37].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[38]", sat[38].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[39]", sat[39].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[40]", sat[40].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[41]", sat[41].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[42]", sat[42].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[43]", sat[43].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[44]", sat[44].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[45]", sat[45].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[46]", sat[46].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[47]", sat[47].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[48]", sat[48].cno, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "cno[49]", sat[49].cno, 0, DataTypeUInt8, uint8_t);

    ASSERT_SIZE(totalSize);
}

static void PopulateMagnetometerMappings(map_name_to_info_t mappings[DID_COUNT], uint32_t id)
{
	typedef magnetometer_t MAP_TYPE;
	map_name_to_info_t& m = mappings[id];
	uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "time", time, 0, DataTypeDouble, double);
    ADD_MAP(m, totalSize, "mag[0]", mag[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag[1]", mag[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag[2]", mag[2], 0, DataTypeFloat, float&);

    ASSERT_SIZE(totalSize);
}

static void PopulateBarometerMappings(map_name_to_info_t mappings[DID_COUNT])
{
	typedef barometer_t MAP_TYPE;
	map_name_to_info_t& m = mappings[DID_BAROMETER];
	uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "time", time, 0, DataTypeDouble, double);
    ADD_MAP(m, totalSize, "bar", bar, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "mslBar", mslBar, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "barTemp", barTemp, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "humidity", humidity, 0, DataTypeFloat, float);

    ASSERT_SIZE(totalSize);
}

static void PopulateIMUDeltaThetaVelocityMappings(map_name_to_info_t mappings[DID_COUNT])
{
	typedef preintegrated_imu_t MAP_TYPE;
	map_name_to_info_t& m = mappings[DID_PREINTEGRATED_IMU];
	uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "time", time, 0, DataTypeDouble, double);
    ADD_MAP(m, totalSize, "theta1[0]", theta1[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "theta1[1]", theta1[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "theta1[2]", theta1[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "vel1[0]", vel1[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "vel1[1]", vel1[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "vel1[2]", vel1[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "theta2[0]", theta2[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "theta2[1]", theta2[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "theta2[2]", theta2[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "vel2[0]", vel2[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "vel2[1]", vel2[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "vel2[2]", vel2[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "dt", dt, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "status", status, 0, DataTypeUInt32, uint32_t);

    ASSERT_SIZE(totalSize);
}

static void PopulateIMUDeltaThetaVelocityMagMappings(map_name_to_info_t mappings[DID_COUNT])
{
	typedef pimu_mag_t MAP_TYPE;
	map_name_to_info_t& m = mappings[DID_PREINTEGRATED_IMU_MAG];
	uint32_t totalSize = 0;
	ADD_MAP(m, totalSize, "imutime", pimu.time, 0, DataTypeDouble, double);
	ADD_MAP(m, totalSize, "theta1[0]", pimu.theta1[0], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "theta1[1]", pimu.theta1[1], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "theta1[2]", pimu.theta1[2], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "vel1[0]", pimu.vel1[0], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "vel1[1]", pimu.vel1[1], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "vel1[2]", pimu.vel1[2], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "theta2[0]", pimu.theta2[0], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "theta2[1]", pimu.theta2[1], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "theta2[2]", pimu.theta2[2], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "vel2[0]", pimu.vel2[0], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "vel2[1]", pimu.vel2[1], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "vel2[2]", pimu.vel2[2], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "dt", pimu.dt, 0, DataTypeFloat, float);
	ADD_MAP(m, totalSize, "imustatus", pimu.status, 0, DataTypeUInt32, uint32_t);
	ADD_MAP(m, totalSize, "mag1time", mag1.time, 0, DataTypeDouble, double);
	ADD_MAP(m, totalSize, "mag1[0]", mag1.mag[0], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "mag1[1]", mag1.mag[1], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "mag1[2]", mag1.mag[2], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "mag2time", mag2.time, 0, DataTypeDouble, double);
	ADD_MAP(m, totalSize, "mag2[0]", mag2.mag[0], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "mag2[1]", mag2.mag[1], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "mag2[2]", mag2.mag[2], 0, DataTypeFloat, float&);

	ASSERT_SIZE(totalSize);
}

static void PopulateIMUMagnetometerMappings(map_name_to_info_t mappings[DID_COUNT], uint32_t id)
{
	typedef imu_mag_t MAP_TYPE;
	map_name_to_info_t& m = mappings[id];
	uint32_t totalSize = 0;
	ADD_MAP(m, totalSize, "time", imu.time, 0, DataTypeDouble, double);
	ADD_MAP(m, totalSize, "pqr1[0]", imu.I[0].pqr[0], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "pqr1[1]", imu.I[0].pqr[1], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "pqr1[2]", imu.I[0].pqr[2], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "acc1[0]", imu.I[0].acc[0], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "acc1[1]", imu.I[0].acc[1], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "acc1[2]", imu.I[0].acc[2], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "pqr2[0]", imu.I[1].pqr[0], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "pqr2[1]", imu.I[1].pqr[1], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "pqr2[2]", imu.I[1].pqr[2], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "acc2[0]", imu.I[1].acc[0], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "acc2[1]", imu.I[1].acc[1], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "acc2[2]", imu.I[1].acc[2], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "imustatus", imu.status, 0, DataTypeUInt32, uint32_t);
	ADD_MAP(m, totalSize, "mag1time", mag1.time, 0, DataTypeDouble, double);
	ADD_MAP(m, totalSize, "mag1[0]", mag1.mag[0], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "mag1[1]", mag1.mag[1], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "mag1[2]", mag1.mag[2], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "mag2time", mag2.time, 0, DataTypeDouble, double);
	ADD_MAP(m, totalSize, "mag2[0]", mag2.mag[0], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "mag2[1]", mag2.mag[1], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "mag2[2]", mag2.mag[2], 0, DataTypeFloat, float&);

	ASSERT_SIZE(totalSize);
}

static void PopulateWheelEncoderMappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef wheel_encoder_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_WHEEL_ENCODER];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "timeOfWeek", timeOfWeek, 0, DataTypeDouble, double);
    ADD_MAP(m, totalSize, "status", status, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "theta_l", theta_l, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "omega_l", omega_l, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "theta_r", theta_r, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "omega_r", omega_r, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "wrap_count_l", wrap_count_l, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "wrap_count_r", wrap_count_r, 0, DataTypeUInt32, uint32_t);

    ASSERT_SIZE(totalSize);
}

static void PopulateConfigMappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef system_command_t MAP_TYPE;
	map_name_to_info_t& m = mappings[DID_SYS_CMD];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "command", command, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "invCommand", invCommand, 0, DataTypeUInt32, uint32_t);

    ASSERT_SIZE(totalSize);
}

static void PopulateFlashConfigMappings(map_name_to_info_t mappings[DID_COUNT])
{
	typedef nvm_flash_cfg_t MAP_TYPE;
	map_name_to_info_t& m = mappings[DID_FLASH_CONFIG];
	uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "size", size, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "checksum", checksum, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "key", key, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "startupImuDtMs", startupImuDtMs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "startupNavDtMs", startupNavDtMs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "ser0BaudRate", ser0BaudRate, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "ser1BaudRate", ser1BaudRate, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "insRotation[0]", insRotation[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "insRotation[1]", insRotation[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "insRotation[2]", insRotation[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "insOffset[0]", insOffset[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "insOffset[1]", insOffset[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "insOffset[2]", insOffset[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "gps1AntOffset[0]", gps1AntOffset[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "gps1AntOffset[1]", gps1AntOffset[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "gps1AntOffset[2]", gps1AntOffset[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "insDynModel", insDynModel, 0, DataTypeUInt8, uint8_t);
	ADD_MAP(m, totalSize, "reserved", reserved, 0, DataTypeUInt8, uint8_t);
	ADD_MAP(m, totalSize, "gnssSatSigConst", gnssSatSigConst, 0, DataTypeUInt16, uint16_t);
	ADD_MAP(m, totalSize, "sysCfgBits", sysCfgBits, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "refLla[0]", refLla[0], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "refLla[1]", refLla[1], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "refLla[2]", refLla[2], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "lastLla[0]", lastLla[0], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "lastLla[1]", lastLla[1], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "lastLla[2]", lastLla[2], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "lastLlaTimeOfWeekMs", lastLlaTimeOfWeekMs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "lastLlaWeek", lastLlaWeek, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "lastLlaUpdateDistance", lastLlaUpdateDistance, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "ioConfig", ioConfig, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cBrdConfig", cBrdConfig, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "magInclination", magInclination, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "magDeclination", magDeclination, 0, DataTypeFloat, float);
	ADD_MAP(m, totalSize, "gps2AntOffset[0]", gps2AntOffset[0], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "gps2AntOffset[1]", gps2AntOffset[1], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "gps2AntOffset[2]", gps2AntOffset[2], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "zeroVelRotation[0]", zeroVelRotation[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "zeroVelRotation[1]", zeroVelRotation[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "zeroVelRotation[2]", zeroVelRotation[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "zeroVelOffset[0]", zeroVelOffset[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "zeroVelOffset[1]", zeroVelOffset[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "zeroVelOffset[2]", zeroVelOffset[2], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "gpsTimeSyncPeriodMs", gpsTimeSyncPeriodMs, 0, DataTypeUInt32, uint32_t);
	ADD_MAP(m, totalSize, "startupGPSDtMs", startupGPSDtMs, 0, DataTypeUInt32, uint32_t);
	ADD_MAP(m, totalSize, "RTKCfgBits", RTKCfgBits, 0, DataTypeUInt32, uint32_t);
	ADD_MAP(m, totalSize, "sensorConfig", sensorConfig, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "wheelConfig.bits", wheelConfig.bits, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "wheelConfig.e_i2l[0]", wheelConfig.e_i2l[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "wheelConfig.e_i2l[1]", wheelConfig.e_i2l[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "wheelConfig.e_i2l[2]", wheelConfig.e_i2l[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "wheelConfig.t_i2l[0]", wheelConfig.t_i2l[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "wheelConfig.t_i2l[1]", wheelConfig.t_i2l[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "wheelConfig.t_i2l[2]", wheelConfig.t_i2l[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "wheelConfig.distance", wheelConfig.distance, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "wheelConfig.diameter", wheelConfig.diameter, 0, DataTypeFloat, float);
	ADD_MAP(m, totalSize, "gpsMinimumElevation", gpsMinimumElevation, 0, DataTypeFloat, float);

    ASSERT_SIZE(totalSize);
}

static void PopulateRtkRelMappings(map_name_to_info_t mappings[DID_COUNT])
{
	typedef gps_rtk_rel_t MAP_TYPE;
	map_name_to_info_t& m = mappings[DID_GPS1_RTK_POS_REL];
	uint32_t totalSize = 0;
	ADD_MAP(m, totalSize, "timeOfWeekMs", timeOfWeekMs, 0, DataTypeUInt32, uint32_t);
	ADD_MAP(m, totalSize, "baseToRoverVector[0]", baseToRoverVector[0], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "baseToRoverVector[1]", baseToRoverVector[1], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "baseToRoverVector[2]", baseToRoverVector[2], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "differentialAge", differentialAge, 0, DataTypeFloat, float);
	ADD_MAP(m, totalSize, "arRatio", arRatio, 0, DataTypeFloat, float);
	ADD_MAP(m, totalSize, "baseToRoverDistance", baseToRoverDistance, 0, DataTypeFloat, float);
	ADD_MAP(m, totalSize, "baseToRoverHeading", baseToRoverHeading, 0, DataTypeFloat, float);
	ADD_MAP(m, totalSize, "baseToRoverHeadingAcc", baseToRoverHeadingAcc, 0, DataTypeFloat, float);
	ADD_MAP(m, totalSize, "status", status, 0, DataTypeUInt32, uint32_t);

	ASSERT_SIZE(totalSize);
}

static void PopulateRtkMiscMappings(map_name_to_info_t mappings[DID_COUNT])
{
	typedef gps_rtk_misc_t MAP_TYPE;
	map_name_to_info_t& m = mappings[DID_GPS1_RTK_POS_MISC];
	uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "timeOfWeekMs", timeOfWeekMs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "accuracyPos[0]", accuracyPos[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "accuracyPos[1]", accuracyPos[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "accuracyPos[2]", accuracyPos[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "accuracyCov[0]", accuracyCov[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "accuracyCov[1]", accuracyCov[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "accuracyCov[2]", accuracyCov[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "arThreshold", arThreshold, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "gDop", gDop, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "hDop", hDop, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "vDop", vDop, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "baseLla[0]", baseLla[0], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "baseLla[1]", baseLla[1], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "baseLla[2]", baseLla[2], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "cycleSlipCount", cycleSlipCount, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "roverGpsObservationCount", roverGpsObservationCount, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "baseGpsObservationCount", baseGpsObservationCount, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "roverGlonassObservationCount", roverGlonassObservationCount, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "baseGlonassObservationCount", baseGlonassObservationCount, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "roverGalileoObservationCount", roverGalileoObservationCount, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "baseGalileoObservationCount", baseGalileoObservationCount, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "roverBeidouObservationCount", roverBeidouObservationCount, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "baseBeidouObservationCount", baseBeidouObservationCount, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "roverQzsObservationCount", roverQzsObservationCount, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "baseQzsObservationCount", baseQzsObservationCount, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "roverGpsEphemerisCount", roverGpsEphemerisCount, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "baseGpsEphemerisCount", baseGpsEphemerisCount, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "roverGlonassEphemerisCount", roverGlonassEphemerisCount, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "baseGlonassEphemerisCount", baseGlonassEphemerisCount, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "roverGalileoEphemerisCount", roverGalileoEphemerisCount, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "baseGalileoEphemerisCount", baseGalileoEphemerisCount, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "roverBeidouEphemerisCount", roverBeidouEphemerisCount, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "baseBeidouEphemerisCount", baseBeidouEphemerisCount, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "roverQzsEphemerisCount", roverQzsEphemerisCount, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "baseQzsEphemerisCount", baseQzsEphemerisCount, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "roverSbasCount", roverSbasCount, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "baseSbasCount", baseSbasCount, 0, DataTypeUInt32, uint32_t);
	ADD_MAP(m, totalSize, "baseAntennaCount", baseAntennaCount, 0, DataTypeUInt32, uint32_t);
	ADD_MAP(m, totalSize, "ionUtcAlmCount", ionUtcAlmCount, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "correctionChecksumFailures", correctionChecksumFailures, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "timeToFirstFixMs", timeToFirstFixMs, 0, DataTypeUInt32, uint32_t);

    ASSERT_SIZE(totalSize);
}

static void PopulateGpsRawMappings(map_name_to_info_t mappings[DID_COUNT], uint32_t id)
{
	typedef gps_raw_t MAP_TYPE;
	map_name_to_info_t& m = mappings[id];
	uint32_t totalSize = 0;

    ADD_MAP(m, totalSize, "receiveIndex", receiverIndex, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "dataType", dataType, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "obsCount", obsCount, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "reserved", reserved, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "dataBuf", data.buf, 0, DataTypeBinary, uint8_t[MEMBERSIZE(MAP_TYPE, data.buf)]);

    ASSERT_SIZE(totalSize);
}

static void PopulateStrobeInTimeMappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef strobe_in_time_t MAP_TYPE;
	map_name_to_info_t& m = mappings[DID_STROBE_IN_TIME];
    uint32_t totalSize = 0;

    ADD_MAP(m, totalSize, "week", week, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "timeOfWeekMs", timeOfWeekMs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "pin", pin, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "count", count, 0, DataTypeUInt32, uint32_t);

    ASSERT_SIZE(totalSize);
}

static void PopulateRtosInfoMappings(map_name_to_info_t mappings[DID_COUNT])
{
	typedef rtos_info_t MAP_TYPE;
	map_name_to_info_t& m = mappings[DID_RTOS_INFO];
	uint32_t totalSize = 0;

    ADD_MAP(m, totalSize, "name[0]", task[0].name, MAX_TASK_NAME_LEN, DataTypeString, char[MAX_TASK_NAME_LEN]);
    ADD_MAP(m, totalSize, "priority[0]", task[0].priority, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "stackUnused[0]", task[0].stackUnused, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "periodMs[0]", task[0].periodMs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "runTimeUs[0]", task[0].runTimeUs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "maxRunTimeUs[0]", task[0].maxRunTimeUs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "averageRunTimeUs[0]", task[0].averageRunTimeUs, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "gapCount[0]", task[0].gapCount, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cpuUsage[0]", task[0].cpuUsage, 0, DataTypeFloat, f_t);
    ADD_MAP(m, totalSize, "handle[0]", task[0].handle, 0, DataTypeUInt32, uint32_t);

    ADD_MAP(m, totalSize, "name[1]", task[1].name, MAX_TASK_NAME_LEN, DataTypeString, char[MAX_TASK_NAME_LEN]);
    ADD_MAP(m, totalSize, "priority[1]", task[1].priority, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "stackUnused[1]", task[1].stackUnused, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "periodMs[1]", task[1].periodMs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "runTimeUs[1]", task[1].runTimeUs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "maxRunTimeUs[1]", task[1].maxRunTimeUs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "averageRunTimeUs[1]", task[1].averageRunTimeUs, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "gapCount[1]", task[1].gapCount, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cpuUsage[1]", task[1].cpuUsage, 0, DataTypeFloat, f_t);
    ADD_MAP(m, totalSize, "handle[1]", task[1].handle, 0, DataTypeUInt32, uint32_t);

    ADD_MAP(m, totalSize, "name[2]", task[2].name, MAX_TASK_NAME_LEN, DataTypeString, char[MAX_TASK_NAME_LEN]);
    ADD_MAP(m, totalSize, "priority[2]", task[2].priority, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "stackUnused[2]", task[2].stackUnused, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "periodMs[2]", task[2].periodMs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "runTimeUs[2]", task[2].runTimeUs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "maxRunTimeUs[2]", task[2].maxRunTimeUs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "averageRunTimeUs[2]", task[2].averageRunTimeUs, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "gapCount[2]", task[2].gapCount, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cpuUsage[2]", task[2].cpuUsage, 0, DataTypeFloat, f_t);
    ADD_MAP(m, totalSize, "handle[2]", task[2].handle, 0, DataTypeUInt32, uint32_t);

    ADD_MAP(m, totalSize, "name[3]", task[3].name, MAX_TASK_NAME_LEN, DataTypeString, char[MAX_TASK_NAME_LEN]);
    ADD_MAP(m, totalSize, "priority[3]", task[3].priority, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "stackUnused[3]", task[3].stackUnused, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "periodMs[3]", task[3].periodMs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "runTimeUs[3]", task[3].runTimeUs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "maxRunTimeUs[3]", task[3].maxRunTimeUs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "averageRunTimeUs[3]", task[3].averageRunTimeUs, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "gapCount[3]", task[3].gapCount, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cpuUsage[3]", task[3].cpuUsage, 0, DataTypeFloat, f_t);
    ADD_MAP(m, totalSize, "handle[3]", task[3].handle, 0, DataTypeUInt32, uint32_t);

    ADD_MAP(m, totalSize, "name[4]", task[4].name, MAX_TASK_NAME_LEN, DataTypeString, char[MAX_TASK_NAME_LEN]);
    ADD_MAP(m, totalSize, "priority[4]", task[4].priority, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "stackUnused[4]", task[4].stackUnused, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "periodMs[4]", task[4].periodMs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "runTimeUs[4]", task[4].runTimeUs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "maxRunTimeUs[4]", task[4].maxRunTimeUs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "averageRunTimeUs[4]", task[4].averageRunTimeUs, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "gapCount[4]", task[4].gapCount, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cpuUsage[4]", task[4].cpuUsage, 0, DataTypeFloat, f_t);
    ADD_MAP(m, totalSize, "handle[4]", task[4].handle, 0, DataTypeUInt32, uint32_t);

    ADD_MAP(m, totalSize, "name[5]", task[5].name, MAX_TASK_NAME_LEN, DataTypeString, char[MAX_TASK_NAME_LEN]);
    ADD_MAP(m, totalSize, "priority[5]", task[5].priority, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "stackUnused[5]", task[5].stackUnused, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "periodMs[5]", task[5].periodMs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "runTimeUs[5]", task[5].runTimeUs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "maxRunTimeUs[5]", task[5].maxRunTimeUs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "averageRunTimeUs[5]", task[5].averageRunTimeUs, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "gapCount[5]", task[5].gapCount, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cpuUsage[5]", task[5].cpuUsage, 0, DataTypeFloat, f_t);
    ADD_MAP(m, totalSize, "handle[5]", task[5].handle, 0, DataTypeUInt32, uint32_t);

    ADD_MAP(m, totalSize, "freeHeapSize", freeHeapSize, 0, DataTypeUInt32, uint32_t);
	ADD_MAP(m, totalSize, "mallocSize", mallocSize, 0, DataTypeUInt32, uint32_t);
	ADD_MAP(m, totalSize, "freeSize", freeSize, 0, DataTypeUInt32, uint32_t);

    ASSERT_SIZE(totalSize);
}
static void PopulateCanConfigMappings(map_name_to_info_t mappings[DID_COUNT])
{
	typedef can_config_t MAP_TYPE;
	map_name_to_info_t& m = mappings[DID_CAN_CONFIG];
	uint32_t totalSize = 0;
	ADD_MAP(m, totalSize, "can_period_mult[CID_INS_TIME]", can_period_mult[CID_INS_TIME], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_period_mult[CID_INS_STATUS]", can_period_mult[CID_INS_STATUS], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_period_mult[CID_INS_EULER]", can_period_mult[CID_INS_EULER], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_period_mult[CID_INS_QUATN2B]", can_period_mult[CID_INS_QUATN2B], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_period_mult[CID_INS_QUATE2B]", can_period_mult[CID_INS_QUATE2B], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_period_mult[CID_INS_UVW]", can_period_mult[CID_INS_UVW], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_period_mult[CID_INS_VE]", can_period_mult[CID_INS_VE], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_period_mult[CID_INS_LAT]", can_period_mult[CID_INS_LAT], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_period_mult[CID_INS_LON]", can_period_mult[CID_INS_LON], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_period_mult[CID_INS_ALT]", can_period_mult[CID_INS_ALT], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_period_mult[CID_INS_NORTH_EAST]", can_period_mult[CID_INS_NORTH_EAST], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_period_mult[CID_INS_DOWN]", can_period_mult[CID_INS_DOWN], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_period_mult[CID_INS_ECEF_X]", can_period_mult[CID_INS_ECEF_X], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_period_mult[CID_INS_ECEF_Y]", can_period_mult[CID_INS_ECEF_Y], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_period_mult[CID_INS_ECEF_Z]", can_period_mult[CID_INS_ECEF_Z], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_period_mult[CID_INS_MSL]", can_period_mult[CID_INS_MSL], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_period_mult[CID_PREINT_PX]", can_period_mult[CID_PREINT_PX], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_period_mult[CID_PREINT_QY]", can_period_mult[CID_PREINT_QY], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_period_mult[CID_PREINT_RZ]", can_period_mult[CID_PREINT_RZ], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_period_mult[CID_DUAL_PX]", can_period_mult[CID_DUAL_PX], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_period_mult[CID_DUAL_QY]", can_period_mult[CID_DUAL_QY], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_period_mult[CID_DUAL_RZ]", can_period_mult[CID_DUAL_RZ], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_period_mult[CID_GPS1_POS]", can_period_mult[CID_GPS1_POS], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_period_mult[CID_ROLL_ROLLRATE]", can_period_mult[CID_ROLL_ROLLRATE], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_period_mult[CID_GPS1_RTK_REL]", can_period_mult[CID_GPS1_RTK_REL], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_transmit_address[CID_INS_TIME]", can_transmit_address[CID_INS_TIME], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_transmit_address[CID_INS_STATUS]", can_transmit_address[CID_INS_STATUS], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_transmit_address[CID_INS_EULER]", can_transmit_address[CID_INS_EULER], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_transmit_address[CID_INS_QUATN2B]", can_transmit_address[CID_INS_QUATN2B], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_transmit_address[CID_INS_QUATE2B]", can_transmit_address[CID_INS_QUATE2B], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_transmit_address[CID_INS_UVW]", can_transmit_address[CID_INS_UVW], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_transmit_address[CID_INS_VE]", can_transmit_address[CID_INS_VE], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_transmit_address[CID_INS_LAT]", can_transmit_address[CID_INS_LAT], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_transmit_address[CID_INS_LON]", can_transmit_address[CID_INS_LON], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_transmit_address[CID_INS_ALT]", can_transmit_address[CID_INS_ALT], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_transmit_address[CID_INS_NORTH_EAST]", can_transmit_address[CID_INS_NORTH_EAST], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_transmit_address[CID_INS_DOWN]", can_transmit_address[CID_INS_DOWN], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_transmit_address[CID_INS_ECEF_X]", can_transmit_address[CID_INS_ECEF_X], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_transmit_address[CID_INS_ECEF_Y]", can_transmit_address[CID_INS_ECEF_Y], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_transmit_address[CID_INS_ECEF_Z]", can_transmit_address[CID_INS_ECEF_Z], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_transmit_address[CID_INS_MSL]", can_transmit_address[CID_INS_MSL], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_transmit_address[CID_PREINT_PX]", can_transmit_address[CID_PREINT_PX], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_transmit_address[CID_PREINT_QY]", can_transmit_address[CID_PREINT_QY], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_transmit_address[CID_PREINT_RZ]", can_transmit_address[CID_PREINT_RZ], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_transmit_address[CID_DUAL_PX]", can_transmit_address[CID_DUAL_PX], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_transmit_address[CID_DUAL_QY]", can_transmit_address[CID_DUAL_QY], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_transmit_address[CID_DUAL_RZ]", can_transmit_address[CID_DUAL_RZ], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_transmit_address[CID_GPS1_POS]", can_transmit_address[CID_GPS1_POS], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_transmit_address[CID_GPS1_RTK_REL]", can_transmit_address[CID_GPS1_RTK_REL], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_transmit_address[CID_ROLL_ROLLRATE]", can_transmit_address[CID_ROLL_ROLLRATE], 0, DataTypeUInt32, uint32_t&);
	ADD_MAP(m, totalSize, "can_baudrate_kbps", can_baudrate_kbps, 0, DataTypeUInt32, uint32_t);
	ADD_MAP(m, totalSize, "can_receive_address", can_receive_address, 0, DataTypeUInt32, uint32_t);

	ASSERT_SIZE(totalSize);
}

static void PopulateDiagMsgMappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef diag_msg_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_DIAGNOSTIC_MESSAGE];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "timeOfWeekMs", timeOfWeekMs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "messageLength", messageLength, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "message", message, MEMBERSIZE(diag_msg_t, message), DataTypeString, char[MEMBERSIZE(diag_msg_t, message)]);

    ASSERT_SIZE(totalSize);
}

#ifdef USE_IS_INTERNAL

static void PopulateSensorsADCMappings(map_name_to_info_t mappings[DID_COUNT])
{
	typedef sys_sensors_adc_t MAP_TYPE;
	map_name_to_info_t& m = mappings[DID_SENSORS_ADC];
	uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "time", time, 0, DataTypeDouble, double);
    ADD_MAP(m, totalSize, "pqr1[0]", mpu[0].pqr[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr1[1]", mpu[0].pqr[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr1[2]", mpu[0].pqr[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc1[0]", mpu[0].acc[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc1[1]", mpu[0].acc[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc1[2]", mpu[0].acc[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag1[0]", mpu[0].mag[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag1[1]", mpu[0].mag[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag1[2]", mpu[0].mag[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "temp1", mpu[0].temp, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "pqr2[0]", mpu[1].pqr[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr2[1]", mpu[1].pqr[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr2[2]", mpu[1].pqr[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc2[0]", mpu[1].acc[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc2[1]", mpu[1].acc[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc2[2]", mpu[1].acc[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag2[0]", mpu[1].mag[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag2[1]", mpu[1].mag[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag2[2]", mpu[1].mag[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "temp2", mpu[1].temp, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "bar", bar, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "barTemp", barTemp, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "humidity", humidity, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "ana[0]", ana[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "ana[1]", ana[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "ana[2]", ana[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "ana[3]", ana[3], 0, DataTypeFloat, float&);

    ASSERT_SIZE(totalSize);
}

static void PopulateSensorsISMappings(map_name_to_info_t mappings[DID_COUNT], uint32_t id)
{
	typedef sensors_w_temp_t MAP_TYPE;
	map_name_to_info_t& m = mappings[id];
	uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "pqr1[0]", mpu[0].pqr[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr1[1]", mpu[0].pqr[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr1[2]", mpu[0].pqr[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc1[0]", mpu[0].acc[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc1[1]", mpu[0].acc[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc1[2]", mpu[0].acc[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag1[0]", mpu[0].mag[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag1[1]", mpu[0].mag[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag1[2]", mpu[0].mag[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "temp1", mpu[0].temp, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "pqr2[0]", mpu[1].pqr[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr2[1]", mpu[1].pqr[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr2[2]", mpu[1].pqr[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc2[0]", mpu[1].acc[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc2[1]", mpu[1].acc[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc2[2]", mpu[1].acc[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag2[0]", mpu[1].mag[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag2[1]", mpu[1].mag[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag2[2]", mpu[1].mag[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "temp2", mpu[1].temp, 0, DataTypeFloat, float);

    ASSERT_SIZE(totalSize);
}

static void PopulateSensorsTCMappings(map_name_to_info_t mappings[DID_COUNT])
{
	typedef sensors_t MAP_TYPE;
	map_name_to_info_t& m = mappings[DID_SENSORS_TC_BIAS];
	uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "pqr1[0]", mpu[0].pqr[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr1[1]", mpu[0].pqr[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr1[2]", mpu[0].pqr[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc1[0]", mpu[0].acc[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc1[1]", mpu[0].acc[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc1[2]", mpu[0].acc[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag1[0]", mpu[0].mag[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag1[1]", mpu[0].mag[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag1[2]", mpu[0].mag[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr2[0]", mpu[1].pqr[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr2[1]", mpu[1].pqr[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr2[2]", mpu[1].pqr[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc2[0]", mpu[1].acc[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc2[1]", mpu[1].acc[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc2[2]", mpu[1].acc[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag2[0]", mpu[1].mag[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag2[1]", mpu[1].mag[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag2[2]", mpu[1].mag[2], 0, DataTypeFloat, float&);

    ASSERT_SIZE(totalSize);
}

static void PopulateSensorsCompMappings(map_name_to_info_t mappings[DID_COUNT])
{
	typedef sensor_compensation_t MAP_TYPE;
	map_name_to_info_t& m = mappings[DID_SCOMP];
	uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "pqr1[0]", mpu[0].lpfLsb.pqr[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr1[1]", mpu[0].lpfLsb.pqr[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr1[2]", mpu[0].lpfLsb.pqr[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc1[0]", mpu[0].lpfLsb.acc[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc1[1]", mpu[0].lpfLsb.acc[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc1[2]", mpu[0].lpfLsb.acc[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag1[0]", mpu[0].lpfLsb.mag[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag1[1]", mpu[0].lpfLsb.mag[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag1[2]", mpu[0].lpfLsb.mag[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "temp1", mpu[0].lpfLsb.temp, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "temp2", mpu[0].temp, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "tempRampRate1", mpu[0].tempRampRate, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "tci1", mpu[0].tci, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "numTcPts1", mpu[0].numTcPts, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "dtTemp1", mpu[0].dtTemp, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "pqr2[0]", mpu[1].lpfLsb.pqr[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr2[1]", mpu[1].lpfLsb.pqr[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr2[2]", mpu[1].lpfLsb.pqr[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc2[0]", mpu[1].lpfLsb.acc[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc2[1]", mpu[1].lpfLsb.acc[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc2[2]", mpu[1].lpfLsb.acc[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag2[0]", mpu[1].lpfLsb.mag[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag2[1]", mpu[1].lpfLsb.mag[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag2[2]", mpu[1].lpfLsb.mag[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "temp3", mpu[1].lpfLsb.temp, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "temp4", mpu[1].temp, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "tempRampRate2", mpu[1].tempRampRate, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "tci2", mpu[1].tci, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "numTcPts2", mpu[1].numTcPts, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "dtTemp2", mpu[1].dtTemp, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "sampleCount", sampleCount, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "calState", calState, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "alignAccel[0]", alignAccel[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "alignAccel[1]", alignAccel[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "alignAccel[2]", alignAccel[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "status", status, 0, DataTypeUInt32, uint32_t);

    ASSERT_SIZE(totalSize);
}

static void PopulateDebugArrayMappings(map_name_to_info_t mappings[DID_COUNT])
{
	typedef debug_array_t MAP_TYPE;
	map_name_to_info_t& m = mappings[DID_DEBUG_ARRAY];
	uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "i[0]", i[0], 0, DataTypeInt32, int32_t&);
    ADD_MAP(m, totalSize, "i[1]", i[1], 0, DataTypeInt32, int32_t&);
    ADD_MAP(m, totalSize, "i[2]", i[2], 0, DataTypeInt32, int32_t&);
    ADD_MAP(m, totalSize, "i[3]", i[3], 0, DataTypeInt32, int32_t&);
    ADD_MAP(m, totalSize, "i[4]", i[4], 0, DataTypeInt32, int32_t&);
    ADD_MAP(m, totalSize, "i[5]", i[5], 0, DataTypeInt32, int32_t&);
    ADD_MAP(m, totalSize, "i[6]", i[6], 0, DataTypeInt32, int32_t&);
    ADD_MAP(m, totalSize, "i[7]", i[7], 0, DataTypeInt32, int32_t&);
    ADD_MAP(m, totalSize, "i[8]", i[8], 0, DataTypeInt32, int32_t&);
    ADD_MAP(m, totalSize, "f[0]", f[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "f[1]", f[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "f[2]", f[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "f[3]", f[3], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "f[4]", f[4], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "f[5]", f[5], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "f[6]", f[6], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "f[7]", f[7], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "f[8]", f[8], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "lf[0]", lf[0], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "lf[1]", lf[1], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "lf[2]", lf[2], 0, DataTypeDouble, double&);

    ASSERT_SIZE(totalSize);
}

static void PopulateUserPage0Mappings(map_name_to_info_t mappings[DID_COUNT])
{
	typedef nvm_group_0_t MAP_TYPE;
	map_name_to_info_t& m = mappings[DID_NVR_USERPAGE_G0];
	uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "size", size, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "checksum", checksum, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "key", key, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "lockBits", lockBits, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "featureBits", featureBits, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "featureHash1", featureHash1, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "featureHash2", featureHash2, 0, DataTypeUInt32, uint32_t);

    ASSERT_SIZE(totalSize);
}

static void PopulateUserPage1Mappings(map_name_to_info_t mappings[DID_COUNT])
{
	typedef nvm_group_1_t MAP_TYPE;
	map_name_to_info_t& m = mappings[DID_NVR_USERPAGE_G1];
	uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "size", size, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "checksum", checksum, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "key", key, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "bKpqr", cf.bKpqr, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "bKuvw", cf.bKuvw, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "oKat1", cf.oKat1, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "oKat2", cf.oKat2, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "oKuvw", cf.oKuvw, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "oKlla", cf.oKlla, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "mag.bias_cal[0]", mag.bias_cal[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.bias_cal[1]", mag.bias_cal[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.bias_cal[2]", mag.bias_cal[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.Wcal[0]", mag.Wcal[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.Wcal[1]", mag.Wcal[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.Wcal[2]", mag.Wcal[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.Wcal[3]", mag.Wcal[3], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.Wcal[4]", mag.Wcal[4], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.Wcal[5]", mag.Wcal[5], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.Wcal[6]", mag.Wcal[6], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.Wcal[7]", mag.Wcal[7], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.Wcal[8]", mag.Wcal[8], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[0]", mag.DtD[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[1]", mag.DtD[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[2]", mag.DtD[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[3]", mag.DtD[3], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[4]", mag.DtD[4], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[5]", mag.DtD[5], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[6]", mag.DtD[6], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[7]", mag.DtD[7], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[8]", mag.DtD[8], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[9]", mag.DtD[9], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[10]", mag.DtD[10], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[11]", mag.DtD[11], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[12]", mag.DtD[12], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[13]", mag.DtD[13], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[14]", mag.DtD[14], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[15]", mag.DtD[15], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[16]", mag.DtD[16], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[17]", mag.DtD[17], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[18]", mag.DtD[18], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[19]", mag.DtD[19], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[20]", mag.DtD[20], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[21]", mag.DtD[21], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[22]", mag.DtD[22], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[23]", mag.DtD[23], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[24]", mag.DtD[24], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[25]", mag.DtD[25], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[26]", mag.DtD[26], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[27]", mag.DtD[27], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[28]", mag.DtD[28], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[29]", mag.DtD[29], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[30]", mag.DtD[30], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[31]", mag.DtD[31], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[32]", mag.DtD[32], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[33]", mag.DtD[33], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[34]", mag.DtD[34], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[35]", mag.DtD[35], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[36]", mag.DtD[36], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[37]", mag.DtD[37], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[38]", mag.DtD[38], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[39]", mag.DtD[39], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[40]", mag.DtD[40], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[41]", mag.DtD[41], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[42]", mag.DtD[42], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[43]", mag.DtD[43], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[44]", mag.DtD[44], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[45]", mag.DtD[45], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[46]", mag.DtD[46], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[47]", mag.DtD[47], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[48]", mag.DtD[48], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[49]", mag.DtD[49], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[50]", mag.DtD[50], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[51]", mag.DtD[51], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[52]", mag.DtD[52], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[53]", mag.DtD[53], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[54]", mag.DtD[54], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[55]", mag.DtD[55], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[56]", mag.DtD[56], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[57]", mag.DtD[57], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[58]", mag.DtD[58], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[59]", mag.DtD[59], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[60]", mag.DtD[60], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[61]", mag.DtD[61], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[62]", mag.DtD[62], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[63]", mag.DtD[63], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[64]", mag.DtD[64], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[65]", mag.DtD[65], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[66]", mag.DtD[66], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[67]", mag.DtD[67], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[68]", mag.DtD[68], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[69]", mag.DtD[69], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[70]", mag.DtD[70], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[71]", mag.DtD[71], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[72]", mag.DtD[72], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[73]", mag.DtD[73], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[74]", mag.DtD[74], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[75]", mag.DtD[75], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[76]", mag.DtD[76], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[77]", mag.DtD[77], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[78]", mag.DtD[78], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[79]", mag.DtD[79], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[80]", mag.DtD[80], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[81]", mag.DtD[81], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[82]", mag.DtD[82], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[83]", mag.DtD[83], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[84]", mag.DtD[84], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[85]", mag.DtD[85], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[86]", mag.DtD[86], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[87]", mag.DtD[87], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[88]", mag.DtD[88], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[89]", mag.DtD[89], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[90]", mag.DtD[90], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[91]", mag.DtD[91], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[92]", mag.DtD[92], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[93]", mag.DtD[93], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[94]", mag.DtD[94], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[95]", mag.DtD[95], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[96]", mag.DtD[96], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[97]", mag.DtD[97], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[98]", mag.DtD[98], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[99]", mag.DtD[99], 0, DataTypeFloat, float&);

    ASSERT_SIZE(totalSize);
}
static void PopulateInl2MagObsInfo(map_name_to_info_t mappings[DID_COUNT])
{
	typedef inl2_mag_obs_info_t MAP_TYPE;
	map_name_to_info_t& m = mappings[DID_INL2_MAG_OBS_INFO];
	uint32_t totalSize = 0;
	ADD_MAP(m, totalSize, "timeOfWeekMs", timeOfWeekMs, 0, DataTypeUInt32, uint32_t);
	ADD_MAP(m, totalSize, "Ncal_samples", Ncal_samples, 0, DataTypeUInt32, uint32_t);
	ADD_MAP(m, totalSize, "ready", ready, 0, DataTypeUInt32, uint32_t);
	ADD_MAP(m, totalSize, "calibrated", calibrated, 0, DataTypeUInt32, uint32_t);
	ADD_MAP(m, totalSize, "auto_recal", auto_recal, 0, DataTypeUInt32, uint32_t);
	ADD_MAP(m, totalSize, "outlier", outlier, 0, DataTypeUInt32, uint32_t);
	ADD_MAP(m, totalSize, "magHdg", magHdg, 0, DataTypeFloat, float);
	ADD_MAP(m, totalSize, "insHdg", insHdg, 0, DataTypeFloat, float);
	ADD_MAP(m, totalSize, "magInsHdgDelta", magInsHdgDelta, 0, DataTypeFloat, float);
	ADD_MAP(m, totalSize, "nis", nis, 0, DataTypeFloat, float);
	ADD_MAP(m, totalSize, "nis_threshold", nis_threshold, 0, DataTypeFloat, float);
	ADD_MAP(m, totalSize, "Wcal[0]", Wcal[0], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "Wcal[1]", Wcal[1], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "Wcal[2]", Wcal[2], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "Wcal[3]", Wcal[3], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "Wcal[4]", Wcal[4], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "Wcal[5]", Wcal[5], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "Wcal[6]", Wcal[6], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "Wcal[7]", Wcal[7], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "Wcal[8]", Wcal[8], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "activeCalSet", activeCalSet, 0, DataTypeUInt32, uint32_t);
	ADD_MAP(m, totalSize, "magHdgOffset", magHdgOffset, 0, DataTypeFloat, float);
	ADD_MAP(m, totalSize, "Tcal", Tcal, 0, DataTypeFloat, float);

	ASSERT_SIZE(totalSize);
}
static void PopulateInl2StatesMappings(map_name_to_info_t mappings[DID_COUNT])
{
	typedef inl2_states_t MAP_TYPE;
	map_name_to_info_t& m = mappings[DID_INL2_STATES];
	uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "timeOfWeek", timeOfWeek, 0, DataTypeDouble, double);
    ADD_MAP(m, totalSize, "qe2b[0]", qe2b[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "qe2b[1]", qe2b[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "qe2b[2]", qe2b[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "qe2b[3]", qe2b[3], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "ve[0]", ve[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "ve[1]", ve[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "ve[2]", ve[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "ecef[0]", ecef[0], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "ecef[1]", ecef[1], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "ecef[2]", ecef[2], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "biasPqr[0]", biasPqr[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "biasPqr[1]", biasPqr[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "biasPqr[2]", biasPqr[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "biasAcc[0]", biasAcc[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "biasAcc[1]", biasAcc[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "biasAcc[2]", biasAcc[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "biasBaro", biasBaro, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "magDec", magDec, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "magInc", magInc, 0, DataTypeFloat, float);

    ASSERT_SIZE(totalSize);
}

// static void PopulateRtkStateMappings(map_name_to_info_t mappings[DID_COUNT])
// {
//     typedef rtk_state_t MAP_TYPE;
//     map_name_to_info_t& m = mappings[DID_RTK_STATE];
//     uint32_t totalSize = 0;
//     ADD_MAP(m, totalSize, "time.time", time.time, 0, DataTypeInt64, int64_t);
//     ADD_MAP(m, totalSize, "time.sec", time.sec, 0, DataTypeDouble, double);
//     ADD_MAP(m, totalSize, "rp[0]", rp_ecef[0], 0, DataTypeDouble, double&);
//     ADD_MAP(m, totalSize, "rp[1]", rp_ecef[1], 0, DataTypeDouble, double&);
//     ADD_MAP(m, totalSize, "rp[2]", rp_ecef[2], 0, DataTypeDouble, double&);
//     ADD_MAP(m, totalSize, "rv[0]", rv_ecef[0], 0, DataTypeDouble, double&);
//     ADD_MAP(m, totalSize, "rv[1]", rv_ecef[1], 0, DataTypeDouble, double&);
//     ADD_MAP(m, totalSize, "rv[2]", rv_ecef[2], 0, DataTypeDouble, double&);
//     ADD_MAP(m, totalSize, "ra[0]", ra_ecef[0], 0, DataTypeDouble, double&);
//     ADD_MAP(m, totalSize, "ra[1]", ra_ecef[1], 0, DataTypeDouble, double&);
//     ADD_MAP(m, totalSize, "ra[2]", ra_ecef[2], 0, DataTypeDouble, double&);
// 
//     ADD_MAP(m, totalSize, "bp[0]", bp_ecef[0], 0, DataTypeDouble, double&);
//     ADD_MAP(m, totalSize, "bp[1]", bp_ecef[1], 0, DataTypeDouble, double&);
//     ADD_MAP(m, totalSize, "bp[2]", bp_ecef[2], 0, DataTypeDouble, double&);
//     ADD_MAP(m, totalSize, "bv[0]", bv_ecef[0], 0, DataTypeDouble, double&);
//     ADD_MAP(m, totalSize, "bv[1]", bv_ecef[1], 0, DataTypeDouble, double&);
//     ADD_MAP(m, totalSize, "bv[2]", bv_ecef[2], 0, DataTypeDouble, double&);
// }

static void PopulateRtkResidualMappings(map_name_to_info_t mappings[DID_COUNT], int DID)
{
    typedef rtk_residual_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "time.time", time.time, 0, DataTypeInt64, int64_t);
    ADD_MAP(m, totalSize, "time.sec", time.sec, 0, DataTypeDouble, double);
    ADD_MAP(m, totalSize, "nv", nv, 0, DataTypeInt32, int32_t);
}

static void PopulateRtkDebugMappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef rtk_debug_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_RTK_DEBUG];
    uint32_t totalSize = 0;

    ADD_MAP(m, totalSize, "time.time", time.time, 0, DataTypeInt64, int64_t);
    ADD_MAP(m, totalSize, "time.sec", time.sec, 0, DataTypeDouble, double);

    ADD_MAP(m, totalSize, "rej_ovfl", rej_ovfl, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "code_outlier", code_outlier, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "phase_outlier", phase_outlier, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "code_large_residual", code_large_residual, 0, DataTypeUInt8, uint8_t);

    ADD_MAP(m, totalSize, "phase_large_residual", phase_large_residual, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "invalid_base_position", invalid_base_position, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "bad_baseline_holdamb", bad_baseline_holdamb, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "base_position_error", base_position_error, 0, DataTypeUInt8, uint8_t);

    ADD_MAP(m, totalSize, "outc_ovfl", outc_ovfl, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "reset_timer", reset_timer, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "use_ubx_position", use_ubx_position, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "large_v2b", large_v2b, 0, DataTypeUInt8, uint8_t);
    
    ADD_MAP(m, totalSize, "base_position_update", base_position_update, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "rover_position_error", rover_position_error, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "reset_bias", reset_bias, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "start_relpos", start_relpos, 0, DataTypeUInt8, uint8_t);

    ADD_MAP(m, totalSize, "end_relpos", end_relpos, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "start_rtkpos", start_rtkpos, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "pnt_pos_error", pnt_pos_error, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "no_base_obs_data", no_base_obs_data, 0, DataTypeUInt8, uint8_t);

    ADD_MAP(m, totalSize, "diff_age_error", diff_age_error, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "moveb_time_sync_error", moveb_time_sync_error, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "waiting_for_rover_packet", waiting_for_rover_packet, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "waiting_for_base_packet", waiting_for_base_packet, 0, DataTypeUInt8, uint8_t);

	ADD_MAP(m, totalSize, "lsq_error", lsq_error, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "lack_of_valid_sats", lack_of_valid_sats, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "divergent_pnt_pos_iteration", divergent_pnt_pos_iteration, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "chi_square_error", chi_square_error, 0, DataTypeUInt8, uint8_t);

    ADD_MAP(m, totalSize, "cycle_slips", cycle_slips, 0, DataTypeUInt32, uint32_t);

	ADD_MAP(m, totalSize, "ubx_error", ubx_error, 0, DataTypeFloat, float);

	ADD_MAP(m, totalSize, "solStatus", solStatus, 0, DataTypeUInt8, uint8_t);
	ADD_MAP(m, totalSize, "rescode_err_marker", rescode_err_marker, 0, DataTypeUInt8, uint8_t);
	ADD_MAP(m, totalSize, "error_count", error_count, 0, DataTypeUInt8, uint8_t);
	ADD_MAP(m, totalSize, "error_code", error_code, 0, DataTypeUInt8, uint8_t);

    ADD_MAP(m, totalSize, "dist2base", dist2base, 0, DataTypeFloat, float);

	ADD_MAP(m, totalSize, "reserved1", reserved1, 0, DataTypeUInt8, uint8_t);
	ADD_MAP(m, totalSize, "gdop_error", gdop_error, 0, DataTypeUInt8, uint8_t);
	ADD_MAP(m, totalSize, "warning_code", warning_code, 0, DataTypeUInt8, uint8_t);
	ADD_MAP(m, totalSize, "warning_count", warning_count, 0, DataTypeUInt8, uint8_t);

    ADD_MAP(m, totalSize, "double_debug[0]", double_debug[0], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "double_debug[1]", double_debug[1], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "double_debug[2]", double_debug[2], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "double_debug[3]", double_debug[3], 0, DataTypeDouble, double&);

	ADD_MAP(m, totalSize, "debug[0]", debug[0], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "debug[1]", debug[1], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "obs_count_bas", obs_count_bas, 0, DataTypeUInt8, uint8_t);
	ADD_MAP(m, totalSize, "obs_count_rov", obs_count_rov, 0, DataTypeUInt8, uint8_t);

	ADD_MAP(m, totalSize, "obs_pairs_filtered", obs_pairs_filtered, 0, DataTypeUInt8, uint8_t);
	ADD_MAP(m, totalSize, "obs_pairs_used", obs_pairs_used, 0, DataTypeUInt8, uint8_t);
	ADD_MAP(m, totalSize, "raw_ptr_queue_overrun", raw_ptr_queue_overrun, 0, DataTypeUInt8, uint8_t);
    ADD_MAP(m, totalSize, "raw_dat_queue_overrun", raw_dat_queue_overrun, 0, DataTypeUInt8, uint8_t);
}


static void PopulateRtkDebug2Mappings(map_name_to_info_t mappings[DID_COUNT])
{
	typedef rtk_debug_2_t MAP_TYPE;
	map_name_to_info_t& m = mappings[DID_RTK_DEBUG_2];
	uint32_t totalSize = 0;

	ADD_MAP(m, totalSize, "time.time", time.time, 0, DataTypeInt64, int64_t);
	ADD_MAP(m, totalSize, "time.sec", time.sec, 0, DataTypeDouble, double);

#if 0	// This doesn't work in Linux

	char str[50];
	for (int i = 0; i < NUMSATSOL; i++)
	{
		SNPRINTF(str, sizeof(str), "satBiasFloat[%d]", i);
		ADD_MAP(m, totalSize, str, satBiasFloat[i], 0, DataTypeFloat, float&);
	}

	for (int i = 0; i < NUMSATSOL; i++)
	{
		SNPRINTF(str, sizeof(str), "satBiasFix[%d]", i);
		ADD_MAP(m, totalSize, str, satBiasFix[i], 0, DataTypeFloat, float&);
	}

	for (int i = 0; i < NUMSATSOL; i++)
	{
		SNPRINTF(str, sizeof(str), "qualL[%d]", i);
		ADD_MAP(m, totalSize, str, qualL[i], 0, DataTypeUInt8, uint8_t&);
	}

    for (int i = 0; i < NUMSATSOL; i++)
    {
        SNPRINTF(str, sizeof(str), "sat[%d]", i);
        ADD_MAP(m, totalSize, str, sat[i], 0, DataTypeUInt8, uint8_t&);
    }

    for (int i = 0; i < NUMSATSOL; i++)
    {
        SNPRINTF(str, sizeof(str), "satBiasCov[%d]", i);
        ADD_MAP(m, totalSize, str, satBiasStd[i], 0, DataTypeFloat, float&);
    }

#else

	ADD_MAP(m, totalSize, "satBiasFloat[0]", satBiasFloat[0], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasFloat[1]", satBiasFloat[1], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasFloat[2]", satBiasFloat[2], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasFloat[3]", satBiasFloat[3], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasFloat[4]", satBiasFloat[4], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasFloat[5]", satBiasFloat[5], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasFloat[6]", satBiasFloat[6], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasFloat[7]", satBiasFloat[7], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasFloat[8]", satBiasFloat[8], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasFloat[9]", satBiasFloat[9], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasFloat[10]", satBiasFloat[10], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasFloat[11]", satBiasFloat[11], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasFloat[12]", satBiasFloat[12], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasFloat[13]", satBiasFloat[13], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasFloat[14]", satBiasFloat[14], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasFloat[15]", satBiasFloat[15], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasFloat[16]", satBiasFloat[16], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasFloat[17]", satBiasFloat[17], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasFloat[18]", satBiasFloat[18], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasFloat[19]", satBiasFloat[19], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasFloat[20]", satBiasFloat[20], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasFloat[21]", satBiasFloat[21], 0, DataTypeFloat, float&);

	ADD_MAP(m, totalSize, "satBiasFix[0]", satBiasFix[0], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasFix[1]", satBiasFix[1], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasFix[2]", satBiasFix[2], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasFix[3]", satBiasFix[3], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasFix[4]", satBiasFix[4], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasFix[5]", satBiasFix[5], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasFix[6]", satBiasFix[6], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasFix[7]", satBiasFix[7], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasFix[8]", satBiasFix[8], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasFix[9]", satBiasFix[9], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasFix[10]", satBiasFix[10], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasFix[11]", satBiasFix[11], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasFix[12]", satBiasFix[12], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasFix[13]", satBiasFix[13], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasFix[14]", satBiasFix[14], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasFix[15]", satBiasFix[15], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasFix[16]", satBiasFix[16], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasFix[17]", satBiasFix[17], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasFix[18]", satBiasFix[18], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasFix[19]", satBiasFix[19], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasFix[20]", satBiasFix[20], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasFix[21]", satBiasFix[21], 0, DataTypeFloat, float&);

	ADD_MAP(m, totalSize, "qualL[0]", qualL[0], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "qualL[1]", qualL[1], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "qualL[2]", qualL[2], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "qualL[3]", qualL[3], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "qualL[4]", qualL[4], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "qualL[5]", qualL[5], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "qualL[6]", qualL[6], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "qualL[7]", qualL[7], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "qualL[8]", qualL[8], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "qualL[9]", qualL[9], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "qualL[10]", qualL[10], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "qualL[11]", qualL[11], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "qualL[12]", qualL[12], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "qualL[13]", qualL[13], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "qualL[14]", qualL[14], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "qualL[15]", qualL[15], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "qualL[16]", qualL[16], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "qualL[17]", qualL[17], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "qualL[18]", qualL[18], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "qualL[19]", qualL[19], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "qualL[20]", qualL[20], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "qualL[21]", qualL[21], 0, DataTypeUInt8, uint8_t&);

	ADD_MAP(m, totalSize, "sat[0]", sat[0], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "sat[1]", sat[1], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "sat[2]", sat[2], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "sat[3]", sat[3], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "sat[4]", sat[4], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "sat[5]", sat[5], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "sat[6]", sat[6], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "sat[7]", sat[7], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "sat[8]", sat[8], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "sat[9]", sat[9], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "sat[10]", sat[10], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "sat[11]", sat[11], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "sat[12]", sat[12], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "sat[13]", sat[13], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "sat[14]", sat[14], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "sat[15]", sat[15], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "sat[16]", sat[16], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "sat[17]", sat[17], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "sat[18]", sat[18], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "sat[19]", sat[19], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "sat[20]", sat[20], 0, DataTypeUInt8, uint8_t&);
	ADD_MAP(m, totalSize, "sat[21]", sat[21], 0, DataTypeUInt8, uint8_t&);

	ADD_MAP(m, totalSize, "satBiasStd[0]", satBiasStd[0], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasStd[1]", satBiasStd[1], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasStd[2]", satBiasStd[2], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasStd[3]", satBiasStd[3], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasStd[4]", satBiasStd[4], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasStd[5]", satBiasStd[5], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasStd[6]", satBiasStd[6], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasStd[7]", satBiasStd[7], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasStd[8]", satBiasStd[8], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasStd[9]", satBiasStd[9], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasStd[10]", satBiasStd[10], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasStd[11]", satBiasStd[11], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasStd[12]", satBiasStd[12], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasStd[13]", satBiasStd[13], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasStd[14]", satBiasStd[14], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasStd[15]", satBiasStd[15], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasStd[16]", satBiasStd[16], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasStd[17]", satBiasStd[17], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasStd[18]", satBiasStd[18], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasStd[19]", satBiasStd[19], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasStd[20]", satBiasStd[20], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "satBiasStd[21]", satBiasStd[21], 0, DataTypeFloat, float&);

    ADD_MAP(m, totalSize, "satLockCnt[0]", satLockCnt[0], 0, DataTypeInt8, int8_t&);
    ADD_MAP(m, totalSize, "satLockCnt[1]", satLockCnt[1], 0, DataTypeInt8, int8_t&);
    ADD_MAP(m, totalSize, "satLockCnt[2]", satLockCnt[2], 0, DataTypeInt8, int8_t&);
    ADD_MAP(m, totalSize, "satLockCnt[3]", satLockCnt[3], 0, DataTypeInt8, int8_t&);
    ADD_MAP(m, totalSize, "satLockCnt[4]", satLockCnt[4], 0, DataTypeInt8, int8_t&);
    ADD_MAP(m, totalSize, "satLockCnt[5]", satLockCnt[5], 0, DataTypeInt8, int8_t&);
    ADD_MAP(m, totalSize, "satLockCnt[6]", satLockCnt[6], 0, DataTypeInt8, int8_t&);
    ADD_MAP(m, totalSize, "satLockCnt[7]", satLockCnt[7], 0, DataTypeInt8, int8_t&);
    ADD_MAP(m, totalSize, "satLockCnt[8]", satLockCnt[8], 0, DataTypeInt8, int8_t&);
    ADD_MAP(m, totalSize, "satLockCnt[9]", satLockCnt[9], 0, DataTypeInt8, int8_t&);
    ADD_MAP(m, totalSize, "satLockCnt[10]", satLockCnt[10], 0, DataTypeInt8, int8_t&);
    ADD_MAP(m, totalSize, "satLockCnt[11]", satLockCnt[11], 0, DataTypeInt8, int8_t&);
    ADD_MAP(m, totalSize, "satLockCnt[12]", satLockCnt[12], 0, DataTypeInt8, int8_t&);
    ADD_MAP(m, totalSize, "satLockCnt[13]", satLockCnt[13], 0, DataTypeInt8, int8_t&);
    ADD_MAP(m, totalSize, "satLockCnt[14]", satLockCnt[14], 0, DataTypeInt8, int8_t&);
    ADD_MAP(m, totalSize, "satLockCnt[15]", satLockCnt[15], 0, DataTypeInt8, int8_t&);
    ADD_MAP(m, totalSize, "satLockCnt[16]", satLockCnt[16], 0, DataTypeInt8, int8_t&);
    ADD_MAP(m, totalSize, "satLockCnt[17]", satLockCnt[17], 0, DataTypeInt8, int8_t&);
    ADD_MAP(m, totalSize, "satLockCnt[18]", satLockCnt[18], 0, DataTypeInt8, int8_t&);
    ADD_MAP(m, totalSize, "satLockCnt[19]", satLockCnt[19], 0, DataTypeInt8, int8_t&);
    ADD_MAP(m, totalSize, "satLockCnt[20]", satLockCnt[20], 0, DataTypeInt8, int8_t&);
    ADD_MAP(m, totalSize, "satLockCnt[21]", satLockCnt[21], 0, DataTypeInt8, int8_t&);

#endif

    ADD_MAP(m, totalSize, "num_biases", num_biases, 0, DataTypeUInt8, uint8_t);
}

#endif // USE_IS_INTERNAL

cISDataMappings::cISDataMappings()
{
	PopulateSizeMappings(m_lookupSize);
	PopulateDeviceInfoMappings(m_lookupInfo);
    PopulateIMUMappings(m_lookupInfo, DID_DUAL_IMU);
    PopulateIMUMappings(m_lookupInfo, DID_DUAL_IMU_RAW);
	PopulateSysParamsMappings(m_lookupInfo);
	PopulateSysSensorsMappings(m_lookupInfo);
	PopulateINS1Mappings(m_lookupInfo);
	PopulateINS2Mappings(m_lookupInfo, DID_INS_2);
	PopulateINS2Mappings(m_lookupInfo, DID_INS_3);
	PopulateINS4Mappings(m_lookupInfo);
	PopulateGpsPosMappings(m_lookupInfo, DID_GPS1_POS);
	PopulateGpsPosMappings(m_lookupInfo, DID_GPS1_UBX_POS);
	PopulateGpsPosMappings(m_lookupInfo, DID_GPS2_POS);
	PopulateGpsPosMappings(m_lookupInfo, DID_GPS1_RTK_POS);
	PopulateGpsVelMappings(m_lookupInfo, DID_GPS1_VEL);
	PopulateGpsVelMappings(m_lookupInfo, DID_GPS2_VEL);
	//PopulateGPSCNOMappings(m_lookupInfo, DID_GPS1_SAT); // too much data, we don't want to log this
	//PopulateGPSCNOMappings(m_lookupInfo, DID_GPS2_SAT); // too much data, we don't want to log this
	PopulateMagnetometerMappings(m_lookupInfo, DID_MAGNETOMETER_1);
    PopulateMagnetometerMappings(m_lookupInfo, DID_MAGNETOMETER_2);
    PopulateBarometerMappings(m_lookupInfo);
    PopulateIMUDeltaThetaVelocityMappings(m_lookupInfo);
    PopulateWheelEncoderMappings(m_lookupInfo);
    PopulateConfigMappings(m_lookupInfo);
	PopulateFlashConfigMappings(m_lookupInfo);
	PopulateRtkRelMappings(m_lookupInfo);
	PopulateRtkMiscMappings(m_lookupInfo);
	PopulateGpsRawMappings(m_lookupInfo, DID_GPS1_RAW);
	PopulateGpsRawMappings(m_lookupInfo, DID_GPS2_RAW);
	PopulateGpsRawMappings(m_lookupInfo, DID_GPS_BASE_RAW);
	PopulateStrobeInTimeMappings(m_lookupInfo);
//	PopulateRtosInfoMappings(m_lookupInfo);
    PopulateDiagMsgMappings(m_lookupInfo);
	PopulateCanConfigMappings(m_lookupInfo);

#ifdef USE_IS_INTERNAL

	PopulateSensorsADCMappings(m_lookupInfo);
	PopulateSensorsISMappings(m_lookupInfo, DID_SENSORS_IS1);
	PopulateSensorsISMappings(m_lookupInfo, DID_SENSORS_IS2);
	PopulateSensorsTCMappings(m_lookupInfo);
	PopulateSensorsCompMappings(m_lookupInfo);
	PopulateDebugArrayMappings(m_lookupInfo);
	PopulateUserPage0Mappings(m_lookupInfo);
	PopulateUserPage1Mappings(m_lookupInfo);
	PopulateInl2MagObsInfo(m_lookupInfo);
	PopulateInl2StatesMappings(m_lookupInfo);
//     PopulateRtkStateMappings(m_lookupInfo);
//     PopulateRtkResidualMappings(m_lookupInfo, DID_RTK_CODE_RESIDUAL);
//     PopulateRtkResidualMappings(m_lookupInfo, DID_RTK_PHASE_RESIDUAL);
	PopulateRtkDebugMappings(m_lookupInfo);
	PopulateRtkDebug2Mappings(m_lookupInfo);
	PopulateIMUDeltaThetaVelocityMagMappings(m_lookupInfo);
	PopulateIMUMagnetometerMappings(m_lookupInfo, DID_DUAL_IMU_RAW_MAG);
	PopulateIMUMagnetometerMappings(m_lookupInfo, DID_DUAL_IMU_MAG);

#endif

	// this mustcome last
	for (uint32_t id = 0; id < DID_COUNT; id++)
	{
		PopulateTimestampField(id, m_timestampFields, m_lookupInfo);
	}
}


cISDataMappings::~cISDataMappings()
{
}


const char* cISDataMappings::GetDataSetName(uint32_t dataId)
{
    static const char* s_dataIdNames[] =
    {
        "null",					// 0: DID_NULL
        "devInfo",				// 1: DID_DEV_INFO,
        "sysFault",				// 2: DID_SYS_FAULT
        "preintegratedImu",		// 3: DID_PREINTEGRATED_IMU
        "ins1",					// 4: DID_INS_1
        "ins2",					// 5: DID_INS_2
        "gpsPos",				// 6: DID_GPS1_POS
        "config",				// 7: DID_SYS_CMD
        "ascii_msg",			// 8: DID_ASCII_BCAST_PERIOD
        "misc",					// 9: DID_INS_MISC
        "sysParams",			// 10: DID_SYS_PARAMS
        "sysSensors",			// 11: DID_SYS_SENSORS
        "flashCfg",				// 12: DID_FLASH_CONFIG
        "gps1Pos",				// 13: DID_GPS1_UBX_POS
        "gps2Pos",				// 14: DID_GPS2_POS
        "gps1CNO",				// 15: DID_GPS1_SAT
        "gps2CNO",				// 16: DID_GPS2_SAT
        "gps1Version",			// 17: DID_GPS1_VERSION
        "gps2Version",			// 18: DID_GPS2_VERSION
        "magCal",				// 19: DID_MAG_CAL
        "diagnosticInfo",		// 20: DID_INTERNAL_DIAGNOSTIC
        "gpsRtkPosRel",			// 21: DID_GPS1_RTK_POS_REL
        "gpsRtkPosMisc",		// 22: DID_GPS1_RTK_POS_MISC
        "featureBits",			// 23: DID_FEATURE_BITS
        "sensorIS1",			// 24: DID_SENSORS_IS1
        "sensorIS2",			// 25: DID_SENSORS_IS2
        "sensorTCbias",			// 26: DID_SENSORS_TC_BIAS
        "ioServos",				// 27: DID_IO
        "sensorLSB",			// 28: DID_SENSORS_ADC
        "scomp",				// 29: DID_SCOMP
        "gps1Vel",				// 30: DID_GPS1_VEL
        "gps2Vel",				// 31: DID_GPS2_VEL
        "hdwParams",			// 32: DID_HDW_PARAMS
        "userPageNvr",			// 33: DID_NVR_MANAGE_USERPAGE
        "userPageSn",			// 34: DID_NVR_USERPAGE_SN
        "userpage0",			// 35: DID_NVR_USERPAGE_G0
        "userpage1",			// 36: DID_NVR_USERPAGE_G1
        "debugString",			// 37: DID_DEBUG_STRING
        "rtosInfo",				// 38: DID_RTOS_INFO
        "debugArray",			// 39: DID_DEBUG_ARRAY
        "sensorCal1",			// 40: DID_SENSORS_CAL1
        "sensorCal2",			// 41: DID_SENSORS_CAL2
        "sensorCalSC",			// 42: DID_CAL_SC
        "sensorCalSC1",			// 43: DID_CAL_SC1
        "sensorCalSC2",			// 44: DID_CAL_SC2
        "sensorSigma",			// 45: DID_SYS_SENSORS_SIGMA
        "sensorAdcSigma",		// 46: DID_SENSORS_ADC_SIGMA
        "insDev1",				// 47: DID_INS_DEV_1
        "inl2States",			// 48: DID_INL2_STATES
        "inl2CovarianceUD",		// 49: DID_INL2_COVARIANCE_UD
        "inl2Status",			// 50: DID_INL2_STATUS
        "inl2Misc",				// 51: DID_INL2_MISC
        "magnetometer1",		// 52: DID_MAGNETOMETER_1
        "barometer",			// 53: DID_BAROMETER
        "gps1RtkPos",			// 54: DID_GPS1_RTK_POS
        "magnetometer2",		// 55: DID_MAGNETOMETER_2
        "commLoopback",     	// 56: DID_COMMUNICATIONS_LOOPBACK
        "imuDualRaw",			// 57: DID_DUAL_IMU_RAW
        "imuDual",				// 58: DID_DUAL_IMU
        "inl2MagObs",			// 59: DID_INL2_MAG_OBS_INFO
        "gpsBaseRaw",			// 60: DID_GPS_BASE_RAW
        "gpsRtkOptions",		// 61: DID_GPS_RTK_OPT
        "userPageInternal",		// 62: DID_NVR_USERPAGE_INTERNAL
        "manufacturingInfo",	// 63: DID_MANUFACTURING_INFO
        "bit",					// 64: DID_BIT
        "ins3",					// 65: DID_INS_3
        "ins4",					// 66: DID_INS_4
        "inl2NedSigma",			// 67: DID_INL2_NED_SIGMA
        "strobeInTime",			// 68: DID_STROBE_IN_TIME
        "gps1Raw",				// 69: DID_GPS1_RAW
        "gps2Raw",				// 70: DID_GPS2_RAW
        "wheelEncoder",         // 71: DID_WHEEL_ENCODER
        "diagnosticMsg",        // 72: DID_DIAGNOSTIC_MESSAGE
        "surveyIn",             // 73: DID_SURVEY_IN
        "sensorCalSCInfo",      // 74: DID_CAL_SC_INFO
        "portMonitor",          // 75: DID_PORT_MONITOR
        "rtkState",             // 76: DID_RTK_STATE
        "rtkPhaseResidual",     // 77: DID_RTK_PHASE_RESIDUAL
        "rtkCodeResidual",      // 78: DID_RTK_CODE_RESIDUAL
        "rtkDebug",             // 79: DID_RTK_DEBUG
        "evbStatus",            // 80: DID_EVB_STATUS
        "evbFlashCfg",          // 81: DID_EVB_FLASH_CFG
        "evb2DebugArray",       // 82: DID_EVB_DEBUG_ARRAY
        "evbRtosInfo",          // 83: DID_EVB_RTOS_INFO
		"imu_mag_raw",			// 84: DID_DUAL_IMU_RAW_MAG
		"imu_mag",				// 85: DID_DUAL_IMU_MAG
		"pimu_mag",				// 86: DID_PREINTEGRATED_IMU_MAG
		"wheelConfig",          // 87: DID_WHEEL_CONFIG
		"positionMeasurement",  // 88: DID_POSITION_MEASUREMENT
		"rtkDebug2",            // 89: DID_RTK_DEBUG_2
		"canconfig",			// 90: 
		"gps2RtkCmpRel",		// 91: DID_GPS2_RTK_CMP_REL
		"gps2RtkCmpMisc",		// 92: DID_GPS2_RTK_CMP_MISC
		"unused_93",			// 93: 
		"unused_94",			// 94: 
		"unused_95"				// 95: 
	};

    STATIC_ASSERT(_ARRAY_ELEMENT_COUNT(s_dataIdNames) == DID_COUNT);

    if (dataId < DID_COUNT)
    {
        return s_dataIdNames[dataId];
    }
    return "unknown";
}


const map_name_to_info_t* cISDataMappings::GetMapInfo(uint32_t dataId)
{

#if PLATFORM_IS_EMBEDDED

	if (s_map == NULLPTR)
	{
		s_map = new cISDataMappings();
	}

#endif

	if (dataId < DID_COUNT)
	{

#if PLATFORM_IS_EMBEDDED

		return &s_map->m_lookupInfo[dataId];

#else

		return &s_map.m_lookupInfo[dataId];

#endif

	}
	return NULLPTR;
}


uint32_t cISDataMappings::GetSize(uint32_t dataId)
{

#if PLATFORM_IS_EMBEDDED

	if (s_map == NULLPTR)
	{
		s_map = new cISDataMappings();
	}

#endif

#if PLATFORM_IS_EMBEDDED

	return (dataId < DID_MAX_COUNT ? s_map->m_lookupSize[dataId] : 0);

#else

	return (dataId < DID_MAX_COUNT ? s_map.m_lookupSize[dataId] : 0);

#endif

}


bool cISDataMappings::StringToData(const char* stringBuffer, int stringLength, const p_data_hdr_t* hdr, uint8_t* dataBuffer, const data_info_t& info, int radix, bool json)
{
	const uint8_t* ptr;
	if (!CanGetFieldData(info, hdr, dataBuffer, ptr))
	{
		return false;
	}

	switch (info.dataType)
	{
	case DataTypeInt8:
		*(int8_t*)ptr = (int8_t)strtol(stringBuffer, NULL, radix);
		break;

	case DataTypeUInt8:
		*(uint8_t*)ptr = (uint8_t)strtoul(stringBuffer, NULL, radix);
		break;

	case DataTypeInt16:
		*(int16_t*)ptr = (int16_t)strtol(stringBuffer, NULL, radix);
		break;

	case DataTypeUInt16:
		*(uint16_t*)ptr = (uint16_t)strtoul(stringBuffer, NULL, radix);
		break;

	case DataTypeInt32:
		*(int32_t*)ptr = (int32_t)strtol(stringBuffer, NULL, radix);
		break;

	case DataTypeUInt32:
		*(uint32_t*)ptr = (uint32_t)strtoul(stringBuffer, NULL, radix);
		break;

	case DataTypeInt64:
		*(int64_t*)ptr = (int64_t)strtoll(stringBuffer, NULL, radix);
		break;

	case DataTypeUInt64:
		*(uint64_t*)ptr = (uint64_t)strtoull(stringBuffer, NULL, radix);
		break;

	case DataTypeFloat:
		*(float*)ptr = strtof(stringBuffer, NULL);
		break;

	case DataTypeDouble:
		*(double*)ptr = strtod(stringBuffer, NULL);
		break;

	case DataTypeString:
	{
		string s2(stringBuffer);
		if (json)
		{
			char c;
			bool escaped = false;
			for (size_t i = 0; i < s2.size(); i++)
			{
				c = s2[i];
				if (c == '\\')
				{
					if (!escaped)
					{
						escaped = true;
						s2.erase(i);
						continue;
					}
				}
				escaped = false;
			}
			s2 = stringBuffer;
		}
		else
		{
			s2.erase(std::remove(s2.begin(), s2.end(), '"'), s2.end());
		}
		// ensure string fits with null terminator
		s2.resize(info.dataSize - 1);
		memcpy((void*)ptr, s2.data(), s2.length());
		memset((uint8_t*)ptr + s2.length(), 0, info.dataSize - s2.length());
	} break;

	case DataTypeBinary:
	{
		// convert hex data back to binary
		size_t len = _MIN(1020, stringLength);
		len -= (len % 2);
		uint8_t* ptr2 = (uint8_t*)ptr;
		for (size_t i = 0; i != len; i += 2)
		{
			*ptr2 = (getHexValue(stringBuffer[i + 1]) << 4) | getHexValue(stringBuffer[i + 2]);
		}
	} break;

	default:
		return false;
	}

	return true;
}


bool cISDataMappings::DataToString(const data_info_t& info, const p_data_hdr_t* hdr, const uint8_t* dataBuffer, data_mapping_string_t stringBuffer, bool json)
{
	const uint8_t* ptr;
	if (!CanGetFieldData(info, hdr, dataBuffer, ptr))
	{
		// pick a default string
		if (info.dataType == DataTypeString)
		{
			stringBuffer[0] = '"';
			stringBuffer[1] = '"';
			stringBuffer[2] = '\0';
		}
		else if (info.dataType == DataTypeBinary)
		{
			if (json)
			{
				stringBuffer[0] = '"';
				stringBuffer[1] = '"';
				stringBuffer[2] = '\0';
			}
			else
			{
				stringBuffer[0] = '\0';
			}
		}
		else
		{
			stringBuffer[0] = '0';
			stringBuffer[1] = '\0';
		}
		return false;
	}

	switch (info.dataType)
	{
	case DataTypeInt8:
		SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%d", (int)*(int8_t*)ptr);
		break;

	case DataTypeUInt8:
		SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%u", (unsigned int)*(uint8_t*)ptr);
		break;

	case DataTypeInt16:
		SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%d", (int)*(int16_t*)ptr);
		break;

	case DataTypeUInt16:
		SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%u", (unsigned int)*(uint16_t*)ptr);
		break;

	case DataTypeInt32:
		SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%d", (int)*(int32_t*)ptr);
		break;

	case DataTypeUInt32:
		SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%u", (unsigned int)*(uint32_t*)ptr);
		break;

	case DataTypeInt64:
		SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%lld", (long long)*(int64_t*)ptr);
		break;

	case DataTypeUInt64:
		SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%llu", (unsigned long long)*(uint64_t*)ptr);
		break;

	case DataTypeFloat:
		SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%.9g", *(float*)ptr);
		break;

	case DataTypeDouble:
		SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%.17g", *(double*)ptr);
		break;

	case DataTypeString:
	{
		stringBuffer[0] = '"';
		int tempIndex = 1;
		char* bufPtr2 = (char*)(dataBuffer + info.dataOffset);
		char* bufPtrEnd = bufPtr2 + _MIN(IS_DATA_MAPPING_MAX_STRING_LENGTH, info.dataSize) - 3;
		for (; bufPtr2 < bufPtrEnd && *bufPtr2 != '\0'; bufPtr2++)
		{
			if (json)
			{
				if (IS_JSON_ESCAPE_CHAR(*bufPtr2))
				{
					if (bufPtr2 < bufPtrEnd - 1)
					{
						stringBuffer[tempIndex++] = '\\';
					}
					else
					{
						break;
					}
				}
			}
			stringBuffer[tempIndex++] = *bufPtr2;
		}
		stringBuffer[tempIndex++] = '"';
		stringBuffer[tempIndex] = '\0';
	} break;

	case DataTypeBinary:
	{
		size_t hexIndex = 1;
		if (json)
		{
			stringBuffer[0] = '"';
		}
		// convert to hex
		const unsigned char* hexTable = getHexLookupTable();
		for (size_t i = 0; i < info.dataSize; i++)
		{
			stringBuffer[hexIndex++] = hexTable[0x0F & (dataBuffer[i] >> 4)];
			stringBuffer[hexIndex++] = hexTable[0x0F & dataBuffer[i]];
		}
		if (json)
		{
			stringBuffer[hexIndex++] = '"';
		}
		stringBuffer[hexIndex] = '\0';
	} break;

	default:
		stringBuffer[0] = '\0';
		return false;
	}
	return true;
}

double cISDataMappings::GetTimestamp(const p_data_hdr_t* hdr, const uint8_t* buf)
{
    if (hdr == NULL || buf == NULL || hdr->id == 0 || hdr->id >= DID_COUNT || hdr->size == 0)
	{
		return 0.0;
	}
	
    // raw data types with observation use a custom timestamp function
    if (hdr->id == DID_GPS1_RAW || hdr->id == DID_GPS2_RAW || hdr->id == DID_GPS_BASE_RAW)
	{
		gps_raw_t* raw = (gps_raw_t*)buf;
		if (raw->dataType == eRawDataType::raw_data_type_observation && raw->obsCount>0)
		{
			const obsd_t& obs = raw->data.obs[0];
			return obs.time.sec + (double)obs.time.time;
		}
		return 0.0;
	}

#if PLATFORM_IS_EMBEDDED

	if (s_map == NULLPTR)
	{
		s_map = new cISDataMappings();
	}

#endif

	const data_info_t* timeStampField =
	
#if PLATFORM_IS_EMBEDDED

		s_map->m_timestampFields[hdr->id];

#else

		s_map.m_timestampFields[hdr->id];

#endif

	if (timeStampField != NULLPTR)
	{
		const uint8_t* ptr;
		if (CanGetFieldData(*timeStampField, hdr, (uint8_t*)buf, ptr))
		{
			if (timeStampField->dataType == DataTypeDouble)
			{
				// field is seconds, use as is
				return *(double*)ptr;
			}
			else if (timeStampField->dataType == DataTypeUInt32)
			{
				// field is milliseconds, convert to seconds
				return 0.001 * (*(uint32_t*)ptr);
			}
		}
	}
	return 0.0;
}

bool cISDataMappings::CanGetFieldData(const data_info_t& info, const p_data_hdr_t* hdr, const uint8_t* buf, const uint8_t*& ptr)
{
	if (buf == NULL)
	{
		return false;
	}
	else if (hdr == NULL)
	{
		// assume buf is large enough for the full data structure
		ptr = buf + info.dataOffset;
		return true;
	}
	int32_t fullSize = (hdr->size == 0 ? GetSize(hdr->id) : hdr->size);
	int32_t offset = (int32_t)info.dataOffset - (int32_t)hdr->offset;
	if (offset >= 0 && offset <= fullSize - (int32_t)info.dataSize)
	{
		ptr = (buf + offset);
		return true;
	}
	ptr = NULL;
	return false;
}
