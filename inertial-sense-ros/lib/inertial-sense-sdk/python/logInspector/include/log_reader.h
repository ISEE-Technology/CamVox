#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include "InertialSense.h"
#include "ISLogger.h"

//#include "Eigen/Core"
//#include "Eigen/St    dVector"

#ifdef WIN32
#pragma comment(lib, "SHELL32.LIB")
#endif

namespace py = pybind11;
using namespace std;

typedef struct
{
  std::vector<std::vector<obsd_t>> obs;
  std::vector<eph_t> eph;
  std::vector<geph_t> gloEph;
  std::vector<sbsmsg_t> sbas;
  std::vector<sta_t> sta;
  std::vector<ion_model_utc_alm_t> ion;
} gps_raw_wrapper_t;

struct DeviceLog
{
    vector<dev_info_t> devInfo;
    vector<system_fault_t> sysFault;
    vector<preintegrated_imu_t> preintegratedImu;
    vector<ins_1_t> ins1;
    vector<ins_2_t> ins2;
    vector<gps_pos_t> gps1UbxPos;
    vector<system_command_t> sysCmd;
//    vector<ascii_msgs_t> asciiBcastPeriod;
//    vector<rmc_t> rmc;
    vector<sys_params_t> sysParams;
    vector<sys_sensors_t> sysSensors;
    vector<nvm_flash_cfg_t> flashConfig;
    vector<gps_pos_t> gps1Pos;
    vector<gps_pos_t> gps2Pos;
    vector<gps_sat_t> gps1Sat;
    vector<gps_sat_t> gps2Sat;
    vector<gps_version_t> gps1Version;
    vector<gps_version_t> gps2Version;
    vector<mag_cal_t> magCal;
    vector<internal_diagnostic_t> internalDiagnostic;
    vector<gps_rtk_rel_t> gps1RtkPosRel;
    vector<gps_rtk_rel_t> gps1RtkCmpRel;
    vector<gps_rtk_misc_t> gps1RtkPosMisc;
    vector<gps_rtk_misc_t> gps1RtkCmpMisc;
    // vector<feature_bits_t> featureBits;
    // vector<sensors_w_temp_t> sensorsIs1;
    // vector<sensors_w_temp_t> sensorsIs2;
    // vector<sensors_t> sensorsTcBias;
    vector<io_t> io;
    // vector<sys_sensors_adc_t> sensorsAdc;
    // vector<sensor_compensation_t> scomp;
    vector<gps_vel_t> gps1Vel;
    vector<gps_vel_t> gps2Vel;
    // vector<hdw_params_t> hdwParams;
    // vector<nvr_manage_t> nvrManageUserpage;
    // vector<nvm_group_sn_t> nvrUserpageSn;
    // vector<nvm_group_0_t> nvrUserpageG0;
    // vector<nvm_group_1_t> nvrUserpageG1;
    // vector<rtos_info_t> rtosInfo;
    vector<debug_string_t> debugString;
    vector<debug_array_t> debugArray;
    vector<sensors_mpu_w_temp_t> sensorsCal1;
    vector<sensors_mpu_w_temp_t> sensorsCal2;
    // vector<sensor_cal_t> calSc;
    // vector<sensor_cal_mpu_t> calSc1;
    // vector<sensor_cal_mpu_t> calSc2;
    vector<sys_sensors_t> sysSensorsSigma;
    vector<sys_sensors_adc_t> sensorsAdcSigma;
    // vector<ins_dev_1_t> insDev1;
    vector<inl2_states_t> inl2States;
    vector<inl2_status_t> inl2Status;
    // vector<inl2_misc_t> inl2Misc;
    vector<magnetometer_t> magnetometer;
    vector<barometer_t> barometer;
    vector<gps_pos_t> gps1RtkPos;
    vector<dual_imu_t> dualImuRaw;
    vector<dual_imu_t> dualImu;
    vector<inl2_mag_obs_info_t> inl2MagObsInfo;
    vector<gps_raw_wrapper_t> gpsBaseRaw {1};
//    vector<gps_rtk_opt_t> gpsRtkOpt;
    vector<manufacturing_info_t> manufacturingInfo;
    vector<bit_t> bit;
    vector<ins_3_t> ins3;
    vector<ins_4_t> ins4;
    vector<inl2_ned_sigma_t> inl2NedSigma;
    vector<strobe_in_time_t> strobeInTime;
    vector<gps_raw_wrapper_t> gps1Raw {1};
    vector<gps_raw_wrapper_t> gps2Raw {1};
    vector<wheel_encoder_t> wheelEncoder;
    // vector<wheel_encoder_config_t> wheelEncoderConfig;
    vector<diag_msg_t> diagnosticMessage;
    vector<survey_in_t> surveyIn;
//    vector<evb2_t> evb2;
    // vector<rtk_state_t> rtkState;
    vector<rtk_residual_t> rtkCodeResidual;
    vector<rtk_residual_t> rtkPhaseResidual;
    vector<rtk_debug_t> rtkDebug;
    // vector<rtk_debug_2_t> rtkDebug2;
//    vector<port_monitor_t> portMonitor;
};

template <typename T>
struct DataLog
{
    int id;
    std::vector<T> data;

    DataLog(int _id)
    {
        _id = id;
    }
};


class LogReader
{
public:
    LogReader();
    ~LogReader();
    bool init(py::object python_class, std::string log_directory, pybind11::list serials);
    bool load();
    void exitHack();
    
    template <typename T>
    void forward_message(eDataIDs did, std::vector<T>& vec, int id);

    template <typename T>
    void log_message(int did, uint8_t* msg, std::vector<T>& vec)
    {
        vec.push_back(*(T*)msg);
    }

private:
    void organizeData(int device_id);
    void forwardData(int id);

    cISLogger logger_;
    DeviceLog* dev_log_ = nullptr;
};
