#include "log_reader.h"

using namespace std;

static py::object g_python_parent;  // Including this inside LogReader class causes problems w/ garbage collection.

LogReader::LogReader()
{
    dev_log_ = nullptr;
}

LogReader::~LogReader()
{
    if (dev_log_ != nullptr)
    {
        delete dev_log_;
        dev_log_ = nullptr;
    }
}

template <>
void LogReader::log_message(int did, uint8_t* msg, std::vector<gps_raw_wrapper_t>& vec)
{
  gps_raw_t* raw_msg = (gps_raw_t*)msg;
  switch (raw_msg->dataType)
  {
  case raw_data_type_observation:
  {
    std::vector<obsd_t> obs{raw_msg->obsCount};
    for (int i = 0; i < raw_msg->obsCount; i++)
      obs.push_back(raw_msg->data.obs[i]);
    vec[0].obs.push_back(obs);
    break;
  }
  case raw_data_type_ephemeris:
    vec[0].eph.push_back(raw_msg->data.eph);
    break;
  case raw_data_type_glonass_ephemeris:
    vec[0].gloEph.push_back(raw_msg->data.gloEph);
    break;
  case raw_data_type_sbas:
    vec[0].sbas.push_back(raw_msg->data.sbas);
    break;
  case raw_data_type_base_station_antenna_position:
    vec[0].sta.push_back(raw_msg->data.sta);
    break;
  case raw_data_type_ionosphere_model_utc_alm:
    vec[0].ion.push_back(raw_msg->data.ion);
    break;
  default:
    break;
  }
}

template <typename T>
void LogReader::forward_message(eDataIDs did, std::vector<T>& vec, int id)
{
    g_python_parent.attr("did_callback")(did, py::array_t<T>(std::vector<ptrdiff_t>{(py::ssize_t)vec.size()}, vec.data()), id);
}

template <>
void LogReader::forward_message(eDataIDs did, std::vector<gps_raw_wrapper_t>& vec, int id)
{
    for (int i = 0; i < (int)vec[0].obs.size(); i++)
    {
        g_python_parent.attr("gps_raw_data_callback")(did, py::array_t<obsd_t>(std::vector<ptrdiff_t>{(py::ssize_t)vec[0].obs[i].size()}, vec[0].obs[i].data()), id, (int)raw_data_type_observation);
    }
    g_python_parent.attr("gps_raw_data_callback")(did, py::array_t<eph_t>(std::vector<ptrdiff_t>{(py::ssize_t)vec[0].eph.size()}, vec[0].eph.data()), id, (int)raw_data_type_ephemeris);
    g_python_parent.attr("gps_raw_data_callback")(did, py::array_t<geph_t>(std::vector<ptrdiff_t>{(py::ssize_t)vec[0].gloEph.size()}, vec[0].gloEph.data()), id, (int)raw_data_type_glonass_ephemeris);
    g_python_parent.attr("gps_raw_data_callback")(did, py::array_t<sbsmsg_t>(std::vector<ptrdiff_t>{(py::ssize_t)vec[0].sbas.size()}, vec[0].sbas.data()), id, (int)raw_data_type_sbas);
    g_python_parent.attr("gps_raw_data_callback")(did, py::array_t<ion_model_utc_alm_t>(std::vector<ptrdiff_t>{(py::ssize_t)vec[0].ion.size()}, vec[0].ion.data()), id, (int)raw_data_type_ionosphere_model_utc_alm);
    g_python_parent.attr("gps_raw_data_callback")(did, py::array_t<sta_t>(std::vector<ptrdiff_t>{(py::ssize_t)vec[0].sta.size()}, vec[0].sta.data()), id, (int)raw_data_type_base_station_antenna_position);
}


bool LogReader::init(py::object python_class, std::string log_directory, py::list serials)
{
    vector<string> stl_serials = serials.cast<vector<string>>();
    cout << "loading DAT file from " << log_directory << endl;
    cout << "reading serial numbers ";
    for (int i = 0; i < (int)stl_serials.size(); i++)
        cout << stl_serials[i] << "\n";
    cout << endl;

    // first try DAT files, if that doesn't work, then try SDAT files
    if (!logger_.LoadFromDirectory(log_directory, cISLogger::LOGTYPE_DAT, stl_serials))
    {
        cout << "unable to find DAT files, trying SDATS";
        if (!logger_.LoadFromDirectory(log_directory, cISLogger::LOGTYPE_SDAT, stl_serials))
            cout << "Unable to load files" << endl;
    }

    cout << "found " << logger_.GetDeviceCount() << " devices\n";
    for (int i = 0; i < (int)logger_.GetDeviceCount(); i++)
    {
        cout << logger_.GetDeviceInfo(i)->serialNumber << "\t";
    }
    cout << endl;

    // python_parent_ = python_class;
    g_python_parent = python_class;
    return true;
}

void LogReader::organizeData(int device_id)
{
    p_data_t* data = NULL;
    while ((data = logger_.ReadData(device_id)))
    {
        // if (data->hdr.id == DID_DEV_INFO)
        //     volatile int debug = 0;

        if (data->hdr.size == 0)
            continue;

        switch (data->hdr.id)
        {

        // This is a helper macro, simply define the DID you want to forward,
        // as well as the datatype of that DID.  So long as the data type
        // has been defined in the PYBIND11_NUMPY_DTYPE macros below,
        // then this will work.  It uses templates to abstract a lot
        // of the tedium of this type of work
        #define HANDLE_MSG(DID, vec) \
        case DID: \
            log_message(data->hdr.id, data->buf, vec); \
            break;

        HANDLE_MSG( DID_DEV_INFO, dev_log_->devInfo );
        HANDLE_MSG( DID_SYS_FAULT, dev_log_->sysFault );
        HANDLE_MSG( DID_PREINTEGRATED_IMU, dev_log_->preintegratedImu );
        HANDLE_MSG( DID_INS_1, dev_log_->ins1 );
        HANDLE_MSG( DID_INS_2, dev_log_->ins2 );
        HANDLE_MSG( DID_GPS1_UBX_POS, dev_log_->gps1UbxPos );
        HANDLE_MSG( DID_SYS_CMD, dev_log_->sysCmd );
        // HANDLE_MSG( DID_ASCII_BCAST_PERIOD, dev_log_->asciiBcastPeriod );
        // HANDLE_MSG( DID_RMC, dev_log_->rmc );
        HANDLE_MSG( DID_SYS_PARAMS, dev_log_->sysParams );
        HANDLE_MSG( DID_SYS_SENSORS, dev_log_->sysSensors );
        HANDLE_MSG( DID_FLASH_CONFIG, dev_log_->flashConfig );
        HANDLE_MSG( DID_GPS1_POS, dev_log_->gps1Pos );
        HANDLE_MSG( DID_GPS2_POS, dev_log_->gps2Pos );
        HANDLE_MSG( DID_GPS1_SAT, dev_log_->gps1Sat );
        HANDLE_MSG( DID_GPS2_SAT, dev_log_->gps2Sat );
        HANDLE_MSG( DID_GPS1_VERSION, dev_log_->gps1Version );
        HANDLE_MSG( DID_GPS2_VERSION, dev_log_->gps2Version );
        HANDLE_MSG( DID_MAG_CAL, dev_log_->magCal );
        HANDLE_MSG( DID_INTERNAL_DIAGNOSTIC, dev_log_->internalDiagnostic );
        HANDLE_MSG( DID_GPS1_RTK_POS_REL, dev_log_->gps1RtkPosRel );
        HANDLE_MSG( DID_GPS1_RTK_POS_MISC, dev_log_->gps1RtkPosMisc );
        HANDLE_MSG( DID_GPS2_RTK_CMP_REL, dev_log_->gps1RtkCmpRel );
        HANDLE_MSG( DID_GPS2_RTK_CMP_MISC, dev_log_->gps1RtkCmpMisc );
        // HANDLE_MSG( DID_FEATURE_BITS, dev_log_->featureBits );
        // HANDLE_MSG( DID_SENSORS_IS1, dev_log_->sensorsIs1 );
        // HANDLE_MSG( DID_SENSORS_IS2, dev_log_->sensorsIs2 );
        // HANDLE_MSG( DID_SENSORS_TC_BIAS, dev_log_->sensorsTcBias );
        HANDLE_MSG( DID_IO, dev_log_->io );
        // HANDLE_MSG( DID_SENSORS_ADC, dev_log_->sensorsAdc );
        // HANDLE_MSG( DID_SCOMP, dev_log_->scomp );
        HANDLE_MSG( DID_GPS1_VEL, dev_log_->gps1Vel );
        HANDLE_MSG( DID_GPS2_VEL, dev_log_->gps2Vel );
        // HANDLE_MSG( DID_HDW_PARAMS, dev_log_->hdwParams );
        // HANDLE_MSG( DID_NVR_MANAGE_USERPAGE, dev_log_->nvrManageUserpage );
        // HANDLE_MSG( DID_NVR_USERPAGE_SN, dev_log_->nvrUserpageSn );
        // HANDLE_MSG( DID_NVR_USERPAGE_G0, dev_log_->nvrUserpageG0 );
        // HANDLE_MSG( DID_NVR_USERPAGE_G1, dev_log_->nvrUserpageG1 );
        // HANDLE_MSG( DID_RTOS_INFO, dev_log_->rtosInfo );
        HANDLE_MSG( DID_DEBUG_STRING, dev_log_->debugString );
        HANDLE_MSG( DID_DEBUG_ARRAY, dev_log_->debugArray );
        HANDLE_MSG( DID_SENSORS_CAL1, dev_log_->sensorsCal1 );
        HANDLE_MSG( DID_SENSORS_CAL2, dev_log_->sensorsCal2 );
        // HANDLE_MSG( DID_CAL_SC, dev_log_->calSc );
        // HANDLE_MSG( DID_CAL_SC1, dev_log_->calSc1 );
        // HANDLE_MSG( DID_CAL_SC2, dev_log_->calSc2 );
        HANDLE_MSG( DID_SYS_SENSORS_SIGMA, dev_log_->sysSensorsSigma );
        HANDLE_MSG( DID_SENSORS_ADC_SIGMA, dev_log_->sensorsAdcSigma );
        // HANDLE_MSG( DID_INS_DEV_1, dev_log_->insDev1 );
        HANDLE_MSG( DID_INL2_STATES, dev_log_->inl2States );
        HANDLE_MSG( DID_INL2_STATUS, dev_log_->inl2Status );
        // HANDLE_MSG( DID_INL2_MISC, dev_log_->inl2Misc );
        HANDLE_MSG( DID_MAGNETOMETER_1, dev_log_->magnetometer );
        HANDLE_MSG( DID_MAGNETOMETER_2, dev_log_->magnetometer );
        HANDLE_MSG( DID_BAROMETER, dev_log_->barometer );
        HANDLE_MSG( DID_GPS1_RTK_POS, dev_log_->gps1RtkPos );
        HANDLE_MSG( DID_DUAL_IMU_RAW, dev_log_->dualImuRaw );
        HANDLE_MSG( DID_DUAL_IMU, dev_log_->dualImu );
        HANDLE_MSG( DID_INL2_MAG_OBS_INFO, dev_log_->inl2MagObsInfo );
        HANDLE_MSG( DID_GPS_BASE_RAW, dev_log_->gpsBaseRaw );
        // HANDLE_MSG( DID_GPS_RTK_OPT, dev_log_->gpsRtkOpt );
        HANDLE_MSG( DID_MANUFACTURING_INFO, dev_log_->manufacturingInfo );
        HANDLE_MSG( DID_BIT, dev_log_->bit );
        HANDLE_MSG( DID_INS_3, dev_log_->ins3 );
        HANDLE_MSG( DID_INS_4, dev_log_->ins4 );
        HANDLE_MSG( DID_INL2_NED_SIGMA, dev_log_->inl2NedSigma );
        HANDLE_MSG( DID_STROBE_IN_TIME, dev_log_->strobeInTime );
        HANDLE_MSG( DID_GPS1_RAW, dev_log_->gps1Raw );
        HANDLE_MSG( DID_GPS2_RAW, dev_log_->gps2Raw );
        HANDLE_MSG( DID_WHEEL_ENCODER, dev_log_->wheelEncoder );
        // HANDLE_MSG( DID_WHEEL_ENCODER_CONFIG, dev_log_->wheelEncoderConfig );
        HANDLE_MSG( DID_DIAGNOSTIC_MESSAGE, dev_log_->diagnosticMessage );
        HANDLE_MSG( DID_SURVEY_IN, dev_log_->surveyIn );
        // HANDLE_MSG( DID_EVB2, dev_log_->evb2 );
        // HANDLE_MSG( DID_PORT_MONITOR, dev_log_->portMonitor );
        // HANDLE_MSG( DID_RTK_STATE, dev_log_->rtkState);
        HANDLE_MSG( DID_RTK_CODE_RESIDUAL, dev_log_->rtkCodeResidual);
        HANDLE_MSG( DID_RTK_PHASE_RESIDUAL, dev_log_->rtkPhaseResidual);
        HANDLE_MSG( DID_RTK_DEBUG, dev_log_->rtkDebug);
        // HANDLE_MSG( DID_RTK_DEBUG_2, dev_log_->rtkDebug2);

        default:
            //            printf("Unhandled IS message DID: %d\n", message_type);
            break;
        }
    }
}

void LogReader::forwardData(int id)
{
    forward_message( DID_DEV_INFO, dev_log_->devInfo , id);
    forward_message( DID_SYS_FAULT, dev_log_->sysFault, id );
    forward_message( DID_PREINTEGRATED_IMU, dev_log_->preintegratedImu, id );
    forward_message( DID_INS_1, dev_log_->ins1, id );
    forward_message( DID_INS_2, dev_log_->ins2, id );
    forward_message( DID_GPS1_UBX_POS, dev_log_->gps1UbxPos, id );
    forward_message( DID_SYS_CMD, dev_log_->sysCmd, id );
    // forward_message( DID_ASCII_BCAST_PERIOD, dev_log_->asciiBcastPeriod, id );
    // forward_message( DID_RMC, dev_log_->rmc, id );
    forward_message( DID_SYS_PARAMS, dev_log_->sysParams, id );
    forward_message( DID_SYS_SENSORS, dev_log_->sysSensors, id );
    forward_message( DID_FLASH_CONFIG, dev_log_->flashConfig, id );
    forward_message( DID_GPS1_POS, dev_log_->gps1Pos, id );
    forward_message( DID_GPS2_POS, dev_log_->gps2Pos, id );
    forward_message( DID_GPS1_SAT, dev_log_->gps1Sat, id );
    forward_message( DID_GPS2_SAT, dev_log_->gps2Sat, id );
    forward_message( DID_GPS1_VERSION, dev_log_->gps1Version, id );
    forward_message( DID_GPS2_VERSION, dev_log_->gps2Version, id );
    forward_message( DID_MAG_CAL, dev_log_->magCal, id );
    forward_message( DID_INTERNAL_DIAGNOSTIC, dev_log_->internalDiagnostic, id );
    forward_message( DID_GPS1_RTK_POS_REL, dev_log_->gps1RtkPosRel, id );
    forward_message( DID_GPS1_RTK_POS_MISC, dev_log_->gps1RtkPosMisc, id );
    forward_message( DID_GPS2_RTK_CMP_REL, dev_log_->gps1RtkCmpRel, id );
    forward_message( DID_GPS2_RTK_CMP_MISC, dev_log_->gps1RtkCmpMisc, id );
    // forward_message( DID_FEATURE_BITS, dev_log_->featureBits, id );
    // forward_message( DID_SENSORS_IS1, dev_log_->sensorsIs1, id );
    // forward_message( DID_SENSORS_IS2, dev_log_->sensorsIs2, id );
    // forward_message( DID_SENSORS_TC_BIAS, dev_log_->sensorsTcBias, id );
    forward_message( DID_IO, dev_log_->io, id );
    // forward_message( DID_SENSORS_ADC, dev_log_->sensorsAdc, id );
    // forward_message( DID_SCOMP, dev_log_->scomp, id );
    forward_message( DID_GPS1_VEL, dev_log_->gps1Vel, id );
    forward_message( DID_GPS2_VEL, dev_log_->gps2Vel, id );
    // forward_message( DID_HDW_PARAMS, dev_log_->hdwParams, id );
    // forward_message( DID_NVR_MANAGE_USERPAGE, dev_log_->nvrManageUserpage, id );
    // forward_message( DID_NVR_USERPAGE_SN, dev_log_->nvrUserpageSn, id );
    // forward_message( DID_NVR_USERPAGE_G0, dev_log_->nvrUserpageG0, id );
    // forward_message( DID_NVR_USERPAGE_G1, dev_log_->nvrUserpageG1, id );
    // forward_message( DID_RTOS_INFO, dev_log_->rtosInfo, id );
    forward_message( DID_DEBUG_STRING, dev_log_->debugString, id );
    forward_message( DID_DEBUG_ARRAY, dev_log_->debugArray, id );
    forward_message( DID_SENSORS_CAL1, dev_log_->sensorsCal1, id );
    forward_message( DID_SENSORS_CAL2, dev_log_->sensorsCal2, id );
    // forward_message( DID_CAL_SC, dev_log_->calSc, id );
    // forward_message( DID_CAL_SC1, dev_log_->calSc1, id );
    // forward_message( DID_CAL_SC2, dev_log_->calSc2, id );
    forward_message( DID_SYS_SENSORS_SIGMA, dev_log_->sysSensorsSigma, id );
    forward_message( DID_SENSORS_ADC_SIGMA, dev_log_->sensorsAdcSigma, id );
    // forward_message( DID_INS_DEV_1, dev_log_->insDev1, id );
    forward_message( DID_INL2_STATES, dev_log_->inl2States, id );
    forward_message( DID_INL2_STATUS, dev_log_->inl2Status, id );
    // forward_message( DID_INL2_MISC, dev_log_->inl2Misc, id );
    forward_message( DID_MAGNETOMETER_1, dev_log_->magnetometer, id );
    forward_message( DID_MAGNETOMETER_2, dev_log_->magnetometer, id );
    forward_message( DID_BAROMETER, dev_log_->barometer, id );
    forward_message( DID_GPS1_RTK_POS, dev_log_->gps1RtkPos, id );
    forward_message( DID_DUAL_IMU_RAW, dev_log_->dualImuRaw, id );
    forward_message( DID_DUAL_IMU, dev_log_->dualImu, id );
    forward_message( DID_INL2_MAG_OBS_INFO, dev_log_->inl2MagObsInfo, id );
    forward_message( DID_GPS_BASE_RAW, dev_log_->gpsBaseRaw, id );
    // forward_message( DID_GPS_RTK_OPT, dev_log_->gpsRtkOpt, id );
    forward_message( DID_MANUFACTURING_INFO, dev_log_->manufacturingInfo, id );
    forward_message( DID_BIT, dev_log_->bit, id );
    forward_message( DID_INS_3, dev_log_->ins3, id );
    forward_message( DID_INS_4, dev_log_->ins4, id );
    forward_message( DID_INL2_NED_SIGMA, dev_log_->inl2NedSigma, id );
    forward_message( DID_STROBE_IN_TIME, dev_log_->strobeInTime, id );
    forward_message( DID_GPS1_RAW, dev_log_->gps1Raw, id );
    forward_message( DID_GPS2_RAW, dev_log_->gps2Raw, id );
    forward_message( DID_WHEEL_ENCODER, dev_log_->wheelEncoder, id );
    // forward_message( DID_WHEEL_ENCODER_CONFIG, dev_log_->wheelEncoderConfig, id );
    forward_message( DID_DIAGNOSTIC_MESSAGE, dev_log_->diagnosticMessage, id );
    forward_message( DID_SURVEY_IN, dev_log_->surveyIn, id );
    // forward_message( DID_EVB2, dev_log_->evb2, id );
    // forward_message( DID_PORT_MONITOR, dev_log_->portMonitor, id );

    // forward_message( DID_RTK_STATE, dev_log_->rtkState, id);
    forward_message( DID_RTK_CODE_RESIDUAL, dev_log_->rtkCodeResidual, id);
    forward_message( DID_RTK_PHASE_RESIDUAL, dev_log_->rtkPhaseResidual, id);
    forward_message( DID_RTK_DEBUG, dev_log_->rtkDebug, id);
    // forward_message( DID_RTK_DEBUG_2, dev_log_->rtkDebug2, id);
}

bool LogReader::load()
{
    for (int i = 0; i < (int)logger_.GetDeviceCount(); i++)
    {
        if (dev_log_ != nullptr)
        {
            delete dev_log_;
        }
        dev_log_ = new DeviceLog();

        organizeData(i);
        forwardData(i);
    }

    return true;
}

void LogReader::exitHack()
{
    // Nasty hack
    exit(0);
}

// Look at the pybind documentation to understand what is going on here.
// Don't change anything unless you know what you are doing
PYBIND11_MODULE(log_reader, m) {
    // Declare a module (the .so file compiled by CMake needs to have the same
    // name as defined here.  Don't change the name without changing the related
    // lines in the CmakeLists.txt file.  We can import this module in python
    // with "import log_reader" so long as the log_reader.so file is in the python path
    m.doc() = "log_reader";

    // Bind the Interface Class
    py::class_<LogReader>(m, "LogReader") // The object will be named IS_Comm in python
            .def(py::init<>()) // constructor
            .def("init", &LogReader::init)
            .def("load", &LogReader::load)
            .def("exitHack", &LogReader::exitHack);

#include "pybindMacros.h"
}
