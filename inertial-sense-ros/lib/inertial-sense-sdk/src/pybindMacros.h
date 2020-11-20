#include "data_sets.h"
// #include "IS_internal.h"

// support types
PYBIND11_NUMPY_DTYPE(gps_sat_sv_t, gnssId, svId, cno, elev, azim, prRes, flags); 
PYBIND11_NUMPY_DTYPE(sensors_mpu_w_temp_t, pqr, acc, mag, temp);
PYBIND11_NUMPY_DTYPE(rtos_task_t, name, priority, stackUnused, periodMs, runTimeUs, maxRunTimeUs, averageRunTimeUs, gapCount, cpuUsage, handle);
PYBIND11_NUMPY_DTYPE(imus_t, pqr, acc);
// PYBIND11_NUMPY_DTYPE(sensors_mpu_t, pqr, acc, mag);
PYBIND11_NUMPY_DTYPE(wheel_encoder_t, timeOfWeek, status, theta_l, theta_r, omega_l, omega_r, wrap_count_l, wrap_count_r);
PYBIND11_NUMPY_DTYPE(wheel_config_t, bits, e_i2l, t_i2l, distance, diameter);

// Public Typcs
PYBIND11_NUMPY_DTYPE(dev_info_t, reserved, serialNumber, hardwareVer, firmwareVer, buildNumber, protocolVer, repoRevision, manufacturer, buildDate, buildTime, addInfo);
PYBIND11_NUMPY_DTYPE(system_fault_t, status, g1Task, g2FileNum, g3LineNum, g4, g5Lr, pc, psr);
PYBIND11_NUMPY_DTYPE(preintegrated_imu_t, time, theta1, theta2, vel1, vel2, dt, status);
PYBIND11_NUMPY_DTYPE(ins_1_t, week, timeOfWeek, insStatus, hdwStatus, theta, uvw, lla, ned);
PYBIND11_NUMPY_DTYPE(ins_2_t, week, timeOfWeek, insStatus, hdwStatus, qn2b, uvw, lla);
PYBIND11_NUMPY_DTYPE(gps_pos_t, week, timeOfWeekMs, status, ecef, lla, hMSL, hAcc, vAcc, pDop, cnoMean, towOffset, leapS, reserved);
PYBIND11_NUMPY_DTYPE(system_command_t, command, invCommand);
PYBIND11_NUMPY_DTYPE(ascii_msgs_t, options, pimu, ppimu, pins1, pins2, pgpsp, reserved, gpgga, gpgll, gpgsa, gprmc);
PYBIND11_NUMPY_DTYPE(rmc_t, bits, options);
PYBIND11_NUMPY_DTYPE(sys_params_t, timeOfWeekMs, insStatus, hdwStatus, imuTemp, baroTemp, mcuTemp, reserved1, imuPeriodMs, navPeriodMs, reserved2, genFaultCode);
PYBIND11_NUMPY_DTYPE(sys_sensors_t, time, temp, pqr, acc, mag, bar, barTemp, mslBar, humidity, vin, ana1, ana3, ana4);
PYBIND11_NUMPY_DTYPE(nvm_flash_cfg_t, size, checksum, key, startupImuDtMs, startupNavDtMs, ser0BaudRate, ser1BaudRate, insRotation, insOffset, gps1AntOffset, insDynModel, reserved, gnssSatSigConst, sysCfgBits, refLla, lastLla, lastLlaTimeOfWeekMs, lastLlaWeek, lastLlaUpdateDistance, ioConfig, cBrdConfig, gps2AntOffset, zeroVelRotation, zeroVelOffset, magInclination, magDeclination, gpsTimeSyncPeriodMs, startupGPSDtMs, RTKCfgBits, sensorConfig, wheelConfig, gpsMinimumElevation);
PYBIND11_NUMPY_DTYPE(gps_sat_t, timeOfWeekMs, numSats, sat);
PYBIND11_NUMPY_DTYPE(gps_version_t, swVersion, hwVersion, extension, reserved);
PYBIND11_NUMPY_DTYPE(mag_cal_t, recalCmd, progress, declination);
PYBIND11_NUMPY_DTYPE(internal_diagnostic_t, gapCountSerialDriver, gapCountSerialParser, rxOverflowCount, txOverflowCount, checksumFailCount);
PYBIND11_NUMPY_DTYPE(gps_rtk_rel_t, timeOfWeekMs, differentialAge, arRatio, baseToRoverVector, baseToRoverDistance, baseToRoverHeading, baseToRoverHeadingAcc);
PYBIND11_NUMPY_DTYPE(gps_rtk_misc_t, timeOfWeekMs, accuracyPos, accuracyCov, arThreshold, gDop, hDop, vDop, baseLla, cycleSlipCount, roverGpsObservationCount, baseGpsObservationCount, roverGlonassObservationCount, baseGlonassObservationCount, roverGalileoObservationCount, baseGalileoObservationCount, roverBeidouObservationCount, baseBeidouObservationCount, roverQzsObservationCount, baseQzsObservationCount, roverGpsEphemerisCount, baseGpsEphemerisCount, roverGlonassEphemerisCount, baseGlonassEphemerisCount, roverGalileoEphemerisCount, baseGalileoEphemerisCount, roverBeidouEphemerisCount, baseBeidouEphemerisCount, roverQzsEphemerisCount, baseQzsEphemerisCount, roverSbasCount, baseSbasCount, baseAntennaCount, ionUtcAlmCount, correctionChecksumFailures, timeToFirstFixMs);
// PYBIND11_NUMPY_DTYPE(sensors_t, mpu);
PYBIND11_NUMPY_DTYPE(io_t, timeOfWeekMs, gpioStatus);
PYBIND11_NUMPY_DTYPE(sys_sensors_adc_t, time, mpu, bar, barTemp, humidity, ana);
PYBIND11_NUMPY_DTYPE(gps_vel_t, timeOfWeekMs, vel, sAcc, status);
PYBIND11_NUMPY_DTYPE(rtos_info_t, freeHeapSize, mallocSize, freeSize, task);
PYBIND11_NUMPY_DTYPE(inl2_states_t, timeOfWeek, qe2b, ve, ecef, biasPqr, biasAcc, biasBaro, magDec, magInc);
PYBIND11_NUMPY_DTYPE(magnetometer_t, time, mag);
PYBIND11_NUMPY_DTYPE(barometer_t, time, bar, mslBar, barTemp, humidity);
PYBIND11_NUMPY_DTYPE(dual_imu_t, time, I, status);
//PYBIND11_NUMPY_DTYPE(gps_raw_t, receiverIndex, dataType, obsCount, reserved, data);
//PYBIND11_NUMPY_DTYPE(gps_rtk_opt_t, mode, soltype, nf, navsys, elmin, snrmin, modear, glomodear, gpsmodear, bdsmodear, arfilter, maxout, minlock, minfixsats, minholdsats, mindropsats, rcvstds, minfix, armaxiter, dynamics, niter, intpref, rovpos, refpos, eratio, err, std, prn, sclkstab, thresar, elmaskar, elmaskhold, thresslip, varholdamb, gainholdamb, maxtdiff, maxinno, maxrejc, maxgdop, baseline, ru, rb, maxaveep, outsingle, prcopt_t);
PYBIND11_NUMPY_DTYPE(manufacturing_info_t, serialNumber, lotNumber, date, key);
PYBIND11_NUMPY_DTYPE(bit_t, state, hdwBitStatus, calBitStatus, tcPqrBias, tcAccBias, tcPqrSlope, tcAccSlope, tcPqrLinearity, tcAccLinearity, pqr, acc, pqrSigma, accSigma);
PYBIND11_NUMPY_DTYPE(ins_3_t, week, timeOfWeek, insStatus, hdwStatus, qn2b, uvw, lla, msl);
PYBIND11_NUMPY_DTYPE(ins_4_t, week, timeOfWeek, insStatus, hdwStatus, qe2b, ve, ecef);
PYBIND11_NUMPY_DTYPE(inl2_ned_sigma_t, timeOfWeekMs, PxyzNED, PvelNED, PattNED, PABias, PWBias, PBaroBias, PDeclination);
PYBIND11_NUMPY_DTYPE(strobe_in_time_t, week, timeOfWeekMs, pin, count);
PYBIND11_NUMPY_DTYPE(diag_msg_t, timeOfWeekMs, messageLength, message);
PYBIND11_NUMPY_DTYPE(survey_in_t, state, maxDurationSec, minAccuracy, elapsedTimeSec, hAccuracy, lla);
//PYBIND11_NUMPY_DTYPE(evb2_t, week, timeOfWeekMs, firmwareVer, comBridgeCfg, loggerMode, loggerElapsedTimeMs, wifiSSID, wifiPSK, wifiIpAddr, serverIpAddr, serverPort);
//PYBIND11_NUMPY_DTYPE(port_monitor_t, portNumber, txTimeMs, txBytesPerS, rxTimeMs, rxBytesPerS, status);
PYBIND11_NUMPY_DTYPE(debug_array_t, i, f, lf);
PYBIND11_NUMPY_DTYPE(debug_string_t, s);

PYBIND11_NUMPY_DTYPE(inl2_status_t, ahrs, zero_accel, zero_angrate, accel_motion, rot_motion, zero_vel, ahrs_gps_cnt, att_err, att_coarse, att_aligned, att_aligning, start_proc_done, mag_cal_good, mag_cal_done, stat_magfield);
PYBIND11_NUMPY_DTYPE(inl2_mag_obs_info_t, timeOfWeekMs, Ncal_samples, ready, calibrated, auto_recal, outlier, magHdg, insHdg, magInsHdgDelta, nis, nis_threshold, Wcal, activeCalSet, magHdgOffset, Tcal);
PYBIND11_NUMPY_DTYPE(gtime_t, time, sec);
PYBIND11_NUMPY_DTYPE(rtk_state_t, time, rp_ecef, rv_ecef, ra_ecef, bp_ecef, bv_ecef, qr, b, qb, sat_id);
PYBIND11_NUMPY_DTYPE(rtk_residual_t, time, nv, sat_id_i, sat_id_j, type, v);
PYBIND11_NUMPY_DTYPE(rtk_debug_t, time, rej_ovfl, code_outlier, phase_outlier, code_large_residual, phase_large_residual, invalid_base_position, bad_baseline_holdamb, base_position_error, outc_ovfl, reset_timer, use_ubx_position, large_v2b, base_position_update, rover_position_error, reset_bias, start_relpos, end_relpos, start_rtkpos, pnt_pos_error, no_base_obs_data, diff_age_error, moveb_time_sync_error, waiting_for_rover_packet, waiting_for_base_packet, lsq_error, lack_of_valid_sats, divergent_pnt_pos_iteration, chi_square_error, cycle_slips, ubx_error, solStatus, rescode_err_marker, error_count, error_code, dist2base, reserved1, gdop_error, warning_count, warning_code, double_debug, debug, obs_count_bas, obs_count_rov, obs_pairs_filtered, obs_pairs_used, raw_ptr_queue_overrun, raw_dat_queue_overrun);

PYBIND11_NUMPY_DTYPE(obsd_t, time, sat, rcv, SNR, LLI, code, qualL, qualP, reserved, L, P, D);

PYBIND11_NUMPY_DTYPE(eph_t, sat, iode, iodc, sva, svh, week, code, flag, toe, toc, ttr, A, e, i0, OMG0, omg, M0, deln, OMGd, idot, crc, crs, cuc, cus, cic, cis, toes, fit, f0, f1, f2, tgd, Adot, ndot);
PYBIND11_NUMPY_DTYPE(geph_t, sat, iode, frq, svh, sva, age, toe, tof, pos, vel, acc, taun, gamn, dtaun);
PYBIND11_NUMPY_DTYPE(sbsmsg_t, week, tow, prn, msg, reserved);
PYBIND11_NUMPY_DTYPE(sta_t, deltype, pos, del, hgt, stationId);
PYBIND11_NUMPY_DTYPE(alm_t, sat, svh, svconf, week, toa, A, e, i0, OMG0, omg, M0, OMGd, toas, f0, f1);
PYBIND11_NUMPY_DTYPE(ion_model_utc_alm_t, ion_gps, ion_gal, ion_qzs, ion_cmp, ion_irn, utc_gps, utc_glo, utc_gal, utc_qzs, utc_cmp, utc_irn, utc_sbs, leaps, alm);


// Internal Data types
// PYBIND11_NUMPY_DTYPE(feature_bits_t, key, featureBits, hash1, hash2);
// PYBIND11_NUMPY_DTYPE(sensor_comp_unit_t, lpfLsb, temp, tempRampRate, tci, numTcPts, dtTemp);
// PYBIND11_NUMPY_DTYPE(imu1_t, pqr, acc, mag);
// PYBIND11_NUMPY_DTYPE(sensors_w_temp_t, mpu);
// PYBIND11_NUMPY_DTYPE(sensor_bias_t, timeOfWeekMs, pqr, acc, mslBar, magI, magB);
// PYBIND11_NUMPY_DTYPE(sensor_compensation_t, mpu, sampleCount, calState, alignAccel, status);
// PYBIND11_NUMPY_DTYPE(hdw_param_imu_t, pqrDev, accDev, pqrSigma, accSigma, mean);
// PYBIND11_NUMPY_DTYPE(hdw_params_t, timeOfWeekMs, I, update, gpsCnoSigma, gpsCnoMean);
// PYBIND11_NUMPY_DTYPE(nvr_manage_t, flash_write_needed, flash_write_count);
// PYBIND11_NUMPY_DTYPE(ins_dev_1_t, week, timeOfWeek, insStatus, hdwStatus, euler, uvw, lla, ned, eulerErr, uvwErr, nedErr);
// PYBIND11_NUMPY_DTYPE(inl2_misc_t, gps_time_last_valid);
// PYBIND11_NUMPY_DTYPE(rtk_debug_2_t, time, satBiasFloat, satBiasFix, qualL, sat, satBiasStd, satLockCnt, num_biases, reserved);


