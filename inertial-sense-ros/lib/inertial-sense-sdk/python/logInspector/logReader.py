import numpy as np
import sys
import glob
import serial
import os
import subprocess
from os.path import expanduser
from scipy.interpolate import interp1d

import sys
# Add the ptdraft folder path to the sys.path list
import sys
sys.path.append('..')
from log_reader import LogReader
import yaml
# from ci_hdw.data_sets import *
from pylib.data_sets import *
from pylib.pose import *
import datetime
from pylib.ISToolsDataSorted import refLla, getTimeFromTowMs, getTimeFromTow, setGpsWeek, getTimeFromGTime

RAD2DEG = 180.0 / np.pi
DEG2RAD = np.pi / 180.0

RED = '\u001b[31m'
RESET = '\u001b[0m'

class Log:
    def __init__(self):
        self.c_log = LogReader()
        self.data = []
        self.serials = []
        self.passRMS = 0    # 1 = pass, -1 = fail, 0 = unknown

    def load(self, directory, serials=['ALL']):
        self.data = []
        self.c_log.init(self, directory, serials)
        self.c_log.load()
        self.sanitize()
        self.data = np.array(self.data)
        self.directory = directory
        self.numDev = self.data.shape[0]
        if self.numDev == 0:
            raise ValueError("No devices found in log")
        self.serials = [self.data[d, DID_DEV_INFO]['serialNumber'][0] for d in range(self.numDev)]
        if 10101 in self.serials:
            self.refINS = True
            refIdx = self.serials.index(10101)
            self.truth = self.data[refIdx].copy()
        else:
            self.refINS = False
            self.refdata = []
        self.compassing = 'Cmp' in str(self.data[0, DID_DEV_INFO]['addInfo'][-1])
        self.rtk = 'Rov' in str(self.data[0, DID_DEV_INFO]['addInfo'][-1])
        self.navMode = (self.data[0, DID_INS_2]['insStatus'][-1] & 0x1000) == 0x1000
        # except:
            # print(RED + "error loading log" + sys.exc_info()[0] + RESET)

    def exitHack(self):
        self.c_log.exitHack()
        
    def did_callback(self, did, arr, dev_id):
        if did >= NUM_DIDS:
            return
        while dev_id >= len(self.data):
            self.data.append([[] for i in range(NUM_DIDS+1)])
        self.data[dev_id][did] = arr

    def gps_raw_data_callback(self, did, arr, dev_id, msg_type):
        if did >= NUM_DIDS:
            return
        if self.data[dev_id][did] == []:
            self.data[dev_id][did] = [[] for i in range(6)]
        if dev_id < len(self.data) and did < len(self.data[dev_id]) and 6 <= len(self.data[dev_id][did]):
            if msg_type == eRawDataType.raw_data_type_observation.value:
                self.data[dev_id][did][0].append(arr)
            elif msg_type == eRawDataType.raw_data_type_ephemeris.value:
                self.data[dev_id][did][1] = arr
            elif msg_type == eRawDataType.raw_data_type_glonass_ephemeris.value:
                self.data[dev_id][did][2] = arr
            elif msg_type == eRawDataType.raw_data_type_sbas.value:
                self.data[dev_id][did][3] = arr
            elif msg_type == eRawDataType.raw_data_type_ionosphere_model_utc_alm.value:
                self.data[dev_id][did][4] = arr
            elif msg_type == eRawDataType.raw_data_type_base_station_antenna_position.value:
                self.data[dev_id][did][5] = arr


    def sanitize(self):
        return
        GPS_start_Time = datetime.datetime.strptime('6/Jan/1980', "%d/%b/%Y")

        # Use DID_INS_1 if necessary
        if np.size(self.data[0][DID_INS_2])==0 and np.size(self.data[0][DID_INS_1])!=0:
            self.data[0][DID_INS_2].resize(np.size(self.data[0][DID_INS_1]))
            self.data[0][DID_INS_2]['timeOfWeek'] = self.data[0][DID_INS_1]['timeOfWeek']
            self.data[0][DID_INS_2]['week'] = self.data[0][DID_INS_1]['week']
            self.data[0][DID_INS_2]['insStatus'] = self.data[0][DID_INS_1]['insStatus']
            self.data[0][DID_INS_2]['hdwStatus'] = self.data[0][DID_INS_1]['hdwStatus']
            self.data[0][DID_INS_2]['qn2b'] = euler2quatArray(self.data[0][DID_INS_1]['theta'])
            self.data[0][DID_INS_2]['uvw'] = self.data[0][DID_INS_1]['uvw']
            self.data[0][DID_INS_2]['lla'] = self.data[0][DID_INS_1]['lla']

        week_time = GPS_start_Time + (datetime.timedelta(weeks=int(self.data[0][DID_INS_2]['week'][-1])))

        for d, dev in enumerate(self.data):
            if len(dev[DID_INS_2]) == 0:
                print("\033[93m" + "missing DID_INS_2 data: removing device\033[0m")
                del self.data[d]
                break

            if len(dev[DID_DEV_INFO]) == 0:
                print("\033[93m" + "missing DID_DEV_INFO data: making some up\033[0m")
                self.data[d][DID_DEV_INFO] = np.resize(self.data[d][DID_DEV_INFO], 1)
                self.data[d][DID_DEV_INFO]['serialNumber'][0] = d

        for d, dev in enumerate(self.data):
            for did in range(len(self.data[0])):
                if isinstance(dev[did], list):
                    continue
                for field in ['towMs', 'timeOfWeekMs', 'tow', 'timeOfWeek']:
                    if field in dev[did].dtype.names:
                        if (np.diff(dev[did][field].astype(np.int64)) < 0).any():
                            idx = np.argmin(np.diff(dev[did][field].astype(np.int64)))
                            if 'Ms' in field:
                                t1 = week_time + datetime.timedelta(milliseconds=int(dev[did][field][idx]))
                                t2 = week_time + datetime.timedelta(milliseconds=int(dev[did][field][idx+1]))
                            else:
                                t1 = week_time + datetime.timedelta(seconds=int(dev[did][field][idx]))
                                t2 = week_time + datetime.timedelta(seconds=int(dev[did][field][idx+1]))
                            print("\033[93m" + "Time went backwards in " + did_name_lookup[did] + r"!!!, " +
                                  " Time went from " + str(t1) + " to "  + str(t2) + ". removing all data "
                                  + ("before" if idx < len(dev[did][field])/2.0 else "after") + "\033[0m")
                            if idx < len(dev[did][field])/2.0:
                                self.data[d][did] = dev[did][idx+1:]
                            else:
                                self.data[d][did] = dev[did][:idx]
                        ms_multiplier = 1000.0 if 'Ms' in field else 1.0
                        if (np.diff(dev[did][field]) > 3600 * ms_multiplier).any():
                            print("\033[93m" + "greater than 1 minute gap in " + did_name_lookup[did]
                                  + " data, assuming GPS fix was acquired during data set, and chopping data"+ "\033[0m")
                            idx = np.argmax(np.diff(dev[did][field])) + did
                            self.data[d][did] = dev[did][idx:]

    def getRMSArray(self):
        if self.numDev > 1 or self.refINS:
            print("\nComputing RMS Accuracies: (%d devices)" % (self.numDev))

            # Build a 3D array of the data.  idx 0 = Device,    idx 1 = t,     idx 2 = [t, lla, uvw, log(q)]
            data = [np.hstack((self.data[i, DID_INS_2]['timeOfWeek'][:, None],
                               self.data[i, DID_INS_2]['lla'],
                               self.data[i, DID_INS_2]['uvw'],
                               self.data[i, DID_INS_2]['qn2b'])) for i in range(self.numDev)]

            # Make sure that the time stamps are realistic
            for dev in range(self.numDev):
                if (np.diff(data[dev][:, 0]) > 10.0).any():
                    print("\033[93m" + "large gaps in data for dev" + str(dev)
                          + "chopping off data before gap".format(dev) + "\033[0m")
                    idx = np.argmax(np.diff(data[dev][:, 0])) + 1
                    data[dev] = data[dev][idx:, :]

            self.min_time = max([np.min(data[i][:, 0]) for i in range(self.numDev)])
            self.max_time = min([np.max(data[i][:, 0]) for i in range(self.numDev)])

            # If we are in compassing mode, then only calculate RMS after all devices have fix
            if self.compassing:
                # time_of_fix_ms = [self.data[dev, DID_GPS1_RTK_CMP_REL]['timeOfWeekMs'][np.argmax(self.data[dev, DID_GPS1_RTK_CMP_REL]['arRatio'] > 3.0)] / 1000.0 for dev in range(self.numDev)]
                time_of_fix_ms = [self.data[dev, DID_GPS1_POS]['timeOfWeekMs'][np.argmax(self.data[dev, DID_GPS1_POS]['status'] & 0x08000000)] / 1000.0 for dev in range(self.numDev)]
                # print time_of_fix_ms
                self.min_time = max(time_of_fix_ms)

            # Use middle third of data
            self.min_time = self.max_time - 2.0*(self.max_time - self.min_time)/3.0
            self.max_time = self.max_time - (self.max_time - self.min_time)/3.0

            # Resample at a steady 100 Hz
            dt = 0.01
            t = np.arange(1.0, self.max_time - self.min_time - 1.0, dt)
            for i in range(self.numDev):
                # Chop off extra data at beginning and end
                data[i] = data[i][data[i][:, 0] > self.min_time]
                data[i] = data[i][data[i][:, 0] < self.max_time]

                # Chop off the min time so everything is wrt to start
                data[i][:, 0] -= self.min_time

                # Interpolate data so that it has all the same timestamps
                fi = interp1d(data[i][:, 0], data[i][:, 1:].T, kind='cubic', fill_value='extrapolate',
                              bounds_error=False)
                data[i] = np.hstack((t[:, None], fi(t).T))

                # Normalize Quaternions
                data[i][:, 7:] /= norm(data[i][:, 7:], axis=1)[:, None]

            # Make a big 3D numpy array we can work with [dev, sample, data]
            data = np.array(data)

            # Convert lla to ned using first device lla at center of data as reference
            refLla = data[0, int(round(len(t) / 2.0)), 1:4].copy()
            for i in range(self.numDev):
                data[i, :, 1:4] = lla2ned(refLla, data[i, :, 1:4])

            self.stateArray = data

    def getRMSTruth(self):
        if not self.refINS:
            # Find Mean Data
            means = np.empty((len(self.stateArray[0]), 10))
            means[:, :6] = np.mean(self.stateArray[:, :, 1:7], axis=0)  # calculate mean position and velocity across devices
            means[:, 6:] = meanOfQuatArray(self.stateArray[:, :, 7:].transpose((1, 0, 2)))  # Calculate mean attitude of all devices at each timestep
            self.truth = means
        else:
            self.refIdx = self.serials.index(10101)
            self.truth = self.stateArray[self.refIdx, :, 1:]
            self.stateArray = np.delete(self.stateArray, self.refIdx, 0)


    def calcAttitudeError(self):
        att_error = np.array([qboxminus(self.stateArray[dev, :, 7:], self.truth[:, 6:]) for dev in range(len(self.stateArray))])
        self.att_error = att_error


    def calculateRMS(self):
        self.data = np.array(self.data)
        self.getRMSArray()
        self.getRMSTruth()
        self.calcAttitudeError()

        # Calculate the Mounting Bias for all devices (assume the mounting bias is the mean of the attitude error)
        self.mount_bias = np.mean(self.att_error, axis=1)
        if self.compassing:
            # When in compassing, assume all units are sharing the same GPS antennas and should therefore have
            # no mounting bias in heading
            self.mount_bias[:, 2] = 0
        self.att_error = self.att_error - self.mount_bias[:,None,:]

        # RMS = sqrt ( 1/N sum(e^2) )
        self.RMS = np.empty((len(self.stateArray), 9))
        self.RMS[:,:6] = np.sqrt(np.mean(np.square(self.stateArray[:, :, 1:7] - self.truth[:,0:6]), axis=1)) # [ pos, vel ]
        self.RMS[:,6:] = np.sqrt(np.mean(np.square(self.att_error[:, :, :]), axis=1)) # [ att }
        self.RMS_euler = self.RMS[:, 6:]  # quat2eulerArray(qexp(RMS[:,6:]))

        # Average RMS across devices
        self.averageRMS = np.mean(self.RMS, axis=0)
        self.averageRMS_euler = self.averageRMS[6:]  # quat2eulerArray(qexp(averageRMS[None,6:]))[0]
        self.mount_bias_euler = self.mount_bias  # quat2eulerArray(qexp(mount_bias))

    def pass_fail(self, ratio):
        if ratio > 1.0:
            self.tmpPassRMS = -1
            return 'FAIL'
        else:
            # self.tmpPassRMS = -1  # Debug
            return 'PASS'

    def printRMSReport(self):
        self.tmpPassRMS = 1
        filename = os.path.join(self.directory, 'RMS_report_new_logger.txt')
        thresholds = np.array([0.35, 0.35, 0.8, # (m)   NED
                               0.2, 0.2, 0.2,   # (m/s) UVW
                               0.11, 0.11, 2.0])  # (deg) ATT (roll, pitch, yaw)
        if self.navMode or self.compassing:
            thresholds[8] = 0.3  # Higher heading accuracy
        else:
            thresholds[:6] = np.inf

        if self.compassing:
            thresholds[0] = 1.0
            thresholds[1] = 1.0
            thresholds[2] = 1.0

        thresholds[6:] *= DEG2RAD  # convert degrees threshold to radians

        self.specRatio = self.averageRMS / thresholds

        uINS_device_idx = [n for n in range(self.numDev) if self.serials[n] != 10101]

        f = open(filename, 'w')
        f.write('*****   Performance Analysis Report - %s   *****\n' % (self.directory))
        f.write('\n')
        f.write('Directory: %s\n' % (self.directory))
        mode = "AHRS"
        if self.navMode: mode = "NAV"
        if self.compassing: mode = "DUAL GNSS"
        if self.refINS: mode += " With NovaTel Reference"
        f.write("\n")

        # Print Table of RMS accuracies
        line = 'Device       '
        if self.navMode:
            f.write(
                '--------------------------------------------------- RMS Accuracy -------------------------------------------\n')
            line = line + 'UVW[  (m/s)   (m/s)   (m/s) ],  NED[    (m)     (m)     (m) ],'
        else:  # AHRS mode
            f.write('-------------- RMS Accuracy --------------\n')
        line = line + ' Att [  (deg)   (deg)   (deg) ]\n'
        f.write(line)

        for n, dev in enumerate(uINS_device_idx):
            devInfo = self.data[dev,DID_DEV_INFO][0]
            line = '%2d SN%d      ' % (n, devInfo['serialNumber'])
            if self.navMode:
                line = line + '[ %6.4f  %6.4f  %6.4f ],     ' % (self.RMS[n, 3], self.RMS[n, 4], self.RMS[n, 5])
                line = line + '[ %6.4f  %6.4f  %6.4f ],     ' % (self.RMS[n, 0], self.RMS[n, 1], self.RMS[n, 2])
            line = line + '[ %6.4f  %6.4f  %6.4f ]\n' % (
            self.RMS_euler[n, 0] * RAD2DEG, self.RMS_euler[n, 1] * RAD2DEG, self.RMS_euler[n, 2] * RAD2DEG)
            f.write(line)

        line = 'AVERAGE:        '
        if self.navMode:
            f.write(
                '------------------------------------------------------------------------------------------------------------\n')
            line = line + '[%7.4f %7.4f %7.4f ],     ' % (self.averageRMS[3], self.averageRMS[4], self.averageRMS[5])
            line = line + '[%7.4f %7.4f %7.4f ],     ' % (self.averageRMS[0], self.averageRMS[1], self.averageRMS[2])
        else:  # AHRS mode
            f.write('------------------------------------------\n')
        line = line + '[%7.4f %7.4f %7.4f ]\n' % (
        self.averageRMS_euler[0] * RAD2DEG, self.averageRMS_euler[1] * RAD2DEG, self.averageRMS_euler[2] * RAD2DEG)
        f.write(line)

        line = 'THRESHOLD:      '
        if self.navMode:
            line = line + '[%7.4f %7.4f %7.4f ],     ' % (thresholds[3], thresholds[4], thresholds[5])
            line = line + '[%7.4f %7.4f %7.4f ],     ' % (thresholds[0], thresholds[1], thresholds[2])
        line = line + '[%7.4f %7.4f %7.4f ]\n' % (
        thresholds[6] * RAD2DEG, thresholds[7] * RAD2DEG, thresholds[8] * RAD2DEG)
        f.write(line)

        line = 'RATIO:          '
        if self.navMode:
            f.write(
                '------------------------------------------------------------------------------------------------------------\n')
            line = line + '[%7.4f %7.4f %7.4f ],     ' % (self.specRatio[3], self.specRatio[4], self.specRatio[5])
            line = line + '[%7.4f %7.4f %7.4f ],     ' % (self.specRatio[0], self.specRatio[1], self.specRatio[2])
        else:  # AHRS mode
            f.write('------------------------------------------\n')
        line = line + '[%7.4f %7.4f %7.4f ]\n' % (self.specRatio[6], self.specRatio[7], self.specRatio[8])
        f.write(line)

        line = 'PASS/FAIL:      '
        if self.navMode:
            line = line + '[   %s    %s    %s ],     ' % (
            self.pass_fail(self.specRatio[3]), self.pass_fail(self.specRatio[4]), self.pass_fail(self.specRatio[5]))  # LLA
            line = line + '[   %s    %s    %s ],     ' % (
            self.pass_fail(self.specRatio[0]), self.pass_fail(self.specRatio[1]), self.pass_fail(self.specRatio[2]))  # UVW
        line = line + '[   %s    %s    %s ]\n' % (
        self.pass_fail(self.specRatio[6]), self.pass_fail(self.specRatio[7]), self.pass_fail(self.specRatio[8]))  # ATT
        f.write(line)

        if self.navMode:
            f.write('                                                                                         ')
        else:  # AHRS mode
            f.write('                  ')
        f.write('(' + mode + ')\n\n')

        # Print Mounting Biases
        f.write('--------------- Angular Mounting Biases ----------------\n')
        f.write('Device       Euler Biases[   (deg)     (deg)     (deg) ]\n')
        for n, dev in enumerate(uINS_device_idx):
            devInfo = self.data[dev, DID_DEV_INFO][0]
            f.write('%2d SN%d               [ %7.4f   %7.4f   %7.4f ]\n' % (
                n, devInfo['serialNumber'], self.mount_bias_euler[n, 0] * RAD2DEG, self.mount_bias_euler[n, 1] * RAD2DEG,
                self.mount_bias_euler[n, 2] * RAD2DEG))
        f.write('\n')

        f.write("----------------- Average Attitude ---------------------\n")
        f.write("Dev:  \t[ Roll\t\tPitch\t\tYaw ]\n")
        for i in range(self.numDev):
            qavg = meanOfQuat(self.stateArray[i, :, 7:])
            euler = quat2euler(qavg.T) * 180.0 / np.pi
            f.write("%d\t%f\t%f\t%f\n" % (self.serials[i], euler[0], euler[1], euler[2]))

        # Print Device Version Information
        f.write(
            '\n\n------------------------------------------- Device Info -------------------------------------------------\n')
        for n, dev in enumerate(uINS_device_idx):
            devInfo = self.data[dev, DID_DEV_INFO][0]
            hver = devInfo['hardwareVer']
            cver = devInfo['protocolVer']
            fver = devInfo['firmwareVer']
            buld = devInfo['buildNumber']
            repo = devInfo['repoRevision']
            date = devInfo['buildDate']
            time = devInfo['buildTime']
            addi = devInfo['addInfo']
            f.write(
                '%2d SN%d  HW: %d.%d.%d.%d   FW: %d.%d.%d.%d build %d repo %d   Proto: %d.%d.%d.%d  Date: %04d-%02d-%02d %02d:%02d:%02d  %s\n' % (
                    n, devInfo['serialNumber'],
                    hver[3], hver[2], hver[1], hver[0],
                    fver[3], fver[2], fver[1], fver[0], buld, repo,
                    cver[3], cver[2], cver[1], cver[0],
                    2000 + date[2], date[1], date[0],
                    time[3], time[2], time[1],
                    addi))
        f.write('\n')

        f.close()

        # Report if RMS passed all
        self.passRMS = self.tmpPassRMS
        return self.passRMS

    def debugPlot(self):
        import matplotlib.pyplot as plt
        colors = ['r', 'g', 'b', 'm']
        plt.figure()
        plt.subplot(3,1,1) # Position
        plt.title("position error")
        for m in range(3):
            for n in range(len(self.stateArray)):
                plt.plot(self.stateArray[n,:,0], self.stateArray[n, :, m+1], color = colors[m])
            plt.plot(self.stateArray[0,:,0], self.truth[:, m], linewidth=2, color = colors[m])
        plt.subplot(3,1,2)
        plt.title("velocity error")
        for m in range(3):
            for n in range(len(self.stateArray)):
                plt.plot(self.stateArray[n,:,0], self.stateArray[n, :, m+4], color = colors[m] )
            plt.plot(self.stateArray[0,:,0], self.truth[:, m+3], linewidth=2, color = colors[m])
        plt.subplot(3,1,3)
        plt.title("attitude")
        for m in range(4):
            for n in range(len(self.stateArray)):
                plt.plot(self.stateArray[n,:,0], self.stateArray[n, :, m+7], color = colors[m])
            plt.plot(self.stateArray[0,:,0], self.truth[:, m+6], linewidth=2, color = colors[m])

        plt.figure()
        for m in range(3):
            plt.subplot(3, 1, m +1)
            for n in range(len(self.stateArray)):
                plt.plot(self.att_error[n, :, m])
        plt.show()

    def openRMSReport(self):
        filename = os.path.join(self.directory, 'RMS_report_new_logger.txt')
        if 'win' in sys.platform:
            subprocess.Popen(["notepad.exe", filename])
        if 'linux' in sys.platform:
            subprocess.Popen(['gedit', filename])


if __name__ == '__main__':
    np.set_printoptions(linewidth=200)
    
    home = expanduser("~")

    # 2nd argument: Log Directory
    if len(sys.argv) >= 2:
        directory = sys.argv[1]
        serials = ["ALL"]

    if 'directory' not in locals():
        print("First parameter must be directory!")
        exit()

        # Load from config.yaml
        file = open(home + "/Documents/Inertial_Sense/config.yaml", 'r')
        config = yaml.load(file)
        directory = config["directory"]
        serials = ["ALL"]

    log = Log()
    log.load(directory)

    # Compute and output RMS Report
    log.calculateRMS()
    # log.debugPlot()
    log.printRMSReport()
    log.openRMSReport()
