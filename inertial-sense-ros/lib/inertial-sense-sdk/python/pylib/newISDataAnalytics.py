'''
Created on Feb 17, 2017

'''
# from numbers import Number
import numpy as np
import os
import sys
# import ctypes as ct
import math
import pylib.ISToolsDataSorted as itd
import subprocess as subprocess
import pdb

from pylab import plt
from scipy.interpolate import interp1d
from pylib.pose import norm, qlog, qexp, lla2ned, quat2eulerArray, meanOfQuatArray, qboxminus, qboxplus, qmult, qinv
import yaml

RAD2DEG = 180 / np.pi
DEG2RAD = np.pi / 180


def checkRawDataDrop(log):
    for dev in log.devices:
        if 'debugArray' in dev.data.keys() and 'GPS1Raw' in dev.data.keys():
            dbg = dev.data['debugArray']['i']
            counts = dev.data['GPS1Raw']['count'][dev.data['GPS1Raw']['type'] == 1]
            total_obs_in_log = np.sum(counts)
            obs_at_start = dbg[0, :]
            obs_at_end = dbg[-1, :]
            print("===================== Raw GPS Message Statistics ======================")
            print("uINS - %d", dev.serialNumber)
            print("recorded observations: %d, " % total_obs_in_log)
            print("observations reported by firmware %d, " % obs_at_end[3])

def checkTempOfLOF(log):
    for dev in log.devices:
        if 'gpsRtkNav' not in dev.data.keys() or 'sysParams' not in dev.data.keys():
            continue
        fix_status = dev.data['gpsRtkNav']['status'] & 0xFF00 == 0xC00
        time_of_lost_fix = dev.data['gpsRtkNav']['towMs'][np.argmax(~fix_status)]
        imu_temp_at_time_of_lost_fix = dev.data['sysParams']['imuTemp'][np.argmax(dev.data['sysParams']['towMs'] > time_of_lost_fix)]
        baro_temp_at_time_of_lost_fix = dev.data['sysParams']['baroTemp'][np.argmax(dev.data['sysParams']['towMs'] > time_of_lost_fix)]
        time_of_regain_fix = dev.data['gpsRtkNav']['towMs'][len(fix_status) - np.argmax(~fix_status[::-1]) - 1]
        imu_temp_at_time_of_regain_fix = dev.data['sysParams']['imuTemp'][np.argmax(dev.data['sysParams']['towMs'] > time_of_regain_fix)]
        baro_temp_at_time_of_regain_fix = dev.data['sysParams']['baroTemp'][np.argmax(dev.data['sysParams']['towMs'] > time_of_regain_fix)]
        print ("dev: {0} lost: IMU = {1}, baro = {2}, regain: IMU={3}, baro={4}".format(dev.data['devInfo']['serialNumber'][0], imu_temp_at_time_of_lost_fix,
                baro_temp_at_time_of_lost_fix, imu_temp_at_time_of_regain_fix, baro_temp_at_time_of_regain_fix))

def calcRMS(log, directory, subdir):
    file = open("/home/superjax/Documents/inertial_sense/config.yaml", 'r')
    config = yaml.load(file)
    directory = config["directory"]
    serials = config['serials']

    numDev = len(log.devices)
    debug = True
    np.set_printoptions(linewidth=200)
    averageRMS = []
    compassing = False
    navMode = (log.devices[0].data['ins2']['iStatus'] & 0x1000)[-1]
    if numDev > 1:

        print("\nComputing RMS Accuracies: (%d devices)" % (numDev))

        # Build a 3D array of the data.  idx 0 = Device,    idx 1 = t,     idx 2 = [t, lla, uvw, log(q)]
        data = [np.hstack((log.devices[i].data['ins2']['tow'][:,None],
                           log.devices[i].data['ins2']['lla'],
                           log.devices[i].data['ins2']['uvw'],
                           log.devices[i].data['ins2']['q'])) for i in range(numDev)]

        # Make sure that the time stamps are realistic
        for dev in range(numDev):
            if (np.diff(data[dev][:,0]) > 10.0).any():
                print("large gaps in data for dev", dev, "chopping off data before gap".format(dev))
                idx = np.argmax(np.diff(data[dev][:,0])) + 1
                data[dev] = data[dev][idx:,:]

        min_time = max([np.min(data[i][:,0]) for i in range(numDev)])
        max_time = min([np.max(data[i][:,0]) for i in range(numDev)])


        # If we are in compassing mode, then only calculate RMS after all devices have fix
        if log.devices[0].data['flashConfig']['RTKCfgBits'][-1] == 8:
            compassing = True
            time_of_fix_ms = [dev.data['gps1RtkCmpRel']['timeOfWeekMs'][np.argmax(dev.data['gps1RtkCmpRel']['arRatio'] > 3.0)] / 1000.0 for dev in log.devices]
            # print time_of_fix_ms
            min_time = max(time_of_fix_ms)


        # only take the second half of the data
        min_time = max_time - (max_time - min_time)/2.0

        # Resample at a steady 100 Hz
        dt = 0.01
        t = np.arange(1.0, max_time - min_time - 1.0, dt)
        for i in range(numDev):
            # Chop off extra data at beginning and end
            data[i] = data[i][data[i][:, 0] > min_time]
            data[i] = data[i][data[i][:, 0] < max_time]

            # Chop off the min time so everything is wrt to start
            data[i][:,0] -= min_time

            # Interpolate data so that it has all the same timestamps
            fi = interp1d(data[i][:,0], data[i][:,1:].T, kind='cubic', fill_value='extrapolate', bounds_error=False)
            data[i] = np.hstack((t[:,None], fi(t).T))

            # Normalize Quaternions
            data[i][:,7:] /= norm(data[i][:,7:], axis=1)[:,None]

        # Make a big 3D numpy array we can work with [dev, sample, data]
        data = np.array(data)


        # Convert lla to ned using first device lla at center of data as reference
        refLla = data[0, int(round(len(t) / 2.0)), 1:4].copy()
        for i in range(numDev):
            data[i, :, 1:4] = lla2ned(refLla, data[i, :, 1:4])

        # Find Mean Data
        means = np.empty((len(data[0]), 10))
        means[:,:6] = np.mean(data[:,:,1:7], axis=0) # calculate mean position and velocity across devices
        means[:,6:] = meanOfQuatArray(data[:,:,7:].transpose((1,0,2))) # Calculate mean attitude of all devices at each timestep

        # calculate the attitude error for each device
        att_error = np.array([qboxminus(data[dev,:, 7:], means[:, 6:]) for dev in range(numDev)])
        # Calculate the Mounting Bias for all devices (assume the mounting bias is the mean of the attitude error)
        mount_bias = np.mean(att_error, axis=1)
        if compassing:
            # When in compassing, assume all units are sharing the same GPS antennas and should therefore have
            # no mounting bias in heading
            mount_bias[:,2] = 0

        # Adjust all attitude errors to the mean by the mounting bias
        # TODO: Talk to Walt about the mount bias - because this probably includes more biases than just the mount bias
        att_error -= mount_bias[:,None,:]

        if debug:
            colors = ['r', 'g', 'b', 'm']
            plt.figure()
            plt.subplot(3,1,1) # Position
            plt.title("position error")
            for m in range(3):
                for n in range(numDev):
                    plt.plot(data[n,:,0], data[n, :, m+1], color = colors[m])
                plt.plot(data[0,:,0], means[:, m], linewidth=2, color = colors[m])
            plt.subplot(3,1,2)
            plt.title("velocity error")
            for m in range(3):
                for n in range(numDev):
                    plt.plot(data[n,:,0], data[n, :, m+4], color = colors[m] )
                plt.plot(data[0,:,0], means[:, m+3], linewidth=2, color = colors[m])
            plt.subplot(3,1,3)
            plt.title("attitude")
            for m in range(4):
                for n in range(numDev):
                    plt.plot(data[n,:,0], data[n, :, m+7], color = colors[m])
                plt.plot(data[0,:,0], means[:, m+6], linewidth=2, color = colors[m])

            plt.figure()
            for m in range(3):
                plt.subplot(3, 1, m +1)
                for n in range(numDev):
                    plt.plot(att_error[n, :, m])
            plt.show()

        # RMS = sqrt ( 1/N sum(e^2) )
        RMS = np.empty((numDev, 9))
        # Calculate RMS for position and velocity
        RMS[:,:6] = np.sqrt(np.mean(np.square(data[:, :, 1:7] - means[:,0:6]), axis=1))
        # Calculate RMS for attitude
        RMS[:,6:] = np.sqrt(np.mean(np.square(att_error[:, :, :]), axis=1))

        # Average RMS across devices
        averageRMS = np.mean(RMS, axis=0)

        print("average RMS = ", averageRMS)

        # Convert Attitude Error To Euler Angles
        RMS_euler = RMS[:,6:] # quat2eulerArray(qexp(RMS[:,6:]))
        averageRMS_euler = averageRMS[6:] #quat2eulerArray(qexp(averageRMS[None,6:]))[0]
        mount_bias_euler = mount_bias #quat2eulerArray(qexp(mount_bias))


        # Below is creating the RMS report
        thresholds = np.array([0.2, 0.2, 0.2, # LLA
                               0.2, 0.2, 0.2, # UVW
                               0.1, 0.1, 2.0]) # ATT (rpy) - (deg)
        if navMode or compassing:
            thresholds[8] = 0.3 # Higher heading accuracy
        else:
            thresholds[:6] = np.inf

        thresholds[6:] *= DEG2RAD # convert degrees threshold to radians



        specRatio = averageRMS / thresholds

        filename = os.path.join(directory, 'RMS_report_new.txt');
        f = open(filename, 'w')
        f.write('*****   Performance Analysis Report - %s   *****\n' % (subdir))
        f.write('\n')
        f.write('Directory: %s\n' % (directory))
        mode = "AHRS"
        if navMode: mode = "NAV"
        if compassing: mode = "DUAL GNSS"
        f.write("\n")

        # Print Table of RMS accuracies
        line = 'Device       '
        if navMode:
            f.write(
                '--------------------------------------------------- RMS Accuracy -------------------------------------------\n')
            line = line + 'UVW[  (m/s)   (m/s)   (m/s) ],  NED[    (m)     (m)     (m) ],'
        else:  # AHRS mode
            f.write('-------------- RMS Accuracy --------------\n')
        line = line + ' Att [  (deg)   (deg)   (deg) ]\n'
        f.write(line)

        for n in range(0, numDev):
            devInfo = itd.cDevInfo(log.devices[n].data['devInfo'])
            line = '%2d SN%d      ' % (n, devInfo.v['serialNumber'][-1])
            if navMode:
                line = line + '[ %6.4f  %6.4f  %6.4f ],     ' % ( RMS[n, 3], RMS[n, 4], RMS[n, 5])
                line = line + '[ %6.4f  %6.4f  %6.4f ],     ' % ( RMS[n, 0], RMS[n, 1], RMS[n, 2])
            line = line + '[ %6.4f  %6.4f  %6.4f ]\n' % (RMS_euler[n, 0] * RAD2DEG, RMS_euler[n, 1] * RAD2DEG, RMS_euler[n, 2] * RAD2DEG)
            f.write(line)

        line = 'AVERAGE:        '
        if navMode:
            f.write('------------------------------------------------------------------------------------------------------------\n')
            line = line + '[%7.4f %7.4f %7.4f ],     ' % (averageRMS[3], averageRMS[4], averageRMS[5])
            line = line + '[%7.4f %7.4f %7.4f ],     ' % (averageRMS[0], averageRMS[1], averageRMS[2])
        else:  # AHRS mode
            f.write('------------------------------------------\n')
        line = line + '[%7.4f %7.4f %7.4f ]\n' % (averageRMS_euler[0] * RAD2DEG, averageRMS_euler[1] * RAD2DEG, averageRMS_euler[2] * RAD2DEG)
        f.write(line)

        line = 'THRESHOLD:      '
        if navMode:
            line = line + '[%7.4f %7.4f %7.4f ],     ' % (thresholds[3], thresholds[4], thresholds[5])
            line = line + '[%7.4f %7.4f %7.4f ],     ' % (thresholds[0], thresholds[1], thresholds[2])
        line = line + '[%7.4f %7.4f %7.4f ]\n' % (thresholds[6] * RAD2DEG, thresholds[7] * RAD2DEG, thresholds[8] * RAD2DEG)
        f.write(line)

        line = 'RATIO:          '
        if navMode:
            f.write('------------------------------------------------------------------------------------------------------------\n')
            line = line + '[%7.4f %7.4f %7.4f ],     ' % (specRatio[3], specRatio[4], specRatio[5])
            line = line + '[%7.4f %7.4f %7.4f ],     ' % (specRatio[0], specRatio[1], specRatio[2])
        else:  # AHRS mode
            f.write('------------------------------------------\n')
        line = line + '[%7.4f %7.4f %7.4f ]\n' % (specRatio[6], specRatio[7], specRatio[8])
        f.write(line)

        def pass_fail(ratio): return 'FAIL' if ratio > 1.0 else 'PASS'

        line = 'PASS/FAIL:      '
        if navMode:
            line        = line + '[   %s    %s    %s ],     ' % (pass_fail(specRatio[3]),pass_fail(specRatio[4]),pass_fail(specRatio[5])) # LLA
            line        = line + '[   %s    %s    %s ],     ' % (pass_fail(specRatio[0]),pass_fail(specRatio[1]),pass_fail(specRatio[2])) # UVW
        line            = line + '[   %s    %s    %s ]\n' % (pass_fail(specRatio[6]),pass_fail(specRatio[7]),pass_fail(specRatio[8]))     # ATT
        f.write(line)

        if navMode:
            f.write('                                                                                         ')
        else:  # AHRS mode
            f.write('                  ')
        f.write('('+mode +' mode)\n\n')

        # Print Mounting Biases
        f.write('--------------- Angular Mounting Biases ----------------\n')
        f.write('Device       Euler Biases[   (deg)     (deg)     (deg) ]\n')
        for n in range(0, numDev):
            devInfo = itd.cDevInfo(log.devices[n].data['devInfo'])
            f.write('%2d SN%d               [ %7.4f   %7.4f   %7.4f ]\n' % (
                n, devInfo.v['serialNumber'][-1], mount_bias_euler[n, 0] * RAD2DEG, mount_bias_euler[n, 1] * RAD2DEG, mount_bias_euler[n, 2] * RAD2DEG))
        f.write('\n')

        # Print Device Version Information
        f.write(
            '------------------------------------------- Device Info -------------------------------------------------\n')
        for n in range(0, numDev):
            devInfo = itd.cDevInfo(log.devices[n].data['devInfo'])
            hver = devInfo.v['hardwareVer'][-1]
            cver = devInfo.v['commVer'][-1]
            fver = devInfo.v['firmwareVer'][-1]
            buld = devInfo.v['build'][-1]
            repo = devInfo.v['repoRevision'][-1]
            date = devInfo.v['buildDate'][-1]
            time = devInfo.v['buildTime'][-1]
            addi = devInfo.v['addInfo'][-1]
            f.write(
                '%2d SN%d  HW: %d.%d.%d.%d   FW: %d.%d.%d.%d build %d repo %d   Proto: %d.%d.%d.%d  Date: %04d-%02d-%02d %02d:%02d:%02d  %s\n' % (
                    n, devInfo.v['serialNumber'][-1],
                    hver[3], hver[2], hver[1], hver[0],
                    fver[3], fver[2], fver[1], fver[0], buld, repo,
                    cver[3], cver[2], cver[1], cver[0],
                    2000 + date[2], date[1], date[0],
                    time[3], time[2], time[1],
                    addi))
        f.write('\n')

        f.close()

        # Automatically open report in Windows
        if 'win' in sys.platform:
            subprocess.Popen(["notepad.exe", filename])  # non-blocking call
        if 'linux' in sys.platform:
            subprocess.Popen(['gedit', filename])

    print("Done.")

    # TODO: Pass out the union of the test errors
    return averageRMS

