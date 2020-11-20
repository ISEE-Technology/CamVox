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

RAD2DEG = 180/np.pi
DEG2RAD = np.pi/180

def calculateDistance(lat1,lon1,lat2,lon2):
    # All angles are in radians

    # Haversine formula
    Re = 6371000 # (m)
    dLat = lat2-lat1
    dLon = lon2-lon1

    a = math.sin(dLat/2)*math.sin(dLat/2) + math.cos(lat1) * math.cos(lat2) * math.sin(dLon/2) * math.sin(dLon/2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

    d = Re * c

    return d

def checkRawDataDrop(log):
    for dev in log.devices:
        if 'debugArray' in dev.data.keys() and 'GPS1Raw' in dev.data.keys():
            dbg = dev.data['debugArray']['i']
            counts = dev.data['GPS1Raw']['count'][dev.data['GPS1Raw']['type'] == 1]
            total_obs_in_log=np.sum(counts)
            obs_at_start = dbg[0,:]
            obs_at_end = dbg[-1, :]
            print("===================== Raw GPS Message Statistics ======================")
            print("uINS - %d", dev.serialNumber)
            print("recorded observations: %d, " % total_obs_in_log)
            print("observations reported by firmware %d, " % obs_at_end[3])


# RMS Support Functions
def handleHeadingWrapping(ins):
    offset = 0
    newData = []
    for dIndex in range(0,len(ins.v['euler'])):
        if dIndex == 0:
            prevHeading = ins.v['euler'][0][2]
            newData.append(ins.v['euler'][0][2])
        else:
            currHeading = ins.v['euler'][dIndex][2]
            if prevHeading - currHeading > np.pi:
                offset += 1
            elif prevHeading - currHeading < -np.pi:
                offset -= 1
            prevHeading = currHeading
            newData.append((currHeading + offset*2*np.pi))

    #plt.plot(ins.v['tow'], newData)

    for dIndex in range(0,len(ins.v['euler'])):
        ins.v['euler'][dIndex][2] = newData[dIndex]
    return ins

# TODO: Compile this function into C-extension
def shiftTime(tow,dataSet,time):

    for n in range(0,len(time)):
        if time[n] > max(tow):
            time[n] = max(tow)
        elif time[n] < min(tow):
            time[n] = min(tow)

    newDataSet = np.zeros((len(time),3),dtype=np.float32)
    for sIndex in range(0,3):
        newData = np.zeros((len(time)),dtype=np.float32)
        oldData = np.zeros((len(tow)),dtype=np.float32)
        for dIndex in range(0,len(tow)):
            oldData[dIndex] = dataSet[dIndex][sIndex]

        f = interp1d(tow,oldData,kind='cubic')
        newData = f(time)

        for n in range(0,len(time)):
            newDataSet[n][sIndex] = newData[n]

    return newDataSet

def computeRmsAccuracies(log, directory, subdir):
    numDev = len(log.devices)
    averageRMS = {}
    if numDev > 1:

        print("\nComputing RMS Accuracies: (%d devices)" % (numDev))

        # Setup Data Analyics 
        # States available in ins structure - ('dataSerNum', 'week', 'tow', 'iStatus', 'hStatus', 'q', 'uvw', 'lla', 'euler')
        # Or data for each device can be called to access converted euler angles
        #   ins = itd.cINS(log.devices[i].data['ins2'])
        States = ['uvw','lla','euler']
        ins = []
        for index in range(0,numDev):
            ins.append(itd.cINS(log.devices[index].data['ins2']))

        # --------------------------------------------- Data handling -------------------------------------------- 
        plt.figure() # Test Figure

        # 1. Unwrapping Heading to handle discontinuity at -180/180
        unwrappedHeading = {}
        for devIndex in range(0,numDev):
            ins[devIndex] = handleHeadingWrapping(ins[devIndex])
            headingData = []
            time = []
            for dIndex in range(0,len(ins[devIndex].v['euler'])):
                headingData.append(ins[devIndex].v['euler'][dIndex][2])
                time.append(ins[devIndex].v['tow'][dIndex])
            plt.plot(time, headingData)

        # TODO: Consider debugging why this is required and instead of shifting the baselines consider Testing for errors
        # TODO: Compile the C Data Shift code to a C extension 
        # 2. Shift data to common time scale
        nPts = len(ins[0].v['tow'])
        time = np.zeros((nPts,1),dtype=np.float32)
        for n in range(0,nPts):
            time[n] = ins[0].v['tow'][n]

        insA = []
        totalRebaselines = 3*numDev
        rebaseline = 0;
        for stateSet in States:
            for devIndex in range(0,numDev):
                insA.append({})
                insA[devIndex][stateSet] = shiftTime(ins[devIndex].v['tow'],ins[devIndex].v[stateSet],time)
                rebaseline += 1
                print("Rebaselining - %d of %d" % (rebaseline,totalRebaselines) )

        # --------------------------------------------- Calc RMS --------------------------------------------         
        cumulativeValues = {}
        averageValues = {}
        bias = {}
        se = {}
        deviceRMS = {}
        averageRMS = {}
        for stateSet in States:

            # 1. Compute Average Values Across All Devices as Truth
            cumulativeValues[stateSet] = [[0.0 for x in range(3)] for y in range(len(insA[0][stateSet]))]
            for devIndex in range(0,numDev):
                for sIndex in range(0,3):
                    for dIndex in range(0,len(insA[0][stateSet])):
                        cumulativeValues[stateSet][dIndex][sIndex] += insA[devIndex][stateSet][dIndex][sIndex]

            averageValues[stateSet] = [[0 for x in range(3)] for y in range(len(insA[0][stateSet]))]
            for sIndex in range(0,3):
                for dIndex in range(0,len(insA[0][stateSet])):
                    averageValues[stateSet][dIndex][sIndex] = cumulativeValues[stateSet][dIndex][sIndex] / numDev # [x / myInt for x in myList]

            # 2. For Angles - Remove mounting biasses 
            bias[stateSet] = [[0 for x in range(3)] for y in range(numDev)]
            if 'euler' in stateSet:
                for devIndex in range(0,numDev):
                    for sIndex in range(0,3):
                        n = len(insA[0][stateSet])
                        cumError = 0
                        for dIndex in range(int(n/2),n-1):
                            cumError += averageValues[stateSet][dIndex][sIndex] - insA[devIndex][stateSet][dIndex][sIndex]
                        bias[stateSet][devIndex][sIndex] = cumError/(n/2-1)

            # 3. Calculate the Squared Error Values for each state in each device
            se[stateSet] = [[[0 for x in range(3)] for y in range(len(insA[0][stateSet]))] for z in range(numDev)]
            for devIndex in range(0,numDev):
                for sIndex in range(0,3):
                    for dIndex in range(0,len(insA[0][stateSet])):
                        se[stateSet][devIndex][dIndex][sIndex] = pow(averageValues[stateSet][dIndex][sIndex] - insA[devIndex][stateSet][dIndex][sIndex] - bias[stateSet][devIndex][sIndex],2)

            # 4. Calculte the RMS for each device
            deviceRMS[stateSet] = [[0 for x in range(3)] for y in range(numDev)]
            for devIndex in range(0,numDev):
                for sIndex in range(0,3):
                    n = len(se[stateSet][devIndex])
                    cumValue = 0;
                    # Only use the second half of the data for the final calculation.  Index starts at n/2
                    for dIndex in range(int(n/2),n-1):
                        cumValue += se[stateSet][devIndex][dIndex][sIndex]
                    deviceRMS[stateSet][devIndex][sIndex] = math.sqrt(cumValue/(n/2-1))

            # 5. Calculate the average RMS values across all devices
            averageRMS[stateSet] = [0 for x in range(3)]
            for sIndex in range(0,3):
                cumValue = 0;
                for devIndex in range(0,numDev):
                    cumValue += deviceRMS[stateSet][devIndex][sIndex]
                averageRMS[stateSet][sIndex]=cumValue/numDev

        # Test Code to overlay average value over heading  
        average = np.zeros(shape=(len(insA[0][stateSet]),1))
        for dIndex in range(0,len(insA[0][stateSet])):
            average[dIndex] = averageValues['euler'][dIndex][2]
        plt.plot(ins[0].v['tow'],average)

#         if not os.path.exists(os.path.join(directory,'post_processed')):
#             os.makedirs(os.path.join(directory,'post_processed'))

        # Convert LLA to NED (m)
        deviceRMS['ned'] = deviceRMS['lla']
        for devNum in range(0,numDev):
            # create lat/lon from initial lat/lon and delta
            n = len(averageValues['lla'])
            iLat = averageValues['lla'][int(n/2)][0] * DEG2RAD
            iLon = averageValues['lla'][int(n/2)][1] * DEG2RAD
            fLat = iLat + deviceRMS['lla'][devNum][0] * DEG2RAD
            fLon = iLon

            dNorth = calculateDistance(iLat,iLon,fLat,fLon)

            fLat = iLat
            fLon = iLon + deviceRMS['lla'][devNum][1] * DEG2RAD

            dEast = calculateDistance(iLat,iLon,fLat,fLon)

            deviceRMS['ned'][devNum][0] = dNorth
            deviceRMS['ned'][devNum][1] = dEast
            deviceRMS['ned'][devNum][2] = -deviceRMS['lla'][devNum][2]

        n = len(averageValues['lla'])
        iLat = averageValues['lla'][int(n/2)][0] * DEG2RAD
        iLon = averageValues['lla'][int(n/2)][1] * DEG2RAD
        fLat = iLat + averageRMS['lla'][0] * DEG2RAD
        fLon = iLon

        dNorth = calculateDistance(iLat,iLon,fLat,fLon)

        fLat = iLat
        fLon = iLon + averageRMS['lla'][1] * DEG2RAD

        dEast = calculateDistance(iLat,iLon,fLat,fLon)

        averageRMS['ned'] = averageRMS['lla']
        averageRMS['ned'][0] = dNorth
        averageRMS['ned'][1] = dEast
        averageRMS['ned'][2] = -averageRMS['lla'][2]

        navMode = ins[0].iStatus().navMode[-1]

        #==================================
        # Specification and Tolerance
        tolerance = 0.9
        specThresh = {}
        specThresh['ned'] = np.array([0.2,0.2,0.2])
        specThresh['uvw'] = np.array([0.1,0.1,0.1])
        specThresh['euler'] = np.array([0.1,0.1,0.0])
        if navMode: # NAV heading
            specThresh['euler'][2] = 0.3
        else:       # AHRS heading
            specThresh['euler'][2] = 2.0

        specThresh['euler'] = tolerance * specThresh['euler']
        specThresh['ned']   = tolerance * specThresh['ned']
        specThresh['uvw']   = tolerance * specThresh['uvw']

        sigTest = {}
        sigTest['euler']    = ['NA  ','NA  ','NA  ']
        sigTest['ned']      = ['NA  ','NA  ','NA  ']
        sigTest['uvw']      = ['NA  ','NA  ','NA  ']
        for i in range(3):
            sigTest['euler'][i] = 'PASS' if averageRMS['euler'][i]*RAD2DEG < specThresh['euler'][i] else 'FAIL'
            if navMode: # Nav
                sigTest['ned'][i] = 'PASS' if averageRMS['ned'][i] < specThresh['ned'][i] else 'FAIL'
                sigTest['uvw'][i] = 'PASS' if averageRMS['uvw'][i] < specThresh['uvw'][i] else 'FAIL'

        filename = os.path.join(directory,'RMS_report.txt');
        f = open(filename, 'w')
        f.write(    '*****   Performance Analysis Report - %s   *****\n' % (subdir))
        f.write(    '\n');
        f.write(    'Directory: %s\n' % (directory))
        f.write(    '\n');

        # Print Table of RMS accuracies
        line =      'Device       '
        if navMode:
            f.write(    '--------------------------------------------------- RMS Accuracy -------------------------------------------\n')
            line = line + 'UVW[  (m/s)   (m/s)   (m/s) ],  NED[    (m)     (m)     (m) ],'
        else:   # AHRS mode
            f.write(    '-------------- RMS Accuracy --------------\n')
        line = line +   '  Eul[  (deg)   (deg)   (deg) ]\n'
        f.write(line)

        for n in range(0,numDev):
            devInfo = itd.cDevInfo(log.devices[n].data['devInfo'])
            line = '%2d SN%d      ' % (n, devInfo.v['serialNumber'][-1])
            if navMode:
                line    = line + '[%7.4f %7.4f %7.4f ],     ' % (deviceRMS['uvw'][n][0],deviceRMS['uvw'][n][1],deviceRMS['uvw'][n][2])
                line    = line + '[%7.4f %7.4f %7.4f ],     ' % (deviceRMS['ned'][n][0],deviceRMS['ned'][n][1],deviceRMS['ned'][n][2])
            line        = line + '[%7.4f %7.4f %7.4f ]\n' % (deviceRMS['euler'][n][0]*RAD2DEG,deviceRMS['euler'][n][1]*RAD2DEG,deviceRMS['euler'][n][2]*RAD2DEG)
            f.write(line)

        line = 'AVERAGE:        '
        if navMode:
            f.write(    '------------------------------------------------------------------------------------------------------------\n')
            line        = line + '[%7.4f %7.4f %7.4f ],     ' % (averageRMS['uvw'][0],averageRMS['uvw'][1],averageRMS['uvw'][2])
            line        = line + '[%7.4f %7.4f %7.4f ],     ' % (averageRMS['ned'][0],averageRMS['ned'][1],averageRMS['ned'][2])
        else:   # AHRS mode
            f.write(    '------------------------------------------\n')
        line            = line + '[%7.4f %7.4f %7.4f ]\n' % (averageRMS['euler'][0]*RAD2DEG,averageRMS['euler'][1]*RAD2DEG,averageRMS['euler'][2]*RAD2DEG)
        f.write(line)

        line = 'THRESHOLD:      '
        if navMode:
            line        = line + '[%7.4f %7.4f %7.4f ],     ' % (specThresh['uvw'][0],specThresh['uvw'][1],specThresh['uvw'][2])
            line        = line + '[%7.4f %7.4f %7.4f ],     ' % (specThresh['ned'][0],specThresh['ned'][1],specThresh['ned'][2])
        line            = line + '[%7.4f %7.4f %7.4f ]\n' % (specThresh['euler'][0],specThresh['euler'][1],specThresh['euler'][2])
        f.write(line)

        line = 'PASS/FAIL:      '
        if navMode:
            line        = line + '[   %s    %s    %s ],     ' % (sigTest['uvw'][0],sigTest['uvw'][1],sigTest['uvw'][2])
            line        = line + '[   %s    %s    %s ],     ' % (sigTest['ned'][0],sigTest['ned'][1],sigTest['ned'][2])
        line            = line + '[   %s    %s    %s ]\n' % (sigTest['euler'][0],sigTest['euler'][1],sigTest['euler'][2])
        f.write(line)

        if navMode:
            f.write('                                                                                         ')
        else:   # AHRS mode
            f.write('                  ')
        f.write('(NAV mode)\n\n' if ins[0].iStatus().navMode[-1] else '(AHRS mode)\n\n')

        # Print Mounting Biases        
        f.write(    '--------------- Angular Mounting Biases ----------------\n')
        f.write(    'Device       Euler Biases[   (deg)     (deg)     (deg) ]\n')
        for n in range(0,numDev):
            devInfo = itd.cDevInfo(log.devices[n].data['devInfo'])
            f.write('%2d SN%d               [ %7.4f   %7.4f   %7.4f ]\n' % (n, devInfo.v['serialNumber'][-1],bias['euler'][n][0]*RAD2DEG,bias['euler'][n][1]*RAD2DEG,bias['euler'][n][2]*RAD2DEG))
        f.write('\n')

        # Print Device Version Information
        f.write(    '------------------------------------------- Device Info -------------------------------------------------\n')
        for n in range(0,numDev):
            devInfo = itd.cDevInfo(log.devices[n].data['devInfo'])
            hver = devInfo.v['hardwareVer'][-1]
            cver = devInfo.v['commVer'][-1]
            fver = devInfo.v['firmwareVer'][-1]
            buld = devInfo.v['build'][-1]
            repo = devInfo.v['repoRevision'][-1]
            date = devInfo.v['buildDate'][-1]
            time = devInfo.v['buildTime'][-1]
            addi = devInfo.v['addInfo'][-1]
            f.write('%2d SN%d  HW: %d.%d.%d.%d   FW: %d.%d.%d.%d build %d repo %d   Proto: %d.%d.%d.%d  Date: %04d-%02d-%02d %02d:%02d:%02d  %s\n' %(
                n, devInfo.v['serialNumber'][-1],
                hver[3], hver[2], hver[1], hver[0],
                fver[3], fver[2], fver[1], fver[0], buld, repo,
                cver[3], cver[2], cver[1], cver[0],
                2000+date[2], date[1], date[0],
                time[3], time[2], time[1],
                addi ) )
        f.write('\n')

        f.close()

        # Automatically open report in Windows
        if 'win' in sys.platform:
            subprocess.Popen(["notepad.exe", filename])    # non-blocking call
        else:
            subprocess.Popen(["gedit", filename])  # non-blocking call


    print("Done.")

    # TODO: Pass out the union of the test errors
    return averageRMS

