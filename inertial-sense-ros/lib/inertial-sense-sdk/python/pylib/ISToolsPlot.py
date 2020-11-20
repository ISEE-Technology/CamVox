import numpy as np
import os
# import sys
import simplekml
# import pdb

import pylib.pose as ps
import pylib.ISToolsDataSorted as itd
import pylib.filterTools as ft
import pylib.plotTools as plotTools
import matplotlib.pyplot as plt
import matplotlib
import datetime

import time as systime
from pylib.ISToolsDataSorted import cObj, refLla, getTimeFromTowMs, getTimeFromTow
from pylib import pose

RAD2DEG = 180/np.pi
DEG2RAD = np.pi/180


pt = plotTools.cPlot()
pt.showUtcTime = 0        # 1 = UTC Time, 2 = decimal seconds, 0 = time starts at zero seconds (NOT UTC)

g_imu1BiasPqr = []
g_imu2BiasPqr = []
g_imu1BiasAcc = []
g_imu2BiasAcc = []
timesCalled = 0
serialNumbers = []

def log2Data(log, dataClass, key):
    if key in log.data.keys():
        d = dataClass(log.data[key])        
        return d;
    else:
        return 0;


def IsLoggerPlot( pe, log, tru=None, startFigure=None, referencePlot=False, saveFigs=False, saveFigsDirectory='', numDevs=None ):
    global pt
    global g_imu1BiasPqr
    global g_imu2BiasPqr
    global g_imu1BiasAcc
    global g_imu2BiasAcc
    global timesCalled
    global serialNumbers
    timesCalled += 1
    try:
        serialNumbers.append(log.data['devInfo']['serialNumber'][0])
    except:
        print ("something is wrong")

    def saveFigures(filename, f=None, fig=None):
        if saveFigs:
            if fig is None:
                if f is None:
                    fig = plt.gcf()
                else:
                    fig = plt.figure(f)
            fsize = fig.get_size_inches()
            # fig.set_size_inches(16,16)
            fig.set_size_inches(20,20)
            fig.savefig(os.path.join(saveFigsDirectory,filename), bbox_inches='tight')
            fig.set_size_inches(fsize)

    def peCheck(key):
        return key in pe and pe[key]

    if pe['showUtcTime']:
        pt.timeIsUtc = pe['showUtcTime']
        itd.setShowUtcTime(1)

        # Print all data structures and their variables
    print("===============  IsLoggerPlot() - datalog keys ===============")
    for key in log.data.keys():
        try:
            print(key, log.data[key].dtype.names)
        except:
            pass


    key = 'ins2'
    if not key in log.data.keys():      # Use whichever ins is available
        key = 'ins1'
        if not key in log.data.keys():
            key = 'insDev1'
            if not key in log.data.keys():
                print("INS data not found!!!")
                return
    print("==============================================================")

    ins = log2Data( log, itd.cINS, key )
    log.ins = ins
    if ins:
        itd.setGpsWeek( ins.v['week'] )

    gps1Pos = log2Data( log, itd.cGPS, 'gps1Pos' )
    # gps1Vel = log2Data( log, itd.cGPS, 'gps1Vel' )
    gps1Ubx = log2Data( log, itd.cGPS, 'gps1UbxPos' )
    gps1RtkPos = log2Data( log, itd.cGPS, 'gps1RtkPos' )
    gps1Raw = log2Data( log, itd.cGPSRaw, 'GPS1Raw')
    gps2Raw = log2Data( log, itd.cGPSRaw, 'GPS2Raw')
    gpsBaseRaw = log2Data( log, itd.cGPSRaw, 'GPSBaseRaw')
    imu1 = log2Data( log, itd.cIMU, 'imu1' )
    dimu = log2Data( log, itd.cIMU, 'dualImu' )
    dimu = log2Data( log, itd.cIMU, 'preintegratedImu' )
    mag1 = log2Data( log, itd.cIMU, 'magnetometer1' )
    mag2 = log2Data( log, itd.cIMU, 'magnetometer2' )
    magInfo = log2Data( log, itd.cSIMPLE, 'inl2MagObs')
    varInfo = log2Data( log, itd.cSIMPLE, 'inl2Variance')
    baro = log2Data( log, itd.cIMU, 'barometer' )

    print("Timestamps:")
    if ins:
        gmt1 = systime.gmtime( ins.time[0]  - systime.timezone )
        gmt2 = systime.gmtime( ins.time[-1] - systime.timezone )
        dgmt = systime.gmtime( ins.time[-1] - ins.time[0] )
        print("   INS:  %d-%d-%d %d:%d:%d - %d:%d:%d  (%d:%d:%d)" % (gmt1.tm_year, gmt1.tm_mon, gmt1.tm_mday,
                                                                     gmt1.tm_hour, gmt1.tm_min, gmt1.tm_sec,
                                                                     gmt2.tm_hour, gmt2.tm_min, gmt2.tm_sec,
                                                                     dgmt.tm_hour, dgmt.tm_min, dgmt.tm_sec))
    if imu1:
        gmt1 = systime.gmtime( imu1.time[0]  - systime.timezone )
        gmt2 = systime.gmtime( imu1.time[-1] - systime.timezone )
        dgmt = systime.gmtime( imu1.time[-1] - imu1.time[0] )
        print("   IMU:  %d-%d-%d %d:%d:%d - %d:%d:%d  (%d:%d:%d)" % (gmt1.tm_year, gmt1.tm_mon, gmt1.tm_mday,
                                                                     gmt1.tm_hour, gmt1.tm_min, gmt1.tm_sec,
                                                                     gmt2.tm_hour, gmt2.tm_min, gmt2.tm_sec,
                                                                     dgmt.tm_hour, dgmt.tm_min, dgmt.tm_sec))
    if gps1Pos:
        gmt1 = systime.gmtime( gps1Pos.time[0]  - systime.timezone )
        gmt2 = systime.gmtime( gps1Pos.time[-1] - systime.timezone )
        dgmt = systime.gmtime( gps1Pos.time[-1] - gps1Pos.time[0] )
        print("   GPS:  %d-%d-%d %d:%d:%d - %d:%d:%d  (%d:%d:%d)" % (gmt1.tm_year, gmt1.tm_mon, gmt1.tm_mday,
                                                                     gmt1.tm_hour, gmt1.tm_min, gmt1.tm_sec,
                                                                     gmt2.tm_hour, gmt2.tm_min, gmt2.tm_sec,
                                                                     dgmt.tm_hour, dgmt.tm_min, dgmt.tm_sec))

    sysp = log2Data( log, itd.cSysParams, 'sysParams' )
    sysp = log2Data( log, itd.cSysParams, 'ins2' )
    if sysp == 0:
        sysp = log2Data( log, itd.cSysParams, 'sysParams' )
    ires = log2Data( log, itd.cInsRes, 'insResources')
    info = log2Data( log, itd.cDevInfo, 'devInfo')

    if info:
        print('Index:', log.index, '  Device S/N: ',info.v['serialNumber'][-1])
        hver = info.v['hardwareVer'][-1]
        cver = info.v['commVer'][-1]
        fver = info.v['firmwareVer'][-1]
        print('  Hardware: %d.%d.%d.%d   Com: %d.%d.%d.%d   Firmware: %d.%d.%d.%d  b%d  r%d' %( hver[0], hver[1], hver[2], hver[3], cver[0], cver[1], cver[2], cver[3], fver[0], fver[1], fver[2], fver[3], info.v['build'][-1], info.v['repoRevision'][-1] ))
    else:
        print('Index:', log.index)

    if 'debugArray' in log.data.keys():
        pe['dbg'] = 1
        dbg = log2Data( log, itd.cSIMPLE, 'debugArray')
        dbg.time = dbg.v['i'][:,0]*0.001
    else:
        pe['dbg'] = 0
        dbg = 0

    if tru:
        biasPqr = np.r_[ 0.0, 0.44, -0.15 ]*DEG2RAD
        biasAcc = np.r_[ -0.038, 0.085, 0.065 ]
        rot     = np.r_[ 0.32, 0.18, 0.5 ]*DEG2RAD

        rImu = itd.cRIMU(tru.data['IMU'])
        rGps = itd.cRGPS(tru.data['GPS'])
        rIns = itd.cRINS(tru.data['INS'])

        print("RINS keys:", rIns.v.dtype.names)
        print("RIMU keys:", rImu.v.dtype.names)
        print("RGPS keys:", rGps.v.dtype.names)
    else:
        rImu = 0
        rGps = 0
        rIns = 0

    #     if imu1:
    #         print("IMU keys:", imu1.v.dtype.names)
    #     if dimu:
    #         print("Dual IMU keys:", dimu.v.dtype.names)
    #     if mag1:
    #         print("Mag keys:", mag1.v.dtype.names)
    #     if mag2:
    #         print("Mag keys:", mag2.v.dtype.names)
    #     if baro:
    #         print("Baro keys:", baro.v.dtype.names)
    #     if gps1Pos:
    #         print("GPS keys:", gps1Pos.v.dtype.names)
    #     if ins:
    #         print("INS keys:", ins.v.dtype.names)
    #     if sysp:
    #         print("SPR keys:", sysp.v.dtype.names)

    # Debug.  Print all of numpy array
    np.set_printoptions(threshold='nan')


    # Start time at zero seconds.  Don't use UTC time.
    meanTowOffset = 0
    if pt.timeIsUtc == 0:
        startTow = ins.time[0]
        ins.time = ins.time - startTow
        if gps1Pos!=0:
            meanTowOffset = np.mean(gps1Pos.v['towOffset'])

        if gps1Pos:
            gps1Pos.time = gps1Pos.time - startTow
        if gps1Ubx:
            gps1Ubx.time = gps1Ubx.time - startTow
        if gps1RtkPos:
            gps1RtkPos.time = gps1RtkPos.time - startTow

        if sysp:
            sysp.time           = sysp.time - startTow
        if ires and 'time' in ires.v:
            ires.time = ires.time - startTow
        if imu1:
            if gps1Pos!=0:
                imu1.time = imu1.time + meanTowOffset - startTow
            else:
                imu1.time = imu1.time - imu1.time[0]

        if dimu:
            if gps1Pos!=0:
                dimu.time = dimu.time + meanTowOffset - startTow
            else:
                dimu.time = dimu.time - dimu.time[0]

        if mag1:
            mag1.time = mag1.time + meanTowOffset - startTow
        if mag2:
            mag2.time = mag2.time + meanTowOffset - startTow
        if baro:
            baro.time = baro.time + meanTowOffset - startTow

        if rIns:
            rIns.v['tow'] = rIns.v['tow'] - startTow
            rImu.v['time'] = rImu.v['time'] - startTow
            rGps.v['tow'] = rGps.v['tow'] - startTow

        if dbg:
            dbg.time = dbg.time - startTow

        if 'ekfStates' in log.data:
            if gps1Pos!=0:
                log.data['ekfStates']['time'] = log.data['ekfStates']['time'] + meanTowOffset - startTow
            else:
                log.data['ekfStates']['time'] = log.data['ekfStates']['time'] - log.data['ekfStates']['time'][0]

                # Fix data
    if 'ned' in ins.v.dtype.names:
        ins.v['ned'] = ins.v['ned'] - ins.v['ned'][0,:]


    ######################################################################
    # Specs
    ######################################################################
    if 'eulerErr' in ins.v.dtype.names and rIns:
        #     start = 0                               # All of data
        start = np.shape(ins.time)[0]/2    # Use last half of data
        print("   ==========   STATISTICS   ==========   ")

        # RMS Accuracy - Attitude
        insEuler = ins.v['eulerErr'][start:,:]
        rIns.offset = np.r_[np.mean(insEuler[:,0]),
                            np.mean(insEuler[:,1]),
                            np.mean(insEuler[:,2])]
        eulerErr = np.c_[insEuler[:,0] - rIns.offset[0],
                         insEuler[:,1] - rIns.offset[1],
                         insEuler[:,2] - rIns.offset[2]]
        ins.eulerErrRms = np.sqrt( np.r_[np.mean(eulerErr[:,0]**2),
                                         np.mean(eulerErr[:,1]**2),
                                         np.mean(eulerErr[:,2]**2)] )
        print("Euler Angles (deg):")
        print("  Offset from Truth:       \t", rIns.offset*RAD2DEG)
        print("  RMS Accuracy:            \t", ins.eulerErrRms*RAD2DEG)


        # RMS Accuracy - Velocity
        uvwErr = ins.v['uvwErr'][start:,:]
        ins.uvwErrRms = np.sqrt( np.r_[np.mean(uvwErr[:,0]**2),
                                       np.mean(uvwErr[:,1]**2),
                                       np.mean(uvwErr[:,2]**2)] )
        print("UVW RMS Accuracy (m/s):    \t", ins.uvwErrRms)


        # RMS Accuracy - Position
        nedErr = ins.v['nedErr'][start:,:]
        ins.nedErrRms = np.sqrt( np.r_[np.mean(nedErr[:,0]**2),
                                       np.mean(nedErr[:,1]**2),
                                       np.mean(nedErr[:,2]**2)] )
        print("NED RMS Accuracy (m):      \t", ins.nedErrRms)

    if peCheck('rawGPSStats'):
        if gps1Raw is not 0:
            print("GPS 1 Corrupt Data Count", gps1Raw.corruptCount, "/", len(gps1Raw.count))
        if gps2Raw is not 0:
            print("GPS 2 Corrupt Data Count", gps2Raw.corruptCount, "/", len(gps2Raw.count))
        if gpsBaseRaw is not 0:
            print("GPS Base Corrupt Data Count", gpsBaseRaw.corruptCount, "/", len(gpsBaseRaw.count))


    ######################################################################
    # Plot Figures
    ######################################################################
    insColor = 'b'
    refColor = 'r'
    rtkColor = 'c'
    ubxColor = 'y'
    rInsColor = 'k'
    rGpsColor = 'm'
    errColor = 'm'
    magColor = 'g'
    barColor = 'c'

    if referencePlot:
        insColor = 'c'

    f = 0
    if startFigure!=None:
        f = startFigure

    # Create list of serial numbers
    IsLoggerPlot.serialNumbers += [log.serialNumber]

    pt.setPreTitle("%s__%s" % (os.path.basename(saveFigsDirectory), log.serialNumber))

    ###################################################
    # Temperature
    #
    if peCheck('temp'):
        f += 1
        if not  'sysParams' in log.data.keys():
            print("Unable to plot sysParams (because it's not there)")
        else:
            sysP = log.data['sysParams']
            time = (sysP['towMs'] - sysP['towMs'][0])/1000.0
            fig, ax = pt.subplots(f, 2, 'Temperature')
            pt.subplotSingle(ax[0], getTimeFromTowMs(sysP['towMs']), sysP['imuTemp'], title='Temperature: IMU')
            pt.subplotSingle(ax[1], getTimeFromTowMs(sysP['towMs']), sysP['baroTemp'], title='Temperature: Baro')
            saveFigures('Temperature', f)


    ###################################################
    # Debug Array
    #
    if peCheck('debugArray_i'):
        f += 1
        if dbg == 0:
            print("Unable to print debug array - messages missing")
        else:
            fig, ax = pt.subplots(f, 1, 'debugArray_i', sharex=True)
            for j in range(9):
                pt.subplotSingle(ax, dbg.time, dbg.v['i'][:,j], title= 'i', options={'label': j})
            plt.legend()
            saveFigures('debugArray_i.svg', f)
    if peCheck('debugArray_f') and dbg:
        f += 1
        if dbg == 0:
            print("Unable to print debug array - messages missing")
        else:
            fig, ax = pt.subplots(f, 1, 'debugArray_f', sharex=True)
            for j in range(9):
                pt.subplotSingle(ax, dbg.time, dbg.v['f'][:,j], title= 'f', options={'label': j})
            plt.legend()
            saveFigures('debugArray_f.svg', f)
    if peCheck('debugArray_lf') and dbg:
        f += 1
        if dbg == 0:
            print("Unable to print debug array - messages missing")
        else:
            fig, ax = pt.subplots(f, 1, 'debugArray_lf', sharex=True)
            for j in range(3):
                pt.subplotSingle(ax, dbg.time, dbg.v['lf'][:,j], title ='lf', options={'label': j})
            plt.legend()
            saveFigures('debugArray_lf.svg', f)

    ###################################################
    # Raw Base GPS
    #
    def inliers(data, m=4):
        d = np.abs(data - np.median(data))
        mdev = np.median(d)
        s = d / mdev if mdev else 0.
        return s < m

    def plotSNR(raw_gps, f, name):
        f += 1
        if raw_gps is None:
            print("unable to plot " + name + "SNR because data is missing")
            return
        satellites = np.unique(raw_gps.obs['sat'])
        fig, ax = pt.subplots(f, len(satellites), 'raw' + name + 'SNR', sharex=True)

        for i, sat in enumerate(satellites):
            valid_indexes = (raw_gps.obs['sat'] == sat)
            pt.subplotSingle(ax[i], raw_gps.obstime[valid_indexes].astype(datetime.datetime), raw_gps.obs['sat'][valid_indexes], title="sat " + str(sat), options='x')
        saveFigures('raw' + name + 'SNR.svg', f)
        return f

    def plotPrange(raw_gps, f, name):
        f += 1
        if raw_gps is None:
            print("unable to plot " + name + "Prange because data is missing")
            return
        satellites = np.unique(raw_gps.obs['sat'])
        fig, ax = pt.subplots(f, len(satellites), 'raw' + name + 'Prange', sharex=True)

        for i, sat in enumerate(satellites):
            valid_indexes = raw_gps.obs['sat'] == sat
            try:
                pt.subplotSingle(ax[i], raw_gps.obstime[valid_indexes].astype(datetime.datetime), raw_gps.obs['P'][valid_indexes], title="sat " + str(sat), options='x')
            except:
                pass
        saveFigures('raw' + name + 'Prange.svg', f)
        return f

    def plotCPhase(raw_gps, f, name):
        f += 1
        if raw_gps is None:
            print("unable to plot " + name + "Cphase because data is missing")
            return
        satellites = np.unique(raw_gps.obs['sat'])
        fig, ax = pt.subplots(f, len(satellites), 'raw' + name + 'CarrierPhase', sharex=True)

        for i, sat in enumerate(satellites):
            valid_indexes = (raw_gps.obs['sat'] == sat) & (raw_gps.obs['L'] != 0)
            try:
                pt.subplotSingle(ax[i], raw_gps.obstime[valid_indexes].astype(datetime.datetime), raw_gps.obs['L'][valid_indexes],
                             title="sat " + str(sat), options='x')
            except:
                print("problem plotting cphase")
        saveFigures('raw' + name + 'CarrierPhase.svg', f)
        return f

    if peCheck('rawBaseGpsSNR') and gpsBaseRaw:
        f = plotSNR(gpsBaseRaw, f, 'BaseRaw')

    if peCheck('rawBaseGpsPrange') and gpsBaseRaw:
        f = plotPrange(gpsBaseRaw, f, 'BaseRaw')

    if peCheck('rawBaseCarrierPhase') and gpsBaseRaw:
        f = plotCPhase(gpsBaseRaw, f, 'BaseRaw')

    ###################################################
    # Raw GPS1
    #
    if peCheck('rawGps1SNR') and gps1Raw:
        f = plotSNR(gps1Raw, f, 'Gps1Raw')

    if peCheck('rawGps1Prange') and gps1Raw:
        f = plotPrange(gps1Raw, f, 'Gps1Raw')

    if peCheck('rawGps1CarrierPhase') and gps1Raw:
        f = plotCPhase(gps1Raw, f, 'Gps1Raw')

    ###################################################
    # Raw GPS1
    #
    if peCheck('rawGps2SNR') and gps2Raw:
        f = plotSNR(gps2Raw, f, 'GPS2Raw')

    if peCheck('rawGps2Prange') and gps2Raw:
        f = plotPrange(gps2Raw, f, 'GPS2Raw')

    if peCheck('rawGps2CarrierPhase') and gps2Raw:
        f = plotCPhase(gps2Raw, f, 'GPS2Raw')

    #############################################
    # Compassing Angle
    if peCheck('compassingAngle'):
        f += 1
        if 'gps1RtkCmpRel' not in log.data.keys():
            print("no gps1RtkCmpRel, cannot plot compassingAngle")
        else:
            legend = []
            fig, ax = pt.subplots(f, 2, 'Heading', sharex=True)

            rtkRel = log.data['gps1RtkCmpRel']

            pt.subplotSingle(ax[0], getTimeFromTowMs(rtkRel['timeOfWeekMs']), rtkRel['headingToBase'] * RAD2DEG, title="RTK")
            pt.subplotSingle(ax[1], getTimeFromTow(ins.v['tow']), ins.v['euler'][:, 2] * RAD2DEG, title="INS")
            saveFigures('compassingAngle.svg', f)

    #############################################
    # VectorToBase
    if peCheck('vectorToBase'):
        #         pt.labels('NED', 'm')
        f += 1
        if 'gps1RtkCmpRel' not in log.data.keys():
            print("no gps1RtkCmpRel, cannot plot vectorToBase")
        else:
            legend = []
            fig, ax = pt.subplots(f, 4, 'vectorToBase', sharex=True)

            rtkRel = log.data['gps1RtkCmpRel']

            # vec2basefix = rtkRel['vectorToBase'].copy()
            # vec2basefloat = rtkRel['vectorToBase'].copy()
            # vec2basesingle = rtkRel['vectorToBase'].copy()
            # dist2basefix = rtkRel['distanceToBase'].copy()
            # dist2basefloat = rtkRel['distanceToBase'].copy()
            # dist2basesingle = rtkRel['distanceToBase'].copy()
            #
            # vec2basefix[rtkRel['arRatio'] < 3.0, :] = np.nan
            # vec2basefloat[(rtkRel['arRatio'] < 0.0) | (rtkRel['arRatio'] > 3.0), :] = np.nan
            # vec2basesingle[rtkRel['arRatio'] > 0.001, :] = np.nan
            # dist2basefix[rtkRel['arRatio'] < 3.0] = np.nan
            # dist2basefloat[(rtkRel['arRatio'] < 0.0) | (rtkRel['arRatio'] > 3.0)] = np.nan
            # dist2basesingle[rtkRel['arRatio'] > 0.001] = np.nan

            time = getTimeFromTowMs(rtkRel['timeOfWeekMs'])

            cmap = matplotlib.cm.get_cmap('Spectral')
            colors = [cmap(i/numDevs) for i in range(numDevs)]

            titles = ["North", "East", "Down"]
            for i in range(3):
                pt.subplotSingle(ax[i], time, rtkRel['vectorToBase'][:,i],title=titles[i], ylabel='m')
                # pt.subplotSingle(ax[i], time, vec2basefix[:,i],title=titles[i], ylabel='m')
                # pt.subplotSingle(ax[i], time, vec2basefloat[:, i], ylabel='m')
                # pt.subplotSingle(ax[i], time, vec2basesingle[:, i], ylabel='m')
                if i == 0:
                    ax[0].legend([str(ser) for ser in serialNumbers])

            pt.subplotSingle(ax[3], time, rtkRel['distanceToBase']
                             , title="DistanceToBase", ylabel='m')
            # pt.subplotSingle(ax[3], time, dist2basefix, title="DistanceToBase", ylabel='m')
            # pt.subplotSingle(ax[3], time, dist2basesingle)
            # pt.subplotSingle(ax[3], time, dist2basefloat)
            saveFigures('vectorToBase.svg', f)

    #############################################
    # VectorToBase
    if peCheck('compassingCircle'):
        f += 1
        if 'gps1RtkCmpRel' not in log.data.keys():
            print("no RTK_Rel, cannot plot compassingCircle")
        else:
            legend = []

            rtkRel = log.data['gps1RtkCmpRel']

            vec2basefix = rtkRel['vectorToBase'].copy()
            vec2basefloat = rtkRel['vectorToBase'].copy()
            vec2basesingle = rtkRel['vectorToBase'].copy()

            vec2basefix[rtkRel['arRatio'] < 3.0, :] = np.nan
            vec2basefloat[(rtkRel['arRatio'] < 0.0) | (rtkRel['arRatio'] > 3.0), :] = np.nan
            vec2basesingle[rtkRel['arRatio'] > 0.001, :] = np.nan

            fig, ax = pt.subplots(f, 2, 'compassingCircle', numCols=2)
            pt.subplotSingle(ax[0,0], vec2basefix[:, 0], vec2basefix[:, 1], options='m')
            pt.subplotSingle(ax[0,0], vec2basefloat[:, 0], vec2basefloat[:, 1], options='b')
            pt.subplotSingle(ax[0,0], vec2basesingle[:, 0], vec2basesingle[:, 1], options='g')
            plt.axis('equal')
            plt.xlabel("ECEF x")
            plt.xlabel("ECEF y")

            pt.subplotSingle(ax[1,0], vec2basefix[:, 1], vec2basefix[:, 2], options='m')
            pt.subplotSingle(ax[1,0], vec2basefloat[:, 1], vec2basefloat[:, 2], options='b')
            pt.subplotSingle(ax[1,0], vec2basesingle[:, 1], vec2basesingle[:, 2], options='g')
            plt.axis('equal')
            plt.xlabel("ECEF y")
            plt.xlabel("ECEF z")

            pt.subplotSingle(ax[0,1], vec2basefix[:, 0], vec2basefix[:, 2], options='m')
            pt.subplotSingle(ax[0,1], vec2basefloat[:, 0], vec2basefloat[:, 2], options='b')
            pt.subplotSingle(ax[0,1], vec2basesingle[:, 0], vec2basesingle[:, 2], options='g')
            plt.xlabel("ECEF x")
            plt.xlabel("ECEF z")

            ax[0,1].legend(['fix', 'float', 'single'])
            plt.axis('equal')
            saveFigures('compassingCircle.svg', f)

    if peCheck('compassingCircle3d'):
        f += 1
        if 'gps1RtkCmpRel' not in log.data.keys():
            print("no RTK_Rel, cannot plot compassingCircle")
        else:
            vec2basefix = rtkRel['vectorToBase'].copy()
            vec2basefloat = rtkRel['vectorToBase'].copy()
            vec2basesingle = rtkRel['vectorToBase'].copy()

            vec2basefix[rtkRel['arRatio'] < 3.0, :] = np.nan
            vec2basefloat[(rtkRel['arRatio'] < 0.0) | (rtkRel['arRatio'] > 3.0), :] = np.nan
            vec2basesingle[rtkRel['arRatio'] > 0.001, :] = np.nan

            fig, ax = pt.plot3D(f, 'compassingCircle3d')
            ax.plot(vec2basefix[:, 0], vec2basefix[:, 1], vec2basefix[:,2], 'm')
            ax.plot(vec2basefloat[:, 0], vec2basefloat[:, 1], vec2basefloat[:,2], 'b')
            ax.plot(vec2basesingle[:, 0], vec2basesingle[:, 1], vec2basesingle[:,2], 'g')
            ax.legend(['fix', 'float', 'single'])
            plt.axis('equal')
            saveFigures('compassingCircle3d.svg', f)

            u, v = np.mgrid[0:2 * np.pi:180j, 0:np.pi:180j]
            x = np.cos(u) * np.sin(v)
            y = np.sin(u) * np.sin(v)
            z = np.cos(v)
            ax.plot_wireframe(x, y, z, color="k", alpha=0.1, linewidth=1.0)

        #############################################
        # Google Earth plot LLA
    if peCheck('googleEarth'):
        tStep = 1

        if pe['googleEarth'] == 2:
            altMode="absolute"
        else:
            altMode="clampToGround"

        # kmlfile = log.directory + "/" + itd.lla2kml(ins.time[insTCnt:], ins.v['lla'][insTCnt:,:], log.serialNumber, "LOG_"+log.serialNumber+"ins.kml", timeStep=tStep, altitudeMode=altMode )
        # kmlfile = log.directory + "/" + itd.lla2kml(ins.time, ins.v['lla'], log.serialNumber, "LOG_"+log.serialNumber+"ins.kml", timeStep=tStep, altitudeMode=altMode )
        kmlfile = itd.lla2kml(ins.time, ins.v['lla'], log.serialNumber, log.directory + "/" + "LOG_"+log.serialNumber+"ins.kml", timeStep=tStep, altitudeMode=altMode )
        os.startfile( kmlfile )
        if gps1Pos:
            # kmlfile = log.directory + "/" + itd.lla2kml(gps1Pos.time, gps1Pos.v['lla'], log.serialNumber, "LOG_"+log.serialNumber+"gps1Pos.kml", timeStep=tStep, altitudeMode=altMode, color=simplekml.Color.red )
            kmlfile = itd.lla2kml(gps1Pos.time, gps1Pos.v['lla'], log.serialNumber, log.directory + "/" + "LOG_"+log.serialNumber+"gps1Pos.kml", timeStep=tStep, altitudeMode=altMode, color=simplekml.Color.red )
            os.startfile( kmlfile )
        if rIns:
            # kmlfile = log.directory +  "/" + itd.lla2kml(rIns.v['time'], rIns.v['lla'], tru.serialNumber, "LOG_"+tru.serialNumber+"refIns.kml", timeStep=tStep, altitudeMode=altMode, color=simplekml.Color.black )
            kmlfile = itd.lla2kml(rIns.v['time'], rIns.v['lla'], tru.serialNumber, log.directory +  "/" + "LOG_"+tru.serialNumber+"refIns.kml", timeStep=tStep, altitudeMode=altMode, color=simplekml.Color.black )
            os.startfile( kmlfile )

        if pe['googleEarth'] == 2:
            return



    #############################################
    # dIMU Time
    if dimu and 0:
        pt.labels('IMU Time', 'ms')
        f += 1;    legend = []
        fig, ax = pt.subplots(f,1,"IMU time")
        timeCnt = len(dimu.time)
        dimuTime = []
        counter = []
        for i in range(0,timeCnt):
            counter.append(i)
            dimuTime.append(np.float64(dimu.time[i]).item())

        pt.subplotSingle(ax, counter, dimu.v['time'])
        plt.ylim( -1, 10000 )

        #############################################
        # Body Velocity
    if peCheck('uvw'):
        pt.labels('UVW', 'm/s')
        f += 1;    legend = []
        if rIns:
            pt.plot3Axes(f, rIns.v['time'], rIns.uvw, options=rInsColor)
            legend += ['Truth']
        pt.plot3Axes(f, getTimeFromTow(ins.v['tow']), ins.v['uvw'], options=insColor)
        legend += ['INS']
        plt.legend(legend)

        if peCheck('uvwDot'):
            f += 1;    legend = []

            if rIns:
                pt.plot3Axes(f, rIns.v['time'], ft.smooth(ft.derivative(rIns.v['time'], rIns.uvw, delta=2), delta=10), options=rInsColor)
                legend += ['Truth']
            pt.plot3Axes(f, ires.time, ft.smooth(ires.v['x_dot.uvw'], delta=20), 'UVW Dot', 'm/s^2', options=insColor)
            legend += ['INS']
            plt.legend(legend)

        saveFigures('uvw.svg', f)

    if peCheck('uvwErr') and 'uvwErr' in ins.v.dtype.names:
        f += 1;    legend = []
        pt.plot3Axes(f, getTimeFromTow(ins.v['tow']), ins.v['uvwErr'], 'UVW Error from Truth INS', 'm/s', options=errColor)


    #############################################
    # Attitude
    if peCheck('att'):
        f += 1;    legend = [str(ser) for ser in serialNumbers]
        fig, ax = pt.subplots(f,3, "Attitude", sharex=True)

        if rIns:
            pt.subplotSingle(ax[0], rIns.v['time'], rIns.v['euler'][:,0]*RAD2DEG, options=rInsColor)

        if referencePlot:
            pt.subplotSingle(ax[0], getTimeFromTow(ins.v['tow']), ins.v['euler'][:,0]*RAD2DEG, 'Roll', 'deg', options=insColor)
        else:
            pt.subplotSingle(ax[0], getTimeFromTow(ins.v['tow']), ins.v['euler'][:,0]*RAD2DEG, 'Roll', 'deg')

        if rIns:
            pt.subplotSingle(ax[1], rIns.v['time'], rIns.v['euler'][:,1]*RAD2DEG, options=rInsColor)

        if referencePlot:
            pt.subplotSingle(ax[1], getTimeFromTow(ins.v['tow']), ins.v['euler'][:,1]*RAD2DEG, 'Pitch', 'deg', options=insColor)
        else:
            pt.subplotSingle(ax[1], getTimeFromTow(ins.v['tow']), ins.v['euler'][:,1]*RAD2DEG, 'Pitch', 'deg')

        if rIns:
            pt.subplotSingle(ax[2], rIns.v['time'], rIns.v['euler'][:,2]*RAD2DEG, options=rInsColor)
            legend += ['Truth']

        if referencePlot:
            pt.subplotSingle(ax[2], getTimeFromTow(ins.v['tow']), ins.v['euler'][:,2]*RAD2DEG, 'Yaw', 'deg', options=insColor)
        else:
            pt.subplotSingle(ax[2], getTimeFromTow(ins.v['tow']), ins.v['euler'][:,2]*RAD2DEG, 'Yaw', 'deg')

        legend += ['INS']

        if pe['att'] >= 2: # Display GPS heading
            # Ignore GPS if timestamp is not valid
            if gps1Pos and np.fabs(np.mean(getTimeFromTow(ins.v['tow'])) - np.mean(gps1Pos.time)) < 1000:
                pt.subplotSingle(ax[2], gps1Pos.time, gps1Pos.v['course']*RAD2DEG, options=refColor)
                legend += ['GPS']

        ax[2].legend(legend)

        saveFigures('attINS.svg', f)


    #############################################
    # Magnetometer NIS Threshold
    if peCheck('magNISThreshold'):
        f += 1;    legend = []
        fig, ax = pt.subplots(f,1, 'magNISThreshold')
        pt.labels('Mag NIS')
        pt.subplotSingle(ax,magInfo.v['towMs'], magInfo.v['nis'] , 'nis' )
        pt.subplotSingle(ax,magInfo.v['towMs'], magInfo.v['nis_threshold'] , 'nis_threshold' )

        #############################################
    # INL2 Vaiances
    if peCheck('variance'):
        f += 1;    legend = []
        fig, ax = pt.subplots(f,3, 'Variance', sharex=True)
        pt.labels('INL2 Position Variance')
        pt.subplotSingle(ax[0],varInfo.v['towMs'], varInfo.v['PxyxNED'][:,0] , 'INS Px N (NED)' )
        pt.subplotSingle(ax[1],varInfo.v['towMs'], varInfo.v['PxyxNED'][:,1] , 'INS Py E (NED)' )
        pt.subplotSingle(ax[2],varInfo.v['towMs'], varInfo.v['PxyxNED'][:,2] , 'INS Pz D (NED)' )

        f += 1;    legend = []
        fig, ax = pt.subplots(f,3, 'Vel Variance', sharex=True)
        pt.labels('INL2 Velocity Variance')
        pt.subplotSingle(ax[0],varInfo.v['towMs'], varInfo.v['PvelNED'][:,0] , 'INS Pvx N (NED)' )
        pt.subplotSingle(ax[1],varInfo.v['towMs'], varInfo.v['PvelNED'][:,1] , 'INS Pvy E (NED)' )
        pt.subplotSingle(ax[2],varInfo.v['towMs'], varInfo.v['PvelNED'][:,2] , 'INS Pvz D (NED)' )

        f += 1;    legend = []
        fig, ax = pt.subplots(f,3, 'Att Variance', sharex=True)
        pt.labels('INL2 Attitude Variance')
        pt.subplotSingle(ax[0],varInfo.v['towMs'], varInfo.v['PattNED'][:,0] , 'INS Pwx (NED)' )
        pt.subplotSingle(ax[1],varInfo.v['towMs'], varInfo.v['PattNED'][:,1] , 'INS Pwy (NED)' )
        pt.subplotSingle(ax[2],varInfo.v['towMs'], varInfo.v['PattNED'][:,2] , 'INS Pwz (NED)' )

        f += 1;    legend = []
        fig, ax = pt.subplots(f,3, 'A Bias Variance', sharex=True)
        pt.labels('INL2 A Bias Variance')
        pt.subplotSingle(ax[0],varInfo.v['towMs'], varInfo.v['PABias'][:,0] , 'INS PA Bias x' )
        pt.subplotSingle(ax[1],varInfo.v['towMs'], varInfo.v['PABias'][:,1] , 'INS PA Bias y' )
        pt.subplotSingle(ax[2],varInfo.v['towMs'], varInfo.v['PABias'][:,2] , 'INS PA Bias z' )

        f += 1;    legend = []
        fig, ax = pt.subplots(f,3, 'W Bias Variance', sharex=True)
        pt.labels('INL2 W Bias Variance')
        pt.subplotSingle(ax[0],varInfo.v['towMs'], varInfo.v['PWBias'][:,0] , 'INS PW Bias x' )
        pt.subplotSingle(ax[1],varInfo.v['towMs'], varInfo.v['PWBias'][:,1] , 'INS PW Bias y' )
        pt.subplotSingle(ax[2],varInfo.v['towMs'], varInfo.v['PWBias'][:,2] , 'INS PW Bias z' )

        f += 1;    legend = []
        fig, ax = pt.subplots(f,1, 'Baro Bias Variance', sharex=True)
        pt.labels('INL2 Baro Bias Variance')
        pt.subplotSingle(ax,varInfo.v['towMs'], varInfo.v['PBaroBias'] , 'Baro Bias' )

        f += 1;    legend = []
        fig, ax = pt.subplots(f,1, 'Declination Variance', sharex=True)
        pt.labels('INL2 Declination Variance')
        pt.subplotSingle(ax,varInfo.v['towMs'], varInfo.v['PDeclination'] , 'Declination' )


        #############################################
    # GPS Velocity - used to analyze GPS velocities
    if peCheck('gpsVel'):
        pt.labels('GPS Vel NED', 'm/s')
        f += 1;    legend = []
        if rIns:
            pt.plot3Axes(f, rIns.v['time'], rIns.v['nedDot'], options=rInsColor )
            legend += ['Truth']
        if gps1Vel:
            pt.plot3Axes(f, gps1Vel.time, gps1Vel.v['velNed'], options=refColor )
            legend += ['GPS.velNed']
        gps1Pos.nedDot = ft.derivative(gps1Pos.time, gps1Pos.ned, delta=2)
        # pt.plot3Axes(f, gps1Vel.time, gps1Vel.nedDot, options='m'  )
        # legend += ['GPS.ned dot']
        plt.legend(legend)
        saveFigures('gpsVel.svg', f)

        #############################################
        # EKF States
    if peCheck('ekfStatesMagField') and 'ekfStates' in log.data.keys():
        f += 1;    legend = []
        fig, ax = pt.subplots(f,2, 'ekfStatesMagField', sharex=True)
        pt.subplotSingle(ax[0], log.data['ekfStates']['time'], log.data['ekfStates']['magInc']*RAD2DEG, 'EKF States - Mag Inclination', 'deg', options=insColor )
        pt.subplotSingle(ax[1], log.data['ekfStates']['time'], log.data['ekfStates']['magDec']*RAD2DEG, 'EKF States - Mag Declination', 'deg', options=insColor )

        saveFigures('ekfStatesMagField.svg', fig=fig)

        #############################################
        # NED Velocity
    if peCheck('velNED'):
        pt.labels('Vel NED', 'm/s')
        f += 1;    legend = []
        crnFreq = 2

        if rIns:
            pt.plot3Axes(f, rIns.v['time'], rIns.v['nedDot'], options=rInsColor )
            legend += ['Truth']
        if gps1Pos:
            pt.plot3Axes(f, gps1Pos.time, gps1Pos.v['ned'], options=refColor )
            legend += ['GPS']
        pt.plot3Axes(f, ins.time, ins.velNed(), options=insColor )
        legend += ['INS']

        plt.legend(legend)

        if peCheck('velNedDot'):
            f += 1;    legend = []

            delta = 2
            gps1Pos.nedVelDot = ft.derivative(gps1Pos.time, gps1Pos.v['ned'], delta)
            ins.nedVelDot = ft.derivative(ins.time, ins.v['vel'], delta)
            ins.nedVelDot = ft.lpfNoDelay(ins.nedVelDot, cornerFreqHz=20, time=ins.time)

            if rIns:
                pt.plot3Axes(f, rIns.v['time'], rIns.nedDotDot, options=rInsColor )
                legend += ['Truth']
            pt.plot3Axes(f, ins.time, ins.nedVelDot )
            legend += ['ins']
            pt.plot3Axes(f, gps1Pos.time, gps1Pos.nedVelDot, 'NED Vel Dot', 'm/s^2', options=refColor  )
            legend += ['gps1Pos']
            plt.legend(legend)

        saveFigures('velNED.svg', f)


        #############################################
        # Position - ECEF
    if peCheck('ecef') and gps1Pos:
        f += 1;    legend = []
        fig, ax = pt.subplots(f,3, 'ECEF', sharex=True)


        instime = getTimeFromTow(ins.v['tow'])
        gpstime = getTimeFromTowMs(gps1Pos.v['timeOfWeekMs'])
        pt.subplotSingle(ax[0], instime, ins.ecef()[:,0], 'X', 'm', options=insColor)
        pt.subplotSingle(ax[0], gpstime, gps1Pos.v['ecef'][:,0], options=refColor )

        pt.subplotSingle(ax[1], instime, ins.ecef()[:,1], 'Y', 'm', options=insColor )
        pt.subplotSingle(ax[1], gpstime, gps1Pos.v['ecef'][:,1], options=refColor )

        pt.labels('Z', 'm')
        pt.subplotSingle(ax[2], instime, ins.ecef()[:,2], options=insColor )
        legend += ['INS']
        pt.subplotSingle(ax[2], gpstime, gps1Pos.v['ecef'][:,2], options=refColor )
        legend += ['GPS']
        ax[2].legend(legend)

        saveFigures('ecef.svg', f)

        #############################################
        # Position - LLA
    if peCheck('lla') and gps1Pos and hasattr(gps1Pos, 'pos'):
        f += 1;    legend = []
        fig, ax = pt.subplots(f,3, 'LLA', sharex=True)
        if rIns:
            pt.subplotSingle(ax[0], rIns.v['time'], rIns.v['lla'][:,0], options=rInsColor )

        instime = getTimeFromTow(ins.v['tow'])
        gpstime = getTimeFromTowMs(gps1Pos.v['timeOfWeekMs'])
        pt.subplotSingle(ax[0], instime, ins.v['lla'][:,0], 'Latitude', 'deg', options=insColor)
        pt.subplotSingle(ax[0], gpstime, gps1Pos.v['lla'][:,0], options=refColor )

        if rIns:
            pt.subplotSingle(ax[1], rIns.v['time'], rIns.v['lla'][:,1], options=rInsColor )

        pt.subplotSingle(ax[1], instime, ins.v['lla'][:,1], 'Longitude', 'deg', options=insColor )
        pt.subplotSingle(ax[1], gpstime, gps1Pos.v['lla'][:,1], options=refColor )

        if rIns:
            pt.subplotSingle(ax[2], rIns.v['time'], rIns.v['lla'][:,2], options=rInsColor )
            legend += ['Truth']

        pt.labels('Elipsoid Alt', 'm')
        pt.subplotSingle(ax[2], instime, ins.v['lla'][:,2], options=insColor )
        legend += ['INS']
        pt.subplotSingle(ax[2], gpstime, gps1Pos.v['lla'][:,2], options=refColor )
        legend += ['GPS']
        if baro:
            pt.subplotSingle(ax[2], baro.time, baro.v['mslBar'], options='m' )
            legend += ['mslBar']
        ax[2].legend(legend)

        saveFigures('lla.svg', f)

        #############################################
        # Position Error
    if peCheck('nedErr') and 'nedErr' in ins.v.dtype.names:
        f += 1;
        pt.plot3Axes(f, ins.time, ins.v['nedErr'], 'NED Error from Truth INS', 'm' )


        #############################################
        # NED
    if peCheck('ned'):
        #         pt.labels('NED', 'm')
        f+=1;  legend = []
        fig, ax = pt.subplots(f,3, 'NED', sharex=True)

        #         if rIns:
        #             pt.plot3Axes(f, rIns.v['time'], rIns.ned, options=rInsColor )
        #             legend += ['Truth']
        #         if pe['ned == 1:     # INS & GPS
        #             j = 0
        #         elif pe['ned == 2:   # GPS & RTK
        #             fd

        if ins:
            instime = getTimeFromTow(ins.v['tow'])
        if gps1Pos:
            gpstime = getTimeFromTowMs(gps1Pos.v['timeOfWeekMs'])

        if 'gps1UbxPos' in log.data.keys():
            global refLla
            # ubxNED = pose.lla2ned(refLla, log.data['gps1RtkPos']['lla'])
            # ubxtime = getTimeFromTowMs(log.data['gps1RtkPos']['timeOfWeekMs'])
            ubxtime = getTimeFromTowMs(gps1Ubx.v['timeOfWeekMs'])
        # else:
        #     rtkNED = None

        if 'gps1RtkPos' in log.data.keys():
            global refLla
            # rtkNED = pose.lla2ned(refLla, log.data['gps1RtkPos']['lla'])
            # rtktime = getTimeFromTowMs(log.data['gps1RtkPos']['timeOfWeekMs'])
            rtktime = getTimeFromTowMs(gps1RtkPos.v['timeOfWeekMs'])
        else:
            rtkNED = None

        if gps1Ubx:
            ubxtime = getTimeFromTowMs(gps1Ubx.v['timeOfWeekMs'])

        pt.subplotSingle(ax[0], instime, ins.ned()[:,0], options=insColor, title="North", ylabel='m')
        legend = ['ins']
        if gps1Pos:
            pt.subplotSingle(ax[0], gpstime, gps1Pos.ned[:,0], options=refColor)
            legend += ['gps1Pos']
        if gps1Ubx:
            pt.subplotSingle(ax[0], ubxtime, gps1Ubx.ned[:,0], options=ubxColor)
            legend += ['gps1Ubx']
        if gps1RtkPos:
            pt.subplotSingle(ax[0], rtktime, gps1RtkPos.ned[:,0], options=rtkColor)
            legend += ['gps1RtkPos']
        # if rtkNED is not None:
        #     pt.subplotSingle(ax[0], rtktime, rtkNED[:,0], options=rtkColor)
        #     legend += ['rtk']
        ax[0].legend(legend)

        pt.subplotSingle(ax[1], instime, ins.ned()[:,1], options=insColor, title="East", ylabel='m')
        legend = ['ins']
        if gps1Pos:
            pt.subplotSingle(ax[1], gpstime, gps1Pos.ned[:,1], options=refColor)
            legend += ['gps1Pos']
        if gps1Ubx:
            pt.subplotSingle(ax[1], ubxtime, gps1Ubx.ned[:,1], options=ubxColor)
            legend += ['gps1Ubx']
        if gps1RtkPos:
            pt.subplotSingle(ax[1], rtktime, gps1RtkPos.ned[:,1], options=rtkColor)
            legend += ['gps1RtkPos']
        # if rtkNED is not None:
        #     pt.subplotSingle(ax[1], rtktime, rtkNED[:,1], options=rtkColor)
        #     legend += ['rtk']
        ax[1].legend(legend)

        pt.subplotSingle(ax[2], instime, ins.ned()[:,2], options=insColor, title="Down", ylabel='m')
        legend = ['ins']
        if gps1Pos:
            pt.subplotSingle(ax[2], gpstime, gps1Pos.ned[:,2], options=refColor)
            legend += ['gps1Pos']
        if gps1Ubx:
            pt.subplotSingle(ax[2], ubxtime, gps1Ubx.ned[:,2], options=ubxColor)
            legend += ['gps1Ubx']
        if gps1RtkPos:
            pt.subplotSingle(ax[2], rtktime, gps1RtkPos.ned[:,2], options=rtkColor)
            legend += ['gps1RtkPos']
        # if rtkNED is not None:
        #     pt.subplotSingle(ax[2], rtktime, rtkNED[:,2], options=rtkColor)
        #     legend += ['rtk']
        ax[2].legend(legend)

        #         pt.plot3Axes(f, ins.time, ins.ned(), options=insColor )
        #         legend += ['ins']
        #         if gps1Pos:
        #             pt.plot3Axes(f, gps1Pos.time, gps1Pos.ned, options=refColor  )
        #             legend += ['gps1Pos']

        saveFigures('ned.svg', f)

        #############################################
        # NED Map
    if peCheck('nedMap'):
        #         pt.labels('NED', 'm')
        f += 1;    legend = []
        #         if rIns:
        #             pt.plot3Axes(f, rIns.v['time'], rIns.ned, options=rInsColor )
        #             legend += ['Truth']

        fig, ax = pt.subplots(f,1, 'NED Map')
        pt.subplotSingle(ax, ins.ned()[:,1], ins.ned()[:,0], options=insColor, title="North East Map")
        legend += ['ins']
        if gps1Pos:
            pt.subplotSingle(ax, gps1Pos.ned[:,1], gps1Pos.ned[:,0], options=refColor)
            legend += ['gps1Pos']
        ax.legend(legend)

        saveFigures('nedMap.svg', f)

        #############################################
        # RTK NED Map
    if peCheck('nedMapRtk'):
        #         pt.labels('NED', 'm')
        f += 1;    legend = []

        fig, ax = pt.subplots(f,1, 'RTK NED Map')
        if gps1RtkPos:
            pt.subplotSingle(ax, gps1RtkPos.ned[:,1], gps1RtkPos.ned[:,0], options=insColor)
            legend += ['rtk']
        if gps1Ubx:
            pt.subplotSingle(ax, gps1Ubx.ned[:,1], gps1Ubx.ned[:,0], options=refColor)
            legend += ['gps1Ubx']
        ax.legend(legend)
        pt.labels('RTK NED Map', 'm')

        saveFigures('nedMapRtk.png', f)

        #############################################
        # NED Error
    #     if peCheck('nedError'):
    #         f+=1;  legend = []
    #         fig, ax = pt.subplots(f,3, 'NED Error', sharex=True)
    #
    #         if pe['nedError == 1:    # INS-GPS
    #
    #         elif pe['nedError == 2:  # GPS-RTK
    #
    #         elif pe['nedError == 3:  # INS-RTK
    #
    #         pt.subplotSingle(ax[0], ins.time, ins.ned()[:,0], options=insColor, title="North", ylabel='m')
    #         legend = ['ins']
    #         if gps1Pos:
    #             pt.subplotSingle(ax[0], gps1Pos.time, gps1Pos.ned[:,0], options=refColor)
    #             legend += ['gps1Pos']
    #         ax[0].legend(legend)
    #
    #         pt.subplotSingle(ax[1], ins.time, ins.ned()[:,1], options=insColor, title="East", ylabel='m')
    #         legend = ['ins']
    #         if gps1Pos:
    #             pt.subplotSingle(ax[1], gps1Pos.time, gps1Pos.ned[:,1], options=refColor)
    #             legend += ['gps1Pos']
    #         ax[1].legend(legend)
    #
    #         pt.subplotSingle(ax[2], ins.time, ins.ned()[:,2], options=insColor, title="Down", ylabel='m')
    #         legend = ['ins']
    #         if gps1Pos:
    #             pt.subplotSingle(ax[2], gps1Pos.time, gps1Pos.ned[:,2], options=refColor)
    #             legend += ['gps1Pos']
    #         ax[2].legend(legend)
    #
    #         saveFigures('nedError.svg', f)

    if peCheck('nve') and not peCheck('showUtcTime'):
        f += 1; legend = []
        fig, ax = pt.subplots(f,1, 'NVE')

        pt.subplotSingle(ax, ins.v['lla'][:,1], ins.v['lla'][:,0], options=insColor, title="Lat Lon Map")
        legend += ['ins']
        if gps1Pos:
            pt.subplotSingle(ax, gps1Pos.v['lla'][:,1], gps1Pos.v['lla'][:,0], options=refColor)
            legend += ['gps1Pos']

        saveFigures('LatLonMap.svg', f)

        #############################################
        # NED Dot
        if peCheck('nedDot'):

            f += 1;    legend = []

            delta = 1
            ins.nedDot = ft.derivative(ins.time, ins.ned(), delta, title="NED dot")
            if gps1Pos:
                gps1Pos.nedDot = ft.derivative(gps1Pos.time, gps1Pos.ned, delta)
            if rIns:
                rIns.nedDot = ft.derivative(rIns.v['time'], rIns.ned, delta)
                pt.plot3Axes(f, rIns.v['time'], rIns.nedDot, options=rInsColor )
                legend += ['Truth']

            pt.plot3Axes(f, ins.time, ins.velNED )
            legend += ['ins']
            if gps1Pos:
                pt.plot3Axes(f, gps1Pos.time, gps1Pos.nedDot, 'NED Dot', 'm/s', options=refColor  )
                legend += ['gps1Pos']
            plt.legend(legend)

    if peCheck('staticBiasAttEst'):
        f += 1;    legend = []
        [ imu1.accAtt,  imu1.accBias] = ps.acc2AttAndBias( imu1.fltAcc())
        if rIns:
            [rImu.accAtt, rImu.accBias] = ps.acc2AttAndBias(rImu.fltAcc())
            pt.plot3Axes(f, rImu.v['time'], rImu.accBias, options=rInsColor)
            legend += ['RefIMU'];
        pt.plot3Axes(f, imu1.time, imu1.accBias, 'IMU Accel Bias','m/s^2' )
        legend += ['imu1'];
        plt.legend(legend)

        f += 1;    legend = []
        if rIns:
            pt.plot3Axes(f, rImu.v['time'], rImu.accAtt*RAD2DEG, options=rInsColor)
            legend += ['RefIMU'];
        pt.plot3Axes(f, imu1.time, imu1.accAtt*RAD2DEG, 'Attitude from Gravity','deg')
        legend += ['imu1'];
        plt.legend(legend)

        #############################################
        # Altitude
    if peCheck('altitude'):
        f += 1;     legend = []
        fig, ax = pt.subplots(f,1, 'Altitude')
        pt.labels('Altitude', 'm')

        pt.subplotSingle(ax, ins.time, ins.v['lla'][:,2], options=insColor )
        legend += ['INS']
        if baro:
            pt.subplotSingle(ax, baro.time, baro.v['mslBar'], options='g' )
            legend += ['Baro MSL']
        if gps1Pos:
            pt.subplotSingle(ax, gps1Pos.time - gps1Pos.time[0], gps1Pos.v['lla'][:,2], options=refColor )
            legend += ['GPS ellipsoid']
            pt.subplotSingle(ax, gps1Pos.time - gps1Pos.time[0], gps1Pos.v['hMSL'], options='m' )
            legend += ['GPS geoid/MSL']
        ax.legend(legend)

    ##########################################################################################
    #   S E N S O R S
    ##########################################################################################
    imu = [cObj(),cObj()]
    mag = [cObj(),cObj()]
    if dimu:
        imu[0].pqr = dimu.i[0].pqr
        imu[1].pqr = dimu.i[1].pqr
        imu[0].acc = dimu.i[0].acc
        imu[1].acc = dimu.i[1].acc
        xlim = None
        if peCheck('sensorXLim'):
            xlim = pe['sensorXLim']

            #############################################
            # Sensors - PQR
        if peCheck('sensorPqr'):
            f += 1;    legend = []
            if peCheck('sensorSeparate'):
                pt.labels('Sensors: PQR1','deg/s')
            else:
                pt.labels('Sensors: PQR','deg/s')
            if pe['sensorPqr'] != 1:
                freq = pe['sensorPqr']
                imu[0].pqr = ft.lpfNoDelay(imu[0].pqr, freq, time=dimu.time)
                imu[1].pqr = ft.lpfNoDelay(imu[1].pqr, freq, time=dimu.time)

            if rIns:
                pt.plot3Axes(f, rImu.v['time'], RAD2DEG * rImu.flt.pqr, options=rInsColor )
                legend += ['Truth']

            i = len(dimu.time)/2
            #             ylim = None
            yspan = None
            if peCheck('sensorPqrSpan'):
                yspan = pe['sensorPqrSpan']
            pt.plot3Axes(f, dimu.time, imu[0].pqr * RAD2DEG, xlim=xlim )
            pt.plot3setYspan(f, yspan)
            legend += ['pqr1']
            if peCheck('sensorSeparate'):
                plt.legend(legend); f += 1; legend = []
                pt.labels('Sensors: PQR2','deg/s')
            pt.plot3Axes(f, dimu.time, imu[1].pqr * RAD2DEG, xlim=xlim)
            pt.plot3setYspan(f, yspan)
            legend += ['pqr2']
            plt.legend(legend)

            saveFigures('sensorPqr.svg', f)

            g_imu1BiasPqr = np.mean(imu[0].pqr * RAD2DEG, axis=0)
            g_imu2BiasPqr = np.mean(imu[1].pqr * RAD2DEG, axis=0)

            #############################################
            # Sensors - Acceleration
        if peCheck('sensorAcc'):
            f += 1;    legend = []
            offset = [0.0,0.0,0.0]
            #             offset = [0.0,0.0,9.8]
            if peCheck('sensorSeparate'):
                pt.labels('Sensors: Accel1','m/s^2')
            else:
                pt.labels('Sensors: Accel','m/s^2')
            if pe['sensorAcc'] != 1:
                freq = pe['sensorAcc']
                imu[0].acc = ft.lpfNoDelay(imu[0].acc, freq, time=dimu.time)
                imu[1].acc = ft.lpfNoDelay(imu[1].acc, freq, time=dimu.time)

            if rIns:
                pt.plot3Axes(f, rImu.v['time'], rImu.flt.acc+offset, options=rInsColor )
                legend += ['Truth'];

            yspan = None
            if peCheck('sensorAccSpan'):
                yspan = pe['sensorAccSpan']
            pt.plot3Axes(f, dimu.time, imu[0].acc+offset, xlim=xlim )
            pt.plot3setYspan(f, yspan)
            legend += ['acc1']
            if peCheck('sensorSeparate'):
                plt.legend(legend); f += 1; legend = []
                pt.labels('Sensors: Accel2','m/s^2')
            pt.plot3Axes(f, dimu.time, imu[1].acc+offset, xlim=xlim )
            pt.plot3setYspan(f, yspan)
            legend += ['acc2']
            plt.legend(legend)

            saveFigures('sensorAccel.svg', f)

            g_imu1BiasAcc = np.mean(imu[0].acc, axis=0)
            g_imu2BiasAcc = np.mean(imu[1].acc, axis=0)

            #############################################
            # FFT - PQR
        if peCheck('fftPqr'):
            f += 1;    legend = []
            fig, ax = pt.subplots(f,3, 'FFT PQR')
            ax[0].set_title("FFT: PQR")

            N = np.shape(imu[0].pqr[:,0])[0]                # Number of samples
            T = np.mean(dimu.time[1:] - dimu.time[0:-1])   # Sample period

            pqrFftX = np.linspace(0.0, 1.0/(2.0*T), N/2)
            ylim = 0.006
            for i in range(0,3):
                pqrFftY = 2.0/N * np.abs(np.fft.fft(imu[0].pqr[:,i])[:N//2])
                ax[i].set_xlabel('Freq (Hz)')
                ax[i].set_ylim([-0.1*ylim,ylim])
                pt.subplotSingle(ax[i], pqrFftX, pqrFftY )

            ax[0].set_ylabel('P')
            ax[1].set_ylabel('Q')
            ax[2].set_ylabel('R')

            saveFigures('fftPqr.svg', f)


            #############################################
            # FFT - Acc
        if peCheck('fftAcc'):
            f += 1;    legend = []
            fig, ax = pt.subplots(f,3, 'FFT Accel')
            ax[0].set_title('FFT: Accel')

            N = np.shape(imu[0].acc[:,0])[0]                # Number of samples
            T = np.mean(dimu.time[1:] - dimu.time[0:-1])   # Sample period

            accFftX = np.linspace(0.0, 1.0/(2.0*T), N/2)
            ylim = 0.25
            for i in range(0,3):
                accFftY = 2.0/N * np.abs(np.fft.fft(imu[0].acc[:,i])[:N//2])
                ax[i].set_xlabel('Freq (Hz)')
                ax[i].set_ylim([-0.1*ylim,ylim])
                pt.subplotSingle(ax[i], accFftX, accFftY )

            ax[0].set_ylabel('X')
            ax[1].set_ylabel('Y')
            ax[2].set_ylabel('Z')

            saveFigures('fftAcc.svg', f)


        #############################################
        # Sensors - Magnetometer
    if peCheck('sensorMag'):
        f += 1;    legend = []
        pt.labels('Sensors: Mag','gauss')
        if pe['sensorMag'] > 1:
            mag_1 = ft.smooth(mag1.v['mag'][:,:], pe['sensorMag'])
            mag_2 = ft.smooth(mag2.v['mag'][:,:], pe['sensorMag'])
        else:
            mag_1 = mag1.v['mag'][:,:]
            mag_2 = mag2.v['mag'][:,:]

        pt.plot3Axes(f, mag1.v['time'], mag_1 )
        pt.plot3Axes(f, mag2.v['time'], mag_2 )
        legend += ['mag 1','mag 2']
        plt.legend(legend)

        saveFigures('sensorMagnetometer.svg', f)

        #############################################
        # Sensors - Magnetometer
    if peCheck('sensorMag'):
        f += 1;    legend = []
        if peCheck('sensorSeparate'):
            pt.labels('Sensors: Mag1','gauss')
        else:
            pt.labels('Sensors: Mag','gauss')
        mag[0].time = mag1.time
        mag[0].mag  = mag1.v['mag'][:,:]
        mag[1].time = mag2.time
        mag[1].mag  = mag2.v['mag'][:,:]

        if pe['sensorMag'] != 1:
            freq = pe['sensorAcc']
            mag[0].mag = ft.lpfNoDelay(mag[0].mag, freq, time=mag[0].time)
            mag[1].mag = ft.lpfNoDelay(mag[1].mag, freq, time=mag[1].time)

        pt.plot3Axes(f, mag[0].time, mag[0].mag, xlim=xlim )
        legend += ['mag1']
        if peCheck('sensorSeparate'):
            plt.legend(legend); f += 1; legend = []
            pt.labels('Sensors: Mag2','gauss')
        pt.plot3Axes(f, mag[1].time, mag[1].mag, xlim=xlim )
        legend += ['mag2']
        plt.legend(legend)

        saveFigures('sensorMag.svg', f)

        #############################################
        # Sensors - Barometer
    if peCheck('sensorBar'):
        f += 1;     legend = []
        fig, ax = pt.subplots(f,1, 'Sensor Baro')

        pt.labels('Barometer MSL', 'm')
        pt.subplotSingle(ax, baro.time, baro.v['mslBar'] )
        legend += ['mslBar']
        if rIns:
            pt.subplotSingle(ax, rIns.v['time'], rIns.v['lla'][:,2], options=rInsColor )
            legend += ['Truth']
        if pe['sensorBar'] == 2:
            pt.subplotSingle(ax, gps1Pos.time, gps1Pos.v['lla'][:,2], options=refColor )
            legend += ['GPS']
        ax.legend(legend)

        saveFigures('baroMsl.svg', f)

        #############################################
        # SensorsIS1
    if  peCheck('sensorIs1PqrVsTemp') or \
        peCheck('sensorIs1AccVsTemp') or \
        peCheck('sensorIs1Pqr') or \
        peCheck('sensorIs1Acc') or \
        peCheck('sensorIs1Mag'):
        mpu = []
        mpu.append(cObj())
        mpu.append(cObj())
        mpu[0].temp = log.data['sensorsIs1']['mpu'][:,0]['temp']
        mpu[1].temp = log.data['sensorsIs1']['mpu'][:,1]['temp']
        dt = 0.004
        mpuTime = np.arange(0.0, np.shape(mpu[0].temp)[0])*dt

        #############################################
        # SensorsIS1 - PQR
        if peCheck('sensorIs1PqrVsTemp'):
            f += 1;    legend = []
            pt.labels('SensorsIS1: PQR vs Temperature','deg/s')
            mpu[0].pqr  = log.data['sensorsIs1']['mpu'][:,0]['pqr']
            mpu[1].pqr  = log.data['sensorsIs1']['mpu'][:,1]['pqr']
            if pe['sensorIs1PqrVsTemp'] != 1:
                freq = pe['sensorIs1PqrVsTemp']
                mpu[0].pqr = ft.lpfNoDelay(mpu[0].pqr, freq, dt=dt)
                mpu[1].pqr = ft.lpfNoDelay(mpu[1].pqr, freq, dt=dt)
            pt.plot3Axes(f, mpu[0].temp, mpu[0].pqr*RAD2DEG, xlabel="temp (C)" )
            pt.plot3Axes(f, mpu[1].temp, mpu[1].pqr*RAD2DEG, xlabel="temp (C)" )
            legend += ['mpu1','mpu2']
            plt.legend(legend)

        if peCheck('sensorIs1Pqr'):
            mpu[0].pqr  = log.data['sensorsIs1']['mpu'][:,0]['pqr']
            mpu[1].pqr  = log.data['sensorsIs1']['mpu'][:,1]['pqr']
            if pe['sensorIs1Pqr'] != 1:
                freq = pe['sensorIs1Pqr']
                mpu[0].pqr = ft.lpfNoDelay(mpu[0].pqr, freq, dt=dt)
                mpu[1].pqr = ft.lpfNoDelay(mpu[1].pqr, freq, dt=dt)

            for m in range(0,2):
                if m==0 or peCheck('sensorSeparate'):
                    f += 1;  legend = []
                    fig, ax = pt.subplots(f,4, 'SensorsIS1: PQR & Temp vs Time', sharex=True)
                legend += ['mpu%d' % (m+1)]
                pt.subplotSingle(ax[0], mpuTime, mpu[m].pqr[:,0]*RAD2DEG, 'P', 'deg/s' )
                pt.subplotSingle(ax[1], mpuTime, mpu[m].pqr[:,1]*RAD2DEG, 'Q', 'deg/s' )
                pt.subplotSingle(ax[2], mpuTime, mpu[m].pqr[:,2]*RAD2DEG, 'R', 'deg/s' )
                pt.subplotSingle(ax[3], mpuTime, mpu[m].temp, 'Temp', 'C' )
                ax[0].legend(legend)

            #############################################
            # SensorsIS1 - Accelerometers
        if peCheck('sensorIs1AccVsTemp'):
            f += 1;    legend = []
            pt.labels('SensorsIS1: Accel vs Temperature','m/s^2')
            mpu[0].acc  = log.data['sensorsIs1']['mpu'][:,0]['acc']
            mpu[1].acc  = log.data['sensorsIs1']['mpu'][:,1]['acc']
            if pe['sensorIs1AccVsTemp'] != 1:
                freq = pe['sensorIs1AccVsTemp']
                mpu[0].acc = ft.lpfNoDelay(mpu[0].acc, freq, dt=dt)
                mpu[1].acc = ft.lpfNoDelay(mpu[1].acc, freq, dt=dt)
            pt.plot3Axes(f, mpu[0].temp, mpu[0].acc, xlabel="temp (C)" )
            pt.plot3Axes(f, mpu[1].temp, mpu[1].acc, xlabel="temp (C)" )
            legend += ['mpu1','mpu2']
            plt.legend(legend)

        if peCheck('sensorIs1Acc'):
            mpu[0].acc  = log.data['sensorsIs1']['mpu'][:,0]['acc']
            mpu[1].acc  = log.data['sensorsIs1']['mpu'][:,1]['acc']
            if pe['sensorIs1Acc'] != 1:
                freq = pe['sensorIs1Acc']
                mpu[0].acc = ft.lpfNoDelay(mpu[0].acc, freq, dt=dt)
                mpu[1].acc = ft.lpfNoDelay(mpu[1].acc, freq, dt=dt)

            for m in range(0,2):
                if m==0 or peCheck('sensorSeparate'):
                    f += 1;  legend = []
                    fig, ax = pt.subplots(f,4, 'SensorsIS1: PQR & Temp vs Time', sharex=True)
                legend += ["mpu%d"%(m+1)]
                pt.subplotSingle(ax[0], mpuTime, mpu[m].acc[:,0], 'Accel X', 'm/s^2' )
                pt.subplotSingle(ax[1], mpuTime, mpu[m].acc[:,1], 'Accel Y', 'm/s^2' )
                pt.subplotSingle(ax[2], mpuTime, mpu[m].acc[:,2], 'Accel Z', 'm/s^2' )
                pt.subplotSingle(ax[3], mpuTime, mpu[m].temp, 'Temp', 'C' )
                ax[0].legend(legend)

            #############################################
            # SensorsIS1 - Magnetometers
        if peCheck('sensorIs1Mag'):
            f += 1;    legend = []
            pt.labels('SensorsIS1: Mag vs Temperature','gauss')
            lgd = ['mpu1','mpu2']
            mpu[0].mag  = log.data['sensorsIs1']['mpu'][:,0]['mag']
            mpu[1].mag  = log.data['sensorsIs1']['mpu'][:,1]['mag']
            if pe['sensorIs1Mag'] != 1:
                freq = pe['sensorMag']
                mpu[0].mag = ft.smooth(mpu[0].mag, pe['sensorIs1Mag'])
                mpu[1].mag = ft.smooth(mpu[1].mag, pe['sensorIs1Mag'])
            pt.plot3Axes(f, mpu[0].temp, mpu[0].mag, xlabel="temp (C)" )
            pt.plot3Axes(f, mpu[1].temp, mpu[1].mag, xlabel="temp (C)" )
            legend += lgd
            plt.legend(legend)


        #############################################
        # GPS Statistics
    if peCheck('gpsStats'):
        f += 1;    legend = []
        if 'gps1Pos' not in log.data.keys():
            print("unable to print gpsStats, because we are missing data")
        else:
            fig, ax = pt.subplots(f,3, 'GPS Stats', sharex=True)

            gpsPos = log.data['gps1Pos']
            time = getTimeFromTowMs(gpsPos['timeOfWeekMs'])

            pt.subplotSingle(ax[0], time, gpsPos['status'] & 0xFF, 'Satellites Used in Solution', '' )
            pt.subplotSingle(ax[1], time, gpsPos['pDop'], 'Accuracy', 'm', options='m' )
            legend = ['pDop']
            pt.subplotSingle(ax[1], time, gpsPos['hAcc'], 'Accuracy', 'm', options='r' )
            legend += ['Hor']
            pt.subplotSingle(ax[1], time, gpsPos['vAcc'], options='b' )
            legend += ['Ver']
            if 'gps1RtkPos' in log.data.keys():
                rtktime = getTimeFromTowMs(log.data['gps1RtkPos']['timeOfWeekMs'])
                pt.subplotSingle(ax[1], rtktime, log.data['gps1RtkPos']['vAcc'], options=rtkColor)
                legend += ['rtkHor']
            ax[1].legend(legend)

            pt.subplotSingle(ax[2], time, gpsPos['cnoMean'], 'CNO', 'dBHz', options='b' )
            legend = ['Mean']
            ax[2].legend(legend)

            saveFigures('gpsStats.svg', f)

    if gps1Ubx and peCheck('gps1Stats'):
        f += 1;    legend = []
        fig, ax = pt.subplots(f,3, 'GPS1 Stats', sharex=True)

        pt.subplotSingle(ax[0], gps1Ubx.time, gps1Ubx.satsUsed, 'Satellites Used in Solution', '' )
        pt.subplotSingle(ax[1], gps1Ubx.time, gps1Ubx.v['pDop'], 'Accuracy', 'm', options='m' )
        legend = ['pDop']
        pt.subplotSingle(ax[1], gps1Ubx.time, gps1Ubx.v['hAcc'], 'Accuracy', 'm', options='r' )
        legend += ['Hor']
        pt.subplotSingle(ax[1], gps1Ubx.time, gps1Ubx.v['vAcc'], options='b' )
        legend += ['Ver']
        ax[1].legend(legend)
        pt.subplotSingle(ax[2], gps1Ubx.time, gps1Ubx.v['cno'], 'CNO', 'dBHz', options='r' )
        legend = ['Max']
        pt.subplotSingle(ax[2], gps1Ubx.time, gps1Ubx.v['cnoMean'], 'CNO', 'dBHz', options='b' )
        legend += ['Mean']
        ax[2].legend(legend)

        saveFigures('gps1Stats.svg', f)

        #############################################
        # RTK-GPS Statistics
    if peCheck('rtkStats'):
        f += 1;    legend = [str(ser) for ser in serialNumbers]
        if 'gps1Pos' not in log.data or 'gps1RtkPosRel' not in log.data:
            print("Unable to plot rtkStats - data missing")
        else:
            n_plots = 5
            if 'gps1RktMisc' in log.data.keys():
                n_plots += 1
            # if 'GPS1Raw' in log.data.keys():
            #     n_plots += 1
            # if 'GPS2Raw' in log.data.keys():
            #     n_plots += 1
            fig, ax = pt.subplots(f,n_plots, 'RTK Stats', sharex=True)

            time = getTimeFromTowMs(log.data['gps1Pos']['timeOfWeekMs'])
            compassing = log.data['gps1Pos']['status'][10] & 0x00400000
            if not compassing:
                fixType = log.data['gps1Pos']['status'] >> 8 & 0xFF
                pt.subplotSingle(ax[0], time, fixType, 'GPS Fix Type: 2=2D, 3=3D, 10=Single, 11=Float, 12=Fix', '' )
                ax[0].legend(legend)

            time = getTimeFromTowMs(log.data['gps1RtkPosRel']['timeOfWeekMs'])

            if compassing:
                fixType = log.data['gps1RtkPosRel']['arRatio'].copy()
                iStatus
                pt.subplotSingle(ax[0], time, fixType, 'GPS Fix Type: 2=2D, 3=3D, 10=Single, 11=Float, 12=Fix', '' )

            if 'gps1RtkPosRel' in log.data.keys():
                disttobase = log.data['gps1RtkPosRel']['distanceToBase']
                disttobase[disttobase > 100000] = np.nan
                pt.subplotSingle(ax[1], time, log.data['gps1RtkPosRel']['differentialAge'], 'RTK: Age of Differential', 's' )
                pt.subplotSingle(ax[2], time, log.data['gps1RtkPosRel']['arRatio'], 'RTK: AR Ratio', 'num' )
                pt.subplotSingle(ax[3], time, disttobase, 'Distance to Base', 'm' )

            n_plot = 4
            if 'gps1RtkPosMisc' in log.data.keys():
                rtkMiscTime = getTimeFromTowMs(log.data['gps1RtkPosMisc']['timeOfWeekMs'])
                pt.subplotSingle(ax[n_plot], rtkMiscTime, log.data['gps1RtkPosMisc']['cycleSlipCount'], 'RTK: Slip Counter', ' ' )
                n_plot += 1



            # if 'GPS1Raw' in log.data.keys():
            #     Obs = log.data['GPS1Raw']['obs']
            #     satellites = np.unique(Obs['sat'])
            #     deltas = np.empty_like(Obs['L'])
            #     for i, sat in enumerate(satellites):
            #         valid_indexes = Obs['sat'] == sat
            #         deltas[valid_indexes] = np.hstack((np.array([0]), np.diff(Obs['L'][valid_indexes])))
            #     obstime = np.array([np.datetime64(int(np.round((t['time'] + t['sec']) * 1000000)), 'us') for t in Obs['time']]).astype(datetime.datetime)
            #     pt.subplotSingle(ax[n_plot], obstime,deltas, 'Delta Carrier Phase GPS 1', ' ')
            #     n_plot +=1
            #
            # if 'GPS2Raw' in log.data.keys():
            #     Obs = log.data['GPS2Raw']['obs']
            #     satellites = np.unique(Obs['sat'])
            #     delta = np.empty_like(Obs['L'])
            #     for i, sat in enumerate(satellites):
            #         valid_indexes = Obs['sat'] == sat
            #         delta[valid_indexes] = np.hstack((np.array([0]), np.diff(Obs['L'][valid_indexes])))
            #     obstime = np.array([np.datetime64(int(np.round((t['time'] + t['sec']) * 1000000)), 'us') for t in Obs['time']]).astype(datetime.datetime)
            #     pt.subplotSingle(ax[n_plot], obstime, delta, 'Delta Carrier Phase GPS 2', ' ')



            saveFigures('rtkStats.svg', f)

    ##########################################################################################
    # INS Status
    ##########################################################################################
    if peCheck('iStatus') and ins:
        f += 1;    legend = []
        fig, ax = pt.subplots(f,1, 'INS Status')

        #         if len(IsLoggerPlot.serialNumbers)
        pt.labels('INS Status')
        cnt = 0

        instime = getTimeFromTow(ins.v['tow'])
        iStatus = ins.iStatus()
        #         ax.text(p1, -cnt*1.5, '___ ALIGNED ___')
        #         cnt+=1
        #         pt.subplotSingle(ax, ins.time, -cnt*1.5+ iStatus.align.coarse.att, options='b' )
        #         ax.text(p1, -cnt*1.5, 'Att Conv. Done')
        #         cnt+=1
        pt.subplotSingle(ax, instime, -cnt*1.5+ iStatus.align.coarse.att, options='b' )
        p1 = ax.get_xlim()[0] + 0.02*(ax.get_xlim()[1] - ax.get_xlim()[0])
        ax.text(p1, -cnt*1.5, 'Att Coarse')
        cnt+=1
        pt.subplotSingle(ax, instime, -cnt*1.5+ iStatus.align.good.att, options='b' )
        ax.text(p1, -cnt*1.5, 'Att Good')
        cnt+=1
        pt.subplotSingle(ax, instime, -cnt*1.5+ iStatus.align.fine.att, options='c' )
        ax.text(p1, -cnt*1.5, 'Att Fine')
        cnt+=1
        pt.subplotSingle(ax, instime, -cnt*1.5+ iStatus.align.coarse.vel, options='g' )
        ax.text(p1, -cnt*1.5, 'Vel Coarse')
        cnt+=1
        pt.subplotSingle(ax, instime, -cnt*1.5+ iStatus.align.good.vel, options='g' )
        ax.text(p1, -cnt*1.5, 'Vel Good')
        cnt+=1
        pt.subplotSingle(ax, instime, -cnt*1.5+ iStatus.align.coarse.pos, options='r' )
        ax.text(p1, -cnt*1.5, 'Pos Coarse')
        cnt+=1
        pt.subplotSingle(ax, instime, -cnt*1.5+ iStatus.align.good.pos, options='r' )
        ax.text(p1, -cnt*1.5, 'Pos Good')
        cnt+=1
        cnt+=1

        #         ax.text(p1, -cnt*1.5, '___ ALIGNING ___')
        pt.subplotSingle(ax, instime, -cnt*1.5+ iStatus.usingGps, options='b' )
        ax.text(p1, -cnt*1.5, 'Using GPS')
        cnt+=1
        pt.subplotSingle(ax, instime, -cnt*1.5+ iStatus.usingMag, options='m' )
        ax.text(p1, -cnt*1.5, 'Using MAG')
        cnt+=1
        cnt+=1
        pt.subplotSingle(ax, instime, -cnt*1.5+ iStatus.navMode )
        ax.text(p1, -cnt*1.5, 'Nav Mode')
        cnt+=1
        cnt+=1
        pt.subplotSingle(ax, instime, -cnt*1.5+ (iStatus.solutionStatus)/4.0 )
        ax.text(p1, -cnt*1.5, 'Solution Status')
        cnt+=1
        cnt+=1

        pt.subplotSingle(ax, instime, -cnt*1.5+ iStatus.magActiveCalSet )
        ax.text(p1, -cnt*1.5, 'Mag Active Cal Set')
        cnt+=1
        pt.subplotSingle(ax, instime, -cnt*1.5+ iStatus.magRecalibrating )
        ax.text(p1, -cnt*1.5, 'Mag Recalibrating')
        cnt+=1
        pt.subplotSingle(ax, instime, -cnt*1.5+ iStatus.magInterOrBadCal )
        ax.text(p1, -cnt*1.5, 'Mag Inter. or Bad Cal')
        cnt+=1
        cnt+=1

        pt.subplotSingle(ax, instime, -cnt*1.5+ iStatus.rtosTaskPeriodOverrun )
        ax.text(p1, -cnt*1.5, 'RTOS Task Period Overrun')
        cnt+=1
        pt.subplotSingle(ax, instime, -cnt*1.5+ iStatus.generalFault )
        ax.text(p1, -cnt*1.5, 'General Fault')
        cnt+=1

        saveFigures('iStatus.svg', f)

        #############################################
        # Solution Status
        f += 1;    legend = []
        fig, ax = pt.subplots(f,1, 'Solution Status')
        pt.labels('Solution Status', ' ')
        pt.subplotSingle(ax, instime, iStatus.solutionStatus )

        saveFigures('SolutionStatus.svg', f)


    ##########################################################################################
    # Hardware Status
    ##########################################################################################
    if peCheck('hStatus') and ins:
        f += 1;    legend = []
        fig, ax = pt.subplots(f,1, 'Hdw Status')

        hStatus = ins.hStatus()

        pt.labels('Hardware Status')
        cnt = 0
        p1 = ins.time[0]

        pt.subplotSingle(ax, ins.time, -cnt*1.5+ hStatus.motionGyrSig )
        ax.text(p1, -cnt*1.5, 'Motion Gyr Sig')
        cnt+=1
        pt.subplotSingle(ax, ins.time, -cnt*1.5+ hStatus.motionAccSig )
        ax.text(p1, -cnt*1.5, 'Motion Acc Sig')
        cnt+=1
        pt.subplotSingle(ax, ins.time, -cnt*1.5+ hStatus.motionGyrDev )
        ax.text(p1, -cnt*1.5, 'Motion Gyr Dev')
        cnt+=1
        pt.subplotSingle(ax, ins.time, -cnt*1.5+ hStatus.motionAccDev )
        ax.text(p1, -cnt*1.5, 'Motion Acc Dev')
        cnt+=1
        cnt+=1

        pt.subplotSingle(ax, ins.time, -cnt*1.5+ hStatus.satellite_rx )
        ax.text(p1, -cnt*1.5, 'Satellite Rx')
        cnt+=1
        cnt+=1

        pt.subplotSingle(ax, ins.time, -cnt*1.5+ hStatus.saturationGyr )
        ax.text(p1, -cnt*1.5, 'Saturation Gyr')
        cnt+=1
        pt.subplotSingle(ax, ins.time, -cnt*1.5+ hStatus.saturationAcc )
        ax.text(p1, -cnt*1.5, 'Saturation Acc')
        cnt+=1
        pt.subplotSingle(ax, ins.time, -cnt*1.5+ hStatus.saturationMag )
        ax.text(p1, -cnt*1.5, 'Saturation Mag')
        cnt+=1
        pt.subplotSingle(ax, ins.time, -cnt*1.5+ hStatus.saturationBaro )
        ax.text(p1, -cnt*1.5, 'Saturation Baro')
        cnt+=1
        pt.subplotSingle(ax, ins.time, -cnt*1.5+ hStatus.saturationHistory )
        ax.text(p1, -cnt*1.5, 'Saturation History')
        cnt+=1
        cnt+=1

        pt.subplotSingle(ax, ins.time, -cnt*1.5+ hStatus.errComTxLimited )
        ax.text(p1, -cnt*1.5, 'Err Com Tx Limited')
        cnt+=1
        pt.subplotSingle(ax, ins.time, -cnt*1.5+ hStatus.errComRxOverrun )
        ax.text(p1, -cnt*1.5, 'Err Com Rx Overrun')
        cnt+=1
        pt.subplotSingle(ax, ins.time, -cnt*1.5+ hStatus.errGpsTxLimited )
        ax.text(p1, -cnt*1.5, 'Err GPS Tx Limited')
        cnt+=1
        pt.subplotSingle(ax, ins.time, -cnt*1.5+ hStatus.errGpsRxOverrun )
        ax.text(p1, -cnt*1.5, 'Err GPS Rx Overrun')
        cnt+=1
        cnt+=1

        pt.subplotSingle(ax, ins.time, -cnt*1.5+ (hStatus.comParseErrCount)/4 )
        ax.text(p1, -cnt*1.5, 'Com Parse Error Count')
        cnt+=1
        pt.subplotSingle(ax, ins.time, -cnt*1.5+ hStatus.selfTestFault )
        ax.text(p1, -cnt*1.5, 'Self Test Fault')
        cnt+=1
        pt.subplotSingle(ax, ins.time, -cnt*1.5+ hStatus.errTemperature )
        ax.text(p1, -cnt*1.5, 'Temperature error')
        cnt+=1
        cnt+=1

        pt.subplotSingle(ax, ins.time, -cnt*1.5+ hStatus.faultWatchdogReset )
        ax.text(p1, -cnt*1.5, 'Watchdog Reset')
        cnt+=1
        pt.subplotSingle(ax, ins.time, -cnt*1.5+ hStatus.faultBODReset )
        ax.text(p1, -cnt*1.5, 'Brownout Reset')
        cnt+=1
        pt.subplotSingle(ax, ins.time, -cnt*1.5+ hStatus.faultPORReset )
        ax.text(p1, -cnt*1.5, 'Power-on Reset')
        cnt+=1
        pt.subplotSingle(ax, ins.time, -cnt*1.5+ hStatus.faultCPUErrReset )
        ax.text(p1, -cnt*1.5, 'CPU Error Reset')
        cnt+=1
        cnt+=1

        saveFigures('hStatus.svg', f)


    #############################################
    # Timestamp
    if peCheck('Timestamp'):
        f += 1;    legend = [];
        fig, ax = pt.subplots(f, 3, 'Timestamp', sharex=True)

        # yspan = 0.05

        dtIns = ins.time[1:] - ins.time[0:-1]
        timeIns = ins.time[1:]
        pt.subplotSingle(ax[0], timeIns, dtIns, 'INS dt', 's')
        # pt.subplotSetYspan(ax[0], yspan);
        print("INS dt (min, max, mean):", np.min(dtIns), np.max(dtIns), np.mean(dtIns))

        if imu1:
            dtImu = imu1.time[1:] - imu1.time[0:-1]
            timeImu = imu1.time[1:]
            pt.subplotSingle(ax[1], timeImu, dtImu, 'IMU dt', 's')
            # pt.subplotSetYspan(ax[1], yspan);
            print("IMU dt (min, max, mean):", np.min(dtImu), np.max(dtImu), np.mean(dtImu))

        if dimu:
            dtDImu = dimu.time[1:] - dimu.time[0:-1]
            timeDImu = dimu.time[1:]
            pt.subplotSingle(ax[1], timeDImu, dtDImu, 'IMU dt', 's')
            # pt.subplotSetYspan(ax[1], yspan);
            print("IMU dt (min, max, mean):", np.min(dtDImu), np.max(dtDImu), np.mean(dtDImu))

        if gps1Pos:
            dtGps = gps1Pos.time[1:] - gps1Pos.time[0:-1]
            timeGps = gps1Pos.time[1:]
            pt.subplotSingle(ax[2], timeGps, dtGps, 'GPS dt', 's')
            # pt.subplotSetYspan(ax[2], yspan);
            print("GPS dt (min, max, mean):", np.min(dtGps), np.max(dtGps), np.mean(dtGps))

        saveFigures('Timestamp.svg', f)

    # return last figure number
    return f

# Initialize function static variables after the function definition 
IsLoggerPlot.serialNumbers = []




