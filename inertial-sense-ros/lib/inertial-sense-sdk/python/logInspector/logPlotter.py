import math
from typing import List, Any, Union

import numpy as np
import matplotlib.pyplot as plt
import sys
import yaml
import os
from os.path import expanduser

BLACK = r"\u001b[30m"
RED = r"\u001b[31m"
GREEN = r"\u001b[32m"
YELLOW = r"\u001b[33m"
BLUE = r"\u001b[34m"
MAGENTA = r"\u001b[35m"
CYAN = r"\u001b[36m"
WHITE = r"\u001b[37m"
RESET = r"\u001b[0m"

RAD2DEG = 180.0 / 3.14159
DEG2RAD = 3.14159 / 180.0

sys.path.append('..')
from logReader import Log
from pylib.ISToolsDataSorted import refLla, getTimeFromTowMs, getTimeFromTow, setGpsWeek, getTimeFromGTime
from pylib.data_sets import *
from pylib.pose import quat2eulerArray, lla2ned, rotate_ecef2ned, quatRotVectArray
import datetime

class logPlot:
    def __init__(self, show, save, format, log):
        self.show = show
        self.save = save
        self.directory = log.directory
        self.format = format
        self.log = log
        self.d = 1
        self.setActiveSerials(self.log.serials)

        setGpsWeek(self.log.data[0, DID_INS_2]['week'][-1])

    def setDownSample(self, dwns):
        self.d = dwns

    def setActiveSerials(self, serials):
        self.active_devs = []
        for d, ser in enumerate(self.log.serials):
            if ser in serials:
                self.active_devs.append(d)

    def configureSubplot(self, ax, title, xlabel):
        ax.set_title(title)
        ax.set_xlabel(xlabel)

    def saveFig(self, fig, name):
        if self.save:
            fsize = fig.get_size_inches()
            # fig.set_size_inches(16,16)
            fig.set_size_inches(20, 20)
            directory = os.path.dirname(self.directory + '/figures/')
            if not os.path.exists(directory):
                os.makedirs(directory)
            fig.savefig(os.path.join(directory + "/" + name + '.' + self.format), bbox_inches='tight')
            fig.set_size_inches(fsize)

    def getData(self, dev, DID, field):
        try:
            return self.log.data[dev, DID][field][::self.d]
        except:
            return []

    def setPlotYSpanMin(self, ax, limit):
        ylim = ax.get_ylim()
        yspn = np.max( [ylim[1] - ylim[0], limit] )
        ylim = (np.mean(ylim)-yspn/2, np.mean(ylim)+yspn/2)
        ax.set_ylim(ylim)

    def posNED(self, fig=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(3,1, sharex=True)
        self.configureSubplot(ax[0], 'North', 'm')
        self.configureSubplot(ax[1], 'East', 'm')
        self.configureSubplot(ax[2], 'Down', 'm')
        fig.suptitle('INS NED - ' + os.path.basename(os.path.normpath(self.log.directory)))
        refLla = None
        for d in self.active_devs:
            if refLla is None:
                refLla = self.getData(d, DID_INS_2, 'lla')[0]
            ned = lla2ned(refLla, self.getData(d, DID_INS_2, 'lla'))
            time = getTimeFromTow(self.getData(d, DID_INS_2, 'timeOfWeek'))
            ax[0].plot(time, ned[:,0], label=self.log.serials[d])
            ax[1].plot(time, ned[:,1])
            ax[2].plot(time, ned[:,2])

            if(np.shape(self.active_devs)[0]==1):
                timeGPS = getTimeFromTowMs(self.getData(d, DID_GPS1_POS, 'timeOfWeekMs'))
                nedGps = lla2ned(self.getData(d, DID_INS_2, 'lla')[0], self.getData(d, DID_GPS1_POS, 'lla'))
                ax[0].plot(timeGPS, nedGps[:, 0], label='GPS')
                ax[1].plot(timeGPS, nedGps[:, 1])
                ax[2].plot(timeGPS, nedGps[:, 2])

        ax[0].legend(ncol=2)
        for a in ax:
            a.grid(True)
        self.saveFig(fig, 'posNED')

    def drawNEDMapArrow(self, ax, ned, heading):
        # arrowLen = 0.2
        # arrowWid = arrowLen/2
        # arrows = np.array([arrowLen * 0.7 * np.cos(heading), arrowLen * 0.7 * np.sin(heading)]).T

        markersize = 10
        downsample = 6
        len = np.shape(heading)[0]
        for i in range(1, len, downsample):
            # ax.arrow(ned[i,1], ned[i,0], arrows[i,1], arrows[i,0], head_width=arrowWid, head_length=arrowLen, length_includes_head=True, fc='k', ec='k')
            ax.plot(ned[i,1], ned[i,0], marker=(3, 0, -heading[i]*(180.0/np.pi)), color='g', markersize=markersize, linestyle='None')
            ax.plot(ned[i,1], ned[i,0], marker=(2, 0, -heading[i]*(180.0/np.pi)), color='k', markersize=markersize, linestyle='None')

    def posNEDMap(self, fig=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(1,1)
        ax.set_xlabel('East (m)')
        ax.set_ylabel('North (m)')
        fig.suptitle('NED Map - ' + os.path.basename(os.path.normpath(self.log.directory)))
        refLla = None
        for d in self.active_devs:
            if refLla is None:
                refLla = self.getData(d, DID_INS_2, 'lla')[0]
            ned = lla2ned(refLla, self.getData(d, DID_INS_2, 'lla'))
            euler = quat2eulerArray(self.getData(d, DID_INS_2, 'qn2b'))
            ax.plot(ned[:,1], ned[:,0], label=self.log.serials[d])

            if(np.shape(self.active_devs)[0]==1):
                self.drawNEDMapArrow(ax, ned, euler[:, 2])

                nedGps = lla2ned(self.getData(d, DID_INS_2, 'lla')[0], self.getData(d, DID_GPS1_POS, 'lla'))
                ax.plot(nedGps[:, 1], nedGps[:, 0], label='GPS')


        ax.set_aspect('equal', 'datalim')
        ax.legend(ncol=2)
        ax.grid(True)

    def posLLA(self, fig=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(3,1, sharex=True)
        self.configureSubplot(ax[0], 'Latitude', 'deg')
        self.configureSubplot(ax[1], 'Longitude', 'deg')
        self.configureSubplot(ax[2], 'Altitude', 'deg')
        fig.suptitle('INS LLA - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            time = getTimeFromTow(self.getData(d, DID_INS_2, 'timeOfWeek'))
            ax[0].plot(time, self.getData(d, DID_INS_2, 'lla')[:,0], label=self.log.serials[d])
            ax[1].plot(time, self.getData(d, DID_INS_2, 'lla')[:,1])
            ax[2].plot(time, self.getData(d, DID_INS_2, 'lla')[:,2], label=self.log.serials[d])

            if(np.shape(self.active_devs)[0]==1):
                timeGPS = getTimeFromTowMs(self.getData(d, DID_GPS1_POS, 'timeOfWeekMs'))
                ax[0].plot(timeGPS, self.getData(d, DID_GPS1_POS, 'lla')[:, 0], label='GPS')
                ax[1].plot(timeGPS, self.getData(d, DID_GPS1_POS, 'lla')[:, 1])
                ax[2].plot(timeGPS, self.getData(d, DID_GPS1_POS, 'lla')[:, 2], label='GPS')

                timeBaro = getTimeFromTow(self.getData(d, DID_BAROMETER, 'time')+ self.getData(d, DID_GPS1_POS, 'towOffset')[-1])
                ax[2].plot(timeBaro, self.getData(d, DID_BAROMETER, 'mslBar'), label='Baro')

        ax[0].legend(ncol=2)
        ax[2].legend(ncol=2)
        for a in ax:
            a.grid(True)
        self.saveFig(fig, 'insLLA')

    def llaGps(self, fig=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(3,1, sharex=True)
        self.configureSubplot(ax[0], 'Latitude', 'deg')
        self.configureSubplot(ax[1], 'Longitude', 'deg')
        self.configureSubplot(ax[2], 'Altitude', 'deg')
        fig.suptitle('GPS LLA - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            time = getTimeFromTowMs(self.getData(d, DID_GPS1_POS, 'timeOfWeekMs'))
            ax[0].plot(time, self.getData(d, DID_GPS1_POS, 'lla')[:,0], label=self.log.serials[d])
            ax[1].plot(time, self.getData(d, DID_GPS1_POS, 'lla')[:,1])
            ax[2].plot(time, self.getData(d, DID_GPS1_POS, 'lla')[:,2])
        ax[0].legend(ncol=2)
        for a in ax:
            a.grid(True)
        self.saveFig(fig, 'llaGPS')

    def velNED(self, fig=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(3,1, sharex=True)
        self.configureSubplot(ax[0], 'North', 'm/s')
        self.configureSubplot(ax[1], 'East', 'm/s')
        self.configureSubplot(ax[2], 'Down', 'm/s')
        fig.suptitle('NED Vel - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            time = getTimeFromTow(self.getData(d, DID_INS_2, 'timeOfWeek'))
            insVelNed = quatRotVectArray(self.getData(d, DID_INS_2, 'qn2b'), self.getData(d, DID_INS_2, 'uvw'))
            ax[0].plot(time, insVelNed[:,0], label=self.log.serials[d])
            ax[1].plot(time, insVelNed[:,1])
            ax[2].plot(time, insVelNed[:,2])

            if np.shape(self.active_devs)[0] == 1:  # Show GPS if #devs is 1
                timeGPS = getTimeFromTowMs(self.getData(d, DID_GPS1_VEL, 'timeOfWeekMs'))
                gpsVelEcef = self.getData(d, DID_GPS1_VEL, 'velEcef')
                R = rotate_ecef2ned(self.getData(d, DID_GPS1_POS, 'lla')[0]*np.pi/180.0)
                gpsVelNed = R.dot(gpsVelEcef.T).T
                ax[0].plot(timeGPS, gpsVelNed[:, 0], label='GPS')
                ax[1].plot(timeGPS, gpsVelNed[:, 1])
                ax[2].plot(timeGPS, gpsVelNed[:, 2])

        ax[0].legend(ncol=2)
        for a in ax:
            a.grid(True)
        self.saveFig(fig, 'velNED')

    def velUVW(self, fig=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(3,1, sharex=True)
        self.configureSubplot(ax[0], 'Vel-X', 'm/s')
        self.configureSubplot(ax[1], 'Vel-Y', 'm/s')
        self.configureSubplot(ax[2], 'Vel-Z', 'm/s')
        fig.suptitle('INS uvw - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            time = getTimeFromTow(self.getData(d, DID_INS_2, 'timeOfWeek'))
            ax[0].plot(time, self.getData(d, DID_INS_2, 'uvw')[:,0], label=self.log.serials[d])
            ax[1].plot(time, self.getData(d, DID_INS_2, 'uvw')[:,1])
            ax[2].plot(time, self.getData(d, DID_INS_2, 'uvw')[:,2])
        ax[0].legend(ncol=2)
        for a in ax:
            a.grid(True)
        self.saveFig(fig, 'velUVW')

    def attitude(self, fig=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(3, 1, sharex=True)
        fig.suptitle('INS Attitude - ' + os.path.basename(os.path.normpath(self.log.directory)))
        self.configureSubplot(ax[0], 'Roll', 'deg')
        self.configureSubplot(ax[1], 'Pitch', 'deg')
        self.configureSubplot(ax[2], 'Yaw', 'deg')
        for d in self.active_devs:
            euler = quat2eulerArray(self.getData(d, DID_INS_2, 'qn2b'))
            time = getTimeFromTow(self.getData(d, DID_INS_2, 'timeOfWeek'))
            ax[0].plot(time, euler[:,0]*RAD2DEG, label=self.log.serials[d])
            ax[1].plot(time, euler[:,1]*RAD2DEG)
            ax[2].plot(time, euler[:,2]*RAD2DEG)
        ax[0].legend(ncol=2)
        for a in ax:
            a.grid(True)
        self.saveFig(fig, 'attINS')

    def heading(self, fig=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(3, 1, sharex=True)
        fig.suptitle('Heading - ' + os.path.basename(os.path.normpath(self.log.directory)))
        self.configureSubplot(ax[0], 'Magnetic Heading', 'deg')
        self.configureSubplot(ax[1], 'RTK Compassing', 'deg')
        self.configureSubplot(ax[2], 'INS Heading', 'deg')
        for d in self.active_devs:
            magTime = getTimeFromTowMs(self.getData(d, DID_INL2_MAG_OBS_INFO, 'timeOfWeekMs'))
            gpsTime = getTimeFromTowMs(self.getData(d, DID_GPS1_RTK_CMP_REL, 'timeOfWeekMs'))
            insTime = getTimeFromTow(self.getData(d, DID_INS_2, 'timeOfWeek'))
            magHdg = self.getData(d, DID_INL2_MAG_OBS_INFO, 'magHdg')
            gpsHdg = self.getData(d, DID_GPS1_RTK_CMP_REL, 'baseToRoverHeading')
            euler = quat2eulerArray(self.getData(d, DID_INS_2, 'qn2b'))
            if magTime:
                ax[0].plot(magTime, magHdg * RAD2DEG)
            if gpsTime:
                ax[1].plot(gpsTime, gpsHdg*RAD2DEG)
            ax[2].plot(insTime, euler[:,2]*RAD2DEG, label=self.log.serials[d])
        ax[2].legend(ncol=2)
        for a in ax:
            a.grid(True)
        self.saveFig(fig, 'heading')

    def insStatus(self, fig=None):
        try:
            if fig is None:
                fig = plt.figure()
            ax = fig.subplots(1, 1, sharex=True)
            fig.suptitle('INS Status - ' + os.path.basename(os.path.normpath(self.log.directory)))

            for d in self.active_devs:
                r = d == self.active_devs[0]    # plot text w/ first device
                cnt = 0
                instime = getTimeFromTow(self.getData(d, DID_INS_2, 'timeOfWeek'))
                iStatus = self.getData(d, DID_INS_2, 'insStatus')

                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x00000001) != 0))
                p1 = ax.get_xlim()[0] + 0.02 * (ax.get_xlim()[1] - ax.get_xlim()[0])
                if r: ax.text(p1, -cnt * 1.5, 'Att Coarse')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x00000010) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Att Fine')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x00000002) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Vel Coarse')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x00000020) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Vel Fine')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x00000004) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Pos Coarse')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x00000040) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Pos Fine')
                cnt += 1
                cnt += 1

                # ax.plot(instime, -cnt * 1.5 + ((iStatus >> 9) & 1))
                # ax.text(p1, -cnt * 1.5, 'GPS Update')
                # cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x00000100) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'GPS aiding Pos/Vel')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x00000080) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'GPS aiding Hdg')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x00000800) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'MAG aiding Hdg')
                cnt += 1
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x00001000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Nav Mode')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x000F0000) >> 16) / 4.0)
                if r: ax.text(p1, -cnt * 1.5, 'Solution Status')
                cnt += 1
                cnt += 1

                ax.plot(instime, -cnt * 1.5 + (((iStatus & 0x03000000) >> 24) == 3))
                if r: ax.text(p1, -cnt * 1.5, 'RTK: Precision Position Valid')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x04000000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'RTK: Compassing Valid (fix & hold)')
                cnt += 1
                cnt += 1

                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x00100000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'RTK: Compassing Baseline UNSET')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x00200000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'RTK: Compassing Baseline BAD')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x08000000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'RTK: No Observ.')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x10000000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'RTK: Base No Pos.')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x20000000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'RTK: Base Pos. Moving')
                cnt += 1
                cnt += 1

                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x00400000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Mag: Recalibrating')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x00800000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Mag: Inter. or Bad Cal')
                cnt += 1
                cnt += 1

                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x40000000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'RTOS Task Period Overrun')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((iStatus & 0x80000000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'General Fault')
                cnt += 1

            ax.grid(True)
            self.saveFig(fig, 'iStatus')
        except:
            print(RED + "problem plotting insStatus: " + sys.exc_info()[0] + RESET)

    def hdwStatus(self, fig=None):
        try:
            if fig is None:
                fig = plt.figure()
            ax = fig.subplots(1, 1, sharex=True)
            fig.suptitle('Hardware Status - ' + os.path.basename(os.path.normpath(self.log.directory)))

            for d in self.active_devs:
                r = d == self.active_devs[0]    # plot text w/ first device
                cnt = 0
                instime = getTimeFromTow(self.getData(d, DID_INS_2, 'timeOfWeek'))
                hStatus = self.getData(d, DID_INS_2, 'hdwStatus')

                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x00000001) != 0))
                p1 = ax.get_xlim()[0] + 0.02 * (ax.get_xlim()[1] - ax.get_xlim()[0])
                if r: ax.text(p1, -cnt * 1.5, 'Motion Gyr Sig')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x00000002) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Motion Acc Sig')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x00000004) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Motion Gyr Dev')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x00000005) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Motion Acc Dev')
                cnt += 1
                cnt += 1

                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x00000010) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Satellite Rx')
                cnt += 1
                cnt += 1

                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x00000100) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Saturation Gyr')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x00000200) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Saturation Acc')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x00000400) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Saturation Mag')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x00000800) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Saturation Baro')
                cnt += 1
                cnt += 1

                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x00010000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Err Com Tx Limited')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x00020000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Err Com Rx Overrun')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x00040000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Err GPS Tx Limited')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x00080000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Err GPS Rx Overrun')
                cnt += 1
                cnt += 1

                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x00F00000) >> 20) / 4)
                if r: ax.text(p1, -cnt * 1.5, 'Com Parse Error Count')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x01000000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'BIT Self Test Fault')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x02000000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Temperature error')
                cnt += 1
                cnt += 1

                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x10000000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Reset Backup Mode')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x20000000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Watchdog Reset')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x30000000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Software Reset')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x40000000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Hardware Reset')
                cnt += 1
                ax.plot(instime, -cnt * 1.5 + ((hStatus & 0x80000000) != 0))
                if r: ax.text(p1, -cnt * 1.5, 'Critical Sys Fault')
                cnt += 1
                cnt += 1
                
            ax.grid(True)
            self.saveFig(fig, 'Hardware Status')
        except:
            print(RED + "problem plotting hdwStatus: " + sys.exc_info()[0] + RESET)

    def gpsStats(self, fig=None, did_gps_pos=DID_GPS1_POS):
        # try:
        if fig is None:
            fig = plt.figure()

        ax = fig.subplots(4, 1, sharex=True)
        if did_gps_pos==DID_GPS1_POS:
            gps_num = 1
        else:
            gps_num = 2
        fig.suptitle('GPS ' + str(gps_num) + ' Stats - ' + os.path.basename(os.path.normpath(self.log.directory)))
        self.configureSubplot(ax[0], 'Satellites Used in Solution', '')
        self.configureSubplot(ax[1], 'Accuracy', 'm')
        self.configureSubplot(ax[2], 'CNO', 'dBHz')
        self.configureSubplot(ax[3], 'Status', '')

        for d in self.active_devs:
            r = d == self.active_devs[0]  # plot text w/ first device
            time = getTimeFromTowMs(self.getData(d, did_gps_pos, 'timeOfWeekMs'))
            gStatus = self.getData(d, did_gps_pos, 'status')

            ax[0].plot(time, gStatus & 0xFF, label=self.log.serials[d])
            ax[1].plot(time, self.getData(d, did_gps_pos, 'pDop'), 'm', label="pDop")
            ax[1].plot(time, self.getData(d, did_gps_pos, 'hAcc'), 'r', label="hAcc")
            ax[1].plot(time, self.getData(d, did_gps_pos, 'vAcc'), 'b', label="vAcc")
            if self.log.data[d, DID_GPS1_RTK_POS] is not []:
                rtktime = getTimeFromTowMs(self.getData(d, DID_GPS1_RTK_POS, 'timeOfWeekMs'))
                ax[1].plot(rtktime, self.getData(d, DID_GPS1_RTK_POS, 'vAcc'), 'g', label="rtkHor")
            if d == 0:
                ax[1].legend(ncol=2)
            ax[2].plot(time, self.getData(d, did_gps_pos, 'cnoMean'))

            cnt = 0
            ax[3].plot(time, -cnt * 1.5 + ((gStatus & 0x04000000) != 0))
            p1 = ax[3].get_xlim()[0] + 0.02 * (ax[3].get_xlim()[1] - ax[3].get_xlim()[0])
            if r: ax[3].text(p1, -cnt * 1.5, 'RTK Positioning Valid')
            cnt += 1
            ax[3].plot(time, -cnt * 1.5 + ((gStatus & 0x08000000) != 0))
            if r: ax[3].text(p1, -cnt * 1.5, 'RTK Compassing Valid (fix & hold)')
            cnt += 1
            ax[3].plot(time, -cnt * 1.5 + ((gStatus & 0x00002000) != 0))
            if r: ax[3].text(p1, -cnt * 1.5, 'GPS Compass Baseline BAD')
            cnt += 1
            ax[3].plot(time, -cnt * 1.5 + ((gStatus & 0x00004000) != 0))
            if r: ax[3].text(p1, -cnt * 1.5, 'GPS Compass Baseline UNSET')
            cnt += 1

        self.setPlotYSpanMin(ax[2], 5)

        ax[0].legend(ncol=2)
        for a in ax:
            a.grid(True)
        self.saveFig(fig, 'Gps Stats')
        # except:
        #     print(RED + "problem plotting gpsStats: " + sys.exc_info()[0] + RESET)

    def gps2Stats(self, fig=None):
        self.gpsStats(fig=fig, did_gps_pos=DID_GPS2_POS)

    def rtkPosStats(self, fig=None):
        self.rtkStats("Position", DID_GPS1_RTK_POS_REL, fig=fig)

    def rtkCmpStats(self, fig=None):
        self.rtkStats("Compassing", DID_GPS1_RTK_CMP_REL, fig=fig)

    def rtkStats(self, name, relDid, fig=None):
        # try:
        n_plots = 6
        if fig is None:
            fig = plt.figure()

        ax = fig.subplots(n_plots, 1, sharex=True)
        fig.suptitle('RTK ' + name + ' Stats - ' + os.path.basename(os.path.normpath(self.log.directory)))
        self.configureSubplot(ax[0], 'GPS Fix Type: 2=2D, 3=3D, 10=Single, 11=Float, 12=Fix', '')
        self.configureSubplot(ax[1], 'Age of Differential', 's')
        self.configureSubplot(ax[2], 'AR Ratio', '')
        self.configureSubplot(ax[3], 'Base to Rover Distance', 'm')
        self.configureSubplot(ax[4], 'Base to Rover Heading', 'deg')
        self.configureSubplot(ax[5], 'Base to Rover Heading Accuracy', 'deg')

        for i, d in enumerate(self.active_devs):
            rtkRelTime = getTimeFromTowMs(self.getData(d, relDid, 'timeOfWeekMs'))
            # rtkMiscTime = getTimeFromTowMs(self.getData(d, DID_GPS1_RTK_CMP_MISC, 'timeOfWeekMs'))
            if not self.log.compassing:
                gps1PosTime = getTimeFromTowMs(self.getData(d, DID_GPS1_POS, 'timeOfWeekMs'))
                fixType = self.getData(d, DID_GPS1_POS, 'status') >> 8 & 0x1F
                ax[0].plot(gps1PosTime, fixType, label=self.log.serials[d])
            else:
                fixType = self.getData(d, relDid, 'arRatio').copy()
                fixType[(fixType > 3)] = 12
                fixType[(fixType > 0) & (fixType < 3)] = 11
                fixType[fixType == 0] = 10
                ax[0].plot(rtkRelTime, fixType, label=self.log.serials[d])
            ax[1].plot(rtkRelTime, self.getData(d, relDid, 'differentialAge'))
            if i == 0:
                ax[2].semilogy(rtkRelTime, np.ones_like(rtkRelTime)*3.0, 'k--')
            ax[2].semilogy(rtkRelTime, self.getData(d, relDid, 'arRatio'))
            dist2base = self.getData(d, relDid, 'baseToRoverDistance')
            dist2base[dist2base > 1e5] = np.nan
            ax[3].plot(rtkRelTime, dist2base)
            ax[4].plot(rtkRelTime, self.getData(d, relDid, 'baseToRoverHeading')*180.0/np.pi)
            ax[5].plot(rtkRelTime, self.getData(d, relDid, 'baseToRoverHeadingAcc')*180.0/np.pi)
            ax[0].legend(ncol=2)

        self.setPlotYSpanMin(ax[1], 0.5)    # Differential age
        self.setPlotYSpanMin(ax[3], 1.0)    # Distance to base

        for a in ax:
            a.grid(True)
        self.saveFig(fig, 'rtk'+name+'Stats')
        # except:
            # print(RED + "problem plotting rtkStats: " + sys.exc_info()[0] + RESET)

    def rtkPosMisc(self, fig=None):
        self.rtkMisc("Position", DID_GPS1_RTK_POS_MISC, fig=fig)

    def rtkCmpMisc(self, fig=None):
        self.rtkMisc("Position", DID_GPS1_RTK_CMP_MISC, fig=fig)

    def rtkMisc(self, name, miscDid, fig=None):
        # try:
        n_plots = 10
        if fig is None:
            fig = plt.figure()

        ax = fig.subplots(5, 2, sharex=True)
        fig.suptitle('RTK ' + name + ' Misc - ' + os.path.basename(os.path.normpath(self.log.directory)))
        self.configureSubplot(ax[0,0], 'Correction checksum failure count', '')
        self.configureSubplot(ax[1,0], 'Time to First Fix', 's')
        self.configureSubplot(ax[2,0], 'GPS Observation Count - Rover', '')
        self.configureSubplot(ax[3,0], 'GPS Observation Count - Base', '')
        self.configureSubplot(ax[4,0], 'Glonass Observation Count - Rover', '')
        self.configureSubplot(ax[0,1], 'Glonass Observation Count - Base', '')
        self.configureSubplot(ax[1,1], 'Galileo Observation Count - Rover', '')
        self.configureSubplot(ax[2,1], 'Galileo Observation Count - Base', '')
        self.configureSubplot(ax[3,1], 'SBAS Count Base', '')
        self.configureSubplot(ax[4,1], 'Base Antenna Position Count', '')

        for i, d in enumerate(self.active_devs):
            # rtkRelTime = getTimeFromTowMs(self.getData(d, DID_GPS1_RTK_POS_REL, 'timeOfWeekMs'))
            rtkMiscTime = getTimeFromTowMs(self.getData(d, miscDid, 'timeOfWeekMs'))
            ax[0,0].plot(rtkMiscTime, self.getData(d, miscDid, 'correctionChecksumFailures'))
            ax[1,0].plot(rtkMiscTime, self.getData(d, miscDid, 'timeToFirstFixMs')*0.001)
            ax[2,0].plot(rtkMiscTime, self.getData(d, miscDid, 'roverGpsObservationCount'))
            ax[3,0].plot(rtkMiscTime, self.getData(d, miscDid, 'baseGpsObservationCount'))
            ax[4,0].plot(rtkMiscTime, self.getData(d, miscDid, 'roverGlonassObservationCount'))
            ax[0,1].plot(rtkMiscTime, self.getData(d, miscDid, 'baseGlonassObservationCount'))
            ax[1,1].plot(rtkMiscTime, self.getData(d, miscDid, 'roverGalileoObservationCount'))
            ax[2,1].plot(rtkMiscTime, self.getData(d, miscDid, 'baseGalileoObservationCount'))
            ax[3,1].plot(rtkMiscTime, self.getData(d, miscDid, 'baseSbasCount'))
            ax[4,1].plot(rtkMiscTime, self.getData(d, miscDid, 'baseAntennaCount'))

            # # ax[0].plot(rtkRelTime, self.getData(d, DID_GPS1_RTK_POS_REL, 'differentialAge'))
            # if i == 0:
            #     ax[2].semilogy(rtkRelTime, np.ones_like(rtkRelTime)*3.0, 'k--')
            # ax[2].semilogy(rtkRelTime, self.getData(d, DID_GPS1_RTK_POS_REL, 'arRatio'))
            # dist2base = self.getData(d, DID_GPS1_RTK_POS_REL, 'distanceToBase')
            # dist2base[dist2base > 1e5] = np.nan
            # ax[3].plot(rtkRelTime, dist2base)
            # ax[4].plot(rtkMiscTime, self.getData(d, miscDid, 'cycleSlipCount'))
            # ax[0].legend(ncol=2)
            for a in ax:
                for b in a:
                    b.grid(True)

        self.saveFig(fig, 'rtk'+name+'Misc')
        # except:
            # print(RED + "problem plotting rtkStats: " + sys.exc_info()[0] + RESET)

    def rtkRel(self, fig=None):
        # try:
        n_plots = 3
        if fig is None:
            fig = plt.figure()

        ax = fig.subplots(3, 1, sharex=True)
        fig.suptitle('RTK Rel - ' + os.path.basename(os.path.normpath(self.log.directory)))
        self.configureSubplot(ax[0], 'GPS Base to Rover Heading', '')
        self.configureSubplot(ax[1], 'GPS Base to Rover Distance', '')

        for i, d in enumerate(self.active_devs):
            rtkRelTime = getTimeFromTowMs(self.getData(d, DID_GPS1_RTK_POS_REL, 'timeOfWeekMs'))
            ax[0].plot(rtkRelTime, self.getData(d, DID_GPS1_RTK_POS_REL, 'baseToRoverHeading')*RAD2DEG)
            ax[1].plot(rtkRelTime, self.getData(d, DID_GPS1_RTK_POS_REL, 'baseToRoverDistance'))

            for a in ax:
                a.grid(True)

        self.saveFig(fig, 'rtkRel')

    def imuPQR(self, fig=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(6, 1, sharex=True)
        self.configureSubplot(ax[0], 'Roll Rate 1', 'deg/s')
        self.configureSubplot(ax[1], 'Roll Rate 2', 'deg/s')
        self.configureSubplot(ax[2], 'Pitch Rate 1', 'deg/s')
        self.configureSubplot(ax[3], 'Pitch Rate 2', 'deg/s')
        self.configureSubplot(ax[4], 'Yaw Rate 1', 'deg/s')
        self.configureSubplot(ax[5], 'Yaw Rate 2', 'deg/s')
        fig.suptitle('PQR - ' + os.path.basename(os.path.normpath(self.log.directory)))
        # ax[0].set_ylim([-.05, .05])
        # ax[1].set_ylim([-.05, .05])
        # ax[2].set_ylim([-.05, .05])
        # ax[3].set_ylim([-.05, .05])
        # ax[4].set_ylim([-.05, .05])
        # ax[5].set_ylim([-.05, .05])
        for d in self.active_devs:
            #imu = self.getData(d, DID_DUAL_IMU, 'I')
            I1 = self.getData(d, DID_DUAL_IMU, 'I')[:,0]
            I2 = self.getData(d, DID_DUAL_IMU, 'I')[:,1]
            p0 = []
            q0 = []
            r0 = []
            p1 = []
            q1 = []
            r1 = []
            if np.shape(I1)[0] != 0:
                time = self.getData(d, DID_DUAL_IMU, 'time') # + self.getData(d, DID_GPS1_POS, 'towOffset')[-1]
                for i in range(0, len(I1)):
                    p0.append(I1[i][0][0]*RAD2DEG)
                    q0.append(I1[i][0][1]*RAD2DEG)
                    r0.append(I1[i][0][2]*RAD2DEG)
                    p1.append(I2[i][0][0]*RAD2DEG)
                    q1.append(I2[i][0][1]*RAD2DEG)
                    r1.append(I2[i][0][2]*RAD2DEG)
            else:
                time = self.getData(d, DID_PREINTEGRATED_IMU, 'time') # + self.getData(d, DID_GPS1_POS, 'towOffset')[-1]
                dt = self.getData(d, DID_PREINTEGRATED_IMU, 'dt')
                pqr0 = self.getData(d, DID_PREINTEGRATED_IMU, 'theta1')
                pqr1 = self.getData(d, DID_PREINTEGRATED_IMU, 'theta2')
                p0 = pqr0[:,0] / dt*RAD2DEG
                q0 = pqr0[:,1] / dt*RAD2DEG
                r0 = pqr0[:,2] / dt*RAD2DEG
                p1 = pqr1[:,0] / dt*RAD2DEG
                q1 = pqr1[:,1] / dt*RAD2DEG
                r1 = pqr1[:,2] / dt*RAD2DEG

            ax[0].plot(time, p0, label=self.log.serials[d])
            ax[1].plot(time, p1, label=self.log.serials[d])
            ax[2].plot(time, q0)
            ax[3].plot(time, q1)
            ax[4].plot(time, r0)
            ax[5].plot(time, r1)
        ax[0].legend(ncol=2)
        for a in ax:
            a.grid(True)
        self.saveFig(fig, 'pqrIMU')

    def imuAcc(self, fig=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(6, 1, sharex=True)
        self.configureSubplot(ax[0], 'Acc X 0', 'm/s^2')
        self.configureSubplot(ax[1], 'Acc X 1', 'm/s^2')
        self.configureSubplot(ax[2], 'Acc Y 0', 'm/s^2')
        self.configureSubplot(ax[3], 'Acc Y 1', 'm/s^2')
        self.configureSubplot(ax[4], 'Acc Z 0', 'm/s^2')
        self.configureSubplot(ax[5], 'Acc Z 1', 'm/s^2')
        fig.suptitle('Accelerometer - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            I1 = self.getData(d, DID_DUAL_IMU, 'I')[:,0]
            I2 = self.getData(d, DID_DUAL_IMU, 'I')[:,1]
            acc0x = []
            acc0y = []
            acc0z = []
            acc1x = []
            acc1y = []
            acc1z = []

            if np.shape(I1)[0] != 0:
                if len(self.getData(d, DID_GPS1_POS, 'towOffset')) == 0:
                    time = self.getData(d, DID_DUAL_IMU, 'time')
                else:
                    time = self.getData(d, DID_DUAL_IMU, 'time') + self.getData(d, DID_GPS1_POS, 'towOffset')[-1]
                for i in range(0, len(I1)):
                    acc0x.append(I1[i][1][0])
                    acc0y.append(I1[i][1][1])
                    acc0z.append(I1[i][1][2])
                    acc1x.append(I2[i][1][0])
                    acc1y.append(I2[i][1][1])
                    acc1z.append(I2[i][1][2])
            else:
                if len(self.getData(d, DID_GPS1_POS, 'towOffset')) == 0:
                    time = self.getData(d, DID_PREINTEGRATED_IMU, 'time')
                else:
                    time = self.getData(d, DID_PREINTEGRATED_IMU, 'time') + self.getData(d, DID_GPS1_POS, 'towOffset')[-1]
                dt = self.getData(d, DID_PREINTEGRATED_IMU, 'dt')
                acc0 = self.getData(d, DID_PREINTEGRATED_IMU, 'vel1')
                acc1 = self.getData(d, DID_PREINTEGRATED_IMU, 'vel2')
                acc0x = acc0[:,0] / dt
                acc0y = acc0[:,1] / dt
                acc0z = acc0[:,2] / dt
                acc1x = acc1[:,0] / dt
                acc1y = acc1[:,1] / dt
                acc1z = acc1[:,2] / dt

            ax[0].plot(time, acc0x, label=self.log.serials[d])
            ax[1].plot(time, acc1x)
            ax[2].plot(time, acc0y)
            ax[3].plot(time, acc1y)
            ax[4].plot(time, acc0z)
            ax[5].plot(time, acc1z)
        ax[0].legend(ncol=2)
        for a in ax:
            a.grid(True)
        self.saveFig(fig, 'accIMU')

    def imuPSD(self, fig=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(3, 2, sharex=True)
        self.configureSubplot(ax[0,0], 'AccX 0 ((m/s^2)^2)', 'Hz')
        self.configureSubplot(ax[1,0], 'AccX 1 ((m/s^2)^2)', 'Hz')
        self.configureSubplot(ax[0,0], 'AccX 0 ((m/s^2)^2)', 'Hz')
        self.configureSubplot(ax[0,1], 'AccX 1 ((m/s^2)^2)', 'Hz')
        self.configureSubplot(ax[1,0], 'AccY 0 ((m/s^2)^2)', 'Hz')
        self.configureSubplot(ax[1,1], 'AccY 1 ((m/s^2)^2)', 'Hz')
        self.configureSubplot(ax[2,0], 'AccZ 0 ((m/s^2)^2)', 'Hz')
        self.configureSubplot(ax[2,1], 'AccZ 1 ((m/s^2)^2)', 'Hz')
        # self.configureSubplot(ax[2], 'PQR 0', '(rad/s)^2')
        # self.configureSubplot(ax[2], 'PQR 1', '(rad/s)^2')
        fig.suptitle('Power Spectrum Density - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            I1 = self.getData(d, DID_DUAL_IMU, 'I')[:,0]
            I2 = self.getData(d, DID_DUAL_IMU, 'I')[:,1]
            acc0x = []
            acc0y = []
            acc0z = []
            acc1x = []
            acc1y = []
            acc1z = []

            if np.shape(I1)[0] != 0:
                time = self.getData(d, DID_DUAL_IMU, 'time')

                for i in range(0, len(I1)):
                    acc0x.append(I1[i][1][0])
                    acc0y.append(I1[i][1][1])
                    acc0z.append(I1[i][1][2])
                    acc1x.append(I2[i][1][0])
                    acc1y.append(I2[i][1][1])
                    acc1z.append(I2[i][1][2])

                    sp = np.fft.fft(acc0x)
                    freq = np.fft.fftfreq(time.shape[-1])
                    # plt.plot(freq, sp.real, freq, sp.imag)

            else:
                time = self.getData(d, DID_PREINTEGRATED_IMU, 'time')
                dt = self.getData(d, DID_PREINTEGRATED_IMU, 'dt')
                acc0 = self.getData(d, DID_PREINTEGRATED_IMU, 'vel1')
                acc1 = self.getData(d, DID_PREINTEGRATED_IMU, 'vel2')
                acc0x = acc0[:,0] / dt
                acc0y = acc0[:,1] / dt
                acc0z = acc0[:,2] / dt
                acc1x = acc1[:,0] / dt
                acc1y = acc1[:,1] / dt
                acc1z = acc1[:,2] / dt
                N = time.size
                psd0 = np.zeros((N//2, 3))
                psd1 = np.zeros((N//2, 3))
                # 1/T = frequency
                Fs = 1 / np.mean(dt)
                f = np.linspace(0, 0.5*Fs, N // 2)
                
                for i in range(3):
                    sp0 = np.fft.fft(acc0[:,i] / dt / 9.8)
                    sp0 = sp0[:N // 2]
                    # psd = abssp*abssp
                    # freq = np.fft.fftfreq(time.shape[-1])
#                    np.append(psd0, [1/N/Fs * np.abs(sp0)**2], axis=1)
                    psd0[:,i] = 1/N/Fs * np.abs(sp0)**2
                    psd0[1:-1,i] = 2 * psd0[1:-1,i]
                    sp1 = np.fft.fft(acc1[:,i] / dt / 9.8)
                    sp1 = sp1[:N // 2]
                    # psd = abssp*abssp
                    # freq = np.fft.fftfreq(time.shape[-1])
#                    np.append(psd0, [1/N/Fs * np.abs(sp0)**2], axis=1)
                    psd1[:,i] = 1/N/Fs * np.abs(sp1)**2
                    psd1[1:-1,i] = 2 * psd1[1:-1,i]


                # plt.plot(freq, sp.real, freq, sp.imag)

            ax[0,0].loglog(f, psd0[:,0])
            ax[1,0].loglog(f, psd0[:,1])
            ax[2,0].loglog(f, psd0[:,2])
            ax[0,1].loglog(f, psd1[:,0])
            ax[1,1].loglog(f, psd1[:,1])
            ax[2,1].loglog(f, psd1[:,2])

            # Set x limits
            xlim = [10, 500]
            ax[0,0].set_xlim(xlim)
            ax[1,0].set_xlim(xlim)
            ax[2,0].set_xlim(xlim)
            ax[0,1].set_xlim(xlim)
            ax[1,1].set_xlim(xlim)
            ax[2,1].set_xlim(xlim)

            # ax[0].plot(freq, sp.real, freq, sp.imag, label=self.log.serials[d])
            # ax[1].plot(time, acc1x)
            # ax[2].plot(time, acc0y)
            # ax[3].plot(time, acc1y)
            # ax[4].plot(time, acc0z)
            # ax[5].plot(time, acc1z)
        ax[0,0].legend(ncol=2)
        for i in range(3):
            for j in range(2):
                ax[i,j].grid(True)
        self.saveFig(fig, 'imuPSD')

    def magnetometer(self, fig=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(6, 1, sharex=True)

        self.configureSubplot(ax[0], 'Mag X 0', 'gauss')
        self.configureSubplot(ax[1], 'Mag X 1', 'gauss')
        self.configureSubplot(ax[2], 'Mag Y 0', 'gauss')
        self.configureSubplot(ax[3], 'Mag Y 1', 'gauss')
        self.configureSubplot(ax[4], 'Mag Z 0', 'gauss')
        self.configureSubplot(ax[5], 'Mag Z 1', 'gauss')
        fig.suptitle('Magnetometer - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            time0 = self.getData(d, DID_MAGNETOMETER_1, 'time') + self.getData(d, DID_GPS1_POS, 'towOffset')[-1]
            mag0 = self.getData(d, DID_MAGNETOMETER_1, 'mag')
            mag0x = mag0[:,0]
            mag0y = mag0[:,1]
            mag0z = mag0[:,2]
            ax[0].plot(time0, mag0x, label=self.log.serials[d])
            ax[2].plot(time0, mag0y)
            ax[4].plot(time0, mag0z)

        ax[0].legend(ncol=2)
        for a in ax:
            a.grid(True)
        self.saveFig(fig, 'magnetometer')


    def temp(self, fig=None):
        try:
            if fig is None:
                fig = plt.figure()
            ax = fig.subplots(2, 1, sharex=True)
            fig.suptitle('Temperature - ' + os.path.basename(os.path.normpath(self.log.directory)))

            for d in self.active_devs:
                time = getTimeFromTowMs(self.getData(d, DID_SYS_PARAMS, 'timeOfWeekMs'))
                ax[0].plot(time, self.getData(d, DID_SYS_PARAMS, 'imuTemp'), label=self.log.serials[d])
                ax[1].plot(time, self.getData(d, DID_SYS_PARAMS, 'baroTemp'))
            for a in ax:
                a.grid(True)
            self.saveFig(fig, 'Temp')
        except:
            print(RED + "problem plotting temp: " + sys.exc_info()[0] + RESET)

    def debugfArr(self, fig=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(5,2, sharex=True)
        fig.suptitle('Debug float Array - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            debug_f = self.getData(d, DID_DEBUG_ARRAY, 'f')
            for i in range(9):
                ax[i%5, i//5].set_ylabel('f[' + str(i) +']')
                ax[i%5, i//5].plot(debug_f[:,i], label=self.log.serials[d])
        ax[0,0].legend(ncol=2)
        for b in ax:
            for a in b:
                a.grid(True)

    def debugiArr(self, fig=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(5,2, sharex=True)
        fig.suptitle('Debug int array - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            debug_i = self.getData(d, DID_DEBUG_ARRAY, 'i')
            for i in range(9):
                ax[i%5, i//5].set_ylabel('i[' + str(i) +']')
                ax[i%5, i//5].plot(debug_i[:,i], label=self.log.serials[d])
        ax[0,0].legend(ncol=2)
        for b in ax:
            for a in b:
                a.grid(True)

    def debuglfArr(self, fig=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(3,1, sharex=True)
        fig.suptitle('Debug double Array - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            debug_lf = self.getData(d, DID_DEBUG_ARRAY, 'lf')
            for i in range(3):
                ax[i].set_ylabel('lf[' + str(i) +']')
                ax[i].plot(debug_lf[:,i], label=self.log.serials[d])
        ax[0].legend(ncol=2)
        for a in ax:
            a.grid(True)

    def magDec(self, fig=None):
        if fig is None:
            fig = plt.figure()

        ax = fig.subplots(2, 1, sharex=True)
        fig.suptitle('Magnetometer Declination - ' + os.path.basename(os.path.normpath(self.log.directory)))
        self.configureSubplot(ax[0], 'Declination', 'deg')
        self.configureSubplot(ax[1], 'Inclination', 'deg')

        for d in self.active_devs:
            time = getTimeFromTow(self.getData(d, DID_INL2_STATES, 'timeOfWeek'))
            declination = 180.0/np.pi * self.getData(d, DID_INL2_STATES, 'magDec')
            inclination = 180.0/np.pi * self.getData(d, DID_INL2_STATES, 'magInc')
            ax[0].plot(time, declination, label=self.log.serials[d])
            ax[1].plot(time, inclination)
        ax[0].legend(ncol=2)
        self.saveFig(fig, 'magDec')
        for a in ax:
            a.grid(True)

    def deltatime(self, fig=None):
        if fig is None:
            fig = plt.figure()

        ax = fig.subplots(3, 1, sharex=True)
        fig.suptitle('Timestamps - ' + os.path.basename(os.path.normpath(self.log.directory)))
        self.configureSubplot(ax[0], 'INS dt', 's')
        self.configureSubplot(ax[1], 'GPS dt', 's')
        self.configureSubplot(ax[2], 'IMU dt', 's')

        for d in self.active_devs:
            dtIns = self.getData(d, DID_INS_2, 'timeOfWeek')[1:] - self.getData(d, DID_INS_2, 'timeOfWeek')[0:-1]
            dtIns = dtIns / self.d
            timeIns = getTimeFromTow(self.getData(d, DID_INS_2, 'timeOfWeek')[1:])

            dtGps = 0.001*(self.getData(d, DID_GPS1_POS, 'timeOfWeekMs')[1:] - self.getData(d, DID_GPS1_POS, 'timeOfWeekMs')[0:-1])
            dtGps = dtGps / self.d
            timeGps = getTimeFromTowMs(self.getData(d, DID_GPS1_POS, 'timeOfWeekMs')[1:])

            dtImu = self.getData(d, DID_PREINTEGRATED_IMU, 'time')[1:] - self.getData(d, DID_PREINTEGRATED_IMU, 'time')[0:-1]
            dtImu = dtImu / self.d
            timeImu = getTimeFromTow(self.getData(d, DID_PREINTEGRATED_IMU, 'time')[1:] + self.getData(d, DID_GPS1_POS, 'towOffset')[-1])

            ax[0].plot(timeIns, dtIns, label=self.log.serials[d])
            ax[1].plot(timeGps, dtGps)
            ax[2].plot(timeImu, dtImu)

        self.setPlotYSpanMin(ax[0], 0.01)
        self.setPlotYSpanMin(ax[1], 0.01)
        self.setPlotYSpanMin(ax[2], 0.05)

        ax[0].legend(ncol=2)
        for a in ax:
            a.grid(True)
        self.saveFig(fig, 'deltatime')

    def gpsRawTime(self, fig=None):
        if fig is None:
            fig = plt.figure()

        ax = fig.subplots(6, 1, sharex=True)
        fig.suptitle('Timestamps - ' + os.path.basename(os.path.normpath(self.log.directory)))
        self.configureSubplot(ax[0], 'GPS1 Raw dt', 's')
        self.configureSubplot(ax[1], 'GPS2 Raw dt', 's')
        self.configureSubplot(ax[2], 'GPS Base Raw dt', 's')
        self.configureSubplot(ax[3], 'GPS1 Raw Number of Satellites Observed', 's')
        self.configureSubplot(ax[4], 'GPS2 Raw Number of Satellites Observed', 's')
        self.configureSubplot(ax[5], 'GPS Base Raw Number of Satellites Observed', 's')

        for d in self.active_devs:
            N1 = len(self.log.data[d, DID_GPS1_RAW][0])
            N2 = len(self.log.data[d, DID_GPS2_RAW][0])
            NB = len(self.log.data[d, DID_GPS_BASE_RAW][0])
            tgps1 = np.zeros(N1)
            nsat1 = np.zeros(N1)
            tgps2 = np.zeros(N2)
            nsat2 = np.zeros(N2)
            tgpsB = np.zeros(NB)
            nsatB = np.zeros(NB)
            cnt = 0
            for iobs in range(N1):
                ns = round(len(self.log.data[d, DID_GPS1_RAW][0][iobs]) * 0.5) # 0.5 because there is a bug that pads half of the data with zeros
                t0 = self.log.data[d, DID_GPS1_RAW][0][iobs]['time']['time'][-1] + \
                     self.log.data[d, DID_GPS1_RAW][0][iobs]['time']['sec'][-1]
                nsat1[cnt] = nsat1[cnt] + ns
                tgps1[cnt] = t0
                if iobs < N1 - 1:
                    t1 = self.log.data[d, DID_GPS1_RAW][0][iobs + 1]['time']['time'][-1] + \
                         self.log.data[d, DID_GPS1_RAW][0][iobs + 1]['time']['sec'][-1]
                    if t1 > t0 + 0.01:
                        cnt = cnt + 1
            tgps1 = tgps1[0: cnt + 1]
            nsat1 = nsat1[0: cnt + 1]
            cnt = 0
            for iobs in range(N2):
                ns = round(len(self.log.data[d, DID_GPS2_RAW][0][iobs]) * 0.5) # 0.5 because there is a bug that pads half of the data with zeros
                t0 = self.log.data[d, DID_GPS2_RAW][0][iobs]['time']['time'][-1] + \
                     self.log.data[d, DID_GPS2_RAW][0][iobs]['time']['sec'][-1]
                nsat2[cnt] = nsat2[cnt] + ns
                tgps2[cnt] = t0
                if iobs < N2 - 1:
                    t1 = self.log.data[d, DID_GPS2_RAW][0][iobs + 1]['time']['time'][-1] + \
                         self.log.data[d, DID_GPS2_RAW][0][iobs + 1]['time']['sec'][-1]
                    if t1 > t0 + 0.01:
                        cnt = cnt + 1
            tgps2 = tgps2[0: cnt + 1]
            nsat2 = nsat2[0: cnt + 1]
            cnt = 0
            for iobs in range(NB):
                ns = round(len(self.log.data[d, DID_GPS_BASE_RAW][0][iobs]) * 0.5) # 0.5 because there is a bug that pads half of the data with zeros
                t0 = self.log.data[d, DID_GPS_BASE_RAW][0][iobs]['time']['time'][-1] + \
                     self.log.data[d, DID_GPS_BASE_RAW][0][iobs]['time']['sec'][-1]
                nsatB[cnt] = nsatB[cnt] + ns
                tgpsB[cnt] = t0
                if iobs < NB - 1:
                    t1 = self.log.data[d, DID_GPS_BASE_RAW][0][iobs + 1]['time']['time'][-1] + \
                         self.log.data[d, DID_GPS_BASE_RAW][0][iobs + 1]['time']['sec'][-1]
                    if t1 > t0 + 0.01:
                        cnt = cnt + 1
            tgpsB = tgpsB[0: cnt + 1]
            nsatB = nsatB[0: cnt + 1]
            dtGps1 = tgps1[1:] - tgps1[0:-1]
#            dtGps1 = dtGps1 / self.d
            dtGps2 = tgps2[1:] - tgps2[0:-1]
#            dtGps2 = dtGps2 / self.d
            dtGpsB = tgpsB[1:] - tgpsB[0:-1]

            ax[0].plot(tgps1[1:], dtGps1, label=self.log.serials[d])
            ax[1].plot(tgps2[1:], dtGps2)
            ax[2].plot(tgpsB[1:], dtGpsB)
            ax[3].plot(tgps1, nsat1, label=self.log.serials[d])
            ax[4].plot(tgps2, nsat2)
            ax[5].plot(tgpsB, nsatB)

        self.setPlotYSpanMin(ax[0], 2.0)
        self.setPlotYSpanMin(ax[1], 2.0)
        self.setPlotYSpanMin(ax[2], 2.0)
        self.setPlotYSpanMin(ax[3], 25)
        self.setPlotYSpanMin(ax[4], 25)
        self.setPlotYSpanMin(ax[5], 25)

        ax[0].legend(ncol=2)
        for a in ax:
            a.grid(True)
        self.saveFig(fig, 'gpsRawTime')

    def ekfBiases(self, fig=None):
        if fig is None:
            fig = plt.figure()
        ax = fig.subplots(4, 2, sharex=True)
        self.configureSubplot(ax[0,0], 'bias P', 'deg/s')
        self.configureSubplot(ax[1,0], 'bias Q', 'deg/s')
        self.configureSubplot(ax[2,0], 'bias R', 'deg/s')
        self.configureSubplot(ax[3,0], 'bias Barometer', 'm')

        self.configureSubplot(ax[0,1], 'bias acc X', 'm/s^2')
        self.configureSubplot(ax[1,1], 'bias acc Y', 'm/s^2')
        self.configureSubplot(ax[2,1], 'bias acc Z', 'm/s^2')
        fig.suptitle('EKF Biases - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            time = getTimeFromTow(self.getData(d, DID_INL2_STATES, 'timeOfWeek'))
            ax[0,0].plot(time, self.getData(d, DID_INL2_STATES, 'biasPqr')[:, 0]*180.0/np.pi, label=self.log.serials[d])
            ax[1,0].plot(time, self.getData(d, DID_INL2_STATES, 'biasPqr')[:, 1]*180.0/np.pi)
            ax[2,0].plot(time, self.getData(d, DID_INL2_STATES, 'biasPqr')[:, 2]*180.0/np.pi)
            ax[3,0].plot(time, self.getData(d, DID_INL2_STATES, 'biasBaro'), label=self.log.serials[d])

            ax[0,1].plot(time, self.getData(d, DID_INL2_STATES, 'biasAcc')[:, 0], label=self.log.serials[d])
            ax[1,1].plot(time, self.getData(d, DID_INL2_STATES, 'biasAcc')[:, 1])
            ax[2,1].plot(time, self.getData(d, DID_INL2_STATES, 'biasAcc')[:, 2])

        ax[0,0].legend(ncol=2)
        for a in ax:
            for b in a:
                b.grid(True)
        self.saveFig(fig, 'ekfBiases')

    def rtkResiduals(self, type, page, fig=None):
        if fig is None:
            fig = plt.figure()

        if type == 'phase':
            did = DID_RTK_PHASE_RESIDUAL
        elif type == 'code':
            did = DID_RTK_CODE_RESIDUAL

        sat_ids = np.unique(self.log.data[0, did]['sat_id_j'])
        sat_ids = sat_ids[sat_ids != 0][page*6:(page+1)*6]

        ax = fig.subplots(6, 1, sharex=True)
        fig.suptitle(type + ' Residuals Page ' + str(page+1) + ' - ' + os.path.basename(os.path.normpath(self.log.directory)))

        for i, id in enumerate(sat_ids):
            if id == 0: continue
            ax[i].set_ylabel(str(id))
            for d in self.active_devs:
                idx = np.where(self.getData(d, did, 'sat_id_j') == id)
                time_idx = idx[0]
                sat_state_idx = idx[1]
                time = np.array(getTimeFromGTime(self.getData(d, did, 'time')))[time_idx]
                residuals = self.getData(d, did, 'v')[time_idx, sat_state_idx]
                residuals[np.abs(residuals) > 1e6] = np.nan
                ax[i].plot(time, residuals, label=self.log.serials[d])
        ax[0].legend(ncol=2)

    def rtkDebug(self, fig=None):
        if fig is None:
            fig = plt.figure()

        fields = list(self.log.data[0, DID_RTK_DEBUG].dtype.names)
        fields.remove('time')
        num_plots = 0
        for field in fields:
            dat = self.log.data[0, DID_RTK_DEBUG][field][0]
            if isinstance(dat, np.ndarray):
                num_plots += len(dat)
            else:
                num_plots += 1

        cols = 4
        rows = math.ceil(num_plots/float(cols))
        ax = fig.subplots(rows, cols, sharex=True)
        fig.suptitle('RTK Debug Counters - ' + os.path.basename(os.path.normpath(self.log.directory)))

        for d in self.active_devs:
            time = np.array(getTimeFromGTime(self.getData(d, DID_RTK_DEBUG, 'time')))
            valid = time > datetime.datetime.strptime('2017', "%Y")

            # Use index rather than time
            if 0:
                time = np.arange(0, len(time))

            i = 0
            for field in fields:
                data = self.getData(d, DID_RTK_DEBUG, field)[valid]
                if (len(data) == 0):
                    continue
                if isinstance(data[0], np.ndarray):
                    for j in range(len(data[0])):
                        ax[ i%rows, i//rows].set_title(field + "_" + str(j))
                        ax[ i % rows, i // rows ].plot(time[valid], data[:,j], label=self.log.serials[d])
                        i += 1
                else:
                    ax[i % rows, i // rows].set_title(field)
                    ax[i % rows, i // rows].title.set_fontsize(8)
                    # for item in ax[i % rows, i // rows].get_yticklabels():
                        # item.set_fontsize(8)
                    ax[i % rows, i // rows].plot(time[valid], data, label=self.log.serials[d])
                    i += 1
        ax[0,0].legend(ncol=2)

    def rtkDebug2(self, fig=None):
        if fig is None:
            fig = plt.figure()

        ax = fig.subplots(6, 4, sharex=True)

        #max_num_biases = 22 #np.array(self.getData(self.active_devs[0], DID_RTK_DEBUG_2, 'num_biases'))
        max_num_biases = self.getData(0, DID_RTK_DEBUG_2, 'num_biases')[-1]
        for r in range(0,6):
            for c in range(0,4):
                self.configureSubplot(ax[r,c], '', '')

        fig.suptitle('RTK Debug2 - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            time = np.array(getTimeFromGTime(self.getData(d, DID_RTK_DEBUG_2, 'time')))
            ib = 0
            for r in range(0, 6):
                for c in range(0, 4):
                    if ib < max_num_biases:
                        ax[r,c].plot(time, self.getData(d, DID_RTK_DEBUG_2, 'satBiasFloat')[:, c + r * 4], label=self.log.serials[d])
                        r1 = r
                        c1 = c
                    ib = ib + 1

        # Show serial numbers
        ax[r1,c1].legend(ncol=2)

        for a in ax:
            for b in a:
                b.grid(True)

    def rtkDebug2Sat(self, fig=None):
        if fig is None:
            fig = plt.figure()

        ax = fig.subplots(6, 4, sharex=True)

        max_num_biases = self.getData(0, DID_RTK_DEBUG_2, 'num_biases')[-1]
        for r in range(0,6):
            for c in range(0,4):
                self.configureSubplot(ax[r,c], '', '')

        fig.suptitle('RTK Debug2 - Sat# - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            time = np.array(getTimeFromGTime(self.getData(d, DID_RTK_DEBUG_2, 'time')))
            ib = 0
            for r in range(0, 6):
                for c in range(0, 4):
                    if ib < max_num_biases:
                        ax[r,c].plot(time, self.getData(d, DID_RTK_DEBUG_2, 'sat')[:, c + r * 4], label=self.log.serials[d])
                        r1 = r
                        c1 = c
                    ib = ib + 1

        # Show serial numbers
        ax[r1,c1].legend(ncol=2)

        for a in ax:
            for b in a:
                b.grid(True)

    def rtkDebug2Std(self, fig=None):
        if fig is None:
            fig = plt.figure()

        ax = fig.subplots(6, 4, sharex=True)

        max_num_biases = self.getData(0, DID_RTK_DEBUG_2, 'num_biases')[-1]
        for r in range(0,6):
            for c in range(0,4):
                self.configureSubplot(ax[r,c], '', '')

        fig.suptitle('RTK Debug 2 - Sat Bias Std - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            time = np.array(getTimeFromGTime(self.getData(d, DID_RTK_DEBUG_2, 'time')))
            ib = 0
            for r in range(0, 6):
                for c in range(0, 4):
                    if ib < max_num_biases:
                        ax[r,c].plot(time, self.getData(d, DID_RTK_DEBUG_2, 'satBiasStd')[:, c + r * 4], label=self.log.serials[d])
                        r1 = r
                        c1 = c
                    ib = ib + 1

        # Show serial numbers
        ax[r1,c1].legend(ncol=2)

        for a in ax:
            for b in a:
                b.grid(True)

    def rtkDebug2Lock(self, fig=None):
        if fig is None:
            fig = plt.figure()

        ax = fig.subplots(6, 4, sharex=True)

        max_num_biases = self.getData(0, DID_RTK_DEBUG_2, 'num_biases')[-1]
        for r in range(0,6):
            for c in range(0,4):
                self.configureSubplot(ax[r,c], '', '')

        fig.suptitle('RTK Debug 2 - Lock Count - ' + os.path.basename(os.path.normpath(self.log.directory)))
        for d in self.active_devs:
            time = np.array(getTimeFromGTime(self.getData(d, DID_RTK_DEBUG_2, 'time')))
            ib = 0
            for r in range(0, 6):
                for c in range(0, 4):
                    if ib < max_num_biases:
                        ax[r,c].plot(time, self.getData(d, DID_RTK_DEBUG_2, 'satLockCnt')[:, c + r * 4], label=self.log.serials[d])
                        r1 = r
                        c1 = c
                    ib = ib + 1

        # Show serial numbers
        ax[r1,c1].legend(ncol=2)

        for a in ax:
            for b in a:
                b.grid(True)

    def wheelEncoder(self, fig=None):
        if fig is None:
            fig = plt.figure()

        fig.suptitle('Wheel Encoder - ' + os.path.basename(os.path.normpath(self.log.directory)))
        ax = fig.subplots(4, 1, sharex=True)
        titles = ['Left Wheel Angle', 'Right Wheel Angle', 'Left Wheel Velocity', 'Right Wheel Velocity']
        fields = ['theta_l', 'theta_r', 'omega_l', 'omega_r']

        for d in self.active_devs:
            time = np.array(getTimeFromTow(self.getData(d, DID_WHEEL_ENCODER, 'timeOfWeek')))
            for i, a in enumerate(ax):
                a.plot(time, self.getData(d, DID_WHEEL_ENCODER, fields[i]), label=self.log.serials[d])
                if i == 0:
                    a.legend(ncol=2)

        for i, a in enumerate(ax):
            a.set_ylabel(fields[i])
            a.set_title(titles[i])
            a.grid(True)




    def showFigs(self):
        if self.show:
            plt.show()


if __name__ == '__main__':
    np.set_printoptions(linewidth=200)
    home = expanduser("~")
    file = open(home + "/Documents/Inertial_Sense/config.yaml", 'r')
    config = yaml.load(file)
    directory = config["directory"]
    directory = "/home/superjax/Code/IS-src/cpp/SDK/CLTool/build/IS_logs"
    directory = r"C:\Users\quaternion\Downloads\20181218_Compass_Drive\20181218 Compass Drive\20181218_101023"
    serials = config['serials']

    log2 = Log()
    log2.load(directory, serials)

    plotter = logPlot(True, True, 'svg', log2)
    plotter.setDownSample(5)
    # plotter.deltatime()
    # plotter.debugfArr()
    # plotter.debugiArr()
    # plotter.debuglfArr()
    # plotter.gpsStats()
    # plotter.rtkStats()
    # plotter.insStatus()
    # plotter.hdwStatus()
    # plotter.temp()
    # plotter.att()
    # plotter.lla()
    # plotter.uvw()
    # plotter.ned()
    # plotter.nedMap()
    # plotter.magDec()
    plotter.rtkDebug()
    #plotter.wheelEncoder()

    plotter.showFigs()
