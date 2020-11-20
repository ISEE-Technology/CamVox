#!/usr/bin/python3

import sys, os, shutil
from PyQt5 import QtCore
from PyQt5.QtWidgets import QWidget, QDialog, QApplication, QPushButton, QVBoxLayout, QLineEdit, QTreeView, QFileSystemModel,\
    QHBoxLayout, QGridLayout, QMainWindow, QSizePolicy, QSpacerItem, QFileDialog, QMessageBox, QLabel, QRadioButton,\
    QAbstractItemView, QMenu, QTableWidget,QTableWidgetItem, QSpinBox, QSpacerItem
from PyQt5.QtGui import QMovie, QPicture, QIcon, QDropEvent, QPixmap, QImage
from PyQt5.Qt import QApplication, QClipboard, QStyle
import json
import io

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
import matplotlib.pyplot as plt
import numpy as np
from threading import Thread

from logReader import Log
from logPlotter import logPlot
import traceback
import yaml
import sys

sys.path.append('..')
sys.path.append('../supernpp/')
sys.path.append('../ci_hdw/')
from pylib.data_sets import *
import subprocess
import re

START_MODE_HOT = 0
START_MODE_COLD = 1
START_MODE_FACTORY = 2

def openFolderWithFileBrowser(path):
    if sys.platform == 'darwin':
        subprocess.check_call(['open', '--', path])
    elif sys.platform == 'linux':
        subprocess.check_call(['xdg-open', path])
    elif sys.platform == 'win32':
        # subprocess.check_call(['explorer', path])     # Not stable
        subprocess.Popen(r'explorer '+path)

def cleanFolder(path, toplevel=True):
    containsDAT = False
    containsSDAT = False
    for fname in os.listdir(path):
        if fname.endswith('.dat'):
            containsDAT = True
        if fname.endswith('.sdat'):
            containsSDAT = True

    for fname in os.listdir(path):
        fpath = os.path.join(path, fname)
        if os.path.isdir( fpath ):
            if fname == 'post_processed':
                removeDirectory(fpath)
            else:
                cleanFolder(fpath, False)
        else:
            # Remove .csv if .dat or .sdat exist
            if containsDAT or containsSDAT:
                if fname.endswith('.csv'):
                    print('Deleting: ' + fpath)
                    os.remove(fpath)
            # Remove .sdat if .dat exist
            if containsDAT:
                if fname.endswith('.sdat'):
                    print('Deleting: ' + fpath)
                    os.remove(fpath)
    if toplevel:
        print('Finished Cleaning!')

def removeDirectory(fpath):
    print('Removing Directory: ' + fpath)
    shutil.rmtree(fpath)

# startMode 0=hot, 1=cold, 2=factory
def setDataInformationDirectory(path, startMode=START_MODE_HOT):
    settings_filename = os.path.expanduser("~") + '/Documents/Inertial_Sense/dataInformation.json'

    if settings_filename is not None:
        # with open(settings_filename, 'r') as f:
        #     data = json.load(f)
        #     f.close()
        data = {}
        data['dataInfo'] = {}
        data['dataInfo']['dataDirectory'] = os.path.dirname(path).replace('\\','/')
        data['dataInfo']['subDirectories'] = [os.path.basename(path)]
        serialnumbers = []
        for root, dirs, files in os.walk(path):
            for filename in files:
                if "LOG_SN" in filename:
                    serialnum = filename[4:11]
                    if serialnum not in serialnumbers:
                        serialnumbers += [serialnum]

        data['processData'] = {}
        data['processData']['datasets'] = [{}]
        data['processData']['datasets'][0]['SerialNumbers'] = serialnumbers
        data['processData']['datasets'][0]['folder'] = os.path.basename(path)
        data['processData']['datasets'][0]['logType'] = 'DAT'
        if startMode == START_MODE_HOT:
            data['processData']['datasets'][0]['startMode'] = 'HOT'
        elif startMode == START_MODE_COLD:
            data['processData']['datasets'][0]['startMode'] = 'COLD'
        else:
            data['processData']['datasets'][0]['startMode'] = 'FACTORY'

        # os.remove(settings_filename)
        with open(settings_filename, 'w') as f:
            json.dump(data, f, indent=4)

def verArrayToString(array):
    return str(array[0]) + '.' + str(array[1]) + '.' + str(array[2]) #+ '.' + str(array[3])

def dateTimeArrayToString(date, time):
    return str(date[1]+2000) + '-' + f'{date[2]:02}' + '-' + f'{date[3]:02}' + ' ' + f'{time[0]:02}' + ':' + f'{time[1]:02}' + ':' + f'{time[2]:02}'

class DeviceInfoDialog(QDialog):

    def __init__(self, log, parent=None):
        super(DeviceInfoDialog, self).__init__(parent)
        self.setWindowTitle("Device Info")

        if np.shape(log.data[0,DID_DEV_INFO])[0] == 0:
            self.label = QLabel('No DID_DEV_INFO data available.')
            self.mainlayout = QVBoxLayout()
            self.mainlayout.addWidget(self.label)
            self.setLayout(self.mainlayout)
            self.resize(400, 200)
            return

        self.table = QTableWidget()
        nfields = len(log.data[0, DID_DEV_INFO].dtype.names)
        field_names = []
        vals = []

        self.table.setColumnCount(9)
        self.table.setHorizontalHeaderLabels(['Serial#','Hardware','Firmware','Build','Protocol','Repo','Build Date','Manufacturer','AddInfo'])

        for d, dev in enumerate(log.data):
            data = dev[DID_DEV_INFO][0]
            self.table.setRowCount(d+1)
            self.table.setItem(d, 0, QTableWidgetItem(str(data[1])))                         # Serial#
            self.table.setItem(d, 1, QTableWidgetItem(verArrayToString(data[2])))  # Hardware version
            self.table.setItem(d, 2, QTableWidgetItem(verArrayToString(data[3])))  # Firmware version
            self.table.setItem(d, 3, QTableWidgetItem(str(data[4])))   # Build
            self.table.setItem(d, 4, QTableWidgetItem(verArrayToString(data[5])))  # Protocol
            self.table.setItem(d, 5, QTableWidgetItem(str(data[6])))   # Repo
            self.table.setItem(d, 6, QTableWidgetItem(dateTimeArrayToString(data[8], data[9]))) # Build Date & Time
            self.table.setItem(d, 7, QTableWidgetItem(data[7].decode('UTF-8')))         # Manufacturer
            self.table.setItem(d, 8, QTableWidgetItem(data[10].decode('UTF-8')))        # Additional Info

        self.mainlayout = QVBoxLayout()
        self.mainlayout.addWidget(self.table)
        self.setLayout(self.mainlayout)
        self.resize(2000, 800)

class FlashConfigDialog(QDialog):
    def __init__(self, log, parent=None):
        super(FlashConfigDialog, self).__init__(parent)
        self.setWindowTitle("Flash Config")

        if np.shape(log.data[0,DID_FLASH_CONFIG])[0] == 0:
            self.label = QLabel('No DID_FLASH_CONFIG data available.')
            self.mainlayout = QVBoxLayout()
            self.mainlayout.addWidget(self.label)
            self.setLayout(self.mainlayout)
            self.resize(400, 200)
            return

        self.table = QTableWidget()
        nfields = len(log.data[0, DID_FLASH_CONFIG].dtype.names)
        field_names = []
        vals = []

        for d, dev in enumerate(log.data):
            vals.append([])
            for f, field in enumerate(dev[DID_FLASH_CONFIG].dtype.names):
                if isinstance(dev[DID_FLASH_CONFIG][field][0], np.ndarray):
                    length = len(dev[DID_FLASH_CONFIG][field][0])
                    if d == 0: nfields +=  length-1 # add extra rows for arrays in flash config
                    for i in range(length):
                        if d == 0: field_names.append(field + "[" + str(i) + "]")
                        vals[d].append(dev[DID_FLASH_CONFIG][field][0][i])
                else:
                    if d == 0: field_names.append(field)
                    vals[d].append(dev[DID_FLASH_CONFIG][field][0])

        self.table.setRowCount(nfields)
        self.table.setColumnCount(log.numDev)

        self.table.setHorizontalHeaderLabels([str(ser) for ser in log.serials])
        self.table.setVerticalHeaderLabels(field_names)

        hex_fields = ['ioConfig', 'cBrdConfig', 'RTKCfgBits', 'sysCfgBits']
        for d in range(log.numDev):
            for f, field in enumerate(field_names):
                if field in hex_fields:
                    self.table.setItem(f, d, QTableWidgetItem(hex(vals[d][f])))
                else:
                    self.table.setItem(f, d, QTableWidgetItem(str(vals[d][f])))

        self.mainlayout = QVBoxLayout()
        self.mainlayout.addWidget(self.table)
        self.setLayout(self.mainlayout)
        self.resize(1280, 900)




class LogInspectorWindow(QMainWindow):
    def __init__(self, configFilePath, parent=None):
        super(LogInspectorWindow, self).__init__(parent)
        self.initMatPlotLib()
        self.configFilePath = configFilePath

        if os.path.exists(self.configFilePath):
            # config.yaml found.  Read from file.
            file = open(self.configFilePath, 'r')
            self.config = yaml.load(file)
            file.close()
        else:
            # config.yaml not found.  Create new file.
            self.config = {}
            self.config['logs_directory'] = os.path.join(os.path.expanduser("~"), "Documents", "Inertial_Sense", "Logs")
            self.config['directory'] = ""
            self.config['serials'] = ["ALL"]
            file = open(self.configFilePath, 'w')
            yaml.dump(self.config, file)
            file.close()

        self.currently_selected = 'posNEDMap'
        self.downsample = 5
        self.plotargs = None
        self.log = None
        self.plotter = None


    def initMatPlotLib(self):
        self.figure = plt.figure()
        self.canvas = FigureCanvas(self.figure)
        self.toolbar = NavigationToolbar(self.canvas, self)
        self.figure.subplots_adjust(left=0.05, right=0.99, bottom=0.05, top=0.91, wspace=0.2, hspace=0.2)

    def addButton(self, name, function, multithreaded=True, layout=None):
        setattr(self, name+"button", QPushButton(name))
        # if multithreaded:
            # setattr(self, name+"buttonThread", Thread(target=function))
            # getattr(self, name+"button").pressed.connect(self.startLoadingIndicator)
            # getattr(self, name+"button").released.connect(getattr(self, name+"buttonThread").start)
        # else:
        getattr(self, name + "button").clicked.connect(function)
        # getattr(self, name + "button").setMinimumWidth(220)
        if layout is None:
            if self.buttonLayoutRightCol.count() < self.buttonLayoutMiddleCol.count():
                self.buttonLayoutRightCol.addWidget(getattr(self, name + "button"))
            elif self.buttonLayoutMiddleCol.count() < self.buttonLayoutLeftCol.count():
                self.buttonLayoutMiddleCol.addWidget(getattr(self, name + "button"))
            else:
                self.buttonLayoutLeftCol.addWidget(getattr(self, name + "button"))
        else:
            layout.addWidget(getattr(self, name + "button"))

    def updatePlot(self):
        self.plot(self.currently_selected, self.plotargs)

    def updateWindowTitle(self):
        if np.shape(self.log.data[0,DID_DEV_INFO])[0] != 0:
            info = self.log.data[0,DID_DEV_INFO][0]
            infoStr = 'SN' + str(info[1]) + ', H:' + verArrayToString(info[2]) + ', F:' + verArrayToString(info[3]) + ' build ' + str(info[4]) + ', ' + dateTimeArrayToString(info[8], info[9]) + ', ' + info[10].decode('UTF-8')
            self.setWindowTitle("LogInspector  -  " + infoStr)

    def choose_directory(self):
        log_dir = config['logs_directory']
        directory = QFileDialog.getExistingDirectory(parent=self, caption='Choose Log Directory', directory=log_dir)

        if directory != '':
            try:
                self.load(directory)
            except Exception as e:
                msg = QMessageBox()
                msg.setIcon(QMessageBox.Critical)
                msg.setText("Unable to load log: " + e.__str__())
                msg.setDetailedText(traceback.format_exc())
                msg.exec()

    def load(self, directory):
        print("loading files from " + directory)
        # if self.log is None:
        self.log = Log()
        self.log.load(directory)
        print("done loading")
        self.plotter = logPlot(False, False, 'svg', self.log)
        self.plotter.setDownSample(self.downsample)
        # str = ''
        # if self.log.navMode:
        #     str += 'NAV '
        # if self.log.rtk:
        #     str += 'RTK '
        # if self.log.compassing:
        #     str += 'Comp '
        # self.statusLabel.setText(str)
        self.updatePlot()
        self.updateWindowTitle()
        self.stopLoadingIndicator()

    def setupUi(self):
        self.setObjectName("LogInspector")
        self.setWindowTitle("LogInspector")
        self.resize(1280, 900)
        self.setWindowFlags(self.windowFlags() |
                                  QtCore.Qt.WindowSystemMenuHint |
                                  QtCore.Qt.WindowMinMaxButtonsHint)
        self.setWindowIcon(QIcon("assets/Magnifying_glass_icon.png"))

        # MainWindow.showMaximized()

        self.createFileTree()
        self.createButtonColumn()
        self.formatButtonColumn()
        self.createBottomToolbar()

        self.figureLayout = QVBoxLayout()
        self.figureLayout.addWidget(self.canvas)
        self.figureLayout.addLayout(self.toolLayout)
        self.figureLayout.setStretchFactor(self.canvas, 1)

        layout = QHBoxLayout()
        layout.addLayout(self.controlLayout)
        layout.addLayout(self.figureLayout)
        layout.setStretch(1, 1)
        widget = QWidget()
        widget.setLayout(layout)
        self.setCentralWidget(widget)
        # self.resize(1280, 900)
        self.resize(1450, 1000)
        self.setAcceptDrops(True)

    def createButtonColumn(self):
        self.controlLayout = QVBoxLayout()
        self.buttonLayoutLeftCol = QVBoxLayout()
        self.buttonLayoutMiddleCol = QVBoxLayout()
        self.buttonLayoutRightCol = QVBoxLayout()
        self.addButton('Pos NED Map', lambda: self.plot('posNEDMap'))
        self.addButton('Pos NED', lambda: self.plot('posNED'))
        self.addButton('Pos LLA', lambda: self.plot('posLLA'))
        self.addButton('GPS LLA', lambda: self.plot('llaGps'))
        self.addButton('Vel NED', lambda: self.plot('velNED'))
        self.addButton('Vel UVW', lambda: self.plot('velUVW'))
        self.addButton('Attitude', lambda: self.plot('attitude'))
        self.addButton('Heading', lambda: self.plot('heading'))
        self.addButton('INS Status', lambda: self.plot('insStatus'))
        self.addButton('HDW Status', lambda: self.plot('hdwStatus'))
        self.addButton('GPS 1 Stats', lambda: self.plot('gpsStats'))
        self.addButton('GPS 2 Stats', lambda: self.plot('gps2Stats'))
        self.addButton('RTK Pos Stats', lambda: self.plot('rtkPosStats'))
        self.addButton('RTK Cmp Stats', lambda: self.plot('rtkCmpStats'))
        self.addButton('Flash Config', lambda: self.showFlashConfig())
        self.addButton('Device Info', lambda: self.showDeviceInfo())
        self.addButton('IMU PQR', lambda: self.plot('imuPQR'))
        self.addButton('IMU Accel', lambda: self.plot('imuAcc'))
        self.addButton('IMU PSD', lambda: self.plot('imuPSD'))
        self.addButton('Magnetometer', lambda: self.plot('magnetometer'))
        self.addButton('Temp', lambda: self.plot('temp'))

    def formatButtonColumn(self):
        self.buttonLayoutLeftCol.setAlignment(QtCore.Qt.AlignTop)
        self.buttonLayoutMiddleCol.setAlignment(QtCore.Qt.AlignTop)
        self.buttonLayoutRightCol.setAlignment(QtCore.Qt.AlignTop)
        self.buttonColumnLayout = QHBoxLayout()
        self.buttonColumnLayout.addLayout(self.buttonLayoutLeftCol)
        self.buttonColumnLayout.addLayout(self.buttonLayoutMiddleCol)
        self.buttonColumnLayout.addLayout(self.buttonLayoutRightCol)
        self.controlLayout.addLayout(self.buttonColumnLayout)
        self.controlDirLayout = QHBoxLayout();
        self.controlDirLayout.addWidget(self.dirLineEdit)
        self.controlLayout.addLayout(self.controlDirLayout)
        self.controlLayout.addWidget(self.fileTree)
        # self.buttonLayout.addItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))
        # self.addButton('load', self.choose_directory, multithreaded=False)


    def createBottomToolbar(self):
        self.toolLayout = QHBoxLayout()
        self.toolLayout.addWidget(self.toolbar)

        self.loadingIndictator = QLabel()
        self.loadingMovie = QMovie('assets/loader.gif')
        self.emptyLoadingPicture = QPicture()
        self.emptyLoadingPicture.load('assets/empty_loader.png')
        self.stopLoadingIndicator()
        self.toolLayout.addWidget(self.loadingIndictator)

        self.toolLayout.addItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))
        # self.toolLayout.addWidget(QSpacerItem(150, 10, QSizePolicy.Expanding))

        self.copyImagePushButton = QPushButton()
        # self.copyImagePushButton.setText("Copy")
        # self.copyImagePushButton.setMinimumWidth(1)
        # self.copyImagePushButton.style().standardIcon(QStyle.SP_DialogOpenButton)
        self.copyImagePushButton.setIcon(self.style().standardIcon(QStyle.SP_DialogSaveButton))
        self.toolLayout.addWidget(self.copyImagePushButton)
        self.copyImagePushButton.clicked.connect(self.copyPlotToClipboard)

        downsampleLabel = QLabel()
        downsampleLabel.setText("DS")
        self.downSampleInput = QSpinBox()
        self.downSampleInput.setValue(self.downsample)
        self.toolLayout.addWidget(downsampleLabel)
        self.toolLayout.addWidget(self.downSampleInput)
        self.downSampleInput.valueChanged.connect(self.changeDownSample)

        self.statusLabel = QLabel()
        self.toolLayout.addWidget(self.statusLabel)

    def changeDownSample(self, val):
        self.downsample = val
        self.plotter.setDownSample(self.downsample)
        self.updatePlot()

    def copyPlotToClipboard(self):
        # pixmap = QPixmap.grabWidget(self.canvas)
        # QApplication.clipboard().setPixmap(pixmap)
        # pixmap.save('test.png')

        # store the image in a buffer using savefig(), this has the
        # advantage of applying all the default savefig parameters
        # such as background color; those would be ignored if you simply
        # grab the canvas using Qt
        buf = io.BytesIO()
        self.figure.savefig(buf)

        QApplication.clipboard().setImage(QImage.fromData(buf.getvalue()))
        buf.close()

    def startLoadingIndicator(self):
        self.loadingIndictator.setMovie(self.loadingMovie)
        self.loadingMovie.start()

    def dragEnterEvent(self, e):
        if (e.mimeData().hasUrls()):
            e.acceptProposedAction()


    def dropEvent(self, e):
        try:
            directory = e.mimeData().urls()[0].toLocalFile()
            self.load(directory)
        except Exception as e:
            self.showError(e)

    def showError(self, e):
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Critical)
        msg.setText("Unable to load log: " + e.__str__())
        msg.setDetailedText(traceback.format_exc())
        msg.exec()

    def createFileTree(self):
        self.dirModel = QFileSystemModel()
        self.dirModel.setRootPath(self.config["logs_directory"])
        self.dirModel.setFilter(QtCore.QDir.Dirs | QtCore.QDir.NoDotAndDotDot)
        self.dirLineEdit = QLineEdit()
        self.dirLineEdit.setText(self.config["logs_directory"])
        self.dirLineEdit.setFixedHeight(25)
        self.dirLineEdit.returnPressed.connect(self.handleTreeDirChange)
        self.fileTree = QTreeView()
        self.fileTree.setModel(self.dirModel)
        self.fileTree.setRootIndex(self.dirModel.index(self.config['logs_directory']))
        self.fileTree.setColumnHidden(1, True)
        self.fileTree.setColumnHidden(2, True)
        self.fileTree.setColumnHidden(3, True)
        self.fileTree.setMinimumWidth(300)
        self.fileTree.clicked.connect(self.handleTreeViewClick)
        self.fileTree.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self.fileTree.setSelectionMode(QAbstractItemView.SingleSelection) 
        self.fileTree.customContextMenuRequested.connect(self.handleTreeViewRightClick)
        # self.populateRMSCheck(self.config['logs_directory'])

    def updateFileTree(self):
        self.dirModel.setRootPath(self.config["logs_directory"])
        self.fileTree.setRootIndex(self.dirModel.index(self.config['logs_directory']))

    def populateRMSCheck(self, directory):
        for subdir in os.listdir(directory):
            path = os.path.join(directory, subdir)
            if os.path.isdir(path):
                self.populateRMSCheck(path)
            elif 'RMS' in subdir:
                f = open(path)
                rms_report = f.read()
                p = re.compile(r'(?<=^PASS/FAIL).*\n', re.M)
                line = re.search(p, rms_report).group()
                failed = True if "FAIL" in line else False
                if failed:
                    pass
                else:
                    pass

    def handleTreeDirChange(self):
        self.config["logs_directory"] = self.dirLineEdit.text()
        self.updateFileTree()

        file = open(self.configFilePath, 'w')
        yaml.dump(self.config, file)
        file.close()

    def handleTreeViewClick(self):
        selected_directory = self.fileTree.model().filePath(self.fileTree.selectedIndexes()[0])
        for fname in os.listdir(selected_directory):
            if fname.endswith('.dat'):
                try:
                    self.load(selected_directory)
                except Exception as e:
                    self.showError(e)
                break

    def handleTreeViewRightClick(self, event):
        selected_directory = os.path.normpath(self.fileTree.model().filePath(self.fileTree.selectedIndexes()[0]))
        menu = QMenu(self)
        copyAction = menu.addAction("Copy path")
        nppActionHot = menu.addAction("Run NPP - Start Hot")
        nppActionCold = menu.addAction("Run NPP - Start Cold")
        nppActionFactory = menu.addAction("Run NPP - Start Factory")
        setDataInfoDirAction = menu.addAction("Set as dataInfo.json directory")
        openAction = menu.addAction("Open folder")
        cleanFolderAction = menu.addAction("Clean folder")
        deleteFolderAction = menu.addAction("Delete folder")
        action = menu.exec_(self.fileTree.viewport().mapToGlobal(event))
        if action == copyAction:
            cb = QApplication.clipboard()
            cb.clear(mode=cb.Clipboard )
            cb.setText(selected_directory, mode=cb.Clipboard)
        if action == nppActionHot:
            cleanFolder(selected_directory)
            setDataInformationDirectory(selected_directory, startMode=START_MODE_HOT)
            from supernpp.supernpp import SuperNPP
            spp = SuperNPP(selected_directory, self.config['serials'])
            spp.run()
        if action == nppActionCold:
            cleanFolder(selected_directory)
            setDataInformationDirectory(selected_directory, startMode=START_MODE_COLD)
            from supernpp.supernpp import SuperNPP
            spp = SuperNPP(selected_directory, self.config['serials'], startMode=START_MODE_COLD)
            spp.run()
        if action == nppActionFactory:
            cleanFolder(selected_directory)
            setDataInformationDirectory(selected_directory, startMode=START_MODE_FACTORY)
            from supernpp.supernpp import SuperNPP
            spp = SuperNPP(selected_directory, self.config['serials'], startMode=START_MODE_FACTORY)
            spp.run()
        if action == setDataInfoDirAction:
            setDataInformationDirectory(selected_directory)
        if action == openAction:
            openFolderWithFileBrowser(selected_directory)
        if action == cleanFolderAction:
            cleanFolder(selected_directory)
        if action == deleteFolderAction:
            msg = QMessageBox(self)
            msg.setIcon(QMessageBox.Question)
            msg.setText("Are you sure you want to delete this folder?\n\n" + selected_directory)
            msg.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
            result = msg.exec()
            if result == QMessageBox.Yes:
                removeDirectory(selected_directory)

    def stopLoadingIndicator(self):
        self.loadingMovie.stop()
        self.loadingIndictator.clear()
        self.loadingIndictator.setPicture(self.emptyLoadingPicture)

    def showDeviceInfo(self):
        dlg = DeviceInfoDialog(self.log, self)
        dlg.show()
        dlg.exec_()

    def showFlashConfig(self):
        dlg = FlashConfigDialog(self.log, self)
        dlg.show()
        dlg.exec_()


    def plot(self, func, args=None):
        print("plotting " + func)
        self.currently_selected = func
        self.plotargs = args

        self.figure.clear()

        if hasattr(self, 'plotter'):
            if args is not None:
                getattr(self.plotter, func)(*args, self.figure)
            else:
                getattr(self.plotter, func)(self.figure)

        self.canvas.draw()
        self.stopLoadingIndicator()
        print("done plotting")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    MainWindow = QMainWindow()

    configFilePath = os.path.join(os.path.expanduser("~"), "Documents", "Inertial_Sense", "config.yaml")

    main = LogInspectorWindow(configFilePath, MainWindow)
    main.setupUi()
    # main.load(directory)
    main.show()

    if len(sys.argv) > 1:
        directory = sys.argv[1]
        main.load(directory)

    app.exec_()
