#!/usr/bin/env python3

from logInspector import LogInspectorWindow

import sys, os
from PyQt5 import QtCore
from PyQt5.QtWidgets import QWidget, QDialog, QApplication, QPushButton, QVBoxLayout, QTreeView, QFileSystemModel,\
    QHBoxLayout, QGridLayout, QMainWindow, QSizePolicy, QSpacerItem, QFileDialog, QMessageBox, QLabel, QRadioButton,\
    QAbstractItemView, QMenu, QTableWidget,QTableWidgetItem, QSpinBox, QCheckBox
from PyQt5.QtGui import QMovie, QPicture, QIcon, QDropEvent
from PyQt5.Qt import QApplication, QClipboard
import traceback
import yaml


class ChooseDevsDialog(QDialog):
    def __init__(self, plotter, parent=None):
        super(ChooseDevsDialog, self).__init__(parent)
        self.setWindowTitle("Choose Devices")
        self.parent = parent
        self.mainLayout = QVBoxLayout()

        self.selectAllButton = QPushButton()
        self.selectAllButton.setText("Select All")
        self.selectAllButton.clicked.connect(self.selectAll)
        self.mainLayout.addWidget(self.selectAllButton)

        self.selectNoneButton = QPushButton()
        self.selectNoneButton.setText("Select None")
        self.selectNoneButton.clicked.connect(self.selectNone)
        self.mainLayout.addWidget(self.selectNoneButton)

        self.checkboxes = []
        for i in range(parent.log.numDev):
            checkbox = QCheckBox()
            checkbox.setText(str(parent.log.serials[i]))
            checkbox.setChecked(i in parent.plotter.active_devs)
            checkbox.clicked.connect(self.updatePlot)
            self.checkboxes.append(checkbox)
            self.mainLayout.addWidget(checkbox)

        self.okbutton = QPushButton()
        self.okbutton.setText("OK")
        self.okbutton.clicked.connect(self.clickedOk)
        self.mainLayout.addWidget(self.okbutton)


        self.setLayout(self.mainLayout)

    def updatePlot(self):
        active_serials = []
        for i, checkbox in enumerate(self.checkboxes):
            if checkbox.isChecked():
                active_serials.append(self.parent.log.serials[i])
        self.parent.plotter.setActiveSerials(active_serials)
        self.parent.updatePlot()

    def clickedOk(self):
        self.close()

    def selectAll(self):
        for checkbox in self.checkboxes:
            checkbox.setChecked(True)
        self.updatePlot()

    def selectNone(self):
        for checkbox in self.checkboxes:
            checkbox.setChecked(False)
        self.updatePlot()


class logInspectorInternal(LogInspectorWindow):
    def __init__(self, config, parent=None):
        super(logInspectorInternal, self).__init__(config, parent)
        self.page = 0


    def createButtonColumn(self):
        super(logInspectorInternal, self).createButtonColumn()
        self.addButton('Debug Int', lambda: self.plot('debugiArr'))
        self.addButton('Debug Float', lambda: self.plot('debugfArr'))
        self.addButton('Debug Double', lambda: self.plot('debuglfArr'))
        self.addButton('Delta Time', lambda: self.plot('deltatime'))
        self.addButton('Mag Decl.', lambda: self.plot('magDec'))
        self.addButton('EKF Biases', lambda: self.plot('ekfBiases'))
        self.addButton('Phase Residuals', lambda: self.plot('rtkResiduals', ('phase', self.page)))
        self.addButton('Code Residuals', lambda: self.plot('rtkResiduals', ('code', self.page)))
        self.addButton('RTK Debug', lambda: self.plot('rtkDebug'))
        self.addButton('RTK Dbg 2', lambda: self.plot('rtkDebug2'))
        self.addButton('RTK Dbg 2 Sat', lambda: self.plot('rtkDebug2Sat'))
        self.addButton('RTK Dbg 2 STD', lambda: self.plot('rtkDebug2Std'))
        self.addButton('RTK Dbg 2 Lock', lambda: self.plot('rtkDebug2Lock'))
        self.addButton('RTK Pos Misc', lambda: self.plot('rtkPosMisc'))
        self.addButton('RTK Cmp Misc', lambda: self.plot('rtkCmpMisc'))
        self.addButton('Wheel Encoder', lambda: self.plot('wheelEncoder'))
        self.addButton('GPS Raw Time', lambda: self.plot('gpsRawTime'))
        #self.addButton('RTK Rel', lambda: self.plot('rtkRel'))

    def createBottomToolbar(self):
        super(logInspectorInternal, self).createBottomToolbar()
        # pageLabel = QLabel()
        # pageLabel.setText("Page")
        # self.pageInput = QSpinBox()
        # self.pageInput.setValue(self.page)
        # self.toolLayout.addWidget(pageLabel)
        # self.toolLayout.addWidget(self.pageInput)
        # self.pageInput.valueChanged.connect(self.changePage)
        # self.toolLayout.addWidget(self.pageInput)

    def changePage(self, val):
        self.page = val
        if self.plotargs is not None:
            self.plotargs = (self.plotargs[0], self.page)
        self.updatePlot()

    def chooseDevs(self):
        try:
            dlg = ChooseDevsDialog(self.plotter, self)
            dlg.show()
            dlg.exec_()
        except Exception as e:
            self.showError(e)

    def RMS(self):
        if self.log is not None:
            self.log.calculateRMS()
            self.log.printRMSReport()
            self.log.openRMSReport()
        # self.stopLoadingIndicator()

    def formatButtonColumn(self):
        super(logInspectorInternal, self).formatButtonColumn()
        self.devicesLayout = QHBoxLayout()
        self.addButton('RMS', self.RMS, layout=self.devicesLayout)
        self.addButton('Choose Devices', self.chooseDevs, layout=self.devicesLayout)
        self.controlLayout.addLayout(self.devicesLayout)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    MainWindow = QMainWindow()

    configFilePath = os.path.join(os.path.expanduser("~"), "Documents", "Inertial_Sense", "config.yaml")

    main = logInspectorInternal(configFilePath, MainWindow)
    main.setupUi()
    # main.load(config['directory'])
    main.show()

    if len(sys.argv) > 1:
        directory = sys.argv[1]
        main.load(directory)

    app.exec_()
