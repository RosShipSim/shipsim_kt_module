# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ship_control_module/controller_ui.ui'
#
# Created by: PyQt5 UI code generator 5.15.2
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(451, 255)
        self.centralwidget = QtWidgets.QWidget(Dialog)
        self.centralwidget.setGeometry(QtCore.QRect(10, 10, 421, 221))
        self.centralwidget.setObjectName("centralwidget")
        self.rudderDial = QtWidgets.QDial(self.centralwidget)
        self.rudderDial.setGeometry(QtCore.QRect(280, 100, 131, 111))
        self.rudderDial.setMinimum(-45)
        self.rudderDial.setMaximum(45)
        self.rudderDial.setInvertedAppearance(False)
        self.rudderDial.setInvertedControls(False)
        self.rudderDial.setObjectName("rudderDial")
        self.rudderLabel = QtWidgets.QLabel(self.centralwidget)
        self.rudderLabel.setGeometry(QtCore.QRect(250, 80, 51, 16))
        self.rudderLabel.setObjectName("rudderLabel")
        self.rudderDialValueLabel = QtWidgets.QLabel(self.centralwidget)
        self.rudderDialValueLabel.setGeometry(QtCore.QRect(310, 80, 31, 16))
        self.rudderDialValueLabel.setObjectName("rudderDialValueLabel")
        self.degreeLabel = QtWidgets.QLabel(self.centralwidget)
        self.degreeLabel.setGeometry(QtCore.QRect(340, 80, 51, 16))
        self.degreeLabel.setObjectName("degreeLabel")
        self.ULabel = QtWidgets.QLabel(self.centralwidget)
        self.ULabel.setGeometry(QtCore.QRect(20, 20, 59, 15))
        self.ULabel.setObjectName("ULabel")
        self.UEdit = QtWidgets.QLineEdit(self.centralwidget)
        self.UEdit.setGeometry(QtCore.QRect(100, 20, 113, 23))
        self.UEdit.setObjectName("UEdit")
        self.startButton = QtWidgets.QPushButton(self.centralwidget)
        self.startButton.setGeometry(QtCore.QRect(250, 20, 61, 23))
        self.startButton.setObjectName("startButton")
        self.stopButton = QtWidgets.QPushButton(self.centralwidget)
        self.stopButton.setGeometry(QtCore.QRect(330, 20, 61, 23))
        self.stopButton.setObjectName("stopButton")
        self.samplingFreqLabel = QtWidgets.QLabel(self.centralwidget)
        self.samplingFreqLabel.setGeometry(QtCore.QRect(10, 60, 121, 16))
        self.samplingFreqLabel.setObjectName("samplingFreqLabel")
        self.samplingFreqEdit = QtWidgets.QLineEdit(self.centralwidget)
        self.samplingFreqEdit.setGeometry(QtCore.QRect(140, 60, 71, 23))
        self.samplingFreqEdit.setObjectName("samplingFreqEdit")

        self.retranslateUi(Dialog)
        self.startButton.clicked.connect(Dialog.clicked_start)
        self.stopButton.clicked.connect(Dialog.clicked_stop)
        self.rudderDial.sliderMoved["int"].connect(Dialog.change_rudder_angle)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "ShipController"))
        self.rudderLabel.setText(_translate("Dialog", "Rudder:"))
        self.rudderDialValueLabel.setText(_translate("Dialog", "0"))
        self.degreeLabel.setText(_translate("Dialog", "degree"))
        self.ULabel.setText(_translate("Dialog", "U [m/s]"))
        self.UEdit.setText(_translate("Dialog", "0.40"))
        self.startButton.setText(_translate("Dialog", "Start"))
        self.stopButton.setText(_translate("Dialog", "Stop"))
        self.samplingFreqLabel.setText(_translate("Dialog", "sampling freq [1/s]"))
        self.samplingFreqEdit.setText(_translate("Dialog", "1.0"))
