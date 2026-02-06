#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'test_io_cutting_machine.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import pyqtSignal
from datetime import time
from enum import Enum
import time as ori_time
from datetime import *
import logging
from PyQt5.QtCore import QTime
import roslib
import rospy
import sys
from std_msgs.msg import (
    String,
    Int32,
    Int16MultiArray,
    MultiArrayLayout,
    MultiArrayDimension,
)
from std_stamped_msgs.msg import (
    StringAction,
    StringStamped,
    StringResult,
    StringFeedback,
    StringGoal,
    Int8Stamped,
    EmptyStamped,
    Float32Stamped,
    Int16MultiArrayStamped,
)

output_status = [0] * 8


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        self.output_status = [0] * 8
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(959, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.groupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox.setGeometry(QtCore.QRect(30, 140, 911, 161))
        self.groupBox.setObjectName("groupBox")
        self.horizontalLayoutWidget = QtWidgets.QWidget(self.groupBox)
        self.horizontalLayoutWidget.setGeometry(QtCore.QRect(50, 50, 684, 80))
        self.horizontalLayoutWidget.setObjectName("horizontalLayoutWidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.IN_1 = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        self.IN_1.setObjectName("IN_1")
        self.horizontalLayout.addWidget(self.IN_1)
        self.IN_2 = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        self.IN_2.setObjectName("IN_2")
        self.horizontalLayout.addWidget(self.IN_2)
        self.IN_3 = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        self.IN_3.setObjectName("IN_3")
        self.horizontalLayout.addWidget(self.IN_3)
        self.IN_4 = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        self.IN_4.setObjectName("IN_4")
        self.horizontalLayout.addWidget(self.IN_4)
        self.IN_5 = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        self.IN_5.setObjectName("IN_5")
        self.horizontalLayout.addWidget(self.IN_5)
        self.IN_6 = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        self.IN_6.setObjectName("IN_6")
        self.horizontalLayout.addWidget(self.IN_6)
        self.IN_7 = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        self.IN_7.setObjectName("IN_7")
        self.horizontalLayout.addWidget(self.IN_7)
        self.IN_8 = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        self.IN_8.setObjectName("IN_8")
        self.horizontalLayout.addWidget(self.IN_8)
        self.groupBox_2 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_2.setGeometry(QtCore.QRect(30, 330, 911, 161))
        self.groupBox_2.setObjectName("groupBox_2")
        self.horizontalLayoutWidget_3 = QtWidgets.QWidget(self.groupBox_2)
        self.horizontalLayoutWidget_3.setGeometry(QtCore.QRect(50, 50, 684, 80))
        self.horizontalLayoutWidget_3.setObjectName("horizontalLayoutWidget_3")
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget_3)
        self.horizontalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.OUT_1 = QtWidgets.QPushButton(self.horizontalLayoutWidget_3)
        self.OUT_1.setObjectName("OUT_1")
        self.horizontalLayout_3.addWidget(self.OUT_1)
        self.OUT_2 = QtWidgets.QPushButton(self.horizontalLayoutWidget_3)
        self.OUT_2.setObjectName("OUT_2")
        self.horizontalLayout_3.addWidget(self.OUT_2)
        self.OUT_3 = QtWidgets.QPushButton(self.horizontalLayoutWidget_3)
        self.OUT_3.setObjectName("OUT_3")
        self.horizontalLayout_3.addWidget(self.OUT_3)
        self.OUT_4 = QtWidgets.QPushButton(self.horizontalLayoutWidget_3)
        self.OUT_4.setObjectName("OUT_4")
        self.horizontalLayout_3.addWidget(self.OUT_4)
        self.OUT_5 = QtWidgets.QPushButton(self.horizontalLayoutWidget_3)
        self.OUT_5.setObjectName("OUT_5")
        self.horizontalLayout_3.addWidget(self.OUT_5)
        self.OUT_6 = QtWidgets.QPushButton(self.horizontalLayoutWidget_3)
        self.OUT_6.setObjectName("OUT_6")
        self.horizontalLayout_3.addWidget(self.OUT_6)
        self.OUT_7 = QtWidgets.QPushButton(self.horizontalLayoutWidget_3)
        self.OUT_7.setObjectName("OUT_7")
        self.horizontalLayout_3.addWidget(self.OUT_7)
        self.OUT_8 = QtWidgets.QPushButton(self.horizontalLayoutWidget_3)
        self.OUT_8.setObjectName("OUT_8")
        self.horizontalLayout_3.addWidget(self.OUT_8)
        self.start_button = QtWidgets.QPushButton(self.centralwidget)
        self.start_button.setGeometry(QtCore.QRect(390, 10, 81, 81))
        self.start_button.setObjectName("start_button")
        self.stop_button = QtWidgets.QPushButton(self.centralwidget)
        self.stop_button.setGeometry(QtCore.QRect(500, 10, 81, 81))
        self.stop_button.setObjectName("stop_button")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 959, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

        # init function
        self.init_function(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.groupBox.setTitle(_translate("MainWindow", "Input"))
        self.IN_1.setText(_translate("MainWindow", "1"))
        self.IN_2.setText(_translate("MainWindow", "2"))
        self.IN_3.setText(_translate("MainWindow", "3"))
        self.IN_4.setText(_translate("MainWindow", "4"))
        self.IN_5.setText(_translate("MainWindow", "5"))
        self.IN_6.setText(_translate("MainWindow", "6"))
        self.IN_7.setText(_translate("MainWindow", "7"))
        self.IN_8.setText(_translate("MainWindow", "8"))
        self.groupBox_2.setTitle(_translate("MainWindow", "Output"))
        self.OUT_1.setText(_translate("MainWindow", "1"))
        self.OUT_2.setText(_translate("MainWindow", "2"))
        self.OUT_3.setText(_translate("MainWindow", "3"))
        self.OUT_4.setText(_translate("MainWindow", "4"))
        self.OUT_5.setText(_translate("MainWindow", "5"))
        self.OUT_6.setText(_translate("MainWindow", "6"))
        self.OUT_7.setText(_translate("MainWindow", "7"))
        self.OUT_8.setText(_translate("MainWindow", "8"))
        self.start_button.setText(_translate("MainWindow", "Start "))
        self.stop_button.setText(_translate("MainWindow", "Stop"))

    def init_function(self, MainWindow):
        self.thread = {}
        self.start_button.clicked.connect(self.start_auto)
        self.stop_button.clicked.connect(self.stop_auto)
        self.OUT_1.clicked.connect(self.set_out_1)
        self.OUT_2.clicked.connect(self.set_out_2)
        self.OUT_3.clicked.connect(self.set_out_3)
        self.OUT_4.clicked.connect(self.set_out_4)
        self.OUT_5.clicked.connect(self.set_out_5)
        self.OUT_6.clicked.connect(self.set_out_6)
        self.OUT_7.clicked.connect(self.set_out_7)
        self.OUT_8.clicked.connect(self.set_out_8)
        self.output_status = [0] * 8

    def start_auto(self):
        stop = False
        self.thread[1] = ThreadAuto(index=1)
        self.thread[1].start()
        self.thread[1].signal_input.connect(self.set_singal_input)
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)

    def stop_auto(self):
        self.thread[1].stop()
        self.stop_button.setEnabled(False)
        self.start_button.setEnabled(True)

    def set_out_1(self):
        global output_status
        self.output_status[1 - 1] = not self.output_status[1 - 1]
        output_status = self.output_status
        if self.output_status[1 - 1]:
            self.OUT_1.setStyleSheet(
                "background-color: \
                            rgba(255,255,0,255); "
            )
        else:
            self.OUT_1.setStyleSheet(
                "background-color: \
                            rgba(255,255,0,0); "
            )

    def set_out_2(self):
        global output_status
        self.output_status[2 - 1] = not self.output_status[2 - 1]
        output_status = self.output_status
        if self.output_status[2 - 1]:
            self.OUT_2.setStyleSheet(
                "background-color: \
                            rgba(255,255,0,255); "
            )
        else:
            self.OUT_2.setStyleSheet(
                "background-color: \
                            rgba(255,255,0,0); "
            )

    def set_out_3(self):
        global output_status
        self.output_status[3 - 1] = not self.output_status[3 - 1]
        output_status = self.output_status
        if self.output_status[3 - 1]:
            self.OUT_3.setStyleSheet(
                "background-color: \
                            rgba(255,255,0,255); "
            )
        else:
            self.OUT_3.setStyleSheet(
                "background-color: \
                            rgba(255,255,0,0); "
            )

    def set_out_4(self):
        global output_status
        self.output_status[4 - 1] = not self.output_status[4 - 1]
        output_status = self.output_status
        if self.output_status[4 - 1]:
            self.OUT_4.setStyleSheet(
                "background-color: \
                            rgba(255,255,0,255); "
            )
        else:
            self.OUT_4.setStyleSheet(
                "background-color: \
                            rgba(255,255,0,0); "
            )

    def set_out_5(self):
        global output_status
        self.output_status[5 - 1] = not self.output_status[5 - 1]
        output_status = self.output_status
        if self.output_status[5 - 1]:
            self.OUT_5.setStyleSheet(
                "background-color: \
                            rgba(255,255,0,255); "
            )
        else:
            self.OUT_5.setStyleSheet(
                "background-color: \
                            rgba(255,255,0,0); "
            )

    def set_out_6(self):
        global output_status
        self.output_status[6 - 1] = not self.output_status[6 - 1]
        output_status = self.output_status
        if self.output_status[6 - 1]:
            self.OUT_6.setStyleSheet(
                "background-color: \
                            rgba(255,255,0,255); "
            )
        else:
            self.OUT_6.setStyleSheet(
                "background-color: \
                            rgba(255,255,0,0); "
            )

    def set_out_7(self):
        global output_status
        self.output_status[7 - 1] = not self.output_status[7 - 1]
        output_status = self.output_status
        if self.output_status[7 - 1]:
            self.OUT_7.setStyleSheet(
                "background-color: \
                            rgba(255,255,0,255); "
            )
        else:
            self.OUT_7.setStyleSheet(
                "background-color: \
                            rgba(255,255,0,0); "
            )

    def set_out_8(self):
        global output_status
        self.output_status[8 - 1] = not self.output_status[8 - 1]
        output_status = self.output_status
        if self.output_status[8 - 1]:
            self.OUT_8.setStyleSheet(
                "background-color: \
                            rgba(255,255,0,255); "
            )
        else:
            self.OUT_8.setStyleSheet(
                "background-color: \
                            rgba(255,255,0,0); "
            )

    def set_singal_input(self, counter):
        if counter[0]:
            self.IN_1.setStyleSheet(
                "background-color: \
                        rgba(255,255,0,255); "
            )
        else:
            self.IN_1.setStyleSheet(
                "background-color: \
                        rgba(255,255,0,0); "
            )
        if counter[1]:
            self.IN_2.setStyleSheet(
                "background-color: \
                        rgba(255,255,0,255); "
            )
        else:
            self.IN_2.setStyleSheet(
                "background-color: \
                        rgba(255,255,0,0); "
            )
        if counter[2]:
            self.IN_3.setStyleSheet(
                "background-color: \
                        rgba(255,255,0,255); "
            )
        else:
            self.IN_3.setStyleSheet(
                "background-color: \
                        rgba(255,255,0,0); "
            )
        if counter[3]:
            self.IN_4.setStyleSheet(
                "background-color: \
                        rgba(255,255,0,255); "
            )
        else:
            self.IN_4.setStyleSheet(
                "background-color: \
                        rgba(255,255,0,0); "
            )
        if counter[4]:
            self.IN_5.setStyleSheet(
                "background-color: \
                        rgba(255,255,0,255); "
            )
        else:
            self.IN_5.setStyleSheet(
                "background-color: \
                        rgba(255,255,0,0); "
            )
        if counter[5]:
            self.IN_6.setStyleSheet(
                "background-color: \
                        rgba(255,255,0,255); "
            )
        else:
            self.IN_6.setStyleSheet(
                "background-color: \
                        rgba(255,255,0,0); "
            )
        if counter[6]:
            self.IN_7.setStyleSheet(
                "background-color: \
                        rgba(255,255,0,255); "
            )
        else:
            self.IN_7.setStyleSheet(
                "background-color: \
                        rgba(255,255,0,0); "
            )
        if counter[7]:
            self.IN_8.setStyleSheet(
                "background-color: \
                        rgba(255,255,0,255); "
            )
        else:
            self.IN_8.setStyleSheet(
                "background-color: \
                        rgba(255,255,0,0); "
            )


class ThreadAuto(QtCore.QThread):
    signal_input = pyqtSignal(object)

    def __init__(self, index=0):
        super().__init__()
        self.index = index
        self.optical_input = [0] * 8
        self.data_optical = Int16MultiArray()
        rospy.Subscriber("/fastech_input", Int16MultiArray, self.optical_sensor_cb)
        # rospy.Subscriber(
        #     "/fastech_output", Int16MultiArray, self.read_fastech_output_cb
        # )

        # Publish
        self.fastech_control_pub = rospy.Publisher(
            "/fastech_control_multiarray", Int16MultiArray, queue_size=10
        )

    def optical_sensor_cb(self, msg):
        self.optical_input = msg.data

    def reset(self):
        pass

    def run(self):
        while True:
            self.data_optical.data = output_status
            self.fastech_control_pub.publish(self.data_optical)

            self.signal_input.emit(self.optical_input)
            rospy.sleep(0.1)

    def stop(self):
        print("Stopping thread...", self.index)
        self.terminate()


if __name__ == "__main__":
    import sys

    rospy.init_node("test_cutting_machine")
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
