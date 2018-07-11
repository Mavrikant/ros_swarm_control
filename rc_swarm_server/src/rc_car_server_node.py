#!/usr/bin/env python
# coding: utf-8

import sys
import rospy

from PyQt5 import QtWidgets, QtGui
from PyQt5.QtWidgets import QApplication, QWidget, QCheckBox
from PyQt5.QtCore import QObject, pyqtSignal,  QRunnable, QThread, QThreadPool, pyqtSlot, Qt
import carUI, window
from RcCarClient import CarConnect
from sensor_msgs.msg import NavSatFix
import json

common_ip = "127.0.0.1"
common_port = 9090
robotsList = list()

tag = "car_"

origin_topic = '/geo/set_origin'
origin_pose = NavSatFix()
abortFlag = False

class WindowApp(QtWidgets.QMainWindow, window.Ui_Form):
    def __init__(self):
        # Это здесь нужно для доступа к переменным, методам
        # и т.д. в файле design.py
        super(WindowApp, self).__init__()

        self.setupUi(self)  # Это нужно для инициализации нашего дизайна

        # init signals
        self.addButton.clicked.connect(self.addItems_btn)
        self.dellButton.clicked.connect(self.delItems__btn)
        self.ArmAllButton.clicked.connect(self.armAll_btn)
        self.DisarmAllButton.clicked.connect(self.disarmAll_btn)
        self.ConnectAllButton.clicked.connect(self.connectAll_btn)
        self.DisconnectAllButton.clicked.connect(self.disconnectAll_btn)

        self.OriginPushButton.clicked.connect(self.setOrigin_btn)
        self.LatSpinBox.valueChanged.connect(self._changeOrigin)
        self.LonSpinBox.valueChanged.connect(self._changeOrigin)
        self.AltSpinBox.valueChanged.connect(self._changeOrigin)

        self.SaveParamsButton.clicked.connect(self._save_params)
        self.LoadParamsButton.clicked.connect(self._load_params)

    """
    Auxiliary function

    """

    def _load_params(self):
        path = self.open_dialog_bnt()
        if path == "":
            return

        #read data from files
        file = open(path, "r")
        json_data = file.read()
        file.close()
        print(json_data)

        # set origin value
        lat = json.loads(json_data)['origin']['lat']
        lon = json.loads(json_data)['origin']['lon']
        alt = json.loads(json_data)['origin']['alt']

        self.LatSpinBox.setValue(lat)
        self.LonSpinBox.setValue(lon)
        self.AltSpinBox.setValue(alt)

        #clear list of drone
        self.delListOfRobots_btn()
        size = json.loads(json_data)['size']

        for i in range(size):
            name = json.loads(json_data)[tag+str(i)]['name']
            ip = json.loads(json_data)[tag+str(i)]['ip']
            port = json.loads(json_data)[tag + str(i)]['port']

            droneClient = CarConnect(ip, port, name)
            robotsList.append(droneClient)
            item_widget = carUI.Ui("%s%s" % (tag, i), droneClient)
            self.item = QtWidgets.QListWidgetItem(self.listOfRobots)
            self.item.setSizeHint(item_widget.sizeHint())
            self.listOfRobots.addItem(self.item)
            self.listOfRobots.setItemWidget(self.item, item_widget)

    def _save_params(self):
        global robotsList, origin_pose
        path = self.save_dialog_btn()
        if path == "":
            return
        origin = {}
        origin['lat'] = origin_pose.latitude
        origin['lon'] = origin_pose.longitude
        origin['alt'] = origin_pose.altitude

        doc = {}
        doc["origin"] = origin
        doc["size"] = len(robotsList)

        for i in range(len(robotsList)):
            drone = {}
            drone['name'] = robotsList[i].name
            drone['ip'] = robotsList[i].ws._ip
            drone['port'] = robotsList[i].ws._port
            doc[str(tag)+str(i)] = drone

        json_data = json.dumps(doc)
        file = open(path, "w")
        file.write(json_data)
        file.close()
        print("Params save to: %s" %path)

    def _changeOrigin(self):
        """
        change origin data
        :return:
        """
        global origin_pose
        Lat = self.LatSpinBox.value()
        Lon = self.LonSpinBox.value()
        Alt = self.AltSpinBox.value()

        origin_pose.latitude = Lat
        origin_pose.longitude = Lon
        origin_pose.altitude = Alt

    """
    GUI elements
    """

    def save_dialog_btn(self):
        options = QtWidgets.QFileDialog.Options()
        options |= QtWidgets.QFileDialog.DontUseNativeDialog
        fileName, param = QtWidgets.QFileDialog.getSaveFileName(self, "Save of drone list", "",
                                                                "Drone params (*.params);;All Files (*)", options=options)
        filter = ''
        if param.find('Drone params (*.params)') != -1 :
            filter =".params"

        return fileName+filter

    def open_dialog_bnt(self):
        options = QtWidgets.QFileDialog.Options()
        options |= QtWidgets.QFileDialog.DontUseNativeDialog
        fileName = ""
        fileName, _ = QtWidgets.QFileDialog.getOpenFileName(self, "QFileDialog.getOpenFileName()", "",
                                                            "Drone params (*.params);;All Files (*)", options=options)
        return fileName

    def setOrigin_btn(self):
        """
        push button set origin data
        :return:
        """
        global origin_pose
        print("===========\n"
              "Set origin\n"
              "===========\n"
              "Lat: %s\n"
              "Lon: %s\n"
              "Alt: %s" % (origin_pose.latitude,
                           origin_pose.longitude,
                           origin_pose.altitude))

        origin_pose.header.stamp = rospy.Time.now()
        for robot in robotsList:
            if robot.is_active():
                robot.set_origin(origin_topic, origin_pose)

    def connectAll_btn(self):
        print("connect all")
        for robot in robotsList:
            if not robot.is_active():
                robot.connect()

    def disconnectAll_btn(self):
        print("disconnect all")

        for robot in robotsList:
            if robot.is_active():
                robot.disconnect()

    def armAll_btn(self):
        print("ArmAll")
        for robot in robotsList:
            robot.arm()

    def disarmAll_btn(self):
        print("disarm all")
        global robotsList

        for robot in robotsList:
            robot.disarm()

    def addItems_btn(self):
        """
        :type droneList: list(
        """
        global robotsList

        robotClient = CarConnect(common_ip, common_port, "%s%s" % (tag, len(robotsList)))
        robotsList.append(robotClient)

        item_widget = carUI.Ui("%s%s" % (tag, len(self.listOfRobots)), robotClient)
        self.item = QtWidgets.QListWidgetItem(self.listOfRobots)
        self.item.setSizeHint(item_widget.sizeHint())
        self.listOfRobots.addItem(self.item)
        self.listOfRobots.setItemWidget(self.item, item_widget)
        print("addItems:",len(self.listOfRobots))

    def delItems__btn(self):
        global robotsList

        if self.listOfRobots.currentRow() < 0:
            return
        robotsList[self.listOfRobots.currentRow()].disconnect()
        del robotsList[self.listOfRobots.currentRow()]
        self.listOfRobots.takeItem(self.listOfRobots.currentRow())
        print("del: %d : len: %d" % (self.listOfRobots.currentRow(), len(robotsList)))

    def delListOfRobots_btn(self):
        global robotsList

        if len(robotsList) == 0:
            print("List is empty:", len(robotsList))
            return

        for i in range(len(robotsList) - 1, -1, -1):
            robotsList[i].disconnect()
            del robotsList[i]
            self.listOfRobots.takeItem(i)
            print("del: %d : len: %d" % (i, len(robotsList)))
        print("List is clear:")

class ROS_run(QObject):
    @pyqtSlot()
    def run(self):
        rate = rospy.Rate(20)
        try:
            while not rospy.is_shutdown() and not abortFlag:

                rate.sleep()
        except:
            pass

if __name__ == '__main__':
    rospy.init_node('swarm_server_node', anonymous=True)

    # Run ROS in the thread
    ros_t = QThread()
    ros_ = ROS_run()
    ros_.moveToThread(ros_t)
    ros_t.started.connect(ros_.run)
    ros_t.start()

    # Run server
    app = QtWidgets.QApplication(sys.argv)
    window = WindowApp()    # Create windows of class ExampleApp
    window.show()           # Show window
    app.exec_()             # exit
    abortFlag = True
