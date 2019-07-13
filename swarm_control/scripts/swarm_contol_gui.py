#!/usr/bin/env python
# coding: utf-8

import sys
import rospy

from PyQt5 import QtWidgets
from PyQt5.QtCore import QObject, pyqtSignal, QThread, pyqtSlot, QTimer
import window

from swarm_msgs.msg import FormationParam, FieldParam, CommonParams
from swarm_msgs.srv import *
import signal
signal.signal(signal.SIGINT, signal.SIG_DFL)

common_param = CommonParams()

param_sub = None
class QSignals(QObject):
    callback = pyqtSignal()

signals = QSignals()

class WindowApp(QtWidgets.QMainWindow, window.Ui_Form):
    def __init__(self):
        # Это здесь нужно для доступа к переменным, методам
        # и т.д. в файле design.py
        super(WindowApp, self).__init__()

        self.setupUi(self)  # Это нужно для инициализации нашего дизайна
        signals.callback.connect(self.update_params)
        # init signals
        self.pushButton_load.clicked.connect(self.open_dialog)

        self.path = ""
        self.mode_list = ("NO_DATA",
        "RANK",
        "SQUARE",
        "KLIN",
        "REVERSE_KLIN",
        "COLUMN",
        "ECHELON",
        "CIRCLE",
        "TRIANGLE",
        "GRID")

        self.comboBox_modes.addItems(self.mode_list)


    # def _load_params(self):
    #     path = self.open_dialog()
    #     if path == "":
    #         return
    #
    #     #read data from files
    #     file = open(path, "r")
    #     json_data = file.read()
    #     file.close()
    #     print(json_data)
    #
    #     # set origin value
    #     lat = json.loads(json_data)['origin']['lat']
    #     lon = json.loads(json_data)['origin']['lon']
    #     alt = json.loads(json_data)['origin']['alt']
    #
    #     self.LatSpinBox.setValue(lat)
    #     self.LonSpinBox.setValue(lon)
    #     self.AltSpinBox.setValue(alt)
    #
    #     #clear list of drone
    #     self.delListOfDrone()
    #     size = json.loads(json_data)['size']
    #
    #     for i in range(size):
    #         name = json.loads(json_data)[tag+str(i)]['name']
    #         ip = json.loads(json_data)[tag+str(i)]['ip']
    #         port = json.loads(json_data)[tag + str(i)]['port']
    #
    #         droneClient = DroneConnect(ip, port, name)
    #         robotsList.append(droneClient)
    #         item_widget = DroneUI.Ui("%s%s" %(tag,i), droneClient)
    #         self.item = QtWidgets.QListWidgetItem(self.listOfDrones)
    #         self.item.setSizeHint(item_widget.sizeHint())
    #         self.listOfDrones.addItem(self.item)
    #         self.listOfDrones.setItemWidget(self.item, item_widget)
    #
    # def _save_params(self):
    #     global robotsList, origin_pose
    #     path = self.save_dialog()
    #     if path == "":
    #         return
    #     origin = {}
    #     origin['lat'] = origin_pose.latitude
    #     origin['lon'] = origin_pose.longitude
    #     origin['alt'] = origin_pose.altitude
    #
    #     doc = {}
    #     doc["origin"] = origin
    #     doc["size"] = len(robotsList)
    #
    #     for i in range(len(robotsList)):
    #         drone = {}
    #         drone['name'] = robotsList[i].name
    #         drone['ip'] = robotsList[i].ws._ip
    #         drone['port'] = robotsList[i].ws._port
    #         doc[str(tag)+str(i)] = drone
    #
    #     json_data = json.dumps(doc)
    #     file = open(path, "w")
    #     file.write(json_data)
    #     file.close()
    #     print("Params save to: %s" %path)

    def save_dialog(self):
        options = QtWidgets.QFileDialog.Options()
        options |= QtWidgets.QFileDialog.DontUseNativeDialog
        fileName, param = QtWidgets.QFileDialog.getSaveFileName(self, "Save of drone list", "",
                                                                "Drone params (*.params);;All Files (*)", options=options)
        filter = ''
        if param.find('Drone params (*.params)') != -1 :
            filter =".swarm"

        self.path = fileName+filter
        print "path", self.path

    def open_dialog(self):
        options = QtWidgets.QFileDialog.Options()
        options |= QtWidgets.QFileDialog.DontUseNativeDialog
        fileName = ""
        fileName, _ = QtWidgets.QFileDialog.getOpenFileName(self, "QFileDialog.getOpenFileName()", "",
                                                            "Drone params (*.swarm);;All Files (*)", options=options)
        self.path = fileName
        print "path", self.path

    def initSignals(self):
        self.spinBox_count.valueChanged.connect(self._changeCout)
        self.doubleSpinBox_dist.valueChanged.connect(self._changeDist)
        self.doubleSpinBox_speed.valueChanged.connect(self._changeSpeed)
        self.doubleSpinBox_radius.valueChanged.connect(self._changeRadius)
        self.doubleSpinBox_force.valueChanged.connect(self._changeForce)
        self.checkBox_use_field.stateChanged.connect(self._setField)
        self.checkBox_allow_z.stateChanged.connect(self._setAllowZ)
        self.comboBox_modes.activated[str].connect(self._setTypeForm)

    def _changeCout(self):
        if self.spinBox_count.value() <= 0:
            common_param.formation.count = 1
            self.spinBox_count.setValue(1)
        else:
            common_param.formation.count = self.spinBox_count.value()

        rospy.wait_for_service('swarm_contol/set_fotmation')
        srv_call = rospy.ServiceProxy("swarm_contol/set_fotmation", FormationSrv)
        req = srv_call(common_param.formation)

        print "req", req, "change cout", self.spinBox_count.value()

        # self.spinBox_cout.value()

    def _changeDist(self):

        common_param.formation.distance = self.doubleSpinBox_dist.value()

        rospy.wait_for_service('swarm_contol/set_fotmation')
        srv_call = rospy.ServiceProxy("swarm_contol/set_fotmation", FormationSrv)
        req = srv_call(common_param.formation)

        print "req", req, "change dist", self.doubleSpinBox_dist.value()

    def _changeSpeed(self):

        common_param.max_vel = self.doubleSpinBox_speed.value()

        rospy.wait_for_service('swarm_contol/set_max_velocity')
        srv_call = rospy.ServiceProxy("swarm_contol/set_max_velocity", FloatSrv)
        req = srv_call(common_param.max_vel)
        print "change speed", req

    def _changeRadius(self):

        common_param.field.r_safe = self.doubleSpinBox_radius.value()
        try:

            rospy.wait_for_service('swarm_contol/set_field')
            srv_call = rospy.ServiceProxy("swarm_contol/set_field", FieldSrv)
            req = srv_call(common_param.field)
            print "change Radius", req
        except:
            print "error"

    def _changeForce(self):

        common_param.field.force_rep = self.doubleSpinBox_force.value()
        try:

            rospy.wait_for_service('swarm_contol/set_field')
            srv_call = rospy.ServiceProxy("swarm_contol/set_field", FieldSrv)
            req = srv_call(common_param.field)
            print "change force", req
        except:
            print "error"

    def _setField(self):
        common_param.field.use_field = self.checkBox_use_field.isChecked()
        try:

            rospy.wait_for_service('swarm_contol/set_field')
            srv_call = rospy.ServiceProxy("swarm_contol/set_field", FieldSrv)
            req = srv_call(common_param.field)
            print "set field", req
        except:
            print "error"

    def _setAllowZ(self):
        common_param.field.allow_z_field = self.checkBox_allow_z.isChecked()
        try:

            rospy.wait_for_service('swarm_contol/set_field')
            srv_call = rospy.ServiceProxy("swarm_contol/set_field", FieldSrv)
            req = srv_call(common_param.field)
            print "set allow z", req
        except:
            print "error"

    def _setTypeForm(self):
        common_param.formation.type = self.comboBox_modes.currentIndex()
        try:
            rospy.wait_for_service('swarm_contol/set_fotmation')
            srv_call = rospy.ServiceProxy("swarm_contol/set_fotmation", FormationSrv)
            req = srv_call(common_param.formation)
            print "set formation", req
        except:
            print "error"

    ### slot
    def update_params(self):
        global param_sub

        self.spinBox_count.setValue(common_param.formation.count)
        self.doubleSpinBox_dist.setValue(common_param.formation.distance)
        self.doubleSpinBox_radius.setValue(common_param.field.r_safe)
        self.doubleSpinBox_force.setValue(common_param.field.force_rep)

        # self.checkBox_use_field.setChecked(common_param.field.use_field)
        self.checkBox_use_field.setChecked(True)
        self.checkBox_allow_z.setChecked(common_param.field.allow_z_field)
        self.doubleSpinBox_speed.setValue(common_param.max_vel)
        param_sub.unregister()
        self.comboBox_modes.setCurrentIndex(common_param.formation.type)

        self.initSignals()



### ROS callback

def common_clb(data):
    """
    get coomon params from swarm control
    :param data:
    :return:
    """
    global common_param
    common_param = data
    print "get data"
    signals.callback.emit()

class ROS_run(QObject):
    # def __init__(self):
    #     super(ROS_run, self).__init_()
    #     print("thead ros start")

    @pyqtSlot()
    def run(self):
        global param_sub
        param_sub = rospy.Subscriber("swarm_contol/state", CommonParams, common_clb)

        rospy.spin()


def sigint_handler(*args):
    """Handler for the SIGINT signal."""
    global abortFlag
    sys.stderr.write('\r')
    abortFlag = True
    app.quit()

abortFlag = False

if __name__ == '__main__':
    rospy.init_node('swarm_contol_gui_node', anonymous=True)
    signal.signal(signal.SIGINT, sigint_handler)

    # run ros thread
    ros_t = QThread()
    ros_ = ROS_run()
    ros_.moveToThread(ros_t)
    ros_t.started.connect(ros_.run)
    ros_t.start()

    # run main
    app = QtWidgets.QApplication(sys.argv)  # Новый экземпляр QApplication

    timer = QTimer()
    timer.start(500)  # You may change this if you wish.
    timer.timeout.connect(lambda: None)  # Let the interpreter run each 500 ms.

    window = WindowApp()  # Создаём объект класса ExampleApp
    window.show()  # Показываем окно
    sys.exit(app.exec_())
    abortFlag = True