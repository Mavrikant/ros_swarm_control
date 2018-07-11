#!/usr/bin/env python
# coding: utf-8

"""
Class for setting up interaction with the rc car
"""

import rospy
from rospy_websocker_client import WebsocketROSClient as ros_ws
from PyQt5.QtCore import QObject, QThread, pyqtSignal, pyqtSlot

from geometry_msgs.msg import PoseStamped, Twist
from std_srvs.srv import SetBool, SetBoolRequest
from sensor_msgs.msg import BatteryState, LaserScan
from sensor_msgs.msg import NavSatFix

from rc_car_msgs.msg import Diagnostics

class CarConnect(QObject):

    diag_signals = pyqtSignal()
    def __init__(self, websocket_ip, port=9090, name =''):
        super(CarConnect, self).__init__()

        # init values
        self.diagnostics = Diagnostics()

        # Subscribe
        self.name = name
        self.ip = websocket_ip
        self.port = port
        self.ws = ros_ws.ws_client(self.ip, self.port)
        ## Sub to server through Rosbridge
        self.ws.subscribe('/mavros/local_position/pose', PoseStamped(), name+"/local/pose")
        self.ws.subscribe('/geo/local_pose', PoseStamped(), name+"/geo/local_pose")
        self.ws.subscribe('/diagnostics', Diagnostics(), name+"/diagnostics")
        self.ws.subscribe('/mavros/global_position/global', NavSatFix(), name+"/global/pose")
        self.ws.subscribe('/scan', LaserScan(), name+"/scan")

        ## Subscribe to Ros topics and publish through Rosbridge server
        self.sub_diagnoctics = rospy.Subscriber(name+"/diagnostics", Diagnostics, self.clb_diag)
        self.sub_goal = rospy.Subscriber(name+"/geo/goal_pose", PoseStamped, self.goal_clb_global)
        self.sub_goal = rospy.Subscriber(name+"/local/goal_pose", PoseStamped, self.goal_clb_local)
        self.sub_goal = rospy.Subscriber(name + "/cmd_vel", Twist, self.cmd_vel_clb)

    """
    Publis function fot GUI
    """
    def connect(self):
        self.ws.connect()

    def disconnect(self):
        self.sub_diagnoctics.unregister()
        self.sub_goal.unregister()
        self.ws.disconnect()

    def is_active(self):
        return self.ws.is_connected()

    def set_origin(self, topic, data):
        print("set origin")
        self.ws.publish(topic, data)

    def arm(self):
        motor_on = SetBoolRequest()
        motor_on.data = True
        self.ws.call_service("/car/set_mode", motor_on)

    def disarm(self):
        motor_on = SetBoolRequest()
        motor_on.data = True
        self.ws.call_service("/car/set_mode", motor_on)

    """
    Collback to topics
    """

    def goal_clb_local(self, data):
        """
        Get goal pose
        :type data: PoseStamped
        :return:
        """
        self.ws.publish("/goal_pose", data)

    def goal_clb_global(self, data):
        """
        Get goal pose
        :type data: PoseStamped
        :return:
        """
        self.ws.publish("/geo/goal_pose", data)

    def cmd_vel_clb(self,data):
        """
        Set cmd vel to car
        :type data: Twist
        :return:
        """
        self.ws.publish("/cmd_vel", data)


    def clb_diag(self, data):
        """
        Get battery data of drone
        :type data: BatteryState
        :return:
        """
        self.diagnostics = data
        self.diag_signals.emit()

    """
    Delete class
    """
    def __del__(self):
        self.ws.__del__()


