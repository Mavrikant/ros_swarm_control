# ros_swarm_control
The ROS package to control the swarm of robots on the base px4 with data transmission through [Rosbridge](http://wiki.ros.org/rosbridge_suite).


[![vedeo](https://img.youtube.com/vi/lle68ZXVatA/0.jpg)](https://www.youtube.com/watch?v=lle68ZXVatA)

# Packages
## swarm_server

Enables monitoring of the status of drones. The operation modes and setting of initial parameters.

`
rosrun swarm_server swarm_server.py
`

![image](https://i.ibb.co/fNkZr8Z/Screenshot-from-2019-07-02-12-05-22.png)


## swarm_control

``
roslaunch swarm_contol swarm_contol.launch 
``

Allows you to control a group of a given formation through RVIZ.
Available supervisory control of group with avoiding system based on potential field.

[![vedeo](https://img.youtube.com/vi/i1Cbbq-XMGQ/0.jpg)](https://www.youtube.com/watch?v=i1Cbbq-XMGQ)

## params
1. In **swarm_contol.launch** set the maximum speed of movement of the group.

2. In **../param/config.yaml** edit formation params of group.

# Dependences

1. Install from apt <br> 
`
sudo apt install python-websocket ros-$ROS_DISTRO-rospy-message-converter
`
2. install [WebsocketROSClientPython
](https://github.com/GigaFlopsis/rospy_websocker_client)


# Branch:
* rc_car (for [rc_car_ros](https://github.com/GigaFlopsis/rc_car_ros))
* px4_drone (for a private project)

### Stage: In the development
