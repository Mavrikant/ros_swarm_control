#!/usr/bin/env python
# coding=utf8

import math
import numpy as np
import sys
import yaml
from numpy import format_parser

import rospy
from drone_msgs.msg import Goal
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from virt_formation import create_virtual_structure
from swarm_msgs.msg import FormationParam

# ---------------------------------------------------------------------------------------------------------------------
# ---------- Global params
tag = "drone_"
max_vel = 2


param_path = "../param/config.yaml"

drone_offset_list = list()  # [name :str, offset :Point, prevPoint: Goal]
markers_goal = MarkerArray()
markers_goal_text = MarkerArray()
markers_lerp = MarkerArray()
markers_lerp_text = MarkerArray()

state_init_flag = False

# Параметры планировщика
r_kor = 0.4  # коридор нулевых сил
r_porog = 1.2  # дальность действия поля отталкивания
c_rep = 0.4  # коэффициент ф-ии отталкивания
size_of_drone = 0

use_yaml = False
goal_common_msgs = Goal()

def setup_market(name, point, id, colorRGBA, text_flag=False):
    """
    Настройка маркера для отображения в rviz
    :type point: Point
    :param point:
    :return:
    """
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.header.stamp = rospy.get_rostime()
    marker.ns = name
    marker.id = id
    marker.action = 0
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1.0

    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.r = colorRGBA[0]
    marker.color.g = colorRGBA[1]
    marker.color.b = colorRGBA[2]
    marker.color.a = colorRGBA[3]
    marker.pose.position.x = point.x
    marker.pose.position.y = point.y
    if text_flag:
        marker.type = Marker.TEXT_VIEW_FACING
        marker.text = str(id)
        marker.pose.position.z = point.z + (marker.scale.z)
    else:
        marker.type = Marker.SPHERE
        marker.pose.position.z = point.z

    return marker

def rotate_vect(a, b, rot):
    """
    Поворачиваем b относительно a на угол rot
    :type a: list
    :type b: list
    :type rot: float
    :type dist: float
    :return: возвращаем точку повёрнутую на нужный угол

    """
    rotate = np.array([[math.cos(-rot), -math.sin(-rot)],
                       [math.sin(-rot), math.cos(-rot)]])

    pos = np.array([[a[0]],
                    [a[1]]])
    val = np.dot(rotate, pos)
    val[0] += b[0]
    val[1] += b[1]
    return [val[0][0], val[1][0], a[0]]

def speed_limit_goal(new_goal, prev_goal, dt, max_speed):
    """

    :type prev_goal: Goal
    :type _goal: Goal
    :param dt: float
    :param max_speed: float
    :return: limit goal
    """
    limit_goal = Goal()

    vec = [new_goal.pose.point.x - prev_goal.pose.point.x,
           new_goal.pose.point.y - prev_goal.pose.point.y,
           new_goal.pose.point.z - prev_goal.pose.point.z]

    dist = np.linalg.norm(vec)

    if dist > max_vel * dt:
        res = vec / dist * max_speed * dt
        limit_goal.pose.point.x = res[0] + prev_goal.pose.point.x
        limit_goal.pose.point.y = res[1] + prev_goal.pose.point.y
        limit_goal.pose.point.z = res[2] + prev_goal.pose.point.z
        return limit_goal

    else:
        return new_goal

def speed_limit_vec(new_goal, prev_goal, dt, max_speed):
    """

    :type prev_goal: Goal
    :type _goal: Goal
    :param dt: float
    :param max_speed: float
    :return: limit goal
    """
    limit_goal = [0., 0., 0.]

    vec = [new_goal[0] - prev_goal[0],
           new_goal[1] - prev_goal[1],
           new_goal[2] - prev_goal[2]]

    dist = np.linalg.norm(vec)

    if dist > max_vel * dt:
        res = vec / dist * max_speed * dt
        limit_goal[0] = res[0] + prev_goal[0]
        limit_goal[1] = res[1] + prev_goal[1]
        limit_goal[2] = res[2] + prev_goal[2]
        return limit_goal

    else:
        return new_goal

def rep_force(dist_to_obs):
    # вида 1/х
    return c_rep / dist_to_obs - c_rep / r_porog


def goal_clb(data):
    """
    Get common goal pose
    :type data:Goal
    :return:
    """
    global goal_common_msgs, state_init_flag

    goal_common_msgs = data

    state_init_flag = True


def repel_from_near(lerp_point, drone_num, course, radius=1.0):
    """
    Отталкиваемся от дронов если они возши в заданный радиус
    :param name:
    :param list:
    :return:
    """
    repel = [0.0, 0.0, 0.0]

    for i in range(len(drone_offset_list)):
        if drone_offset_list[i][0] != drone_offset_list[drone_num][0]:
            dist, vec = get_distance(lerp_point, drone_offset_list[i][2])
            if dist < radius:
                if abs(course) <= np.deg2rad(90):
                    dir = 1.0
                else:
                    dir = -1.0
                # print("%s -> %s. dist:%s, vec:%s" % (drone_offset_list[drone_num][0],drone_offset_list[i][0],dist, vec))
                repel[0] += vec[0] * rep_force(dist)
                repel[1] += vec[1] * rep_force(dist)
                if vec[2] == 0.:
                    repel[2] += max_vel * rep_force(dist) * dir
                else:
                    repel[2] += vec[2] * rep_force(dist) * dir
    return repel


def get_distance(a, b):
    """
    Distance between 2 GOAL
    :type a: Goal
    :type b: Goal
    :return: float
    """
    vec = [a.pose.point.x - b.pose.point.x,
           a.pose.point.y - b.pose.point.y,
           a.pose.point.z - b.pose.point.z]

    return np.linalg.norm(vec), vec


def get_course(target_goal, current_goal):
    """
    :type target_goal: Goal
    :type current_goal: Goal
    :return:
    """
    vec = [target_goal.pose.point.x - current_goal.pose.point.x,
           target_goal.pose.point.y - current_goal.pose.point.y]

    res = math.atan2(vec[1], vec[0])
    return res


def rotate_goal(a, b, rot):
    """
    Поворачиваем b относительно a на угол rot
    :type a: Point
    :type b: Point
    :type rot: float
    :type dit: float
    :param a: common goal pose
    :param b: drone_0_offset of drone goal
    :param rot: rotate of current pose
    :return: возвращаем точку повёрнутую на нужный угол

    """
    new_goal = Goal()
    rotate = np.array([[math.cos(rot), -math.sin(rot)],
                       [math.sin(rot), math.cos(rot)]])

    pos = np.array([[b.x],
                    [b.y]])

    res = np.dot(rotate, pos)
    res[0] += a.x
    res[1] += a.y
    new_goal.pose.point.x = float(res[0])
    new_goal.pose.point.y = float(res[1])
    new_goal.pose.point.z = float(a.z + b.z)
    new_goal.pose.course = float(rot)
    return new_goal


def get_drone_pose(common_goal, offset):
    new_pose = Goal()
    return new_pose


def load_params_from_path(path):
    """
    Load data of drones from yaml
    :type path: str
    :return:
    """
    global size_of_drone, drone_offset_list, markers_goal, goal_common_msgs, state_init_flag

    try:
        file = open(path, "r")
        data = str(file.read())
        file.close()
    except:
        rospy.logerr("Param error path")
        sys.exit()



    drone_offset_list = list()
    markers_goal.markers = list()
    markers_lerp.markers = list()
    markers_lerp_text.markers = list()
    markers_goal_text.markers = list()

    # set data "name" and "offset" to list
    size_of_drone = yaml.load(data)['size']
    i = 0
    while len(drone_offset_list) < size_of_drone:
        try:
            offset_str = yaml.load(data)[tag + str(i)]
            offset_data = Point()
            offset_data.x = offset_str['x']
            offset_data.y = offset_str['y']
            offset_data.z = offset_str['z']
            new_goal = Goal()
            new_goal.pose.point.x = offset_data.x
            new_goal.pose.point.y = offset_data.y
            new_goal.pose.point.z = offset_data.z

            if state_init_flag:
                new_goal.pose.point.x += goal_common_msgs.pose.point.x
                new_goal.pose.point.y += goal_common_msgs.pose.point.y
                new_goal.pose.point.z += goal_common_msgs.pose.point.z

            drone_offset_list.append([tag + str(i), offset_data, new_goal])
            markers_goal.markers.append(Marker())
            markers_goal_text.markers.append(Marker())
            markers_lerp.markers.append(Marker())
            markers_lerp_text.markers.append(Marker())
        except:
            rospy.logerr("%s:param not found" % (tag + str(i)))
        i += 1


def load_params(form):
    """
    Load data of drones from yaml
    :type formation: FormationParam
    :return:
    """
    global size_of_drone, drone_offset_list, markers_goal, state_init_flag, goal_common_msgs

    struct, group_size = create_virtual_structure(form)

    size_of_drone = form.size
    # drone_offset_list [name :str, offset :Point, prevPoint: Goal]

    drone_offset_list = list()
    markers_goal.markers = list()
    markers_lerp.markers = list()
    markers_lerp_text.markers = list()
    markers_goal_text.markers = list()
    for i in range(size_of_drone):
        # try:
        offset_data = Point()
        offset_data.x = struct[i][0]
        offset_data.y = struct[i][1]
        offset_data.z = 0.
        new_goal = Goal()
        new_goal.pose.point.x = offset_data.x
        new_goal.pose.point.y = offset_data.y
        new_goal.pose.point.z = offset_data.z

        drone_offset_list.append([form.tag + "_" + str(i), offset_data, new_goal])
        markers_goal.markers.append(Marker())
        markers_goal_text.markers.append(Marker())
        markers_lerp.markers.append(Marker())
        markers_lerp_text.markers.append(Marker())
    reset_pose()

def reset_pose():
    """
    Reset formation position

    :return:
    """
    global drone_offset_list, state_init_flag, goal

    for item in drone_offset_list:
        i = item
    #
    #     drone_offset_list.append([form.tag + "_" + str(i), offset_data, new_goal])
    #
    #     new_goal.pose.point.x += goal_common_msgs.pose.point.x
    #     new_goal.pose.point.y += goal_common_msgs.pose.point.y
    #     new_goal.pose.point.z += goal_common_msgs.pose.point.z



if __name__ == '__main__':
    rospy.init_node("swarm_goal_node")
    print("swarm_goal_node: init")

    # get param
    param_path = rospy.get_param("~param", param_path)
    tag = rospy.get_param("~tag", tag)
    max_vel = rospy.get_param("~max_vel", max_vel)
    use_yaml = rospy.get_param("~use_yaml", use_yaml)


    if use_yaml:
        print("yaml path:",param_path)
        if param_path == "":
            rospy.logerr("Param not set")
            sys.exit()

        load_params_from_path(param_path)
    else:
        # test
        form = FormationParam()
        form.type = FormationParam.KLIN
        form.size = 3
        form.distance = 2
        form.tag = "drone"
        load_params(form)


    rospy.Subscriber("/goal_pose", Goal, goal_clb)


    # pub markers
    pub_marker_goal_array = rospy.Publisher("/goal_markers", MarkerArray, queue_size=10)
    pub_marker_goal_array_text = rospy.Publisher("/goal_markers/text", MarkerArray, queue_size=10)
    pub_marker_goal_lerp_array = rospy.Publisher("/goal_markers/lerp", MarkerArray, queue_size=10)
    pub_marker_goal_lerp_text_array = rospy.Publisher("/goal_markers/text", MarkerArray, queue_size=10)


    rate = rospy.Rate(20)

    drone_goal_msgs = Goal()
    old_time = rospy.get_time()

    try:
        while not rospy.is_shutdown():
            if not state_init_flag:
                rate.sleep()
                continue

            name = ""
            dt = rospy.get_time() - old_time

            # Перебираем массив всех дронов
            for i in range(len(drone_offset_list)):

                name_of_drone = drone_offset_list[i][0]
                # получаем целевую точку куда нужно двигаться
                drone_goal_msgs = rotate_goal(goal_common_msgs.pose.point, drone_offset_list[i][1],
                                              goal_common_msgs.pose.course)
                drone_goal_msgs.pose.course = goal_common_msgs.pose.course

                # Интерполируем точку от текущей до целевой
                if state_init_flag:
                    lerp_goal = speed_limit_goal(drone_goal_msgs, drone_offset_list[i][2], dt, max_vel)
                    lerp_goal.pose.course = goal_common_msgs.pose.course
                else:
                    lerp_goal = drone_goal_msgs

                # get course
                course = get_course(drone_goal_msgs, lerp_goal)

                # отталкиваемся от соседей
                repel_vec = speed_limit_vec(repel_from_near(lerp_goal, i, course, r_porog), [0, 0, 0], dt, max_vel)
                repel_rot_vec = rotate_vect(repel_vec, [0, 0, 0], np.deg2rad(-25))
                lerp_goal.pose.point.x += repel_rot_vec[0]
                lerp_goal.pose.point.y += repel_rot_vec[1]
                lerp_goal.pose.point.z += repel_rot_vec[2]
                # указываем интерполированную точку как предыдущую
                drone_offset_list[i][2] = lerp_goal

                markers_lerp.markers[i] = setup_market(name_of_drone,
                                                       lerp_goal.pose.point,
                                                       i, [0.0, 0.0, 1.0, 0.7])
                markers_lerp_text.markers[i] = setup_market(name_of_drone,
                                                       lerp_goal.pose.point,
                                                       i, [1.0, 0.0, 0.0, 1.0], text_flag=True)

                markers_goal.markers[i] = setup_market(name_of_drone,
                                                       drone_goal_msgs.pose.point,
                                                       i, [0.0, 1.0, 0.0, 1.0])

                markers_goal_text.markers[i] = setup_market(name_of_drone,
                                                       drone_goal_msgs.pose.point,
                                                       i, [0.0, 0.3, 0.0, 1.0], text_flag=True)
                rospy.Publisher(name_of_drone + "/geo/goal_pose", Goal, queue_size=10).publish(lerp_goal)

            # print("len array:", len(drone_offset_list), size_of_drone)
            pub_marker_goal_array.publish(markers_goal)
            pub_marker_goal_array_text.publish(markers_goal_text)
            pub_marker_goal_lerp_array.publish(markers_lerp)
            pub_marker_goal_lerp_text_array.publish(markers_lerp_text)
            old_time = rospy.get_time()

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
