#! /usr/bin/env python
# Created by Chen Yuxuan
# Modified by Tian Bo
from collections import deque
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from move_base_msgs.msg import MoveBaseActionGoal

FILTER_VALUE = 10.0
def get_range(data, angle, deg=True):
    if deg:
        angle = np.deg2rad(angle)
    dis = data.ranges[int((angle - data.angle_min) / data.angle_increment)]
    if dis < data.range_min or dis > data.range_max:
        dis = FILTER_VALUE
    return dis


def wall_following_callback(data):
    """
    Implements simple wall following at
    https://linklab-uva.github.io/autonomousracing/assets/files/assgn4-print.pdf
    """
    # the angle between the two laser rays
    THETA = np.pi / 180 * 60
    # target distance from wall
    TARGET_DIS = 0.7
    # the distance to project the car forward
    LOOK_AHEAD_DIS = 3
    P = 0.5

    # naming convention according to above pdf
    b = get_range(data, -90)
    a = get_range(data, -90 + np.rad2deg(THETA))
    # print(f"a{a:1.1f} b{b:1.1f}")
    alpha = np.arctan((a * np.cos(THETA) - b) / (a * np.sin(THETA)))
    AB = b * np.cos(alpha)
    projected_dis = AB + LOOK_AHEAD_DIS * np.sin(alpha)
    error = TARGET_DIS - projected_dis
    # print(f"error{error:1.1f}")
    steering_angle = P * error
    # 新增速度调整逻辑
    MIN_SPEED = 0.5  # 最小速度
    MAX_SPEED = 3.5  # 最大速度
    SPEED_ADAPTATION_FACTOR = 0.7  # 调整因子，影响速度变化的敏感度

    # 计算动态速度
    if abs(error-TARGET_DIS / 2) <= 0.1:  # 当误差较小，允许较高速度
        speed = MAX_SPEED
    else:  # 当误差较大，降低速度
        speed = MAX_SPEED - abs(error) * SPEED_ADAPTATION_FACTOR
        speed = min(max(speed, MIN_SPEED), MAX_SPEED)  # 确保速度在允许范围内

    front_dis = get_range(data, 0)
    #speed can be set to 0.5 to 3.5 m/s, 3 by default
    # speed = 0.5
    angle_filter = steering_angle
    # # 新增滤波器变量
    # FILTER_WINDOW_SIZE = 5  # 滤波窗口大小，可根据需要调整
    # steering_angles_buffer = deque(maxlen=FILTER_WINDOW_SIZE)  # 使用deque作为循环缓冲区

    # # 添加当前转向角到缓冲区并计算平均值
    # steering_angles_buffer.append(steering_angle)
    # filtered_steering_angle = sum(steering_angles_buffer) / FILTER_WINDOW_SIZE
    drive_msg = AckermannDriveStamped()
    drive_msg.drive.steering_angle=angle_filter 
    drive_msg.drive.speed=speed
    drive_pub.publish(drive_msg)

def check_move_base_status():
   try:
      rospy.wait_for_message("/tianracer/move_base/goal",MoveBaseActionGoal,timeout=3.0)
      return True
   except rospy.ROSException as e:
      rospy.loginfo("move base has not yet available:%s",e)
      return False

if __name__ == '__main__': 
  # try:
    rospy.init_node("wall_following")
    # while not check_move_base_status():
    #     rospy.loginfo_once("Waiting for /tianracer/move/base/goal message...")
    #     rospy.sleep(1)

    rospy.loginfo_once("Received goal for move_base, starting wall following node...")
    scan_sub = rospy.Subscriber('/tianracer/scan', LaserScan, wall_following_callback)
    drive_pub = rospy.Publisher('/tianracer/ackermann_cmd_stamped', AckermannDriveStamped, queue_size=1)
    try:
      rospy.spin()
    except rospy.ROSInterruptException:
      pass