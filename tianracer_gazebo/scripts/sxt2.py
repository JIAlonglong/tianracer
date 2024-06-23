'''
Author: robox-xx 118261752+robox-xx@users.noreply.github.com
Date: 2024-06-21 11:46:04
LastEditors: robox-xx 118261752+robox-xx@users.noreply.github.com
LastEditTime: 2024-06-21 14:07:25
FilePath: /tianbot_ws/src/tianracer/tianracer_gazebo/scripts/sxt2.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
#! /usr/bin/env python
# Created by Chen Yuxuan
# Modified by Tian Bo

import rospy
import cv2
import math
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
   
def ROI(img,x1,x2,y1,y2):
   #图片截图
  img_part = img[x1:x2,y1:y2]
  return img_part

def thresh_binary(img,the):
   (_, pic_thresh_binary) = cv2.threshold(img,the, 255, cv2.THRESH_BINARY)
   pic_thresh_binary = cv2.medianBlur(pic_thresh_binary, 5)
   return pic_thresh_binary


def image_callback(data):
    # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        frame = np.array(cv_image, dtype=np.uint8)
    except CvBridgeError as e:
        rospy.INFO(e)
    
    # 创建灰度图像
    grey_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # cv2.imshow('grey_image', grey_image)
    ret,thresh = cv2.threshold(grey_image,220,255,cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # 初始化最大轮廓面积和索引
    max_area = 0
    max_index = -1
 
    # 遍历所有轮廓
    for i, contour in enumerate(contours):
        # 计算轮廓面积
        area = cv2.contourArea(contour)
    
        # 更新最大轮廓面积和索引
        if area > max_area:
            max_area = area
            max_index = i
 
    # 找到最大轮廓
    max_contour = contours[max_index]
    cv2.drawContours(frame, [max_contour], -1, (0, 255, 0), 3)
    # print(max_contour)

    # 轮廓点的坐标
    points = []
    for point in max_contour:
        points.append((point[0][0], point[0][1]))
        # print(point)
    # points = points.astype(np.float32)
    
    points = np.array(points,dtype=np.float32)
    # 使用cv2.fitLine()进行直线拟合
    [vx, vy, x, y] = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)
    if vx != 0:
        m = vy / vx
        c = y - m * x
        y0 = np.array([400, int(c)])
        y1 = np.array([800, int(m * 100 + c)])

    cv2.line(frame, (y0[1]-200, y0[0]), (y1[1]-200, y1[0]), (255, 0, 0), 2)
    # print((y0[0], y0[1]), (y1[0], y[1]))
    # cv2.line(frame, (45, 505), (799, 514), (255, 255, 0), 2)
    
    
    cv2.imshow('Line Fitting', thresh)
    cv2.imshow('image_middle', frame)
    cv2.waitKey(1)
  
    k=0
    speed = 0
    # if k == 0 :
    #   speed = -2.0
    # k_m +
    # drive_msg = AckermannDriveStamped()
    # drive_msg.drive.steering_angle=k
    # drive_msg.drive.speed=speed
    # drive_pub.publish(drive_msg)
    

if __name__ == '__main__': 
  # try:
    rospy.init_node("cv_bridge_test")
    bridge = CvBridge()
    image_sub = rospy.Subscriber("/tianracer/usb_cam/image_raw", Image, image_callback)
    drive_pub = rospy.Publisher('/tianracer/ackermann_cmd_stamped', AckermannDriveStamped, queue_size=1)
    try:
      rospy.spin()
    except rospy.ROSInterruptException:
      pass