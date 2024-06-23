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

    k_l = 0
    k_r = 0
    k_m = 0
    # 左边图像
    image_left = ROI(frame,200,600,0,300)
    l1 = thresh_binary(image_left,250)
    l2 = cv2.Canny(l1,100,100)   
    # cv2.imshow('image_left', l2)
    lines_l = cv2.HoughLinesP(l2,1,np.pi/180,50,2,0,20)
    if lines_l is not None:
      for line_l in lines_l:
        line_l=line_l.reshape(4)
        #length.append(math.sqrt((line[0] - line[2]) ** 2 + (line[1] - line[3]) ** 2))
        cv2.line(image_left,(line_l[0],line_l[1]),(line_l[2],line_l[3]),(0,0,255),1)
        k_l = (line_l[3]-line_l[1])/(line_l[2]-line_l[0])
    cv2.imshow('image_left', image_left)
    

    # 右边图像
    image_right = ROI(frame,200,600,500,800) 
    r1 = thresh_binary(image_right,250)
    r2 = cv2.Canny(r1,100,100) 
    # cv2.imshow('image_right', r2)

    lines_r = cv2.HoughLinesP(r2,1,np.pi/180,50,2,0,20)
    if lines_r is not None:
      for line_r in lines_r:
        line_r=line_r.reshape(4)
        #length.append(math.sqrt((line[0] - line[2]) ** 2 + (line[1] - line[3]) ** 2))
        cv2.line(image_right,(line_r[0],line_r[1]),(line_r[2],line_r[3]),(0,0,255),1)
        k_r = (line_r[3]-line_r[1])/(line_r[2]-line_r[0])
    cv2.imshow('image_right', image_right)
    

    # 中间图像
    image_middle = ROI(frame,200,600,300,500) 
    m1 = thresh_binary(image_middle,250)
    m2 = cv2.Canny(m1,100,100) 
    # cv2.imshow('image_middle', m2)

    lines_m = cv2.HoughLinesP(m2,1,np.pi/180,50,2,0,20)
    if lines_m is not None:
      for line_m in lines_m:
        line_m=line_m.reshape(4)
        #length.append(math.sqrt((line[0] - line[2]) ** 2 + (line[1] - line[3]) ** 2))
        cv2.line(image_middle,(line_m[0],line_m[1]),(line_m[2],line_m[3]),(0,0,255),1)
        k_m = (line_m[3]-line_m[1])/(line_m[2]-line_m[0])
    cv2.imshow('image_middle', image_middle)
    
    
    cv2.waitKey(1)
    l_angle = math.atan(k_l)
    r_angle = math.atan(k_r)
    print(l_angle,r_angle)
    k=r_angle-l_angle
    speed = 2.5
    k=k_l+k_r
    speed = 2
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