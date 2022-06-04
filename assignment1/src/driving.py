#!/usr/bin/env python
# -*- coding: utf-8 -*-

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import numpy as np
import cv2
import math
import rospy, rospkg, time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from math import *
import signal
import sys
import os
import random

#=============================================
# 터미널에서 Ctrl-C 키입력으로 프로그램 실행을 끝낼 때
# 그 처리시간을 줄이기 위한 함수
#=============================================
def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
image = np.empty(shape=[0]) # 카메라 이미지를 담을 변수
bridge = CvBridge() 
motor = None # 모터 토픽을 담을 변수

#=============================================
# 프로그램에서 사용할 상수 선언부
#=============================================
CAM_FPS = 30    # 카메라 FPS - 초당 30장의 사진을 보냄
WIDTH, HEIGHT = 640, 480    # 카메라 이미지 가로x세로 크기

#=============================================
# 콜백함수 - 카메라 토픽을 처리하는 콜백함수
# 카메라 이미지 토픽이 도착하면 자동으로 호출되는 함수
# 토픽에서 이미지 정보를 꺼내 image 변수에 옮겨 담음.
#=============================================
def img_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

#=============================================
# 모터 토픽을 발행하는 함수  
# 입력으로 받은 angle과 speed 값을 
# 모터 토픽에 옮겨 담은 후에 토픽을 발행함.
#=============================================
def drive(angle, speed):

    global motor

    motor_msg = xycar_motor()
    motor_msg.angle = angle
    motor_msg.speed = speed

    motor.publish(motor_msg)

#=============================================
#ROI 지역설정
#=============================================

def region_of_interest(img_1, vertices, color3=(255,255,255), color1=255): # ROI 셋팅

    mask = np.zeros_like(img_1) # mask = img와 같은 크기의 빈 이미지
    
    if len(img_1.shape) > 2: # Color 이미지(3채널)라면 :
        color = color3
    else: # 흑백 이미지(1채널)라면 :
        color = color1
        
    # vertices에 정한 점들로 이뤄진 다각형부분(ROI 설정부분)을 color로 채움 
    cv2.fillPoly(mask, vertices, color)
    
    # 이미지와 color로 채워진 ROI를 합침
    ROI_image = cv2.bitwise_and(img_1, mask)
    return ROI_image

#=============================================
#빨간색으로 차선 인식
#=============================================

def mark_img(img_1, blue_threshold=200, green_threshold=200, red_threshold=200): # 흰색 차선 찾기

    global img, mark
    #  BGR 제한 값
    bgr_threshold = [blue_threshold, green_threshold, red_threshold]

    # BGR 제한 값보다 작으면 검은색으로
    thresholds = (img[:,:,0] < bgr_threshold[0]) \
                | (img[:,:,1] < bgr_threshold[1]) \
                | (img[:,:,2] < bgr_threshold[2])
    mark[thresholds] = [0,0,0]
    return mark


#=============================================
# 실질적인 메인 함수 
# 카메라 토픽을 받아 각종 영상처리와 알고리즘을 통해
# 차선의 위치를 파악한 후에 조향각을 결정하고,
# 최종적으로 모터 토픽을 발행하는 일을 수행함. 
#=============================================


def start():

    # 위에서 선언한 변수를 start() 안에서 사용하고자 함
    global motor, image, img, mark

    #=========================================
    # ROS 노드를 생성하고 초기화 함.
    # 카메라 토픽을 구독하고 모터 토픽을 발행할 것임을 선언
    #=========================================
    rospy.init_node('driving')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)

    print ("----- Xycar self driving -----")

    # 첫번째 카메라 토픽이 도착할 때까지 기다림.
    while not image.size == (WIDTH * HEIGHT * 3):
        continue
 
    #=========================================
    # 메인 루프 
    # 카메라 토픽이 도착하는 주기에 맞춰 한번씩 루프를 돌면서 
    # "이미지처리 +차선위치찾기 +조향각결정 +모터토픽발행" 
    # 작업을 반복적으로 수행함.
    #=========================================
    while not rospy.is_shutdown():



        # 이미지처리를 위해 카메라 원본이미지를 img에 복사 저장
        img = image.copy()  
        mark = np.copy(img)
        height, width = img.shape[:2]



	#======================================================

        vertices = np.array([[(-130,height),(width/2-70, height/2+40), (width/2+80, height/2+40), (width+110,height)]], dtype=np.int32)
        roi_img = region_of_interest(image, vertices,(0,0,255))

        mark = np.copy(roi_img)
        mark = mark_img(roi_img)	
        
        color_thresholds = (mark[:,:,0] == 0) & (mark[:,:,1] == 0) & (mark[:,:,2] > 200)
        img[color_thresholds] = [0,0,255]
            # 디버깅을 위해 모니터에 이미지를 디스플레이
        cv2.imshow('roi_white',mark) # 흰색 차선 추출 결과 출력       
        cv2.imshow("CAM View", img)
        cv2.waitKey(1)       
        
            #=========================================
            # 핸들조향각 값인 angle값 정하기.
            # 차선의 위치 정보를 이용해서 angle값을 설정함.        
            #=========================================
        
            # 우선 테스트를 위해 직진(0값)으로 설정
        angle = 0
        
            #=========================================
            # 차량의 속도 값인 speed값 정하기.
            # 직선 코스에서는 빠른 속도로 주행하고 
            # 회전구간에서는 느린 속도로 주행하도록 설정함.
            #=========================================

            # 우선 테스트를 위해 느린속도(10값)로 설정
        speed = 10
        
            # drive() 호출. drive()함수 안에서 모터 토픽이 발행됨.
        drive(angle, speed)


#=============================================
# 메인 함수
# 가장 먼저 호출되는 함수로 여기서 start() 함수를 호출함.
# start() 함수가 실질적인 메인 함수임. 
#=============================================
if __name__ == '__main__':
    start()

