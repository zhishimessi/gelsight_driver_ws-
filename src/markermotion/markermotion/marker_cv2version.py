#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import time
import numpy as np
import os
import cv2
from rclpy.executors import SingleThreadedExecutor

class GelSight_Img(object):
    def __init__(self, node, frame, frame0=None):
        self.node = node  
        self.reinit(frame, frame0)
        self.bridge = CvBridge()
        self.contactmap = np.zeros([480, 640])

        # marker locations
        self.MarkerAvailable = True  # whether the markers are found
        self.calMarker_on = False

        # parameters for display
        self.IsDisplay = False  # whether to publish the marker motion image
        self.showScale = 8

    def loc_markerArea(self):
        '''match the area of the markers; work for the Bnz GelSight'''
        MarkerThresh = -30
        I = self.img.astype(np.double) - self.f0
        self.MarkerMask = np.amax(I, 2) < MarkerThresh

    def displayIm(self):
        disIm = self.img
        markerCenter = np.around(self.flowcenter[:, 0:2]).astype(np.int16)
        for i in range(self.MarkerCount):
            if self.markerU[i] != 0:
                cv2.line(disIm, (markerCenter[i, 0], markerCenter[i, 1]),
                        (int(self.flowcenter[i, 0] + self.markerU[i] * self.showScale),
                        int(self.flowcenter[i, 1] + self.markerV[i] * self.showScale)),
                        (0, 255, 255), 2)
        # 替换ROS2发布为cv2本地显示
        cv2.namedWindow('GelSight Marker Motion', cv2.WINDOW_NORMAL)  # 自适应窗口
        cv2.imshow('GelSight Marker Motion', disIm)
        cv2.waitKey(1)  # 1ms阻塞（必须传值，否则窗口无响应，且避免阻塞ROS2回调）

    def find_markers(self):
        self.loc_markerArea()
        areaThresh1 = 50
        areaThresh2 = 400
        MarkerCenter = np.empty([0, 3])

        contours = cv2.findContours(self.MarkerMask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # ROS2兼容：适配OpenCV不同版本的contours返回值
        if len(contours) == 3:
            contours = contours[1]
        else:
            contours = contours[0]
            
        if len(contours) < 25:  # if too little markers, then give up
            self.MarkerAvailable = False
            return MarkerCenter

        for contour in contours:
            AreaCount = cv2.contourArea(contour)
            if areaThresh1 < AreaCount < areaThresh2:
                t = cv2.moments(contour)
                MarkerCenter = np.append(MarkerCenter,
                                         [[t['m10'] / t['m00'], t['m01'] / t['m00'], AreaCount]], axis=0)
        return MarkerCenter

    def cal_marker_center_motion(self, MarkerCenter):
        Nt = len(MarkerCenter)
        no_seq2 = np.zeros(Nt)
        center_now = np.zeros([self.MarkerCount, 3])
        for i in range(Nt):
            dif = np.abs(MarkerCenter[i, 0] - self.marker_last[:, 0]) + np.abs(MarkerCenter[i, 1] - self.marker_last[:, 1])
            no_seq2[i] = np.argmin(dif * (100 + np.abs(MarkerCenter[i, 2] - self.flowcenter[:, 2])))

        for i in range(self.MarkerCount):
            dif = np.abs(MarkerCenter[:, 0] - self.marker_last[i, 0]) + np.abs(MarkerCenter[:, 1] - self.marker_last[i, 1])
            t = dif * (100 + np.abs(MarkerCenter[:, 2] - self.flowcenter[i, 2]))
            a = np.amin(t) / 100
            b = np.argmin(t)
            if self.flowcenter[i, 2] < a:  # for small area
                self.markerU[i] = 0
                self.markerV[i] = 0
                center_now[i] = self.flowcenter[i]
            elif i == no_seq2[b]:
                self.markerU[i] = MarkerCenter[b, 0] - self.flowcenter[i, 0]
                self.markerV[i] = MarkerCenter[b, 1] - self.flowcenter[i, 1]
                center_now[i] = MarkerCenter[b]
            else:
                self.markerU[i] = 0
                self.markerV[i] = 0
                center_now[i] = self.flowcenter[i]
        return center_now

    def update_markerMotion(self, img=None):
        if img is not None:
            self.img = img
        MarkerCenter = self.find_markers()
        self.marker_last = self.cal_marker_center_motion(MarkerCenter)
        if self.IsDisplay:
            self.displayIm()

    def iniMarkerPos(self):
        # set the current marker position as the initial positions of the markers
        self.flowcenter = self.marker_last

    def start_display_markerIm(self):
        self.IsDisplay = True

    def stop_display_markerIm(self):
        self.IsDisplay = False

    def detect_contact(self, img=None, ColorThresh=1):
        if not self.calMarker_on:
            self.update_markerMotion(img)

        isContact = False

        # contact detection based on color
        diffim = np.int16(self.img) - self.f0
        self.contactmap = diffim.max(axis=2) - diffim.min(axis=2)
        countnum = np.logical_and(self.contactmap > 10, diffim.max(axis=2) > 0).sum()

        if countnum > self.touchthresh * ColorThresh:  # there is touch
            isContact = True

        # contact detection based on marker motion
        motion = np.abs(self.markerU) + np.abs(self.markerV)
        MotionNum = (motion > self.touchMarkerMovThresh * np.sqrt(ColorThresh)).sum()
        if MotionNum > self.touchMarkerNumThresh:
            isContact = True

        return isContact, countnum

    def ini_contactDetect(self):
        diffim = np.int16(self.img) - self.f0
        maxim = diffim.max(axis=2)
        contactmap = maxim - diffim.min(axis=2)
        countnum = np.logical_and(contactmap > 10, maxim > 0).sum()

        contactmap[contactmap < 10] = 0
        contactmap[maxim <= 0] = 0
        cv2.imwrite('iniContact.png', contactmap)

        self.touchthresh = round((countnum + 1500) * 1.0)
        self.touchMarkerMovThresh = 1
        self.touchMarkerNumThresh = 20

    def reinit(self, frame, frame0=None):
        self.img = frame  # current frame
        if frame0 is not None:
            self.f0 = frame0  # frame0 is the low
        else:
            self.f0 = np.int16(cv2.GaussianBlur(self.img, (101, 101), 50))
        self.ini_contactDetect()

        # for markers
        self.flowcenter = self.find_markers()  # center of all the markers; x,y
        self.marker_last = self.flowcenter
        self.MarkerCount = len(self.flowcenter)
        self.markerU = np.zeros(self.MarkerCount)  # X motion of all the markers. 0 if the marker is not found
        self.markerV = np.zeros(self.MarkerCount)  # Y motion of all the markers. 0 if the marker is not found


class GelSight_Bridge(Node):
    def __init__(self):
        super().__init__('gelsight_bridge_node')  # ROS2节点初始化
        self.bridge = CvBridge()
        self.img = np.zeros((100, 100, 3))
        self.topic = '/image_raw'
        
        # 回调组：避免回调冲突
        self.callback_group = MutuallyExclusiveCallbackGroup()
        
        # 初始图像加载
        self.save_iniImg()

        # 订阅器初始化
        self.sub_IncomeIm = None
        self.touchtest_on = False
        self.calMarker_on = False
        self.isContact = False
        self.callbackCount = 0
        self.startContactDetectFlag = False

        self.contactFunc = None
        self.contact_Thresh = 1
        self.touchtestInied = False

        # 初始化标记追踪实例
        self.GelSightIm = GelSight_Img(self, self.ini_img, self.frame0)

    def save_iniImg(self):
        self.get_logger().info(f"等待初始图像：{self.topic}")
        img_msg = None  # 存储收到的图像消息

        # 临时回调函数：收到消息后赋值
        def temp_callback(msg):
            nonlocal img_msg
            img_msg = msg

        # 1. 创建临时订阅器（订阅/image_raw）
        temp_sub = self.create_subscription(Image, self.topic, temp_callback, 10)
        
        # 2. 阻塞等待5秒（自旋直到收到消息）
        start_time = self.get_clock().now()
        while img_msg is None:
            # 检查超时（5秒）
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > 5.0:
                self.destroy_subscription(temp_sub)  # 销毁订阅器
                raise TimeoutError(f"超时5秒,未收到图像")
            
            rclpy.spin_once(self, timeout_sec=0.1)  # 自旋等待回调
        
        # 3. 销毁临时订阅器（避免内存泄漏）
        self.destroy_subscription(temp_sub)

        self.ini_img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        self.frame0 = np.int16(cv2.GaussianBlur(self.ini_img, (101, 101), 50))
        self.get_logger().info("初始图像加载成功")

    def show_MarkerImg(self):
        self.GelSightIm.start_display_markerIm()

    def stop_show_MarkerImg(self):
        self.GelSightIm.stop_display_markerIm()

    def start_calMarker(self):
        if not self.calMarker_on:
            self.calMarker_on = True
            self.callbackCount += 1
            self.GelSightIm.calMarker_on = True
            # 创建ROS2订阅器
            self.sub_IncomeIm = self.create_subscription(
                Image,
                self.topic,
                self.callback_incomeIm,
                1,
                callback_group=self.callback_group
            )

    def stop_calMarker(self):
        if self.calMarker_on:
            self.calMarker_on = False
            self.GelSightIm.calMarker_on = False
            self.callbackCount -= 1
            if self.callbackCount <= 0:
                self.callbackCount = 0
                if self.sub_IncomeIm is not None:
                    self.destroy_subscription(self.sub_IncomeIm)
                    self.sub_IncomeIm = None

    def callback_incomeIm(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error("CVBridge error in callback: {}".format(e))
            return

        if self.startContactDetectFlag:
            self.startContactDetectFlag = False
            self.ini_img = self.img
            self.frame0 = np.int16(cv2.GaussianBlur(self.ini_img, (101, 101), 50))
            self.GelSightIm.reinit(self.img, self.frame0)
            self.touchtestInied = True

        # calculate marker motion; MUST BE THE FIRST
        if self.calMarker_on:
            self.GelSightIm.update_markerMotion(self.img)

        # detecting contact
        if self.touchtest_on and self.touchtestInied:
            self.isContact, k = self.GelSightIm.detect_contact(self.img, self.contact_Thresh)
            if self.isContact and self.contactFunc is not None:
                self.contactFunc()

def main(args=None):
    import rclpy
    rclpy.init(args=args)
    # 初始化核心节点
    gelsight_node = GelSight_Bridge()
    try:
        # 默认启动标记显示和计算
        gelsight_node.show_MarkerImg()
        gelsight_node.start_calMarker()
        rclpy.spin(gelsight_node)
    except KeyboardInterrupt:
        gelsight_node.get_logger().info("GelSight bridge node shutdown requested")
    finally:
        # 清理资源
        gelsight_node.stop_show_MarkerImg()
        gelsight_node.stop_calMarker()
        gelsight_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()