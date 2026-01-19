#!/usr/bin/env python3
'''
ros2 run image_publisher image_publisher_node --ros-args -p 
filename:=/home/donghy/Desktop/thesis/gelsight_driver/Bnz/ROS2/gelsight_driver_ws/src/markermotion/video/video.avi
ros2 run markermotion gelsight_driver 
ros2 run rqt_image_view rqt_image_view
'''

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
        self.node = node  # 持有ROS2节点引用
        self.reinit(frame, frame0)
        self.bridge = CvBridge()
        self.contactmap = np.zeros([480, 640])

        # 初始te marker locations
        self.MarkerAvailable = True  # whether the markers are found
        self.calMarker_on = False

        # parameters for display
        self.IsDisplay = False  # whether to publish the marker motion image
        self.showScale = 8
        # ROS2发布器
        self.pub = self.node.create_publisher(Image, '/gelsight/MarkerMotion', 2)

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
        self.pub.publish(self.bridge.cv2_to_imgmsg(disIm, "bgr8"))

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
        self.markerSlipInited = False
        self.markerU = np.zeros(self.MarkerCount)  # X motion of all the markers. 0 if the marker is not found
        self.markerV = np.zeros(self.MarkerCount)  # Y motion of all the markers. 0 if the marker is not found


class GelSight_Bridge(Node):
    def __init__(self):
        super().__init__('gelsight_bridge_node')  # ROS2节点初始化
        self.bridge = CvBridge()
        self.img = np.zeros((100, 100, 3))
        self.writepath = '/home/robot/catkin_ws/data/' + time.strftime("%y%m%d") + '/'
        self.writecount = 0
        self.topic = '/image_raw'
        
        # 回调组：避免回调冲突
        self.callback_group = MutuallyExclusiveCallbackGroup()
        
        # 初始图像加载
        self.save_iniImg()

        # 订阅器初始化（ROS2需先声明，后赋值）
        self.sub_IncomeIm = None
        self.sub_saveim = None
        self.sub_IncomeIm_working = False

        self.saveim_on = False
        self.touchtest_on = False
        self.calMarker_on = False
        self.isContact = False
        self.callbackCount = 0
        self.startContactDetectFlag = False

        self.args = None
        self.contactFunc = None
        self.contact_Thresh = 1
        self.touchtestInied = False
        self.savename = 'Im'
        self.img_record = np.zeros((480, 640, 3, 500), dtype=np.uint8)
        self.t = 0
        self.trial = 0

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
                raise TimeoutError(f"超时5秒，未收到图像")
            
            rclpy.spin_once(self, timeout_sec=0.1)  # 自旋等待回调
        
        # 3. 销毁临时订阅器（避免内存泄漏）
        self.destroy_subscription(temp_sub)

        # 4. 后续逻辑和ROS1一致
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

    def callback_saveim(self, data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # ROS2时间戳获取
            now = self.get_clock().now()
            sec = now.seconds
            nsec = now.nanoseconds
            timestamp = int(sec % 1e5 * 1e2 + nsec / 1e7)
            filename = f"{self.savename}_{self.writecount}_{timestamp}.jpg"
            cv2.imwrite(os.path.join(self.writepath, filename), img)
            self.writecount += 1
        except CvBridgeError as e:
            self.get_logger().error("CVBridge error: {}".format(e))

    def start_saveIm(self, savename='Im'):
        '''start the raw GelSight image to save the frames, with the preflix name savename'''
        self.writecount = 0
        self.savename = savename
        self.sub_saveim = self.create_subscription(
            Image,
            self.topic,
            self.callback_saveim,
            1,
            callback_group=self.callback_group
        )

    def stop_saveIm(self):
        if self.sub_saveim is not None:
            self.destroy_subscription(self.sub_saveim)
            self.sub_saveim = None

    def stop_contactDetect(self):
        if self.touchtest_on:
            self.touchtestInied = False
            self.touchtest_on = False
            self.callbackCount -= 1
            if self.callbackCount <= 0:
                self.callbackCount = 0
                if self.sub_IncomeIm is not None:
                    self.destroy_subscription(self.sub_IncomeIm)
                    self.sub_IncomeIm = None

    def start_contactDetect(self, contactfunc=None, contact_Thresh=1):
        self.contactFunc = contactfunc
        self.contact_Thresh = contact_Thresh
        self.isContact = False
        self.startContactDetectFlag = True
        if not self.touchtest_on:
            self.touchtest_on = True
            self.callbackCount += 1
            self.sub_IncomeIm = self.create_subscription(
                Image,
                self.topic,
                self.callback_incomeIm,
                1,
                callback_group=self.callback_group
            )

    def Change_SaveDir(self, dir_path):
        if dir_path is None or not isinstance(dir_path, str):
            self.get_logger().error("Error: a destination directory should be given")
            return
        self.writepath = dir_path
        if not os.path.isdir(self.writepath):
            os.makedirs(self.writepath)
            self.get_logger().info(f"Folder created: {self.writepath}")

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


############# Test Functions #################
def test_saveRawIm(savefolder=None, savename=None):
    '''save the raw images from GelSight. save folder is the writing path; savename is the preflix of the images.'''
    rclpy.init()
    if savename is None:
        savename = 'GelIm'
    if savefolder is None:
        savefolder = '/home/robot/Documents/GelSight_Rec/'

    gelsight_node = GelSight_Bridge()
    gelsight_node.Change_SaveDir(savefolder)
    gelsight_node.get_logger().info('start saving GelSight images')
    gelsight_node.start_saveIm(savename)
    
    # ROS2睡眠+自旋（替代time.sleep）
    executor = SingleThreadedExecutor()
    executor.add_node(gelsight_node)
    start_time = time.time()
    while time.time() - start_time < 10 and rclpy.ok():
        executor.spin_once(timeout_sec=0.1)
    
    gelsight_node.get_logger().info('finish saving images')
    gelsight_node.stop_saveIm()
    gelsight_node.destroy_node()
    rclpy.shutdown()


def test_showMarker():
    rclpy.init()
    gelsight_node = GelSight_Bridge()
    gelsight_node.get_logger().info("ini done")
    gelsight_node.show_MarkerImg()
    gelsight_node.start_calMarker()

    # ROS2自旋（替代rospy.spin）
    try:
        rclpy.spin(gelsight_node)
    except KeyboardInterrupt:
        gelsight_node.get_logger().info("Stopping marker display...")
    finally:
        gelsight_node.stop_show_MarkerImg()
        gelsight_node.stop_calMarker()
        gelsight_node.destroy_node()
        rclpy.shutdown()


def test_ContactDetect():
    ''' Contact detection using color change and marker motion '''
    ContactThresh = 1.5  # higher = harder to detect contact

    def ContactFunc():
        print('!!!!!In Contact')

    rclpy.init()
    gelsight_node = GelSight_Bridge()
    gelsight_node.start_contactDetect(ContactFunc, ContactThresh)
    
    # ROS2睡眠+自旋
    executor = SingleThreadedExecutor()
    executor.add_node(gelsight_node)
    start_time = time.time()
    while time.time() - start_time < 10 and rclpy.ok():
        executor.spin_once(timeout_sec=0.1)
    
    gelsight_node.stop_contactDetect()
    gelsight_node.destroy_node()
    rclpy.shutdown()

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

# if __name__ == '__main__':
    # test_saveRawIm()
    # test_showMarker()
    # test_ContactDetect()
