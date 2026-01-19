#!/home/donghy/miniconda3/bin/python3
# ROS 2版本 - 捕获标准球图像以便后续calibration.py生成table
from sensor_msgs.msg import CompressedImage
import numpy as np
import time
import rclpy
from rclpy.node import Node 
import math
import cv2
import os
import sys
import argparse

count = 0

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--images", required=True, help="Number of calibration images to capture")
args = vars(ap.parse_args())
num = int(args["images"])

print('[INFO] Capturing {} images...'.format(num))
print('[INFO] Do not press the calibration ball to the sensor for the first image (ref.jpg)...')
print('[INFO] {} images will be captured at a regular time interval...'.format(num))
print('[INFO] Beginning calibration image capture...')
time.sleep(2)

current_script_path = os.path.abspath(__file__)
current_script_dir = os.path.dirname(current_script_path)
pkg_path = os.path.dirname(current_script_dir)

save_dir = os.path.join(pkg_path, 'test_data')
if not os.path.exists(save_dir):
    os.makedirs(save_dir)
    print('[INFO] Created directory: {}'.format(save_dir))

class SlipDetectionReaction(Node):
    def __init__(self):
        super().__init__('slip_detector')
        self.image_sub = self.create_subscription(
            CompressedImage,
            "/image_raw/compressed",
            self.call_back,
            1 
        )
        self.image_sub 

    def rgb2gray(self, rgb):
        return np.dot(rgb[...,:3], [0.5, 0.5, 0.])

    def call_back(self, data):
        global count
        t = time.time()
    
        np_arr = np.frombuffer(data.data, np.uint8)
        raw_imag = cv2.imdecode(np_arr, cv2.IMREAD_COLOR).astype(np.uint8)

        cv2.imshow('raw_image', raw_imag)
        cv2.waitKey(1)

        count += 1
        frame_idx = count // 100  
        
        # 保存参考图（第100帧）
        if frame_idx == 1:
            cv2.imwrite(os.path.join(save_dir, 'ref.jpg'), raw_imag)
            print('[INFO] Saved reference image to {}'.format(save_dir))
        # 保存采样图（第200~(num+1)*100帧）
        elif 1 < frame_idx <= (num + 1):
            sample_name = 'sample_{}.jpg'.format(frame_idx - 1)
            cv2.imwrite(os.path.join(save_dir, sample_name), raw_imag)
            print('[INFO] Saved sample {} to {}'.format(frame_idx - 1, save_dir))
        # 采集完成后退出
        elif frame_idx > (num + 1):
            cv2.destroyAllWindows()
            print('[INFO] Calibration image capture complete...')
            print('[INFO] Please remove empty images from {}'.format(save_dir))
            print('[INFO] Press Ctrl+C in terminal to exit...')
            rclpy.shutdown()
            return

def main(args=None):
    rclpy.init(args=args)
    slip_detector = SlipDetectionReaction()
    rclpy.spin(slip_detector)
    
    slip_detector.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
