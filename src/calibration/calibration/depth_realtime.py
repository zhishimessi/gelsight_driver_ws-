#!/home/donghy/miniconda3/bin/python3
# -*- coding: utf-8 -*-

from sensor_msgs.msg import CompressedImage, JointState, ChannelFloat32,PointCloud2, PointField
from std_msgs.msg import Bool
import numpy as np
import time
from scipy import ndimage
import matplotlib.pyplot as plt
from visualization_msgs.msg import *
import rclpy
from rclpy.node import Node
import math
import cv2
import os
import pickle
import std_srvs.srv
# from fast_poisson import fast_poisson
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits import mplot3d
import collections
from std_msgs.msg import Header
import struct

from scipy.fftpack import dst
from scipy.fftpack import idst

def fast_poisson(gx,gy):
    
#    j = 1:ydim-1; 
#	k = 1:xdim-1;
#
#	% Laplacian
#	gyy(j+1,k) = gy(j+1,k) - gy(j,k); 
#	gxx(j,k+1) = gx(j,k+1) - gx(j,k);
    
    
    m,n = gx.shape
    gxx = np.zeros((m,n))
    gyy = np.zeros((m,n))
    f = np.zeros((m,n))
    img = np.zeros((m,n))
    gyy[1:,:-1] = gy[1:,:-1] - gy[:-1,:-1]
    gxx[:-1,1:] = gx[:-1,1:] - gx[:-1,:-1]
    f = gxx + gyy 
    
    f2 = f[1:-1,1:-1].copy()
    
    f_sinx = dst(f2,norm='ortho')
    f_sinxy = dst(f_sinx.T,norm='ortho').T
    

    
    x_mesh, y_mesh = np.meshgrid(range(n-2),range(m-2)) 
    x_mesh = x_mesh +1
    y_mesh = y_mesh +1
    denom = (2*np.cos(np.pi*x_mesh/(n-1))-2) + (2*np.cos(np.pi*y_mesh/(m-1))-2)
    
    
    f3 = f_sinxy/denom
#    plt.figure(10)
#    plt.imshow(denom)
#    plt.show()
    f_realx = idst(f3,norm='ortho')
    f_realxy = idst(f_realx.T,norm='ortho').T
    img[1:-1,1:-1] = f_realxy.copy()
    return img

class SlipDetectionReaction(Node):
    def __init__(self):
        super().__init__('depth_reconstruction')
        
        current_script_path = os.path.abspath(__file__)
        current_script_dir = os.path.dirname(current_script_path)
        pkg_root = os.path.dirname(current_script_dir)
        table_save_dir = os.path.join(pkg_root, 'load')

        self.img_counter1 = 0
        self.img_counter2 = 0
        # self.table = np.load(os.path.join(table_save_dir, "table_3_smooth.npy"))
        self.table = np.load("/home/donghy/Desktop/thesis/gelsight_driver/Bnz/ROS2/gelsight_driver_ws/src/calibration/load/table_3_smooth.npy")
        self.pad = 20
        self.zeropoint = -90
        self.lookscale = 180
        self.bin_num = 90
        self.con_flag1 = True
        self.con_flag2 = True
        self.reset_shape1 = True
        self.reset_shape2 = True
        self.restart1 = False
        self.restart2 = False
        self.reset_trigger1 = False
        self.reset_trigger2 = False
        self.slip_indicator1 = False
        self.slip_indicator2 = False
        self.refresh1 = False
        self.refresh2 = False
        self.showimage1 = False
        self.scale = 1
        self.kernel = self.make_kernel(31, 'circle')
        self.kernel2 = self.make_kernel(9, 'circle')

        self.image_sub2 = self.create_subscription(
            CompressedImage,
            "/image_raw/compressed",
            self.call_back1,
            1,  
        )
        self.image_sub2

    def call_back1(self, data):
        t = time.time()
        if self.con_flag1:
            np_arr = np.frombuffer(data.data, np.uint8)
            raw_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            ref_image = self.crop_image(raw_image, self.pad)
            marker = self.marker_detection(ref_image.copy())
            keypoints = self.find_dots((1 - marker) * 255)
            if self.reset_shape1:
                marker_mask = self.make_mask(ref_image.copy(), keypoints)
                ref_image = cv2.inpaint(ref_image, marker_mask, 3,
                                        cv2.INPAINT_TELEA)
                self.red_mask = (ref_image[:, :, 2] > 12).astype(np.uint8)
                self.dmask1 = self.defect_mask(ref_image[:, :, 0])
                self.ref_blur1 = cv2.GaussianBlur(ref_image.astype(np.float32),
                                                (3, 3), 0)
                self.blur_inverse1 = 1 + ((np.mean(self.ref_blur1) /
                                        (self.ref_blur1 + 1)) - 1) * 2

            self.u_addon1 = list(np.zeros(len(keypoints)))
            self.v_addon1 = list(np.zeros(len(keypoints)))
            self.x_iniref1 = []
            self.y_iniref1 = []
            marker_num = len(keypoints)
            self.mp_array = np.zeros((marker_num, 3, 200))
            for i in range(marker_num):
                self.x_iniref1.append(keypoints[i].pt[0] / self.scale)
                self.y_iniref1.append(keypoints[i].pt[1] / self.scale)
                self.mp_array[i, :, self.img_counter1 % 100] = np.array(
                    [keypoints[i].pt[0], keypoints[i].pt[1], 0])
            self.index_ref = np.linspace(0, marker_num - 1,
                                        marker_num).astype(int)
            self.con_flag1 = False
            self.reset_shape1 = False
        else:
            if self.restart1:
                self.con_flag1 = True
                self.restart1 = False

            np_arr = np.frombuffer(data.data, np.uint8)
            raw_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            raw_image = self.crop_image(raw_image, self.pad)
            raw_image = cv2.GaussianBlur(raw_image.astype(np.float32), (3, 3),
                                        0)

            marker_mask = self.marker_detection(raw_image) * self.dmask1
            grad_img2 = self.matching_v2(raw_image, self.ref_blur1,
                                        self.blur_inverse1)
            
            grad_x = grad_img2[:, :, 0] * (1 - marker_mask) * self.red_mask  
            grad_y = grad_img2[:, :, 1] * (1 - marker_mask) * self.red_mask

            grad_x_scaled = grad_x * 10.0  
            grad_y_scaled = grad_y * 10.0
            
            denom = np.sqrt(1.0 + grad_x_scaled**2 + grad_y_scaled**2)
            normal_x = -grad_x_scaled / denom
            normal_y = -grad_y_scaled / denom
            normal_z = 1.0 / denom
            
            normal_vector = np.stack([normal_x, normal_y, normal_z], axis=-1)
            normal_magnitude = np.linalg.norm(normal_vector, axis=-1, keepdims=True)
            normal_magnitude[normal_magnitude == 0] = 1e-8
            normal_vector_normalized = normal_vector / normal_magnitude

            N_disp = 0.5 * (normal_vector_normalized + 1.0)
            N_disp = np.clip(N_disp, 0, 1)
            
            N_disp_cv2 = (N_disp * 255).astype(np.uint8)
            N_disp_cv2 = cv2.cvtColor(N_disp_cv2, cv2.COLOR_RGB2BGR)
            cv2.imshow('Normal Vector (RGB)', N_disp_cv2)

            depth = fast_poisson(grad_x, grad_y)
            depth[depth < 0] = 0

            depth_normalized = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
            depth_normalized = depth_normalized.astype(np.uint8)
            
            # COLORMAP_VIRIDIS / COLORMAP_MAGMA / COLORMAP_INFERNO 
            depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_VIRIDIS)
            
            cv2.namedWindow('depth', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('depth', 800, 600)  
            cv2.imshow('depth', depth_colored) 
            cv2.waitKey(1)

        self.img_counter1 += 1
        print(1 / (time.time() - t))

    def crop_image(self, img, pad):
        return img[pad:-pad, pad:-pad]

    def defect_mask(self, img):
        pad = 20
        im_mask = np.ones((img.shape))
        im_mask[:pad, :] = 0
        im_mask[-pad:, :] = 0
        im_mask[:, :pad * 2 + 20] = 0
        im_mask[:, -pad:] = 0
        return im_mask.astype(int)

    def flow_calculate_in_contact(self, keypoints2, x_initial, y_initial,
                                  u_ref, v_ref):
        x2, y2, u, v, x1_paired, y1_paired, x2_paired, y2_paired = [], [], [], [], [], [], [], []

        refresh = False
        for i in range(len(keypoints2)):
            x2.append(keypoints2[i].pt[0] / self.scale)
            y2.append(keypoints2[i].pt[1] / self.scale)

        x2 = np.array(x2)
        y2 = np.array(y2)

        for i in range(x2.shape[0]):
            distance = list(((np.array(x_initial) - x2[i])**2 +
                             (np.array(y_initial) - y2[i])**2))
            if len(distance) == 0:
                break
            min_index = distance.index(min(distance))
            u_temp = x2[i] - x_initial[min_index]
            v_temp = y2[i] - y_initial[min_index]
            shift_length = np.sqrt(u_temp**2 + v_temp**2)

            if shift_length < 12:
                x1_paired.append(x_initial[min_index] - u_ref[min_index])
                y1_paired.append(y_initial[min_index] - v_ref[min_index])
                x2_paired.append(x2[i])
                y2_paired.append(y2[i])
                u.append(u_temp + u_ref[min_index])
                v.append(v_temp + v_ref[min_index])

                del x_initial[min_index], y_initial[min_index], u_ref[
                    min_index], v_ref[min_index]

                if shift_length > 7:
                    refresh = True

        return x1_paired, y1_paired, x2_paired, y2_paired, u, v, refresh

    def flow_calculate_global(self, keypoints2, x_initial, y_initial, u_ref,
                              v_ref, contact_mask, depth):
        x2, y2, u, v, x1_paired, y1_paired, x2_paired, y2_paired  = [], [], [], [], [], [], [], []
        x1_return, y1_return, x2_return, y2_return, u_return, v_return = [],[],[],[],[],[]
        index_ref = []

        for i in range(len(keypoints2)):
            x2.append(keypoints2[i].pt[0] / self.scale)
            y2.append(keypoints2[i].pt[1] / self.scale)

        x2 = np.array(x2)
        y2 = np.array(y2)

        for i in range(x2.shape[0]):
            distance = (((np.array(x_initial) - x2[i])**2 +
                         (np.array(y_initial) - y2[i])**2))

            if len(distance) == 0:
                break
            min_index = np.argmin(distance)

            u_temp = x2[i] - x_initial[min_index]
            v_temp = y2[i] - y_initial[min_index]
            shift_length = np.sqrt(u_temp**2 + v_temp**2)
            
            if shift_length < 12:
                x1_paired.append(x_initial[min_index] - u_ref[min_index])
                y1_paired.append(y_initial[min_index] - v_ref[min_index])
                x2_paired.append(x2[i])
                y2_paired.append(y2[i])
                index_ref.append(self.index_ref[min_index])
                self.mp_array[self.index_ref[min_index],:,self.img_counter1%100] = \
                np.array([x2[i], y2[i], depth[int(y2[i]), int(x2[i])]])
                u.append(u_temp + u_ref[min_index])
                v.append(v_temp + v_ref[min_index])

        self.index_ref = list(index_ref)
        inbound_check = contact_mask[np.array(y2_paired).astype(np.uint16),
                                     np.array(x2_paired).
                                     astype(np.uint16)] * np.array(
                                         range(len(x2_paired))).astype(int)

        final_list = list(set(inbound_check) - set([0]))
        x1_inbound = np.array(x1_paired)[final_list]
        y1_inbound = np.array(y1_paired)[final_list]
        x2_inbound = np.array(x2_paired)[final_list]
        y2_inbound = np.array(y2_paired)[final_list]
        u_inbound = np.array(u)[final_list]
        v_inbound = np.array(v)[final_list]

        x1_return = np.array(x1_paired)
        y1_return = np.array(y1_paired)
        x2_return = np.array(x2_paired)
        y2_return = np.array(y2_paired)
        u_return = np.array(u)
        v_return = np.array(v)

        return x1_return, y1_return, x2_return, y2_return, u_return, v_return, \
            list(x2_paired), list(y2_paired), list(u), list(v)

    def matching_v2(self, test_img, ref_blur, blur_inverse):
        diff_temp1 = test_img - ref_blur
        diff_temp2 = diff_temp1 * blur_inverse
        diff_temp3 = np.clip((diff_temp2 - self.zeropoint) / self.lookscale, 0,
                             0.999)
        diff = (diff_temp3 * self.bin_num).astype(int)
        grad_img = self.table[diff[:, :, 0], diff[:, :, 1], diff[:, :, 2], :]
        return grad_img

    def find_dots(self, binary_image):
        params = cv2.SimpleBlobDetector_Params()
        params.minThreshold = 1
        params.maxThreshold = 12
        params.minDistBetweenBlobs = 9
        params.filterByArea = True
        params.minArea = 5
        params.filterByCircularity = False
        params.filterByConvexity = False
        params.filterByInertia = False
        params.minInertiaRatio = 0.5
        detector = cv2.SimpleBlobDetector_create(params)
        keypoints = detector.detect(binary_image.astype(np.uint8))

        return keypoints

    def make_mask(self, img, keypoints):
        img = np.zeros_like(img[:, :, 0])
        for i in range(len(keypoints)):
            cv2.ellipse(img,
                        (int(keypoints[i].pt[0]), int(keypoints[i].pt[1])),
                        (9, 6), 0, 0, 360, (1), -1)

        return img

    def marker_detection(self, raw_image):
        m, n = raw_image.shape[1], raw_image.shape[0]
        raw_image_blur = cv2.GaussianBlur(raw_image.astype(np.float32), (5, 5),
                                          0)
        ref_blur = cv2.GaussianBlur(raw_image.astype(np.float32), (25, 25), 0)
        diff = ref_blur - raw_image_blur
        diff *= 16.0
        diff[diff < 0.] = 0.
        diff[diff > 255.] = 255.
        mask = ((diff[:, :, 0] > 25) & (diff[:, :, 2] > 25) &
                (diff[:, :, 1] > 120))
        mask = cv2.resize(mask.astype(np.uint8), (m, n))
        mask = cv2.dilate(mask, self.kernel2, iterations=1)
        return mask

    def contact_detection(self, raw_image, ref_blur, marker_mask, kernel):
        diff_img = np.max(np.abs(raw_image.astype(np.float32) - ref_blur),
                          axis=2)
        contact_mask = (((diff_img > 12).astype(np.uint8) *
                         (1 - marker_mask)) * self.dmask1).astype(np.uint8)
        print('max', np.max(diff_img))
        cv2.imshow('contact', contact_mask * 255)
        cv2.waitKey(1)
        contact_mask = cv2.dilate(contact_mask, kernel, iterations=1)
        contact_mask = cv2.erode(contact_mask, kernel, iterations=1)
        cv2.imshow('contact_mask', contact_mask * 255)
        cv2.waitKey(1)
        return contact_mask

    def make_kernel(self, n, k_type):
        if k_type == 'circle':
            kernal = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (n, n))
        else:
            kernal = cv2.getStructuringElement(cv2.MORPH_RECT, (n, n))
        return kernal

    def dispOpticalFlow(self, im_cal, x, y, u, v, name, slip_indicator):
        mask2 = np.zeros_like(im_cal)
        amf = 1
        x = np.array(x).astype(np.int16)
        y = np.array(y).astype(np.int16)
        for i in range(u.shape[0]):
            mask2 = cv2.line(mask2,
                             (int(x[i] + u[i] * amf), int(y[i] + v[i] * amf)),
                             (x[i], y[i]), [0, 120, 120], 2)

        img = cv2.add(im_cal / 1.5, mask2)
        cv2.imshow(name, img.astype(np.uint8))
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    print("start")
    plt.ion()
 
    slip_detector = SlipDetectionReaction()
    
    try:
        rclpy.spin(slip_detector)
    except KeyboardInterrupt:
        pass
    finally:
        slip_detector.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
        # 关闭3D绘图窗口
        plt.close('all')

if __name__ == "__main__":
    main()
