#Usage: 
#!/usr/bin/env python3
# this script read rosbag and corespondent intrinsic parameters get from quaterKalibr output rectified fisheye image with topic
# output: rosbag has oak_ffc_4p/rectified_CAM_A oak_ffc_4p/rectified_CAM_B  oak_ffc_4p/rectified_CAM_C oak_ffc_4p/rectified_CAM_D
# oak_ffc_4p/rectified_CAM_A_left oak_ffc_4p/rectified_CAM_A_right
# FisheyeUndist can be used in other similar scenario.

import cv2 as cv
from transformations import *
import numpy as np

## read four camera intrinsic parameters


class FisheyeUndist:
    def __init__(self, camera_matrix, dist_coeffs, xi, fov=190, width=1000, height=200, extrinsic=None):
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.xi = xi
        self.generatePinhole2(fov, width, height)
        self.generatePinhole1(fov, width, height)
        if extrinsic is None:
            self.extrinsic = np.eye(4)
        else:
            self.extrinsic = extrinsic

    def generateUndistMapPinhole(self, R, focal_length, width, height):
        # R: rotation matrix
        # Knew: new camera matrix
        # size: image size
        pts3d = np.zeros((1, width*height, 3), dtype=np.float32)
        c = 0
        for i in range(height):
            for j in range(width):
                p3d = R@[j - width/2, i - height/2, focal_length]
                pts3d[0, c] = p3d
                c += 1
        rvec, tvec = np.array([0., 0., 0.]), np.array([0., 0., 0.])
        pts2d_raw, _ = cv.omnidir.projectPoints(pts3d, rvec, tvec, self.camera_matrix, self.xi, self.dist_coeffs)
        mapxy = pts2d_raw.reshape((height, width, 2))
        #Map from imgPtsundist to pts2d_raw
        mapx, mapy = cv.convertMaps(mapxy, None, cv.CV_32FC1)
        return mapx, mapy

    def getPinholeCamExtrinsic(self, idx):
        if idx == 0:
            return self.extrinsic @ euler_matrix(0, -np.pi/4, 0, 'sxyz')
        else:
            return self.extrinsic @ euler_matrix(0, np.pi/4, 0, 'sxyz')

    def generatePinhole2(self, fov, width, height):  #split fisheye into two pinhole
        pinhole_fov = np.deg2rad(fov - 90)
        focal_gen = width / 2 / np.tan(pinhole_fov / 2)
        R0 = euler_matrix(0, -np.pi/4, 0, 'sxyz')[0:3, 0:3]
        R1 = euler_matrix(0, np.pi/4, 0, 'sxyz')[0:3, 0:3]
        self.focal_gen = focal_gen
        map1 = self.generateUndistMapPinhole(R0, focal_gen, width, height)
        map2 = self.generateUndistMapPinhole(R1, focal_gen, width, height)
        self.maps = [map1, map2]

    def generatePinhole1(self, fov, width, height):
        pinhole_fov = np.deg2rad(fov)
        focal_gen = width / pinhole_fov
        R0 = euler_matrix(0, 0, 0, 'sxyz')[0:3, 0:3]
        self.focal_gen = focal_gen
        map1 = self.generateUndistMapPinhole(R0, focal_gen, width, height)
        self.mono_map = map1
    
    def undistAll(self, img):
        imgs = []
        for map in self.maps:
            _img = cv.remap(img, map[0], map[1], cv.INTER_AREA)
            imgs.append(_img)
        return imgs
    
    def undistMono(self, img):
        map = self.mono_map
        mono_img = cv.remap(img, map[0], map[1], cv.INTER_AREA)
        return mono_img
    
    
    def undist(self, img, idx):
        return cv.remap(img, self.maps[idx][0], self.maps[idx][1], cv.INTER_AREA)
    

# This tool distort fisheye image to 2 

if __name__ == "__main__":
    # Test code
    import argparse
    parser = argparse.ArgumentParser(description='Fisheye undist')
    parser.add_argument("-i","--input", type=str, help="input image file")
    parser.add_argument("-f","--fov", type=float, default=190, help="hoizon fov of fisheye")
    args = parser.parse_args()

    K = np.array([[1124.7283061913233, 0, 632.7040736742416],
                [0, 1124.8562955001719,  327.56651546593287],
                [0, 0, 1]])
    D = np.array([-0.21171233219482108, 0.4394442689857262, 0.001541272532917834, -0.0007433853293709857])
    xi = 2.2176903753419963

    undist = FisheyeUndist(K, D, xi, fov=args.fov)

    print(f"read image from{args.input}")
    img = cv.imread(r"/home/khalil/ssd_data_1/firefly_dataset/led_flash/split/image_127_1_split.jpg")
    imgs = undist.undistAll(img)
    mono_img = undist.undistMono(img)
    # imgs.append(mono_img)

    show_left = imgs[0]
    show_right = imgs[1]
    # show_mono = imgs[2]
    # for i in range(1, len(imgs)):
    #     show = cv.hconcat([show, imgs[i]])
    cv.namedWindow("raw", cv.WINDOW_NORMAL|cv.WINDOW_GUI_EXPANDED)
    cv.imshow("raw", img)
    cv.namedWindow("Undist_left", cv.WINDOW_NORMAL|cv.WINDOW_GUI_EXPANDED)
    cv.imshow("Undist_left", show_left)
    cv.namedWindow("Undist_right", cv.WINDOW_NORMAL|cv.WINDOW_GUI_EXPANDED)
    cv.imshow("Undist_right", show_right)
    cv.namedWindow("Undist_mono", cv.WINDOW_NORMAL|cv.WINDOW_GUI_EXPANDED)
    cv.imshow("Undist_mono", mono_img)
    cv.waitKey(0)