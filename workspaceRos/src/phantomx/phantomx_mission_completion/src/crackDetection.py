#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt

def preprocessing(img):
    """
        Input:
            img: RGBD
        Output:
            new_img: image in GrayLevel
            distance: D channel of the input img
    """
    rgb=img[:,:,:3]
    bgr = rgb[:, :, ::-1]
    distance=img[:,:-1]
    new_img =  cv.cvtColor(bgr, cv.COLOR_BGR2GRAY)
    return(new_img,distance)

def isCrack(img,dist,thresh_level=10):
    """
        Input:
            img : GrayLevel
            dist : Matrix of distance of each pixel of img
            thresh_level : Threshold level for binarization

        Output:
            crack_dist : None if no crack detection, distance of the crack (img) if detection
    """
    th = img<thresh_level
    crack_dist=th*dist
    if not True in th:
        return None
    else:
        return crack_dist

if __name__ == '__main__':
    """
        Test of the detection with local pictures
    """

    PATH='../ressources/ImagesTest/'

    img = cv.imread(PATH+'image14.png')

    img,dist=preprocessing(img)

    plt.figure()
    plt.imshow(img,cmap="gray")
    crack_dist=isCrack(img,img)
    if crack_dist is not None:
        print("Crack detection")
        plt.figure()
        plt.imshow(crack_dist,cmap="gray")
    else:
        print("No crack detection")
    plt.show()
