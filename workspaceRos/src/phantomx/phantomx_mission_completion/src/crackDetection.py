#!/usr/bin/env python
# -*- coding: utf-8

import rospy
import rospkg
import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt

def preprocessing(img):
    """
        Input:
            img: BGRD
        Output:
            new_img: image in GrayLevel
            distance: D channel of the input img
    """
    bgr=img[:,:,:3]
    distance=img[:,:-1]
    new_img =  cv.cvtColor(bgr, cv.COLOR_BGR2GRAY)
    return(new_img,distance)

def isCrack(img,dist,thres=0.15):
    """
        Input:
            img : GrayLevel
            dist : Matrix of distance of each pixel of img
            thres : Threshold to detect or not a crack from its histograme larger

        Output:
            crack_dist : None if no crack detection, distance of the crack (img) if detection
    """
    hist = cv.calcHist([img], [0], None, [256], [0, 256])
    larger_hist=np.std(hist>1)
    if (larger_hist<thres):
        return None
    else:
        ret,th = cv.threshold(img,0,255,cv.THRESH_BINARY+cv.THRESH_OTSU)
        crack_dist=th*dist
        return(crack_dist)

if __name__ == '__main__':
    """
        Test of the detection with local pictures
    """

    PATH='../ressources/ImagesTest/'

    img = cv.imread(PATH+'Capture d’écran de 2020-01-08 10-55-36.png')
    #    img = cv.imread(PATH+'Capture d’écran de 2020-01-08 10-55-48_SANS_FISSURE.png')

    plt.figure()
    plt.imshow(img)
    img,dist=preprocessing(img)
    crack_dist=isCrack(img,img)
    if crack_dist is not None:
        print("Crack detection")
        plt.figure()
        plt.imshow(crack_dist)
    else:
        print("No crack detection")
    plt.show()
