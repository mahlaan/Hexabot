#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
import time 
import cv2 as cv
from crackDetection import isCrack
from cv_bridge import CvBridge
import matplotlib.pyplot as plt

class CrackDetector():
    """
        Class to analyze images from Kinect and to publish images with crack detected.
    """
    # On definit les fonctiond de callback pour chaque topic : elles vont stocker les images collectees dans les variables de classe correspondantes
    def __init__(self,rate,plotting=False):
        
        self.plotting = plotting
    # On cree les variables qui vont stocker les images 
        
        self.imgFrontDepth = None
        self.imgFrontColor = None
        
    # Pour utiliser opencv
        self.bridge = CvBridge()

    # On cree les subscribers pour chaque image : front, left et right, en couleur et profondeur
    
        rospy.Subscriber("/phantomx/camera_front/depth/image_raw", Image,self.callbackFrontDepth)
        rospy.Subscriber("/phantomx/camera_front/color/image_raw", Image,self.callbackFrontColor)
                
    # On cree les publisher des images de detection de fissures sur 3 differents topics front, left et right
    
        self.rate = rospy.Rate(rate)
        
        self.pubFront =  rospy.Publisher('/phantomx/crack_image',Image,queue_size=10)

        time.sleep(3)
        self.publisherGeneral()     
    
    # On spin
        rospy.spin()
    
    # On definit maintenant les publishers qui seront appeles par les callbacks des subscribers


    def callbackFrontDepth(self,img):
        """
            Convert an Image message (depth) to an image whithout nan values
        """
        self.imgFrontDepth = self.bridge.imgmsg_to_cv2(img)
        self.imgFrontDepth = np.nan_to_num(self.imgFrontDepth)
        
    def callbackFrontColor(self,img):
        """
            Convert an Image message to an image in graylevel
        """
        self.imgFrontColor = cv.cvtColor(self.bridge.imgmsg_to_cv2(img), cv.COLOR_BGR2GRAY)


    def publisherFront(self):
        """
            Publish the result of isCrask() fonction from crackDetection.py
            -----
            If self.plotting is True then plot the Input Data (GrayScale), The Depth data and the result of isCrack()
        """
        crack_dist=isCrack(self.imgFrontColor,self.imgFrontDepth)
        if crack_dist is not None:
            to_send = self.bridge.cv2_to_imgmsg(crack_dist)
            self.pubFront.publish(to_send)
            if self.plotting:
                plt.figure()
                plt.subplot(131)
                plt.imshow(imgColor,cmap='gray')
                plt.title("Image brut")
                plt.subplot(132)
                plt.imshow(imgDepth,cmap='gray')
                plt.title("Depth")
                plt.subplot(133)
                plt.imshow(crack_dist,cmap='gray')
                plt.title("Crack Distance")
                plt.show()
    
    def publisherGeneral(self):
        """
            Main loop for Publisher call
        """
        while not rospy.is_shutdown():
            if (self.imgFrontDepth is not None) and (self.imgFrontDepth is not None):
                self.publisherFront()
            self.rate.sleep()
                               
if __name__ == '__main__':
    rospy.init_node("crackDetector")
    CrackDetector(0.5)

