#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import time 
import cv2 as cv
from crackDetection import isCrack
from cv_bridge import CvBridge
import matplotlib.pyplot as plt

class ImageCollector():
    
    # On definit les fonctiond de callback pour chaque topic : elles vont stocker les images collectees dans les variables de classe correspondantes
       
    def callbackFrontDepth(self,img):
	    self.imgFrontDepth = self.bridge.imgmsg_to_cv2(img)
	    
    def callbackFrontColor(self,img):
	    self.imgFrontColor = cv.cvtColor(self.bridge.imgmsg_to_cv2(img), cv.COLOR_BGR2GRAY)
	    
    def callbackLeftDepth(self,img):
	    self.imgLeftDepth = self.bridge.imgmsg_to_cv2(img)
	    
    def callbackLeftColor(self,img):
	    self.imgLeftColor = cv.cvtColor(self.bridge.imgmsg_to_cv2(img), cv.COLOR_BGR2GRAY)
	    
    def callbackRightDepth(self,img):
	    self.imgRightDepth = self.bridge.imgmsg_to_cv2(img)
	    
    def callbackRightColor(self,img):
	    self.imgRightColor = cv.cvtColor(self.bridge.imgmsg_to_cv2(img), cv.COLOR_BGR2GRAY)

    def __init__(self,rate):
        
    # On cree les variables qui vont stocker les images 
        
        self.imgFrontDepth = None
        self.imgFrontColor = None
        self.imgLeftDepth = None
        self.imgLeftColor = None
        self.imgRightDepth = None
        self.imgRightColor = None
        
    # Pour utiliser opencv
        self.bridge = CvBridge()

    # On cree les subscribers pour chaque image : front, left et right, en couleur et profondeur
    
        rospy.Subscriber("/phantomx/camera_front/depth/image_raw", Image,self.callbackFrontDepth)
        rospy.Subscriber("/phantomx/camera_front/color/image_raw", Image,self.callbackFrontColor)
        rospy.Subscriber("/phantomx/camera_left/depth/image_raw", Image,self.callbackLeftDepth)
        rospy.Subscriber("/phantomx/camera_left/color/image_raw", Image,self.callbackLeftColor)
        rospy.Subscriber("/phantomx/camera_right/depth/image_raw", Image,self.callbackRightDepth)
        rospy.Subscriber("/phantomx/camera_right/color/image_raw", Image,self.callbackRightColor)
                
    # On cree les publisher des images de detection de fissures sur 3 differents topics front, left et right
    
        self.rate = rospy.Rate(rate)
        
        self.pubFront =  rospy.Publisher('/front',Image,queue_size=10)
#        self.pubFrontColor =  rospy.Publisher('/front_color',Image,queue_size=10)
        self.pubLeft =  rospy.Publisher('/left',Image,queue_size=10)
#        self.pubLeftColor =  rospy.Publisher('/left_color',Image,queue_size=10)
        self.pubRight =  rospy.Publisher('/right',Image,queue_size=10)
#        self.pubRightColor =  rospy.Publisher('/right_color',Image,queue_size=10)
        
        time.sleep(3)
        self.publisherGeneral()     
    
    # On spin
        rospy.spin()
    
    # On definit maintenant les publishers qui seront appeles par les callbacks des subscribers

    def publisherFront(self,imgColor,imgDepth):
        crack_dist=isCrack(imgColor,imgDepth)
        if crack_dist is not None:
            print("Crack detection front camera")
            self.pubFront.publish(self.bridge.cv2_to_imgmsg(crack_dist))
            plt.figure()
            plt.imshow(imgColor)
            plt.show()
        else:
            print("No crack detection front camera")
        

#    def publisherFrontColor(self,img):
#        self.pubFrontColor.publish(img)

    def publisherLeft(self,imgColor,imgDepth):
        crack_dist=isCrack(imgColor,imgDepth)
        if crack_dist is not None:
            print("Crack detection left camera")
            self.pubLeft.publish(self.bridge.cv2_to_imgmsg(crack_dist))
        else:
            print("No crack detection left camera")

#    def publisherLeftColor(self,img):
#        self.pubLeftColor.publish(img)

    def publisherRight(self,imgColor,imgDepth):
        crack_dist=isCrack(imgColor,imgDepth)
        if crack_dist is not None:
            print("Crack detection right camera")
            self.pubRight.publish(self.bridge.cv2_to_imgmsg(crack_dist))
        else:
            print("No crack detection right camera")


#    def publisherRightColor(self,img):
#        self.pubRightColor.publish(img)
    
    def publisherGeneral(self):
        while not rospy.is_shutdown():
            self.publisherFront(self.imgFrontColor,self.imgFrontDepth)
#            self.publisherFrontColor(self.imgFrontColor)
            self.publisherLeft(self.imgLeftColor,self.imgLeftDepth)
#            self.publisherLeftColor(self.imgLeftColor)
            self.publisherRight(self.imgRightColor,self.imgRightDepth)
#            self.publisherRightColor(self.imgRightColor)
            self.rate.sleep()
                               
if __name__ == '__main__':
    rospy.init_node("collector")
    coll = ImageCollector(0.05)

