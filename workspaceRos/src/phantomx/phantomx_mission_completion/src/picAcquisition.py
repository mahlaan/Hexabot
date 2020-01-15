#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import time 

class ImageCollector():
    
    # On definit les fonctiond de callback pour chaque topic : elles vont stocker les images collectees dans les variables de classe correspondantes
       
    def callbackFrontDepth(self,img):
	    self.imgFrontDepth = img
	    
    def callbackFrontColor(self,img):
	    self.imgFrontColor = img
	    
    def callbackLeftDepth(self,img):
	    self.imgLeftDepth = img
	    
    def callbackLeftColor(self,img):
	    self.imgLeftColor = img
	    
    def callbackRightDepth(self,img):
	    self.imgRightDepth = img
	    
    def callbackRightColor(self,img):
	    self.imgRightColor = img

    def __init__(self,rate):
        
    # On cree les variables qui vont stocker les images 
        
        self.imgFrontDepth = None
        self.imgFrontColor = None
        self.imgLeftDepth = None
        self.imgLeftColor = None
        self.imgRightDepth = None
        self.imgRightColor = None

    # On cree les subscribers pour chaque image : front, left et right, en couleur et profondeur
    
        rospy.Subscriber("/phantomx/camera_front/depth/image_raw", Image,self.callbackFrontDepth)
        rospy.Subscriber("/phantomx/camera_front/color/image_raw", Image,self.callbackFrontColor)
        rospy.Subscriber("/phantomx/camera_left/depth/image_raw", Image,self.callbackLeftDepth)
        rospy.Subscriber("/phantomx/camera_left/color/image_raw", Image,self.callbackLeftColor)
        rospy.Subscriber("/phantomx/camera_right/depth/image_raw", Image,self.callbackRightDepth)
        rospy.Subscriber("/phantomx/camera_right/color/image_raw", Image,self.callbackRightColor)
        
        

        
    # On cree les publisher de ces memes images sur des topics differents (avec un rate qu on peut choisir)
    
        self.rate = rospy.Rate(rate)
        
        self.pubFrontDepth =  rospy.Publisher('/front_depth',Image,queue_size=10)
        self.pubFrontColor =  rospy.Publisher('/front_color',Image,queue_size=10)
        self.pubLeftDepth =  rospy.Publisher('/left_depth',Image,queue_size=10)
        self.pubLeftColor =  rospy.Publisher('/left_color',Image,queue_size=10)
        self.pubRightDepth =  rospy.Publisher('/right_depth',Image,queue_size=10)
        self.pubRightColor =  rospy.Publisher('/right_color',Image,queue_size=10)
        
        time.sleep(3)
        self.publisherGeneral()     
    # On spin
        rospy.spin()
    
    # On definit maintenant les publishers qui seront appeles par les callbacks des subscribers

    def publisherFrontDepth(self,img):
        self.pubFrontDepth.publish(img)

    def publisherFrontColor(self,img):
        self.pubFrontColor.publish(img)

    def publisherLeftDepth(self,img):
        self.pubLeftDepth.publish(img)

    def publisherLeftColor(self,img):
        self.pubLeftColor.publish(img)

    def publisherRightDepth(self,img):
        self.pubRightDepth.publish(img)

    def publisherRightColor(self,img):
        self.pubRightColor.publish(img)
    
    def publisherGeneral(self):
        while not rospy.is_shutdown():
            self.publisherFrontDepth(self.imgFrontDepth)
            self.publisherFrontColor(self.imgFrontColor)
            self.publisherLeftDepth(self.imgLeftDepth)
            self.publisherLeftColor(self.imgLeftColor)
            self.publisherRightDepth(self.imgRightDepth)
            self.publisherRightColor(self.imgRightColor)
            self.rate.sleep()
                               
if __name__ == '__main__':
    rospy.init_node("collector")
    coll = ImageCollector(0.1)

