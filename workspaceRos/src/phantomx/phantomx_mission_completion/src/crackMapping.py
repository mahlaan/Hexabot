#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Jan 29 10:50:57 2020

@author: ehnla
"""


from crackDetection import *


from sensor_msgs.msg import Image as ImageMSG
from sensor_msgs.msg import CameraInfo as CamInfoMSG
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import image_geometry
import tf2_ros as tf2
from tf2_geometry_msgs import PointStamped
import time

from visualization_msgs.msg import Marker, MarkerArray


class CrackMap():

    
    def callbackDepthPic(self,img):
        """
        Callback : take the ROS message returned by /phantomx/crack_image (Image). Save the ROS message
        and the depth picture into this message.

        ------
        Input : 
            
            img : Image Message : ROS message containing the last message published by 
            the topic /phantomx/crack_image (a depth picture containing the depth of 
            all the detected cracks on a picture).
            
                
        """
        self.depthMsg = img
        self.depthPic = self.bridge.imgmsg_to_cv2(img)
        self.PicID += 1
        return None

	    
    def callbackCIFrontColor(self,cami):
        """
        Temporary callback : only take once the camera_info matrix, then
        unsubscribe from the concerned topic.

        ------
        Input : 
            
            cami : Camera Info Message : ROS message containing all the intrinsic
            parameters of the camera (fx, fy, distortion coefficients ...)
                
        """
        self.CIFrontColor = cami
        self.CIPtopic.unregister()
        return None

        
    def fromPix2camRef(self,pixD, check_pix = False):
        """
        Express the coordinates of a pixel on the picture frame into the camera
        sensor frame.
        
        ------
        Input : 
            
            pixD : the coordinates of the pixel on the picture frame,
            and the depth of the pixel : (u,v,depth of pix)
            
            check_pix : bool : to check if the transformation picture frame --> sensor frame
            is correct (by checking the transformation sensor frame --> picture frame)
            
        Return : 
            
            p_real_3d : the coordinates of the pixel on the camera sensor frame : 
            (depth of pix = X, Y, Z)
                
        """
        
        # take camera_info 
        self.cameraModel.fromCameraInfo(self.CIFrontColor)
        
        # remove distortion 
        pix_rec = self.cameraModel.rectifyPoint( (pixD[0],pixD[1]) )

        # gives the unit vector between the point and the centre of the camera
        # (passing through an unfixed laser beam), in the image reflex
        p_3D = self.cameraModel.projectPixelTo3dRay((pix_rec[0],pix_rec[1]))
        
        # change from image frame to camera frame for unit vector
        p_3D = np.matmul(self.R_rpic2rcam,p_3D)
        
        # Associates a laser beam with the unit vector using the information 
        # depth (pix[2])
        Mp3d = np.array([p_3D[0],p_3D[1],p_3D[2]]).T
        
        p_real_3D = (pixD[2]/p_3D[0])*Mp3d

        if (check_pix):
        
            print(p_real_3D)
            
            # take 3d point in camera ref, turn into picture ref
            
            p_c_3D = np.matmul(self.R_rcam2rpic,p_real_3D)
            
            # take 3d point in picture ref in 2D
            
            p_check = self.cameraModel.project3dToPixel(p_c_3D)
            # normally, norm(p_check - pixD) < 1e-10
            print("p_check : ", p_check) 
            
        return p_real_3D
    
    def fromCamRef2caveRef(self,pr3d,current_depthMsg):
        """
        Express the coordinates of a pixel on the camera frame into the cave frame.      

        ------
        Input : 
            
           pr3d : the coordinates of the pixel on the camera sensor frame : 
           (depth of pix = X, Y, Z)
            
           current_depthMsg : Image Message : ROS message containing the informations
           about the current depth picture processed (header, stamp, ...)
            
        Return : 
            
            p_cave : the coordinates of the pixel on the cave frame : 
            (depth of pix = X, Y, Z)
                
        """
        p_cam = PointStamped()

        
        p_cam.header.seq = current_depthMsg.header.seq
        
        p_cam.header.frame_id = 'camera_front'
        p_cam.point.x = pr3d[0]
        p_cam.point.y = pr3d[1]
        p_cam.point.z = pr3d[2]

        stamp = rospy.Time.now()
        p_cam.header.stamp = stamp

        p_cave = self.buffer.transform_full(p_cam,'map',  current_depthMsg.header.stamp, 'camera_front')

        return p_cave
    

    def allPix2world(self):
        """
        Take all the pixels corresponding to separates crack on a picture, and returns the 
        barycenter of those cracks.

        Returns
        -------
        
            allpts : list, containing the barycenter of all the separate cracks on the
            current picture

        """
        allpts = []
        
        current_depthPic = self.depthPic.copy()
        current_depthMsg = self.depthMsg
        self.cur_PicID = self.PicID
        
        # binarize the current depth picture (in order to select the contours of the cracks
        # on the picture)
        bin_current_depthPic = cv2.threshold(current_depthPic,
                1e-5, 255,
                cv2.THRESH_BINARY)[1]
        
        tst = np.uint8( bin_current_depthPic.copy() )
        
        try:
            contours, hierarchy = cv2.findContours(tst, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        except:
            contours, hierarchy = cv2.findContours(tst, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1:]

        for i in range (len(contours)):
            # for each contour corresponding to a crack, return the barycenter of this crack, and express 
            # it into the cave frame
            cnt = contours[i]
            M = cv2.moments(cnt)
            cx = int(M['m10']/(M['m00']+1*10**-5))
            cy = int(M['m01']/(M['m00']+1*10**-5))
            depth = current_depthPic[cy,cx]
            bary_fis = self.fromCamRef2caveRef(self.fromPix2camRef( (cx,cy,depth) ), current_depthMsg )
            if(bary_fis.point.z > 0.15):   
                # odd condition, added to remove problems related to frame changes 
                # (indicates points related to cracks at the robot position)
 
                allpts.append(bary_fis)
        return allpts
    
    def checkSameCracks(self,allpts, dist = 0.5):
        """
        Check if the new crack barycenters detected are not too close of the last
        crack barycenters. Suppress the new crack barycenters that are too much close
        of the last ones (determined by dist), add the other cracks to the list containing 
        all the cracks (allCracksInCave).

        ------
        Input : 
            
            allpts : list, containing the barycenter of all the separate cracks on the
            current picture
            
            dist : minimal 3D distance between the barycenter of the cracks.
                
        """
        for i in range(len(self.allCracksInCave)):
            bary_fis = self.allCracksInCave[i]
            x,y,z = bary_fis.point.x, bary_fis.point.y,bary_fis.point.z
            Pt = np.array([[x],[y],[z]])
            j = 0
            while j < len(allpts):
                nbary_fis = allpts[j]
                nx,ny,nz = nbary_fis.point.x, nbary_fis.point.y, nbary_fis.point.z
                
                nPt = np.array([[nx],[ny],[nz]])
                if ( np.linalg.norm(nPt-Pt) < dist):
                    # suppress the new points if distance between pts really small
                    del(allpts[j])

                j+= 1
                
        self.allCracksInCave += allpts     
        return None
        
    
    def displayCracks(self, display_console = False):
        """
        Display all the barycenter of the cracks detected, into the topic 
        /phantomx/crack_markers and also in the terminal (chosen by the user)

        ------
        Input : 
            
            display_console : bool, indicates if the coordinates of the 
            markers associated to the barycenters must be published into the
            terminal

        """
        
        if (display_console):
            if (self.nb_cracks!= len(self.allCracksInCave)):
                # Only display into the terminal if new cracks have been found
                print("---------------- %% Cracks found %% -----------------\n")
                
                for i in range(len(self.allCracksInCave)):
                    print("Crack number %i : \n"%(i+1))
                    print(self.allCracksInCave[i])
                    print("\n")
                    
                self.nb_cracks = len(self.allCracksInCave)
        
        if (True):
            liste_marker = []
            
            for i  in range( len(self.allCracksInCave) ) :
                fissure = self.allCracksInCave[i]
                marker = Marker()
                marker.header.frame_id = "/map"
                marker.header.stamp = rospy.get_rostime()
                marker.type = 2
                marker.pose.position.x = fissure.point.x
                marker.pose.position.y = fissure.point.y
                marker.pose.position.z = fissure.point.z
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                marker.id = i
                marker.scale.x = 0.3
                marker.scale.y = 0.3
                marker.scale.z = 0.3
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                marker.lifetime = rospy.Duration(1)
                liste_marker.append(marker)
                

            self.markerPub.publish(MarkerArray(markers=liste_marker))
            
        return None
    
    def checkCracks(self):
        """
        Take the barycenter of all the cracks detected on the current depth 
        picture, and check if the new crack barycenters detected are not too close of the last
        crack barycenters.
        After all the treatments, check if a new ROS depth message has not been 
        recorded by the programm. If it's not the case, suppress the ROS depth
        message that has just been processed.

        """
        allpts = self.allPix2world()
        self.checkSameCracks(allpts,self.dist_center_cracks)
        if (self.cur_PicID == self.PicID):

            self.depthMsg = None
            self.depthPic = None
        return None
        
    
    
    def __init__(self,rate):

        """
        
        Constructor of the CrackMap class. 
        
        
        ------
        
        Input : 
            
            rate : the rate associated to rospy.
        
        Parameters : 
            
            PicID : int, associated with the ID of the last depth message
            recorded.
            
            cur_PicID : int, associated with the ID of the currently processed
            depth message (by the checkCracks function)
            
            CIFrontColor : Camera Info object, containing all the intrinsic parameters
            of the camera.
            
            depthMsg : Image Message : ROS message containing the last message published by 
            the topic /phantomx/crack_image (a depth picture containing the depth of 
            all the detected cracks on a picture).
            
            depthPic : OpenCV Matrix, containing the last depth picture published by
            the topic /phantomx/crack_image.
            
            allCracksInCave : list, containing all the cracks detected since the beginning
            of the simulation.
            
            nb_cracks : int, containing the number of cracks detected since the beginning of the
            simulation.
            
            dist_center_cracks : float, indicating the minimum 3D distance between the barycenters
            of the cracks.
            
            R_rpic2rcam : rotationnal matrix from the picture frame to the camera sensor frame
            
            R_rcam2rpic : rotationnal matrix from the camera sensor frame to the picture frame
            
            buffer : BufferInterface object, stores the last transformation matrices between frames for
            a certain time (60 seconds here).
            
            listener : TransformListener object, used to express the coordinates of a pixel from the 
            camera sensor frame into the cave frame.
            
        """
        
        self.bridge = CvBridge()
        self.PicID = 0
        self.cur_PicID = 0
        self.CIFrontColor = None
        self.depthMsg = None
        self.depthPic = None
        self.allCracksInCave = []
        self.nb_cracks = len(self.allCracksInCave)
        
        self.dist_center_cracks = 0.2
        
        self.buffer = tf2.Buffer(rospy.Duration(60)) # prend 60s de tf 
        self.listener = tf2.TransformListener(self.buffer)
        
    
        self.CIPtopic = rospy.Subscriber("/phantomx/camera_front/color/camera_info", CamInfoMSG,self.callbackCIFrontColor)
        rospy.Subscriber("/phantomx/crack_image", ImageMSG,self.callbackDepthPic)


        self.rate = rospy.Rate(rate)
        
        self.markerPub = rospy.Publisher('/phantomx/crack_markers', MarkerArray, queue_size=10)

        time.sleep(3)
        
        self.cameraModel = image_geometry.PinholeCameraModel()

        self.R_rcam2rpic = np.array([[0.0,-1.0,0.0],[0.,0.,-1.0],[1.0,0.0,0.0]])
        self.R_rpic2rcam = np.linalg.inv(self.R_rcam2rpic)
        
        while not rospy.is_shutdown():
            if (self.depthMsg is not None):

                self.checkCracks()
            self.displayCracks()
            self.rate.sleep()
        
        return None


if __name__ == "__main__":
    rospy.init_node("crackMapping", anonymous=False, log_level=rospy.DEBUG)
    cm = CrackMap(1)

