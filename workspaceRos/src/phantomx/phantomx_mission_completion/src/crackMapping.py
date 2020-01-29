#!/usr/bin/env python
# -*- coding: utf-8

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
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

# if__name__ == "__main__":
    

from visualization_msgs.msg import Marker, MarkerArray


#     pass


class CrackMap():
    
    # On definit les fonctions de callback pour chaque topic : elles vont stocker les images collectees dans les variables de classe correspondantes
    def callbackDepthPic(self,img):
        self.depthMsg = img
        self.depthPic = self.bridge.imgmsg_to_cv2(img)
        self.PicID += 1
        return(None)
    
    def callbackCIFrontDepth(self,cami):
        self.CIFrontDepth = cami
        self.CIDtopic.unregister()
        return(None)
        # faire unregister de subscriber ap recup
        # ou rospy.waitformessage (recup un mess puis arrête)
        
	    
    def callbackCIFrontColor(self,cami):
        self.CIFrontColor = cami
        self.CIPtopic.unregister()
        return(None)

        
    def fromPix2camRef(self,pixD):
        """
        pixD : (u,v,depth of pix)
        """
        
         # récup du topic
        self.cameraModel.fromCameraInfo(self.CIFrontColor)
        
        # enlève distorsion du pixel
        pix_rec = self.cameraModel.rectifyPoint( (pixD[0],pixD[1]) )
        #print(pix_rec)
        
        # donne le vecteur unitaire entre le point et le centre de la caméra
        # (passant par un rayon laser non fixé), dans le ref de l'image
        p_3D = self.cameraModel.projectPixelTo3dRay((pix_rec[0],pix_rec[1]))
        
        #print(p_3D)
        
        # passage de ref image à ref camera pour vecteur unitaire
        p_3D = np.matmul(self.R_rpic2rcam,p_3D)
        
        #print(p_3D)
        
        # associe un rayon laser au vecteur unitaire grâce à l'information 
        # de profondeur (pix[2])
        Mp3d = np.array([p_3D[0],p_3D[1],p_3D[2]]).T
        
        p_real_3D = (pixD[2]/p_3D[0])*Mp3d

        # # to check if transformation is ok
        #print(p_real_3D)
        
        # take 3d point in camera ref, turn into picture ref
        # p_c_3D = np.matmul(self.R_rcam2rpic,p_real_3D)
        
        # # take 3d point in picture ref in 2D
        # p_check = self.cameraModel.project3dToPixel(p_c_3D)
        
        # print("p_check : ", p_check) 
        
        # # normally, norm(p_check - pixD) < 1e-10
        
        return p_real_3D
    
    def fromCamRef2caveRef(self,pr3d,current_depthMsg):
        
        p_cam = PointStamped()
        #p_cam.header.seq = self.depthPic.header.seq
        
        p_cam.header.seq = current_depthMsg.header.seq
        
        p_cam.header.frame_id = 'camera_front'
        p_cam.point.x = pr3d[0]
        p_cam.point.y = pr3d[1]
        p_cam.point.z = pr3d[2]

        #print(p_cam)


        
        stamp = rospy.Time.now()
        p_cam.header.stamp = stamp
        #trans = self.buffer.lookup_transform('map', 'camera_front', current_depthMsg.header.stamp)
        #self.listener.waitForTransform('/camera_front', '/map', rospy.Time.now(), rospy.Duration(1.0))
        p_cave = self.buffer.transform_full(p_cam,'map',  current_depthMsg.header.stamp, 'camera_front')
        
        #p_cave = self.listener.transformPoint('/map', p_cam)
        
        #print(p_cave)
        
        return p_cave
    

    def allPix2world(self):
        current_depthPic = self.depthPic.copy()
        current_depthMsg = self.depthMsg
        self.cur_PicID = self.PicID
        
        # print(current_depthPic)
        # print( current_depthPic.shape )
        
        bin_current_depthPic = cv2.threshold(current_depthPic,
                1e-5, 255,
                cv2.THRESH_BINARY)[1]
        # print("Got a pic!")
        # print(bin_current_depthPic)

        # barycenter only        
        allpts = []
        
        tst = np.uint8( bin_current_depthPic.copy() )
        
        try :
            contours, hierarchy = cv2.findContours(tst, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)#[1:]
        except :
            contours, hierarchy = cv2.findContours(tst, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1:]

        for i in range (len(contours)):
            cnt = contours[i]
            M = cv2.moments(cnt)
            cx = int(M['m10']/(M['m00']+1*10**-5))
            cy = int(M['m01']/(M['m00']+1*10**-5))
            depth = current_depthPic[cy,cx]
            bary_fis = self.fromCamRef2caveRef(self.fromPix2camRef( (cx,cy,depth) ), current_depthMsg )
            if bary_fis.point.z > 0.15:
                allpts.append(bary_fis)
                
        return(allpts)
    
    
    # def change
    
    def checkSameCracks(self,allpts, dist = 0.5, min_dist = 0.5):
        
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
                    # suppress the newest ponits if distance between pts really small
                    del(allpts[j])
                elif (np.linalg.norm(nPt)<min_dist):
                    del(allpts[j])
                j+= 1
                
        self.allCracksInCave += allpts     
        return(None)
        
    
    def displayCracks(self, display_console = True):
        """" display the current cracks """
        
        if (display_console):
            if (self.nb_cracks!= len(self.allCracksInCave)):
                print("---------------- %% Cracks found %% -----------------\n")
                
                for i in range(len(self.allCracksInCave)):
                    print("Crack number %i : \n"%(i+1))
                    print(self.allCracksInCave[i])
                    print("\n")
                    
                    #marker = self.allCracksInCave[i]
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
                
            
             # Marker(header=Header(stamp=rospy.Time.now(),
             #                                  frame_id=self.map_frame),
             #                          pose=particle.as_pose(),
             #                          type=0,
             #                          scale=Vector3(x=particle.w*2,y=particle.w*1,z=particle.w*5),
             #                          id=index,
             #                          color=ColorRGBA(r=1,a=1))
            self.markerPub.publish(MarkerArray(markers=liste_marker))
            
        return(None)
    
    def checkCracks(self):
        allpts = self.allPix2world()
        self.checkSameCracks(allpts,self.dist_center_cracks,self.dist_close_cracks)
        if (self.cur_PicID == self.PicID):
            # during all the treatments, no more pics were sent --> we can suppress the currently saved pic
            self.depthMsg = None
            self.depthPic = None
        return(None)
        
    
    
    def __init__(self,rate):
        
    # On cree les variables qui vont stocker les images 
        self.bridge = CvBridge()
        self.PicID = 0
        self.cur_PicID = 0
        self.CIFrontDepth = None
        self.CIFrontColor = None
        self.depthMsg = None
        self.depthPic = None
        self.allCracksInCave = []
        self.nb_cracks = len(self.allCracksInCave)
        
        self.dist_center_cracks = 0.5
        self.dist_close_cracks = 4.0
        #self.current_depthPic = None
        
        self.buffer = tf2.Buffer(rospy.Duration(60)) # prend 60s de tf 
        self.listener = tf2.TransformListener(self.buffer)
        
    # On cree les subscribers pour chaque image : front, left et right, en couleur et profondeur
    
        self.CIDtopic = rospy.Subscriber("/phantomx/camera_front/depth/camera_info", CamInfoMSG,self.callbackCIFrontDepth)
        self.CIPtopic = rospy.Subscriber("/phantomx/camera_front/color/camera_info", CamInfoMSG,self.callbackCIFrontColor)
        rospy.Subscriber("/phantomx/crack_image", ImageMSG,self.callbackDepthPic)

    # On cree les publisher de ces memes images sur des topics differents (avec un rate qu on peut choisir)
    
        self.rate = rospy.Rate(rate)
        
        self.markerPub = rospy.Publisher('/phantomx/crack_markers', MarkerArray, queue_size=10)
        
        # self.pubFrontDepth =  rospy.Publisher('/front_depth',Image,queue_size=10)
        # self.pubFrontColor =  rospy.Publisher('/front_color',Image,queue_size=10)
        # self.pubLeftDepth =  rospy.Publisher('/left_depth',Image,queue_size=10)
        # self.pubLeftColor =  rospy.Publisher('/left_color',Image,queue_size=10)
        # self.pubRightDepth =  rospy.Publisher('/right_depth',Image,queue_size=10)
        # self.pubRightColor =  rospy.Publisher('/right_color',Image,queue_size=10)
        
        time.sleep(3)
        
        self.cameraModel = image_geometry.PinholeCameraModel()
        # p_pix = PointStamped()
        # stamp = rospy.Time()
        
        self.R_rcam2rpic = np.array([[0.0,-1.0,0.0],[0.,0.,-1.0],[1.0,0.0,0.0]])
        self.R_rpic2rcam = np.linalg.inv(self.R_rcam2rpic)
        
        # on veut : pixel vers 3d
    
        #p_world.header.seq = self.camera_image.header.seq
        #p_world.header.stamp = stamp
        #p_world.header.frame_id = '/world'
        #pix = np.array([200,300.0,1.0]).T
        # p_pix.point.x = pix[0]
        # p_pix.point.y = pix[1]
        # p_pix.point.z = pix[2]imgFrontDepth
        
        
        # listener = tf.TransformListener()
        # listener.waitForTransform('/camera_left', '/base_link', stamp, rospy.Duration(1.0))
        # # transform de camera vers base
        # p_camera = listener.transformPoint('/camera_left', p_world)

        
        #print(self.R_rpic2rcam)
        
        
        
        while not rospy.is_shutdown():
            if (self.depthMsg is not None):
                self.checkCracks()
            self.displayCracks()
            self.rate.sleep()
        
        
        
        # pr3d = self.fromPix2camRef(pix)
        # p_world = self.fromCamRef2caveRef(pr3d)
    
    # cas de passage de world (3d) vers pixel (2d)
    
    # passage de réf camera à ref image
    #     # The navigation frame has X pointing forward, Y left and Z up, whereas the
    # # vision frame has X pointing right, Y down and Z forward; hence the need to
    # # reassign axes here.
        
        # (x,y,z) : ref picture
        # p_cam.point : ref camera
        

        
        
        
    #     x = -p_camera.point.y
    #     y = -p_camera.point.z
    #     z = p_camera.point.x
    
        # camera = image_geometry.PinholeCameraModel()
        # camera.fromCameraInfo(camera_info)
    #     p_image = camera.project3dToPixel((x, y, z))
    
            
            
        #     self.publisherGeneral()     
        # # On spin
        #     rospy.spin()
        





if __name__ == "__main__":
    rospy.init_node("crackMapping", anonymous=False, log_level=rospy.DEBUG)
    cm = CrackMap(1)
    # scan = rospy.Subscriber("/phantomx/", LaserScan, lidarRead)
    

