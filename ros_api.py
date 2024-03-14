
import rospy
import message_filters

import cv2

import open3d as o3d

import numpy as np

from math import dist

from open3d_resources.map_registration import R3D

from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

from cv_bridge import CvBridge, CvBridgeError


def image_transport(img_msg):
    try:
        return CvBridge().imgmsg_to_cv2(img_msg, "passthrough")

    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))


class R3DROS(R3D):
    def __init__(self, vehicle_name, depth_trunc, image_width, image_height, fov):
        super().__init__(depth_trunc, image_width, image_height, fov)
        # rospy.Subscriber("/airsim_node/"+vehicle_name+"/stereo/Scene", \
        #                  Image, self._callback_rgb, queue_size=1)
        # rospy.Subscriber("/airsim_node/"+vehicle_name+"/stereo/DepthPlanar", \
        #                  Image, self._callback_depth, queue_size=1)
        # rospy.Subscriber("/airsim_node/"+vehicle_name+"/odom_local_ned", \
        #                  Odometry, self._callback_odom)
        
        rgb_sub = message_filters.Subscriber("/airsim_node/"+vehicle_name+"/stereo/Scene", Image)
        depth_sub = message_filters.Subscriber("/airsim_node/"+vehicle_name+"/stereo/DepthPlanar", Image)
        odom_sub = message_filters.Subscriber("/airsim_node/"+vehicle_name+"/odom_local_ned", Odometry)
        tf_sub = message_filters.Subscriber("/airsim_node/"+vehicle_name+"/stereo/tf", TransformStamped)
        
        ts = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub, odom_sub], 10, 0.1, allow_headerless=True)
        ts.registerCallback(self._callback)
        
        self.rgb = None
        self.depth = None

        self.past_position = [0, 0, 0]
        self.past_orientation = [0, 0, 0, 1]
        self.position = [0, 0, 0]
        self.orientation = [0, 0, 0, 1]
        
        
        
        # rospy.logwarn('noetic.py - R3DROS - __init__')

    # Callbacks
    def callback_image(func):
        def callback(self, *args, **kwargs):
            data, img_type = func(self, *args, **kwargs)
            if data:
                cv_img = image_transport(data)
                    # resized_img = cv2.resize(cv_img.copy(), (100, 100), interpolation = cv2.INTER_AREA)
                self.__setattr__(img_type, cv_img)
            else:
                info = f"Error in {img_type} cam!"
                self.__pub_info.publish(info)

        return callback
    
    @callback_image
    def _callback_rgb(self, data):
        # rospy.logwarn('noetic.py - R3DROS - _callback_rgb')
        return data, "rgb"
    
    @callback_image
    def _callback_depth(self, data):
        # rospy.logwarn('noetic.py - R3DROS - _callback_depth')
        return data, "depth"
    
    def _callback_odom(self, data):
        # return
        # rospy.logwarn('noetic.py - R3DROS - _callback_odom')
        position_ = data.pose.pose.position
        orientation_ = data.pose.pose.orientation
        self.past_position = self.position
        self.past_orientation = self.orientation
        
        dx = position_.x - self.past_position[0]
        dy = position_.y - self.past_position[1]
        dz = position_.z - self.past_position[2]
        
        self.position = [position_.x - dx/2, position_.y - dy/2, position_.z - dz/2]
        self.orientation = [orientation_.x, orientation_.y, orientation_.z, orientation_.w]
        
        # if dist(self.position, self.past_position) > 0 and self.rgb is not None and self.depth is not None :
        # # if self.rgb is not None and self.depth is not None :
            
        #     F= self.registration(self.rgb[...,::-1].copy(), self.depth.astype(np.float32), self.position, self.orientation)
        #     rospy.logwarn(f"position : {self.position} --- PCD center : {self.pcd.get_center()}")
            # rospy.logwarn(f"")
            # geos = [self.pcd]
            # geos.extend(self.cams)
            # o3d.visualization.draw_geometries(geos)
        # rospy.logwarn(f"dif : {dist(self.position, self.past_position)}")
        # rospy.logwarn(f"position : {self.position} ---- past position : {self.past_position}")
        
    def _callback_tf(self, data):
        # return
        # rospy.logwarn('noetic.py - R3DROS - _callback_odom')
        position_ = data.transform.translation
        orientation_ = data.transform.rotation
        self.past_position = self.position
        self.past_orientation = self.orientation
        self.position = [position_.x, position_.y, position_.z]
        self.orientation = [orientation_.x, orientation_.y, orientation_.z, orientation_.w]
        
        
        
    def _callback(self, rgb, depth, odom):
        self._callback_rgb(rgb)
        self._callback_depth(depth)
        self._callback_odom(odom)
        # self._callback_tf(tf)
        
        if dist(self.position, self.past_position) > 0 and self.rgb is not None and self.depth is not None :
        # if self.rgb is not None and self.depth is not None :
            
            result, ev= self.registration(self.rgb[...,::-1].copy(), self.depth.astype(np.float32), self.position, self.orientation)
            rospy.logwarn(f"result : {result} --- evaluation : {ev}")
        
    

        # if self.rgb is not None and self.depth is not None and position_ is not None and orientation_ is not None and self.i <= 30:
        #     position = [position_.x, position_.y, position_.z]
        #     orientation = [orientation_.x, orientation_.y, orientation_.z, orientation_.w]
        #     # rospy.logwarn(f'rgb : {np.asarray(self.rgb).shape} ---- orientation : {orientation}')
            
        #     self.pcd, self.view = self.registration(self.rgb[...,::-1].copy(), self.depth.astype(np.float32), position, orientation)
        #     # rospy.logwarn(f' PCD : {pcd}')
        #     self.i+=1
        
     
    ##Functions
    # def map_registration(self):
    #     rospy.logwarn(f"dist : {dist(self.position, self.past_position)} ")
    #     if dist(self.position, self.past_position) > 0:
    #         self.pcd, self.view = self.registration(self.rgb[...,::-1].copy(), self.depth.astype(np.float32), self.position, self.orientation)
    #         rospy.logwarn(f"pcd : {self.pcd} ---- view : {self.view }")
    #         o3d.visualization.draw_geometries([self.pcd] + self.view)
        
            
