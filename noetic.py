
import rospy
import cv2

import numpy as np

from open3d_resources.map_registration import R3D

from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

from cv_bridge import CvBridge, CvBridgeError


def image_transport(img_msg):
    try:
        return CvBridge().imgmsg_to_cv2(img_msg, "passthrough")

    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))


class R3DROS(R3D):
    def __init__(self, vehicle_name, depth_trunc, image_width, image_height, fov):
        super().__init__(depth_trunc, image_width, image_height, fov)

        rospy.Subscriber("/airsim_node/"+vehicle_name+"/stereo/Scene", \
                         Image, self._callback_rgb)
        rospy.Subscriber("/airsim_node/"+vehicle_name+"/stereo/DepthPlanar", \
                         Image, self._callback_depth)
        rospy.Subscriber("/airsim_node/"+vehicle_name+"/odom_local_ned", \
                         Odometry, self._callback_odom)

        self.rgb = None
        self.depth = None

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
        # rospy.logwarn('noetic.py - R3DROS - _callback_odom')
        position_ = data.pose.pose.position
        orientation_ = data.pose.pose.orientation

        if self.rgb is not None and self.depth is not None and position_ is not None and orientation_ is not None and self.i <= 30:
            position = [position_.x, position_.y, position_.z]
            orientation = [orientation_.x, orientation_.y, orientation_.z, orientation_.w]
            # rospy.logwarn(f'position : {position} ---- orientation : {orientation}')
            pcd, cams = self.registration(self.rgb, self.depth.astype(np.float32), position, orientation)
            rospy.logwarn(f'PCD : {pcd} views {cams}')
