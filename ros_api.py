
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

import ros_numpy
import numpy as np
import tf.transformations as t
import rospy
import copy
import image_geometry
import cv2
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Transform, TransformStamped, Vector3
import numpy as np
import numpy.matlib as npm


def image_transport(img_msg):
    try:
        return CvBridge().imgmsg_to_cv2(img_msg, "passthrough")

    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))


class R3DROS(R3D):
    @staticmethod
    def pose_to_pq(pose):
        """ convert a ROS PoseS message into position/quaternion np arrays
        Args:
            pose (geometry_msgs/Pose): ROS geometric message to be converted
        Returns:
            p (np.array): position array of [x, y, z]
            q (np.array): quaternion array of [x, y, z, w]
        source codes from https://answers.ros.org/question/332407/transformstamped-to-transformation-matrix-python/
        """
        
        p = np.array([pose.position.x, pose.position.y, pose.position.z])
        q = np.array([-pose.orientation.x, -pose.orientation.y,
                    -pose.orientation.z, pose.orientation.w])
        return p, q
        
    
    @staticmethod
    def transform_to_pq(transform):
        """ convert a ROS Transform message into position/quaternion np arrays
        Args:
            transform (geometry_msgs/Transform): ROS geometric message to be converted
        Returns:
            p (np.array): position array of [x, y, z]
            q (np.array): quaternion array of [x, y, z, w]
        source codes from https://answers.ros.org/question/332407/transformstamped-to-transformation-matrix-python/
        """
        
        
        # transform.rotation.z = (np.pi/2) - transform.rotation.z if transform.rotation.z < np.pi/2 else transform.rotation.z
        
        
        p = np.array([transform.translation.x, transform.translation.y, transform.translation.z])
        q = np.array([transform.rotation.x, transform.rotation.y,
                    transform.rotation.z, transform.rotation.w])
        
        # transform.rotation.z = transform.rotation.z
        # rotx = np.array([[1, 0, 0, 0],
        #                  [0, np.cos(-transform.rotation.y), -np.sin(-transform.rotation.y), 0],
        #                  [0, np.sin(-transform.rotation.y), np.cos(-transform.rotation.y), 0],
        #                  [0, 0, 0, 1]])
        
        # roty = np.array([[np.cos(-transform.rotation.z), 0, np.sin(-transform.rotation.z), 0],
        #                  [0, 1, 0, 0],
        #                  [-np.sin(-transform.rotation.z), 0, np.cos(-transform.rotation.z), 0],
        #                  [0, 0, 0, 1]])
        
        # rotz = np.array([[np.cos(-transform.rotation.x), -np.sin(-transform.rotation.x), 0, 0],
        #                  [np.sin(-transform.rotation.x), np.cos(-transform.rotation.x), 0, 0],
        #                  [0, 0, 1, 0],
        #                  [0, 0, 0, 1]])
        
        
        
        # x, y, z = t.euler_from_quaternion(q)
        # me = t.euler_matrix(x, y, z) 
        # q = t.quaternion_from_matrix(me)
        # rospy.logwarn(f"{me}")
        
        
        
        return p, q

    
    @staticmethod
    def o3dpc_to_rospc(o3dpc, frame_id=None, stamp=None):
        """ convert open3d point cloud to ros point cloud
        Args:
            o3dpc (open3d.geometry.PointCloud): open3d point cloud
            frame_id (string): frame id of ros point cloud header
            stamp (rospy.Time): time stamp of ros point cloud header
        Returns:
            rospc (sensor.msg.PointCloud2): ros point cloud message
        """

        cloud_npy = np.asarray(copy.deepcopy(o3dpc.points))
        is_color = o3dpc.colors
            

        n_points = len(cloud_npy[:, 0])
        if is_color:
            data = np.zeros(n_points, dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('rgb', np.uint32)
            ])
        else:
            data = np.zeros(n_points, dtype=[
                ('x', np.float32),
                ('y', np.float32),
                ('z', np.float32)
                ])
        data['x'] = cloud_npy[:, 0]
        data['y'] = cloud_npy[:, 1]
        data['z'] = cloud_npy[:, 2]
        
        if is_color:
            rgb_npy = np.asarray(copy.deepcopy(o3dpc.colors))
            rgb_npy = np.floor(rgb_npy*255) # nx3 matrix
            rgb_npy = rgb_npy[:, 0] * 2**16 + rgb_npy[:, 1] * 2**8 + rgb_npy[:, 2]  
            rgb_npy = rgb_npy.astype(np.uint32)
            data['rgb'] = rgb_npy

        rospc = ros_numpy.msgify(PointCloud2, data)
        if frame_id is not None:
            rospc.header.frame_id = frame_id

        if stamp is None:
            rospc.header.stamp = rospy.Time.now()
        else:
            rospc.header.stamp = stamp
        rospc.height = 1
        rospc.width = n_points
        rospc.fields = []
        rospc.fields.append(PointField(
                                name="x",
                                offset=0,
                                datatype=PointField.FLOAT32, count=1))
        rospc.fields.append(PointField(
                                name="y",
                                offset=4,
                                datatype=PointField.FLOAT32, count=1))
        rospc.fields.append(PointField(
                                name="z",
                                offset=8,
                                datatype=PointField.FLOAT32, count=1))    

        if is_color:
            rospc.fields.append(PointField(
                            name="rgb",
                            offset=12,
                            datatype=PointField.UINT32, count=1))    
            rospc.point_step = 16
        else:
            rospc.point_step = 12
        
        rospc.is_bigendian = False
        rospc.row_step = rospc.point_step * n_points
        rospc.is_dense = True
        return rospc
    
    def rospc_to_o3dpc(self, rospc, remove_nans=False):
        """ covert ros point cloud to open3d point cloud
        Args: 
            rospc (sensor.msg.PointCloud2): ros point cloud message
            remove_nans (bool): if true, ignore the NaN points
        Returns: 
            o3dpc (open3d.geometry.PointCloud): open3d point cloud
        """
        field_names = [field.name for field in rospc.fields]
        is_rgb = 'rgb' in field_names
        cloud_array = ros_numpy.point_cloud2.pointcloud2_to_array(rospc).ravel()
        if remove_nans:
            mask = np.isfinite(cloud_array['x']) & np.isfinite(cloud_array['y']) & np.isfinite(cloud_array['z'])
            cloud_array = cloud_array[mask]
        if is_rgb:
            cloud_npy = np.zeros(cloud_array.shape + (4,), dtype=np.float64)
        else: 
            cloud_npy = np.zeros(cloud_array.shape + (3,), dtype=np.float64)

        cloud_npy[...,0] = cloud_array['x']
        cloud_npy[...,1] = cloud_array['y']
        cloud_npy[...,2] = cloud_array['z']
        o3dpc = o3d.geometry.PointCloud()

        if len(np.shape(cloud_npy)) == 3:
            cloud_npy = np.reshape(cloud_npy[:, :, :3], [-1, 3], 'F')
        o3dpc.points = o3d.utility.Vector3dVector(cloud_npy[:, :3])

        if is_rgb:
            rgb_npy = cloud_array['rgb']
            rgb_npy.dtype = np.uint32
            r = np.asarray((rgb_npy >> 16) & 255, dtype=np.uint8)
            g = np.asarray((rgb_npy >> 8) & 255, dtype=np.uint8)
            b = np.asarray(rgb_npy & 255, dtype=np.uint8)
            rgb_npy = np.asarray([r, g, b])
            rgb_npy = rgb_npy.astype(np.float64)/255
            rgb_npy = np.swapaxes(rgb_npy, 0, 1)
            o3dpc.colors = o3d.utility.Vector3dVector(rgb_npy)
        return o3dpc
    
    def __init__(self, vehicle_name, depth_trunc, image_width, image_height, fov):
        super().__init__(depth_trunc, image_width, image_height, fov)
        # rospy.Subscriber("/airsim_node/"+vehicle_name+"/stereo/Scene", \
        #                  Image, self._callback_rgb, queue_size=1)
        # rospy.Subscriber("/airsim_node/"+vehicle_name+"/stereo/DepthPlanar", \
        #                  Image, self._callback_depth, queue_size=1)
        # rospy.Subscriber("/airsim_node/"+vehicle_name+"/odom_local_ned", \
        #                  Odometry, self._callback_odom)    @staticmethod

        
        rgb_sub = message_filters.Subscriber("/airsim_node/"+vehicle_name+"/stereo/Scene", Image)
        depth_sub = message_filters.Subscriber("/airsim_node/"+vehicle_name+"/stereo/DepthPlanar", Image)
        odom_sub = message_filters.Subscriber("/airsim_node/"+vehicle_name+"/odom_local_ned", Odometry)
        tf_sub = message_filters.Subscriber("/airsim_node/"+vehicle_name+"/stereo/tf", TransformStamped)
        
        # rospy.Subscriber("/rtabmap/cloud_map", PointCloud2, self._callback_pcd, queue_size=10)
        
        ts = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub, tf_sub], 10, 0.1, allow_headerless=True)
        ts.registerCallback(self._callback)
        
        self.rgb = None
        self.depth = None

        self.past_position = [0, 0, 0]
        self.past_orientation = [0, 0, 0, 1]
        self.position = [0, 0, 0]
        self.orientation = [0, 0, 0, 1]
        
        self.ros_pcd = None
        self.o3d_pcd = None
        self.pub_pcd = rospy.Publisher("/airsim_node/"+vehicle_name+"/stereo/points", PointCloud2, queue_size=10)
        
        
        self.odom = None
        self.tf = TransformStamped()
        self.pastF = np.zeros(1)
        
        matrix = np.array(t.euler_matrix(0, 0, np.pi/2))    # @ np.array(t.euler_matrix(-np.pi/2, 0, 0))
        self.air_o3d = t.quaternion_from_matrix(matrix)
        
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
        # position_ = data.pose.pose.position
        # orientation_ = data.pose.pose.orientation
        # self.past_position = self.position
        # self.past_orientation = self.orientation
        
        # self.position = [position_.x, position_.y, position_.z]
        # self.orientation = [orientation_.x, orientation_.y, orientation_.z, orientation_.w]
        self.odom = data
                
    def _callback_tf(self, data):
        # return
        # rospy.logwarn('noetic.py - R3DROS - _callback_odom')
        # position_ = data.transform.translation
        # orientation_ = data.transform.rotation
        # self.past_position = self.position
        # self.past_orientation = self.orientation
        # self.position = [position_.x, position_.y, position_.z]
        # self.orientation = [orientation_.x, orientation_.y, orientation_.z, orientation_.w]
        self.tf.transform.translation.x = data.transform.translation.x
        self.tf.transform.translation.y = data.transform.translation.y
        self.tf.transform.translation.z = data.transform.translation.z
        
        self.tf.transform.rotation.x = data.transform.rotation.x
        self.tf.transform.rotation.y = data.transform.rotation.y
        self.tf.transform.rotation.z = data.transform.rotation.z
        self.tf.transform.rotation.w = data.transform.rotation.w
        
        
        
        
    def _callback(self, rgb, depth, tf):
        self._callback_rgb(rgb)
        self._callback_depth(depth)
        # self._callback_odom(odom)
        self._callback_tf(tf)
        
        # o3d_pcd, _ = self.create_point_cloud(self.rgb[...,::-1].copy(), self.depth.astype(np.float32), self.position, self.orientation)
        # self.ros_pcd = self.o3dpc_to_rospc(o3d_pcd)
        # # rospy.logwarn(f"{self.ros_pcd}")
        # self.pub_pcd.publish(self.ros_pcd)
        # if dist(self.position, self.past_position) > 0 and self.rgb is not None and self.depth is not None :
            
            # result, ev= self.registration(self.rgb[...,::-1].copy(), self.depth.astype(np.float32), self.position, self.orientation)
            # rospy.logwarn(f" --- evaluation : {ev}")
        
    

        # if self.rgb is not None and self.depth is not None and position_ is not None and orientation_ is not None and self.i <= 30:
        #     position = [position_.x, position_.y, position_.z]
        #     orientation = [orientation_.x, orientation_.y, orientation_.z, orientation_.w]
        #     # rospy.logwarn(f'rgb : {np.asarray(self.rgb).shape} ---- orientation : {orientation}')
            
        #     self.pcd, self.view = self.registration(self.rgb[...,::-1].copy(), self.depth.astype(np.float32), position, orientation)
        #     # rospy.logwarn(f' PCD : {pcd}')
        #     self.i+=1
        
        F, p, q = self.msg_to_se3(tf)
        x, y, z = p
        qx, qy, qz, qw = q
    
        rotx = np.array([[1, 0, 0],
                         [0, np.cos(np.pi/2), -np.sin(np.pi/2)],
                         [0, np.sin(np.pi/2), np.cos(np.pi/2)]])
        
        roty = np.array([[np.cos(-np.pi/2), 0, np.sin(-np.pi/2)],
                         [0, 1, 0],
                         [-np.sin(-np.pi/2), 0, np.cos(-np.pi/2)]])
        
        R = rotx @ roty
        
        # rotz = np.array([[np.cos(-np.pi/2), -np.sin(-np.pi/2), 0, 0],
        #                  [np.sin(-np.pi/2), np.cos(-np.pi/2), 0, 0],
        #                  [0, 0, 1, 0],
        #                  [0, 0, 0, 1]])
        
        # roty = np.array([[np.cos(-np.pi/2), 0, np.sin(-np.pi/2), 0],
        #                  [0, 1, 0, 0],
        #                  [-np.sin(-np.pi/2), 0, np.cos(-np.pi/2), 0],
        #                  [0, 0, 0, 1]])
        
        # H_a_o = rotz @ rotz @ rotz @ roty
        # H_o_d = H_a_o @ F
        
        R_a_x = o3d.geometry.get_rotation_matrix_from_xyz((-np.pi/2, 0, 0))
        R_a_y = o3d.geometry.get_rotation_matrix_from_xyz((0, np.pi/2, 0))
        R_a_z = o3d.geometry.get_rotation_matrix_from_xyz((0, 0, -np.pi/2))
        
        RR = o3d.geometry.get_rotation_matrix_from_xyz((np.pi/2, np.pi/2, np.pi)) 
        
        R_d = o3d.geometry.get_rotation_matrix_from_quaternion((-qx, -qy, -qz, qw))
        
        o3d_pcd = self.rgb_point_cloud(self.rgb[...,::-1].copy(), self.depth.astype(np.float32))
        # o3d_pcd.rotate(R_a_x)
        # o3d_pcd.rotate(R_a_x)
        # o3d_pcd.rotate(R_a_x)
        # o3d_pcd.rotate(R_a_y)
        # o3d_pcd.rotate(R_d)
        
        
        o3d_pcd.translate((-x, -y, -z))
        o3d_pcd.rotate(np.linalg.inv(R_d))
        o3d_pcd.rotate(R_a_x)
        o3d_pcd.rotate(R_a_y)
        
        
        
            
        # self.pastF = F
        # np.linalg.inv(F)
        # self.pcd.transform(F)
        self.pcd += o3d_pcd
        self.geos = [self.pcd]
        
        
        
    def _callback_pcd(self, data):
        self.o3d_pcd = self.rospc_to_o3dpc(data)
        
    
    ##Functions
    # def map_registration(self):
    #     rospy.logwarn(f"dist : {dist(self.position, self.past_position)} ")
    #     if dist(self.position, self.past_position) > 0:
    #         self.pcd, self.view = self.registration(self.rgb[...,::-1].copy(), self.depth.astype(np.float32), self.position, self.orientation)
    #         rospy.logwarn(f"pcd : {self.pcd} ---- view : {self.view }")
    #         o3d.visualization.draw_geometries([self.pcd] + self.view)
    
    def pose_stamped_to_pq(self, pose_stamped):
        """ convert a ROS PoseStamped message into position/quaternion np arrays
        Args:
            pose_stamped (geometry_msgs/PoseStamped): ROS geometric message to be converted
        Returns:
            p (np.array): position array of [x, y, z]
            q (np.array): quaternion array of [x, y, z, w]
        source codes from https://answers.ros.org/question/332407/transformstamped-to-transformation-matrix-python/
        """
        return self.pose_to_pq(pose_stamped.pose)
    
    def transform_stamped_to_pq(self, transform_stamped):
        """ convert a ROS TransformStamped message into position/quaternion np arrays
        Args:
            transform_stamped (geometry_msgs/TransformStamped): ROS geometric message to be converted
        Returns:
            p (np.array): position array of [x, y, z]
            q (np.array): quaternion array of [x, y, z, w]
        source codes from https://answers.ros.org/question/332407/transformstamped-to-transformation-matrix-python/
        """
        return self.transform_to_pq(transform_stamped.transform)
    
    def msg_to_se3(self, msg):
        """ convert geometric ROS messages to SE(3)
        Args:
            msg (geometry_msgs/Pose, geometry_msgs/PoseStamped, 
            geometry_msgs/Transform, geometry_msgs/TransformStamped): ROS geometric messages to be converted
        Returns:
            se3 (np.array): a 4x4 SE(3) matrix as a numpy array
        source codes from https://answers.ros.org/question/332407/transformstamped-to-transformation-matrix-python/
        """
        if isinstance(msg, Pose):
            p, q = self.pose_to_pq(msg)
        elif isinstance(msg, PoseStamped):
            p, q = self.pose_stamped_to_pq(msg)
        elif isinstance(msg, Transform):
            p, q = self.transform_to_pq(msg)
        elif isinstance(msg, TransformStamped):
            p, q = self.transform_stamped_to_pq(msg)
        else:
            raise TypeError("Invalid type for conversion to SE(3)")
        # rospy.logwarn(f"{t.euler_from_quaternion(q)}")
        norm = np.linalg.norm(q)
        if np.abs(norm - 1.0) > 1e-3:
            raise ValueError(
                "Received un-normalized quaternion (q = {0:s} ||q|| = {1:3.6f})".format(
                    str(q), np.linalg.norm(q)))
        elif np.abs(norm - 1.0) > 1e-6:
            q = q / norm
        
        #conjugate = t.quaternion_conjugate(q)
        # inv = t.quaternion_inverse(q)
        se3 = t.quaternion_matrix(q) 
        
        # tt = t.quaternion_multiply(self.air_o3d, q)
        # se3 = t.quaternion_matrix(tt)
        # rospy.logwarn(f"{t.euler_from_quaternion(q)} {se3}")
        se3[0:3, -1] = p
        return se3, p, q
    
    def do_transform_point(self, o3dpc, transform_stamped):
        """ transform a input cloud with respect to the specific frame
            open3d version of tf2_geometry_msgs.do_transform_point
        Args: 
            o3dpc (open3d.geometry.PointCloud): open3d point cloud
            transform_stamped (geometry_msgs.msgs.TransformStamped): transform to be applied 
        Returns:
            o3dpc (open3d.geometry.PointCloud): transformed open3d point cloud
        """
        H = self.msg_to_se3(transform_stamped)
        o3dpc = copy.deepcopy(o3dpc)
        o3dpc.transform(np.linalg.inv(H))
        return o3dpc
            
            
