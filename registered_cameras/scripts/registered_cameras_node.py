#!/usr/bin/env python

import os
import rospy
import yaml
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

# rosbag record -O new_ocelli_data.bag /kinect2/sd/image_depth /kinect2/sd/image_ir /kinect2/hd/image_color /camera/image_raw /kinect2/sd/points /kinect2_link 

class RegisteredCameras:
    def __init__(self, calibration_yaml):
        self.bridge = CvBridge()
        with open(os.path.expanduser(calibration_yaml), 'r') as f:
            self.calibration_data = yaml.safe_load(f)
            
        """    
        # Getting the data from the yaml file
        K_depth = np.array(self.calibration_data['cam0']['intrinsics']).reshape((3, 3))
        D_depth = np.array(self.calibration_data['cam0']['distortion_coeffs'])
        
        K_rgb = np.array(self.calibration_data['cam1']['intrinsics']).reshape((3, 3))
        D_rgb = np.array(self.calibration_data['cam1']['distortion_coeffs'])
        
        T_depth_rgb = np.array(self.calibration_data['cam1']['T_cn_cnm1']).reshape((4, 4))
        """
            
        print(self.calibration_data)
        print("SPACE")
        
        # Intrinsic Parameters
        self.fx_rgb = self.calibration_data['cam0']['intrinsics'][0] # horizontal scale
        self.fy_rgb = self.calibration_data['cam0']['intrinsics'][1] # vertical scale
        self.cx_rgb = self.calibration_data['cam0']['intrinsics'][2] # horizontal position
        self.cy_rgb = self.calibration_data['cam0']['intrinsics'][3] # vertical position
        
        self.fx_depth = self.calibration_data['cam1']['intrinsics'][0]
        self.fy_depth = self.calibration_data['cam1']['intrinsics'][1]
        self.cx_depth = self.calibration_data['cam1']['intrinsics'][2]
        self.cy_depth = self.calibration_data['cam1']['intrinsics'][3]

        # Extrinsic Parameters
        self.T_cn_cnm1 = np.array(self.calibration_data['cam1']['T_cn_cnm1']).reshape((4, 4))    
        


        self.depth_sub = rospy.Subscriber("/kinect2/sd/image_depth", Image, self.depth_callback)
        self.rgb_sub = rospy.Subscriber("/camera/image_raw", Image, self.rgb_callback)
        
        self.ir_sub = rospy.Subscriber("/kinect2/sd/image_ir", Image, self.ir_callback)
        
        self.point_cloud_pub = rospy.Publisher("/registered_point_cloud", PointCloud2, queue_size=10)
        
        self.registered_pub = rospy.Publisher("/registered_images", Image, queue_size=1)
        
        self.synced_rgb_pub = rospy.Publisher("/synchronized_rgb", Image, queue_size=1)
        self.synced_ir_pub = rospy.Publisher("/synchronized_ir", Image, queue_size=1)

        self.depth_img = None
        self.rgb_img = None
        self.ir_img = None

    def depth_callback(self, msg):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        
        #self.publish_registered_point_cloud()
        
        
        #if self.rgb_img is not None:
            #self.create_dummy_point_cloud()
        #self.publish_registered_point_cloud()


    def rgb_callback(self, msg):

        self.rgb_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    

        #print('rgb callback')
        #if self.depth_img is not None:
        #    self.publish_registered_point_cloud()
        
    def ir_callback(self, msg):
        
        #alpha = 0.5
        #beta = 20
        
        alpha = 1
        beta = 0
        
        adjusted_image = cv2.convertScaleAbs(self.rgb_img, alpha=alpha, beta=beta)
        
        self.synced_ir_pub.publish(msg)
        if self.rgb_img is not None:
            self.synced_rgb_pub.publish(self.bridge.cv2_to_imgmsg(adjusted_image, encoding='bgr8'))
            self.create_dummy_point_cloud()
        
        
        
    def create_dummy_point_cloud(self):
        
        rgbImage = self.rgb_img
        depthImage = self.depth_img
        
        scale_factor = 0.5
        

        # Resize the depth image to match the dimensions of the RGB image
        #depthImage = cv2.resize(depthImage, None, fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_AREA)   
        
        height, width = depthImage.shape
        
        # Create a meshgrid for pixel coordinates
        x, y = np.meshgrid(np.arange(width), np.arange(height))
        
        # Calculate the real-world coordinates from the depth image
        x_depth = (x - self.cx_depth) * depthImage / self.fx_depth
        y_depth = (y - self.cy_depth) * depthImage / self.fy_depth
        z_depth = depthImage 
        
        # Stack depth coordinates
        depth_coords = np.column_stack((x_depth.flatten(), y_depth.flatten(), z_depth.flatten(), np.ones((height*width,))))
        
        # Transform depth coordinates to color coordinates
        color_coords = np.matmul(self.T_cn_cnm1, depth_coords.T).T  
        
        #x_translation = 80
        #y_translation = 80
        
        x_translation = -5
        y_translation = 0
        
        # Project color coordinates onto color image plane
        x_color = color_coords[:, 0] * self.fx_rgb / color_coords[:, 2] + self.cx_rgb + x_translation
        y_color = color_coords[:, 1] * self.fy_rgb / color_coords[:, 2] + self.cy_rgb + y_translation
        
        # Create a mask for valid color coordinates
        valid_mask = (x_color >= 0) & (x_color < rgbImage.shape[1]) & (y_color >= 0) & (y_color < rgbImage.shape[0])


        # Clip color coordinates to be within the color image
        x_color = np.clip(x_color, 0, rgbImage.shape[1] - 1).astype(np.int)
        y_color = np.clip(y_color, 0, rgbImage.shape[0] - 1).astype(np.int)
        
        # Apply the mask to the depth image
        masked_depth = np.zeros_like(depthImage)
        masked_depth[valid_mask.reshape(height, width)] = depthImage[valid_mask.reshape(height, width)]
        
        """FOR THE REGISTERED IMAGE"""
        
        # Create a meshgrid of color coordinates
        color_coordsR = np.column_stack((x_color.flatten(), y_color.flatten())).reshape(height, width, 2)
                
        # Remap the color image to the depth image resolution using bilinear interpolation
        registered_color = cv2.remap(rgbImage, color_coordsR.astype(np.float32), None, cv2.INTER_LINEAR)
    
        
        # Convert the depth image to 8-bit
        depth_img_8bit = cv2.normalize(depthImage, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
    
        # Stack the registered color image and depth image horizontally
        registered_img = np.hstack((registered_color, cv2.cvtColor(depth_img_8bit, cv2.COLOR_GRAY2BGR)))
        
        
        
        registered_msg = self.bridge.cv2_to_imgmsg(registered_color, encoding='bgr8')
        
        self.registered_pub.publish(registered_msg)
        
        """END OF REGISTERED IMAGE"""

        # Extract color values from the color image
        colors = rgbImage[y_color, x_color].reshape(height, width, 3)
        
        
        """
        points = []
        
        for i in range(height):
            for j in range(width):
                
                rgbHeight, rgbWidth, _ = self.rgb_img.shape
                if i < 120 and j < 160:
                    pixel = self.rgb_img[i, j]
                    r = pixel[2]
                    g = pixel[1]
                    b = pixel[0]
                else:
                    r = 255
                    g = 0
                    b = 0 
                
                x = np.float32(x_depth[i, j]) # Red Arrow
                y = np.float32(y_depth[i, j]) # Green Arrow
                z = np.float32(z_depth[i, j]) # Blue Arrow

                rgb = np.uint32(r) << 16 | np.uint32(g) << 8 | np.uint32(b)
                points.append([x, y, z, rgb])
        """
        
        # Convert the color values to UINT32 format
        colors_uint32 = (colors[:, :, 2].astype(np.uint32) << 16) | (colors[:, :, 1].astype(np.uint32) << 8) | colors[:, :, 0].astype(np.uint32)
        colors_uint32 = colors_uint32.flatten()
                
        # Create the colored point cloud
        points = np.column_stack((x_depth.flatten(), y_depth.flatten(), z_depth.flatten(), colors_uint32))
        
        
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'base_link'
        
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]
    
        cloud_msg = pc2.create_cloud(header, fields, points)
        cloud_msg.is_dense = False
        cloud_msg.height = 1
        cloud_msg.width = len(points)
        
        point_cloud_msg = cloud_msg
    
        self.point_cloud_pub.publish(point_cloud_msg)
        

    def publish_registered_point_cloud(self):
        if self.rgb_image is None or self.depth_image is None:
            return
    
        # Intrinsic Parameters
        fx_rgb, fy_rgb, cx_rgb, cy_rgb = ['cam0']['intrinsics']
        """
        # Get intrinsic parameters from calibration data
        K_rgb = np.array(self.calibration_data['cam0']['intrinsics']).reshape((3, 3))
        K_depth = np.array(self.calibration_data['cam1']['intrinsics']).reshape((3, 3))
    
        # Get the depth scale
        depth_scale = 0.001  # depth scale in meters (adjust if necessary)
        
        # Compute the extrinsics
        T_rgb_depth = np.linalg.inv(np.array(self.calibration_data['cam1']['T_cn_cnm1']).reshape((4, 4)))
        
        # Create a colored point cloud message
        point_cloud_msg = pc2.create_cloud_xyzrgb32(
            rospy.Time.now(),
            fields=[
                pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1),
                pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1),
                pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1),
                pc2.PointField('rgb', 12, pc2.PointField.FLOAT32, 1)
            ],
            data=np.hstack([registered_points, registered_colors]).tobytes()
        )
    
        # Publish the registered point cloud
        self.point_cloud_pub.publish(point_cloud_msg)
        """
        
      

    def create_point_cloud(self, depth_img, rgb_img, K, T):
        points = []

        """
        for v in range(depth_img.shape[0]):
            for u in range(depth_img.shape[1]):
                z = depth_img[v, u] * 0.001  # Convert depth to meters
                if z > 0:
                    x = (u - K[0, 2]) * z / K[0, 0]
                    y = (v - K[1, 2]) * z / K[1, 1]

                    pt_depth = np.array([x, y, z, 1]).reshape((4, 1))
                    pt_rgb = np.dot(T, pt_depth)

                    r, g, b = rgb_img[v, u]
                    points.append([pt_rgb[0, 0], pt_rgb[1, 0], pt_rgb[2, 0], r, g, b])

        """
        
        dummy_points = []

        # Number of dummy points you want to create
        num_points = 100
        
        for i in range(num_points):
            x = float(i) / 100.0
            y = float(i) / 200.0
            r = int((i * 255) / num_points)
            g = int(((num_points - i) * 255) / num_points)
            b = 128
            dummy_points.append((x, y, r, g, b))
        
        return pc2.create_cloud_xyzrgb(
            rospy.Time.now(),
            fields=[
                pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1),
                pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1),
                #pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1),
                pc2.PointField('r', 8, pc2.PointField.UINT8, 1),
                pc2.PointField('g', 9, pc2.PointField.UINT8, 1),
                pc2.PointField('b', 10, pc2.PointField.UINT8, 1),
            ],
            data=dummy_points
        )


if __name__ == '__main__':
    rospy.init_node('registered_cameras_node')
    calibration_yaml = rospy.get_param('~calibration_yaml')
    rc = RegisteredCameras(calibration_yaml)
    rospy.spin()
