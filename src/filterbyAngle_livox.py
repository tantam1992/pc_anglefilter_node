#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from sensor_msgs import point_cloud2
import pcl_ros
import math

def filter_callback(msg):

    # Define the desired range of horizontal angle (in radians)
    # min_angle = -math.pi/4  # Example: minimum angle of -45 degrees
    # max_angle = math.pi/4   # Example: maximum angle of 45 degrees
    min_angle = -math.pi/2 
    max_angle = math.pi/2
    isremain = True   # if isremain, points within the range are remained, otherwise, removed
           
    out_pc = [] 
    
    # Example: Convert PointCloud2 message to PCL point cloud
    for point in point_cloud2.read_points(msg, field_names=["x","y","z"]):
        
        
        # Example: Filter points based on horizontal angle
    
        # Calculate the horizontal angle of the point
        angle = math.atan2(point[1], point[0])
        
       
        # Filter out points outside the desired range
        if isremain:
            if min_angle <= angle <= max_angle:
                out_pc.append(point)
        else:
            if not (min_angle <= angle <= max_angle):
                out_pc.append(point)
     
    
    # Example: Convert PCL point cloud back to PointCloud2 message
    processed_cloud_msg = point_cloud2.create_cloud_xyz32(msg.header, out_pc)
	
    # Publish the processed point cloud to another topic
    pub.publish(processed_cloud_msg)
    
if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('pc_anglefilter_node')

    # Create a subscriber to listen to the input PointCloud2 topic
    sub = rospy.Subscriber('/livox/lidar', PointCloud2, filter_callback)

    # Create a publisher to publish the processed PointCloud2 message to another topic
    pub = rospy.Publisher('/livox/lidar_filtered', PointCloud2, queue_size=10)

    # Spin the ROS node
    rospy.spin()
