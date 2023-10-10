# Basic ROS 2 program to subscribe to real-time streaming 
# video from your built-in webcam
# This code is modified by Berk Calli from the following author.
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
  
# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
import numpy as np
import pcl
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from queue import *

class PCSubscriber(Node):
  """
  Create an PCSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('point_cloud_subscriber')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.

    self.subscription = self.create_subscription(
      PointCloud2, 
      '/realsense/points', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning

    # Create the publisher. This publisher will publish an Image
    # to the video_frames topic. The queue size is 10 messages.
    self.publisher_ = self.create_publisher(PointCloud2, 'fit_PC', 10)
   
   
  def listener_callback(self, msg):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving point cloud')
    
    #1-d point-cloud message of float-32s. Composed of pointfields with x, y, z, and rgb. A point is comprised of bytes where
    #x: 0-3
    #y: 4-7
    #z: 8-11
    #rgb: 15-19
    pcmsg = msg;
    
    #converts binary blob to numpy array of tuple (x, y, z, rgb);
    #pcnp = pc2.read_points(msg);
    
    #converts binary blob to numpy array of tuple (x, y, z, rgb);
    pcls = pc2.read_points_list(msg, skip_nans=True, field_names=("x", "y", "z"));
    
    #converts list to PointCloud
    pc = pcl.PointCloud();
    pc.from_list(pcls);
    
    # from https://github.com/strawlab/python-pcl/blob/master/examples/official/Filtering/VoxelGrid_160.py
    #sor = pc.make_voxel_grid_filter()
    #sor.set_leaf_size(0.01, 0.01, 0.01)
    #pc_filtered = sor.filter()
    
    # taken from https://github.com/strawlab/python-pcl/blob/master/examples/official/Segmentation/Plane_model_segmentation.py
    # segments out plane
    seg = pc.make_segmenter_normals(ksearch=50)
    seg.set_optimize_coefficients(True)
    seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_distance_threshold(0.005)
    seg.set_normal_distance_weight(0.01)
    seg.set_max_iterations(100)
    indices, coefficients = seg.segment()
    
    non_plane = pc.extract(indices, True)
    
    # clusters the shapes found
    # from https://github.com/strawlab/python-pcl/blob/master/examples/official/Segmentation/cluster_extraction.py
    tree = non_plane.make_kdtree()
    ec = non_plane.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.02)
    ec.set_MinClusterSize(100)
    ec.set_MaxClusterSize(25000)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()
    
    cloud_cluster = pcl.PointCloud()
    self.get_logger().info('Before cluster loop')
    for cluster in cluster_indices:
    	
    	
    	#cloud_cluster = pcl.PointCloud()
    	
   	#print('indices = ' + str(len(indices)))
    	points = np.zeros((len(indices), 3), dtype=np.float32)
    	self.get_logger().info('Before enumerating loop')
    		
    	for i, index in enumerate(cluster):
    		points[i][0] = non_plane[index][0]
    		points[i][1] = non_plane[index][1]
    		points[i][2] = non_plane[index][2]
    		#self.get_logger().info('Still enumerating loop')
    			
    	cloud_cluster.from_array(points)
    	self.get_logger().info('Copying complete')
    	
    	# identifies cylinder
    	seg = cloud_cluster.make_segmenter_normals(ksearch=50)
    	self.get_logger().info('Segmenter made')
    	seg.set_optimize_coefficients(True)
    	self.get_logger().info('Coefficients Optimized')
    	seg.set_model_type(pcl.SACMODEL_CYLINDER)
    	self.get_logger().info('Model sacd')
    	seg.set_method_type(pcl.SAC_RANSAC)
    	self.get_logger().info('Model ransacd')
    	seg.set_distance_threshold(0.03)
    	self.get_logger().info('Distance thresheld')
    	seg.set_normal_distance_weight(0.01)
    	self.get_logger().info('Distance')
    	seg.set_max_iterations(100)
    	cyl_indices, cyl_coefficients = seg.segment()
    	
    	self.get_logger().info('Done with cylinder')
    	
    	# identifies sphere
    	#seg = cloud_cluster.make_segmenter_normals(ksearch=50)
    	#seg.set_model_type(pcl.SACMODEL_SPHERE)
    	#seg.set_method_type(pcl.SAC_RANSAC)
    	#seg.set_distance_threshold(0.01)
    	#seg.set_normal_distance_weight(0.01)
    	#seg.set_max_iterations(100)
    	#sphere_indices, sphere_coefficients = seg.segment()
    	
    	self.get_logger().info('Done with sphere')
    	
    	# identifies cone
    	#seg = cloud_cluster.make_segmenter_normals(ksearch=50)
    	#seg.set_optimize_coefficients(True)
    	#seg.set_model_type(pcl.SACMODEL_CONE)
    	#seg.set_method_type(pcl.SAC_RANSAC)
    	#seg.set_distance_threshold(0.02)
    	#seg.set_normal_distance_weight(0.01)
    	#seg.set_max_iterations(100)
    	#cone_indices, cone_coefficients = seg.segment()
    	
    	self.get_logger().info('Done with cone')
    	
    	#cylinder_fit = len(cyl_indices)/cloud_cluster.size * 100
    	#sphere_fit = len(sphere_indices)/cloud_cluster.size * 100
    	#cone_fit = len(cone_indices)/cloud_cluster.size * 100
    	
    	#self.get_logger().info('Index: %f, Cylinder fit: %f, Sphere fit: %f, Cone fit: %f' % (i, cylinder_fit, sphere_fit, cone_fit))
    	
    	#sphere = cloud_cluster.extract(sphere_indices, False)
    	#cylinder = cloud_cluster.extract(cyl_indices, False)
    	#cone = cloud_cluster.extract(cyl_indices, False)
    	
    	#if sphere_fit > cylinder_fit and sphere_fit > cone_fit:
    		#pcls = sphere.to_list()
    		#self.get_logger().info('Sphere Selected')
    	#elif cylinder_fit > sphere_fit and cylinder_fit > cone_fit:
    		#pcls = cylinder.to_list()
    		#self.get_logger().info('Cylinder Selected')
    	#elif cone_fit > sphere_fit and cone_fit > cylinder_fit:
    		#pcls = cone.to_list()
    		#self.get_logger().info('Cone Selected')
    	self.get_logger().info('##DONE WITH INNER##')
    		
    self.get_logger().info('*** DONE WITH LOOP***')
    pcmsg = pc2.create_cloud_xyz32(pcmsg.header,pcls)
    self.publisher_.publish(pcmsg)
    
    


 

def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  point_cloud_subscriber = PCSubscriber()
  
  # Spin the node so the callback function is called.
  rclpy.spin(point_cloud_subscriber)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  point_cloud_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
