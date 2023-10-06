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
    
    # taken from https://github.com/strawlab/python-pcl/blob/master/examples/official/Segmentation/Plane_model_segmentation.py
    seg = pc.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    indices, model = seg.segment()
    
    #print(type(seg));

    pcls = pc.to_list();
    pcmsg = pc2.create_cloud_xyz32(pcmsg.header,pcls);

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
