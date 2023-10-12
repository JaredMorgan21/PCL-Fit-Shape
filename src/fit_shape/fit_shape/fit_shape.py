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
import math
from scipy.linalg import norm

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
    pcmsg = msg
    
    #converts binary blob to numpy array of tuple (x, y, z, rgb);
    #pcnp = pc2.read_points(msg);
    
    #converts binary blob to numpy array of tuple (x, y, z, rgb);
    pcls = pc2.read_points_list(msg, skip_nans=True, field_names=("x", "y", "z"));
    
    #converts list to PointCloud
    pc = pcl.PointCloud()
    pc.from_list(pcls)
    
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
    seg.set_distance_threshold(0.01)
    seg.set_normal_distance_weight(0.01)
    seg.set_max_iterations(100)
    indices, coefficients = seg.segment()
    
    # Extracting the indices from the pc without the major plane
    non_plane = pc.extract(indices, True)

    # Filter/downsample with voxel grid WIP
    #vg = non_plane.make_voxel_grid_filter()
    #vg.set_leaf_size(0.001, 0.001, 0.001)
    #non_plane = vg.filter()
    
    # clusters the shapes found
    # from https://github.com/strawlab/python-pcl/blob/master/examples/official/Segmentation/cluster_extraction.py
    tree = non_plane.make_kdtree()
    ec = non_plane.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.1)
    ec.set_MinClusterSize(50)
    ec.set_MaxClusterSize(25000)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()
    
    #WIP
    pcls = []

    # Creates a new point cloud for each cluster in the loop
    pc_cluster = pcl.PointCloud()
    # Loops through the extract clusters from the point cloud
    for j, indices in enumerate(cluster_indices):
        # Logs the size of the point cloud of the object
        self.get_logger().info('Object {:.0f} detected of size {:.0f}' .format(j+1, len(indices)))
        # Creates an empty array
        points = np.zeros((len(indices), 3), dtype=np.float32)
        # Convert the points from the cluster nested in the point cloud to an array
        for i, index in enumerate(indices):
            points[i][0] = non_plane[index][0]
            points[i][1] = non_plane[index][1]
            points[i][2] = non_plane[index][2]

        # Converts np array to point cloud object
        pc_cluster.from_array(points)
        
        # identifies cylinder
        seg = pc_cluster.make_segmenter_normals(ksearch=50)
        seg.set_optimize_coefficients(True)
        seg.set_model_type(pcl.SACMODEL_CYLINDER)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_distance_threshold(0.03)
        seg.set_normal_distance_weight(0.01)
        seg.set_max_iterations(100)
        cyl_indices, cyl_coefficients = seg.segment()
        
        # identifies sphere
        seg = pc_cluster.make_segmenter_normals(ksearch=50)
        seg.set_optimize_coefficients(True)
        seg.set_model_type(pcl.SACMODEL_SPHERE)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_distance_threshold(0.01)
        seg.set_normal_distance_weight(0.01)
        seg.set_max_iterations(100)
        sphere_indices, sphere_coefficients = seg.segment()
        
        # identifies cone
        seg = pc_cluster.make_segmenter_normals(ksearch=50)
        seg.set_optimize_coefficients(True)
        seg.set_model_type(pcl.SACMODEL_CONE)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_distance_threshold(0.03)
        seg.set_normal_distance_weight(0.01)
        seg.set_max_iterations(100)
        cone_indices, cone_coefficients = seg.segment()
        
        # current fit algorithm
        cylinder_fit = len(cyl_indices)/pc_cluster.size * 100
        sphere_fit = len(sphere_indices)/pc_cluster.size * 100
        cone_fit = len(cone_indices)/pc_cluster.size * 100
        
        # Print to terminal and log
        self.get_logger().info('Cylinder fit: %f, Sphere fit: %f, Cone fit: %f' % (cylinder_fit, sphere_fit, cone_fit))
        
        # Re-extract the shapes from the point cloud with out the major plane
        sphere = non_plane.extract(sphere_indices, False)
        cylinder = non_plane.extract(cyl_indices, False)
        cone = non_plane.extract(cyl_indices, False)
        
        # selects model based on fit algorithm involving the size of the point cloud after segmentation
        if sphere_fit > cylinder_fit and sphere_fit > cone_fit:
             self.get_logger().info('Sphere selected of radius %f' % sphere_coefficients[3])
             pcls = pcls + sphere.to_list()
    	     
             # from https://stackoverflow.com/questions/22930211/how-to-generate-sphere-coordinates-in-python
             a = 80
             r = sphere_coefficients[3]
             theta = np.radians(np.linspace(0, 360, a+1))
             phi = np.radians(np.linspace(0, 360, a+1))
             
             # Creates the points for the sphere
             x = r * np.einsum("i,j->ij", np.cos(phi), np.sin(theta))
             y = r * np.einsum("i,j->ij", np.sin(phi), np.sin(theta))
             z = r * np.einsum("i,j->ij", np.ones(len(theta)), np.cos(theta))
             xyz = np.array([x.flatten(), y.flatten(), z.flatten()])
             
             xs = xyz[0]
             ys = xyz[1]
             zs = xyz[2]
             
             # Add the sphere poitns to the point cloud
             for i, val in enumerate(xs):
             	pcls.append([xs[i] + sphere_coefficients[0], ys[i] + sphere_coefficients[1], zs[i] + sphere_coefficients[2]])
        # Checks if cylinder fit is the highest
        elif cylinder_fit > sphere_fit and cylinder_fit > cone_fit:
    	     #pcls = pcls + cylinder.to_list();
             self.get_logger().info('Cylinder selected of radius %f' % cyl_coefficients[6])

             # Extracts the features of the cylinder
             # radius
             R = cyl_coefficients[6]
             # center point
             p0 = np.array([cyl_coefficients[0], cyl_coefficients[1], cyl_coefficients[2]])
             # direction of axis with respect to world frame
             axis = np.array([cyl_coefficients[3], cyl_coefficients[4], cyl_coefficients[5]])
             # Manually set the legnth given that it is not
             # determined by the fit_shape function
             height = 0.3
    	    
             # Creates a cylinder based on the parameters listed above
             cyl_points = make_cylinder(p0, axis, R, height);
             xs = cyl_points[0].flatten();
             ys = cyl_points[1].flatten();
             zs = cyl_points[2].flatten();
             # Appends the cylinder poitns to the pointcloud
             for i, val in enumerate(xs):
                  pcls.append([xs[i], ys[i], zs[i]])
    	     
        # Checks if the cone fit is the highest
        elif cone_fit >= sphere_fit and cone_fit >= cylinder_fit:
    	     #pcls = pcls + cone.to_list();
    	     self.get_logger().info('Cone Selected')
    	     
           # Creates the points for the non-truncated postion of the cone
    	     height = 0.3
    	     p0 = np.array([cone_coefficients[0], cone_coefficients[1], cone_coefficients[2]])
    	     axis = np.array([cyl_coefficients[3], cyl_coefficients[4], cyl_coefficients[5]])
    	     R0 = 0
    	     R1 = height*math.tan(cone_coefficients[6])
    	     
           # Creates the points for the trunctaed portion of the cone
    	     cone_points = truncated_cone(p0, axis, R0, R1, height)
    	     xs = cone_points[0].flatten()
    	     ys = cone_points[1].flatten()
    	     zs = cone_points[2].flatten()
    	     
           # Add the points to a point cloud
    	     for i, val in enumerate(xs):
    	     	pcls.append([xs[i], ys[i], zs[i]])

    # Publish teh point cloud without the major plane
    # and with the projected shapes. 
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
                   
# from https://stackoverflow.com/questions/48703275/3d-truncated-cone-in-python
def make_cylinder(p0, axis, R0, height):
    R1 = R0
    """
    Based on https://stackoverflow.com/a/39823124/190597 (astrokeat)
    """
    # vector in direction of axis
    v = axis
    # find magnitude of vector
    mag = norm(v)
    # unit vector in direction of axis
    v = v / mag
    # make some vector not in the same direction as v
    not_v = np.array([1, 1, 0])
    if (v == not_v).all():
        not_v = np.array([0, 1, 0])
    # make vector perpendicular to v
    n1 = np.cross(v, not_v)
    # print n1,'\t',norm(n1)
    # normalize n1
    n1 /= norm(n1)
    # make unit vector perpendicular to v and n1
    n2 = np.cross(v, n1)
    # surface ranges over t from 0 to length of axis and 0 to 2*pi
    n = 80
    t = np.linspace(0, height, n)
    theta = np.linspace(0, 2 * np.pi, n)
    # use meshgrid to make 2d arrays
    t, theta = np.meshgrid(t, theta)
    R = np.linspace(R0, R1, n)
    # generate coordinates for surface
    X, Y, Z = [p0[i] + v[i] * t + R *
               np.sin(theta) * n1[i] + R * np.cos(theta) * n2[i] for i in [0, 1, 2]]
    return [X,Y,Z]
    
# from https://stackoverflow.com/questions/48703275/3d-truncated-cone-in-python
def truncated_cone(p0, axis, R0, R1, height):
    """
    Based on https://stackoverflow.com/a/39823124/190597 (astrokeat)
    """
    # vector in direction of axis
    v = axis
    # find magnitude of vector
    mag = norm(v)
    # unit vector in direction of axis
    v = v / mag
    # make some vector not in the same direction as v
    not_v = np.array([1, 1, 0])
    if (v == not_v).all():
        not_v = np.array([0, 1, 0])
    # make vector perpendicular to v
    n1 = np.cross(v, not_v)
    # print n1,'\t',norm(n1)
    # normalize n1
    n1 /= norm(n1)
    # make unit vector perpendicular to v and n1
    n2 = np.cross(v, n1)
    # surface ranges over t from 0 to length of axis and 0 to 2*pi
    n = 80
    t = np.linspace(0, height, n)
    theta = np.linspace(0, 2 * np.pi, n)
    # use meshgrid to make 2d arrays
    t, theta = np.meshgrid(t, theta)
    R = np.linspace(R0, R1, n)
    # generate coordinates for surface
    X, Y, Z = [p0[i] + v[i] * t + R *
               np.sin(theta) * n1[i] + R * np.cos(theta) * n2[i] for i in [0, 1, 2]]
    return [X,Y,Z]

if __name__ == '__main__':
  main()
