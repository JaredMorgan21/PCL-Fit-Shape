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
 
class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.

    self.subscription = self.create_subscription(
      Image, 
      '/camera1/image_raw', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning

    # Create the publisher. This publisher will publish an Image
    # to the video_frames topic. The queue size is 10 messages.
    self.publisher_ = self.create_publisher(Float64MultiArray, 'centers', 10)

      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
   
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving video frame')

    current_frame = self.br.imgmsg_to_cv2(data);
    frame_hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV);

    mask = cv2.inRange(frame_hsv, (0, 50, 125), (255, 255, 255));
    current_frame = cv2.bitwise_and(current_frame, current_frame, mask=mask);
    
    centers = []
    i=0;

    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)    
    for contour in contours:
    	area = cv2.contourArea(contour);
    	if area > 100:
    		x,y,w,h = cv2.boundingRect(contour);
    		center = (x+int(w/2), y+int(h/2));
    		text = str(center);
    		font = cv2.FONT_HERSHEY_SIMPLEX
    		scale = 0.5;
    		textSize = cv2.getTextSize(text, font, scale, 1)[0];
    		b,g,r = current_frame[center[1], center[0]];
    		cv2.putText(current_frame, str(center), (int(center[0] - textSize[0]/2), y), font, scale, (int(b), int(g), int(r)), 1);
    		#cv2.circle(current_frame, center, 1, (0,255,0),2);
    		centers.append(float(center[0]));
    		centers.append(float(center[1]));
    		i+=1

    #with_contours = cv2.drawContours(current_frame, contours, -1, (0,255,0), 3);

    # PLACE YOUR CODE HERE. PROCESS THE CURRENT FRAME AND PUBLISH IT. IF YOU ARE HAVING DIFFICULTY PUBLISHING IT YOU CAN USE THE FOLLOWING LINES TO DISPLAY IT VIA OPENCV FUNCTIONS
    cv2.imshow("output_image", current_frame)
    cv2.waitKey(1)
    

    # Publish the image.
    # The 'cv2_to_imgmsg' method converts an OpenCV
    # image to a ROS 2 image message
    #print(centers)
    msg = Float64MultiArray();
    msg.data = centers;
    self.publisher_.publish(msg)

    
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_subscriber = ImageSubscriber()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
