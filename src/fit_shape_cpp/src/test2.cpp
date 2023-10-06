// header for ROS core functionalities
#include "rclcpp/rclcpp.hpp"

// including image message type to be able to receive and publish it
#include "sensor_msgs/msg/image.hpp"

// headers regarding the connection between opencv and ROS
#include <image_transport/image_transport.hpp>
#include "cv_bridge/cv_bridge.h"

// OpenCV core functions and various image processing algorithms
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using std::placeholders::_1;

// Defining a class that will be utilize in the "main" function
class ImageSubscriber : public rclcpp::Node
{

	// Declaring pointer to the publisher and subscriber the publish and receive images.
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

  public:
	// Constructor of the class. The class is derived from the parent class "Node" of rclcpp and
	// the node is called "image_processor", and each time it receive a data, it will call the callbackImage() function
    	ImageSubscriber() : Node("image_processor")
	{
		// Defining the subscriber: it will receive "Image" type data from the topic /camera1/image_raw 
      		subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/camera1/image_raw", 10, std::bind(&ImageSubscriber::callbackImage, this, _1));

	//defining the publisher: it will publish "Image" type data to the "output_image" topic
            publisher_ = this->create_publisher<sensor_msgs::msg::Image>("output_image", 10);
    }

  private:

	// callback function which will be triggered each time the subscriber_ receives new data.
	// The data it receives is of Image type.
	void callbackImage(const sensor_msgs::msg::Image::SharedPtr msg) const
	{    
		// converting the ROS message type data to opencv type
		cv_bridge::CvImagePtr cv_ptr;
		cv_ptr = cv_bridge::toCvCopy(msg,"bgr8" );
		
		// creating an opencv image that has 1 channel (gray scale as opposed to color bgr8 image type above.)
		cv::Mat dst =cv:: Mat::zeros( cv_ptr->image.size(), CV_32FC1);

		// converting colorimage to the gray scale image using the cvtcolor function
		cv::cvtColor(cv_ptr->image, dst, cv::COLOR_BGR2GRAY);
		
		// converting the gray scale image to ros message data type (mono8) 
		sensor_msgs::msg::Image::SharedPtr msg_to_send = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", dst).toImageMsg();

		// publishing the image
     		publisher_->publish(*msg_to_send);

    }
    
};

int main(int argc, char * argv[])
{

	//initialize ROS
	rclcpp::init(argc, argv);

	//create the 
	rclcpp::spin(std::make_shared<ImageSubscriber>());
	rclcpp::shutdown();
  return 0;
}
