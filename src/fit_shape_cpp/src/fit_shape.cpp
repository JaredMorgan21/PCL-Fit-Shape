// header for ROS core functionalities
#include "rclcpp/rclcpp.hpp"

// including image message type to be able to receive and publish it
#include "sensor_msgs/msg/point_cloud2.hpp"

using std::placeholders::_1;

// Defining a class that will be utilize in the "main" function
class PCSubscriber : public rclcpp::Node
{

	// Declaring pointer to the publisher and subscriber the publish and receive images.
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

  public:
	// Constructor of the class. The class is derived from the parent class "Node" of rclcpp and
	// the node is called "image_processor", and each time it receive a data, it will call the callbackImage() function
    	PCSubscriber() : Node("pc_processor")
	{
		// Defining the subscriber: it will receive "PointCloud2" type data from the topic /camera1/image_raw 
      		subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/realsense/points", 10, 		std::bind(&PCSubscriber::callbackPC, this, _1));

	//defining the publisher: it will publish "Image" type data to the "output_image" topic
            publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("fit_pc", 10);
    }

  private:

	// callback function which will be triggered each time the subscriber_ receives new data.
	// The data it receives is of PointCloud2 type.
	void callbackPC(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
	{    
		std::cout << msg;
		
		sensor_msgs::msg::PointCloud2 output;
		// publishing the image.
   		publisher_->publish(output);
    }
    
};

int main(int argc, char * argv[])
{

	//initialize ROS
	rclcpp::init(argc, argv);

	//create the 
	rclcpp::spin(std::make_shared<PCSubscriber>());
	rclcpp::shutdown();
  return 0;
}
