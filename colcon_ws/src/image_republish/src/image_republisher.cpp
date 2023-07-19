#include "image_republish/image_republisher.hpp"


using namespace std::placeholders;

namespace image_republish 
{


ImageRepublisher::ImageRepublisher(const rclcpp::NodeOptions & options)
  : Node("image_republisher", options)
  {


	  printf("creating node\n");

	  sub_ = image_transport::create_camera_subscription(
			  this, 
			  "image",
			  std::bind(&ImageRepublisher::camera_callback, this, _1, _2),
			  "raw",
			  rmw_qos_profile_sensor_data
			  );


	  printf("created subscriber\n");


	  pub_ = image_transport::create_camera_publisher(
			  this, 
			  "republished_image",
			  rmw_qos_profile_sensor_data
			  );

	  printf("created publisher\n");



  }


void ImageRepublisher::camera_callback(const sensor_msgs::msg::Image::ConstSharedPtr & img, 
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr & cam_info)
{

	(void) img;
	(void) cam_info;

	//printf("I got an image\n");


	pub_.publish(img, cam_info);

}


} // namespace image_republish


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(image_republish::ImageRepublisher)
