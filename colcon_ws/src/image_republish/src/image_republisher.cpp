#include "image_republish/image_republisher.hpp"


using namespace std::placeholders;

namespace image_republish 
{


ImageRepublisher::ImageRepublisher(const rclcpp::NodeOptions & options)
  : Node("image_republisher", options)
  {

	// Get parameters
        input_image_topic_ = declare_parameter("input_image_topic", "/camera/image");
        output_image_topic_ = declare_parameter("output_image_topic", "/camera/image_modified");

	RCLCPP_INFO(get_logger(), "creating node");

	sub_ = image_transport::create_camera_subscription(
			  this, 
			  input_image_topic_,
			  std::bind(&ImageRepublisher::camera_callback, this, _1, _2),
			  "raw",
			  rmw_qos_profile_sensor_data
			  );


	  RCLCPP_INFO(get_logger(), "created subscriber");


	  pub_ = image_transport::create_camera_publisher(
			  this, 
			  output_image_topic_,
			  rmw_qos_profile_sensor_data
			  );
	  
	  RCLCPP_INFO(get_logger(), "created publisher");

  }


void ImageRepublisher::camera_callback(const sensor_msgs::msg::Image::ConstSharedPtr & img, 
		const sensor_msgs::msg::CameraInfo::ConstSharedPtr & cam_info)
{

// img->header.frame_id = new_frame_id_;
//	cam_info->header.frame_id = new_frame_id_;

	pub_.publish(img, cam_info);

}


} // namespace image_republish


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(image_republish::ImageRepublisher)
