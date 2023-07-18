#ifndef IMAGE_REPUBLISHER
#define IMAGE_REPUBLISHER

#include <rcutils/logging_macros.h>

#include <image_transport/camera_publisher.hpp>
#include <image_transport/camera_subscriber.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>


namespace image_republish 
{


class ImageRepublisher : public rclcpp::Node
{

	public:
		explicit ImageRepublisher(const rclcpp::NodeOptions & options);

		virtual ~ImageRepublisher() {}


	protected:

		void camera_callback(const sensor_msgs::msg::Image::ConstSharedPtr & img,
				const sensor_msgs::msg::CameraInfo::ConstSharedPtr & cam_info);

	private:

		//publishers
		image_transport::CameraPublisher pub_;
		
		//subscribers
		image_transport::CameraSubscriber sub_;
};

} //namespace image_republish


#endif // IMAGE_REPUBLISH
