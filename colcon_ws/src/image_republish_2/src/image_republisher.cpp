#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class ImageRepublisher : public rclcpp::Node {
public:
    ImageRepublisher() : Node("image_republisher") {
        // Get parameters
        declare_parameter("input_image_topic", "/camera/image");
        declare_parameter("output_image_topic", "/camera/image_modified");
        declare_parameter("new_frame_id", "modified_frame");

        get_parameter("input_image_topic", input_image_topic_);
        get_parameter("output_image_topic", output_image_topic_);
        get_parameter("new_frame_id", new_frame_id_);

        // Set up the publisher
        pub_ = create_publisher<sensor_msgs::msg::Image>(output_image_topic_, rclcpp::SensorDataQoS());

        // Set up the subscriber
        sub_ = create_subscription<sensor_msgs::msg::Image>(
            input_image_topic_, rclcpp::SensorDataQoS(), std::bind(&ImageRepublisher::imageCallback, this, std::placeholders::_1));
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Modify the frame_id
        auto modified_msg = std::make_shared<sensor_msgs::msg::Image>(*msg);
        modified_msg->header.frame_id = new_frame_id_;

        // Publish the modified image
        pub_->publish(*modified_msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    std::string input_image_topic_;
    std::string output_image_topic_;
    std::string new_frame_id_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageRepublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

