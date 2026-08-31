#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/CompressedImage.hpp"
#include "geometry_msgs/msg/PoseStamped.hpp"


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class frontend_node : public rclcpp::Node
{
  public:
    frontend_node()
    : Node("frontend_node"), count_(0)
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("vo/pose", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&frontend_node::timer_callback, this));
            subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/camera/image_raw/compressed", 10, std::bind(&frontend_node::topic_callback, this, _1));
    }

  private:
    void timer_callback()
    {
      auto message = geometry_msgs::msg::PoseStamped();
      message.header.stamp = this->get_clock()->now();
      message.pose.position.x = 1.0;
      message.pose.position.y = 2.0;
      message.pose.position.z = 3.0;
      RCLCPP_INFO(this->get_logger(), "Publishing pose: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    size_t count_;


    void topic_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) const
    {
    try
         {
             cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
             cv::waitKey(30);
           }
    catch (cv_bridge::Exception& e)
           {
             ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
           }
       
    RCLCPP_INFO(this->get_logger(), "I heard a compressed image");
    }
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<frontend_node>());
  rclcpp::shutdown();
  return 0;
}
