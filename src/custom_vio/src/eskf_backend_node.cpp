#include <chrono>
#include <functional>
#include <memory>
#include <string>
// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
// #include "sensor_msgs/msg/compressed_image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class eskf_backend_node : public rclcpp::Node
{
  public:
    eskf_backend_node()
    : Node("eskf_backend_node"), count_(0)
    {
      publisher_odom = this->create_publisher<nav_msgs::msg::Odometry>("/fused_odom", 10);
      timer_ = this->create_wall_timer(
      50ms, std::bind(&eskf_backend_node::odom_callback, this));


      subscription_imu = this->create_subscription<sensor_msgs::msg::Imu>(
      "/mobile_sensor/imu", 10, std::bind(&eskf_backend_node::imu_callback, this, _1));


      subscription_pose = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/vo/pose", 10, std::bind(&eskf_backend_node::pose_callback, this, _1));

    }

  private:
    void odom_callback()
    {
      auto message = nav_msgs::msg::Odometry();
      message.header.stamp = this->get_clock()->now();
      message.header.frame_id = "odom";
      message.child_frame_id = "odom_child";
      message.pose.pose.position.x = 1.0;
      message.pose.pose.position.y = 2.0;
      message.pose.pose.position.z = 3.0;

      message.pose.pose.orientation.x = 0.0;
      message.pose.pose.orientation.y = 0.0;
      message.pose.pose.orientation.z = 0.0;
      message.pose.pose.orientation.w = 1.0;

    //   message.pose.covariance = std::array<int> vec(36, 0);

      message.twist.twist.linear.x = 0 ;
      message.twist.twist.linear.y = 0 ;
      message.twist.twist.linear.z = 0;
      message.twist.twist.angular.x = 0; 
      message.twist.twist.angular.y = 0; 
      message.twist.twist.angular.z = 0;

    //   message.twist.covariance = std::array<int> vec(36, 0);
//     message.pose.covariance.fill(0.0);
// message.twist.covariance.fill(0.0);



      RCLCPP_INFO(this->get_logger(), "Publishing pose ");
      publisher_odom->publish(message);
    }
    
    
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) const
    
    {
        auto msg_local = geometry_msgs::msg::PoseStamped();
        msg_local.header.stamp =    msg->header.stamp;
        msg_local.pose.position.x = msg->pose.position.x;
        msg_local.pose.position.y = msg->pose.position.y;
        msg_local.pose.position.z = msg->pose.position.z;
        
        
        RCLCPP_INFO(this->get_logger(), " message pos x = %f , message pos y = %f, message pos z = %f ", msg_local.pose.position.x,msg_local.pose.position.y, msg_local.pose.position.z);

        
        // RCLCPP_INFO(this->get_logger(), " message pos x = %f , message pos y = %f, message pos z = %f ", msg_local.pose.position.x,msg_local.pose.position.y, msg_local.pose.position.z);
    }


    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) const
    
    {
        auto message = sensor_msgs::msg::Imu();

        message.header.stamp =    msg->header.stamp;
 
        message.orientation.x = msg->orientation.x;
        message.orientation.y = msg->orientation.y;
        message.orientation.z = msg->orientation.z;
        message.orientation.w = msg->orientation.w;

        message.orientation_covariance = msg->orientation_covariance;
        message.linear_acceleration_covariance  = msg->linear_acceleration_covariance ;

        message.linear_acceleration.x = msg->linear_acceleration.x ;
        message.linear_acceleration.y = msg->linear_acceleration.y ;
        message.linear_acceleration.z = msg->linear_acceleration.z;

        message.angular_velocity.x =  msg->angular_velocity.x; 
        message.angular_velocity.y =  msg->angular_velocity.y; 
        message.angular_velocity.z = msg->angular_velocity.z;

        //   message.twist.covariance = std::array<int> vec(36, 0);
    //     message.pose.covariance.fill(0.0);
    // message.twist.covariance.fill(0.0);



        RCLCPP_INFO(this->get_logger(), " recived some imu data ");

    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_pose;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_imu;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_odom;
    size_t count_;






};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<eskf_backend_node>());
  rclcpp::shutdown();
  return 0;
}
