#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "cnpy.h"
#include <vector>
#include <opencv2/video/tracking.hpp>
#include <opencv2/features2d.hpp>


using std::placeholders::_1;
using namespace std::chrono_literals;



k_path = "..\\metrics\\K_M.npy";
d_path = "..\\metrics\\D_M.npy";

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
      50ms, std::bind(&frontend_node::timer_callback, this));


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
      RCLCPP_INFO(this->get_logger(), "Publishing pose: '%f'", message.pose.position.x);
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    size_t count_;

    std::tuple<cv::Mat, cv::Mat, std::vector<cv::KeyPoint>, cv::Mat> features_ext(cv::Mat img_frame){
      cnpy::NpyArray K = cnpy::npy_load(k_path);
      cnpy::NpyArray D = cnpy::npy_load(d_path);
      // get the dimentions of the image for undistortion
      cv::Size s = img.size();
      int width = s.width;
      int height = s.height;

      // undistort the image using the camera matrix and distortion coefficients
      cv::Mat newCamMatrix = cv::getOptimalNewCameraMatrix(K,D,s, 0.0, centerPrincipalPoint = false );
      frame = cv::undistort(img_frame, newCamMatrix, K, D);  // ****
      cv :: Mat gray_undistorted; // ****
      cv::cvtColor(frame, gray_undistorted, cv::COLOR_BGR2GRAY);


      // features extraction using ORB detector
      static Ptr<ORB> detector = cv::ORB::create(100); // multiple params here can be changed to tune the detector
      std::vector<cv::KeyPoint> keypoints; // ****
      detector->detect(gray_undistorted, keypoints);
      cv::Mat descriptors;   // ****
      detector->compute(gray_undistorted, keypoints, descriptors);

      return std::make_tuple(frame, gray_undistorted, keypoints, descriptors);
    }

    void optical_flow(cv::Mat frame_n, cv::mat frame_n_1 , std::vector<cv::KeyPoint> keypoints_n){




    }

    void vo_tracker(cv::Mat frame){






    }

    


    void topic_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) const
    {
    try
         {      cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
                cv::imshow("view", image);
            //  cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
             cv::waitKey(1);
           }
    catch (cv_bridge::Exception& e)
           {
             RCLCPP_ERROR(this->get_logger(), "Could not convert from to 'bgr8'.");
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
