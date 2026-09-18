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
#include <opencv2/calib3d.hpp>

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




// defining the function to extract the features from the image and return the undistorted image, gray scale image, keypoints and descriptors
    std::tuple<cv::Mat, cv::Mat, std::vector<cv::KeyPoint>, cv::Mat> features_ext(cv::Mat img_frame){
      cv :: Mat K = cnpy::npy_load(k_path);
      cv :: Mat D = cnpy::npy_load(d_path);
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



// defination of the error function to calculate the error between the forward and backward tracked points
    std::vector<int> err_call(std::vector<cv::KeyPoint> forwardPts, std::vector<cv::KeyPoint> backPts){
      
      std::vector<int> err;
      std::vector<bool> mask;
      if (forwardPts.size() != backPts.size()) {
          std::cerr << "Error: forwardPts and backPts must have the same size." << std::endl;
          return {};
      }
      for (size_t i = 0; i < forwardPts.size(); ++i) {
          float dx = forwardPts[i].pt.x - backPts[i].pt.x;
          float dy = forwardPts[i].pt.y - backPts[i].pt.y;
          err.push_back(std::sqrt(dx * dx + dy * dy));
      }
      for (int i = 0 ; i <err.size(); i ++){
        if (err[i]<1.0){
          mask.push_back(true);
        }
        else{
          mask.push_back(false);
        }
      }
      return mask;
    }


  
// defination of the optical flow function to track the points between the two images
    std::vector<cv::KeyPoint> optical_flow(cv::Mat prevImg, cv::mat nextImg , std::vector<cv::KeyPoint> prevPts){
        std::vector<cv::KeyPoint> nextPts;
        std::vector<std::uchar> status;
        std::vector<std::float> err;
        cv::calcOpticalFlowPyrLK(	prevImg, nextImg, prevPts, nextPts,status,err,);
      
      return nextPts;
      }



// defination of the vo_tracker function to track the points between the two images and calculate the trajectory

    void vo_tracker(cv::Mat prevImg , cv::Mat nextImg){
      // initializing the arrays to store the trajectories 
      std::vector<std::float> trajX;
      std::vector<std::float> trajY;
      std::vector<std::float> trajZ;


      cv :: Mat cameraMatrix = cnpy::npy_load(k_path);
      cv :: Mat D = cnpy::npy_load(d_path);



      // extract feature points from the prev frame
      _, prevGrayUndistorted, prvPts, _ = features_ext(prevImg);
      if (prvPts.size()<8){
        // check not to move forward untill enought points are there 
        std::cout<<"Not enough points in the prev frame"<<endl;
       return 0; 
      }
      else{

        //  cvt next img to gray scale 
      cv :: Mat nextGrayUndistorted; 
      cv::cvtColor(nextImg, nextGrayUndistorted, cv::COLOR_BGR2GRAY);

      //  Forward Pass :: tracking of points between the prev frame and the new frame

      nextPts = optical_flow(prevGrayUndistorted,nextGrayUndistorted,prevPts);
      if (nextPts.size()<8){
              // check not to move forward untill enought points are there 
              std::cout<<"Not enough points in the next frame"<<endl;
            return 0; 
            }

      // Backward Pass :: tracking of the next points from the new image to the prev image
      prevPts_back = optical_flow(nextGrayUndistorted,prevGrayUndistorted,nextPts);

      // Calculate the error between the forward and backward tracked points

      std::vector<int> mask = err_call(prevPts,prevPts_back);
      std::vector<cv::KeyPoint> filteredPrevPts;
      std::vector<cv::KeyPoint> filteredNextPts;

      // filter the points based on the mask
      for (int i= 0 ; i < mask.size(); i ++)
      {
        if (mask[i]){
          filteredPrevPts.push_back(prevPts[i]);
          filteredNextPts.push_back(nextPts[i]);
        }
        else{
          continue;
        }

      }

      if (filteredPrevPts.size()<8 || filteredNextPts.size()<8)
      {
              // check not to move forward untill enought points are there 
            std::cout<<"Not enough points in the filtered frames"<<endl;
            return 0; 
      }

      cv :: Mat essenMatrix =  cv::findEssentialMat	(	filteresPrevPts,filteresPrevPts,cameraMatrix,RANSAC,0.999,1.0,noArray() );

      cv :: Mat R, t;

      cv::recoverPose	(	essenMatrix,
            filteredPrevPts,
            filteredNextPts,
            cameraMatrix,R,t,
            noArray()
            )	

      
















      }
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
