#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
// #include "cnpy.hpp"
#include <vector>
#include <opencv2/video/tracking.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d.hpp>

struct CameraCalib {
    std::string name;
    Eigen::Matrix3d K;
    Eigen::Matrix<double,5,1> D;
};

std::vector<CameraCalib> load_cameras(const std::string& path)
{
    YAML::Node root = YAML::LoadFile(path);       // throws if file missing
    std::vector<CameraCalib> cams;

    for (const auto& cam : root["cameras"]) {
        CameraCalib c;
        c.name = cam["name"].as<std::string>();

        const auto& K = cam["intrinsics"];
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                c.K(i,j) = K[i][j].as<double>();

        const auto& D = cam["dist_coeffs"];
        for (int i = 0; i < 5; ++i)
            c.D[i] = D[i].as<double>();

        cams.push_back(c);
    }
    return cams;
}




// k_path = "..\\metrics\\K_M.npy";
// d_path = "..\\metrics\\D_M.npy";

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

      auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

      qos.reliable();
      qos.transient_local();



      subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/camera/image_raw/compressed", qos, std::bind(&frontend_node::topic_callback, this, _1));
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
    // rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
    size_t count_;
    mutable double prev_time = 0.0;





// defining the function to extract the features from the image and return the undistorted image, gray scale image, keypoints and descriptors
    std::tuple<cv::Mat, cv::Mat, std::vector<cv::KeyPoint>, cv::Mat> features_ext(cv::Mat img_frame,std::vector<CameraCalib> cams) const{
      // cv :: Mat K = cams[0].K.cast<double>();
      // cv :: Mat D = cams[0].D.cast<double>();

      cv::Mat K;
      cv::eigen2cv(cams[0].K, K);   // K is already double, no cast needed
      cv::Mat D;
      cv::eigen2cv(cams[0].D, D);   // D is already double, no cast needed
      // get the dimentions of the image for undistortion
      cv::Size s = img_frame.size();
      // int width = s.width;
      // int height = s.height;
      // RCLCPP_INFO(this->get_logger(),
      //       "camera_matrix: rows=%d cols=%d type=%d",
      //       K.rows,
      //       K.cols,
      //       K.type());
      // RCLCPP_INFO(this->get_logger(),
      //       "D: rows=%d cols=%d type=%d",
      //       D.rows, D.cols, D.type());
      // RCLCPP_INFO(this->get_logger(),
      //       "Image size: width=%d height=%d",
      //       s.width, s.height);

      //     std::cout << "K =\n" << K << std::endl;
      //     std::cout << "D =\n" << D << std::endl;

      // undistort the image using the camera matrix and distortion coefficients
      cv::Mat newCamMatrix = cv::getOptimalNewCameraMatrix(K,D,s, 0.0);


//       RCLCPP_INFO(
//     this->get_logger(),
//     "newCamMatrix: rows=%d cols=%d type=%d",
//     newCamMatrix.rows,
//     newCamMatrix.cols,
//     newCamMatrix.type()
// );
      cv::Mat frame;
      cv::undistort(img_frame, frame, K, D, newCamMatrix);// ****
      cv :: Mat gray_undistorted; // ****
      cv::cvtColor(frame, gray_undistorted, cv::COLOR_BGR2GRAY);


      // features extraction using ORB detector
      static cv::Ptr<cv::ORB> detector = cv::ORB::create(1000); // multiple params here can be changed to tune the detector
      std::vector<cv::KeyPoint> keypoints; // ****
      detector->detect(gray_undistorted, keypoints);
      cv::Mat descriptors;   // ****
      detector->compute(gray_undistorted, keypoints, descriptors);

      return std::make_tuple(frame, gray_undistorted, keypoints, descriptors);
    }



// defination of the error function to calculate the error between the forward and backward tracked points
    std::vector<bool> err_call(std::vector<cv::KeyPoint> forwardPts, std::vector<cv::KeyPoint> backPts){
      
      std::vector<int> err;
      std::vector<bool> mask;
      if (forwardPts.size() != backPts.size()) {
          RCLCPP_ERROR(this->get_logger(), "Error: forwardPts and backPts must have the same size.");

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
    // std::vector<cv::KeyPoint> optical_flow(cv::Mat prevImg, cv::Mat nextImg , std::vector<cv::KeyPoint> prevPts){
    //     std::vector<cv::KeyPoint> nextPts;
    //     std::vector<unsigned char> status;
    //     std::vector<float> err;
    //     cv::calcOpticalFlowPyrLK(	prevImg, nextImg, prevPts, nextPts,status,err);
      
    //   return nextPts;
    //   }



// // defination of the vo_tracker function to track the points between the two images and calculate the trajectory

//     cv::Mat vo_tracker(cv::Mat prevImg , cv::Mat nextImg, cv::Mat P_global){
//       // initializing the arrays to store the trajectories 
//       std::vector<std::float> trajX;
//       std::vector<std::float> trajY;
//       std::vector<std::float> trajZ;


//       cv :: Mat cameraMatrix = cams[0].K.cast<float>();
//       cv :: Mat D = cams[0].D.cast<float>();
//       cv::Mat prevGrayUndistorted, nextGrayUndistorted;



//       // extract feature points from the prev frame
//       _, prevGrayUndistorted, prvPts, _ = features_ext(prevImg);
//       if (prvPts.size()<8){
//         // check not to move forward untill enought points are there 
//         RCLCPP_INFO(this->get_logger(), "Not enough points in the prev frame");
//        return 0; 
//       }
//       else{

//         //  cvt next img to gray scale 
//       cv :: Mat nextGrayUndistorted; 
//       cv::cvtColor(nextImg, nextGrayUndistorted, cv::COLOR_BGR2GRAY);

//       //  Forward Pass :: tracking of points between the prev frame and the new frame

//       nextPts = optical_flow(prevGrayUndistorted,nextGrayUndistorted,prevPts);
//       if (nextPts.size()<8){
//               // check not to move forward untill enought points are there 
//               RCLCPP_INFO(this->get_logger(), "Not enough points in the next frame");
//             return 0; 
//             }

//       // Backward Pass :: tracking of the next points from the new image to the prev image
//       prevPts_back = optical_flow(nextGrayUndistorted,prevGrayUndistorted,nextPts);

//       // Calculate the error between the forward and backward tracked points

//       std::vector<int> mask = err_call(prevPts,prevPts_back);
//       std::vector<cv::KeyPoint> filteredPrevPts;
//       std::vector<cv::KeyPoint> filteredNextPts;

//       // filter the points based on the mask
//       for (int i= 0 ; i < mask.size(); i ++)
//       {
//         if (mask[i]){
//           filteredPrevPts.push_back(prevPts[i]);
//           filteredNextPts.push_back(nextPts[i]);
//         }
//         else{
//           continue;
//         }

//       }

//       if (filteredPrevPts.size()<8 || filteredNextPts.size()<8)
//       {
//               // check not to move forward untill enought points are there 
//             RCLCPP_INFO(this->get_logger(), "Not enough points in the filtered frames");
//             return 0; 
//       }

//       cv :: Mat essenMatrix =  cv::findEssentialMat	(	filteresPrevPts,filteresPrevPts,cameraMatrix,RANSAC,0.999,1.0,noArray() );

//       cv :: Mat R, t;

//       cv::recoverPose	(	essenMatrix,
//             filteredPrevPts,
//             filteredNextPts,
//             cameraMatrix,R,t,
//             noArray()
//             )	;
//       cv::Mat P_step = cv::Mat::eye(4, 4, CV_64F);
//       // P
//       R.copyTo(P_step(cv::Range(0, 3), cv::Range(0, 3)));
//       t.copyTo(P_step(cv::Range(0, 3), cv::Range(3, 4)));

//       cv::Mat P_global = P_global * P_step;

//       return P_global;


//       }
//     }

    


    void topic_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) const
    {

    //    RCLCPP_INFO(
    //     this->get_logger(),
    //     "========== CALLBACK RECEIVED ==========");

    // RCLCPP_INFO(
    //     this->get_logger(),
    //     "Image bytes: %zu",
    //     msg->data.size());

    // RCLCPP_INFO(
    //     this->get_logger(),
    //     "Timestamp: %d.%09d",
    //     msg->header.stamp.sec,
    //     msg->header.stamp.nanosec);
    try
         {      // check1 
           auto timestamp = msg->header.stamp;
           double time_in_seconds = timestamp.sec + timestamp.nanosec / 1e9;
           auto interval = time_in_seconds - prev_time;
           prev_time = time_in_seconds;
           std::vector<double> fps_values;
           auto fps = 1.0 / interval;
           fps_values.push_back(fps);
           
           
            RCLCPP_INFO(this->get_logger(), "frame time: %f and interval: %f", timestamp.sec + timestamp.nanosec / 1e9, interval);
            RCLCPP_INFO(this->get_logger(), "FPS: %f", fps);            
          //  if (fps_values.size() > 10) {
          //   double sum = std::accumulate(fps_values.begin(), fps_values.end(), 0.0);
          //   double average_fps = sum / fps_values.size();
          //   fps_values.erase(fps_values.begin());
          //   RCLCPP_INFO(this->get_logger(), "Average FPS: %f", average_fps);
          //  };

          cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);

          if (image.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to decode image"); 
                
           }
          else {
            RCLCPP_INFO(this->get_logger(), "Image decoded successfully");


            std::string pkg_share = ament_index_cpp::get_package_share_directory("custom_vio");
            std::string yaml_path = pkg_share + "/metrics/camera_calib.yaml";

            auto cams = load_cameras(yaml_path);
          //   // sample code to see if features_ext is working or not
      

            auto [undistorted_image, gray_undistorted, keypoints, descriptors] = features_ext(image, cams);
            cv::drawKeypoints 	( 	undistorted_image, keypoints, image, cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DEFAULT );

            cv::imshow("view", image);
            //  cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
             cv::waitKey(1);
          }
        
        
        }
    catch (const std::exception& e) {
                                    RCLCPP_ERROR(this->get_logger(), "Callback exception: %s", e.what());
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
