## Phase 1: Data Architecture and Core Nodes

**Package Structure**

* Create `custom_vio` containing `CMakeLists.txt` and `package.xml`, explicitly declaring dependencies on `rclcpp`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `cv_bridge`, Eigen3, and OpenCV.
* **TODO:** Run `ros2 pkg create --build-type ament_cmake custom_vio` in your workspace.
* **TODO:** Update `package.xml` with dependencies and configure `CMakeLists.txt` to find and link OpenCV and Eigen3.

**Message Synchronization**

* Implement `message_filters::sync_policies::ApproximateTime` inside `vo_frontend.cpp` to correctly align the 30Hz `/mobile_sensor/image_raw` callbacks with your internal processing clock.
* **TODO:** Write the `vo_frontend` node class.
* **TODO:** Include the `message_filters` headers and verify that image callbacks are firing at the correct frame rate.

**Thread-Safe IMU Buffer**

* Develop `eskf_backend.cpp` containing a `std::deque<sensor_msgs::msg::Imu>` protected by a `std::mutex` to asynchronously queue the 100Hz `/mobile_sensor/imu` messages without blocking the main thread.
* **TODO:** Create the `eskf_backend` node with a fast callback group for the IMU subscriber.
* **TODO:** Implement the mutex lock to prevent race conditions when the front-end requests IMU data.

---

## Phase 2: Visual Front-End and ROS 2 Services

**Optical Flow Tracking**

* Write `feature_tracker.cpp` to extract FAST keypoints and track them across sequential frames using OpenCV's `cv::calcOpticalFlowPyrLK`, computing the relative $3 \times 3$ rotation matrix and $3 \times 1$ translation vector via `cv::recoverPose`.
* **TODO:** Implement `cv_bridge` to convert ROS images to `cv::Mat`.
* **TODO:** Write the feature extraction and tracking loop, ensuring out-of-bounds or lost points are discarded.

**Pose Publishing**

* Transform the computed relative motion into a global trajectory and publish it at 30Hz as a `geometry_msgs::msg::PoseStamped` message on the custom `/vo/pose` topic.
* **TODO:** Integrate camera intrinsic parameters (focal length, optical center) into the pose recovery math.
* **TODO:** Initialize the publisher and broadcast the global pose.

**Tracking Reset Service**

* Define a `std_srvs/srv/Trigger` service inside `vo_frontend.cpp` named `/reset_odometry` that clears the active keypoint vectors and resets the local coordinate frame to the origin when called.
* **TODO:** Set up the ROS 2 Service Server in the front-end node.
* **TODO:** Write the callback function to clear historical pose matrices and feature arrays.

---

## Phase 3: Error-State Kalman Filter and Actions

**State Formulation**

* Define the 15-DOF nominal state vector in `eskf_math.hpp` as $\mathbf{x} = [\mathbf{p}, \mathbf{v}, \mathbf{q}, \mathbf{b}_a, \mathbf{b}_g]^T$ alongside its corresponding error state $\delta \mathbf{x}$ to robustly handle $SO(3)$ quaternion manifolds without singularities.
* **TODO:** Create `eskf_math.hpp` and define the Eigen vectors and matrices for the state, covariance $P$, process noise $Q$, and measurement noise $R$.

**Action Server Calibration**

* Create a custom action `custom_vio_interfaces/action/CalibrateImu` in `imu_calibrator.cpp` that records static data for 10 seconds, calculates the bias vectors, and provides a continuous percentage-complete feedback loop.
* **TODO:** Define the `.action` file in a separate interfaces package and compile it.
* **TODO:** Implement the Action Server to compute the mean (bias) of the buffered IMU data while publishing feedback.

**Filter Updates**

* Implement the strapdown integration for the prediction step at 100Hz using the IMU buffer, and trigger the Kalman correction equations (Jacobian matrix updates) whenever a new pose arrives on `/vo/pose`.
* **TODO:** Write the IMU kinematics math to propagate the state forward in time.
* **TODO:** Write the Kalman gain and error-state reset logic for the measurement update step.

---

## Phase 4: Output, Transforms, and Evaluation

**TF2 Broadcasting**

* Integrate `tf2_ros::TransformBroadcaster` into the backend node to continuously publish the dynamic transformation tree linking the static `odom` frame to the mobile `base_link` frame.
* **TODO:** Include the `tf2` headers and populate a `geometry_msgs::msg::TransformStamped` message with your fused pose.
* **TODO:** Verify the transform tree in the terminal using `ros2 run tf2_tools view_frames`.

**Launch File Orchestration**

* Write a Python launch file `vio_bringup.launch.py` to concurrently start the front-end, back-end, and RViz2, loading parameters like camera intrinsics from a `config.yaml` file.
* **TODO:** Create the `launch` and `config` directories in your package.
* **TODO:** Write the Python script and YAML parameter file, ensuring all node names and topic remaps are correct.

**Benchmarking**

* Record the `/fused_odom` output into a `.db3` bag file and utilize the `evo_ape` Python toolkit to mathematically compare your estimated trajectory against the phone's WebXR ground truth `/mobile_sensor/pose`.
* **TODO:** Record a live dataset by walking a loop in your room.
* **TODO:** Run the `evo_ape` command and generate error plots to document in your portfolio.