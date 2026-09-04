
---

# `VIO_PROJECT_PLAN.md`

## System Architecture Overview

```
                          [ Phone Sensor Node ]
                                    │
           ┌────────────────────────┴────────────────────────┐
           │                                                 │
 30 Hz Camera Stream                               100 Hz IMU Stream
   /mobile_sensor/image_raw                           /mobile_sensor/imu
           │                                                 │
           ▼                                                 ▼
┌───────────────────────┐                         ┌──────────────────────┐
│  VO Front-End Node    │                         │  ESKF Back-End Node  │
│ (Optical Flow & Pose) │                         │ (IMU Queue & Fusion) │
└──────────┬────────────┘                         └──────────┬───────────┘
           │                                                 │
    /vo/pose (30 Hz)                                         │
           │                                                 │
           └────────────────────────┬────────────────────────┘
                                    │
                                    ▼
                         ┌──────────────────────┐
                         │ State Propagation &  │
                         │   Kalman Correction  │
                         └──────────┬───────────┘
                                    │
               ┌────────────────────┴────────────────────┐
               │                                         │
               ▼                                         ▼
      /fused_odom (100 Hz)                      TF Broadcast
    (nav_msgs/Odometry)                       (odom ──> base_link)

```

---

## Phase 1: Data Architecture & Plumbing Setup

The focus of Phase 1 is establishing the data pipeline infrastructure. Before executing state estimation math, you will create a multi-threaded ROS 2 node architecture that ingests 100 Hz IMU data and 30 Hz image streams, buffers them safely, and prevents data drops or race conditions.

### 1.1 Package and Workspace Architecture

Set up the workspace repository structure using `ament_cmake` for C++ performance (or `ament_python` if building in Python).

```
custom_vio_ws/
└── src/
    ├── custom_vio/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── include/custom_vio/
    │   │   ├── vo_frontend_node.hpp
    │   │   ├── eskf_backend_node.hpp
    │   │   └── imu_buffer.hpp
    │   ├── src/
    │   │   ├── vo_frontend_node.cpp
    │   │   └── eskf_backend_node.cpp
    │   └── config/
    │       └── camera_params.yaml
    └── custom_vio_interfaces/
        ├── CMakeLists.txt
        ├── package.xml
        └── action/
            └── CalibrateImu.action

```

* **TODO:** Initialize the ROS 2 packages in your workspace: `ros2 pkg create --build-type ament_cmake custom_vio` and `ros2 pkg create --build-type ament_cmake custom_vio_interfaces`.
* **TODO:** Configure `package.xml` to depend on `rclcpp`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `cv_bridge`, `image_transport`, `tf2`, `tf2_ros`, `Eigen3`, and `OpenCV`.
* **TODO:** Modify `CMakeLists.txt` to locate dependencies via `find_package()`, configure include directories, and declare executable targets for `vo_frontend_node` and `eskf_backend_node`.

### 1.2 Visual Odometry Front-End Skeleton Node

Build a node to process camera frames without frame drops.

* **Node Name:** `vo_frontend_node`
* **Subscribed Topic:** `/camera/image_raw/compressed` (`sensor_msgs/msg/CompressedImage`)
* **Published Topic:** `/vo/pose` (`geometry_msgs/msg/PoseStamped`)
* **Execution Strategy:** Use `cv_bridge::toCvCopy` to convert incoming ROS image payloads into `cv::Mat` (monochrome `bgr8` or `mono8`).
* **TODO:** Create `vo_frontend_node.cpp` inheriting from `rclcpp::Node`.
* **TODO:** Set up the subscriber to `/mobile_sensor/image_raw` using an `rclcpp::SensorDataQoS` profile to minimize transport latency over Wi-Fi.
* **TODO:** Write the `imageCallback` skeleton that converts the image via `cv_bridge`, logs frame timestamps, and tracks frame arrival intervals to verify stable 30 Hz delivery.
* **TODO:** Initialize the `/vo/pose` publisher.

### 1.3 ESKF Back-End Skeleton Node

Build the state-estimation node using multi-threaded execution to prevent fast IMU callbacks from blocking visual pose updates.

* **Node Name:** `eskf_backend_node`
* **Subscribed Topics:**
* `/mobile_sensor/imu` (`sensor_msgs/msg/Imu`)
* `/vo/pose` (`geometry_msgs/msg/PoseStamped`)


* **Published Topic:** `/fused_odom` (`nav_msgs/msg/Odometry`)
* **Callback Groups:** Create two separate `Reentrant` or `MutuallyExclusive` callback groups. Assign the high-rate IMU subscriber to Callback Group A and the visual pose subscriber to Callback Group B.
* **TODO:** Implement `eskf_backend_node.cpp` with a Multi-Threaded Executor (`rclcpp::executors::MultiThreadedExecutor`).
* **TODO:** Add the IMU subscriber bound to Callback Group A.
* **TODO:** Add the `/vo/pose` subscriber bound to Callback Group B.
* **TODO:** Initialize the `/fused_odom` publisher.

### 1.4 Thread-Safe IMU Data Buffering & Synchronization

Camera frames arrive at ~33 ms intervals, whereas IMU samples arrive at ~10 ms intervals. The backend must retain an ordered timeline of IMU measurements so it can integrate IMU data between visual frames.

* **Data Structure:** Implement a custom `ImuBuffer` class wrapping a `std::deque<sensor_msgs::msg::Imu>`. Protect access using a `std::mutex` and `std::lock_guard`.
* **Extraction Logic:** When a visual pose with timestamp $t_{\text{frame}}$ arrives, query the buffer to extract all IMU samples satisfying $t_{\text{prev\_frame}} < t_{\text{imu}} \le t_{\text{frame}}$, keeping those samples in the buffer for numerical propagation while discarding older data.
* **TODO:** Write `imu_buffer.hpp` defining the thread-safe `std::deque` and mutex locking methods (`push()`, `pop_until()`, `clear()`).
* **TODO:** Instantiate `ImuBuffer` inside `eskf_backend_node.cpp`.
* **TODO:** Test the pipeline by printing queue sizes inside the visual pose callback to confirm the IMU buffer fills and empties correctly without data races.
Here is the breakdown of how header files work, followed by the highly detailed, code-free steps to build your IMU buffer.

**The Purpose of `.hpp` (Header) Files**
In C++, `.hpp` files act as a "table of contents" or a "blueprint." They tell the compiler *what* exists (the names of classes, functions, and variables) without explaining *how* they work. The actual logic goes into the `.cpp` file. When you write `#include "my_file.hpp"`, you are sharing that blueprint with other parts of your program.

**Will the other `.hpp` files be auto-generated?**
**No.** In ROS 2, you must manually write all the `.hpp` and `.cpp` files for your custom nodes (like `vo_frontend_node` and `eskf_backend_node`). The *only* files ROS 2 auto-generates are the headers for custom Messages, Services, and Actions (like the `CalibrateImu.action` you will make later).

---

**Expanded TODO 1: Design the `imu_buffer.hpp` Blueprint**

* **Location:** Create a new file named `imu_buffer.hpp` inside your `custom_vio/include/custom_vio/` folder.
* **Include Guards:** At the very top, add a "pragma once" directive. This prevents the C++ compiler from accidentally reading this blueprint twice if multiple files include it.
* **Library Includes:** Import the standard C++ tools you need: the `deque` (double-ended queue), `vector` (dynamic array), and `mutex` (the locking mechanism). You also need to include the ROS 2 IMU message header.
* **Class Definition:** Define a new class named `ImuBuffer`.
* **Private Section (The Data):**
* Declare a `std::deque` designed to hold ROS 2 IMU messages. This is your actual buffer.
* Declare a `std::mutex`. Think of a mutex like a "bathroom key." Because your IMU thread and your Camera thread are running simultaneously, they might both try to access the buffer at the exact same microsecond, causing a crash. The mutex ensures only one thread can touch the buffer at a time.


* **Public Section (The Interface):**
* Declare a `push` method that accepts a new IMU message.
* Declare a `get_measurements_until` method that accepts a target timestamp and returns a list (vector) of IMU messages.
* Declare a `clear` method to empty the queue.



**Expanded TODO 2: Write the Buffer Logic**

*(For a simple utility class like this, you can write the logic directly inside the `.hpp` file under the declarations).*

* **Logic for `push`:** When this method is called, it must first "lock" the mutex. Once locked, it pushes the incoming IMU message to the back of the deque. Finally, it "unlocks" the mutex so other threads can use it.
* **Logic for `get_measurements_until`:**
1. Lock the mutex.
2. Create an empty result list.
3. Start a loop that looks at the "front" (oldest) message in the deque.
4. If the deque is empty, or if the oldest message is newer than the target timestamp, break the loop.
5. Otherwise, copy that oldest message into your result list, and pop (delete) it from the front of the deque.
6. Unlock the mutex and return the result list.


* **Logic for `clear`:** Lock the mutex, call the standard clear command on the deque, and unlock.

**Expanded TODO 3: Instantiate and Use in the Backend Node**

* **Include the File:** At the top of `eskf_backend_node.cpp`, include your new `imu_buffer.hpp` file.
* **Declare the Object:** In the private section of your backend node class (where you declared your publishers and subscribers), declare a variable of type `ImuBuffer`.
* **Update the IMU Callback:** Inside your high-speed `imu_callback`, take the incoming IMU message and pass it directly into the buffer's `push` method. You do not need to do any math here; just store the data.
* **Update the Pose Callback:** Inside your slower `pose_callback`, extract the exact timestamp from the incoming visual pose message. Pass that timestamp into the buffer's `get_measurements_until` method.
* **Verify the Output:** Store the result of that method in a local variable. Write a ROS 2 logging statement (`RCLCPP_INFO`) to print the size of that returned list.

If everything is working perfectly, you should see the node printing that it extracted roughly 3 to 4 IMU messages every time a single visual pose arrives (since 100Hz is roughly 3.3 times faster than 30Hz).
---

## Phase 2: Visual Odometry Front-End & ROS 2 Services

In Phase 2, you will implement the computer vision tracking pipeline. This module tracks optical features across video frames, solves for 5-point relative essential matrices using RANSAC, integrates camera motion, and exposes a ROS 2 Service to reset tracking if the pipeline drifts or loses feature tracking.

```
Incoming Image Frame (t)
         │
         ▼
[ FAST Keypoint Extraction ] ──(If keypoint count < threshold)
         │
         ▼
[ KLT Optical Flow Tracking ] ──(Track features from frame t-1 to t)
         │
         ▼
[ RANSAC 5-Point Algorithm ] ──(Compute Essential Matrix E & Recover R, t)
         │
         ▼
[ Relative Motion Accumulation ] ──> Publish /vo/pose
         │
 (If Tracking Fails)
         │
         ▼
[ /reset_odometry Service Callback ] ──> Clears features, resets origin

```

### 2.1 Feature Detection & Optical Flow Tracking

Implement a 2D feature tracking pipeline using the Kanade-Lucas-Tomasi (KLT) algorithm rather than matching heavy feature descriptors frame-by-frame.

* **Detection Strategy:** Detect keypoints using `cv::FAST` or `cv::goodFeaturesToTrack` (Shi-Tomasi corner detector) when the active feature count falls below a set threshold (e.g., 150 points).
* **Tracking Strategy:** Track keypoints across consecutive frames using pyramidal Lucas-Kanade optical flow (`cv::calcOpticalFlowPyrLK`). Discard keypoints whose status vector signals tracking failure or whose flow vectors fall outside valid spatial bounds.
* **TODO:** Write `feature_tracker.cpp` and `feature_tracker.hpp` containing keypoint tracking structures.
* **TODO:** Implement feature extraction logic to replenish keypoints when the feature pool drops below `min_features`.
* **TODO:** Implement `cv::calcOpticalFlowPyrLK` to compute forward-backward optical flow error and reject invalid point movements.

### 2.2 Relative Motion Estimation (5-Point Algorithm)

Estimate the 3D rotation matrix $\mathbf{R} \in SO(3)$ and normalized translation vector $\mathbf{t} \in S^2$ between consecutive keypoint sets.

* **Math Operations:**
1. Pass tracked 2D-2D pixel correspondences $(\mathbf{p}_{k-1}, \mathbf{p}_k)$ along with camera intrinsic parameters $(f_x, f_y, c_x, c_y)$ to `cv::findEssentialMat` using RANSAC.
2. Pass the estimated Essential Matrix $\mathbf{E}$ to `cv::recoverPose` to extract the relative rotation $\mathbf{R}_{\text{rel}}$ and relative translation unit vector $\mathbf{t}_{\text{rel}}$.


* **Scale Ambiguity:** Monocular visual odometry lacks absolute scale. Set the translation magnitude $\Vert{}\mathbf{t}_{\text{rel}}\Vert{}$ to a relative unit scale or estimate it dynamically using IMU double-integration between visual frames.
* **TODO:** Parse camera calibration matrix $\mathbf{K}$ from `camera_params.yaml`.
* **TODO:** Implement `cv::findEssentialMat` using `cv::RANSAC` with a threshold of 1.0 pixel and a confidence of 0.999.
* **TODO:** Implement `cv::recoverPose` to isolate valid inlier rotations and translations.

### 2.3 Trajectory Accumulation & Pose Publishing

Accumulate frame-to-frame relative transformations into an absolute visual world frame ($W_{\text{vo}}$).

* **Kinematic Chain:**

$$\mathbf{R}_{\text{global}}(k) = \mathbf{R}_{\text{global}}(k-1) \cdot \mathbf{R}_{\text{rel}}$$


$$\mathbf{p}_{\text{global}}(k) = \mathbf{p}_{\text{global}}(k-1) + \mathbf{R}_{\text{global}}(k-1) \cdot \mathbf{t}_{\text{rel}}$$


* **TODO:** Convert the accumulated rotation matrix $\mathbf{R}_{\text{global}}$ into a normalized orientation quaternion $(x, y, z, w)$.
* **TODO:** Populate a `geometry_msgs::msg::PoseStamped` payload containing the calculated position, orientation quaternion, frame ID (`odom`), and original image timestamp.
* **TODO:** Publish the message on `/vo/pose`.

### 2.4 ROS 2 Reset Service Integration

When aggressive motion causes optical flow tracking to fail, expose a synchronous ROS 2 Service to reset the front-end tracking state.

* **Service Name:** `/reset_odometry`
* **Service Type:** `std_srvs/srv/Trigger`
* **Callback Behavior:** Reset accumulated positions to zero, clear current feature tracking arrays, re-trigger full keypoint detection on the next frame, and return a successful execution status string.
* **TODO:** Include `<std_srvs/srv/trigger.hpp>` in `vo_frontend_node.hpp`.
* **TODO:** Instantiate an `rclcpp::Service<std_srvs::srv::Trigger>` server named `/reset_odometry`.
* **TODO:** Write the service callback function to clear feature vectors, reset orientation and position states, and return `success = true`.
* **TODO:** Test the service from the terminal using `ros2 service call /reset_odometry std_srvs/srv/Trigger "{}"`.

---

## Phase 3: Error-State Kalman Filter (ESKF) & ROS 2 Actions

Phase 3 implements the core sensor fusion mathematics. You will build an **Error-State Kalman Filter (ESKF)** that uses high-rate IMU data for nominal state propagation and low-rate Visual Odometry poses for error-state updates.

Before running the filter, you will execute a ROS 2 Action that computes initial static IMU biases over a long duration.

```
                       [ Start Calibration Action ]
                                    │
                                    ▼
                         (Collect 10s Static IMU Data)
                                    │
                                    ▼
                       [ Compute Biases ba, bg & Gravity ]
                                    │
                                    ▼
                           [ Initialize ESKF ]
                                    │
   ┌────────────────────────────────┴────────────────────────────────┐
   │                                                                 │
100 Hz IMU Stream                                            30 Hz Visual Pose
   │                                                                 │
   ▼                                                                 ▼
[ High-Hz Nominal Propagation ]                           [ Low-Hz Measurement Update ]
 - Integrate p, v, q kinematics                             - Compute Innovation Residual H
 - Propagate Covariance Matrix P                            - Compute Kalman Gain K
                                                            - Update Error State dx
                                                            - Inject dx into Nominal State
                                                            - Reset Error State dx -> 0

```

### 3.1 ESKF Mathematical Formulation

Define state representations using Lie Algebra on $SO(3)$ to prevent gimbal lock and quaternion normalization issues.

* **True State Vector $\mathbf{x} \in \mathbb{R}^{16}$:**

$$\mathbf{x} = \begin{bmatrix} \mathbf{p} & \mathbf{v} & \mathbf{q} & \mathbf{b}_a & \mathbf{b}_g \end{bmatrix}^T$$


* Position $\mathbf{p} \in \mathbb{R}^3$, Velocity $\mathbf{v} \in \mathbb{R}^3$, Orientation Quaternion $\mathbf{q} \in S^3$, Acceleration Bias $\mathbf{b}_a \in \mathbb{R}^3$, Gyroscope Bias $\mathbf{b}_g \in \mathbb{R}^3$.


* **Error State Vector $\delta \mathbf{x} \in \mathbb{R}^{15}$:**

$$\delta \mathbf{x} = \begin{bmatrix} \delta \mathbf{p} & \delta \mathbf{v} & \delta \boldsymbol{\theta} & \delta \mathbf{b}_a & \delta \mathbf{b}_g \end{bmatrix}^T$$


* Where $\delta \boldsymbol{\theta} \in \mathbb{R}^3$ represents small angular errors in vector space.


* **TODO:** Create `eskf_math.hpp` incorporating the `Eigen` matrix library.
* **TODO:** Define type aliases for the $15 \times 15$ Error Covariance Matrix $\mathbf{P}$, Process Noise Covariance Matrix $\mathbf{Q}$, Measurement Matrix $\mathbf{H}$, and Measurement Noise Covariance Matrix $\mathbf{R}$.
* **TODO:** Implement utility functions for converting 3D rotation vectors $\delta \boldsymbol{\theta}$ to Quaternions via the exponential map $\exp(\delta \boldsymbol{\theta})$.

### 3.2 Static IMU Calibration ROS 2 Action Server

Build a ROS 2 Action Server to estimate static accelerometer and gyroscope biases before launching the filter.

* **Action Interface:** `custom_vio_interfaces/action/CalibrateImu.action`
```action
# Goal
int32 duration_seconds
---
# Result
geometry_msgs/Vector3 accel_bias
geometry_msgs/Vector3 gyro_bias
bool success
---
# Feedback
float32 percent_complete

```


* **TODO:** Create `CalibrateImu.action` inside `custom_vio_interfaces/action/`.
* **TODO:** Update `custom_vio_interfaces/CMakeLists.txt` to generate action interfaces via `rosidl_generate_interfaces()`.
* **TODO:** Implement the Action Server inside `imu_calibrator.cpp` (or as a component in `eskf_backend_node.cpp`).
* **TODO:** Write the goal execution loop: collect raw IMU data for `duration_seconds` while the phone remains stationary, publish `percent_complete` feedback, compute average accelerometer and gyroscope bias vectors ($\mathbf{b}_a, \mathbf{b}_g$), alignment to gravity $g = 9.81 \, \text{m/s}^2$, and return the result.

### 3.3 High-Frequency IMU Nominal State Propagation

Integrate IMU measurements forward in time at 100 Hz to update position, velocity, and orientation estimates.

* **Kinematic Propagation Equations:**

$$\mathbf{p}_{k+1} = \mathbf{p}_k + \mathbf{v}_k \Delta t + \frac{1}{2} \left( \mathbf{R}(\mathbf{q}_k) (\mathbf{a}_m - \mathbf{b}_a) + \mathbf{g} \right) \Delta t^2$$


$$\mathbf{v}_{k+1} = \mathbf{v}_k + \left( \mathbf{R}(\mathbf{q}_k) (\mathbf{a}_m - \mathbf{b}_a) + \mathbf{g} \right) \Delta t$$


$$\mathbf{q}_{k+1} = \mathbf{q}_k \otimes q\left( (\boldsymbol{\omega}_m - \mathbf{b}_g) \Delta t \right)$$


* **Covariance Propagation:**

$$\mathbf{P}_{k+1} = \mathbf{F}_x \mathbf{P}_k \mathbf{F}_x^T + \mathbf{F}_i \mathbf{Q} \mathbf{F}_i^T$$


* Where $\mathbf{F}_x$ is the $15 \times 15$ Error State Transition Jacobian matrix evaluated at state $\mathbf{x}$.


* **TODO:** Write the continuous discrete error transition Jacobian builder $\mathbf{F}_x(\mathbf{x}, \mathbf{a}_m, \boldsymbol{\omega}_m, \Delta t)$ in `eskf_math.hpp`.
* **TODO:** Implement nominal state integration in the IMU queue processing loop.
* **TODO:** Implement covariance prediction $\mathbf{P} \leftarrow \mathbf{F}_x \mathbf{P} \mathbf{F}_x^T + \mathbf{Q}$.

### 3.4 Low-Frequency Visual Pose Measurement Update

Correct state estimate drift whenever a visual pose estimate arrives on `/vo/pose`.

* **Measurement Model:**

$$\mathbf{z}_v = \begin{bmatrix} \mathbf{p}_{\text{vo}} \\ \mathbf{q}_{\text{vo}} \end{bmatrix}$$


$$\mathbf{y} = \mathbf{z}_v - \mathbf{h}(\mathbf{x}) \quad \text{(Innovation Residual)}$$


* **Kalman Update Equations:**

$$\mathbf{S} = \mathbf{H} \mathbf{P} \mathbf{H}^T + \mathbf{R}_v$$


$$\mathbf{K} = \mathbf{P} \mathbf{H}^T \mathbf{S}^{-1} \quad \text{(Kalman Gain)}$$


$$\delta \mathbf{x} = \mathbf{K} \mathbf{y} \quad \text{(Computed Error State)}$$


$$\mathbf{P} \leftarrow (\mathbf{I} - \mathbf{K} \mathbf{H}) \mathbf{P} (\mathbf{I} - \mathbf{K} \mathbf{H})^T + \mathbf{K} \mathbf{R}_v \mathbf{K}^T \quad \text{(Joseph Form Covariance Update)}$$


* **Error Injection & Reset:** Inject the computed error state $\delta \mathbf{x}$ into the nominal state vector ($\mathbf{p} \leftarrow \mathbf{p} + \delta \mathbf{p}$, $\mathbf{q} \leftarrow \mathbf{q} \otimes \exp(\delta \boldsymbol{\theta})$, etc.), then reset the error state vector $\delta \mathbf{x} \leftarrow \mathbf{0}$.
* **TODO:** Construct the measurement Jacobian matrix $\mathbf{H}$ mapping the 15-DOF error state to visual pose observations.
* **TODO:** Compute the innovation residual vector $\mathbf{y}$ and apply quaternion error subtraction logic.
* **TODO:** Compute the Kalman gain matrix $\mathbf{K}$ and execute error state correction.
* **TODO:** Inject error corrections into the nominal state variables and reset $\delta \mathbf{x}$ to zero.

---

## Phase 4: Output, Transform Trees & Evaluation

Phase 4 completes the ROS 2 integration lifecycle. You will implement dynamic coordinate transformations (`tf2`), write a unified launch file using YAML parameter configurations, record test runs, and quantitatively benchmark the estimated trajectory against the phone's WebXR ground truth using trajectory evaluation tools.

```
                  [ vio_bringup.launch.py ]
                              │
     ┌────────────────────────┼────────────────────────┐
     │                        │                        │
     ▼                        ▼                        ▼
[ vo_frontend_node ]   [ eskf_backend_node ]       [ RViz2 ]
(Loads camera params)  (Broadcasts Dynamic TF) (Loads default config)
                              │
                              ▼
                TF Tree: odom ──> base_link
                              │
                              ▼
                [ ros2 bag record /fused_odom ]
                              │
                              ▼
                [ EVO Benchmarking Analysis ]
             (Computes Absolute Pose Error ATE)

```

### 4.1 ROS 2 Dynamic Transform Broadcasting

Publish dynamic frame transforms to integrate your VIO node with standard ROS 2 visualization tools (RViz2) and spatial frame chains.

* **Frame Architecture:**
* `odom`: Fixed, non-drifting world reference frame origin.
* `base_link`: Mobile phone body-centric moving frame.


* **Transform Broadcaster:** Use `tf2_ros::TransformBroadcaster` to broadcast the spatial transform mapping `odom` $\rightarrow$ `base_link` inside the backend output loop.
* **TODO:** Include `<tf2_ros/transform_broadcaster.h>` in `eskf_backend_node.hpp`.
* **TODO:** Instantiate `std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_`.
* **TODO:** Populate a `geometry_msgs::msg::TransformStamped` payload using the fused position and orientation from the ESKF state vector.
* **TODO:** Broadcast the transform dynamically at 100 Hz alongside your `/fused_odom` publisher.

### 4.2 Unified System Launch File & Configuration Setup

Write a ROS 2 Python launch file to spin up all nodes, load parameters, and configure visualization with a single terminal command.

* **Files Created:**
* `custom_vio/launch/vio_bringup.launch.py`
* `custom_vio/config/params.yaml`
* `custom_vio/config/rviz_config.rviz`


* **YAML Config Parameters:** Define camera intrinsic parameters, feature tracking limits, IMU noise density ($\sigma_a, \sigma_g$), random walk noise parameters, and topic names.
* **TODO:** Create `params.yaml` with parameters for image width, height, camera intrinsics ($f_x, f_y, c_x, c_y$), and IMU covariance values.
* **TODO:** Write `vio_bringup.launch.py` using `launch` and `launch_ros` packages to start `vo_frontend_node`, `eskf_backend_node`, and `rviz2`.
* **TODO:** Ensure the launch file passes `params.yaml` directly into the nodes.
* **TODO:** Test system startup using `ros2 launch custom_vio vio_bringup.launch.py`.

### 4.3 Trajectory Bagging & EVO Evaluation Benchmarking

Record live flight data and benchmark performance against the mobile phone's native WebXR spatial tracking (`/mobile_sensor/pose`).

* **Data Collection Protocol:**
1. Start the ROS 2 mobile sensor bridge node on your phone and laptop.
2. Run the static IMU calibration action to compute sensor biases.
3. Launch the VIO pipeline and walk a complete loop around a room, returning to the starting point.
4. Record the topics to a bag file:
```bash
ros2 bag record -o vio_test_run /fused_odom /mobile_sensor/pose

```




* **Quantitative Metrics:** Use `evo` (Python trajectory evaluation tool) to compute:
* **Absolute Pose Error (ATE):** Evaluates global trajectory drift.
* **Relative Pose Error (RPE):** Evaluates local frame-to-frame drift.


* **TODO:** Install the EVO benchmarking library: `pip install evo --upgrade`.
* **TODO:** Convert the recorded ROS 2 bag file topics into evaluation formats using `evo_ros`.
* **TODO:** Compute the Absolute Trajectory Error (ATE) between your `/fused_odom` trajectory and ground truth `/mobile_sensor/pose`:
```bash
evo_ape ros2 vio_test_run.db3 /mobile_sensor/pose /fused_odom -va --plot --plot_mode xy

```


* **TODO:** Export generated trajectory comparison plots and save the calculated RMSE (Root Mean Square Error) statistics to document your results.

---

## Final Project Deliverables

When all phases are complete, your workspace repository will contain:

1. **Custom C++ VIO Nodes:** Real-time visual tracking and custom Error-State Kalman Filtering code using Eigen and OpenCV.
2. **ROS 2 Interfaces:** Custom Action for static sensor bias calibration (`CalibrateImu.action`) and Service for tracking resets (`/reset_odometry`).
3. **TF Transformation Tree:** Live spatial transform publishing mapping `odom` to `base_link`.
4. **Empirical Benchmarking Plots:** Quantifiable trajectory accuracy verification against WebXR ground-truth data, demonstrating real-world sensor fusion.