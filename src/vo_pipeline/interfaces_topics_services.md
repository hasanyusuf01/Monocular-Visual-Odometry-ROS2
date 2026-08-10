Here is a complete breakdown of all the **Nodes**, **Topics**, **Services**, and **Interfaces** that make up this Monocular Visual Odometry project.

---

## 1. Nodes (The Execution Units)

* **`camera_driver_node`**: Reads frames from your webcam/IP camera and retrieves the intrinsic calibration parameters.
* **`vo_estimator_node`**: Runs Lucas-Kanade optical flow, estimates the Essential Matrix with RANSAC, and recovers relative frame-to-frame movement ($R$ and $T$).
* **`motion_filter_node`**: Evaluates incoming relative movements, rejects absurd spikes, applies temporal smoothing (EKF or Low-Pass filter), and integrates position into global coordinates.

---

## 2. Topics (Continuous Streaming Data)

| Topic Name | Direction | Interface / Message Type | Purpose |
| --- | --- | --- | --- |
| `/camera/image_raw` | `camera_driver` $\rightarrow$ `vo_estimator` | `sensor_msgs/msg/Image` | Unprocessed video frames from the camera. |
| `/camera/camera_info` | `camera_driver` $\rightarrow$ `vo_estimator` | `sensor_msgs/msg/CameraInfo` | Camera intrinsic matrix ($K$) and distortion parameters ($D$) needed to compute 3D rays. |
| `/vo/raw_odom` | `vo_estimator` $\rightarrow$ `motion_filter` | `nav_msgs/msg/Odometry` | Raw, unfiltered frame-to-frame relative translation and rotation. |
| `/vo/filtered_odom` | `motion_filter` $\rightarrow$ RViz / System | `nav_msgs/msg/Odometry` | Outlier-rejected, smoothed global position $(X, Y, Z)$ and orientation. |
| `/vo/debug_markers` | `vo_estimator` $\rightarrow$ RViz2 | `visualization_msgs/msg/MarkerArray` | 3D optical flow vectors and tracked feature points for visual debugging in RViz2. |

---

## 3. Services (Request / Response Control)

Adding these services completes your control loop, allowing live manipulation of the pipeline without restarting nodes:

| Service Name | Server Node | Interface / Service Type | Request Payload | Response / Action |
| --- | --- | --- | --- | --- |
| `/reset_odometry` | `motion_filter_node` | `std_srvs/srv/Trigger` | Empty | Resets accumulated position back to $(0,0,0)$ and clears the state filter's memory. Returns `success=True`. |
| `/set_feature_params` | `vo_estimator_node` | Custom or `rcl_interfaces/srv/SetParameters` | `max_corners` (int), `ransac_thresh` (float) | Dynamically adjusts Shi-Tomasi feature count or RANSAC rejection sensitivity on the fly. |

---

## 4. Interfaces (Data Structures & Types)

An **Interface** in ROS2 defines the exact data fields inside a Topic or Service. This project uses both standard ROS2 interfaces and optional custom interfaces:

### Standard ROS2 Interfaces Used:

* **`sensor_msgs/msg/Image`**: Holds raw pixel buffers, image width/height, and timestamp.
* **`sensor_msgs/msg/CameraInfo`**: Holds the $3 \times 3$ intrinsic matrix ($K$), projection matrix ($P$), and distortion model.
* **`geometry_msgs/msg/Odometry`**: Holds 3D Position (`x, y, z`), Orientation (`quaternion`), and Velocity (`twist`).
* **`std_srvs/srv/Trigger`**: A standard service interface containing an empty request and a response payload: `bool success`, `string message`.

### Custom Interface (Optional enhancement for your package):

If you want to create a custom package interface (e.g., `vo_interfaces/msg/FlowStats.msg`), you can record execution statistics:

```text
# vo_interfaces/msg/FlowStats.msg
std_msgs/Header header
uint32 tracked_features_count
uint32 ransac_inliers_count
float32 mean_reprojection_error
bool motion_rejected

```

---

## Architecture Flow Map

```text
                       [camera_driver_node]
                                |
             +------------------+------------------+
             | TOPIC: /camera/image_raw            | TOPIC: /camera/camera_info
             v                                     v
                       [vo_estimator_node]
                                |
             +------------------+------------------+
             | TOPIC: /vo/raw_odom                 | TOPIC: /vo/debug_markers
             v                                     v
                     [motion_filter_node]               (RViz2 Visualizer)
                            |       ^
 TOPIC: /vo/filtered_odom   |       |  SERVICE: /reset_odometry
                            v       +--------------------- [CLI / Dashboard]
                     (Navigation / Map)

```