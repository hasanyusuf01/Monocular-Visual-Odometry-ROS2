#!/usr/bin/env python3

# 1. IMPORTS: Bring in the tools you need
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image,CameraInfo  # The message type for sending images
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge     # The translator between OpenCV and ROS
import cv2                         # OpenCV itself
import message_filters
from vo_pipeline.helper import vo_tracker
import numpy as np



class IpCamRead(Node):
    
    def __init__(self):
        # Name this node 'ip_cam_publisher' in the ROS2 graph
        super().__init__('ip_cam_listener')
        self.cam_info_flag = False

        # constants
        self.NUM_features = 100
        self.f0 = None

        self.f1 = None
        self.P_global = np.eye(4)


        self.bridge = CvBridge()

        img_sub = message_filters.Subscriber(self,Image,'/camera/raw_frames')
        info_sub = message_filters.Subscriber(self,CameraInfo,'/camera/camera_info')
        self.sync = message_filters.ApproximateTimeSynchronizer([img_sub, info_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.listener_callback)
        # self.frame_subscriber = self.create_subscription(Image,'/camera/raw_frames' , self.listener_callback,10)
        # self.cam_info_subscriber = self.create_subscription(CameraInfo,'/camera/camera_info',self.listener_callback_info,10)
        self.get_logger().info("IP Camera subscriber Node has been started.")
        self.odometry_publisher = self.create_publisher(Odometry,'/vo/raw_odom',10)


    def get_dummy_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.child_frame_id = 'camera_optical_frame'
        msg.header.frame_id = 'odom_msg'
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.covariance = [0.0] * 36  # Ignored for now

        # --- TWIST (How fast the camera is moving) ---
        msg.twist.twist.linear.x = 0.5  # Moving forward at 0.5 m/s
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = 0.1 # Rotating slightly
        
        msg.twist.covariance = [0.0] * 36 # Ignored for now
        
        return msg
    def get_odom(self,info_cam, f0, f1, del_T, P_global):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.child_frame_id = 'camera_optical_frame'
        msg.header.frame_id = 'odom_msg'
        # f0 = self.f0
        # f1 = self.f1
        K_matrix = np.array(info_cam.k, dtype=float).reshape(3, 3)
        D_matrix = np.array(info_cam.d, dtype=float)
        x, y , z ,quat_xyzw , angular_velocities, linear_velocities, P_global = vo_tracker(self.NUM_features,K_matrix,D_matrix, f0, f1,del_T, P_global)
        self.get_logger().info(f"Position: x={x}, y={y}, z={z}")
        if x is None or y is None or z is None:
            self.get_logger().warn("Odometry calculation failed due to insufficient features.")
            return self.get_dummy_odom(), P_global
        x = float(x)
        y = float(y)
        z = float(z)
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.x = quat_xyzw[0]
        msg.pose.pose.orientation.y = quat_xyzw[1]
        msg.pose.pose.orientation.z = quat_xyzw[2]
        msg.pose.covariance = [0.0] * 36  # Ignored for now



        # --- TWIST (How fast the camera is moving) ---
        self.get_logger().info(f"Linear Velocities: {linear_velocities[0][0]}")
        self.get_logger().info(f"Angular Velocities: {angular_velocities}")

        linear_velocities = np.array(linear_velocities, dtype=float)
        angular_velocities = np.array(angular_velocities, dtype=float)
        msg.twist.twist.linear.x = linear_velocities[0][0]  # Moving forward at 0.5 m/s
        msg.twist.twist.linear.y = linear_velocities[1][0]
        msg.twist.twist.linear.z = linear_velocities[2][0]
        
        msg.twist.twist.angular.x = angular_velocities[0]
        msg.twist.twist.angular.y = angular_velocities[1]
        msg.twist.twist.angular.z = angular_velocities[2] # Rotating slightly
        
        msg.twist.covariance = [0.0] * 36 # Ignored for now
        
        return msg, P_global
        
    def listener_callback(self,img_msg, info_msg):
        frame = self.bridge.imgmsg_to_cv2(img_msg)

        if self.cam_info_flag == False:
            self.cam_info = info_msg
            # self.get_logger().info(f'info_msg{info_msg.k}')
            self.cam_info_flag = True
        if self.f0 == None and self.f1 == None:
            # first time so the current frame is set to f0 
            self.f0 = img_msg
            f0 = self.bridge.imgmsg_to_cv2(self.f0)
            odom_msg = self.get_dummy_odom()
            self.odometry_publisher.publish(odom_msg)
        else:
            # from second timeonwards f0 = prev set frame and f1 = current one
            self.f1 = img_msg
            f0 = self.bridge.imgmsg_to_cv2(self.f0)
            f1 = self.bridge.imgmsg_to_cv2(self.f1)
            # Convert f1 time to total seconds (float)
            t1 = self.f1.header.stamp.sec + (self.f1.header.stamp.nanosec * 1e-9)
            # Convert f0 time to total seconds (float)
            t0 = self.f0.header.stamp.sec + (self.f0.header.stamp.nanosec * 1e-9)
            # Calculate the time difference in seconds
            del_T = t1 - t0
            # self.get_logger().info(f"Time difference (del_T): {del_T} seconds")
            # h,w,c = frame.shape
            odom_msg, P_global = self.get_odom(info_msg,f0,f1,del_T,self.P_global)
            self.P_global = P_global
            self.f0 = self.f1
            # self.get_logger().info(f'frame odom msg {odom_msg}')

            self.odometry_publisher.publish(odom_msg)


            frame = cv2.resize(frame,(640,480))
            cv2.imshow("camera", frame)
            cv2.waitKey(1)
    

    # (Optional but good practice) Cleanup when the node shuts down
    def destroy_node(self):
        # TODO 9: Release the camera
        # Call .release() on your VideoCapture object so it doesn't lock up your camera hardware.
        super().destroy_node()

# 4. MAIN LOOP: The entry point that keeps the script running
def main(args=None):
    # Initialize the ROS2 communication lines
    rclpy.init(args=args)
    
    # Create an instance of your node
    node = IpCamRead()
    
    try:
        # Keep the node alive and listening/publishing (Spins the node)
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("clossing subscriber")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()