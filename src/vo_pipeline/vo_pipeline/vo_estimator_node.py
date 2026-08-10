#!/usr/bin/env python3

# 1. IMPORTS: Bring in the tools you need
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image,CameraInfo  # The message type for sending images
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge     # The translator between OpenCV and ROS
import cv2                         # OpenCV itself
import message_filters
from helper import detectFeatures, opticalFlow


class VIO():
    def __init__(self, num_features):
        self.num_features = num_features
            
    def vo_tracker(self,frame,cam_info):
        K = cam_info.k
        D = cam_info.d 
        P= cam_info.P
        detector = detectFeatures(self.NUM_features, self.K, self.D)


        pass



class IpCamRead(Node):
    
    def __init__(self):
        # Name this node 'ip_cam_publisher' in the ROS2 graph
        super().__init__('ip_cam_listener')

        # constants
        self.NUM_features = 100


        self.bridge = CvBridge()

        img_sub = message_filters.Subscriber(self,Image,'/camera/raw_frames')
        info_sub = message_filters.Subscriber(self,CameraInfo,'/camera/camera_info')
        self.sync = message_filters.ApproximateTimeSynchronizer([img_sub, info_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.listener_callback)
        # self.frame_subscriber = self.create_subscription(Image,'/camera/raw_frames' , self.listener_callback,10)
        # self.cam_info_subscriber = self.create_subscription(CameraInfo,'/camera/camera_info',self.listener_callback_info,10)
        self.get_logger().info("IP Camera subscriber Node has been started.")
        self.odometry_publisher = self.create_publisher(Odometry,'/vo/raw_odom',10)


    # 3. THE CALLBACK: This runs every time the timer ticks
    def get_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.child_frame_id = 'odom_msg'
        msg.header.frame_id = 'camera_optical_frame'
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

        
    def listener_callback(self,img_msg, info_msg):
        frame = self.bridge.imgmsg_to_cv2(img_msg)
        self.get_logger().info(f'info_msg{info_msg.k}')

        h,w,c = frame.shape
        self.get_logger().info(f'frame recived \n frame shape is {h} - {w} - {c}')
        odom_msg = self.get_odom()
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