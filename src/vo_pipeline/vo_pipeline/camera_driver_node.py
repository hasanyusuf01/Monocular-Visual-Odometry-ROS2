#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo  # The message type for sending images
from cv_bridge import CvBridge     # The translator between OpenCV and ROS
import cv2                         # OpenCV itself
import numpy as np 
# 2. CLASS DEFINITION: Every node is a class that inherits from 'Node'
class IPCamPublisher(Node):
    
    def __init__(self):
        # Name this node 'ip_cam_publisher' in the ROS2 graph
        super().__init__('ip_cam_publisher')
        self.IP  = "http://172.16.0.154:8080/video"
        self.cam_K_matrics_path = './src/vo_pipeline/matrics/k_M.npy'
        self.cam_D_matrics_path = './src/vo_pipeline/matrics/D_M.npy'


        self.cam_K_matrics = np.load(self.cam_K_matrics_path)
        self.cam_D_matrics = np.load(self.cam_D_matrics_path)

        # self.get_logger().info(f"k = {self.cam_K_matrics},\n D = {self.cam_D_matrics}")


        self.publisher = self.create_publisher(Image,'/camera/raw_frames',10)
        self.publisher_camInfo = self.create_publisher(CameraInfo,'/camera/camera_info',10)
        self.bridge = CvBridge()
        # if IP:
        self.cap = cv2.VideoCapture(self.IP)
        # self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.create_timer(0.05,callback = self.timer_callback)
        
        self.get_logger().info("IP Camera Publisher Node has been started.")
    def get_cam_info(self,width,height,timestamp):
        msg = CameraInfo()
        msg.header.stamp = timestamp
        msg.width = width 
        msg.height = height
        msg.distortion_model = "plumb_bob"
        k =  self.cam_K_matrics.flatten()
        msg.d = self.cam_D_matrics.tolist()
        msg.k = self.cam_K_matrics.flatten()
        # self.get_logger().info(f"k = {msg.k},\n D = {self.cam_D_matrics.tolist()}")
        fx, _ , cx, _ , fy, cy , _  ,_ ,_ = k

      
        # Rectification Matrix (R) - Identity matrix for monocular cameras
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        # Projection Matrix (P) - 3x4 row-major matrix
        msg.p = [fx, 0.0, cx, 0.0,
                 0.0, fy, cy, 0.0,
                 0.0, 0.0, 1.0, 0.0]
                 
        return msg       
    # 3. THE CALLBACK: This runs every time the timer ticks
    def timer_callback(self):
        ret, frame = self.cap.read()


        if ret:
            pub_frame = self.bridge.cv2_to_imgmsg(frame, encoding ="bgr8")
            height , width , c = frame.shape
            pub_frame.header.frame_id = "camera_optical_frame"
            timestamp = self.get_clock().now().to_msg()
            pub_frame.header.stamp = timestamp
            self.publisher.publish(pub_frame)

            info_msg = self.get_cam_info(width,height,timestamp)
            self.publisher_camInfo.publish(info_msg)
            self.get_logger().info(f'publishing frame and cam info')
        else:
            self.get_logger().error('something bad has happened cant read images')
    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

# 4. MAIN LOOP: The entry point that keeps the script running
def main(args=None):
    # Initialize the ROS2 communication lines
    rclpy.init(args=args)
    
    # Create an instance of your node
    node = IPCamPublisher()
    
    try:
        # Keep the node alive and listening/publishing (Spins the node)
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Gracefully handle Ctrl+C
        print("clossing the frame capture")
    finally:
        # Clean up and shut down
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()