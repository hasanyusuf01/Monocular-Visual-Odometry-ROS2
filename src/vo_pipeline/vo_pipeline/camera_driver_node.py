#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image  # The message type for sending images
from cv_bridge import CvBridge     # The translator between OpenCV and ROS
import cv2                         # OpenCV itself

# 2. CLASS DEFINITION: Every node is a class that inherits from 'Node'
class IPCamPublisher(Node):
    
    def __init__(self):
        # Name this node 'ip_cam_publisher' in the ROS2 graph
        super().__init__('ip_cam_publisher')
        self.IP  = "http://172.16.0.154:8080/video"
        self.publisher = self.create_publisher(Image,'/camera/raw_frames',10)
        self.bridge = CvBridge()
        # if IP:
        self.cap = cv2.VideoCapture(self.IP)
        # self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.create_timer(0.05,callback = self.timer_callback)
        
        self.get_logger().info("IP Camera Publisher Node has been started.")

    # 3. THE CALLBACK: This runs every time the timer ticks
    def timer_callback(self):
        ret, frame = self.cap.read()

        if ret:
            pub_frame = self.bridge.cv2_to_imgmsg(frame, encoding ="bgr8")
            self.publisher.publish(pub_frame)
            self.get_logger().info('publishing')
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