#!/usr/bin/env python3

# 1. IMPORTS: Bring in the tools you need
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image  # The message type for sending images
from cv_bridge import CvBridge     # The translator between OpenCV and ROS
import cv2                         # OpenCV itself

# 2. CLASS DEFINITION: Every node is a class that inherits from 'Node'
class IpCamRead(Node):
    
    def __init__(self):
        # Name this node 'ip_cam_publisher' in the ROS2 graph
        super().__init__('ip_cam_listener')
        
        self.bridge = CvBridge()
        self.subscriber = self.create_subscription(Image,'/camera/raw_frames' , self.listener_callback,10)
        self.get_logger().info("IP Camera subscriber Node has been started.")

    # 3. THE CALLBACK: This runs every time the timer ticks
    def listener_callback(self,msg):
        frame = self.bridge.imgmsg_to_cv2(msg)

        h,w,c = frame.shape
        self.get_logger().info(f'frame shape is {h} - {w} - {c}')
        frame = cv2.resize(frame,(640,480))
        cv2.imshow("camera", frame)
        cv2.waitKey(1)
        # else:
        #     self.get_logger().error('something bad has happened cant show images')


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