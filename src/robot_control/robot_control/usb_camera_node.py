#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2

class USBCameraNode(Node):
    def __init__(self):
        super().__init__('usb_camera_node')
        
        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('width', 320)
        self.declare_parameter('height', 240)
        
        device = self.get_parameter('device').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        
        # Publisher for frames
        self.image_pub = self.create_publisher(Image, '/camera/usb_raw', 10)
        
        # Subscriber for frame requests
        self.request_sub = self.create_subscription(
            Bool, 
            '/camera/request_frame', 
            self.on_frame_request, 
            10
        )
        
        self.bridge = CvBridge()
        
        # Open USB camera
        self.cap = cv2.VideoCapture(device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera at {device}")
            return
        
        self.get_logger().info(f"USB camera ready: {device} @ {width}x{height}")
        self.get_logger().info("Waiting for frame requests on /camera/request_frame")
        
        self.frame_count = 0
    
    def on_frame_request(self, msg: Bool):
        """Capture and publish a frame when requested"""
        if not msg.data:
            return  # Ignore false requests
        
        ret, frame = self.cap.read()
        
        if ret:
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = 'usb_camera'
            
            self.image_pub.publish(ros_image)
            
            self.frame_count += 1
            self.get_logger().info(f"Captured frame {self.frame_count} on request")
        else:
            self.get_logger().warn("Failed to capture frame")
    
    def destroy_node(self):
        if self.cap:
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = USBCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
