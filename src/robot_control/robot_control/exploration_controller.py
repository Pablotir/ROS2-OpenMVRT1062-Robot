#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

class ExplorationController(Node):
    def __init__(self):
        super().__init__('exploration_controller')
        
        self.direction_sub = self.create_subscription(
            String,
            '/ai/direction',
            self.on_direction,
            10
        )
        
        self.motor_plan_pub = self.create_publisher(String, '/llm/motor_plan', 10)
        self.movement_complete_pub = self.create_publisher(Bool, '/robot/movement_complete', 10)
        
        self.get_logger().info("Exploration Controller started")
        
        self.movement_duration = 3.0
        self.completion_timer = None
    
    def on_direction(self, msg: String):
        direction = msg.data.strip().lower()
        
        if 'forward' in direction:
            self.get_logger().info("Moving forward to explore")
            plan = "1,80,2,-80,3,80,4,-80"
        elif 'left' in direction:
            self.get_logger().info("Turning left")
            plan = "1,-60,2,-60,3,-60,4,-60"
        elif 'right' in direction:
            self.get_logger().info("Turning right")
            plan = "1,60,2,60,3,60,4,60"
        else:
            self.get_logger().info("Moving forward (default)")
            plan = "1,80,2,-80,3,80,4,-80"
        
        plan_msg = String()
        plan_msg.data = plan
        self.motor_plan_pub.publish(plan_msg)
        
        if self.completion_timer is not None:
            self.completion_timer.cancel()
        
        self.completion_timer = self.create_timer(self.movement_duration, self.signal_movement_complete)
    
    def signal_movement_complete(self):
        self.get_logger().info("Movement duration elapsed, signaling completion")
        
        if self.completion_timer is not None:
            self.completion_timer.cancel()
            self.completion_timer = None
        
        complete_msg = Bool()
        complete_msg.data = True
        self.movement_complete_pub.publish(complete_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ExplorationController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
