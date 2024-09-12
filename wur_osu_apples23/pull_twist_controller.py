import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, TwistStamped
from std_srvs.srv import Empty


class PTController(Node):
    
    def __init__(self):
        
        super().__init__('pull_twist_controller')
        
        self.max_velocity = 0.1 # * 0.6 m/s
        self.vel_cmd = Vector3() # * 0.6 m/s

        self.publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)

        self.timer = self.create_timer(0.01, self.timer_callback)
        
        self.running = False

        self.start_service = self.create_service(Empty, 'pull_twist/start_controller', self.start)
        self.stop_service = self.create_service(Empty, 'pull_twist/stop_controller', self.stop)

    ## SERVICES

    def start(self, request, response):

        self.running = True
        return response

    def stop(self, request, response):

        self.running = False
        self.iter = 0
        return response

    ## SUBSCRIBERS & PUBLISHERS


    def timer_callback(self):

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "tool0"

        if self.running:

            msg.twist.linear.z = -1*self.max_velocity
            msg.twist.angular.z = 1.0
            
            self.publisher.publish(msg)

def main():

    rclpy.init()

    node = PTController()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':

    main()