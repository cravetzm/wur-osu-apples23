import rclpy
from rclpy.node import Node

import numpy as np

from geometry_msgs.msg import WrenchStamped, TransformStamped
from std_msgs.msg import Bool
from scipy.spatial.transform import Rotation

import time
import copy

class SpringApple(Node):
    
    def __init__(self):

        super().__init__('wrench_simulator')
        
        self.position = np.zeros(shape=(3,1), dtype = float)
        self.R = []
        self.current_force = np.zeros(shape=(3,1), dtype = float)
        self.get_logger().info("force array initiated to: {}".format(self.current_force))

        self.measured_force = np.zeros(shape=(3,1), dtype = float)
        
        self.apple_rad = 0.05
        self.stem_h = 0.05     
        self.history= {"force": [], "position": [], "velocity": []}

        self.subscriber = self.create_subscription(TransformStamped, '/tool_pose', self.update_pose, 10)
        self.publisher = self.create_publisher(WrenchStamped, '/wrench', 10)
        self.status_publisher = self.create_publisher(Bool, '/sim_status', 10)

        self.status = False

        self.equilibrium = copy.copy(self.position)

        self.l_effective = self.apple_rad+self.stem_h

        self.anchor_point = copy.copy(self.equilibrium)

        self.stiffness = 600        

        self.timer = self.create_timer(0.1, self.timer_callback)

    def update_pose(self, msg):

        self.position[0] = msg.transform.translation.x
        self.position[1] = msg.transform.translation.y
        self.position[2] = msg.transform.translation.z

        #self.get_logger().info("ee coordinates: {}".format(self.position))


        if not self.status:

            quat = [msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w]
            rot = Rotation.from_quat(quat)
            self.R = rot.inv().as_matrix()

            self.equilibrium = copy.copy(self.position)
            self.get_logger().info("starting coordinates: {}".format(self.equilibrium))

            offset = np.zeros(shape=(3,1), dtype = float)
            offset[2] = self.l_effective + 0.05
            self.get_logger().info("offset is: {}".format(offset))
            self.anchor_point = copy.copy(self.equilibrium)+offset
            self.get_logger().info("anchor point set to: {}".format(self.anchor_point))

            self.status = True
        
    def apple_dynamics(self):
        
        spring_extension = self.anchor_point - self.position
        spring_length = np.linalg.norm(spring_extension)
        force_dir = spring_extension/spring_length
        
        self.current_force = self.stiffness*(spring_length-self.l_effective)*force_dir
        self.measured_force = self.add_noise(self.rotate_force())

        #debugging
        displacement = self.position - self.equilibrium

        self.get_logger().info("R = {}".format(self.R))

        self.get_logger().info("current position is {}".format(self.position))
        self.get_logger().info("anchor point is {}".format(self.anchor_point))
        self.get_logger().info("vector to anchor point is {}".format(spring_extension))
        self.get_logger().info("displacement is {}".format(displacement))
        self.get_logger().info("current force is {} (should be zero)".format(self.current_force))
        self.get_logger().info("measured force is {} (should be small)".format(self.measured_force))


    def timer_callback(self):

        if self.status:
            self.apple_dynamics()

            out = WrenchStamped()

            #self.get_logger().info("measured_force is {}".format(self.measured_force))

            out.wrench.force.x = self.measured_force.item(0)
            out.wrench.force.y = self.measured_force.item(1)
            out.wrench.force.z = self.measured_force.item(2)

            self.publisher.publish(out)

        stat_out = Bool()
        stat_out.data = self.status

        self.status_publisher.publish(stat_out)

    def rotate_force(self):

        to_rotate = self.current_force
        rotated = np.matmul(self.R,to_rotate)

        return rotated


    def add_noise(self,force):

        noise = np.random.normal(0.0, 0.09, [3,1])
        force = force + noise

        return force


def main():

    rclpy.init()

    node = SpringApple()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':

    main()