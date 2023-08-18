import rclpy
from rclpy.node import Node

from apple_msgs.srv import Get3DVect
from geometry_msgs.msg import WrenchStamped

import numpy as np

class Listener(Node):
    
    def __init__(self):
        super().__init__('measurement_listener')

        self.force_service = self.create_service(SetValue, 'get_force', self.send_force)
        self.current_force = np.array([0.0,0.0,0.0])
        self.force_listener = self.create_subscription(WrenchStamped, '/force_torque_sensor_broadcaster/wrench', self.log_force, 10)

    def send_force(self, request, response):
        
        response.x = self.current_force[0]
        response.y = self.current_force[1]
        response.z = self.current_force[2]

        return response

    def log_force(self, msg):

        wrench = msg.wrench 
        current_force = np.array([wrench.force.x, wrench.force.y, wrench.force.z]) - self.ee_weight
        

        
