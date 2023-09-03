import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from apple_msgs.srv import Get3DVect
from geometry_msgs.msg import WrenchStamped, Vector3, TransformStamped

import numpy as np

class Listener(Node):
    
    def __init__(self):
        super().__init__('measurement_listener')

        self.force_service = self.create_service(Get3DVect, 'get_force', self.send_force)
        self.current_force = np.array([0.0,0.0,0.0])
        self.force_listener = self.create_subscription(WrenchStamped, '/force_torque_sensor_broadcaster/wrench', self.log_force, 10)

        self.imu1 = np.array([0.0,0.0,0.0])
        self.imu2 = np.array([0.0,0.0,0.0])
        self.imu3 = np.array([0.0,0.0,0.0])

        self.imu1_service = self.create_service(Get3DVect, 'get_imu1_orientation', self.send_orientation_1)
        self.imu1_listener = self.create_subscription(Vector3, '/orient0', self.log_orientation_1, qos_profile = qos_profile_sensor_data)

        self.imu2_service = self.create_service(Get3DVect, 'get_imu2_orientation', self.send_orientation_2)
        self.imu2_listener = self.create_subscription(Vector3, '/orient1', self.log_orientation_2, qos_profile = qos_profile_sensor_data)

        self.imu3_service = self.create_service(Get3DVect, 'get_imu3_orientation', self.send_orientation_3)
        self.imu3_listener = self.create_subscription(Vector3, '/orient2', self.log_orientation_3, qos_profile = qos_profile_sensor_data)

        self.probe_service = self.create_service(Get3DVect, 'get_probe_position', self.probe)
        self.probe_listener = self.create_subscription(TransformStamped, '/probe_pose', self.log_pose, 10)

        self.probe_position = np.array([0.0,0.0,0.0])

    def send_force(self, request, response):
        
        response.x = self.current_force[0]
        response.y = self.current_force[1]
        response.z = self.current_force[2]

        return response

    def send_orientation_1(self, request, response):
        
        response.x = self.imu1[0]
        response.y = self.imu1[1]
        response.z = self.imu1[2]

        return response

    def send_orientation_2(self, request, response):
        
        response.x = self.imu2[0]
        response.y = self.imu2[1]
        response.z = self.imu2[2]

        return response

    def send_orientation_3(self, request, response):
        
        response.x = self.imu3[0]
        response.y = self.imu3[1]
        response.z = self.imu3[2]

        return response

    def probe(self,request, response):

        response.x = self.probe_position[0]
        response.y = self.probe_position[1]
        response.z = self.probe_position[2]
        
        return response

    def log_force(self, msg):

        wrench = msg.wrench 
        self.current_force = np.array([wrench.force.x, wrench.force.y, wrench.force.z])
        
    def log_orientation_1(self, msg):

        self.imu1 = np.array([msg.x, msg.y, msg.z])
        
    def log_orientation_2(self, msg):

        self.imu2 = np.array([msg.x, msg.y, msg.z])

    def log_orientation_3(self, msg):
        
        self.imu3 = np.array([msg.x, msg.y, msg.z])

    def log_pose(self, msg):

        position = msg.transform.translation
        self.probe_position = np.array([position.x, position.y, position.z])
        

def main():

    rclpy.init()

    node = Listener()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':

    main()