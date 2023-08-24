import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, WrenchStamped, TwistStamped
from apple_msgs.srv import SetValue
from std_srvs.srv import Empty


class PickController(Node):
    
    def __init__(self):
        
        super().__init__('pick_controller')
        
        self.goal= 0 #N
        self.max_velocity = 0.1 # * 0.6 m/s
        self.vel_cmd = Vector3() # * 0.6 m/s

        self.subscription = self.create_subscription(WrenchStamped, '/force_torque_sensor_broadcaster/wrench', self.proccess_force_meas, 10)
        self.publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)

        self.timer = self.create_timer(0.01, self.timer_callback)
        
        self.ee_weight = np.array([0.0,0.0,0.0])
        self.last_t = np.array([0.0,0.0,0.0])

        self.running = False
        self.iter = 0
        self.max_iter = 1000

        self.goal_service = self.create_service(SetValue, 'set_goal', self.change_goal)
        self.timer_service = self.create_service(SetValue, 'set_timer', self.set_timer)
        self.x_service = self.create_service(SetValue, 'set_ee_x', self.log_ee_x)
        self.y_service = self.create_service(SetValue, 'set_ee_y', self.log_ee_y)
        self.z_service = self.create_service(SetValue, 'set_ee_z', self.log_ee_z)
        self.start_service = self.create_service(Empty, 'start_controller', self.start)
        self.stop_service = self.create_service(Empty, 'stop_controller', self.stop)

    ## SERVICES

    def change_goal(self, request, response):
        
        self.goal = request.val
        response.success = True
        return response

    def set_timer(self, request, response):

        self.max_iter = int(request.val)
        response.success = True
        return response

    def log_ee_x(self, request, response):

         self.ee_weight[0] = request.val
         response.success = True
         return response

    def log_ee_y(self, request, response):

         self.ee_weight[1] = request.val
         response.success = True
         return response

    def log_ee_z(self, request, response):

         self.ee_weight[2] = request.val
         response.success = True
         return response

    def start(self, request, response):

        self.running = True
        return response

    def stop(self, request, response):

        self.running = False
        self.iter = 0
        return response

    ## SUBSCRIBERS & PUBLISHERS

    def proccess_force_meas(self, msg):
        
        wrench = msg.wrench 

        current_force = np.array([wrench.force.x, wrench.force.y, wrench.force.z]) - self.ee_weight
        
        if self.running:
            self.update_velocity(current_force)
        else:
            self.set_initial_tangent(current_force)


    def timer_callback(self):

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "tool0"

        if self.running:
            msg.twist.linear = self.vel_cmd
            self.iter = self.iter + 1
        
            if self.iter == self.max_iter:
                self.running = False
                self.iter = 0
                self.get_logger().info("finished")

            self.publisher.publish(msg)

    ## HELPERS

    def update_velocity(self, force):

        f = np.linalg.norm(force)
        e_f = f-self.goal
        
        n_hat = force/ f
        
        t = self.choose_tangent(n_hat)
        t_hat = t/np.linalg.norm(t)
        
        new = self.max_velocity*(np.tanh(e_f) * n_hat + (1-np.tanh(np.abs(e_f))) * t_hat)    
        
        self.vel_cmd.x = new[0]
        self.vel_cmd.y = new[1]
        self.vel_cmd.z = new[2]

        
    def set_initial_tangent(self, force):

        highest_mag = np.argmax(np.abs(force[0:2]))
        signs = np.sign(force[0:2])

        if highest_mag == 0:
            pre_cross = np.multiply(signs[0], [0.0, 1.0, 0.0])
        else: 
            pre_cross = np.multiply(-1.0 * signs[1], [1.0, 0.0, 0.0])
            
        t = np.cross(pre_cross, force)

        self.last_t = t/np.linalg.norm(t)
    
    def choose_tangent(self, force):

        new_tangent = np.cross(force, np.cross(self.last_t, force))
        self.last_t = new_tangent
        return new_tangent

def main():

    rclpy.init()

    node = PickController()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':

    main()