import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, WrenchStamped, TwistStamped, TransformStamped
from std_msgs.msg import Float64
from apple_msgs.srv import SetValue
from std_srvs.srv import Empty
from scipy.spatial.transform import Rotation


class PickController(Node):
    
    def __init__(self):
        
        super().__init__('pick_controller')
        
        self.goal= 0.0 #N
        self.max_velocity = 0.1 # * 0.6 m/s
        self.vel_cmd = Vector3() # * 0.6 m/s
        self.min_tension = 5.0

        self.wrench_subscription = self.create_subscription(WrenchStamped, '/filtered_wrench', self.process_force_meas, 10)
        self.pose_subscription = self.create_subscription(TransformStamped,'/tool_pose', self.remove_gravity, 10)
        
        self.cmd_publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.goal_publisher = self.create_publisher(Float64, '/hc_force_goal', 10)
        self.tangent_publisher = self.create_publisher(Vector3, '/hc_tangent', 10)

        self.timer = self.create_timer(0.01, self.timer_callback)
        
        self.ee_weight = 0.0
        self.force_from_gravity = np.array([0.0, 0.0, 0.0])
        self.last_t = np.array([0.0,0.0,0.0])

        self.running = False

        self.goal_service = self.create_service(SetValue, 'set_goal', self.change_goal)
        self.weight_service = self.create_service(SetValue, 'set_ee_weight', self.log_ee_weight) #N
        self.start_service = self.create_service(Empty, 'start_controller', self.start)
        self.stop_service = self.create_service(Empty, 'stop_controller', self.stop)
        

    ## SERVICES

    def change_goal(self, request, response):
        
        self.goal = request.val
        response.success = True
        return response

    def start(self, request, response):

        self.running = True
        return response

    def stop(self, request, response):

        self.running = False
        self.get_logger().info("finished")
        return response
    
    def log_ee_weight(self, request, response):

         self.ee_weight = request.val
         response.success = True
         return response
     

    ## SUBSCRIBERS & PUBLISHERS

    def process_force_meas(self, msg):
        
        wrench = msg.wrench 

        current_force = np.array([wrench.force.x, wrench.force.y,
                                  wrench.force.z]) - self.force_from_gravity #todo: check dimensions
        
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

            self.cmd_publisher.publish(msg)

            msg2 = Float64()
            msg2.data = self.goal
            self.goal_publisher.publish(msg2)

            msg3 = Vector3()
            msg3.x = self.last_t[0]
            msg3.y = self.last_t[1]
            msg3.z = self.last_t[2]
            self.tangent_publisher.publish(msg3)

    ## HELPERS

    def update_velocity(self, force):

        
        f = np.linalg.norm(force)
        e_f = f-self.goal

        if (np.abs(e_f) <= 1): #np.abs(e_f) <= 0.02 * self.goal) or 
            e_f = 0.0


        n_hat = force/ f
        
        t = self.choose_tangent(n_hat)
        t_hat = t/np.linalg.norm(t)
        
        if f >= self.min_tension:
            new = self.max_velocity*(np.tanh(e_f) * n_hat + 
                                     (1-np.tanh(np.abs(e_f))) * t_hat)    
        else:
            new = [0,0,-1*self.max_velocity]
                
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
    
    def remove_gravity(self, pose_msg):
        
        quat_msg = pose_msg.transform.rotation
        quat_vac = [quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w]
        
        r = Rotation.from_quat(quat_vac)
        
        #rotate force into ee frame
        self.force_from_gravity = np.transpose(np.matmul(r.as_matrix(),np.transpose([0,0,-1*self.ee_weight])))

        
        #option later to use unrotated lever arm and rotated force to get torque
        

def main():

    rclpy.init()

    node = PickController()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':

    main()
