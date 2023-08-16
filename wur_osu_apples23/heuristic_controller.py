import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, WrenchStamped, TwistStamped

class PickController(Node):
    
    def __init__(self):
        
        super().__init__('pick_controller')
        
        self.goal= 15 #N
        self.max_velocity = 0.1 #dm/s
        self.vel_cmd = Vector3() #dm/s
        
        self.subscription = self.create_subscription(WrenchStamped, '/force_torque_sensor_broadcaster/wrench', self.update_velocity, 10)
        self.publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)

        self.timer = self.create_timer(0.01, self.timer_callback)
        
        self.initialized = False
        self.initial_force = None

        self.running = True
        self.iter = 0
        self.max_iter = 1000

    def change_goal(self, msg):
        
        self.goal = msg.data
        
    def update_velocity(self, msg):
        
        wrench = msg.wrench

        if not self.initialized:
            self.initial_force = np.array([wrench.force.x, wrench.force.y, wrench.force.z])
            self.initialized = True
            self.get_logger().info("initialized")

        current_force = np.array([wrench.force.x, wrench.force.y, wrench.force.z]) - self.initial_force
        f = np.linalg.norm(current_force)
        e_f = f-self.goal
        #self.get_logger().info("current force difference is {} from goal".format(e_f))
        
        n_hat = current_force/ f
        
        t = choose_tangent(n_hat)
        t_hat = t/np.linalg.norm(t)

        #self.get_logger().info("current force is: {}".format(current_force))
        #self.get_logger().info("current force error is: {}".format(e_f))
        #self.get_logger().info("normal velocity proportion is: {}".format(np.tanh(e_f) ))
        #self.get_logger().info("tangent velocity proportion is: {}".format((1-np.tanh(np.abs(e_f)))))
        
        new = self.max_velocity*(np.tanh(e_f) * n_hat + (1-np.tanh(np.abs(e_f))) * t_hat)


        #self.get_logger().info("moving in direction: {}".format(new/np.linalg.norm(new)))
    
        
        self.vel_cmd.x = new[0]
        self.vel_cmd.y = new[1]
        self.vel_cmd.z = new[2]

    def timer_callback(self):

        self.iter = self.iter + 1

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "tool0"

        if self.running:
            msg.twist.linear = self.vel_cmd

        
        if self.iter == self.max_iter:
            self.running = False
            self.iter = 0
            self.get_logger().info("finished")

        self.publisher.publish(msg)
        
def choose_tangent(v):
    
    a = v[0]
    b = v[1]
    c = v[2]
    
    
    candidates = [[-b, a, 0], [0, c, -b], [-c, 0, a]]
    choice = np.argmin([0, -b, a])
    
    selection = np.array(candidates[choice])
    
    return selection

def main():

    rclpy.init()

    node = PickController()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':

    main()