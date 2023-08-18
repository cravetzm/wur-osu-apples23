import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped, Twist


class SquareMakingController(Node):

    def __init__(self):
        super().__init__('square_maker')
        self.get_clock().now()
        self.publisher_ = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 1)
        self.timer = self.create_timer(0.002, self.timer_callback)
        self.sequence = {"down":"right", "right":"up", "up":"left", "left":"down"}
        self.dir_def = {"down":[0.0,0.1], "right":[0.1,0.0], "up":[0.0,-0.1], "left":[-0.1, 0.0]}
        self.curr_dir = "down"
        self.loops = 0

        self.get_logger().info("Going {}!".format(self.curr_dir))

    def timer_callback(self):

        self.set_direction()

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "tool0"
        msg.twist = self.construct_twist()


        self.publisher_.publish(msg)

    def set_direction(self):

        if self.loops == 3000:
            self.loops = 0
            self.curr_dir = self.sequence[self.curr_dir]
            self.get_logger().info("Going {}!".format(self.curr_dir))

        self.loops = self.loops + 1

    def construct_twist(self):
        
        twist = Twist()
        vals = self.dir_def[self.curr_dir]

        twist.linear.x = vals[0]
        twist.linear.y = vals[1]

        return twist


def main(args=None):
    rclpy.init(args=args)

    robo_artist = SquareMakingController()

    rclpy.spin(robo_artist)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robo_artist.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()