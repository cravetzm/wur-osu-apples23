import rclpy
from rclpy.node import Node

from apple_msgs.srv import SetValue, Get3DVect
from std_srvs.srv import Empty

import time
import subprocess
from datetime import datetime
import csv

class PickManager(Node):

    def __init__(self):

        super().__init__('pick_manager')

        ## Define topics to record here

        self.to_record = []
        imu_topics = ['/gyro0', '/gyro1', '/gyro2', \
        '/accel0', '/accel1', '/accel2', \
        '/mag0', '/mag1', '/mag2', \
        '/orient0', '/orient1', '/orient2']
        controller_topics = ['/servo_node/delta_twist_cmds']
        robot_topics = ['/force_torque_sensor_broadcaster/wrench']

        #self.to_record.extend(imu_topics)
        self.to_record.extend(controller_topics)
        self.to_record.extend(robot_topics)

        self.get_logger().info("topics to record: {}".format(self.to_record))

        ## Subscribe to any services

        self.get_force_cli = self.create_client(Get3DVect, 'get_force')
        while not self.get_force_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service get_force not available, waiting again...')
        self.get_force_req = Get3DVect.Request()

        self.set_goal_cli = self.create_client(SetValue, 'set_goal')
        while not self.set_goal_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service set_goal not available, waiting again...')
        self.set_goal_req = SetValue.Request()
        
        self.set_timer_cli = self.create_client(SetValue, 'set_timer')
        while not self.set_timer_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service set_timer not available, waiting again...')
        self.set_timer_req = SetValue.Request()

        self.set_ee_x_cli = self.create_client(SetValue, 'set_ee_x')
        while not self.set_ee_x_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service set_ee_x not available, waiting again...')
        self.set_ee_x_req = SetValue.Request()

        self.set_ee_y_cli = self.create_client(SetValue, 'set_ee_y')
        while not self.set_ee_y_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service set_ee_y not available, waiting again...')
        self.set_ee_y_req = SetValue.Request()

        self.set_ee_z_cli = self.create_client(SetValue, 'set_ee_z')
        while not self.set_ee_z_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service set_ee_z not available, waiting again...')
        self.set_ee_z_req = SetValue.Request()

        self.start_controller_cli = self.create_client(Empty, 'start_controller')
        while not self.start_controller_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service start_controller not available, waiting again...')
        self.start_controller_req = Empty.Request()

        self.stop_controller_cli = self.create_client(Empty, 'stop_controller')
        while not self.stop_controller_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service stop_controller not available, waiting again...')
        self.stop_controller_req = Empty.Request()

        # Internal variables
        self.repeat = True

    ## Functions

    def configure_controller(self, goal, timer, force):

        self.set_goal_req.val = goal
        self.future = self.set_goal_cli.call_async(self.set_goal_req)
        rclpy.spin_until_future_complete(self, self.future)

        self.set_timer_req.val = timer
        self.future = self.set_timer_cli.call_async(self.set_timer_req)
        rclpy.spin_until_future_complete(self, self.future)

        self.set_ee_x_req.val = force[0]
        self.future = self.set_ee_x_cli.call_async(self.set_ee_x_req)
        rclpy.spin_until_future_complete(self, self.future)

        self.set_ee_y_req.val = force[1]
        self.future = self.set_ee_y_cli.call_async(self.set_ee_y_req)
        rclpy.spin_until_future_complete(self, self.future)

        self.set_ee_z_req.val = force[2]
        self.future = self.set_ee_z_cli.call_async(self.set_ee_z_req)
        rclpy.spin_until_future_complete(self, self.future)

    def start_controller(self):

        self.future = self.start_controller_cli.call_async(self.start_controller_req)
        rclpy.spin_until_future_complete(self, self.future)

    def measure_force(self):

        self.future = self.get_force_cli.call_async(self.get_force_req)
        rclpy.spin_until_future_complete(self, self.future)
        response = self.future.result()
        force = [response.x, response.y, response.z]
        return force

    def continue_or_quit(self):

        answer_given = False

        while not answer_given:
            answer = input("Perform another pick? Please enter 'y' for yes or 'n' for no")
            if answer == 'y':
                answer_given = True
            elif answer == 'n':
                self.repeat = False
                answer_given = True
            else:
                print("Invalid answer. Try again")

    
    def write_csv(self, data, name):
        header = ["imu 1 location", "imu 2 location", "imu 3 location", \
        "abscission layer location", "branch p1 location", "branch p2 location", "branch p3 location", \
        "branch d1", "branch d2", "branch d3", "dropped fruit"]
        csv_name = name + "_metadata.csv"

        with open(csv_name, 'wb') as f:
            writer = csv.writer(f)

            writer.writerow(header)
            writer.writerow(data)

    def run_heuristic_controller(self, ee_weight):

            goals = [0.0, 1.0, 0.0, 5.0, 0.0, 10.0, 0.0, 15.0]
            times = [1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0]

            for i in range(len(goals)):

                print("Setting goal to {} [N] for {} [s].".format(goals[i], times[i]/100))

                self.configure_controller(goals[i], times[i], ee_weight)
                self.start_controller()
                time.sleep(times[i]/100) #There has to be a more elegant way to do this but I can't figure it out.

    ## Main Loop

    def loop(self):

        while self.repeat:

            timestamp = datetime.now().strftime("%m-%d-%Y-%H-%M-%S")

            imu_orientations = []
            dropped = 0
            csv_data = []

            print("Let's begin the logging process. We will measure some static or initial values.")
            # todo: probe IMU locations
            # todo: probe abscission layer
            # todo: probe branch points
            # todo: measure branch diameter

            print("Recording initial orientation of the IMU. Please do not perturb branch. This process takes 10 seconds.")

            time.sleep(10)

            #todo: log IMU orientations (requires new services in listener)

            input("Please position end effector for approach, then press ENTER.")

            ee_weight = self.measure_force()

            input("Please drive the robot to the apple and perform a grasp. Do not change end effector orientation. When finished, press ENTER.")

            print("Thank you. Now beginning recording.")

            cmd = ["ros2", "bag", "record", "-o", timestamp]
            cmd.extend(self.to_record)
            print(cmd)
            p = subprocess.Popen(cmd)

            print("Initiating controller. Goal force is 0[N].")

            self.run_heuristic_controller(ee_weight)

            print("Finished controller sequence. Shutting down rosbag recording.")

            p.terminate()
            time.sleep(0.5)
            p.kill()
            time.sleep(0.5)

            dropped = input("Enter number of dropped fruits.")

            self.write_csv(csv_data, timestamp)

            self.continue_or_quit()
            


def main():

    rclpy.init()

    node = PickManager()

    node.loop()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':

    main()