import rclpy
from rclpy.node import Node

from apple_msgs.srv import SetValue, Get3DVect, Recorder
from std_srvs.srv import Empty, Trigger

import time
import subprocess
from datetime import datetime
import csv

class PickManager(Node):

    def __init__(self):

        super().__init__('pick_manager')

        ## Define topics to record here
        ## Subscribe to any services

        self.get_force_cli = self.create_client(Get3DVect, 'get_force')
        self.wait_for_srv(self.get_force_cli)
        self.get_force_req = Get3DVect.Request()

        self.set_goal_cli = self.create_client(SetValue, 'set_goal')
        self.wait_for_srv(self.set_goal_cli)
        self.set_goal_req = SetValue.Request()

        self.start_controller_cli = self.create_client(Empty, 'start_controller')
        self.wait_for_srv(self.start_controller_cli)
        self.start_controller_req = Empty.Request()

        self.stop_controller_cli = self.create_client(Empty, 'stop_controller')
        self.wait_for_srv(self.stop_controller_cli)
        self.stop_controller_req = Empty.Request()

        self.pt_stop_controller_cli  = self.create_client(Empty, 'pull_twist/stop_controller')
        self.wait_for_srv(self.pt_stop_controller_cli)
        self.pt_stop_controller_req = Empty.Request()

        self.pull_twist_cli = self.create_client(Empty, 'pull_twist/start_controller')
        self.wait_for_srv(self.pull_twist_cli)
        self.pull_twist_req = Empty.Request()   

        self.probe_cli = self.create_client(Get3DVect, 'get_probe_position')
        self.wait_for_srv(self.probe_cli)
        self.probe_req = Get3DVect.Request()      

        self.zero_ft_cli = self.create_client(Trigger, '/io_and_status_controller/zero_ftsensor')
        self.wait_for_srv(self.zero_ft_cli)
        self.zero_ft_req = Trigger.Request()  

        # Internal variables
        self.repeat = True

    ## Functions

    def configure_controller(self, goal):

        self.set_goal_req.val = goal
        self.future = self.set_goal_cli.call_async(self.set_goal_req)
        rclpy.spin_until_future_complete(self, self.future)

    def start_controller(self):

        self.future = self.start_controller_cli.call_async(self.start_controller_req)
        rclpy.spin_until_future_complete(self, self.future)

    def stop_controller(self):

        self.future = self.stop_controller_cli.call_async(self.stop_controller_req)
        rclpy.spin_until_future_complete(self, self.future)
        self.future = self.pt_stop_controller_cli.call_async(self.pt_stop_controller_req)
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
            answer = input("Perform another pick? Please enter 'y' for yes or 'n' for no: ")
            if answer == 'y':
                answer_given = True
            elif answer == 'n':
                self.repeat = False
                answer_given = True
            else:
                print("Invalid answer. Try again.")

    def select_controller(self):

        answer_given = False

        while not answer_given:
            answer = input("Please select a controller. Enter 'a' for heuristic controller or 'b' for pull and twist: ")
            if answer == 'a' or answer == 'b':
                answer_given = True
                return answer
            else:
                print("Invalid answer. Try again.")


    def wait_for_srv(self, srv):
        while not srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    
    def run_heuristic_controller(self):

        goal =  15.0 #WUR to set, must be more than pre-tension force

        print("Setting goal to {} [N].".format(goal))

        self.configure_controller(goal)
        self.start_controller()

        selection = input("Press ENTER to stop controller.")
        self.stop_controller()
        time.sleep(0.1)

    def run_pull_twist(self):
        
        response = ''

        self.future = self.pull_twist_cli.call_async(self.pull_twist_req)
        rclpy.spin_until_future_complete(self, self.future)

        selection = input("Press ENTER to stop controller.")
        self.stop_controller()
        time.sleep(0.1)
        

    def zero_ft(self):

        self.future = self.zero_ft_cli.call_async(self.zero_ft_req)
        rclpy.spin_until_future_complete(self, self.future)

    ## Main Loop

    def loop(self):

        while self.repeat:

            input("Press ENTER to begin pick")
            
            input("Please drive the robot to the apple and perform a grasp. When finished, press ENTER.")
            input("Press play on the teach pendant, then press ENTER.")
            
            print("Thank you. Now beginning.")


            time.sleep(1)

            controller = self.select_controller()


            print("Initiating controller.")
            
            if controller == 'a':
                self.run_heuristic_controller()
            elif controller == 'b':
                self.run_pull_twist()

            print("Finished controller sequence. Shutting down rosbag recording.")

            self.continue_or_quit()
            


def main():

    rclpy.init()

    node = PickManager()

    node.loop()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':

    main()