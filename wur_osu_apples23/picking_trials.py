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

        self.to_record = []
        imu_topics = ['/gyro0', '/gyro1', '/gyro2', \
        '/accel0', '/accel1', '/accel2', \
        '/mag0', '/mag1', '/mag2', \
        '/orient0', '/orient1', '/orient2']
        controller_topics = ['/v_cmd_unfiltered', '/hc_force_goal', '/servo_node/delta_twist_cmds', '/filtered_wrench', '/hc_tangent']
        robot_topics = ['/force_torque_sensor_broadcaster/wrench', '/tool_pose']

        self.to_record.extend(imu_topics)
        self.to_record.extend(controller_topics)
        self.to_record.extend(robot_topics)

        self.get_logger().info("topics to record: {}".format(self.to_record))

        ## Subscribe to any services

        self.get_force_cli = self.create_client(Get3DVect, 'get_force')
        self.wait_for_srv(self.get_force_cli)
        self.get_force_req = Get3DVect.Request()

        self.set_goal_cli = self.create_client(SetValue, 'set_goal')
        self.wait_for_srv(self.set_goal_cli)
        self.set_goal_req = SetValue.Request()
        
        self.set_timer_cli = self.create_client(SetValue, 'set_timer')
        self.wait_for_srv(self.set_timer_cli)
        self.set_timer_req = SetValue.Request()

        self.set_ee_x_cli = self.create_client(SetValue, 'set_ee_x')
        self.wait_for_srv(self.set_ee_x_cli)
        self.set_ee_x_req = SetValue.Request()

        self.set_ee_y_cli = self.create_client(SetValue, 'set_ee_y')
        self.wait_for_srv(self.set_ee_y_cli)
        self.set_ee_y_req = SetValue.Request()

        self.set_ee_z_cli = self.create_client(SetValue, 'set_ee_z')
        self.wait_for_srv(self.set_ee_z_cli)
        self.set_ee_z_req = SetValue.Request()

        self.start_controller_cli = self.create_client(Empty, 'start_controller')
        self.wait_for_srv(self.set_ee_z_cli)
        self.start_controller_req = Empty.Request()

        self.stop_controller_cli = self.create_client(Empty, 'stop_controller')
        self.wait_for_srv(self.stop_controller_cli)
        self.stop_controller_req = Empty.Request()

        self.pull_twist_cli = self.create_client(Empty, 'pull_twist/start_controller')
        self.wait_for_srv(self.pull_twist_cli)
        self.pull_twist_req = Empty.Request()

        self.get_imu1_orientation_cli = self.create_client(Get3DVect, 'get_imu1_orientation')
        self.wait_for_srv(self.get_imu1_orientation_cli)
        self.get_imu1_orientation_req = Get3DVect.Request()

        self.get_imu2_orientation_cli = self.create_client(Get3DVect, 'get_imu2_orientation')
        self.wait_for_srv(self.get_imu2_orientation_cli)
        self.get_imu2_orientation_req = Get3DVect.Request()

        self.get_imu3_orientation_cli = self.create_client(Get3DVect, 'get_imu3_orientation')
        self.wait_for_srv(self.get_imu3_orientation_cli)
        self.get_imu3_orientation_req = Get3DVect.Request()

        self.start_recording_cli = self.create_client(Recorder, 'start_recording')
        self.wait_for_srv(self.start_recording_cli)
        self.start_recording_req = Recorder.Request()

        self.stop_recording_cli = self.create_client(Empty, 'stop_recording')
        self.wait_for_srv(self.stop_recording_cli)
        self.stop_recording_req = Empty.Request()        

        self.probe_cli = self.create_client(Get3DVect, 'get_probe_position')
        self.wait_for_srv(self.probe_cli)
        self.probe_req = Get3DVect.Request()      

        self.zero_ft_cli = self.create_client(Trigger, '/io_and_status_controller/zero_ftsensor')
        self.wait_for_srv(self.zero_ft_cli)
        self.zero_ft_req = Trigger.Request()  

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

    def stop_controller(self):

        self.future = self.stop_controller_cli.call_async(self.stop_controller_req)
        rclpy.spin_until_future_complete(self, self.future)

    def measure_force(self):

        self.future = self.get_force_cli.call_async(self.get_force_req)
        rclpy.spin_until_future_complete(self, self.future)
        response = self.future.result()
        force = [response.x, response.y, response.z]
        return force

    def poll_imus(self):

        self.future = self.get_imu1_orientation_cli.call_async(self.get_imu1_orientation_req)
        rclpy.spin_until_future_complete(self, self.future)
        response = self.future.result()
        imu1 = [response.x, response.y, response.z]

        self.future = self.get_imu2_orientation_cli.call_async(self.get_imu2_orientation_req)
        rclpy.spin_until_future_complete(self, self.future)
        response = self.future.result()
        imu2 = [response.x, response.y, response.z]

        self.future = self.get_imu3_orientation_cli.call_async(self.get_imu3_orientation_req)
        rclpy.spin_until_future_complete(self, self.future)
        response = self.future.result()
        imu3 = [response.x, response.y, response.z]

        return [imu1, imu2, imu3]

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

    
    def write_csv(self, data, name):
        header = ["imu 1 location", "imu 2 location", "imu 3 location", "imu 1 orientation", "imu 2 orientation", "imu 3 orientation", "abscission layer location", "dropped fruit", "ee weight"]
        csv_name = name + "_metadata.csv"

        with open(csv_name, 'w') as f:
            writer = csv.writer(f)

            writer.writerow(header)
            writer.writerow(data)

    def wait_for_srv(self, srv):
        while not srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    
    def run_heuristic_controller(self, ee_weight):

        goals = [0.0, 5.0, 0.0, 10.0, 0.0, 15.0]
        times = [1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0]

        selection = ''

        for i in range(len(goals)):

            if selection != 's':

                print("Setting goal to {} [N] for {} [s].".format(goals[i], times[i]/100))

                self.configure_controller(goals[i], times[i], ee_weight)
                self.start_controller()

                selection = input("Press ENTER when controller finishes to continue sequence. Or, enter 's' to stop sequence.")
                self.stop_controller()
                time.sleep(0.1)

            else:
                pass

    def run_pull_twist(self):
        
        response = ''

        while response != 'n':
            self.future = self.pull_twist_cli.call_async(self.pull_twist_req)
            rclpy.spin_until_future_complete(self, self.future)
            time.sleep(2)

            response_given = False

            while not response_given:
                response = input("Try again? Enter 'y' for yes or 'n' for no: ")
                if response == 'y' or response == 'n':
                    response_given = True
        
    def start_recording(self, name):

        self.start_recording_req.topics = self.to_record
        self.start_recording_req.bagname = name

        self.future = self.start_recording_cli.call_async(self.start_recording_req)
        rclpy.spin_until_future_complete(self, self.future)


    def stop_recording(self):
        self.future = self.stop_recording_cli.call_async(self.stop_recording_req)
        rclpy.spin_until_future_complete(self, self.future)

    def probe_point(self):

        self.future = self.probe_cli.call_async(self.probe_req)
        rclpy.spin_until_future_complete(self, self.future)
        response = self.future.result()
        point = [response.x, response.y, response.z]
        
        return point

    def zero_ft(self):

        self.future = self.zero_ft_cli.call_async(self.zero_ft_req)
        rclpy.spin_until_future_complete(self, self.future)

    ## Main Loop

    def loop(self):

        while self.repeat:

            input("Press ENTER to begin pick")

            timestamp = datetime.now().strftime("%m-%d-%Y-%H-%M-%S")

            imu_orientations = []
            imu_locations = []
            dropped = 0
            abscission_layer = []
            csv_data = []

            print("Let's begin the logging process. We will measure some static or initial values.")
            
            for i in range(3):
                input("Place probe at IMU {} location. Press ENTER to record point".format(i + 1))
                imu_locations.append(self.probe_point())


            input("Place probe at abscission layer location. Press ENTER to record point".format(i + 1))
            abscission_layer = self.probe_point()
            
            input("Record the branch diameter now. Then, press ENTER to proceed.")

            print("Recording initial orientation of the IMU. Please do not perturb branch. This process takes 10 seconds.")

            time.sleep(10)

            imu_orientations = self.poll_imus()
            
            input("Please position end effector for approach, then press ENTER.")

#            self.zero_ft()
#            time.sleep(1)
            ee_weight = self.measure_force()

            input("Please drive the robot to the apple and perform a grasp. Do not change end effector orientation. When finished, press ENTER.")
            input("Press play on the teach pendant, then press ENTER.")
            
            print("Thank you. Now beginning recording.")
            self.start_recording(timestamp)


            time.sleep(1)

            controller = self.select_controller()


            print("Initiating controller.")
            
            if controller == 'a':
                self.run_heuristic_controller(ee_weight)
            elif controller == 'b':
                self.run_pull_twist()

            print("Finished controller sequence. Shutting down rosbag recording.")

            self.stop_recording()

            dropped = input("Enter number of dropped fruits.")
            if len(imu_locations) != 0:
                csv_data.extend(imu_locations)
            else:
                csv_data.extend([[],[],[]])

            if len(imu_orientations) != 0:
                csv_data.extend(imu_orientations)
            else:
                csv_data.extend([[],[],[]])

            csv_data.append(abscission_layer)
            csv_data.append(dropped)
            csv_data.append(ee_weight)

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