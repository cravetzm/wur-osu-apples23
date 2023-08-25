import rclpy
from rclpy.node import Node

from apple_msgs.srv import Recorder
from std_srvs.srv import Empty

import subprocess
import time

class RecorderNode(Node):
    
    def __init__(self):
        
        super().__init__('recorder')

        start_service = self.create_service(Recorder, 'start_recording', self.start_recording)
        stop_service = self.create_service(Empty, 'stop_recording', self.stop_recording)
        self.p = None

    def start_recording(self, request, response):

        command =  ["ros2", "bag", "record", "--storage", "sqlite3", "-o"]
        command.append(request.bagname)
        command.extend(request.topics)

        self.get_logger().info("sending command: {}".format(command))

        self.p = subprocess.Popen(command)

        return response

    def stop_recording(self, request, response):

        self.p.terminate()
        time.sleep(0.5)
        self.p.kill()

def main():

    rclpy.init()

    node = RecorderNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':

    main()


