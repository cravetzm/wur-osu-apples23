import subprocess
import time

def main():

    p = subprocess.Popen(["ros2", "bag", "record", "-a"])
    time.sleep(10)
    p.terminate()
    time.sleep(0.5)
    p.kill()

if __name__ == '__main__':

    main()