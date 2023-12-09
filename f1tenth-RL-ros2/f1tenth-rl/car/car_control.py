import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

from threading import Thread
import time
import argparse

try:
    from geometry_msgs.msg import PoseWithCovarianceStamped
except ImportError:
    pass

try:
    from car.sensors import Sensors
except ImportError:
    from sensors import Sensors

PUBLISHER_WAIT = 0.005

MAX_SPEED_REDUCTION = 4.5
STEERING_SPEED_REDUCTION = 4.5
BACKWARD_SPEED_REDUCTION = 4.5
LIGHTLY_STEERING_REDUCTION = 2.4
BACKWARD_SECONDS = 1.5

MAX_SPEED_REDUCTION_SIM = 1
STEERING_SPEED_REDUCTION_SIM = 1.4
BACKWARD_SPEED_REDUCTION_SIM = 3
LIGHTLY_STEERING_REDUCTION_SIM = 2.4
BACKWARD_SECONDS_SIM = 1.5
USE_RESET_INSTEAD_OF_BACKWARDS_SIM = False

MIN_SPEED_REDUCTION = 5

class Drive(Node):
    def __init__(self, sensors, is_simulator=False):
        super().__init__('drive')
        self.is_simulator = is_simulator
        if not is_simulator:
            topic = "/vesc/high_level/ackermann_cmd_mux/input/nav_0"
            max_steering = 0.34
            self.max_speed_reduction = MAX_SPEED_REDUCTION
            self.steering_speed_reduction = STEERING_SPEED_REDUCTION
            self.backward_speed_reduction = BACKWARD_SPEED_REDUCTION
            self.lightly_steering_reduction = LIGHTLY_STEERING_REDUCTION
            self.backward_seconds = BACKWARD_SECONDS
        else:
            topic = "/drive"
            max_steering = 0.4189
            self.max_speed_reduction = MAX_SPEED_REDUCTION_SIM
            self.steering_speed_reduction = STEERING_SPEED_REDUCTION_SIM
            self.backward_speed_reduction = BACKWARD_SPEED_REDUCTION_SIM
            self.lightly_steering_reduction = LIGHTLY_STEERING_REDUCTION_SIM
            self.backward_seconds = BACKWARD_SECONDS_SIM
            self.reset_publisher = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        # self.declare_parameter("max_speed", rclpy.Parameter.Type.DOUBLE)
        # self.declare_parameter("max_steering", rclpy.Parameter.Type.DOUBLE)
        self.max_speed = 5.0
        self.max_steering = 0.4186
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, topic, 10)
        self.sensors = sensors
        self.stop()
        process = Thread(target=self.drive_command_runner)
        process.daemon = True
        process.start()
        print("max_speed: ", self.max_speed, ", max_steering: ", self.max_steering)

    def forward(self):
        self.send_drive_command(self.max_speed/self.max_speed_reduction, 0.0)
    
    def backward(self):
        self.send_drive_command(-self.max_speed/self.backward_speed_reduction, 0.0)
    
    def stop(self):
        self.send_drive_command(0.0, 0.0)
    
    def right(self):
        self.send_drive_command(self.max_speed/self.steering_speed_reduction, -self.max_steering)

    def left(self):
        self.send_drive_command(self.max_speed/self.steering_speed_reduction, self.max_steering)

    def lightly_right(self):
        self.send_drive_command(self.max_speed/self.steering_speed_reduction, -self.max_steering/self.lightly_steering_reduction)

    def lightly_left(self):
        self.send_drive_command(self.max_speed/self.steering_speed_reduction, self.max_steering/self.lightly_steering_reduction)

    def slowdown(self):
        speed = self.last_speed/2 if self.last_speed/2 > self.max_speed/MIN_SPEED_REDUCTION else self.max_speed/MIN_SPEED_REDUCTION
        self.send_drive_command(speed, self.last_angle)

    def send_drive_command(self, speed, steering_angle):
        self.last_angle = steering_angle
        self.last_speed = speed
        ack_msg = AckermannDriveStamped()
        ack_msg.drive.speed = speed
        ack_msg.drive.steering_angle = steering_angle
        self.ack_msg = ack_msg

    def drive_command_runner(self):
        while True:
            self.drive_publisher.publish(self.ack_msg)
            time.sleep(PUBLISHER_WAIT)

    def backward_until_obstacle(self):
        if USE_RESET_INSTEAD_OF_BACKWARDS_SIM and self.is_simulator:
            self.reset_simulator()
        else:
            self.backward()
            start = time.time()
            while not self.sensors.back_obstacle() and time.time() - start < self.backward_seconds:
                time.sleep(0.01)
            self.stop()
            time.sleep(0.1)


    def reset_simulator(self):
        if self.is_simulator:
            self.reset_publisher.publish(PoseWithCovarianceStamped())
    
    def main(self):
        parser = argparse.ArgumentParser()
        parser.add_argument("--simulator", action='store_true', help="to set the use of the simulator")
        args = parser.parse_args()
        # drive = Drive(args.simulator)
        # rclpy.spin_once(self)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--simulator", action='store_true', help="to set the use of the simulator")
    args = parser.parse_args()

    run_seconds = 0.3
    rclpy.init(args=None)
    drive = Drive(args.simulator)
    rclpy.spin(drive)
    # while True:
    #     print("Write command")
    #     cmd = input()
    #     start = time.time()
    #     if cmd == "w":
    #         while time.time() - start < run_seconds:
    #             drive.forward()
    #     if cmd == "s":
    #         while time.time() - start < run_seconds:
    #             drive.backward()
    #     if cmd == "a":
    #         while time.time() - start < run_seconds:
    #             drive.lightly_left()
    #     if cmd == "d":
    #         while time.time() - start < run_seconds:
    #             drive.lightly_right()
    #     if cmd == "aa":
    #         while time.time() - start < run_seconds:
    #             drive.left()
    #     if cmd == "dd":
    #         while time.time() - start < run_seconds:
    #             drive.right()
    #     if cmd == " ":
    #         while time.time() - start < run_seconds:
    #             drive.stop()
    #     if cmd == "buo":
    #         drive.backward_until_obstacle()
    #     if cmd == "q":
    #         exit()
