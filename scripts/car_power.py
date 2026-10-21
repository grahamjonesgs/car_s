#!/usr/bin/env python3
"""Shut down or reboot the car's Pi from ROS.

Runs on the Pi, started by robot_car.launch.py. Offers two services
(std_srvs/Trigger):

    ros2 service call /car/shutdown std_srvs/srv/Trigger
    ros2 service call /car/reboot std_srvs/srv/Trigger

On a call it disables the motors, replies, and a moment later runs
sudo systemctl poweroff (or reboot), so systemd stops the car_s service
cleanly first. The user running it needs passwordless sudo.
"""
import subprocess

import rclpy
import rclpy.executors
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

DELAY_S = 2.0   # time for the reply to reach the caller before going down


class CarPower(Node):
    def __init__(self):
        super().__init__('car_power')
        self.motor = self.create_publisher(Bool, '/motor_on', 10)
        self.create_service(Trigger, '/car/shutdown',
                            lambda req, res: self.request('poweroff', res))
        self.create_service(Trigger, '/car/reboot',
                            lambda req, res: self.request('reboot', res))
        self.pending = None

    def request(self, action, response):
        if self.pending is not None:
            response.success = False
            response.message = f'Already going to {self.pending}'
            return response
        self.get_logger().warn(f'{action} requested: motors off, then {action} in {DELAY_S:.0f} s')
        self.motor.publish(Bool(data=False))
        self.pending = action
        self.timer = self.create_timer(DELAY_S, self.go)
        response.success = True
        response.message = f'Motors off, {action} in {DELAY_S:.0f} s'
        return response

    def go(self):
        self.timer.cancel()
        self.motor.publish(Bool(data=False))
        result = subprocess.run(['sudo', '-n', 'systemctl', self.pending],
                                capture_output=True, text=True)
        if result.returncode != 0:
            self.get_logger().error(
                f'systemctl {self.pending} failed: {result.stderr.strip()}')
            self.pending = None


def main():
    rclpy.init()
    try:
        rclpy.spin(CarPower())
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
