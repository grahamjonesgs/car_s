#!/usr/bin/env python3
"""Shut down or reboot the car's Pi from ROS.

Runs on the Pi, started by robot_car.launch.py. Offers two services
(std_srvs/Trigger):

    ros2 service call /car/shutdown std_srvs/srv/Trigger
    ros2 service call /car/reboot std_srvs/srv/Trigger

On a call it checks that sudo will allow the command, disables the motors,
replies, keeps the motors disabled for a moment, then runs
sudo systemctl poweroff (or reboot), so systemd stops the car_s service
cleanly first. Only these two commands need passwordless sudo, e.g. in
/etc/sudoers.d/car_power:

    graham ALL=(root) NOPASSWD: /usr/bin/systemctl poweroff, /usr/bin/systemctl reboot
"""
import subprocess

import rclpy
import rclpy.executors
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

DELAY_S = 2.0      # time for the reply to reach the caller before going down
MOTOR_OFF_S = 0.2  # how often to repeat motor_on false while waiting
SYSTEMCTL = '/usr/bin/systemctl'


def sudo_allows(action):
    """True if sudo would run systemctl <action> without a password."""
    try:
        return subprocess.run(['sudo', '-n', '-l', SYSTEMCTL, action],
                              capture_output=True, timeout=5).returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


class CarPower(Node):
    def __init__(self):
        super().__init__('car_power')
        self.motor = self.create_publisher(Bool, '/motor_on', 10)
        self.create_service(Trigger, '/car/shutdown',
                            lambda req, res: self.request('poweroff', res))
        self.create_service(Trigger, '/car/reboot',
                            lambda req, res: self.request('reboot', res))
        self.pending = None
        self.motor_timer = None
        self.go_timer = None
        for action in ('poweroff', 'reboot'):
            if not sudo_allows(action):
                self.get_logger().warn(
                    f'sudo will not run "systemctl {action}" without a password, '
                    'so it will be refused (see scripts/car_power.py)')

    def request(self, action, response):
        if self.pending is not None:
            response.success = False
            response.message = f'Already going to {self.pending}'
            return response
        if not sudo_allows(action):
            response.success = False
            response.message = (f'Refused: sudo will not run "systemctl {action}" '
                                'without a password. The Pi is still running')
            self.get_logger().error(response.message)
            return response
        self.get_logger().warn(f'{action} requested: motors off, then {action} in {DELAY_S:.0f} s')
        self.pending = action
        self.motors_off()
        # Keep them off until the end: something else (pc_bringup, key_cmd)
        # could otherwise enable them again during the wait
        self.motor_timer = self.create_timer(MOTOR_OFF_S, self.motors_off)
        self.go_timer = self.create_timer(DELAY_S, self.go)
        response.success = True
        response.message = f'Motors off, {action} in {DELAY_S:.0f} s'
        return response

    def motors_off(self):
        self.motor.publish(Bool(data=False))

    def go(self):
        self.destroy_timer(self.go_timer)
        self.go_timer = None
        self.motors_off()
        try:
            result = subprocess.run(['sudo', '-n', SYSTEMCTL, self.pending],
                                    capture_output=True, text=True, timeout=30)
            failed = result.returncode != 0
            reason = result.stderr.strip()
        except (OSError, subprocess.SubprocessError) as e:
            failed, reason = True, str(e)
        if failed:
            self.get_logger().error(f'systemctl {self.pending} failed: {reason}')
            self.destroy_timer(self.motor_timer)
            self.motor_timer = None
            self.pending = None


def main():
    rclpy.init()
    node = CarPower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
