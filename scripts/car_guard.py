#!/usr/bin/env python3
"""Keep the car set up the way pc_bringup.launch.py needs it.

Runs on the PC, started by pc_bringup.launch.py. The car's own settings are
lost whenever its car_s service restarts (a crash, /car/reboot, the Pi
starting late), so instead of setting them once at launch this keeps
checking:

- ekf: the controller's publish_odom_tf must be false, or its TF fights the
  EKF's. Set whenever /car_controller appears, and rechecked every few
  seconds.
- start_lidar: /scan must be arriving. /start_motor is called whenever scans
  stop for a while (the patched lidar driver ignores a start while running).
- motors_on: the motors are enabled when the controller first appears. After
  a car restart they are left off, with a warning: re-arming them by itself
  after something went wrong would be unsafe.
"""
import rclpy
import rclpy.executors
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Empty

CONTROLLER = 'car_controller'
CHECK_S = 1.0          # how often to look
TF_RECHECK_S = 5.0     # how often to confirm publish_odom_tf is still false
SCAN_GAP_S = 5.0       # no /scan for this long: start the lidar
LIDAR_RETRY_S = 10.0   # at most one /start_motor per this long


class CarGuard(Node):
    def __init__(self):
        super().__init__('car_guard')
        self.ekf = self.declare_parameter('ekf', True).value
        self.start_lidar = self.declare_parameter('start_lidar', True).value
        self.motors_on = self.declare_parameter('motors_on', False).value

        self.set_params = self.create_client(SetParameters, f'/{CONTROLLER}/set_parameters')
        self.get_params = self.create_client(GetParameters, f'/{CONTROLLER}/get_parameters')
        self.start_motor = self.create_client(Empty, '/start_motor')
        self.motor_pub = self.create_publisher(Bool, '/motor_on', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

        now = self.now()
        self.present = False
        self.seen_before = False
        self.tf_off = False
        self.last_tf_check = now
        self.last_scan = now
        self.last_lidar_start = now - LIDAR_RETRY_S
        self.busy = set()  # service calls in flight
        self.enables_to_send = 0
        self.create_timer(CHECK_S, self.check)
        self.get_logger().info(
            f'Watching the car: ekf {self.ekf}, start_lidar {self.start_lidar}, '
            f'motors_on {self.motors_on}')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_scan(self, _):
        self.last_scan = self.now()

    def controller_present(self):
        return any(name == CONTROLLER and ns == '/'
                   for name, ns in self.get_node_names_and_namespaces())

    def check(self):
        present = self.controller_present()
        now = self.now()
        if present and not self.present:
            self.appeared()
        elif self.present and not present:
            self.get_logger().warn(f'/{CONTROLLER} has gone: is the car still running?')
        self.present = present
        if not present:
            return

        # Only once the controller is subscribed, or the message is lost
        if self.enables_to_send and self.motor_pub.get_subscription_count() > 0:
            self.motor_pub.publish(Bool(data=True))
            self.enables_to_send -= 1
            if not self.enables_to_send:
                self.get_logger().info('Motors enabled')

        if self.ekf:
            if not self.tf_off:
                self.set_tf_off()
            elif now - self.last_tf_check > TF_RECHECK_S:
                self.last_tf_check = now
                self.check_tf_off()

        if (self.start_lidar and now - self.last_scan > SCAN_GAP_S
                and now - self.last_lidar_start > LIDAR_RETRY_S
                and 'lidar' not in self.busy and self.start_motor.service_is_ready()):
            self.last_lidar_start = now
            self.busy.add('lidar')
            self.get_logger().info('No /scan: starting the lidar motor')
            self.start_motor.call_async(Empty.Request()).add_done_callback(
                lambda _: self.busy.discard('lidar'))

    def appeared(self):
        self.tf_off = False
        if not self.seen_before:
            self.get_logger().info(f'/{CONTROLLER} found')
            if self.motors_on:
                self.enables_to_send = 3
        else:
            self.get_logger().error(
                f'/{CONTROLLER} came back: the car restarted. TF and lidar are '
                'set up again, but its motors are OFF. Enable them with m in '
                'key_cmd once it is safe')
        self.seen_before = True
        # Let the car's own start-up (which stops the lidar) finish before
        # deciding the lidar needs starting
        self.last_scan = self.now()

    def set_tf_off(self):
        if 'set_tf' in self.busy or not self.set_params.service_is_ready():
            return
        self.busy.add('set_tf')
        value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=False)
        req = SetParameters.Request(parameters=[Parameter(name='publish_odom_tf', value=value)])

        def done(future):
            self.busy.discard('set_tf')
            result = future.result()
            if result and result.results and result.results[0].successful:
                self.tf_off = True
                self.last_tf_check = self.now()
                self.get_logger().info('Car TF off: the EKF publishes odom -> base_link')
        self.set_params.call_async(req).add_done_callback(done)

    def check_tf_off(self):
        if 'get_tf' in self.busy or not self.get_params.service_is_ready():
            return
        self.busy.add('get_tf')

        def done(future):
            self.busy.discard('get_tf')
            result = future.result()
            if result and result.values and result.values[0].bool_value:
                self.get_logger().warn('Car TF was back on: turning it off again')
                self.tf_off = False
        self.get_params.call_async(GetParameters.Request(names=['publish_odom_tf'])).add_done_callback(done)


def main():
    rclpy.init()
    node = CarGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
