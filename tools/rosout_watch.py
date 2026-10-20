"""Print the log messages of every node, from /rosout.

Handy to see Nav2, SLAM or explore_lite messages from another terminal or
machine, since a launch only prints them in its own terminal.

    python3 tools/rosout_watch.py             # until Ctrl-C
    python3 tools/rosout_watch.py 60          # for 60 s
    python3 tools/rosout_watch.py 60 WARN     # warnings and errors only
"""
import sys
import time

import rclpy
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

LEVELS = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'}
# Nodes that are noisy and rarely interesting
SKIP = ('rviz', 'transform_listener', 'rosout_watch')


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else float('inf')
    names = {v: k for k, v in LEVELS.items()}
    min_level = names.get(sys.argv[2].upper(), 20) if len(sys.argv) > 2 else 20

    rclpy.init()
    node = Node('rosout_watch')

    def show(m):
        if m.level < min_level or m.name.startswith(SKIP):
            return
        stamp = time.strftime('%H:%M:%S', time.localtime(m.stamp.sec))
        print(f'{stamp} {LEVELS.get(m.level, m.level):5s} [{m.name}] {m.msg}', flush=True)

    node.create_subscription(Log, '/rosout', show,
                             QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))
    end = time.time() + duration
    try:
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
