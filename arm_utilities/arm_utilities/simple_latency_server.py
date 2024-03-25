#! /usr/bin/env python3
import socket

from arm_interfaces.srv import TestLatency

import rclpy
from rclpy.node import Node


class SimpleLatencyServer(Node):
    def __init__(self):
        super().__init__("simple_latency_server")
        self.server = self.create_service(TestLatency, 'test_latency', self.test_latency)

    def test_latency(self, req: TestLatency.Request, res: TestLatency.Response):
        print("=" * 20)
        print(req.client_name)
        print(req.header.stamp)
        res.server_name = socket.gethostname()
        res.received = self.get_clock().now().to_msg()
        return res


def main():
    rclpy.init()
    node = SimpleLatencyServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
