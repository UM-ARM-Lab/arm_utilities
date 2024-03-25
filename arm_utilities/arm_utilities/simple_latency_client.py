#! /usr/bin/env python3
import socket
from time import sleep

from matplotlib import pyplot as plt

from rclpy.node import Node
import rclpy
from arm_interfaces.srv import TestLatency


def main():
    rclpy.init()

    node = Node("simple_latency_client")

    client = node.create_client(TestLatency, 'test_latency')

    while not client.wait_for_service(timeout_sec=1.0):
        pass

    one_way_times = []
    for _ in range(50):
        req = TestLatency.Request()
        t1 = node.get_clock().now().to_msg()

        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        res: TestLatency.Response = future.result()

        t2 = node.get_clock().now().to_msg()
        round_trip_tim_ms = (t2.sec - t1.sec) * 1000 + (t2.nanosec - t1.nanosec) / 1000000
        one_way_time_ms = round_trip_tim_ms / 2
        one_way_times.append(one_way_time_ms)

        sleep(0.1)

    client_name = socket.gethostname()
    server_name = res.server_name

    plt.figure()
    plt.title(f"One-way latency {client_name}->{server_name}")
    plt.xlabel("Time (ms)")
    plt.ylabel("Frequency")
    plt.hist(one_way_times)
    plt.show()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
