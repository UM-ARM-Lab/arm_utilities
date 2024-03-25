#! /usr/bin/env python3

# sends a simple Trigger message to a server and waits for a response
from rclpy.node import Node
import rclpy
from std_srvs.srv import Trigger

def main():
    rclpy.init()

    node = Node("simple_latency_client")

    client = node.create_client(Trigger, 'simple_latency_server')

    while not client.wait_for_service(timeout_sec=1.0):
        pass

    req = Trigger.Request()
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    res = future.result()



    rclpy.shutdown()


if __name__ == '__main__':
    main()
