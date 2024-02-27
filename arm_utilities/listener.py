import threading
from copy import deepcopy

from rclpy.node import Node
from threading import Lock
from rclpy import logging
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

logger = logging.get_logger("Listener")


class Listener:
    def __init__(self, node: Node, topic_type, topic_name, qos=10):
        """
        Listener is a wrapper around a subscriber where the callback simply records the latest msg.

        Listener does not consume the message
            (for consuming behavior, use the standard ros callback pattern)
        Listener does not check timestamps of message headers

        Parameters:
            node:
            topic_name:      name of topic to subscribe to
            topic_type: type of message received on topic
            wait_for_data:  block constructor until a message has been received
        """

        self.data = None
        self.lock = Lock()
        self.event = threading.Event()

        self.topic_name = topic_name
        self.callback_group = MutuallyExclusiveCallbackGroup()
        self.subscriber = node.create_subscription(topic_type, topic_name, self.callback, qos, callback_group=self.callback_group)

    def callback(self, msg):
        with self.lock:
            self.data = msg
            if not self.event.is_set():
                self.event.set()

    def get(self, block_until_data=True):
        """
        Returns the latest msg from the subscribed topic

        Parameters:
            block_until_data (bool): block if no message has been received yet.
                                     Guarantees a msg is returned (not None)
        """
        if block_until_data:
            self.event.wait()

        with self.lock:
            return deepcopy(self.data)
