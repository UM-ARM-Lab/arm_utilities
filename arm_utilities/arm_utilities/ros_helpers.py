#! /usr/bin/env python
from time import perf_counter

import rclpy.logging
from rclpy.publisher import Publisher
from sensor_msgs.msg import Joy

logger = rclpy.logging.get_logger("ros_helpers")


def joy_to_xbox(joy: Joy, xpad: bool = True):
    """
    Transforms a joystick sensor_msg to a XBox controller for easier code readability

    Parameters:
    joy: xbox msg
    xpad: True if using the default xpad driver, False if using xboxdrv

    Returns:
    xbox struct where fields are the button names
    """

    class Xbox_msg():
        def __str__(self):
            items = vars(self).items()
            return "\n".join("%s: %s" % item for item in items)

    x = Xbox_msg()
    if xpad:
        x.A, x.B, x.X, x.Y, x.LB, x.RB, x.back, x.start, x.power, x.stick_button_left, x.stick_button_right, x.DL, x.DR, x.DU, x.DD = joy.buttons
        x.LH, x.LV, x.LT, x.RH, x.RV, x.RT, x.DH, x.DV = joy.axes
    else:
        x.A, x.B, x.X, x.Y, x.LB, x.RB, x.back, x.start, x.power, x.stick_button_left, x.stick_button_right = joy.buttons
        x.LH, x.LV, x.LT, x.RH, x.RV, x.RT, x.DH, x.DV = joy.axes
    return x


def wait_for_subscriber(pub: Publisher, timeout_sec: float = 10.0):
    t0 = perf_counter()
    while pub.get_subscription_count() == 0:
        if (perf_counter() - t0) > timeout_sec:
            raise TimeoutError("Timeout waiting for subscriber")
