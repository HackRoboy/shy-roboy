#!/usr/bin/env python

import rclpy
from std_msgs.msg import Int8
from roboy_cognition_msgs.srv import Talk

shout_node = None
last_state = 0
current_state = 0

def process_state(data):
    if current_state == 2 and last_state != 2:
        shout()

def shout():
    shout_node.getlogger().info('\tSHOUT')
    client = shout_node.create_client(Talk,'roboy/cognition/speech/synthesis/talk')
    request = Talk.Request()
    request.text = "Please don't touch me"
    future = client.call_async(request) 
    rclpy.spin_until_future_complete(shout_node, future)

def main(args=None):
    rclpy.init()
    shout_node = rclpy.create_node('shouter')
    shout_node.create_subscription(Int8, 'shy_roboy/state',  process_state
    while rclpy.ok():
        rclpy.spin_once(shout_node)
    shout_node.destroy_node()
    rclpy.shutdown()

