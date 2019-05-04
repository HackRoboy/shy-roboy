#!/usr/bin/env python


import rclpy
from roboy_cognition_msgs.srv import Talk

node = None

def callback(request, response):
    global node
    node.get_logger().info(str(request.text))
    response.success = True
    return response


def main(args=None):
    global node
    rclpy.init(args=args)

    node = rclpy.create_node('minimal_service')

    srv = node.create_service(Talk, 'roboy/cognition/speech/synthesis/talk', callback)
    while rclpy.ok():
        rclpy.spin_once(node)

    node.destroy_service(srv)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
