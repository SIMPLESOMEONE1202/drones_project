#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class GCSNode(Node):

    def __init__(self):
        super().__init__('gcs_node')

        self.comm_topic = '/swarm/communication'

        # Publisher
        self.comm_pub = self.create_publisher(
            String,
            self.comm_topic,
            10
        )

        # Subscriber
        self.comm_sub = self.create_subscription(
            String,
            self.comm_topic,
            self.communication_callback,
            10
        )

        self.get_logger().info(
            'GCS communication node started'
        )

    def send_ack(self, receiver):

        msg = String()

        msg.data = (
            f'ACK|'
            f'sender=GCS|'
            f'receiver={receiver}'
        )

        self.comm_pub.publish(msg)

        self.get_logger().info(
            f'SENT: {msg.data}'
        )

    def communication_callback(self, msg):

        self.get_logger().info(
            f'RECEIVED: {msg.data}'
        )

        parts = msg.data.split('|')

        if len(parts) < 2:
            return

        message_type = parts[0]

        sender = None
        receiver = None

        for part in parts:

            if part.startswith('sender='):
                sender = part.split('=', 1)[1]

            elif part.startswith('receiver='):
                receiver = part.split('=', 1)[1]

        # Respond to HELLO messages from drones
        if (
            message_type == 'HELLO'
            and sender is not None
            and sender != 'GCS'
        ):
            self.send_ack(sender)


def main(args=None):

    rclpy.init(args=args)

    node = GCSNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
