#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from drone_msgs.msg import CommEvent


class CommunicationNode(Node):

    def __init__(self):
        super().__init__('communication_node')

        # Drone ID
        self.declare_parameter('drone_id', 'drone_1')
        self.drone_id = self.get_parameter('drone_id').value

        # Communication protocol mode
        self.protocol_mode = 'CENTRALIZED'

        # Common communication topic
        self.comm_topic = '/swarm/communication'

        # Communication event topic
        self.event_topic = '/swarm/communication/events'

        # Protocol publisher
        self.comm_pub = self.create_publisher(
            String,
            self.comm_topic,
            10
        )

        # Protocol subscriber
        self.comm_sub = self.create_subscription(
            String,
            self.comm_topic,
            self.communication_callback,
            10
        )

        # Event publisher
        self.event_pub = self.create_publisher(
            CommEvent,
            self.event_topic,
            10
        )

        # Store sent-message timestamps for latency measurement
        self.pending_messages = {}

        # Send HELLO periodically
        self.hello_timer = self.create_timer(
            2.0,
            self.send_hello
        )

        self.get_logger().info(
            f'{self.drone_id} communication node started'
        )

    def send_message(self, message):

        msg = String()
        msg.data = message

        # Store timestamp before publishing
        send_time = time.perf_counter()

        self.comm_pub.publish(msg)

        # Store timestamp using the exact message content
        self.pending_messages[message] = send_time

        self.get_logger().info(
            f'SENT: {message}'
        )

    def publish_event(
        self,
        sender,
        receiver,
        event_type,
        latency_ms,
        success=True
    ):

        event = CommEvent()

        # ROS timestamp
        event.header.stamp = self.get_clock().now().to_msg()

        event.sender_id = sender
        event.receiver_id = receiver
        event.event_type = event_type
        event.protocol_mode = self.protocol_mode

        # Distance intentionally not modeled in Review 2
        event.distance = 0.0

        event.latency_ms = latency_ms

        # Basic communication energy model
        event.battery_cost_percent = 0.001

        event.success = success

        self.event_pub.publish(event)

    def send_hello(self):

        message = (
            f'HELLO|'
            f'sender={self.drone_id}|'
            f'receiver=ALL'
        )

        self.send_message(message)

    def send_ack(self, receiver):

        message = (
            f'ACK|'
            f'sender={self.drone_id}|'
            f'receiver={receiver}'
        )

        self.send_message(message)

    def communication_callback(self, msg):

        # Ignore our own messages
        if f'sender={self.drone_id}' in msg.data:
            return

        receive_time = time.perf_counter()

        self.get_logger().info(
            f'RECEIVED: {msg.data}'
        )

        # Split message fields
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

        if sender is None:
            return

        # Ignore messages explicitly intended for another receiver
        if receiver not in ('ALL', self.drone_id):
            return

        # Calculate latency if this message was sent by this
        # communication network and its timestamp is available.
        latency_ms = 0.0

        if msg.data in self.pending_messages:

            send_time = self.pending_messages.pop(msg.data)

            latency_ms = (
                receive_time - send_time
            ) * 1000.0

        # HELLO -> HANDSHAKE event + ACK
        if message_type == 'HELLO':

            self.publish_event(
                sender=sender,
                receiver=self.drone_id,
                event_type='HANDSHAKE',
                latency_ms=latency_ms,
                success=True
            )

            self.send_ack(sender)

        # ACK -> HANDSHAKE event
        elif message_type == 'ACK':

            self.publish_event(
                sender=sender,
                receiver=self.drone_id,
                event_type='HANDSHAKE',
                latency_ms=latency_ms,
                success=True
            )


def main(args=None):

    rclpy.init(args=args)

    node = CommunicationNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()