"""
Node that receives Twist commands and re-publishes them as TwistStamped.

This node acts as a middleware adapter between standard
teleoperation/navigation nodes and the ROS 2 Control Ackermann controller:

* **Subscribes** to unstamped velocity commands (``geometry_msgs/Twist``)
  from the twist_mux (``/cmd_vel_mux``).
* **Adds a Timestamp** using the current node clock (vital for proper
  synchronization in simulation and real hardware).
* **Publishes** stamped commands (``geometry_msgs/TwistStamped``) to the
  controller's reference topic.

Usage is critical because ``ackermann_steering_controller`` often requires
stamped messages to verify command freshness.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class TwistStamper(Node):
    """
    ROS 2 Node that converts unstamped Twist messages into TwistStamped msgs.
    """

    def __init__(self):
        super().__init__('twist_stamper')

        # -----------------------------------------------------------------
        # Subscribers
        # -----------------------------------------------------------------
        # Listen to the multiplexed output from twist_mux.
        # This topic aggregates commands from Nav2, Teleop, etc.
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel_mux',
            self.listener_callback,
            10
        )

        # -----------------------------------------------------------------
        # Publishers
        # -----------------------------------------------------------------
        # Publish the stamped message to the Ackermann controller.
        # The controller expects TwistStamped to ensure commands are not stale.
        self.publisher = self.create_publisher(
            TwistStamped,
            '/ackermann_steering_controller/reference',
            10
        )

        # Log startup info
        self.get_logger().info(
            'Twist Stamper Started: /cmd_vel_mux -> '
            '/ackermann_steering_controller/reference'
        )

    def listener_callback(self, msg_in: Twist):
        """
        Callback function executed when a new Twist message is received.

        :param msg_in: The incoming unstamped Twist message.
        """
        # Create a new TwistStamped message
        msg_out = TwistStamped()

        # 1. Stamp the message with the current ROS time
        #    This allows the controller to reject old/latent commands.
        msg_out.header.stamp = self.get_clock().now().to_msg()

        # 2. Set the frame ID
        #    'base_footprint' is standard for robot velocity commands.
        msg_out.header.frame_id = 'base_footprint'

        # 3. Copy the velocity data (linear & angular) from input to output
        msg_out.twist = msg_in

        # Publish the stamped message
        self.publisher.publish(msg_out)


def main(args=None):
    """
    Main entry point for the node.
    """
    rclpy.init(args=args)

    try:
        node = TwistStamper()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up and shutdown
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
