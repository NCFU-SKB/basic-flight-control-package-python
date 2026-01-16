#!/usr/bin/env python3

"""
Module for managing drone navigation setpoints using ROS 2.

Includes QoS profiles for reliable setpoint communication.

The module uses publishers to send setpoint messages to the drone. These publishers
are responsible for communicating desired positions, velocities, attitudes, and thrusts
to the drone's flight controller through ROS 2 topics.
"""

from   abc import ABC, abstractmethod
import math

from   geometry_msgs.msg import PoseStamped, TwistStamped
from   mavros_msgs.msg import AttitudeTarget, Thrust
from   rclpy.node import Node
from   rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from   std_msgs.msg import Header


# QoS profile for setpoints - more reliable than sensors
setpoint_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,  # TCP-like
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.TRANSIENT_LOCAL
)


class MavrosSetpoint(ABC):
    """Abstract base class for setpoint publishers.

    Subclasses must define TOPIC and msg_type and implement publish_setpoint.
    This class provides a ROS2 publisher with a shared QoS and a couple of
    small helper utilities (header creation and clamping).
    """
    TOPIC: str
    msg_type: type

    def __init__(self, drone_name: str, node: Node):
        """Create publisher and attach node/logger references.

        Args:
            drone_name: Prefix for topic names (drone namespace).
            node: rclpy node used to create the publisher.
        """
        self.node = node
        self.drone_name = drone_name
        self.publisher = node.create_publisher(
            self.msg_type,
            self.drone_name + self.TOPIC,
            setpoint_qos
        )
        self.node.get_logger().info(f'Created setpoint publisher for {self.drone_name + self.TOPIC}')

    def _create_header(self) -> Header:
        """Create a Header message populated with the current time.

        Returns:
            std_msgs.msg.Header: header with timestamp and frame_id set.
        """
        header = Header()
        header.stamp = self.node.get_clock().now().to_msg()
        header.frame_id = "base_link"
        return header

    @abstractmethod
    def publish(self, **kwargs):
        """Publish a setpoint message.

        Subclasses must accept keyword arguments for their specific message
        fields and publish the appropriate ROS message.
        """
        raise NotImplementedError


class PositionSetpoint(MavrosSetpoint):
    """Position control publisher using PoseStamped.

    Publishes local position setpoints (x, y, z) and orientation encoded
    as a quaternion from yaw. The yaw parameter is expected in radians and
    corresponds to the local frame (documentation/comment uses NED).
    """
    TOPIC = '/mavros/setpoint_position/local'

    def __init__(self, drone_name: str, node: Node):
        """Create a PositionSetpoint publisher.

        Args:
            drone_name: Namespace prefix for topics.
            node: rclpy node used for publishing.
        """
        self.msg_type = PoseStamped
        super().__init__(drone_name, node)

    def publish(self, x: float = 0.0, y: float = 0.0, z: float = 2.0,
                         yaw: float = 0.0):
        """Publish a PoseStamped position setpoint.

        Args:
            x: X coordinate in meters.
            y: Y coordinate in meters.
            z: Z coordinate in meters.
            yaw: Yaw angle in radians (used to build quaternion).
        """
        msg = PoseStamped()
        msg.header = self._create_header()

        # Position
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)

        # Orientation (convert yaw to quaternion)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)

        self.publisher.publish(msg)


class VelocitySetpoint(MavrosSetpoint):
    """Velocity control publisher using TwistStamped"""
    TOPIC = '/mavros/setpoint_velocity/cmd_vel'

    def __init__(self, drone_name: str, node: Node):
        self.msg_type = TwistStamped
        super().__init__(drone_name, node)

    def publish(self, vx: float = 0.0, vy: float = 0.0, vz: float = 0.0, yaw_rate: float = 0.0):
        """Publish a TwistStamped velocity setpoint."""
        msg = TwistStamped()
        msg.header = self._create_header()

        # Linear velocities (body frame)
        msg.twist.linear.x = float(vx)   # forward/backward
        msg.twist.linear.y = float(vy)   # left/right
        msg.twist.linear.z = float(vz)   # up/down

        # Angular velocities
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = float(yaw_rate)

        self.publisher.publish(msg)