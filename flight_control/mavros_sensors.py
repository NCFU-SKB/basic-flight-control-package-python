#!/usr/bin/env python3

"""MAVROS sensor subscribers.

Provides thin, typed wrappers around common MAVROS sensor topics (IMU,
magnetometer, GPS, local position and barometer). Each wrapper creates a
subscriber with a consistent QoS profile and exposes convenient accessors
to the most commonly used fields of the underlying ROS messages.
"""

from   abc import ABC, abstractmethod
import math
from   typing import Optional

from   geometry_msgs.msg import Vector3, Quaternion, PoseStamped, TwistStamped
from   mavros_msgs.msg import Altitude
from   sensor_msgs.msg import Imu
from   rclpy.node import Node
from   rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


# QoS profile for sensors: best-effort, small buffer, volatile durability.
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,  # UDP-like
    history=HistoryPolicy.KEEP_LAST,            # only store up to N samples
    depth=5,                                    # store last 5 samples
    durability=DurabilityPolicy.VOLATILE        # do not persist samples
)


class MavrosSensor(ABC):
    """Abstract base class for a MAVROS sensor subscriber.

    Subclasses should define TOPIC and msg_type and implement `subscribe_callback`.
    The base class creates the ROS subscription and stores the latest received
    message in `self.data`.
    """
    TOPIC: str
    msg_type: type

    def __init__(self, drone_name: str, node: Node) -> None:
        """Create subscriber and initialize internal state.

        Args:
            drone_name: Topic namespace / prefix for the drone (e.g. '/drone1').
            node: rclpy Node used to create subscriptions and logging.
        """
        self.node = node
        self.drone_name = drone_name
        self.data: Optional[object] = None
        self.logger = self.node.get_logger()

        # Create subscriber with standardized QoS
        self.subscription = node.create_subscription(
            self.msg_type,
            self.drone_name + self.TOPIC,
            self.subscribe_callback,
            sensor_qos
        )
        self.logger.info(f'Subscribed to {self.drone_name + self.TOPIC}')

    @abstractmethod
    def subscribe_callback(self, data) -> None:
        """Callback invoked on incoming messages for the subscribed topic.

        Subclasses must implement this to accept the incoming message and
        update internal state accordingly.
        """
        raise NotImplementedError
    

class AltitudeSensor(MavrosSensor):
    TOPIC = '/mavros/altitude'
    msg_type = Altitude

    @property
    def monotonic_altitude(self) -> float:
        if self.data:
            return self.data.monotonic
        
    @property
    def locale_altitude(self) -> float:
        if self.data:
            return self.data.local

    def subscribe_callback(self, data: Altitude) -> None:
        """Store incoming PoseStamped message."""
        self.data = data

class ImuSensor(MavrosSensor):
    TOPIC = '/mavros/imu/data'
    msg_type = Imu

    @property
    def linear_velocity(self) -> Vector3:
        return self.data.linear_acceleration

    def subscribe_callback(self, data: Imu):
        self.data = data

class InfoSensor(MavrosSensor):
    """Subscriber for local position and orientation (PoseStamped)."""

    TOPIC = '/mavros/local_position/pose'
    msg_type = PoseStamped

    def __init__(self, drone_name: str, node: Node) -> None:
        """Initialize local position subscriber."""
        self.altitude = AltitudeSensor(drone_name=drone_name, node=node)
        super().__init__(drone_name=drone_name, node=node)
    
    @property
    def position(self) -> Vector3:
        position = self.data.pose.position
        altitude = self.altitude.monotonic_altitude
        result = Vector3()
        result.x, result.y, result.z = position.x, position.y, altitude
        return result
    
    @property
    def loc_z(self) -> float:
        return self.altitude.locale_altitude
    
    @property
    def orientation(self) -> Quaternion:
        return self.data.pose.orientation
    
    @property
    def pitch(self) -> float:
        orientation = self.orientation
        x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch_rad = math.copysign(math.pi / 2, sinp)
        else:
            pitch_rad = math.asin(sinp)
        return math.degrees(pitch_rad)
    
    @property
    def roll(self) -> float:
        orientation = self.orientation
        x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll_rad = math.atan2(sinr_cosp, cosr_cosp)
        return math.degrees(roll_rad)

    def get_yaw(self, system: str = 'NED') -> float:
        """Return yaw in degrees.
        NED: 0° = North, 90° = East. Result is in range [0, 360).
        ENU: 0° = East, 90° = North. Result is in range [0, 360).
        """
        orientation = self.orientation
        x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
        
        t0, t1 = +2.0 * (w * z + x * y), +1.0 - 2.0 * (y * y + z * z)
        yaw_deg = (math.degrees(math.atan2(t0, t1)) + 360) % 360

        return (90.0 - yaw_deg) % 360.0 if system.lower() == 'ned' else yaw_deg

    def print_full_info(self) -> None:
        """Log a readable dump of local position information."""
        self.logger.info('-' * 50)
        self.logger.info('LOCAL POSITION DATA')
        self.logger.info(str(self))
        self.logger.info('-' * 50)

    def __str__(self) -> str:
        """Return a human-readable string representation of the latest pose."""
        position = self.position
        orientation = self.orientation
        return (
            f'Position: x={position.x:.2f}, y={position.y:.2f}, z={position.z:.2f}\n'
            f'Orientation: x={orientation.x:.2f}, y={orientation.y:.2f}, '
            f'z={orientation.z:.2f}, w={orientation.w:.2f}'
            f'Yaw: {self.get_yaw():.2f}, Pitch: {self.get_pitch():.2f}, '
            f'Roll: {self.get_roll():.2f}'
        )

    def subscribe_callback(self, data: PoseStamped) -> None:
        """Store incoming PoseStamped message."""
        self.data = data
        # self.print_full_info()

class VelocitySensor(MavrosSensor):
    """Subscriber for local velocity (TwistStamped)."""

    TOPIC = '/mavros/local_position/velocity_body'
    msg_type = TwistStamped

    def __init__(self, drone_name: str, node: Node) -> None:
        super().__init__(drone_name=drone_name, node=node)
    
    @property
    def linear_velocity(self) -> Vector3:
        return self.data.twist.linear if self.data else Vector3()
    
    @property 
    def angular_velocity(self) -> Vector3:
        return self.data.twist.angular if self.data else Vector3()

    @property
    def horizontal_speed(self) -> float:
        vel = self.linear_velocity
        return math.sqrt(vel.x**2 + vel.y**2)

    def subscribe_callback(self, data: TwistStamped) -> None:
        self.data = data