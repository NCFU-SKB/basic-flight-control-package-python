#!/usr/bin/env python3

"""
MAVROS service clients (Non-blocking, Asynchronous Refactor).

Provides simple, typed, asynchronous service client wrappers for common MAVROS
services. Each client is a thin adapter over an rclpy service client that uses
a callback-based approach to avoid blocking the ROS 2 executor. This is the
recommended pattern for calling services from within node callbacks (like timers
or subscriptions) to prevent deadlocks.
"""

from abc import ABC
from typing import Callable

from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from rclpy.node import Node
from rclpy.task import Future


class MavrosService(ABC):
    """Abstract base for a non-blocking MAVROS service client."""
    TOPIC: str
    srv_type: type

    def __init__(self, drone_name: str, node: Node) -> None:
        """
        Create a service client and wait for it to become available.

        Args:
            drone_name: Namespace or prefix for the drone (topic prefix).
            node: rclpy Node used to create the service client.
        """
        self.node = node
        self.drone_name = drone_name
        self.logger = self.node.get_logger()
        self.client = node.create_client(self.srv_type, self.drone_name + self.TOPIC)
        self.wait_for_service()

    def wait_for_service(self) -> None:
        """
        Wait until the ROS service is available.
        This is a one-time blocking call intended to be used only during initialization.
        """
        self.logger.info(f'Waiting for {self.drone_name + self.TOPIC} service...')
        self.client.wait_for_service()
        self.logger.info(f'{self.drone_name + self.TOPIC} service ready!')

    def _execute_service_call_async(self, request, done_callback: Callable[[Future], None]):
        """
        Execute a service call asynchronously and attach a completion callback.

        Args:
            request: A service request message instance.
            done_callback: A function to call when the service call is complete.
                           It will receive the 'future' object as its only argument.
        """
        future = self.client.call_async(request)
        future.add_done_callback(done_callback)


class ArmingClient(MavrosService):
    """Asynchronous client for the /mavros/cmd/arming service."""
    TOPIC = '/mavros/cmd/arming'
    srv_type = CommandBool

    def __init__(self, drone_name: str, node: Node) -> None:
        super().__init__(drone_name=drone_name, node=node)

    def call_service_async(self, arm: bool, done_callback: Callable[[Future], None]):
        """
        Asynchronously call the arming/disarming service.

        Args:
            arm: True to arm, False to disarm.
            done_callback: Function to be called upon completion.
        """
        request = CommandBool.Request()
        request.value = arm
        self._execute_service_call_async(request, done_callback)

    def arm(self, done_callback: Callable[[Future], None]):
        """Convenience: asynchronously arm the drone."""
        self.call_service_async(arm=True, done_callback=done_callback)

    def disarm(self, done_callback: Callable[[Future], None]):
        """Convenience: asynchronously disarm the drone."""
        self.call_service_async(arm=False, done_callback=done_callback)


class LandClient(MavrosService):
    """Asynchronous client for the /mavros/cmd/land service."""
    TOPIC = '/mavros/cmd/land'
    srv_type = CommandTOL

    def __init__(self, drone_name: str, node: Node) -> None:
        super().__init__(drone_name=drone_name, node=node)

    def call_service_async(self, done_callback: Callable[[Future], None], 
                           latitude: float = 0.0, longitude: float = 0.0,
                           min_pitch: float = 0.0, yaw: float = 0.0):
        """
        Asynchronously request landing.

        Args:
            done_callback: Function to be called upon completion.
            (optional args): Latitude, longitude, etc. for the landing command.
        """
        request = CommandTOL.Request()
        request.altitude = 0.0
        request.latitude = latitude
        request.longitude = longitude
        request.min_pitch = min_pitch
        request.yaw = yaw
        self._execute_service_call_async(request, done_callback)

    def land(self, done_callback: Callable[[Future], None]):
        """Convenience: asynchronously land at the current position."""
        self.call_service_async(done_callback)


class SetModeClient(MavrosService):
    """Asynchronous client for the /mavros/set_mode service."""
    TOPIC = '/mavros/set_mode'
    srv_type = SetMode

    def __init__(self, drone_name: str, node: Node) -> None:
        super().__init__(drone_name=drone_name, node=node)

    def call_service_async(self, custom_mode: str, done_callback: Callable[[Future], None], base_mode: int = 0):
        """
        Asynchronously request a flight mode change.

        Args:
            custom_mode: Mode name string (e.g., "OFFBOARD", "GUIDED").
            done_callback: Function to be called upon completion.
            base_mode: Integer base mode value (default 0).
        """
        request = SetMode.Request()
        request.base_mode = base_mode
        request.custom_mode = custom_mode
        self._execute_service_call_async(request, done_callback)

    def set_offboard_mode(self, done_callback: Callable[[Future], None]):
        """Convenience: asynchronously set OFFBOARD mode."""
        self.call_service_async("OFFBOARD", done_callback)

    def set_guided_mode(self, done_callback: Callable[[Future], None]):
        """Convenience: asynchronously set GUIDED mode."""
        self.call_service_async("GUIDED", done_callback)

    def set_stabilize_mode(self, done_callback: Callable[[Future], None]):
        """Convenience: asynchronously set STABILIZE mode."""
        self.call_service_async("STABILIZE", done_callback)

    def set_land_mode(self, done_callback: Callable[[Future], None]):
        """Convenience: asynchronously set LAND mode."""
        self.call_service_async("LAND", done_callback)

    def set_rtl_mode(self, done_callback: Callable[[Future], None]):
        """Convenience: asynchronously set RTL (Return to Launch) mode."""
        self.call_service_async("RTL", done_callback)