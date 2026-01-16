#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2

from flight_control.drone import Drone
from flight_control.missions.example import ExampleMission


class FlightControlNode(Node):
    def __init__(self, drone_name: str = 'uav1'):
        super().__init__('flight_control')
        drone = Drone(
            drone_name=drone_name, 
            node=self,
        )
        
        self.mission = ExampleMission(drone=drone)
        
        self.get_logger().info("Mission will start in 5 seconds...")
        self.create_timer(5, self.start_mission_callback)

    def start_mission_callback(self):
        """One-shot timer callback to start the mission."""
        self.mission.start()


def main(args=None):
    rclpy.init(args=args)
    navigator = FlightControlNode()
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info("Mission interrupted by user (Ctrl+C)")
    finally:
        navigator.get_logger().info("Shutting down")
        navigator.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()