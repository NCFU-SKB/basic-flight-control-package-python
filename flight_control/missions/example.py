#!/usr/bin/env python3
from enum import Enum

from shapely.geometry import LineString, Point

from flight_control.drone import Drone
from flight_control.missions.missions import Mission


class Status(Enum):
    TAKEOFF = 'takeoff'
    RECTANGLE = 'rectangle'
    LAND = 'land'


class ExampleMission(Mission):
    HEIGHT = 7
    
    def __init__(self, drone: Drone):
        super().__init__(name="Example mission", drone=drone)
        self.status: Status = None

    def on_mission_start(self):
        self.drone.height = self.HEIGHT
        self.status = Status.TAKEOFF
        self.logger.info("Example mission started!")

    def on_mission_end(self):
        self.logger.info("Example mission ended!")

    def on_setpoint_callback(self):
        self.drone.update_navigation()
        match self.status.value:
            case Status.TAKEOFF.value:
                self.logger.error(f'At height: {self.drone.at_height}. Current height (relative): {self.drone.info_sens.loc_z}. Monotonic: {self.drone.info_sens.position.z}')
                if self.drone.at_height:
                    self.logger.warning('At height!')
                    self.drone.path = LineString([(0, 0), (10, 0), (10, 10), (0, 10), (0, 0)])
                    self.status = Status.RECTANGLE
            case Status.RECTANGLE.value:
                if self.drone.at_end:
                    self.logger.error('We are at the end of the path!')
                    self.drone.height = 0
                    self.status = Status.LAND
            case Status.LAND.value:
                self.logger.error(f'At height: {self.drone.at_height}. Current height (relative): {self.drone.info_sens.loc_z}. Monotonic: {self.drone.info_sens.position.z}')
                if self.drone.at_height + 1:
                    self.logger.warning('At height!')
                    self.drone.land_client.land(land)
                    self.status = None
                    self.on_mission_end()
            case _:
                pass
            

async def land(cringobus):
    print('aboba')
