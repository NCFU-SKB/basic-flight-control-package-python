#!/usr/bin/env python3

import math

from rclpy.node import Node
from shapely.geometry import LineString, Point

from flight_control.mavros_sensors import InfoSensor, VelocitySensor, ImuSensor
from flight_control.mavros_services import ArmingClient, LandClient, SetModeClient
from flight_control.mavros_setpoints import PositionSetpoint, VelocitySetpoint
from flight_control.logger import CustomDroneLogger


class Drone:
    MAX_HORIZONTAL_SPEED = 20.0 # m/s
    MAX_VERTICAL_SPEED = 12.0 # m/s
    SAFE_HORIZONTAL_SPEED = 0.25 # m/s
    SAFE_VERTICAL_SPEED = 0.55# m/s
    EXTRA_HEIGHT_COEF = 0.2

    SLOW_DOWN_YAW_ANGLE = 45
    HEIGHT_ERROR_DIST = 1.0 
    YAW_ERROR_DEG = 15.0
    STABLE_ERROR_ANGLE = 5.0
    NEAR_DIST_GAP_ERROR = 1.5 # m  
    PROGRESS_GAP_ERROR = 0.25
    AT_HEIGHT_ERR = 0.25

    SWITCH_SETPOINT_TYPE_PROGRESS = 0.9
    STRAIGHTNESS_THRESHOLD_ANGLE = 35.0
    STRAIGHT_CONFIDENCE = 0.8

    def __init__(self, drone_name: str, node: Node):
        self.drone_name = drone_name
        self.node = node
        self.logger = CustomDroneLogger(ros_logger=node.get_logger(), drone_name=drone_name)
        
        self._path: LineString | Point | None = None
        self.progress: float = 0.0
        self.height: float = 0.0
        self.safe_speed: bool = False

        self.target_yaw: float = None
        self.hold_yaw: float = None
        self.yaw_system: str = 'NED'

        self._path_points = None
        self._point_index = None
        self._leading_line = None
        self._end_point_dist: float = None
        
        self.init_sensors()
        self.init_services()
        self.init_setpoints()

    @property
    def path(self) -> LineString | Point | None:
        return self._path

    @path.setter
    def path(self, path: LineString | Point | None):
        self._path = path
        self.progress = 0.0
        self._path_points = None
        self._point_index = None
        self._leading_line = None

        if isinstance(self._path, Point):
            position = self.info_sens.position
            point = Point(position.x, position.y)
            dx, dy = self._path.x - point.x, self._path.y - point.y
            self._end_point_dist = math.sqrt(dx**2 + dy**2)

    @property
    def max_horizontal_speed(self) -> float:
        return self.MAX_HORIZONTAL_SPEED if not self.safe_speed else self.SAFE_HORIZONTAL_SPEED
    
    @property
    def max_vertical_speed(self) -> float:
        return self.MAX_VERTICAL_SPEED if not self.safe_speed else self.SAFE_VERTICAL_SPEED
    
    @property
    def at_height(self) -> bool:
        return self.height + self.AT_HEIGHT_ERR > self.info_sens.position.z > self.height - self.AT_HEIGHT_ERR
    
    @property
    def ready(self) -> bool:
        return self.info_sens.data is not None
    
    @property
    def stable(self) -> bool:
        pitch_check = abs(self.info_sens.pitch) < self.STABLE_ERROR_ANGLE
        roll_check = abs(self.info_sens.roll) < self.STABLE_ERROR_ANGLE
        return pitch_check and roll_check
    
    @property
    def at_end(self) -> bool:
        point = Point(self.info_sens.position.x, self.info_sens.position.y)

        end = self.path
        if isinstance(self.path, LineString):
            end = Point(self.path.coords[-1][0], self.path.coords[-1][1])

        dx, dy = end.x - point.x, end.y - point.y
        distance = math.sqrt(dx**2 + dy**2)
        return self.progress > 0.5 and distance < 0.5
    
    @property
    def rotated(self) -> bool:
        return self.target_yaw is None

    def init_sensors(self):
        self.info_sens = InfoSensor(drone_name=self.drone_name, node=self.node)
        self.imu_sens = ImuSensor(drone_name=self.drone_name, node=self.node)
        self.velocity_sens = VelocitySensor(drone_name=self.drone_name, node=self.node)

    def init_services(self):
        self.arming_client = ArmingClient(drone_name=self.drone_name, node=self.node)
        self.land_client = LandClient(drone_name=self.drone_name, node=self.node)
        self.set_mode_client = SetModeClient(drone_name=self.drone_name, node=self.node)

    def init_setpoints(self):
        self.position_sp = PositionSetpoint(drone_name=self.drone_name, node=self.node)
        self.velocity_sp = VelocitySetpoint(drone_name=self.drone_name, node=self.node)    

    def update_navigation(self):
        position = self.info_sens.position

        yaw_rate = self._calculate_yaw() if self.target_yaw or self.hold_yaw else 0

        height_error = self.height - self.info_sens.position.z
        if abs(height_error) < 0.15:
            vz = 0
        elif abs(height_error) < self.HEIGHT_ERROR_DIST:
            vz = self.max_vertical_speed * (height_error / self.HEIGHT_ERROR_DIST)
        else:
            vz = self.max_vertical_speed * math.copysign(1, height_error)

        if not self.path or self.path.is_empty:
            self.velocity_sp.publish(vx=0, vy=0, vz=vz, yaw_rate=yaw_rate)
            return
        
        yaw = 0
        if self.hold_yaw:
            yaw = (90.0 - self.hold_yaw) % 360.0 if self.yaw_system.lower() == 'ned' else self.hold_yaw

        z_coord = ((self.height - self.info_sens.position.z) + self.info_sens.loc_z) + self.EXTRA_HEIGHT_COEF
        if isinstance(self.path, Point) and self._leading_line is None:
            dx, dy = self.path.x - position.x, self.path.y - position.y
            dist = math.sqrt(dx**2 + dy**2)
            self.progress = (self._end_point_dist - dist) / self._end_point_dist
            self.position_sp.publish(x=self.path.x, y=self.path.y, z=z_coord, yaw=math.radians(yaw))
            return
        elif isinstance(self.path, LineString) and self._leading_line is None:
            self._leading_line = True
            self._path_points = self.get_angle_points(self.path, angle_threshold=35)
            self._point_index = 0
        elif isinstance(self.path, LineString) and self._leading_line:
            move_point = self._path_points[self._point_index]
            self.position_sp.publish(x=move_point.x, y=move_point.y, z=z_coord, yaw=math.radians(yaw))
            cur_point = Point(self.info_sens.position.x, self.info_sens.position.y)
            dist = math.hypot(move_point.x - cur_point.x, move_point.y - cur_point.y)
            self.progress = self._point_index / len(self._path_points)
            if dist <= 0.3:
                self._point_index += 1
            if self._point_index >= len(self._path_points):
                self._leading_line = None
                self.path = move_point
            

    def abs_rotate_to(self, target_deg: float, system: str = 'NED'):
        self.target_yaw = (target_deg + 180) % 360 - 180
        self.hold_yaw = self.target_yaw
        self.yaw_system = system

    def rotate_to(self, degree: float, system: str = 'NED'):
        yaw = self.info_sens.get_yaw(system=system.lower())
        self.target_yaw = (yaw + degree + 180) % 360 - 180
        self.hold_yaw = self.target_yaw
        self.yaw_system = system

    def _calculate_yaw(self):
        yaw = (self.info_sens.get_yaw(system=self.yaw_system.lower()) + 180) % 360 - 180
        
        angle_diff = self.hold_yaw - yaw
        angle_diff = (angle_diff + 180) % 360 - 180
        
        k = 1
        if abs(angle_diff) < self.SLOW_DOWN_YAW_ANGLE:
            k = max(0.85, abs(angle_diff) / self.SLOW_DOWN_YAW_ANGLE)
        
        if abs(angle_diff) < self.YAW_ERROR_DEG:
            self.target_yaw = None
            return 0.0
        
        return -math.radians(angle_diff) * k

    def build_path_to_point(self, position: tuple[float, float]) -> LineString:
        current_pos = self.info_sens.position
        return LineString([(current_pos.x, current_pos.y), (position[0], position[1])])

    def build_path_from_point_to_point(
            self, start_point: tuple[float, float], end_point: tuple[float, float]) -> LineString:
        return LineString([(start_point[0], start_point[1]), (end_point[0], end_point[1])])
    
    def get_angle_points(self, path: LineString, angle_threshold: float = 20.0) -> list[Point]:
        if path.is_empty or len(path.coords) < 1.5:
            return 1.0
        
        res_points = []
        coords = list(path.coords)
        
        for i in range(1, len(coords) - 1):
            v1 = (coords[i][0] - coords[i-1][0], coords[i][1] - coords[i-1][1])
            v2 = (coords[i+1][0] - coords[i][0], coords[i+1][1] - coords[i][1])
            
            len1 = math.sqrt(v1[0]**2 + v1[1]**2)
            len2 = math.sqrt(v2[0]**2 + v2[1]**2)
            
            if len1 > 0.1 and len2 > 0.1:
                cos_angle = (v1[0]*v2[0] + v1[1]*v2[1]) / (len1 * len2)
                cos_angle = max(-1, min(1, cos_angle))
                
                angle = math.degrees(math.acos(abs(cos_angle)))
                deviation_angle = 180 - angle
                
                if deviation_angle > angle_threshold:
                    res_points.append(Point(coords[i][0], coords[i][1]))  
        
        res_points.append(Point(coords[-1][0], coords[-1][1]))
        return res_points

    