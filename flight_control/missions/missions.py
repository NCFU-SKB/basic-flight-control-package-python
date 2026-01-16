#!/usr/bin/env python3
"""
Abstract base class for drone missions (Fully asynchronous refactor).
"""

from abc import ABC, abstractmethod
import time
from flight_control.drone import Drone

class Mission(ABC):
    """Abstract base class for all drone missions, using a non-blocking, callback-based design."""

    def __init__(self, name: str, drone: Drone):
        self.name = name
        self.drone = drone
        self.logger = drone.logger
        self.is_running = False
        self.start_time = None
        
        # Таймеры должны быть атрибутами класса, чтобы их не удалил сборщик мусора
        self._init_timer = None
        self._setpoint_timer = None

    def start(self):
        """Starts the mission by initiating the asynchronous initialization sequence."""
        if self.is_running:
            self.logger.warning(f"Mission {self.name} is already running")
            return

        self.logger.info(f"Starting mission: {self.name}")
        self.is_running = True
        self.start_time = time.time()
        
        # Запускаем первый шаг асинхронной цепочки инициализации
        self._initialize_offboard_sequence_step1_send_setpoints()

    def stop(self):
        """Stops the mission execution and cleans up resources."""
        if not self.is_running:
            return

        self.logger.info(f"Stopping mission: {self.name}")
        self.is_running = False

        # Важно отменить все активные таймеры при остановке
        if self._setpoint_timer and not self._setpoint_timer.is_canceled:
            self._setpoint_timer.cancel()
        if self._init_timer and not self._init_timer.is_canceled:
            self._init_timer.cancel()
        
        self.on_mission_end()

    # --- Асинхронная цепочка инициализации ---

    def _initialize_offboard_sequence_step1_send_setpoints(self):
        """Step 1: Send initial setpoints to prepare for OFFBOARD mode."""
        self.logger.info("Initializing OFFBOARD mode (Step 1: Sending initial setpoints)...")
        
        setpoints_to_send = 15  # Отправляем 1.5 секунды
        self._init_counter = 0

        def send_initial_setpoint_callback():
            if self._init_counter < setpoints_to_send:
                self.drone.position_sp.publish(0, 0, 1, 0)
                self._init_counter += 1
            else:
                # После отправки всех setpoint'ов, отменяем этот таймер и переходим к следующему шагу
                self._init_timer.cancel()
                self._initialize_offboard_sequence_step2_set_mode()

        self._init_timer = self.drone.node.create_timer(0.1, send_initial_setpoint_callback)

    def _initialize_offboard_sequence_step2_set_mode(self):
        """Step 2: Asynchronously call the set_mode service."""
        self.logger.info("Step 2: Setting OFFBOARD mode...")
        self.drone.set_mode_client.call_service_async(
            custom_mode='OFFBOARD',
            done_callback=self._set_mode_done_callback
        )

    def _set_mode_done_callback(self, future):
        """Callback for when the set_mode service call is complete."""
        try:
            response = future.result()
            if not response.mode_sent:
                raise RuntimeError("Failed to send OFFBOARD mode command.")
            self.logger.info("OFFBOARD mode set successfully. Step 3: Arming drone...")
            # Если успешно, переходим к следующему шагу
            self._initialize_offboard_sequence_step3_arm_drone()
        except Exception as e:
            self.logger.error(f"Failed to set OFFBOARD mode: {e}")
            self.stop()

    def _initialize_offboard_sequence_step3_arm_drone(self):
        """Step 3: Asynchronously call the arming service."""
        self.drone.arming_client.call_service_async(
            arm=True,
            done_callback=self._arm_drone_done_callback
        )

    def _arm_drone_done_callback(self, future):
        """Callback for when the arming service call is complete. This is the final initialization step."""
        try:
            response = future.result()
            if not response.success:
                raise RuntimeError("Arming was not successful.")
            
            self.logger.info("OFFBOARD mode active and drone armed. Mission is live!")
            
            # --- Инициализация завершена успешно ---
            # Вызываем пользовательский хук начала миссии
            self.on_mission_start()
            
            # Запускаем основной цикл миссии через таймер
            self._setpoint_timer = self.drone.node.create_timer(0.05, self.on_setpoint_callback) # 20Hz

        except Exception as e:
            self.logger.error(f"Failed to arm drone: {e}")
            self.stop()

    # --- Публичный интерфейс для дочерних классов миссий (остается без изменений) ---

    def get_elapsed_time(self) -> float:
        """Get elapsed time since mission start."""
        if self.start_time is None:
            return 0.0
        return time.time() - self.start_time

    @abstractmethod
    def on_mission_start(self):
        """Called when mission starts - setup mission-specific resources."""
        pass

    @abstractmethod
    def on_setpoint_callback(self):
        """Called every setpoint cycle (20Hz) - mission-specific updates."""
        pass

    @abstractmethod
    def on_mission_end(self):
        """Called when mission ends - cleanup mission-specific resources."""
        pass