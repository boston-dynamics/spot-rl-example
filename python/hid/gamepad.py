# Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

import json
import os
from collections import deque
from dataclasses import dataclass
from threading import Thread

import numpy as np
import pygame


@dataclass
class AxisConfig:
    """dataclass holding configuration data for a single axis"""

    index: int  # pygame axis index to control this axis
    inverted: bool  # true if the pygame axis input should be inverted
    deadband: float  # percentage of joystick travel that should return zero
    min_forward_dir: float  # magnitude of output when joystick is at forward edge of deadband
    max_forward_dir: float  # magnitude of output when joystick is fully forward
    min_reverse_dir: float  # magnitude of output when joystick is at reverse edge of deadband
    max_reverse_dir: float  # magnitude of output when joystick is fully reversed


@dataclass
class GamepadConfig:
    """dataclass holding gamepad configuration data, default values are for PS4 controller"""

    x_axis_config :AxisConfig = AxisConfig(1, True, 0.2, 0.25, 1, 0.25, 1)
    y_axis_config : AxisConfig = AxisConfig(0, True, 0.2, 0.25, 1, 0.25, 1)
    yaw_axis_config : AxisConfig = AxisConfig(3, True, 0.2, 0.0, 1, 0.0, 1)
    median_filter_window: int = 10  # size of buffer used for salt and pepper filtering


def load_gamepad_configuration(file: os.PathLike) -> GamepadConfig:
    """parse json file and populate an Gamepad config dataclass

    arguments
    file -- the path to the json file containing gamepad configuration

    return GamepadConfig containing needed data
    """

    with open(file) as f:
        config_file = json.load(f)

        axis_mapping = config_file["axis_mapping"]
        filter = config_file["median_filter"]

        forward_backward = AxisConfig(
            index=axis_mapping["forward_backward"]["index"],
            inverted=axis_mapping["forward_backward"]["inverted"],
            deadband=config_file["deadband"],
            min_forward_dir=config_file["forward"]["min_velocity"],
            max_forward_dir=config_file["forward"]["max_velocity"],
            min_reverse_dir=config_file["backward"]["min_velocity"],
            max_reverse_dir=config_file["backward"]["max_velocity"],
        )

        lateral = AxisConfig(
            index=axis_mapping["lateral"]["index"],
            inverted=axis_mapping["lateral"]["inverted"],
            deadband=config_file["deadband"],
            min_forward_dir=config_file["lateral"]["min_velocity"],
            max_forward_dir=config_file["lateral"]["max_velocity"],
            min_reverse_dir=config_file["lateral"]["min_velocity"],
            max_reverse_dir=config_file["lateral"]["max_velocity"],
        )

        yaw = AxisConfig(
            index=axis_mapping["yaw"]["index"],
            inverted=axis_mapping["yaw"]["inverted"],
            deadband=config_file["deadband"],
            min_forward_dir=config_file["yaw"]["min_velocity"],
            max_forward_dir=config_file["yaw"]["max_velocity"],
            min_reverse_dir=config_file["yaw"]["min_velocity"],
            max_reverse_dir=config_file["yaw"]["max_velocity"],
        )

        return GamepadConfig(
            x_axis_config=forward_backward,
            y_axis_config=lateral,
            yaw_axis_config=yaw,
            median_filter_window=filter["window_size"],
        )


def interpolate(start, end, percent):
    return start + percent * (end - start)


def joystick_connected():
    if not pygame.get_init():
        pygame.init()
    if not pygame.joystick.get_init():
        pygame.joystick.init()

    return pygame.joystick.get_count() > 0


class Gamepad:
    def __init__(self, context, config: GamepadConfig):
        if not pygame.get_init():
            pygame.init()
        if not pygame.joystick.get_init():
            pygame.joystick.init()

        self._context = context
        self.x_vel = 0
        self.y_vel = 0
        self.yaw = 0
        self.joystick = pygame.joystick.Joystick(0)
        self._stopping = False
        self._listening_thread = None
        self._config = config

        buffer_length = config.median_filter_window
        self._x_buffer = deque([0] * buffer_length, buffer_length)
        self._y_buffer = deque([0] * buffer_length, buffer_length)
        self._yaw_buffer = deque([0] * buffer_length, buffer_length)

        print(f"[INFO] Initialized {self.joystick.get_name()}")
        print(f"[INFO] Joystick power level {self.joystick.get_power_level()}")

    def _apply_curve(self, value: float, cfg: AxisConfig):
        if cfg.inverted:
            value = -value

        if abs(value) < cfg.deadband:
            return 0
        elif value > 0:
            slope = (cfg.max_forward_dir - cfg.min_forward_dir) / (1.0 - cfg.deadband)
            shift = cfg.max_forward_dir - slope
            value = slope * value + shift
            return np.clip(value, cfg.min_forward_dir, cfg.max_forward_dir)
        else:
            slope = (cfg.max_reverse_dir - cfg.min_reverse_dir) / (1.0 - cfg.deadband)
            shift = cfg.max_reverse_dir - slope
            value = slope * value + shift
            return np.clip(value, -cfg.max_reverse_dir, -cfg.min_reverse_dir)

    def start_listening(self):
        self._listening_thread = Thread(target=self.listen)
        self._listening_thread.start()

    def listen(self):
        while not self._stopping:
            # Handle events
            pygame.event.pump()
            x_input = self.joystick.get_axis(self._config.x_axis_config.index)
            y_input = self.joystick.get_axis(self._config.y_axis_config.index)
            yaw_input = self.joystick.get_axis(self._config.yaw_axis_config.index)

            x_vel = self._apply_curve(x_input, self._config.x_axis_config)
            y_vel = self._apply_curve(y_input, self._config.y_axis_config)
            yaw = self._apply_curve(yaw_input, self._config.yaw_axis_config)

            self._x_buffer.append(x_vel)
            self._y_buffer.append(y_vel)
            self._yaw_buffer.append(yaw)

            self.x_vel = np.median(self._x_buffer)
            self.y_vel = np.median(self._y_buffer)
            self.yaw = np.median(self._yaw_buffer)

            self._context.velocity_cmd = [self.x_vel, self.y_vel, self.yaw]
            # update inputs at 100hz this should mean we always have a new data for the 50Hz command update
            pygame.time.wait(10)

    def stop_listening(self):
        if self._listening_thread is not None:
            self._stopping = True
            self._listening_thread.join()
