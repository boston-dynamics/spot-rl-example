# Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

import json
import os
import re
from dataclasses import dataclass
from typing import List

from orbit.orbit_constants import ordered_joint_names_orbit
from utils.dict_tools import dict_from_lists, set_matching


@dataclass
class OrbitConfig:
    """dataclass holding data extracted from orbits training configuration"""

    kp: List[float]
    kd: List[float]
    default_joints: List[float]
    standing_height: float
    action_scale: float


def detect_config_file(directory: os.PathLike) -> os.PathLike:
    """find json file in policy directory

    arguments
    directory -- path where policy and training configuration can be found

    return filepath to json file
    """
    print(os.listdir(directory))
    files = [f for f in os.listdir(directory) if f.endswith(".json")]
    print(files)
    if len(files) == 1:
        return os.path.join(directory, files[0])
    return None


def detect_policy_file(directory: os.PathLike) -> os.PathLike:
    """find onnx file in policy directory

    arguments
    directory -- path where policy and training configuration can be found

    return filepath to onnx file
    """
    files = [f for f in os.listdir(directory) if f.endswith(".onnx")]
    if len(files) == 1:
        return os.path.join(directory, files[0])
    return None


def load_configuration(file: os.PathLike) -> OrbitConfig:
    """parse json file and populate an OrbitConfig dataclass

    arguments
    file -- the path to the json file containing training configuration

    return OrbitConfig containing needed training configuration
    """

    joint_kp = dict_from_lists(ordered_joint_names_orbit, [None] * 12)
    joint_kd = dict_from_lists(ordered_joint_names_orbit, [None] * 12)
    joint_offsets = dict_from_lists(ordered_joint_names_orbit, [None] * 12)

    with open(file) as f:
        env_config = json.load(f)
        actuators = env_config["scene"]["robot"]["actuators"]
        for group in actuators.keys():
            regex = re.compile(actuators[group]["joint_names_expr"][0])

            set_matching(joint_kp, regex, actuators[group]["stiffness"])
            set_matching(joint_kd, regex, actuators[group]["damping"])

        default_joint_data = env_config["scene"]["robot"]["init_state"]["joint_pos"]
        default_joint_expressions = default_joint_data.keys()
        for expression in default_joint_expressions:
            regex = re.compile(expression)
            set_matching(joint_offsets, regex, default_joint_data[expression])

        action_scale = env_config["actions"]["joint_pos"]["scale"]
        standing_height = env_config["scene"]["robot"]["init_state"]["pos"][2]

    return OrbitConfig(
        kp=joint_kp,
        kd=joint_kd,
        default_joints=joint_offsets,
        standing_height=standing_height,
        action_scale=action_scale,
    )
