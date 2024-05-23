# Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

import math
import time
from contextlib import nullcontext
from threading import Thread
from typing import Any, Callable, List

from bosdyn.api.robot_command_pb2 import JointControlStreamRequest
from bosdyn.api.robot_state_pb2 import RobotStateStreamResponse


class RepeatedTimer(Thread):
    def __init__(self, dt_seconds: float, target: Callable, args: List[Any] = []) -> None:
        super().__init__()
        self._dt_seconds = dt_seconds
        self._target = target
        self._args = args
        self._stopping = False

    def run(self):
        run_time = time.monotonic()
        while not self._stopping:
            now = time.monotonic()
            num_dt = math.ceil((now - run_time) / self._dt_seconds)
            run_time += num_dt * self._dt_seconds
            time.sleep(run_time - now)
            self._target(*self._args)

    def stop(self):
        self._stopping = True


class MockSpot:
    def __init__(self):
        self._state_stream_stopping = False
        self._command_stream_stopping = False

    def start_state_stream(self, on_state_update: Callable[[RobotStateStreamResponse], None]):
        self._state_msg = RobotStateStreamResponse()
        self._state_msg.kinematic_state.odom_tform_body.rotation.w = 1
        self._state_msg.joint_states.position.extend([0] * 12)
        self._state_msg.joint_states.velocity.extend([0] * 12)
        self._state_msg.joint_states.load.extend([0] * 12)

        self._stateUpdates = RepeatedTimer(1 / 333, on_state_update, args=[self._state_msg])
        self._stateUpdates.start()

    def start_command_stream(
        self, command_policy: Callable[[None], JointControlStreamRequest], timing_policy: Callable[[None], None]
    ):
        self._timing_policy = timing_policy
        self._command_generator = command_policy

        self._command_thread = Thread(target=self._commandUpdate)
        self._command_thread.start()

    def lease_keep_alive(self):
        return nullcontext()

    def _commandUpdate(self):
        while not self._command_stream_stopping:
            self._timing_policy()
            self._command_generator()

    def power_on(self):
        pass

    def stand(self, body_height: float):
        pass

    def stop_state_stream(self):
        if self._stateUpdates is not None:
            self._stateUpdates.stop()
            self._stateUpdates.join()

    def stop_command_stream(self):
        if self._command_thread is not None:
            self._command_stream_stopping = True
            self._command_thread.join()
