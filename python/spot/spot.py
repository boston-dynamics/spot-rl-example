# Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

import time
from threading import Thread
from typing import Callable

import bosdyn.client
import bosdyn.client.lease
import bosdyn.client.util
from bosdyn import geometry
from bosdyn.api.robot_command_pb2 import JointControlStreamRequest
from bosdyn.api.robot_state_pb2 import RobotStateStreamResponse
from bosdyn.client.robot_command import (
    RobotCommandBuilder,
    RobotCommandClient,
    RobotCommandStreamingClient,
    blocking_stand,
)
from bosdyn.client.robot_state import RobotStateStreamingClient


class Spot:
    """wrapper around bosdyn API"""

    def __init__(self, config) -> None:
        """setup spot sdk, connect to robot and confirm estop

        arguments
        config -- arguments for connecting to spot must contain hostname
        """

        self._started_streaming = False
        self._activate_thread_stopping = False
        self._command_stream_stopping = False
        self._state_stream_stopping = False

        self._command_thread = None
        self._state_thread = None

        bosdyn.client.util.setup_logging(config.verbose)

        self.sdk = bosdyn.client.create_standard_sdk("JointControlClient")

        # Register the non standard api clients
        self.sdk.register_service_client(RobotCommandStreamingClient)
        self.sdk.register_service_client(RobotStateStreamingClient)
        self.robot = self.sdk.create_robot(config.hostname)
        bosdyn.client.util.authenticate(self.robot)
        self.robot.time_sync.wait_for_sync()
        assert not self.robot.is_estopped(), "Robot is estopped. Please use an external E-Stop client."

    def __del__(self):
        """clean up active streams and threads if spot goes out of scope or is deleted"""
        self.stop_command_stream()
        self.stop_state_stream()

    def lease_keep_alive(self):
        """acquire lease and keep it alive as long as return is in scope
        use this function in a with statement around other commands

        return scoped lease
        """
        lease_client = self.robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)
        return bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True)

    def power_on(self):
        """Turn on power to robot's motors."""
        self.robot.logger.info("Powering on robot... This may take several seconds.")
        self.robot.power_on(timeout_sec=20)
        if not self.robot.is_powered_on():
            raise RuntimeError("Robot power on failed.")
        self.robot.logger.info("Robot powered on.")

    def stand(self, body_height: float = 0.0):
        """Tell spot to stand and block until it is standing

        arguments
        body_height -- controls height of standing as delta from default or 0.525m

        """
        self._command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
        # Stand the robot
        params = RobotCommandBuilder.mobility_params(body_height, footprint_R_body=geometry.EulerZXY())

        blocking_stand(self._command_client, 10, 1.0, params)

    def start_state_stream(self, on_state_update: Callable[[RobotStateStreamResponse], None]):
        """The robot state streaming client will allow us to get the robot's joint and imu information.

        arguments
        on_state_update -- Callable that will be called at ~333Hz with latest state data

        """
        self.robot_state_streaming_client = self.robot.ensure_client(RobotStateStreamingClient.default_service_name)
        self._state_thread = Thread(target=self._handle_state_stream, args=[on_state_update])
        self._state_thread.start()

    def stop_state_stream(self):
        """stop listening to state stream updates this will also tell spot to stop sending them"""
        if self._state_thread is not None:
            self._state_stream_stopping = True
            self._state_thread.join()

    def start_command_stream(
        self, command_policy: Callable[[None], JointControlStreamRequest], timing_policy: Callable[[None], None]
    ):
        """create command streamt to send joint level commands to spot

            command stream will repeatedly call timing_policy to block until a command should be sent
            and then call command_policy to create one command

        arguments
        command_policy -- Callable that will create one joint command
        timing_policy -- Callable that blocks until the next time a command should be generated

        """
        # Async activate once streaming has started
        self._activate_thread = Thread(target=self.activate)
        self._activate_thread.start()

        self._command_thread = Thread(target=self._run_command_stream, args=[command_policy, timing_policy])
        self._command_thread.start()

    def stop_command_stream(self):
        """Stop sending joint commands robot will revert to bosdyns standing controller"""
        if self._command_thread is not None:
            self._command_stream_stopping = True
            self._command_thread.join()

        if self._activate_thread is not None:
            self._activate_thread_stopping = True
            self._activate_thread.join()

    def _handle_state_stream(self, on_state_update: Callable[[RobotStateStreamResponse], None]):
        """private function to be run in state stream thread
            listens for state steam events and calls users callback

        arguments
        on_state_update -- callback supplied to start_state_stream
        """
        for state in self.robot_state_streaming_client.get_robot_state_stream():
            on_state_update(state)

            if self._state_stream_stopping:
                return

    def _run_command_stream(
        self, command_policy: Callable[[None], JointControlStreamRequest], timing_policy: Callable[[None], None]
    ):
        """private function to be run in command stream thread handles opening grpc
            stream

        arguments
        command_policy -- callback supplied to start_command_stream to create commands
        timing_policy -- callback supplied to start_command_stream to control timing
        """

        self._command_streaming_client = self.robot.ensure_client(RobotCommandStreamingClient.default_service_name)

        try:
            self.robot.logger.info("Starting command stream")
            res = self._command_streaming_client.send_joint_control_commands(
                self._command_stream_loop(command_policy, timing_policy)
            )
            print(res)
        finally:
            self._activate_thread_stopping = True

            if self._activate_thread:
                self._activate_thread.join()

        # Power the robot off. By specifying "cut_immediately=False", a safe power off command
        # is issued to the robot. This will attempt to sit the robot before powering off.
        self.robot.power_off(cut_immediately=False, timeout_sec=20)
        assert not self.robot.is_powered_on(), "Robot power off failed."
        self.robot.logger.info("Robot safely powered off.")

    def _command_stream_loop(
        self, command_policy: Callable[[None], JointControlStreamRequest], timing_policy: Callable[[None], None]
    ):
        """coroutine needed for command stream. repeatedly calls timing_policty
        to block until next dt and then yields the result of command_policy once

        arguments
        command_policy -- callback supplied to start_command_stream to create commands
        timing_policy -- callback supplied to start_command_stream to control timing
        """

        while not self._command_stream_stopping:
            if timing_policy():
                yield command_policy()
                self._started_streaming = True
            else:
                print("timing policy timeout")
                return
        print("stopping is True")

    # Method to activate full body joint control through RobotCommand
    def activate(self):
        self._command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)

        # Wait for streaming to start
        while not self._started_streaming:
            time.sleep(0.001)

            if self._activate_thread_stopping:
                return

        # Activate joint control
        self.robot.logger.info("Activating joint control")
        joint_command = RobotCommandBuilder.joint_command()

        try:
            self._command_client.robot_command(joint_command)
        finally:
            # Signal everything else to stop too
            self._activate_thread_stopping = True
