import sys
import time
import copy
import numpy as np
from pathlib import Path
from typing import Dict, Any, List
import cv2

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelSubscriber,
    ChannelPublisher,
)
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_._MotorState_ import MotorState_
from unitree_sdk2py.idl.unitree_go.msg.dds_._IMUState_ import IMUState_
from unitree_sdk2py.idl.nav_msgs.msg.dds_._Odometry_ import Odometry_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.go2.obstacles_avoid.obstacles_avoid_client import (
    ObstaclesAvoidClient,
)
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
    MotionSwitcherClient,
)
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.go2.video.video_client import VideoClient
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
from ark.tools.log import log
from ark.system.driver.robot_driver import RobotDriver


class UnitreeGo2Driver(RobotDriver):
    def __init__(
        self, component_name: str, component_config: Dict[str, Any] = None
    ) -> None:
        """!
        Initialze the Unitree Go 2 driver

        @param component_name Name of the robot.
        @param component_config Configuration dictionary for the robot.
        """
        super().__init__(component_name, component_config, False)
        self.network_interface = self.config.get("network_interface", "")

        # TODO: Get this from the config
        self.joint_names = [
            "FR_hip_joint",
            "FR_thigh_joint",
            "FR_calf_joint",
            "FL_hip_joint",
            "FL_thigh_joint",
            "FL_calf_joint",
            "RR_hip_joint",
            "RR_thigh_joint",
            "RR_calf_joint",
            "RL_hip_joint",
            "RL_thigh_joint",
            "RL_calf_joint",
        ]

        if self.network_interface != "":
            ChannelFactoryInitialize(0, self.network_interface)
        else:
            ChannelFactoryInitialize(0)

        # TODO; Add high level control

        # Add camera
        self.video_client = VideoClient()  # Create a video client
        self.video_client.SetTimeout(3.0)
        self.video_client.Init()

        # TODO: Add odometry

        # TODO: Add IMU

        self.num_joints = len(self.joint_names)

        msc = MotionSwitcherClient()
        msc.Init()
        msc.ReleaseMode()

        sc = SportClient()
        sc.Init()
        sc.StandDown()

        # self.odom_subscriber = ChannelSubscriber("rt/utlidar/robot_odom", Odometry_)
        # self.odom_subscriber.Init(self.odom_callback)

        self.joint_positions = [0.0] * self.num_joints
        self.joint_velocities = [0.0] * self.num_joints
        self.jont_accelerations = [0.0] * self.num_joints

        # Get subscribers
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateMessageHandler, 10)

        self.lidar_subscriber = ChannelSubscriber("rt/utlidar/cloud", PointCloud2_)
        self.lidar_subscriber.Init(self.LidarMessageHandler, 10)

        self.lowstate_publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowstate_publisher.Init()

    def LidarMessageHandler(self, msg: PointCloud2_):
        self.lidar_data = msg

    def LowStateMessageHandler(self, msg: LowState_):
        self.low_state = msg
        for i in range(12):
            self.joint_positions[i] = msg.motor_state[i].q
            self.joint_velocities[i] = msg.motor_state[i].dq
            self.jont_accelerations[i] = msg.motor_state[i].ddq

        # print("JP:", self.joint_positions)

    def pass_joint_positions(self, joints: List[str]) -> Dict[str, float]:
        # pack self.joint_positions into a dictionary
        joint_positions = {}
        for joint in joints:
            if joint in self.joint_names:
                index = self.joint_names.index(joint)
                joint_positions[joint] = self.joint_positions[index]
            else:
                log.error(f"Joint {joint} not found in joint names.")
        # print("Joint Positions:", joint_positions)
        return joint_positions

    def pass_joint_velocities(self, joints: List[str]) -> Dict[str, float]:
        joint_velocities = {}
        for joint in joints:
            if joint in self.joint_names:
                index = self.joint_names.index(joint)
                joint_velocities[joint] = self.joint_velocities[index]
            else:
                log.error(f"Joint {joint} not found in joint names.")
        # print("Joint Positions:", joint_positions)
        return joint_velocities

    def pass_joint_acceleration(self, joints: List[str]) -> Dict[str, float]:
        jont_accelerations = {}
        for joint in joints:
            if joint in self.joint_names:
                index = self.joint_names.index(joint)
                jont_accelerations[joint] = self.jont_accelerations[index]
            else:
                log.error(f"Joint {joint} not found in joint names.")
        # print("Joint Positions:", joint_positions)
        return jont_accelerations

    def pass_joint_efforts(self, joints: [List[str]]) -> Dict[str, float]:
        raise NotImplementedError

    def pass_joint_group_control_cmd(
        self, control_mode: str, cmd: Dict[str, float], **kwargs
    ) -> None:
        low_cmd = unitree_go_msg_dds__LowCmd_()
        print("Control Mode:", control_mode)
        print("Command:", cmd)
        crc = CRC()
        # extract all the cmd as a list
        cmd_list = list(cmd.values())

        if control_mode == "position":
            for i in range(12):
                low_cmd.motor_cmd[i].mode = 0x01
                low_cmd.motor_cmd[i].q = cmd_list[i]
                low_cmd.motor_cmd[i].dq = 0.0
                low_cmd.motor_cmd[i].kp = 60.0
                low_cmd.motor_cmd[i].kd = 5.0
                low_cmd.motor_cmd[i].tau = 0.0
        # velocity control

        # torque control

        # send the command to the robot dog
        low_cmd.crc = crc.Crc(low_cmd)
        self.lowstate_publisher.Write(low_cmd)

    def pass_lidar_data(self):
        return self.lidar_data

    def pass_camera_image(self):
        code, data = self.video_client.GetImageSample()

        if code != 0:
            log.error("Failed to get Image")
            return np.zeros((480, 640, 3), dtype=np.uint8)
        image_data = np.frombuffer(bytes(data), dtype=np.uint8)
        image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
        return image

    def check_torque_status(self, joints: List[str]) -> Dict[str, bool]:
        raise NotImplementedError

    def shutdown_driver(self):
        if self.control == "task_space":
            self.move_client.Move(0.0, 0.0, 0.0)
            self.move_client.UseRemoteCommandFromApi(False)
