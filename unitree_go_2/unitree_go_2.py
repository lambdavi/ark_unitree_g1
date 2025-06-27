from typing import Any, Dict, Optional
from dataclasses import dataclass
from enum import Enum

from ark.system.component.robot import Robot
from ark.system.driver.robot_driver import RobotDriver
from arktypes import (
    joint_state_t,
    force_t,
    imu_t,
    joint_group_command_t,
    string_t,
    pose_2d_t,
    velocity_2d_t,
    float_t
)
from arktypes import pack, unpack
from unitree_go_2.unitree_go_2_driver import UnitreeGo2Driver
from unitree_go_2.unitree_go_2_plotter import UnitreeGo2Plotter


@dataclass
class Drivers(Enum):
    PYBULLET_DRIVER = None
    DRIVER = UnitreeGo2Driver

class UnitreeGo2(Robot):
    def __init__(self,
                 name: str,
                 global_config: Dict[str, Any] = None,
                 driver: RobotDriver = None,
                 ) -> None:

        super().__init__(name = name,
                         global_config = global_config,
                         driver = driver,
                         )
        self.state = None

        control_mode = driver.config.get("control", "task_space")

        # Create names
        self.joint_pub_name = self.name + "/joint_states"
        self.force_pub_name = self.name + "/foot_forces"
        self.imu_pub_name = self.name + "/imu"
        self.width_service_name = self.name + "/width"

        if control_mode == "task_space":
            self.subscriber_name = self.name + "/control_velocity"
            self.subscriber_type = velocity_2d_t
            subscriber_callback = self.control_robot_task_space
        elif control_mode == "joint_space":
            self.subscriber_name = self.name + "/joint_group_command"
            self.subscriber_type = joint_group_command_t
            subscriber_callback = self.control_robot_joint_space

        if self.sim == True:
            self.joint_pub_name = self.joint_pub_name + "/sim"
            self.force_pub_name = self.force_pub_name + "/sim"
            self.imu_pub_name = self.imu_pub_name + "/sim"
            self.subscriber_name = self.subscriber_name + "/sim"
            self.width_service_name = self.width_service_name + "/sim"

        self.component_channels = [(self.joint_pub_name, joint_state_t),
                                   (self.force_pub_name, force_t),
                                   (self.imu_pub_name, imu_t)]

        self.odometry = driver.config.get("odometry", True)
        if self.odometry:
            self.init_odometry()

        self.create_subscriber(self.subscriber_name, self.subscriber_type, subscriber_callback)
        self.component_channels_init(self.component_channels)

        # Create robot width service
        self.robot_width = driver.config.get("width", 0.31)
        self.create_service(self.width_service_name, string_t, float_t, self.send_robot_width)

    def init_odometry(self):
        # Init publish channel
        self.odometry_pub_name = self.name + "/odometry"
        if self.sim:
            self.odometry_pub_name = self.odometry_pub_name + "/sim"
        self.component_channels.append((self.odometry_pub_name, velocity_2d_t))

        # Init plotter
        self.show = self._driver.config.get("show_odometry", False)
        if self.show:
            frequency = self._driver.config.get("show_frequency", 30)
            self.plotter = UnitreeGo2Plotter()
            self.create_stepper(frequency, self.update_plotter)

    def update_plotter(self):
        if self.state is not None:
            self.plotter.update(self.state)

    def get_robot_data(self):
        pass

    def get_state(self):
        self.state = self._driver.get_state()

    def pack_data(self):
        joint_msg = pack.pack_joint_state(**self.state["joint_state"])
        force_msg = pack.pack_force(**self.state["foot_force"])
        imu_msg = pack.pack_imu(**self.state["imu"])
        msgs = {self.joint_pub_name: joint_msg,
                self.force_pub_name: force_msg,
                self.imu_pub_name: imu_msg}

        if self.odometry:
            odom_msg = pack.pack_velocity_2d(**self.state["odometry"])
            msgs[self.odometry_pub_name] = odom_msg
        return msgs

    def step_component(self):
        self.get_state()
        if self.state is not None:
            packed = self.pack_data()
            self.component_multi_publisher.publish(packed)

    def control_robot_joint_space(self, t, channel_name, msg):
        joint_group_command, _ = unpack.unpack_joint_group_command(msg)
        self._driver.control_robot_joint_space(joint_group_command)

    def control_robot_task_space(self, t, channel_name, msg):
        v_x, v_y, w = unpack.unpack_velocity_2d(msg)
        self._driver.control_robot_task_space(v_x=v_x, v_y=v_y, w=w)

    def send_robot_width(self, channel: str, msg: string_t) -> float_t:
        msg = pack.pack_float(self.robot_width)
        return msg

    def suspend_node(self):
        super().suspend_node()
        if self.odometry:
            if self.show:
                self.plotter.stop()