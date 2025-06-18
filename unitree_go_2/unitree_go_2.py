from typing import Any, Dict, Optional
from dataclasses import dataclass
from enum import Enum

from ark.system.component.robot import Robot
from ark.system.driver.robot_driver import RobotDriver
from arktypes import (
    joint_state_t,
    force_t,
    joint_group_command_t,
    string_t,
    pose_2d_t,
    velocity_2d_t,
    float_t
)
from arktypes import pack
from unitree_go_2.unitree_go_2_driver import UnitreeGo2Driver


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
        self._msg =  None

        # Create names
        self.joint_pub_name = self.name + "/joint_states"
        self.force_pub_name = self.name + "/foot_forces"
        self.subscriber_name = self.name + "/joint_group_command"
        self.width_service_name = self.name + "/width"

        if self.sim == True:
            self.joint_pub_name = self.joint_pub_name + "/sim"
            self.force_pub_name = self.force_pub_name + "/sim"
            self.subscriber_name = self.subscriber_name + "/sim"
            self.width_service_name = self.width_service_name + "/sim"

        # Create publishers/subscribers
        component_channels = [(self.joint_pub_name, joint_state_t),
                              (self.force_pub_name, force_t)]
        self.create_subscriber(self.subscriber_name, joint_group_command_t, self._joint_group_command_callback)
        self.component_channels_init(component_channels)

        # Create robot width service
        self.robot_width = driver.config.get("width", 0.31)
        self.create_service(self.width_service_name, string_t, float_t, self.send_robot_width)

    def get_robot_data(self):
        if self._msg:
            msg = self._msg
            group_name = msg.name
            cmd = msg.cmd
            cmd_dict = {}
            for joint, goal in zip(list(self.joint_groups[group_name]["actuated_joints"]), cmd):
                cmd_dict[joint] = goal
            self._msg = None
            self.control_joint_group(group_name, cmd_dict)

        data = self._driver.get_robot_data()
        return data

    def pack_data(self, data):
        joint_msg = pack.pack_joint_state(**data["joint_state"])
        force_msg = pack.pack_force(**data["force"])
        msgs = {self.joint_pub_name: joint_msg,
                self.force_pub_name: force_msg}
        return msgs

    def step_component(self):
        data = self.get_robot_data()
        if data is not None:
            packed = self.pack_data(data)
            self.component_multi_publisher.publish(packed)

    def _joint_group_command_callback(self, t, channel_name, msg):
        self._msg = msg

    def send_robot_width(self, channel: str, msg: string_t) -> float_t:
        msg = pack.pack_float(self.robot_width)
        return msg