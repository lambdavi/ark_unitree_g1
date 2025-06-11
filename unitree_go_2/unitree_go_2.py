from typing import Any, Dict, Optional
from dataclasses import dataclass
from enum import Enum

from ark.system.component.robot import Robot
from ark.system.driver.robot_driver import RobotDriver
from arktypes import (
    joint_state_t, 
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
        self.subscriber_name = self.name + "/joint_group_command"
        self.width_service_name = self.name + "/width"
        component_channels = [(self.joint_pub_name, joint_state_t)]

        if self.sim == True:
            self.joint_pub_name = self.joint_pub_name + "/sim"
            self.actual_pose_pub_name = self.name + "/actual_pose" + "/sim"
            self.actual_velocity_pub_name = self.name + "/actual_velocity" + "/sim"  
            self.subscriber_name = self.subscriber_name + "/sim"
            self.width_service_name = self.width_service_name + "/sim"

            component_channels.append((self.actual_pose_pub_name, pose_2d_t))
            component_channels.append((self.actual_velocity_pub_name, velocity_2d_t))

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

        robot_info = {
            "joint_positions": self.get_joint_positions(),
        }

        if self.sim:
            robot_info["base_state"] = self.get_base_state()
        return robot_info 

    def get_base_state(self):
        return self._driver.get_base_state()

    def pack_data(self, robot_info):
        pos_dict = robot_info["joint_positions"]
        msg = joint_state_t()
        msg.n = len(pos_dict)
        msg.name = list(pos_dict.keys())
        msg.position = list(pos_dict.values())
        msg.velocity = [0.0] * msg.n
        msg.effort = [0.0] * msg.n

        msgs = {self.joint_pub_name: msg}

        if self.sim:
            base_state = robot_info["base_state"]
            pose_msg = pack.pack_pose_2d(x=base_state["x"], y=base_state["y"], theta=base_state["theta"])
            velocity_msg = pack.pack_velocity_2d(linear_velocity=base_state["linear_velocity"], angular_velocity=base_state["angular_velocity"])
            msgs[self.actual_pose_pub_name] = pose_msg
            msgs[self.actual_velocity_pub_name] = velocity_msg

        return msgs
           
    def _joint_group_command_callback(self, t, channel_name, msg):
        self._msg = msg

    def send_robot_width(self, channel: str, msg: string_t) -> float_t:
        msg = pack.pack_float(self.robot_width)
        return msg