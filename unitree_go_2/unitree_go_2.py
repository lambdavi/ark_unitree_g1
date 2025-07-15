from typing import Any, Dict, Optional
from dataclasses import dataclass
from enum import Enum
from ark.client.comm_infrastructure.base_node import main
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
from arktypes.utils import pack, unpack
from unitree_go_2_driver import UnitreeGo2Driver
# from unitree_go_2.unitree_go_2_plotter import UnitreeGo2Plotter
from ark.system.pybullet.pybullet_robot_driver import BulletRobotDriver

@dataclass
class Drivers(Enum):
    PYBULLET_DRIVER = BulletRobotDriver
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
            subscriber_callback = self._joint_group_command_callback

        if self.sim == True:
            self.joint_pub_name = self.joint_pub_name + "/sim"
            self.force_pub_name = self.force_pub_name + "/sim"
            self.imu_pub_name = self.imu_pub_name + "/sim"
            self.subscriber_name = self.subscriber_name + "/sim"
            self.width_service_name = self.width_service_name + "/sim"

        self.component_channels = {self.joint_pub_name: joint_state_t,
                                   self.force_pub_name: force_t,
                                   self.imu_pub_name: imu_t}

        self.odometry = driver.config.get("odometry", True)
        if self.odometry:
            self.init_odometry()

        self.create_subscriber(self.subscriber_name, self.subscriber_type, subscriber_callback)
        self.component_channels_init(self.component_channels)

        # Create robot width service
        self.robot_width = driver.config.get("width", 0.31)
        self.create_service(self.width_service_name, string_t, float_t, self.send_robot_width)
        self.joint_group_command = None

    def control_robot(self):
        if self.joint_group_command:
            cmd_dict = {}
            group_name = self.joint_group_command['name']
            for joint, goal in zip(list(self.joint_groups[self.joint_group_command['name']]["actuated_joints"]), self.joint_group_command['cmd']):
                cmd_dict[joint] = goal
            self._joint_cmd_msg = None
            control_mode = self.joint_groups[group_name]["control_mode"]
            self.control_joint_group(control_mode, cmd_dict)

    def get_state(self):
        return self.get_joint_positions()
    
    def pack_data(self, state):
        msg = joint_state_t()
        msg.n = len(state)
        msg.name = list(state.keys())
        msg.position = list(state.values())
        msg.velocity = [0.0] * msg.n
        msg.effort = [0.0] * msg.n

        return {
            self.joint_pub_name: msg
        }

    def _joint_group_command_callback(self, t, channel_name, msg):
        cmd, name = unpack.joint_group_command(msg)
        self.joint_group_command = {
            "cmd": cmd,
            "name": name,
        }

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

CONFIG_PATH = "/home/sarthakdas/Ark/ark_unitree_go_2/tests/go2_pybullet_sim/config/global_config.yaml"
if __name__ == "__main__":
    # raise NotImplementedError("This robot is not meant to be run as a standalone node. Please use the ark simulator")
    name = "unitree_go_2"
    driver = UnitreeGo2Driver(name, CONFIG_PATH)
    main(UnitreeGo2, name, CONFIG_PATH, driver)