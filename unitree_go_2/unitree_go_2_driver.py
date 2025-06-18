import time
import numpy as np
from typing import Dict, Any, List

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_._LowState_ import LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_._MotorState_ import MotorState_

from ark.system.driver.robot_driver import RobotDriver


class UnitreeGo2Driver(RobotDriver):
    def __init__(self,
                 component_name: str,
                 component_config: Dict[str, Any] = None
                 ) -> None:
        """!
        Initialze the Unitree Go 2 driver

        @param component_name Name of the robot.
        @param component_config Configuration dictionary for the robot.
        """
        super().__init__(component_name, component_config, False)
        self.network_interface = self.config.get("network_interface", "")
        self.start = time.perf_counter()
        self.seq = 0
        self.frame_id = "Robot State"

        if self.network_interface != "":
            ChannelFactoryInitialize(0, self.network_interface)
        else:
            ChannelFactoryInitialize(0)

        self.subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.subscriber.Init(self.state_callback)
        self.data = None

    def get_header(self):
        timestamp =  time.perf_counter() - self.start
        sec = int(timestamp)
        nsec = int((timestamp - sec) * 1e9)
        stamp = {"sec": sec,
                 "nsec":  nsec}
        header = {"seq": self.seq,
                  "stamp": stamp,
                  "frame_id": self.frame_id}
        self.seq += 1
        return header

    def get_joint_state(self, msgs: List[MotorState_]) ->  Dict[str, Any]:
        header = self.get_header()
        names = [
            "FR_0", "FR_1", "FR_2",
            "FL_0", "FL_1", "FL_2",
            "RR_0", "RR_1", "RR_2",
            "RL_0", "RL_1", "RL_2"
        ]
        positions, velocities, efforts = [], [], []
        for i in range(len(names)):
            positions.append(msgs[i].q)
            velocities.append(msgs[i].dq)
            efforts.append(msgs[i].tau_est)

        joint_state = {"header": header,
                       "name": names,
                       "position": np.array(positions),
                       "velocity": np.array(velocities),
                       "effort": np.array(efforts)
        }
        return joint_state

    def get_force(self, force: List[float]) -> Dict[str, np.ndarray]:
        force = np.array(force).astype(np.float32)
        force = {"name": ["FR", "FL", "RR", "RL"],
                 "force": force}
        return force

    def state_callback(self, msg: LowState_) -> None:
        """!
        Callback function to store the incoming robot data.

        @param msg LowState_ message containing the joint and force data.
        @return None
        """
        joint_state = self.get_joint_state(msg.motor_state)
        force = self.get_force(msg.foot_force)

        self.data = {
            "joint_state": joint_state,
            "force": force
        }

    def get_robot_data(self):
        return self.data

    def check_torque_status(self) -> bool:
        raise NotImplementedError

    def pass_joint_positions(self, joints: List[str]) -> Dict[str, float]:
        raise NotImplementedError

    def pass_joint_velocities(self, joints: List[str]) -> Dict[str, float]:
        raise NotImplementedError

    def pass_joint_efforts(self, joints: List[str]) -> Dict[str, float]:
        raise NotImplementedError

    def pass_joint_group_control_cmd(self, control_mode: str, joints: List[str], cmd: Dict[str, float], **kwargs) -> None:
        raise NotImplementedError

    def shutdown_driver(self):
        pass
