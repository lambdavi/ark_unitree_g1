import time
import copy
import numpy as np
from pathlib import Path
from typing import Dict, Any, List

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_._LowState_ import LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_._MotorState_ import MotorState_
from unitree_sdk2py.idl.unitree_go.msg.dds_._IMUState_ import IMUState_

from ark.system.driver.robot_driver import RobotDriver
from unitree_go_2.unitree_go_2_odometry import UnitreeGo2Odometry


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
        self.run_odometry = self.config.get("odometry", True)

        if self.run_odometry:
            class_path = self.config.get("class_dir", None)
            urdf_path = self.config.get("urdf_path", "urdf/urdf/go2_description.urdf")
            if class_path is not None:
                urdf_path = Path(class_path) / urdf_path
            else:
                urdf_path = Path(urdf_path)

            contact_force_threshold = self.config.get("contact_force_threshold", 20.0)
            self.odometry = UnitreeGo2Odometry(urdf_path=urdf_path, contact_force_threshold=contact_force_threshold)

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
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"
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

    def get_imu(self, msg: IMUState_) -> Dict[str, np.ndarray]:
        # Store orientation as quaternion (x, y, z, w), gyroscope and accelerometer data
        orientation = np.array([msg.quaternion[1], msg.quaternion[2], msg.quaternion[3], msg.quaternion[0]])
        gyro = np.array(msg.gyroscope)
        accel = np.array(msg.accelerometer)

        imu = {
            "orientation": orientation,
            "gyro": gyro,
            "accel": accel
        }
        return imu

    def state_callback(self, msg: LowState_) -> None:
        """!
        Callback function to store the incoming robot data.

        @param msg LowState_ message containing the joint and force data.
        @return None
        """
        joint_state = self.get_joint_state(msg.motor_state)
        force = self.get_force(msg.foot_force)
        imu = self.get_imu(msg.imu_state)

        self.data = {
            "joint_state": joint_state,
            "foot_force": force,
            "imu": imu
        }

    def get_robot_data(self):
        data = copy.deepcopy(self.data)
        if self.run_odometry:
            v_x, v_y, w = self.odometry.run(**data)
            data["odometry"] = {"v_x": v_x, "v_y": v_y, "w": w}

        return data

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
