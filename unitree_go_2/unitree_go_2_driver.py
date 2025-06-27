import sys
import time
import copy
import numpy as np
from pathlib import Path
from typing import Dict, Any, List

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber, ChannelPublisher
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_._MotorState_ import MotorState_
from unitree_sdk2py.idl.unitree_go.msg.dds_._IMUState_ import IMUState_
from unitree_sdk2py.idl.nav_msgs.msg.dds_._Odometry_ import Odometry_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.go2.obstacles_avoid.obstacles_avoid_client import ObstaclesAvoidClient
from unitree_sdk2py.utils.crc import CRC

from ark.tools.log import log
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
        self.control = self.config.get("control", "task_space")
        self.joint_names = [
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"
        ]
        self.num_joints = len(self.joint_names)

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


        # Get subscribers
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.lowstate_callback)

        self.odom_subscriber = ChannelSubscriber("rt/utlidar/robot_odom", Odometry_)
        self.odom_subscriber.Init(self.odom_callback)

        # Set control mode
        if self.control == "task_space":
            self.init_task_space_control()
        elif self.control == "joint_space":
            self.init_joint_space_control()
        else:
            log.error(f"Control mode: {self.control} not implemented")
            raise NotImplementedError

        self.lowstate = None
        self.unitree_odom = None

    def init_task_space_control(self):

        # Create obstacle avoidance client for moving
        self.move_client = ObstaclesAvoidClient()
        self.move_client.SetTimeout(3.0)
        self.move_client.Init()

        # Set obstacle avoidance
        spinner = ['|', '/', '-', '\\']
        i = 0
        while not self.move_client.SwitchGet()[1]:
            self.move_client.SwitchSet(True)
            sys.stdout.write('\rWaiting for obstacle avoidance switch... ' + spinner[i % len(spinner)])
            sys.stdout.flush()
            i += 1
            time.sleep(0.1)
        log.ok("Set Obstacle Avoidance Mode for Unitree Go 2")

        self.move_client.UseRemoteCommandFromApi(True)
        time.sleep(0.5)
        log.ok("Set to use remote commands from Unitree SDK")
        log.ok("Initialized Unitree Go 2 Control Mode: Task Space Control")

    def init_joint_space_control(self):
        # Values to indicate to not move the joint
        PosStopF = 2.146e9
        VelStopF = 16000.0

        # Create publisher
        self.lowcmd_publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher.Init()

        # Create default joint limits. (Taken from https://support.unitree.com/home/en/developer and subtracted 4 degrees)
        HIP_MIN = -0.767945
        HIP_MAX = 0.767945
        THIGH_MIN = -1.50098
        THIGH_MAX = 3.42085
        CALF_MIN = -2.6529
        CALF_MAX = -0.767945
        default_joint_min = [
            HIP_MIN, THIGH_MIN, CALF_MIN,
            HIP_MIN, THIGH_MIN, CALF_MIN,
            HIP_MIN, THIGH_MIN, CALF_MIN,
        ]
        default_joint_max = [
            HIP_MAX, THIGH_MAX, CALF_MAX,
            HIP_MAX, THIGH_MAX, CALF_MAX,
            HIP_MAX, THIGH_MAX, CALF_MAX,
        ]
        # Create joint space control config
        default_joint_space_control_config = {
            "Kp": 60.0,
            "Kd": 5.0,
            "joint_min": default_joint_min,
            "joint_max": default_joint_max
        }
        self.joint_space_control_config = self.config.get("joint_space_control", default_joint_space_control_config)
        self.Kp = self.joint_space_control_config["Kp"]
        self.Kd = self.joint_space_control_config["Kd"]
        self.joint_min = np.array(self.joint_space_control_config["joint_min"])
        self.joint_max = np.array(self.joint_space_control_config["joint_max"])

        # Create default joint command
        self.default_joint_cmd = unitree_go_msg_dds__LowCmd_()
        self.default_joint_cmd.head[0] = 0xFE
        self.default_joint_cmd.head[1] = 0xEF
        self.default_joint_cmd.level_flag = 0xFF
        self.default_joint_cmd.gpio = 0
        for i in range(20):
            self.default_joint_cmd.motor_cmd[i].mode = 0x01  # (PMSM) mode
            self.default_joint_cmd.motor_cmd[i].q = PosStopF
            self.default_joint_cmd.motor_cmd[i].kp = 0
            self.default_joint_cmd.motor_cmd[i].dq = VelStopF
            self.default_joint_cmd.motor_cmd[i].kd = 0
            self.default_joint_cmd.motor_cmd[i].tau = 0

        self.crc = CRC()
        log.ok("Initialized Unitree Go 2 Control Mode: Joint Space Control")

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
        positions, velocities, efforts = [], [], []
        for i in range(len(self.joint_names)):
            positions.append(msgs[i].q)
            velocities.append(msgs[i].dq)
            efforts.append(msgs[i].tau_est)

        joint_state = {"header": header,
                       "name": self.joint_names,
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

    def lowstate_callback(self, msg: LowState_) -> None:
        """!
        Callback function to store the incoming robot data.

        @param msg LowState_ message containing
        @return None
        """
        joint_state = self.get_joint_state(msg.motor_state)
        force = self.get_force(msg.foot_force)
        imu = self.get_imu(msg.imu_state)

        self.lowstate = {
            "joint_state": joint_state,
            "foot_force": force,
            "imu": imu
        }

    def odom_callback(self, msg: Odometry_) -> None:
        """!
        Get the velocities from Unitree Go 2 odometry
        """
        twist = msg.twist.twist
        self.unitree_odom = {"v_x": twist.linear.x,
                             "v_y": twist.linear.y,
                             "w": twist.angular.z}

    def get_state(self):
        state = {}
        lowstate = copy.deepcopy(self.lowstate)
        unitree_odom = copy.deepcopy(self.unitree_odom)

        if lowstate is not None:
            state.update(lowstate)

        if unitree_odom is not None:
            state["unitree_odometry"] = unitree_odom

        if self.run_odometry and lowstate is not None:
            v_x, v_y, w = self.odometry.run(**lowstate)
            state["odometry"] = {"v_x": v_x, "v_y": v_y, "w": w}

        return state

    def control_robot_task_space(self, v_x, v_y, w):
        if self.control != "task_space":
            log.error(f"[Control Mode Error] Expected 'task_space' but got '{self.control}'. Cannot execute task-space velocity command.")
            return

        self.move_client.Move(v_x, v_y, w)

    def control_robot_joint_space(self, joint_angles):
        if self.control != "joint_space":
            log.error(f"[Control Mode Error] Expected 'joint_space' but got '{self.control}'. Cannot execute joint-space command.")
            return

        if len(joint_angles) != self.num_joints:
            log.error(f"Invalid joint command length: expected {self.num_joints}, got {len(joint_angles)}")
            return

        # Clip to joint limits
        joint_angles = np.array(joint_angles)
        joint_angles = np.clip(joint_angles, self.joint_min, self.joint_max)

        # Construct command
        joint_cmd = copy.deepcopy(self.default_joint_cmd)
        joint_cmd.crc = self.crc.Crc(joint_cmd)

        for i in range(self.num_joints):
            joint_cmd.motor_cmd[i].q = joint_angles[i]
            joint_cmd.motor_cmd[i].dq = 0.0
            joint_cmd.motor_cmd[i].kp = self.Kp
            joint_cmd.motor_cmd[i].kd = self.Kd
            joint_cmd.motor_cmd[i].tau = 0

        # Send command to Unitree Go 2
        self.lowcmd_publisher.Write(joint_cmd)

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
        if self.control == "task_space":
            self.move_client.Move(0.0, 0.0, 0.0)
            self.move_client.UseRemoteCommandFromApi(False)