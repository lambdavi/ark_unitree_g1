import numpy as np
import pinocchio as pin

from pathlib import Path
from pinocchio.robot_wrapper import RobotWrapper
from typing import Any, Dict, Tuple, List

from ark.tools.log import log

class UnitreeGo2Odometry:
    """!
    Implements odometry for the Unitree Go 2 based off of https://github.com/inria-paris-robotics-lab/go2_odometry/tree/master

    This method uses the joint positions, joint velocities, and foot contact forces to estimate the base's linear (v_x, v_y) and angular (w) velocities,
    by using kinematic equations given the known configuraton of the robot.
    """
    def __init__(self, urdf_path: Path, contact_force_threshold: float):
        package_dir = urdf_path.parent.resolve()
        self.robot = RobotWrapper.BuildFromURDF(str(urdf_path), package_dirs=[str(package_dir)], root_joint=pin.JointModelFreeFlyer())
        self.model = self.robot.model
        self.data = self.robot.data

        self.ee_names = {
            'FL': 'FL_foot',
            'FR': 'FR_foot',
            'RL': 'RL_foot',
            'RR': 'RR_foot'
        }

        self.leg_joint_names = {
            'FL': ['FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint'],
            'FR': ['FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint'],
            'RL': ['RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint'],
            'RR': ['RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint']
        }

        self.joint_name_to_index = {name: self.model.getJointId(name) for names in self.leg_joint_names.values() for name in names}
        self.contact_force_threshold = contact_force_threshold  # N, adjust based on real sensor behavior

    def run(
        self,
        joint_state: Dict[str, Any],
        foot_force: Dict[str, np.ndarray],
        imu: Dict[str, Any]
    ) -> Tuple[float, float, float]:
        """
        Computes (vx, vy, wz) from joint states, foot forces, and IMU.
        """
        joint_pos = joint_state["position"]
        joint_vel = joint_state["velocity"]
        joint_names = joint_state["name"]
        foot_force = dict(zip(foot_force["name"], foot_force["force"].tolist()))

        # Assemble full q, dq with free-flyer
        nq = self.model.nq
        nv = self.model.nv
        q = np.zeros(nq)
        v = np.zeros(nv)

        # Base pose set to identity (or if available from state estimator, use that)
        q[:7] = np.array([0, 0, 0, 0, 0, 0, 1])  # [x y z qx qy qz qw]
        v[:6] = 0  # base velocity is what we're solving for

        # Fill in joint states
        for name, pos, vel in zip(joint_names, joint_pos, joint_vel):
            if name in self.joint_name_to_index:
                idx = self.model.getJointId(name)
                q_idx = self.model.joints[idx].idx_q
                v_idx = self.model.joints[idx].idx_v
                q[q_idx] = pos
                v[v_idx] = vel

        pin.forwardKinematics(self.model, self.data, q, v)
        pin.updateFramePlacements(self.model, self.data)

        A = []
        b = []

        # For each stance leg
        for leg, foot_name in self.ee_names.items():
            force = foot_force.get(leg, 0)
            if force < self.contact_force_threshold:
                continue  # not in contact

            frame_id = self.model.getFrameId(foot_name)
            foot_pos = self.data.oMf[frame_id].translation  # in base frame
            J = pin.computeFrameJacobian(self.model, self.data, q, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            J_lin = J[:3, :6]  # linear velocity wrt base motion

            # Compute foot spatial velocity (J * v)
            v_foot = J @ v

            # Assume foot world velocity is zero (stance phase)
            A.append(J_lin)
            b.append(-v_foot[:3])  # base linear velocity makes foot move; invert that

        if len(A) >= 2:
            A = np.vstack(A)
            b = np.hstack(b)
            base_twist, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
            vx, vy = base_twist[0], base_twist[1]
        else:
            log.warning(f"Not enough feet are contacted with the ground (Num Contacts = {len(A)}) to solve odometry")
            vx, vy = 0.0, 0.0

        # Angular velocity from IMU
        angular_vel = imu.get("gyro", np.zeros(3))
        w = angular_vel[2]  # yaw rate

        return vx, vy, w
