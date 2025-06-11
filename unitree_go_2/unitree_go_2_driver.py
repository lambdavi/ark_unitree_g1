from typing import Dict, Any, List


from ark.system.driver.robot_driver import RobotDriver


class UnitreeGo2Driver(RobotDriver):
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
