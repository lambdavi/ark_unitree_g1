from ark.client.comm_infrastructure.base_node import BaseNode,  main
from ark.tools.log import log
from arktypes import string_t, joint_state_t, joint_group_command_t
from arktypes.utils import pack, unpack
import time
from typing import Dict, Any, Optional

class ListenerNode(BaseNode):

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        super().__init__("Real to Sim")
        self.create_subscriber("unitree_go_2/joint_states", joint_state_t, self.callback)
        self.pub = self.create_publisher("unitree_go_2_sim/joint_group_command/sim", joint_group_command_t)

    def callback(self, t, channel_name, msg):
        _, _, joint_data, _, _ = unpack.joint_state(msg)
        print("Received joint state:", joint_data)
        joint_command_msg = pack.joint_group_command(joint_data, 'all')
        self.pub.publish(joint_command_msg)


if __name__ == "__main__":
    main(ListenerNode)