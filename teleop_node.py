import termios
import tty
import sys
import signal
from select import select

from ark.client.comm_infrastructure.base_node import BaseNode, main
from ark.tools.log import log
from arktypes import velocity_2d_t, pack

CONFIG_PATH = "config/global.yaml"

class TeleopNode(BaseNode):
    """!
    Node to perform teleoperation on Unitree Go 2.

    Controls:
    W/S: Move forward/back
    A/D: Move left/right
    Q/E: Turn CCW/CW
    """
    def __init__(self, path: str) -> None:
        """!
        Initialize the save map node.

        @param path  Path to the configuration file.
        """
        super().__init__("teleop", path)
        robot_name = "UnitreeGo2"
        self.frequency = self.config.get("frequency", 120)
        self.linear_velocity = self.config.get("linear_velocity", 0.3)
        self.angular_velocity = self.config.get("angular_velocity", 0.3)

        # Keyboard
        self.settings = termios.tcgetattr(sys.stdin)
        self.key_timeout = 1 / self.frequency
        signal.signal(signal.SIGINT, self.signal_handler)

        # Get control channel
        network_config = self.global_config.get("network", {})
        sim = network_config.get("sim", True)
        control_channel = robot_name + "/control_velocity"
        if sim:
            control_channel = control_channel + "/sim"

        # Setup publisher and stepper
        self.pub = self.create_publisher(control_channel, velocity_2d_t)
        self.create_stepper(self.frequency, self.step)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        rlist, _, _ = select([sys.stdin], [], [], self.key_timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def get_control_velocity(self):
        """!
        Gets the control velocity based on keyboard commands
        @return: Tuple (v_x, v_y, w)
        """

        # Define base speed
        v_x = 0.0
        v_y = 0.0
        w = 0.0

        key = self.get_key()

        # Check for quit command
        if key == '\x03':  # Ctrl+C
            self.signal_handler(None, None)

        # Check key states (non-blocking)
        if key == 'w':
            v_x = self.linear_velocity
        if key == 's':
            v_x = -self.linear_velocity
        if key == 'a':
            v_y = self.linear_velocity
        if key == 'd':
            v_y = -self.linear_velocity
        if key == 'q':
            w = self.angular_velocity
        if key == 'e':
            w = -self.angular_velocity

        return v_x, v_y, w

    def step(self):
        v_x, v_y, w = self.get_control_velocity()
        msg = pack.pack_velocity_2d(v_x=v_x, v_y=v_y, w=w)
        self.pub.publish(msg)

    def signal_handler(self, signum, frame):
        """Handle Ctrl+C gracefully"""
        log.ok("\nStopped listening for keyboard commands in teleop Node")
        self.suspend_node()
        sys.exit(0)

    def suspend_node(self):
        """Restore original terminal settings"""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        super().suspend_node()



if __name__ == "__main__":
    main(TeleopNode, CONFIG_PATH)