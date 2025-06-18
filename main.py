from unitree_go_2.unitree_go_2 import UnitreeGo2
from unitree_go_2.unitree_go_2_driver import UnitreeGo2Driver

from ark.client.comm_infrastructure.base_node import main

CONFIG_PATH = "config/global.yaml"

if __name__ == "__main__":
    name = "UnitreeGo2"
    driver = UnitreeGo2Driver(component_name=name, component_config=CONFIG_PATH)
    main(UnitreeGo2, name, CONFIG_PATH, driver)