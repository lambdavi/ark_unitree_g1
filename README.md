# Unitree Go 2

## Install
```
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip install -e .
cd ..
pip install -r requirements.txt
```
## How to Run
You can run the Unitree Go 2 node via:
```
python main.py
```

If you would like to control the robot with a keyboard, start the teleoperation node:
```
python teleop_node.py

Controls:
W/S: Move forward/back
A/D: Move left/right
Q/E: Turn CCW/CW
```

## How to View Cyclone DSS Topics
Unitree Go 2 uses Cyclone DDS as a form of middleware, for publishing and subcribing to different topics. To view a list of all topics:

#### Install Cyclone DDS
```
# Clone repo
git clone https://github.com/eclipse-cyclonedds/cyclonedds.git
cd cyclonedds

# Build with CMake
mkdir build && cd build
cmake -DBUILD_EXAMPLES=ON -DENABLE_TOPIC_DISCOVERY=ON ..
make -j$(nproc)
```

#### Set your config
```
export CYCLONEDDS_URI=file://$PWD/cyclonedds-config.xml
```

#### Run the list topics example
```
./bin/listtopics
```