# Unitree Go 2

## Install
```
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip install -e .
cd ..
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