# ros2_asus_xtion

## Installation

```shell
$ cd ~/ros2_ws/src
$ git clone --recurse-submodules https://github.com/mgonzs13/ros2_asus_xtion
$ cd ~/ros2_ws
$ rosdep install --from-paths src --ignore-src -r -y
$ colcon build
```

## Usage

```shell
$ ros2 launch asus_xtion asus_xtion.launch.py
```

```shell
$ ros2 launch asus_xtion_visualization rviz2.launch.py
```
