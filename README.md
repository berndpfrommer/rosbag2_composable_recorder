# rosbag2 composable recorder

A composable recorder node for ROS2 to allow recording rosbags without
inter process communication.

NOTE: If you are running on ROS2 Jazzy or later, do not use this
repo. A
[composable recorder node](https://github.com/ros2/rosbag2?tab=readme-ov-file#using-with-composition)
is now part of the standard rosbag2 framework.

## Supported platforms

Currently only tested under Ubuntu 20.04/22.04. Will require ROS2 Galactic
or later.

## How to build
Create a workspace (e.g. ``ros2_ws``), clone this repo, and build:
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone -b galactic https://github.com/ros2/rosbag2.git
git clone https://github.com/berndpfrommer/rosbag2_composable_recorder
cd ..
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
. install/setup.bash
```

## How to run

This recorder node is meant to be used as a
[composable node](https://docs.ros.org/en/foxy/Tutorials/Composition.html) together
with other nodes that produce a large number of messages. The launch
file ``recorder.launch.py`` can serve as a template. Make a copy, edit
it to your liking, then launch it.
```
ros2 launch rosbag2_composable_recorder recorder.launch.py

```
Setting the parameter ``start_recording_immediately`` will cause the
recording to start immediately. To start the recording later:
```
ros2 service call /start_recording std_srvs/srv/Trigger
```
Alternatively, use the python script:
```
ros2 run rosbag2_composable_recorder start_recording.py
```
To stop the recording, either kill (Ctrl-C) the recording driver or call the stop service:
```
ros2 service call /stop_recording std_srvs/srv/Trigger
```

## Parameters

- ``bag_name``: (default: empty) prefix of directory name used for
    bag. Note: if you stop the recording and restart, the recorder node
    will throw an exception and exit because the bag already
    exists. Use ``bag_prefix`` in this case.
- ``bag_prefix``: (default: ``rosbag_2``) prefix of directory name used for storage. A
    timestamp will be appended. This parameter is only used when no
    ``bag_name`` is specified.
- ``topics``: (default: empty) array of strings that specifies the topics to record.
- ``record_all``: (default: False) when this is set, all topics are recorded.
- ``disable_discovery``: (default: False) disable discovery of topics
    that occured after recording was launched.
- ``storage_id``: (default: sqlite3) storage container format.
- ``serialization_format``: (default: cdr) serialization format.
- ``max_cache_size``: (default: 100MB) size (in bytes) of cache before
    writing to disk. See ``ros2 bag record --help`` for more.
- ``start_recording_immediately``: (default: False) do not wait for
    service call before recording is started.


## License

This software is issued under the Apache License Version 2.0.
