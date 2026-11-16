---
title: rosbag2
description: "rosbag2 records and replays the messages flowing through a ROS 2 system. Covers recording, compression, QoS overrides, inspecting and converting bags, reading and writing them from Python, the MCAP default format, the 2026 additions (remote record and stop services, Python playback control, circular recording, message-loss reporting), and bags as the raw material for robot-learning datasets."
aliases:
  - ros2 bag
  - rosbag
  - MCAP
tags:
  - rosbag2
  - ros2
  - data-recording
  - debugging
  - testing
  - robotics-software
  - dataset
type: tool
level: intermediate
subfield:
  - software
status: reviewed
author: Jordan_Smith
date: 2025-04-28
last_reviewed: 2026-11-15
as_of: 2026-11
prerequisites:
  - "[[ROS_2_Overview]]"
related:
  - "[[RViz_Tutorial]]"
  - "[[Robot_Learning_Datasets_and_Benchmarks]]"
  - "[[Teleoperation_and_Data_Collection]]"
  - "[[TF_and_Topic_Architecture]]"
key_takeaways:
  - "A bag is a time-stamped log of topics; replaying it reproduces what the robot saw, so perception and planning bugs can be debugged off the robot."
  - "MCAP is the default storage format (SQLite3 remains available); it is indexed, compressible per chunk and opens directly in Foxglove."
  - "Lyrical (2026) made recording controllable by services and from Python, added circular recording and message-loss reporting, which suits always-on data collection for learning."
sources:
  - "Open Robotics (2026). rosbag2: ROS 2 bag recording and playback. https://github.com/ros2/rosbag2"
  - "Open Robotics (2026). ROS 2 Lyrical Luth release notes (rosbag2 services, Python control, circular recording). https://github.com/ros2/ros2_documentation/blob/rolling/source/Releases/Release-Lyrical-Luth.rst"
  - "Foxglove (2026). MCAP: a container file format for multimodal log data. https://github.com/foxglove/mcap"
  - "Cadene, R. et al. (2024). LeRobot: State-of-the-art Machine Learning for Real-World Robotics in Pytorch. https://github.com/huggingface/lerobot"
---

# rosbag2

**rosbag2** is the ROS 2 tool for recording and playing back data published on ROS [[ROS_2_Communication|Topics]]. It captures a timestamped log of messages across any number of topics into a **bag file** — a portable, queryable data archive that can be replayed to reproduce system behavior exactly as it occurred.

rosbag2 is a successor to `rosbag` from ROS 1, with significant improvements: a pluggable storage backend (MCAP by default since Iron in 2023; SQLite3 also bundled), improved performance for high-bandwidth data (cameras, LiDAR), and a flexible serialization layer.

---

## Why Use rosbag2?

* **Debugging**: Record a failure on a real robot, then replay the bag file at your desk to investigate the issue without needing the robot.
* **Algorithm development**: Record sensor data once and replay it repeatedly to develop and test algorithms (SLAM, perception, localization) offline.
* **Regression testing**: Replay known bags through updated software to verify behavior hasn't changed.
* **Dataset collection**: Build labeled datasets for machine learning from real robot data.
* **Simulation**: Use bags as a source of realistic sensor data for algorithm development without a full physics simulation.
* **Post-mortem analysis**: Reconstruct what happened before a crash or unexpected behavior.

---

## Recording

### Basic Recording

```bash
# Record all topics
ros2 bag record -a

# Record specific topics
ros2 bag record /scan /odom /camera/image_raw /cmd_vel /tf /tf_static

# Record with a custom output name
ros2 bag record -o my_experiment /scan /odom /camera/image_raw

# Record with a duration limit (seconds)
ros2 bag record -a --duration 60

# Record with a file size limit (bytes)
ros2 bag record -a --max-bag-size 1073741824  # 1 GB
```

This creates a directory `my_experiment/` (or `rosbag2_YYYY_MM_DD-HH_MM_SS/` by default) containing:
* `my_experiment_0.mcap` — the messages, in MCAP format (`.db3` if you chose SQLite3).
* `metadata.yaml` — Bag metadata: topics, message types, duration, message counts.

### Compression

For storage efficiency, especially with point cloud or image data:

```bash
# Record with zstd compression (best compression/speed tradeoff)
ros2 bag record -a --compression-mode file --compression-format zstd

# Record with per-message compression
ros2 bag record -a --compression-mode message --compression-format zstd
```

### QoS Overrides

When recording topics with non-default QoS (e.g., best-effort sensor streams), specify overrides to ensure messages are received:

```bash
# Override to best-effort reliability for a high-frequency sensor topic
ros2 bag record /points --qos-profile-overrides-path qos_override.yaml
```

`qos_override.yaml`:
```yaml
/points:
  reliability: best_effort
  durability: volatile
```

---

## Playback

### Basic Playback

```bash
# Play a bag file
ros2 bag play my_experiment/

# Play at a different speed (0.5 = half speed, 2.0 = double speed)
ros2 bag play my_experiment/ --rate 0.5

# Loop continuously
ros2 bag play my_experiment/ --loop

# Start at a specific time offset (seconds from bag start)
ros2 bag play my_experiment/ --start-offset 30.0

# Pause at start, step through manually
ros2 bag play my_experiment/ --start-paused

# Play only specific topics
ros2 bag play my_experiment/ --topics /scan /odom
```

### Simulated Time

When playing back a bag for algorithm testing, nodes should use the **bag's recorded timestamps** rather than wall-clock time. Set the `/use_sim_time` parameter and enable bag clock publishing:

```bash
# Publish /clock topic from bag timestamps
ros2 bag play my_experiment/ --clock

# In a separate terminal or launch file, set all nodes to use sim time
ros2 param set /my_node use_sim_time true
```

In launch files:
```python
Node(
    package='my_robot',
    executable='slam_node',
    parameters=[{'use_sim_time': True}]
)
```

---

## Inspection

```bash
# Show bag metadata (topics, types, duration, message counts)
ros2 bag info my_experiment/

# Example output:
# Files:             my_experiment_0.mcap
# Bag size:          1.2 GiB
# Storage id:        mcap
# Duration:          120.5s
# Start:             May  2 2025 14:23:11.123 (1746185791.123)
# End:               May  2 2025 14:25:11.623 (1746185911.623)
# Messages:          245,872
#
# Topic information:
#   Topic: /scan | Type: sensor_msgs/msg/LaserScan | Count: 12,060 | Serialization Format: cdr
#   Topic: /odom | Type: nav_msgs/msg/Odometry    | Count: 24,100 | Serialization Format: cdr
#   Topic: /camera/image_raw | Type: sensor_msgs/msg/Image | Count: 6,024 | Serialization Format: cdr
```

---

## Filtering and Conversion

### Extract a Time Window

```bash
# Extract messages between t=30s and t=90s into a new bag
ros2 bag filter my_experiment/ -o filtered_bag \
  --start-time 1746185821 --end-time 1746185881
```

### Convert Storage Format

```bash
# Convert an older SQLite3 bag to MCAP (indexed, seekable, opens in Foxglove)
ros2 bag convert \
  --input my_experiment/ \
  --output-options "uri: my_experiment_mcap, storage_id: mcap"
```

**MCAP** is an open, indexed container format created by Foxglove. It seeks quickly in large files, compresses per chunk while staying indexed, and opens directly in Foxglove. It has been rosbag2's default since Iron (2023).

### Reindex a Corrupted Bag

If recording is interrupted (power loss, crash), the bag may need reindexing:

```bash
ros2 bag reindex my_experiment/
```

---

## Programmatic Access

### Reading a Bag in Python

```python
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan

storage_options = StorageOptions(uri='my_experiment/', storage_id='mcap')
converter_options = ConverterOptions('', '')

reader = SequentialReader()
reader.open(storage_options, converter_options)

# Get topic metadata
topic_types = reader.get_all_topics_and_types()
type_map = {t.name: t.type for t in topic_types}

# Read messages
while reader.has_next():
    topic, data, timestamp = reader.read_next()

    if topic == '/scan':
        msg = deserialize_message(data, LaserScan)
        print(f't={timestamp/1e9:.3f}s  ranges[0]={msg.ranges[0]:.3f}m')
```

### Writing a Bag in Python

```python
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from rclpy.serialization import serialize_message
from std_msgs.msg import String
import time

storage_options = StorageOptions(uri='output_bag/', storage_id='mcap')
converter_options = ConverterOptions('', '')

writer = SequentialWriter()
writer.open(storage_options, converter_options)

topic_info = TopicMetadata(
    name='/chatter',
    type='std_msgs/msg/String',
    serialization_format='cdr'
)
writer.create_topic(topic_info)

msg = String()
for i in range(100):
    msg.data = f'Hello, world! ({i})'
    writer.write('/chatter', serialize_message(msg), int(time.time() * 1e9))

del writer  # flushes and closes
```

---

## Visualization with Foxglove

**Foxglove** is a widely used tool for visualizing and analyzing rosbag2 data in a browser or desktop app. (Its original open-source *Studio* repository was archived in 2024 when development moved to a closed-source product; the MCAP format and `foxglove_bridge` remain open source.) It supports MCAP and ROS 2 bag files natively and provides panels for:

* 3D scene visualization (robot model, point clouds, markers).
* Time-series plots.
* Image panels (raw, compressed).
* Topic message inspection.
* User-defined layouts saved as `.foxglove-layout` files.

```bash
# Install Foxglove bridge for live ROS 2 visualization
sudo apt install ros-$ROS_DISTRO-foxglove-bridge
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
# Then open https://app.foxglove.dev and connect to ws://localhost:8765
```

---

## Common Workflows

### Development Workflow with Bags

```
1. Deploy robot → Record bag: ros2 bag record -a -o field_run_001
2. Bring bag back to desk
3. Replay bag: ros2 bag play field_run_001/ --clock
4. Run algorithm node against replayed data
5. Record algorithm output: ros2 bag record /slam/map /slam/pose
6. Analyze results in Foxglove or RViz
7. Iterate algorithm, repeat from step 3
```

### CI/CD Testing with Bags

```bash
# In a CI pipeline: replay a reference bag and check algorithm output
ros2 bag play reference_bag/ --clock &
sleep 2
ros2 run my_package evaluate_slam --bag reference_bag/ --expected-output expected.yaml
```

---

## New in Lyrical (2026)

- **Remote control by services.** A running recorder exposes `~/record`, `~/stop`, `~/start_discovery` and `~/stop_discovery`, so a fleet manager or a teleoperation UI can start and stop recording without a terminal:

```bash
ros2 service call /rosbag2_recorder/stop rosbag2_interfaces/srv/Stop "{}"
ros2 service call /rosbag2_recorder/record rosbag2_interfaces/srv/Record "{uri: 'file:///data/episode_0042'}"
```

- **Python control.** `rosbag2_py.Recorder` and `rosbag2_py.Player` can pause, resume, seek and step from Python, which makes scripted evaluation and dataset tooling simpler.
- **Circular recording** with a limit on the number of bag files keeps only the most recent data: useful as a "black box" that is saved only when something goes wrong.
- **Message-loss observability** reports dropped messages early instead of leaving silent gaps in a dataset.

## Bags and robot learning

Teleoperation demonstrations for [[Imitation_Learning|imitation learning]] are usually recorded as bags (camera images, joint states, commands) and then converted into a learning dataset format such as LeRobot's. Three habits make that conversion painless:
- record the robot description and `/tf_static`;
- keep image and joint topics on one clock;
- split recordings at episode boundaries (one bag per demonstration, or service calls per episode).

See [[Teleoperation_and_Data_Collection]] and [[Robot_Learning_Datasets_and_Benchmarks]].

## Dataview

```dataview
LIST FROM "" WHERE contains(file.outlinks, [[rosbag2]])
```
