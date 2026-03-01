---
title: rosbag2
description: Explains rosbag2, the ROS 2 tool for recording and playing back ROS topic data, covering recording, playback, filtering, conversion, and use cases in debugging, testing, and dataset collection.
tags:
  - rosbag2
  - ROS2
  - data-recording
  - debugging
  - testing
  - robotics-software
  - middleware
  - dataset
  - simulation
layout: default
category: robotics
author: Jordan_Smith_&_le_Chat
date: 2025-05-02
permalink: /rosbag2/
related:
  - "[[Robot_Operating_System_(ROS)]]"
  - "[[ROS_2_Overview]]"
  - "[[Topics]]"
  - "[[Nodes]]"
  - "[[RViz_Tutorial]]"
  - "[[Sensor_Fusion]]"
  - "[[SLAM_with_ROS]]"
  - "[[Machine_Learning]]"
  - "[[Custom_Packages_and_Nodes]]"
---

# rosbag2

**rosbag2** is the ROS 2 tool for recording and playing back data published on ROS [[Topics]]. It captures a timestamped log of messages across any number of topics into a **bag file** — a portable, queryable data archive that can be replayed to reproduce system behavior exactly as it occurred.

rosbag2 is a successor to `rosbag` from ROS 1, with significant improvements: a pluggable storage backend (SQLite3 by default, with MCAP support), improved performance for high-bandwidth data (cameras, LiDAR), and a flexible serialization layer.

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
* `my_experiment_0.db3` — SQLite3 database with message data.
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
# Files:             my_experiment_0.db3
# Bag size:          1.2 GiB
# Storage id:        sqlite3
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
# Convert from SQLite3 to MCAP format (better for large bags, seekable)
ros2 bag convert \
  --input my_experiment/ \
  --output-options "uri: my_experiment_mcap, storage_id: mcap"
```

**MCAP** (Machine-readable Capture) is a modern, indexed bag format developed by Foxglove with superior seek performance for large files and native support in the **Foxglove Studio** visualization tool.

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

storage_options = StorageOptions(uri='my_experiment/', storage_id='sqlite3')
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

storage_options = StorageOptions(uri='output_bag/', storage_id='sqlite3')
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

## Visualization with Foxglove Studio

**Foxglove Studio** is a popular open-source tool for visualizing and analyzing rosbag2 data with a web-based interface. It supports MCAP and ROS 2 bag files natively and provides panels for:

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

## Dataview Plugin Features

```dataview
LIST FROM #rosbag2 OR #debugging WHERE contains(file.outlinks, [[rosbag2]])
```

```dataview
TABLE title, description FROM #ROS WHERE contains(file.tags, "data-recording")
```
