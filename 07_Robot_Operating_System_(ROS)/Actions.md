---
title: Actions
description: Explains the ROS Actions communication paradigm, which provides an asynchronous, goal-oriented interface for long-running tasks with continuous feedback and the ability to cancel in-flight goals.
tags:
  - ROS
  - ROS2
  - actions
  - middleware
  - robotics-software
  - communication
  - asynchronous
  - software-stack
layout: default
category: robotics
author: Jordan_Smith_&_le_Chat
date: 2025-05-02
permalink: /actions/
related:
  - "[[Robot_Operating_System_(ROS)]]"
  - "[[ROS_2_Overview]]"
  - "[[Topics]]"
  - "[[Services]]"
  - "[[Nodes]]"
  - "[[Custom_Packages_and_Nodes]]"
  - "[[Nav2_Navigation]]"
  - "[[MoveIt2]]"
  - "[[TF_and_Topic_Architecture]]"
---

# Actions

**Actions** are a ROS communication paradigm designed for **long-running, goal-oriented tasks** that require feedback during execution and the ability to be cancelled before completion. Actions fill a critical gap between the two other primary communication mechanisms:

* **[[Topics]]** (publish/subscribe): Continuous, asynchronous, fire-and-forget data streams. No confirmation of receipt or completion.
* **[[Services]]** (request/reply): Synchronous, blocking calls. The caller waits for the full response before proceeding — unsuitable for operations that take seconds or minutes.

Actions combine the strengths of both: a client sends a **goal** asynchronously, receives continuous **feedback** while the task executes, and eventually receives a **result** when it completes (or is cancelled/aborted).

---

## Action Interface Components

An action is defined by a `.action` file with three message sections separated by `---`:

```
# Goal: what the client wants done
geometry_msgs/PoseStamped target_pose
float32 max_velocity
---
# Result: returned when the action finishes (success or failure)
bool success
float32 final_distance_to_goal
string message
---
# Feedback: sent periodically while the action is executing
float32 current_distance_to_goal
float32 estimated_time_remaining
geometry_msgs/Pose current_pose
```

This three-part structure (`NavigateToPose.action`) is processed by the ROS build system to generate all necessary C++ and Python client/server types.

---

## Action Lifecycle

The action lifecycle involves a coordinated sequence of messages between client and server:

```
Client                          Action Server
  |                                  |
  |--- SendGoal(goal) -------------> |
  |                                  |  (server accepts or rejects)
  | <-- GoalResponse(accepted) ----- |
  |                                  |  (server executes task)
  | <-- Feedback(progress) --------- |  (periodic)
  | <-- Feedback(progress) --------- |  (periodic)
  |                                  |
  |  [optional: cancel goal]         |
  |--- CancelGoal() --------------> |
  | <-- CancelResponse() ---------- |
  |                                  |
  | <-- Result(success/failure) ---- |  (on completion)
  |                                  |
```

### States

The goal transitions through well-defined states on the server:

```
ACCEPTED → EXECUTING → SUCCEEDED
                    ↘ ABORTED
                    ↘ CANCELED (if cancel requested)
```

---

## Writing an Action Server (rclpy)

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from my_robot_interfaces.action import NavigateToPose
import time

class NavigationActionServer(Node):
    def __init__(self):
        super().__init__('navigation_action_server')
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.execute_callback
        )
        self.get_logger().info('Navigation action server ready.')

    def execute_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info(f'Executing goal: navigate to {goal_handle.request.target_pose}')

        feedback_msg = NavigateToPose.Feedback()
        result = NavigateToPose.Result()

        # Simulate navigation with periodic feedback
        for step in range(10):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = 'Goal cancelled by client'
                return result

            # Compute and publish feedback
            feedback_msg.current_distance_to_goal = 10.0 - step * 1.0
            feedback_msg.estimated_time_remaining = (10 - step) * 0.5
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5)

        goal_handle.succeed()
        result.success = True
        result.final_distance_to_goal = 0.0
        result.message = 'Goal reached successfully'
        return result

def main():
    rclpy.init()
    server = NavigationActionServer()
    rclpy.spin(server)

if __name__ == '__main__':
    main()
```

---

## Writing an Action Client (rclpy)

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from my_robot_interfaces.action import NavigateToPose

class NavigationActionClient(Node):
    def __init__(self):
        super().__init__('navigation_action_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, target_pose: PoseStamped):
        # Wait for action server to become available
        self._action_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.target_pose = target_pose
        goal_msg.max_velocity = 0.5

        self.get_logger().info('Sending goal...')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Goal rejected by server.')
            return
        self.get_logger().info('Goal accepted, waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'Distance remaining: {fb.current_distance_to_goal:.2f} m, '
            f'ETA: {fb.estimated_time_remaining:.1f} s'
        )

    def result_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.get_logger().info(
            f'Result: success={result.success}, message="{result.message}"'
        )

def main():
    rclpy.init()
    client = NavigationActionClient()
    # Build a target pose and send goal
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = 3.0
    pose.pose.position.y = 1.5
    client.send_goal(pose)
    rclpy.spin(client)

if __name__ == '__main__':
    main()
```

---

## Actions vs. Services vs. Topics

| Feature | Topics | Services | Actions |
|---|---|---|---|
| Communication style | Async pub/sub | Sync request/reply | Async goal/feedback/result |
| Blocking | No | Yes (client blocks) | No |
| Feedback during execution | No | No | Yes |
| Cancellable | No | No | Yes |
| Use case | Sensor streams, state | One-shot queries, config | Long-running tasks |
| Examples | `/scan`, `/cmd_vel` | `/set_parameter` | Navigate, pick-and-place |

---

## Command-Line Interaction

```bash
# List available action servers
ros2 action list

# Show info about an action
ros2 action info /navigate_to_pose

# Show action type definition
ros2 interface show nav2_msgs/action/NavigateToPose

# Send a goal from the command line
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 2.0}}}" \
  --feedback
```

---

## Real-World Usage

Actions are the standard interface for many critical ROS subsystems:

* **[[Nav2_Navigation|Nav2]]**: `NavigateToPose`, `NavigateThroughPoses`, `ComputePathToPose` — all action interfaces.
* **[[MoveIt2]]**: `MoveGroup`, `ExecuteTrajectory`, `GripperCommand` — motion planning and execution via actions.
* **Manipulation**: Pick-and-place, grasp execution, tool change sequences.
* **Mission execution**: High-level task sequencers dispatching navigation and manipulation goals to subordinate action servers.

---

## Defining a Custom Action Type

Create `my_robot_interfaces/action/Inspect.action`:

```
# Goal
string target_object_name
bool capture_image
---
# Result
bool object_found
float32[] joint_config_at_object
sensor_msgs/Image inspection_image
---
# Feedback
string current_phase   # e.g. "moving", "searching", "inspecting"
float32 search_coverage_percent
```

Register in `CMakeLists.txt`:

```cmake
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Inspect.action"
  DEPENDENCIES geometry_msgs sensor_msgs
)
```

And in `package.xml`:

```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

---

## Dataview Plugin Features

```dataview
LIST FROM #ROS OR #robotics-software WHERE contains(file.outlinks, [[Actions]])
```

```dataview
TABLE title, description FROM #ROS WHERE contains(file.tags, "actions")
```
