# collision_check_server

A small ROS 2 package that exposes a single service, `/collision_check`, for
batch-checking a set of joint configurations for **joint-limit violations**
and **collisions** against a live MoveIt planning scene.

It is designed to be used alongside
[`ik_solver`](https://github.com/JRL-CARI-CNR-UNIBS/ik_solver): an IK solver
can generate many candidate joint solutions for a Cartesian target, and this
node quickly tells you which of those solutions are actually reachable
without hitting the robot itself or an obstacle in the scene. Packaging this
as a standalone service means client code does not need to link MoveIt or
implement its own collision-checking logic — it only needs the
`ik_solver_msgs` message definitions.

- **Package:** `collision_checker_server`
- **Node executable:** `ik_collision_checker` (installed name; source is `src/node.cpp`)
- **Node name:** `ik_collision_check`
- **Service:** `~/collision_check` → globally `/collision_check`
- **Service type:** `ik_solver_msgs/srv/CollisionChecking`
- **License:** BSD 3-Clause
- **Maintainer:** Federico Parma (federico.parma@unibs.it)

---

## Table of contents

- [How it works](#how-it-works)
- [Package layout](#package-layout)
- [Dependencies](#dependencies)
- [Building](#building)
- [Parameters](#parameters)
- [Service API](#service-api)
- [Tutorial](#tutorial)
  - [1. Prerequisites](#1-prerequisites)
  - [2. Write a parameter file](#2-write-a-parameter-file)
  - [3. Launch the node](#3-launch-the-node)
  - [4. Inspect the service from the CLI](#4-inspect-the-service-from-the-cli)
  - [5. Call the service from Python (rclpy)](#5-call-the-service-from-python-rclpy)
  - [6. Call the service from C++](#6-call-the-service-from-c)
  - [7. Reading the results](#7-reading-the-results)
- [Concurrency model](#concurrency-model)
- [Known limitations](#known-limitations)
- [Troubleshooting](#troubleshooting)

---

## How it works

The node is built around two cooperating pieces:

1. **`node.cpp`** (the executable entry point) owns a background loop that
   keeps a `MoveCollisionChecker` instance in sync with the world. At 10 Hz it
   calls the standard MoveIt `/get_planning_scene` service (normally provided
   by `move_group`) and pushes the result into the checker as a **full**
   scene update. This is what keeps the node aware of octomap changes,
   attached objects, added/removed collision objects, etc., even between
   client requests.

2. **`MoveCollisionChecker`** (`ik_collision_checker.hpp/.cpp`) owns a MoveIt
   `planning_scene::PlanningScene` built from the `robot_description`
   parameter, and serves the `/collision_check` service. For every incoming
   request it:
   - applies the request's own `planning_scene` field as a **diff** update
     (so a client can, e.g., add a payload/attached object just for this
     check without racing the background poller),
   - for each candidate configuration in `req.solutions`:
     1. sets the joint positions on a `RobotState`,
     2. checks `satisfiesBounds()` for the configured joint group — if any
        joint is out of its position bounds, the configuration is marked
        infeasible (`feasibility = 0.0`, `out_of_bound = true`) and the
        offending joint names are listed; collision checking is **skipped**
        for that configuration,
     3. otherwise runs `PlanningScene::checkCollision()` against the current
        `AllowedCollisionMatrix`, and sets `feasibility = 0.0` if a
        collision is found, `1.0` otherwise,
     4. if `req.detailed` is true, also records contact pairs and the
        MoveIt-reported `distance`.

To avoid the background poller mutating the scene mid-check, `node.cpp`
skips its periodic update whenever `MoveCollisionChecker::is_processing_request_`
is `true`, and `MoveCollisionChecker` itself guards the scene with a mutex
(`scene_mtx_`) around both the periodic update and the service callback.

Before every scene update (diff or full), the node reapplies **link padding
and scaling** to every link in the robot model, using the `padding`/`scaling`
parameters described below — this happens on every update, so it survives
full scene replacements coming from `/get_planning_scene`.

## Package layout

```
collision_check_server/
├── include/collision_checker_server/
│   └── ik_collision_checker.hpp   # MoveCollisionChecker class declaration
├── src/
│   ├── collision_checker_server/
│   │   └── ik_collision_checker.cpp  # MoveCollisionChecker implementation
│   └── node.cpp                   # main(): scene-polling loop + service init
├── CMakeLists.txt
├── package.xml
└── LICENSE
```

`CMakeLists.txt` builds two targets:
- `collision_checker_server` — a shared library containing `MoveCollisionChecker`.
- `collision_checker_server_node` — the executable, installed as
  `ik_collision_checker` under `lib/collision_checker_server/`, linked
  against the library above.

## Dependencies

ROS 2 / MoveIt:
- `rclcpp`
- `moveit_core`
- `moveit_ros_planning`
- `moveit_msgs`
- `ik_solver_msgs` (provides the `CollisionChecking` service and the
  `Configuration` / `CollisionResult` messages)

CNR CARI-UNIBS libraries:
- `cnr_param` — used to read the `padding` and `scaling` parameters (see
  below); falls back gracefully to compiled-in defaults if not found.
- `cnr_yaml`, `cnr_logger` — transitive dependencies of `cnr_param`.

At runtime the node also expects, on the ROS graph:
- a `robot_description` parameter (standard MoveIt convention — typically
  published by whatever brings up `move_group` / `robot_state_publisher`),
- a working `/get_planning_scene` service, normally provided by `move_group`.

## Building

Inside your ROS 2 workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select collision_checker_server
source install/setup.bash
```

Make sure `ik_solver_msgs`, `cnr_param`, `cnr_yaml`, and `cnr_logger` are
already built (or included in the same workspace) before building this
package.

## Parameters

| Parameter     | Type   | Required | Default | Source                                   | Description |
|---------------|--------|----------|---------|-------------------------------------------|--------------|
| `group_name`  | string | **yes**  | —       | standard ROS 2 node parameter             | Name of the MoveIt joint model group whose joints are set from `req.solutions` and whose bounds are checked. Constructor **throws `std::invalid_argument`** if missing or empty. |
| `padding`     | double | no       | `0.01`  | `cnr_param` (see below)                   | Padding (meters) applied to every robot link before collision checking. |
| `scaling`     | double | no       | `1.0`   | `cnr_param` (see below)                   | Scale factor applied to every robot link's collision geometry. |

Notes:

- `group_name` must be declared/set the normal ROS 2 way, e.g. via a
  `ros__parameters` YAML block or `-p group_name:=manipulator` on the
  command line, on the node's own namespace.
- `padding` and `scaling` are read through `cnr_param`, which resolves keys
  under **both** ROS1-style (`/ns/node/key`) and ROS2-style
  (`/ns.node.key`) naming and can fall back to a "mapped file" YAML source
  in addition to the live ROS 2 parameter server. In this node they are
  looked up at `<node_namespace>/padding` and `<node_namespace>/scaling`. If
  `cnr_param` can't find them, the C++ default member initializers
  (`0.01` / `1.0`) are silently kept — there's no hard failure for these two.
- All three parameters are logged at startup (`INFO` level), including the
  active/passive/mimic joints of `group_name` and their variable indices —
  useful for sanity-checking that `group_name` resolves to what you expect.

## Service API

`ik_solver_msgs/srv/CollisionChecking`:

```
# Request
ik_solver_msgs/Configuration[] solutions   # candidate joint configurations to test
string[] joint_names                       # names for the values in each solution's `configuration`
bool detailed                              # if true, also compute distances and colliding link names
moveit_msgs/PlanningScene planning_scene   # applied as a scene *diff* before checking
---
# Response
ik_solver_msgs/CollisionResult[] result
```

`ik_solver_msgs/msg/Configuration`:

```
float64[] configuration   # one joint configuration, ordered like `joint_names`
```

`ik_solver_msgs/msg/CollisionResult` (one per input solution, same order):

```
ik_solver_msgs/Configuration solution   # echoes the input configuration
string[] colliding_obj_first            # first body of each contact pair (only if `detailed`)
string[] colliding_obj_second           # second body of each contact pair (only if `detailed`)
bool out_of_bound                       # true if any joint violated its position bounds
string[] out_of_bound_links             # names of the joints that were out of bounds
float64 feasibility                     # 0.0 = collision or out of bounds, 1.0 = feasible
float64 distance                        # MoveIt collision distance (only meaningful if `detailed`)
time stamp                              # not populated by this node — left at default
```

Important request constraint: **`joint_names.size()` must equal the number
of active joints in `group_name`.** If it doesn't, the callback logs a
warning listing both sets of names, returns **without populating `result`**
(see [Known limitations](#known-limitations)), and the client will receive
an empty `result` array.

## Tutorial

This walks through starting the node against any MoveIt-enabled robot in
your workspace and calling the service both from the CLI and from client
code.

### 1. Prerequisites

You need a running MoveIt stack for your robot: something publishing
`robot_description` and a `move_group` node (or anything else that serves
`/get_planning_scene`). Bring that up however your robot package normally
does it, e.g.:

```bash
ros2 launch <your_robot_moveit_config> demo.launch.py
```

Verify the service this node depends on exists before continuing:

```bash
ros2 service list | grep get_planning_scene
# -> /get_planning_scene
```

### 2. Write a parameter file

Create `collision_check_params.yaml`:

```yaml
/ik_collision_check:
  ros__parameters:
    group_name: "manipulator"   # <-- replace with your MoveIt group name
    padding: 0.02
    scaling: 1.0
```

Replace `manipulator` with an actual planning group defined in your robot's
SRDF (check `ros2 param get` on `move_group` or your SRDF file if unsure).

### 3. Launch the node

```bash
ros2 run collision_checker_server ik_collision_checker \
  --ros-args \
  -r __node:=ik_collision_check \
  --params-file collision_check_params.yaml
```

You should see startup logs listing every joint in `group_name` with its
index, followed by:

```
[INFO] [ik_collision_check]: Collision checking padding: 0.020, scaling: 1.000
[INFO] [ik_collision_check]: Starting periodic planning scene updates.
```

Confirm the service is up:

```bash
ros2 service list | grep collision_check
ros2 service type /collision_check
# -> ik_solver_msgs/srv/CollisionChecking
```

### 4. Inspect the service from the CLI

`ros2 service call` works for a quick smoke test, but `PlanningScene` is a
large nested message, so for anything beyond a trivial call prefer the
Python/C++ clients below. A minimal call that checks a single configuration
against whatever scene the node already has cached (empty `planning_scene`
diff, so nothing changes) looks like:

```bash
ros2 service call /collision_check ik_solver_msgs/srv/CollisionChecking \
"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6'],
  solutions: [{configuration: [0.0, -0.5, 0.5, 0.0, 0.5, 0.0]}],
  detailed: true,
  planning_scene: {}}"
```

Replace the joint names/values with the ones for your `group_name`. The
response prints one `CollisionResult` per requested configuration.

### 5. Call the service from Python (rclpy)

This example fetches the current planning scene, asks for one candidate
configuration to be checked, and prints whether it's feasible.

```python
import rclpy
from rclpy.node import Node
from ik_solver_msgs.srv import CollisionChecking
from ik_solver_msgs.msg import Configuration
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents


class CollisionCheckClient(Node):
    def __init__(self):
        super().__init__('collision_check_client')
        self.scene_cli = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.cc_cli = self.create_client(CollisionChecking, '/collision_check')
        self.scene_cli.wait_for_service()
        self.cc_cli.wait_for_service()

    def check(self, joint_names, configuration, detailed=True):
        scene_req = GetPlanningScene.Request()
        scene_req.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY \
            | PlanningSceneComponents.WORLD_OBJECT_NAMES \
            | PlanningSceneComponents.ROBOT_STATE
        scene_future = self.scene_cli.call_async(scene_req)
        rclpy.spin_until_future_complete(self, scene_future)

        req = CollisionChecking.Request()
        req.joint_names = joint_names
        req.solutions = [Configuration(configuration=configuration)]
        req.detailed = detailed
        req.planning_scene = scene_future.result().scene

        future = self.cc_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().result


def main():
    rclpy.init()
    client = CollisionCheckClient()

    results = client.check(
        joint_names=['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'],
        configuration=[0.0, -0.5, 0.5, 0.0, 0.5, 0.0],
    )

    for i, r in enumerate(results):
        if r.out_of_bound:
            print(f'[{i}] OUT OF BOUNDS: {list(r.out_of_bound_links)}')
        elif r.feasibility > 0.5:
            print(f'[{i}] FEASIBLE (distance={r.distance:.4f})')
        else:
            pairs = list(zip(r.colliding_obj_first, r.colliding_obj_second))
            print(f'[{i}] COLLISION between: {pairs}')

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 6. Call the service from C++

```cpp
#include <rclcpp/rclcpp.hpp>
#include <ik_solver_msgs/srv/collision_checking.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("collision_check_client");

  auto scene_client = node->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
  auto cc_client = node->create_client<ik_solver_msgs::srv::CollisionChecking>("/collision_check");
  scene_client->wait_for_service();
  cc_client->wait_for_service();

  auto scene_req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  auto scene_future = scene_client->async_send_request(scene_req);
  rclcpp::spin_until_future_complete(node, scene_future);

  auto req = std::make_shared<ik_solver_msgs::srv::CollisionChecking::Request>();
  req->joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
  ik_solver_msgs::msg::Configuration cfg;
  cfg.configuration = {0.0, -0.5, 0.5, 0.0, 0.5, 0.0};
  req->solutions = {cfg};
  req->detailed = true;
  req->planning_scene = scene_future.get()->scene;

  auto future = cc_client->async_send_request(req);
  rclcpp::spin_until_future_complete(node, future);

  for (const auto &r : future.get()->result)
  {
    RCLCPP_INFO(node->get_logger(), "feasibility=%.1f out_of_bound=%d distance=%.4f",
                r.feasibility, r.out_of_bound, r.distance);
  }

  rclcpp::shutdown();
  return 0;
}
```

### 7. Reading the results

For each `CollisionResult` in `result` (same order as the input `solutions`):

- `feasibility == 1.0` → configuration is within joint limits **and**
  collision-free.
- `feasibility == 0.0` and `out_of_bound == true` → one or more joints
  listed in `out_of_bound_links` violated their position bounds; collision
  was **not** checked for this configuration.
- `feasibility == 0.0` and `out_of_bound == false` → the configuration is
  in collision. If `detailed` was `true`, `colliding_obj_first[i]` /
  `colliding_obj_second[i]` give the names of each colliding pair, and
  `distance` is MoveIt's reported penetration/clearance distance.

## Concurrency model

- `node.cpp` runs an `rclcpp::MultiThreadedExecutor` on its own thread so
  the `/collision_check` service callback and the periodic
  `/get_planning_scene` polling can be in flight without one blocking the
  other's callback delivery.
- `MoveCollisionChecker::is_processing_request_` is an `std::atomic<bool>`
  that the main loop checks before issuing a new scene poll — while a
  request is being served, the periodic update is skipped for that cycle
  rather than queued.
- `MoveCollisionChecker::scene_mtx_` serializes actual reads/writes of the
  `PlanningScene` object between `updatePlanningScene()` and
  `collisionCheck()`.

## Known limitations

- If `req.joint_names` doesn't have exactly the same number of entries as
  the active joints of `group_name`, the callback logs a warning and
  returns with `res->result` left **empty** — the code comment explicitly
  notes `"RESORTING IS NOT IMPLEMENTED YET"`. The request currently assumes
  `joint_names` is not just the same *set* but also usable positionally
  against `group_name`'s active joints.
- `CollisionResult::stamp` is never set by this node (left at its default
  value).
- The node has no dedicated launch file in this package; it must be
  launched with an explicit `--params-file` (or equivalent) as shown in the
  tutorial, alongside a separately-launched MoveIt stack.

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Node throws / exits immediately with `.../group_name is not specified` | `group_name` parameter missing from the params file or CLI args, or set on the wrong node name/namespace. |
| Startup hangs before "Starting periodic planning scene updates." | `/get_planning_scene` isn't available yet — make sure `move_group` (or another provider) is up first. |
| `res->result` comes back empty | `joint_names` in the request doesn't match the count of active joints in `group_name` — check the `WARN` logs, which print both lists. |
| All configurations report `out_of_bound` | Check the joint value units/order against what `state.getJointModelGroup(group_name_)->getActiveJointModelNames()` logs at startup — values are set positionally by matching `joint_names[i]` to `req->solutions[*].configuration[i]`. |
| Collisions reported that seem too conservative/permissive | Check `padding`/`scaling` — both are applied to *every* link on every scene update. |
