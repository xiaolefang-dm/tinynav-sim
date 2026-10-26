# Go2 simulation with the TinyNav application

Use the existing Go2 model with a fixed standing pose. Set
`TINYNAV_GO2_MOTION=kinematic` when running `gazebo/robots/go2/spawn.sh`.
In this mode the model is static and `/cmd_vel` updates its planar pose through
Gazebo's `set_pose` service. No gait, joint controllers, or velocity servo run.
Forward/backward, lateral motion, and yaw use body coordinates, with a 300 ms
command timeout. Movement follows simulation time and stops while paused.
This mode tests navigation and takeover, not leg dynamics or physical contact;
direct pose updates can pass through obstacles.

The Go2 sensor head also publishes aligned depth on `/sim/depth` for the
Looper input adapter. `looper_source.py` converts images, depth, and Gazebo
ground-truth poses to Looper-compatible topics. It uses the robot's TF camera
extrinsics and Gazebo CameraInfo, rather than model-specific camera offsets.
Color, infrared, depth, and visual poses share one output timestamp after
association on the simulation clock. The adapter also publishes color camera
intrinsics and the Looper camera transform used by offline mapping.
This adapter supplies ground truth; it does not estimate VIO or inject faults.

## Start

Run inside the development/simulation containers with the ROS environment
loaded. The Gazebo container needs the existing Go2 dependencies: xacro,
robot_state_publisher, ros_gz_sim, ign_ros2_control, controller_manager, and
ros2_controllers for gait mode. Kinematic mode only needs the model,
robot_state_publisher, ros_gz_sim, and ros_gz_bridge. All ROS processes must share the same domain and DDS setup.
For host-network containers on one machine, the existing
`tools/probes/cyclone_localhost_unicast.xml` profile avoids cross-container
shared-memory transport issues.

Launch the existing Go2 world/spawn chain with the simulator's trajectory
follower and keyboard teleop disabled; TinyNav owns these commands. The
`gazebo/robots/go2/spawn.sh` starts the pose controller and service bridge
when `TINYNAV_GO2_MOTION=kinematic`; its default gait mode remains available.
Bridge the following Gazebo inputs into the application's ROS domain:

| Gazebo topic | ROS topic | ROS type |
|---|---|---|
| `/camera/camera/infra1/image_rect_raw` | `/sim/infra1` | sensor_msgs/msg/Image |
| `/camera/camera/infra1/camera_info` | `/sim/camera_info` | sensor_msgs/msg/CameraInfo |
| `/sim/depth` | `/sim/depth` | sensor_msgs/msg/Image |
| `/camera/camera/color/image_raw` | `/sim/color` | sensor_msgs/msg/Image |
| `/clock` | `/clock` | rosgraph_msgs/msg/Clock |
| `/world/<world>/pose/info` | `/sim/world_pose` | tf2_msgs/msg/TFMessage |

Run `python3 looper_source.py`. Defaults are model `go2`, body frame
`base_link`, camera frame `camera_infra1_link`, and output prefix
`/camera/camera`. Model and camera frame can be selected with `--model` and
`--camera-frame`. The robot_state_publisher supplies the camera TF.

Start the application backend with the stock Go2 configuration:

```bash
export ROBOT_TYPE=go2
export TINYNAV_DB_PATH=/tinynav/tinynav_db/sim-home-go2
python3 run_tinynav.py
```

The launcher only selects Looper input and skips the physical chassis driver;
it does not replace robot geometry or node launch commands in TinyNav.
Build the web frontend using the normal TinyNav workflow. Use a map recorded
with the Go2 sensor head; maps made with a different camera height should be
kept separately.

Take control keeps perception, planning, and POI tracking running. Manual
arrival can advance or complete a task. Exit manual hands an unfinished task
back to automatic control without Continue.

## Front and rear units

The Go2 xacro includes a second, identical 544x480 / 15 Hz stereo + RGB +
aligned-depth unit at the rear, with a 51 mm stereo baseline and 180-degree
mounting yaw. Both units include IMU sensors. Rear infra1 is at body
`(-0.19, -0.0255, 0.36)` metres; front infra1 is at `(0.19, 0.0255, 0.36)`.

Load the same ROS domain/DDS settings and `IGN_PARTITION` as Gazebo, then run:

```bash
bash tools/manual_intervention/bridge_looper.sh
python3 tools/manual_intervention/looper_source.py --rear
```

The bridge uses `TINYNAV_WORLD_NAME` (default `yard`). Native rear images are
remapped to `/sim/rear/{infra1,infra2,color,camera_info,depth}`; normalized Looper
outputs use `/camera/rear/`. A shared bounded timestamp cache makes frames from
the same simulation instant carry exactly the same output timestamp across
both units. Pose/image association still uses simulation time. Unique rear TF
aliases prevent the rear RGB transform overwriting the front mapping transform.
Without `--rear`, the adapter publishes the original front unit only.

In Pilot, enable the rear unit in Device > Cameras and save its installation.
Pilot records the rig manifest with the bag, stores both directions at each
keypose and fuses rear depth for occupancy/planning. The adapter still supplies
Gazebo ground truth, not independently estimated front/rear VIO.

## Stair height following in kinematic mode

For `--world stairs`, the pose controller automatically loads the named walkable
collisions from `gazebo/worlds/stairs.sdf`. It follows both flights and landings
in Z while retaining the fixed standing pose and yaw-only orientation. The
geometry source is the SDF's `stairs_switchback` model: `col_f1_s*`, `col_f2_s*`
and `col_plat*` box collisions; rails and visual meshes are not walkable surfaces.
`--terrain-sdf PATH` selects another SDF using the same explicit naming contract.

Height changes slew at at most 0.5 m/s of simulation time and finish settling
after movement stops. Horizontal transitions with more than 0.20 m of ground
height change are rejected: the robot cannot jump onto a high platform from
the side or walk directly off its edge. Yaw remains usable at a blocked edge.
Flat worlds retain the model's initial Z instead of resetting it to zero.
This remains a kinematic mapping/navigation aid: no gait, foot contact, body
pitch, general mesh collision or physical climbing capability is simulated.

Validation inside the simulation container:

```bash
python3 -m unittest discover -s tools/manual_intervention -p test_terrain.py -v
```

The tests load the real staircase geometry, cover both ascent/descent flights
and landings, limit vertical slew, reject side jumps and large drops, and keep
flat-world spawn height. Live Gazebo verification checks body Z and both camera
poses at the 1.2 m / 2.4 m landings and after descending to ground.

## Manual two-floor elevator

`gazebo/worlds/elevator.sdf` is a self-contained two-floor test world, generated by
`generate_elevator.py`. Floor 0 is blue, floor 1 is orange, and the vertical travel
is 3 m. The dog starts at `(0, 0, 0)` facing +X toward the entrance at X=3 m. The
cabin spans X=3..6 m and Y=-1..1 m. Asymmetric landmarks and existing repository
textures provide camera features. No external factory mesh is required.

Inside the sim container, load the same ROS domain, DDS and `IGN_PARTITION` as
Pilot, then run:

```bash
bash tools/manual_intervention/run_elevator.sh
```

The launcher starts Gazebo, the existing kinematic Go2 spawn, its pose controller,
and the elevator driver. Run `bridge_looper.sh` and `looper_source.py --rear` in
the Pilot adapter container with `TINYNAV_WORLD_NAME=elevator`, as with other
worlds. Use a separate `TINYNAV_DB_PATH` for elevator recordings/maps.

Open `http://<sim-host>:8092/` for manual door/floor controls, and the normal Pilot
App for WASD/left-right driving. Manual test sequence:

1. Open the doors at floor 0; drive forward into the cabin centre (about X=4.5 m).
2. Close the doors; select floor 1. Release all motion keys during travel.
3. Once floor 1 is confirmed, open the doors; drive backwards out to X<2.5 m.
4. Repeat in the other direction. The floor-1 landing supports the dog even if
   the empty cabin is sent away.

The driver exposes ROS `/elevator/command` (JSON String) and `/elevator/state`
(JSON String, 20 Hz). The same command schema is available through HTTP POST
`/command`: `{"action":"open"}`, `{"action":"close"}`, or
`{"action":"floor","floor":1}`. GET `/status` includes height, floor, motion,
door fraction, robot position, confirmed state and errors. Command acceptance is
not motion completion; wait for confirmed terminal state. The driver publishes
new cabin/door poses only after Gazebo's set-pose service acknowledges the batch.

Kinematic motion adds elevator-specific constraints using a 0.32 m circular
footprint. Closed doors, cabin walls, landing edges and a missing cabin block
entry. An occupied doorway refuses closing and reverses a closing door; open
doors refuse floor travel. Inside the cabin, the dog follows cabin height even
with no velocity command. XY motion is held while the cabin travels. This does
not simulate contact forces, emergency braking or elevator safety certification.
The current path is manual only; no Pilot elevator mode or automatic relocalizing
is implemented in this stage.

Verification (inside the simulation container):

```bash
cd tools/manual_intervention
python3 -m unittest test_elevator test_terrain
```

Home deployment uses the existing `tinynav-sim-home` and `tinynav-pilot-home`
containers, with elevator assets in this branch and `home-elevator` as a separate
Pilot data root. The previous stairs runtime environment is backed up at
`/runtime/pilot-env.before-elevator.sh` outside the repo.

Live round-trip probe (start at floor 0 with the door closed and the dog in the
landing; run in the Pilot container with ROS/Python dependencies loaded):

```bash
python3 /sim-tools/tools/probes/probe_elevator_roundtrip.py
```

This drives the dog through the normal Pilot teleop WebSocket, verifies occupied
and closed-door interlocks, carries it up/down, checks that an empty elevator does
not move a dog on the landing, and returns to floor 0. Stop other manual inputs
while this probe runs. Home verification completed this full round-trip.
