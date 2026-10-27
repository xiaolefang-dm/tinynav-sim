# Automatic exit validation — 2026-10-27

Branch: `xiaole/elevator-auto-exit`, based on the saved automatic-entry baseline.

## Live results

- Upper-floor automatic exit: passed; all nonzero translation commands negative.
- Entry upstairs, cabin travel to floor 0, automatic exit: passed; body remained at
  ground height, stopped outside, door confirmed closed.
- Cabin start with about 19 degrees of yaw: heading alignment reduced error below
  3 degrees before reverse planning. Operator Stop left the robot stationary and
  the door open; no automatic retry occurred. Explicit re-arm then completed exit.
- Final standalone probe: ground-floor entry, cabin travel to floor 1, reverse
  exit and door close passed. Final body approximately (1.868, -0.001, 3.0) m;
  nonzero linear commands between -0.15 and -0.045 m/s.

Pilot's focused suite: 32 passed. Full suite: 609 passed, 3 skipped, 2 failures.
The same two failures reproduced on the unmodified Pilot baseline: MQTT pose
serialization expectation and charge-interface wait (the home runtime sets the
interface wait to zero). Flutter analysis of the changed control file and release
web build passed.

## Reproduction

With an idle robot in the elevator scene, front/rear cameras enabled, and Pilot
configured for the simulator, run inside the Pilot container:

```bash
source /runtime/pilot-env.sh
python3 /sim-tools/tools/probes/probe_elevator_exit.py
```

The probe uses the HTTP control API, WebSocket teleop only to position the starting
robot, and ROS truth/velocity observations to assert motion. It performs automatic
entry, travels to the other floor, selects exit, opens the door, and asserts that
all exit translation is backwards and that the robot stops outside with the door
closed. It moves the robot and doors; do not run during an operator's test.

The adjacent JSON-line logs preserve observed stages. This remains fixed-geometry
simulation validation, not real elevator detection or destination-map handoff.
