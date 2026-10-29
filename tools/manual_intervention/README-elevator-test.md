# Elevator Test button

The elevator page exposes **Test**, **Cancel Test**, and the current step/error.
Pilot's optional `python3 -m pilot.tools.elevator_test` service runs the behaviour
tree on the robot side (loopback :8110). Start it in Pilot's normal environment;
Pilot's HTTP face (:8000) and operator status (:8100) must also be running.
The simulation remains the authority for confirmed door and floor states.

The fixed rehearsal uses map `in`, front localization and POI 1 to enter, then
map `out`, rear localization and POI 2 to leave at the other floor. The maps and
POIs must already exist. Navigation starts only after localization; each door
operation and floor change waits for confirmed completion. Test stops Nav at
the end. Cancellation/failure stops the mission and Nav without moving doors.
Manual elevator commands are refused while the test is active.

Home keeps the elevator page at :8092 and the POI editor at :8094.
