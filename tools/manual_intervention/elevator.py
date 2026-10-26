"""Two-floor kinematic elevator geometry and interlocks, in metres and seconds."""

from dataclasses import dataclass
import math

DOOR_X = 3.0
FLOOR_HEIGHT = 3.0
RADIUS = 0.32


def in_cabin(pose):
    x, y = pose[:2]
    return DOOR_X + RADIUS <= x <= 6 - RADIUS and abs(y) <= 1 - RADIUS


def in_doorway(pose, height):
    x, y, z = pose[:3]
    return abs(x - DOOR_X) < RADIUS + 0.10 and abs(y) < 1.2 and abs(z - height) < 0.4


@dataclass
class Elevator:
    height: float = 0.0
    door: float = 0.0
    target: int = 0
    motion: str = "idle"

    def floor(self):
        return next(
            (f for f in (0, 1) if abs(self.height - f * FLOOR_HEIGHT) < 0.005), None
        )

    def command(self, action, floor=None, robot=None):
        if action == "open":
            if self.motion == "moving" or self.floor() is None:
                raise ValueError("Cannot open doors between floors")
            self.motion = "opening"
        elif action == "close":
            if robot is not None and in_doorway(robot, self.height):
                raise ValueError("Doorway occupied: move fully inside or outside first")
            if self.motion == "moving":
                raise ValueError("Elevator is moving")
            self.motion = "closing"
        elif action == "floor":
            if type(floor) is not int or floor not in (0, 1):
                raise ValueError("Floor must be 0 or 1")
            if self.door > 0.001 or self.motion not in ("idle", "moving"):
                raise ValueError("Close doors before selecting a floor")
            if robot is not None and in_doorway(robot, self.height):
                raise ValueError("Doorway occupied")
            self.target = floor
            self.motion = (
                "moving" if abs(self.height - floor * FLOOR_HEIGHT) > 0.005 else "idle"
            )
        else:
            raise ValueError("Unknown elevator command")

    def step(self, dt, robot=None):
        if self.motion == "moving":
            diff = self.target * FLOOR_HEIGHT - self.height
            self.height += max(-0.5 * dt, min(0.5 * dt, diff))
            if abs(diff) <= 0.5 * dt:
                self.height = self.target * FLOOR_HEIGHT
                self.motion = "idle"
        elif self.motion in ("opening", "closing"):
            if (
                self.motion == "closing"
                and robot is not None
                and in_doorway(robot, self.height)
            ):
                self.motion = "opening"
            sign = 1 if self.motion == "opening" else -1
            self.door = max(0.0, min(1.0, self.door + sign * 0.5 * dt))
            if self.door in (0.0, 1.0):
                self.motion = "idle"

    def state(self):
        return dict(
            height=round(self.height, 4),
            floor=self.floor(),
            target=self.target,
            motion=self.motion,
            door_fraction=round(self.door, 4),
            door_open=self.door >= 0.99 and self.motion != "closing",
        )

    def entities(self):
        poses = {"elevator_cabin": (4.5, 0.0, self.height)}
        for side, direction in (("left", 1), ("right", -1)):
            y = direction * (0.45 + self.door * 0.95)
            poses[f"cabin_{side}"] = (3.08, y, self.height)
            for floor in (0, 1):
                opened = self.door if self.floor() == floor else 0.0
                poses[f"landing_{floor}_{side}"] = (
                    3.0,
                    direction * (0.45 + opened * 0.95),
                    floor * FLOOR_HEIGHT,
                )
        return poses


def constrain(previous, proposed, state):
    """Conservative circular footprint; cabin support is independent of /cmd_vel."""
    if not state or not state.get("confirmed", False):
        return previous
    height = float(state["height"])
    x, y, z, yaw = proposed
    carried = in_cabin(previous)
    if state["motion"] == "moving":
        return (*previous[:2], height, previous[3]) if carried else previous
    support = height if carried else round(previous[2] / FLOOR_HEIGHT) * FLOOR_HEIGHT
    allowed = -5 + RADIUS <= x <= 6 - RADIUS and abs(y) <= 3 - RADIUS
    if x > DOOR_X - RADIUS:
        aligned = abs(height - support) < 0.02
        door_zone = x < DOOR_X + RADIUS
        if door_zone:
            allowed &= aligned and state["door_open"] and abs(y) <= 0.9 - RADIUS
        else:
            allowed &= abs(y) <= 1 - RADIUS and aligned
        # A segment crossing a closed doorway must never skip over the gate.
        if (previous[0] - DOOR_X) * (x - DOOR_X) < 0:
            allowed &= aligned and state["door_open"] and abs(y) <= 0.9 - RADIUS
    if not allowed:
        return (*previous[:2], support, previous[3])
    return x, y, support, math.remainder(yaw, 2 * math.pi)
