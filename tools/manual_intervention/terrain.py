"""Kinematic height following for the named walkable collisions in stairs.sdf.

This is a scene-geometry adapter, not contact dynamics or mesh collision.
"""

from dataclasses import dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET


@dataclass(frozen=True)
class Surface:
    xmin: float
    xmax: float
    ymin: float
    ymax: float
    height: float

    def contains(self, x, y):
        return (
            self.xmin - 1e-8 <= x <= self.xmax + 1e-8
            and self.ymin - 1e-8 <= y <= self.ymax + 1e-8
        )


def translation(element):
    values = [float(v) for v in element.findtext("pose", "0 0 0 0 0 0").split()]
    if (
        len(values) != 6
        or not all(math.isfinite(v) for v in values)
        or any(abs(v) > 1e-8 for v in values[3:])
    ):
        raise ValueError("Stair terrain requires finite, axis-aligned SDF poses")
    return values[:3]


@dataclass(frozen=True)
class StairTerrain:
    surfaces: tuple
    max_step: float = 0.20
    vertical_speed: float = 0.5

    @classmethod
    def from_sdf(cls, path):
        model = (
            ET.parse(Path(path))
            .getroot()
            .find("world/model[@name='stairs_switchback']")
        )
        if model is None:
            raise ValueError("No stairs_switchback model in terrain SDF")
        origin = translation(model)
        surfaces = []
        for link in model.findall("link"):
            link_origin = translation(link)
            for collision in link.findall("collision"):
                if not collision.get("name", "").startswith(
                    ("col_f1_s", "col_f2_s", "col_plat")
                ):
                    continue
                pose = translation(collision)
                size = [
                    float(v)
                    for v in collision.findtext("geometry/box/size", "").split()
                ]
                if len(size) != 3 or not all(math.isfinite(v) and v > 0 for v in size):
                    raise ValueError("Walkable stair collision must be a finite box")
                x, y, z = [a + b + c for a, b, c in zip(origin, link_origin, pose)]
                dx, dy, dz = size
                surfaces.append(
                    Surface(x - dx / 2, x + dx / 2, y - dy / 2, y + dy / 2, z + dz / 2)
                )
        if not surfaces:
            raise ValueError("No walkable stair collisions found")
        return cls(tuple(surfaces))

    def height(self, x, y):
        return max((s.height for s in self.surfaces if s.contains(x, y)), default=0.0)


def advance(pose, command, dt, terrain=None):
    """Body-origin (x,y,z,yaw), with bounded Z slew and high-edge rejection."""
    x, y, z, yaw = pose
    vx, vy, wz = command
    angle = yaw + wz * dt / 2
    nx = x + (vx * math.cos(angle) - vy * math.sin(angle)) * dt
    ny = y + (vx * math.sin(angle) + vy * math.cos(angle)) * dt
    heading = math.remainder(yaw + wz * dt, 2 * math.pi)
    if terrain is None:
        return nx, ny, z, heading
    current_height = terrain.height(x, y)
    target_height = terrain.height(nx, ny)
    if abs(target_height - current_height) > terrain.max_step + 1e-8:
        nx, ny, target_height = x, y, current_height
    dz = max(
        -terrain.vertical_speed * dt,
        min(terrain.vertical_speed * dt, target_height - z),
    )
    return nx, ny, z + dz, heading
