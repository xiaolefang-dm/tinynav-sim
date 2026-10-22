"""Real staircase geometry, ascent/descent and forbidden high edges."""

import math
from pathlib import Path
import unittest
from terrain import StairTerrain, advance


class TerrainTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.terrain = StairTerrain.from_sdf(
            Path(__file__).resolve().parents[2] / "gazebo/worlds/stairs.sdf"
        )

    def test_reads_treads_and_landings_without_treating_rails_as_floor(self):
        t = self.terrain
        self.assertEqual(len(t.surfaces), 18)
        for i in range(8):
            self.assertAlmostEqual(t.height(2.14 + 0.28 * i, 0), 0.15 * (i + 1))
            self.assertAlmostEqual(
                t.height(4.94, 0.84 + 0.28 * i), 1.2 + 0.15 * (i + 1)
            )
        self.assertAlmostEqual(t.height(4.94, 0), 1.2)
        self.assertAlmostEqual(t.height(4.94, 3.64), 2.4)
        self.assertEqual(t.height(3.0, 0.725), 0)

    def test_climbs_both_flights_then_descends_to_ground_with_bounded_slew(self):
        pose = (0.0, 0.0, 0.0, 0.0)
        heights = []
        for axis, target in [(0, 4.94), (1, 3.64), (1, 0.0), (0, 0.0)]:
            for _ in range(1000):
                delta = target - pose[axis]
                if abs(delta) < 0.001:
                    break
                speed = max(-0.5, min(0.5, delta / 0.05))
                command = (speed, 0, 0) if axis == 0 else (0, speed, 0)
                nxt = advance(pose, command, 0.05, self.terrain)
                self.assertLessEqual(abs(nxt[2] - pose[2]), 0.025 + 1e-8)
                pose = nxt
                heights.append(pose[2])
            self.assertAlmostEqual(pose[axis], target, places=3)
            for _ in range(20):
                pose = advance(pose, (0, 0, 0), 0.05, self.terrain)
            if axis == 1 and target == 3.64:
                self.assertAlmostEqual(pose[2], 2.4)
        self.assertAlmostEqual(max(heights), 2.4)
        self.assertAlmostEqual(pose[2], 0.0)

    def test_rejects_side_jump_and_large_drop_but_allows_turning(self):
        for pose, command in [
            ((4.23, 1.4, 0.0, 0.0), (0.5, 0, 0.5)),
            ((4.25, 3.64, 2.4, 0.0), (-0.5, 0, 0.5)),
        ]:
            nxt = advance(pose, command, 0.05, self.terrain)
            self.assertEqual(nxt[:3], pose[:3])
            self.assertGreater(nxt[3], pose[3])

    def test_flat_mode_preserves_initial_height(self):
        nxt = advance((1.0, 2.0, 0.28, math.pi / 2), (0.5, 0, 0), 0.1)
        self.assertAlmostEqual(nxt[0], 1.0)
        self.assertAlmostEqual(nxt[1], 2.05)
        self.assertEqual(nxt[2], 0.28)


if __name__ == "__main__":
    unittest.main()
