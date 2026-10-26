"""Manual elevator interlocks and kinematic cabin support."""

import unittest
from elevator import Elevator, constrain


def state(lift):
    return {**lift.state(), "confirmed": True}


class ElevatorTest(unittest.TestCase):
    def test_closed_door_blocks_entry_and_open_door_allows_it(self):
        lift = Elevator()
        p = (2.6, 0.0, 0.0, 0.0)
        q = (2.75, 0.0, 0.0, 0.0)
        self.assertEqual(constrain(p, q, state(lift)), p)
        lift.door = 1.0
        self.assertEqual(constrain(p, q, state(lift)), q)

    def test_other_floor_door_cannot_be_crossed(self):
        lift = Elevator(height=3.0, door=1.0, target=1)
        p = (2.6, 0.0, 0.0, 0.0)
        self.assertEqual(constrain(p, (2.8, 0.0, 0.0, 0.0), state(lift)), p)

    def test_no_close_or_travel_with_robot_in_doorway(self):
        lift = Elevator(door=1.0)
        with self.assertRaisesRegex(ValueError, "Doorway"):
            lift.command("close", robot=(3.0, 0.0, 0.0))
        with self.assertRaisesRegex(ValueError, "Close doors"):
            lift.command("floor", 1)

    def test_door_obstruction_reopens(self):
        lift = Elevator(door=0.5, motion="closing")
        lift.step(0.1, (3.0, 0.0, 0.0))
        self.assertEqual(lift.motion, "opening")
        self.assertGreater(lift.door, 0.5)

    def test_robot_follows_car_without_command_and_cannot_walk_in_transit(self):
        lift = Elevator()
        lift.command("floor", 1, robot=(4.5, 0.0, 0.0))
        lift.step(2.0)
        self.assertEqual(
            constrain((4.5, 0.0, 0.0, 0.0), (4.6, 0.1, 0.0, 0.1), state(lift)),
            (4.5, 0.0, 1.0, 0.0),
        )
        with self.assertRaises(ValueError):
            lift.command("open")
        lift.step(4.0)
        self.assertEqual(lift.floor(), 1)
        lift.command("open")
        lift.step(2.0)
        self.assertTrue(lift.state()["door_open"])

    def test_cabin_walls_and_closed_exit_are_blocked(self):
        lift = Elevator(height=3.0, target=1)
        self.assertEqual(
            constrain((4.5, 0.65, 3.0, 0.0), (4.5, 0.75, 3.0, 0.0), state(lift)),
            (4.5, 0.65, 3.0, 0.0),
        )
        self.assertEqual(
            constrain((3.4, 0.0, 3.0, 0.0), (3.3, 0.0, 3.0, 0.0), state(lift)),
            (3.4, 0.0, 3.0, 0.0),
        )

    def test_exit_with_open_door_keeps_upper_floor_height(self):
        lift = Elevator(height=3.0, door=1.0, target=1)
        self.assertEqual(
            constrain((3.4, 0.0, 3.0, 0.0), (3.3, 0.0, 3.0, 0.0), state(lift)),
            (3.3, 0.0, 3.0, 0.0),
        )
        self.assertEqual(
            constrain((2.7, 0.0, 3.0, 0.0), (2.6, 0.0, 3.0, 0.0), state(lift)),
            (2.6, 0.0, 3.0, 0.0),
        )

    def test_unconfirmed_state_freezes_motion(self):
        p = (0.0, 0.0, 0.0, 0.0)
        self.assertEqual(constrain(p, (1.0, 0.0, 0.0, 0.0), None), p)

    def test_invalid_floor_and_actions_are_rejected(self):
        for floor in (True, 2, -1, "1", float("nan")):
            with self.assertRaises(ValueError):
                Elevator().command("floor", floor)
        with self.assertRaises(ValueError):
            Elevator().command("invalid")

    def test_only_current_landing_doors_move(self):
        poses = Elevator(height=3.0, door=1.0, target=1).entities()
        self.assertEqual(poses["landing_0_left"][1], 0.45)
        self.assertEqual(poses["landing_1_left"][1], 1.4)


if __name__ == "__main__":
    unittest.main()
