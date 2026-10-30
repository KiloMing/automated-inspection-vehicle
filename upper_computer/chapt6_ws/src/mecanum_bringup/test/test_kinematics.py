import math
import random
import unittest
from mecanum_bringup.kinematics import Geometry, inverse, forward, limit_wheels


class KinematicsTest(unittest.TestCase):
    def setUp(self):
        self.g = Geometry(0.05, 0.15, 0.125)

    def test_basis_vectors(self):
        self.assertEqual(inverse(1, 0, 0, self.g), (20, 20, 20, 20))
        self.assertEqual(inverse(0, 1, 0, self.g), (-20, 20, 20, -20))
        self.assertEqual(inverse(0, 0, 1, self.g), (-5.5, 5.5, -5.5, 5.5))

    def test_round_trip(self):
        rng = random.Random(3)
        for _ in range(200):
            v = [rng.uniform(-2, 2) for _ in range(3)]
            for actual, expected in zip(forward(inverse(*v, self.g), self.g), v):
                self.assertAlmostEqual(actual, expected)

    def test_invalid_geometry(self):
        for bad in (0, -1, math.inf, math.nan):
            for args in ((bad, .1, .1), (.1, bad, .1), (.1, .1, bad)):
                with self.assertRaises(ValueError):
                    Geometry(*args)

    def test_invalid_velocity(self):
        for bad in (math.nan, math.inf):
            with self.assertRaises(ValueError):
                inverse(bad, 0, 0, self.g)
            with self.assertRaises(ValueError):
                forward((0, bad, 0, 0), self.g)
        with self.assertRaises(ValueError):
            forward((0, 0, 0), self.g)

    def test_saturation_preserves_direction(self):
        raw = inverse(.5, .2, .3, self.g)
        limited = limit_wheels(raw, 4)
        self.assertAlmostEqual(max(map(abs, limited)), 4)
        ratios = [a / b for a, b in zip(limited, raw)]
        self.assertAlmostEqual(min(ratios), max(ratios))
        self.assertEqual(limit_wheels((0, 0, 0, 0), 4), (0, 0, 0, 0))
        for bad in (0, -1, math.nan, math.inf):
            with self.assertRaises(ValueError):
                limit_wheels(raw, bad)


if __name__ == '__main__':
    unittest.main()
