"""The public API is fenced by ``__all__``; changing it should be a deliberate decision."""

import unittest

import inspect_robots_b601


class ApiSnapshotTests(unittest.TestCase):
    def test_public_api(self):
        self.assertEqual(sorted(inspect_robots_b601.__all__), ["B601Embodiment", "__version__", "run_preflight"])
        for name in inspect_robots_b601.__all__:
            self.assertTrue(hasattr(inspect_robots_b601, name), name)


if __name__ == "__main__":
    unittest.main()
