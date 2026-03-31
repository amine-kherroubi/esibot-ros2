import unittest

from esibot_bringup.esibot_driver import EsibotDriver


class TestBringupSmoke(unittest.TestCase):
    def test_driver_import(self):
        self.assertTrue(hasattr(EsibotDriver, "__init__"))


if __name__ == "__main__":
    unittest.main()
