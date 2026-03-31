import unittest

from esibot_logging import get_logger


class TestLoggingSmoke(unittest.TestCase):
    def test_get_logger(self):
        logger = get_logger(__name__)
        self.assertIsNotNone(logger)


if __name__ == "__main__":
    unittest.main()
