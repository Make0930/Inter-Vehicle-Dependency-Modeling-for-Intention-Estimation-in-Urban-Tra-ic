from vehicle_prediction_pipeline.argpaser import parse_arguments
import unittest


class ParserTest(unittest.TestCase):
    def test(self):
        parse_arguments()
        self.assertEqual(1, 1)


if __name__ == '__main__':
    unittest.main()
