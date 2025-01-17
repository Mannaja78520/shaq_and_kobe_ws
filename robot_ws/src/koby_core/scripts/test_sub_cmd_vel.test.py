import unittest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from test_sub_cmd_vel import test_sub_cmd_vel

class TestSubCmdVel(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = test_sub_cmd_vel()

    def tearDown(self):
        self.node.destroy_node()

if __name__ == '__main__':
    unittest.main()
