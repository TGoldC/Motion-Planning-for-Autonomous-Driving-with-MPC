import unittest
from configuration import ReferencePath


class TestReferencePath(unittest.TestCase):
    def setUp(self):
        self.test_class = ReferencePath("/home/xin/PycharmProjects/MPFAV_MPC/scenarios", "USA_Peach-2_1_T-1.xml")

    def test_route(self):
        self.assertEqual(self.test_class.route, real_route, 'false')  # How can I know the real_route firstly???

    def tearDown(self):
        print('test end')


if __name__ == '__main__':
    unittest.main()