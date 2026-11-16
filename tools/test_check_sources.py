"""How check_sources.py sorts fetch results. Run: python3 -m unittest discover -s tools"""
import os, sys, unittest
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import check_sources as C


class Classify(unittest.TestCase):
    def test_ok(self):
        self.assertEqual(C.classify(200)[0], 'ok')
        self.assertEqual(C.classify(302)[0], 'ok')

    def test_blocked_never_fails(self):
        for code in (401, 403, 429):
            self.assertEqual(C.classify(code)[0], 'blocked')
        self.assertEqual(C.classify(0, 'URLError: <urlopen error timed out>')[0], 'blocked')
        self.assertEqual(C.classify(0, 'URLError: <urlopen error [SSL: CERTIFICATE_VERIFY_FAILED] certificate verify failed>')[0], 'blocked')

    def test_dead(self):
        self.assertEqual(C.classify(404)[0], 'dead')
        self.assertEqual(C.classify(500)[0], 'dead')
        self.assertEqual(C.classify(0, 'URLError: <urlopen error [Errno 111] Connection refused>')[0], 'dead')
        self.assertEqual(C.classify(0, 'URLError: <urlopen error [Errno 8] nodename nor servname provided, or not known>')[0], 'dead')


class Citations(unittest.TestCase):
    def test_format(self):
        m = C.CITE.match('Chi, C. et al. (2023). Diffusion Policy: Visuomotor Policy Learning via Action Diffusion. https://arxiv.org/abs/2303.04137')
        self.assertEqual((m.group('year'), m.group('url')), ('2023', 'https://arxiv.org/abs/2303.04137'))
        self.assertIsNone(C.CITE.match('Open Robotics. Why ROS 2? https://design.ros2.org/articles/why_ros2.html'))   # no year

    def test_titles_match_loosely(self):
        self.assertTrue(C.same_title('π0: A Vision-Language-Action Flow Model for General Robot Control',
                                     '$\\pi_0$: A Vision-Language-Action Flow Model for General Robot Control'))
        self.assertFalse(C.same_title('Diffusion Policy', 'Learning Quadrupedal Locomotion over Challenging Terrain'))


if __name__ == '__main__':
    unittest.main()
