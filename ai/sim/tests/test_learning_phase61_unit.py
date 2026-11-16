import unittest
import numpy as np
from learning.actions import normalize_action, clamp_reach
from learning.curriculum import agents_for_episode
from learning.config import LearningConfig
from learning.rewards import reward

class LearningUnitTests(unittest.TestCase):
    def test_curriculum(self):
        self.assertEqual([agents_for_episode(i) for i in (0,299,300,599,600,999)], [1,1,2,2,6,6])
    def test_actions(self):
        np.testing.assert_array_equal(normalize_action([2,-3,0]), [1,-1,0])
        with self.assertRaises(ValueError): normalize_action([float('nan'),0,0])
    def test_reach(self):
        point=clamp_reach([0,0,0],[1,0,-1],np.zeros(3),0.43,0)
        self.assertAlmostEqual(np.linalg.norm(point),0.43)
        self.assertGreaterEqual(point[2],0)
    def test_reward(self):
        cfg=LearningConfig()
        self.assertGreater(reward(.2,.1,False,False,cfg),reward(.2,.2,False,False,cfg))

if __name__ == '__main__': unittest.main()
