import unittest
import numpy as np
from tools.comparison_quality import sample_poses, choose_anchor

class QualityTest(unittest.TestCase):
    def poses(self, n):
        p = np.tile(np.eye(4), (n, 1, 1))
        p[:, 0, 3] = np.arange(n)
        return p

    def test_exact_good_sample_does_not_need_previous_good(self):
        out, valid = sample_poses([0, 1, 2], self.poses(3), [False, True, True], [1], 2)
        self.assertTrue(valid[0])
        self.assertEqual(out[0, 0, 3], 1)

    def test_no_extrapolation_or_crossing_invalid_gap(self):
        _, valid = sample_poses([0, 1, 2], self.poses(3), [True, False, True], [-1, .5, 1.5, 3], 2)
        self.assertFalse(valid.any())
        _, valid = sample_poses([0, 1], self.poses(2), [True, True], [.5], .1)
        self.assertFalse(valid.any())

    def test_duplicate_times_fail(self):
        with self.assertRaisesRegex(ValueError, 'strictly increasing'):
            sample_poses([0, 0], self.poses(2), [True, True], [0], .1)

    def test_bad_mocap_and_late_anchor_rejected(self):
        self.assertEqual(choose_anchor([0, .1, .2], [False, True, True], [True, False, True], .3), 2)
        with self.assertRaisesRegex(ValueError, '初始化'):
            choose_anchor([0, .1, .2], [False, True, True], [True, False, True], .15)

    def test_nan_pose_is_not_interpolated(self):
        p=self.poses(3);p[1]=np.nan
        _, valid=sample_poses([0, 1, 2],p,[True]*3,[.5, 2],2)
        self.assertEqual(valid.tolist(),[False,True])

if __name__ == '__main__':unittest.main()
