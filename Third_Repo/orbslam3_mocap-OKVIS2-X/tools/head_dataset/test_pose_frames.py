"""Regression checks for camera optical-centre output; no dataset needed."""
import tempfile
import unittest
from pathlib import Path
import numpy as np
from pose_frames import to_cam0, export_cam0, world_flu, quaternion_rotation, rotation_quaternion, R_OPTICAL_FLU


class OpticalPoseTest(unittest.TestCase):
    def test_rotated_lever_arm_and_rotation_order(self):
        transform = np.array([[1.,0,0,.1], [0,0,-1,.2], [0,1,0,.3], [0,0,0,1]])
        stamp=9007199254740993
        row=[stamp,1.,2.,3.,0.,0.,np.sqrt(.5),np.sqrt(.5), *range(9)]
        result=to_cam0([row],transform)[0]
        np.testing.assert_allclose(result[1:4], [.8,2.1,3.3], atol=1e-14)
        np.testing.assert_allclose(result[4:8], [.5,.5,.5,.5], atol=1e-14)
        self.assertEqual(result[0],stamp)
        self.assertEqual(result[8:],row[8:])
        with tempfile.TemporaryDirectory() as directory:
            export_cam0(directory,{'ba':[result]})
            lines=(Path(directory)/'trajectory_cam0.tum').read_text().splitlines()
            self.assertTrue(lines[1].startswith('9007199.254740993 '))
            self.assertIn('T_worldFLU_cam0FLU',lines[0])

    def test_empty_and_identity(self):
        self.assertEqual(to_cam0([],np.eye(4)),[])
        row=[1,4.,5.,6.,0.,0.,0.,1.]
        np.testing.assert_allclose(to_cam0([row],np.eye(4)),[row])

    def test_world_and_body_flu_with_shared_map_transform(self):
        # Level camera faces estimator +Y; output heading must become +X.
        q=[-np.sqrt(.5),0,0,np.sqrt(.5)]
        streams={'ba': [[1,3.,4.,5.,*q,1,2,3,4,5,6,7,8,9],
                        [2,3.,6.,8.,*q,1,2,3,4,5,6,7,8,9]],
                 'live': [[1,3.1,4.2,5.3,*q]]}
        converted,origin,H=world_flu(streams,'ba')
        np.testing.assert_array_equal(converted['ba'][0][1:4],[0,0,0])
        np.testing.assert_allclose(converted['ba'][1][1:4],[2,0,3],atol=1e-14)
        np.testing.assert_allclose(converted['live'][0][1:4],[.2,-.1,.3],atol=1e-14)
        np.testing.assert_allclose(quaternion_rotation(converted['ba'][0][4:8]),np.eye(3),atol=1e-14)
        np.testing.assert_allclose(converted['ba'][0][8:11],[2,-1,3],atol=1e-14)
        self.assertEqual(converted['ba'][0][11:],streams['ba'][0][11:])
        np.testing.assert_allclose(H@[0,0,1],[0,0,1],atol=1e-14)
        np.testing.assert_allclose(np.linalg.det(H),1)
        np.testing.assert_array_equal(R_OPTICAL_FLU.T@[0,0,1],[1,0,0])
        np.testing.assert_array_equal(R_OPTICAL_FLU.T@[1,0,0],[0,-1,0])
        np.testing.assert_array_equal(R_OPTICAL_FLU.T@[0,1,0],[0,0,-1])
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory)
            (folder/'okvis2-slam-final_map.csv').write_text('landmarks:\n7,4,6,8\nkeyframes:\n')
            metadata={'origin_in_estimator_world_m':origin.tolist(),
                      'R_world_flu_from_estimator_world':H.tolist()}
            export_cam0(folder,converted,metadata)
            values=(folder/'landmarks_cam0_world.csv').read_text().splitlines()[1].split(',')
            np.testing.assert_allclose([float(v) for v in values[1:]],[2,-1,3],atol=1e-14)

    def test_tilt_is_preserved_while_world_stays_level(self):
        from scipy.spatial.transform import Rotation
        expected=Rotation.from_euler('ZYX',[43,20,-12],degrees=True).as_matrix()
        q=Rotation.from_matrix(expected@R_OPTICAL_FLU.T).as_quat()
        converted,_,H=world_flu({'ba':[[1,1,2,3,*q]]},'ba')
        actual=quaternion_rotation(converted['ba'][0][4:8])
        np.testing.assert_allclose(actual,H@expected,atol=1e-14)
        np.testing.assert_allclose(H@[0,0,1],[0,0,1],atol=1e-14)
        self.assertGreater(actual[0,0],0)
        self.assertAlmostEqual(actual[1,0],0,places=14)
        self.assertFalse(np.allclose(actual,np.eye(3)))
        # Positions and camera-local directions must transform consistently.
        point=np.array([.3,-.2,.8])
        np.testing.assert_allclose(actual@point,H@expected@point,atol=1e-14)

    def test_vertical_heading_is_rejected(self):
        with self.assertRaisesRegex(ValueError,'vertically'):
            world_flu({'live':[[1,0,0,0,0,0,0,1]]},'live')
        streams,origin,H=world_flu({'live':[]},'live')
        self.assertEqual(streams,{'live':[]})
        np.testing.assert_array_equal(origin,[0,0,0])
        np.testing.assert_array_equal(H,np.eye(3))


if __name__=='__main__':
    unittest.main()
