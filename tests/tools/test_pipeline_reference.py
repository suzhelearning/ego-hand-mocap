import unittest
import numpy as np
from mocap_pipeline.reference import sample_reference

class ReferenceTest(unittest.TestCase):
    def test_pairing_uses_actual_times_and_separate_validity(self):
        ref={'times':np.array([0.,.2,.7]),'valid':np.array([True,False,True]),'poses':np.tile(np.eye(4),(3,1,1)),
             'rms':np.array([.001,.001,.009]),'vertices':np.zeros((3,2,3))}
        p=sample_reference(ref,np.array([-.1,.18,.68,.8]),0,3)
        self.assertEqual(p['indices'].tolist(),[0,1,2,2])
        self.assertEqual(p['covered'].tolist(),[False,True,True,False])
        self.assertFalse(p['device_valid'].any())
        self.assertEqual(p['hand_valid'].tolist(),[False,True,True,False])
        np.testing.assert_allclose(p['delta_s'],[.1,.02,.02,-.1])

if __name__=='__main__':unittest.main()

class MissingHandTest(unittest.TestCase):
    def test_explicit_absence_never_creates_valid_hand(self):
        ref={'times':np.array([0.,.2]),'valid':np.ones(2,bool),'poses':np.tile(np.eye(4),(2,1,1)),
             'rms':np.zeros(2),'vertices':np.zeros((2,778,3)),'mocap_hand_available':False}
        result=sample_reference(ref,np.array([.1]),0,3)
        self.assertTrue(result['device_valid'][0])
        self.assertFalse(result['hand_valid'][0])
