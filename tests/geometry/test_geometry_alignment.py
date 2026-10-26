import copy
import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
sys.path.insert(0,str(Path(__file__).resolve().parent))
from test_geometry_metrics import fixture
from geometry_metrics import evaluate, segment_minimum

class AlignmentCases(unittest.TestCase):
    def test_translation_rotation_exchange_invariance(self):
        ps=[[.2,.3],[.4,1.2],[-2,.4],[2,.7]]
        ref=segment_minimum(*ps,.2,.4,0,10)
        for theta in [0,.3,1.7,math.pi]:
            c,s=math.cos(theta),math.sin(theta)
            transformed=[[c*x-s*y+123,s*x+c*y-456] for x,y in ps]
            got=segment_minimum(*transformed,.2,.4,0,10)
            self.assertAlmostEqual(got['clearance_m'],ref['clearance_m'],places=12)
            self.assertAlmostEqual(got['alpha'],ref['alpha'],places=12)
        swap=segment_minimum(ps[2],ps[3],ps[0],ps[1],.4,.2,0,10)
        self.assertAlmostEqual(swap['clearance_m'],ref['clearance_m'],places=14)
    def test_reverse_segment_time_rejected(self):
        with self.assertRaises(ValueError):segment_minimum([0,0],[0,0],[2,0],[3,0],.5,.5,10,0)
    def test_multi_pair_missing_preserves_partial(self):
        d=fixture();c=copy.deepcopy(d['objects'][1]);c.update(name='third',entity_id=3,samples=[])
        d['objects'].append(c);d['pairs'].append(['robot','third']);r=evaluate(d)
        self.assertEqual(r['coverage'],'partial');self.assertEqual(r['pairs'][0]['coverage'],'full');self.assertEqual(r['pairs'][1]['coverage'],'none')
    def test_unknown_object_rejected(self):
        d=fixture();d['pairs'].append(['robot','unknown'])
        with self.assertRaises(ValueError):evaluate(d)
    def test_dense_other_series_cannot_hide_raw_gap(self):
        d=fixture(a=[(t,0.,0.) for t in range(11)],gap=9);p=evaluate(d)['pairs'][0]
        self.assertEqual(p['covered_ns'],0);self.assertTrue(all(not x['valid'] for x in p['intervals']))
    def test_isolated_observed_overlap_survives_gap(self):
        d=fixture(a=[(0,0.,0.),(5,0.,0.),(10,0.,0.)],b=[(5,0.,0.)]);p=evaluate(d)['pairs'][0]
        self.assertEqual(p['coverage'],'partial');self.assertIsNone(p['linear_model_min'])
        self.assertTrue(p['sample_min_geometric_contact']);self.assertIsNone(p['geometric_contact'])
    def test_radius_invalid_with_empty_series(self):
        d=fixture(a=[]);d['objects'][0]['radius_m']=-1
        with self.assertRaises(ValueError):evaluate(d)

if __name__=='__main__':unittest.main()
