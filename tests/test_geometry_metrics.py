import copy
import importlib.util
import math
from pathlib import Path
import random
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from geometry_metrics import evaluate, signed_clearance, segment_minimum


def fixture(a=None, b=None, interval=(0, 10), gap=10):
    def obj(name, identity, values):
        return {'name': name, 'entity_id': identity, 'radius_m': .5, 'center_offset_xyz': [0.,0.,0.],
                'samples': [{'t_ns': t, 'entity_id': identity, 'frame': 'world', 'epoch': 'fixture:0',
                             'xyz': [x,y,0.], 'center_xyz': [x,y,0.], 'quaternion_xyzw': [0.,0.,0.,1.],
                             'source': {'file': 'synthetic.jsonl', 'line': i+1, 'seq': i+1, 'pose_index': identity-1}}
                            for i, (t,x,y) in enumerate(values)]}
    return {'schema_version': 1, 'record_origin': 'fixture', 'run_id': 'fixture', 'frame': 'world', 'epoch': 'fixture:0',
            'interval_ns': list(interval), 'max_gap_ns': gap, 'numeric_epsilon_m': 1e-12,
            'objects': [obj('robot',1,a if a is not None else [(0,0.,0.),(10,0.,0.)]),
                        obj('actor',2,b if b is not None else [(0,-2.,0.),(10,2.,0.)])],
            'pairs': [['robot','actor']], 'physical_error_bound_m': None}


class Geometry(unittest.TestCase):
    def test_clearance(self):
        for b, expected in [([3,0],2),([1,0],0),([0,0],-1)]:
            self.assertEqual(signed_clearance([0,0],b,.5,.5),expected)
    def test_interior_crossing(self):
        p=evaluate(fixture())['pairs'][0]
        self.assertEqual(p['sample_min']['clearance_m'],1)
        self.assertEqual(p['linear_model_min']['clearance_m'],-1)
        self.assertEqual(p['linear_model_min']['time_ns'],5)
    def test_tangent(self):
        p=segment_minimum([0,0],[0,0],[-2,1],[2,1],.5,.5,0,10)
        self.assertEqual(p['clearance_m'],0);self.assertEqual(p['time_ns'],5)
    def test_endpoints(self):
        for b0,b1,t in [([2,0],[3,0],0),([3,0],[2,0],10)]:
            self.assertEqual(segment_minimum([0,0],[0,0],b0,b1,.5,.5,0,10)['time_ns'],t)
    def test_same_velocity(self):
        p=segment_minimum([0,0],[1,1],[2,0],[3,1],.5,.5,0,10)
        self.assertEqual(p['time_ns'],0);self.assertEqual(p['clearance_m'],1)
    def test_tiny_relative_motion(self):
        p=segment_minimum([0,0],[0,0],[-1e-200,0],[1e-200,0],0,0,0,10)
        self.assertEqual(p['alpha'],.5);self.assertEqual(p['clearance_m'],0)
    def test_degenerate(self):
        self.assertEqual(segment_minimum([0,0],[0,0],[2,0],[2,0],.5,.5,2,2)['time_ns'],2)
        with self.assertRaises(ValueError): segment_minimum([0,0],[1,0],[2,0],[2,0],.5,.5,2,2)
    def test_invalid_numbers(self):
        for x in [True,float('nan'),float('inf'),1e7]:
            with self.subTest(x=x),self.assertRaises(ValueError): signed_clearance([x,0],[0,0],1,1)
        with self.assertRaises(ValueError):signed_clearance([0,0],[0,0],-1,1)
    def test_gap_endpoints_not_bridge(self):
        p=evaluate(fixture(gap=9))['pairs'][0]
        self.assertEqual(p['coverage'],'partial');self.assertIsNone(p['linear_model_min'])
        self.assertEqual(p['intervals'][0]['reason'],'RAW_GAP_EXCEEDED')
    def test_multirate(self):
        p=evaluate(fixture(a=[(0,0.,0.),(5,0.,0.),(10,0.,0.)]))['pairs'][0]
        self.assertEqual(p['coverage'],'full');self.assertEqual(p['aligned_sample_min']['time_ns'],5)
        self.assertFalse(p['aligned_sample_min']['direct'])
    def test_no_extrapolation(self):
        p=evaluate(fixture(b=[(2,-2.,0.),(8,2.,0.)]))['pairs'][0]
        self.assertEqual(p['coverage'],'partial');self.assertEqual(p['covered_ns'],6)
    def test_no_data(self):
        self.assertEqual(evaluate(fixture(b=[]))['coverage'],'none')
    def test_single_sample(self):
        self.assertEqual(evaluate(fixture(b=[(5,0.,0.)]))['coverage'],'partial')
        self.assertEqual(evaluate(fixture(b=[(5,0.,0.)],interval=(5,5)))['coverage'],'full')
    def test_identity_epoch_frame(self):
        for key,value in [('entity_id',3),('entity_id',True),('frame','odom'),('epoch','fixture:1')]:
            d=fixture();d['objects'][0]['samples'][0][key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):evaluate(d)
    def test_nonfinite_full_pose(self):
        for key in ['xyz','center_xyz','quaternion_xyzw']:
            d=fixture();d['objects'][0]['samples'][0][key][-1]=float('nan')
            with self.assertRaises(ValueError):evaluate(d)
    def test_duplicates(self):
        d=fixture();d['objects'][0]['samples'].insert(1,copy.deepcopy(d['objects'][0]['samples'][0]))
        self.assertEqual(evaluate(d)['coverage'],'full')
        d['objects'][0]['samples'][1]['xyz'][0]=.1
        with self.assertRaises(ValueError):evaluate(d)
    def test_reverse(self):
        d=fixture();d['objects'][0]['samples'].reverse()
        with self.assertRaises(ValueError):evaluate(d)
    def test_gap_large_integer_time(self):
        t=8_000_000_000_000_000
        d=fixture(a=[(t,0.,0.),(t+3,0.,0.)],b=[(t,2.,0.),(t+3,2.,0.)],interval=(t,t+3),gap=2)
        self.assertEqual(evaluate(d)['coverage'],'partial')
    def test_partial_preserves_overlap_and_unknown(self):
        d=fixture(interval=(-0,20));r=evaluate(d)
        self.assertEqual(r['coverage'],'partial');self.assertTrue(r['pairs'][0]['geometric_contact']);self.assertIsNone(r['collision'])
    def test_random_dense_upper_bound(self):
        rng=random.Random(706)
        for _ in range(100):
            ps=[[rng.uniform(-4,4),rng.uniform(-4,4)] for _ in range(4)]
            m=segment_minimum(*ps,.2,.3,0,10)['clearance_m']
            samples=[]
            for k in range(1001):
                f=k/1000;a=[ps[0][j]+f*(ps[1][j]-ps[0][j]) for j in range(2)];b=[ps[2][j]+f*(ps[3][j]-ps[2][j]) for j in range(2)]
                samples.append(math.dist(a,b)-.5)
            self.assertLessEqual(m,min(samples)+1e-12)
            self.assertLess(min(samples)-m,.012)

if __name__=='__main__':unittest.main()
