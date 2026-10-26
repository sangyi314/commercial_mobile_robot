import contextlib
import copy
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from .test_geometry_metrics import fixture
from tools.geometry.geometry_inputs import sha, verify_sources, pose, loads, collisions, schema, SOURCE_PATHS
from tools.geometry.evaluation import main, output_guard

class InputsCLI(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name);(self.root/'in').mkdir()
        self.path=self.root/'in/fixture.json';self.write(fixture())
    def tearDown(self):self.temp.cleanup()
    def write(self,d):self.path.write_text(json.dumps(d))
    def call(self,*args):
        with contextlib.redirect_stdout(io.StringIO()),contextlib.redirect_stderr(io.StringIO()):
            return main(['--fixture-input',str(self.path),*args])
    def test_check_no_side_effects(self):
        before={str(p):sha(p) for p in self.root.rglob('*') if p.is_file()}
        self.assertEqual(self.call('--allow-fixtures','--check'),0)
        self.assertEqual(before,{str(p):sha(p) for p in self.root.rglob('*') if p.is_file()})
    def test_output_exclusive_and_valid_schema(self):
        out=self.root/'out';self.assertEqual(self.call('--allow-fixtures','--output-dir',str(out)),0)
        original=sha(out/'report.json');schema(json.loads((out/'report.json').read_text()),'evaluation_report')
        self.assertEqual(self.call('--allow-fixtures','--output-dir',str(out)),2)
        self.assertEqual(sha(out/'report.json'),original)
    def test_fixture_explicit(self):self.assertEqual(self.call('--check'),2)
    def test_partial_and_none_codes(self):
        self.write(fixture(gap=9));self.assertEqual(self.call('--allow-fixtures','--check'),4)
        self.write(fixture(b=[]));self.assertEqual(self.call('--allow-fixtures','--check'),3)
    def test_wrong_source(self):
        d=fixture();d['record_origin']='real';self.write(d)
        self.assertEqual(self.call('--allow-fixtures','--check'),2)
    def test_symlink_and_source_output(self):
        (self.root/'alias').symlink_to(self.root/'in')
        for p in [self.root/'alias/out',self.root/'in/out']:
            self.assertEqual(self.call('--allow-fixtures','--output-dir',str(p)),2)
    def test_changed_input(self):
        for name in SOURCE_PATHS:
            p=self.root/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text('a')
        spec={'source_hashes':{p:sha(self.root/p) for p in SOURCE_PATHS}}
        verify_sources(self.root,spec);(self.root/SOURCE_PATHS[0]).write_text('b')
        with self.assertRaisesRegex(ValueError,'SOURCE_HASH_CHANGED'):verify_sources(self.root,spec)
    def test_missing_pose_not_zero(self):
        with self.assertRaises(ValueError):pose({'orientation':{'w':1}})
        self.assertEqual(pose({'position':{},'orientation':{'w':1}})[0],[0.,0.,0.])
    def test_duplicate_json_and_nonfinite(self):
        for s in ['{"a":1,"a":2}','{"a":NaN}']:
            with self.assertRaises(ValueError):loads(s)
    def test_collision_not_visual_radius(self):
        p=self.root/'model.sdf';p.write_text('<sdf><model name="x"><link name="b"><visual name="v"><geometry><sphere><radius>100</radius></sphere></geometry></visual><collision name="c"><geometry><sphere><radius>0.1</radius></sphere></geometry></collision></link></model></sdf>')
        self.assertEqual(collisions(p)[1][0]['origin_bound_m'],.1)
    def test_schema_rejects_unknown_fields(self):
        d=fixture();d['trusted_pass']=True
        with self.assertRaises(Exception):schema(d,'evaluation_input')
    def test_physical_error_cannot_be_silently_assumed(self):
        d=fixture();d['physical_error_bound_m']=0
        with self.assertRaises(Exception):schema(d,'evaluation_input')
    def test_saved_counterexamples(self):
        folder=Path(__file__).resolve().parents[1]/'fixtures/geometry'
        for name,expected in [('interior',0),('gap',4),('wrong_epoch',2),('wrong_frame',2),('negative_radius',2),('conflicting_duplicate',2),('missing_pose',3)]:
            self.write(json.loads((folder/(name+'.json')).read_text()))
            with self.subTest(name=name):self.assertEqual(self.call('--allow-fixtures','--check'),expected)

if __name__=='__main__':unittest.main()
