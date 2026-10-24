import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
sys.path.insert(0,str(Path(__file__).resolve().parent))
from geometry_inputs import actual_model, pose, raw_time, scene_identity, schema, collisions, prepare, SOURCE_PATHS, sha
from evaluate_geometry import main
from test_geometry_metrics import fixture

class RawInputCases(unittest.TestCase):
    def test_actual_not_command_or_localization(self):
        actual={'id':41,'name':'actor','position':{'x':2},'orientation':{'w':1},'command_pose':{'x':999},'amcl':{'x':999},'odom':{'x':999}}
        self.assertEqual(pose(actual_model([actual],'actor',41))[0],[2.,0.,0.])
        self.assertIsNone(actual_model([{'id':7,'name':'odom'}],'actor',41))
    def test_wrong_entity_name_replacement(self):
        for entry in [{'id':41,'name':'wrong'},{'id':42,'name':'actor'}]:
            with self.assertRaises(ValueError):actual_model([entry],'actor',41)
    def test_wrong_time_and_bool(self):
        for s in [{'sec':True},{'nsec':1e9},{'sec':-1},{'sec':'nan'},{'nsec':True}]:
            with self.assertRaises(ValueError):raw_time(s)
        self.assertEqual(raw_time({'sec':'1','nsec':2}),1000000002)
    def test_raw_missing_position_never_zero(self):
        with self.assertRaises(ValueError):actual_model([{'id':41,'name':'actor','orientation':{'w':1}}],'actor',41)
    def test_partial_scene_identity_change(self):
        with tempfile.TemporaryDirectory() as t:
            p=Path(t)/'fixture.jsonl'
            rows=[{'kind':'scene','seq':1,'record_origin':'fixture','protobuf':{'model':[{'name':'robot','id':1,'link':[{'name':'body','id':11}]},{'name':'actor','id':2,'link':[{'name':'body','id':22}]}]}},
                  {'kind':'scene','seq':2,'record_origin':'fixture','protobuf':{'model':[{'name':'actor','id':3,'link':[{'name':'body','id':33}]}]}}]
            p.write_text('\n'.join(json.dumps(x) for x in rows)+'\n')
            with self.assertRaisesRegex(ValueError,'IDENTITY_CHANGED'):scene_identity(p,['robot','actor'])
    def test_unknown_units_schema(self):
        d=fixture();d['units']='millimeters'
        with self.assertRaises(Exception):schema(d,'evaluation_input')
    def test_missing_run_result_is_not_recreated(self):
        with tempfile.TemporaryDirectory() as t:
            p=Path(t);run=p/'source';run.mkdir();(run/'manifest.json').write_text('{}')
            sources={name:'0'*64 for name in SOURCE_PATHS};sources['manifest.json']=sha(run/'manifest.json')
            spec={'schema_version':1,'run_id':'explicit-missing-input-test','record_origin':'real','epoch':'test:0','frame':'world',
                  'scope':{'interval_ns':[0,10],'rule':'accepted_to_max_terminal_plus_5s_or_frozen_motion_end','uuid':'synthetic','action_lines':[1,2],'motion_arm_line':1,'terminal':'SUCCEEDED'},
                  'max_gap_ns':50000000,'robot_name':'robot','obstacle_name':'actor','scene_identity':{'models':{'robot':{'id':1,'links':{'body':11}},'actor':{'id':2,'links':{'body':22}}},'line':1,'seq':1},
                  'source_hashes':sources,'robot_radius_rule':'all_collision_3d_bound_about_model_origin','physical_error_bound_m':None}
            # A synthetic candidate cannot become a real run: result is absent and
            # the existing validator is never bypassed. No output is produced.
            s=p/'spec.json';s.write_text(json.dumps(spec));schema(spec,'evaluation_spec')
            before={str(x):x.read_bytes() for x in p.rglob('*') if x.is_file()}
            with contextlib.redirect_stderr(io.StringIO()),contextlib.redirect_stdout(io.StringIO()):
                code=main(['--run-dir',str(run),'--spec',str(s),'--check'])
            self.assertEqual(code,3);self.assertFalse((run/'result.json').exists())
            self.assertEqual(before,{str(x):x.read_bytes() for x in p.rglob('*') if x.is_file()})
    def test_fixture_run_not_promoted_to_real(self):
        with tempfile.TemporaryDirectory() as t:
            p=Path(t);(p/'manifest.json').write_text('{"record_origin":"fixture"}')
            with self.assertRaisesRegex(ValueError,'REAL_RUN_REQUIRED'):prepare(p)

if __name__=='__main__':unittest.main()
