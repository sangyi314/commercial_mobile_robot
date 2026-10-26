"""Offline preparation tests. Real inputs are explicitly selected, never 'latest'.

Set CMR_GEOMETRY_TEST_RUNS to a JSON array of archived run paths for integration.
Temporary outputs are specs/checks only; no formal evaluation report is created.
"""
import contextlib
import copy
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
import tools.geometry.spec_preparation as cli
from tools.geometry.geometry_inputs import ROOT, SOURCE_PATHS, prepare, schema, sha

class PreparationGuards(unittest.TestCase):
    def test_fixture_is_not_real(self):
        with tempfile.TemporaryDirectory() as t:
            run=Path(t)/'run';run.mkdir()
            (run/'manifest.json').write_text('{"record_origin":"fixture"}')
            with contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(cli.main(['--run-dir',str(run),'--output',str(Path(t)/'spec')]),2)
            self.assertFalse((Path(t)/'spec').exists())

    def test_existing_output_checked_before_input(self):
        with tempfile.TemporaryDirectory() as t:
            p=Path(t);out=p/'spec';out.write_text('keep')
            with patch.object(cli,'prepare') as source, contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(cli.main(['--run-dir',t,'--output',str(out)]),2)
                source.assert_not_called()
            self.assertEqual(out.read_text(),'keep')

    def test_catalog_rejected_by_spec_schema(self):
        with self.assertRaises(Exception):
            schema({'catalog_version':2,'specs':[]},'evaluation_spec')

    def test_output_in_run_and_symlink(self):
        with tempfile.TemporaryDirectory() as t:
            p=Path(t);run=p/'run';run.mkdir();(p/'alias').symlink_to(p,target_is_directory=True)
            for out in [run/'spec',p/'alias/spec']:
                with contextlib.redirect_stderr(io.StringIO()):
                    self.assertEqual(cli.main(['--run-dir',str(run),'--output',str(out)]),2)
                self.assertFalse(out.exists())

@unittest.skipUnless(os.environ.get('CMR_GEOMETRY_TEST_RUNS'),'explicit archived runs required')
class ArchivedPreparation(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.runs=[Path(p) for p in json.loads(os.environ['CMR_GEOMETRY_TEST_RUNS'])]
        if len(cls.runs)<2: raise ValueError('two explicit archived runs required')
        cls.specs=[prepare(r) for r in cls.runs]

    def generate(self,run,out,candidate=None):
        stdout=io.StringIO();stderr=io.StringIO()
        manager=patch.object(cli,'prepare',return_value=candidate) if candidate is not None else contextlib.nullcontext()
        with manager,contextlib.redirect_stdout(stdout),contextlib.redirect_stderr(stderr):
            code=cli.main(['--run-dir',str(run),'--output',str(out)])
        return code,stdout.getvalue(),stderr.getvalue()

    def check(self,run,path):
        return subprocess.run([sys.executable,'-B',str(ROOT/'tools/evaluate_geometry.py'),
                               '--run-dir',str(run),'--spec',str(path),'--check'],
                              capture_output=True,text=True,timeout=120)

    def test_real_generation_determinism_check_and_preservation(self):
        before={str(r/p):sha(r/p) for r in self.runs for p in SOURCE_PATHS}
        historical={str(p):sha(p) for p in (ROOT/'experiments/evaluations').glob('*') if p.is_file()}
        with tempfile.TemporaryDirectory() as t:
            for i,run in enumerate(self.runs):
                a=Path(t)/f'{i}a.json';b=Path(t)/f'{i}b.json'
                code,stdout,stderr=self.generate(run,a)
                self.assertEqual(code,0,stderr);schema(json.loads(a.read_text()),'evaluation_spec')
                review=json.loads(stderr)
                self.assertEqual(review['status'],'input_adapter_validated')
                self.assertIsNone(review['physical_error_bound_m'])
                self.assertEqual(stdout.strip(),str(a))
                self.assertEqual(self.generate(run,b)[0],0)
                self.assertEqual(a.read_bytes(),b.read_bytes())
                self.assertEqual(self.check(run,a).returncode,0)
                original=sha(a)
                self.assertEqual(self.generate(run,a)[0],2)
                self.assertEqual(sha(a),original)
                # Existing concrete specs remain byte-identical in substance.
                old=ROOT/'experiments/evaluations'/(run.name+'.json')
                if old.exists():self.assertEqual(json.loads(a.read_text()),json.loads(old.read_text()))
        self.assertEqual(before,{p:sha(p) for p in before})
        self.assertEqual(historical,{p:sha(p) for p in historical})

    def test_runs_have_distinct_frozen_identity(self):
        a,b=self.specs[:2]
        for key in ['run_id','epoch','scope','source_hashes']:
            self.assertNotEqual(a[key],b[key])
        # Entity IDs can legitimately repeat across separate cold starts.

    def test_invalid_candidates_never_written(self):
        cases=[]
        d=copy.deepcopy(self.specs[0]);del d['epoch'];cases.append(d)
        d=copy.deepcopy(self.specs[0]);d['source_hashes']['manifest.json']='0'*64;cases.append(d)
        d=copy.deepcopy(self.specs[0]);d['scene_identity']['models'][d['robot_name']]['id']+=10000;cases.append(d)
        d=copy.deepcopy(self.specs[0]);d['frame']='map';cases.append(d)
        with tempfile.TemporaryDirectory() as t:
            for i,d in enumerate(cases):
                out=Path(t)/str(i);code,_,_=self.generate(self.runs[0],out,d)
                self.assertEqual(code,2);self.assertFalse(out.exists())

    def test_missing_geometry_is_rejected(self):
        with tempfile.TemporaryDirectory() as t:
            p=Path(t);run=p/'copy';run.mkdir()
            (run/'manifest.json').write_bytes((self.runs[0]/'manifest.json').read_bytes())
            out=p/'spec'
            code,_,error=self.generate(run,out)
            self.assertEqual(code,2);self.assertIn('robot.sdf',error);self.assertFalse(out.exists())

    def test_evaluator_rejects_other_run_and_catalog(self):
        with tempfile.TemporaryDirectory() as t:
            p=Path(t)/'spec.json';p.write_text(json.dumps(self.specs[0]))
            self.assertNotEqual(self.check(self.runs[1],p).returncode,0)
            p.write_text(json.dumps({'catalog_version':2,'specs':[]}))
            self.assertEqual(self.check(self.runs[0],p).returncode,2)
