import sys
from pathlib import Path
import pytest
from tools.batch.batch_core import *
def test_prestart_rejection_not_real_start(tmp_path):
 o=audit_run(tmp_path,{'run_id':'never-created','method':'mppi','purpose':'normal'},2)
 assert o['safe_to_resume'] and not o['pass'] and not o['actual_start'] and not o['cold_start']
def test_incomplete_is_not_repaired(tmp_path):
 r=tmp_path/'evidence/runs/bad';r.mkdir(parents=True);before=list(r.iterdir())
 o=audit_run(tmp_path,{'run_id':'bad','method':'dwb','purpose':'fault'},3)
 assert not o['safe_to_resume'] and o['audit_error'];assert before==list(r.iterdir())
def test_duplicate_completion_rejected():
 with pytest.raises(ValueError):attempts([{'kind':'INTENT','attempt_id':'a'},{'kind':'CALL_FINISHED','attempt_id':'a'},{'kind':'CALL_FINISHED','attempt_id':'a'}])
