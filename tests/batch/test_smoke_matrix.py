import sys,json,copy
from pathlib import Path
import pytest
from tools.batch.batch_core import *


def test_journal_append_hash_truncation_and_mutation(tmp_path):
 p=tmp_path/'journal';j=Journal(p);j.append('INTENT',attempt_id='a1',slot_id='s1');j.append('CALL_STARTED',attempt_id='a1',identity={'pid':1});assert len(journal_read(p))==2
 saved=p.read_bytes();p.write_bytes(saved[:-1])
 with pytest.raises(ValueError,match='truncated'):journal_read(p)
 p.write_bytes(saved.replace(b'"pid": 1',b'"pid": 2'))
 with pytest.raises(ValueError,match='hash'):journal_read(p)

def test_no_duplicate_dispatch_and_failed_resume():
 m={'slots':[{'slot_id':'s1'},{'slot_id':'s2'}]};r=[{'kind':'INTENT','attempt_id':'a1','slot_id':'s1'}]
 with pytest.raises(ValueError,match='unresolved'):choose_slot(m,r)
 r.append({'kind':'CALL_FINISHED','attempt_id':'a1','outcome':{'pass':False,'safe_to_resume':True,'first_reason':'X'}})
 with pytest.raises(ValueError,match='review'):choose_slot(m,r)
 assert choose_slot(m,r,'reviewed')['slot_id']=='s1'
 r.extend([{'kind':'INTENT','attempt_id':'a2','slot_id':'s1'},{'kind':'CALL_FINISHED','attempt_id':'a2','outcome':{'pass':True}}]);assert choose_slot(m,r)['slot_id']=='s2'

def test_same_cause_twice_and_unsafe_stop():
 m={'slots':[{'slot_id':'s1'}]};r=[]
 for a in ['a1','a2']:r.extend([{'kind':'INTENT','attempt_id':a,'slot_id':'s1'},{'kind':'CALL_FINISHED','attempt_id':a,'outcome':{'pass':False,'safe_to_resume':True,'first_reason':'X'}}])
 with pytest.raises(ValueError,match='twice'):choose_slot(m,r,'reviewed')
 r[-1]['outcome']['safe_to_resume']=False
 with pytest.raises(ValueError,match='unsafe'):choose_slot(m,r,'reviewed')

def test_preflight_no_side_effects(monkeypatch,tmp_path):
 import tools.batch.matrix as run_smoke_matrix, subprocess
 root=Path(__file__).resolve().parents[2];before={str(p) for d in ['evidence/runs','evidence/batches'] for p in (root/d).glob('*')}
 monkeypatch.setattr(subprocess,'Popen',lambda *a,**k:pytest.fail('check starts process'))
 monkeypatch.setattr(sys,'argv',['matrix','--matrix',str(root/'experiments/p0_smoke_matrix.yaml'),'--batch-dir',str(root/'evidence/batches/p0-08-check-never-created'),'--check'])
 assert run_smoke_matrix.main()==0
 assert before=={str(p) for d in ['evidence/runs','evidence/batches'] for p in (root/d).glob('*')}

def test_bad_matrix_order_and_hash():
 root=Path(__file__).resolve().parents[2];m=strict_json((root/'experiments/p0_smoke_matrix.yaml').read_text());m['slots'][0]['method']='dwb'
 with pytest.raises(ValueError,match='order'):validate_matrix(m,root,False)
 m=strict_json((root/'experiments/p0_smoke_matrix.yaml').read_text());m['slots'][0]['profile']['sha256']='0'*64
 with pytest.raises(ValueError,match='identity'):validate_matrix(m,root,False)

def test_first_cause_ignores_attempt_timestamp():
 m={'slots':[{'slot_id':'s1'}]};rows=[]
 for i in range(2):
  reason=json.dumps({'code':'CLOCK_STALE','phase':'WAIT_RESULT','detail':'missing','monotonic_s':i+10})
  rows.extend([{'kind':'INTENT','attempt_id':str(i),'slot_id':'s1'},{'kind':'CALL_FINISHED','attempt_id':str(i),'outcome':{'pass':False,'safe_to_resume':True,'first_reason':reason}}])
 with pytest.raises(ValueError,match='twice'):choose_slot(m,rows,'reviewed')
