# SPDX-License-Identifier: MIT
import sys,json
from tools.mission import audit_delivery
def test_valid_records_with_failed_session_return_failure(monkeypatch,tmp_path):
 output=tmp_path/'audit.json';monkeypatch.setattr(sys,'argv',['audit_delivery','--session','fixture','--output',str(output)])
 monkeypatch.setattr(audit_delivery,'audit_session',lambda root:{'valid':True,'errors':[],'session_pass':False,'business_ok':False,'cleanup_ok':True})
 assert audit_delivery.main()==3
 assert json.loads(output.read_text())['valid'] is True
def test_protocol_audit_has_its_own_validation_layer(monkeypatch,tmp_path):
 output=tmp_path/'audit.json';monkeypatch.setattr(sys,'argv',['audit_delivery','--case','fixture','--output',str(output)])
 monkeypatch.setattr(audit_delivery,'audit_protocol_case',lambda root:{'valid':True,'errors':[],'synthetic':True})
 assert audit_delivery.main()==0
