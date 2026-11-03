# SPDX-License-Identifier: MIT
import hashlib
import pytest
from tools.delivery.records import validate_references,decode_record
def test_reference_validator_does_not_trust_json_pass(tmp_path):
 p=tmp_path/'raw.log';p.write_text('real raw')
 ref={'path':'raw.log','sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
 assert validate_references(tmp_path,[ref])==[]
 assert validate_references(tmp_path,[{**ref,'sha256':'0'*64}])
 assert validate_references(tmp_path,[{'path':'missing','sha256':'0'*64}])
 assert validate_references(tmp_path,[{'path':'../foreign','sha256':'0'*64}])
def test_corrupt_or_unknown_record_fails_closed():
 for value in ['','99 1','3 1 "m"','1 1 a b c']:
  with pytest.raises(ValueError):decode_record(value)


def test_closed_sqlite_audit_preserves_every_evidence_byte(tmp_path):
 import sqlite3
 from tools.delivery.records import read_database
 dbpath=tmp_path/'mission.db'
 with sqlite3.connect(dbpath) as db:
  db.execute('PRAGMA user_version=2')
  for table in ['missions','manager','events','operations','registrations','queue_events','command_decisions','delivery_policy']:db.execute('CREATE TABLE '+table+'(x)')
 db.close()
 (tmp_path/'mission.db.owner.lock').touch()
 def inventory():return {p.name:p.read_bytes() for p in tmp_path.iterdir()}
 before=inventory();assert read_database(dbpath)['events']==[];assert inventory()==before
 # Do not silently ignore committed or hot-journal state in immutable mode.
 (tmp_path/'mission.db-wal').write_bytes(b'pending WAL')
 before=inventory()
 with pytest.raises(ValueError,match='pending SQLite'):read_database(dbpath)
 assert inventory()==before

def test_closed_sqlite_audit_refuses_a_live_owner(tmp_path):
 import fcntl
 from tools.delivery.records import read_database
 dbpath=tmp_path/'mission.db';dbpath.write_bytes(b'do not open')
 with (tmp_path/'mission.db.owner.lock').open('wb') as lock:
  fcntl.flock(lock.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
  with pytest.raises(ValueError,match='live Mission owner'):read_database(dbpath)
 assert dbpath.read_bytes()==b'do not open'
