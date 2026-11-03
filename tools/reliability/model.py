# SPDX-License-Identifier: MIT
"""Independent finite FIFO reference model; never imports the CMR reducer/Store."""
import json,hashlib
from pathlib import Path
class QueueModel:
 def __init__(self):self.orders=[];self.active=None;self.paused=False
 def receive(self,op,id):
  row=next((r for r in self.orders if r['mission_id']==id),None)
  if op in [0,1]:
   if row:return 'EXISTING'
   if self.active is None and not self.paused and not any(r['queue_state']=='WAITING' for r in self.orders):state='ACTIVE';self.active=id;code='ACCEPTED'
   elif sum(r['queue_state']=='WAITING' for r in self.orders)>=4:return 'QUEUE_FULL'
   else:state='WAITING';code='QUEUED'
   self.orders.append({'mission_id':id,'queue_seq':len(self.orders)+1,'queue_state':state});return code
  if op==2:
   if row is None:return 'NOT_FOUND'
   if row['queue_state']=='WAITING':row['queue_state']='CANCELED';return 'QUEUE_CANCELED'
   if row['queue_state'] in ['CANCELED','FINISHED']:return 'ALREADY_TERMINAL'
   self.paused=True;self.active=None;row['queue_state']='FINISHED';return 'CANCEL_PENDING'
  if op==3:
   if self.active:return 'BUSY'
   if self.paused:return 'SCHEDULE_PAUSED'
   row=next((r for r in self.orders if r['queue_state']=='WAITING'),None)
   if row is None:return 'EMPTY'
   row['queue_state']='ACTIVE';self.active=row['mission_id'];return 'PROMOTED'
  self.paused=True;return 'PAUSED'
 def snapshot(self):return {'capacity':4,'orders':self.orders,'schedule_paused':self.paused}
def audit_replays(root):
 files=sorted(Path(root).rglob('replay.jsonl'));errors=[];count=0;seeds=[]
 for file in files:
  model=QueueModel();rows=[json.loads(x) for x in file.read_text().splitlines()];seed=rows[0]['seed'];seeds.append(seed)
  if len(rows)!=200:errors.append(str(file)+': not 200 operations')
  for i,row in enumerate(rows):
   actual=model.receive(row['operation'],row['mission_id']);count+=1
   if row['receive_order']!=i or actual!=row['decision'] or model.snapshot()!=row['queue']:
    errors.append({'seed':seed,'first_mismatch':i,'reference_decision':actual,'actual':row});break
 return {'valid':not errors,'errors':errors,'seeds':seeds,'operations':count,'generator':'std::mt19937 C++ delivery_store_test seeds v1; replay independent Python QueueModel v1','coverage':'admission/idempotence/capacity/cancel/no-motion resolution/FIFO/scheduling. Navigation, lease, crash and handover interleavings are separately checked by the registered deterministic and fixed-seed suites; this model does not certify unmodelled transitions.'}


def audit_handover_replays(root):
 from tools.delivery.records import decode_record
 files=sorted(Path(root).rglob('handover-replay.jsonl'));errors=[];count=0;seeds=[]
 for file in files:
  rows=[json.loads(x) for x in file.read_text().splitlines()];seeds.append(rows[0]['seed']);handed=False;pending='';returns=0;bounds=None
  if len(rows)!=200:errors.append(str(file)+': not 200 operations')
  for i,row in enumerate(rows):
   before=decode_record(row['before']);after=decode_record(row['after']);op=row['operation'];now=row['now'];count+=1
   if i%10==0:
    handed=False;pending='';returns=0;bounds=(before['deadline'],before['handover_until'],before['stop_until'],before['arrival_until'])
   if op==0 and not handed and not pending and now<min(bounds):handed=True;returns+=1
   elif op==1 and not pending:pending='CANCELED'
   elif op==2 and not pending and (now>=bounds[0] or not handed and now>=bounds[1]):pending='FAILED'
   if (after['handed_over'],after['pending'],row['return_intents'])!=(handed,pending,returns) or after['deadline']!=bounds[0]:
    errors.append({'seed':row['seed'],'first_mismatch':i,'reference':{'handed':handed,'pending':pending,'returns':returns},'actual':row});break
 return {'valid':not errors,'errors':errors,'seeds':seeds,'operations':count,'generator':'std::mt19937 handover episodes v1; independent Python acceptance/deadline/cancel first-writer model v1','scope':'synthetic pure FSM episodes, separate from actual SQLite/ROS/Gazebo evidence'}
