#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Explicit simulation authorization fixture. No Action client or motion feedback."""
import argparse,json,os,select,signal,sys,time,sqlite3
from pathlib import Path
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--run-dir',required=True);a=ap.parse_args();run=Path(a.run_dir);p=json.loads((run/'artifacts/profile.json').read_text());e=p['execution_v1']
 if os.environ.get('ROS_DOMAIN_ID')!='71' or e['synthetic'] or e['goal_owner']!='original Runner cmr_worker':raise ValueError('simulation fixture configuration')
 import rclpy
 from rclpy.signals import SignalHandlerOptions
 from cmr_interfaces.srv import ExecutionLease
 from cmr_interfaces.msg import ExecutionIdentity
 from rosidl_runtime_py.convert import message_to_ordereddict as dic
 rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=rclpy.create_node('authorization_fixture',namespace=e['namespace']);c=node.create_client(ExecutionLease,'lease');seq=0;lease='';who=ExecutionIdentity();nonce=0;active=False;running=True;last_renew=0.;heart=0.;buf=b'';f=(run/'runtime/authorization-fixture.jsonl').open('x')
 db=sqlite3.connect(run/'runtime/authorization.db');db.execute('PRAGMA journal_mode=WAL');db.execute('PRAGMA synchronous=FULL');db.execute('CREATE TABLE intents(seq INTEGER PRIMARY KEY, request_id TEXT UNIQUE, operation TEXT NOT NULL, payload TEXT NOT NULL, response TEXT)');db.commit()
 def end(sig,frame):
  nonlocal running
  running=False
 signal.signal(signal.SIGINT,end);signal.signal(signal.SIGTERM,end)
 def record(k,v):f.write(json.dumps({'kind':k,'local_monotonic_s':time.monotonic(),'data':v})+'\n');f.flush()
 def wire(k,d,rid=''):
  nonlocal seq
  seq+=1;print(json.dumps({'protocol_version':1,'run_id':run.name,'seq':seq,'request_id':rid,'type':k,'source_monotonic_s':time.monotonic(),'ros_s':None,'data':d}),flush=True)
 def call(op,q=None):
  nonlocal nonce,heart
  if time.monotonic()-heart>.4:wire('HEARTBEAT',{'active':active});heart=time.monotonic()
  if q is None:
   nonce+=1;q=ExecutionLease.Request();q.identity=who;q.request_id='fixture_'+str(nonce);q.request_seq=nonce;q.lease_id=lease;q.old_goal_resolved=True
  q.operation=op;record('request',dic(q));db.execute('INSERT INTO intents(request_id,operation,payload) VALUES(?,?,?)',(q.request_id+'.'+op,op,json.dumps(dic(q))));db.commit();future=c.call_async(q);until=time.monotonic()+.8
  while not future.done() and time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.005)
  if not future.done():c.remove_pending_request(future);raise TimeoutError('fixture lease RPC')
  r=future.result();record('response',dic(r));db.execute('UPDATE intents SET response=? WHERE request_id=?',(json.dumps(dic(r)),q.request_id+'.'+op));db.commit();return r,q
 def challenge(op):
  r,q=call('challenge_'+op)
  if r.code!='CHALLENGE':return r
  q.challenge=r.challenge;return call(op,q)[0]
 def revoke():
  nonlocal active
  active=False;return dic(call('revoke')[0])
 os.set_blocking(sys.stdin.fileno(),False);wire('HELLO',{'fixture':True,'no_action_client':True})
 try:
  while running:
   rclpy.spin_once(node,timeout_sec=.005);now=time.monotonic()
   if now-heart>.4:wire('HEARTBEAT',{'active':active});heart=now
   if active and now-last_renew>.08:
    r=challenge('renew');last_renew=time.monotonic()
    if r.code!='RENEWED':active=False;wire('LEASE_LOST',dic(r))
   if select.select([sys.stdin],[],[],.005)[0]:
    data=os.read(sys.stdin.fileno(),65536)
    if not data:break
    buf+=data
    while b'\n' in buf:
     line,buf=buf.split(b'\n',1);cmd=json.loads(line);rid=cmd['request_id'];op=cmd['data']['op']
     try:
      if cmd['run_id']!=run.name or time.monotonic()>=cmd['data']['deadline']:raise ValueError('expired/wrong run')
      if op=='begin':
       until=min(cmd['data']['deadline'],time.monotonic()+10)
       while time.monotonic()<until:
        if not c.service_is_ready():rclpy.spin_once(node,timeout_sec=.02);continue
        r,q=call('inspect');who=ExecutionIdentity(run_id=run.name,gate_epoch=r.gate_epoch,owner_epoch=e['owner_epoch'],mission_id='simulation_fixture',operation_id=run.name+'.out',attempt_id=run.name+'.out.0');r=challenge('begin')
        if r.code=='GRANTED':lease=r.lease_id;active=True;last_renew=time.monotonic();break
        rclpy.spin_once(node,timeout_sec=.02);time.sleep(.05)
       else:raise RuntimeError('initial independent stationary/route qualification absent')
       value=dic(r)
      elif op=='revoke':value=revoke()
      elif op=='exit':value=revoke() if lease else {'unarmed':True};running=False
      else:raise ValueError('unknown fixture op')
      wire('RESULT',{'ok':True,'value':value},rid)
     except Exception as ex:active=False;wire('RESULT',{'ok':False,'error':str(ex)},rid)
 finally:
  if lease:
   try:revoke()
   except Exception as ex:record('shutdown_revoke_unknown',str(ex))
  db.close();f.close();node.destroy_node();rclpy.shutdown()
 return 0
if __name__=='__main__':raise SystemExit(main())
