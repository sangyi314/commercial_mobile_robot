#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Only owned child processes. stdin/stdout is a deterministic crash barrier."""
import json,os,selectors,signal,sqlite3,subprocess,sys
from pathlib import Path
binary=Path(sys.argv[1]);root=Path(sys.argv[2]);root.mkdir(parents=True,exist_ok=False)
results=[]
for window in ['submit-before','submit-after','intent-after','send-after','accept-after','cancel-after','terminal-before','terminal-after']:
 d=root/window;d.mkdir();db=d/'mission.db';sink=d/'receiver.log'
 p=subprocess.Popen([str(binary),str(db),'crash',window,str(sink)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,bufsize=1)
 lines=[]
 try:
  # select raw fd and read one byte avoids TextIO prefetch hiding a ready line.
  buf=b'';sel=selectors.DefaultSelector();sel.register(p.stdout,selectors.EVENT_READ)
  while True:
   if not sel.select(15):raise RuntimeError('barrier timeout '+window)
   chunk=os.read(p.stdout.fileno(),1)
   if not chunk:raise RuntimeError('child ended before barrier '+window)
   buf+=chunk
   if chunk==b'\n':
    line=buf.decode();lines.append(line);buf=b''
    if line.strip()=='BARRIER '+window:break
  second=subprocess.run([str(binary),str(db),'inspect'],text=True,capture_output=True,timeout=10)
  (d/'second-owner.stderr').write_text(second.stderr)
  assert second.returncode!=0 and 'owner lock' in second.stderr,'second process was not excluded'
  os.kill(p.pid,signal.SIGKILL);p.wait(timeout=10)
  assert p.returncode==-signal.SIGKILL
 finally:
  if p.poll() is None:p.kill();p.wait(timeout=10)
  p.stdin.close();sel.close()
 (d/'child.stdout').write_text(''.join(lines));(d/'child.stderr').write_text(p.stderr.read());p.stdout.close();p.stderr.close()
 before=sink.read_text() if sink.exists() else ''
 recovered=subprocess.run([str(binary),str(db),'inspect'],text=True,capture_output=True,timeout=15)
 (d/'recovery.stdout').write_text(recovered.stdout);(d/'recovery.stderr').write_text(recovered.stderr)
 assert recovered.returncode==0,recovered.stderr
 assert (sink.read_text() if sink.exists() else '')==before,'replayed external action'
 expected='ABSENT' if window=='submit-before' else 'COMPLETED' if window=='terminal-after' else 'PAUSED'
 assert recovered.stdout.startswith(expected),(window,recovered.stdout)
 with sqlite3.connect(db) as conn:
  snapshot={t:conn.execute('SELECT * FROM '+t).fetchall() for t in ['missions','manager','operations','events']}
 (d/'database.json').write_text(json.dumps(snapshot,indent=2))
 active=snapshot['manager'][0][1]
 assert (active is None)==(expected in ['ABSENT','COMPLETED'])
 row={'window':window,'pid':p.pid,'signal':'SIGKILL','exit':p.returncode,'recovery':expected,'sends':sum(x.split()[1]=='0' for x in before.splitlines()),'no_replay':True}
 results.append(row)
(root/'results.json').write_text(json.dumps(results,indent=2));print(json.dumps(results))
