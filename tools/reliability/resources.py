# SPDX-License-Identifier: MIT
"""Read only the original supervisor's owned /proc trees; no signalling."""
import os,time,shutil
from pathlib import Path
class Resources:
 def __init__(self,supervisor,root):self.owner=supervisor;self.root=root;self.previous={};self.last=None
 def sample(self):
  now=time.monotonic();members={}
  for item in self.owner.owned:
   if item.closed:continue
   for info in self.owner.members(item):members[info['pid']]=info
  rows=[];ticks=os.sysconf('SC_CLK_TCK');pages=os.sysconf('SC_PAGE_SIZE')
  for pid,info in members.items():
   try:
    fields=Path('/proc/'+str(pid)+'/stat').read_text().split(') ')[1].split();start=fields[19]
    if str(start)!=str(info['start_ticks']):raise RuntimeError('PID birth changed')
    cpu=(int(fields[11])+int(fields[12]))/ticks;rss=int(fields[21])*pages;key=(pid,start);old=self.previous.get(key);percent=0. if old is None else max(0.,(cpu-old[0])/(now-old[1])*100);self.previous[key]=(cpu,now)
    rows.append({'pid':pid,'start_ticks':start,'rss_bytes':rss,'cpu_percent':percent})
   except (FileNotFoundError,ProcessLookupError):continue
  return {'steady_s':now,'processes':rows,'rss_bytes':sum(x['rss_bytes'] for x in rows),'cpu_percent':sum(x['cpu_percent'] for x in rows),'free_bytes':shutil.disk_usage(self.root).free}
