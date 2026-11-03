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
  from tools.runner.runner_processes import identity
  members.setdefault(os.getpid(),identity(os.getpid()))
  rows=[];ticks=os.sysconf('SC_CLK_TCK');pages=os.sysconf('SC_PAGE_SIZE')
  for pid,info in members.items():
   try:
    text=Path('/proc/'+str(pid)+'/stat').read_text();fields=text[text.rfind(')')+2:].split();start=fields[19]
    if str(start)!=str(info['start_ticks']):raise RuntimeError('PID birth changed')
    cpu=(int(fields[11])+int(fields[12]))/ticks;rss=int(fields[21])*pages;key=(pid,start);old=self.previous.get(key);percent=0. if old is None else max(0.,(cpu-old[0])/(now-old[1])*100);self.previous[key]=(cpu,now)
    rows.append({'pid':pid,'start_ticks':start,'rss_bytes':rss,'cpu_percent':percent})
   except (FileNotFoundError,ProcessLookupError):continue
  return {'steady_s':now,'processes':rows,'rss_bytes':sum(x['rss_bytes'] for x in rows),'cpu_percent':sum(x['cpu_percent'] for x in rows),'free_bytes':shutil.disk_usage(self.root).free}


class RecordingProgress:
 """Observe byte growth on steady time; never call mtime a durability proof."""
 def __init__(self,root,gap_s,now=None):
  self.root=Path(root);self.gap=gap_s;self.last_bytes=self.size();self.last_advance=time.monotonic() if now is None else now
 def size(self):return sum(p.stat().st_size for p in (self.root/'bag').glob('*.mcap'))
 def sample(self,now=None,require_advance=False):
  now=time.monotonic() if now is None else now;size=self.size()
  if size<self.last_bytes:raise RuntimeError('recording file truncated')
  advanced=size>self.last_bytes
  if advanced:self.last_bytes=size;self.last_advance=now
  age=now-self.last_advance
  if size<=0 or age<0 or age>self.gap or require_advance and not advanced:raise RuntimeError('recorder byte progress missing')
  return {'bag_bytes':size,'bag_last_progress_age_s':age,'disk_visibility_only':True}
