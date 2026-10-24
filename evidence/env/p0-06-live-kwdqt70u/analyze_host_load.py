"""Read closed actual host/Gazebo records. Output is a new review, never a run edit."""
from pathlib import Path
import argparse,json,math,statistics,sys
ROOT=Path('/home/robot/commercial_mobile_robot');sys.path.insert(0,str(ROOT/'tools'))
from inspect_dynamic_recording import raw_rows,ns

def distribution(values):
 a=sorted(values)
 return {'n':len(a),'mean':statistics.mean(a),'median':statistics.median(a),'minimum':a[0],'maximum':a[-1],'p05':a[max(0,math.ceil(.05*len(a))-1)],'p95':a[math.ceil(.95*len(a))-1]} if a else None

def review(run,host):
 run=Path(run);host=Path(host);events=raw_rows(host/'host-load.jsonl');raw=raw_rows(run/'runtime/gazebo-raw.jsonl');commands=raw_rows(run/'runtime/motion-commands.jsonl')
 report=json.loads((run/'runtime/dynamic-report.json').read_text());window=report['analysis_window'];start,end=window['start_ns'],window['end_ns']
 clocks=[x for x in raw if x['kind']=='clock'];nearstart=min(clocks,key=lambda x:abs(ns(x['protobuf']['sim'])-start));nearend=min(clocks,key=lambda x:abs(ns(x['protobuf']['sim'])-end));lo,hi=nearstart['receive_monotonic_s'],nearend['receive_monotonic_s']
 vm=[]
 for x in events:
  if x['kind']=='ubuntu_cpu' and x['exit']==0:
   j=json.loads(x['raw']);j['host_mid']=(x['host_send_monotonic_s']+x['host_receive_monotonic_s'])/2;j['rtt']=x['host_receive_monotonic_s']-x['host_send_monotonic_s'];vm.append(j)
 if len(vm)<5:raise ValueError('host/VM mapping samples missing')
 offsets=[x['monotonic_s']-x['host_mid'] for x in vm];offset=statistics.median(offsets);uncertainty=max(x['rtt'] for x in vm)/2+max(abs(x-offset) for x in offsets)
 mac=[x['busy_fraction'] for x in events if x['kind']=='mac_cpu' and lo+uncertainty<=x['host_monotonic_s']+offset<=hi-uncertainty]
 if len(mac)<5 or uncertainty>1.:raise ValueError('host motion-window coverage or clock uncertainty')
 vmcpu=[]
 for a,b in zip(vm,vm[1:]):
  if not lo<=a['monotonic_s']<=b['monotonic_s']<=hi:continue
  previous=[int(x) for x in a['cpu_stat'].split()[1:9]];current=[int(x) for x in b['cpu_stat'].split()[1:9]];d=[y-x for x,y in zip(previous,current)]
  if min(d)<0 or sum(d)<=0:raise ValueError('proc cpu counter changed')
  vmcpu.append(1-(d[3]+d[4])/sum(d))
 stats=[x for x in raw if x['kind']=='stats' and start<=ns(x['protobuf'].get('sim_time',{}))<=end and not x['protobuf'].get('paused',False)]
 rtf=[float(x['protobuf']['real_time_factor']) for x in stats if 'real_time_factor' in x['protobuf']]
 workers=[]
 for p in sorted(host.glob('load-worker-*.log')):
  j=json.loads(p.read_text());duration=j['end_monotonic_s']-j['start_monotonic_s'];j['own_cpu_fraction']=j['cpu_s']/duration;workers.append(j)
 metadata=json.loads((host/'metadata.json').read_text());errors=json.loads((host/'sampling-errors.json').read_text())
 if errors:raise ValueError(errors)
 if metadata['load_request']=='higher' and (len(workers)!=2 or any(x['nice']!=19 or x['own_cpu_fraction']<.8 or x['end_monotonic_s']-x['start_monotonic_s']>121 for x in workers)):raise ValueError('higher load not established by actual owned worker')
 if metadata['load_request']=='low' and workers:raise ValueError('low run has pressure workers')
 return {'run':str(run),'host_evidence':str(host),'condition':metadata['load_request'],'mac_cpus':metadata['logical_cpus'],'ubuntu_cpus':sorted(set(x['cpus'] for x in vm)),'source_motion_window_ns':[start,end],'vm_monotonic_window_s':[lo,hi],'host_to_vm_offset_s':offset,'mapping_uncertainty_wall_s':uncertainty,'mapping_method':'independent read-only SSH clock sample RTT; not a fit or shift of trajectories; Mac parent clock per trial and worker clocks have separate origins','mac_busy_fraction':distribution(mac),'ubuntu_busy_fraction':distribution(vmcpu),'active_gazebo_rtf':distribution(rtf),'owned_load_workers':workers,'trajectory':{k:v for k,v in report['trajectory'].items() if k!='paused_source_duplicate_groups'},'probe_pass':report['probe_pass'],'dynamic_recording_valid':report['dynamic_recording_valid'],'human_confirmation':None}

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--run',required=True);p.add_argument('--host',required=True);p.add_argument('--output',required=True);a=p.parse_args();j=review(a.run,a.host);Path(a.output).open('x').write(json.dumps(j,indent=2)+'\n');print(json.dumps({k:j[k] for k in ['condition','mac_busy_fraction','ubuntu_busy_fraction','active_gazebo_rtf','mapping_uncertainty_wall_s']}))
