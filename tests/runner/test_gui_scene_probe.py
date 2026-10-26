"""Scene response/updates checks with fake Transport; no Gazebo processes."""
import sys
from pathlib import Path
import pytest
pytest.importorskip('gz.transport13')
import tools.runner.gui_scene_probe as probe

def scene_fixture():
 scene=probe.Scene()
 for i,name in enumerate(['ground_plane','turtlebot3_world','turtlebot3_waffle'],1):
  model=scene.model.add();model.name=name;model.id=i
  if name=='turtlebot3_world':model=model.model.add();model.name='obstacles'
  model.link.add().visual.add().name='geometry'
 return scene

@pytest.mark.parametrize('case',['valid','missing_model','empty_geometry','request_timeout','stale_clock','missing_pose','subscribe_failed','wrong_partition'])
def test_probe_rejects_missing_evidence_and_unsubscribes(monkeypatch,tmp_path,capsys,case):
 scene=scene_fixture();calls=[];subscribed=[]
 if case=='missing_model':del scene.model[-1]
 if case=='empty_geometry':scene.model[1].ClearField('model')
 class Node:
  def request(self,*args):
   assert not subscribed
   calls.append('request');return case!='request_timeout',scene
  def subscribe(self,typ,topic,callback):
   subscribed.append(topic)
   if case=='subscribe_failed':return False
   for i in range(1,4):
    m=typ()
    if topic.endswith('/clock'):m.sim.sec=1 if case=='stale_clock' else i
    elif topic.endswith('/stats'):m.sim_time.sec=i
    elif case!='missing_pose':m.pose.add().id=3
    callback(m)
   return True
  def unsubscribe(self,topic):calls.append(topic);return True
 monkeypatch.setattr(probe,'Node',Node);monkeypatch.setenv('GZ_PARTITION','wrong' if case=='wrong_partition' else 'cmr_'+tmp_path.name)
 monkeypatch.setattr(sys,'argv',['probe',str(tmp_path)])
 if case=='valid':
  probe.main()
  import json
  result=json.loads(capsys.readouterr().out);assert result['valid'] and result['visual_confirmed'] is None
 else:
  with pytest.raises(ValueError):probe.main()
 assert set(subscribed)<=set(calls)
 if case in ['wrong_partition','request_timeout']:assert not subscribed
