# SPDX-License-Identifier: MIT
from tools.delivery.records import expected_business
def db(source='synthetic'):
 return {'records':{'m':{'state':'COMPLETED','handed_over':True,'handover_source':source}},'authorized_send_intents':[{},{}],'waiting':[],'active_slot':None}
def test_completion_requires_the_exact_reviewed_handover_source():
 assert expected_business('G02',db())
 assert not expected_business('G02',db('source=synthetic handover_id=m.h.1'))
 assert not expected_business('G01',db())
 assert expected_business('G01',db('human'))
def test_business_success_never_hides_unresolved_queue_or_slot():
 sample=db();sample['waiting']=['q'];assert not expected_business('G02',sample)
 sample=db();sample['active_slot']='m';assert not expected_business('G02',sample)
 sample=db();sample['records']['m']['handed_over']=False;assert not expected_business('G02',sample)
