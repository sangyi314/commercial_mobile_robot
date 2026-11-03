#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Versioned Mission service client. It never creates a navigation Action client."""
import argparse,json,os,time,uuid
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--service',required=True);p.add_argument('--check',action='store_true');p.add_argument('operation',choices=['submit','query','queue','cancel','handover','pause-schedule','resume-schedule']);p.add_argument('--mission-id',default='');p.add_argument('--command-id');p.add_argument('--created-at-unix-ms',type=int,help='reuse the original value when retrying Submit');p.add_argument('--delivery-station',default='delivery');p.add_argument('--return-station',default='home');p.add_argument('--map-version',default='');p.add_argument('--station-version',default='');p.add_argument('--handover-id',default='');p.add_argument('--entry-version',type=int,default=0);p.add_argument('--source',choices=['human','synthetic'],default='human');a=p.parse_args()
 if not a.service.startswith(('/cmr_execution/','/cmr_test/')) or not a.service.endswith('/delivery'):p.error('explicit scoped delivery service required')
 if a.operation in ['submit','query','cancel','handover'] and not a.mission_id:p.error('mission-id required')
 if a.operation=='submit' and (not a.map_version or not a.station_version):p.error('explicit map/station versions required')
 if a.operation=='handover' and (not a.handover_id or a.entry_version<=0):p.error('explicit handover identity and entry version required')
 if a.check:print('CHECK_OK no ROS/DB/process/filesystem write');return 0
 if os.environ.get('ROS_DOMAIN_ID') not in ['71','72']:p.error('explicit reviewed ROS_DOMAIN_ID=71/72 required')
 import rclpy
 from cmr_interfaces.srv import DeliveryCommand
 from rosidl_runtime_py.convert import message_to_ordereddict as dic
 rclpy.init();node=rclpy.create_node('cmr_delivery_operator');client=node.create_client(DeliveryCommand,a.service)
 try:
  if not client.wait_for_service(timeout_sec=5):raise TimeoutError('Mission service unavailable')
  q=DeliveryCommand.Request();q.operation=a.operation.replace('-','_');q.mission_id=a.mission_id;q.command_id=a.command_id or 'operator_'+uuid.uuid4().hex;q.schema_version=1;q.delivery_station=a.delivery_station;q.return_station=a.return_station;q.map_version=a.map_version;q.station_version=a.station_version;q.created_at_unix_ms=a.created_at_unix_ms if a.created_at_unix_ms is not None else int(time.time()*1000);q.handover_id=a.handover_id;q.handover_entry_version=a.entry_version;q.source=a.source
  f=client.call_async(q);rclpy.spin_until_future_complete(node,f,timeout_sec=5)
  if not f.done():client.remove_pending_request(f);raise TimeoutError('response unknown; query/retry same command ID; never infer rollback')
  print(json.dumps({'service':a.service,'request':dic(q),'response':dic(f.result())},ensure_ascii=False))
  return 0 if f.result().code not in ['STORAGE_ERROR','INVALID_OPERATION','CLOSED','OVERLOADED'] else 3
 finally:node.destroy_client(client);node.destroy_node();rclpy.shutdown()
if __name__=='__main__':raise SystemExit(main())
