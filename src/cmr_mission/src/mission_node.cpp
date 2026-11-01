// SPDX-License-Identifier: MIT
#include "cmr_mission/nav2_adapter.hpp"
#include "cmr_mission/execution_port.hpp"
#include "cmr_mission/mission_runtime.hpp"
#include "cmr_mission/action_correlation.hpp"
#include <cmr_interfaces/srv/mission_command.hpp>
#include <cmr_interfaces/srv/synthetic_event.hpp>
#include <csignal>
#include <chrono>
#include <atomic>
#include <iostream>
#include <thread>
#include <set>
#include <map>
namespace cmr::mission {
using Api=cmr_interfaces::srv::MissionCommand;using Fixture=cmr_interfaces::srv::SyntheticEvent;using Clock=std::chrono::steady_clock;
static std::atomic<bool> interrupted{false};
static void signal_handler(int){interrupted=true;}
class MissionNode:public rclcpp::Node {
 Clock::time_point start=Clock::now();std::string epoch;std::unique_ptr<Runtime> runtime;std::unique_ptr<NavigationAdapter> adapter;std::unique_ptr<ExecutionPort> execution;bool gate_mode=false;std::string execution_run,execution_ns;std::optional<std::future<Decision>> health;Clock::time_point health_at{};bool health_fault=false;std::map<std::string,std::uint64_t> stop_generations;
 rclcpp::Service<Api>::SharedPtr api;rclcpp::Service<Fixture>::SharedPtr fixture;rclcpp::TimerBase::SharedPtr timer;
 struct PendingApi{std::shared_ptr<rmw_request_id_t> header;std::future<Api::Response> future;};
 struct PendingFixture{std::shared_ptr<rmw_request_id_t> header;std::future<Fixture::Response> future;};
 std::vector<PendingApi> requests;std::vector<PendingFixture> fixtures;std::vector<std::future<Decision>> events;std::atomic<bool> closing{false};bool shutdown_queued=false;std::size_t mailbox=16;std::uint64_t seq=0;std::int64_t last_ros_clock=-1;
 public:
 MissionNode():Node("mission_node"){}
 std::int64_t now_ms()const{return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now()-start).count();}
 Event ev(const Record& r,Kind kind,std::string id) {Event e;e.kind=kind;e.id=std::move(id);e.mission=r.request.mission_id;e.epoch=r.epoch;e.operation=r.operation;e.attempt=r.attempt;e.uuid=r.uuid;e.now=now_ms();return e;}
 static void fill(Api::Response& response,const Record& r){response.state=state_name(r.state);response.state_version=r.version;response.pending=r.pending;response.operation_id=r.operation;response.attempt_id=r.attempt;response.goal_uuid=r.uuid;response.detail=encode(r);}
 void start_node(){
  const bool test_mode=declare_parameter("test_mode",false);const auto action=declare_parameter<std::string>("action_name","");const auto db=declare_parameter<std::string>("database","");epoch=declare_parameter<std::string>("process_epoch","");mailbox=static_cast<std::size_t>(declare_parameter("mailbox_capacity",16));
  const auto execution_mode=declare_parameter<std::string>("execution_mode","fixture");gate_mode=execution_mode=="gate";execution_run=declare_parameter<std::string>("execution_run_id","");execution_ns=declare_parameter<std::string>("execution_namespace",get_namespace());
  if(execution_mode!="fixture"&&execution_mode!="gate")throw std::runtime_error("invalid execution mode");if(gate_mode&&(!valid_identifier(execution_run)||execution_ns!=get_namespace()))throw std::runtime_error("execution run/namespace mismatch");
  AdapterConfig c;c.synthetic_revoke=!gate_mode;c.action=action;c.epoch=epoch;c.readiness_ms=declare_parameter("readiness_ms",1500);c.accept_ms=declare_parameter("accept_ms",1500);c.execute_ms=declare_parameter("execute_ms",10000);c.cancel_ms=declare_parameter("cancel_ms",1000);c.collect_ms=declare_parameter("collect_ms",3000);
  const std::set<std::string> allowed={"test_mode","action_name","database","process_epoch","mailbox_capacity","readiness_ms","accept_ms","execute_ms","cancel_ms","collect_ms","use_sim_time","execution_mode","execution_run_id","execution_namespace"};
  for(const auto& value:get_node_parameters_interface()->get_parameter_overrides())if(!allowed.count(value.first))throw std::runtime_error("unknown parameter: "+value.first);
  const char* domain=getenv("ROS_DOMAIN_ID");if(!test_mode)throw std::runtime_error("PRODUCTION_REFUSED: physical station/navigation policy not enabled; synthetic fixture prohibited");
  if(!domain||std::string(domain)!="72"||action.rfind("/cmr_test/",0)!=0||db.empty()||!valid_identifier(epoch)||mailbox==0||mailbox>256)throw std::runtime_error("invalid isolated test configuration");
  auto sink=[this](Event e){if(events.size()>=32)return false;try{const auto received=now_ms();events.push_back(runtime->call([this,e,received](Store& s)mutable{e.now=now_ms();if(auto current=s.get(e.mission)){e=scope_callback(std::move(e),*current);if(gate_mode&&e.uuid.empty())e.uuid=current->uuid;}if(e.kind==Kind::Stop&&e.execution_verified){const auto elapsed=std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now().time_since_epoch()-std::chrono::nanoseconds(e.source_received_steady_ns)).count();e.valid_until=e.now+200-elapsed;auto record=s.get(e.mission);if(e.source_received_steady_ns<=0||elapsed<0||elapsed>=200||!record||!execution->evidence_current(*record,e.source_generation))e.source_fresh=false;}auto d=s.apply(e);if(gate_mode&&d.changed&&e.kind==Kind::Stop)stop_generations[e.attempt]=e.source_generation;if(gate_mode&&d.changed&&e.kind==Kind::ExecutionClock)stop_generations.erase(e.attempt);s.dispatch();if(gate_mode){auto r=s.active();if(r&&r->state==State::Paused&&!r->pending.empty()&&r->goal_terminal&&r->revoked&&r->stopped&&e.now<r->stop_until&&stop_generations.count(r->attempt)&&execution->evidence_current(*r,stop_generations[r->attempt])){auto resolve=ev(*r,Kind::Resolve,"execution-resolve-"+epoch+"-"+std::to_string(r->version));resolve.detail="verified A+D and Goal terminal";s.apply(resolve);s.dispatch();}}return d;}));return true;}catch(const std::exception& error){std::cerr<<"EVENT_MAILBOX "<<error.what()<<std::endl;return false;}};
  adapter=std::make_unique<NavigationAdapter>(shared_from_this(),c,sink);
  if(gate_mode)execution=std::make_unique<ExecutionPort>(shared_from_this(),ExecutionConfig{execution_run,epoch,execution_ns,true},*adapter,sink);
  runtime=std::make_unique<Runtime>(db,epoch,mailbox,gate_mode?static_cast<Port*>(execution.get()):static_cast<Port*>(adapter.get()));
  api=create_service<Api>("command",[this](std::shared_ptr<rmw_request_id_t> h,Api::Request::SharedPtr q){
   std::cout<<"BUSINESS_RX seq="<<++seq<<" operation="<<q->operation<<" command="<<q->command_id<<" mission="<<q->mission_id<<std::endl;
   if(closing||requests.size()>=64){Api::Response r;r.code=closing?"CLOSED":"OVERLOADED";api->send_response(*h,r);return;}
   try{requests.push_back({h,runtime->call([this,q](Store& s){Api::Response response;try{
    if(closing){response.code="CLOSED";return response;}
    if(q->operation=="submit"){
     if(!s.get(q->mission_id) && (q->delivery_station!="a"||q->return_station!="b"||q->map_version!="map"||q->station_version!="stations")){response.code="INVALID_STATION_VERSION";return response;}
     MissionRequest request{q->schema_version,q->mission_id,q->delivery_station,q->return_station,q->map_version,q->station_version,q->created_at_unix_ms};const auto decision=s.submit(request);
     static const char* codes[]={"INVALID","INVALID_CONTEXT","EXISTING","CONFLICT","BUSY","ACCEPTED"};response.code=codes[static_cast<int>(decision)];
     if(decision==Admission::CreateIntent){if(gate_mode)stop_generations.clear();auto r=*s.get(q->mission_id);auto e=ev(r,Kind::Prepare,"prepare-"+epoch+"-"+q->mission_id);s.apply(e);s.dispatch();}
    }else if(q->operation=="query")response.code="FOUND";
    else if(q->operation=="cancel"||q->operation=="handover"){
     if(!valid_identifier(q->command_id)){response.code="INVALID_COMMAND_ID";return response;}
     auto record=s.get(q->mission_id);if(!record){response.code="NOT_FOUND";return response;}
     auto e=ev(*record,q->operation=="cancel"?Kind::Cancel:Kind::Handover,"api-"+q->command_id);e.expected_version=q->expected_state_version;e.detail="API "+q->operation+" expected="+std::to_string(q->expected_state_version);
     auto d=s.apply(e);response.code=d.reason=="COMMAND_CONFLICT"?"COMMAND_CONFLICT":terminal(d.next.state)?"ALREADY_TERMINAL":(d.changed||d.reason=="DUPLICATE")?(q->operation=="cancel"?"CANCEL_PENDING":"CONFIRMED"):"REJECTED";s.dispatch();
    }else{response.code="INVALID_OPERATION";return response;}
    if(auto record=s.get(q->mission_id))fill(response,*record);else if(q->operation=="query")response.code="NOT_FOUND";
   }catch(const std::exception& e){response.code="STORAGE_ERROR";response.detail=e.what();}return response;})});}
   catch(const std::exception& e){Api::Response response;response.code="OVERLOADED";response.detail=e.what();api->send_response(*h,response);}
  });
  fixture=create_service<Fixture>("synthetic_event",[this](std::shared_ptr<rmw_request_id_t> h,Fixture::Request::SharedPtr q){
   if(closing||fixtures.size()>=32){Fixture::Response r;r.code="OVERLOADED";fixture->send_response(*h,r);return;}
   try{fixtures.push_back({h,runtime->call([this,q](Store& s){Fixture::Response response;try{
    if(!q->synthetic){response.code="SYNTHETIC_REQUIRED";return response;}
    if(q->kind=="owner_pause"){if(q->valid_for_ms<=0||q->valid_for_ms>1000){response.code="INVALID";return response;}std::cout<<"OWNER_PAUSE_BEGIN "<<q->valid_for_ms<<std::endl;std::this_thread::sleep_for(std::chrono::milliseconds(q->valid_for_ms));std::cout<<"OWNER_PAUSE_END"<<std::endl;response.code="APPLIED";return response;}
    if(gate_mode){response.code="FIXTURE_STOP_PROHIBITED";return response;}
    auto record=s.get(q->mission_id);if(!record){response.code="NOT_FOUND";return response;}
    Kind kind;if(q->kind=="stop")kind=Kind::Stop;else if(q->kind=="arrival")kind=Kind::Arrival;else if(q->kind=="resolve")kind=Kind::Resolve;else if(q->kind=="revoke")kind=Kind::RevokeAck;else if(q->kind=="fatal")kind=Kind::Fatal;else {response.code="INVALID";return response;}
    auto e=ev(*record,kind,q->event_id);e.epoch=q->epoch;e.operation=q->operation_id;e.attempt=q->attempt_id;e.uuid=q->goal_uuid;e.gate_epoch=q->gate_epoch;e.clock_epoch=q->clock_epoch;e.synthetic=true;e.observed_from=e.now-1;e.observed_to=e.now;e.valid_until=e.now+q->valid_for_ms;e.detail="synthetic=true fixture";
    if(e.epoch!=record->epoch){response.code="IDENTITY";return response;}
    if(e.operation!=record->operation||e.attempt!=record->attempt||e.uuid!=record->uuid){response.code="OPERATION";return response;}
    auto d=s.apply(e);response.code=d.changed?"APPLIED":d.reason;response.state=state_name(d.next.state);response.detail=d.reason;s.dispatch();
   }catch(const std::exception& error){response.code="STORAGE_ERROR";response.detail=error.what();}return response;})});}catch(const std::exception& e){Fixture::Response r;r.code="OVERLOADED";r.detail=e.what();fixture->send_response(*h,r);}
  });
  timer=create_wall_timer(std::chrono::milliseconds(5),[this]{poll();});std::cout<<"MISSION_READY epoch="<<epoch<<" synthetic=true"<<std::endl;
 }
 void poll(){
  if(get_clock()->ros_time_is_active()){
   const auto stamp=get_clock()->now().nanoseconds();
   if(stamp!=last_ros_clock){last_ros_clock=stamp;std::cout<<"ROS_CLOCK_NS "<<stamp<<std::endl;}
  }
  if(closing)queue_shutdown();
  if(execution)execution->tick();
  if(gate_mode&&!closing&&!health_fault&&!execution->faulted()){
   if(health&&health->wait_for(std::chrono::seconds(0))==std::future_status::ready){try{health->get();}catch(const std::exception& e){health_fault=true;std::cerr<<"EXECUTION_OWNER_HEALTH_FAULT "<<e.what()<<std::endl;}health.reset();}
   if(!health&&Clock::now()-health_at>=std::chrono::milliseconds(80)){try{health=runtime->call([this](Store& s){auto r=s.active();if(!r)return Decision{};auto e=ev(*r,Kind::ExecutionRenew,"lease-health-"+epoch+"-"+std::to_string(r->version));e.execution_verified=true;e.detail="owner transaction completed; renewal intent";auto d=s.apply(e);s.dispatch();return d;});health_at=Clock::now();}catch(const std::exception& e){std::cerr<<"EXECUTION_OWNER_HEALTH_DEFERRED "<<e.what()<<std::endl;}}
  }
  adapter->tick();
  for(auto it=requests.begin();it!=requests.end();){if(it->future.wait_for(std::chrono::seconds(0))==std::future_status::ready){auto r=it->future.get();api->send_response(*it->header,r);it=requests.erase(it);}else ++it;}
  for(auto it=fixtures.begin();it!=fixtures.end();){if(it->future.wait_for(std::chrono::seconds(0))==std::future_status::ready){auto r=it->future.get();fixture->send_response(*it->header,r);it=fixtures.erase(it);}else ++it;}
  for(auto it=events.begin();it!=events.end();){if(it->wait_for(std::chrono::seconds(0))==std::future_status::ready){try{auto d=it->get();std::cout<<"OWNER_EVENT "<<d.reason<<" state="<<state_name(d.next.state)<<std::endl;}catch(const std::exception& e){std::cerr<<"OWNER_EVENT_ERROR "<<e.what()<<std::endl;}it=events.erase(it);}else ++it;}
 }
 void queue_shutdown(){if(shutdown_queued)return;try{events.push_back(runtime->call([this](Store& s){auto r=s.active();if(!r)return Decision{};auto e=ev(*r,Kind::Cancel,"shutdown-"+epoch);e.detail="shutdown";auto d=s.apply(e);s.dispatch();return d;}));shutdown_queued=true;}catch(const std::exception& e){std::cerr<<"SHUTDOWN_REQUEST_RETRY "<<e.what()<<std::endl;}}
 void begin_shutdown(){closing=true;if(execution)execution->begin_shutdown();adapter->begin_shutdown();queue_shutdown();}
 void finish(){if(execution)execution->stop();adapter->stop();runtime->close();poll();std::cout<<"MISSION_CLOSED requests="<<requests.size()<<" events="<<events.size()<<std::endl;}
};
}
int main(int argc,char** argv){using namespace cmr::mission;rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);std::signal(SIGINT,signal_handler);std::signal(SIGTERM,signal_handler);try{
 auto node=std::make_shared<MissionNode>();node->start_node();rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);
 while(rclcpp::ok()&&!interrupted){executor.spin_some(std::chrono::milliseconds(10));std::this_thread::sleep_for(std::chrono::milliseconds(2));}
 node->begin_shutdown();const auto deadline=Clock::now()+std::chrono::milliseconds(1600);while(rclcpp::ok()&&Clock::now()<deadline){executor.spin_some(std::chrono::milliseconds(10));std::this_thread::sleep_for(std::chrono::milliseconds(2));}
 node->finish();executor.cancel();executor.remove_node(node);node.reset();rclcpp::shutdown();return 0;
 }catch(const std::exception& e){std::cerr<<"MISSION_FATAL "<<e.what()<<std::endl;rclcpp::shutdown();return 2;}}
