// SPDX-License-Identifier: MIT
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <cmr_interfaces/srv/fake_control.hpp>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <map>
#include <iostream>
#include <iomanip>
#include <sstream>
using Action=nav2_msgs::action::NavigateToPose;using Handle=rclcpp_action::ServerGoalHandle<Action>;using Control=cmr_interfaces::srv::FakeControl;using Clock=std::chrono::steady_clock;
std::string uuid(const rclcpp_action::GoalUUID& id){std::ostringstream s;for(auto c:id)s<<std::hex<<std::setw(2)<<std::setfill('0')<<static_cast<int>(c);return s.str();}
class Fake:public rclcpp::Node{
 std::mutex mutex;std::condition_variable cv;bool release=false,reject=false,reject_cancel=false;int delay=0,cancel_delay=200;std::size_t requests=0,peak=0;
 struct Goal{std::shared_ptr<Handle> handle;bool done=false;Clock::time_point cancel{};};std::map<std::string,Goal> goals;
 rclcpp::CallbackGroup::SharedPtr action_group;rclcpp_action::Server<Action>::SharedPtr action;rclcpp::Service<Control>::SharedPtr control;rclcpp::TimerBase::SharedPtr timer;
 public:
 Fake():Node("fake_nav2_server"){
  auto name=declare_parameter<std::string>("action_name","");if(name.rfind("/cmr_test/",0)!=0)throw std::runtime_error("test action required");
  action_group=create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  action=rclcpp_action::create_server<Action>(get_node_base_interface(),get_node_clock_interface(),get_node_logging_interface(),get_node_waitables_interface(),name,
   [this](const rclcpp_action::GoalUUID& id,std::shared_ptr<const Action::Goal> goal){std::unique_lock<std::mutex> lock(mutex);++requests;std::cout<<"GOAL_REQUEST uuid="<<uuid(id)<<" count="<<requests<<" x="<<goal->pose.pose.position.x<<" frame="<<goal->pose.header.frame_id<<std::endl;
    const bool reject_this=reject;const int wait=delay;release=false;if(wait>0){std::cout<<"ACCEPT_BARRIER uuid="<<uuid(id)<<std::endl;cv.wait_for(lock,std::chrono::milliseconds(wait),[this]{return release;});}
    std::cout<<(reject_this?"REJECT ":"ACCEPT ")<<uuid(id)<<std::endl;return reject_this?rclcpp_action::GoalResponse::REJECT:rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
   [this](std::shared_ptr<Handle> handle){std::lock_guard<std::mutex> lock(mutex);const auto id=uuid(handle->get_goal_id());std::cout<<"CANCEL_REQUEST uuid="<<id<<" reject="<<reject_cancel<<std::endl;if(reject_cancel)return rclcpp_action::CancelResponse::REJECT;goals[id].cancel=Clock::now();return rclcpp_action::CancelResponse::ACCEPT;},
   [this](std::shared_ptr<Handle> handle){std::lock_guard<std::mutex> lock(mutex);goals[uuid(handle->get_goal_id())].handle=handle;std::size_t active=0;for(auto& x:goals)active+=!x.second.done;peak=std::max(peak,active);std::cout<<"ACTIVE peak="<<peak<<std::endl;},rcl_action_server_get_default_options(),action_group);
  control=create_service<Control>("control",[this](Control::Request::SharedPtr q,Control::Response::SharedPtr response){std::lock_guard<std::mutex> lock(mutex);response->success=true;std::cout<<"CONTROL case="<<q->case_id<<" op="<<q->command<<" uuid="<<q->goal_uuid<<std::endl;
   if(q->command=="reject")reject=q->value_ms!=0;
   else if(q->command=="reject_cancel")reject_cancel=q->value_ms!=0;
   else if(q->command=="delay_accept"){if(q->value_ms<0||q->value_ms>5000){response->success=false;return;}delay=static_cast<int>(q->value_ms);}
   else if(q->command=="cancel_delay"){if(q->value_ms<0||q->value_ms>5000){response->success=false;return;}cancel_delay=static_cast<int>(q->value_ms);}
   else if(q->command=="release_accept"){release=true;cv.notify_all();}
   else if(q->command=="success"||q->command=="abort"){
    bool hit=false;for(auto& pair:goals){auto& g=pair.second;if(g.done||!g.handle||(!q->goal_uuid.empty()&&pair.first!=q->goal_uuid))continue;auto result=std::make_shared<Action::Result>();if(q->command=="success")g.handle->succeed(result);else g.handle->abort(result);g.done=true;hit=true;std::cout<<"RESULT uuid="<<pair.first<<" kind="<<q->command<<std::endl;}response->success=hit;
   }else if(q->command!="stats")response->success=false;
   response->detail="requests="+std::to_string(requests)+" peak="+std::to_string(peak);});
  timer=create_wall_timer(std::chrono::milliseconds(30),[this]{std::lock_guard<std::mutex> lock(mutex);for(auto& pair:goals){auto& g=pair.second;if(g.done||!g.handle)continue;if(g.handle->is_canceling()&&g.cancel!=Clock::time_point{}&&Clock::now()-g.cancel>=std::chrono::milliseconds(cancel_delay)){g.handle->canceled(std::make_shared<Action::Result>());g.done=true;std::cout<<"RESULT uuid="<<pair.first<<" kind=canceled"<<std::endl;}else if(g.handle->is_executing()){auto f=std::make_shared<Action::Feedback>();f->distance_remaining=1.0;g.handle->publish_feedback(f);std::cout<<"FEEDBACK uuid="<<pair.first<<std::endl;}}});
  std::cout<<"FAKE_READY"<<std::endl;
 }
};
int main(int argc,char** argv){rclcpp::init(argc,argv);try{auto node=std::make_shared<Fake>();rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),3);executor.add_node(node);executor.spin();executor.remove_node(node);node.reset();rclcpp::shutdown();return 0;}catch(const std::exception& e){std::cerr<<"FAKE_ERROR "<<e.what()<<std::endl;rclcpp::shutdown();return 2;}}
