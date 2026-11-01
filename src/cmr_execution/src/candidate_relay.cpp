// SPDX-License-Identifier: MIT
// Transport-only adapter for a controlled candidate-stream fault. No restamping,
// velocity changes, authorization, Action clients, or final-output publishing.
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <atomic>
#include <csignal>
#include <thread>
static std::atomic<bool> running{true},forwarding{true};
static void signal_handler(int sig){if(sig==SIGUSR1)forwarding=false;else if(sig==SIGUSR2)forwarding=true;else running=false;}
int main(int argc,char** argv){rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);for(int sig:{SIGINT,SIGTERM,SIGUSR1,SIGUSR2})std::signal(sig,signal_handler);auto node=std::make_shared<rclcpp::Node>("candidate_relay");auto qos=rclcpp::QoS(1).reliable().durability_volatile();auto pub=node->create_publisher<geometry_msgs::msg::TwistStamped>("candidate",qos);auto sub=node->create_subscription<geometry_msgs::msg::TwistStamped>("candidate_raw",qos,[pub](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg){if(forwarding)pub->publish(*msg);});rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);while(rclcpp::ok()&&running){executor.spin_some(std::chrono::milliseconds(5));std::this_thread::sleep_for(std::chrono::milliseconds(1));}executor.cancel();executor.remove_node(node);sub.reset();pub.reset();node.reset();rclcpp::shutdown();return 0;}
