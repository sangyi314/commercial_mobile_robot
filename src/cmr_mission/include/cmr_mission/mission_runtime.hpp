// SPDX-License-Identifier: MIT
#pragma once
#include "mission_store.hpp"
#include <condition_variable>
#include <deque>
#include <future>
#include <mutex>
#include <thread>
namespace cmr::mission {
class Runtime {
 std::mutex mutex;std::condition_variable cv;std::deque<std::function<void(Store&)>> queue;
 bool closing{false};std::size_t capacity;std::thread worker;
 public:
 Runtime(std::string path,std::string epoch,std::size_t limit=16,Port* port=nullptr,Store::Hook hook={}):capacity(limit){
  if(!limit)throw std::invalid_argument("zero mailbox capacity");
  std::promise<void> ready;auto wait=ready.get_future();
  worker=std::thread([this,path,epoch,port,hook,ready=std::move(ready)]()mutable{
   std::unique_ptr<Store> store;
   try{store=std::make_unique<Store>(path,epoch,port,hook);ready.set_value();}catch(...){ready.set_exception(std::current_exception());return;}
   for(;;){std::function<void(Store&)> task;{std::unique_lock<std::mutex> lock(mutex);cv.wait(lock,[this]{return closing||!queue.empty();});if(queue.empty()&&closing)return;task=std::move(queue.front());queue.pop_front();}task(*store);}
  });
  try{wait.get();}catch(...){worker.join();throw;}
 }
 ~Runtime(){close();}
 Runtime(const Runtime&)=delete;Runtime& operator=(const Runtime&)=delete;
 void close(){ {std::lock_guard<std::mutex> lock(mutex);closing=true;}cv.notify_all();if(worker.joinable())worker.join();}
 template<class F> auto call(F f)->std::future<decltype(f(std::declval<Store&>()))>{
  using R=decltype(f(std::declval<Store&>()));auto task=std::make_shared<std::packaged_task<R(Store&)>>(std::move(f));auto result=task->get_future();
  {std::lock_guard<std::mutex> lock(mutex);if(closing)throw std::runtime_error("CLOSED");if(queue.size()>=capacity)throw std::runtime_error("OVERLOADED");queue.push_back([task](Store& s){(*task)(s);});}cv.notify_one();return result;
 }
};
}
