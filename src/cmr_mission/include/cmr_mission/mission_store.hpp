// SPDX-License-Identifier: MIT
#pragma once
#include "mission.hpp"
#include <functional>
#include <memory>
namespace cmr::mission {
struct Port { virtual ~Port()=default; virtual void execute(const Effect&)=0; };
class Store {
 struct Impl; std::unique_ptr<Impl> p;
 public:
 using Hook=std::function<void(const std::string&)>;
 Store(const std::string& path,const std::string& epoch,Port* port=nullptr,Hook hook={});
 ~Store(); Store(const Store&)=delete; Store& operator=(const Store&)=delete;
 Admission submit(const MissionRequest&);
 std::optional<Record> get(const std::string&);
 Decision apply(const Event&);
 void dispatch();
};
}
