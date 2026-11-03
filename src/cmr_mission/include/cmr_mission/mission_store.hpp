// SPDX-License-Identifier: MIT
#pragma once
#include "mission.hpp"
#include "delivery.hpp"
#include <functional>
#include <memory>
namespace cmr::mission {
struct Port { virtual ~Port()=default; virtual void execute(const Effect&)=0; };
struct StoreOptions {
 bool delivery{false};std::size_t queue_capacity{4};std::int64_t task_ms{240000},handover_ms{120000};
 std::function<std::int64_t()> now;
 std::function<bool(const Json&)> eligible;
};
struct Registration {MissionRequest request;Json snapshot;std::string queue_state;std::uint64_t queue_seq{};std::optional<Record> record;};
struct DeliveryReply {std::string code;std::optional<Registration> registration;std::optional<Record> record;bool duplicate{false};};
class Store {
 struct Impl; std::unique_ptr<Impl> p;
 public:
 using Hook=std::function<void(const std::string&)>;
 Store(const std::string& path,const std::string& epoch,Port* port=nullptr,Hook hook={},StoreOptions options={});
 ~Store(); Store(const Store&)=delete; Store& operator=(const Store&)=delete;
 Admission submit(const MissionRequest&);
 std::optional<Record> get(const std::string&);
 std::optional<Record> active(); // owner-only persisted slot lookup, including shutdown
 Decision apply(const Event&);
 void dispatch();
 DeliveryReply register_delivery(const MissionRequest&,const Json& snapshot);
 std::optional<Registration> registration(const std::string&);
 DeliveryReply cancel_delivery(const std::string& id,const std::string& command_id);
 DeliveryReply confirm_handover(const std::string& id,const std::string& command_id,const std::string& handover_id,std::uint64_t handover_version,const std::string& source,bool current_evidence);
 DeliveryReply promote();
 DeliveryReply schedule(bool resume);
 Json queue_snapshot();
};
}
