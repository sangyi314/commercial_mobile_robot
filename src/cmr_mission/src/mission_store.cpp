// SPDX-License-Identifier: MIT
#include "cmr_mission/mission_store.hpp"
#include <sqlite3.h>
#include <filesystem>
#include <sys/file.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdexcept>
#include <iostream>
#include <thread>
#include <algorithm>
namespace cmr::mission {
namespace {
struct Db {
 sqlite3* value{};
 ~Db(){if(value)sqlite3_close_v2(value);}
 void sql(const std::string& s){char* err=nullptr;int rc=sqlite3_exec(value,s.c_str(),nullptr,nullptr,&err);std::string message=err?err:"";sqlite3_free(err);if(rc!=SQLITE_OK)throw std::runtime_error("sqlite "+std::to_string(rc)+": "+message);}
};
struct Stmt {
 sqlite3_stmt* value{}; sqlite3* db;
 Stmt(Db& d,const char* q):db(d.value){if(sqlite3_prepare_v2(db,q,-1,&value,nullptr)!=SQLITE_OK)throw std::runtime_error(sqlite3_errmsg(db));}
 ~Stmt(){if(value) {int rc=sqlite3_finalize(value);if(rc!=SQLITE_OK) std::cerr<<"statement finalize: "<<rc<<'\n';}}
 Stmt(const Stmt&)=delete;Stmt& operator=(const Stmt&)=delete;
 void bind(int i,const std::string& s){if(sqlite3_bind_text(value,i,s.c_str(),static_cast<int>(s.size()),SQLITE_TRANSIENT)!=SQLITE_OK)throw std::runtime_error("bind error");}
 bool step(){int rc=sqlite3_step(value);if(rc==SQLITE_ROW)return true;if(rc==SQLITE_DONE)return false;throw std::runtime_error(sqlite3_errmsg(db));}
 std::string text(int i){const auto* c=sqlite3_column_text(value,i);return c?reinterpret_cast<const char*>(c):"";}
};
struct Tx {
 Db& db;bool done{false};explicit Tx(Db& d):db(d){db.sql("BEGIN IMMEDIATE");}
 ~Tx(){if(!done){try{db.sql("ROLLBACK");}catch(const std::exception& e){std::cerr<<"rollback: "<<e.what()<<'\n';}}}
 void commit(){db.sql("COMMIT");done=true;}
};
struct Lock {
 int fd{-1};
 ~Lock(){if(fd>=0)close(fd);}
 void acquire(const std::string& path){fd=open(path.c_str(),O_CREAT|O_RDWR|O_CLOEXEC|O_NOFOLLOW,0600);if(fd<0||flock(fd,LOCK_EX|LOCK_NB)!=0)throw std::runtime_error("mission owner lock unavailable");}
};
}
struct Store::Impl {
 Lock lock;Db db;std::string epoch;Port* port;Hook hook;StoreOptions options;std::thread::id owner=std::this_thread::get_id();bool fault{false};
 void check(){if(owner!=std::this_thread::get_id())throw std::runtime_error("store wrong owner thread");if(fault)throw std::runtime_error("store fault latched");}
 void hit(const std::string& s){if(hook)hook(s);}
 std::optional<Record> get(const std::string& id){Stmt q(db,"SELECT record FROM missions WHERE id=?");q.bind(1,id);if(!q.step())return {};return decode(q.text(0));}
 void save(const Record& r){Stmt q(db,"INSERT INTO missions(id,record,terminal) VALUES(?,?,?) ON CONFLICT(id) DO UPDATE SET record=excluded.record,terminal=excluded.terminal");q.bind(1,r.request.mission_id);q.bind(2,encode(r));q.bind(3,terminal(r.state)?"1":"0");q.step();}
 void audit(const Record& r,const Event& e,const Decision& d){Stmt q(db,"INSERT INTO events(mission,event_id,kind,identity,reason,before_version,after_record) VALUES(?,?,?,?,?,?,?)");q.bind(1,r.request.mission_id);q.bind(2,e.id);q.bind(3,std::to_string(static_cast<int>(e.kind)));q.bind(4,e.epoch+"/"+e.operation+"/"+e.attempt+"/"+e.uuid+" detail="+e.detail);q.bind(5,d.reason);q.bind(6,std::to_string(r.version));q.bind(7,encode(d.next));q.step();}
 void effects(const std::vector<Effect>& effects){for(const auto& e:effects){Stmt q(db,"INSERT INTO operations(id,mission,operation,attempt,uuid,kind,status) VALUES(?,?,?,?,?,?,'INTENT')");q.bind(1,e.id);q.bind(2,e.mission);q.bind(3,e.operation);q.bind(4,e.attempt);q.bind(5,e.uuid);q.bind(6,std::to_string(static_cast<int>(e.kind)));q.step();}}

 void delivery_check(){check();if(!options.delivery)throw std::runtime_error("delivery mode disabled");}
 std::int64_t now(){auto n=options.now();if(n<0)throw std::runtime_error("invalid owner clock");return n;}
 bool paused(){Stmt q(db,"SELECT schedule_paused FROM manager WHERE singleton=1");q.step();return sqlite3_column_int(q.value,0)!=0;}
 bool held(){Stmt q(db,"SELECT active IS NOT NULL FROM manager WHERE singleton=1");q.step();return sqlite3_column_int(q.value,0)!=0;}
 std::optional<Registration> registration(const std::string& id){Stmt q(db,"SELECT request,snapshot,queue_state,queue_seq FROM registrations WHERE mission=?");q.bind(1,id);if(!q.step())return {};return Registration{decode(q.text(0)).request,strict_json(q.text(1)),q.text(2),static_cast<std::uint64_t>(sqlite3_column_int64(q.value,3)),get(id)};}
 void queue_event(const std::string& id,const std::string& kind,const std::string& detail){Stmt q(db,"INSERT INTO queue_events(mission,kind,detail) VALUES(?,?,?)");q.bind(1,id);q.bind(2,kind);Stmt high(db,"SELECT COALESCE(max(seq),0) FROM events");high.step();Json envelope={{"detail",detail},{"owner_epoch",epoch},{"owner_now_ms",now()},{"fsm_high_water_seq",sqlite3_column_int64(high.value,0)}};q.bind(3,envelope.dump());q.step();}
 void activate(const Registration& r){Record n;n.request=r.request;n.epoch=epoch;n.delivery_managed=true;n.deadline=now()+options.task_ms;n.handover_budget=options.handover_ms;save(n);Stmt q(db,"UPDATE manager SET active=? WHERE singleton=1 AND active IS NULL");q.bind(1,n.request.mission_id);q.step();if(sqlite3_changes(db.value)!=1)throw std::runtime_error("active slot race");Stmt u(db,"UPDATE registrations SET queue_state='ACTIVE' WHERE mission=? AND queue_state IN('WAITING','ACTIVE')");u.bind(1,n.request.mission_id);u.step();Event e;e.id="activate";e.mission=n.request.mission_id;e.epoch=epoch;e.now=now();audit(n,e,{n,{},true,"ACTIVATED"});queue_event(n.request.mission_id,"ACTIVATED","owner transaction; no Goal yet");}
 std::optional<DeliveryReply> cached(const std::string& id,const std::string& command,const std::string& content){Stmt q(db,"SELECT content,decision FROM command_decisions WHERE mission=? AND command_id=?");q.bind(1,id);q.bind(2,command);if(!q.step())return {};if(q.text(0)!=content)return DeliveryReply{"COMMAND_CONFLICT",registration(id),{},true};auto j=strict_json(q.text(1));DeliveryReply r;r.code=j.at("code");r.duplicate=true;if(j.contains("record"))r.record=decode(j.at("record"));r.registration=registration(id);return r;}
 void remember(const std::string& id,const std::string& command,const std::string& content,const DeliveryReply& r){Json j={{"code",r.code}};if(r.record)j["record"]=encode(*r.record);Stmt q(db,"INSERT INTO command_decisions(mission,command_id,content,decision) VALUES(?,?,?,?)");q.bind(1,id);q.bind(2,command);q.bind(3,content);q.bind(4,j.dump());q.step();queue_event(id,"COMMAND",Json{{"command_id",command},{"content",content},{"decision",j}}.dump());}
 void emergency() noexcept {fault=true;try{Stmt q(db,"SELECT record FROM missions WHERE id=(SELECT active FROM manager WHERE singleton=1)");if(q.step()&&port){auto r=decode(q.text(0));port->execute({"emergency-revoke",r.request.mission_id,r.operation,r.attempt,r.uuid,EffectKind::Revoke});if(!r.uuid.empty())port->execute({"emergency-cancel",r.request.mission_id,r.operation,r.attempt,r.uuid,EffectKind::CancelExact});}std::cerr<<"STORAGE_FAULT: no new send; emergency actions are NOT persisted\n";}catch(...){std::cerr<<"STORAGE_FAULT: emergency path unavailable\n";}}
};
Store::Store(const std::string& name,const std::string& epoch,Port* port,Hook hook,StoreOptions options):p(std::make_unique<Impl>()) {
 if(!valid_identifier(epoch))throw std::runtime_error("invalid process epoch");
 namespace fs=std::filesystem;fs::path input=fs::absolute(name).lexically_normal();
 if(fs::weakly_canonical(input)!=input || fs::is_symlink(input))throw std::runtime_error("path aliases unsupported");
 if(fs::exists(input)){struct stat st{};if(stat(input.c_str(),&st)||st.st_nlink!=1)throw std::runtime_error("DB hardlinks unsupported");}
 p->lock.acquire(input.string()+".owner.lock");
 if(sqlite3_open_v2(input.c_str(),&p->db.value,SQLITE_OPEN_READWRITE|SQLITE_OPEN_CREATE|SQLITE_OPEN_NOMUTEX,nullptr)!=SQLITE_OK)throw std::runtime_error("open database");
 sqlite3_busy_timeout(p->db.value,100);p->epoch=epoch;p->port=port;p->hook=std::move(hook);p->options=std::move(options);
 if(p->options.delivery&&(!p->options.now||!p->options.eligible||p->options.queue_capacity>64||p->options.task_ms<=0||p->options.task_ms>600000||p->options.handover_ms<=0||p->options.handover_ms>p->options.task_ms))throw std::runtime_error("invalid delivery options");
 int version=0;{Stmt q(p->db,"PRAGMA user_version");q.step();version=sqlite3_column_int(q.value,0);}
 if(version!=0&&version!=1&&version!=2)throw std::runtime_error("unsupported schema");
 if((version==1&&p->options.delivery)||(version==2&&!p->options.delivery))throw std::runtime_error("MIGRATION_REQUIRED: schema/mode mismatch; existing DB not modified");
 if(version==0){Stmt q(p->db,"SELECT count(*) FROM sqlite_master WHERE type='table'");q.step();if(sqlite3_column_int(q.value,0)!=0)throw std::runtime_error("unversioned database");}
 p->db.sql("PRAGMA foreign_keys=ON; PRAGMA journal_mode=WAL; PRAGMA synchronous=FULL;");
 {Stmt q(p->db,"PRAGMA foreign_keys");q.step();if(sqlite3_column_int(q.value,0)!=1)throw std::runtime_error("foreign keys disabled");}
 {Stmt q(p->db,"PRAGMA journal_mode");q.step();if(q.text(0)!="wal")throw std::runtime_error("not WAL");}
 {Stmt q(p->db,"PRAGMA synchronous");q.step();if(sqlite3_column_int(q.value,0)!=2)throw std::runtime_error("not FULL");}
 Tx tx(p->db);
 if(version==0)p->db.sql("CREATE TABLE missions(id TEXT PRIMARY KEY,record TEXT NOT NULL,terminal INTEGER NOT NULL CHECK(terminal IN(0,1))); CREATE TABLE manager(singleton INTEGER PRIMARY KEY CHECK(singleton=1),active TEXT REFERENCES missions(id),epoch TEXT NOT NULL);INSERT INTO manager VALUES(1,NULL,'');CREATE TABLE events(seq INTEGER PRIMARY KEY AUTOINCREMENT,mission TEXT NOT NULL REFERENCES missions(id),event_id TEXT NOT NULL,kind TEXT NOT NULL,identity TEXT NOT NULL,reason TEXT NOT NULL,before_version INTEGER NOT NULL,after_record TEXT NOT NULL,UNIQUE(mission,event_id));CREATE TABLE operations(id TEXT PRIMARY KEY,mission TEXT NOT NULL REFERENCES missions(id),operation TEXT NOT NULL,attempt TEXT NOT NULL,uuid TEXT NOT NULL,kind INTEGER NOT NULL,status TEXT NOT NULL CHECK(status IN('INTENT','DISPATCHING','SENT','UNCERTAIN')));PRAGMA user_version=1;");

 if(p->options.delivery){
  if(version==0){p->db.sql("ALTER TABLE manager ADD COLUMN schedule_paused INTEGER NOT NULL DEFAULT 0 CHECK(schedule_paused IN(0,1));CREATE TABLE registrations(queue_seq INTEGER PRIMARY KEY AUTOINCREMENT,mission TEXT NOT NULL UNIQUE,request TEXT NOT NULL,snapshot TEXT NOT NULL,queue_state TEXT NOT NULL CHECK(queue_state IN('WAITING','ACTIVE','CANCELED','FINISHED')));CREATE TABLE queue_events(seq INTEGER PRIMARY KEY AUTOINCREMENT,mission TEXT NOT NULL,kind TEXT NOT NULL,detail TEXT NOT NULL);CREATE TABLE command_decisions(mission TEXT NOT NULL,command_id TEXT NOT NULL,content TEXT NOT NULL,decision TEXT NOT NULL,PRIMARY KEY(mission,command_id));CREATE TABLE delivery_policy(singleton INTEGER PRIMARY KEY CHECK(singleton=1),capacity INTEGER NOT NULL,task_ms INTEGER NOT NULL,handover_ms INTEGER NOT NULL);PRAGMA user_version=2;");Stmt q(p->db,"INSERT INTO delivery_policy VALUES(1,?,?,?)");q.bind(1,std::to_string(p->options.queue_capacity));q.bind(2,std::to_string(p->options.task_ms));q.bind(3,std::to_string(p->options.handover_ms));q.step();}
  else {Stmt q(p->db,"SELECT capacity,task_ms,handover_ms FROM delivery_policy WHERE singleton=1");if(!q.step()||q.text(0)!=std::to_string(p->options.queue_capacity)||q.text(1)!=std::to_string(p->options.task_ms)||q.text(2)!=std::to_string(p->options.handover_ms))throw std::runtime_error("stored delivery policy mismatch");p->db.sql("UPDATE manager SET schedule_paused=1 WHERE singleton=1");p->queue_event("", "RESTART_PAUSED",epoch);}
 }
 {Stmt q(p->db,"SELECT id FROM missions WHERE terminal=0");std::vector<std::string> ids;while(q.step())ids.push_back(q.text(0));if(ids.size()>1)throw std::runtime_error("multiple active missions");for(const auto& id:ids){auto r=*p->get(id);Event e;e.id="restart-"+epoch;e.mission=id;e.epoch=r.epoch;e.kind=Kind::Restart;auto d=reduce(r,e);d.next.epoch=epoch;p->save(d.next);p->audit(r,e,d);}}
 p->db.sql("UPDATE operations SET status='UNCERTAIN' WHERE status IN('INTENT','DISPATCHING')");
 {Stmt q(p->db,"UPDATE manager SET epoch=? WHERE singleton=1");q.bind(1,epoch);q.step();}tx.commit();
}
Store::~Store()=default;
std::optional<Record> Store::get(const std::string& id){p->check();return p->get(id);}
std::optional<Record> Store::active(){p->check();Stmt q(p->db,"SELECT active FROM manager WHERE singleton=1");if(!q.step()||sqlite3_column_type(q.value,0)==SQLITE_NULL)return std::nullopt;return p->get(q.text(0));}
Admission Store::submit(const MissionRequest& r){p->check();try{Tx tx(p->db);AdmissionContext c;if(auto found=p->get(r.mission_id))c.existing=found->request;{Stmt q(p->db,"SELECT active IS NOT NULL FROM manager");q.step();c.active_slot_held=sqlite3_column_int(q.value,0)!=0;}
 auto a=decide_admission(r,c);if(a==Admission::CreateIntent){Record n;n.request=r;n.epoch=p->epoch;p->save(n);Stmt q(p->db,"UPDATE manager SET active=? WHERE singleton=1 AND active IS NULL");q.bind(1,r.mission_id);q.step();Event e;e.id="submit";e.mission=r.mission_id;e.epoch=p->epoch;p->audit(n,e,{n,{},true,"ACCEPTED"});p->hit("submit-before-commit");}tx.commit();p->hit("submit-after-commit");return a;
 }catch(...){p->emergency();throw;}}
Decision Store::apply(const Event& e){p->check();try{Tx tx(p->db);auto record=p->get(e.mission);if(!record){tx.commit();return {Record{}, {},false,"UNKNOWN_MISSION"};}auto r=*record;
 {Stmt q(p->db,"SELECT kind,identity FROM events WHERE mission=? AND event_id=?");q.bind(1,e.mission);q.bind(2,e.id);if(q.step()){
 const auto identity=q.text(1);const auto mark=identity.find(" detail=");
 const bool conflict=e.detail.rfind("API ",0)==0 && (q.text(0)!=std::to_string(static_cast<int>(e.kind)) || mark==std::string::npos || identity.substr(mark+8)!=e.detail);
 tx.commit();return {r,{},false,conflict?"COMMAND_CONFLICT":"DUPLICATE"};}}
 auto d=reduce(r,e);p->audit(r,e,d);if(d.changed){p->save(d.next);p->effects(d.effects);if(terminal(d.next.state)){Stmt q(p->db,"UPDATE manager SET active=NULL WHERE active=?");q.bind(1,e.mission);q.step();if(p->options.delivery){Stmt u(p->db,"UPDATE registrations SET queue_state='FINISHED' WHERE mission=?");u.bind(1,e.mission);u.step();p->queue_event(e.mission,"FINISHED",state_name(d.next.state));if(d.next.state!=State::Completed)p->db.sql("UPDATE manager SET schedule_paused=1 WHERE singleton=1");}}}
 p->hit(terminal(d.next.state)?"terminal-before-commit":"event-before-commit");tx.commit();p->hit(terminal(d.next.state)?"terminal-after-commit":"event-after-commit");return d;
 }catch(...){p->emergency();throw;}}
void Store::dispatch(){p->check();if(!p->port)return;try{
 std::vector<Effect> effects;{Stmt q(p->db,"SELECT id,mission,operation,attempt,uuid,kind FROM operations WHERE status='INTENT' ORDER BY rowid");while(q.step())effects.push_back({q.text(0),q.text(1),q.text(2),q.text(3),q.text(4),static_cast<EffectKind>(sqlite3_column_int(q.value,5))});}
 for(auto e:effects){
 if(p->options.delivery&&(e.kind==EffectKind::Send||e.kind==EffectKind::SendAuthorized)){auto r=p->registration(e.mission);if(!r)throw std::runtime_error("missing frozen registration");const auto leg=e.operation.find(".return")!=std::string::npos?"return":"outbound";e.navigation_payload=Json{{"target",r->snapshot.at(leg).at("pose")},{"bt",r->snapshot.at("bt")},{"map",r->snapshot.at("map")},{"catalog_sha256",r->snapshot.at("catalog_sha256")}}.dump();}
 if(e.kind==EffectKind::Send||e.kind==EffectKind::SendAuthorized||e.kind==EffectKind::BeginLease){auto r=p->get(e.mission);if(!r || r->state==State::Paused || terminal(r->state) || r->phase!=Phase::SendPending || r->goal_terminal || r->operation!=e.operation || r->attempt!=e.attempt){Tx tx(p->db);Stmt q(p->db,"UPDATE operations SET status='UNCERTAIN' WHERE id=?");q.bind(1,e.id);q.step();tx.commit();continue;}}
 {Tx tx(p->db);Stmt q(p->db,"UPDATE operations SET status='DISPATCHING' WHERE id=? AND status='INTENT'");q.bind(1,e.id);q.step();tx.commit();}
 p->hit("dispatch-before-port");p->port->execute(e);p->hit("dispatch-after-port");
 {Tx tx(p->db);Stmt q(p->db,"UPDATE operations SET status='SENT' WHERE id=?");q.bind(1,e.id);q.step();tx.commit();}}
 }catch(...){p->emergency();throw;}}

std::optional<Registration> Store::registration(const std::string& id){p->delivery_check();return p->registration(id);}
DeliveryReply Store::register_delivery(const MissionRequest& request,const Json& snapshot){p->delivery_check();
 if(validate(request)!=Validation::Valid)return {"INVALID",{},{},false};
 try{if(snapshot.at("map_version")!=request.map_version||snapshot.at("station_version")!=request.station_version||snapshot.at("outbound").at("station_id")!=request.delivery_station||snapshot.at("return").at("station_id")!=request.return_station) return {"INVALID_STATION_VERSION",{},{},false};checked_pose(snapshot.at("outbound").at("pose"));checked_pose(snapshot.at("return").at("pose"));}catch(...){return {"INVALID_SNAPSHOT",{},{},false};}
 try{Tx tx(p->db);DeliveryReply reply;
  if(auto r=p->registration(request.mission_id)){reply={same_business_content(request,r->request)&&snapshot==r->snapshot?"EXISTING":"CONFLICT",r,r->record,true};}
  else if(!p->options.eligible(snapshot))reply.code="INPUT_UNQUALIFIED";
  else{
   std::size_t waiting=0;{Stmt q(p->db,"SELECT count(*) FROM registrations WHERE queue_state='WAITING'");q.step();waiting=static_cast<std::size_t>(sqlite3_column_int64(q.value,0));}
   const bool direct=!p->held()&&!p->paused()&&waiting==0;
   if(!direct&&(p->options.queue_capacity==0||waiting>=p->options.queue_capacity))reply.code=p->options.queue_capacity==0?"BUSY":"QUEUE_FULL";
   else{Record blob;blob.request=request;Stmt q(p->db,"INSERT INTO registrations(mission,request,snapshot,queue_state) VALUES(?,?,?,?)");q.bind(1,request.mission_id);q.bind(2,encode(blob));q.bind(3,snapshot.dump());q.bind(4,direct?"ACTIVE":"WAITING");q.step();auto r=*p->registration(request.mission_id);if(direct)p->activate(r);reply.code=direct?"ACCEPTED":"QUEUED";reply.registration=p->registration(request.mission_id);reply.record=reply.registration->record;}
  }
  p->queue_event(request.mission_id,"SUBMIT",Json{{"code",reply.code},{"duplicate",reply.duplicate}}.dump());p->hit("delivery-submit-before-commit");tx.commit();p->hit("delivery-submit-after-commit");return reply;
 }catch(...){p->emergency();throw;}
}
DeliveryReply Store::promote(){p->delivery_check();try{Tx tx(p->db);DeliveryReply reply;
 if(p->held())reply.code="BUSY";else if(p->paused())reply.code="SCHEDULE_PAUSED";else {Stmt q(p->db,"SELECT mission FROM registrations WHERE queue_state='WAITING' ORDER BY queue_seq LIMIT 1");if(!q.step())reply.code="EMPTY";else{auto r=*p->registration(q.text(0));if(!p->options.eligible(r.snapshot)){p->db.sql("UPDATE manager SET schedule_paused=1 WHERE singleton=1");p->queue_event(r.request.mission_id,"HEAD_INPUT_MISMATCH","original frozen resource required");reply.code="HEAD_INPUT_MISMATCH";}else{p->activate(r);reply.code="PROMOTED";reply.registration=p->registration(r.request.mission_id);reply.record=reply.registration->record;}}}
 p->hit("promote-before-commit");tx.commit();p->hit("promote-after-commit");return reply;
 }catch(...){p->emergency();throw;}}
DeliveryReply Store::schedule(bool resume){p->delivery_check();try{Tx tx(p->db);DeliveryReply reply;
 if(resume&&p->held())reply.code="ACTIVE_UNRESOLVED";else {p->db.sql(std::string("UPDATE manager SET schedule_paused=")+(resume?"0":"1")+" WHERE singleton=1");p->queue_event("",resume?"SCHEDULE_RESUMED":"SCHEDULE_PAUSED","explicit owner command; not Goal/stop evidence");reply.code=resume?"RESUMED":"PAUSED";}
 tx.commit();return reply;}catch(...){p->emergency();throw;}}
Json Store::queue_snapshot(){p->delivery_check();Json out={{"capacity",p->options.queue_capacity},{"schedule_paused",p->paused()},{"orders",Json::array()}};Stmt q(p->db,"SELECT mission,queue_seq,queue_state FROM registrations ORDER BY queue_seq");while(q.step())out["orders"].push_back({{"mission_id",q.text(0)},{"queue_seq",sqlite3_column_int64(q.value,1)},{"queue_state",q.text(2)}});return out;}
DeliveryReply Store::cancel_delivery(const std::string& id,const std::string& command){p->delivery_check();if(!valid_identifier(command))return {"INVALID_COMMAND_ID",{},{},false};const std::string content="cancel";
 try{Tx tx(p->db);if(auto known=p->cached(id,command,content)){p->queue_event(id,"COMMAND_RETRY",known->code);tx.commit();return *known;}
 DeliveryReply reply;auto reg=p->registration(id);reply.registration=reg;
 if(!reg)reply.code="NOT_FOUND";
 else if(reg->queue_state=="WAITING"){Stmt q(p->db,"UPDATE registrations SET queue_state='CANCELED' WHERE mission=? AND queue_state='WAITING'");q.bind(1,id);q.step();p->queue_event(id,"QUEUE_CANCELED",command);reply.code="QUEUE_CANCELED";}
 else if(reg->queue_state=="CANCELED"||reg->queue_state=="FINISHED")reply.code="ALREADY_TERMINAL";
 else{auto r=*reg->record;Event e;e.id="delivery-cancel-"+command;e.mission=id;e.epoch=r.epoch;e.operation=r.operation;e.attempt=r.attempt;e.uuid=r.uuid;e.kind=Kind::Cancel;e.now=p->now();e.detail="delivery API cancel";auto d=reduce(r,e);p->audit(r,e,d);if(d.changed){p->save(d.next);p->effects(d.effects);}reply.code=d.changed||d.reason=="TERMINATION_PENDING"?"CANCEL_PENDING":"REJECTED";reply.record=d.next;p->db.sql("UPDATE manager SET schedule_paused=1 WHERE singleton=1");}
 p->remember(id,command,content,reply);p->hit("delivery-cancel-before-commit");tx.commit();p->hit("delivery-cancel-after-commit");reply.registration=p->registration(id);return reply;
 }catch(...){p->emergency();throw;}}
DeliveryReply Store::confirm_handover(const std::string& id,const std::string& command,const std::string& token,std::uint64_t version,const std::string& source,bool evidence){p->delivery_check();if(!valid_identifier(command)||!valid_identifier(token)||(source!="human"&&source!="synthetic"))return {"INVALID_HANDOVER",{},{},false};
 const std::string content=Json{{"operation","handover"},{"handover_id",token},{"entry_version",version},{"source",source}}.dump();
 try{
  bool expired_at_commit=false;std::int64_t commit_deadline=0;
  {Tx tx(p->db);if(auto known=p->cached(id,command,content)){p->queue_event(id,"COMMAND_RETRY",known->code);tx.commit();return *known;}
   DeliveryReply reply;auto reg=p->registration(id);reply.registration=reg;
   if(!reg||!reg->record)reply.code="NOT_ACTIVE";else{auto r=*reg->record;reply.record=r;
    if(r.handover_id!=token||r.handover_version!=version)reply.code="HANDOVER_IDENTITY";
    else if(!evidence||!p->options.eligible(reg->snapshot))reply.code="EVIDENCE_UNQUALIFIED";
    else{Event e;e.id="handover-"+command;e.mission=id;e.epoch=r.epoch;e.operation=r.operation;e.attempt=r.attempt;e.uuid=r.uuid;e.kind=Kind::Handover;e.expected_version=r.version;e.now=p->now();e.detail=source;auto d=reduce(r,e);reply.code=d.changed?"CONFIRMED":"REJECTED";
     if(d.changed){commit_deadline=std::min({r.deadline,r.handover_until,r.stop_until,r.arrival_until});p->hit("handover-before-commit");if(p->now()>=r.deadline||p->now()>=r.handover_until||p->now()>=r.stop_until||p->now()>=r.arrival_until||!p->options.eligible(reg->snapshot)){expired_at_commit=true;}else{p->audit(r,e,d);p->save(d.next);p->effects(d.effects);reply.record=d.next;}}
    }
   }
   if(!expired_at_commit){p->remember(id,command,content,reply);if(commit_deadline&&p->now()>=commit_deadline)expired_at_commit=true;else{tx.commit();p->hit("handover-after-commit");return reply;}}
  }
  // A late commit guard is a business rejection, not a storage fault. The old
  // transaction has rolled back; persist a rejection, never a return intention.
  Tx tx(p->db);DeliveryReply reply{"HANDOVER_EXPIRED_BEFORE_COMMIT",p->registration(id),p->get(id),false};p->remember(id,command,content,reply);tx.commit();return reply;
 }catch(...){p->emergency();throw;}
}
}
