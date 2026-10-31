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
 Lock lock;Db db;std::string epoch;Port* port;Hook hook;std::thread::id owner=std::this_thread::get_id();bool fault{false};
 void check(){if(owner!=std::this_thread::get_id())throw std::runtime_error("store wrong owner thread");if(fault)throw std::runtime_error("store fault latched");}
 void hit(const std::string& s){if(hook)hook(s);}
 std::optional<Record> get(const std::string& id){Stmt q(db,"SELECT record FROM missions WHERE id=?");q.bind(1,id);if(!q.step())return {};return decode(q.text(0));}
 void save(const Record& r){Stmt q(db,"INSERT INTO missions(id,record,terminal) VALUES(?,?,?) ON CONFLICT(id) DO UPDATE SET record=excluded.record,terminal=excluded.terminal");q.bind(1,r.request.mission_id);q.bind(2,encode(r));q.bind(3,terminal(r.state)?"1":"0");q.step();}
 void audit(const Record& r,const Event& e,const Decision& d){Stmt q(db,"INSERT INTO events(mission,event_id,kind,identity,reason,before_version,after_record) VALUES(?,?,?,?,?,?,?)");q.bind(1,r.request.mission_id);q.bind(2,e.id);q.bind(3,std::to_string(static_cast<int>(e.kind)));q.bind(4,e.epoch+"/"+e.operation+"/"+e.attempt+"/"+e.uuid);q.bind(5,d.reason);q.bind(6,std::to_string(r.version));q.bind(7,encode(d.next));q.step();}
 void effects(const std::vector<Effect>& effects){for(const auto& e:effects){Stmt q(db,"INSERT INTO operations(id,mission,operation,attempt,uuid,kind,status) VALUES(?,?,?,?,?,?,'INTENT')");q.bind(1,e.id);q.bind(2,e.mission);q.bind(3,e.operation);q.bind(4,e.attempt);q.bind(5,e.uuid);q.bind(6,std::to_string(static_cast<int>(e.kind)));q.step();}}
 void emergency() noexcept {fault=true;try{Stmt q(db,"SELECT record FROM missions WHERE id=(SELECT active FROM manager WHERE singleton=1)");if(q.step()&&port){auto r=decode(q.text(0));port->execute({"emergency-revoke",r.request.mission_id,r.operation,r.attempt,r.uuid,EffectKind::Revoke});if(!r.uuid.empty())port->execute({"emergency-cancel",r.request.mission_id,r.operation,r.attempt,r.uuid,EffectKind::CancelExact});}std::cerr<<"STORAGE_FAULT: no new send; emergency actions are NOT persisted\n";}catch(...){std::cerr<<"STORAGE_FAULT: emergency path unavailable\n";}}
};
Store::Store(const std::string& name,const std::string& epoch,Port* port,Hook hook):p(std::make_unique<Impl>()) {
 if(!valid_identifier(epoch))throw std::runtime_error("invalid process epoch");
 namespace fs=std::filesystem;fs::path input=fs::absolute(name).lexically_normal();
 if(fs::weakly_canonical(input)!=input || fs::is_symlink(input))throw std::runtime_error("path aliases unsupported");
 if(fs::exists(input)){struct stat st{};if(stat(input.c_str(),&st)||st.st_nlink!=1)throw std::runtime_error("DB hardlinks unsupported");}
 p->lock.acquire(input.string()+".owner.lock");
 if(sqlite3_open_v2(input.c_str(),&p->db.value,SQLITE_OPEN_READWRITE|SQLITE_OPEN_CREATE|SQLITE_OPEN_NOMUTEX,nullptr)!=SQLITE_OK)throw std::runtime_error("open database");
 sqlite3_busy_timeout(p->db.value,100);p->epoch=epoch;p->port=port;p->hook=std::move(hook);
 int version=0;{Stmt q(p->db,"PRAGMA user_version");q.step();version=sqlite3_column_int(q.value,0);}
 if(version!=0 && version!=1)throw std::runtime_error("unsupported schema");
 if(version==0){Stmt q(p->db,"SELECT count(*) FROM sqlite_master WHERE type='table'");q.step();if(sqlite3_column_int(q.value,0)!=0)throw std::runtime_error("unversioned database");}
 p->db.sql("PRAGMA foreign_keys=ON; PRAGMA journal_mode=WAL; PRAGMA synchronous=FULL;");
 {Stmt q(p->db,"PRAGMA foreign_keys");q.step();if(sqlite3_column_int(q.value,0)!=1)throw std::runtime_error("foreign keys disabled");}
 {Stmt q(p->db,"PRAGMA journal_mode");q.step();if(q.text(0)!="wal")throw std::runtime_error("not WAL");}
 {Stmt q(p->db,"PRAGMA synchronous");q.step();if(sqlite3_column_int(q.value,0)!=2)throw std::runtime_error("not FULL");}
 Tx tx(p->db);
 if(version==0)p->db.sql("CREATE TABLE missions(id TEXT PRIMARY KEY,record TEXT NOT NULL,terminal INTEGER NOT NULL CHECK(terminal IN(0,1))); CREATE TABLE manager(singleton INTEGER PRIMARY KEY CHECK(singleton=1),active TEXT REFERENCES missions(id),epoch TEXT NOT NULL);INSERT INTO manager VALUES(1,NULL,'');CREATE TABLE events(seq INTEGER PRIMARY KEY AUTOINCREMENT,mission TEXT NOT NULL REFERENCES missions(id),event_id TEXT NOT NULL,kind TEXT NOT NULL,identity TEXT NOT NULL,reason TEXT NOT NULL,before_version INTEGER NOT NULL,after_record TEXT NOT NULL,UNIQUE(mission,event_id));CREATE TABLE operations(id TEXT PRIMARY KEY,mission TEXT NOT NULL REFERENCES missions(id),operation TEXT NOT NULL,attempt TEXT NOT NULL,uuid TEXT NOT NULL,kind INTEGER NOT NULL,status TEXT NOT NULL CHECK(status IN('INTENT','DISPATCHING','SENT','UNCERTAIN')));PRAGMA user_version=1;");
 {Stmt q(p->db,"SELECT id FROM missions WHERE terminal=0");std::vector<std::string> ids;while(q.step())ids.push_back(q.text(0));if(ids.size()>1)throw std::runtime_error("multiple active missions");for(const auto& id:ids){auto r=*p->get(id);Event e;e.id="restart-"+epoch;e.mission=id;e.epoch=r.epoch;e.kind=Kind::Restart;auto d=reduce(r,e);d.next.epoch=epoch;p->save(d.next);p->audit(r,e,d);}}
 p->db.sql("UPDATE operations SET status='UNCERTAIN' WHERE status IN('INTENT','DISPATCHING')");
 {Stmt q(p->db,"UPDATE manager SET epoch=? WHERE singleton=1");q.bind(1,epoch);q.step();}tx.commit();
}
Store::~Store()=default;
std::optional<Record> Store::get(const std::string& id){p->check();return p->get(id);}
Admission Store::submit(const MissionRequest& r){p->check();try{Tx tx(p->db);AdmissionContext c;if(auto found=p->get(r.mission_id))c.existing=found->request;{Stmt q(p->db,"SELECT active IS NOT NULL FROM manager");q.step();c.active_slot_held=sqlite3_column_int(q.value,0)!=0;}
 auto a=decide_admission(r,c);if(a==Admission::CreateIntent){Record n;n.request=r;n.epoch=p->epoch;p->save(n);Stmt q(p->db,"UPDATE manager SET active=? WHERE singleton=1 AND active IS NULL");q.bind(1,r.mission_id);q.step();Event e;e.id="submit";e.mission=r.mission_id;e.epoch=p->epoch;p->audit(n,e,{n,{},true,"ACCEPTED"});p->hit("submit-before-commit");}tx.commit();p->hit("submit-after-commit");return a;
 }catch(...){p->emergency();throw;}}
Decision Store::apply(const Event& e){p->check();try{Tx tx(p->db);auto record=p->get(e.mission);if(!record){tx.commit();return {Record{}, {},false,"UNKNOWN_MISSION"};}auto r=*record;
 {Stmt q(p->db,"SELECT 1 FROM events WHERE mission=? AND event_id=?");q.bind(1,e.mission);q.bind(2,e.id);if(q.step()){tx.commit();return {r,{},false,"DUPLICATE"};}}
 auto d=reduce(r,e);p->audit(r,e,d);if(d.changed){p->save(d.next);p->effects(d.effects);if(terminal(d.next.state)){Stmt q(p->db,"UPDATE manager SET active=NULL WHERE active=?");q.bind(1,e.mission);q.step();}}
 p->hit(terminal(d.next.state)?"terminal-before-commit":"event-before-commit");tx.commit();p->hit(terminal(d.next.state)?"terminal-after-commit":"event-after-commit");return d;
 }catch(...){p->emergency();throw;}}
void Store::dispatch(){p->check();if(!p->port)return;try{
 std::vector<Effect> effects;{Stmt q(p->db,"SELECT id,mission,operation,attempt,uuid,kind FROM operations WHERE status='INTENT' ORDER BY rowid");while(q.step())effects.push_back({q.text(0),q.text(1),q.text(2),q.text(3),q.text(4),static_cast<EffectKind>(sqlite3_column_int(q.value,5))});}
 for(const auto& e:effects){if(e.kind==EffectKind::Send){auto r=p->get(e.mission);if(!r || r->state==State::Paused || terminal(r->state) || r->phase!=Phase::SendPending || r->goal_terminal || r->operation!=e.operation || r->attempt!=e.attempt){Tx tx(p->db);Stmt q(p->db,"UPDATE operations SET status='UNCERTAIN' WHERE id=?");q.bind(1,e.id);q.step();tx.commit();continue;}}
 {Tx tx(p->db);Stmt q(p->db,"UPDATE operations SET status='DISPATCHING' WHERE id=? AND status='INTENT'");q.bind(1,e.id);q.step();tx.commit();}
 p->hit("dispatch-before-port");p->port->execute(e);p->hit("dispatch-after-port");
 {Tx tx(p->db);Stmt q(p->db,"UPDATE operations SET status='SENT' WHERE id=?");q.bind(1,e.id);q.step();tx.commit();}}
 }catch(...){p->emergency();throw;}}
}
