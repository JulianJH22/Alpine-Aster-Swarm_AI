#include "swarm/gateway.hpp"
#include <json-c/json.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <utility>
#include <cstring>

namespace swarm {
namespace {
struct Json {
 json_object* p;
 explicit Json(json_object* value):p(value) { if (!p) throw std::runtime_error("Invalid JSON"); }
 ~Json(){json_object_put(p);}
 Json(const Json&)=delete;
};
json_object* field(json_object* p,const char* key) {
 json_object* v=nullptr;
 if (!p || !json_object_object_get_ex(p,key,&v)) throw std::runtime_error(std::string("Missing JSON field: ")+key);
 return v;
}
std::string str(json_object* p){return json_object_get_string(p);}
std::string quote(const std::string& s) {
 Json v(json_object_new_string(s.c_str()));
 return json_object_to_json_string_ext(v.p,JSON_C_TO_STRING_PLAIN);
}
std::string maneuver_name(ManeuverType t){
 switch(t){case ManeuverType::Inspect:return "Inspect";case ManeuverType::Grasp:return "Grasp";case ManeuverType::Move:return "Move";case ManeuverType::Hold:return "Hold";case ManeuverType::Release:return "Release";case ManeuverType::Sort:return "Sort";}
 throw std::runtime_error("Unknown maneuver");
}
ManeuverType maneuver_type(const std::string& s){
 for(auto t:{ManeuverType::Inspect,ManeuverType::Grasp,ManeuverType::Move,ManeuverType::Hold,ManeuverType::Release,ManeuverType::Sort}) if(maneuver_name(t)==s)return t;
 throw std::runtime_error("Unknown remote maneuver: "+s);
}
ExecutionStatus status(const std::string& s){
 if(s=="accepted" || s=="running")return ExecutionStatus::Accepted;
 if(s=="completed")return ExecutionStatus::Completed;
 if(s=="failed" || s=="cancelled")return ExecutionStatus::Failed;
 return ExecutionStatus::Rejected;
}
Pose pose_from(json_object* p){
 Pose pose;
 auto* pos=field(p,"position");
 if(json_object_array_length(pos)<3)throw std::runtime_error("Invalid position");
 pose.x=json_object_get_double(json_object_array_get_idx(pos,0));
 pose.y=json_object_get_double(json_object_array_get_idx(pos,1));
 pose.z=json_object_get_double(json_object_array_get_idx(pos,2));
 return pose;
}
}
GatewayClient::GatewayClient(std::string host,int port,int timeout_ms){
 if(port<1 || port>65535 || timeout_ms<=0)throw std::invalid_argument("Invalid gateway settings");
 addrinfo hints{}; hints.ai_family=AF_UNSPEC; hints.ai_socktype=SOCK_STREAM;
 addrinfo* addresses=nullptr;
 int rc=getaddrinfo(host.c_str(),std::to_string(port).c_str(),&hints,&addresses);
 if(rc)throw std::runtime_error(gai_strerror(rc));
 for(auto* a=addresses;a;a=a->ai_next){
  int fd=::socket(a->ai_family,a->ai_socktype,a->ai_protocol);
  if(fd<0)continue;
  timeval tv{timeout_ms/1000,(timeout_ms%1000)*1000};
  setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,&tv,sizeof(tv));
  setsockopt(fd,SOL_SOCKET,SO_SNDTIMEO,&tv,sizeof(tv));
  if(::connect(fd,a->ai_addr,a->ai_addrlen)==0){socket_=fd;break;}
  ::close(fd);
 }
 freeaddrinfo(addresses);
 if(socket_<0)throw std::runtime_error("Cannot connect to simulator gateway");
 // Fail fast on incompatible protocol.
 Json hello(json_tokener_parse(request("hello").c_str()));
 if(str(field(hello.p,"protocol"))!="swarm-sim/0.1")throw std::runtime_error("Incompatible gateway protocol");
}
GatewayClient::~GatewayClient(){if(socket_>=0)::close(socket_);}
std::string GatewayClient::request(const std::string& operation,const std::string& json_args){
 const auto id=++next_id_;
 std::string msg="{\"id\":"+std::to_string(id)+",\"op\":"+quote(operation)+",\"args\":"+json_args+"}\n";
 size_t sent=0;
 while(sent<msg.size()){
  ssize_t n=::send(socket_,msg.data()+sent,msg.size()-sent,MSG_NOSIGNAL);
  if(n<=0)throw std::runtime_error("Gateway send failed");
  sent+=static_cast<size_t>(n);
 }
 while(buffer_.find('\n')==std::string::npos){
  char b[4096];auto n=::recv(socket_,b,sizeof(b),0);
  if(n<=0)throw std::runtime_error("Gateway disconnected or timed out");
  buffer_.append(b,static_cast<size_t>(n));
  if(buffer_.size()>1024*1024)throw std::runtime_error("Gateway response too large");
 }
 auto cut=buffer_.find('\n');auto line=buffer_.substr(0,cut);buffer_.erase(0,cut+1);
 Json resp(json_tokener_parse(line.c_str()));
 if(json_object_get_int64(field(resp.p,"id"))!=static_cast<int64_t>(id))throw std::runtime_error("Gateway response ID mismatch");
 if(!json_object_get_boolean(field(resp.p,"ok")))throw std::runtime_error("Gateway: "+str(field(resp.p,"error")));
 return json_object_to_json_string_ext(field(resp.p,"result"),JSON_C_TO_STRING_PLAIN);
}
double GatewayClient::step(double seconds){
 if(!std::isfinite(seconds) || seconds<=0)throw std::invalid_argument("Invalid step duration");
 Json j(json_tokener_parse(request("step","{\"seconds\":"+std::to_string(seconds)+"}").c_str()));
 return json_object_get_double(field(j.p,"sim_time"));
}
bool GatewayClient::run_until_idle(double timeout_seconds){
 if(!std::isfinite(timeout_seconds) || timeout_seconds<=0)throw std::invalid_argument("Invalid timeout");
 Json j(json_tokener_parse(request("run_until_idle","{\"timeout_s\":"+std::to_string(timeout_seconds)+"}").c_str()));
 return json_object_get_boolean(field(j.p,"idle"));
}
GatewayRobotArm::GatewayRobotArm(std::string id,std::shared_ptr<GatewayClient> client):id_(std::move(id)),client_(std::move(client)){
 if(id_.empty() || !client_)throw std::invalid_argument("Missing gateway arm configuration");
}
RobotState GatewayRobotArm::state() const{
 Json j(json_tokener_parse(client_->request("state","{\"robot_id\":"+quote(id_)+"}").c_str()));
 auto* tcp=field(j.p,"tcp");
 return {str(field(j.p,"id")),pose_from(tcp),json_object_get_boolean(field(j.p,"available")) && json_object_get_boolean(field(j.p,"valid")),str(field(j.p,"status"))};
}
RobotCapabilities GatewayRobotArm::capabilities() const{
 Json j(json_tokener_parse(client_->request("capabilities","{\"robot_id\":"+quote(id_)+"}").c_str()));
 RobotCapabilities caps;
 auto* a=field(j.p,"maneuvers");
 for(size_t i=0;i<json_object_array_length(a);++i)caps.maneuvers.push_back(maneuver_type(str(json_object_array_get_idx(a,i))));
 return caps;
}
ManeuverResult GatewayRobotArm::execute(const ManeuverCommand& c){
 if(c.task_id.empty())return {c.task_id,ExecutionStatus::Rejected,"Missing task ID"};
 // Non-blocking submit: Accepted is NOT completion. The simulator clock is owned by the caller.
 const std::string execution_id=id_+":"+c.task_id;
 const auto& p=c.target;
 // ZYX Euler -> xyzw quaternion. Identity orientation is explicit, not "unspecified".
 const double cr=std::cos(p.roll/2),sr=std::sin(p.roll/2);
 const double cp=std::cos(p.pitch/2),sp=std::sin(p.pitch/2);
 const double cy=std::cos(p.yaw/2),sy=std::sin(p.yaw/2);
 const double qx=sr*cp*cy-cr*sp*sy;
 const double qy=cr*sp*cy+sr*cp*sy;
 const double qz=cr*cp*sy-sr*sp*cy;
 const double qw=cr*cp*cy+sr*sp*sy;
 const std::string target="{\"position\":["+std::to_string(p.x)+","+std::to_string(p.y)+","+std::to_string(p.z)+"],\"orientation\":["+std::to_string(qx)+","+std::to_string(qy)+","+std::to_string(qz)+","+std::to_string(qw)+"],\"frame\":\"world\"}";
 const std::string cmd="{\"execution_id\":"+quote(execution_id)+",\"task_id\":"+quote(c.task_id)+",\"robot_id\":"+quote(id_)+",\"type\":"+quote(maneuver_name(c.type))+",\"object_id\":"+quote(c.object_id)+",\"target\":"+target+"}";
 try {
  Json j(json_tokener_parse(client_->request("submit","{\"command\":"+cmd+"}").c_str()));
  return {c.task_id,status(str(field(j.p,"status"))),str(field(j.p,"detail"))};
 }catch(const std::exception& e){return {c.task_id,ExecutionStatus::Failed,e.what()};}
}
ManeuverResult GatewayRobotArm::poll(const std::string& task_id) const{
 try {
  Json j(json_tokener_parse(client_->request("status","{\"robot_id\":"+quote(id_)+",\"execution_id\":"+quote(id_+":"+task_id)+"}").c_str()));
  return {task_id,status(str(field(j.p,"status"))),str(field(j.p,"detail"))};
 }catch(const std::exception& e){return {task_id,ExecutionStatus::Failed,e.what()};}
}
bool GatewayRobotArm::cancel(const std::string& task_id){
 try {
  Json j(json_tokener_parse(client_->request("cancel","{\"robot_id\":"+quote(id_)+",\"id\":"+quote(id_+":"+task_id)+"}").c_str()));
  return json_object_get_boolean(field(j.p,"cancelled"));
 }catch(...){return false;}
}
GatewayEnvironment::GatewayEnvironment(std::shared_ptr<GatewayClient> client):client_(std::move(client)){
 if(!client_)throw std::invalid_argument("Missing gateway client");
}
WorldSnapshot GatewayEnvironment::observe() const{
 Json j(json_tokener_parse(client_->request("observe").c_str()));
 WorldSnapshot w;
 auto* a=field(j.p,"objects");
 for(size_t i=0;i<json_object_array_length(a);++i){
  auto* obj=json_object_array_get_idx(a,i);
  if(json_object_get_boolean(field(obj,"valid")))w.objects.push_back({str(field(obj,"object_id")),pose_from(field(obj,"pose"))});
 }
 return w;
}
}
