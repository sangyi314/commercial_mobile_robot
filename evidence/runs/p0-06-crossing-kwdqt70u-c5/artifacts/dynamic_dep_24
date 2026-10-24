#include <gz/msgs/pose_v.pb.h>
#include <google/protobuf/util/json_util.h>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <cmath>
#include <limits>
int main(){
 gz::msgs::Pose_V source;source.mutable_header()->mutable_stamp()->set_sec(1234567890123LL);source.mutable_header()->mutable_stamp()->set_nsec(987654321);
 auto *pose=source.add_pose();pose->set_id(42);pose->set_name("scripted_obstacle");pose->mutable_position()->set_x(1.123456789012345);pose->mutable_position()->set_z(.3);pose->mutable_orientation()->set_w(1);
 google::protobuf::util::JsonPrintOptions options;options.preserve_proto_field_names=true;std::string text;
 if(!google::protobuf::util::MessageToJsonString(source,&text,options).ok())throw std::runtime_error("encode");
 auto json=nlohmann::json::parse(text);
 if(!json["header"]["stamp"]["sec"].is_string())throw std::runtime_error("int64 source stamp must remain lossless");
 gz::msgs::Pose_V restored;
 if(!google::protobuf::util::JsonStringToMessage(text,&restored).ok()||restored.SerializeAsString()!=source.SerializeAsString())throw std::runtime_error("roundtrip changed identity/stamp/pose");
 source.mutable_pose(0)->mutable_position()->set_x(std::numeric_limits<double>::quiet_NaN());
 text.clear();google::protobuf::util::MessageToJsonString(source,&text,options);json=nlohmann::json::parse(text);
 if(json["pose"][0]["position"]["x"]!="NaN")throw std::runtime_error("nonfinite truth must not be replaced with zero");
}
