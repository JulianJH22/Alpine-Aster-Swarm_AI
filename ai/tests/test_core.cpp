#include "swarm/core.hpp"
#include <cassert>
#include <memory>
#include <stdexcept>
#include <vector>
class NullMpc final : public swarm::IMpcPlanner {
public:
 std::optional<swarm::ManeuverCommand> propose(const swarm::ManeuverCommand&, const swarm::MpcContext&) override {
  return std::nullopt;
 }
};
class PassThroughMpc final : public swarm::IMpcPlanner {
public:
 std::optional<swarm::ManeuverCommand> propose(const swarm::ManeuverCommand& goal, const swarm::MpcContext& context) override {
  assert(context.robots.size() == 6);
  return goal;
 }
};
int main() {
 using namespace swarm;
 LayoutConfig layout;
 std::vector<std::shared_ptr<IRobotArm>> robots;
 for(int i=0;i<6;++i) {
  auto id="arm_"+std::to_string(i+1);
  layout.stations.push_back({id,i});
  robots.push_back(std::make_shared<MockRobotArm>(id));
 }
 MultiAgentSystem sys(layout,robots,std::make_shared<MockEnvironment>(),std::make_shared<MockSafetyPolicy>());
 assert(sys.agents().size()==6);
 assert(sys.layout().polygon_sides==6);
 assert(sys.dispatch({"t1",ManeuverType::Move,"object",{}}).status==ExecutionStatus::Completed);
 assert(sys.dispatch({"",ManeuverType::Move,"object",{}}).status==ExecutionStatus::Rejected);
 for(const auto& agent:sys.agents()) assert(!agent.current_role.has_value());
 NullMpc no_solution;
 assert(sys.dispatchWithMpc({"mpc_none",ManeuverType::Move,"object",{}}, no_solution).status==ExecutionStatus::Rejected);
 PassThroughMpc passthrough;
 assert(sys.dispatchWithMpc({"mpc_ok",ManeuverType::Move,"object",{}}, passthrough).status==ExecutionStatus::Completed);
 auto bad=layout; bad.stations[5].hex_vertex=0;
 bool threw=false;
 try { MultiAgentSystem invalid(bad,robots,std::make_shared<MockEnvironment>(),std::make_shared<MockSafetyPolicy>()); }
 catch(const std::invalid_argument&) { threw=true; }
 assert(threw);
}
