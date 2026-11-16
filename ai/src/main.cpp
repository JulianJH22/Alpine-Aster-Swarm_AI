#include "swarm/core.hpp"
#include <iostream>
#include <memory>
#include <vector>
int main() {
 using namespace swarm;
 LayoutConfig layout;
 std::vector<std::shared_ptr<IRobotArm>> robots;
 for (int i=0;i<6;++i) {
   const auto id = "arm_" + std::to_string(i+1);
   layout.stations.push_back({id,i});
   robots.push_back(std::make_shared<MockRobotArm>(id));
 }
 MultiAgentSystem system(layout, robots, std::make_shared<MockEnvironment>(),
                         std::make_shared<MockSafetyPolicy>());
 for (int i=0;i<8;++i) {
   ManeuverCommand cmd{"task_"+std::to_string(i), ManeuverType::Grasp, "object_1", {}};
   auto result=system.dispatch(cmd);
   std::cout << cmd.task_id << ": " << to_string(result.status) << " (" << result.detail << ")\n";
 }
 std::cout << "Six agents; six occupied vertices in a configurable hexagon; no physical simulation.\n";
}
