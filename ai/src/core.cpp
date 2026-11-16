#include "swarm/core.hpp"
#include <algorithm>
#include <set>
#include <stdexcept>
#include <utility>
namespace swarm {
MockRobotArm::MockRobotArm(std::string id) { state_.id = std::move(id); }
RobotState MockRobotArm::state() const { return state_; }
RobotCapabilities MockRobotArm::capabilities() const {
 return {{ManeuverType::Inspect, ManeuverType::Grasp, ManeuverType::Move,
          ManeuverType::Hold, ManeuverType::Release, ManeuverType::Sort}};
}
ManeuverResult MockRobotArm::execute(const ManeuverCommand& command) {
 if (!state_.available) return {command.task_id, ExecutionStatus::Rejected, "Robot unavailable"};
 // Logical stub: completion is synthetic; no motion, forces or collision checks.
 state_.pose = command.target;
 return {command.task_id, ExecutionStatus::Completed, "Mock completion only"};
}
bool MockRobotArm::cancel(const std::string&) { return true; }
MultiAgentSystem::MultiAgentSystem(LayoutConfig layout,
 std::vector<std::shared_ptr<IRobotArm>> robots,
 std::shared_ptr<IEnvironment> environment,
 std::shared_ptr<ISafetyPolicy> safety)
 : layout_(std::move(layout)), environment_(std::move(environment)), safety_(std::move(safety)) {
 if (!environment_ || !safety_) throw std::invalid_argument("Missing environment or safety policy");
 if (layout_.polygon_sides != 6 || layout_.stations.size() != 6 || robots.size() != 6)
   throw std::invalid_argument("Expected six stations and six robots in a hexagon");
 std::set<int> vertices;
 std::set<std::string> ids;
 for (const auto& station : layout_.stations) {
   if (station.hex_vertex < 0 || station.hex_vertex >= layout_.polygon_sides ||
       !vertices.insert(station.hex_vertex).second || !ids.insert(station.robot_id).second)
     throw std::invalid_argument("Invalid or duplicate station");
 }
 for (auto& robot : robots) {
   if (!robot || !ids.contains(robot->state().id)) throw std::invalid_argument("Robot ID mismatch");
   agents_.push_back({robot->state().id, std::move(robot), std::nullopt});
 }
 std::set<std::string> actual;
 for (const auto& a : agents_) if (!actual.insert(a.id).second)
   throw std::invalid_argument("Duplicate robot instance");
}
ManeuverResult MultiAgentSystem::dispatch(const ManeuverCommand& command) {
 if (command.task_id.empty()) return {command.task_id, ExecutionStatus::Rejected, "Missing task ID"};
 for (std::size_t offset = 0; offset < agents_.size(); ++offset) {
   auto index = (next_agent_ + offset) % agents_.size();
   auto& agent = agents_[index];
   auto state = agent.robot->state();
   const auto supported = agent.robot->capabilities().maneuvers;
   if (!state.available || std::find(supported.begin(), supported.end(), command.type) == supported.end()) continue;
   if (!safety_->permit(state, command)) continue;
   agent.current_role = command.type;
   auto result = agent.robot->execute(command);
   agent.current_role.reset();
   if (result.status != ExecutionStatus::Rejected) {
     next_agent_ = (index + 1) % agents_.size();
     return result;
   }
 }
 return {command.task_id, ExecutionStatus::Rejected, "No eligible or permitted robot"};
}
ManeuverResult MultiAgentSystem::dispatchWithMpc(const ManeuverCommand& goal, IMpcPlanner& planner) {
 MpcContext context{observe(), {}};
 for (const auto& agent : agents_) context.robots.push_back(agent.robot->state());
 auto command = planner.propose(goal, context);
 if (!command) return {goal.task_id, ExecutionStatus::Rejected, "MPC did not propose a maneuver"};
 if (command->task_id != goal.task_id)
   return {goal.task_id, ExecutionStatus::Rejected, "MPC task ID mismatch"};
 return dispatch(*command);
}
std::string to_string(ExecutionStatus status) {
 switch (status) {
 case ExecutionStatus::Accepted: return "accepted";
 case ExecutionStatus::Rejected: return "rejected";
 case ExecutionStatus::Completed: return "completed";
 case ExecutionStatus::Failed: return "failed";
 }
 return "unknown";
}
}
