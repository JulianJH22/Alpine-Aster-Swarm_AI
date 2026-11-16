#pragma once
#include <array>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace swarm {
enum class ManeuverType { Inspect, Grasp, Move, Hold, Release, Sort };
enum class ExecutionStatus { Accepted, Rejected, Completed, Failed };
struct Pose { double x{}, y{}, z{}, roll{}, pitch{}, yaw{}; };
struct RobotState { std::string id; Pose pose; bool available{true}; std::string status{"idle"}; };
struct RobotCapabilities { std::vector<ManeuverType> maneuvers; };
struct ManeuverCommand { std::string task_id; ManeuverType type; std::string object_id; Pose target; };
struct ManeuverResult { std::string task_id; ExecutionStatus status; std::string detail; };
struct Observation { std::string object_id; Pose pose; };
struct WorldSnapshot { std::vector<Observation> objects; };
class IRobotArm {
public:
 virtual ~IRobotArm() = default;
 virtual RobotState state() const = 0;
 virtual RobotCapabilities capabilities() const = 0;
 virtual ManeuverResult execute(const ManeuverCommand&) = 0;
 virtual bool cancel(const std::string& task_id) = 0;
};
class IEnvironment {
public:
 virtual ~IEnvironment() = default;
 virtual WorldSnapshot observe() const = 0;
};
class ISafetyPolicy {
public:
 virtual ~ISafetyPolicy() = default;
 virtual bool permit(const RobotState&, const ManeuverCommand&) const = 0;
};
class MockRobotArm final : public IRobotArm {
public:
 explicit MockRobotArm(std::string id);
 RobotState state() const override;
 RobotCapabilities capabilities() const override;
 ManeuverResult execute(const ManeuverCommand&) override;
 bool cancel(const std::string&) override;
private:
 RobotState state_;
};
class MockEnvironment final : public IEnvironment {
public:
 WorldSnapshot observe() const override { return {}; }
};
// A placeholder policy for LOGICAL TESTS ONLY; never connect to physical actuators.
class MockSafetyPolicy final : public ISafetyPolicy {
public:
 bool permit(const RobotState&, const ManeuverCommand&) const override { return true; }
};
struct RobotAgent {
 std::string id;
 std::shared_ptr<IRobotArm> robot;
 std::optional<ManeuverType> current_role; // Assigned per task, never hard-coded.
};
// MPC contract: concrete predictive models, horizons and solvers are deferred.
// MPC is an optimization-based controller; its dynamics model may be identified from data.
struct MpcContext {
 WorldSnapshot world;
 std::vector<RobotState> robots;
};
class IMpcPlanner {
public:
 virtual ~IMpcPlanner() = default;
 virtual std::optional<ManeuverCommand> propose(const ManeuverCommand& goal,
                                                const MpcContext& context) = 0;
};
struct StationConfig { std::string robot_id; int hex_vertex; };
struct LayoutConfig {
 int polygon_sides{6};
 std::vector<StationConfig> stations;
};
class MultiAgentSystem {
public:
 MultiAgentSystem(LayoutConfig layout, std::vector<std::shared_ptr<IRobotArm>> robots,
                  std::shared_ptr<IEnvironment> environment,
                  std::shared_ptr<ISafetyPolicy> safety);
 ManeuverResult dispatch(const ManeuverCommand& command);
 ManeuverResult dispatchWithMpc(const ManeuverCommand& goal, IMpcPlanner& planner);
 const std::vector<RobotAgent>& agents() const { return agents_; }
 const LayoutConfig& layout() const { return layout_; }
 WorldSnapshot observe() const { return environment_->observe(); }
private:
 LayoutConfig layout_;
 std::vector<RobotAgent> agents_;
 std::shared_ptr<IEnvironment> environment_;
 std::shared_ptr<ISafetyPolicy> safety_;
 std::size_t next_agent_{0};
};
std::string to_string(ExecutionStatus status);
}
