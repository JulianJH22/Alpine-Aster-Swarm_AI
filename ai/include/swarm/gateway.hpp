#pragma once
#include "swarm/core.hpp"
#include <memory>
#include <string>

namespace swarm {
// Optional Linux/POSIX adapter. Not suitable for safety-critical hardware control.
class GatewayClient {
public:
 GatewayClient(std::string host="127.0.0.1", int port=7777, int timeout_ms=3000);
 ~GatewayClient();
 GatewayClient(const GatewayClient&)=delete;
 GatewayClient& operator=(const GatewayClient&)=delete;
 std::string request(const std::string& operation, const std::string& json_args="{}");
 double step(double seconds); // Advances simulator clock; never called implicitly by execute().
 bool run_until_idle(double timeout_seconds=120.0);
private:
 int socket_{-1};
 unsigned long next_id_{0};
 std::string buffer_;
};
class GatewayRobotArm final : public IRobotArm {
public:
 GatewayRobotArm(std::string id, std::shared_ptr<GatewayClient> client);
 RobotState state() const override;
 RobotCapabilities capabilities() const override;
 ManeuverResult execute(const ManeuverCommand&) override;
 bool cancel(const std::string& task_id) override;
 ManeuverResult poll(const std::string& task_id) const;
private:
 std::string id_;
 std::shared_ptr<GatewayClient> client_;
};
class GatewayEnvironment final : public IEnvironment {
public:
 explicit GatewayEnvironment(std::shared_ptr<GatewayClient> client);
 WorldSnapshot observe() const override;
private:
 std::shared_ptr<GatewayClient> client_;
};
}
