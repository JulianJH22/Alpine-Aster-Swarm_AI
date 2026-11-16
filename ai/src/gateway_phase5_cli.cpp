#include "swarm/gateway.hpp"
#include <iostream>
#include <string>
#include <stdexcept>
#include <cstdlib>

int main(int argc, char** argv) {
    if (argc < 4) {
        std::cerr << "Usage: swarm_phase5_cli HOST PORT hello|snapshot|reset|step|samples|events [argument]\n";
        return 2;
    }
    try {
        const std::string command = argv[3];
        std::string op, args = "{}";
        if (command == "hello") op = "hello";
        else if (command == "snapshot") op = "snapshot";
        else if (command == "reset") op = "reset";
        else if (command == "step") {
            if (argc != 5) throw std::invalid_argument("step requires seconds");
            double seconds = std::stod(argv[4]);
            if (!(seconds > 0 && seconds <= 10)) throw std::invalid_argument("seconds must be >0 and <=10");
            op = "step";
            args = "{\"seconds\":" + std::to_string(seconds) + "}";
        } else if (command == "samples") {
            op = "telemetry_samples";
            if (argc == 5) {
                int index = std::stoi(argv[4]);
                if (index < 0) throw std::invalid_argument("index must be >=0");
                args = "{\"since_index\":" + std::to_string(index) + "}";
            }
        } else if (command == "events") op = "telemetry";
        else throw std::invalid_argument("Unknown command");
        swarm::GatewayClient client(argv[1], std::stoi(argv[2]));
        std::cout << client.request(op, args) << '\n';
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Phase 5 gateway error: " << e.what() << '\n';
        return 1;
    }
}
