#include "swarm/gateway.hpp"
#include <json-c/json.h>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <random>
#include <iomanip>

namespace {
struct Json {
 json_object* value;
 explicit Json(const std::string& text) : value(json_tokener_parse(text.c_str())) {
  if (!value) throw std::runtime_error("Respuesta JSON invalida");
 }
 ~Json() { json_object_put(value); }
 Json(const Json&) = delete;
};
json_object* required(json_object* object, const char* key) {
 json_object* out = nullptr;
 if (!json_object_object_get_ex(object, key, &out)) throw std::runtime_error(std::string("Falta campo: ") + key);
 return out;
}
std::string string_field(json_object* object, const char* key) {
 return json_object_get_string(required(object, key));
}
std::string quoted(const std::string& value) {
 json_object* obj = json_object_new_string(value.c_str());
 std::string result = json_object_to_json_string_ext(obj, JSON_C_TO_STRING_PLAIN);
 json_object_put(obj);
 return result;
}
std::string new_run_id() {
 std::random_device rd;
 std::ostringstream out;
 out << "run_" << std::hex << std::setfill('0');
 for (int i = 0; i < 4; ++i) out << std::setw(8) << rd();
 return out.str();
}
struct Job {
 std::string robot, execution, task, maneuver, object;
 std::string target_json; // empty => null: orientation left to simulator defaults
};
std::string submit_args(const Job& job) {
 std::string command = "{\"execution_id\":" + quoted(job.execution) +
   ",\"task_id\":" + quoted(job.task) + ",\"robot_id\":" + quoted(job.robot) +
   ",\"type\":" + quoted(job.maneuver) + ",\"object_id\":" + quoted(job.object);
 if (!job.target_json.empty()) command += ",\"target\":" + job.target_json;
 return "{\"command\":" + command + "}}";
}
void check_status(swarm::GatewayClient& client, const Job& job, bool require_completed) {
 Json response(client.request("status", "{\"robot_id\":" + quoted(job.robot) +
    ",\"execution_id\":" + quoted(job.execution) + "}"));
 const auto status = string_field(response.value, "status");
 const auto detail = string_field(response.value, "detail");
 std::cout << job.robot << " " << job.maneuver << " [" << job.execution << "]: " << status
           << " (" << detail << ")\n";
 if (require_completed && status != "completed") throw std::runtime_error("Maniobra no completada: " + job.execution);
}
void stage(swarm::GatewayClient& client, const std::vector<Job>& jobs, double timeout_s) {
 // Both commands are submitted before advancing the shared simulation clock.
 std::vector<const Job*> accepted;
 try {
  for (const auto& job : jobs) {
   Json response(client.request("submit", submit_args(job)));
   const auto status = string_field(response.value, "status");
   std::cout << "submit " << job.robot << " " << job.maneuver << ": " << status << "\n";
   if (status != "accepted" && status != "running")
    throw std::runtime_error("Maniobra rechazada: " + job.execution + " (" + string_field(response.value, "detail") + ")");
   accepted.push_back(&job);
  }
  if (!client.run_until_idle(timeout_s)) throw std::runtime_error("Tiempo de simulacion agotado");
  for (const auto& job : jobs) check_status(client, job, true);
 } catch (...) {
  for (const auto* job : accepted) {
   try { client.request("cancel", "{\"robot_id\":" + quoted(job->robot) +
       ",\"id\":" + quoted(job->execution) + "}"); } catch (...) {}
  }
  throw;
 }
}
}
int main(int argc, char** argv) {
 try {
  if (argc > 4) {
   std::cerr << "Uso: swarm_control_demo [host=127.0.0.1] [port=7777] [demo.json]\n";
   return 2;
  }
  const std::string host = argc >= 2 ? argv[1] : "127.0.0.1";
  const int port = argc >= 3 ? std::stoi(argv[2]) : 7777;
  const std::string config_path = argc >= 4 ? argv[3] : "config/control_demo.json";
  std::ifstream input(config_path);
  if (!input) throw std::runtime_error("No se puede abrir configuracion: " + config_path);
  std::ostringstream content;
  content << input.rdbuf();
  Json config(content.str());
  auto* entries = required(config.value, "stages");
  if (json_object_array_length(entries) != 2) throw std::runtime_error("La demo necesita dos etapas");
  const std::string run_id = new_run_id();
  std::vector<std::vector<Job>> stages;
  for (int i = 0; i < 2; ++i) {
   auto* entries_in_stage = json_object_array_get_idx(entries, i);
   if (json_object_array_length(entries_in_stage) != 2) throw std::runtime_error("Cada etapa necesita dos maniobras");
   std::vector<Job> jobs;
   for (int j = 0; j < 2; ++j) {
    auto* item = json_object_array_get_idx(entries_in_stage, j);
    Job job;
    job.robot = string_field(item, "robot_id");
    job.task = string_field(item, "execution_id"); // Stable logical task ID from config
    job.execution = run_id + ":" + job.task; // Unique for each program invocation
    job.maneuver = string_field(item, "type");
    json_object* object = nullptr;
    if (json_object_object_get_ex(item, "object_id", &object)) job.object = json_object_get_string(object);
    json_object* target = nullptr;
    if (json_object_object_get_ex(item, "target", &target))
      job.target_json = json_object_to_json_string_ext(target, JSON_C_TO_STRING_PLAIN);
    jobs.push_back(std::move(job));
   }
   stages.push_back(std::move(jobs));
  }
  swarm::GatewayClient client(host, port, 5000);
  Json hello(client.request("hello"));
  auto* robots = required(hello.value, "robots");
  if (json_object_array_length(robots) != 6) throw std::runtime_error("Se esperaban seis brazos");
  Json objects(client.request("observe"));
  auto* observed = required(objects.value, "objects");
  for (const auto& stage_jobs : stages) {
   for (const auto& job : stage_jobs) {
    if (job.maneuver != "Grasp") continue;
    bool found = false;
    for (size_t i = 0; i < json_object_array_length(observed); ++i) {
     auto* obj = json_object_array_get_idx(observed, i);
     found |= string_field(obj, "object_id") == job.object;
    }
    if (!found) throw std::runtime_error("Objeto no observado: " + job.object);
   }
  }
  std::cout << "Conexion correcta: seis brazos. Demo C++ de dos brazos.\n";
  std::cout << "Identificador de esta ejecucion: " << run_id << "\n";
  stage(client, stages[0], 30.0);
  stage(client, stages[1], 30.0);
  std::cout << "Demo completada desde C++; MPC no utilizado.\n";
  return 0;
 } catch (const std::exception& e) {
  std::cerr << "Error de integracion: " << e.what() << "\n";
  return 1;
 }
}
