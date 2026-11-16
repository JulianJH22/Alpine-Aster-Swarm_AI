#include "swarm/gateway.hpp"
#include <iostream>
#include <memory>
#include <vector>
int main(int argc,char** argv){
 try{
  auto client=std::make_shared<swarm::GatewayClient>(argc>1?argv[1]:"127.0.0.1",argc>2?std::stoi(argv[2]):7777);
  swarm::LayoutConfig layout;
  std::vector<std::shared_ptr<swarm::IRobotArm>> robots;
  for(int i=0;i<6;++i){
   std::string id="arm_"+std::to_string(i+1);
   layout.stations.push_back({id,i});
   robots.push_back(std::make_shared<swarm::GatewayRobotArm>(id,client));
  }
  swarm::MultiAgentSystem sys(layout,robots,std::make_shared<swarm::GatewayEnvironment>(client),std::make_shared<swarm::MockSafetyPolicy>());
  std::cout<<"Connected: "<<sys.agents().size()<<" agents, "<<sys.observe().objects.size()<<" observed objects\n";
  std::cout<<"Read-only handshake complete. No maneuver submitted.\n";
  return 0;
 }catch(const std::exception& e){std::cerr<<"Gateway error: "<<e.what()<<"\n";return 1;}
}
