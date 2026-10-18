// Experiment-local Gazebo server entry. No PX4/control/physics modifications.
// Explicit SetSeed accepts zero; stock gz sim 8.15 skips that value.
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/config.hh>
#include <gz/math/Rand.hh>
#include <gz/common/Console.hh>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include <unistd.h>
#include <cstdlib>

int main(int argc, char **argv) {
  try {
    if (argc != 4 && argc != 5) throw std::runtime_error("seeded_server <seed> <world.sdf> <receipt.json> OR --probe <legacy|fixed> <seed>");
    const bool probe=std::string(argv[1])=="--probe";
    std::string raw=probe?argv[3]:argv[1];
    if(raw!="0" && raw!="1" && raw!="2" && raw!="3" && raw!="4") throw std::runtime_error("Registered seeds are 0, 1, 2, 3 and 4");
    unsigned int seed=static_cast<unsigned int>(std::stoul(raw));
    gz::sim::ServerConfig config;
    bool set=!probe || std::string(argv[2])=="fixed" || seed!=0;
    if(probe && std::string(argv[2])!="fixed" && std::string(argv[2])!="legacy") throw std::runtime_error("invalid probe mode");
    if(set) config.SetSeed(seed);
    if(probe) {
      std::cout << "{\"requested\":"<<seed<<",\"effective\":"<<gz::math::Rand::Seed()<<",\"set_called\":"<<(set?"true":"false")<<",\"draws\":[" << std::setprecision(17);
      for(int i=0;i<8;++i) std::cout<<(i?",":"")<<gz::math::Rand::DblNormal(0,1);
      std::cout<<"]}\n"; return 0;
    }
    if(std::getenv("HNUTER_CONTAINER")==nullptr) throw std::runtime_error("isolated container required");
    if(gz::math::Rand::Seed()!=seed) throw std::runtime_error("seed readback mismatch");
    gz::common::Console::SetVerbosity(1);
    // Matches the old -r -s <world>, with all optional engines/recording disabled.
    config.SetSdfFile(argv[2]);
    config.SetInitialSimTime(0.0);
    config.SetHeadlessRendering(false);
    {std::ofstream f(argv[3]);if(!f) throw std::runtime_error("cannot write seed receipt");
     f<<"{\"requested_seed\":"<<seed<<",\"effective_seed_before_server\":"<<gz::math::Rand::Seed()<<",\"set_seed_called\":true,\"draws_consumed_by_audit\":0,\"pid\":"<<getpid()<<",\"gz_version\":\""<<GZ_SIM_VERSION_FULL<<"\"}\n";}
    gz::sim::Server server(config);
    return server.Run(true,0,false)?0:1;
  } catch(const std::exception &e) {std::cerr<<e.what()<<"\n";return 2;}
}
