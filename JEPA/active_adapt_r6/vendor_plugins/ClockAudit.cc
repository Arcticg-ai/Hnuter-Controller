// Evaluator-only physical-time availability boundary. No state or control inputs.
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>
#include <fstream>
#include <iomanip>
#include <time.h>
namespace jepa {
class ClockAudit final : public gz::sim::System, public gz::sim::ISystemConfigure,
                        public gz::sim::ISystemPreUpdate, public gz::sim::ISystemPostUpdate {
 std::ofstream out;
 public:
 void Configure(const gz::sim::Entity &,const std::shared_ptr<const sdf::Element> &s,
                gz::sim::EntityComponentManager &,gz::sim::EventManager &) override {
   out.open(s->Get<std::string>("output"));out<<std::setprecision(17);
   out<<"generation_monotonic_ns,sim_upper_s,dt_s,iteration,stage,sim_upper_us\n";
 }
 void Write(const gz::sim::UpdateInfo &i,int stage) {
   if(i.paused)return;
   struct timespec ts;clock_gettime(CLOCK_MONOTONIC,&ts);
   const long long ns=static_cast<long long>(ts.tv_sec)*1000000000LL+ts.tv_nsec;
   // info.simTime is an upper bound even in pre-physics callback. Extra dt is
   // deliberately conservative for plugin ordering and native timestamp phase.
   const double dt=std::chrono::duration<double>(i.dt).count();
   out<<ns<<','<<std::chrono::duration<double>(i.simTime).count()+dt<<','<<dt<<','<<i.iterations<<','<<stage<<','<<(std::chrono::duration_cast<std::chrono::nanoseconds>(i.simTime+i.dt).count()+999)/1000<<'\n';
   if(i.iterations%250==0)out.flush();
 }
 void PreUpdate(const gz::sim::UpdateInfo &i,gz::sim::EntityComponentManager &) override {Write(i,0);}
 void PostUpdate(const gz::sim::UpdateInfo &i,const gz::sim::EntityComponentManager &) override {Write(i,1);}
};
}
GZ_ADD_PLUGIN(jepa::ClockAudit,gz::sim::System,jepa::ClockAudit::ISystemConfigure,
              jepa::ClockAudit::ISystemPreUpdate,jepa::ClockAudit::ISystemPostUpdate)
