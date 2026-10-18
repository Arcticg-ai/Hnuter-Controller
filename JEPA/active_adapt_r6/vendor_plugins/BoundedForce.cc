// Simulation-only exogenous force anchored to a committed reference clock; no state feedback or learned outputs.
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/plugin/Register.hh>
#include <fstream>
#include <iomanip>
#include <cmath>
namespace jepa {
class BoundedForce final: public gz::sim::System,public gz::sim::ISystemConfigure,public gz::sim::ISystemPreUpdate {
 std::vector<std::pair<gz::sim::Entity,double>> links;double total=0,start=62,end=70,fy=.6,anchor_time=-1;std::string anchor_file;bool anchored=false;std::ofstream out;
 public:
 void Configure(const gz::sim::Entity &e,const std::shared_ptr<const sdf::Element> &s,gz::sim::EntityComponentManager &ecm,gz::sim::EventManager &)override{
  anchor_file=s->Get<std::string>("anchor_file");
  start=s->Get<double>("start");end=s->Get<double>("end");fy=s->Get<double>("force_y");
  for(auto id:gz::sim::Model(e).Links(ecm)){auto in=ecm.Component<gz::sim::components::Inertial>(id);if(in){double m=in->Data().MassMatrix().Mass();links.push_back({id,m});total+=m;}}
  if(total<=0 || std::abs(fy)>.8 || end<=start)throw std::runtime_error("invalid bounded disturbance");
  out.open(s->Get<std::string>("output"));out<<std::setprecision(17)<<"sim_t,force_y_N,total_mass_kg,anchor_received_sim_t,start_s,end_s\n";
 }
 void PreUpdate(const gz::sim::UpdateInfo &i,gz::sim::EntityComponentManager &ecm)override{
  if(i.paused)return;double t=std::chrono::duration<double>(i.simTime).count();double f=0;
  if(!anchored && i.iterations%25==0){
   std::ifstream input(anchor_file);double epoch;
   if(input>>epoch){
    if(!std::isfinite(epoch)||epoch<0||epoch+start<=t+1)throw std::runtime_error("late or invalid reference epoch");
    start+=epoch;end+=epoch;anchored=true;anchor_time=t;
   }
  }
  if(anchored && t>start && t<end){double s=std::sin(M_PI*(t-start)/(end-start));f=fy*s*s;}
  for(auto [id,m]:links)gz::sim::Link(id).AddWorldForce(ecm,gz::math::Vector3d(0,f*m/total,0));
  out<<t<<','<<f<<','<<total<<','<<anchor_time<<','<<(anchored?start:-1)<<','<<(anchored?end:-1)<<'\n';if(i.iterations%250==0)out.flush();
 }
};
}
GZ_ADD_PLUGIN(jepa::BoundedForce,gz::sim::System,jepa::BoundedForce::ISystemConfigure,jepa::BoundedForce::ISystemPreUpdate)
