// SITL diagnostic only. Reads ECM state; never publishes or applies commands.
// CSV is an offline oracle and must not be read by a flight estimator.
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/plugin/Register.hh>
#include <fstream>
#include <iomanip>
#include <vector>
#include <string>

namespace jepa {
class TruthRecorder final : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate, public gz::sim::ISystemConfigurePriority {
  std::ofstream out;
  std::vector<gz::sim::Entity> links, joints;
  bool ready = false;
 public:
  // Motors use priority 0. Sample their applied commands after all PreUpdates,
  // before the physics Update phase; no ordering assumption about PostUpdates.
  PriorityType ConfigurePriority() override {return 100;}
  void Configure(const gz::sim::Entity &e,
      const std::shared_ptr<const sdf::Element> &s,
      gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override {
    gz::sim::Model model(e);
    links=model.Links(ecm); joints=model.Joints(ecm);
    out.open(s->Get<std::string>("output")); out<<std::setprecision(17);
    out<<"t,dt,iteration,stage";
    for(auto id:links) {
      gz::sim::Link(id).EnableVelocityChecks(ecm);
      const auto n=ecm.Component<gz::sim::components::Name>(id)->Data();
      for(const auto &suffix:{"px","py","pz","qw","qx","qy","qz",
          "vx","vy","vz","wx","wy","wz","fx","fy","fz","tx","ty","tz"})
        out<<','<<n<<'_'<<suffix;
    }
    for(auto id:joints) {
      if(!ecm.Component<gz::sim::components::JointPosition>(id))
        ecm.CreateComponent(id,gz::sim::components::JointPosition());
      if(!ecm.Component<gz::sim::components::JointVelocity>(id))
        ecm.CreateComponent(id,gz::sim::components::JointVelocity());
      const auto n=ecm.Component<gz::sim::components::Name>(id)->Data();
      out<<','<<n<<"_q,"<<n<<"_qd";
    }
    out<<'\n'; ready=out.good();
  }
  void Write(const gz::sim::UpdateInfo &info,
      const gz::sim::EntityComponentManager &ecm, bool pre) {
    if(!ready || info.paused || info.iterations%10!=0) return;
    const double dt=std::chrono::duration<double>(info.dt).count();
    const double t=std::chrono::duration<double>(info.simTime).count();
    out<<(pre?t-dt:t)<<','<<dt<<','<<info.iterations<<','<<(pre?0:1);
    auto vec=[&](const gz::math::Vector3d &v){out<<','<<v.X()<<','<<v.Y()<<','<<v.Z();};
    for(auto id:links) {
      gz::sim::Link link(id);
      auto p=link.WorldPose(ecm).value_or(gz::sim::worldPose(id,ecm));
      vec(p.Pos()); auto q=p.Rot(); out<<','<<q.W()<<','<<q.X()<<','<<q.Y()<<','<<q.Z();
      vec(link.WorldLinearVelocity(ecm).value_or(gz::math::Vector3d::Zero));
      vec(link.WorldAngularVelocity(ecm).value_or(gz::math::Vector3d::Zero));
      auto wr=ecm.Component<gz::sim::components::ExternalWorldWrenchCmd>(id);
      if(wr) {
        auto f=wr->Data().force(); auto m=wr->Data().torque();
        out<<','<<f.x()<<','<<f.y()<<','<<f.z()<<','<<m.x()<<','<<m.y()<<','<<m.z();
      } else out<<",0,0,0,0,0,0";
    }
    for(auto id:joints) {
      auto p=ecm.Component<gz::sim::components::JointPosition>(id);
      auto v=ecm.Component<gz::sim::components::JointVelocity>(id);
      out<<','<<(p&&!p->Data().empty()?p->Data()[0]:0.)
         <<','<<(v&&!v->Data().empty()?v->Data()[0]:0.);
    }
    out<<'\n';
    if(info.iterations%250==0) out.flush();
  }
  void PreUpdate(const gz::sim::UpdateInfo &i,gz::sim::EntityComponentManager &e) override {Write(i,e,true);}
  void PostUpdate(const gz::sim::UpdateInfo &i,const gz::sim::EntityComponentManager &e) override {Write(i,e,false);}
};
}
GZ_ADD_PLUGIN(jepa::TruthRecorder, gz::sim::System, jepa::TruthRecorder::ISystemConfigure,
  jepa::TruthRecorder::ISystemPreUpdate, jepa::TruthRecorder::ISystemPostUpdate,
  jepa::TruthRecorder::ISystemConfigurePriority)
