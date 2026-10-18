from pathlib import Path
import shutil,difflib,json,hashlib,sys
R=Path(__file__).resolve().parent;P=Path(sys.argv[1]).resolve() if len(sys.argv)>1 else R.parent.parent.parent/'jepa-px4-gps3-worktree';D=P/'src/modules/simulation/gz_bridge'
for n in ['GZBridge.cpp','GZBridge.hpp']:assert (D/n).read_bytes()==(R/'backup'/n).read_bytes()
h=(D/'GZBridge.hpp').read_text().replace('#include "GZGimbal.hpp"','#include "GZGimbal.hpp"\n#if defined(CONFIG_ARCH_BOARD_PX4_SITL)\n#include "GpsNoiseRng.hpp"\n#include <fstream>\n#include <mutex>\n#endif')
h=h.replace('static float generate_wgn();','float generate_wgn();')
h=h.replace('\t// GPS noise model','''#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
	GpsNoiseRng _gps_rng;
	bool _gps_seeded{false}, _gps_seed_valid{true};
	uint32_t _gps_seed{0};
	uint64_t _gps_callbacks{0};
	std::mutex _gps_mutex;
	std::ofstream _gps_audit;
#endif
	// GPS noise model''')
s=(D/'GZBridge.cpp').read_text().replace('#include <string>','#include <string>\n#include <cstdlib>\n#include <cerrno>\n#include <iomanip>')
s=s.replace('\tupdateParams();\n}', '''	updateParams();
#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
	if (const char *value = std::getenv("JEPA_GPS_SEED")) {
		char *end=nullptr; errno=0; const unsigned long parsed=std::strtoul(value,&end,10);
		_gps_seed_valid = *value && *value!='-' && end && !*end && !errno && parsed<=UINT32_MAX;
		if (_gps_seed_valid) {
			_gps_seed=static_cast<uint32_t>(parsed); _gps_rng.seed(_gps_seed); _gps_seeded=true;
			PX4_INFO("JEPA GPS private MT19937 seed=%u; native noise amplitudes unchanged",_gps_seed);
			if(const char *path=std::getenv("JEPA_GPS_AUDIT")) {
				_gps_audit.open(path); _gps_seed_valid=_gps_audit.is_open();
				_gps_audit << "seed,callback_index,source_ns,receive_us,normal_draws,pos_n,pos_e,pos_d,vel_n,vel_e,vel_d,lat_in,lon_in,alt_in,lat_out,lon_out,alt_out\\n";
				_gps_audit << std::setprecision(17);
			}
		}
	}
#endif
}''',1)
s=s.replace('int GZBridge::init()\n{','''int GZBridge::init()
{
#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
	if (!_gps_seed_valid) {PX4_ERR("Invalid JEPA GPS seed/audit configuration"); return PX4_ERROR;}
#endif''')
s=s.replace('float GZBridge::generate_wgn()\n{','''float GZBridge::generate_wgn()
{
#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
	if (_gps_seeded) {return _gps_rng.normal();}
#endif''')
s=s.replace('void GZBridge::navSatCallback(const gz::msgs::NavSat &msg)\n{','''void GZBridge::navSatCallback(const gz::msgs::NavSat &msg)
{
#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
	std::unique_lock<std::mutex> gps_lock(_gps_mutex,std::defer_lock);
	if(_gps_seeded) {gps_lock.lock(); ++_gps_callbacks;}
#endif''')
needle='\taddGpsNoise(latitude, longitude, altitude, vel_north, vel_east, vel_down);'
s=s.replace(needle,needle+'''
#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
	if(_gps_audit.is_open()) {
		const int64_t source_ns=int64_t(msg.header().stamp().sec())*1000000000LL+msg.header().stamp().nsec();
		_gps_audit << _gps_seed << ',' << _gps_callbacks << ',' << source_ns << ',' << timestamp << ',' << _gps_rng.draws() << ','
		 << _gps_pos_noise_n << ',' << _gps_pos_noise_e << ',' << _gps_pos_noise_d << ',' << _gps_vel_noise_n << ',' << _gps_vel_noise_e << ',' << _gps_vel_noise_d << ','
		 << msg.latitude_deg() << ',' << msg.longitude_deg() << ',' << msg.altitude() << ',' << latitude << ',' << longitude << ',' << altitude << '\\n';
	}
#endif''')
s=s.replace('int GZBridge::custom_command(int argc, char *argv[])\n{\n', 'int GZBridge::custom_command(int argc, char *argv[])\n{\n' + '#if defined(CONFIG_ARCH_BOARD_PX4_SITL)\n\tif (argc > 0 && strcmp(argv[0], "gps_audit_close") == 0) {\n\t\tGZBridge *instance = _object.load();\n\t\tif (!instance || !instance->_gps_seeded) { return PX4_ERROR; }\n\t\tstd::lock_guard<std::mutex> guard(instance->_gps_mutex);\n\t\tif (!instance->_gps_audit.is_open()) { return PX4_ERROR; }\n\t\tinstance->_gps_audit.close();\n\t\tif (instance->_gps_audit.fail()) { PX4_ERR("JEPA GPS audit close failed"); return PX4_ERROR; }\n\t\tPX4_INFO("JEPA GPS audit closed");\n\t\treturn PX4_OK;\n\t}\n#endif\n')
changes={'GZBridge.cpp':s,'GZBridge.hpp':h}
for n,t in changes.items():(D/n).write_text(t)
shutil.copy2(R/'GpsNoiseRng.hpp',D/'GpsNoiseRng.hpp')
(R/'bridge.patch').write_text(''.join(''.join(difflib.unified_diff((R/'backup'/n).read_text().splitlines(True),t.splitlines(True),fromfile='a/src/modules/simulation/gz_bridge/'+n,tofile='b/src/modules/simulation/gz_bridge/'+n)) for n,t in changes.items()))
# The state recurrence and amplitudes must remain byte-for-byte unchanged.
def recurrence(t):return t[t.index('void GZBridge::addGpsNoise'):t.index('void GZBridge::navSatCallback')]
assert recurrence(s)==recurrence((R/'backup/GZBridge.cpp').read_text())
print('Only isolated SITL bridge RNG and audit changed; noise recurrence identical')
