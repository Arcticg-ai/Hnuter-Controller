#include "/home/hnuter/PX4-Hnuter/PX4-Autopilot-Hnuter-tail-sitl/src/modules/land_detector/HnuterDirectLandingGuard.hpp"
#include <cassert>
#include <limits>

int main()
{
    using land_detector::hnuter_direct_low_demand;
    assert(!hnuter_direct_low_demand(true, 1000000, 900000, .5f, .32f));
    assert(hnuter_direct_low_demand(true, 1000000, 900000, .1f, .32f));
    assert(hnuter_direct_low_demand(true, 1000000, 900000, 0.f, .32f));
    assert(!hnuter_direct_low_demand(true, 1000000, 0, 0.f, .32f));
    assert(!hnuter_direct_low_demand(true, 1000000, 500000, 0.f, .32f));
    assert(!hnuter_direct_low_demand(true, 1000000, 1000001, 0.f, .32f));
    assert(!hnuter_direct_low_demand(true, 1000000, 900000, -1.f, .32f));
    assert(!hnuter_direct_low_demand(true, 1000000, 900000,
                                   std::numeric_limits<float>::quiet_NaN(), .32f));
    assert(hnuter_direct_low_demand(false, 1000000, 0, .5f, .32f));
}
