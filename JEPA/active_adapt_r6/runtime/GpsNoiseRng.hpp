#pragma once
#include <cstdint>
#include <cmath>
#include <random>
// SITL-only private uniform stream. Keep the original polar Box-Muller
// transform, float precision, cached second variate and Gaussian N(0,1).
class GpsNoiseRng {
public:
 void seed(uint32_t value) { engine.seed(value); first=true; v1=v2=s=0.f; count=0; }
 float normal() {
  float x;
  if(first) {
   do {float u1=uniform(),u2=uniform();v1=2.f*u1-1.f;v2=2.f*u2-1.f;s=v1*v1+v2*v2;} while(s>=1.f||std::fabs(s)<1e-8f);
   x=v1*float(std::sqrt(-2.f*float(std::log(s))/s));
  } else {x=v2*float(std::sqrt(-2.f*float(std::log(s))/s));}
  first=!first;++count;return x;
 }
 uint64_t draws() const{return count;}
private:
 float uniform(){return static_cast<float>(engine()>>1)/2147483647.f;}
 std::mt19937 engine{0}; bool first=true;float v1=0.f,v2=0.f,s=0.f;uint64_t count=0;
};
