#include "person_follow_sim/scenarios.hpp"
#include <cstdlib>
#include <iostream>
using namespace person_follow_sim;
int count=0;
void check(bool value, const char * name) {++count; if (!value) {std::cerr<<name<<'\n'; std::exit(1);}}
int main()
{
  auto p=scenario("stationary", 3);
  auto optical=to_optical(p,0,0,0,0.25,1.0);
  check(std::abs(optical.z-2.25)<1e-9 && optical.x==0 && optical.y==0, "camera mounting offset");
  p.y=1;
  check(to_optical(p,0,0,0,0.25,1).x<0, "left person maps to negative optical x");
  p.y=-1;
  check(to_optical(p,0,0,0,0.25,1).x>0, "right person maps to positive optical x");
  p={0,2.5,1,true,true};
  check(std::abs(to_optical(p,0,0,pi/2,0.25,1).x)<1e-9, "robot yaw closes feedback loop");
  check(in_view({0,0,2},1.2,0.9), "visible forward person");
  check(!in_view({0,0,-2},1.2,0.9), "behind robot invisible");
  check(!in_view({2,0,1},1.2,0.9), "horizontal FOV excludes target");
  check(!in_view({0,2,1},1.2,0.9), "vertical FOV excludes target");
  check(!in_view({0,0,0.1},1.2,0.9), "too close to measure depth");
  check(!scenario("occlusion",12.5).detected, "one second occlusion");
  check(!scenario("occlusion",23).detected, "two second occlusion");
  check(!scenario("occlusion",36).depth, "three second occlusion");
  check(scenario("occlusion",38).detected, "target returns after occlusion");
  check(scenario("invalid_depth",23).detected && !scenario("invalid_depth",23).depth, "detection separated from depth");
  check(scenario("away",20).x>scenario("away",5).x, "away trajectory");
  check(scenario("approach",25).x<scenario("approach",10).x, "approaching trajectory");
  const auto closest=scenario("approach",25);
  check(closest.x-1.0<1.45, "approach reaches close-distance holding test after initial stop");
  check(in_view(to_optical(closest,1.0,0,0,0.25,1.0),1.2,0.9),
    "close-distance test remains visible after initial stop");
  check(scenario("approach",50).x>scenario("approach",30).x, "resume trajectory");
  check(scenario("turn",20).y>0, "turn trajectory");
  check(scenario("demo",65.1).x==scenario("demo",64.9).x &&
    scenario("demo",65.1).y==scenario("demo",64.9).y, "demo ends without teleporting target");
  bool threw=false; try {scenario("typo",0);} catch(const std::invalid_argument &) {threw=true;}
  check(threw,"unknown scenario rejected");
  std::cout<<count<<" scenario checks passed\n";
}
