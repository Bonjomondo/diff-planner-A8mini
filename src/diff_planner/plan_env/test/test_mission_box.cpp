#include <plan_env/mission_box.h>
#include <cassert>
#include <limits>

int main() {
  MissionBox box;
  assert(!box.excludes(100,100,100));
  box.configure(true, {-2,-3,.3,2,3,2.5});
  assert(!box.excludes(0,0,1));
  for (const auto &p : std::vector<std::vector<double>>{{-2,0,1},{2,0,1},{0,-3,1},{0,3,1},{0,0,.3},{0,0,2.5}})
    assert(box.excludes(p[0],p[1],p[2]));
  assert(box.excludes(std::numeric_limits<double>::quiet_NaN(),0,1));
  for (auto values : std::vector<std::vector<double>>{{},{0,0,0,0,0,0},{-2,-3,.3,2,3,std::numeric_limits<double>::infinity()}}) {
    bool rejected = false;
    try { box.configure(true,values); } catch (const std::runtime_error&) { rejected = true; }
    assert(rejected && box.enabled && !box.excludes(0,0,1));
  }
  box.configure(false,{});
  assert(!box.excludes(100,100,100));
}
