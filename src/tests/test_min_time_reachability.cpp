#include <smp/multipurpose/minimum_time_reachability.hpp>
#include <smp/extenders/dubins.hpp>

int main() {

  smp::multipurpose::MinimumTimeReachability<StateDubins, InputDubins> mtr;
  return 0;
}
