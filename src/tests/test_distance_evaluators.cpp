#include <smp/distance_evaluators/kdtree.hpp>
#include <smp/extenders/dubins.hpp>

int main() {
  smp::distance_evaluators::KDTree<smp::StateDubins, smp::InputDubins, double, double, 3> de;

  return 0;
}
