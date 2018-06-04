#include <smp/collision_checkers/multiple_circles_mrpt.hpp>
#include <smp/collision_checkers/standard.hpp>

#include <smp/extenders/dubins.hpp>

int main() {
  smp::collision_checkers::MultipleCirclesMRPT<smp::StateDubins,
                                               smp::InputDubins>
      collision_checker_mrpt_dubins;
  smp::collision_checkers::Standard<smp::StateDubins, smp::InputDubins, 3>
      collision_checker_standard_dubins;

  return 0;
}
