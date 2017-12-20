// Standard header files
#include <iostream>

// SMP HEADER FILES ------
#include <smp/components/collision_checkers/standard.hpp>
#include <smp/components/distance_evaluators/kdtree.hpp>
#include <smp/components/extenders/dubins.hpp>
#include <smp/components/multipurpose/minimum_time_reachability.hpp>
#include <smp/components/samplers/uniform.hpp>

#include <smp/planners/rrtstar.hpp>

#include <smp/planner_utils/trajectory.hpp>

// PARAMETERS TO THE PROBLEM
// ***********************************************************************************
// *
#define NUM_DIMENSIONS                                                         \
  3 // Change the number of dimensions from here. Scale it up to 6 - 10
    //   dimensions to see the convergence of RRT* towards an optimal solution
    //   in very high dimensional configuration spaces without employing any
    //   heuristics.

#define EXTENSION_LENGTH                                                       \
  10.0 // Maximum length of an extension. This parameter should ideally
       //   be equal longest straight line from the initial state to
       //   anywhere in the state space. In other words, this parameter
       //   should be "sqrt(d) L", where d is the dimensionality of space
//   and L is the side length of a box containing the obstacle free space.
//   NOTE: Smaller values of this parameter will lead to a good feasible
//   solution very quickly, whilenot affecting the asymptotic optimality
//   property of the RRT* algorithm.
// *
// *************************************************************************************************************

// SMP TYPE DEFINITIONS -------
// State, input, vertex_data, and edge_data definitions
typedef smp::state_dubins StateDubins;
typedef smp::input_dubins InputDubins;
typedef smp::minimum_time_reachability_vertex_data vertex_data_t;
typedef smp::minimum_time_reachability_edge_data edge_data_t;

// Create the typeparams structure
typedef struct _typeparams {
  typedef StateDubins state;
  typedef InputDubins input;
  typedef vertex_data_t vertex_data;
  typedef edge_data_t edge_data;
} typeparams;

// Define the trajectory type
typedef smp::trajectory<typeparams> trajectory_t;

// Define all planner component types
typedef smp::sampler_uniform<typeparams, NUM_DIMENSIONS> UniformSampler;
typedef smp::distance_evaluator_kdtree<typeparams, NUM_DIMENSIONS>
    KDTreeDistanceEvaluator;
typedef smp::extender_dubins<typeparams> ExtenderDubins;
typedef smp::collision_checker_standard<typeparams, NUM_DIMENSIONS>
    CollisionCheckerStandard;
typedef smp::minimum_time_reachability<typeparams, NUM_DIMENSIONS>
    MinimumTimeReachability;

// Define all algorithm types
typedef smp::rrtstar<typeparams> RRTStar;

int main() {

  using std::cout;
  using std::endl;

  // 1. CREATE PLANNING OBJECTS

  // 1.a Create the components
  UniformSampler sampler;
  KDTreeDistanceEvaluator distance_evaluator;
  ExtenderDubins extender;
  CollisionCheckerStandard collision_checker;
  MinimumTimeReachability min_time_reachability;

  // 1.b Create the planner algorithm -- Note that the min_time_reachability
  // variable acts both
  //                                       as a model checker and a cost
  //                                       evaluator.
  RRTStar planner(sampler, distance_evaluator, extender, collision_checker,
                    min_time_reachability, min_time_reachability);

  planner.parameters.set_phase(
      2); // The phase parameter can be used to run the algorithm as an RRT,
          // See the documentation of the RRG algorithm for more information.

  planner.parameters.set_gamma(
      35.0); // Set this parameter should be set at least to the side length of
             //   the (bounded) state space. E.g., if the state space is a box
             //   with side length L, then this parameter should be set to at
  //   least L for rapid and efficient convergence in trajectory space.
  planner.parameters.set_dimension(NUM_DIMENSIONS);
  planner.parameters.set_max_radius(EXTENSION_LENGTH);

  // 2. INITALIZE PLANNING OBJECTS
  // 2.a Initialize the sampler
  smp::region<NUM_DIMENSIONS> sampler_support;
  for (int i = 0; i < NUM_DIMENSIONS; i++) {
    sampler_support.center[i] = 0.0;
    sampler_support.size[i] = 20.0;
  }
  sampler.set_support(sampler_support);

  // 2.b Initialize the distance evaluator
  //     Nothing to initialize. One could change the kdtree weights.

  // 2.c Initialize the extender

  // 2.d Initialize the collision checker
  smp::region<NUM_DIMENSIONS> obstacle_new;
  for (int i = 0; i < 2; i++) {
    obstacle_new.center[i] = 5.0;
    obstacle_new.size[i] = 5.0;
  }
  obstacle_new.center[1] = 4.0;
  for (int i = 2; i < NUM_DIMENSIONS; i++) {
    obstacle_new.center[i] = 0.0;
    obstacle_new.size[i] = 20.0;
  }
  collision_checker.add_obstacle(obstacle_new);

  for (int i = 0; i < 2; i++) {
    obstacle_new.center[i] = -5.0;
    obstacle_new.size[i] = 5.0;
  }
  collision_checker.add_obstacle(obstacle_new);

  obstacle_new.center[0] = 5.0;
  obstacle_new.center[1] = -5.0;
  for (int i = 0; i < 2; i++) {
    obstacle_new.size[i] = 5.0;
  }
  if (NUM_DIMENSIONS >= 3) {
    obstacle_new.center[2] = 7.0;
    obstacle_new.size[2] = 5.0;
  }
  collision_checker.add_obstacle(obstacle_new);
  if (NUM_DIMENSIONS >= 3) {
    obstacle_new.center[2] = -7.0;
    obstacle_new.size[2] = 5.0;
  }
  collision_checker.add_obstacle(obstacle_new);

  obstacle_new.center[0] = -5.0;
  obstacle_new.center[1] = 5.0;
  obstacle_new.size[0] = 10.0;
  obstacle_new.size[1] = 5.0;
  if (NUM_DIMENSIONS >= 3) {
    obstacle_new.center[2] = 0.0;
    obstacle_new.size[2] = 10.0;
  }
  collision_checker.add_obstacle(obstacle_new);

  // 2.e Initialize the model checker and the cost evaluator (with
  // minimum_time_reachability).
  smp::region<NUM_DIMENSIONS> region_goal;
  for (int i = 0; i < 2; i++) {
    region_goal.center[i] = 8.0;
    region_goal.size[i] = 2.0;
  }
  for (int i = 2; i < NUM_DIMENSIONS; i++) {
    region_goal.center[i] = 0.0;
    region_goal.size[i] = 20.0;
  }
  min_time_reachability.set_goal_region(region_goal);

  // 2.f Initialize the planner
  StateDubins *state_initial = new StateDubins;
  for (int i = 0; i < NUM_DIMENSIONS; i++) {
    state_initial->state_vars[i] = 0.0;
  }
  planner.initialize(state_initial);

  // 3. RUN THE PLANNER
  for (int i = 0; i < 5000; i++) {
    planner.iteration();

    if (i % 100 == 0) {
      cout << "Iteration : " << i << endl;
    }
  }

  // 4. GET THE RESULTS
  trajectory_t trajectory_final;
  min_time_reachability.get_solution(trajectory_final);

  return 1;
}
