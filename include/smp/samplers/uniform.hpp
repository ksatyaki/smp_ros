/*! \file components/samplers/uniform.h
  \brief The uniform sampler

  The sampler provides random samples of states that are uniformly distributed
  in
  a bounded region.

  * Copyright (C) 2018 Sertac Karaman
  *
  * This program is free software: you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation, either version 3 of the License, or
  * (at your option) any later version.
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program.  If not, see <http://www.gnu.org/licenses/>
  *
  */

#pragma once

#include <smp/region.hpp>
#include <smp/samplers/base.hpp>

namespace smp {
namespace samplers {

//! Implements the sampler components that relies on uniform sampling.
/*!
  A sampler component that implements uniform sampling.

  \ingroup samplers
*/
template <class State, int NUM_DIMENSIONS> class Uniform : public Base<State> {

  using region_t = Region<NUM_DIMENSIONS>;

  region_t support;

  region_t goal_region;

  double goal_bias{0.0};

public:
  Uniform() {
    // Initialize the sampling distribution support.
    for (int i = 0; i < NUM_DIMENSIONS; i++) {
      support.center[i] = 0.0;
      support.size[i] = 1.0;
    }
  }

  int set_goal_bias(double bias, const Region<NUM_DIMENSIONS> &region_goal) {
    this->goal_bias = bias;
    this->goal_region = region_goal;
    time_t t;
    srand((unsigned)time(&t));
    return 1;
  }

  ~Uniform() {}

  int sample(State **state_sample_out) {

    if (NUM_DIMENSIONS <= 0) {
      return 0;
    }

    State *state_new = new State;

    double randnum1 = double(rand()) / double(RAND_MAX);

    if (goal_bias > randnum1) {
      for (int i = 0; i < NUM_DIMENSIONS; i++)
        (*state_new)[i] = goal_region.size[i] * rand() / (RAND_MAX + 1.0) -
                          goal_region.size[i] / 2.0 + goal_region.center[i];
      *state_sample_out = state_new;
      return 1;
    }

    // Generate an independent random variable for each axis.
    for (int i = 0; i < NUM_DIMENSIONS; i++)
      (*state_new)[i] = support.size[i] * rand() / (RAND_MAX + 1.0) -
                        support.size[i] / 2.0 + support.center[i];

    *state_sample_out = state_new;

    return 1;
  }

  /**
   * \brief Sets the dimensions and position of the rectangular bounding box of
   *        the support.
   *
   * Uniform distribution only makes sense in a bounded support, which can be
   * set
   * using this function. This sampler function only draws samples from a
   * rectangular
   * box in the Euclidean space with dimensions NUM_DIMENSIONS, which is a
   * template
   * parameter to the uniform sampler class. If the support variable is not set,
   * i.e.,
   * this function is never called, then the support is initialized to the unit
   * cube
   * centered at the origin by default.
   *
   * @param support_in New support for the uniform sampling distribution.
   *
   * @returns Returns 1 for success, a non-positive number for failure.
   */
  int set_support(const region_t support_in) {

    support = support_in;

    return 1;
  }
};
} // namespace samplers
} // namespace smp
