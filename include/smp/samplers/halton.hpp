/*! \file components/samplers/halton.h
  \brief The halton deterministic sampler

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

#include <smp/external_libraries/halton/halton.h>
#include <smp/region.hpp>
#include <smp/samplers/base.hpp>

namespace smp {
namespace samplers {
//! Implements the sampler components that relies on halton sampling.
/*!
  A sampler component that implements halton sampling.

  \ingroup samplers
*/
template <class State, int NUM_DIMENSIONS> class Halton : public Base<State> {

  using region_t = Region<NUM_DIMENSIONS>;

  region_t support;

public:
  Halton() {
    // Initialize the sampling distribution support.
    for (int i = 0; i < NUM_DIMENSIONS; i++) {
      support.center[i] = 0.0;
      support.size[i] = 1.0;
    }

    // Initialize the halton sequence dimension
    halton_dim_num_set(NUM_DIMENSIONS);
  }

  ~Halton() {}

  int sample(State **state_sample_out) {

    if (NUM_DIMENSIONS <= 0)
      return 0;

    State *state_new = new State;

    double halton_sample[NUM_DIMENSIONS];

    halton(halton_sample);

    // Generate an independent random variable for each axis.
    for (int i = 0; i < NUM_DIMENSIONS; i++)
      (*state_new)[i] = support.size[i] * halton_sample[i] -
                        support.size[i] / 2.0 + support.center[i];

    *state_sample_out = state_new;

    return 1;
  }

  /**
   * \brief Sets the dimensions and position of the rectangular bounding box of
   *        the support.
   *
   * Halton distribution only makes sense in a bounded support, which can be set
   * using this function. This sampler function only draws samples from a
   * rectangular
   * box in the Euclidean space with dimensions NUM_DIMENSIONS, which is a
   * template
   * parameter to the Halton sampler class. If the support variable is not set,
   * i.e.,
   * this function is never called, then the support is initialized to the unit
   * cube
   * centered at the origin by default.
   *
   * @param support_in New support for the Halton sampling distribution.
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
