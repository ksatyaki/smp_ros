/*! \file components/samplers/base.h
  \brief The abstract sampler

  The sampler provides random or quasi-random sample states.
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

#include <smp/vertex_edge.hpp>

namespace smp {
namespace samplers {

//! The abstract class that specifies the structure of a sampler component.
/*!
  A sampler component provides random or quasi-random samples of states.
  This abstract class specifies how the sample function should be
  implemented in any derived class.

  \ingroup samplers_base
*/
template <class State> class Base {

public:
  virtual ~Base(){};
  /**
   * \brief Provides a sample state from the state space.
   *
   * This function creates (by allocating the memory) for a new state
   * that is sampled (randomly or quasi-randomly) from the state space.
   * It returns a pointer to the new state.
   *
   * @param state_sample_out A pointer to the state that will be returned.
   *                         This variable can be set to, e.g., the address
   *                         of a null pointer.
   *
   * @returns Returns 1 for success, a non-positive number for failure.
   */
  virtual int sample(State **state_sample_out) = 0;
};
} // namespace samplers
} // namespace smp
