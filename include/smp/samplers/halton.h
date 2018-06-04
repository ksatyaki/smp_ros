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


#ifndef _SMP_SAMPLER_HALTON_H_
#define _SMP_SAMPLER_HALTON_H_

#include <smp/common/region.h>
#include <smp/components/samplers/base.h>
#include <smp/external_libraries/halton/halton.h>

namespace smp {

//! Implements the sampler components that relies on halton sampling.
/*!
  A sampler component that implements halton sampling.

  \ingroup samplers
*/
template <class typeparams, int NUM_DIMENSIONS>
class sampler_halton : public sampler_base<typeparams> {

  typedef typename typeparams::state state_t;
  typedef typename typeparams::vertex_data vertex_data_t;
  typedef typename typeparams::edge_data edge_data_t;

  typedef vertex<typeparams> vertex_t;
  typedef edge<typeparams> edge_t;

  typedef region<NUM_DIMENSIONS> region_t;

  region_t support;

public:
  sampler_halton();
  ~sampler_halton();

  int sm_update_insert_vertex(vertex_t *vertex_in);

  int sm_update_insert_edge(edge_t *edge_in);

  int sm_update_delete_vertex(vertex_t *vertex_in);

  int sm_update_delete_edge(edge_t *edge_in);

  int sample(state_t **state_sample_out);

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
  int set_support(const region_t support_in);
};
}

#endif
