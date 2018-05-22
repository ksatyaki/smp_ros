/*
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

#ifndef _SMP_REGION_HPP_
#define _SMP_REGION_HPP_

#include <smp/common/region.h>

template <int NUM_DIMENSIONS> smp::region<NUM_DIMENSIONS>::region() {

  for (int i = 0; i < NUM_DIMENSIONS; i++) {
    this->center[i] = 0.0;
    this->size[i] = 0.0;
  }
}

template <int NUM_DIMENSIONS> smp::region<NUM_DIMENSIONS>::~region() {}

template <int NUM_DIMENSIONS>
smp::region<NUM_DIMENSIONS>::region(
    const smp::region<NUM_DIMENSIONS> &region_in) {

  for (int i = 0; i < NUM_DIMENSIONS; i++) {
    this->center[i] = region_in.center[i];
    this->size[i] = region_in.size[i];
  }
}

template <int NUM_DIMENSIONS>
const smp::region<NUM_DIMENSIONS> &smp::region<NUM_DIMENSIONS>::
operator=(const smp::region<NUM_DIMENSIONS> &region_in) {

  if (&region_in != this) {
    for (int i = 0; i < NUM_DIMENSIONS; i++) {
      this->center[i] = region_in.center[i];
      this->size[i] = region_in.size[i];
    }
  }

  return *this;
}

#endif
