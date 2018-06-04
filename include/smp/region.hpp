/*  \file region.h
    \brief The standard brute-force collision checker

  This file implements the a region class, which defines a rectangular region
  in the Euclidean space, the dimension of which is a template argument.

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

namespace smp {

//! A rectangular Region in an Euclidean space of prespecified dimension.
/*!
  This class implements a rectangular in an Euclidean space of a certain
  dimension given by a template parameter.
*/
template <int NUM_DIMENSIONS> class Region {
public:
  //! The coordinates of the center of the region.
  double center[NUM_DIMENSIONS];

  //! The size of the region in each dimension.
  double size[NUM_DIMENSIONS];

  Region() {

    for (int i = 0; i < NUM_DIMENSIONS; i++) {
      this->center[i] = 0.0;
      this->size[i] = 0.0;
    }
  }

  ~Region() {}

  /**
   * \brief Copy constructor
   */
  Region(const Region<NUM_DIMENSIONS> &region_in) {

    for (int i = 0; i < NUM_DIMENSIONS; i++) {
      this->center[i] = region_in.center[i];
      this->size[i] = region_in.size[i];
    }
  }

  /**
   * \brief Equality operator
   *
   * Two states are equal if and only if all their components are equal. This
   * function
   * checks whether this criterion is satisfied.
   */
  const Region<NUM_DIMENSIONS> &
  operator=(const Region<NUM_DIMENSIONS> &region_in) {

    if (&region_in != this) {
      for (int i = 0; i < NUM_DIMENSIONS; i++) {
        this->center[i] = region_in.center[i];
        this->size[i] = region_in.size[i];
      }
    }

    return *this;
  }
};
} // namespace smp

