/*! \file /components/extenders/state_array_double.h
  \brief An implementation of an state data structure as a double array.

  This file includes a class that provides the implementation of the state
  data structure as a double array.

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

#ifndef _SMP_STATE_ARRAY_DOUBLE_H_
#define _SMP_STATE_ARRAY_DOUBLE_H_

namespace smp {

//! Implementation of the state data structure as a double array.
/*!
  This class implements the state data structure as a double array. The
  dimension of the array is a template parameter to the class.

  \ingroup states
*/
template <int NUM_STATES> class StateArrayDouble {

public:
  //! State variables array.
  /*!
    The implementation of the state variables as a double array
    of size NUM_INPUTS, which is the template argument for this class.
  */
  double state_vars[NUM_STATES];

  StateArrayDouble();
  ~StateArrayDouble();

  /**
   * \brief Copy constructor
   */
  StateArrayDouble(const StateArrayDouble<NUM_STATES> &state_in);

  /**
   * \brief Equality operator
   *
   * Two states are equal if and only if all their components are equal. This
   * function checks whether this criterion is satisfied.
   */
  const StateArrayDouble<NUM_STATES> &
  operator=(const StateArrayDouble<NUM_STATES> &state_in);

  /**
   * \brief The bracket operator that returns the given element from the array.
   *
   * The bracket operator returns a reference to the indexed element in the
   * array.
   *
   * @param index_in The index of the state variable.
   *
   * @returns Returns a reference to the state variable with index index_in.
   */
  inline double &operator[](int index_in);
};
}

#endif
