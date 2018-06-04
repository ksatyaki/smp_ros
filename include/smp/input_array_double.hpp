/*! \file /components/extenders/input_array_double.h
  \brief An implementation of an input data structure as a double array.

  This file includes a class that provides the implementation of the input
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

#ifndef _SMP_INPUT_ARRAY_DOUBLE_H_
#define _SMP_INPUT_ARRAY_DOUBLE_H_

namespace smp {

//! Implementation of the input data structure as a double array.
/*!
  This class implements the input data structure as a double array. The
  dimension of the array is a template parameter to the class.

  \ingroup inputs
*/
template <int NUM_INPUTS> class InputArrayDouble {

public:
  //! Input variables array.
  /*!
    The implementation of the input variables as a double array
    of size NUM_INPUTS, which is the template argument for this class.
  */
  double input_vars[NUM_INPUTS];

  InputArrayDouble() {

    for (int i = 0; i < NUM_INPUTS; i++)
      input_vars[i] = 0.0;
  }

  ~InputArrayDouble(){};

  /**
   * \brief The copy constructor
   */
  InputArrayDouble(const InputArrayDouble<NUM_INPUTS> &input_in) {

    for (int i = 0; i < NUM_INPUTS; i++)
      input_vars[i] = input_in.input_vars[i];
  }

  /**
   * \brief The equality operator
   *
   * Two inputs are equal if and only if all their components are equal. This
   * function checks whether this criterion is satisfied.
   */
  const InputArrayDouble<NUM_INPUTS> &
  operator=(const InputArrayDouble<NUM_INPUTS> &input_in) {

    if (&input_in != this) {
      for (int i = 0; i < NUM_INPUTS; i++)
        input_vars[i] = input_in.input_vars[i];
    }

    return *this;
  }

  /**
   * \brief The bracket operator that returns the given element from the array.
   *
   * The bracket operator returns a reference to the indexed element in the
   * array.
   *
   * @param index_in The index of the input variable.
   *
   * @returns Returns a reference to the input variable with index index_in.
   */
  inline double &operator[](int index_in) {

    if ((index_in < NUM_INPUTS) && (index_in >= 0))
      return input_vars[index_in];

    // TODO: ideally throw an exception also.
    return input_vars[0];
  }
};
} // namespace smp

#endif
