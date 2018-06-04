/*! \file planners/planner_parameters.h
  \brief Classes that handle parameters of the algorithms.

  This file includes classes that handle operations related to the parameters
  of the algorithms.

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

namespace planners {

//! Parameters for the RRG and the RRT* algorithms
/*!
  The RRG and RRT* algorithms share a certain set of common parameters, which
  can
  be handled using this class. The class provides user functions for easy and
  correct handling of the parameters.
*/
class Parameters {

  // Phase parameter of the algorithm.
  int phase{2};

  // Gamma parameter in the Near vertices computation.
  double gamma{20.0};

  // Dimensionality of the space in the Near vertices computation.
  int dimension{3};

  // Maximum radius in the Near vertices computation.
  double max_radius{10.0};

  // The fixed radius parameter of the related feature.
  double fixed_radius{-1.0};

public:
  Parameters(){};
  ~Parameters(){};

  /**
   * @name Algorithm parameter handlers
   */
  //@{

  /**
   * \brief Sets the phase parameter of the algorithm.
   *
   * The RRT* phase parameter tells the algorithm exactly which phases it should
   * execute.
   * Possible values are 0, 1, and 2. If the phase parameter is set to 0, then
   * the RRT*
   * will behave exactly like an RRT. If the phase parameter is set to 1, then
   * the RRT*
   * will find the minimum-cost parent for the newly added vertex in additition
   * to the
   * steps performed in phase 1. If the phase parameter is set to 2, then the
   * RRT* will
   * perform the rewiring operation as well in addition to the operations
   * performed in
   * phase 1. By default, the phase parameter is set to 2.
   *
   * @param phase_in The new phase parameter.
   *
   * @returns Returns 1 for success, and a non-positive number to indicate an
   * error.
   */
  int set_phase(int phase_in) {

    if ((0 <= phase_in) && (phase_in <= 2)) {
      phase = phase_in;
      return 1;
    } else {
      return 0;
    }
  }

  /**
   * \brief Returns the current phase parameter of the algorithm.
   *
   * This function returns the current phase parameter of the algorithm.
   * For a detailed explanation of the phase parameter, see the documentation
   * for the rrtstar<typeparams>::set_phase function.
   *
   * @returns Returns the phase parameter of the algorithm.
   */
  int get_phase() { return phase; }

  /**
   * \brief Sets the gamma parameter used in the Near node computation of the
   * algorithm.
   *
   * The gamma parameter is a fixed number that can be computed from environment
   * variables.
   * The exact value of gamma for a given environment is given in the paper
   * (Karaman and Frazzoli, IJRR'11). This function directly takes the computed
   * number and
   * uses in the computation of the near nodes.
   *
   * @param gamma_in The new gamma number.
   *
   * @returns Returns 1 for success, and a non-positive number to indicate an
   * error.
   */
  int set_gamma(double gamma_in) {

    if (0.0 < gamma_in) {
      gamma = gamma_in;
      return 1;
    } else {
      return 0;
    }
  }

  /**
   * \brief Returns the current gamma number of the algorithm.
   *
   * This function returns the current value of the gamma parameter of the
   * algorithm.
   * For a detailed explanation of the gamma number, see the documentation for
   * the
   * rrtstar<typeparams>::set_gamma function.
   *
   * @returns Returns the phase parameter of the algorithm.
   */
  double get_gamma() { return gamma; }

  /**
   * \brief Sets the dimension parameter used in the Near node computation of
   * the algorithm.
   *
   * The dimension parameter is the dimensionality of the Euclidean space that
   * the
   * states of the system lie in. The calculation of the volume of the ball that
   * is used in computation of the near nodes requires the knowledge of the
   * dimensionality of the Euclidean space that the vertices lie in.
   *
   * @param dimension_in The new dimensionality parameter.
   *
   * @returns Returns 1 for success, and a non-positive number to indicate an
   * error.
   */
  int set_dimension(int dimension_in) {

    if (2 <= dimension_in) {
      dimension = dimension_in;
      return 1;
    } else {
      return 0;
    }
  }

  /**
   * \brief Returns the current dimension parameter of the algorithm.
   *
   * This function returns the current dimension parameter of the algorithm.
   * For a detailed explanation of the dimension parameter, see the
   * documentation
   * for the rrtstar<typeparams>::set_dimension function.
   *
   * @returns Returns the dimension parameter of the algorithm.
   */
  int get_dimension() { return dimension; }

  /**
   * \brief Sets the maximum radius used in the Near node computation of the
   * algorithm.
   *
   * Computation of the near nodes is carried out by considering all the nodes
   * that
   * lie within a ball of certain radius, an upper bound for which is set using
   * this
   * function. This upper bound can be used for practical purposes, e.g., when
   * the
   * extension function returns trajectories that have length bounded by a
   * certain
   * number.
   *
   * @param max_radius_in The new dimensionality parameter.
   *
   * @returns Returns 1 for success, and a non-positive number to indicate an
   * error.
   */
  int set_max_radius(double max_radius_in) {

    if (0 < max_radius_in) {
      max_radius = max_radius_in;
      return 1;
    } else {
      return 0;
    }
  }

  /**
   * \brief Returns the current maximum radius parameter of the algorithm.
   *
   * This function returns the current maximum radius parameter of the
   * algorithm.
   * For a detailed explanation of the maximum radius parameter, see the
   * documentation for the rrtstar<typeparams>::set_max_radius function.
   *
   * @returns Returns the maximum radius parameter of the algorithm.
   */
  double get_max_radius() { return max_radius; }

  /**
   * \brief Sets (or resets) the use of fixed radius parameter, and sets the
   * fixed
   *        radius parameter to the value given by the argument.
   *
   * The computation of the near nodes is carried out by computing the set of
   * all nodes within a certain radius. In certain applications, the user may
   * want to preset this parameter to some value. This parameter can be fixed
   * over the iterations or it can be modified (by the user) before each
   * iteration.
   * This function will set the radius to a fixed value given as an argument.
   * Whenever this function is called with an argument that is a positive real
   * number, the RRT* algorithm will fix the size of the ball to that particular
   * value. If the function is called with a non-positive argument, then the
   * radius
   * of the ball is computed by the RRT* algorithm itself according to the
   * scaling rule provided in the paper (Karaman and Frazzoli, IJRR'11).
   *
   * @param fixed_radius_in The new fixed radius parameter, if the argument is
   * positive.
   *                        Disables the fixed radius feature, otherwise.
   *
   * @returns Returns 1 for success, and a non-positive number to indicate an
   * error.
   */
  int set_fixed_radius(double fixed_radius_in) {

    if (fixed_radius_in > 0.0)
      fixed_radius = fixed_radius_in;
    else
      fixed_radius = -1.0;

    return 1;
  }

  /**
   * \brief Resets the use of fixed radius parameter.
   *
   * The fixed radius feature provided by this class is explained in the
   * documentation
   * of the rrtstar<typeparams>::set_radius function, which can disable this
   * feature
   * when called with a non-positive argument. This function provides a direct
   * call for
   * this purpose.
   *
   * @returns Returns 1 for success, and a non-positive number to indicate an
   * error.
   */
  void reset_fixed_radius() { fixed_radius = -1.0; }

  /**
   * \brief Returns the current radius parameter of the algorithm.
   *
   * This function returns the current fixed_radius parameter of the algorithm.
   * If the algorithm is not using the fixed radius parameter, but instead
   * is computing the radius of the ball itself, then this function call will
   * return the value -1.0.
   *
   * @returns Returns the fixed radius parameter of the algorithm, if fixed
   * radius
   *          is being used. It returns -1.0, otherwise.
   */
  double get_fixed_radius() { return fixed_radius; }

  //@}
};
} // namespace planners
} // namespace smp
