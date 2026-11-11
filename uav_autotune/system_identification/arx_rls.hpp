/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file arx_rls.hpp
 * @brief Efficient recursive weighted least-squares algorithm without matrix
 * inversion
 *
 * Assumes an ARX (autoregressive) model:
 * A(q^-1)y(k) = q^-d * B(q^-1)u(k) + A(q^-1)e(k)
 *
 * with:
 * q^-i  backward shift operator
 * A(q^-1) = 1 + a_1*q^-1 +...+ a_n*q^-n
 * B(q^-1) = b_0 + b_1*q^-1...+ b_m*q^-m
 * n  order of A(q^-1)
 * m  order of B(q^-1)
 * d  delay
 * u  input of the system
 * y  output of the system
 * e  white noise input
 *
 * References:
 * - Identification de systemes dynamiques, D.Bonvin and A.Karimi, epfl, 2011
 *
 * @author Mathieu Bresciani <mathieu@auterion.com>
 */

#pragma once

#include <stddef.h>
#include <Eigen/Core>

template <size_t N, size_t M, size_t D>
class ArxRls final
{
public:
  ArxRls();

  ~ArxRls();

  void setForgettingFactor(float time_constant, float dt);

  void setForgettingFactor(float lambda);

  /*
   * return the vector of estimated parameters
   * [a_1 .. a_n b_0 .. b_m]'
   */
  const Eigen::Vector<float, N + M + 1> & getCoefficients() const;

  const Eigen::Vector<float, N + M + 1> getVariances() const;

  float getInnovation() const;

  const Eigen::Vector<float, N + M + 1> & getDiffEstimate() const;

  void reset(
    const Eigen::Vector<float, N + M + 1> & theta_init = Eigen::Vector<float, N + M + 1>::Zero());

  void update(float u, float y);

private:
  void addInputOutput(float u, float y);

  void shiftRegisters();

  bool isBufferFull() const;

  Eigen::Vector<float, N + M + 1> constructDesignVector() const;

  void fixCovarianceErrors();

  Eigen::Matrix<float, N + M + 1, N + M + 1> _P;
  Eigen::Vector<float, N + M + 1> _theta_hat;
  Eigen::Vector<float, N + M + 1> _diff_theta_hat;
  float _innovation{};
  float _u[M + D + 1]{};
  float _y[N + 1]{};
  unsigned _nb_samples{0};
  float _lambda{1.f};
};
