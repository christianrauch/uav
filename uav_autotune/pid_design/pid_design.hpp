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
 * @file pid_design.hpp
 * @brief Helper functions to design PID controllers using identified models
 *
 * @author Mathieu Bresciani <mathieu@auterion.com>
 */

#pragma once

#include <Eigen/Core>
#include <cfloat>

namespace pid_design
{

/*
 * Compute a set of PID gains using General Minimum Variance Control law design
 *
 * @param num: the coefficients of the numerator of the discrete-time plant
 * transfer function
 * @param den: the coefficients of the denominator of the discrete-time plant
 * transfer function
 * @param dt: the sampling time in seconds
 * @param sigma: the desired closed-loop rise time in seconds
 * @param delta: the damping index (between 0 and 2). 0 is critical damping, 1
 * is Butterworth
 * @param lbda: the "detuning" coefficients. This affects the gain of the
 * controller kc only (increase to detune the controller)
 *
 * @return The PID gains in standard form: u = kc*[1 + ki*dt + kd/dt]*e
 *         kc: the controller gain
 *         ki: the integral gain (= 1/Ti)
 *         kd: the derivative gain (= Td)
 *
 * Reference:
 * T.Yamatoto, K.Fujii and M.Kaneda, Design and implementation of a self-tuning
 * pid controller, 1998
 */

Eigen::Vector3f computePidGmvc(
  const Eigen::Vector3f & num, const Eigen::Vector3f & den, float dt, float sigma = 0.1f,
  float delta = 1.f, float lbda = 0.5f);
}  // namespace pid_design
