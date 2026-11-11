/****************************************************************************
 *
 *   Copyright (c) 2019-2020 PX4 Development Team. All rights reserved.
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
 * @file AlphaFilter.hpp
 *
 * @brief First order "alpha" IIR digital filter also known as leaky integrator
 * or forgetting average.
 *
 * @author Mathieu Bresciani <brescianimathieu@gmail.com>
 * @author Matthias Grob <maetugr@gmail.com>
 */

#include "AlphaFilter.hpp"

#include <cfloat>
#include <cmath>
#include <numbers>

// instantiate template for specific types
template class AlphaFilter<float>;
template class AlphaFilter<Eigen::Vector3f>;
template class AlphaFilter<Eigen::Quaternionf>;

static constexpr float M_TWOPI_F = 2 * std::numbers::pi_v<float>;

template <typename T>
AlphaFilter<T>::AlphaFilter() = default;

template <typename T>
AlphaFilter<T>::AlphaFilter(float sample_interval, float time_constant)
{
  setParameters(sample_interval, time_constant);
}

template <typename T>
AlphaFilter<T>::AlphaFilter(float time_constant) : _time_constant(time_constant){};

template <typename T>
AlphaFilter<T>::~AlphaFilter() = default;

/**
 * Set filter parameters for time abstraction
 *
 * Both parameters have to be provided in the same units.
 *
 * @param sample_interval interval between two samples in seconds
 * @param time_constant filter time constant determining convergence in
 * seconds
 */
template <typename T>
void AlphaFilter<T>::setParameters(float sample_interval, float time_constant)
{
  const float denominator = time_constant + sample_interval;

  if (denominator > FLT_EPSILON)
  {
    setAlpha(sample_interval / denominator);
  }

  _time_constant = time_constant;
}

template <typename T>
bool AlphaFilter<T>::setCutoffFreq(float sample_freq, float cutoff_freq)
{
  if (
    (sample_freq <= 0.f) || (cutoff_freq <= 0.f) || (cutoff_freq >= sample_freq / 2.f) ||
    !std::isfinite(sample_freq) || !std::isfinite(cutoff_freq))
  {
    // Invalid parameters
    return false;
  }

  setParameters(1.f / sample_freq, 1.f / (M_TWOPI_F * cutoff_freq));
  return true;
}

template <typename T>
void AlphaFilter<T>::setCutoffFreq(float cutoff_freq)
{
  if (cutoff_freq > FLT_EPSILON)
  {
    _time_constant = 1.f / (M_TWOPI_F * cutoff_freq);
  }
  else
  {
    _time_constant = 0.f;
  }
}

/**
 * Set filter parameter alpha directly without time abstraction
 *
 * @param alpha [0,1] filter weight for the previous state. High value - long
 * time constant.
 */
template <typename T>
void AlphaFilter<T>::setAlpha(float alpha)
{
  _alpha = alpha;
}

/**
 * Set filter state to an initial value
 *
 * @param sample new initial value
 */
template <typename T>
void AlphaFilter<T>::reset(const T & sample)
{
  _filter_state = sample;
}

/**
 * Add a new raw value to the filter
 *
 * @return retrieve the filtered result
 */
template <typename T>
const T & AlphaFilter<T>::update(const T & sample)
{
  _filter_state = updateCalculation(sample);
  return _filter_state;
}

template <typename T>
const T AlphaFilter<T>::update(const T & sample, float dt)
{
  setParameters(dt, _time_constant);
  return update(sample);
}

template <typename T>
const T & AlphaFilter<T>::getState() const
{
  return _filter_state;
}

template <typename T>
float AlphaFilter<T>::getCutoffFreq() const
{
  return 1.f / (M_TWOPI_F * _time_constant);
}

template <typename T>
T AlphaFilter<T>::updateCalculation(const T & sample)
{
  return _filter_state + _alpha * (sample - _filter_state);
}

/* Specialization for 3D rotations
 * The filter is computed on the 3-sphere of unit quaternions instead of the
 * cartesian space Additions and subtractions are done using the quaternion
 * multiplication and the error is scaled on the tangent space.
 */
template <>
Eigen::Quaternionf AlphaFilter<Eigen::Quaternionf>::updateCalculation(
  const Eigen::Quaternionf & sample)
{
  Eigen::Quaternionf q_error(_filter_state.conjugate() * sample);

  if (q_error.w() < 0.f)
  {
    q_error.coeffs() *= -1.f;
  }

  const Eigen::AngleAxisf error_angle_axis(q_error);
  return _filter_state *
         Eigen::Quaternionf(
           Eigen::AngleAxisf(_alpha * error_angle_axis.angle(), error_angle_axis.axis()));
}
