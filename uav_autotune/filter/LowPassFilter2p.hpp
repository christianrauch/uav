/****************************************************************************
 *
 *   Copyright (C) 2012-2021 PX4 Development Team. All rights reserved.
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

/// @file	LowPassFilter2p.hpp
/// @brief	A class to implement a second order low pass filter
/// Author: Leonard Hall <LeonardTHall@gmail.com>
/// Adapted for PX4 by Andrew Tridgell

#pragma once

#include <Eigen/Core>

template <typename T>
class LowPassFilter2p
{
public:
  // LowPassFilter2p() = default;
  LowPassFilter2p();

  LowPassFilter2p(float sample_freq, float cutoff_freq);

  // Change filter parameters
  void set_cutoff_frequency(float sample_freq, float cutoff_freq);

  /**
   * Add a new raw value to the filter
   *
   * @return retrieve the filtered result
   */
  T apply(const T & sample);

  // Filter array of samples in place using the Direct form II.
  void applyArray(T samples[], int num_samples);

  // Return the cutoff frequency
  float get_cutoff_freq() const;

  // Return the sample frequency
  float get_sample_freq() const;

  float getMagnitudeResponse(float frequency) const;

  // Reset the filter state to this value
  template <typename U = T, std::enable_if_t<std::is_floating_point_v<U>, bool> = true>
  T reset(const T & sample);

  template <
    typename U = T, std::enable_if_t<std::is_base_of_v<Eigen::MatrixBase<U>, U>, bool> = true>
  T reset(const T & sample);

  void disable();

protected:
  T _delay_element_1{};  // buffered sample -1
  T _delay_element_2{};  // buffered sample -2

  // All the coefficients are normalized by a0, so a0 becomes 1 here
  float _a1{0.f};
  float _a2{0.f};

  float _b0{1.f};
  float _b1{0.f};
  float _b2{0.f};

  float _cutoff_freq{0.f};
  float _sample_freq{0.f};
};
