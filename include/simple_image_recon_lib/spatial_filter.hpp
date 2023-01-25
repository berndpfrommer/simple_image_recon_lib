// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2022 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SIMPLE_IMAGE_RECON_LIB_SPATIAL_FILTER_HPP
#define SIMPLE_IMAGE_RECON_LIB_SPATIAL_FILTER_HPP

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <queue>
#include <vector>

namespace simple_image_recon_lib
{
namespace spatial_filter
{
static inline size_t idx(uint16_t x, uint16_t y, uint16_t w) { return (y * w + x); }

template <typename T, int N>
static T filter(
  const T * state, uint16_t x, uint16_t y, uint16_t width, uint16_t height,
  const std::array<std::array<float, N>, N> & K)
{
  constexpr int w = N / 2;
  const uint16_t x_min = std::max(0, static_cast<int>(x) - w);
  const uint16_t x_max = std::min(static_cast<int>(x) + w + 1, static_cast<int>(width));
  const uint16_t y_min = std::max(0, static_cast<int>(y) - w);
  const uint16_t y_max = std::min(static_cast<int>(y) + w + 1, static_cast<int>(height));

  T sum(0, 0, 0, state[idx(x, y, width)].getActivity());

  for (uint16_t iy = y_min; iy < y_max; iy++) {
    const int ky = iy - y + w;
    for (uint16_t ix = x_min; ix < x_max; ix++) {
      const int kx = ix - x + w;
      // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
      sum += state[iy * width + ix] * K[ky][kx];
    }
  }
  return (sum);
}

template <typename T>
static T filter_3x3(
  const T * s, uint16_t x, uint16_t y, uint16_t w, uint16_t h,
  const std::array<std::array<float, 3>, 3> & K)
{
  // first initialize the filter sum with the center, from where
  // it also gets its pixel count (but not the activity!)
  const auto cc = K[1][1];
  const auto & center = s[idx(x, y, w)];
  T sum(center.getL() * cc, center.getL_lag() * cc, 0, center.getPixelCount());
  if (x > 0) {            // not at the left boundary
    if (x < w - 1) {      // not at the right boundary
      if (y > 0) {        // not at the top boundary
        if (y < h - 1) {  // at none of the boundaries
          // update entire region
          sum += s[idx(x - 1, y - 1, w)] * K[0][0];
          sum += s[idx(x, y - 1, w)] * K[1][0];
          sum += s[idx(x + 1, y - 1, w)] * K[2][0];
          sum += s[idx(x - 1, y, w)] * K[0][1];
          sum += s[idx(x + 1, y, w)] * K[2][1];
          sum += s[idx(x - 1, y + 1, w)] * K[0][2];
          sum += s[idx(x, y + 1, w)] * K[1][2];
          sum += s[idx(x + 1, y + 1, w)] * K[2][2];
        } else {  // at bottom boundary, but not corner
          sum += s[idx(x - 1, y - 1, w)] * K[0][0];
          sum += s[idx(x, y - 1, w)] * K[1][0];
          sum += s[idx(x + 1, y - 1, w)] * K[2][0];
          sum += s[idx(x - 1, y, w)] * K[0][1];
          sum += s[idx(x + 1, y, w)] * K[2][1];
        }
      } else {  // at top boundary, but not corner
        sum += s[idx(x - 1, y, w)] * K[0][1];
        sum += s[idx(x + 1, y, w)] * K[2][1];
        sum += s[idx(x - 1, y + 1, w)] * K[0][2];
        sum += s[idx(x, y + 1, w)] * K[1][2];
        sum += s[idx(x + 1, y + 1, w)] * K[2][2];
      }
    } else {              // somewhere at the right boundary
      if (y > 0) {        // not at the top boundary
        if (y < h - 1) {  // at the right boundary, but not corner
          sum += s[idx(x - 1, y - 1, w)] * K[0][0];
          sum += s[idx(x, y - 1, w)] * K[1][0];
          sum += s[idx(x - 1, y, w)] * K[0][1];
          sum += s[idx(x - 1, y + 1, w)] * K[0][2];
          sum += s[idx(x, y + 1, w)] * K[1][2];
        } else {  // at bottom right corner
          sum += s[idx(x - 1, y - 1, w)] * K[0][0];
          sum += s[idx(x, y - 1, w)] * K[1][0];
          sum += s[idx(x - 1, y, w)] * K[0][1];
        }
      } else {  // at the top right corner
        sum += s[idx(x - 1, y, w)] * K[0][1];
        sum += s[idx(x - 1, y + 1, w)] * K[0][2];
        sum += s[idx(x, y + 1, w)] * K[1][2];
      }
    }
  } else {              // somewhere at the left boundary
    if (y > 0) {        // not at the top left corner
      if (y < h - 1) {  // at the left boundary, but not at corner
        sum += s[idx(x, y - 1, w)] * K[1][0];
        sum += s[idx(x + 1, y - 1, w)] * K[2][0];
        sum += s[idx(x + 1, y, w)] * K[2][1];
        sum += s[idx(x, y + 1, w)] * K[1][2];
        sum += s[idx(x + 1, y + 1, w)] * K[2][2];
      } else {  // at the bottom left corner
        sum += s[idx(x, y - 1, w)] * K[1][0];
        sum += s[idx(x + 1, y - 1, w)] * K[2][0];
        sum += s[idx(x + 1, y, w)] * K[2][1];
      }
    } else {  // at the top left corner
      sum += s[idx(x + 1, y, w)] * K[2][1];
      sum += s[idx(x, y + 1, w)] * K[1][2];
      sum += s[idx(x + 1, y + 1, w)] * K[2][2];
    }
  }
  return (sum);
}
}  // namespace spatial_filter
}  // namespace simple_image_recon_lib
#endif  // SIMPLE_IMAGE_RECON_LIB_SPATIAL_FILTER_HPP
