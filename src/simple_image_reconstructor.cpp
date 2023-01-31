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

#include "simple_image_recon_lib/simple_image_reconstructor.hpp"

#include <cmath>
#include <iostream>
#include <limits>

namespace simple_image_recon_lib
{
static void compute_alpha_beta(const double T_cut, double * alpha, double * beta)
{
  // compute the filter coefficients alpha and beta (see frequency cam paper)
  const double omega_cut = 2 * M_PI / T_cut;
  const double phi = 2 - std::cos(omega_cut);
  *alpha = (1.0 - std::sin(omega_cut)) / std::cos(omega_cut);
  *beta = phi - std::sqrt(phi * phi - 1.0);  // see paper
}

void SimpleImageReconstructor::initialize(
  size_t width, size_t height, uint32_t cutoffTime, uint32_t tileSize, double fillRatio)
{
  width_ = width;
  height_ = height;
  tileSize_ = tileSize;
  // compute filter coefficients
  double alpha(0);
  double beta(0);
  compute_alpha_beta(static_cast<double>(cutoffTime), &alpha, &beta);
  c_[0] = alpha;
  c_[1] = 1.0 - alpha;
  c_[2] = beta;
  c_[3] = 0.5 * (1 + beta);
  state_.resize(width * height, State());
  tileStrideY_ = width * tileSize;
  constexpr int maxArea = (1 << ACTIVITY_LOW_BIT);
  if (tileSize * tileSize > maxArea) {
    // guard against overflow of count of occupied pixels in tile
    std::cerr << "activity tile size too big: " << tileSize << " must be < "
              << static_cast<int>(std::sqrt(maxArea)) << std::endl;
    throw(std::runtime_error("activity tile size too big"));
  }
  setFillRatio(fillRatio);
}

void SimpleImageReconstructor::getImage(uint8_t * img, size_t stride) const
{
  // find min and max for normalization
  float minL = std::numeric_limits<float>::max();
  float maxL = std::numeric_limits<float>::min();
  for (size_t i = 0; i < height_ * width_; i++) {
    if (state_[i].getL() > maxL) {
      maxL = state_[i].getL();
    }
    if (state_[i].getL() < minL) {
      minL = state_[i].getL();
    }
  }
  // copy image over

  const float scale = 255.0F / (maxL - minL);
  for (size_t iy = 0; iy < height_; iy++) {
    const size_t y_off = iy * stride;
    const size_t y_off_state = iy * width_;
    for (size_t ix = 0; ix < width_; ix++) {
      const auto & s = state_[y_off_state + ix];
      img[y_off + ix] = static_cast<uint8_t>((s.getL() - minL) * scale);
    }
  }
}

void SimpleImageReconstructor::setFillRatio(double fill_ratio)
{
  std::cout << "setting fill ratio: " << fill_ratio << std::endl;
  fillRatioDenom_ = 100;
  // how many tiles per pixel when fully filled
  const double tiles_per_pixel = 1.0 / (tileSize_ * tileSize_);
  // a fill ratio below 1 pixel per tile is not achievable
  const double r = std::min(1.0, std::max(fill_ratio, tiles_per_pixel + 1e-3));
  const double nt_np = tiles_per_pixel / r;
  fillRatioNum_ = static_cast<uint64_t>(nt_np * fillRatioDenom_);
  std::cout << "fill ratio num: " << fillRatioNum_ << std::endl;
  std::cout << "fill ratio denom: " << fillRatioDenom_ << std::endl;
}

}  // namespace simple_image_recon_lib
