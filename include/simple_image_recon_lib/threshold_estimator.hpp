// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2025 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#ifndef SIMPLE_IMAGE_RECON_LIB_THRESHOLD_ESTIMATOR_HPP
#define SIMPLE_IMAGE_RECON_LIB_THRESHOLD_ESTIMATOR_HPP

#include <cstdint>
#include <simple_image_recon_lib/state.hpp>
#include <vector>

namespace simple_image_recon_lib
{
class ThresholdEstimator
{
public:
  ThresholdEstimator() = default;

  void initialize(
    uint16_t sensor_width, uint16_t sensor_height, uint16_t region_width, uint16_t region_height,
    float mix_coeff)
  {
    region_width_ = region_width;
    region_height_ = region_height;
    stride_ = sensor_width / region_width;
    count_.resize(stride_ * (sensor_height / region_height_), 0);
    std::cout << "init TE with: " << region_width_ << " x " << region_height_ << " s: " << stride_
              << " mc: " << mix_coeff_ << std::endl;
    // the factor of 0.5 is to get the threshold to be approximately 1,
    // since we are counting ON and OFF events
    normalizer_ = 0.5 / (region_width_ * region_height_);
    mix_coeff_ = mix_coeff;
    one_minus_mix_coeff_ = 1.0 - mix_coeff_;
  }

  inline uint64_t updateThreshold(State * s, uint8_t p, uint16_t ex, uint16_t ey)
  {
    const auto count = count_[ex / region_width_ + (ey / region_height_) * stride_]++;
    const auto d_n = (count - s->getLastCount(p)) * normalizer_;
#if 0    
    if (ex == 320 && ey == 240 && p == 0) {
      std::cout << s->getLastCount() << " " << count << " " << d_n << std::endl;
    }
#endif
    s->updateThreshold(p, one_minus_mix_coeff_, mix_coeff_ * d_n);
    s->setLastCount(p, count);
    return (count);
  }

private:
  std::vector<uint64_t> count_;
  uint16_t region_width_{0};
  uint16_t region_height_{0};
  uint32_t stride_{0};
  float normalizer_{0};
  float mix_coeff_{1.0};
  float one_minus_mix_coeff_{0};
};
}  // namespace simple_image_recon_lib
#endif  // SIMPLE_IMAGE_RECON_LIB_THRESHOLD_ESTIMATOR_HPP
