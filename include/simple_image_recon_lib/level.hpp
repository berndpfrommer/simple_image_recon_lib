// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2024 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#ifndef SIMPLE_IMAGE_RECON_LIB__LEVEL_HPP_
#define SIMPLE_IMAGE_RECON_LIB__LEVEL_HPP_
#include <cstdint>
#include <iostream>
#include <memory>
#include <simple_image_recon_lib/pixel.hpp>
#include <simple_image_recon_lib/region.hpp>
#include <simple_image_recon_lib/tile.hpp>
#include <tuple>
#include <vector>

namespace simple_image_recon_lib
{
class Level
{
public:
  using SharedPtr = std::shared_ptr<Level>;
  Level(
    uint16_t pixel_size_x, uint16_t pixel_size_y, uint16_t num_pixels_x, uint16_t num_pixels_y,
    uint16_t num_tiles_x, uint16_t num_tiles_y, uint16_t num_regions_x, uint16_t num_regions_y);

  void event(uint32_t t, uint16_t ex, uint16_t ey, uint8_t polarity)
  {
    uint16_t x_p = ex / pixel_size_x_;
    uint16_t y_p = ey / pixel_size_y_;

    const auto x_t = x_p / num_tiles_x_;
    const auto y_t = y_p / num_tiles_y_;

    const auto x_r = x_t / num_regions_x_;
    const auto y_r = y_t / num_regions_y_;
    regions_[y_r * num_regions_x_ + x_r].update(
      t, static_cast<int8_t>(polarity), &pixels_[y_p * num_pixels_x_ + x_p],
      &tiles_[y_t * num_tiles_x_ + x_t]);
  }

  static SharedPtr make(
    size_t idx, uint16_t * width, uint16_t * height, uint16_t full_width, uint16_t full_height);

  friend std::ostream & operator<<(std::ostream & os, const Level & lev);

private:
  // ------------------- variables ------------------
  std::vector<Region> regions_;
  std::vector<Pixel> pixels_;
  std::vector<Tile> tiles_;
  uint16_t pixel_size_x_{0};
  uint16_t pixel_size_y_{0};
  uint16_t num_pixels_x_{0};
  uint16_t num_pixels_y_{0};
  uint16_t num_tiles_x_{0};
  uint16_t num_tiles_y_{0};
  uint16_t num_regions_x_{0};
  uint16_t num_regions_y_{0};
};
std::ostream & operator<<(const std::ostream & os, const Level & lev);
}  // namespace simple_image_recon_lib
#endif  // SIMPLE_IMAGE_RECON_LIB__LEVEL_HPP_
