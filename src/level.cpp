// -*-c++-*--------------------------------------------------------------------
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

#include <array>
#include <iostream>
#include <simple_image_recon_lib/level.hpp>

namespace simple_image_recon_lib
{
Level::Level(
  uint16_t pixel_size_x, uint16_t pixel_size_y, uint16_t num_pixels_x, uint16_t num_pixels_y,
  uint16_t num_tiles_x, uint16_t num_tiles_y, uint16_t num_regions_x, uint16_t num_regions_y)
: pixel_size_x_(pixel_size_x),
  pixel_size_y_(pixel_size_y),
  num_pixels_x_(num_pixels_x),
  num_pixels_y_(num_pixels_y),
  num_tiles_x_(num_tiles_x),
  num_tiles_y_(num_tiles_y),
  num_regions_x_(num_regions_x),
  num_regions_y_(num_regions_y)
{
  regions_.resize(num_regions_x * num_regions_y);
  tiles_.resize(num_tiles_x * num_tiles_y);
  pixels_.resize(num_pixels_x * num_pixels_y);
}

static size_t peel_off_factor(uint16_t x)
{
  if (x != 1) {
    const std::array<int, 6> factors{{2, 3, 5, 7, 11, 13}};
    for (int f : factors) {
      if (x % f == 0) {
        return (x / f);
      }
    }
    std::cout << "cannot find prime factor for image!" << std::endl;
    throw(std::runtime_error("weird image resolution!"));
  }
  return (x);
}

Level::SharedPtr Level::make(
  size_t idx, uint16_t * w, uint16_t * h, uint16_t full_width, uint16_t full_height)
{
  if (*w == 1 && *h == 1) {
    return (nullptr);
  }
  const auto num_pixels_x = peel_off_factor(*w);
  const auto pixel_size_x = full_width / num_pixels_x;
  const auto num_tiles_x = peel_off_factor(num_pixels_x);
  const auto num_regions_x = peel_off_factor(num_tiles_x);
  const auto num_pixels_y = peel_off_factor(*h);
  const auto pixel_size_y = full_height / num_pixels_y;
  const auto num_tiles_y = peel_off_factor(num_pixels_y);
  const auto num_regions_y = peel_off_factor(num_tiles_y);
  *w = num_regions_x;
  *h = num_regions_y;
  return (std::make_shared<Level>(
    pixel_size_x, pixel_size_y, num_pixels_x, num_pixels_y, num_tiles_x, num_tiles_y, num_regions_x,
    num_regions_y));
}

std::ostream & operator<<(std::ostream & os, const Level & lev)
{
  os << "pix_sz: " << lev.pixel_size_x_ << "x" << lev.pixel_size_y_
     << " num_px: " << lev.num_pixels_x_ << "x" << lev.num_pixels_y_
     << " num_ti: " << lev.num_tiles_x_ << "x" << lev.num_tiles_y_
     << " num_rg: " << lev.num_regions_x_ << "x" << lev.num_regions_y_;
  return (os);
}
}  // namespace simple_image_recon_lib
