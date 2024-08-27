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

#ifndef SIMPLE_IMAGE_RECON_LIB__TILE_HPP_
#define SIMPLE_IMAGE_RECON_LIB__TILE_HPP_

#include <cstdint>

namespace simple_image_recon_lib
{
class Tile
{
public:
  Tile() = default;
  auto getNumPixActive() const { return (num_pix_active_); }
  void incNumPixActive() { num_pix_active_++; }
  void decNumPixActive() { num_pix_active_--; }

private:
  uint16_t num_pix_active_{0};
};
}  // namespace simple_image_recon_lib
#endif  // SIMPLE_IMAGE_RECON_LIB__TILE_HPP_
