// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#ifndef SIMPLE_IMAGE_RECON_LIB_SUBTILES_HPP
#define SIMPLE_IMAGE_RECON_LIB_SUBTILES_HPP

#include <cstdint>

#include "simple_image_recon_lib/state.hpp"

namespace simple_image_recon_lib
{
class Subtiles
{
public:
  Subtiles() = default;
  virtual ~Subtiles() = default;
  Subtiles(const Subtiles &) = default;
  Subtiles(Subtiles &&) = default;
  Subtiles & operator=(const Subtiles &) = default;
  Subtiles & operator=(Subtiles &&) = default;

  using state_t = State::state_t;
  virtual State setState(const State & s, uint16_t x, uint16_t y) = 0;
  virtual const State & getState(uint16_t x_t, uint16_t y_t) const = 0;
};
}  // namespace simple_image_recon_lib
#endif  // SIMPLE_IMAGE_RECON_LIB_SUBTILES_HPP
