// -*-c++-*--------------------------------------------------------------------
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

#include <array>
#include <iostream>
#include <memory>
#include <simple_image_recon_lib/time_keeper.hpp>

namespace simple_image_recon_lib
{
void TimeKeeper::initialize(uint16_t width, uint16_t height)
{
  uint16_t w = width;
  uint16_t h = height;
  std::shared_ptr<Level> level;
  size_t level_idx = 0;
  do {
    level = Level::make(level_idx++, &w, &h, width, height);
    if (level) {
      std::cout << *level << std::endl;
      levels_.push_back(level);
    }
  } while (level);
}
}  // namespace simple_image_recon_lib
