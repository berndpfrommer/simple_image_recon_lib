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

#ifndef SIMPLE_IMAGE_RECON_LIB__TIME_KEEPER_HPP_
#define SIMPLE_IMAGE_RECON_LIB__TIME_KEEPER_HPP_
#include <simple_image_recon_lib/level.hpp>
#include <vector>

namespace simple_image_recon_lib
{
class TimeKeeper
{
public:
  TimeKeeper() = default;
  void event(uint32_t t, uint16_t ex, uint16_t ey, uint8_t polarity)
  {
    (void)t;
    for (auto & level : levels_) {
      level->event(t, ex, ey, polarity);
    }
  }

  void initialize(uint16_t width, uint16_t height);
  const std::vector<Level::SharedPtr> & getLevels() const { return (levels_); }

private:
  // ------------------- variables ------------------
  std::vector<Level::SharedPtr> levels_;
};
}  // namespace simple_image_recon_lib
#endif  // SIMPLE_IMAGE_RECON_LIB__TIME_KEEPER_HPP_
