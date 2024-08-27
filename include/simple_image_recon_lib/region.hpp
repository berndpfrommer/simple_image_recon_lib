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

#ifndef SIMPLE_IMAGE_RECON_LIB__REGION_HPP_
#define SIMPLE_IMAGE_RECON_LIB__REGION_HPP_
#include <cstdint>
#include <queue>
#include <simple_image_recon_lib/event.hpp>
#include <simple_image_recon_lib/pixel.hpp>
#include <simple_image_recon_lib/tile.hpp>
#include <stdexcept>
#include <tuple>
#include <vector>

namespace simple_image_recon_lib
{
class Region
{
public:
  Region() = default;
  void update(uint32_t t, int8_t polarity, Pixel * pixel, Tile * tile)
  {
    (void)t;
    (void)polarity;
    if (!pixel->isActive()) {
      num_occupied_pixels_++;
      if (tile->getNumPixActive() == 0) {
        num_occupied_tiles_++;  // first active pixel in this tile
      }
      tile->incNumPixActive();  // bump number of active pixels in this tile
    }
    pixel->incNumEventsInQueue();
    events_.push({pixel, tile});
    processEventQueue();  // adjusts size of event window
  }

private:
  void processEventQueue()
  {
    while (events_.size() > event_window_size_) {
      auto p = events_.front().first;
      if (!p->isActive()) {
        throw std::runtime_error("inactivating inactive pixel!");
      }
      p->decNumEventsInQueue();
      if (!p->isActive()) {
        auto tile = events_.front().second;
        if (tile->getNumPixActive() == 0) {
          throw std::runtime_error("empty tile!");
        }
        // remove number of pixels in this tile
        tile->decNumPixActive();
        if (tile->getNumPixActive() == 0) {
          num_occupied_tiles_--;
        }
        num_occupied_pixels_--;
      }
      events_.pop();  // remove element now
    }
    // adjust event window size up or down to match the fill ratio:
    // new_size = old_size * current_fill_ratio / desired_fill_ratio
    // The idea is that as the event window increases, the features will "fill out"
    event_window_size_ = (event_window_size_ * num_occupied_tiles_ * fill_ratio_denom_) /
                         (num_occupied_pixels_ * fill_ratio_num_);
  }

  // ------------------- variables ------------------
  std::queue<std::pair<Pixel *, Tile *>> events_;  // queue with buffered events
  uint32_t num_occupied_pixels_{0};
  uint32_t num_occupied_tiles_{0};
  size_t event_window_size_{0};
  static constexpr uint32_t fill_ratio_denom_{2};
  static constexpr uint32_t fill_ratio_num_{1};
};
}  // namespace simple_image_recon_lib
#endif  // SIMPLE_IMAGE_RECON_LIB__REGION_HPP_
