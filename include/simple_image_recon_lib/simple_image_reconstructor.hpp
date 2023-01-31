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

#ifndef SIMPLE_IMAGE_RECON_LIB_SIMPLE_IMAGE_RECONSTRUCTOR_HPP
#define SIMPLE_IMAGE_RECON_LIB_SIMPLE_IMAGE_RECONSTRUCTOR_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <queue>
#include <vector>

#include "simple_image_recon_lib/spatial_filter.hpp"
#include "simple_image_recon_lib/state.hpp"

namespace simple_image_recon_lib
{
class SimpleImageReconstructor
{
public:
  typedef float state_t;
  static constexpr std::array<std::array<state_t, 3>, 3> GAUSSIAN_3x3 = {
    {{0.0625, 0.125, 0.0625}, {0.125, 0.25, 0.125}, {0.0625, 0.125, 0.0625}}};
  static constexpr std::array<std::array<state_t, 5>, 5> GAUSSIAN_5x5 = {
    {{0.003663, 0.01465201, 0.02564103, 0.01465201, 0.003663},
     {0.01465201, 0.05860806, 0.0952381, 0.05860806, 0.01465201},
     {0.02564103, 0.0952381, 0.15018315, 0.0952381, 0.02564103},
     {0.01465201, 0.05860806, 0.0952381, 0.05860806, 0.01465201},
     {0.003663, 0.01465201, 0.02564103, 0.01465201, 0.003663}}};
  static constexpr std::array<std::array<state_t, 3>, 3> FAKE_3x3 = {
    {{0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 0.0}}};

  SimpleImageReconstructor() = default;

  void event(uint32_t t, uint16_t ex, uint16_t ey, uint8_t polarity)
  {
    static std::ofstream event_active("event_active.txt");
    static std::ofstream state_debug("state.txt");
    auto & s = state_[ey * width_ + ex];
    int8_t p = static_cast<bool>(polarity) ? 1 : -1;
    // raw change in polarity, will be 0 or +-2
    const auto dp = static_cast<float>(p - s.getP());
    // run the temporal filter
    const auto L = c_[0] * s.getL() + c_[1] * s.getL_lag() + c_[2] * dp;
    if (ex == 319 && ey == 239) {
      state_debug << t << " " << L << " " << s.getL() << " " << s.getL_lag() << " " << (int)s.getP()
                  << " " << dp << std::endl;
    }
    // update state
    s.setL_lag(s.getL());
    s.setL(L);
    s.setP(p);
    // run activity detector
    if (!s.isActive()) {
      numOccupiedPixels_++;
      // state of top left corner of tile has actual pixel-in-tile count
      auto & tile = state_[getTileIdx(ex, ey)];
      if (tile.getNumPixActive() == 0) {
        numOccupiedTiles_++;  // first active pixel in this tile
      }
      tile.incNumPixActive();  // bump number of pixels in this tile
      // no update if an opposite polarity event is already active
      if (ex == 319 && ey == 239) {
        event_active << t << std::endl;
      }
    }
    s.incNumEventsInQueue();
    events_.push(Event(t, ex, ey, polarity));
    processEventQueue();  // adjusts size of event window
    currentTime_ = t;
  }

  void processEventQueue()
  {
    static std::ofstream wsize("window_size.txt");
    static std::ofstream event_inactive("event_inactive.txt");
    static std::ofstream filtering("filtering.txt");
    while (events_.size() > eventWindowSize_) {
      const Event & e = events_.front();
      auto & s = state_[e.y() * width_ + e.x()];
      if (!s.isActive()) {
        std::cerr << e.x() << " " << e.y() << " is inactive!" << std::endl;
        throw std::runtime_error("inactivating inactive pixel!");
      }
      s.decNumEventsInQueue();
      if (!s.isActive()) {
        State s_old = s;
        // s =  spatial_filter::filter<State, 3>(&state_[0], e.x(), e.y(), width_, height_, GAUSSIAN_3x3);
        s = spatial_filter::filter_3x3(&state_[0], e.x(), e.y(), width_, height_, GAUSSIAN_3x3);
        // s = spatial_filter::filter<State, 5>(&state_[0], e.x(), e.y(), width_, height_, GAUSSIAN_5x5);
        if (e.x() == 319 && e.y() == 239) {
          filtering << currentTime_ << " " << s_old.getL() << " " << s_old.getL_lag() << " "
                    << s.getL() << " " << s.getL_lag() << std::endl;
        }
        auto & tile = state_[getTileIdx(e.x(), e.y())];  // state of top left corner of tile
        if (tile.getNumPixActive() == 0) {
          std::cerr << e.x() << " " << e.y() << " tile " << getTileIdx(e.x(), e.y()) << " is empty!"
                    << std::endl;
          throw std::runtime_error("empty tile!");
        }
        // remove number of pixels in this tile
        tile.decNumPixActive();
        if (tile.getNumPixActive() == 0) {
          numOccupiedTiles_--;
        }
        numOccupiedPixels_--;
        if (e.x() == 319 && e.y() == 239) {
          event_inactive << currentTime_ << " " << e.t() << " " << eventWindowSize_ << std::endl;
        }
      }
      events_.pop();  // remove element now
    }
    // adjust event window size up or down to match the fill ratio:
    // new_size = old_size * current_fill_ratio / desired_fill_ratio
    // The idea is that as the event window increases, the features will "fill out"
    eventWindowSize_ = (eventWindowSize_ * numOccupiedTiles_ * fillRatioDenom_) /
                       (numOccupiedPixels_ * fillRatioNum_);
  }

  void initialize(
    size_t width, size_t height, uint32_t cutoffTime, uint32_t tileSize, double fillRatio);
  void getImage(uint8_t * img, size_t stride) const;
  size_t getWidth() const { return (width_); }
  size_t getHeight() const { return (height_); }

  const std::vector<State> & getState() const { return (state_); }

  size_t getEventWindowSize() const { return (eventWindowSize_); }

  inline size_t getTileIdx(uint16_t ex, uint16_t ey) const
  {
    return ((ey / tileSize_) * tileStrideY_ + (ex / tileSize_) * tileSize_);
  }

private:
  class Event
  {
  public:
    explicit Event(uint32_t t_a, uint16_t x, uint16_t y, int8_t p) : time(t_a), ex(x), ey(y), ep(p)
    {
    }
    uint32_t t() const { return (time); }
    uint16_t x() const { return (ex); }
    uint16_t y() const { return (ey); }
    int8_t p() const { return (ep); }

  private:
    uint32_t time;
    uint16_t ex;
    uint16_t ey;
    int8_t ep;
  };

  void setFillRatio(double fill_ratio);

  // ------------------- variables ------------------
  size_t width_{0};
  size_t height_{0};
  std::vector<State> state_;         // filter state
  std::array<float, 3> c_{0, 0, 0};  // filter coefficients, see frequency cam paper
  // ---------- related to activity detection
  static constexpr int START_WINDOW_SIZE = 2000;
  uint16_t tileSize_{0};                         // width of tiles
  uint16_t tileStrideY_{0};                      // size of stride in tiled image
  uint64_t eventWindowSize_{START_WINDOW_SIZE};  // current event window size
  uint64_t fillRatioDenom_{2};                   // denominator of fill ratio
  uint64_t fillRatioNum_{1};                     // numerator of fill ratio
  uint64_t numOccupiedPixels_{0};                // currently occupied number of pixels
  uint64_t numOccupiedTiles_{0};                 // currently occupied number of blocks
  std::queue<Event> events_;                     // queue with buffered events
  // -------- debugging
  uint32_t currentTime_{0};
  static constexpr uint8_t ACTIVITY_ON_BIT = 6;
  static constexpr uint8_t ACTIVITY_OFF_BIT = 7;
  static constexpr uint8_t ACTIVITY_LOW_BIT = ACTIVITY_ON_BIT;
  static constexpr uint8_t ACTIVITY_ON_MASK = static_cast<uint8_t>(1) << ACTIVITY_ON_BIT;
  static constexpr uint8_t ACTIVITY_OFF_MASK = static_cast<uint8_t>(1) << ACTIVITY_OFF_BIT;
  static constexpr uint8_t ACTIVITY_MASK = ACTIVITY_ON_MASK | ACTIVITY_OFF_MASK;
  static constexpr uint8_t INV_ACTIVITY_MASK = static_cast<uint8_t>(~ACTIVITY_MASK);
  static constexpr uint8_t PIXEL_COUNT_MASK = (1 << ACTIVITY_LOW_BIT) - 1;
};
}  // namespace simple_image_recon_lib
#endif  // SIMPLE_IMAGE_RECON_LIB_SIMPLE_IMAGE_RECONSTRUCTOR_HPP
