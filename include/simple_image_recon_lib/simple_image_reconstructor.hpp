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
#include <iostream>
#include <queue>
#include <vector>

#include "simple_image_recon_lib/spatial_filter.hpp"

namespace simple_image_recon_lib
{
class SimpleImageReconstructor
{
public:
  typedef float state_t;
  static constexpr std::array<state_t, 9> GAUSSIAN_3x3 = {0.0625, 0.125,  0.0625, 0.125, 0.25,
                                                          0.125,  0.0625, 0.125,  0.0625};
  class State
  {
  public:
    explicit State(state_t L_a = 0, state_t L_lag_a = 0, int8_t p_a = 0, uint8_t pc = 0)
    : L(L_a), L_lag(L_lag_a), p(p_a), pixelCount(pc)
    {
    }
    inline void operator+=(const State & s)
    {
      L += s.L;
      L_lag += s.L_lag;
      // leave other fields untouched
    }

    inline State operator*(const float c) const { return (State(c * L, c * L_lag)); }

    inline state_t getL() const { return (L); }
    inline state_t getL_lag() const { return (L_lag); }
    inline int8_t getP() const { return (p); }
    inline uint8_t getPixelCount() const { return (pixelCount & ((1 << ACTIVITY_BIT) - 1)); }
    inline bool isActive() const { return ((pixelCount & (1 << ACTIVITY_BIT)) != 0); }
    inline void incPixelCount() { pixelCount++; }
    inline void decPixelCount() { pixelCount--; }
    inline void markActive() { pixelCount |= (1 << ACTIVITY_BIT); }
    inline void markInActive() { pixelCount &= ((1 << ACTIVITY_BIT) - 1); }

    inline void setL(state_t f) { L = f; }
    inline void setL_lag(state_t f) { L_lag = f; }
    inline void setP(int8_t i) { p = i; }

  private:
    // ------ variables -------
    state_t L;
    state_t L_lag;
    int8_t p;
    uint8_t pixelCount{0};
  };

  SimpleImageReconstructor() = default;

  void event(uint16_t ex, uint16_t ey, uint8_t polarity)
  {
    auto & s = state_[ey * width_ + ex];
    int8_t p = static_cast<bool>(polarity) ? 1 : -1;
    // raw change in polarity, will be 0 or +-2
    const auto dp = static_cast<float>(p - s.getP());
    // run the temporal filter
    const auto L = c_[0] * s.getL() + c_[1] * s.getL_lag() + c_[2] * dp;
    // update state
    s.setL_lag(s.getL());
    s.setL(L);
    s.setP(p);
    // run activity detector
    if (!s.isActive()) {
      // only mark pixel as active if it isn't already. Drops duplicate events
      s.markActive();
      // state of top left corner of tile has actual pixel-in-tile count
      auto & tile = state_[getTileIdx(ex, ey)];
      numOccupiedPixels_++;
      events_.push(Event(ex, ey, p));
      if (tile.getPixelCount() == 0) {
        numOccupiedTiles_++;  // no more pixels in this tile
      }
      tile.incPixelCount();  // bump number of pixels in this tile
      updateEventWindowSize();
    }
  }

  void updateEventWindowSize()
  {
    while (events_.size() > eventWindowSize_) {
      const Event & e = events_.front();
      auto & s = state_[e.y() * width_ + e.x()];
      if (!s.isActive()) {
        std::cerr << e.x() << " " << e.y() << " is inactive!" << std::endl;
        throw std::runtime_error("inactivating inactive pixel!");
      }
      s.markInActive();
      // filter state spatially and mark as inactive
      s = spatial_filter::filter_3x3(&state_[0], e.x(), e.y(), width_, height_, GAUSSIAN_3x3);

      // now update tile
      auto & tile = state_[getTileIdx(e.x(), e.y())];  // state of top left corner of tile
      if (tile.getPixelCount() == 0) {
        std::cerr << e.x() << " " << e.y() << " tile is empty!" << std::endl;
        throw std::runtime_error("empty tile!");
      }
      // remove number of pixels in this tile
      tile.decPixelCount();
      if (tile.getPixelCount() == 0) {
        numOccupiedTiles_--;
      }
      numOccupiedPixels_--;
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

  inline size_t getTileIdx(uint16_t ex, uint16_t ey) const
  {
    return ((ey / tileSize_) * tileStrideY_ + (ex / tileSize_) * tileSize_);
  }

private:
  class Event
  {
  public:
    explicit Event(uint16_t x, uint16_t y, int8_t p) : ex(x), ey(y), ep(p) {}
    uint16_t x() const { return (ex); }
    uint16_t y() const { return (ey); }
    int8_t p() const { return (ep); }

  private:
    uint16_t ex;
    uint16_t ey;
    int8_t ep;
  };

  void setFillRatio(double fill_ratio);

  // ------------------- variables ------------------
  size_t width_{0};
  size_t height_{0};
  std::vector<State> state_;  // filter state
  std::array<float, 3> c_{0, 0, 0};  // filter coefficients, see frequency cam paper
  // ---------- related to activity detection
  static constexpr int START_WINDOW_SIZE = 2000;
  uint16_t tileSize_{0};            // width of tiles
  uint16_t tileStrideY_{0};         // size of stride in tiled image
  uint64_t eventWindowSize_{START_WINDOW_SIZE};  // current event window size
  uint64_t fillRatioDenom_{2};      // denominator of fill ratio
  uint64_t fillRatioNum_{1};        // numerator of fill ratio
  uint64_t numOccupiedPixels_{0};   // currently occupied number of pixels
  uint64_t numOccupiedTiles_{0};    // currently occupied number of blocks
  std::queue<Event> events_;        // queue with buffered events
  static constexpr int ACTIVITY_BIT = 7;
};
}  // namespace simple_image_recon_lib
#endif  // SIMPLE_IMAGE_RECON_LIB_SIMPLE_IMAGE_RECONSTRUCTOR_HPP
