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
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <queue>
#include <vector>

#include "simple_image_recon_lib/activity_tile_layer.hpp"
#include "simple_image_recon_lib/spatial_filter.hpp"
#include "simple_image_recon_lib/state.hpp"
#include "simple_image_recon_lib/subtiles.hpp"

namespace simple_image_recon_lib
{
class SimpleImageReconstructor : public Subtiles
{
public:
  using state_t = State::state_t;
  static constexpr std::array<std::array<state_t, 3>, 3> GAUSSIAN_3x3 = {
    {{0.0625, 0.125, 0.0625}, {0.125, 0.25, 0.125}, {0.0625, 0.125, 0.0625}}};
  static constexpr std::array<std::array<state_t, 5>, 5> GAUSSIAN_5x5 = {
    {{0.003663, 0.01465201, 0.02564103, 0.01465201, 0.003663},
     {0.01465201, 0.05860806, 0.0952381, 0.05860806, 0.01465201},
     {0.02564103, 0.0952381, 0.15018315, 0.0952381, 0.02564103},
     {0.01465201, 0.05860806, 0.0952381, 0.05860806, 0.01465201},
     {0.003663, 0.01465201, 0.02564103, 0.01465201, 0.003663}}};

  SimpleImageReconstructor() = default;
  ~SimpleImageReconstructor() override = default;
  SimpleImageReconstructor(const SimpleImageReconstructor &) = delete;
  SimpleImageReconstructor(SimpleImageReconstructor &&) = delete;
  SimpleImageReconstructor & operator=(const SimpleImageReconstructor &) = delete;
  SimpleImageReconstructor & operator=(SimpleImageReconstructor &&) = delete;

  // called by upper layers to update the state
  // returns state of tile after update
  State setState(const State & ss, uint16_t x_t, uint16_t y_t) override
  {
    if (std::isnan(ss.getL())) {
      std::cerr << "got nan: " << x_t << " " << y_t << std::endl;
      throw std::runtime_error("got nan!");
    }
    auto & tile = tile_[y_t * tileStrideY_ + x_t];
    if (tile.isActive()) {
      return (tile);  // don't force set the state of active tiles
    }
    state_t Lnew(0);
    // update state of all inactive pixels in the tile
    for (uint16_t x = x_t * tileSize_; x < static_cast<uint16_t>((x_t + 1) * tileSize_); x++) {
      for (uint16_t y = y_t * tileSize_; y < static_cast<uint16_t>((y_t + 1) * tileSize_); y++) {
        auto & target = state_[y * width_ + x];
        if (!target.isActive()) {
          target.setL(ss.getL());       // update only this part of the state
          target.setL_last(ss.getL());  // update the last level
          Lnew += ss.getL();
        } else {
          Lnew += target.getL_last();  // use the last value that has been propagated up
        }
      }
    }
    Lnew *= invTileArea_;
    tile.setL(Lnew);
    tile.setL_last(Lnew);
    return (tile);
  }

  // called by upper layers for debugging
  const State & getState(uint16_t x_t, uint16_t y_t) const override
  {
    return (tile_[y_t * tileStrideY_ + x_t]);
  }

  void event(uint16_t ex, uint16_t ey, uint8_t polarity)
  {
    auto & s = state_[ey * width_ + ex];
    int8_t p = static_cast<bool>(polarity) ? 1 : -1;
    // raw change in polarity, will be 0 or +-2
    const auto dp = static_cast<float>(p - s.getP());
    // run the temporal filter
    const auto L = c_[0] * s.getL() + c_[1] * s.getL_lag() + c_[2] * dp;
    const state_t deltaState(L - s.getL_last());  // how much pixel has changed
    // update state
    s.setL_lag(s.getL());
    s.setL(L);
    s.setP(p);
    const uint16_t tx = ex / tileSize_;
    const uint16_t ty = ey / tileSize_;
    auto & tile = tile_[ty * tileStrideY_ + tx];
    // run activity detector
    if (!s.isActive(p)) {  // drop duplicate events of same polarity
      if (!s.isActive()) {
        // if other-polarity event already turned the pixel active, don't count it
        numOccupiedPixels_++;
        // state of top left corner of tile has actual pixel-in-tile count
        const auto oldCount = tile.getNumActive();
        if (oldCount == 0) {
          numOccupiedTiles_++;  // first active pixel in this tile
        }
        tile.incNumActive();  // bump number of active pixels in this tile
        if (oldCount < activityThreshold_ && tile.getNumActive() >= activityThreshold_) {
          activityTileLayer_[0].subTileActive(tx, ty);
        }
      }
      s.markActive(p);  // mark this polarity active
      events_.push(Event(ex, ey, p));
      processEventQueue();  // adjusts size of event window
    }
    // if state has changed too much, notify higher layers
    if (activityTileLayer_[0].isAboveThreshold(std::abs(deltaState))) {
      // only take action if the pixel has substantially changed
      s.setL_last(L);
      // update tile average
      const auto oldL = tile.getL();
      const auto newL = (oldL * tileArea_ + deltaState) * invTileArea_;
      tile.setL(newL);
      const auto deltaTile = newL - tile.getL_last();
      if (activityTileLayer_[0].isAboveThreshold(std::abs(deltaTile))) {
        tile.setL_last(newL);
        activityTileLayer_[0].subtileHasChanged(State(deltaTile), tx, ty);
      }
    }
  }

  void initialize(
    size_t width, size_t height, uint32_t cutoffTime, uint32_t tileSize, double fillRatio);
  void getImage(uint8_t * img, size_t stride) const;
  size_t getWidth() const { return (width_); }
  size_t getHeight() const { return (height_); }

  const std::vector<State> & getState() const { return (state_); }

  const ActivityTileLayer * getActivity(int layer) const;

  size_t getEventWindowSize() const { return (eventWindowSize_); }

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
  inline size_t getTileIdx(uint16_t ex, uint16_t ey) const
  {
    return ((ey / tileSize_) * tileStrideY_ + (ex / tileSize_));
  }

  void setFillRatio(double fill_ratio);
  void processEventQueue()
  {
    while (events_.size() > eventWindowSize_) {
      const Event & e = events_.front();
      auto & s = state_[e.y() * width_ + e.x()];
      const uint16_t tx = e.x() / tileSize_;
      const uint16_t ty = e.y() / tileSize_;
      auto & tile = tile_[ty * tileStrideY_ + tx];
      if (!s.isActive()) {
        std::cerr << e.x() << " " << e.y() << " is inactive!" << std::endl;
        throw std::runtime_error("inactivating inactive pixel!");
      }
      s.markInActive(e.p());
      if (!s.isActive()) {  // wait for both polarities to be inactive
#ifdef USE_GAUSSIAN_FILTER
        //s = spatial_filter::filter<State, 3>(&state_[0], e.x(), e.y(), width_, height_, GAUSSIAN_3x3);
        s =
          spatial_filter::filter<State, 5>(&state_[0], e.x(), e.y(), width_, height_, GAUSSIAN_5x5);
#endif
        const auto oldCount = tile.getNumActive();
        if (oldCount == 0) {
          std::cerr << e.x() << " " << e.y() << " tile is empty!" << std::endl;
          throw std::runtime_error("empty tile!");
        }
        // decrease number of pixels in this tile
        tile.decNumActive();
        if (tile.getNumActive() == 0) {
          numOccupiedTiles_--;
        }
        if (oldCount >= activityThreshold_ && tile.getNumActive() < activityThreshold_) {
          activityTileLayer_[0].subTileInactive(tx, ty);
        }
        numOccupiedPixels_--;
      }
      events_.pop();  // remove element now
    }
    // adjust event window size up or down to match the fill ratio:
    // new_size = old_size * current_fill_ratio / desired_fill_ratio
    // The idea is that as the event window increases, the features will "fill out"
    eventWindowSize_ = (eventWindowSize_ * numOccupiedTiles_ * fillRatioDenom_) /
                       (numOccupiedPixels_ * fillRatioNum_);
  }

  // ------------------- variables ------------------
  size_t width_{0};
  size_t height_{0};
  std::vector<State> state_;         // filter state
  std::array<float, 3> c_{0, 0, 0};  // filter coefficients, see frequency cam paper
  uint8_t activityThreshold_{1};     // number of active pixels before tile is active
  std::vector<ActivityTileLayer> activityTileLayer_;
  // ---------- related to activity detection
  static constexpr int START_WINDOW_SIZE = 2000;
  uint16_t tileSize_{0};                         // width/height of tiles
  std::vector<State> tile_;                      // array with tiles
  state_t tileArea_{0};                          // tile size ^2
  state_t invTileArea_{0};                       // inverse of tile area
  uint16_t tileStrideY_{0};                      // size of stride in tiled image
  uint64_t eventWindowSize_{START_WINDOW_SIZE};  // current event window size
  uint64_t fillRatioDenom_{2};                   // denominator of fill ratio
  uint64_t fillRatioNum_{1};                     // numerator of fill ratio
  uint64_t numOccupiedPixels_{0};                // currently occupied number of pixels
  uint64_t numOccupiedTiles_{0};                 // currently occupied number of blocks
  std::queue<Event> events_;                     // queue with buffered events
};
}  // namespace simple_image_recon_lib
#endif  // SIMPLE_IMAGE_RECON_LIB_SIMPLE_IMAGE_RECONSTRUCTOR_HPP
