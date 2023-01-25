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

#ifndef SIMPLE_IMAGE_RECON_LIB_ACTIVITY_TILE_LAYER_HPP
#define SIMPLE_IMAGE_RECON_LIB_ACTIVITY_TILE_LAYER_HPP

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <vector>

#include "simple_image_recon_lib/state.hpp"
#include "simple_image_recon_lib/subtiles.hpp"

namespace simple_image_recon_lib
{
class ActivityTileLayer : public Subtiles
{
public:
  using state_t = State::state_t;

  explicit ActivityTileLayer(
    uint16_t width, uint16_t height, uint16_t tileSizeX, uint16_t tileSizeY,
    float fillRatioThreshold);

  ActivityTileLayer() = default;

  void setHigherTiles(ActivityTileLayer * layer) { higherTiles_ = layer; }
  void setSubtiles(Subtiles * s) { subtiles_ = s; }

  void subTileActive(uint16_t x, uint16_t y)
  {
    uint16_t x_t(0);
    uint16_t y_t(0);
    const size_t tileIdx = getTileIdx(x, y, &x_t, &y_t);
    if (tileIdx >= tile_.size()) {
      std::cerr << "bad active idx: " << tileIdx << " " << tile_.size() << std::endl;
      throw std::runtime_error("tile idx");
    }
    auto & tile = tile_[tileIdx];
    if (tile.getNumActive() >= tileSizeX_ * tileSizeY_) {
      std::cerr << "too many active subtiles!" << std::endl;
      throw std::runtime_error("too many active");
    }
    tile.incNumActive();
    if (!tile.isActive() && tile.getNumActive() >= activeOnThreshold_) {
      tile.markActive();
      if (higherTiles_ != nullptr) {
        higherTiles_->subTileActive(x_t, y_t);
      }
    }
  }

  void subTileInactive(uint16_t x, uint16_t y)
  {
    uint16_t x_t(0);
    uint16_t y_t(0);
    const size_t tileIdx = getTileIdx(x, y, &x_t, &y_t);
    if (tileIdx >= tile_.size()) {
      std::cerr << "bad inactive idx: " << tileIdx << " " << tile_.size() << std::endl;
      throw std::runtime_error("tile idx");
    }
    auto & tile = tile_[tileIdx];
    if (tile.getNumActive() == 0) {
      std::cerr << "subtile already inactive!" << std::endl;
      throw std::runtime_error("subtile already inactive");
    }
    tile.decNumActive();
    if (tile.isActive() && tile.getNumActive() < activeOnThreshold_) {
      tile.markInActive();
      if (higherTiles_ != nullptr) {
        higherTiles_->subTileInactive(x_t, y_t);
      }
    }
  }

  // called by higher layers
  // return value: new state of this tile
  State setState(const State & targetState, uint16_t x_t, uint16_t y_t) override
  {
    size_t tileIdx = y_t * tileStrideY_ + x_t;
    if (tileIdx >= tile_.size()) {
      std::cerr << "bad subtile idx: " << tileIdx << " " << tile_.size() << std::endl;
      throw std::runtime_error("tile idx");
    }
    auto & tile = tile_[tileIdx];
    if (tile.isActive()) {
      return (State(tile.getL_last()));  // active tiles do not get updated
    }
    const auto avgL = tile.getL();
    // if tile average is already within threshold, we are done
    if (!isAboveThreshold(std::abs(targetState.getL() - avgL))) {
      return (State(tile.getL_last()));
    }
    const state_t newAvgL = updateSubtiles(targetState, x_t, y_t);
    tile.setL(newAvgL);
    tile.setL_last(newAvgL);  // because we
    return (State(newAvgL));
  }

  inline state_t updatedTileAverage(const state_t & avgL, const state_t & dL) const
  {
    return ((avgL * tileArea_ + dL) * invTileArea_);
  }

  bool checkTileAverage(uint16_t x_t, uint16_t y_t) const
  {
    const uint16_t xis = x_t * tileSizeX_;
    const uint16_t xie = xis + tileSizeX_;
    const uint16_t yis = y_t * tileSizeY_;
    const uint16_t yie = yis + tileSizeY_;

    state_t sum(0);
    for (uint16_t xi = xis; xi < xie; xi++) {
      for (uint16_t yi = yis; yi < yie; yi++) {
        const auto & s = subtiles_->getState(xi, yi);
        sum += s.getL_last();
      }
    }
    sum *= invTileArea_;
    const auto & tile = tile_[y_t * tileStrideY_ + x_t];
    const bool isGood = std::abs(sum - tile.getL()) < 1e-6;
    return (isGood);
  }

  // called by lower layers
  void subtileHasChanged(const State & deltaState, uint16_t x, uint16_t y)
  {
    uint16_t x_t(0);
    uint16_t y_t(0);
    const size_t tileIdx = getTileIdx(x, y, &x_t, &y_t);
    if (tileIdx >= tile_.size()) {
      std::cerr << "bad subtile idx: " << tileIdx << " " << tile_.size() << std::endl;
      throw std::runtime_error("tile idx");
    }
    auto & tile = tile_[tileIdx];
    const state_t avgL = tile.getL();
    const state_t & dL = deltaState.getL();
    //
    // update state lower in the pyramid
    //
    const State targetAvg(updatedTileAverage(avgL, dL));
    const state_t newL = updateSubtiles(targetAvg, x_t, y_t);
    tile.setL(newL);
    const state_t dLTile = newL - tile.getL_last();
    if (!checkTileAverage(x_t, y_t)) {
      std::cout << "tile average screwed up!!!" << std::endl;
      throw std::runtime_error("tile state screwed up!");
    }
    //
    // inform states higher in the pyramid about the new average
    //
    if (isAboveThreshold(std::abs(dLTile)) && higherTiles_ != nullptr) {
      tile.setL_last(newL);
      const bool wasActive = tile.isActive();
      // tile is about to send an update up the pyramid, must mark
      // it temporarily active so it won't in turn be updated
      if (!wasActive) {
        tile.markActive();
      }
      higherTiles_->subtileHasChanged(State(dLTile), x_t, y_t);
      if (!wasActive) {
        tile.markInActive();  // restore previous state
      }
    }
  }

  void setLevel(uint32_t level) { level_ = level; }

  const State & getState(uint16_t x_t, uint16_t y_t) const override
  {
    return (tile_[y_t * tileStrideY_ + x_t]);
  }

  bool isAboveThreshold(state_t deltaL) const { return (deltaL > avgChangeThreshold_); }
  const std::vector<State> & getTiles() const { return (tile_); }
  uint16_t getTileSizeX() const { return (tileSizeX_); }
  uint16_t getTileSizeY() const { return (tileSizeY_); }
  uint16_t getTileStrideY() const { return (tileStrideY_); }
  ActivityTileLayer * getHigherLayer() const { return (higherTiles_); }
  uint16_t getWidth() const { return (tileStrideY_); }
  uint16_t getHeight() const { return (static_cast<uint16_t>(tile_.size() / tileStrideY_)); }
  const State * getData() const { return (tile_.data()); }

  static void make_activity_tiles(
    std::vector<ActivityTileLayer> * layers, uint32_t width, uint32_t height,
    float fillRatioThreshold);

private:
  state_t updateSubtiles(const State & targetState, uint16_t x_t, uint16_t y_t) const
  {
    const uint16_t xis = x_t * tileSizeX_;
    const uint16_t xie = xis + tileSizeX_;
    const uint16_t yis = y_t * tileSizeY_;
    const uint16_t yie = yis + tileSizeY_;
    state_t dLSum(0);
    for (uint16_t xi = xis; xi < xie; xi++) {
      for (uint16_t yi = yis; yi < yie; yi++) {
        // the active tile that called this function
        // will be skipped, so does not contribute to the sum
        const auto dL = subtiles_->setState(targetState, xi, yi).getL();
        dLSum += dL;
      }
    }
    state_t newAvg = dLSum * invTileArea_;
    return (newAvg);
  }

  size_t getTileIdx(uint16_t x, uint16_t y, uint16_t * x_t, uint16_t * y_t) const
  {
    *x_t = x / tileSizeX_;
    *y_t = y / tileSizeY_;
    return ((*y_t) * tileStrideY_ + (*x_t));
  }

  // ------------------- variables ------------------
  ActivityTileLayer * higherTiles_{nullptr};
  uint32_t level_{0};
  uint16_t tileSizeX_{0};
  uint16_t tileSizeY_{0};
  uint16_t tileStrideY_{0};
  float tileArea_{0};
  float invTileArea_{0};
  uint32_t activeOnThreshold_{0};
  state_t avgChangeThreshold_{0.1};
  std::vector<State> tile_;
  Subtiles * subtiles_{nullptr};
};
}  // namespace simple_image_recon_lib
#endif  // SIMPLE_IMAGE_RECON_LIB_ACTIVITY_TILE_LAYER_HPP
