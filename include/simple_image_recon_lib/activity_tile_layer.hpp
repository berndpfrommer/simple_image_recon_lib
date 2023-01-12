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

namespace simple_image_recon_lib
{
class ActivityTileLayer
{
public:
  class Tile
  {
  public:
    uint8_t getNumActive() const { return (num_active & INV_ACTIVE_BIT_MASK); }
    bool isActive() const { return ((num_active & ACTIVE_BIT_MASK) != 0); }
    void setActive() { num_active |= ACTIVE_BIT_MASK; }
    void setInactive() { num_active &= INV_ACTIVE_BIT_MASK; }
    void incNumActiveSubtiles() { num_active++; }
    void decNumActiveSubtiles() { num_active--; }

    uint8_t num_active{0};

  private:
    static constexpr uint8_t ACTIVE_BIT_MASK = 1 << 7;
    static constexpr uint8_t INV_ACTIVE_BIT_MASK = static_cast<uint8_t>(~ACTIVE_BIT_MASK);
  };

  explicit ActivityTileLayer(
    uint16_t width, uint16_t height, uint16_t tileSizeX, uint16_t tileSizeY,
    float fillRatioThreshold);

  ActivityTileLayer() = default;

  void setHigherTiles(ActivityTileLayer * layer) { higherTiles_ = layer; }

  void subTileActive(uint16_t x, uint16_t y)
  {
    uint16_t x_t(0);
    uint16_t y_t(0);
    const size_t tileIdx = getTileIdx(x, y, &x_t, &y_t);
    if (tileIdx >= tile_.size()) {
      std::cerr << "bad active idx: " << tileIdx << " " << tile_.size() << std::endl;
      throw std::runtime_error("tile idx");
    }
    Tile & tile = tile_[tileIdx];
    tile.incNumActiveSubtiles();
    if (!tile.isActive() && tile.getNumActive() >= activeOnThreshold_) {
      tile.setActive();
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
    Tile & tile = tile_[tileIdx];
    tile.decNumActiveSubtiles();
    if (tile.isActive() && tile.getNumActive() < activeOnThreshold_) {
      tile.setInactive();
      if (higherTiles_ != nullptr) {
        higherTiles_->subTileInactive(x_t, y_t);
      }
    }
  }
  const std::vector<Tile> & getTiles() const { return (tile_); }
  uint16_t getTileSizeX() const { return (tileSizeX_); }
  uint16_t getTileSizeY() const { return (tileSizeY_); }
  ActivityTileLayer * getHigherLayer() const { return (higherTiles_); }
  uint16_t getWidth() const { return (tileStrideY_); }
  uint16_t getHeight() const { return (static_cast<uint16_t>(tile_.size() / tileStrideY_)); }
  const Tile * getData() const { return (tile_.data()); }

  static void make_activity_tiles(
    std::vector<ActivityTileLayer> * layers, uint32_t width, uint32_t height,
    float fillRatioThreshold);

private:
  size_t getTileIdx(uint16_t x, uint16_t y, uint16_t * x_t, uint16_t * y_t) const
  {
    *x_t = x / tileSizeX_;
    *y_t = y / tileSizeY_;
    return ((*y_t) * tileStrideY_ + (*x_t));
  }

  // ------------------- variables ------------------
  ActivityTileLayer * higherTiles_{nullptr};
  uint16_t tileSizeX_{0};
  uint16_t tileSizeY_{0};
  uint16_t tileStrideY_{0};
  uint32_t activeOnThreshold_{0};
  std::vector<Tile> tile_;
};
}  // namespace simple_image_recon_lib
#endif  // SIMPLE_IMAGE_RECON_LIB_ACTIVITY_TILE_LAYER_HPP
