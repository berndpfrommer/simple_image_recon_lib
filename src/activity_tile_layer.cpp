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

#include "simple_image_recon_lib/activity_tile_layer.hpp"

#include <array>
#include <cmath>
#include <iostream>

namespace simple_image_recon_lib
{
ActivityTileLayer::ActivityTileLayer(
  uint16_t width, uint16_t height, uint16_t tileSizeX, uint16_t tileSizeY, float fillRatioThreshold)
: tileSizeX_(tileSizeX), tileSizeY_(tileSizeY)
{
  tileStrideY_ = width / tileSizeX;  // width in number of tiles in X direction
  tile_.resize(tileStrideY_ * (height / tileSizeY));
  activeOnThreshold_ =
    static_cast<uint32_t>(fillRatioThreshold * static_cast<float>(tileSizeX_ * tileSizeY_));
  std::cout << "created layer: nst: " << width << " x " << height << " -> " << tileStrideY_ << " x "
            << (height / tileSizeY) << " num tiles: " << tile_.size() << std::endl;
}

static bool find_factor(
  uint32_t fullWidth, uint32_t * tiledWidth, uint32_t * factorWidth, uint32_t fullHeight,
  uint32_t * tiledHeight, uint32_t * factorHeight)
{
  static const std::array<uint32_t, 4> factors = {2, 3, 5, 7};
  for (const auto f : factors) {
    if (fullWidth % f == 0 && fullHeight % f == 0) {
      *factorWidth = f;
      *factorHeight = f;
      *tiledWidth = fullWidth / f;
      *tiledHeight = fullHeight / f;
      return (false);
    }
  }
  *tiledWidth = 1;
  *tiledHeight = 1;
  *factorWidth = fullWidth;
  *factorHeight = fullHeight;
  return (true);  // done factoring
}

void ActivityTileLayer::make_activity_tiles(
  std::vector<ActivityTileLayer> * layers, uint32_t width, uint32_t height,
  float fillRatioThreshold)
{
  uint32_t w = width;
  uint32_t h = height;
  layers->clear();

  bool doneFactoring(false);
  while (!doneFactoring) {
    uint32_t tileWidth(0);
    uint32_t tileHeight(0);
    uint32_t newWidth(0);
    uint32_t newHeight(0);
    doneFactoring = find_factor(w, &newWidth, &tileWidth, h, &newHeight, &tileHeight);
    layers->emplace_back(w, h, tileWidth, tileHeight, fillRatioThreshold);
    w = newWidth;
    h = newHeight;
  }
  for (size_t i = 0; i < layers->size() - 1; i++) {
    (*layers)[i].setHigherTiles(&(*layers)[i + 1]);
  }
}
}  // namespace simple_image_recon_lib
