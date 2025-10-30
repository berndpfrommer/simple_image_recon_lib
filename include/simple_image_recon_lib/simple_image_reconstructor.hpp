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
#include <cstring>
#include <deque>
#include <fstream>
#include <iostream>
#include <limits>
#include <vector>

#include "simple_image_recon_lib/spatial_filter.hpp"
#include "simple_image_recon_lib/state.hpp"

#define LIKELY(x) __builtin_expect(!!(x), 1)
#define UNLIKELY(x) __builtin_expect(!!(x), 0)
#define SANITY_CHECKS

namespace simple_image_recon_lib
{
template <int tile_size>
inline size_t getTileIndex(uint16_t ex, uint16_t ey, uint16_t tileStrideY)
{
  return ((ey / tile_size) * tileStrideY + (ex / tile_size) * tile_size);
}
template <>
inline size_t getTileIndex<2>(uint16_t ex, uint16_t ey, uint16_t tileStrideY)

{
  return ((ey >> 1) * tileStrideY + (ex & ~1));
}

template <uint8_t tile_size = 2>
class SimpleImageReconstructor
{
public:
  using state_t = float;
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
    auto & s = state_[ey * width_ + ex];
    const auto p = static_cast<state_t>((polarity == 0) ? -1 : 1);
#ifdef RESCALE
    // change in polarity, will be scale * (0 or +-2)
    const auto dp = s.getScale() * static_cast<float>(p - s.getPbar());
#else
    // raw change in polarity, will be 0 or +-2
    const auto dp = static_cast<float>(p - s.getPbar());
#endif
    // run the temporal filter
    const auto L = c_[2] * s.getL() + c_[3] * dp;
    // update state
    s.setPbar(s.getPbar() * c_[0] + p * c_[1]);
    s.setL(L);
    // run activity detector
#define USE_SPATIAL_FILTER
#ifdef USE_SPATIAL_FILTER
    if (!s.isActive()) {
      numOccupiedPixels_ += fillRatioDenom_;
      // state of top left corner of tile has actual pixel-in-tile count
      auto & tile = state_[getTileIdx(ex, ey)];
      if (tile.getNumPixActive() == 0) {
        numOccupiedTiles_ += fillRatioNum_;  // first active pixel in this tile
      }
      tile.incNumPixActive();  // bump number of pixels in this tile
    }
    s.incNumEventsInQueue();
    events_.push_back(Event(ex, ey, static_cast<int8_t>(polarity)));
    processEventQueue();  // adjusts size of event window
#endif
    currentTime_ = t;
  }

  void processEventQueue()
  {
    while (events_.size() > eventWindowSize_) {
      const Event & e = events_.front();
      auto & s = state_[e.y() * width_ + e.x()];
#ifdef SANITY_CHECKS
      if (!s.isActive()) {
        std::cerr << e.x() << " " << e.y() << " is inactive!" << std::endl;
        throw std::runtime_error("inactivating inactive pixel!");
      }
#endif
      s.decNumEventsInQueue();
      if (!s.isActive()) {
// #define SPATIAL_FILTER_5x5
#ifdef SPATIAL_FILTER_5x5
        s =
          spatial_filter::filter<State, 5>(&state_[0], e.x(), e.y(), width_, height_, GAUSSIAN_5x5);
#else
        // s =  spatial_filter::filter<State, 3>(&state_[0], e.x(), e.y(), width_, height_, GAUSSIAN_3x3);
        s = spatial_filter::filter_3x3(state_.data(), e.x(), e.y(), width_, height_, GAUSSIAN_3x3);
#endif
        auto & tile = state_[getTileIdx(e.x(), e.y())];  // state of top left corner of tile
#ifdef SANITY_CHECKS
        if (tile.getNumPixActive() == 0) {
          std::cerr << e.x() << " " << e.y() << " tile " << getTileIdx(e.x(), e.y()) << " is empty!"
                    << std::endl;
          throw std::runtime_error("empty tile!");
        }
#endif
        // remove number of pixels in this tile
        tile.decNumPixActive();
        if (tile.getNumPixActive() == 0) {
          numOccupiedTiles_ -= fillRatioNum_;
        }
        numOccupiedPixels_ -= fillRatioDenom_;
        num_filtered_++;
      }
      events_.pop_front();  // remove element now
    }
    // adjust event window size up or down to match the fill ratio:
    // new_size = old_size * current_fill_ratio / desired_fill_ratio
    // The idea is that as the event window increases, the features will "fill out"
#define AVOID_DIVISION
#ifdef AVOID_DIVISION
    const int64_t ntfn = numOccupiedTiles_;
    int64_t npfd = numOccupiedPixels_;
    if (UNLIKELY(npfd <= 1)) {
      npfd = fillRatioDenom_;
    }
    if (LIKELY(std::abs(500 * (ntfn - npfd)) > npfd)) {
      const uint64_t targetSize = (eventWindowSize_ * ntfn) / npfd;
      // prevent the event window from collapsing to zero and from growing without bounds
      eventWindowSize_ = std::max(minWindowSize_, std::min(maxWindowSize_, targetSize));
    }
#else
    const uint64_t targetSize =
      (eventWindowSize_ * numOccupiedTiles_) / (std::max(numOccupiedPixels_, fillRatioDenom_));
    eventWindowSize_ = std::max(minWindowSize_, std::min(maxWindowSize_, targetSize));
#endif
  }

  size_t getWidth() const { return (width_); }
  size_t getHeight() const { return (height_); }
  size_t getCurrentQueueSize() const { return (events_.size()); }
  double getCurrentFillRatio() const
  {
    return (
      numOccupiedTiles_ == 0
        ? -1.0
        : static_cast<double>(numOccupiedPixels_) / (numOccupiedTiles_ * tile_size * tile_size));
  }

  const std::vector<State> & getState() const { return (state_); }

  size_t getEventWindowSize() const { return (eventWindowSize_); }

  inline size_t getTileIdx(uint16_t ex, uint16_t ey) const
  {
    return (getTileIndex<tile_size>(ex, ey, tileStrideY_));
  }

  void initialize(size_t width, size_t height, uint32_t cutoffTime, double fillRatio)
  {
    width_ = width;
    height_ = height;
    // compute filter coefficients
    double alpha(0);
    double beta(0);
    computeAlphaBeta(static_cast<double>(cutoffTime), &alpha, &beta);
    c_[0] = static_cast<float>(alpha);
    c_[1] = static_cast<float>(1.0 - alpha);
    c_[2] = static_cast<float>(beta);
    c_[3] = static_cast<float>(0.5 * (1 + beta));
    state_.resize(width * height, State());
    tileStrideY_ = width * tile_size;
    constexpr int maxArea = (1 << ACTIVITY_LOW_BIT);
    if (tile_size * tile_size > maxArea) {
      // guard against overflow of count of occupied pixels in tile
      std::cerr << "activity tile size too big: " << tile_size << " must be < "
                << static_cast<int>(std::sqrt(maxArea)) << std::endl;
      throw(std::runtime_error("activity tile size too big"));
    }
    // disable any queue usage if tile size is set to zero
    maxWindowSize_ = tile_size > 0 ? static_cast<uint64_t>(width_ * height_) : 0;
    setFillRatio(fillRatio);
  }

  void getImage(uint8_t * img, size_t stride) const
  {
    // find min and max for normalization
    float minL = std::numeric_limits<float>::max();
    float maxL = std::numeric_limits<float>::min();
    for (size_t i = 0; i < height_ * width_; i++) {
      if (state_[i].getL() > maxL) {
        maxL = state_[i].getL();
      }
      if (state_[i].getL() < minL) {
        minL = state_[i].getL();
      }
    }
    // copy image over

    const float scale = 255.0F / (maxL - minL);
    for (size_t iy = 0; iy < height_; iy++) {
      const size_t y_off = iy * stride;
      const size_t y_off_state = iy * width_;
      for (size_t ix = 0; ix < width_; ix++) {
        const auto & s = state_[y_off_state + ix];
        img[y_off + ix] = static_cast<uint8_t>((s.getL() - minL) * scale);
      }
    }
  }

  void getActivePixelImage(uint8_t * img, size_t stride) const
  {
    // clear image
    memset(img, 0, height_ * stride);
    for (const auto & qe : events_) {
      img[qe.y() * stride + qe.x()]++;
    }
  }

  void setFillRatio(double fill_ratio)
  {
    fillRatioDenom_ = 100;
    // A is the area of the tile (in pixels)
    const double A = static_cast<double>(tile_size * tile_size);
    // how many tiles per pixel when fully filled
    const double tiles_per_pixel = 1.0 / A;
    // a fill ratio below 1 pixel per tile is not achievable
    const double r = std::min(1.0, std::max(fill_ratio, tiles_per_pixel + 1e-3));
    const double np_nt = A * r;  // targeted number of pixels per tile
    fillRatioNum_ = static_cast<uint64_t>(np_nt * fillRatioDenom_);
    // The update equation for the queue length q is
    // q_{k+1} = floor(q_k * f)
    // where f is the current gain:
    // f = (num_tiles * 100 * np_nt) / (num_pixels * 100)
    // For the queue to be able to grow, we must ensure that
    // q_k * f > q_k + 1
    // meaning q_k > 1/(f - 1)
    // The largest that f can become is when num_tiles == num_pixels,
    // in which case f = np_t, and so q_k > 1 / (np_nt - 1)
    //
    minWindowSize_ = A > 0 ? std::ceil((1.0 / (np_nt - 1.0))) : 0;
  }

#ifdef RESCALE
  void readScaleFile(const std::string & fname)
  {
    std::ifstream file;
    file.open(fname);
    if (!file.is_open()) {
      throw std::runtime_error("cannot open scale file: " + fname);
    }
    const uint32_t n_pix = width_ * height_;
    uint64_t sum{0};
    uint32_t s;
    for (size_t idx = 0; (file >> s) && (idx < n_pix * 2); idx++) {
      sum += s;
    }
    const double ntot_avg = sum / n_pix;
    file.close();
    file.open(fname);
    uint32_t n_on, n_off;
    size_t idx = 0;
    double ss{0}, ss2{0}, sum_inv{0};
    for (; (file >> n_on) && (file >> n_off) && (idx < n_pix); idx++) {
      const double C_i = ntot_avg / static_cast<double>(n_on + n_off);
      state_[idx].scale = C_i;
      ss += C_i;
      ss2 += C_i * C_i;
      sum_inv += 1.0 / C_i;
    }
    ss = ss / n_pix;
    ss2 = ss2 / n_pix;
    const double stddev = std::sqrt(ss2 - ss * ss);
    std::cout << "read scale file: " << fname << " with " << idx << " entries and " << ntot_avg
              << " events/pixel, avg C: " << ss << " stdev: " << stddev
              << " harmonic mean: " << (n_pix / sum_inv) << std::endl;
  }
#endif

private:
  class Event
  {
  public:
    explicit Event(uint16_t x = 0, uint16_t y = 0, int8_t p = 0) : ex(x), ey(y | (p << 15)) {}
    inline uint16_t x() const { return (ex); }
    inline uint16_t y() const { return (ey & 0x7fff); }
    inline int8_t p() const { return ((ey & 0x8000) >> 15); }

  private:
    uint16_t ex{0};
    uint16_t ey{0};
  };
  void computeAlphaBeta(const double T_cut, double * alpha, double * beta)
  {
    // compute the filter coefficients alpha and beta (see frequency cam paper)
    const double omega_cut = 2 * M_PI / T_cut;
    const double phi = 2 - std::cos(omega_cut);
    *alpha = (1.0 - std::sin(omega_cut)) / std::cos(omega_cut);
    *beta = phi - std::sqrt(phi * phi - 1.0);  // see paper
  }

  // ------------------- variables ------------------
  size_t width_{0};
  size_t height_{0};
  std::vector<State> state_;            // filter state
  std::array<float, 4> c_{0, 0, 0, 0};  // filter coefficients
  // ---------- related to activity detection
  static constexpr int START_WINDOW_SIZE = 2000;
  uint16_t tileStrideY_{0};                      // size of stride in tiled image
  uint64_t eventWindowSize_{START_WINDOW_SIZE};  // current event window size
  uint64_t fillRatioDenom_{2};                   // denominator of fill ratio
  uint64_t fillRatioNum_{1};                     // numerator of fill ratio
  uint64_t numOccupiedPixels_{0};                // currently occupied number of pixels
  uint64_t numOccupiedTiles_{0};                 // currently occupied number of blocks
  uint64_t maxWindowSize_{0};                    // maximum size of event window
  uint64_t minWindowSize_{0};                    // minimum size of event window
  std::deque<Event> events_;                     // queue with buffered events
  // -------- debugging
  uint32_t currentTime_{0};
  uint32_t num_filtered_{0};
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
