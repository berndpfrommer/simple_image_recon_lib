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

#ifndef SIMPLE_IMAGE_RECON_LIB_STATE_HPP
#define SIMPLE_IMAGE_RECON_LIB_STATE_HPP

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

#define MINIMAL_STATE
#define COUNT_EVENTS
#define SUPPORT_SCALE

namespace simple_image_recon_lib
{
class State
{
public:
  using state_t = float;
  explicit State(
    state_t L_a = 0, state_t pbar_a = 0, uint8_t npa = 0, uint16_t neiq = 0, float sc = 1,
    uint64_t * lc = nullptr, const float * thr = nullptr)
  : L(L_a),
    pbar(pbar_a)
#ifndef MINIMAL_STATE
    ,
    numPixActive(npa),
    numEventsInQueue(neiq)
#endif
#ifdef SUPPORT_SCALE
    ,
    scale(sc)
#endif
  {
    if (lc != nullptr) {
      last_count[0] = lc[0];
      last_count[1] = lc[1];
    }
    if (thr != nullptr) {
      threshold[0] = thr[0];
      threshold[1] = thr[1];
    }
  }
  inline void operator+=(const State & s)
  {
    L += s.L;
    // leave other fields untouched
  }

  inline State operator*(const float c) const { return (State(c * L)); }

  inline state_t getL() const { return (L); }
  inline state_t getPbar() const { return (pbar); }
  inline uint64_t getLastCount(uint8_t p) const { return (last_count[p]); }
  inline void setL(state_t f) { L = f; }
  inline void setPbar(state_t f) { pbar = f; }
  inline void updateThreshold(uint8_t p, float discount, float v)
  {
    threshold[p] = std::clamp(threshold[p] * discount + v, 0.75f, 1.25f);
  }
  inline void setLastCount(uint8_t p, uint64_t count) { last_count[p] = count; };

#ifdef COUNT_EVENTS
  const uint64_t * getNumEvents() const { return (num_events_); }
#endif
  float getThreshold(uint8_t p) const { return (threshold[p]); }
  // ----------- related to activity -----------------------
#ifndef MINIMAL_STATE
  uint16_t getNumEventsInQueue() const { return (numEventsInQueue); }
  uint8_t getNumPixActive() const { return (numPixActive); }

  inline bool isActive() const { return (numEventsInQueue != 0); }
  inline void incNumPixActive() { numPixActive++; }
  inline void decNumPixActive() { numPixActive--; }
  inline void incNumEventsInQueue() { numEventsInQueue++; }
  inline void decNumEventsInQueue() { numEventsInQueue--; }
#endif
  // make variables public so they can be exposed to e.g. pybind11
  // ------ variables -------
  state_t L{0};
  state_t pbar{0};
#ifndef MINIMAL_STATE
  uint8_t numPixActive{0};
  uint16_t numEventsInQueue{0};
#endif
#ifdef COUNT_EVENTS
  uint64_t num_events_[2]{0, 0};
#endif
#ifdef SUPPORT_SCALE
  float scale{1.0};
#endif
  uint64_t last_count[2] = {0, 0};
  float threshold[2] = {1.0, 1.0};
  static constexpr int max_num_active() { return (255); };  // 8 bit

private:
};
}  // namespace simple_image_recon_lib
#endif  // SIMPLE_IMAGE_RECON_LIB_STATE_HPP
