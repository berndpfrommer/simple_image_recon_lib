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

#include <array>
#include <cstddef>
#include <cstdint>

namespace simple_image_recon_lib
{
class State
{
public:
  using state_t = float;
  explicit State(
    state_t L_a = 0, state_t L_lag_a = 0, int8_t p_a = 0, uint8_t npa = 0, uint16_t neiq = 0)
  : L(L_a), L_lag(L_lag_a), p(p_a), numPixActive(npa), numEventsInQueue(neiq)
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
  inline void setL(state_t f) { L = f; }
  inline void setL_lag(state_t f) { L_lag = f; }
  inline void setP(int8_t i) { p = i; }

  // ----------- related to activity -----------------------
  uint16_t getNumEventsInQueue() const { return (numEventsInQueue); }
  uint8_t getNumPixActive() const { return (numPixActive); }

  inline bool isActive() const { return (numEventsInQueue != 0); }
  inline void incNumPixActive() { numPixActive++; }
  inline void decNumPixActive() { numPixActive--; }
  inline void incNumEventsInQueue() { numEventsInQueue++; }
  inline void decNumEventsInQueue() { numEventsInQueue--; }

  // make variables public so they can be exposed to e.g. pybind11
  // ------ variables -------
  state_t L{0};
  state_t L_lag{0};
  int8_t p{0};
  uint8_t numPixActive{0};
  uint16_t numEventsInQueue{0};

  static constexpr int max_num_active() { return (255); };  // 8 bit

private:
};
}  // namespace simple_image_recon_lib
#endif  // SIMPLE_IMAGE_RECON_LIB_STATE_HPP
