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

#include <cstddef>
#include <cstdint>

namespace simple_image_recon_lib
{
class State
{
public:
  using state_t = float;
  explicit State(
    state_t L_a = 0, state_t L_lag_a = 0, state_t L_last_a = 0, int8_t p_a = 0, uint8_t ac = 0)
    : L(L_a), L_lag(L_lag_a), L_last(L_last_a), p(p_a), activity(ac)
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
  inline state_t getL_last() const { return (L_last); }
  inline int8_t getP() const { return (p); }
  inline void setL(state_t f) { L = f; }
  inline void setL_lag(state_t f) { L_lag = f; }
  inline void setL_last(state_t f) { L_last = f; }
  inline void setP(int8_t i) { p = i; }

  // ----------- related to activity -----------------------
  uint8_t getNumActive() const { return (activity & INV_ACTIVITY_MASK); }

  inline bool isActive(int8_t p) const
  {
    return (( activity & (p == 1 ? ACTIVITY_ON_MASK : ACTIVITY_OFF_MASK)) != 0);
  }
  inline bool isActive() const { return (( activity & ACTIVITY_MASK) != 0); }

  inline void markActive(int8_t p)
  {
     activity |= (p == 1 ? ACTIVITY_ON_MASK : ACTIVITY_OFF_MASK);
  }
  inline void markActive()
  {
    activity |= ACTIVITY_MASK;
  }

  inline void markInActive(int8_t p)
  {
     activity &= (p == 1 ? (~ACTIVITY_ON_MASK) : (~ACTIVITY_OFF_MASK));
  }

  inline void markInActive() { activity &= INV_ACTIVITY_MASK; }

  inline void incNumActive() {  activity++; }
  inline void decNumActive() {  activity--; }

  
  // make variables public so they can be exposed to e.g. pybind11
  // ------ variables -------
  state_t L{0};
  state_t L_lag{0};
  state_t L_last{0};
  int8_t p{0};
  uint8_t activity{0};

  static constexpr int max_num_active() { return ((1 << ACTIVITY_LOW_BIT) - 1); }

private:
  static constexpr uint8_t ACTIVITY_ON_BIT = 6;
  static constexpr uint8_t ACTIVITY_OFF_BIT = 7;
  static constexpr uint8_t ACTIVITY_LOW_BIT = ACTIVITY_ON_BIT;
  static constexpr uint8_t ACTIVITY_ON_MASK = static_cast<uint8_t>(1) << ACTIVITY_ON_BIT;
  static constexpr uint8_t ACTIVITY_OFF_MASK = static_cast<uint8_t>(1) << ACTIVITY_OFF_BIT;
  static constexpr uint8_t ACTIVITY_MASK = ACTIVITY_ON_MASK | ACTIVITY_OFF_MASK;
  static constexpr uint8_t INV_ACTIVITY_MASK = static_cast<uint8_t>(~ACTIVITY_MASK);
};
}  // namespace simple_image_recon_lib
#endif  // SIMPLE_IMAGE_RECON_LIB_STATE_HPP
