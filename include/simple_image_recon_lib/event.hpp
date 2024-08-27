// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2024 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#ifndef SIMPLE_IMAGE_RECON_LIB__EVENT_HPP_
#define SIMPLE_IMAGE_RECON_LIB__EVENT_HPP_

#include <cstdint>

namespace simple_image_recon_lib
{
class Event
{
public:
  explicit Event(uint32_t t_a, uint16_t x, uint16_t y, int8_t p) : time(t_a), ex(x), ey(y), ep(p) {}
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
}  // namespace simple_image_recon_lib
#endif  // SIMPLE_IMAGE_RECON_LIB__EVENT_HPP_
