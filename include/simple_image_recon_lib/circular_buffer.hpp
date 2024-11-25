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

#ifndef SIMPLE_IMAGE_RECON_LIB__CIRCULAR_BUFFER_HPP__
#define SIMPLE_IMAGE_RECON_LIB__CIRCULAR_BUFFER_HPP__

#include <vector>

namespace simple_image_recon_lib
{

template <class T>
class CircularBuffer
{
public:
  CircularBuffer(size_t max_capacity = 0) { buffer_.resize(max_capacity, T(0, 0, 0, 0)); }

  void push(const T & a)
  {
    if (size() >= buffer_.size()) {
      throw(std::runtime_error("event queue overflow!"));
    }
    buffer_[end_] = a;
    end_ = (end_ + 1) % buffer_.size();
  }

  void pop()
  {
    if (begin_ == end_) {
      throw(std::runtime_error("event queue underflow!"));
    }
    begin_ = (begin_ + 1) % buffer_.size();
  }

  const T & front() { return (buffer_[begin_]); }

  size_t size() const { return ((end_ - begin_ + buffer_.size()) % buffer_.size()); }

private:
  // variables
  size_t begin_{0};
  size_t end_{0};
  std::vector<T> buffer_;
};
}  // namespace simple_image_recon_lib
#endif  // SIMPLE_IMAGE_RECON_LIB__CIRCULAR_BUFFER_HPP__
