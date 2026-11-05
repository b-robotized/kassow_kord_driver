// Copyright (c) 2026 b»robotized
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential

#ifndef KASSOW_KORD_HARDWARE_INTERFACE__BIT_HELPERS_HPP_
#define KASSOW_KORD_HARDWARE_INTERFACE__BIT_HELPERS_HPP_

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace kassow_kord_hardware_interface
{
namespace bit_helpers
{

// using `bitfield` instead of `mask`, as I consider a "mask" to be something that changes a state
// of bits. for example, io_prev_command I'd consider a bitfield, and desired_mask would actually be
// a mask. not critical, but food for thought. Both are a collection of bits in the end.
[[nodiscard]] inline bool is_set(uint64_t bitfield, size_t i) { return (bitfield >> i) & 0x1u; }

[[nodiscard]] inline uint64_t set(uint64_t bitfield, size_t i) { return bitfield | (1ULL << i); }

[[nodiscard]] inline uint64_t clear(uint64_t bitfield, size_t i) { return bitfield & ~(1ULL << i); }

[[nodiscard]] inline double bit_to_double(bool bit) { return bit ? 1.0 : 0.0; }

[[nodiscard]] inline bool double_to_bit(double cmd) { return cmd > 0.5; }

// subset of bits that go 0->1 between current and desired
[[nodiscard]] inline uint64_t get_bits_to_enable(uint64_t desired, uint64_t current)
{
  return desired & ~current;
}

// subset of bits that go 1->0 between current and desired
[[nodiscard]] inline uint64_t get_bits_to_disable(uint64_t desired, uint64_t current)
{
  return ~desired & current;
}

}  // namespace bit_helpers
}  // namespace kassow_kord_hardware_interface

#endif  // KASSOW_KORD_HARDWARE_INTERFACE__BIT_HELPERS_HPP_
