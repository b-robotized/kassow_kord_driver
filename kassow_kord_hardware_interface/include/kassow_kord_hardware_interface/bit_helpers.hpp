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

// Extract the boolean value of bit i from mask.
inline bool get_bit(uint64_t mask, size_t i) { return (mask >> i) & 0x1u; }

// Return a new mask with bit i set to 1.
inline uint64_t set_bit(uint64_t mask, size_t i) { return mask | (1ULL << i); }

// Return a new mask with bit i cleared to 0.
inline uint64_t clear_bit(uint64_t mask, size_t i) { return mask & ~(1ULL << i); }

// Set bit i if cmd > 0.5. If cmd is NaN, preserve bit i from prev. Otherwise leave bit cleared.
inline uint64_t build_mask(uint64_t mask, size_t i, double cmd, uint64_t prev = 0)
{
  if (std::isnan(cmd))
  {
    return get_bit(prev, i) ? set_bit(mask, i) : mask;
  }
  return cmd > 0.5 ? set_bit(mask, i) : mask;
}

// Convert a boolean bit to a double state value (1.0 or 0.0).
inline double bit_to_double(bool bit) { return bit ? 1.0 : 0.0; }

// Convert a double command value to a boolean bit (true if cmd > 0.5).
inline bool double_to_bit(double cmd) { return cmd > 0.5; }

// Return a mask of bits that differ between desired and prev.
inline uint64_t changed_bits(uint64_t desired, uint64_t prev) { return desired ^ prev; }

// Return the subset of changed bits that should be turned ON.
inline uint64_t enable_bits(uint64_t desired, uint64_t changed) { return desired & changed; }

// Return the subset of changed bits that should be turned OFF.
inline uint64_t disable_bits(uint64_t desired, uint64_t changed) { return ~desired & changed; }

// Apply an enable mask to prev (mark enabled bits as sent).
inline void apply_enable(uint64_t & prev, uint64_t enable_mask) { prev |= enable_mask; }

// Apply a disable mask to prev (mark disabled bits as sent).
inline void apply_disable(uint64_t & prev, uint64_t disable_mask) { prev &= ~disable_mask; }

}  // namespace bit_helpers
}  // namespace kassow_kord_hardware_interface

#endif  // KASSOW_KORD_HARDWARE_INTERFACE__BIT_HELPERS_HPP_
