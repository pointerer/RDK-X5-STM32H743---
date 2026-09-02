#ifndef ELFIN3_JOG__JOG_MATH_HPP_
#define ELFIN3_JOG__JOG_MATH_HPP_

#include <cmath>
#include <cstdint>

namespace elfin3_jog
{

inline constexpr std::int64_t kJogCountsPerRevolution = 13238272;
inline constexpr std::int64_t kJogCountsPerUpdate = 100;
inline constexpr double kJogTwoPi = 6.28318530717958647692;

inline double counts_to_radians(const std::int64_t counts)
{
  return static_cast<double>(counts) * kJogTwoPi /
         static_cast<double>(kJogCountsPerRevolution);
}

inline std::int64_t radians_to_counts(const double radians)
{
  return std::llround(
    radians * static_cast<double>(kJogCountsPerRevolution) / kJogTwoPi);
}

inline bool bounded_jog_step(
  const std::int64_t current, const std::int8_t direction,
  const std::int64_t lower, const std::int64_t upper,
  std::int64_t & next)
{
  const std::int64_t candidate = current + direction * kJogCountsPerUpdate;
  if (candidate < lower || candidate > upper) {
    next = current;
    return false;
  }
  next = candidate;
  return true;
}

}  // namespace elfin3_jog

#endif  // ELFIN3_JOG__JOG_MATH_HPP_
