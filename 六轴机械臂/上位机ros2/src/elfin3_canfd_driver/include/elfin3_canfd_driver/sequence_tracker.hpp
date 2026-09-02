#pragma once

#include <cstdint>

namespace elfin3_canfd
{

enum class SequenceResult
{
  kFirst,
  kAccepted,
  kAcceptedWithGap,
  kDuplicate,
  kBackward,
};

class SequenceTracker
{
public:
  SequenceResult accept(const std::uint8_t sequence)
  {
    if (!initialized_) {
      initialized_ = true;
      last_sequence_ = sequence;
      ++accepted_count_;
      return SequenceResult::kFirst;
    }

    const auto delta = static_cast<std::uint8_t>(sequence - last_sequence_);
    if (delta == 0U) {
      ++duplicate_count_;
      return SequenceResult::kDuplicate;
    }
    if (delta >= 128U) {
      ++backward_count_;
      return SequenceResult::kBackward;
    }

    last_sequence_ = sequence;
    ++accepted_count_;
    if (delta == 1U) {
      return SequenceResult::kAccepted;
    }
    lost_count_ += static_cast<std::uint64_t>(delta - 1U);
    return SequenceResult::kAcceptedWithGap;
  }

  void reset()
  {
    initialized_ = false;
    last_sequence_ = 0U;
  }

  bool initialized() const {return initialized_;}
  std::uint8_t last_sequence() const {return last_sequence_;}
  std::uint64_t accepted_count() const {return accepted_count_;}
  std::uint64_t lost_count() const {return lost_count_;}
  std::uint64_t duplicate_count() const {return duplicate_count_;}
  std::uint64_t backward_count() const {return backward_count_;}

private:
  bool initialized_{false};
  std::uint8_t last_sequence_{0};
  std::uint64_t accepted_count_{0};
  std::uint64_t lost_count_{0};
  std::uint64_t duplicate_count_{0};
  std::uint64_t backward_count_{0};
};

}  // namespace elfin3_canfd
