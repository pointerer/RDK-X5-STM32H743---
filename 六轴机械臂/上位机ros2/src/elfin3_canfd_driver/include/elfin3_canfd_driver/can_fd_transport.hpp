#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>

namespace elfin3_canfd
{

struct CanFdFrame
{
  std::uint32_t id{0};
  std::uint8_t length{0};
  std::array<std::uint8_t, 64> data{};
  std::chrono::steady_clock::time_point received_at{};
};

struct TransportStats
{
  std::uint64_t transmitted{0};
  std::uint64_t received{0};
  std::uint64_t malformed{0};
  std::uint64_t tx_errors{0};
  std::uint64_t rx_errors{0};
  std::uint64_t can_error_frames{0};
};

enum class ReceiveResult
{
  kFrame,
  kNoData,
  kCanError,
  kError,
};

class CanFdTransport
{
public:
  CanFdTransport() = default;
  ~CanFdTransport();

  CanFdTransport(const CanFdTransport &) = delete;
  CanFdTransport & operator=(const CanFdTransport &) = delete;

  bool open(const std::string & interface_name, std::string & error);
  void close();
  bool is_open() const;

  bool send(
    std::uint32_t id, const std::uint8_t * data, std::size_t size, std::string & error);
  ReceiveResult receive(CanFdFrame & frame, std::uint32_t & can_error, std::string & error);

  const TransportStats & stats() const {return stats_;}

private:
  int socket_fd_{-1};
  TransportStats stats_{};
};

}  // namespace elfin3_canfd
