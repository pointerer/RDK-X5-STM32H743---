#include "elfin3_canfd_driver/can_fd_transport.hpp"

#include <cerrno>
#include <cstring>

#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "elfin3_canfd_driver/protocol.hpp"

namespace elfin3_canfd
{
namespace
{

std::string errno_text(const char * operation)
{
  return std::string(operation) + ": " + std::strerror(errno);
}

std::size_t expected_size(const std::uint32_t id)
{
  switch (id) {
    case kPositionFeedbackId: return kPositionFeedbackSize;
    case kDetailedStatusId: return kDetailedStatusSize;
    case kDiagnosticEventId: return kDiagnosticEventSize;
    case kDeviceHeartbeatId: return kDeviceHeartbeatSize;
    case kTrajectoryStatusId: return kTrajectoryStatusSize;
    default: return 0U;
  }
}

bool valid_tx_id_and_size(const std::uint32_t id, const std::size_t size)
{
  return (id == kCspCommandId && size == kCspCommandSize) ||
         (id == kMotionControlId && size == kMotionControlSize) ||
         (id == kParameterRequestId && size == kParameterRequestSize) ||
         (id == kHostHeartbeatId && size == kHostHeartbeatSize);
}

}  // namespace

CanFdTransport::~CanFdTransport()
{
  close();
}

bool CanFdTransport::open(const std::string & interface_name, std::string & error)
{
  close();
  if (interface_name.empty() || interface_name.size() >= IFNAMSIZ) {
    error = "invalid CAN interface name";
    return false;
  }

  socket_fd_ = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
  if (socket_fd_ < 0) {
    error = errno_text("socket");
    return false;
  }

  const int enable = 1;
  if (::setsockopt(
      socket_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable, sizeof(enable)) < 0)
  {
    error = errno_text("setsockopt CAN_RAW_FD_FRAMES");
    close();
    return false;
  }

  constexpr can_err_mask_t error_mask = CAN_ERR_MASK;
  if (::setsockopt(
      socket_fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &error_mask, sizeof(error_mask)) < 0)
  {
    error = errno_text("setsockopt CAN_RAW_ERR_FILTER");
    close();
    return false;
  }

  constexpr canid_t exact_standard_mask =
    CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG;
  const can_filter filters[] = {
    {kPositionFeedbackId, exact_standard_mask},
    {kDetailedStatusId, exact_standard_mask},
    {kDiagnosticEventId, exact_standard_mask},
    {kDeviceHeartbeatId, exact_standard_mask},
    {kTrajectoryStatusId, exact_standard_mask},
  };
  if (::setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, filters, sizeof(filters)) < 0) {
    error = errno_text("setsockopt CAN_RAW_FILTER");
    close();
    return false;
  }

  ifreq request{};
  std::strncpy(request.ifr_name, interface_name.c_str(), IFNAMSIZ - 1U);
  if (::ioctl(socket_fd_, SIOCGIFINDEX, &request) < 0) {
    error = errno_text("ioctl SIOCGIFINDEX");
    close();
    return false;
  }

  sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = request.ifr_ifindex;
  if (::bind(socket_fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
    error = errno_text("bind");
    close();
    return false;
  }

  error.clear();
  return true;
}

void CanFdTransport::close()
{
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
}

bool CanFdTransport::is_open() const
{
  return socket_fd_ >= 0;
}

bool CanFdTransport::send(
  const std::uint32_t id, const std::uint8_t * data, const std::size_t size,
  std::string & error)
{
  if (!is_open()) {
    error = "CAN socket is not open";
    ++stats_.tx_errors;
    return false;
  }
  if (data == nullptr || !valid_tx_id_and_size(id, size)) {
    error = "invalid CAN FD transmit ID or payload size";
    ++stats_.tx_errors;
    return false;
  }

  canfd_frame raw{};
  raw.can_id = id;
  raw.len = static_cast<__u8>(size);
  raw.flags = CANFD_BRS;
  std::memcpy(raw.data, data, size);

  const auto written = ::write(socket_fd_, &raw, CANFD_MTU);
  if (written != CANFD_MTU) {
    error = written < 0 ? errno_text("write") : "short CAN FD write";
    ++stats_.tx_errors;
    return false;
  }
  ++stats_.transmitted;
  error.clear();
  return true;
}

ReceiveResult CanFdTransport::receive(
  CanFdFrame & frame, std::uint32_t & can_error, std::string & error)
{
  if (!is_open()) {
    error = "CAN socket is not open";
    ++stats_.rx_errors;
    return ReceiveResult::kError;
  }

  canfd_frame raw{};
  const auto received = ::read(socket_fd_, &raw, CANFD_MTU);
  if (received < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return ReceiveResult::kNoData;
    }
    error = errno_text("read");
    ++stats_.rx_errors;
    return ReceiveResult::kError;
  }

  if ((raw.can_id & CAN_ERR_FLAG) != 0U) {
    can_error = raw.can_id & CAN_ERR_MASK;
    ++stats_.can_error_frames;
    return ReceiveResult::kCanError;
  }
  if (received != CANFD_MTU || (raw.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG)) != 0U ||
    (raw.flags & CANFD_BRS) == 0U)
  {
    error = "received frame is not a standard CAN FD+BRS data frame";
    ++stats_.malformed;
    return ReceiveResult::kError;
  }

  const auto id = raw.can_id & CAN_SFF_MASK;
  const auto size = expected_size(id);
  if (size == 0U || raw.len != size) {
    error = "unexpected CAN FD receive ID or payload size";
    ++stats_.malformed;
    return ReceiveResult::kError;
  }

  CanFdFrame decoded;
  decoded.id = id;
  decoded.length = raw.len;
  std::memcpy(decoded.data.data(), raw.data, raw.len);
  decoded.received_at = std::chrono::steady_clock::now();
  frame = decoded;
  can_error = 0U;
  ++stats_.received;
  error.clear();
  return ReceiveResult::kFrame;
}

}  // namespace elfin3_canfd
