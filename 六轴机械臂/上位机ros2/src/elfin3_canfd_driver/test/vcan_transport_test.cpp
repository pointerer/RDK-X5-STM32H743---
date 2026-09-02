#include <array>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "elfin3_canfd_driver/byte_utils.hpp"
#include "elfin3_canfd_driver/can_fd_transport.hpp"
#include "elfin3_canfd_driver/crc16.hpp"
#include "elfin3_canfd_driver/decode.hpp"
#include "elfin3_canfd_driver/encode.hpp"
#include "elfin3_canfd_driver/protocol.hpp"

namespace
{

class PeerSocket
{
public:
  ~PeerSocket()
  {
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }

  bool open(const std::string & interface_name)
  {
    fd_ = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    if (fd_ < 0) {
      return false;
    }
    const int enable = 1;
    if (::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable, sizeof(enable)) < 0) {
      return false;
    }
    if (::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &enable, sizeof(enable)) < 0) {
      return false;
    }
    ifreq request{};
    std::strncpy(request.ifr_name, interface_name.c_str(), IFNAMSIZ - 1U);
    if (::ioctl(fd_, SIOCGIFINDEX, &request) < 0) {
      return false;
    }
    sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = request.ifr_ifindex;
    return ::bind(fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) == 0;
  }

  bool send(const std::uint32_t id, const std::uint8_t * data, const std::size_t size)
  {
    canfd_frame frame{};
    frame.can_id = id;
    frame.len = static_cast<__u8>(size);
    frame.flags = CANFD_BRS;
    std::memcpy(frame.data, data, size);
    return ::write(fd_, &frame, CANFD_MTU) == CANFD_MTU;
  }

  bool receive(canfd_frame & frame)
  {
    for (int attempt = 0; attempt < 100; ++attempt) {
      if (::read(fd_, &frame, CANFD_MTU) == CANFD_MTU) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
  }

private:
  int fd_{-1};
};

TEST(VcanTransport, SendsAndReceivesCanFdBrsFrames)
{
  const char * environment = std::getenv("ELFIN3_CANFD_TEST_INTERFACE");
  if (environment == nullptr || environment[0] == '\0') {
    GTEST_SKIP() << "Set ELFIN3_CANFD_TEST_INTERFACE=vcan0 to run the integration test";
  }
  const std::string interface_name(environment);

  PeerSocket peer;
  ASSERT_TRUE(peer.open(interface_name)) <<
    "Unable to open peer SocketCAN interface " << interface_name;
  elfin3_canfd::CanFdTransport transport;
  std::string error;
  ASSERT_TRUE(transport.open(interface_name, error)) << error;

  elfin3_canfd::DeviceHeartbeatFrame heartbeat{};
  heartbeat[0] = 7;
  heartbeat[1] = elfin3_canfd::kProtocolVersion;
  heartbeat[2] = 3;
  heartbeat[3] = 0x07;
  elfin3_canfd::write_u16_le(heartbeat.data() + 4U, 0x1234);
  ASSERT_TRUE(elfin3_canfd::append_crc(heartbeat.data(), heartbeat.size()));
  ASSERT_TRUE(peer.send(elfin3_canfd::kDeviceHeartbeatId, heartbeat.data(), heartbeat.size()));

  elfin3_canfd::CanFdFrame received;
  std::uint32_t can_error = 0;
  elfin3_canfd::ReceiveResult result = elfin3_canfd::ReceiveResult::kNoData;
  for (int attempt = 0; attempt < 100 && result == elfin3_canfd::ReceiveResult::kNoData;
    ++attempt)
  {
    result = transport.receive(received, can_error, error);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_EQ(result, elfin3_canfd::ReceiveResult::kFrame) << error;
  EXPECT_EQ(received.id, elfin3_canfd::kDeviceHeartbeatId);
  EXPECT_EQ(received.length, elfin3_canfd::kDeviceHeartbeatSize);

  const elfin3_canfd::TrajectoryStatusFrame trajectory_status{
    0x00, 0x02, 0x00, 0x00, 0x0c, 0x01, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xb3, 0x1d};
  ASSERT_TRUE(peer.send(
      elfin3_canfd::kTrajectoryStatusId, trajectory_status.data(), trajectory_status.size()));
  result = elfin3_canfd::ReceiveResult::kNoData;
  for (int attempt = 0; attempt < 100 && result == elfin3_canfd::ReceiveResult::kNoData;
    ++attempt)
  {
    result = transport.receive(received, can_error, error);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_EQ(result, elfin3_canfd::ReceiveResult::kFrame) << error;
  EXPECT_EQ(received.id, elfin3_canfd::kTrajectoryStatusId);
  EXPECT_EQ(received.length, elfin3_canfd::kTrajectoryStatusSize);

  elfin3_canfd::HostHeartbeat host;
  host.sequence = 8;
  host.state = elfin3_canfd::HostState::kReady;
  elfin3_canfd::HostHeartbeatFrame host_frame{};
  ASSERT_TRUE(elfin3_canfd::encode_host_heartbeat(host, host_frame));
  ASSERT_TRUE(transport.send(
      elfin3_canfd::kHostHeartbeatId, host_frame.data(), host_frame.size(), error)) << error;

  canfd_frame peer_received{};
  ASSERT_TRUE(peer.receive(peer_received));
  EXPECT_EQ(peer_received.can_id & CAN_SFF_MASK, elfin3_canfd::kHostHeartbeatId);
  EXPECT_EQ(peer_received.len, elfin3_canfd::kHostHeartbeatSize);
  EXPECT_NE(peer_received.flags & CANFD_BRS, 0U);
}

}  // namespace
