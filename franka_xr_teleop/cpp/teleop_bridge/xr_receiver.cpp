#include "xr_receiver.h"

#include <algorithm>
#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>

namespace teleop {
namespace {

#pragma pack(push, 1)
struct XRWirePacket {
  uint64_t timestamp_ns;
  uint64_t sequence_id;
  uint8_t teleop_enabled;
  uint8_t clutch_pressed;
  double target_position_xyz[3];
  double target_orientation_xyzw[4];
  float gripper_command;
  uint8_t reserved[3];
};
#pragma pack(pop)

static_assert(sizeof(XRWirePacket) == 81, "Unexpected XR packet size.");

uint64_t MonotonicNowNs() {
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

bool IsFiniteArray(const double* data, size_t count) {
  for (size_t i = 0; i < count; ++i) {
    if (!std::isfinite(data[i])) {
      return false;
    }
  }
  return true;
}

bool ParsePacket(const uint8_t* bytes, size_t len, XRCommand* out) {
  if (len < sizeof(XRWirePacket)) {
    return false;
  }

  XRWirePacket wire{};
  std::memcpy(&wire, bytes, sizeof(wire));

  if (!IsFiniteArray(wire.target_position_xyz, 3) ||
      !IsFiniteArray(wire.target_orientation_xyzw, 4) ||
      !std::isfinite(static_cast<double>(wire.gripper_command))) {
    return false;
  }

  out->timestamp_ns = wire.timestamp_ns;
  out->sequence_id = wire.sequence_id;
  out->teleop_enabled = wire.teleop_enabled != 0;
  out->clutch_pressed = wire.clutch_pressed != 0;
  out->target_position_xyz = {
      wire.target_position_xyz[0], wire.target_position_xyz[1], wire.target_position_xyz[2]};
  out->target_orientation_xyzw = {wire.target_orientation_xyzw[0],
                                  wire.target_orientation_xyzw[1],
                                  wire.target_orientation_xyzw[2],
                                  wire.target_orientation_xyzw[3]};
  out->gripper_command = std::clamp(static_cast<double>(wire.gripper_command), 0.0, 1.0);

  // Guard against a sender that does not fill timestamps yet.
  if (out->timestamp_ns == 0) {
    out->timestamp_ns = MonotonicNowNs();
  }
  return true;
}

}  // namespace

XRReceiver::XRReceiver(std::string bind_ip,
                       uint16_t bind_port,
                       LatestCommandBuffer* cmd_buffer,
                       std::atomic<bool>* stop_requested)
    : bind_ip_(std::move(bind_ip)),
      bind_port_(bind_port),
      cmd_buffer_(cmd_buffer),
      stop_requested_(stop_requested) {}

XRReceiver::~XRReceiver() {
  Stop();
}

bool XRReceiver::Start() {
  if (running_.exchange(true, std::memory_order_acq_rel)) {
    return true;
  }
  thread_ = std::thread(&XRReceiver::ReceiveLoop, this);
  return true;
}

void XRReceiver::Stop() {
  running_.store(false, std::memory_order_release);
  if (thread_.joinable()) {
    thread_.join();
  }
}

void XRReceiver::ReceiveLoop() {
  const int sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    std::cerr << "XRReceiver: failed to create UDP socket\n";
    running_.store(false, std::memory_order_release);
    return;
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(bind_port_);
  if (bind_ip_.empty() || bind_ip_ == "0.0.0.0") {
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
  } else if (inet_pton(AF_INET, bind_ip_.c_str(), &addr.sin_addr) != 1) {
    std::cerr << "XRReceiver: invalid bind IP " << bind_ip_ << "\n";
    close(sock);
    running_.store(false, std::memory_order_release);
    return;
  }

  int enable_reuse = 1;
  setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &enable_reuse, sizeof(enable_reuse));

  if (bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    std::cerr << "XRReceiver: bind failed on UDP " << bind_port_ << "\n";
    close(sock);
    running_.store(false, std::memory_order_release);
    return;
  }

  const int flags = fcntl(sock, F_GETFL, 0);
  if (flags >= 0) {
    (void)fcntl(sock, F_SETFL, flags | O_NONBLOCK);
  }

  std::array<uint8_t, 512> buffer{};
  pollfd pfd{};
  pfd.fd = sock;
  pfd.events = POLLIN;

  while (running_.load(std::memory_order_acquire) &&
         !stop_requested_->load(std::memory_order_acquire)) {
    const int poll_ret = poll(&pfd, 1, 20);
    if (poll_ret <= 0) {
      continue;
    }

    if ((pfd.revents & POLLIN) == 0) {
      continue;
    }

    sockaddr_in src{};
    socklen_t src_len = sizeof(src);
    const ssize_t n = recvfrom(sock,
                               buffer.data(),
                               buffer.size(),
                               0,
                               reinterpret_cast<sockaddr*>(&src),
                               &src_len);
    if (n < 0) {
      if (errno == EWOULDBLOCK || errno == EAGAIN) {
        continue;
      }
      continue;
    }

    XRCommand cmd{};
    if (!ParsePacket(buffer.data(), static_cast<size_t>(n), &cmd)) {
      dropped_count_.fetch_add(1, std::memory_order_acq_rel);
      continue;
    }

    cmd_buffer_->Publish(cmd);
    received_count_.fetch_add(1, std::memory_order_acq_rel);
    last_packet_time_ns_.store(MonotonicNowNs(), std::memory_order_release);
  }

  close(sock);
  running_.store(false, std::memory_order_release);
}

}  // namespace teleop
