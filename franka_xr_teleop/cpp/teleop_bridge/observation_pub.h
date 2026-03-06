#pragma once

#include <cstdint>
#include <string>

#include "common_types.h"

namespace teleop {

class ObservationPublisher {
 public:
  ObservationPublisher(std::string dst_ip, uint16_t dst_port);
  ~ObservationPublisher();

  bool Start();
  void Stop();

  bool Publish(const RobotObservation& obs);

 private:
  std::string ToJson(const RobotObservation& obs) const;

  std::string dst_ip_;
  uint16_t dst_port_;
  int sock_ = -1;
  bool started_ = false;
};

}  // namespace teleop
