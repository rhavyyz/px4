#pragma once

#include "FlightTask.hpp"

class FlightTaskExampleTask : public FlightTask
{
public:
  FlightTaskExampleTask() = default;
  virtual ~FlightTaskExampleTask() = default;

  bool update();
  bool activate(const trajectory_setpoint_s &last_setpoint) override;

private:
  float _origin_z{0.f};
};
