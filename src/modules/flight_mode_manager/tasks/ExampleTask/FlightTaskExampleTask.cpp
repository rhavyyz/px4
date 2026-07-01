#include "FlightTaskExampleTask.hpp"

bool FlightTaskExampleTask::activate(const trajectory_setpoint_s &last_setpoint)
{
  bool ret = FlightTask::activate(last_setpoint);
  PX4_INFO("ExampleTask activate was called! ret: %d", ret); // report if activation was successful
  return ret;
}

bool FlightTaskExampleTask::update()
{
  PX4_INFO("ExampleTask update was called!"); // report update
  return true;
}
