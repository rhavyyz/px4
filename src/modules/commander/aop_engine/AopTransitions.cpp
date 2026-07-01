#include "AopTransitions.hpp"

#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_status.h>

static constexpr uint8_t OVERRIDE_DISARM    = 0xFE;
static constexpr uint8_t OVERRIDE_TERMINATE = 0xFF;

Transition go_to_state(uint8_t nav_state, uint8_t fallback)
{
	return {nav_state, fallback, {}};
}

Transition go_to_safe_land()
{
	return {
		vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER,
		vehicle_status_s::NAVIGATION_STATE_DESCEND,
		[](const AopTelemetry &t, const AopEnv &, const AopMission *)
		{
			static uORB::Publication<vehicle_command_s> pub{ORB_ID(vehicle_command)};
			vehicle_command_s cmd{};
			cmd.timestamp = hrt_absolute_time();
			cmd.command   = vehicle_command_s::VEHICLE_CMD_DO_REPOSITION;
			cmd.param5    = t.safe_land.lat;
			cmd.param6    = t.safe_land.lon;
			cmd.param7    = static_cast<double>(t.safe_land.alt);
			cmd.param2    = 1;
			pub.publish(cmd);
		}
	};
}

Transition go_to_coord(uint8_t nav_state, GeoCoord coord)
{
	return {
		nav_state,
		vehicle_status_s::NAVIGATION_STATE_DESCEND,
		[coord](const AopTelemetry &, const AopEnv &, const AopMission *)
		{
			static uORB::Publication<vehicle_command_s> pub{ORB_ID(vehicle_command)};
			vehicle_command_s cmd{};
			cmd.timestamp = hrt_absolute_time();
			cmd.command   = vehicle_command_s::VEHICLE_CMD_DO_REPOSITION;
			cmd.param5    = coord.lat;
			cmd.param6    = coord.lon;
			cmd.param7    = static_cast<double>(coord.alt);
			cmd.param2    = 1;
			pub.publish(cmd);
		}
	};
}

Transition go_to_safe_land_then_land()
{
	return {
		vehicle_status_s::NAVIGATION_STATE_AUTO_LAND,
		vehicle_status_s::NAVIGATION_STATE_DESCEND,
		[](const AopTelemetry &t, const AopEnv &, const AopMission *)
		{
			static uORB::Publication<vehicle_command_s> pub{ORB_ID(vehicle_command)};
			vehicle_command_s cmd{};
			cmd.timestamp = hrt_absolute_time();
			cmd.command   = vehicle_command_s::VEHICLE_CMD_DO_REPOSITION;
			cmd.param5    = t.safe_land.lat;
			cmd.param6    = t.safe_land.lon;
			cmd.param7    = static_cast<double>(t.safe_land.alt);
			cmd.param2    = 1;
			pub.publish(cmd);
		}
	};
}

Transition go_to_disarm()
{
	return {OVERRIDE_DISARM, 0, {}};
}

Transition go_to_terminate()
{
	return {OVERRIDE_TERMINATE, 0, {}};
}
