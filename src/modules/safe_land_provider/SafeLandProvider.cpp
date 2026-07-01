#include "SafeLandProvider.hpp"
#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <math.h>

ModuleBase::Descriptor SafeLandProvider::desc{task_spawn, custom_command, print_usage};

// Lake bounds in PX4 NED local frame (x=North, y=East).
// Derived from lake_safe_land.sdf: ENU pose (0, 60, 0), size 40x30 m.
//   SDF ENU x (East)  -> PX4 y: [-20, +20]
//   SDF ENU y (North) -> PX4 x: [45,  +75]
static constexpr float LAKE_N_MIN = 45.f;
static constexpr float LAKE_N_MAX = 75.f;
static constexpr float LAKE_E_MIN = -20.f;
static constexpr float LAKE_E_MAX =  20.f;

SafeLandProvider::SafeLandProvider() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

bool SafeLandProvider::init()
{
	ScheduleOnInterval(1_s);
	PX4_INFO("safe_land_provider: started, 1 Hz interval");
	return true;
}

void SafeLandProvider::Run()
{
	if (should_exit()) {
		PX4_INFO("safe_land_provider: stopping");
		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}

	_local_pos_sub.update();
	const vehicle_local_position_s &lp = _local_pos_sub.get();

	PX4_DEBUG("safe_land_provider: pos xy_valid=%d x=%.2f y=%.2f",
		  (int)lp.xy_valid, (double)lp.x, (double)lp.y);

	safe_land_topic_s msg{};
	msg.timestamp = hrt_absolute_time();

	const bool inside = lp.xy_valid
			    && lp.x >= LAKE_N_MIN && lp.x <= LAKE_N_MAX
			    && lp.y >= LAKE_E_MIN && lp.y <= LAKE_E_MAX;

	if (inside && !_was_inside) {
		PX4_INFO("safe_land_provider: entered lake region (x=%.1f y=%.1f)",
			 (double)lp.x, (double)lp.y);
	} else if (!inside && _was_inside) {
		PX4_INFO("safe_land_provider: left lake region");
	}

	if (inside) {
		// Wait for EKF2 to set a valid GPS reference origin before projecting.
		// xy_valid alone doesn't guarantee ref_lat/ref_lon are populated.
		const bool ref_valid = (lp.ref_lat * lp.ref_lat + lp.ref_lon * lp.ref_lon) > 1e-15;

		if (!ref_valid) {
			msg.valid = false;
			_pub.publish(msg);
			_was_inside = inside;
			return;
		}

		const float d_south = lp.x - LAKE_N_MIN;
		const float d_north = LAKE_N_MAX - lp.x;
		const float d_west  = lp.y - LAKE_E_MIN;
		const float d_east  = LAKE_E_MAX - lp.y;

		float ex = lp.x;
		float ey = lp.y;

		if (d_south <= d_north && d_south <= d_west && d_south <= d_east) {
			ex = LAKE_N_MIN;
		} else if (d_north <= d_west && d_north <= d_east) {
			ex = LAKE_N_MAX;
		} else if (d_west <= d_east) {
			ey = LAKE_E_MIN;
		} else {
			ey = LAKE_E_MAX;
		}

		MapProjection ref;
		ref.initReference(lp.ref_lat, lp.ref_lon);
		double lat_esc, lon_esc;
		ref.reproject(ex, ey, lat_esc, lon_esc);

		msg.valid = true;
		msg.lat = lat_esc;
		msg.lon = lon_esc;
		msg.alt = lp.ref_alt;

		PX4_INFO("safe_land_provider: publish escape lat=%.6f lon=%.6f alt=%.2f",
			 msg.lat, msg.lon, (double)msg.alt);

	} else {
		msg.valid = false;
		PX4_DEBUG("safe_land_provider: outside lake zone");
	}

	_pub.publish(msg);
	_was_inside = inside;
}

int SafeLandProvider::task_spawn(int argc, char *argv[])
{
	SafeLandProvider *instance = new SafeLandProvider();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

int SafeLandProvider::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int SafeLandProvider::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(R"DESCR_STR(
### Description
Publishes safe landing zone candidates on the safe_land_topic uORB topic.
Monitors vehicle_local_position and emits the nearest point outside the lake
boundary when the drone is detected inside the lake region. Emits valid=false
when the drone is outside the unsafe zone.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("safe_land_provider", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int safe_land_provider_main(int argc, char *argv[])
{
	return ModuleBase::main(SafeLandProvider::desc, argc, argv);
}
