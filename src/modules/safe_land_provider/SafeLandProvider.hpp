#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/safe_land_topic.h>
#include <uORB/topics/vehicle_local_position.h>

using namespace time_literals;

class SafeLandProvider : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	SafeLandProvider();
	~SafeLandProvider() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	uORB::Publication<safe_land_topic_s>              _pub{ORB_ID(safe_land_topic)};
	uORB::SubscriptionData<vehicle_local_position_s>  _local_pos_sub{ORB_ID(vehicle_local_position)};

	bool _was_inside{false};
};

extern "C" __EXPORT int safe_land_provider_main(int argc, char *argv[]);
