#include "ZhengYuDemo.hpp"

#include <float.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/events.h>


using namespace matrix;

ZhengYuDemo::ZhengYuDemo() :
	SuperBlock(nullptr, "MPC"),
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)

{
	parameters_update(true);
}


ZhengYuDemo::~ZhengYuDemo()
{
	perf_free(_cycle_perf);
}

bool ZhengYuDemo::init()
{
	if (!_local_pos_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	_time_stamp_last_loop = hrt_absolute_time();
	ScheduleNow();

	return true;
}

void ZhengYuDemo::parameters_update(bool force)
{
	
}



void ZhengYuDemo::Run()
{
	if (should_exit()) {
		_local_pos_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// reschedule backup
	ScheduleDelayed(100_ms);

	parameters_update(false);

	perf_begin(_cycle_perf);
	vehicle_local_position_s local_pos;

	if (_local_pos_sub.update(&local_pos)) {
		const hrt_abstime time_stamp_now = local_pos.timestamp_sample;
		const float dt = math::constrain(((time_stamp_now - _time_stamp_last_loop) * 1e-6f), 0.002f, 0.04f);
		_time_stamp_last_loop = time_stamp_now;

		printf("Hello Sky! %f\r\n", (double)dt);
	}
	perf_end(_cycle_perf);
}



int ZhengYuDemo::task_spawn(int argc, char *argv[])
{

	ZhengYuDemo *instance = new ZhengYuDemo();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int ZhengYuDemo::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ZhengYuDemo::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
	ZhengYu Demo.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("zhengyu_demo", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int zhengyu_demo_main(int argc, char *argv[])
{
	return ZhengYuDemo::main(argc, argv);
}
