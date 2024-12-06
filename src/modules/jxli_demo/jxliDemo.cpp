#include "jxliDemo.hpp"

#include <float.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/events.h>


using namespace matrix;

jxliDemo::jxliDemo() :
	SuperBlock(nullptr, "MPC"),
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)

{
	parameters_update(true);
}


jxliDemo::~jxliDemo()
{
	perf_free(_cycle_perf);
}

bool jxliDemo::init()
{
	if (!_local_pos_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	_time_stamp_last_loop = hrt_absolute_time();
	ScheduleNow();

	return true;
}

void jxliDemo::parameters_update(bool force)
{
	if (_parameter_update_sub.updated() || force){
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		ModuleParams::updateParams();
		SuperBlock::updateParams();

		jx_li_en = _param_jx_li_en.get();
		jx_li_len = _param_jx_li_len.get();
	}
}



void jxliDemo::Run()
{
	if (should_exit()) {
		_local_pos_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// reschedule backup
	ScheduleDelayed(1000_ms);

	parameters_update(false);

	perf_begin(_cycle_perf);
	vehicle_local_position_s local_pos;

	if (_local_pos_sub.update(&local_pos)) {
		const hrt_abstime time_stamp_now = local_pos.timestamp_sample;
		const float dt = math::constrain(((time_stamp_now - _time_stamp_last_loop) * 1e-6f), 0.002f, 0.04f);
		_time_stamp_last_loop = time_stamp_now;

        	if (jx_li_en) {
			printf("Hello sky! %f\r\n", (double)dt);
		}
		else{
			printf("Hello Sky! %f\r\n", (double)jx_li_len);
		}

		sensor_combined_s imu;
		if (_sensor_combined_sub.update(&imu)){
			jxli_demo_s jxli;
			jxli.enable = true;
			jxli.timestamp = hrt_absolute_time();
			jxli.acc[0] = imu.accelerometer_m_s2[0];
			jxli.acc[1] = imu.accelerometer_m_s2[1];
			jxli.acc[2] = imu.accelerometer_m_s2[2];
			jxli.acc_norm = sqrt(jxli.acc[0]*jxli.acc[0] + jxli.acc[1]*jxli.acc[1] + jxli.acc[2]*jxli.acc[2]);
			_jxli_demo_pub.publish(jxli);
		}
        }
	perf_end(_cycle_perf);
}



int jxliDemo::task_spawn(int argc, char *argv[])
{

	jxliDemo *instance = new jxliDemo();

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

int jxliDemo::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int jxliDemo::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
	jxli Demo.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("jxli_demo", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int jxli_demo_main(int argc, char *argv[])
{
	return jxliDemo::main(argc, argv);
}
