/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "YuFengDemo.hpp"

#include <float.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/events.h>


using namespace matrix;

YuFengDemo::YuFengDemo() :
	SuperBlock(nullptr, "MPC"),
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)

{
	parameters_update(true);
}


YuFengDemo::~YuFengDemo()
{
	perf_free(_cycle_perf);
}

bool YuFengDemo::init()
{
	if (!_local_pos_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	_time_stamp_last_loop = hrt_absolute_time();
	ScheduleNow();

	return true;
}

void YuFengDemo::parameters_update(bool force)
{
	if (_parameter_update_sub.updated() || force){
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		ModuleParams::updateParams();
		SuperBlock::updateParams();

		yu_feng_en = _param_yu_feng_en.get();
		yu_feng_len = _param_yu_feng_len.get();
	}
}



void YuFengDemo::Run()
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

		if (yu_feng_en) {
			printf("Hello sky! %f\r\n", (double)dt);
		}
		else{
			printf("Hello Sky! %f\r\n", (double)yu_feng_len);
		}

		sensor_combined_s imu;
		if (_sensor_combined_sub.update(&imu)){
			yufeng_demo_s yufeng;
			yufeng.enable = true;
			yufeng.timestamp = hrt_absolute_time();
			yufeng.acc[0] = imu.accelerometer_m_s2[0];
			yufeng.acc[1] = imu.accelerometer_m_s2[1];
			yufeng.acc[2] = imu.accelerometer_m_s2[2];
			yufeng.acc_norm = sqrt(yufeng.acc[0]*yufeng.acc[0] + yufeng.acc[1]*yufeng.acc[1] + yufeng.acc[2]*yufeng.acc[2]);
			_yufeng_demo_pub.publish(yufeng);
		}
	}
	perf_end(_cycle_perf);
}



int YuFengDemo::task_spawn(int argc, char *argv[])
{

	YuFengDemo *instance = new YuFengDemo();

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

int YuFengDemo::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int YuFengDemo::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
	YuFeng Demo.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("yufeng_demo", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int yufeng_demo_main(int argc, char *argv[])
{
	return YuFengDemo::main(argc, argv);
}
