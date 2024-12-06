#pragma once


#include <drivers/drv_hrt.h>
#include <lib/controllib/blocks.hpp>
#include <lib/hysteresis/hysteresis.h>
#include <lib/perf/perf_counter.h>
#include <lib/slew_rate/SlewRateYaw.hpp>
#include <lib/systemlib/mavlink_log.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/hover_thrust_estimate.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_constraints.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/jxli_demo.h>
#include <uORB/topics/sensor_combined.h>


using namespace time_literals;

class jxliDemo : public ModuleBase<jxliDemo>, public control::SuperBlock,
	public ModuleParams, public px4::ScheduledWorkItem
{
public:
	jxliDemo();
	~jxliDemo() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	hrt_abstime	_time_stamp_last_loop{0};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time")};

	uORB::SubscriptionCallbackWorkItem _local_pos_sub {this, ORB_ID(vehicle_local_position)};	/**< vehicle local position */
        uORB::SubscriptionInterval _parameter_update_sub {ORB_ID(parameter_update), 1_s};
	uORB::Subscription _sensor_combined_sub {ORB_ID(sensor_combined)};
	uORB::Publication<jxli_demo_s> _jxli_demo_pub {ORB_ID(jxli_demo)};	/**< vehicle local position setpoint publication */

	void parameters_update(bool force);

	bool jx_li_en;
	float jx_li_len;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::JX_LI_LEN>)     _param_jx_li_len,
		(ParamInt<px4::params::JX_LI_EN>)        _param_jx_li_en
	)

};

