/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "VehicleInfoSender.hpp"

VehicleInfoSender::VehicleInfoSender() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

VehicleInfoSender::~VehicleInfoSender()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool VehicleInfoSender::init()
{
	ScheduleOnInterval(1_s);

	return true;
}

void VehicleInfoSender::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)
	}


	vehicle_global_position_s global_pos;
	vehicle_attitude_s attitude;
	vehicle_local_position_s local_pos;
#ifdef __PX4_NUTTX
	sensor_gps_s sensor_gps;
#endif
	vehicle_info_s vehicle_info{};

	_vehicle_global_position_sub.update(&global_pos);
	_vehicle_attitude_sub.update(&attitude);
	_vehicle_local_position_sub.update(&local_pos);
#ifdef __PX4_NUTTX
	_sensor_gps_sub.update(&sensor_gps);
#endif

	vehicle_info.timestamp = hrt_absolute_time();

	vehicle_info.team_id = _param_team_id.get();

	vehicle_info.latitude = global_pos.lat;
	vehicle_info.longitude = global_pos.lon;
	vehicle_info.altitude = global_pos.alt;

	matrix::Eulerf euler(matrix::Quatf(attitude.q));

	vehicle_info.roll = euler.phi() * (180.0f / M_PI_F);
	vehicle_info.pitch = euler.theta() * (180.0f / M_PI_F);
	vehicle_info.heading = euler.psi() * (180.0f / M_PI_F);

	vehicle_info.velocity = sqrtf((local_pos.vx * local_pos.vx) +
                                (local_pos.vy * local_pos.vy) +
                                (local_pos.vz * local_pos.vz));

#ifdef __PX4_NUTTX
	vehicle_info.time_utc_usec = sensor_gps.time_utc_usec;
#else
	auto timenow = std::chrono::system_clock::now();
	auto timenow_usec = std::chrono::duration_cast<std::chrono::microseconds>(timenow.time_since_epoch()).count();
	vehicle_info.time_utc_usec = timenow_usec;
#endif

	_vehicle_info_pub.publish(vehicle_info);


	perf_end(_loop_perf);
}

int VehicleInfoSender::task_spawn(int argc, char *argv[])
{
	VehicleInfoSender *instance = new VehicleInfoSender();

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

int VehicleInfoSender::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int VehicleInfoSender::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int VehicleInfoSender::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
The module that publishes data to the vehicle_info topic every second.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("vehicle_info_sender", "sender");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int vehicle_info_sender_main(int argc, char *argv[])
{
	return VehicleInfoSender::main(argc, argv);
}
