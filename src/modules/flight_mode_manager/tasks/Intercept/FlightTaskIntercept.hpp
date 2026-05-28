/****************************************************************************
 *
 *   Copyright (c) 2019-2023 PX4 Development Team. All rights reserved.
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

#pragma once

#include "FlightTask.hpp"

#include <lib/intercept_guidance/intercept_guidance.hpp>
#include <lib/target_uav_predictor/target_uav_predictor.hpp>
#include <lib/geo/geo.h>

#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/target_uav_info.h>


using namespace matrix;

class FlightTaskIntercept : public FlightTask
{
public:

	FlightTaskIntercept() = default;
	virtual ~FlightTaskIntercept() = default;

	bool activate(const trajectory_setpoint_s &last_setpoint) override;
	bool update() override;

private:

	uORB::Subscription		_vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	vehicle_local_position_s	_vehicle_local_position{};

	uORB::Subscription		_sensor_gps_sub{ORB_ID(sensor_gps)};
	sensor_gps_s			_sensor_gps{};

	uORB::Subscription		_target_uav_info_sub{ORB_ID(target_uav_info)};
	target_uav_info_s		_target_uav_info{};

	InterceptGuidance 	_intercept_guidance;
	TargetUavPredictor	_target_uav_predictor;
	MapProjection 		_map_projection;

	Vector3d 	_target_last_position_global;
	Vector3f 	_target_last_position_ned;

	Eulerf 		_target_last_attitude;
	float 		_target_last_velocity;
	uint64_t	_target_last_seen;

	Vector3f	_target_est_delta_position_ned{};
	Vector3f	_target_position_est_ned{};

	Vector3f	_vehicle_position_ned{};

	hrt_abstime	_ned_ref_timestamp{0};

};
