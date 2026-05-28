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
/**
 * @file FlightTaskIntercept.cpp
 */

#include "FlightTaskIntercept.hpp"

bool FlightTaskIntercept::activate(const trajectory_setpoint_s &last_setpoint)
{
	bool ret = FlightTask::activate(last_setpoint);
	return ret;
}

bool FlightTaskIntercept::update()
{
	_vehicle_local_position_sub.update(&_vehicle_local_position);
	_vehicle_position_ned = Vector3f(_vehicle_local_position.x,
					_vehicle_local_position.y,
					_vehicle_local_position.z);


	if (_vehicle_local_position.ref_timestamp != _ned_ref_timestamp) {
		_map_projection.initReference(_vehicle_local_position.ref_lat,
						_vehicle_local_position.ref_lon);

		_ned_ref_timestamp = _vehicle_local_position.ref_timestamp;
	}


	if (_target_uav_info_sub.updated()) {

		_target_uav_info_sub.update(&_target_uav_info);


		_target_last_position_global = Vector3d(_target_uav_info.latitude,
						_target_uav_info.longitude,
						_target_uav_info.altitude);

		_map_projection.project(_target_last_position_global(0), _target_last_position_global(1),
				_target_last_position_ned(0), _target_last_position_ned(1));
		_target_last_position_ned(2) = -((float)_target_last_position_global(2) - _vehicle_local_position.ref_alt);

		_target_last_attitude 	= Eulerf(math::radians(_target_uav_info.roll),
						math::radians(_target_uav_info.pitch),
						math::radians(_target_uav_info.heading));

		_target_last_velocity	= _target_uav_info.velocity;

		_target_last_seen	= _target_uav_info.time_utc_usec;

		_target_uav_predictor.assignTargetInfo(_target_last_position_ned,
							_target_last_attitude,
							_target_last_velocity,
							_target_last_seen);
	}

	if (_sensor_gps_sub.updated()) {
		_sensor_gps_sub.update(&_sensor_gps);
	}

	_target_est_delta_position_ned = _target_uav_predictor.estimateTargetPosDelta(_sensor_gps.time_utc_usec);
	_target_position_est_ned = _target_last_position_ned + _target_est_delta_position_ned;

	_position_setpoint = _intercept_guidance.calculateSetpoint(_vehicle_position_ned, _target_position_est_ned);

	return true;
}
