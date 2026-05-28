#include "target_uav_predictor.hpp"


void TargetUavPredictor::assignTargetInfo(const Vector3f &target_last_pos_ned,
						const Eulerf &target_last_attitude,
						const float &target_last_velocity,
						const uint64_t &target_last_seen)
{
	_target_last_pos_ned 	= target_last_pos_ned;
	_target_last_attitude	= target_last_attitude;
	_target_last_velocity 	= target_last_velocity;
	_target_last_seen	= target_last_seen;

}


Vector3f TargetUavPredictor::estimateTargetPosDelta(const uint64_t &time_utc_usec)
{
	float delta_time_s = (time_utc_usec - _target_last_seen) / 1e6f;

	_target_last_velocity_vector(0) = _target_last_velocity * cosf(_target_last_attitude.theta());
	_target_last_velocity_vector(1) = _target_last_velocity * sinf(_target_last_attitude.theta());

	_target_yawrate = _gravity_constant * tanf(_target_last_attitude.phi()) / _target_last_velocity_vector(0);

	if (fabsf(_target_yawrate) < 0.001f){

		_target_est_delta_ned(0) = _target_last_velocity_vector(0) * cosf(_target_last_attitude.psi()) * delta_time_s;

		_target_est_delta_ned(1) = _target_last_velocity_vector(0) * sinf(_target_last_attitude.psi()) * delta_time_s;

	} else {

		_target_est_delta_ned(0) = (_target_last_velocity_vector(0) / _target_yawrate)
							* (sinf(_target_last_attitude.psi() + _target_yawrate * delta_time_s) - sinf(_target_last_attitude.psi()));

		_target_est_delta_ned(1) = (_target_last_velocity_vector(0) / _target_yawrate)
							* (-cosf(_target_last_attitude.psi() + _target_yawrate * delta_time_s) + cosf(_target_last_attitude.psi()));

	}

	_target_est_delta_ned(2) = -_target_last_velocity_vector(1) * delta_time_s;


	return _target_est_delta_ned;
}
