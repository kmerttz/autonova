#pragma once

#include <cstdint>
#include <matrix/math.hpp>

using namespace matrix;

class TargetUavPredictor
{
public:
	TargetUavPredictor() = default;
	~TargetUavPredictor() = default;

	void assignTargetInfo(const Vector3f &target_last_pos_ned,
				const Eulerf &target_last_attitude,
				const float &target_last_velocity,
				const uint64_t &target_last_seen);

	Vector3f estimateTargetPosDelta(const uint64_t &time_utc_usec);

private:

	Vector3f 	_target_last_pos_ned;
	Eulerf		_target_last_attitude;
	float		_target_last_velocity;
	uint64_t	_target_last_seen;

	Vector2f	_target_last_velocity_vector; // 0: Horizontal 1: Vertical
	float		_target_yawrate;

	Vector3f	_target_est_delta_ned;

	const float 	_gravity_constant = 9.81f;
};
