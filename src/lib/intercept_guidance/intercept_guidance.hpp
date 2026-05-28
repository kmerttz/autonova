#pragma once

#include <matrix/math.hpp>

using namespace matrix;

class InterceptGuidance
{
public:
	InterceptGuidance() = default;
	~InterceptGuidance() = default;

	/**
	 * @brief Calculates a setpoint lagging behind the target by a specified distance.
	 *
	 * This function generates a setpoint on the line connecting the target and
	 * vehicle, placing the setpoint 'lag_distance' meters behind the target.
	 *
	 * @param vehicle_position Current vehicle position in global frame
	 * @param target_position Target vehicle position in global frame
	 * @return matrix::Vector3f Calculated position setpoint in NED frame
	 */
	Vector3f calculateSetpoint(const Vector3f &vehicle_ned_position,
				   const Vector3f &target_ned_position,
				   const float &lag_distance);

private:


};
