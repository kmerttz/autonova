#include "intercept_guidance.hpp"

Vector3f InterceptGuidance::calculateSetpoint(const Vector3f &vehicle_ned_position,
                                              const Vector3f &target_ned_position,
					      const float &lag_distance)
{

    Vector3f target_to_vehicle_vector = vehicle_ned_position - target_ned_position;

    target_to_vehicle_vector.normalize();

    Vector3f setpoint_ned = target_ned_position + (target_to_vehicle_vector * lag_distance);

    return setpoint_ned;
}
