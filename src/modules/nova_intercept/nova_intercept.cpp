// ---------------   بِسْــــمِ اللّٰهِ الرَّحْمَـنِ الرَّحِيـمِ   ---------------

#include "nova_intercept.hpp"

NovaIntercept::NovaIntercept() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

NovaIntercept::~NovaIntercept()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}


void NovaIntercept::RegisterInterceptMode()
{
	register_ext_component_request_s register_ext_component_request{};
	register_ext_component_request.timestamp = hrt_absolute_time();
	strncpy(register_ext_component_request.name, "Intercept", sizeof(register_ext_component_request.name) - 1);
	register_ext_component_request.request_id = _mode_request_id;
	register_ext_component_request.px4_ros2_api_version = 1;
	register_ext_component_request.register_arming_check = true;
	register_ext_component_request.register_mode = true;
	_register_ext_component_request_pub.publish(register_ext_component_request);
}


void NovaIntercept::UnregisterInterceptMode(int8_t arming_check_id, int8_t mode_id)
{
	unregister_ext_component_s unregister_ext_component{};
	unregister_ext_component.timestamp = hrt_absolute_time();
	strncpy(unregister_ext_component.name, "Intercept", sizeof(unregister_ext_component.name) - 1);
	unregister_ext_component.arming_check_id = arming_check_id;
	unregister_ext_component.mode_id = mode_id;
	_unregister_ext_component_pub.publish(unregister_ext_component);
}


void NovaIntercept::ConfigureInterceptMode(int8_t mode_id)
{
	vehicle_control_mode_s vehicle_control_mode{};

	vehicle_control_mode.source_id = mode_id;

	vehicle_control_mode.flag_control_auto_enabled = true;
	vehicle_control_mode.flag_control_manual_enabled = false;

	vehicle_control_mode.flag_control_position_enabled = true;
	vehicle_control_mode.flag_control_velocity_enabled = true;
	vehicle_control_mode.flag_control_altitude_enabled = true;
	vehicle_control_mode.flag_control_climb_rate_enabled = true;

	vehicle_control_mode.flag_control_attitude_enabled = true;
	vehicle_control_mode.flag_control_rates_enabled = true;
	vehicle_control_mode.flag_control_allocation_enabled = true;
	vehicle_control_mode.flag_control_offboard_enabled = true;

	vehicle_control_mode.flag_control_termination_enabled = false;

	_config_control_setpoints_pub.publish(vehicle_control_mode);
}


void NovaIntercept::ReplyToArmingCheck(int8_t request_id)
{
	arming_check_reply_s arming_check_reply;
	arming_check_reply.timestamp = hrt_absolute_time();
	arming_check_reply.request_id = request_id;
	arming_check_reply.registration_id = _arming_check_id;
	arming_check_reply.health_component_index = arming_check_reply.HEALTH_COMPONENT_INDEX_NONE;
	arming_check_reply.num_events = 0;
	arming_check_reply.can_arm_and_run = true;
	arming_check_reply.mode_req_angular_velocity = true;
	arming_check_reply.mode_req_local_position = true;
	arming_check_reply.mode_req_attitude = true;
	arming_check_reply.mode_req_local_alt = true;
	arming_check_reply.mode_req_home_position = false;
	arming_check_reply.mode_req_mission = false;
	arming_check_reply.mode_req_global_position = true;
	arming_check_reply.mode_req_prevent_arming = false;
	arming_check_reply.mode_req_manual_control = false;
	_arming_check_reply_pub.publish(arming_check_reply);
}


void NovaIntercept::CheckModeRegistration()
{
	register_ext_component_reply_s register_ext_component_reply;
	int tries = register_ext_component_reply.ORB_QUEUE_LENGTH;

	while (_register_ext_component_reply_sub.update(&register_ext_component_reply) && --tries >= 0) {
		if (register_ext_component_reply.request_id == _mode_request_id && register_ext_component_reply.success) {
			_arming_check_id = register_ext_component_reply.arming_check_id;
			_mode_id = register_ext_component_reply.mode_id;
			PX4_INFO("Intercept mode registration successful, arming_check_id: %d, mode_id: %d", _arming_check_id, _mode_id);
			ConfigureInterceptMode(_mode_id);
			break;
		}
	}
}

void NovaIntercept::PopulatePositionSetpoint()
{

	if (_vehicle_local_position_sub.update(&_vehicle_local_position)) {
		_vehicle_position_ned = Vector3f(_vehicle_local_position.x,
						_vehicle_local_position.y,
						_vehicle_local_position.z);
	}

	if (_target_uav_info_sub.update(&_target_uav_info)) {

		Vector3d target_last_position_global = Vector3d(_target_uav_info.latitude,
								_target_uav_info.longitude,
								_target_uav_info.altitude);

		_map_projection.project(target_last_position_global(0),
					target_last_position_global(1),
					_target_last_position_ned(0),
					_target_last_position_ned(1));

		_target_last_position_ned(2) = -((float)target_last_position_global(2) - _vehicle_local_position.ref_alt);

		Eulerf target_last_attitude = Eulerf(math::radians(_target_uav_info.roll),
						math::radians(_target_uav_info.pitch),
						math::radians(_target_uav_info.heading));

		float target_last_velocity = _target_uav_info.velocity;

		hrt_abstime target_last_seen = _target_uav_info.time_utc_usec;

		_target_uav_predictor.assignTargetInfo(_target_last_position_ned,
							target_last_attitude,
							target_last_velocity,
							target_last_seen);
	}

#ifdef __PX4_NUTTX
	if (_sensor_gps_sub.updated()) {
		_sensor_gps_sub.update(&_sensor_gps);
	}

	Vector3f target_est_delta_position_ned = _target_uav_predictor.estimateTargetPosDelta(_sensor_gps.time_utc_usec);
#else
	auto timenow = std::chrono::system_clock::now();
	auto timenow_usec = std::chrono::duration_cast<std::chrono::microseconds>(timenow.time_since_epoch()).count();

	Vector3f target_est_delta_position_ned = _target_uav_predictor.estimateTargetPosDelta(timenow_usec);
#endif

	Vector3f target_position_est_ned = _target_last_position_ned + target_est_delta_position_ned;

	Vector3f position_setpoint = _intercept_guidance.calculateSetpoint(_vehicle_position_ned, target_position_est_ned, _lag_distance);


	_trajectory_setpoint.timestamp = hrt_absolute_time();

	_trajectory_setpoint.position[0] = position_setpoint(0);
	_trajectory_setpoint.position[1] = position_setpoint(1);
	_trajectory_setpoint.position[2] = position_setpoint(2);

	_trajectory_setpoint.velocity[0] = NAN;
	_trajectory_setpoint.velocity[1] = NAN;
	_trajectory_setpoint.velocity[2] = NAN;

	_trajectory_setpoint.acceleration[0] = NAN;
	_trajectory_setpoint.acceleration[1] = NAN;
	_trajectory_setpoint.acceleration[2] = NAN;

	_trajectory_setpoint.jerk[0] = NAN;
	_trajectory_setpoint.jerk[1] = NAN;
	_trajectory_setpoint.jerk[2] = NAN;

	_trajectory_setpoint.yaw = NAN;
	_trajectory_setpoint.yawspeed = NAN;

	_trajectory_setpoint_pub.publish(_trajectory_setpoint);
}

bool NovaIntercept::init()
{
	ScheduleOnInterval(100_ms);
	return true;
}

void NovaIntercept::Run()
{
	if (should_exit()) {
		if (_sent_mode_registration) {
			UnregisterInterceptMode(_arming_check_id, _mode_id);
		}
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	if (_parameter_update_sub.updated() || !_got_any_param) {
		if (!_got_any_param) {
			_got_any_param = true;
		}
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
		_lag_distance = _param_intrcpt_lag_dist.get();
	}

	if (!_sent_mode_registration) {
		RegisterInterceptMode();
		_sent_mode_registration = true;
		return;
	}

	if (_mode_id == -1 || _arming_check_id == -1) {
		CheckModeRegistration();
		return;
	}

	if (_arming_check_request_sub.updated()) {
		arming_check_request_s arming_check_request;
		_arming_check_request_sub.copy(&arming_check_request);
		ReplyToArmingCheck(arming_check_request.request_id);
	}

	if (_vehicle_local_position_sub.updated()) {
		_vehicle_local_position_sub.update(&_vehicle_local_position);
	}

	if (_vehicle_local_position.ref_timestamp != _ned_ref_timestamp) {
		_map_projection.initReference(_vehicle_local_position.ref_lat,
						_vehicle_local_position.ref_lon);

		_ned_ref_timestamp = _vehicle_local_position.ref_timestamp;
	}

	_vehicle_status_sub.update(&_vehicle_status);
	if (_vehicle_status.nav_state == _mode_id) {
		PopulatePositionSetpoint();
	}

	perf_end(_loop_perf);
}

int NovaIntercept::task_spawn(int argc, char *argv[])
{
	NovaIntercept *instance = new NovaIntercept();

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

int NovaIntercept::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int NovaIntercept::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int NovaIntercept::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
intercept module.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("nova_intercept", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int nova_intercept_main(int argc, char *argv[])
{
	return NovaIntercept::main(argc, argv);
}
