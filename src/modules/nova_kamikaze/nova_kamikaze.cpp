// ---------------   بِسْــــمِ اللّٰهِ الرَّحْمَـنِ الرَّحِيـمِ   ---------------

#include "nova_kamikaze.hpp"

NovaKamikaze::NovaKamikaze() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

NovaKamikaze::~NovaKamikaze()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}


void NovaKamikaze::RegisterKamikazeMode()
{
	register_ext_component_request_s register_ext_component_request{};
	register_ext_component_request.timestamp = hrt_absolute_time();
	strncpy(register_ext_component_request.name, "Kamikaze", sizeof(register_ext_component_request.name) - 1);
	register_ext_component_request.request_id = _mode_request_id;
	register_ext_component_request.px4_ros2_api_version = 1;
	register_ext_component_request.register_arming_check = true;
	register_ext_component_request.register_mode = true;
	_register_ext_component_request_pub.publish(register_ext_component_request);
}


void NovaKamikaze::UnregisterKamikazeMode(int8_t arming_check_id, int8_t mode_id)
{
	unregister_ext_component_s unregister_ext_component{};
	unregister_ext_component.timestamp = hrt_absolute_time();
	strncpy(unregister_ext_component.name, "Kamikaze", sizeof(unregister_ext_component.name) - 1);
	unregister_ext_component.arming_check_id = arming_check_id;
	unregister_ext_component.mode_id = mode_id;
	_unregister_ext_component_pub.publish(unregister_ext_component);
}


void NovaKamikaze::ConfigureKamikazeMode(int8_t mode_id)
{
	vehicle_control_mode_s config_control_setpoints{};
	config_control_setpoints.source_id = mode_id;
	config_control_setpoints.flag_control_attitude_enabled = true;
	config_control_setpoints.flag_control_rates_enabled = true;
	config_control_setpoints.flag_control_allocation_enabled = true;
	_config_control_setpoints_pub.publish(config_control_setpoints);
}


void NovaKamikaze::ReplyToArmingCheck(int8_t request_id)
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
	arming_check_reply.mode_req_global_position = false;
	arming_check_reply.mode_req_prevent_arming = false;
	arming_check_reply.mode_req_manual_control = false;
	_arming_check_reply_pub.publish(arming_check_reply);
}


void NovaKamikaze::CheckModeRegistration()
{
	register_ext_component_reply_s register_ext_component_reply;
	int tries = register_ext_component_reply.ORB_QUEUE_LENGTH;

	while (_register_ext_component_reply_sub.update(&register_ext_component_reply) && --tries >= 0) {
		if (register_ext_component_reply.request_id == _mode_request_id && register_ext_component_reply.success) {
			_arming_check_id = register_ext_component_reply.arming_check_id;
			_mode_id = register_ext_component_reply.mode_id;
			PX4_INFO("Kamikaze mode registration successful, arming_check_id: %d, mode_id: %d", _arming_check_id, _mode_id);
			ConfigureKamikazeMode(_mode_id);
			break;
		}
	}

	_kamikaze_info.timestamp = hrt_absolute_time();
	_kamikaze_info.mode_id = _mode_id;
	_kamikaze_pub.publish(_kamikaze_info);
}

void NovaKamikaze::PopulateAttitudeSetpoint(int8_t kamik_state)
{
	_local_position_sub.update(&_position);
	_attitude_sub.update(&_attitude);

	matrix::Quatf q_current(_attitude.q);
	matrix::Eulerf euler_current(q_current);

	float kp_pitch = 0.65f;
	float current_pitch = euler_current(1);
	float current_pitch_err_deg = 0.0f;
	float thrust = 0.0f;

	if (kamik_state == _dive) {
		current_pitch_err_deg = math::degrees(current_pitch) + _param_dive_ang.get();
		thrust = _param_dive_thr.get();
	} else if (kamik_state == _level) {
		current_pitch_err_deg = math::degrees(current_pitch);
		thrust = _param_level_thr.get();
	} else if (kamik_state == _rise) {
		current_pitch_err_deg = math::degrees(current_pitch) - _param_rise_ang.get();
		thrust = _param_rise_thr.get();
	}

	const float yaw_rad_desired = _position.heading;
	float roll_rad_desired = 0.0f;
	float pitch_rad_desired = current_pitch - math::radians(current_pitch_err_deg * kp_pitch);


	matrix::Eulerf euler_desired(roll_rad_desired, pitch_rad_desired, yaw_rad_desired);
	matrix::Quatf q_desired(euler_desired);

	_attitude_setpoint.timestamp = hrt_absolute_time();
	_attitude_setpoint.q_d[0] = q_desired(0);
	_attitude_setpoint.q_d[1] = q_desired(1);
	_attitude_setpoint.q_d[2] = q_desired(2);
	_attitude_setpoint.q_d[3] = q_desired(3);
	_attitude_setpoint.thrust_body[0] = thrust;
	_attitude_setpoint_pub.publish(_attitude_setpoint);
}

bool NovaKamikaze::CheckForStage(int8_t kamik_state)
{
	_attitude_sub.update(&_attitude);

	matrix::Quatf q_current(_attitude.q);
	matrix::Eulerf euler_current(q_current);

	float current_pitch = euler_current(1);

	if (kamik_state == _dive) {
		if (-_position.z <= (_param_dive_alt.get() * 1.1f)) {
			return true;
		}
	} else if (kamik_state == _level) {
		if (fabsf(math::degrees(current_pitch)) < 7) {
			return true;
		}
	} else if (kamik_state == _rise) {
		if (-_position.z >= (_param_rise_alt.get() * 0.9f)) {
			return true;
		}
	}

	return false;
}

void NovaKamikaze::SetModeHold()
{
        vehicle_command_s cmd{};
        cmd.timestamp = hrt_absolute_time();
        cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
        cmd.param1 = 1;
	cmd.param2 = 4; // MAIN AUTO ID
	cmd.param3 = 3; // SUB HOLD ID

        cmd.target_system = _vehicle_status.system_id;
        cmd.target_component = _vehicle_status.component_id;

        cmd.source_system = _vehicle_status.system_id;
        cmd.source_component = _vehicle_status.component_id;

        cmd.confirmation = false;
        cmd.from_external = false;

        _vehicle_command_pub.publish(cmd);
}

bool NovaKamikaze::init()
{
	if (!_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}
	return true;
}

void NovaKamikaze::Run()
{
	if (should_exit()) {
		_angular_velocity_sub.unregisterCallback();
		if (_sent_mode_registration) {
			UnregisterKamikazeMode(_arming_check_id, _mode_id);
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
		if (_param_dive_ang.get() != _dive_ang) {
			_dive_ang = _param_dive_ang.get();
			int dive_alt = _param_dive_alt.get();

			_kamikaze_info.timestamp = hrt_absolute_time();
			_kamikaze_info.dive_ang = _dive_ang;
			_kamikaze_info.dive_alt = dive_alt;
			_kamikaze_pub.publish(_kamikaze_info);
		}
	}

	if (!_sent_mode_registration) {
		RegisterKamikazeMode();
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

	_vehicle_status_sub.update(&_vehicle_status);
	if (_vehicle_status.nav_state == _mode_id) {
		if (!_dive_ok) {
			PopulateAttitudeSetpoint(_dive);
			if(CheckForStage(_dive)) {
				_dive_ok = true;
			}
			return;
		}
		if (!_level_ok) {
			PopulateAttitudeSetpoint(_level);
			if(CheckForStage(_level)) {
				_level_ok = true;
			}
			return;
		}
		if (!_rise_ok) {
			PopulateAttitudeSetpoint(_rise);
			if(CheckForStage(_rise)) {
				_rise_ok = true;
			}
			return;
		}

		_dive_ok = false;
		_level_ok = false;
		_rise_ok = false;
		SetModeHold();

	}

	perf_end(_loop_perf);
}

int NovaKamikaze::task_spawn(int argc, char *argv[])
{
	NovaKamikaze *instance = new NovaKamikaze();

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

int NovaKamikaze::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int NovaKamikaze::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int NovaKamikaze::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
kamikaze module.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("nova_kamikaze", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int nova_kamikaze_main(int argc, char *argv[])
{
	return NovaKamikaze::main(argc, argv);
}
