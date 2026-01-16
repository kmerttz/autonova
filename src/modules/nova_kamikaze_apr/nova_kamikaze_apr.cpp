// ---------------   بِسْــــمِ اللّٰهِ الرَّحْمَـنِ الرَّحِيـمِ   ---------------

#include "nova_kamikaze_apr.hpp"

NovaKamikazeApr::NovaKamikazeApr() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

NovaKamikazeApr::~NovaKamikazeApr()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool NovaKamikazeApr::init()
{
	ScheduleOnInterval(100_ms); // 10 Hz rate
	return true;
}

void NovaKamikazeApr::orbit(float rad, float lat, float lon, float alt)
{
	vehicle_command_s cmd{};

	cmd.timestamp = hrt_absolute_time();

	cmd.command = vehicle_command_s::VEHICLE_CMD_DO_ORBIT;

	cmd.param1 = rad;   // radius in meters
	cmd.param2 = NAN;   // Use vehicle default velocity, or current velocity if already orbiting.
	cmd.param3 = 3.0f;  // Vehicle front follows flight path (tangential to circle).
	cmd.param4 = 0.0f;  // Orbit forever
	cmd.param5 = lat;   // latitude
	cmd.param6 = lon;   // longitude
	cmd.param7 = alt;   // altitude

	cmd.target_system = 1;
	cmd.target_component = 1;

	cmd.source_system = 1;
	cmd.source_component = 1;

	cmd.confirmation = false;
	cmd.from_external = false;

	_vehicle_cmd_pub.publish(cmd);
}

void NovaKamikazeApr::calculate_orbit_center()
{
    _heading_apr2qr = get_bearing_to_next_waypoint(_apr_loc.lat, _apr_loc.lon, _qr_loc.lat, _qr_loc.lon);

    float heading_apr2center;

    if (_orb_side) {
        heading_apr2center = _heading_apr2qr + M_PI_2_F; // RIGHT
    } else {

        heading_apr2center = _heading_apr2qr - M_PI_2_F; // LEFT
    }

    heading_apr2center = matrix::wrap_pi(heading_apr2center);

    waypoint_from_heading_and_distance(_apr_loc.lat, _apr_loc.lon,
					heading_apr2center, _orb_rad,
					&_orb_loc.lat, &_orb_loc.lon);
}

void NovaKamikazeApr::check_orbit_progress()
{
    _global_position_sub.update(&_global_pos);
    _local_position_sub.update(&_local_pos);

    float distance_now2orb = get_distance_to_next_waypoint(_orb_loc.lat, _orb_loc.lon, _global_pos.lat, _global_pos.lon);

    if (distance_now2orb <= _orb_rad * 1.1f && !_orbit_started) {
        _orbit_started = true;
        _orbit_entry_heading = _local_pos.heading;
    }

    if (_orbit_started) {
        float heading_diff = matrix::wrap_pi(_local_pos.heading - _orbit_entry_heading);

        if (!_half_lap_completed) {
            if (fabsf(heading_diff) >= 2.8f) {
                _half_lap_completed = true;
            }
        } else {
            if (fabsf(heading_diff) <= 0.1f) {
                _first_lap_completed = true;
            }
        }
    }
}

void NovaKamikazeApr::calculate_qr_target_coordinates(int distance_to_target)
{

	waypoint_from_heading_and_distance(
		_qr_loc.lat,
		_qr_loc.lon,
		_heading_apr2qr,
		distance_to_target,
		&_qr_target_loc.lat,
		&_qr_target_loc.lon
	);

}

void NovaKamikazeApr::check_route_completion()
{
	_global_position_sub.update(&_global_pos);

	if (_kamikaze_sub.updated()) {
		_kamikaze_sub.copy(&_kamikaze_info);
	}

	float distance_now2qr = get_distance_to_next_waypoint(_qr_loc.lat, _qr_loc.lon, _global_pos.lat, _global_pos.lon);

	float completion_threshold =  ((tanf(math::radians((float)(90 - _kamikaze_info.dive_ang)))) * (_apr_alt - _kamikaze_info.dive_alt)) + (_kamikaze_info.dive_ang * _dive_ofs);

	if (distance_now2qr <= completion_threshold) {
		_route_completed = true;
	}
}

void NovaKamikazeApr::set_mode_kamikaze()
{
	if (_kamikaze_sub.updated()) {
		_kamikaze_sub.copy(&_kamikaze_info);
	}

        _vehicle_command.timestamp = hrt_absolute_time();
        _vehicle_command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
        _vehicle_command.param1 = 1;
	_vehicle_command.param2 = 4; // MAIN AUTO ID
	_vehicle_command.param3 = _kamikaze_info.mode_id - 12; // SUB KAMIKAZE ID

        _vehicle_command.target_system = _vehicle_status.system_id;
        _vehicle_command.target_component = _vehicle_status.component_id;

        _vehicle_command.source_system = _vehicle_status.system_id;
        _vehicle_command.source_component = _vehicle_status.component_id;

        _vehicle_command.confirmation = false;
        _vehicle_command.from_external = false;

        _vehicle_cmd_pub.publish(_vehicle_command);
}

void NovaKamikazeApr::set_mode_hold()
{
        _vehicle_command.timestamp = hrt_absolute_time();
        _vehicle_command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
        _vehicle_command.param1 = 1;
	_vehicle_command.param2 = 4; // MAIN AUTO ID
	_vehicle_command.param3 = 3; // SUB HOLD ID

        _vehicle_command.target_system = _vehicle_status.system_id;
        _vehicle_command.target_component = _vehicle_status.component_id;

        _vehicle_command.source_system = _vehicle_status.system_id;
        _vehicle_command.source_component = _vehicle_status.component_id;

        _vehicle_command.confirmation = false;
        _vehicle_command.from_external = false;

        _vehicle_cmd_pub.publish(_vehicle_command);
}

void NovaKamikazeApr::Run()
{
	if (should_exit()) {
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

		_qr_loc.lat  = _param_kmkz_qr_lat.get();
		_qr_loc.lon  = _param_kmkz_qr_lon.get();
		_apr_loc.lat = _param_kmkz_apr_lat.get();
		_apr_loc.lon = _param_kmkz_apr_lon.get();
		_apr_alt     = _param_kmkz_apr_alt.get();
		_orb_rad     = _param_kmkz_orb_rad.get();
		_orb_side    = _param_kmkz_orb_side.get();
		_dive_ofs    = _param_kmkz_dive_ofs.get();
	}

	_vehicle_command_sub.update(&_vehicle_command);
	if (_vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_KMKZ) {
		int condition = roundf(_vehicle_command.param1);
		if (condition == 1) {
			_kamikaze_active = true;
			_approach_status = ApproachState::IDLE;
		} else {
			_approach_status = ApproachState::ABORT;
		}
	}

	if (_kamikaze_active) {
		switch (_approach_status) {

			case ApproachState::IDLE:
				calculate_orbit_center();
				orbit(_orb_rad, _orb_loc.lat, _orb_loc.lon, _apr_alt);
				_approach_status = ApproachState::APPROACHING;
				break;

			case ApproachState::APPROACHING:
				check_orbit_progress();
				if (_orbit_started) {
					_approach_status = ApproachState::ORBITING;
				}
				break;

			case ApproachState::ORBITING:
				check_orbit_progress();
				if (_first_lap_completed) {
					_local_position_sub.update(&_local_pos);
					if (fabsf(_heading_apr2qr - _local_pos.heading) < 0.05f) {
						calculate_qr_target_coordinates(100);
						orbit(0.2f, _qr_target_loc.lat, _qr_target_loc.lon, _apr_alt);
						_approach_status = ApproachState::ENROUTE;
					}
				}
				break;

			case ApproachState::ENROUTE:
				check_route_completion();
				if (_route_completed) {
					set_mode_kamikaze();
					_param_kmkz_apr_start.set(0);
					_approach_status = ApproachState::COMPLETED;
				}
				break;

			case ApproachState::COMPLETED:
				_orbit_started = false;
				_half_lap_completed = false;
				_first_lap_completed = false;
				_route_completed = false;
				_kamikaze_active = false;
				break;

			case ApproachState::ABORT:
				set_mode_hold();
				_orbit_started = false;
				_half_lap_completed = false;
				_first_lap_completed = false;
				_route_completed = false;
				_kamikaze_active = false;
				break;
			}
	}
	perf_end(_loop_perf);
}

int NovaKamikazeApr::task_spawn(int argc, char *argv[])
{
	NovaKamikazeApr *instance = new NovaKamikazeApr();

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

int NovaKamikazeApr::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int NovaKamikazeApr::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int NovaKamikazeApr::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Nova test module to send vehicle commands for testing purposes.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("nova_kamikaze_apr", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int nova_kamikaze_apr_main(int argc, char *argv[])
{
	return NovaKamikazeApr::main(argc, argv);
}
