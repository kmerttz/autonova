#include "nova_test_cmd.hpp"

NovaTestCmd::NovaTestCmd() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

NovaTestCmd::~NovaTestCmd()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool NovaTestCmd::init()
{
	ScheduleOnInterval(100_ms); // 10 Hz rate
	return true;
}

void NovaTestCmd::orbit()
{
	vehicle_command_s cmd{};

	cmd.timestamp = hrt_absolute_time();

	cmd.command = vehicle_command_s::VEHICLE_CMD_DO_ORBIT;

	cmd.param1 = 50.0f; // radius in meters
	cmd.param2 = NAN;   // Use vehicle default velocity, or current velocity if already orbiting.
	cmd.param3 = 3.0f; // Vehicle front follows flight path (tangential to circle).
	cmd.param4 = 0.0f; // Orbit forever
	cmd.param5 = 47.397637f; // latitude
	cmd.param6 = 8.554521f;  // longitude
	cmd.param7 = 50.0f;     // altitude

	cmd.target_system = 1;
	cmd.target_component = 1;

	cmd.source_system = 1;
	cmd.source_component = 1;

	cmd.confirmation = false;
	cmd.from_external = false;

	_vehicle_cmd_pub.publish(cmd);
}


void NovaTestCmd::gotoloc()
{
	vehicle_command_s cmd{};

	cmd.timestamp = hrt_absolute_time();

	cmd.command = vehicle_command_s::VEHICLE_CMD_DO_ORBIT;

	cmd.param1 = 0.1f; // radius in meters
	cmd.param2 = NAN;   // Use vehicle default velocity, or current velocity if already orbiting.
	cmd.param3 = 3.0f; // Vehicle front follows flight path (tangential to circle).
	cmd.param4 = 0.0f; // Orbit forever
	cmd.param5 = 47.397637f; // latitude
	cmd.param6 = 8.554521f;  // longitude
	cmd.param7 = 50.0f;     // altitude

	cmd.target_system = 1;
	cmd.target_component = 1;

	cmd.source_system = 1;
	cmd.source_component = 1;

	cmd.confirmation = false;
	cmd.from_external = false;

	_vehicle_cmd_pub.publish(cmd);
}
void NovaTestCmd::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	if (_parameter_update_sub.updated()) {

		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	if (_param_test_cmd_select.get() == 1) {
		orbit();
		_param_test_cmd_select.set(0);

	} else if (_param_test_cmd_select.get() == 2) {
		gotoloc();
		_param_test_cmd_select.set(0);

	}

	perf_end(_loop_perf);
}

int NovaTestCmd::task_spawn(int argc, char *argv[])
{
	NovaTestCmd *instance = new NovaTestCmd();

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

int NovaTestCmd::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int NovaTestCmd::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int NovaTestCmd::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Nova test module to send vehicle commands for testing purposes.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("nova_test_cmd", "test");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int nova_test_cmd_main(int argc, char *argv[])
{
	return NovaTestCmd::main(argc, argv);
}
