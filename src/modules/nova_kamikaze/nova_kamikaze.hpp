#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <matrix/matrix/math.hpp>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>

#include <uORB/topics/register_ext_component_reply.h>
#include <uORB/topics/arming_check_request.h>
#include <uORB/topics/register_ext_component_request.h>
#include <uORB/topics/unregister_ext_component.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/arming_check_reply.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/nova_kamikaze.h>


using namespace time_literals;

class NovaKamikaze : public ModuleBase<NovaKamikaze>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	NovaKamikaze();
	~NovaKamikaze() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	// Functions
	void Run() override;
	void RegisterKamikazeMode();
	void UnregisterKamikazeMode(int8_t arming_check_id, int8_t mode_id);
	void ConfigureKamikazeMode(int8_t mode_id);
	void ReplyToArmingCheck(int8_t request_id);
	void CheckModeRegistration();
	void PopulateAttitudeSetpoint(int8_t kamik_state);
	bool CheckForStage(int8_t kamik_state);
	void SetModeHold();

	// Publications
	uORB::Publication<vehicle_attitude_setpoint_s> 		_attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Publication<register_ext_component_request_s> 	_register_ext_component_request_pub{ORB_ID(register_ext_component_request)};
	uORB::Publication<unregister_ext_component_s> 		_unregister_ext_component_pub{ORB_ID(unregister_ext_component)};
	uORB::Publication<vehicle_control_mode_s> 		_config_control_setpoints_pub{ORB_ID(config_control_setpoints)};
	uORB::Publication<arming_check_reply_s> 		_arming_check_reply_pub{ORB_ID(arming_check_reply)};
	uORB::Publication<vehicle_command_s> 		        _vehicle_command_pub{ORB_ID(vehicle_command)};
	uORB::Publication<nova_kamikaze_s>                      _kamikaze_pub{ORB_ID(nova_kamikaze)};

	// Subscriptions
	uORB::SubscriptionCallbackWorkItem 	_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};
	uORB::SubscriptionInterval         	_parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription		 	_vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription 			_register_ext_component_reply_sub{ORB_ID(register_ext_component_reply)};
	uORB::Subscription 			_arming_check_request_sub{ORB_ID(arming_check_request)};
	uORB::Subscription 			_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription 			_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription 			_kamikaze_sub{ORB_ID(nova_kamikaze)};

	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	bool  _got_any_param{false};

	bool _sent_mode_registration{false};
	uint8_t _mode_request_id{200}; //Random value
	int8_t _arming_check_id{-1};
	int8_t _mode_id{-1};

	int _dive_ang{0};

	int8_t _dive{0};
	int8_t _level{1};
	int8_t _rise{2};

	bool _dive_ok{false};
	bool _level_ok{false};
	bool _rise_ok{false};

	vehicle_status_s 		_vehicle_status{};
	vehicle_attitude_setpoint_s	_attitude_setpoint{};
	vehicle_local_position_s	_position{};
	vehicle_attitude_s		_attitude{};
	nova_kamikaze_s                 _kamikaze_info{};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::KMKZ_DIVE_ANG>) _param_dive_ang,
		(ParamFloat<px4::params::KMKZ_DIVE_THR>) _param_dive_thr,
		(ParamInt<px4::params::KMKZ_DIVE_ALT>) _param_dive_alt,
		(ParamFloat<px4::params::KMKZ_LEVEL_THR>) _param_level_thr,
		(ParamInt<px4::params::KMKZ_RISE_ANG>) _param_rise_ang,
		(ParamFloat<px4::params::KMKZ_RISE_THR>) _param_rise_thr,
		(ParamInt<px4::params::KMKZ_RISE_ALT>) _param_rise_alt
	)

};
