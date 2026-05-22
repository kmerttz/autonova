#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <lib/intercept_guidance/intercept_guidance.hpp>
#include <lib/target_uav_predictor/target_uav_predictor.hpp>
#include <lib/geo/geo.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <matrix/matrix/math.hpp>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>
//#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/target_uav_info.h>
#ifdef __PX4_NUTTX
#include <uORB/topics/sensor_gps.h>
#else
#include <chrono>
#endif

#include <uORB/topics/register_ext_component_reply.h>
#include <uORB/topics/arming_check_request.h>
#include <uORB/topics/register_ext_component_request.h>
#include <uORB/topics/unregister_ext_component.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/arming_check_reply.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_status.h>



using namespace time_literals;

class NovaIntercept : public ModuleBase<NovaIntercept>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	NovaIntercept();
	~NovaIntercept() override;

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
	void RegisterInterceptMode();
	void UnregisterInterceptMode(int8_t arming_check_id, int8_t mode_id);
	void ConfigureInterceptMode(int8_t mode_id);
	void ReplyToArmingCheck(int8_t request_id);
	void CheckModeRegistration();

	void PopulatePositionSetpoint();

	// Publications
	//uORB::Publication<vehicle_local_position_setpoint_s> 	_vehicle_local_position_setpoint_pub{ORB_ID(vehicle_local_position_setpoint)};
	uORB::Publication<trajectory_setpoint_s> 	        _trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};

	uORB::Publication<register_ext_component_request_s> 	_register_ext_component_request_pub{ORB_ID(register_ext_component_request)};
	uORB::Publication<unregister_ext_component_s> 		_unregister_ext_component_pub{ORB_ID(unregister_ext_component)};
	uORB::Publication<vehicle_control_mode_s> 		_config_control_setpoints_pub{ORB_ID(config_control_setpoints)};
	uORB::Publication<arming_check_reply_s> 		_arming_check_reply_pub{ORB_ID(arming_check_reply)};
	uORB::Publication<vehicle_command_s> 		        _vehicle_command_pub{ORB_ID(vehicle_command)};

	// Subscriptions
	uORB::SubscriptionInterval         	_parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription		 	_vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription 			_register_ext_component_reply_sub{ORB_ID(register_ext_component_reply)};
	uORB::Subscription 			_arming_check_request_sub{ORB_ID(arming_check_request)};

	uORB::Subscription 			_vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription 			_target_uav_info_sub{ORB_ID(target_uav_info)};
#ifdef __PX4_NUTTX
	uORB::Subscription 			_sensor_gps_sub{ORB_ID(sensor_gps)};
#endif

	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	bool  _got_any_param{false};

	bool _sent_mode_registration{false};
	uint8_t _mode_request_id{201}; //Random value
	int8_t _arming_check_id{-1};
	int8_t _mode_id{-1};

	Vector3f _target_last_position_ned;
	Vector3f _vehicle_position_ned;

	float _lag_distance{0.f};
	hrt_abstime		_ned_ref_timestamp{0};

	vehicle_status_s 			_vehicle_status{};
	vehicle_local_position_s		_vehicle_local_position{};
	//vehicle_local_position_setpoint_s	_vehicle_local_position_setpoint{};
	trajectory_setpoint_s			_trajectory_setpoint{};
	target_uav_info_s			_target_uav_info{};
#ifdef __PX4_NUTTX
	sensor_gps_s				_sensor_gps{};
#endif

	// Parameters
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::INTRCPT_LAG_DIST>) _param_intrcpt_lag_dist
	)

	TargetUavPredictor	_target_uav_predictor;
	InterceptGuidance 	_intercept_guidance;
	MapProjection 		_map_projection;

};
