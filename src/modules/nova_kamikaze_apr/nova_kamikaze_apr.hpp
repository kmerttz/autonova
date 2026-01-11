#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <lib/geo/geo.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/nova_kamikaze.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

class NovaKamikazeApr : public ModuleBase<NovaKamikazeApr>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	NovaKamikazeApr();
	~NovaKamikazeApr() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;

	void orbit(float rad, float lat, float lon, float alt);
	void calculate_orbit_center();
	void check_orbit_progress();
	void calculate_qr_target_coordinates(int distance_to_target);
	void check_route_completion();
	void set_mode_kamikaze();

	// Publications
	uORB::Publication<vehicle_command_s> _vehicle_cmd_pub{ORB_ID(vehicle_command)};

	// Subscriptions
	uORB::SubscriptionInterval           _parameter_update_sub{ORB_ID(parameter_update), 1_s}; // subscription limited to 1 Hz updates
	uORB::Subscription	             _global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription	             _local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription	             _kamikaze_sub{ORB_ID(nova_kamikaze)};
	uORB::Subscription	             _vehicle_status_sub{ORB_ID(vehicle_status)};


	enum class ApproachState {
		IDLE,
		APPROACHING,
		ORBITING,
		ENROUTE,
		COMPLETED
	};

	struct qr_location_s {
		double lat;
		double lon;
	};

	struct qr_target_location_s {
		double lat;
		double lon;
	};

	struct apr_location_s {
		double lat;
		double lon;
	};

	struct orb_location_s {
		double lat;
		double lon;
	};

	bool  _got_any_param{false};
	int   _apr_alt{};
	int   _orb_rad{};
	int   _orb_side{};
	float _dive_ofs{};
	float _heading_apr2qr{};

	bool  _orbit_started{false};
	bool  _half_lap_completed{false};
	bool  _first_lap_completed{false};
	bool  _route_completed{false};
	float _orbit_entry_heading{};


	vehicle_global_position_s   _global_pos{};
	vehicle_local_position_s    _local_pos{};
	vehicle_status_s 	    _vehicle_status{};
	nova_kamikaze_s    	    _kamikaze_info{};

	qr_location_s		_qr_loc{};
	qr_target_location_s 	_qr_target_loc{};
	apr_location_s		_apr_loc{};
	orb_location_s		_orb_loc{};

	ApproachState _approach_status{ApproachState::IDLE};

	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::KMKZ_QR_LAT>) _param_kmkz_qr_lat,
		(ParamFloat<px4::params::KMKZ_QR_LON>) _param_kmkz_qr_lon,
		(ParamInt<px4::params::KMKZ_APR_ALT>) _param_kmkz_apr_alt,
		(ParamFloat<px4::params::KMKZ_APR_LAT>) _param_kmkz_apr_lat,
		(ParamFloat<px4::params::KMKZ_APR_LON>) _param_kmkz_apr_lon,
		(ParamInt<px4::params::KMKZ_ORB_RAD>) _param_kmkz_orb_rad,
		(ParamInt<px4::params::KMKZ_ORB_SIDE>) _param_kmkz_orb_side,
		(ParamFloat<px4::params::KMKZ_DIVE_OFS>) _param_kmkz_dive_ofs,
		(ParamInt<px4::params::KMKZ_APR_START>) _param_kmkz_apr_start
	)

};
