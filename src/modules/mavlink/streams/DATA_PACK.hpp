#ifndef DATA_PACK_HPP
#define DATA_PACK_HPP

#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_info.h>
#include <lib/matrix/matrix/math.hpp>

class MavlinkStreamDataPack : public MavlinkStream
{
public:
    static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamDataPack(mavlink); }

    const char *get_name() const override { return MavlinkStreamDataPack::get_name_static(); }
    static constexpr const char *get_name_static() { return "DATA_PACK"; }

    uint16_t get_id() override { return get_id_static(); }
    static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_DATA_PACK; }

    unsigned get_size() override { return MAVLINK_MSG_ID_DATA_PACK_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES; }

private:
    explicit MavlinkStreamDataPack(Mavlink *mavlink) : MavlinkStream(mavlink) {}

    uORB::Subscription _vehicle_info_sub{ORB_ID(vehicle_info)};

    bool send() override
    {
        vehicle_info_s vehicle_info{};

        if (_vehicle_info_sub.update(&vehicle_info)) {

            mavlink_data_pack_t msg{};

            msg.takim_numarasi = vehicle_info.team_id;

            msg.iha_enlem = vehicle_info.latitude;
            msg.iha_boylam = vehicle_info.longitude;
            msg.iha_irtifa = vehicle_info.altitude;

            msg.iha_yatis   = vehicle_info.roll;
            msg.iha_dikilme = vehicle_info.pitch;
            msg.iha_yonelme = vehicle_info.heading;

            msg.iha_hiz = vehicle_info.velocity;

	    msg.zaman = vehicle_info.time_utc_usec;

            mavlink_msg_data_pack_send_struct(_mavlink->get_channel(), &msg);

            return true;
        }
        return false;
    }
};

#endif
