#include "air.h"

#include "mavlink/common/mavlink.h"
#include "Navio2/serial_port.h"
#include <cmath>

void* telemetry_loop(void* arguments) {
    usleep(300000);
    uint64_t t0 = current_time_microseconds();
    uint64_t t, now;
    thread_struct* args = (thread_struct*)arguments;
    double* array = args->array;
    const air_config* cfg = args->cfg;
    std::map<std::string, int> keys = cfg->keys;
    Generic_Port* port;
    port = new Serial_Port("/dev/ttyAMA0", 57600);
    port->start();
    printf("Opening MAVLink port.\t[/dev/ttyAMA0]\n");
    int max_sleep = hertz_to_microseconds(cfg->TELEMETRY_LOOP_RATE);

    while (true) {
        t = current_time_microseconds() - t0;
        uint32_t time_boot_ms = (uint32_t)(t / 1000); // ms since boot

        // IDs
        const uint8_t sysid = 1;                       // vehicle/system id
        const uint8_t compid = MAV_COMP_ID_AUTOPILOT1; // component id

        // Heartbeat
        mavlink_message_t msg_heartbeat;
        uint8_t base_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED;
        uint32_t custom_mode = 0;
        uint8_t system_status = MAV_STATE_ACTIVE;
        mavlink_msg_heartbeat_pack(sysid, compid, &msg_heartbeat,
                                   MAV_TYPE_FIXED_WING,
                                   MAV_AUTOPILOT_GENERIC,
                                   base_mode,
                                   custom_mode,
                                   system_status);
        port->write_message(msg_heartbeat);

        // Attitude from estimator (xh_0)
        float roll = (float)array[keys["xh_0_PHI"]];
        float pitch = (float)array[keys["xh_0_THETA"]];
        float yaw = (float)array[keys["xh_0_PSI"]];
        float rollspeed = (float)array[keys["xh_0_P"]];
        float pitchspeed = (float)array[keys["xh_0_Q"]];
        float yawspeed = (float)array[keys["xh_0_R"]];
        mavlink_message_t msg_att;
        mavlink_msg_attitude_pack(sysid, compid, &msg_att, time_boot_ms, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed);
        port->write_message(msg_att);

        // Global position from GPS and estimator
        int32_t lat = 0;
        int32_t lon = 0;
        int32_t alt_msl_mm = 0;
        int32_t rel_alt_mm = 0;
        int16_t vx_cms = 0;
        int16_t vy_cms = 0;
        int16_t vz_cms = 0;
        uint16_t hdg_cdeg = 0;

        // If GPS values look valid, populate
        double gps_lat = array[keys["y_GPS_POSN_LAT"]];   // deg
        double gps_lon = array[keys["y_GPS_POSN_LON"]];   // deg
        double gps_alt = array[keys["y_GPS_POSN_ALT"]];   // m AMSL
        double gps_vn = array[keys["y_GPS_VEL_N"]];       // m/s
        double gps_ve = array[keys["y_GPS_VEL_E"]];       // m/s
        double gps_vd = array[keys["y_GPS_VEL_D"]];       // m/s (down +)
        int gps_status = (int)array[keys["y_GPS_STATUS"]];

        if (gps_status >= 2 && std::abs(gps_lat) > 0.001 && std::abs(gps_lon) > 0.001) {
            lat = (int32_t)llround(gps_lat * 1e7);
            lon = (int32_t)llround(gps_lon * 1e7);
            alt_msl_mm = (int32_t)llround(gps_alt * 1000.0);
            // Relative altitude from estimator z (NED down positive)
            rel_alt_mm = (int32_t)llround((-array[keys["xh_0_Z"]]) * 1000.0);
            vx_cms = (int16_t)llround(gps_vn * 100.0);
            vy_cms = (int16_t)llround(gps_ve * 100.0);
            vz_cms = (int16_t)llround(gps_vd * 100.0);
            // Heading from yaw (0..360 deg)
            double yaw_deg = array[keys["xh_0_PSI"]] * 180.0 / M_PI;
            if (yaw_deg < 0) yaw_deg += 360.0;
            if (yaw_deg >= 360.0) yaw_deg -= 360.0;
            hdg_cdeg = (uint16_t)llround(yaw_deg * 100.0);
        }

        mavlink_message_t msg_global;
        mavlink_msg_global_position_int_pack(sysid, compid, &msg_global, time_boot_ms,
                                             lat, lon, alt_msl_mm, rel_alt_mm,
                                             vx_cms, vy_cms, vz_cms, hdg_cdeg);
        port->write_message(msg_global);

        // Optional: VFR_HUD (airspeed/groundspeed/heading/alt)
        // Use VT from estimator as airspeed approximation and GPS VN/VE for groundspeed
        float airspeed = (float)array[keys["xh_0_VT"]];
        float groundspeed = (float)std::sqrt(gps_vn * gps_vn + gps_ve * gps_ve);
        int16_t heading = (int16_t)(hdg_cdeg / 100);
        int16_t throttle = (int16_t)((array[keys["rcin_CHANNEL_" + std::to_string(cfg->THROTTLE_CHANNEL)]] - 1000.0) / 10.0); // 0..100%
        float alt_rel_m = (float)(rel_alt_mm / 1000.0);
        float climb = (float)(-gps_vd); // m/s up
        mavlink_message_t msg_hud;
        mavlink_msg_vfr_hud_pack(sysid, compid, &msg_hud, airspeed, groundspeed, heading, throttle, alt_rel_m, climb);
        port->write_message(msg_hud);

        now = current_time_microseconds() - t0;
        int sleep_time = (int)(max_sleep - (now - t));
        usleep(std::max(sleep_time, 0));
    }
}