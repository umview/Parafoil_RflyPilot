#ifndef _GPS_API_
#define _GPS_API_
#include "include.h"
#include "ubx.h"
#include <iostream>
#include <ostream>

/* General: Checksum */
typedef struct {
	uint8_t ck_a;
	uint8_t ck_b;
} ubx_checksum_t ;
#define GPS_BUFFER_LENGTH 148
typedef union
{
    uint8_t buffer[GPS_BUFFER_LENGTH * 2];
    struct {
        uint8_t frame_old[GPS_BUFFER_LENGTH];
        uint8_t frame_new[GPS_BUFFER_LENGTH];
    }frame;
}gps_pvt_msg_typedef;

//#define GPS_DEBUG 1

typedef struct 
{
    bool updated;
    bool ned_origin_valid;
    bool gps_is_good;
    uint64_t timestamp;
    double lon;
    double lat;
    double lon_origin;
    double lat_origin;
    float yaw_offset;
    float height;
    float vel_ned[3];
    float pos_ned[3];
    float hacc;
    float vacc;
    float sacc;
    float heading;
    float headacc;
    uint8_t numSV;
    uint8_t fixType;
}sensor_gps_typedef;

const double CONSTANTS_RADIUS_OF_EARTH = 6371393;
class gps_api_typedef
{
public:
    int gps_debug;
    gps_pvt_msg_typedef pvt_msg;
    sensor_gps_typedef sensor_gps;
	gps_api_typedef(void);
    void init(char *_port, speed_t speed);
    bool gps_config(char *_port);
	int _serial_fd;
	void run(void);
	bool sendMessage(const uint16_t msg, const uint8_t *payload, const uint16_t length);
	int gps_write(uint8_t *buf, int buf_length);
	void calcChecksum(const uint8_t *buffer, const uint16_t length, ubx_checksum_t *checksum);
    bool pollOrRead(uint8_t * buff, int buf_length);
    bool gps_msg_decode(uint8_t *frame, int MSG_LENGHT);
    bool packet_check(uint8_t *frame, int index, int length);
    void packet_decode(uint8_t *packet);
    void NAV_CLASS_decode(uint8_t *packet);
    void ACK_CLASS_decode(uint8_t *packet);
    bool gps_pvt_check(ubx_payload_rx_nav_pvt_t *pvt);
    bool get_ned_origin(bool _ned_origin_valid, bool gps_is_good, ubx_payload_rx_nav_pvt_t *pvt,
                                    double *_lat_origin, double *_lon_origin);
    ubx_buf_t   _buf{};
    template<typename T>
    bool cfgValset(uint32_t key_id, T value, int &msg_size);
    int initCfgValset(void);

};

void LLA2NED(double ref_lat, double ref_lon, double lat, double lon, float *x, float *y);
void *gps_thread(void *ptr);
void start_gps(const char *gps_serial);

extern class gps_api_typedef gps_api;
#endif