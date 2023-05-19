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
    enum class GPSRestartType {
        None,
        Hot,
        Warm,
        Cold
    };
    GPSRestartType _resetType;
    int gps_debug;
    gps_pvt_msg_typedef pvt_msg;
    sensor_gps_typedef sensor_gps;
    uint8_t ack_clsID;
    uint8_t ack_msgID;
    bool pvt_msg_available;
    /*********for realtime decoding*****/
    ubx_decode_state_t _decode_state;
    uint8_t _rx_ck_a{0};
    uint8_t _rx_ck_b{0};
    uint16_t _rx_payload_length{0};
    uint16_t _rx_payload_index{0};
    uint16_t _rx_msg{};
    ubx_buf_t   _buf{};
    int payloadRxAdd(const uint8_t b);

    /**********************************/
	gps_api_typedef(void);
    void init(char *_port, speed_t speed);
    bool gps_config(char *_port);
	int _serial_fd;
	void run(void);
    void run2(void);
	bool sendMessage(const uint16_t msg, const uint8_t *payload, const uint16_t length);
    bool sendMessageACK(const uint16_t msg, const uint8_t *payload, const uint16_t length);
    void pvt_decode(ubx_payload_rx_nav_pvt_t* nav_pvt);
    void realtime_decode_msg(uint16_t _msg, uint8_t *payload);

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
    void decodeInit();
    void addByteToChecksum(const uint8_t b);
    uint32_t read_available_bytes(uint8_t *frame, int MAX_MSG_LENGHT);
    int realtime_decode(uint8_t b);
    template<typename T>
    bool cfgValset(uint32_t key_id, T value, int &msg_size);
    int initCfgValset(void);

    int reset(GPSRestartType restart_type);
    // int restartSurveyIn(void);
    bool cfgValsetPort(uint32_t key_id, uint8_t value, int &msg_size);
    int configureDevicePreV27(void);
    int configureDevice(void);
    int disableMsg(uint8_t msg_class, uint8_t msg_id);
    // int waitForAck(const uint16_t msg, const unsigned timeout, const bool report);

    bool _proto_ver_27_or_higher{false}; ///< true if protocol version 27 or higher detected
    bool _use_nav_pvt{false};
    uint64_t last_time_read{0};
    ubx_ack_state_t  _ack_state{UBX_ACK_IDLE};
    bool _configured{false};
};

void LLA2NED(double ref_lat, double ref_lon, double lat, double lon, float *x, float *y);
void *gps_thread(void *ptr);
void start_gps(const char *gps_serial);

extern class gps_api_typedef gps_api;
#endif