#ifndef _SBUS_API_
#define _SBUS_API_
#include "include.h"
#include "SBUS.h"
void sbus_demo(void);

#define RC_MIDDLE_VALUE  1000.f
#define RC_HALF_RANGE 600.f

class sbus_api_typedef
{
public:

	sbus_api_typedef(void);
	void init(char path[]);
	void packet_print(void);
	static void onPacket(const sbus_packet_t &packet);
	void loop(bool demo_en);
	//void get_rc_input(uint16_t *value, int n_value);
};
extern class sbus_api_typedef sbus_api;

#endif