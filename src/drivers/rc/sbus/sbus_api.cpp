#include "sbus_api.h"

#include <iostream>
#include <chrono>

using std::cout;
using std::endl;

static SBUS sbus;
static sbus_packet_t rc_input;
static sbus_packet_t _buff;
struct sbus_packet_t rc_input_msg_struct;



class sbus_api_typedef sbus_api;
void *sbus_thread(void *ptr)
{
    sbus_api.init((char *)ptr);
    sbus_api.loop(true); //true for demo; false for normal running
    // NOTIC : to run the demo well, short TX-RX is needed
    return NULL;
}
void start_sbus(const char *sbus_serial)
{

  bool ret = create_thread("sbus", sbus_thread, (void*)sbus_serial);

}
sbus_api_typedef::sbus_api_typedef(void)
{

}
void sbus_api_typedef::init(char path[])
{

    sbus.onPacket(onPacket);

    sbus_err_t err = sbus.install(path, false);  // false for non-blocking
    if (err != SBUS_OK)
    {
        printf("SBUS install error\n");
        //return err;
    }

    printf("SBUS installed\n");	
}
void sbus_api_typedef::packet_print(void)
{
        for (int i = 0; i < 16; ++i)
            cout << "ch" << i + 1 << ": " << rc_input.channels[i] << "\t";

        cout << "ch17: " << (rc_input.ch17 ? "true" : "false") << "\t"
             << "ch18: " << (rc_input.ch18 ? "true" : "false");

        if (rc_input.frameLost)
            cout << "\tFrame lost";

        if (rc_input.failsafe)
            cout << "\tFailsafe active";

        cout << endl;
}
static uint16_t limit(uint16_t x, uint16_t min, uint16_t max)
{
    if(x > max) return max;
    else if(x < min) return min;
    else return x;
}
static void rc_normalized(uint16_t channels[SBUS_NUM_CHANNELS])
{
    static int i = 0;
    for(i = 0; i < 16; i++)
    {
        channels[i] = (uint16_t)((channels[i] - RC_MIDDLE_VALUE) / RC_HALF_RANGE * 500 + 1500);
        channels[i] = limit(channels[i], 1000u, 2000u);
    }
}
void sbus_api_typedef::onPacket(const sbus_packet_t &packet)
{
    uint16_t tmp = 0;
    _buff = packet;
    rc_normalized(_buff.channels);
    rc_input = _buff;
    tmp = rc_input.channels[2];
    rc_input.channels[2] = rc_input.channels[3];//switch channel
    rc_input.channels[3] = tmp;
    rc_input_msg.publish(&rc_input);//Attention!!! spin in interrupt
    //printf("%d %d %d %d \n%d %d %d %d\n",rc_input.channels[0],rc_input.channels[1],rc_input.channels[2],rc_input.channels[3],
    //       rc_input.channels[4],rc_input.channels[5],rc_input.channels[6],rc_input.channels[7]);

}
// void sbus_api_typedef::get_rc_input(uint16_t *value, int n_value)
// {
//     for(int i = 0; i < n_value; i++)
//     {
//         value[i] = rc_input.channels[i];
//     }
//     value[3] = rc_input.channels[2];
//     value[2] = rc_input.channels[3];
// }


void sbus_api_typedef::loop(bool demo_en)
{
	sbus_err_t err;
    static int cnt = 0;
  // non-blocking mode, read() will check if any data is available and return immediately
    while ((err = sbus.read()) != SBUS_FAIL)
    {
        // desync means a packet was misaligned and not received properly
        if (err == SBUS_ERR_DESYNC)
        {
            printf("SBUS desync\n");
        }

        // if(demo_en && cnt++ > 10) // for demo
        // {
        //     cnt = 0;
        //     sbus_packet_t packet;
        //     packet.ch17 = true;
        //     packet.ch18 = true;
        //     for (int i = 0; i < 16; i++)
        //     {
        //         packet.channels[i] = get_time_now() % 100;
        //     }
        //     // packet_print();

        //     // make sure to limit sending frequency, SBUS is not that fast
        //     sbus.write(packet);
        // }

        usleep(10000);
    }

}
