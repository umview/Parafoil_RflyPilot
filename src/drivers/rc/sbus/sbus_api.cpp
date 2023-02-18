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
    rc_input_typedef rc_input;

    // uint64_t timestamp;
    // uint16_t channels[SBUS_NUM_CHANNELS];
    // bool ch17, ch18;
    // bool failsafe;
    // bool frameLost;

    // rc_input.timestamp = get_time_now();
    // for(int i = 0; i < SBUS_NUM_CHANNELS; i++)
    // {
    //     rc_input.channels[i] = 1000;
    // }
    // rc_input.ch17 = false;
    // rc_input.ch18 = false;
    // rc
    // rc_input_msg.publish(&rc_input);//Attention!!! spin in interrupt

    sbus_api.loop(false); //true for demo; false for normal running
    // NOTIC : to run the demo well, short TX-RX is needed
    return NULL;
}
void start_sbus(const char *sbus_serial)
{
  int rc;
  pthread_t thr;
  if(rc = pthread_create(&thr, NULL, sbus_thread, (void *)sbus_serial))
  {
    printf(" thread cretated failed %d \n", rc);
  }
  printf("sbus thread cretated\n"); 
}

static bool range_check(float x, float min, float max)
{
    if(x > min && x < max)
        return true;
    else 
        return false;
}
void rc_check(void)
{
    rc_input_typedef rc_input;
    float t0 = get_time_now() / 1e6;
    usleep(500000);
    while(1)
    {
        rc_input_msg.read(&rc_input);
        if(range_check(rc_input.channels[0], 1400,1600)
            && range_check(rc_input.channels[1], 1400,1600)
            && range_check(rc_input.channels[2], 1400,1600)
            && range_check(rc_input.channels[3], 0,1100) 
            && range_check(rc_input.channels[5], 0,1100)
            && rc_input.failsafe == false
            && rc_input.frameLost == false)
        {
            if(((get_time_now() / 1e6) - t0) > 1)
            {
                printf("rc checking passed\n");
                break;
            }
        }else{
            t0 = get_time_now() / 1e6;
        }
        usleep(100000);
        printf("checking remote controller status\n");
    printf("rc channels input\n%d %d %d %d \n%d %d %d %d\n",rc_input.channels[0],rc_input.channels[1],rc_input.channels[2],rc_input.channels[3],
          rc_input.channels[4],rc_input.channels[5],rc_input.channels[6],rc_input.channels[7]);
    printf("failsafe %d framelost %d\n", rc_input.failsafe, rc_input.frameLost);
    }
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
        printf("SBUS install error: %d\n", (int)err);
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
    rc_input.timestamp = get_time_now();
    rc_input_msg.publish(&rc_input);//Attention!!! spin in interrupt
    // printf("%d %d %d %d \n%d %d %d %d\n",rc_input.channels[0],rc_input.channels[1],rc_input.channels[2],rc_input.channels[3],
    //       rc_input.channels[4],rc_input.channels[5],rc_input.channels[6],rc_input.channels[7]);
    // printf("failsafe %d framelost %d\n", rc_input.failsafe, rc_input.frameLost);
}


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

        if(demo_en && cnt++ > 10) // for demo
        {
            cnt = 0;
            sbus_packet_t packet;
            packet.ch17 = true;
            packet.ch18 = true;
            for (int i = 0; i < 16; i++)
            {
                packet.channels[i] = get_time_now() % 100;
            }
            // packet_print();

            // make sure to limit sending frequency, SBUS is not that fast
            sbus.write(packet);
        }

        usleep(10000);
    }

}
