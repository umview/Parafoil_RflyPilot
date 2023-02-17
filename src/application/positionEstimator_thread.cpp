#include "positionEstimator_thread.h"

static local_position_estimator lpe_Obj; // Instance of model class
class adaptive_delay_typedef lpe_rate(0.5,15,10);

void * thread_lpe(void * ptr)
{
    timespec thread_lpe_sleep;
    thread_lpe_sleep.tv_sec = 0;
    thread_lpe_sleep.tv_nsec = 2*1000*1000;//10ms

    /* define lpe input struct */
    core_bind(0);

    /* lpe init */
    lpe_Obj.initialize();
    int cont = 100;
    //read gps data
    gps_msg_typedef _gps;
    //read accel sensor data message
    imu_typedef _imu;
    gyro_typedef _gyro;
    accel_typedef _accel;
    // read barometer data 
    baro_typedef _baro_rcv;
    //push data
    lpe_output_typedef _lpe_msg;      

    scope_data_typedef _pos_est_debug;


    while (1)
    {   
        // if(scheduler.lpe_flag)
        // {
        //     #ifndef USE_ADAPTIVE_DELAY
        //     scheduler.lpe_flag = false;
        //     #endif
            if(gps_msg.read(&_gps))
            {//read success
                memcpy(&lpe_Obj.rtU._m_gps_s,&_gps,sizeof(lpe_Obj.rtU._m_gps_s));
            }//otherwise do nothing let the struct keep 


            // if(fc_api.valid_mode == EXP2 || fc_api.valid_mode == SIH){
                if(accel_msg.read(&_accel)){
                memcpy(&lpe_Obj.rtU._m_accel_s,&_accel,sizeof(lpe_Obj.rtU._m_accel_s));
                }
            // }else{
            //     if(imu_msg.read(&_imu)){
            //         memcpy(lpe_Obj.rtU._m_accel_s.accel_data,_imu.accel,sizeof(lpe_Obj.rtU._m_accel_s.accel_data));
            //         lpe_Obj.rtU._m_accel_s.time_stamp = _imu.timestamp;
            //     }
            // } 

            //read complementory filter output message
            cf_output_typedef _cf_msg;
            if(cf_output_msg.read(&_cf_msg))memcpy(&lpe_Obj.rtU._e_cf_s,&_cf_msg,sizeof(lpe_Obj.rtU._e_cf_s));
            

            // if(baro_msg.read(&_baro_rcv,1))memcpy(&lpe_Obj.rtU._m_baro_s,&_baro_rcv, sizeof(lpe_Obj.rtU._m_baro_s));

            //set usec
            lpe_Obj.rtU.usec = get_time_now();

            //step
            lpe_Obj.step();


            memcpy(&_lpe_msg,&lpe_Obj.rtY._e_lpe_s,sizeof(_lpe_msg));
            lpe_output_msg.publish(&_lpe_msg);

            for(int i = 0; i < 3; i++)
            {
                _pos_est_debug.data[i] = _accel.accel[i];
            }
            pos_est_scope_msg.publish(&_pos_est_debug);

            cont--;
            if (cont<1)
            {   
                // printf("roll: %f, pitch: %f, yaw: %f\n", _cf_msg.roll, _cf_msg.pitch, _cf_msg.yaw);
                // printf("px: %f, py: %f, pz: %f ", lpe_Obj.rtY._e_lpe_s.pos_ned[0], lpe_Obj.rtY._e_lpe_s.pos_ned[1], lpe_Obj.rtY._e_lpe_s.pos_ned[2]);
                // printf("vx: %f, vy: %f, vz: %f ", lpe_Obj.rtY._e_lpe_s.vel_ned[0], lpe_Obj.rtY._e_lpe_s.vel_ned[1], lpe_Obj.rtY._e_lpe_s.vel_ned[2]);
                // printf("abx: %f, aby: %f, abz: %f\n", lpe_Obj.rtY._e_lpe_s.accel_bias[0], lpe_Obj.rtY._e_lpe_s.accel_bias[1], lpe_Obj.rtY._e_lpe_s.accel_bias[2]);
                // printf("body_accx: %f, body_accy: %f, body_accz: %f\n", lpe_Obj.rtY._e_lpe_s.pos_accel_body[0], lpe_Obj.rtY._e_lpe_s.pos_accel_body[1], lpe_Obj.rtY._e_lpe_s.pos_accel_body[2]);
                cont = 100;
            }
        //}
        //#ifndef USE_ADAPTIVE_DELAY
        nanosleep(&thread_lpe_sleep, NULL);
        // #else
        // lpe_rate.delay_us(1e6/config.lpe_rate);
        // #endif
    }

    return NULL;
}

void start_lpe(void)
{

  bool ret = create_thread("lpe", thread_lpe, NULL);

}