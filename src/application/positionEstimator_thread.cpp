#include "positionEstimator_thread.h"

static local_position_estimator lpe_Obj; // Instance of model class
class adaptive_delay_typedef lpe_adaotive_delay(0.5,15,10);

void * thread_lpe(void * ptr)
{
    timespec thread_lpe_sleep;
    thread_lpe_sleep.tv_sec = 0;
    thread_lpe_sleep.tv_nsec = 2*1000*1000;//10ms

    /* define lpe input struct */
    core_bind(POS_EST_CORE);

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
    lpe_output_typedef _lpeLowPass_msg;      

    scope_data_typedef _pos_est_debug;

    rflypilot_config_typedef config;
    rflypilot_config_msg.read(&config);
    while (1)
    {   

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
            

            if(baro_msg.read(&_baro_rcv))memcpy(&lpe_Obj.rtU._m_baro_s,&_baro_rcv, sizeof(lpe_Obj.rtU._m_baro_s));

            //set usec
            lpe_Obj.rtU.usec = get_time_now();

            //step
            lpe_Obj.step();


            memcpy(&_lpe_msg,&lpe_Obj.rtY._e_lpe_s,sizeof(_lpe_msg));
            lpe_output_msg.publish(&_lpe_msg);

            memcpy(&_lpeLowPass_msg, &lpe_Obj.rtY._e_lpelp_s,sizeof(_lpeLowPass_msg));
            lpeLowPass_output_msg.publish(&_lpeLowPass_msg);

            for(int i = 0; i < 3; i++)
            {
                _pos_est_debug.data[i] = _accel.accel[i];
            }
            pos_est_scope_msg.publish(&_pos_est_debug);

            static bool gpsInitOK_last = false;
            if(gpsInitOK&&!gpsInitOK_last)printf("GPS Init status %d, GPS Alt Origin is %f \n",gpsInitOK, gpsAltOrigin);
            gpsInitOK_last = gpsInitOK;
            
            static bool baroInitOK_last = false;
            if(baroInitOK&&!baroInitOK_last)printf("Baro Init status %d, Baro Alt Origin is %f, Baro pressure is: %f \n",baroInitOK, baroAltOrigin, _baro_rcv.pressure);
            baroInitOK_last = baroInitOK;

            cont--;
            if (cont<1)
            {   
                if(!gpsInitOK)printf("GPS hacc is %f, vacc is %f, sacc is %f, numSV is %d\n", _gps.hacc, _gps.vacc, _gps.sacc, _gps.numSV);
                // printf("roll: %f, pitch: %f, yaw: %f\n", _cf_msg.roll, _cf_msg.pitch, _cf_msg.yaw);
                //  printf("px: %f, py: %f, pz: %f ", lpe_Obj.rtY._e_lpe_s.pos_ned[0], lpe_Obj.rtY._e_lpe_s.pos_ned[1], lpe_Obj.rtY._e_lpe_s.pos_ned[2]);
                //  printf("vx: %f, vy: %f, vz: %f ", lpe_Obj.rtY._e_lpe_s.vel_ned[0], lpe_Obj.rtY._e_lpe_s.vel_ned[1], lpe_Obj.rtY._e_lpe_s.vel_ned[2]);
                // printf("abx: %f, aby: %f, abz: %f\n", lpe_Obj.rtY._e_lpe_s.accel_bias[0], lpe_Obj.rtY._e_lpe_s.accel_bias[1], lpe_Obj.rtY._e_lpe_s.accel_bias[2]);
                // printf("body_accx: %f, body_accy: %f, body_accz: %f\n", lpe_Obj.rtY._e_lpe_s.pos_accel_body[0], lpe_Obj.rtY._e_lpe_s.pos_accel_body[1], lpe_Obj.rtY._e_lpe_s.pos_accel_body[2]);
                // printf("Baro Init status %d, Baro Alt Origin is %f, Baro pressure is: %f ",baroInitOK, baroAltOrigin, _baro_rcv.pressure);
                // printf("P_p: %f, %f, %f ", lpe_Obj.rtY._e_lpe_status_s.p_diag[0],lpe_Obj.rtY._e_lpe_status_s.p_diag[1],lpe_Obj.rtY._e_lpe_status_s.p_diag[2]);
                // printf("P_v: %f, %f, %f ", lpe_Obj.rtY._e_lpe_status_s.p_diag[3],lpe_Obj.rtY._e_lpe_status_s.p_diag[4],lpe_Obj.rtY._e_lpe_status_s.p_diag[5]);
                // printf("P_b: %f, %f, %f \n", lpe_Obj.rtY._e_lpe_status_s.p_diag[6],lpe_Obj.rtY._e_lpe_status_s.p_diag[7],lpe_Obj.rtY._e_lpe_status_s.p_diag[8]);
                cont = 1000;
            }
        //}
        //nanosleep(&thread_lpe_sleep, NULL);
        delay_us_combined((uint64_t)(1000000.f / config.lpe_rate),&scheduler.lpe_flag,&lpe_adaotive_delay);

    }

    return NULL;
}

void start_lpe(void)
{

  bool ret = create_thread("lpe", thread_lpe, NULL);

}