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
    int cont = 0;
    //read gps data
    gps_msg_typedef _gps;
    //read accel sensor data message
    // imu_typedef _imu;
    gyro_typedef _gyro;
    accel_typedef _accel;
    // read barometer data 
    baro_typedef _baro_rcv;
    //push data
    lpe_output_typedef _lpe_msg;
    lpe_output_typedef _lpeLowPass_msg;      
    lpe_status_typedef _lpe_status_msg;//lpe_status_msg

    scope_data_typedef _pos_est_debug;

    rflypilot_config_typedef config;
    rflypilot_config_msg.read(&config);
    #if USING_THREAD_SYNC
    char count_for_att = 0;
    #endif
    while (1)
    {   
        #if USING_THREAD_SYNC     
        pthread_mutex_lock(&mutex_imu2lpe);  
        pthread_cond_wait(&cond_imu2lpe, &mutex_imu2lpe);  
        pthread_mutex_unlock(&mutex_imu2lpe);
        #endif
            if(gps_msg.read(&_gps))
            {//read success
                memcpy(&lpe_Obj.rtU._m_gps_s,&_gps,sizeof(lpe_Obj.rtU._m_gps_s));
            }//otherwise do nothing let the struct keep 

            if(accel_msg.read(&_accel)){
                memcpy(&lpe_Obj.rtU._m_accel_s,&_accel,sizeof(lpe_Obj.rtU._m_accel_s));
            }

            //read complementory filter output message
            cf_output_typedef _cf_msg;
            if(cf_output_msg.read(&_cf_msg))memcpy(&lpe_Obj.rtU._e_cf_s,&_cf_msg,sizeof(lpe_Obj.rtU._e_cf_s));
            

            if(baro_msg.read(&_baro_rcv))memcpy(&lpe_Obj.rtU._m_baro_s,&_baro_rcv, sizeof(lpe_Obj.rtU._m_baro_s));

            //set timestamp
            lpe_Obj.rtU.time_stamp = get_time_now();

            //step
            lpe_Obj.step();


            memcpy(&_lpe_msg,&lpe_Obj.rtY._e_lpe_s,sizeof(_lpe_msg));
            lpe_output_msg.publish(&_lpe_msg);

            memcpy(&_lpeLowPass_msg, &lpe_Obj.rtY._e_lpelp_s,sizeof(_lpeLowPass_msg));
            lpeLowPass_output_msg.publish(&_lpeLowPass_msg);

            memcpy(&_lpe_status_msg, &lpe_Obj.rtY._e_lpe_status_s, sizeof(_lpe_status_msg));
            lpe_status_msg.publish(&_lpe_status_msg);

            for(int i = 0; i < 3; i++)
            {
                _pos_est_debug.data[i] = _accel.accel[i];
            }
            pos_est_scope_msg.publish(&_pos_est_debug);

            static bool gpsTimeout_last = true;
            if(!gpsTimeout&&gpsTimeout_last)printf("[lpe] GPS Init status %d, GPS Alt Origin is %f \n", gpsTimeout, gpsAltOrigin);
            gpsTimeout_last = gpsTimeout;
            
            static bool baroTimeout_last = true;
            if(!baroTimeout&&baroTimeout_last)printf("[lpe] Baro Init status %d, Baro Alt Origin is %f, Baro pressure is: %f \n", baroTimeout, baroAltOrigin, _baro_rcv.pressure);
            baroTimeout_last = baroTimeout;

            cont++;
            if (cont>3200)
            {   
                // printf("[lpe] EST Z&XY: %d, %d, baroAltOrigin %f, gpsAltOrigin %f\n",est_zInitialized, est_xyInitialized, baroAltOrigin, gpsAltOrigin);            
                cont = 0;
            }
        
        #if USING_THREAD_SYNC
		if(count_for_att == LPE_ATT)
		{
            pthread_mutex_lock(&mutex_lpe2att);  
            pthread_cond_signal(&cond_lpe2att);   
            pthread_mutex_unlock(&mutex_lpe2att);
            count_for_att = 0;
		}
		count_for_att++;
        #else
        //nanosleep(&thread_lpe_sleep, NULL);
        delay_us_combined((uint64_t)(1000000.f / config.lpe_rate),&scheduler.lpe_flag,&lpe_adaotive_delay);
        #endif
    }

    return NULL;
}

void start_lpe(void)
{

  bool ret = create_thread("lpe", thread_lpe, NULL);

}