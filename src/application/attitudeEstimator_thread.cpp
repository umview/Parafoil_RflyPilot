#include "attitudeEstimator_thread.h"

static ComplementaryFilter AttitudeEstimator_Obj;// Instance of model class
class adaptive_delay_typedef attitude_est_delay(0.5,15,10);

void * thread_attitudeEstimator(void * ptr)
{
    timespec thread_attitudeEstimator_sleep;
    thread_attitudeEstimator_sleep.tv_sec = 0;
    thread_attitudeEstimator_sleep.tv_nsec = 2*1000*1000;//2ms

    /* define attitude estimator input struct */
    core_bind(ATT_EST_CORE);

    /* attitudeEstimator init */
    AttitudeEstimator_Obj.initialize();
    int cont = 500;
    //read accel and gyro
    // imu_typedef _imu;
    // imu_raw_typedef _imu_raw;
    gyro_typedef _gyro;
    accel_typedef _accel;
    // gyro_raw_typedef _gyro_raw;
    // accel_raw_typedef _accel_raw;
    //push cf data
    cf_output_typedef _cf_msg;
    //read mag data from ringbuffer memcpy
    mag_typedef _mag;     
    //set lpe data
    lpe_output_typedef _lpelp_msg;  
          
    scope_data_typedef _att_est_debug;
    rflypilot_config_typedef config;
    rflypilot_config_msg.read(&config);
    thread_msg_typedef _attest;
    #if USING_THREAD_SYNC
    // char count_for_mag=0;
    char count_for_ctrl=0;
    #endif
    while (1)
    {
        #if USING_THREAD_SYNC     
        pthread_mutex_lock(&mutex_lpe2att);  
        pthread_cond_wait(&cond_lpe2att, &mutex_lpe2att);  
        pthread_mutex_unlock(&mutex_lpe2att);
        #endif
        // if(TASK_SCHEDULE_DEBUG)
        // {
        //     _attest.timestamp = get_time_now();
        //     _attest.data_timestamp = _gyro.timestamp;
        //     attest_thread_msg.publish(&_attest); 
        // }

            if(gyro_msg.read(&_gyro))memcpy(&AttitudeEstimator_Obj.rtU._m_gyro_s,&_gyro,sizeof(AttitudeEstimator_Obj.rtU._m_gyro_s));
            // uint64_t time_interval_gyro = get_time_now() - _gyro.timestamp;
            if(accel_msg.read(&_accel))memcpy(&AttitudeEstimator_Obj.rtU._m_accel_s,&_accel,sizeof(AttitudeEstimator_Obj.rtU._m_accel_s));
            // uint64_t time_interval_accel = get_time_now() - _accel.timestamp;
            // uint64_t time_interval_mag;
            if(mag_msg.read(&_mag))
            {
                memcpy(&AttitudeEstimator_Obj.rtU._m_mag_s,&_mag,sizeof(AttitudeEstimator_Obj.rtU._m_mag_s));
                // time_interval_mag = get_time_now() - _mag.timestamp;
            }

            if(lpeLowPass_output_msg.read(&_lpelp_msg)){
                memcpy(&AttitudeEstimator_Obj.rtU._e_lpe_s, &_lpelp_msg, sizeof(AttitudeEstimator_Obj.rtU._e_lpe_s));
            }
            uint64_t time_interval_lpe = get_time_now() - _lpelp_msg.timestamp;

            // mag decl data in beijing
            AttitudeEstimator_Obj.rtU.mag_decl = -0.124;
            AttitudeEstimator_Obj.rtU.time_stamp = get_time_now();
            
            //step
            AttitudeEstimator_Obj.step();


            memcpy(&_cf_msg,&AttitudeEstimator_Obj.rtY._e_cf_s,sizeof(_cf_msg));
            cf_output_msg.publish(&_cf_msg);
            

            for(int i = 0; i < 3; i++)
            {
                _att_est_debug.data[i] = _accel.accel[i];
            }
            att_est_scope_msg.publish(&_att_est_debug);

            cont--;
            if (cont<1)
            {   
                // printf("roll: %f, pitch: %f, yaw: %f\n", AttitudeEstimator_Obj.rtY._e_cf_s.roll, AttitudeEstimator_Obj.rtY._e_cf_s.pitch, AttitudeEstimator_Obj.rtY._e_cf_s.yaw);
                // printf("gyro x: %f, gyro y: %f, gyro z: %f ", _gyro.gyro[0], _gyro.gyro[1], _gyro.gyro[2]);
                // printf("accel x: %f, accel y: %f, accel z: %f ", _accel.accel[0], _accel.accel[1], _accel.accel[2]);
                // printf("mag x: %f, mag y: %f, mag z: %f \n", _mag.mag[0], _mag.mag[1], _mag.mag[2]);
                // printf("Accel Correct: %f %f %f \n", AttitudeEstimator_Obj.rtY._e_cf_status_s.correct_accel[0],AttitudeEstimator_Obj.rtY._e_cf_status_s.correct_accel[1],AttitudeEstimator_Obj.rtY._e_cf_status_s.correct_accel[2]);
                // printf("Mag Correct: %f %f %f \n", AttitudeEstimator_Obj.rtY._e_cf_status_s.correct_mag[0],AttitudeEstimator_Obj.rtY._e_cf_status_s.correct_mag[1],AttitudeEstimator_Obj.rtY._e_cf_status_s.correct_mag[2]);
                // printf("info: LPE time interval from published to used: %lld us\n", time_interval_lpe);
                cont = 3200;
            }

            #if USING_THREAD_SYNC
            if(count_for_ctrl == ATT_CTRL){
                pthread_mutex_lock(&mutex_att2ctrl);  
                pthread_cond_signal(&cond_att2ctrl);   
                pthread_mutex_unlock(&mutex_att2ctrl);
                count_for_ctrl = 0;
            }
            count_for_ctrl++;
            // usleep(400);//900 is best; delay is minimal
            // if(count_for_mag == 8){
            //     pthread_mutex_lock(&mutex_mag2imu);  
            //     pthread_cond_signal(&cond_mag2imu);   
            //     pthread_mutex_unlock(&mutex_mag2imu);
            //     count_for_mag = 0;
            // }
            // count_for_mag++;
            #else
            // nanosleep(&thread_attitudeEstimator_sleep,NULL);
            delay_us_combined((uint64_t)(1000000.f / config.attitude_est_rate),&scheduler.att_est_flag,&attitude_est_delay);
            #endif
    }

    return NULL;
}

void start_attitudeEstimator(void)
{

  bool ret = create_thread("attitudeEstimator", thread_attitudeEstimator, NULL);

}
