#include "attitudeEstimator_thread.h"

static ComplementaryFilter AttitudeEstimator_Obj;// Instance of model class
class adaptive_delay_typedef attitude_est_delay(0.5,15,10);

void * thread_attitudeEstimator(void * ptr)
{
    timespec thread_attitudeEstimator_sleep;
    thread_attitudeEstimator_sleep.tv_sec = 0;
    thread_attitudeEstimator_sleep.tv_nsec = 2*1000*1000;//2ms

    /* define attitude estimator input struct */
    core_bind(0);

    /* attitudeEstimator init */
    AttitudeEstimator_Obj.initialize();
    int cont = 500;
    //read accel and gyro
    imu_typedef _imu;
    imu_raw_typedef _imu_raw;
    gyro_typedef _gyro;
    accel_typedef _accel;
    gyro_raw_typedef _gyro_raw;
    accel_raw_typedef _accel_raw;
    //push cf data
    cf_output_typedef _cf_msg;
    //read mag data from ringbuffer memcpy
    mag_typedef _mag;     
    //set lpe data
    lpe_output_typedef _lpe_msg;  
          
    scope_data_typedef _att_est_debug;
    rflypilot_config_typedef config;
    rflypilot_config_msg.read(&config);

    while (1)
    {   

                if(gyro_msg.read(&_gyro) && accel_msg.read(&_accel)){
                    memcpy(&AttitudeEstimator_Obj.rtU._m_gyro_s,&_gyro,sizeof(AttitudeEstimator_Obj.rtU._m_gyro_s));
                    memcpy(&AttitudeEstimator_Obj.rtU._m_accel_s,&_accel,sizeof(AttitudeEstimator_Obj.rtU._m_accel_s));
                    _imu.timestamp = _gyro.timestamp;
                    for(int i = 0; i < 3; i++)
                    {
                        _imu.accel[i] = _accel.accel[i];
                        _imu.gyro[i] = _gyro.gyro[i];
                    }
                    imu_msg.publish(&_imu);
                    gyro_raw_msg.read(&_gyro_raw);
                    accel_raw_msg.read(&_accel_raw);
                    _imu_raw.timestamp = _gyro_raw.timestamp;
                    for(int i = 0; i < 3; i++)
                    {
                        _imu_raw.accel[i] = _accel_raw.accel[i];
                        _imu_raw.gyro[i] = _gyro_raw.gyro[i];
                    }
                    imu_raw_msg.publish(&_imu_raw);

                }
            // }else{
            //     if(imu_msg.read(&_imu)){
            //         for(int i= 0;i<3;i++)
            //         {
            //             AttitudeEstimator_Obj.rtU._m_accel_s.accel_data[i] = _imu.accel[i];
            //             AttitudeEstimator_Obj.rtU._m_gyro_s.gyro_data[i] = _imu.gyro[i];
            //         }
            //         AttitudeEstimator_Obj.rtU._m_accel_s.time_stamp = _imu.timestamp;
            //         AttitudeEstimator_Obj.rtU._m_gyro_s.time_stamp = _imu.timestamp;
            //     }
            // }

            if(mag_msg.read(&_mag))memcpy(&AttitudeEstimator_Obj.rtU._m_mag_s,&_mag,sizeof(AttitudeEstimator_Obj.rtU._m_mag_s));


            if(lpe_output_msg.read(&_lpe_msg)){
                memcpy(&AttitudeEstimator_Obj.rtU._e_lpe_s, &_lpe_msg, sizeof(AttitudeEstimator_Obj.rtU._e_lpe_s));
            }

            // mag decl data in beijing
            AttitudeEstimator_Obj.rtU.mag_decl = -0.124;
            AttitudeEstimator_Obj.rtU.usec = get_time_now();
            
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
                cont = 500;
            }

        //nanosleep(&thread_attitudeEstimator_sleep,NULL);
        delay_us_combined((uint64_t)(1000000.f / config.attitude_est_rate),&scheduler.att_est_flag,&attitude_est_delay);
    }

    return NULL;
}

void start_attitudeEstimator(void)
{

  bool ret = create_thread("attitudeEstimator", thread_attitudeEstimator, NULL);

}
