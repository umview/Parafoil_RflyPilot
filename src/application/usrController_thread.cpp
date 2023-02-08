#include "usrController_thread.h"

static usrController usrController_Obj;

void * thread_usrController(void * ptr)
{
    timespec thread_usrController_sleep;
    thread_usrController_sleep.tv_sec = 0;
    thread_usrController_sleep.tv_nsec = 2.5*1000*1000;//2ms

    /* define input struct */
    core_bind(2);

    /* usrController init */
    usrController_Obj.initialize();
    int cont = 500;

    sbus_packet_t _rc_input_msg;//rc_input_msg
    //use imu or, gyro and accel according to fc mode 
    imu_typedef _imu_msg;//imu_msg
    gyro_typedef _gyro_msg;//gyro_msg
    accel_typedef _accel_msg;//accel_msg

    mag_typedef _mag_msg;//mag_msg
    baro_typedef _baro_msg;//baro_msg 
    gps_msg_typedef _gps_msg;//gps_msg

    cf_output_typedef _cf_msg;//cf_output_msg     
    lpe_output_typedef _lpe_msg;//lpe_output_msg

    //at present mpc_output_typedef is used for tranfor control, but in future actuator_output_typedef will be used
    actuator_output_typedef _actuator_output_msg;//actuator_output_msg
    mpc_output_typedef _mpc_output_msg;//mpc_output_msg
    pwm_output_typedef _pwm_output_msg;//pwm_output_msg

    while (1)
    {   
        // if(scheduler.att_est_flag)
        // {
            // #ifndef USE_ADAPTIVE_DELAY
            // scheduler.att_est_flag = false;
            // #endif

            //time_stamp
            usrController_Obj.usrController_U.time_stamp = get_time_now();
            //rc
            rc_input_msg.read(&_rc_input_msg);
            memcpy(&usrController_Obj.usrController_U._c_subs_s, &_rc_input_msg, sizeof(usrController_Obj.usrController_U._c_subs_s));
            // usrController_Obj.usrController_U._c_subs_s.channels[0] = 1500;
            // usrController_Obj.usrController_U._c_subs_s.channels[1] = 1500;
            usrController_Obj.usrController_U._c_subs_s.channels[2] = _rc_input_msg.channels[3];
            usrController_Obj.usrController_U._c_subs_s.channels[3] = _rc_input_msg.channels[2];
            //set gyro and accel
            //EXP2 use real pilotpi with real sensor
            // if(fc_api.valid_mode == EXP2 || fc_api.valid_mode == SIH){
                gyro_msg.read(&_gyro_msg);
                accel_msg.read(&_accel_msg);
                memcpy(&usrController_Obj.usrController_U._m_gyro_s,&_gyro_msg,sizeof(usrController_Obj.usrController_U._m_gyro_s));
                memcpy(&usrController_Obj.usrController_U._m_accel_s,&_accel_msg,sizeof(usrController_Obj.usrController_U._m_accel_s));
            // }else{
            //     imu_msg.read(&_imu_msg);
            //     memcpy(&usrController_Obj.usrController_U._m_accel_s.accel_data, &_imu_msg.accel, sizeof(usrController_Obj.usrController_U._m_accel_s.accel_data));
            //     memcpy(&usrController_Obj.usrController_U._m_gyro_s.gyro_data, &_imu_msg.gyro, sizeof(usrController_Obj.usrController_U._m_gyro_s.gyro_data));
            //     usrController_Obj.usrController_U._m_accel_s.time_stamp = _imu_msg.timestamp;
            //     usrController_Obj.usrController_U._m_gyro_s.time_stamp = _imu_msg.timestamp;
            // }
            // mag check pass
            mag_msg.read(&_mag_msg);
            memcpy(&usrController_Obj.usrController_U._m_mag_s, &_mag_msg, sizeof(usrController_Obj.usrController_U._m_mag_s));
            // baro check pass
            baro_msg.read(&_baro_msg);
            memcpy(&usrController_Obj.usrController_U._m_baro_s, &_baro_msg, sizeof(usrController_Obj.usrController_U._m_baro_s));
            // gps check pass
            gps_msg.read(&_gps_msg);
            memcpy(&usrController_Obj.usrController_U._m_gps_s, &_gps_msg, sizeof(usrController_Obj.usrController_U._m_gps_s));
            // cf check pass
            cf_output_msg.read(&_cf_msg);
            memcpy(&usrController_Obj.usrController_U._e_cf_s, &_cf_msg, sizeof(usrController_Obj.usrController_U._e_cf_s));
            //lpe check pass
            lpe_output_msg.read(&_lpe_msg);
            memcpy(&usrController_Obj.usrController_U._e_lpe_s, &_lpe_msg, sizeof(usrController_Obj.usrController_U._e_lpe_s));
            //run step
            usrController_Obj.step();
            //set output
            memcpy(&_mpc_output_msg.thrust,&usrController_Obj.usrController_Y._c_out_s.thrust, sizeof(_mpc_output_msg.thrust));
            _mpc_output_msg.timestamp = usrController_Obj.usrController_Y._c_out_s.time_stamp;
            mpc_output_msg.publish(&_mpc_output_msg);
            
            memcpy(&_pwm_output_msg.pwm, &usrController_Obj.usrController_Y._c_out_s.pwm, sizeof(_pwm_output_msg.pwm));
            _pwm_output_msg.timestamp = usrController_Obj.usrController_Y._c_out_s.time_stamp;
            pwm_output_msg.publish(&_pwm_output_msg);

            // pca9685_dev.updatePWM(usrController_Obj.usrController_Y._c_out_s.pwm,4);
            // memcpy(&_actuator_output_msg.actuator_output, &usrController_Obj.usrController_Y._c_out_s.pwm, sizeof(_actuator_output_msg.actuator_output));
            // _actuator_output_msg.timestamp = usrController_Obj.usrController_Y._c_out_s.time_stamp;
            // actuator_output_msg.publish(&_actuator_output_msg);
            
            cont--;
            if (cont<1)
            {   
                cont = 500;
                // printf("thrust: %f %f %f %f ", _mpc_output_msg.thrust[0], _mpc_output_msg.thrust[1], _mpc_output_msg.thrust[2], _mpc_output_msg.thrust[3]);
                // printf("pwm: %d %d %d %d ",_pwm_output_msg.pwm[0], _pwm_output_msg.pwm[1], _pwm_output_msg.pwm[2], _pwm_output_msg.pwm[3]);
                // printf("gyro: %f %f %f ",usrController_Obj.usrController_U._m_gyro_s.gyro_data[0], usrController_Obj.usrController_U._m_gyro_s.gyro_data[1], usrController_Obj.usrController_U._m_gyro_s.gyro_data[2]);
                // printf("quat: %f %f %f %f ",usrController_Obj.usrController_U._e_cf_s.quat_data[0], usrController_Obj.usrController_U._e_cf_s.quat_data[1], usrController_Obj.usrController_U._e_cf_s.quat_data[2], usrController_Obj.usrController_U._e_cf_s.quat_data[3]);
                // printf("vel: %f %f %f ", usrController_Obj.usrController_U._e_lpe_s.vel_ned[0], usrController_Obj.usrController_U._e_lpe_s.vel_ned[1], usrController_Obj.usrController_U._e_lpe_s.vel_ned[2]);
                // printf("sbus: %d %d %d %d \n", usrController_Obj.usrController_U._c_subs_s.channels[0], usrController_Obj.usrController_U._c_subs_s.channels[1], usrController_Obj.usrController_U._c_subs_s.channels[2], usrController_Obj.usrController_U._c_subs_s.channels[3]);
            }
        // }
        // #ifndef USE_ADAPTIVE_DELAY
        nanosleep(&thread_usrController_sleep,NULL);
        // #else
        // attitude_est_delay.delay_us(1e6/config.attitude_est_rate);
        // #endif
    }

    return NULL;
}

void start_usrController(void)
{
    int rc;
    pthread_t thr_usrController;
    if(rc = pthread_create(&thr_usrController, NULL, thread_usrController, NULL))
    {
		printf("usrController thread cretated failed %d \n", rc);
    }
    printf("usrController thread created with process pid : %d\n", (int)getpid()); 
}