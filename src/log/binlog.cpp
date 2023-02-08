#include "binlog.h"

union log_union_typedef binlog_data;
int fp;
void binlog_demo(void)
{
    // //从键盘输入的数据放入a，从文件读取的数据放入b
    // #define len  5
    // int a[len], b[len];
    // int i, size = sizeof(int);
    // if( (fp=fopen("../log/log.bin", "a+")) == NULL ){  //以二进制方式打开
    //     puts("Fail to open file!");
    //     exit(0);
    // }
  
    // //从键盘输入数据 并保存到数组a
    // a[0] = 1;
    // a[1] = 2;
    // a[2] = 3;
    // a[3] = 4;
    // a[4] = 5;
    // //将数组a的内容写入到文件
    // fwrite(a, size, len, fp);
    // //将文件中的位置指针重新定位到文件开头
    // rewind(fp);
    // //从文件读取内容并保存到数组b
    // fread(b, size, len, fp);
    // //在屏幕上显示数组b的内容
    // for(i=0; i<len; i++){
    //     printf("%d ", b[i]);
    // }
    // printf("\n");
    // fclose(fp);
}
void binlog_write(void)
{
	static bool initd = false;
    static char time_buf[50]={0};
    get_compact_time_string(time(NULL), time_buf);    
    if(!initd)
    {
		fp=open(time_buf, O_RDWR | O_CREAT,0777);
        if(fp == -1){
                printf("open failed!\n");
        }else{
                printf("open success!\n");
        }
	    printf("sizeof debug log typedef : %d\n",sizeof(struct debug_log_typedef));
	    initd = true;
    }


    //将数组a的内容写入到文件
    write(fp,binlog_data.buff, sizeof(struct debug_log_typedef));
    //fflush(fp);
}
void log_data_prepare(void)
{
    static float *buff = binlog_data.s.data;
    static int i = 0;
    buff[0] = 1234;

    cf_output_typedef _att;
    lpe_output_typedef _lpe;
    imu_typedef _imu;
    pid_output_typedef _pid_output;
    gps_msg_typedef _gps;
    mag_typedef _mag;
    actuator_output_typedef _actuator_output;
    imu_raw_typedef _imu_raw;
    cf_output_msg.read(&_att);
    lpe_output_msg.read(&_lpe);
    imu_msg.read(&_imu);
    pid_output_msg.read(&_pid_output);
    gps_msg.read(&_gps);
    mag_msg.read(&_mag);
    actuator_output_msg.read(&_actuator_output);
    imu_raw_msg.read(&_imu_raw);
    for(i = 0; i < 4; i++)
    {
        buff[i+1] = _att.quat[i];
    }
    for(i = 0; i < 3; i++)
    {
        buff[i+5] = _imu.gyro[i];
    }
    for(i = 0; i<4;i++)
    {
      buff[i+8] = fc_api.u_control_publish[i];
    }
    buff[11+1] = mpc_output_msg.publish_rate_hz;
    for(i = 0; i < 3; i++)buff[i+13] = nmpc_api.accel[i];
    for(i = 0; i < 3; i++)buff[i+16] = pid_api.obj.pid_controller_Y.accel_sp[i];
    for(i = 0; i < 4; i++)buff[i+19] = pid_api.obj.pid_controller_Y.q_sp[i];//q_sp
    for(i = 0; i < 3; i++)buff[i+23] = 0;//omega_ref
    for(i = 0; i < 4; i++)buff[i+26] = (float)_actuator_output.actuator_output[i];//pwm
    buff[30] = nmpc_api.cost;//cost
    buff[31] = pid_output_msg.publish_rate_hz;
    buff[32] = (float)fc_api.ControlAlg;
    buff[33] = get_time_now() / 1e6;
    buff[34] = _imu.accel[0];
    buff[35] = _imu.accel[1];
    buff[36] = _imu.accel[2];
    // buff[37] = _pid_output.vel_sp_ned[0];
    // buff[38] = _pid_output.vel_sp_ned[1];
    // buff[39] = _pid_output.vel_sp_ned[2];
    for(i = 0; i< 3; i++)buff[i+37] = _gps.pos_ned[i];
    for(i = 0; i< 3; i++)buff[i+40] = _gps.vel_ned[i];
    for(i = 0; i< 3; i++)buff[i+43] = _lpe.pos_ned[i];
    for(i = 0; i< 3; i++)buff[i+46] = _lpe.vel_ned[i];
    for(i = 0; i< 3; i++)buff[i+49] = _lpe.pos_accel_body[i];
    for(i = 0; i< 3; i++)buff[i+52] = _lpe.accel_bias[i];
    buff[55] = _att.roll;
    buff[56] = _att.pitch;
    buff[57] = _att.yaw;
    for(i = 0; i< 3; i++)buff[i+58] = _pid_output.vel_sp_ned[i];
    for(i = 0; i< 3; i++)buff[i+61] = _pid_output.pos_sp_ned[i];
    for(i = 0; i< 3; i++)buff[i+64] = _pid_output.accel_setpoint[i];
    buff[67] = pid_output_msg.publish_rate_hz;
    buff[68] = mpc_output_msg.publish_rate_hz;
    buff[69] = cf_output_msg.publish_rate_hz;
    buff[70] = lpe_output_msg.publish_rate_hz;
    buff[71] = imu_msg.publish_rate_hz;
    buff[72] = actuator_output_msg.publish_rate_hz;
    buff[73] = gyro_msg.publish_rate_hz;
    buff[74] = mag_msg.publish_rate_hz;
    buff[75] = (float)mpc_delay.delay_ns_output;
    buff[76] = (float)pid_controller_delay.delay_ns_output;
    buff[77] = (float)actuator_delay.delay_ns_output;
    buff[78] = (float)icm20689_delay.delay_ns_output;
    buff[79] = (float)attitude_est_delay.delay_ns_output;
    //for(i = 0; i< 3; i++)buff[i+75] = _mag.mag[i];
    //for(i = 0; i< 3; i++)buff[i+75] = _imu_raw.gyro[i];


}