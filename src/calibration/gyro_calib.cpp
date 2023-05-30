#include "sensor_calibration.h"
bool calibration_typedef::calib_gyro(void)
{
	// imu_raw_typedef _imu_raw;
    gyro_raw_typedef _gyro_raw;
    accel_raw_typedef _accel_raw;
    double x_offset = 0;
    double y_offset = 0;
    double z_offset = 0;
    printf("start gyro calibration process......\n");
    printf("Please do not move the aircraft\n");
    FILE *fp;
    if( calibration_debug && (fp=fopen("gyro_data.txt","wt+")) == NULL ){
        printf("Fail to open file!\n");
        return false;
    }

    for (int readcount = 0; readcount < 1400; readcount++)
    {
        usleep(calibration_delay_us);
        // imu_raw_msg.read(&_imu_raw);
        gyro_raw_msg.read(&_gyro_raw);
        x_offset += _gyro_raw.gyro[0];
        y_offset += _gyro_raw.gyro[1];
        z_offset += _gyro_raw.gyro[2];
        if(calibration_debug)fprintf(fp,"%f %f %f\n", _gyro_raw.gyro[0],_gyro_raw.gyro[1],_gyro_raw.gyro[2]);   
        //printf("%f %f %f\n", _gyro_raw.gyro[0],_gyro_raw.gyro[1],_gyro_raw.gyro[2]);
    }

    calib_data.gyro_offset[0] = x_offset / 1400;
    calib_data.gyro_offset[1] = y_offset / 1400;
    calib_data.gyro_offset[2] = z_offset / 1400;
    for(int i = 0; i < 3; i++)calib_data.gyro_scale[i] = 1;
    printf(" gyro calibration parameter is:\n");
    printf(" gyro_x_offset =	%f\n", calib_data.gyro_offset[0]);
    printf(" gyro_y_offset =	%f\n", calib_data.gyro_offset[1]);
    printf(" gyro_z_offset =	%f\n", calib_data.gyro_offset[2]);
    fclose(fp);
    /** @aciton end */
    return true;
}
void calibration_typedef::apply_gyro_calibration(float gyro[3],float gyro_calib[3])
{
    if(sensor_calib_enable)
    {
        for(int i = 0 ; i < 3; i++)
        {
            gyro_calib[i] = calib_data.gyro_scale[i] * gyro[i] - calib_data.gyro_offset[i];
        }
    }else{
        for(int i = 0 ; i < 3; i++)
        {
            gyro_calib[i] = gyro[i];
        }
    }

}