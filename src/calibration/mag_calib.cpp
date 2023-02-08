#include "sensor_calibration.h"
bool calibration_typedef::calib_mag(float mag_norm)
{
    mag_raw_typedef _mag_raw;
    imu_raw_typedef _imu_raw;
    /** @aciton ¿ªÊ¼Ð£×¼Á÷³Ì */
    double x_scale;
    double y_scale;
    double z_scale;
    double x_offset;
    double y_offset;
    double z_offset;

    double A[49];
    for (int i = 0; i < 49; i++)
    {
        A[i] = 0.0;
    }

    float  holding_time = 0;//¼ÇÂ¼Ê±ÖÓµÎ´ðÊý
    FILE *fp;
    if( calibration_debug && (fp=fopen("mag_data.txt","wt+")) == NULL ){
        printf("Fail to open file!\n");
        return false;
    }
    printf("start mag calibration process......");

    printf("Please place the (-z) axis of the aircraft upward\n");
    do
    {
        usleep(mag_calibration_delay_us);
        imu_raw_msg.read(&_imu_raw);
    }
    while (!orientation_check(_imu_raw.accel,_imu_raw.gyro,  NZ_UP));//¼ì²âµ½»úÉíÕý·Å
    printf("NZ_UP is detected!\n");
    printf("accel data: accel x = %1.4f, y = %1.4f, z = %1.4f\n", _imu_raw.accel[0], _imu_raw.accel[1],_imu_raw.accel[2]);
    usleep(mag_calibration_delay_us);
    mag_raw_msg.read(&_mag_raw);  
    printf("mag data: mag x = %f, y = %f, z = %f\n", _mag_raw.mag[0], _mag_raw.mag[1], _mag_raw.mag[2]);  
    holding_time = get_time_now()/1e6;//»ñÈ¡µ±Ç°Ê±ÖÓtick  1ms/tick
    printf("Please rotate the aircraft around -z axis\n");
    do
    {
        usleep(mag_calibration_delay_us);
        mag_raw_msg.read(&_mag_raw);
        ellipsoid_method_step1( _mag_raw.mag[0], _mag_raw.mag[1],_mag_raw.mag[2], A);
        if(calibration_debug)fprintf(fp,"%f %f %f\n", _imu_raw.accel[0],_imu_raw.accel[1],_imu_raw.accel[2]);   
    }
    while (get_time_now()/1e6 - holding_time < 7); //¼ì²âµ½ÒÑ¾­¹ýÈ¥7000Ê±ÖÓtick 7s
    usleep(mag_calibration_delay_us);
    printf("step1 is completed!\n");
    printf("A = %.4f\n", A[48]);
    printf(" ");



    printf("Please place the (+x) axis of the aircraft upward\n");
    do
    {
        usleep(mag_calibration_delay_us);
        imu_raw_msg.read(&_imu_raw);
    }
    while (!orientation_check(_imu_raw.accel, _imu_raw.gyro, PX_UP));//¼ì²âµ½»úÉí¶Ç³¯Ìì
    printf("PX_UP is detected!\n");
    printf("accel data: accel x = %1.4f, y = %1.4f, z = %1.4f\n", _imu_raw.accel[0], _imu_raw.accel[1],_imu_raw.accel[2]);
    usleep(mag_calibration_delay_us);
    mag_raw_msg.read(&_mag_raw);  
    printf("mag data: mag x = %f, y = %f, z = %f\n", _mag_raw.mag[0], _mag_raw.mag[1], _mag_raw.mag[2]);    
    holding_time = get_time_now()/1e6;//»ñÈ¡µ±Ç°Ê±ÖÓtick  1ms/tick
    printf("Please rotate the aircraft around +x axis\n");

    do
    {
        usleep(mag_calibration_delay_us);
        mag_raw_msg.read(&_mag_raw);
        ellipsoid_method_step1( _mag_raw.mag[0], _mag_raw.mag[1],_mag_raw.mag[2], A);
        if(calibration_debug)fprintf(fp,"%f %f %f\n", _imu_raw.accel[0],_imu_raw.accel[1],_imu_raw.accel[2]);   
    
    }
    while (get_time_now()/1e6 - holding_time < 7); //¼ì²âµ½ÒÑ¾­¹ýÈ¥7000Ê±ÖÓtick 7s
        usleep(mag_calibration_delay_us);
    printf("step2 is completed!\n");
    printf("A = %1.4f\n", A[48]);
    printf(" ");


    printf("Please place the (-x) axis of the aircraft upward\n");
    do
    {
        usleep(mag_calibration_delay_us);
        imu_raw_msg.read(&_imu_raw);
    }
    while (!orientation_check(_imu_raw.accel,_imu_raw.gyro,  NX_UP));//¼ì²âµ½»úÉíroll=90deg
    printf("NX_UP is detected!\n");
    printf("accel data: accel x = %1.4f, y = %1.4f, z = %1.4f\n", _imu_raw.accel[0], _imu_raw.accel[1],_imu_raw.accel[2]);
    usleep(mag_calibration_delay_us);
    mag_raw_msg.read(&_mag_raw);  
    printf("mag data: mag x = %f, y = %f, z = %f\n", _mag_raw.mag[0], _mag_raw.mag[1], _mag_raw.mag[2]);    
    holding_time = get_time_now()/1e6;//»ñÈ¡µ±Ç°Ê±ÖÓtick  1ms/tick
    printf("Please rotate the aircraft around -x axis\n");

    do
    {
        usleep(mag_calibration_delay_us);
        mag_raw_msg.read(&_mag_raw);
        ellipsoid_method_step1( _mag_raw.mag[0], _mag_raw.mag[1],_mag_raw.mag[2], A);
        if(calibration_debug)fprintf(fp,"%f %f %f\n", _imu_raw.accel[0],_imu_raw.accel[1],_imu_raw.accel[2]);   
    
    }
    while (get_time_now()/1e6 - holding_time < 7); //¼ì²âµ½ÒÑ¾­¹ýÈ¥7000Ê±ÖÓtick 7s
        usleep(mag_calibration_delay_us);
    printf("step3 is completed!\n");
    printf("A = %1.4f\n", A[48]);
    printf(" ");


    printf("Please place the (-y) axis of the aircraft upward\n");
    do
    {
        usleep(mag_calibration_delay_us);
        imu_raw_msg.read(&_imu_raw);
    }
    while (!orientation_check(_imu_raw.accel,_imu_raw.gyro,  NY_UP));//¼ì²âµ½»úÉíroll=-90deg
    printf("NY_UP is detected!\n");
    printf("accel data: accel x = %1.4f, y = %1.4f, z = %1.4f\n", _imu_raw.accel[0], _imu_raw.accel[1],_imu_raw.accel[2]);
    usleep(mag_calibration_delay_us);
    mag_raw_msg.read(&_mag_raw);  
    printf("mag data: mag x = %f, y = %f, z = %f\n", _mag_raw.mag[0], _mag_raw.mag[1], _mag_raw.mag[2]);    
    holding_time = get_time_now()/1e6;//»ñÈ¡µ±Ç°Ê±ÖÓtick  1ms/tick
    printf("Please rotate the aircraft around -y axis\n");

    do
    {
        usleep(mag_calibration_delay_us);
        mag_raw_msg.read(&_mag_raw);
        ellipsoid_method_step1( _mag_raw.mag[0], _mag_raw.mag[1],_mag_raw.mag[2], A);
        if(calibration_debug)fprintf(fp,"%f %f %f\n", _imu_raw.accel[0],_imu_raw.accel[1],_imu_raw.accel[2]);   
    
    }
    while (get_time_now()/1e6 - holding_time < 7); //¼ì²âµ½ÒÑ¾­¹ýÈ¥7000Ê±ÖÓtick 7s
        usleep(mag_calibration_delay_us);
    printf("step4 is completed!");
    printf("A = %1.4f\n", A[48]);
    printf(" ");


    printf("Please place the (+y) axis of the aircraft upward\n");
    do
    {
        usleep(mag_calibration_delay_us);
        imu_raw_msg.read(&_imu_raw);
    }
    while (!orientation_check(_imu_raw.accel,_imu_raw.gyro, PY_UP));//¼ì²âµ½»úÉípitch = -90deg
    printf("PY_UP is detected!\n");
    printf("accel data: accel x = %1.4f, y = %1.4f, z = %1.4f\n", _imu_raw.accel[0], _imu_raw.accel[1],_imu_raw.accel[2]);
    usleep(mag_calibration_delay_us);
    mag_raw_msg.read(&_mag_raw);  
    printf("mag data: mag x = %f, y = %f, z = %f\n", _mag_raw.mag[0], _mag_raw.mag[1], _mag_raw.mag[2]);
    holding_time = get_time_now()/1e6;//»ñÈ¡µ±Ç°Ê±ÖÓtick  1ms/tick
    printf("Please rotate the aircraft around +y axis\n");

    do
    {
        usleep(mag_calibration_delay_us);
        mag_raw_msg.read(&_mag_raw);
        ellipsoid_method_step1( _mag_raw.mag[0], _mag_raw.mag[1],_mag_raw.mag[2], A);
        if(calibration_debug)fprintf(fp,"%f %f %f\n", _imu_raw.accel[0],_imu_raw.accel[1],_imu_raw.accel[2]);   
    
    }
    while (get_time_now()/1e6 - holding_time < 7); //¼ì²âµ½ÒÑ¾­¹ýÈ¥7000Ê±ÖÓtick 7s
        usleep(mag_calibration_delay_us);
    printf("step5 is completed!\n");
    printf("A = %1.4f\n", A[48]);
    printf(" ");


    printf("Please place the (+z) axis of the aircraft upward\n");
    do
    {
        usleep(mag_calibration_delay_us);
        imu_raw_msg.read(&_imu_raw);
    }
    while (!orientation_check(_imu_raw.accel,_imu_raw.gyro,  PZ_UP));//¼ì²âµ½»úÉípitch = 90deg
    printf("PZ_UP is detected!\n");
    printf("accel data: accel x = %1.4f, y = %1.4f, z = %1.4f\n", _imu_raw.accel[0], _imu_raw.accel[1],_imu_raw.accel[2]);
    usleep(mag_calibration_delay_us);
    mag_raw_msg.read(&_mag_raw);  
    printf("mag data: mag x = %f, y = %f, z = %f\n", _mag_raw.mag[0], _mag_raw.mag[1], _mag_raw.mag[2]);    
    holding_time = get_time_now()/1e6;//»ñÈ¡µ±Ç°Ê±ÖÓtick  1ms/tick
    printf("Please rotate the aircraft around +z axis\n");

    do
    {
        usleep(mag_calibration_delay_us);
        mag_raw_msg.read(&_mag_raw);
        ellipsoid_method_step1( _mag_raw.mag[0], _mag_raw.mag[1],_mag_raw.mag[2], A);
        if(calibration_debug)fprintf(fp,"%f %f %f\n", _imu_raw.accel[0],_imu_raw.accel[1],_imu_raw.accel[2]);   
    
    }
    while (get_time_now()/1e6 - holding_time < 7); //¼ì²âµ½ÒÑ¾­¹ýÈ¥7000Ê±ÖÓtick 7s
        usleep(mag_calibration_delay_us);
    printf("step6 is completed!\n");
    printf("A = %1.4f\n", A[48]);
    printf(" ");
    fclose(fp);
    ellipsoid_method_step2(A,
                           &calib_data.mag_scale[0],
                           &calib_data.mag_scale[1],
                           &calib_data.mag_scale[2],
                           &calib_data.mag_offset[0],
                           &calib_data.mag_offset[1],
                           &calib_data.mag_offset[2]);
    for(int i = 0; i < 3; i++)
    {
        calib_data.mag_scale[i] = calib_data.mag_scale[i] * mag_norm;
        calib_data.mag_offset[i] = calib_data.mag_offset[i] * mag_norm;
    }
    printf("%f %f %f \n%f %f %f\n",calib_data.mag_scale[0],calib_data.mag_scale[1],calib_data.mag_scale[2],
        calib_data.mag_offset[0],calib_data.mag_offset[1],calib_data.mag_offset[2]);
    return true;
}

void calibration_typedef::apply_mag_calibration(float mag[3],float mag_calib[3])
{
    if(sensor_calib_enable)
    {
        for(int i = 0 ; i < 3; i++)
        {
            mag_calib[i] = calib_data.mag_scale[i] * mag[i] + calib_data.mag_offset[i];
        }
    }else{
        for(int i = 0 ; i < 3; i++)
        {
            mag_calib[i] = mag[i];
        }
    }

}