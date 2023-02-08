#include "sensor_calibration.h"
bool calibration_typedef::calib_accel(double G)
{
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
    if( calibration_debug && (fp=fopen("accel_data.txt","wt+")) == NULL ){
        printf("Fail to open file!\n");
        return false;
    }
                      
    printf("start accel calibration process......\n");

    printf("Please place the (+x) axis of the aircraft upward\n");
    do
    {
        usleep(calibration_delay_us);
        imu_raw_msg.read(&_imu_raw);
        //printf("accel : time %lld : %f %f %f\n", _imu_raw.timestamp,_imu_raw.accel[0], _imu_raw.accel[1], _imu_raw.accel[2]);
        //printf("gyro : time %lld : %f %f %f\n", _imu_raw.timestamp,_imu_raw.gyro[0], _imu_raw.gyro[1], _imu_raw.gyro[2]);
    }
    while (!orientation_check(_imu_raw.accel,_imu_raw.gyro,  PX_UP));//¼ì²âµ½»úÉíÕý·Å
    printf("PX_UP is detected!\n");
    printf("accel data: accel x = %1.4f, y = %1.4f, z = %1.4f\n", _imu_raw.accel[0], _imu_raw.accel[1],_imu_raw.accel[2]);
    holding_time = get_time_now()/1e6;//»ñÈ¡µ±Ç°Ê±ÖÓtick  1ms/tick
    printf("Please do not move the aircraft\n");

    do
    {
        usleep(calibration_delay_us);
        imu_raw_msg.read(&_imu_raw);
        ellipsoid_method_step1( _imu_raw.accel[0], _imu_raw.accel[1],_imu_raw.accel[2], A);
        if(calibration_debug)fprintf(fp,"%f %f %f\n", _imu_raw.accel[0],_imu_raw.accel[1],_imu_raw.accel[2]);   
    }
    while (get_time_now()/1e6 - holding_time < 7); //¼ì²âµ½ÒÑ¾­¹ýÈ¥7000Ê±ÖÓtick 7s
    usleep(calibration_delay_us);
    printf("step1 is completed!\n");
    printf("A = %.4f\n", A[48]);
    printf("\n");



    printf("Please place the (+z) axis of the aircraft upward\n");
    do
    {
        usleep(calibration_delay_us);
        imu_raw_msg.read(&_imu_raw);
    }
    while (!orientation_check(_imu_raw.accel, _imu_raw.gyro, PZ_UP));//¼ì²âµ½»úÉí¶Ç³¯Ìì
    printf("PZ_UP is detected!\n");
    printf("accel data: accel x = %1.4f, y = %1.4f, z = %1.4f\n", _imu_raw.accel[0], _imu_raw.accel[1],_imu_raw.accel[2]);
    holding_time = get_time_now()/1e6;//»ñÈ¡µ±Ç°Ê±ÖÓtick  1ms/tick
    printf("Please do not move the aircraft\n");

    do
    {
        usleep(calibration_delay_us);
        imu_raw_msg.read(&_imu_raw);
        ellipsoid_method_step1( _imu_raw.accel[0], _imu_raw.accel[1],_imu_raw.accel[2], A);
        if(calibration_debug)fprintf(fp,"%f %f %f\n", _imu_raw.accel[0],_imu_raw.accel[1],_imu_raw.accel[2]);   

    }
    while (get_time_now()/1e6 - holding_time < 7); //¼ì²âµ½ÒÑ¾­¹ýÈ¥7000Ê±ÖÓtick 7s
        usleep(calibration_delay_us);
    printf("step2 is completed!\n");
    printf("A = %1.4f\n", A[48]);
    printf("\n");


     printf("Please place the (-x) axis of the aircraft upward\n");

    do
    {
        usleep(calibration_delay_us);
        imu_raw_msg.read(&_imu_raw);
    }
    while (!orientation_check(_imu_raw.accel,_imu_raw.gyro,  NX_UP));//¼ì²âµ½»úÉíroll=90deg
    printf("NX_UP is detected!\n");
    printf("accel data: accel x = %1.4f, y = %1.4f, z = %1.4f\n", _imu_raw.accel[0], _imu_raw.accel[1],_imu_raw.accel[2]);
    holding_time = get_time_now()/1e6;//»ñÈ¡µ±Ç°Ê±ÖÓtick  1ms/tick
    printf("Please do not move the aircraft\n");

    do
    {
        usleep(calibration_delay_us);
        imu_raw_msg.read(&_imu_raw);
        ellipsoid_method_step1( _imu_raw.accel[0], _imu_raw.accel[1],_imu_raw.accel[2], A);
        if(calibration_debug)fprintf(fp,"%f %f %f\n", _imu_raw.accel[0],_imu_raw.accel[1],_imu_raw.accel[2]);   

    }
    while (get_time_now()/1e6 - holding_time < 7); //¼ì²âµ½ÒÑ¾­¹ýÈ¥7000Ê±ÖÓtick 7s
        usleep(calibration_delay_us);
    printf("step3 is completed!\n");
    printf("A = %1.4f\n", A[48]);
    printf("\n");


    printf("Please place the (-z) axis of the aircraft upward\n");
    do
    {
        usleep(calibration_delay_us);
        imu_raw_msg.read(&_imu_raw);
    }
    while (!orientation_check(_imu_raw.accel,_imu_raw.gyro,  NZ_UP));//¼ì²âµ½»úÉíroll=-90deg
    printf("NZ_UP is detected!\n");
    printf("accel data: accel x = %1.4f, y = %1.4f, z = %1.4f\n", _imu_raw.accel[0], _imu_raw.accel[1],_imu_raw.accel[2]);
    holding_time = get_time_now()/1e6;//»ñÈ¡µ±Ç°Ê±ÖÓtick  1ms/tick
    printf("Please do not move the aircraft\n");

    do
    {
        usleep(calibration_delay_us);
        imu_raw_msg.read(&_imu_raw);
        ellipsoid_method_step1( _imu_raw.accel[0], _imu_raw.accel[1],_imu_raw.accel[2], A);
        if(calibration_debug)fprintf(fp,"%f %f %f\n", _imu_raw.accel[0],_imu_raw.accel[1],_imu_raw.accel[2]);   

    }
    while (get_time_now()/1e6 - holding_time < 7); //¼ì²âµ½ÒÑ¾­¹ýÈ¥7000Ê±ÖÓtick 7s
        usleep(calibration_delay_us);
    printf("step4 is completed!\n");
    printf("A = %1.4f\n", A[48]);
    printf("\n");


    printf("Please place the (-y) axis of the aircraft upward\n");
    do
    {
        usleep(calibration_delay_us);
        imu_raw_msg.read(&_imu_raw);
    }
    while (!orientation_check(_imu_raw.accel,_imu_raw.gyro, NY_UP));//¼ì²âµ½»úÉípitch = -90deg
    printf("NY_UP is detected!\n");
    printf("accel data: accel x = %1.4f, y = %1.4f, z = %1.4f\n", _imu_raw.accel[0], _imu_raw.accel[1],_imu_raw.accel[2]);
    holding_time = get_time_now()/1e6;//»ñÈ¡µ±Ç°Ê±ÖÓtick  1ms/tick
    printf("Please do not move the aircraft\n");

    do
    {
        usleep(calibration_delay_us);
        imu_raw_msg.read(&_imu_raw);
        ellipsoid_method_step1( _imu_raw.accel[0], _imu_raw.accel[1],_imu_raw.accel[2], A);
        if(calibration_debug)fprintf(fp,"%f %f %f\n", _imu_raw.accel[0],_imu_raw.accel[1],_imu_raw.accel[2]);   

    }
    while (get_time_now()/1e6 - holding_time < 7); //¼ì²âµ½ÒÑ¾­¹ýÈ¥7000Ê±ÖÓtick 7s
        usleep(calibration_delay_us);
    printf("step5 is completed!\n");
    printf("A = %1.4f\n", A[48]);
    printf("\n");


    printf("Please place the (+y) axis of the aircraft upward\n");
    do
    {
        usleep(calibration_delay_us);
        imu_raw_msg.read(&_imu_raw);
    }
    while (!orientation_check(_imu_raw.accel,_imu_raw.gyro,  PY_UP));//¼ì²âµ½»úÉípitch = 90deg
    printf("PY_UP is detected!\n");
    printf("accel data: accel x = %1.4f, y = %1.4f, z = %1.4f\n", _imu_raw.accel[0], _imu_raw.accel[1],_imu_raw.accel[2]);
    holding_time = get_time_now()/1e6;//»ñÈ¡µ±Ç°Ê±ÖÓtick  1ms/tick
    printf("Please do not move the aircraft\n");

    do
    {
        usleep(calibration_delay_us);
        imu_raw_msg.read(&_imu_raw);
        ellipsoid_method_step1( _imu_raw.accel[0], _imu_raw.accel[1],_imu_raw.accel[2], A);
        if(calibration_debug)fprintf(fp,"%f %f %f\n", _imu_raw.accel[0],_imu_raw.accel[1],_imu_raw.accel[2]);   

    }
    while (get_time_now()/1e6 - holding_time < 7); //¼ì²âµ½ÒÑ¾­¹ýÈ¥7000Ê±ÖÓtick 7s
        usleep(calibration_delay_us);
    printf("step6 is completed!\n");
    printf("A = %1.4f\n", A[48]);
    printf("\n");
    fclose(fp);

    ellipsoid_method_step2(A,
                           &calib_data.accel_scale[0],
                           &calib_data.accel_scale[1],
                           &calib_data.accel_scale[2],
                           &calib_data.accel_offset[0],
                           &calib_data.accel_offset[1],
                           &calib_data.accel_offset[2]);
    for(int i = 0; i < 3; i++)
    {
    	calib_data.accel_scale[i] = calib_data.accel_scale[i] * G;
    	calib_data.accel_offset[i] = calib_data.accel_offset[i] * G;
    }
    printf("%f %f %f \n%f %f %f\n",calib_data.accel_scale[0],calib_data.accel_scale[1],calib_data.accel_scale[2],
    	calib_data.accel_offset[0],calib_data.accel_offset[1],calib_data.accel_offset[2]);
    return true;

}
void calibration_typedef::apply_accel_calibration(float accel[3],float accel_calib[3])
{
    if(sensor_calib_enable)
    {
        for(int i = 0 ; i < 3; i++)
        {
            accel_calib[i] = calib_data.accel_scale[i] * accel[i] + calib_data.accel_offset[i];
        }
    }else{
        for(int i = 0 ; i < 3; i++)
        {
            accel_calib[i] = accel[i];
        }
    }

}