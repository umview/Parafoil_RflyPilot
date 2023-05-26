#include "console.h"

class console_class_typedef console;
void *console_task(void *arg)
{
    //get_hight_Pri(1);
    console.run();
    return 0;
}

void start_console(void)
{

  bool ret = create_thread("console", console_task, NULL);

}

void console_class_typedef::run(void)
{
    char input_string[MAX_SIZE];

    int attr = 1;/* 清除非阻塞标志 */
    ioctl(STDIN_FILENO, FIONBIO, &attr);
    // ioctl(STDERR_FILENO, FIONBIO, &attr);
    
    for(;;){    
        usleep(20*1000);
        if(!read_input(input_string)) {
            continue;
        }else{
            // fflush(stdin);
            int len = strlen(input_string);
            // printf("read len is %d\n", len);
            // printf("input string is %s\n", input_string);
            input_string[len-1]='\0';
            if(exec_command(input_string)) break;
            printf("vehicle > ");
        }
    }
}
int console_class_typedef::exec_command(char* user_input)
{
    char* inputs[MAX_SIZE];
    bzero(inputs, MAX_SIZE); 
    char* token = strtok(user_input, " ");
    int cnt=0;
    while (token != NULL) {
        inputs[cnt] = token;
        cnt++;
        token = strtok(NULL, " "); 
    } 
    if(strcmp(inputs[0], "exit")==0){
        printf("Bye.\nCtrl-C to kill\n");
        //DEV_HARDWARE_SPI_end();
        tcflush(gps_api._serial_fd, TCIOFLUSH);
        exit(0);
        return 1;
    }
    if(strcmp(inputs[0], "status")==0){

        return 0;
    }
    if(!strcmp(inputs[0], "disarm"))
    {
        //disarm();
        printf("UAV shutdown !\n");
        return 0;
    }
    if(!strcmp(inputs[0], "arm"))
    {
        //arm();
        printf("UAV engine on !\n");
        return 0;
    } 

    if(!strcmp(inputs[0], "calib"))
    {
        if(cnt != 2)
        {
            printf(" calib [options] !!!!!!\n");
            return 0;
        }
        // calibration during arming is NOT ALLOWED
        // if(fc_api.output_enable)
        // {
        //     printf("ERROR: ARMED!!!!!!\n");
        //     printf("Sensor Calibration Disabled\n");
        //     return 0;
        // }
        start_calibration(inputs[1]);
        //calibration.entry(inputs[1]);


        return 0;
    }

    if(!strcmp(inputs[0], "setpwm"))
    {
        if(cnt == 2)
        {
            float testpwm = (float)(atof(inputs[1]));
            printf("setpwm as %f!\n",testpwm);
            float pwm_output[8] = {testpwm,testpwm,testpwm,testpwm,testpwm,testpwm,testpwm,testpwm};
            pca9685_dev.updatePWM(pwm_output,8);
        }
        
        return 0;
    }                      

    return 0;
}

