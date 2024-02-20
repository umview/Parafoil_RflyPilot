#include "include.h"

char time_buf[50]={0};

int main(int argc, const char *argv[])
{
  (void)(argc);
  (void)(argv);
  get_time_now();// reset time counter begin from zero
  get_compact_time_string(time(NULL), time_buf);   







  read_param();
  calibration.calibration_file_check_and_load();

  usleep(500000);

  rflypilot_config_typedef _config_msg;
  rflypilot_config_msg.read(&_config_msg);

  //_config_msg.validation_mode = SIH;
  start_console();


   start_sbus("/dev/ttyAMA0");
   //rc_check();
   start_system_app();

   start_scope(NULL);

  switch(_config_msg.validation_mode)
  {
    case HIL:
      printf("mode : HIL\n");
      pca9685_dev.pca9685_init(I2C_BUS_1,PWM_FREQ, false);
      // start_icm20689();
      start_icm20689_new();
      start_ist8310();
      start_baro();
      start_gps("/dev/ttySC0");
      usleep(500000);
      start_attitudeEstimator();
      start_lpe();
      usleep(500000);
      start_usrController();
      //start_basicController();
      // start_log(NULL);
      start_ulog(NULL);
    break;

    case EXP:
    printf("mode : EXP\n");
      pca9685_dev.pca9685_init(I2C_BUS_1,PWM_FREQ, true);
      pca9685_dev_aux.pca9685_init(I2C_BUS_0,333, true);

      start_icm42688p();    
      start_baro();
      start_gps("/dev/ttySC0");
      start_qmc5883l();
      usleep(500000);
      start_attitudeEstimator();
      start_lpe();
      usleep(500000);
      start_usrController();
      //start_basicController();
      start_log(NULL);
      //start_ulog(NULL);

      //start_mytask();
    break;

    case SIH:
      printf("mode : SIH\n");
      start_sih();
      usleep(500000);
      if(!(_config_msg.sih_use_real_state))
      {
        start_attitudeEstimator();
        start_lpe();
      }
      usleep(500000);      
      start_usrController();
    break;

    case OFFBOARD:
      printf("mode : OFFBOARD\n");
      pca9685_dev.pca9685_init(I2C_BUS_1,PWM_FREQ, true);
      pca9685_dev_aux.pca9685_init(I2C_BUS_0,SERVO_PWM_FREQ, true);

      start_offboard();
    break;

    default:
      printf("undifined mode\n");
      return 0;
    break;
  }

  sleep(4);
  start_screen();

   
//   if(CONSOLE_ENABLE)
//   {
//     start_console();
//   }
//scheduler.start_system_timer(TIMER_TICK_RATE);
//   printf("System Ready !!!!!!!!!!!!!!!\n");


// // DIR *d = opendir("/dev/shm");
// // if(d == NULL)
// // {
// //   printf("error opendir\n");
// // }

// // struct dirent* entry;
// // while((entry = readdir(d)) != NULL)
// // {
// //   printf("dir: %d\n", atoi(entry->d_name));
// // }
// // closedir(d);

  while(1)
  {
    //printf("hello world\n");
    sleep(1);
  }
  return 0;
}



/**********************************************/

class adaptive_delay_typedef adp_delay(0.5,15,0);//定义对象，kp=0.5，ki=15，offset=400us


void * thread_send(void * ptr)
{
  my_data_typedef send_my_data;
  uint32_t cnt = 0;
  for(;;)
  {
    send_my_data.data[0] = 1;
    send_my_data.data[1] = get_time_now()/1e6;
    my_data_msg.publish(&send_my_data);
    if(cnt++ == 300)
    {
      printf("send: %f , %f\n",send_my_data.data[0], send_my_data.data[1]);
      cnt = 0;
    }
    adp_delay.delay_us(1000);
  }
}

void * thread_recv(void * ptr)
{
  my_data_typedef recv_my_data;

  for(;;)
  {
    my_data_msg.read(&recv_my_data);
    printf("recv: %f , %f\n",recv_my_data.data[0], recv_my_data.data[1]);
    printf("send rate : %f \n", my_data_msg.publish_rate_hz);
    usleep(100000);
  }
}

int _main(int argc, const char *argv[])
{



bool ret = create_thread("thread_send", thread_send, NULL);
     ret = create_thread("thread_recv", thread_recv, NULL);
  while(1)
  {
    sleep(1);
  }

  return 0;

}
