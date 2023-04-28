#include "include.h"


int main(int argc, const char *argv[])
{
  (void)(argc);
  (void)(argv);
  get_time_now();// reset time counter begin from zero




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
      pca9685_dev.pca9685_init(1500, false);
      start_icm20689();
      start_ist8310();
      start_baro();
      start_gps("/dev/ttySC0");
      usleep(500000);
      start_attitudeEstimator();
      start_lpe();
      usleep(500000);
      start_usrController();
      //start_basicController();
      start_log(NULL);
    break;

    case EXP:
    printf("mode : EXP\n");
      pca9685_dev.pca9685_init(1500, false);
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
      pca9685_dev.pca9685_init(1500, false);
      start_offboard();
    break;

    default:
      printf("undifined mode\n");
      return 0;
    break;
  }

   
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
