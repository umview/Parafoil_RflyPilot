#include "include.h"


int main(int argc, const char *argv[])
{
  (void)(argc);
  (void)(argv);
  get_time_now();// reset time counter begin from zero

  //read_param();
  calibration.calibration_file_check_and_load();

  usleep(500000);

  rflypilot_config_typedef _config_msg;
  rflypilot_config_msg.read(&_config_msg);


  _config_msg.validation_mode = SIH;
  start_console();
  switch(_config_msg.validation_mode)
  {
    case HIL:
      printf("mode : HIL\n");
      pca9685_dev.pca9685_init(400);
      start_icm20689();
      start_ist8310();
      start_baro();
      start_gps("/dev/ttySC0");
      start_sbus("/dev/ttyAMA0");
      usleep(500000);
      start_attitudeEstimator();
      start_lpe();
      usleep(500000);
      start_usrController();
      start_scope("192.168.199.152");  //start realtime scope (ip address configure is needed, pc scope : udp_recv.slx)
      start_log("/dev/shm");
    break;

    case EXP:
    printf("mode : EXP\n");
      pca9685_dev.pca9685_init(400);
      start_icm42688p();    
      start_baro();
      start_gps("/dev/ttyUSB0");
      start_qmc5883l();
      start_sbus("/dev/ttyAMA0");
      usleep(500000);
      start_attitudeEstimator();
      start_lpe();
      usleep(500000);
      start_usrController();
      start_scope("192.168.199.152");  //start realtime scope (ip address configure is needed, pc scope : udp_recv.slx)
      start_log("/dev/shm");
    break;

    case SIH:
      printf("mode : SIH\n");
      start_sih();
      start_sbus("/dev/ttyAMA0");
      usleep(500000);      
      start_usrController();

      start_scope("192.168.199.152");  //start realtime scope (ip address configure is needed, pc scope : udp_recv.slx)
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
//   scheduler.start_system_timer(2000);
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
    printf("hello world\n");
    sleep(1);
  }
  return 0;
}
