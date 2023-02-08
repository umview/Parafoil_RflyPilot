#include "include.h"


int main(int argc, const char *argv[])
{
  (void)(argc);
  (void)(argv);
  get_time_now();
  // start_sbus("/dev/ttyAMA0");
  // start_gps("/dev/ttySC0");

  // start_baro();
  // start_ist8310();
  // start_qmc5883l();
  // start_icm42688p();
  // start_icm20689();
  // pca9685_dev.pca9685_init(400);
  start_attitudeEstimator();
  start_lpe();
  start_usrController();
  start_sih();
  start_scope();
  start_console();
  //printf("sizeof(uint64_t) %d\n", sizeof(uint64_t));
  //printf("sizeof(unsigned long long) %d\n", sizeof(unsigned long long));
//   get_time_now();

//   read_param();// read param from nmpc.conf
//   calibration.calibration_file_check_and_load();
//   fc_api.init();
//   switch(fc_api.valid_mode)
//   {
//     case HIL:
//       usleep(500000);
//       start_attitudeEstimator();
//       start_lpe();
//       usleep(500000);

//       start_state_machine();
//       start_pid_controller();
//       usleep(500000);

//       start_nmpc();
//       start_scope();  //start realtime scope (ip address configure is needed, pc scope : udp_recv.slx)
//       start_actuator();
//     break;

//     case EXP:
//       usleep(500000);
//       start_attitudeEstimator();
//       start_lpe();
//       usleep(500000);

//       start_state_machine();
//       start_pid_controller();
//       usleep(500000);

//       start_nmpc();
//       start_scope();  //start realtime scope (ip address configure is needed, pc scope : udp_recv.slx)
//       start_actuator();
//     break;

//     case HIL2:
//       // pca9685_dev.pca9685_init(400);
//       start_icm20689();
//       start_ist8310();
//       start_baro();
//       start_gps("/dev/ttySC0");
//       start_sbus("/dev/ttyAMA0");

//       usleep(500000);
//       start_attitudeEstimator();
//       start_lpe();
//       usleep(500000);

//       start_state_machine();
//       start_pid_controller();
//       usleep(500000);

//       // start_nmpc();
//       start_usrController();
//       start_scope();  //start realtime scope (ip address configure is needed, pc scope : udp_recv.slx)
//       start_actuator();
//     break;

//     case EXP2:
//       start_icm42688p();    
//       start_baro();
//       start_gps("/dev/ttyUSB0");
//       start_qmc5883l();
//       start_sbus("/dev/ttyAMA0");

//       usleep(500000);
//       start_attitudeEstimator();
//       start_lpe();
//       usleep(500000);

//       start_state_machine();
//       start_pid_controller();
//       usleep(500000);

//       start_nmpc();
//       start_scope();  //start realtime scope (ip address configure is needed, pc scope : udp_recv.slx)
//       start_actuator();
//     break;

//     case SIH:
//       start_sih();
//       start_sbus("/dev/ttyAMA0");
//       start_usrController();

//       // usleep(500000);
//       // start_attitudeEstimator();
//       // start_lpe();
//       // usleep(500000);

//       start_state_machine();
//       // start_pid_controller();
//       usleep(500000);

//       // start_nmpc();
//       start_scope();  //start realtime scope (ip address configure is needed, pc scope : udp_recv.slx)
//       start_actuator(); // start nmpc controller
//     break;

//     default:
//       printf("undifined mode\n");
//       return 0;
//     break;
//   }

   
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
