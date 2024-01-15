#include "screen.h"

#define STRINGLENGTH "15"
// ANSI控制码 :
// \033[0m 关闭所有属性 
// \033[1m 设置高亮度
// \033[2m 设置白高亮度
// \033[3m 设置斜体 
// \03[4m 下划线 
// \033[5m 闪烁 
// \033[7m 反显 
// \033[8m 消隐 
// \033[30m -- \033[37m 设置前景色 
// \033[40m -- \033[47m 设置背景色 
// \033[nA 光标上移n行 
// \03[nB 光标下移n行 
// \033[nC 光标右移n行 
// \033[nD 光标左移n行 
// \033[y;xH设置光标位置 
// \033[2J 清屏 
// \033[K 清除从光标到行尾的内容 
// \033[s 保存光标位置 
// \033[u 恢复光标位置 
// \033[?25l 隐藏光标 
// \33[?25h 显示光标

// 背景色                        字体色
// 40: 黑                          30: 黑
// 41: 红                          31: 红
// 42: 绿                          32: 绿
// 43: 黄                          33: 黄
// 44: 蓝                          34: 蓝
// 45: 紫                          35: 紫
// 46: 深绿                      36: 深绿
// 47: 白色                      37: 白色
#define SCREENTITLE "\033[1;30;42m"

#define CAPTION "\033[1;32;40m"
#define LIGHT_PURPLE "\033[1;31;47m"
#define HIGHLIGHT_PURPLE "\033[2;31;47m"
#define NONE "\033[0m"

class screen Screen;

void *screen_task(void *arg)
{
  core_bind(SCREEN_CORE);
    //get_hight_Pri(1);
    //将当前光标往上移动1行
    // printf("\033[%dA",1);
    Screen.startscreen();
    Screen.outputscreen();
    printf("vehicle > ");
    //显示光标
    printf("\033[?25h");
    while (1)
    {
      printf("\033[s");
      Screen.outputscreen();
      printf("\033[u");

      usleep(1000*1000);
      
      // printf("\033[6n\n");//显示当前光标行和列
    }
    return 0;
}

void start_screen(void)
{
  bool ret = create_thread("screen", screen_task, NULL);
}

int screen::printline(uint64_t timestamp, float data1, float data2, float data3, const char *string1, const char *string2, const char *string3)
{
  // remove current line
  printf("\033[K");
  // print line
  printf(CAPTION "%9.9s:" NONE HIGHLIGHT_PURPLE "%10lld " NONE CAPTION "%10.10s:" NONE HIGHLIGHT_PURPLE "%+12.2f " NONE CAPTION "%10.10s:" NONE HIGHLIGHT_PURPLE "%+12.2f " NONE CAPTION "%10.10s:" NONE HIGHLIGHT_PURPLE "%+12.2f " NONE"\n", "Timestamp", timestamp, string1, data1, string2, data2, string3, data3);
  // add line num
  // this->lineMoveUp++;
  return 0;
}

int screen::endscreen(void)
{
  printf("\033[1;1H");
  //将当前光标往上移动lineMoveUp行
  // printf("\033[%dA",this->lineMoveUp);
  // this->lineMoveUp = 0;
}

int screen::startscreen(void)
{
  /* 换页清屏移动至1行1列 */
  printf("\f\033[2J\033[1;1H");
  /* 打印第一行 */
  // printf(SCREENTITLE "%-90.90s" NONE "\n","RflyPilot");
}

int screen::outputscreen(void)
{
  sbus_packet_t _rc_input_msg;//rc_input_msg
  gyro_typedef _gyro_msg;//gyro_msg
  accel_typedef _accel_msg;//accel_msg

  mag_typedef _mag_msg;//mag_msg
  baro_typedef _baro_msg;//baro_msg 
  gps_msg_typedef _gps_msg;//gps_msg

  cf_output_typedef _cf_msg;//cf_output_msg
  cf_status_typedef _cf_status_msg;//cf_status_msg

  lpe_output_typedef _lpe_msg;//lpe_output_msg
  lpe_status_typedef _lpe_status_msg;//lpe_status_msg

  //actuator_output_typedef will be used
  actuator_output_typedef _actuator_output_msg;//actuator_output_msg
  actuator_output_typedef _aux_actuator_output_msg;//actuator_output_msg
  actuator_output_typedef _control_output_msg;//control_output_msg

  rflypilot_config_typedef _config_msg;
  cf_output_msg.read(&_cf_msg);
  cf_status_msg.read(&_cf_status_msg);

  gyro_msg.read(&_gyro_msg);
  accel_msg.read(&_accel_msg);
  mag_msg.read(&_mag_msg);
  gps_msg.read(&_gps_msg);
  lpe_output_msg.read(&_lpe_msg);
  lpe_status_msg.read(&_lpe_status_msg);
  control_output_msg.read(&_control_output_msg);
  
  this->endscreen();
  this->printline(_lpe_msg.timestamp, _lpe_msg.pos_ned[0],_lpe_msg.pos_ned[1],_lpe_msg.pos_ned[2],"lpe px","lpe py","lpe pz");
  this->printline(_lpe_msg.timestamp, _lpe_msg.vel_ned[0],_lpe_msg.vel_ned[1],_lpe_msg.vel_ned[2],"lpe vx","lpe vy","lpe vz");
  this->printline(_lpe_status_msg.timestamp, _lpe_status_msg.baroBeta, _lpe_status_msg.baroInnov, _lpe_status_msg.baroAltRef, "baroBeta", "baroInnov", "baroAltRef");
  this->printline(_lpe_status_msg.timestamp, _lpe_status_msg.gpsBeta, _lpe_status_msg.gpsInnov[2], _lpe_status_msg.gpsAltRef, "gpsBeta", "gpsInnovZ", "gpsAltRef");

  this->printline(_cf_msg.timestamp, 57.3*_cf_msg.roll, 57.3*_cf_msg.pitch, 57.3*_cf_msg.yaw, "roll", "pitch","yaw");
//   this->printline(_cf_status_msg.timestamp, _cf_status_msg.magDelay*1000, _cf_status_msg.magDelayIndex, _cf_status_msg.magDelayStatus, "magDelayms","magDelayIndex","magDelayStatus");

  this->printline(_gyro_msg.timestamp, 57.3*_gyro_msg.gyro[0], 57.3*_gyro_msg.gyro[1], 57.3*_gyro_msg.gyro[2], "gyro x", "gyro y","gyro z");
  this->printline(_accel_msg.timestamp, _accel_msg.accel[0], _accel_msg.accel[1], _accel_msg.accel[2], "accel x", "accel y","accel z");
  this->printline(_mag_msg.timestamp, _mag_msg.mag[0], _mag_msg.mag[1], mag_msg.publish_rate_hz, "mag x", "mag y","magHz");
  this->printline(_gps_msg.timestamp, _gps_msg.hacc, _gps_msg.vacc, _gps_msg.sacc, "hacc", "vacc", "sacc");
  this->printline(_gps_msg.timestamp, _gps_msg.fixType, _gps_msg.numSV, _gps_msg.updated, "fixType", "numSV", "Updated");
  this->printline(_control_output_msg.timestamp, _control_output_msg.actuator_output[0], _control_output_msg.actuator_output[1], _control_output_msg.actuator_output[2], "PWM1", "PWM2", "PWM3");
  this->printline(get_time_now(), accel_msg.publish_rate_hz,cf_output_msg.publish_rate_hz, lpe_output_msg.publish_rate_hz, "imuRate", "attRate", "lpeRate");
  this->printline(get_time_now(), actuator_output_msg.publish_rate_hz, baro_msg.publish_rate_hz, gps_msg.publish_rate_hz, "ctrlRate", "baroRate","gpsRate");
  // this->printline(get_time_now(), accel_msg.publish_rate_hz,actuator_output_msg.publish_rate_hz, gps_msg.publish_rate_hz, "imuHz", "ctrlHz", "GPSHz");
  // this->printline(get_time_now(), cf_output_msg.publish_rate_hz, lpe_output_msg.publish_rate_hz, baro_msg.publish_rate_hz, "cfHz", "lpeHz","baroHz");

}
