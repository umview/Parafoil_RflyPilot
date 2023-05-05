#include "parameter_read.h"


//config_typedef config;



static CONF *conf; //需要的数据结构
static CONF_VALUE *value; //键/值参数数据结构
static char **key; //获取所有键
static CONF_VALUE **list; //获取所有键/值参数
static int code; //返回的错误代码
static void print_key(char **key)
{
  int i=0;

  printf("all key is:\n");
  //遍历
  while(key[i] != NULL)
  {
    printf("key : %s\n",key[i]);
    ++i;
  }
}

static void print_all(CONF_VALUE **value)
{
  int i;
  int j;

  printf("all:\n");
  //遍历
  for(i=0;value[i] != NULL;++i)
  {
    j=0;
    printf("key:%s ",value[i]->key);

    while(value[i]->value[j] != NULL)
    {
      printf(" value:%s",value[i]->value[j]);
      ++j;
    }

    printf("\n");
  }
}
static void print_all_float(CONF_VALUE **value)
{
  int i;
  int j;

  printf("all:\n");
  //遍历
  for(i=0;value[i] != NULL;++i)
  {
    j=0;
    printf("key:%s ",value[i]->key);

    while(value[i]->value[j] != NULL)
    {
      printf(" value:%f",atof(value[i]->value[j]));
      ++j;
    }

    printf("\n");
  }
}
static float get_param(const char *key)
{
  float _value;
  value=conf_value_get(conf,key);
  _value = atof(value->value[0]);
  if(value)
    printf("%s is %f\n",key, _value);
  return _value;
}
void read_param(void)
{

  //打开并初始化数据结构
  //出错时返回NULL
  if((conf=conf_open("./rflypilot.txt")) == NULL)
    printf("err open\n");

  //开始解析配置文件
  //成功时返回0
  //出错时返回错误代码
  //错误代码可使用conf_error函数打印错误信息
  if((code=conf_parse(conf)) != 0)
  {
    conf_error(code);
    printf("%d\n",code);
  }
  // //得到所有键
  // //出错时返回NULL
  // key=conf_key_list(conf);
  // if(key)
  //   print_key(key);
  //printf("key has %d\n",conf_count(conf));
  //得到所有键/值参数
  //出错时返回NULL
  // list=conf_value_get_all(conf);
  // if(list)
  //   print_all_float(list);

  rflypilot_config_typedef _config_msg;
  _config_msg.timestamp = get_time_now();
  _config_msg.imu_rate = get_param("imu_rate");
  _config_msg.mag_rate = get_param("mag_rate");
  _config_msg.attitude_est_rate = get_param("attitude_est_rate");
  _config_msg.lpe_rate = get_param("lpe_rate");
  _config_msg.controller_rate = get_param("controller_rate");
  _config_msg.accel_cutoff_hz = get_param("accel_cutoff_hz");
  _config_msg.gyro_cutoff_hz = get_param("gyro_cutoff_hz");

  _config_msg.validation_mode = validation_mode_typedef((int)(get_param("valid_mode")));
  _config_msg.sih_use_real_state = (int)get_param("sih_use_real_state");
  _config_msg.scheduler_mode = scheduler_mode_typedef((int)(get_param("scheduler_mode")));

  _config_msg.sys_log_en = (int)get_param("sys_log_en");
  _config_msg.est_log_en = (int)get_param("est_log_en");
  _config_msg.ctrl_log_en = (int)get_param("ctrl_log_en");

  _config_msg.sys_scope_en = (int)(int)get_param("sys_scope_en");
  _config_msg.est_scope_en = (int)get_param("est_scope_en");
  _config_msg.ctrl_scope_en = (int)get_param("ctrl_scope_en");
  printf("#########################################\n");
  value=conf_value_get(conf,"scope_ip");
  strcpy(_config_msg.scope_ip, value->value[0]);
  printf("scope ip : %s\n",_config_msg.scope_ip);
  value=conf_value_get(conf,"station_ip");
  strcpy(_config_msg.station_ip, value->value[0]);
  printf("scope ip : %s\n",_config_msg.station_ip);

  value=conf_value_get(conf,"log_dir");
  strcpy(_config_msg.log_dir, value->value[0]);
  printf("log_dir  : %s\n",_config_msg.log_dir);

  rflypilot_config_msg.publish(&_config_msg);



  //释放内存
  conf_free(conf);
}