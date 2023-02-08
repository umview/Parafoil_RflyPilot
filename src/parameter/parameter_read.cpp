#include "parameter_read.h"


config_typedef config;



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
  if((conf=conf_open("../nmpc.conf")) == NULL)
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
  printf("key has %d\n",conf_count(conf));
  //得到所有键/值参数
  //出错时返回NULL
  // list=conf_value_get_all(conf);
  // if(list)
  //   print_all_float(list);


  config.imu_rate = get_param("imu_rate");
  config.pid_controller_rate = get_param("pid_controller_rate");
  config.mpc_rate = get_param("mpc_rate");
  config.actuator_rate = get_param("actuator_rate");
  config.mag_rate = get_param("mag_rate");
  config.attitude_est_rate = get_param("attitude_est_rate");
  config.lpe_rate = get_param("lpe_rate");
  config.accel_cutoff = get_param("accel_cutoff");
  config.gyro_cutoff = get_param("gyro_cutoff");



  //释放内存
  conf_free(conf);
}