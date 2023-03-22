#include "sensor_calibration.h"
class calibration_typedef calibration("./calibration.txt");
static CONF *conf; //需要的数据结构

static void * thread_calibration(void * ptr)
{
  printf("thread calibration running\n");
  unsigned long long t0 = 0;
  unsigned long long t1 = 0;
  //get_hight_Pri(2);
  calibration.init();
  calibration.run_calibration();
  calibration.calibration_busy_flag = false;
  pthread_exit((void*)"calibration finished\n");

}

void start_calibration(char *inputs)
{
  int rc;
  pthread_t thr;
	calibration_mode_typedef _mode;
    if(!strcmp(inputs, "accel"))
    {
    	_mode = CAL_ACCEL;
    	printf("accel calibration mode\n");

    }else if(!strcmp(inputs, "gyro"))
    {
    	_mode = CAL_GYRO;
    	printf("gyro calibration mode\n");

    }else if(!strcmp(inputs, "mag"))
    {
    	_mode = CAL_MAG;
    	printf("mag calibration mode\n");

    }else if(!strcmp(inputs, "rc"))
    {
    	_mode = CAL_RC;
    	printf("rc calibration mode\n");

    }else if(!strcmp(inputs, "baro"))
    {
    	_mode = CAL_BARO;
    	printf("baro calibration mode\n");
    }else if(!strcmp(inputs, "save"))
    {
        _mode = SAVE;
        printf("SAVE calibration mode\n");

    }else if(!strcmp(inputs, "load"))
    {
        _mode = LOAD;
        printf("LOAD calibration mode\n");
    }else
    {
    	printf("undefined %s\n", inputs);
        printf("HELP: calib [options]\n[options]\ngyro : gyro calibration\naccel : accel calibration\nmag : mag calibration\nsave : save param to calibration.conf\nload : load param from calibration.conf\n");




    	return;
    }

    calibration.calib_mode = _mode;


  bool ret = create_thread("Calibration", thread_calibration, NULL);

  return;
}


calibration_typedef::calibration_typedef(const char * _file)
{
	calib_mode = CAL_DISABLE;
	calibration_busy_flag = false;
    filename = _file;
}



void calibration_typedef::init(void)
{
	calibration_busy_flag = true;
}


void calibration_typedef::run_calibration(void)
{

	switch(calib_mode)
	{
		case CAL_ACCEL:
			calib_accel(9.81);
		break;

		case CAL_GYRO:
            calib_gyro();
		break;

		case CAL_MAG:
            calib_mag(1);
		break;

		case CAL_BARO:

		break;

		case CAL_RC:

		break;

        case SAVE:
            write_config(calib_data);
        break;

        case LOAD:
            load_param(&calib_data);
        break;

		case CAL_DISABLE:
		default:
		break;
	}
    printf("calibration finished !\n");

}
void calibration_typedef::write_config(calib_data_typedef config)
{
    FILE *fp;
    char str[20];
    if( (fp=fopen(filename,"wt+")) == NULL ){
        printf("Fail to open file!\n");
        return ;
    }
    sprintf(str,"%f",config.accel_scale[0]);
    fprintf(fp,"accel_scale_0 = %s\n", str);   
    sprintf(str,"%f",config.accel_scale[1]);
    fprintf(fp,"accel_scale_1 = %s\n", str);   
    sprintf(str,"%f",config.accel_scale[2]);
    fprintf(fp,"accel_scale_2 = %s\n", str);   
    sprintf(str,"%f",config.accel_offset[0]);
    fprintf(fp,"accel_offset_0 = %s\n", str);   
    sprintf(str,"%f",config.accel_offset[1]);
    fprintf(fp,"accel_offset_1 = %s\n", str);   
    sprintf(str,"%f",config.accel_offset[2]);
    fprintf(fp,"accel_offset_2 = %s\n", str);        

    sprintf(str,"%f",config.gyro_scale[0]);
    fprintf(fp,"gyro_scale_0 = %s\n", str);   
    sprintf(str,"%f",config.gyro_scale[1]);
    fprintf(fp,"gyro_scale_1 = %s\n", str);   
    sprintf(str,"%f",config.gyro_scale[2]);
    fprintf(fp,"gyro_scale_2 = %s\n", str);   
    sprintf(str,"%f",config.gyro_offset[0]);
    fprintf(fp,"gyro_offset_0 = %s\n", str);   
    sprintf(str,"%f",config.gyro_offset[1]);
    fprintf(fp,"gyro_offset_1 = %s\n", str);   
    sprintf(str,"%f",config.gyro_offset[2]);
    fprintf(fp,"gyro_offset_2 = %s\n", str); 


    sprintf(str,"%f",config.mag_scale[0]);
    fprintf(fp,"mag_scale_0 = %s\n", str);   
    sprintf(str,"%f",config.mag_scale[1]);
    fprintf(fp,"mag_scale_1 = %s\n", str);   
    sprintf(str,"%f",config.mag_scale[2]);
    fprintf(fp,"mag_scale_2 = %s\n", str);   
    sprintf(str,"%f",config.mag_offset[0]);
    fprintf(fp,"mag_offset_0 = %s\n", str);   
    sprintf(str,"%f",config.mag_offset[1]);
    fprintf(fp,"mag_offset_1 = %s\n", str);   
    sprintf(str,"%f",config.mag_offset[2]);
    fprintf(fp,"mag_offset_2 = %s\n", str);                        
    fclose(fp);

}

bool calibration_typedef::orientation_check(float accel[3],float gyro[3],orientation_typedef _dir)
{
    static float _time = get_time_now() / 1e6;
    float check_time = 2;
    if(sqrtf(gyro[0]*gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]) > (10.f / 180 * M_PI))
    {
        _time = get_time_now() / 1e6;
        //printf("large angular velocity\n");// %f-%f = %f \n",  sqrtf(gyro[0]*gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]), (20.f / 180 * M_PI),sqrtf(gyro[0]*gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]),-(20 / 180 * M_PI));
        return false;
    }
    switch(_dir)
    {
        case NZ_UP:
        if(accel[2] < -8 && fabsf(accel[0]) < 2 && fabsf(accel[1]) < 2)
        {
            if((get_time_now()/1e6 - _time) > check_time)
            {
                _time = get_time_now() / 1e6;
                return true;
            }
        }else{
            _time = get_time_now() / 1e6;
        }
        break;

        case PZ_UP:
        if(accel[2] > 8 && fabsf(accel[0]) < 2 && fabsf(accel[1]) < 2)
        {
            if((get_time_now()/1e6 - _time) > check_time)
            {
                _time = get_time_now() / 1e6;
                return true;
            }
        }else{
            _time = get_time_now() / 1e6;
        }
        break;

        case NY_UP:
        if(accel[1] < -8 && fabsf(accel[0]) < 2 && fabsf(accel[2]) < 2)
        {
            if((get_time_now()/1e6 - _time) > check_time)
            {
                _time = get_time_now() / 1e6;
                return true;
            }
        }else{
            _time = get_time_now() / 1e6;
        }
        break;

        case PY_UP:
        if(accel[1] > 8 && fabsf(accel[0]) < 2 && fabsf(accel[2]) < 2)
        {
            if((get_time_now()/1e6 - _time) > check_time)
            {
                _time = get_time_now() / 1e6;
                return true;
            }
        }else{
            _time = get_time_now() / 1e6;
        }
        break;
        case NX_UP:
        if(accel[0] < -8 && fabsf(accel[2]) < 2 && fabsf(accel[1]) < 2)
        {
            if((get_time_now()/1e6 - _time) > check_time)
            {
                _time = get_time_now() / 1e6;
                return true;
            }
        }else{
            _time = get_time_now() / 1e6;
        }
        break;

        case PX_UP:
        if(accel[0] > 8 && fabsf(accel[2]) < 2 && fabsf(accel[1]) < 2)
        {
            if((get_time_now()/1e6 - _time) > check_time)
            {
                _time = get_time_now() / 1e6;
                return true;
            }
        }else{
            _time = get_time_now() / 1e6;
        }
        break;

        default:
        printf("undefined oriention\n");
        break;
    }

    return false;
}
static float get_param(const char *key)
{
  float _value;
  CONF_VALUE *value; //键/值参数数据结构

  value=conf_value_get(conf,key);
  _value = atof(value->value[0]);
  if(value)
    printf("%s is %f\n",key, _value);
  return _value;
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
void calibration_typedef::load_param(calib_data_typedef *config)
{
    static char **key; //获取所有键
    static CONF_VALUE **list; //获取所有键/值参数
    static int code; //返回的错误代码    
  //打开并初始化数据结构
  //出错时返回NULL
  if((conf=conf_open(filename)) == NULL)
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
  list=conf_value_get_all(conf);
  if(list)
    print_all(list);
    config->accel_scale[0] = get_param("accel_scale_0");
    config->accel_scale[1] = get_param("accel_scale_1");
    config->accel_scale[2] = get_param("accel_scale_2");
    config->accel_offset[0] = get_param("accel_offset_0");
    config->accel_offset[1] = get_param("accel_offset_1");
    config->accel_offset[2] = get_param("accel_offset_2");


    config->gyro_scale[0] = get_param("gyro_scale_0");
    config->gyro_scale[1] = get_param("gyro_scale_1");
    config->gyro_scale[2] = get_param("gyro_scale_2");
    config->gyro_offset[0] = get_param("gyro_offset_0");
    config->gyro_offset[1] = get_param("gyro_offset_1");
    config->gyro_offset[2] = get_param("gyro_offset_2");

    config->mag_scale[0] = get_param("mag_scale_0");
    config->mag_scale[1] = get_param("mag_scale_1");
    config->mag_scale[2] = get_param("mag_scale_2");
    config->mag_offset[0] = get_param("mag_offset_0");
    config->mag_offset[1] = get_param("mag_offset_1");
    config->mag_offset[2] = get_param("mag_offset_2");


    conf_free(conf);
}

void calibration_typedef::calibration_file_check_and_load(void)
{
    FILE *fp;
    if( (fp=fopen(filename,"r")) == NULL ){
        printf("The file: %s DOESN'T EXISIT !\n",filename);
        sensor_calib_enable = false;
        return;
    }else{
        load_param(&calib_data);
        printf("calibration data loads successfully\n");
        sensor_calib_enable = true;
    }

}

bool calibration_typedef::calib_baro(void)
{
	return true;
}
bool calibration_typedef::calib_rc(void)
{
	return true;
}