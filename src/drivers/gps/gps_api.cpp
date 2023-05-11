#include "gps_api.h"

class gps_api_typedef gps_api;

void *gps_thread(void *ptr)
{
    //sbus_api.init((char *)ptr);
  gps_api.init((char *)ptr,B38400);
  if(gps_api.gps_config((char *)ptr))
  {
    printf("gps configure finished\n");
  }else{
    printf("gps configure error !!!!!!!!!!\n");
  }
  for(;;)
  {
    gps_api.run();
    usleep(2000);
  }
}
void start_gps(const char *gps_serial)
{

  bool ret = create_thread("gps", gps_thread, (void*)gps_serial);

}



gps_api_typedef::gps_api_typedef(void)
{
    _serial_fd = -1;
    gps_debug = 0;
}
void gps_api_typedef::init(char *_port, speed_t speed)
{
    //char *_port = "/dev/ttyUSB0";

    //set_device_bus(0);
    _serial_fd = open(_port, O_RDWR | O_NOCTTY);

    if (_serial_fd < 0) {
        printf("failed to open %s err: %d", _port, errno);
        //continue;
    }

    struct termios opt; //BASIC SETTING
    //清空串口接收缓冲区
    tcflush(_serial_fd, TCIOFLUSH);
    // 获取串口参数opt
    tcgetattr(_serial_fd, &opt);
    //设置串口输出波特率
    cfsetospeed(&opt, speed);
    //设置串口输入波特率
    cfsetispeed(&opt, speed);
    //设置数据位数 8位数据位
    opt.c_cflag &= ~CSIZE;
    opt.c_cflag |= CS8;
    //校验位 无校验位
    opt.c_cflag &= ~PARENB;
    opt.c_iflag &= ~INPCK;
    //设置停止位  1位停止位
    opt.c_cflag &= ~CSTOPB;
    opt.c_cflag |= CLOCAL | CREAD;
    opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    opt.c_oflag &= ~OPOST;
    opt.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
    //
    opt.c_cc[VTIME]=11;
    opt.c_cc[VMIN]=0;
    //更新配置
    tcsetattr(_serial_fd, TCSANOW, &opt);
    //第四部分代码/
    tcflush(_serial_fd,TCIOFLUSH);


    //printf("closed\n");
    //close(_serial_fd);
}
bool gps_api_typedef::gps_config(char *_port)
{

    ubx_payload_tx_cfg_prt_t cfg_prt[2];
    memset(cfg_prt, 0, 2*sizeof(ubx_payload_tx_cfg_prt_t));
    cfg_prt[0].portID       = UBX_TX_CFG_PRT_PORTID;
    cfg_prt[0].mode     = UBX_TX_CFG_PRT_MODE;
    cfg_prt[0].baudRate = 115200;
    cfg_prt[0].inProtoMask  = UBX_TX_CFG_PRT_PROTO_UBX | UBX_TX_CFG_PRT_PROTO_RTCM;
    cfg_prt[0].outProtoMask = UBX_TX_CFG_PRT_PROTO_UBX;
    cfg_prt[1].portID       = UBX_TX_CFG_PRT_PORTID_USB;
    cfg_prt[1].mode     = UBX_TX_CFG_PRT_MODE;
    cfg_prt[1].baudRate = 115200;
    cfg_prt[1].inProtoMask  = UBX_TX_CFG_PRT_PROTO_UBX | UBX_TX_CFG_PRT_PROTO_RTCM;
    cfg_prt[1].outProtoMask = UBX_TX_CFG_PRT_PROTO_UBX;
    printf("info: set port baudrate ...\n");
    sendMessage(UBX_MSG_CFG_PRT, (uint8_t *)cfg_prt,  2*sizeof(ubx_payload_tx_cfg_prt_t));
    printf("info: set port baudrate ok\n");
// 注意此处波特率修改后并没有接受ACK,后面重新配置后检查ACK
    usleep(200*1000);


    close(_serial_fd);
    init(_port,B115200);
    printf("info: check port baudrate ...\n");
    sendMessageACK(UBX_MSG_CFG_PRT, (uint8_t *)cfg_prt,  2*sizeof(ubx_payload_tx_cfg_prt_t));
    printf("info: check port baudrate ack ok\n");
    //ubx_payload_tx_cfg_rate_t cfg_rate;
    // cfg_rate.measRate = 1000;
    // cfg_rate.navRate = 0x01 << 8 | 0x00;
    // cfg_rate.timeRef = 0;
    // sendMessageACK(UBX_MSG_CFG_RATE, (uint8_t *)&cfg_rate, sizeof(cfg_rate));   
    // printf("change nav msg to 1Hz\n");



    ubx_payload_tx_cfg_rate_t cfg_rate;
    cfg_rate.measRate = 0xc8;
    cfg_rate.navRate = 0x01;
    cfg_rate.timeRef = 0;
    printf("info: gps rate config ... \n");
    sendMessageACK(UBX_MSG_CFG_RATE, (uint8_t *)&cfg_rate, sizeof(cfg_rate));   
    printf("info: gps rate config ok \n");


    ubx_payload_tx_cfg_nav5_t nav5_msg;
    nav5_msg.mask = 0x05;
    nav5_msg.dynModel = 0x06;//0x06 1g   0x07 2g
    nav5_msg.fixMode = 0x02;
    nav5_msg.fixedAlt = 0;
    nav5_msg.fixedAltVar = 0;
    nav5_msg.minElev = 0;
    nav5_msg.drLimit = 0;
    nav5_msg.pDop = 0;
    nav5_msg.tDop = 0;
    nav5_msg.pAcc = 0;
    nav5_msg.tAcc = 0;
    nav5_msg.staticHoldThresh = 0;
    nav5_msg.dgpsTimeOut = 0;
    nav5_msg.cnoThreshNumSVs = 0;        /**< (ubx7+ only, else 0) */
    nav5_msg.cnoThresh = 0;              /**< (ubx7+ only, else 0) */
    nav5_msg.reserved = 0;
    nav5_msg.staticHoldMaxDist = 0;      /**< (ubx8+ only, else 0) */
    nav5_msg.utcStandard = 0;            /**< (ubx8+ only, else 0) */
    nav5_msg.reserved3 = 0;
    nav5_msg.reserved4 = 0;
    printf("info: gps nav5 config ...\n");
    sendMessageACK(UBX_MSG_CFG_NAV5, (uint8_t *)&nav5_msg, sizeof(nav5_msg));   
    //usleep(200000);
    printf("info: gps nav5 config ok \n");

    ubx_payload_tx_cfg_msg_t cfg_msg; //0x01 0x04 
    cfg_msg.msg = UBX_MSG_NAV_DOP;
    cfg_msg.rate = 1;
    printf("info: gps dop config ... \n");
    sendMessageACK(UBX_MSG_CFG_MSG, (uint8_t *)&cfg_msg, sizeof(cfg_msg));   
    printf("info: gps dop config ok \n");

    cfg_msg.msg = UBX_MSG_NAV_PVT; // 0x01 0x07
    cfg_msg.rate = 1;
    printf("info: gps pvt config ... \n");
    sendMessageACK(UBX_MSG_CFG_MSG, (uint8_t *)&cfg_msg, sizeof(cfg_msg));
    printf("info: gps pvt config ok \n");

    cfg_msg.msg = UBX_MSG_NAV_SAT; // 0x01 0x35
    cfg_msg.rate = 0;
    printf("info: gps sat config ... \n");
    sendMessageACK(UBX_MSG_CFG_MSG, (uint8_t *)&cfg_msg, sizeof(cfg_msg));
    printf("info: gps sat config ok \n");

    cfg_msg.msg = UBX_MSG_MON_HW; // 0x0a 0x09
    cfg_msg.rate = 0;
    printf("info: gps hw config ... \n");
    sendMessageACK(UBX_MSG_CFG_MSG, (uint8_t *)&cfg_msg, sizeof(cfg_msg));
    printf("info: gps hw config ok \n");


    printf("init ok\n");
    printf("size of ubx_payload_rx_nav_pvt_t %d\n",sizeof(ubx_payload_rx_nav_pvt_t));
    return true;
}
void gps_api_typedef::run(void)
{
    //static bool initd = false;
    static uint8_t *p_frame_new = pvt_msg.frame.frame_new;//ping-pong alg
    static uint8_t *p_frame_old = pvt_msg.frame.frame_old;
    if(pollOrRead(p_frame_new, GPS_BUFFER_LENGTH))
    {

        if(gps_debug)printf(" data acquired\n");
        if(gps_msg_decode(pvt_msg.buffer, GPS_BUFFER_LENGTH))
        {

        }
        memcpy((void*)p_frame_old, (void*)p_frame_new, GPS_BUFFER_LENGTH);
    }
}
bool gps_api_typedef::gps_msg_decode(uint8_t *frame, int MSG_LENGHT)
{
    int i = 0;
    int frame_start_index = 0;
    for(i = (MSG_LENGHT*2-2); i >= 0; i--)
    {
        if(frame[i] == UBX_SYNC1 && frame[i+1] == UBX_SYNC2)// check frame header
        {
            if(gps_debug)printf("index found %d\n",i);
            if(packet_check(frame, i, MSG_LENGHT * 2))
            {
                frame_start_index = i;
                packet_decode((uint8_t*)(frame + frame_start_index));
            }
        }
    }
    if(i == 0)
    {
        printf("no complete frame found!!!!!!!!!!!!!!!!!!\n");
        return false;
    }
    if(gps_debug)printf(" Complete frame found\n");

    return true;

}
void gps_api_typedef::packet_decode(uint8_t *packet)
{
    uint8_t msg_class = packet[2];
    uint8_t msg_id = packet[3];
    uint8_t playload_length = packet[4] | packet[5] << 8;
    if(gps_debug)printf("msg class %x, id %x, playload length %d\n",
        msg_class,
        msg_id,
        playload_length);    
    switch(msg_class)
    {
        case UBX_CLASS_NAV:
            NAV_CLASS_decode(packet);
        break;

        case UBX_CLASS_ACK:
            ACK_CLASS_decode(packet);
        break;

        default:

            if(gps_debug)printf("ERROR : undefined class %x\n", msg_class);
        break;
    }
}
void gps_api_typedef::NAV_CLASS_decode(uint8_t *packet)
{
    uint8_t msg_id = packet[3];
    ubx_payload_rx_nav_dop_t *nav_dop;
    ubx_payload_rx_nav_pvt_t *nav_pvt;
    static int cnt = 0;
    switch(msg_id)
    {
        case UBX_ID_NAV_DOP:
            nav_dop = (ubx_payload_rx_nav_dop_t*)&packet[6];
        break;

        case UBX_ID_NAV_PVT:
            nav_pvt = (ubx_payload_rx_nav_pvt_t*)&packet[6];
            sensor_gps.timestamp = get_time_now();
            sensor_gps.gps_is_good = gps_pvt_check(nav_pvt);
            sensor_gps.ned_origin_valid = get_ned_origin(sensor_gps.ned_origin_valid, sensor_gps.gps_is_good, nav_pvt,
                                          &sensor_gps.lat_origin, &sensor_gps.lon_origin);
            sensor_gps.updated = sensor_gps.gps_is_good && sensor_gps.ned_origin_valid;
            sensor_gps.lon = nav_pvt->lon * 1e-7;
            sensor_gps.lat = nav_pvt->lat * 1e-7;
            sensor_gps.fixType = nav_pvt->fixType;
            sensor_gps.height = nav_pvt->hMSL * 1e-3;
            sensor_gps.vel_ned[0] = nav_pvt->velN * 1e-3;
            sensor_gps.vel_ned[1] = nav_pvt->velE * 1e-3;
            sensor_gps.vel_ned[2] = nav_pvt->velD * 1e-3;
            sensor_gps.hacc = nav_pvt->hAcc * 1e-3;
            sensor_gps.vacc = nav_pvt->vAcc * 1e-3;
            sensor_gps.sacc = nav_pvt->sAcc * 1e-3;
            sensor_gps.heading = nav_pvt->headMot * 1e-5 * M_PI / 180.f;
            sensor_gps.headacc = nav_pvt->headAcc * 1e-5 * M_PI / 180.f; 
            sensor_gps.numSV = nav_pvt->numSV;
            if(sensor_gps.ned_origin_valid)
            {
                LLA2NED(sensor_gps.lat_origin, sensor_gps.lon_origin,
                        sensor_gps.lat,sensor_gps.lon,
                        &sensor_gps.pos_ned[0], &sensor_gps.pos_ned[1]);
                sensor_gps.pos_ned[2] = - sensor_gps.height;
            }
            gps_msg.publish((gps_msg_typedef*)&sensor_gps);
            //printf("gps publish rate : %f\n", gps_msg.publish_rate_hz);
            if(gps_debug){
            printf("gps time stamp :%f\n", sensor_gps.timestamp/1e6);
            printf("cnt %d\n",cnt++);
            printf("time stamp %f\n",sensor_gps.timestamp / 1e6);
            printf("updated : %d , gps_is_good %d, ned_origin_valid %d\n", sensor_gps.updated, sensor_gps.gps_is_good, sensor_gps.ned_origin_valid);
            printf("lon %lf lat %lf alt %lf\n",sensor_gps.lon, sensor_gps.lat, sensor_gps.height);
            printf("heading :%f\n",sensor_gps.heading);
            printf("fixType : %d\n", sensor_gps.fixType);
            printf("hacc %f, vacc %f, sacc %f, headacc %f\n",
                sensor_gps.hacc,sensor_gps.vacc,sensor_gps.sacc,sensor_gps.headacc);
            printf("numSV %d\n",sensor_gps.numSV);
            printf("ned origin %lf %lf\n", sensor_gps.lon_origin, sensor_gps.lat_origin);
            printf("NED %f %f %f\n",sensor_gps.pos_ned[0],sensor_gps.pos_ned[1],sensor_gps.pos_ned[2]);
            }
        break;

        default:
            if(gps_debug)printf("ERROR : NAV_CLASS : undefined id %x\n",msg_id);

        break;
    }
}
void gps_api_typedef::ACK_CLASS_decode(uint8_t *packet)
{
    uint8_t msg_class = packet[2];
    uint8_t msg_id = packet[3];
    uint8_t playload_length = packet[4] | packet[5] << 8;
    ubx_payload_rx_ack_ack_t *ack;
    ubx_payload_rx_ack_nak_t *nack;
    switch(msg_id)
    {
        case UBX_ID_ACK_NAK:
            nack = (ubx_payload_rx_ack_nak_t*)&packet[6];
            ack_clsID = nack->clsID;
            ack_msgID = nack->msgID;
            printf("recv nack clsID %x msgID %x\n", ack_clsID, ack_msgID);
        break;

        case UBX_ID_ACK_ACK:
            ack = (ubx_payload_rx_ack_ack_t*)&packet[6];
            ack_clsID = ack->clsID;
            ack_msgID = ack->msgID;
            printf("recv ack clsID %x msgID %x\n", ack_clsID, ack_msgID);

        break;

        default:
        printf("gps ack class msg id error!  clsID %d \n", msg_id);
        break;
    }
}
bool gps_api_typedef::packet_check(uint8_t *frame, int index, int length)
{
    uint16_t playload_length = 0;
    // if((index + 1) > (length - 1))//sync 2
    // {

    // }else if((index + 2) > (length - 1))// class
    // {

    // }else if((index + 3) > (length - 1))// msg id
    // {

    // }else if((index + 4) > (length - 1))// length 1
    // {

    // }else 
    if((index + 5) > (length - 1))// length 2
    {
        if(gps_debug)printf(" length 2 wrong\n");
        return false;
    }

    playload_length = frame[index+4] | frame[index+5] << 8;
    
    if((index + 5 + playload_length + 2) > (length - 1))
    {
        if(gps_debug)printf("playload_length wrong\n");
        return false;
    }
    ubx_checksum_t checksum = {0, 0};    
    calcChecksum((uint8_t*)(frame + index + 2), 4 + playload_length, &checksum); // skip 2 sync bytes
    if(checksum.ck_a == frame[index + 5 + playload_length + 1] 
        && checksum.ck_b == frame[index + 5 + playload_length + 2])
    {
        return true;
    }else{
        printf("checksum failed\n");
        return false;
    }
}
bool gps_api_typedef::sendMessageACK(const uint16_t msg, const uint8_t *payload, const uint16_t length)
{
    uint32_t cnt = 0;
    ubx_header_t   header = {UBX_SYNC1, UBX_SYNC2, 0, 0};
    ubx_checksum_t checksum = {0, 0};

    // Populate header
    header.msg  = msg;
    header.length   = length;

    // Calculate checksum
 gps_config:   
    checksum.ck_a = 0;
    checksum.ck_b = 0;
    // calc Checksum step1
    calcChecksum(((uint8_t *)&header) + 2, sizeof(header) - 2, &checksum); // skip 2 sync bytes
    // calc Checksum step2
    if (payload != nullptr) {
        calcChecksum(payload, length, &checksum);
    }

    // Send message
    if (gps_write((uint8_t *)&header, sizeof(header)) != sizeof(header)) {
        printf("false 1\n");
        return false;
    }

    if (payload && gps_write((uint8_t *)payload, length) != length) {
        printf("false 2\n");
        return false;
    }

    if (gps_write((uint8_t *)&checksum, sizeof(checksum)) != sizeof(checksum)) {
        printf("false 3\n");
        return false;
    }
    usleep(200*1000);
  for(;;)//add ACK checker
  {
    run();
    usleep(20*1000);

    if((ack_clsID| ack_msgID << 8) == msg)
    {
        return true;
    }
    cnt ++;
    if(cnt * 0.02 > 1)//1s timeout
    {
        cnt = 0;
        printf("error ! \ngps config time out : \nexpected :%x %x\nrecv : %x %x\n",(uint8_t)(msg>>8),(uint8_t)((msg <<8)>>8),ack_msgID,ack_clsID);
        goto gps_config;
    }
  }

    return false;
}
bool gps_api_typedef::sendMessage(const uint16_t msg, const uint8_t *payload, const uint16_t length)
{
    uint32_t cnt = 0;
    ubx_header_t   header = {UBX_SYNC1, UBX_SYNC2, 0, 0};
    ubx_checksum_t checksum = {0, 0};

    // Populate header
    header.msg  = msg;
    header.length   = length;

    // Calculate checksum
    calcChecksum(((uint8_t *)&header) + 2, sizeof(header) - 2, &checksum); // skip 2 sync bytes

    if (payload != nullptr) {
        calcChecksum(payload, length, &checksum);
    }

    // Send message
    if (gps_write((uint8_t *)&header, sizeof(header)) != sizeof(header)) {
        printf("false 1\n");
        return false;
    }

    if (payload && gps_write((uint8_t *)payload, length) != length) {
        printf("false 2\n");
        return false;
    }

    if (gps_write((uint8_t *)&checksum, sizeof(checksum)) != sizeof(checksum)) {
        printf("false 3\n");
        return false;
    }

  // for(;;)//add ACK checker
  // {
  //   run();
  //   usleep(2000);
  //   if((ack_clsID << 8 | ack_msgID) == msg)
  //   {
  //       return true;
  //   }
  //   cnt ++;
  //   if(cnt * 0.002 > 1)
  //   {
  //       cnt = 0;
  //       printf("gps config time out : %d\n",msg);
  //       goto gps_config;
  //   }
  // }
    usleep(200000);
    return true;
}
int gps_api_typedef::gps_write(uint8_t *buf, int buf_length)
{
    for(int i = 0; i < buf_length; i++)
    {
        printf("%x ",buf[i]);

    }
    printf("\n");
    return write(_serial_fd, buf, buf_length);
}

void gps_api_typedef::calcChecksum(const uint8_t *buffer, const uint16_t length, ubx_checksum_t *checksum)
{
    for (uint16_t i = 0; i < length; i++) {
        checksum->ck_a = checksum->ck_a + buffer[i];
        checksum->ck_b = checksum->ck_b + checksum->ck_a;
    }
    // printf("calcChecksum is: %X and %X\n", checksum->ck_a,checksum->ck_b);
}
bool gps_api_typedef::pollOrRead(uint8_t * buff, int buf_length)
{

    int err = 0, ret = 0;
    int bytes_available = 0;
    err = ::ioctl(_serial_fd, FIONREAD, (unsigned long)&bytes_available);
    //printf("data remain : %d\n", bytes_available);
    if (err != 0 || bytes_available < (int)buf_length) {
        return false;
    }else{
        ret = ::read(_serial_fd, buff, buf_length);
        if (ret != buf_length) {
            printf("ret != buf_length %d\n", ret);
            return false;
        }else
        return true;
    }

}
bool gps_api_typedef::get_ned_origin(bool _ned_origin_valid, bool gps_is_good, ubx_payload_rx_nav_pvt_t *pvt,
                                    double *_lat_origin, double *_lon_origin)
{
    static int cnt = 0;
    #define gps_record_num 6
    static double lat_record = 0;
    static double lon_record = 0;
    if(_ned_origin_valid && gps_is_good)
    {
        return true;
    }else{
        if(gps_is_good)
        {
            lat_record += pvt->lat * 1e-7;
            lon_record += pvt->lon * 1e-7;
            cnt ++;
            if(cnt == gps_record_num)
            {
                *_lat_origin = lat_record / gps_record_num;
                *_lon_origin = lon_record / gps_record_num;
                cnt = 0;
                lat_record = 0;
                lon_record = 0;
                return true;
            }else{
                return false;
            }
        }else{
            cnt = 0;
            lat_record = 0;
            lon_record = 0;
            return false;
        } 
    }
}
bool gps_api_typedef::gps_pvt_check(ubx_payload_rx_nav_pvt_t *pvt)
{

    if(pvt->fixType >= 3 && pvt->numSV >= 7)
    {
        if(pvt->hAcc*1e-3 < 5.5 && pvt->vAcc*1e-3 < 5)
        {
            return true;
        }return false;
    }else{
        return false;
    }
}
static double my_radians(double deg)
{
    return deg / 180 * M_PI;
}
static double constrain(double x, double min, double max)
{
    if(x <= min)
    {
        return min;
    }else if(x >= max)
    {
        return max;
    }else{
        return x;
    }
}

template<typename T>
bool gps_api_typedef::cfgValset(uint32_t key_id, T value, int &msg_size)
{
    if (msg_size + sizeof(key_id) + sizeof(value) > sizeof(_buf)) {
        // If this happens use several CFG-VALSET messages instead of one
        printf("buf for CFG_VALSET too small\n");
        return false;
    }

    uint8_t *buffer = (uint8_t *)&_buf.payload_tx_cfg_valset;
    memcpy(buffer + msg_size, &key_id, sizeof(key_id));
    msg_size += sizeof(key_id);
    memcpy(buffer + msg_size, &value, sizeof(value));
    msg_size += sizeof(value);
    return true;
}
int gps_api_typedef::initCfgValset(void)
{
    memset(&_buf.payload_tx_cfg_valset, 0, sizeof(_buf.payload_tx_cfg_valset));
    _buf.payload_tx_cfg_valset.layers = UBX_CFG_LAYER_RAM;
    return sizeof(_buf.payload_tx_cfg_valset) - sizeof(_buf.payload_tx_cfg_valset.cfgData);
}


void LLA2NED(double ref_lat, double ref_lon, double lat, double lon, float *x, float *y)
{
    const double ref_lat_rad = my_radians(ref_lat);
    const double ref_lon_rad = my_radians(ref_lon);
    const double ref_sin_lat = sin(ref_lat_rad);
    const double ref_cos_lat = cos(ref_lat_rad);

    const double lat_rad = my_radians(lat);
    const double lon_rad = my_radians(lon);


    const double sin_lat = sin(lat_rad);
    const double cos_lat = cos(lat_rad);

    const double cos_d_lon = cos(lon_rad - ref_lon_rad);

    const double arg = constrain(ref_sin_lat * sin_lat + ref_cos_lat * cos_lat * cos_d_lon, -1.0,  1.0);
    const double c = acos(arg);

    double k = 1.0;

    if (fabs(c) > 0) {
        k = (c / sin(c));
    }

    *x = (float)(k * (ref_cos_lat * sin_lat - ref_sin_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH);
    *y = (float)(k * cos_lat * sin(lon_rad - ref_lon_rad) * CONSTANTS_RADIUS_OF_EARTH);
}
