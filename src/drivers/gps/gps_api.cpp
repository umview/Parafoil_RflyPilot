#include "gps_api.h"

class gps_api_typedef gps_api;

void *gps_thread(void *ptr)
{
    //sbus_api.init((char *)ptr);
  gps_api.init((char *)ptr, B230400);
  if(gps_api.gps_config((char *)ptr))
  {
    printf("info: gps configure success\n");
  }else{
    printf("warning: gps configure fail !\n");
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
    _configured = false;
    ubx_payload_tx_cfg_prt_t cfg_prt[2];

    // reset
    _resetType = GPSRestartType::Cold;
    // if(!reset(_resetType))
    // {
    //     printf("info: Reset complete\n");
    //     usleep(1000*1000);
    // }else{
    //     printf("error: Reset complete\n");
    // }
    unsigned desired_baudrate = 115200;
    const unsigned baudrates[] = {38400, 57600, 9600, 115200, 230400, 460800, 921600};
    unsigned baud_i;
    int cfg_valset_msg_size;

    // 无论波特率是多少， 逐次发送波特率设置消息， 目标波特率是desired_baudrate
    for (baud_i = 0; baud_i < sizeof(baudrates) / sizeof(baudrates[0]); baud_i++)
    {
        close(_serial_fd);
        switch (baudrates[baud_i])
        {
            case 38400U:
                init(_port,B38400);
                break;

            case 57600U:
                init(_port,B57600);
                break;

            case 9600U:
                init(_port,B9600);
                break;
                
            case 115200U:
                init(_port,B115200);
                break;

            case 230400U:
                init(_port,B230400);
                break;
                
            case 460800U:
                init(_port,B460800);
                break;

            case 921600U:
                init(_port,B921600);
                break;
            
            default:
                break;
        }
        usleep(40*1000);
        
        // Use valset protocol set boundrate
        cfg_valset_msg_size = initCfgValset();
        cfgValset<uint32_t>(UBX_CFG_KEY_CFG_UART1_BAUDRATE, desired_baudrate, cfg_valset_msg_size);

        if (!sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
            return false;
        }

        //use cfg protocol
        memset(cfg_prt, 0, 2*sizeof(ubx_payload_tx_cfg_prt_t));
        cfg_prt[0].portID       = UBX_TX_CFG_PRT_PORTID;
        cfg_prt[0].mode     = UBX_TX_CFG_PRT_MODE;
        cfg_prt[0].baudRate = desired_baudrate;
        cfg_prt[0].inProtoMask  = UBX_TX_CFG_PRT_PROTO_UBX | UBX_TX_CFG_PRT_PROTO_RTCM;
        cfg_prt[0].outProtoMask = UBX_TX_CFG_PRT_PROTO_UBX;
        cfg_prt[1].portID       = UBX_TX_CFG_PRT_PORTID_USB;
        cfg_prt[1].mode     = UBX_TX_CFG_PRT_MODE;
        cfg_prt[1].baudRate = desired_baudrate;
        cfg_prt[1].inProtoMask  = UBX_TX_CFG_PRT_PROTO_UBX | UBX_TX_CFG_PRT_PROTO_RTCM;
        cfg_prt[1].outProtoMask = UBX_TX_CFG_PRT_PROTO_UBX;
        
        printf("info: set port baudrate as %d\n", desired_baudrate);
        sendMessage(UBX_MSG_CFG_PRT, (uint8_t *)cfg_prt,  2*sizeof(ubx_payload_tx_cfg_prt_t));
        

        usleep(200*1000);
    }

    // 正式开始配置GPS
    // set CM4 UART bandrate as "desired_baudrate"
    close(_serial_fd);
    switch (desired_baudrate)
    {
        case 38400U:
            init(_port,B38400);
            break;

        case 57600U:
            init(_port,B57600);
            break;

        case 9600U:
            init(_port,B9600);
            break;
            
        case 115200U:
            init(_port,B115200);
            break;

        case 230400U:
            init(_port,B230400);
            break;
            
        case 460800U:
            init(_port,B460800);
            break;

        case 921600U:
            init(_port,B921600);
            break;
        
        default:
            break;
    }
    usleep(40*1000);

    // try CFG-VALSET: if we get an ACK we know we can use protocol version 27+
    cfg_valset_msg_size = initCfgValset();
    // UART1
    cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1_STOPBITS, 1, cfg_valset_msg_size);
    cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1_DATABITS, 0, cfg_valset_msg_size);
    cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1_PARITY, 0, cfg_valset_msg_size);
    cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1INPROT_UBX, 1, cfg_valset_msg_size);
    cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1INPROT_RTCM3X, 1, cfg_valset_msg_size);
    cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1INPROT_NMEA, 0, cfg_valset_msg_size);
    cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1OUTPROT_UBX, 1, cfg_valset_msg_size);

    // if (_output_mode != OutputMode::GPS) {
    //     cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1OUTPROT_RTCM3X, 1, cfg_valset_msg_size);
    // }

    cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1OUTPROT_NMEA, 0, cfg_valset_msg_size);

    // USB
    cfgValset<uint8_t>(UBX_CFG_KEY_CFG_USBINPROT_UBX, 1, cfg_valset_msg_size);
    cfgValset<uint8_t>(UBX_CFG_KEY_CFG_USBINPROT_RTCM3X, 1, cfg_valset_msg_size);
    cfgValset<uint8_t>(UBX_CFG_KEY_CFG_USBINPROT_NMEA, 0, cfg_valset_msg_size);
    cfgValset<uint8_t>(UBX_CFG_KEY_CFG_USBOUTPROT_UBX, 1, cfg_valset_msg_size);

    // if (_output_mode != OutputMode::GPS) {
    //     cfgValset<uint8_t>(UBX_CFG_KEY_CFG_USBOUTPROT_RTCM3X, 1, cfg_valset_msg_size);
    // }

    cfgValset<uint8_t>(UBX_CFG_KEY_CFG_USBOUTPROT_NMEA, 0, cfg_valset_msg_size);
    
    bool cfg_valset_success = false;
    if (sendMessageACK(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
        if (_ack_state == UBX_ACK_GOT_ACK)
        {
            cfg_valset_success = true;
            printf("info: protocol version 27+ (new)\n");
            _ack_state = UBX_ACK_IDLE;
        }
        if (_ack_state == UBX_ACK_GOT_NAK)
        {
            cfg_valset_success = false;
            printf("info: protocol version pre27 (old)\n");
            _ack_state = UBX_ACK_IDLE;
        }
    }

    if (cfg_valset_success)
    {
        _proto_ver_27_or_higher = true;

        // Now we only have to change the baudrate
        cfg_valset_msg_size = initCfgValset();
        cfgValset<uint32_t>(UBX_CFG_KEY_CFG_UART1_BAUDRATE, desired_baudrate, cfg_valset_msg_size);

        printf("info: check port baudrate ...\n");
        if (!sendMessageACK(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
            // continue;
            return false;
        }
        printf("info: check port baudrate ack ok\n");

        // /* no ACK is expected here, but read the buffer anyway in case we actually get an ACK */
        // waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, false);

        // close(_serial_fd);
        // init(_port,B115200);

    }else{
        _proto_ver_27_or_higher = false;

        // memset(cfg_prt, 0, 2*sizeof(ubx_payload_tx_cfg_prt_t));
        // cfg_prt[0].portID       = UBX_TX_CFG_PRT_PORTID;
        // cfg_prt[0].mode     = UBX_TX_CFG_PRT_MODE;
        // cfg_prt[0].baudRate = 115200;
        // cfg_prt[0].inProtoMask  = UBX_TX_CFG_PRT_PROTO_UBX | UBX_TX_CFG_PRT_PROTO_RTCM;
        // cfg_prt[0].outProtoMask = UBX_TX_CFG_PRT_PROTO_UBX;
        // cfg_prt[1].portID       = UBX_TX_CFG_PRT_PORTID_USB;
        // cfg_prt[1].mode     = UBX_TX_CFG_PRT_MODE;
        // cfg_prt[1].baudRate = 115200;
        // cfg_prt[1].inProtoMask  = UBX_TX_CFG_PRT_PROTO_UBX | UBX_TX_CFG_PRT_PROTO_RTCM;
        // cfg_prt[1].outProtoMask = UBX_TX_CFG_PRT_PROTO_UBX;
        
        // printf("info: set port baudrate ...\n");
        // sendMessage(UBX_MSG_CFG_PRT, (uint8_t *)cfg_prt,  2*sizeof(ubx_payload_tx_cfg_prt_t));
        // printf("info: set port baudrate ok\n");
        
        // // 注意此处波特率修改后并没有接受ACK,后面重新配置后检查ACK
        // usleep(200*1000);

        // close(_serial_fd);
        // init(_port,B115200);
        
        printf("info: check port baudrate ...\n");
        sendMessageACK(UBX_MSG_CFG_PRT, (uint8_t *)cfg_prt,  2*sizeof(ubx_payload_tx_cfg_prt_t));
        printf("info: check port baudrate ack ok\n");
    }

    int ret;

	if (_proto_ver_27_or_higher) {
		ret = configureDevice();

	} else {
		ret = configureDevicePreV27();
	}

	if (ret != 0) {
		return false;
	}
    //ubx_payload_tx_cfg_rate_t cfg_rate;
    // cfg_rate.measRate = 1000;
    // cfg_rate.navRate = 0x01 << 8 | 0x00;
    // cfg_rate.timeRef = 0;
    // sendMessageACK(UBX_MSG_CFG_RATE, (uint8_t *)&cfg_rate, sizeof(cfg_rate));   
    // printf("change nav msg to 1Hz\n");



    printf("info: gps init ok\n");
    printf("size of ubx_payload_rx_nav_pvt_t %d\n",sizeof(ubx_payload_rx_nav_pvt_t));
    _configured = true;
    return true;
}

int gps_api_typedef::configureDevicePreV27()
{
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

    ubx_payload_tx_cfg_msg_t cfg_msg; 
    
    cfg_msg.msg = UBX_MSG_NAV_PVT; // 0x01 0x07
    cfg_msg.rate = 1;
    printf("info: gps pvt config ... \n");
    sendMessageACK(UBX_MSG_CFG_MSG, (uint8_t *)&cfg_msg, sizeof(cfg_msg));
    printf("info: gps pvt config ok \n");

    
    cfg_msg.msg = UBX_MSG_NAV_DOP; //0x01 0x04 
    cfg_msg.rate = 5;
    printf("info: gps dop config ... \n");
    sendMessageACK(UBX_MSG_CFG_MSG, (uint8_t *)&cfg_msg, sizeof(cfg_msg));   
    printf("info: gps dop config ok \n");

    cfg_msg.msg = UBX_MSG_NAV_SAT; // 0x01 0x35
    cfg_msg.rate = 0;
    printf("info: gps sat config ... \n");
    sendMessageACK(UBX_MSG_CFG_MSG, (uint8_t *)&cfg_msg, sizeof(cfg_msg));
    printf("info: gps sat config ok \n");

    cfg_msg.msg = UBX_MSG_MON_HW; // 0x0a 0x09
    cfg_msg.rate = 5;
    printf("info: gps hw config ... \n");
    sendMessageACK(UBX_MSG_CFG_MSG, (uint8_t *)&cfg_msg, sizeof(cfg_msg));
    printf("info: gps hw config ok \n");

    return 0;
}

void gps_api_typedef::run(void)
{
    //static bool initd = false;
    static uint8_t *p_frame_new = pvt_msg.frame.frame_new;//ping-pong alg
    static uint8_t *p_frame_old = pvt_msg.frame.frame_old;
    if(pollOrRead(p_frame_new, GPS_BUFFER_LENGTH))
    {
        //if read success, update read time
        last_time_read = get_time_now();

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
int gps_api_typedef::disableMsg(uint8_t msg_class, uint8_t msg_id)
{
    if(_configured)
    {
        printf("ERROR : undefined class 0x%02x, id 0x%02x\n", msg_class, msg_id);
        if(_proto_ver_27_or_higher){//NEW
            uint32_t key_id = 0;
            if(msg_class == 0x01 && msg_id == 0x20)
            {   
                key_id = 0x20910047; //CFG-MSGOUT-UBX_NAV_TIMEGPS_I2C
                int cfg_valset_msg_size = initCfgValset();
                cfgValsetPort(key_id, 0, cfg_valset_msg_size);
                printf("warning: disable unexpected messages \n");
                sendMessageACK(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size);
                printf("warning: disable unexpected messages ok, msg class: 0x%02x, msg id: 0x%02x \n",msg_class, msg_id);
            }
            if(msg_class == 0x0A && msg_id == 0x38)
            {   
                key_id = 0x20910359; //CFG-MSGOUT-UBX_MON_RF_I2C
                int cfg_valset_msg_size = initCfgValset();
                cfgValsetPort(key_id, 0, cfg_valset_msg_size);
                printf("warning: disable unexpected messages \n");
                sendMessageACK(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size);
                printf("warning: disable unexpected messages ok, msg class: 0x%02x, msg id: 0x%02x \n",msg_class, msg_id);
            }
            if(msg_class == 0x0A && msg_id == 0x0B)
            {   
                key_id = 0x209101b9;//CFG-MSGOUT-UBX_MON_HW2_I2C 
                int cfg_valset_msg_size = initCfgValset();
                cfgValsetPort(key_id, 0, cfg_valset_msg_size);
                printf("warning: disable unexpected messages \n");
                sendMessageACK(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size);
                printf("warning: disable unexpected messages ok, msg class: 0x%02x, msg id: 0x%02x \n",msg_class, msg_id);
            }
        }else{//OLD
            ubx_payload_tx_cfg_msg_t cfg_msg; 
            cfg_msg.msg = (((uint16_t)msg_class) | ((uint16_t)msg_id)<<8);
            cfg_msg.rate = 0;
            printf("warning: disable unexpected messages \n");
            sendMessageACK(UBX_MSG_CFG_MSG, (uint8_t *)&cfg_msg, sizeof(cfg_msg));
            printf("warning: disable unexpected messages ok, msg class: 0x%02x, msg id: 0x%02x \n",msg_class, msg_id);
        }
    }
    
}
void gps_api_typedef::packet_decode(uint8_t *packet)
{
    uint8_t msg_class = packet[2];
    uint8_t msg_id = packet[3];
    uint8_t playload_length = packet[4] | packet[5] << 8;
    if(gps_debug)printf("info: msg class %x, id %x, playload length %d\n",
        msg_class,
        msg_id,
        playload_length);    
    switch(msg_class)
    {
        case UBX_CLASS_NAV:
            if(msg_id == 0x07 || msg_id == 0x04){
                NAV_CLASS_decode(packet);
            }else{
                disableMsg(msg_class, msg_id);
            }
            
        break;

        case UBX_CLASS_ACK:
            ACK_CLASS_decode(packet);
        break;

        case UBX_CLASS_MON:
            if(msg_id != 0x09){
                disableMsg(msg_class, msg_id);
            }
        break;

        default:
            disableMsg(msg_class, msg_id);
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
            if((nav_pvt->flags & UBX_RX_NAV_PVT_FLAGS_GNSSFIXOK)==1)
            {
                sensor_gps.fixType = nav_pvt->fixType;
                if (nav_pvt->flags & UBX_RX_NAV_PVT_FLAGS_DIFFSOLN)
                {
                    sensor_gps.fixType = 4;//DGPS
                }
            }
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
            _ack_state = UBX_ACK_GOT_NAK;
            printf("recv nack clsID %x msgID %x\n", ack_clsID, ack_msgID);
        break;

        case UBX_ID_ACK_ACK:
            ack = (ubx_payload_rx_ack_ack_t*)&packet[6];
            ack_clsID = ack->clsID;
            ack_msgID = ack->msgID;
            _ack_state = UBX_ACK_GOT_ACK;
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
        printf("checksum failed, msg class 0x%02x, msg id: 0x%02x; sum a: 0x%02x and 0x%02X, sum b: 0x%02x and 0x%02X\n", frame[index+2],frame[index+3], checksum.ck_a,frame[index + 5 + playload_length + 1], checksum.ck_b, frame[index + 5 + playload_length + 2]);
        printf("fail frame is: ");
        for (int i = 0; i < playload_length + 8; i++)
        {
            /* code */
            printf("0x%02x ",frame[index + i]);
        }
        printf("\n");
        
        return false;
    }
}

// -1 = NAK, error or timeout, 0 = ACK
// int	gps_api_typedef::waitForAck(const uint16_t msg, const unsigned timeout, const bool report)
// {
// 	int ret = -1;

// 	_ack_state = UBX_ACK_WAITING;
// 	_ack_waiting_msg = msg;	// memorize sent msg class&ID for ACK check

// 	gps_abstime time_started = gps_absolute_time();

// 	while ((_ack_state == UBX_ACK_WAITING) && (gps_absolute_time() < time_started + timeout * 1000)) {
// 		receive(timeout);
// 	}

// 	if (_ack_state == UBX_ACK_GOT_ACK) {
// 		ret = 0;	// ACK received ok

// 	} else if (report) {
// 		if (_ack_state == UBX_ACK_GOT_NAK) {
// 			// UBX_DEBUG("ubx msg 0x%04x NAK", SWAP16((unsigned)msg));

// 		} else {
// 			// UBX_DEBUG("ubx msg 0x%04x ACK timeout", SWAP16((unsigned)msg));
// 		}
// 	}

// 	_ack_state = UBX_ACK_IDLE;
// 	return ret;
// }

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
        ack_clsID = 0;
        ack_msgID = 0;
        _ack_state = UBX_ACK_WAITING;
        run();
        usleep(20*1000);

        if((ack_clsID | ack_msgID << 8) == msg)
        {
            return true;
        }
        cnt ++;
        if(cnt * 0.02 > 1)//1s timeout
        {
            cnt = 0;
            printf("warning ! gps config time out:\nexpected: %x %x\nrecv: %x %x\n",(uint8_t)(msg>>8),(uint8_t)((msg <<8)>>8),ack_msgID,ack_clsID);
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
    printf("info: the data write to gps:");
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
    static uint32_t time_last = get_time_now();
    uint32_t time_now = 0;
    int err = 0, ret = 0;
    int bytes_available = 0;
    err = ::ioctl(_serial_fd, FIONREAD, (unsigned long)&bytes_available);

    //printf("data remain : %d\n", bytes_available);
    if (err != 0 || bytes_available < (int)buf_length) {
        // printf("info: bytes_available is %d\n", bytes_available);
        // if (err == 0 && bytes_available > 0 && get_time_now() - last_time_read> 500000000U)//500ms
        // {
            
        // }else{
        time_now = get_time_now();
        if((time_now - time_last) > 10 * 1000)// timeout checker
        {
            time_last = time_now;

            //printf("uart buffer timeout , only %d bytes available\n",bytes_available);
            memset(buff,0xff,buf_length);//reset buffer while timeout
            ret = ::read(_serial_fd, buff, bytes_available);// read bytes_available bytes
            if (ret != bytes_available) {
                printf("ret != bytes_available %d\n", ret);
                return false;
            }else{// if success return ok ( buf_length - bytes_available bytes are 0)
                return true;
            }
        }else{
            time_last = time_now;
        }
            return false;
        // }
    }else{
        ret = ::read(_serial_fd, buff, buf_length);
        if (ret != buf_length) {
            printf("ret != buf_length %d\n", ret);
            return false;
        }else{
            // printf("info: data received is:");
            // for (int i = 0; i < buf_length; i++)
            // {
            //     printf("%x ",buff[i]);
            // }
            // printf("\n");
            return true;
        }
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

int gps_api_typedef::reset(GPSRestartType restart_type)
{
    memset(&_buf.payload_tx_cfg_rst, 0, sizeof(_buf.payload_tx_cfg_rst));
	// _buf.payload_tx_cfg_rst.resetMode = UBX_TX_CFG_RST_MODE_SOFTWARE;
    _buf.payload_tx_cfg_rst.resetMode = UBX_TX_CFG_RST_MODE_HARDWARE;

	switch (restart_type) {
	case GPSRestartType::Hot:
		_buf.payload_tx_cfg_rst.navBbrMask = UBX_TX_CFG_RST_BBR_MODE_HOT_START;
		break;

	case GPSRestartType::Warm:
		_buf.payload_tx_cfg_rst.navBbrMask = UBX_TX_CFG_RST_BBR_MODE_WARM_START;
		break;

	case GPSRestartType::Cold:
		_buf.payload_tx_cfg_rst.navBbrMask = UBX_TX_CFG_RST_BBR_MODE_COLD_START;
		break;

	default:
		return -2;
	}

	if (sendMessage(UBX_MSG_CFG_RST, (uint8_t *)&_buf, sizeof(_buf.payload_tx_cfg_rst))) {
		return 0;
	}

	return -2;
}

int gps_api_typedef::configureDevice()
{
    uint8_t _dyn_model = 6;//7: <2g; 6: <1g
	/* set configuration parameters */
	int cfg_valset_msg_size = initCfgValset();
	cfgValset<uint8_t>(UBX_CFG_KEY_NAVSPG_FIXMODE, 3 /* Auto 2d/3d */, cfg_valset_msg_size);
	cfgValset<uint8_t>(UBX_CFG_KEY_NAVSPG_UTCSTANDARD, 3 /* USNO (U.S. Naval Observatory derived from GPS) */,
			   cfg_valset_msg_size);
	cfgValset<uint8_t>(UBX_CFG_KEY_NAVSPG_DYNMODEL, _dyn_model, cfg_valset_msg_size);

	// disable odometer & filtering
	cfgValset<uint8_t>(UBX_CFG_KEY_ODO_USE_ODO, 0, cfg_valset_msg_size);
	cfgValset<uint8_t>(UBX_CFG_KEY_ODO_USE_COG, 0, cfg_valset_msg_size);
	cfgValset<uint8_t>(UBX_CFG_KEY_ODO_OUTLPVEL, 0, cfg_valset_msg_size);
	cfgValset<uint8_t>(UBX_CFG_KEY_ODO_OUTLPCOG, 0, cfg_valset_msg_size);

	// enable jamming monitor
	cfgValset<uint8_t>(UBX_CFG_KEY_ITFM_ENABLE, 1, cfg_valset_msg_size);

	// measurement rate
	// In case of F9P not in moving base mode we use 10Hz, otherwise 8Hz (receivers such as M9N can go higher as well, but
	// the number of used satellites will be restricted to 16. Not mentioned in datasheet)
	int rate_meas;
    rate_meas = 125;
	// if (_mode != UBXMode::Normal) {
	// 	rate_meas = 125; //8Hz for heading.

	// } else {
	// 	rate_meas = (_board == Board::u_blox9_F9P) ? 100 : 125;
	// }

	cfgValset<uint16_t>(UBX_CFG_KEY_RATE_MEAS, rate_meas, cfg_valset_msg_size);
	cfgValset<uint16_t>(UBX_CFG_KEY_RATE_NAV, 1, cfg_valset_msg_size);
	cfgValset<uint8_t>(UBX_CFG_KEY_RATE_TIMEREF, 0, cfg_valset_msg_size);

	if (!sendMessageACK(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
		return -1;
	}
    printf("info: Rate config ok\n");

	// if (waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, true) < 0) {
	// 	return -1;
	// }

	// // RTK (optional, as only RTK devices like F9P support it)
	// cfg_valset_msg_size = initCfgValset();
	// cfgValset<uint8_t>(UBX_CFG_KEY_NAVHPG_DGNSSMODE, 3 /* RTK Fixed */, cfg_valset_msg_size);

	// if (!sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
	// 	return -1;
	// }

	// waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, false);

	// // configure active GNSS systems (leave signal bands as is)
	// if (static_cast<int32_t>(gnssSystems) != 0) {
	// 	cfg_valset_msg_size = initCfgValset();

	// 	// GPS and QZSS should always be enabled and disabled together, according to uBlox
	// 	if (gnssSystems & GNSSSystemsMask::ENABLE_GPS) {
	// 		UBX_DEBUG("GNSS Systems: Use GPS + QZSS");
	// 		cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_GPS_ENA, 1, cfg_valset_msg_size);
	// 		cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_QZSS_ENA, 1, cfg_valset_msg_size);

	// 	} else {
	// 		cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_GPS_ENA, 0, cfg_valset_msg_size);
	// 		cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_QZSS_ENA, 0, cfg_valset_msg_size);
	// 	}

	// 	if (gnssSystems & GNSSSystemsMask::ENABLE_GALILEO) {
	// 		UBX_DEBUG("GNSS Systems: Use Galileo");
	// 		cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_GAL_ENA, 1, cfg_valset_msg_size);

	// 	} else {
	// 		cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_GAL_ENA, 0, cfg_valset_msg_size);
	// 	}


	// 	if (gnssSystems & GNSSSystemsMask::ENABLE_BEIDOU) {
	// 		UBX_DEBUG("GNSS Systems: Use BeiDou");
	// 		cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_BDS_ENA, 1, cfg_valset_msg_size);

	// 	} else {
	// 		cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_BDS_ENA, 0, cfg_valset_msg_size);
	// 	}

	// 	if (gnssSystems & GNSSSystemsMask::ENABLE_GLONASS) {
	// 		cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_GLO_ENA, 1, cfg_valset_msg_size);

	// 	} else {
	// 		cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_GLO_ENA, 0, cfg_valset_msg_size);
	// 	}

	// 	if (!sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
	// 		GPS_ERR("UBX GNSS config send failed");
	// 		return -1;
	// 	}

	// 	if (waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, true) < 0) {
	// 		return -1;
	// 	}

	// 	// send SBAS config separately, because it seems to be buggy (with u-center, too)
	// 	cfg_valset_msg_size = initCfgValset();

	// 	if (gnssSystems & GNSSSystemsMask::ENABLE_SBAS) {
	// 		UBX_DEBUG("GNSS Systems: Use SBAS");
	// 		cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_SBAS_ENA, 1, cfg_valset_msg_size);
	// 		cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_SBAS_L1CA_ENA, 1, cfg_valset_msg_size);

	// 	} else {
	// 		cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_SBAS_ENA, 0, cfg_valset_msg_size);
	// 	}

	// 	if (!sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
	// 		return -1;
	// 	}

	// 	waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, true);
	// }

	// Configure message rates
	// Send a new CFG-VALSET message to make sure it does not get too large
	cfg_valset_msg_size = initCfgValset();
	cfgValsetPort(UBX_CFG_KEY_MSGOUT_UBX_NAV_PVT_I2C, 1, cfg_valset_msg_size);
	_use_nav_pvt = true;
	cfgValsetPort(UBX_CFG_KEY_MSGOUT_UBX_NAV_DOP_I2C, 1, cfg_valset_msg_size);
	cfgValsetPort(UBX_CFG_KEY_MSGOUT_UBX_NAV_SAT_I2C, 0, cfg_valset_msg_size);
	cfgValsetPort(UBX_CFG_KEY_MSGOUT_UBX_MON_RF_I2C, 1, cfg_valset_msg_size);

	if (!sendMessageACK(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
		return -1;
	}

	// if (waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, true) < 0) {
	// 	return -1;
	// }

	// if (_interface == Interface::UART) {
		// Disable GPS protocols at I2C
		cfg_valset_msg_size = initCfgValset();
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_I2CINPROT_UBX, 0, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_I2CINPROT_NMEA, 0, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_I2CINPROT_RTCM3X, 0, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_I2COUTPROT_UBX, 0, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_I2COUTPROT_NMEA, 0, cfg_valset_msg_size);

		// if (_board == Board::u_blox9_F9P) {
		// 	cfgValset<uint8_t>(UBX_CFG_KEY_CFG_I2COUTPROT_RTCM3X, 0, cfg_valset_msg_size);
		// }

		if (!sendMessageACK(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
			return -1;
		}

		// if (waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, true) < 0) {
		// 	return -1;
		// }
	// }

	// int uart2_baudrate = 230400;

	// if (_mode == UBXMode::RoverWithMovingBase) {
	// 	UBX_DEBUG("Configuring UART2 for rover");
	// 	cfg_valset_msg_size = initCfgValset();
	// 	cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1OUTPROT_UBX, 1, cfg_valset_msg_size);
	// 	cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1OUTPROT_RTCM3X, 0, cfg_valset_msg_size);
	// 	// heading output period 1 second
	// 	cfgValset<uint8_t>(UBX_CFG_KEY_MSGOUT_UBX_NAV_RELPOSNED_UART1, 1, cfg_valset_msg_size);
	// 	// enable RTCM input on uart2 + set baudrate
	// 	cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2_STOPBITS, 1, cfg_valset_msg_size);
	// 	cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2_DATABITS, 0, cfg_valset_msg_size);
	// 	cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2_PARITY, 0, cfg_valset_msg_size);
	// 	cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2INPROT_UBX, 0, cfg_valset_msg_size);
	// 	cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2INPROT_RTCM3X, 1, cfg_valset_msg_size);
	// 	cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2INPROT_NMEA, 0, cfg_valset_msg_size);
	// 	cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2OUTPROT_UBX, 0, cfg_valset_msg_size);
	// 	cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2OUTPROT_RTCM3X, 0, cfg_valset_msg_size);
	// 	cfgValset<uint32_t>(UBX_CFG_KEY_CFG_UART2_BAUDRATE, uart2_baudrate, cfg_valset_msg_size);

	// 	if (!sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
	// 		return -1;
	// 	}

	// 	if (waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, true) < 0) {
	// 		return -1;
	// 	}

	// } else if (_mode == UBXMode::MovingBase) {
	// 	UBX_DEBUG("Configuring UART2 for moving base");
	// 	// enable RTCM output on uart2 + set baudrate
	// 	cfg_valset_msg_size = initCfgValset();
	// 	cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2_STOPBITS, 1, cfg_valset_msg_size);
	// 	cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2_DATABITS, 0, cfg_valset_msg_size);
	// 	cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2_PARITY, 0, cfg_valset_msg_size);
	// 	cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2INPROT_UBX, 0, cfg_valset_msg_size);
	// 	cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2INPROT_RTCM3X, 0, cfg_valset_msg_size);
	// 	cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2INPROT_NMEA, 0, cfg_valset_msg_size);
	// 	cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2OUTPROT_UBX, 0, cfg_valset_msg_size);
	// 	cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2OUTPROT_RTCM3X, 1, cfg_valset_msg_size);
	// 	cfgValset<uint32_t>(UBX_CFG_KEY_CFG_UART2_BAUDRATE, uart2_baudrate, cfg_valset_msg_size);

	// 	cfgValset<uint8_t>(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE4072_0_UART2, 1, cfg_valset_msg_size);
	// 	cfgValset<uint8_t>(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1230_UART2, 1, cfg_valset_msg_size);
	// 	cfgValset<uint8_t>(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1074_UART2, 1, cfg_valset_msg_size);
	// 	cfgValset<uint8_t>(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1084_UART2, 1, cfg_valset_msg_size);
	// 	cfgValset<uint8_t>(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1094_UART2, 1, cfg_valset_msg_size);
	// 	cfgValset<uint8_t>(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1124_UART2, 1, cfg_valset_msg_size);


	// 	if (!sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
	// 		return -1;
	// 	}

	// 	if (waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, true) < 0) {
	// 		return -1;
	// 	}

	// } else if (_mode == UBXMode::RoverWithMovingBaseUART1) {
	// 	UBX_DEBUG("Configuring UART1 for rover");
	// 	// heading output period 1 second
	// 	cfg_valset_msg_size = initCfgValset();
	// 	cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1INPROT_UBX, 1, cfg_valset_msg_size);
	// 	cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1INPROT_RTCM3X, 1, cfg_valset_msg_size);
	// 	cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1INPROT_NMEA, 0, cfg_valset_msg_size);
	// 	cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1OUTPROT_UBX, 1, cfg_valset_msg_size);
	// 	cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1OUTPROT_RTCM3X, 0, cfg_valset_msg_size);
	// 	cfgValset<uint8_t>(UBX_CFG_KEY_MSGOUT_UBX_NAV_RELPOSNED_UART1, 1, cfg_valset_msg_size);

	// 	if (!sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
	// 		return -1;
	// 	}

	// 	if (waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, true) < 0) {
	// 		return -1;
	// 	}

	// } else if (_mode == UBXMode::MovingBaseUART1) {
	// 	UBX_DEBUG("Configuring UART1 for moving base");
	// 	// enable RTCM output on uart1
	// 	cfg_valset_msg_size = initCfgValset();
	// 	cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1INPROT_UBX, 1, cfg_valset_msg_size);
	// 	cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1INPROT_RTCM3X, 0, cfg_valset_msg_size);
	// 	cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1INPROT_NMEA, 0, cfg_valset_msg_size);
	// 	cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1OUTPROT_UBX, 1, cfg_valset_msg_size);
	// 	cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1OUTPROT_RTCM3X, 1, cfg_valset_msg_size);

	// 	cfgValset<uint8_t>(UBX_CFG_KEY_MSGOUT_UBX_NAV_RELPOSNED_UART1, 0, cfg_valset_msg_size);
	// 	cfgValset<uint8_t>(UBX_CFG_KEY_MSGOUT_UBX_NAV_RELPOSNED_UART2, 0, cfg_valset_msg_size);

	// 	cfgValset<uint8_t>(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE4072_0_UART1, 1, cfg_valset_msg_size);
	// 	cfgValset<uint8_t>(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1230_UART1, 1, cfg_valset_msg_size);
	// 	cfgValset<uint8_t>(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1074_UART1, 1, cfg_valset_msg_size);
	// 	cfgValset<uint8_t>(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1084_UART1, 1, cfg_valset_msg_size);
	// 	cfgValset<uint8_t>(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1094_UART1, 1, cfg_valset_msg_size);
	// 	cfgValset<uint8_t>(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1124_UART1, 1, cfg_valset_msg_size);

	// 	if (!sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
	// 		return -1;
	// 	}

	// 	if (waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, true) < 0) {
	// 		return -1;
	// 	}

	// }

	return 0;
}

bool gps_api_typedef::cfgValsetPort(uint32_t key_id, uint8_t value, int &msg_size)
{
	// if (_interface == Interface::SPI) {
	// 	if (!cfgValset<uint8_t>(key_id + 4, value, msg_size)) {
	// 		return false;
	// 	}

	// } else {
		// enable on UART1 & USB (TODO: should we enable UART2 too? -> better would be to detect the port)
		if (!cfgValset<uint8_t>(key_id + 1, value, msg_size)) {
			return false;
		}

		if (!cfgValset<uint8_t>(key_id + 3, value, msg_size)) {
			return false;
		}
	// }

	return true;
}

// int gps_api_typedef::restartSurveyIn()
// {
// 	// if (_output_mode != OutputMode::RTCM) {
// 	// 	return -1;
// 	// }

// 	if (!_proto_ver_27_or_higher) {
// 		return restartSurveyInPreV27();
// 	}

// 	//disable RTCM output
// 	int cfg_valset_msg_size = initCfgValset();
// 	cfgValsetPort(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1005_I2C, 0, cfg_valset_msg_size);
// 	cfgValsetPort(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1077_I2C, 0, cfg_valset_msg_size);
// 	cfgValsetPort(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1087_I2C, 0, cfg_valset_msg_size);
// 	cfgValsetPort(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1230_I2C, 0, cfg_valset_msg_size);
// 	cfgValsetPort(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1097_I2C, 0, cfg_valset_msg_size);
// 	cfgValsetPort(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1127_I2C, 0, cfg_valset_msg_size);
// 	sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size);
// 	waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, false);

// 	if (_base_settings.type == BaseSettingsType::survey_in) {
// 		UBX_DEBUG("Starting Survey-in");

// 		cfg_valset_msg_size = initCfgValset();
// 		cfgValset<uint8_t>(UBX_CFG_KEY_TMODE_MODE, 1 /* Survey-in */, cfg_valset_msg_size);
// 		cfgValset<uint32_t>(UBX_CFG_KEY_TMODE_SVIN_MIN_DUR, _base_settings.settings.survey_in.min_dur, cfg_valset_msg_size);
// 		cfgValset<uint32_t>(UBX_CFG_KEY_TMODE_SVIN_ACC_LIMIT, _base_settings.settings.survey_in.acc_limit, cfg_valset_msg_size);
// 		cfgValsetPort(UBX_CFG_KEY_MSGOUT_UBX_NAV_SVIN_I2C, 5, cfg_valset_msg_size);

// 		if (!sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
// 			return -1;
// 		}

// 		if (waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, true) < 0) {
// 			return -1;
// 		}

// 	} else {
// 		// UBX_DEBUG("Setting fixed base position");

// 		const FixedPositionSettings &settings = _base_settings.settings.fixed_position;
// 		cfg_valset_msg_size = initCfgValset();
// 		cfgValset<uint8_t>(UBX_CFG_KEY_TMODE_MODE, 2 /* Fixed Mode */, cfg_valset_msg_size);
// 		cfgValset<uint8_t>(UBX_CFG_KEY_TMODE_POS_TYPE, 1 /* Lat/Lon/Height */, cfg_valset_msg_size);
// 		int64_t lat64 = (int64_t)(settings.latitude * 1e9);
// 		cfgValset<int32_t>(UBX_CFG_KEY_TMODE_LAT, (int32_t)(lat64 / 100), cfg_valset_msg_size);
// 		cfgValset<int8_t>(UBX_CFG_KEY_TMODE_LAT_HP, lat64 % 100 /* range [-99, 99] */, cfg_valset_msg_size);
// 		int64_t lon64 = (int64_t)(settings.longitude * 1e9);
// 		cfgValset<int32_t>(UBX_CFG_KEY_TMODE_LON, (int32_t)(lon64 / 100), cfg_valset_msg_size);
// 		cfgValset<int8_t>(UBX_CFG_KEY_TMODE_LON_HP, lon64 % 100 /* range [-99, 99] */, cfg_valset_msg_size);
// 		int64_t alt64 = (int64_t)((double)settings.altitude * 1e4);
// 		cfgValset<int32_t>(UBX_CFG_KEY_TMODE_HEIGHT, (int32_t)(alt64 / 100) /* cm */, cfg_valset_msg_size);
// 		cfgValset<int8_t>(UBX_CFG_KEY_TMODE_HEIGHT_HP, alt64 % 100 /* 0.1mm */, cfg_valset_msg_size);
// 		cfgValset<uint32_t>(UBX_CFG_KEY_TMODE_FIXED_POS_ACC, (uint32_t)(settings.position_accuracy * 10.f),
// 				    cfg_valset_msg_size);

// 		if (!sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
// 			return -1;
// 		}

// 		if (waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, true) < 0) {
// 			return -1;
// 		}

// 		// directly enable RTCM3 output
// 		return activateRTCMOutput(true);

// 	}

// 	return 0;
// }

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
