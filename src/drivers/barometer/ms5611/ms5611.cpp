#include "ms5611.h"

#define PX4_ERROR (-1)
#define PX4_OK 0
#define ENOENT    2
#define OK 0
#define EINVAL   22


/*using example*/
class MS5611 c_ms5611;
void * thread_baro(void * ptr)
{
    timespec thread_baro_sleep;
    thread_baro_sleep.tv_sec = 0;
    thread_baro_sleep.tv_nsec = 10*1000*1000;//10ms
    
    while (1)
    {
        c_ms5611.RunImpl();
        // nanosleep(&thread_baro_sleep,NULL);
        // usleep(500*1000);
    }
    return NULL;
}

void start_baro(void)
{

  bool ret = create_thread("barometer", thread_baro, NULL);

}


/******************************************/
class i2c i2c_ms5611(MS5611_ADDRESS_1);

// int MS5611::init()
// {
// 	int ret = i2c_ms5611.init(MS5611_DEVICE_BASE_PATH);

// 	if (ret != PX4_OK) {
// 		printf("i2c_ms5611.init failed (%i)", ret);
// 		return ret;
// 	}

// 	return 0;
// }
MS5611::MS5611()
{
	if(_device_type==MS5607_DEVICE)
	{
		printf("_device_type is MS5607\n\r");
		_device_type = MS5611_DEVICE;
		printf("_device_type is MS5611 but now is set as MS5611\n\r");
	}
	else if(_device_type==MS5611_DEVICE)
	{
		printf("_device_type is MS5611\n\r");
	}
	else
	{
		_device_type = MS5611_DEVICE;
		printf("_device_type is unknown but now is set as MS5611\n\r");
	}
}

int MS5611::init()
{
	int ret;
    /* init i2c device */
    ret = i2c_ms5611.init(MS5611_DEVICE_BASE_PATH);
	if (ret != PX4_OK) {
		printf("i2c_ms5611.init failed (%i)", ret);
		return ret;
	}
	// _prom_s = &_interface._prom.c[0];
	ret = _interface.probe();
	if (ret != PX4_OK) {
		printf("MS5611_I2C probe failed (%i)", ret);
		return ret;
	}

	/* do a first measurement cycle to populate reports with valid data */
	_measure_phase = 0;

	while (true) {
		/* do temperature first */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		usleep(MS5611_CONVERSION_INTERVAL);

		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* now do a pressure measurement */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		usleep(MS5611_CONVERSION_INTERVAL);

		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		// /* state machine will have generated a report, copy it out */
		// const sensor_baro_s &brp = _px4_barometer.get();

		// if (_device_type == MS5607_DEVICE) {
		// 	if (brp.pressure < 520.0f) {
		// 		/* This is likely not this device, abort */
		// 		ret = -EINVAL;
		// 		break;

		// 	} else if (brp.pressure > 1500.0f) {
		// 		/* This is likely not this device, abort */
		// 		ret = -EINVAL;
		// 		break;
		// 	}
		// }

		// switch (_device_type) {
		// default:

		// /* fall through */
		// case MS5611_DEVICE:
		// 	_interface->set_device_type(DRV_BARO_DEVTYPE_MS5611);
		// 	_px4_barometer.set_device_type(DRV_BARO_DEVTYPE_MS5611);
		// 	break;

		// case MS5607_DEVICE:
		// 	_interface->set_device_type(DRV_BARO_DEVTYPE_MS5607);
		// 	_px4_barometer.set_device_type(DRV_BARO_DEVTYPE_MS5607);
		// 	break;
		// }

		ret = OK;

		break;
	}

	if (ret == 0) {
		start();
	}

	return ret;
}


void MS5611::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_measure_phase = 0;

	/* schedule a cycle to start things */
	// ScheduleDelayed(MS5611_CONVERSION_INTERVAL);
	usleep(MS5611_CONVERSION_INTERVAL);
}

int MS5611::measure()
{
	// perf_begin(_measure_perf);

	/*
	 * In phase zero, request temperature; in other phases, request pressure.
	 */
	unsigned addr = (_measure_phase == 0) ? ADDR_CMD_CONVERT_D2 : ADDR_CMD_CONVERT_D1;
	// printf("measure phase: %d\n\r", _measure_phase);
	/*
	 * Send the command to begin measuring.
	 */
	int ret = _interface.ioctl(IOCTL_MEASURE, addr);

	if (OK != ret) {
		// perf_count(_comms_errors);
	}

	// _px4_barometer.set_error_count(perf_event_count(_comms_errors));

	// perf_end(_measure_perf);

	return ret;
}

int MS5611::collect()
{
	uint32_t raw;

	// perf_begin(_sample_perf);

	/* read the most recent measurement - read offset/size are hardcoded in the interface */
	const uint64_t timestamp_sample = get_time_now();
	int ret = _interface.read(0, (void *)&raw, 0);
	// printf("measure data: %d\n\r",raw);
	if (ret < 0) {
		// perf_count(_comms_errors);
		// perf_end(_sample_perf);
		printf("ms5611 i2c read collect fail\n\r");
		return ret;
	}

	/* handle a measurement */
	if (_measure_phase == 0) {

		/* temperature offset (in ADC units) */
		int32_t dT = (int32_t)raw - ((int32_t)_interface._prom.s.c5_reference_temp << 8);

		/* absolute temperature in centidegrees - note intermediate value is outside 32-bit range */
		int32_t TEMP = 2000 + (int32_t)(((int64_t)dT * _interface._prom.s.c6_temp_coeff_temp) >> 23);

		/* base sensor scale/offset values */
		if (_device_type == MS5611_DEVICE) {
			/* Perform MS5611 Caculation */
			_OFF  = ((int64_t)_interface._prom.s.c2_pressure_offset << 16) + (((int64_t)_interface._prom.s.c4_temp_coeff_pres_offset * dT) >> 7);
			_SENS = ((int64_t)_interface._prom.s.c1_pressure_sens << 15) + (((int64_t)_interface._prom.s.c3_temp_coeff_pres_sens * dT) >> 8);

			/* MS5611 temperature compensation */
			if (TEMP < 2000) {

				int32_t T2 = POW2(dT) >> 31;

				int64_t f = POW2((int64_t)TEMP - 2000);
				int64_t OFF2 = 5 * f >> 1;
				int64_t SENS2 = 5 * f >> 2;

				if (TEMP < -1500) {

					int64_t f2 = POW2(TEMP + 1500);
					OFF2 += 7 * f2;
					SENS2 += 11 * f2 >> 1;
				}

				TEMP -= T2;
				_OFF  -= OFF2;
				_SENS -= SENS2;
			}

		} else if (_device_type == MS5607_DEVICE) {
			/* Perform MS5607 Caculation */
			_OFF  = ((int64_t)_interface._prom.s.c2_pressure_offset << 17) + (((int64_t)_interface._prom.s.c4_temp_coeff_pres_offset * dT) >> 6);
			_SENS = ((int64_t)_interface._prom.s.c1_pressure_sens << 16) + (((int64_t)_interface._prom.s.c3_temp_coeff_pres_sens * dT) >> 7);

			/* MS5607 temperature compensation */
			if (TEMP < 2000) {

				int32_t T2 = POW2(dT) >> 31;

				int64_t f = POW2((int64_t)TEMP - 2000);
				int64_t OFF2 = 61 * f >> 4;
				int64_t SENS2 = 2 * f;

				if (TEMP < -1500) {
					int64_t f2 = POW2(TEMP + 1500);
					OFF2 += 15 * f2;
					SENS2 += 8 * f2;
				}

				TEMP -= T2;
				_OFF  -= OFF2;
				_SENS -= SENS2;
			}
		}

		float temperature = TEMP / 100.0f;
		// _px4_barometer.set_temperature(temperature);
        _temperature = temperature;

	} else {
		/* pressure calculation, result in Pa */
		int32_t P = (((raw * _SENS) >> 21) - _OFF) >> 15;

		float pressure = P / 100.0f;		/* convert to millibar */

		// _px4_barometer.update(timestamp_sample, pressure);
        _timestamp_sample = timestamp_sample;
        _pressure = pressure;

		#ifndef TESTMODE
		// publish baro data
		baro_typedef _baro;
		_baro.timestamp = _timestamp_sample;
		_baro.temperature = _pressure;
		_baro.temperature = _temperature;
		baro_msg.publish(&_baro);
		#endif
	}

	/* update the measurement state machine */
	INCREMENT(_measure_phase, MS5611_MEASUREMENT_RATIO + 1);
	
	// perf_end(_sample_perf);

	return OK;
}

void MS5611::RunImpl()
{
	int ret;
	unsigned dummy;

	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		ret = collect();

		if (ret != OK) {
			if (ret == -6) {
				/*
				 * The ms5611 seems to regularly fail to respond to
				 * its address; this happens often enough that we'd rather not
				 * spam the console with a message for this.
				 */
			} else {
				//DEVICE_LOG("collection error %d", ret);
			}

			/* issue a reset command to the sensor */
			_interface.ioctl(IOCTL_RESET, dummy);
			/* reset the collection state machine and try again - we need
			 * to wait 2.8 ms after issuing the sensor reset command
			 * according to the MS5611 datasheet
			 */
			// ScheduleDelayed(2800);
			usleep(2800);
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;
	}

	/* measurement phase */
	ret = measure();

	if (ret != OK) {
		/* issue a reset command to the sensor */
		_interface.ioctl(IOCTL_RESET, dummy);
		/* reset the collection state machine and try again */
		start();
		return;
	}

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	// ScheduleDelayed(MS5611_CONVERSION_INTERVAL);
	usleep(MS5611_CONVERSION_INTERVAL);
}

int MS5611_I2C::read(unsigned offset, void *data, unsigned count)
{
	union _cvt {
		uint8_t	b[4];
		uint32_t w;
	} *cvt = (_cvt *)data;
	uint8_t buf[3];

	/* read the most recent measurement */
	uint8_t cmd = 0;
	int ret = i2c_ms5611.transfer(&cmd, 1, &buf[0], 3);

	if (ret == PX4_OK) {
		/* fetch the raw value */
		cvt->b[0] = buf[2];
		cvt->b[1] = buf[1];
		cvt->b[2] = buf[0];
		cvt->b[3] = 0;
	}

	return ret;
}

int MS5611_I2C::ioctl(unsigned operation, unsigned &arg)
{
	int ret;

	switch (operation) {
	case IOCTL_RESET:
		ret = _reset();
		if(ret < PX4_OK)
		printf("ms5611 i2c read _reset fail\n\r");
		break;

	case IOCTL_MEASURE:
		ret = _measure(arg);
		if(ret < PX4_OK)
		printf("ms5611 i2c read _measure fail\n\r");
		break;

	default:
		ret = EINVAL;
	}

	return ret;
}

int MS5611_I2C::probe()
{
	i2c_ms5611._retries = 10;
    
	if ((PX4_OK == _probe_address(MS5611_ADDRESS_1)) ||
	    (PX4_OK == _probe_address(MS5611_ADDRESS_2))) {
		/*
		 * Disable retries; we may enable them selectively in some cases,
		 * but the device gets confused if we retry some of the commands.
		 */
		i2c_ms5611._retries = 0;
		return PX4_OK;
	}

	return -EIO;
}

int MS5611_I2C::_probe_address(uint8_t address)
{
	/* select the address we are going to try */
	i2c_ms5611.set_device_address(address);

	/* send reset command */
	if (PX4_OK != _reset()) {
		return -EIO;
	}

	/* read PROM */
	if (PX4_OK != _read_prom()) {
		return -EIO;
	}
	printf("ms5611 i2c addr is: %x\n\r",address);
	return PX4_OK;
}

int MS5611_I2C::_reset()
{
	unsigned	old_retrycount = i2c_ms5611._retries;
	uint8_t		cmd = ADDR_RESET_CMD;
	int		result;

	/* bump the retry count */
	i2c_ms5611._retries = 10;
	result = i2c_ms5611.transfer(&cmd, 1, nullptr, 0);
	i2c_ms5611._retries = old_retrycount;

	return result;
}

int MS5611_I2C::_measure(unsigned addr)
{
	/*
	 * Disable retries on this command; we can't know whether failure
	 * means the device did or did not see the command.
	 */
	i2c_ms5611._retries = 0;

	uint8_t cmd = addr;
	return i2c_ms5611.transfer(&cmd, 1, nullptr, 0);
}

int MS5611_I2C::_read_prom()
{
	uint8_t		prom_buf[2];
	union {
		uint8_t		b[2];
		uint16_t	w;
	} cvt;

	/*
	 * Wait for PROM contents to be in the device (2.8 ms) in the case we are
	 * called immediately after reset.
	 */
	usleep(3000);

	uint8_t last_val = 0;
	bool bits_stuck = true;

	/* read and convert PROM words */
	for (int i = 0; i < 8; i++) {
		uint8_t cmd = ADDR_PROM_SETUP + (i * 2);

		if (PX4_OK != i2c_ms5611.transfer(&cmd, 1, &prom_buf[0], 2)) {
			break;
		}

		/* check if all bytes are zero */
		if (i == 0) {
			/* initialize to first byte read */
			last_val = prom_buf[0];
		}

		if ((prom_buf[0] != last_val) || (prom_buf[1] != last_val)) {
			bits_stuck = false;
		}

		/* assemble 16 bit value and convert from big endian (sensor) to little endian (MCU) */
		cvt.b[0] = prom_buf[1];
		cvt.b[1] = prom_buf[0];
		_prom.c[i] = cvt.w;
	}
	printf("factory_setup: %d, %d, %d, %d, %d, %d, %d, serial_and_crc: %d,\n\r", _prom.s.factory_setup,_prom.s.c1_pressure_sens,_prom.s.c2_pressure_offset, _prom.s.c3_temp_coeff_pres_sens, _prom.s.c4_temp_coeff_pres_offset,_prom.s.c5_reference_temp,_prom.s.c6_temp_coeff_temp,_prom.s.serial_and_crc);
	/* calculate CRC and return success/failure accordingly */
	return (crc4(&_prom.c[0]) && !bits_stuck) ? PX4_OK : -EIO;
	// return 0;
}


// namespace ms5611
// {

/**
 * MS5611 crc4 cribbed from the datasheet
 */
bool crc4(uint16_t *n_prom)
{
	int16_t cnt;
	uint16_t n_rem;
	uint16_t crc_read;
	uint8_t n_bit;

	n_rem = 0x00;

	/* save the read crc */
	crc_read = n_prom[7];

	/* remove CRC byte */
	n_prom[7] = (0xFF00 & (n_prom[7]));

	for (cnt = 0; cnt < 16; cnt++) {
		/* uneven bytes */
		if (cnt & 1) {
			n_rem ^= (uint8_t)((n_prom[cnt >> 1]) & 0x00FF);

		} else {
			n_rem ^= (uint8_t)(n_prom[cnt >> 1] >> 8);
		}

		for (n_bit = 8; n_bit > 0; n_bit--) {
			if (n_rem & 0x8000) {
				n_rem = (n_rem << 1) ^ 0x3000;

			} else {
				n_rem = (n_rem << 1);
			}
		}
	}

	/* final 4 bit remainder is CRC value */
	n_rem = (0x000F & (n_rem >> 12));
	n_prom[7] = crc_read;

	/* return true if CRCs match */
	return (0x000F & crc_read) == (n_rem ^ 0x00);
}

// } // namespace ms5611