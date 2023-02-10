#include "ist8310.h"
// #define TESTMODE
#ifndef TESTMODE
	#include "systime.h"
#endif

// #include "time.h"
// #include "../config.h"

#define PX4_ERROR (-1)
#define PX4_OK 0
#define ENOENT    2
#define OK 0
#define EINVAL   22

using namespace iSentek_IST8310;
/* using example */
class IST8310 ist8310;

void * thread_ist8310(void * ptr)
{
    timespec thread_mag_sleep;
    thread_mag_sleep.tv_sec = 0;
    thread_mag_sleep.tv_nsec = 10*1000*1000;//10ms
    core_bind(1);
    while (1)
    {
        // ist8310.ist8310_read(magData, timestamp);
        ist8310.RunImpl();
        // nanosleep(&thread_mag_sleep,NULL);
        // usleep(500*1000);
    }
    return NULL;
}
void start_ist8310(void)
{
    printf("start ist8310\n");
    int ret = 0;

    ret = ist8310.init();
    ret = ist8310.probe();
    // ret = ist8310.Configure();

  ret = create_thread("ist8310", thread_ist8310, NULL);

}

/* class IST8310's Achievement  */

#ifndef _SYSTIME_H_
struct timeval tv;
struct timezone tz;
uint64_t time_in_usec = 0;
uint64_t time_in_usec_last = 0;
static uint64_t _time_now(void);
uint64_t get_time_now(void);
static uint64_t _time_now(void)
{
    gettimeofday(&tv, &tz);
    time_in_usec_last = time_in_usec;
    time_in_usec = tv.tv_sec * 1e6 + tv.tv_usec;
    return time_in_usec;
}
uint64_t get_time_now(void)
{
    static uint64_t start_time = _time_now();
    return _time_now() - start_time;
}
#endif

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}
class i2c i2c_ist8310(iSentek_IST8310::I2C_ADDRESS_DEFAULT);
// IST8310::IST8310(I2CSPIBusOption bus_option, int bus, uint8_t addr, int bus_frequency, enum Rotation rotation) :
// 	I2C(DRV_MAG_DEVTYPE_IST8310, MODULE_NAME, bus, addr, bus_frequency),
// 	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus, addr),
// 	_px4_mag(get_device_id(), rotation)
// {
// 	_px4_mag.set_external(external());
// }
IST8310::IST8310()
{
}
// IST8310::~IST8310()
// {
// 	perf_free(_reset_perf);
// 	perf_free(_bad_register_perf);
// 	perf_free(_bad_transfer_perf);
// }

int IST8310::init()
{
	int ret = i2c_ist8310.init(IST8310_DEVICE_BASE_PATH);

	if (ret != PX4_OK) {
		printf("i2c_ist8310.init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool IST8310::Reset()
{
	_state = STATE::RESET;
	// ScheduleClear();
	// ScheduleNow();
	return true;
}

// void IST8310::print_status()
// {
// 	I2CSPIDriverBase::print_status();

// 	perf_print_counter(_reset_perf);
// 	perf_print_counter(_bad_register_perf);
// 	perf_print_counter(_bad_transfer_perf);
// }

int IST8310::probe()
{
	const uint8_t WAI = RegisterRead(iSentek_IST8310::Register::WAI);

	if (WAI != Device_ID) {
		printf("unexpected WAI 0x%02x", WAI);
		return PX4_ERROR;
	}

	return PX4_OK;
}

void IST8310::RunImpl()
{
	const uint64_t now = get_time_now();

	switch (_state) {
	case STATE::RESET:
		// CNTL2: Software Reset
		RegisterWrite(iSentek_IST8310::Register::CNTL2, iSentek_IST8310::CNTL2_BIT::SRST);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		// perf_count(_reset_perf);
		usleep(50*1000); // Power On Reset: max 50ms
		// ScheduleDelayed(50_ms); // Power On Reset: max 50ms
		break;

	case STATE::WAIT_FOR_RESET:

		// SRST: This bit is automatically reset to zero after POR routine
		if ((RegisterRead(iSentek_IST8310::Register::WAI) == Device_ID)
		    && ((RegisterRead(iSentek_IST8310::Register::CNTL2) & iSentek_IST8310::CNTL2_BIT::SRST) == 0)) {

			// if reset succeeded then configure
			_state = STATE::CONFIGURE;
			// ScheduleDelayed(10_ms);
			usleep(10*1000);

		} else {
			// RESET not complete
			if (elapsed_time(_reset_timestamp) > 1000*1000) {
				printf("Reset failed, retrying\n");
				_state = STATE::RESET;
				// ScheduleDelayed(100_ms);
				usleep(100*1000);

			} else {
				printf("Reset not complete, check again in 10 ms\n");
				// ScheduleDelayed(10_ms);
				usleep(10*1000);
			}
		}

		break;

	case STATE::CONFIGURE:
		if (Configure()) {
			// if configure succeeded then start measurement cycle
			_state = STATE::MEASURE;
			// ScheduleDelayed(20_ms);
			usleep(20*1000);

		} else {
			// CONFIGURE not complete
			if (elapsed_time(_reset_timestamp) > 1000*1000) {
				printf("Configure failed, resetting");
				_state = STATE::RESET;

			} else {
				printf("Configure failed, retrying");
			}

			// ScheduleDelayed(100_ms);
			usleep(100*1000);
		}

		break;

	case STATE::MEASURE:
		RegisterWrite(iSentek_IST8310::Register::CNTL1, iSentek_IST8310::CNTL1_BIT::MODE_SINGLE_MEASUREMENT);
		_state = STATE::READ;
		// ScheduleDelayed(20_ms); // Wait at least 6ms. (minimum waiting time for 16 times internal average setup)
		usleep(20*1000);
		break;

	case STATE::READ: {
			struct TransferBuffer {
				uint8_t STAT1;
				uint8_t DATAXL;
				uint8_t DATAXH;
				uint8_t DATAYL;
				uint8_t DATAYH;
				uint8_t DATAZL;
				uint8_t DATAZH;
			} buffer{};

			bool success = false;
			uint8_t cmd = static_cast<uint8_t>(iSentek_IST8310::Register::STAT1);

			if (i2c_ist8310.transfer(&cmd, 1, (uint8_t *)&buffer, sizeof(buffer)) == PX4_OK) {

				if (buffer.STAT1 & STAT1_BIT::DRDY) {
					int16_t x = combine(buffer.DATAXH, buffer.DATAXL);
					int16_t y = combine(buffer.DATAYH, buffer.DATAYL);
					int16_t z = combine(buffer.DATAZH, buffer.DATAZL);

					// sensor's frame is +x forward, +y right, +z up
					z = (z == INT16_MIN) ? INT16_MAX : -z; // flip z

					// _px4_mag.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf));
					// _px4_mag.update(now, x, y, z);
					magData[0] = (float)y/1320;
					magData[1] = (float)x/1320;
					magData[2] = (float)z/1320;
					timestamp[0] = now;

					success = true;

					if (_failure_count > 0) {
						_failure_count--;
					}

					#ifndef TESTMODE
					mag_raw_typedef _mag_raw;
    				mag_typedef _mag;
					_mag_raw.timestamp = timestamp[0];
					_mag_raw.mag[0] = magData[0];
					_mag_raw.mag[1] = magData[1];
					_mag_raw.mag[2] = magData[2];
					calibration.apply_mag_calibration(_mag_raw.mag, _mag.mag);
					_mag.timestamp = _mag_raw.timestamp;
					mag_raw_msg.publish(&_mag_raw);
					mag_msg.publish(&_mag);
					#else
					// printf("time %f %f %f %f\n", (float)timestamp[0]/1e6,magData[0], magData[1], magData[2]);
					#endif
				}

			} else {
				printf("ist8310 measure data read fail!\n\r");
				// perf_count(_bad_transfer_perf);
			}

			if (!success) {
				_failure_count++;

				// full reset if things are failing consistently
				if (_failure_count > 10) {
					Reset();
					return;
				}
			}

			if (!success || elapsed_time(_last_config_check_timestamp) > 100*1000) {
				// check configuration registers periodically or immediately following any failure
				if (RegisterCheck(_register_cfg[_checked_register])) {
					_last_config_check_timestamp = now;
					_checked_register = (_checked_register + 1) % size_register_cfg;

				} else {
					// register check failed, force reset
					// perf_count(_bad_register_perf);
					Reset();
					return;
				}
			}

			// initiate next measurement
			RegisterWrite(iSentek_IST8310::Register::CNTL1, iSentek_IST8310::CNTL1_BIT::MODE_SINGLE_MEASUREMENT);
			// ScheduleDelayed(20_ms); // Wait at least 6ms. (minimum waiting time for 16 times internal average setup)
			usleep(10*1000);
		}

		break;
	}
}

bool IST8310::Configure()
{
	// first set and clear all configured register bits
	for (const auto &reg_cfg : _register_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	// now check that all are configured
	bool success = true;

	for (const auto &reg_cfg : _register_cfg) {
		if (!RegisterCheck(reg_cfg)) {
			success = false;
		}
	}

	set_scale(1.f / 1320.f); // 1320 LSB/Gauss

	return success;
}

bool IST8310::RegisterCheck(const register_config_t &reg_cfg)
{
	bool success = true;

	const uint8_t reg_value = RegisterRead(reg_cfg.reg);

	if (reg_cfg.set_bits && ((reg_value & reg_cfg.set_bits) != reg_cfg.set_bits)) {
		printf("0x%02hhX: 0x%02hhX (0x%02hhX not set)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.set_bits);
		success = false;
	}

	if (reg_cfg.clear_bits && ((reg_value & reg_cfg.clear_bits) != 0)) {
		printf("0x%02hhX: 0x%02hhX (0x%02hhX not cleared)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.clear_bits);
		success = false;
	}

	return success;
}

uint8_t IST8310::RegisterRead(iSentek_IST8310::Register reg)
{
	const uint8_t cmd = static_cast<uint8_t>(reg);
	uint8_t buffer{};
	i2c_ist8310.transfer(&cmd, 1, &buffer, 1);
	return buffer;
}

void IST8310::RegisterWrite(iSentek_IST8310::Register reg, uint8_t value)
{
	uint8_t buffer[2] { (uint8_t)reg, value };
	i2c_ist8310.transfer(buffer, sizeof(buffer), nullptr, 0);
}

void IST8310::RegisterSetAndClearBits(iSentek_IST8310::Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);
	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}

uint64_t IST8310::elapsed_time(uint64_t base_time)
{
	return get_time_now()-base_time;
}

void IST8310::set_scale(float scale)
{
	measurement_scale = scale;
}

/*
* read mag data straightforwardly
*/
int IST8310::ist8310_read(float *magData, uint64_t *timestamp)
{
	RegisterWrite(iSentek_IST8310::Register::CNTL1, iSentek_IST8310::CNTL1_BIT::MODE_SINGLE_MEASUREMENT);
	usleep(1*1000);
	struct TransferBuffer {
		uint8_t STAT1;
		uint8_t DATAXL;
		uint8_t DATAXH;
		uint8_t DATAYL;
		uint8_t DATAYH;
		uint8_t DATAZL;
		uint8_t DATAZH;
	} buffer{};

	bool success = false;
	uint8_t cmd = static_cast<uint8_t>(iSentek_IST8310::Register::STAT1);

	if (i2c_ist8310.transfer(&cmd, 1, (uint8_t *)&buffer, sizeof(buffer)) == PX4_OK) 
	{

		if (buffer.STAT1 & STAT1_BIT::DRDY) {
			int16_t x = combine(buffer.DATAXH, buffer.DATAXL);
			int16_t y = combine(buffer.DATAYH, buffer.DATAYL);
			int16_t z = combine(buffer.DATAZH, buffer.DATAZL);

			// sensor's frame is +x forward, +y right, +z up
			z = (z == INT16_MIN) ? INT16_MAX : -z; // flip z

			// _px4_mag.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf));
			magData[0] = (float)x/1320;
			magData[1] = (float)y/1320;
			magData[2] = (float)z/1320;
			timestamp[0] = get_time_now();
			// timestamp[0] = 2;
			// _px4_mag.update(now, x, y, z);

			success = true;

			if (_failure_count > 0) {
				_failure_count--;
			}
		}

	}

	// initiate next measurement
	RegisterWrite(iSentek_IST8310::Register::CNTL1, iSentek_IST8310::CNTL1_BIT::MODE_SINGLE_MEASUREMENT);
	// ScheduleDelayed(20_ms); // Wait at least 6ms. (minimum waiting time for 16 times internal average setup)
	usleep(1*1000);
	return 0;
}

