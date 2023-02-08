#include "QMC5883L.h"
#ifndef TESTMODE
	#include "msg_def.h"
	#include "systime.h"
	#include "sensor_calibration.h"
	// #include "config.h"
#else
	#include "../../px4lib/test_time.h"
#endif
// using namespace time_literals;
#define PX4_ERROR (-1)
#define PX4_OK 0
#define ENOENT    2
#define OK 0
#define EINVAL   22

using namespace QST_QMC5883L;
/* using example */

class QMC5883L qmc5883l;

void * thread_qmc5883l(void * ptr)
{
    timespec thread_qmc5883l_sleep;
    thread_qmc5883l_sleep.tv_sec = 0;
    thread_qmc5883l_sleep.tv_nsec = 10*1000*1000;//10ms
    core_bind(1);
    printf("qmc5883l thread starting !!!!!!!!!!!!!\n");
    while (1)
    {
        qmc5883l.RunImpl();
        nanosleep(&thread_qmc5883l_sleep,NULL);
    }
    return NULL;
}
void start_qmc5883l(void)
{
    printf("start qmc5883l\n");
    int ret = 0;
    
    ret = qmc5883l.init();
    ret = qmc5883l.probe();
    /* thread create */
    int rc;
    pthread_t thr_qmc5883l;
    if(rc = pthread_create(&thr_qmc5883l, NULL, thread_qmc5883l, NULL))
    {
		printf(" thread cretated failed %d \n", rc);
    }
    printf("qmc588l process pid : %d\n", (int)getpid()); 
}

//********************************************************************//
class i2c i2c_qmc5883l(QST_QMC5883L::I2C_ADDRESS_DEFAULT);

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

// QMC5883L::QMC5883L(I2CSPIBusOption bus_option, int bus, int bus_frequency, enum Rotation rotation) :
// 	I2C(DRV_MAG_DEVTYPE_QMC5883L, MODULE_NAME, bus, I2C_ADDRESS_DEFAULT, bus_frequency),
// 	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
// 	_px4_mag(get_device_id(), rotation)
// {
// 	_px4_mag.set_external(external());
// }

// QMC5883L::~QMC5883L()
// {
// 	perf_free(_reset_perf);
// 	perf_free(_bad_register_perf);
// 	perf_free(_bad_transfer_perf);
// }

int QMC5883L::init()
{
	int ret = i2c_qmc5883l.init(QMC5883L_DEVICE_BASE_PATH);

	if (ret != PX4_OK) {
		printf("i2c_qmc5883l init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool QMC5883L::Reset()
{
	_state = STATE::RESET;
	// ScheduleClear();
	// ScheduleNow();
	return true;
}

// void QMC5883L::print_status()
// {
// 	I2CSPIDriverBase::print_status();

// 	perf_print_counter(_reset_perf);
// 	perf_print_counter(_bad_register_perf);
// 	perf_print_counter(_bad_transfer_perf);
// }

int QMC5883L::probe()
{
	_retries = 1;

	for (int i = 0; i < 3; i++) {
		// first read 0x0 once
		const uint8_t cmd = 0;
		uint8_t buffer{};

		if (i2c_qmc5883l.transfer(&cmd, 1, &buffer, 1) == PX4_OK) {
			const uint8_t CHIP_ID = RegisterRead(QST_QMC5883L::Register::CHIP_ID);

			if (CHIP_ID == Chip_ID) {
				return PX4_OK;
			}
		}
	}

	return PX4_ERROR;
}

void QMC5883L::RunImpl()
{
	const uint64_t now = get_time_now();
	#ifndef TESTMODE
	mag_raw_typedef _mag_raw;
    mag_typedef _mag;
	#endif

	switch (_state) {
	case STATE::RESET:
		// CNTL2: Software Reset
		RegisterWrite(QST_QMC5883L::Register::CNTL2, QST_QMC5883L::CNTL2_BIT::SOFT_RST);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		// perf_count(_reset_perf);
		// usleep(100_ms); // POR Completion Time
		usleep(100*1000);
		break;

	case STATE::WAIT_FOR_RESET:

		// SOFT_RST: This bit is automatically reset to zero after POR routine
		if ((RegisterRead(QST_QMC5883L::Register::CHIP_ID) == Chip_ID)
		    && ((RegisterRead(QST_QMC5883L::Register::CNTL2) & QST_QMC5883L::CNTL2_BIT::SOFT_RST) == 0)) {

			// if reset succeeded then configure
			_state = STATE::CONFIGURE;
			// usleep(10_ms);
			usleep(10*1000);

		} else {
			// RESET not complete
			if (get_time_now()-_reset_timestamp > 1000*1000) {
				printf("Reset failed, retrying");
				_state = STATE::RESET;
				usleep(100*1000);

			} else {
				printf("Reset not complete, check again in 10 ms");
				usleep(10*1000);
			}
		}

		break;

	case STATE::CONFIGURE:
		if (Configure()) {
			// if configure succeeded then start reading every 20 ms (50 Hz)
			_state = STATE::READ;
			// ScheduleOnInterval(20_ms, 20_ms);
			usleep(20*1000);

		} else {
			// CONFIGURE not complete
			if (get_time_now()-_reset_timestamp > 1000*1000) {
				printf("Configure failed, resetting");
				_state = STATE::RESET;

			} else {
				printf("Configure failed, retrying");
			}

			usleep(100*1000);
		}

		break;

	case STATE::READ: {
			struct TransferBuffer {
				uint8_t X_LSB;
				uint8_t X_MSB;
				uint8_t Y_LSB;
				uint8_t Y_MSB;
				uint8_t Z_LSB;
				uint8_t Z_MSB;
				uint8_t STATUS;
			} buffer{};

			bool success = false;
			uint8_t cmd = static_cast<uint8_t>(QST_QMC5883L::Register::X_LSB);

			if (i2c_qmc5883l.transfer(&cmd, 1, (uint8_t *)&buffer, sizeof(buffer)) == PX4_OK) {
				// process data if successful transfer, no overflow
				if ((buffer.STATUS & STATUS_BIT::OVL) == 0) {
					int16_t x = combine(buffer.X_MSB, buffer.X_LSB);
					int16_t y = combine(buffer.Y_MSB, buffer.Y_LSB);
					int16_t z = combine(buffer.Z_MSB, buffer.Z_LSB);

					if (x != _prev_data[0] || y != _prev_data[1] || z != _prev_data[2]) {
						_prev_data[0] = x;
						_prev_data[1] = y;
						_prev_data[2] = z;

						// Sensor orientation
						//  Forward X := +X
						//  Right   Y := -Y
						//  Down    Z := -Z
						y = (y == INT16_MIN) ? INT16_MAX : -y; // -y
						z = (z == INT16_MIN) ? INT16_MAX : -z; // -z

						// _px4_mag.update(now, x, y, z);
						magData[0] = (float)y/12000;
						magData[1] = -(float)x/12000;
						magData[2] = (float)z/12000;
						timestamp[0] = now;

						success = true;

						#ifndef TESTMODE
						// publish mag data
						_mag_raw.timestamp = timestamp[0];
						_mag_raw.mag[0] = magData[0];
						_mag_raw.mag[1] = magData[1];
						_mag_raw.mag[2] = magData[2];
						calibration.apply_mag_calibration(_mag_raw.mag, _mag.mag);
						_mag.timestamp = _mag_raw.timestamp;
						mag_raw_msg.publish(&_mag_raw);
						mag_msg.publish(&_mag);
						//printf("time %lld, %f %f %f\n", _mag.timestamp, _mag.mag[0], _mag.mag[1], _mag.mag[2]);
						#endif

						if (_failure_count > 0) {
							_failure_count--;
						}
					}
				}

			} else {
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

			if (!success || (get_time_now()-_last_config_check_timestamp) > 100*1000) {
				// check configuration registers periodically or immediately following any failure
				if (RegisterCheck(_register_cfg[_checked_register])) {
					_last_config_check_timestamp = now;
					_checked_register = (_checked_register + 1) % size_register_cfg;

				} else {
					// register check failed, force reset
					// perf_count(_bad_register_perf);
					Reset();
				}
			}
		}

		break;
	}
}

bool QMC5883L::Configure()
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

	set_scale(1.f / 12000.f); // 12000 LSB/Gauss (Field Range = ±2G)

	return success;
}

bool QMC5883L::RegisterCheck(const register_config_t &reg_cfg)
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

uint8_t QMC5883L::RegisterRead(QST_QMC5883L::Register reg)
{
	const uint8_t cmd = static_cast<uint8_t>(reg);
	uint8_t buffer{};
	i2c_qmc5883l.transfer(&cmd, 1, &buffer, 1);
	return buffer;
}

void QMC5883L::RegisterWrite(QST_QMC5883L::Register reg, uint8_t value)
{
	uint8_t buffer[2] { (uint8_t)reg, value };
	i2c_qmc5883l.transfer(buffer, sizeof(buffer), nullptr, 0);
}

void QMC5883L::RegisterSetAndClearBits(QST_QMC5883L::Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);
	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}

void QMC5883L::set_scale(float scale)
{
	measurement_scale = scale;
}
