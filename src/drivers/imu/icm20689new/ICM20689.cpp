#include "ICM20689.h"

#define spi0_path        "/dev/spidev0.0"//前面的0表示SPI1，后面的0表示片选引脚0
// using namespace time_literals;
using namespace math_px4;

/* using example */
class adaptive_delay_typedef icm20689new_delay(0.5,15,400);
class ICM20689 my_icm20689;
void * thread_icm20689_new(void * ptr)
{
	timespec thread_icm20689_sleep;
    thread_icm20689_sleep.tv_sec = 0;
    thread_icm20689_sleep.tv_nsec = 1*1000*1000;//1250us
    core_bind(IMU_CORE);
    rflypilot_config_typedef config;
    rflypilot_config_msg.read(&config);
    thread_msg_typedef _imu_thread;
	#if USING_THREAD_SYNC
	char count_for_lpe = 0;
	#endif
    while (1)
    {
     //  if(TASK_SCHEDULE_DEBUG)
     //  {
    	// _imu_thread.timestamp = get_time_now();
    	// imu_thread_msg.publish(&_imu_thread);
     //  }

		my_icm20689.RunImpl();
		#if USING_THREAD_SYNC
		if(count_for_lpe == IMU_LPE)
		{
		pthread_mutex_lock(&mutex_imu2lpe);  
		pthread_cond_signal(&cond_imu2lpe);   
		pthread_mutex_unlock(&mutex_imu2lpe);
		count_for_lpe = 0;
		}
		count_for_lpe++;
		#endif
        // nanosleep(&thread_icm20689_sleep,NULL);
        delay_us_combined((uint64_t)(1000000.f / config.imu_rate),&scheduler.imu_flag,&icm20689new_delay);

    }
    return NULL;
}
void start_icm20689_new()
{
    printf("start icm20689new\n");
    int ret = 0;

    ret = my_icm20689.icm20689_init();
    ret = my_icm20689.probe();
    /* thread create */
    int rc;
    pthread_t thr_icm20689;
    if(rc = pthread_create(&thr_icm20689, NULL, thread_icm20689_new, NULL))
    {
		printf(" thread cretated failed %d \n", rc);
    }
    printf("icm20689 process pid : %d\n", (int)getpid()); 
}
/******************** icm20689 driver **************************/
static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

/* Define PX4_ISFINITE */
#ifdef __cplusplus
constexpr bool PX4_ISFINITE(float x) { return __builtin_isfinite(x); }
constexpr bool PX4_ISFINITE(double x) { return __builtin_isfinite(x); }
#endif /* __cplusplus */

// ICM20689::ICM20689(I2CSPIBusOption bus_option, int bus, uint32_t device, enum Rotation rotation, int bus_frequency,
// 		   spi_mode_e spi_mode, spi_drdy_gpio_t drdy_gpio) :
// 	SPI(DRV_IMU_DEVTYPE_ICM20689, MODULE_NAME, bus, device, spi_mode, bus_frequency),
// 	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
// 	_drdy_gpio(drdy_gpio),
// 	_px4_accel(get_device_id(), ORB_PRIO_HIGH, rotation),
// 	_px4_gyro(get_device_id(), ORB_PRIO_HIGH, rotation)
// {
// 	if (drdy_gpio != 0) {
// 		_drdy_missed_perf = perf_alloc(PC_COUNT, MODULE_NAME": DRDY missed");
// 	}

// 	ConfigureSampleRate(_px4_gyro.get_max_rate_hz());
// }

// ICM20689::~ICM20689()
// {
// 	perf_free(_bad_register_perf);
// 	perf_free(_bad_transfer_perf);
// 	perf_free(_fifo_empty_perf);
// 	perf_free(_fifo_overflow_perf);
// 	perf_free(_fifo_reset_perf);
// 	perf_free(_drdy_missed_perf);
// }

int ICM20689::icm20689_init()
{
	int ret = SPI::init(spi0_path, SPIDEV_MODE3, 10000000);

	if (ret != PX4_OK) {
		printf("SPI::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool ICM20689::Reset()
{
	_state = STATE::RESET;
	DataReadyInterruptDisable();
	// ScheduleClear();
	// ScheduleNow();
	return true;
}

void ICM20689::exit_and_cleanup()
{
	DataReadyInterruptDisable();
	// I2CSPIDriverBase::exit_and_cleanup();
}

// void ICM20689::print_status()
// {
// 	I2CSPIDriverBase::print_status();

// 	PX4_INFO("FIFO empty interval: %d us (%.1f Hz)", _fifo_empty_interval_us, 1e6 / _fifo_empty_interval_us);

// 	perf_print_counter(_bad_register_perf);
// 	perf_print_counter(_bad_transfer_perf);
// 	perf_print_counter(_fifo_empty_perf);
// 	perf_print_counter(_fifo_overflow_perf);
// 	perf_print_counter(_fifo_reset_perf);
// 	perf_print_counter(_drdy_missed_perf);
// }

int ICM20689::probe()
{
	const uint8_t whoami = RegisterRead(InvenSense_ICM20689::Register::WHO_AM_I);

	if (whoami != InvenSense_ICM20689::WHOAMI) {
		printf("unexpected WHO_AM_I 0x%02x", whoami);
		return PX4_ERROR;
	}

	return PX4_OK;
}

void ICM20689::RunImpl()
{
	const hrt_abstime now = get_time_now();

	switch (_state) {
	case STATE::RESET:
		// PWR_MGMT_1: Device Reset
		RegisterWrite(InvenSense_ICM20689::Register::PWR_MGMT_1, PWR_MGMT_1_BIT::DEVICE_RESET);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		// ScheduleDelayed(100_ms);
		usleep(100*1000);
		break;

	case STATE::WAIT_FOR_RESET:

		// The reset value is 0x00 for all registers other than the registers below
		//  Document Number: DS-000114 Page Page 35 of 53
		if ((RegisterRead(InvenSense_ICM20689::Register::WHO_AM_I) == InvenSense_ICM20689::WHOAMI)
		    && (RegisterRead(InvenSense_ICM20689::Register::PWR_MGMT_1) == 0x40)) {

			// Wakeup and reset digital signal path
			RegisterWrite(InvenSense_ICM20689::Register::PWR_MGMT_1, PWR_MGMT_1_BIT::CLKSEL_0);
			RegisterWrite(InvenSense_ICM20689::Register::SIGNAL_PATH_RESET, InvenSense_ICM20689::SIGNAL_PATH_RESET_BIT::ACCEL_RST | InvenSense_ICM20689::SIGNAL_PATH_RESET_BIT::TEMP_RST);
			RegisterWrite(InvenSense_ICM20689::Register::USER_CTRL, USER_CTRL_BIT::SIG_COND_RST | USER_CTRL_BIT::I2C_IF_DIS);

			// if reset succeeded then configure
			_state = STATE::CONFIGURE;
			// ScheduleDelayed(35_ms); // max 35 ms start-up time from sleep
			usleep(35*1000);

		} else {
			// RESET not complete
			// if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
			if (get_time_now()-_reset_timestamp > 1000*1000) {
				printf("Reset failed, retrying");
				_state = STATE::RESET;
				// ScheduleDelayed(100_ms);
				usleep(100*1000);

			} else {
				printf("Reset not complete, check again in 10 ms");
				// ScheduleDelayed(10_ms);
				usleep(10*1000);
			}
		}

		break;

	case STATE::CONFIGURE:
		if (Configure()) {
			// if configure succeeded then start reading from FIFO
			_state = STATE::FIFO_READ;

			if (DataReadyInterruptConfigure()) {
				_data_ready_interrupt_enabled = true;

				// backup schedule as a watchdog timeout
				// ScheduleDelayed(100_ms);
				usleep(100*1000);

			} else {
				_data_ready_interrupt_enabled = false;
				// ScheduleOnInterval(_fifo_empty_interval_us, _fifo_empty_interval_us);
				usleep(_fifo_empty_interval_us*2);
			}

			FIFOReset();

		} else {
			// CONFIGURE not complete
			if (get_time_now()-_reset_timestamp > 1000*1000) {
				printf("Configure failed, resetting");
				_state = STATE::RESET;

			} else {
				printf("Configure failed, retrying");
			}

			// ScheduleDelayed(100_ms);
			usleep(100*1000);
		}

		break;

	case STATE::FIFO_READ: {
			if (_data_ready_interrupt_enabled) {
				// scheduled from interrupt if _drdy_fifo_read_samples was set
				if (_drdy_fifo_read_samples.fetch_and(0) != _fifo_gyro_samples) {
					// perf_count(_drdy_missed_perf);
				}

				// push backup schedule back
				// ScheduleDelayed(_fifo_empty_interval_us * 2);
				usleep(_fifo_empty_interval_us * 2);
			}

			// always check current FIFO count
			bool success = false;
			const uint16_t fifo_count = FIFOReadCount();

			if (fifo_count >= InvenSense_ICM20689::FIFO::SIZE) {
				FIFOReset();
				// perf_count(_fifo_overflow_perf);

			} else if (fifo_count == 0) {
				// perf_count(_fifo_empty_perf);

			} else {
				// FIFO count (size in bytes) should be a multiple of the FIFO::DATA structure
				const uint8_t samples = (fifo_count / sizeof(InvenSense_ICM20689::FIFO::DATA) / SAMPLES_PER_TRANSFER) *
							SAMPLES_PER_TRANSFER; // round down to nearest

				if (samples > FIFO_MAX_SAMPLES) {
					// not technically an overflow, but more samples than we expected or can publish
					FIFOReset();
					// perf_count(_fifo_overflow_perf);

				} else if (samples >= 1) {
					if (FIFORead(now, samples)) {
						success = true;

						if (_failure_count > 0) {
							_failure_count--;
						}
					}
				}
			}

			if (!success) {
				_failure_count++;

				// full reset if things are failing consistently
				if (_failure_count > 10) {
					Reset();
					return;
				}
			}

			if (!success || ((get_time_now() - _last_config_check_timestamp) > 100*1000) ) {
				// check configuration registers periodically or immediately following any failure
				if (RegisterCheck(_register_cfg[_checked_register])) {
					_last_config_check_timestamp = now;
					_checked_register = (_checked_register + 1) % size_register_cfg;

				} else {
					// register check failed, force reset
					// perf_count(_bad_register_perf);
					Reset();
				}

			} else {
				// periodically update temperature (~1 Hz)
				if (get_time_now() - _temperature_update_timestamp >= 1000*1000*1000) {
					UpdateTemperature();
					_temperature_update_timestamp = now;
				}
			}
		}

		break;
	}
}

void ICM20689::ConfigureAccel()
{
	const uint8_t ACCEL_FS_SEL = RegisterRead(InvenSense_ICM20689::Register::ACCEL_CONFIG) & (InvenSense_ICM20689::Bit4 | InvenSense_ICM20689::Bit3); // [4:3] ACCEL_FS_SEL[1:0]

	switch (ACCEL_FS_SEL) {
	case InvenSense_ICM20689::ACCEL_FS_SEL_2G:
		_px4_accel.set_scale(CONSTANTS_ONE_G / 16384.f);
		_px4_accel.set_range(2.f * CONSTANTS_ONE_G);
		break;

	case InvenSense_ICM20689::ACCEL_FS_SEL_4G:
		_px4_accel.set_scale(CONSTANTS_ONE_G / 8192.f);
		_px4_accel.set_range(4.f * CONSTANTS_ONE_G);
		break;

	case InvenSense_ICM20689::ACCEL_FS_SEL_8G:
		_px4_accel.set_scale(CONSTANTS_ONE_G / 4096.f);
		_px4_accel.set_range(8.f * CONSTANTS_ONE_G);
		break;

	case InvenSense_ICM20689::ACCEL_FS_SEL_16G:
		_px4_accel.set_scale(CONSTANTS_ONE_G / 2048.f);
		_px4_accel.set_range(16.f * CONSTANTS_ONE_G);
		break;
	}
}

void ICM20689::ConfigureGyro()
{
	const uint8_t FS_SEL = RegisterRead(InvenSense_ICM20689::Register::GYRO_CONFIG) & (InvenSense_ICM20689::Bit4 | InvenSense_ICM20689::Bit3); // [4:3] FS_SEL[1:0]

	float range_dps = 0.f;

	switch (FS_SEL) {
	case FS_SEL_250_DPS:
		range_dps = 250.f;
		break;

	case FS_SEL_500_DPS:
		range_dps = 500.f;
		break;

	case FS_SEL_1000_DPS:
		range_dps = 1000.f;
		break;

	case FS_SEL_2000_DPS:
		range_dps = 2000.f;
		break;
	}

	_px4_gyro.set_scale(math_px4::radians(range_dps / 32768.f));
	_px4_gyro.set_range(math_px4::radians(range_dps));
}

void ICM20689::ConfigureSampleRate(int sample_rate)
{
	if (sample_rate == 0) {
		sample_rate = 800; // default to 800 Hz
	}

	// round down to nearest FIFO sample dt * SAMPLES_PER_TRANSFER
	const float min_interval = FIFO_SAMPLE_DT * SAMPLES_PER_TRANSFER;
	_fifo_empty_interval_us = math_px4::max(roundf((1e6f / (float)sample_rate) / min_interval) * min_interval, min_interval);

	_fifo_gyro_samples = roundf(math_px4::min((float)_fifo_empty_interval_us / (1e6f / GYRO_RATE), (float)FIFO_MAX_SAMPLES));

	// recompute FIFO empty interval (us) with actual gyro sample limit
	_fifo_empty_interval_us = _fifo_gyro_samples * (1e6f / GYRO_RATE);
}

bool ICM20689::Configure()
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

	ConfigureAccel();
	ConfigureGyro();

	return success;
}

int ICM20689::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<ICM20689 *>(arg)->DataReady();
	return 0;
}

void ICM20689::DataReady()
{
	uint32_t expected = 0;

	// at least the required number of samples in the FIFO
	if (((_drdy_count.fetch_add(1) + 1) >= _fifo_gyro_samples)
	    && _drdy_fifo_read_samples.compare_exchange(&expected, _fifo_gyro_samples)) {

		_drdy_count.store(0);
		// ScheduleNow();
	}
}

bool ICM20689::DataReadyInterruptConfigure()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	// Setup data ready on falling edge
	// return px4_arch_gpiosetevent(_drdy_gpio, false, true, true, &DataReadyInterruptCallback, this) == 0;
	return false;
}

bool ICM20689::DataReadyInterruptDisable()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	// return px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
	return false;
}

bool ICM20689::RegisterCheck(const register_config_t &reg_cfg)
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

uint8_t ICM20689::RegisterRead(InvenSense_ICM20689::Register reg)
{
	uint8_t cmd[2] {};
	cmd[0] = static_cast<uint8_t>(reg) | InvenSense_ICM20689::DIR_READ;
	transfer(cmd, cmd, sizeof(cmd));
	return cmd[1];
}

void ICM20689::RegisterWrite(InvenSense_ICM20689::Register reg, uint8_t value)
{
	uint8_t cmd[2] { (uint8_t)reg, value };
	transfer(cmd, cmd, sizeof(cmd));
}

void ICM20689::RegisterSetAndClearBits(InvenSense_ICM20689::Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);

	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}

uint16_t ICM20689::FIFOReadCount()
{
	// read FIFO count
	uint8_t fifo_count_buf[3] {};
	fifo_count_buf[0] = static_cast<uint8_t>(InvenSense_ICM20689::Register::FIFO_COUNTH) | InvenSense_ICM20689::DIR_READ;

	if (transfer(fifo_count_buf, fifo_count_buf, sizeof(fifo_count_buf)) != PX4_OK) {
		// perf_count(_bad_transfer_perf);
		return 0;
	}

	return combine(fifo_count_buf[1], fifo_count_buf[2]);
}

bool ICM20689::FIFORead(const hrt_abstime &timestamp_sample, uint8_t samples)
{
	FIFOTransferBuffer buffer{};
	const size_t transfer_size = math_px4::min(samples * sizeof(InvenSense_ICM20689::FIFO::DATA) + 1, InvenSense_ICM20689::FIFO::SIZE);

	if (transfer((uint8_t *)&buffer, (uint8_t *)&buffer, transfer_size) != PX4_OK) {
		// perf_count(_bad_transfer_perf);
		return false;
	}


	ProcessGyro(timestamp_sample, buffer.f, samples);
	return ProcessAccel(timestamp_sample, buffer.f, samples);
}

void ICM20689::FIFOReset()
{
	// perf_count(_fifo_reset_perf);

	// FIFO_EN: disable FIFO
	RegisterWrite(InvenSense_ICM20689::Register::FIFO_EN, 0);

	// USER_CTRL: reset FIFO
	RegisterSetAndClearBits(InvenSense_ICM20689::Register::USER_CTRL, USER_CTRL_BIT::FIFO_RST, USER_CTRL_BIT::FIFO_EN);

	// reset while FIFO is disabled
	_drdy_count.store(0);
	_drdy_fifo_read_samples.store(0);

	// FIFO_EN: enable both gyro and accel
	// USER_CTRL: re-enable FIFO
	for (const auto &r : _register_cfg) {
		if ((r.reg == InvenSense_ICM20689::Register::FIFO_EN) || (r.reg == InvenSense_ICM20689::Register::USER_CTRL)) {
			RegisterSetAndClearBits(r.reg, r.set_bits, r.clear_bits);
		}
	}
}

static bool fifo_accel_equal(const InvenSense_ICM20689::FIFO::DATA &f0, const InvenSense_ICM20689::FIFO::DATA &f1)
{
	return (memcmp(&f0.ACCEL_XOUT_H, &f1.ACCEL_XOUT_H, 6) == 0);
}

bool ICM20689::ProcessAccel(const hrt_abstime &timestamp_sample, const InvenSense_ICM20689::FIFO::DATA fifo[], const uint8_t samples)
{
	sensor_accel_fifo_s accel{};
	accel.timestamp_sample = timestamp_sample;
	accel.samples = 0;
	accel.dt = FIFO_SAMPLE_DT * SAMPLES_PER_TRANSFER;

	bool bad_data = false;

	// accel data is doubled in FIFO, but might be shifted
	int accel_first_sample = 1;

	if (samples >= 4) {
		if (fifo_accel_equal(fifo[0], fifo[1]) && fifo_accel_equal(fifo[2], fifo[3])) {
			// [A0, A1, A2, A3]
			//  A0==A1, A2==A3
			accel_first_sample = 1;

		} else if (fifo_accel_equal(fifo[1], fifo[2])) {
			// [A0, A1, A2, A3]
			//  A0, A1==A2, A3
			accel_first_sample = 0;

		} else {
			// no matching accel samples is an error
			bad_data = true;
			// perf_count(_bad_transfer_perf);
		}
	}

	for (int i = accel_first_sample; i < samples; i = i + SAMPLES_PER_TRANSFER) {
		int16_t accel_x = combine(fifo[i].ACCEL_XOUT_H, fifo[i].ACCEL_XOUT_L);
		int16_t accel_y = combine(fifo[i].ACCEL_YOUT_H, fifo[i].ACCEL_YOUT_L);
		int16_t accel_z = combine(fifo[i].ACCEL_ZOUT_H, fifo[i].ACCEL_ZOUT_L);

		// sensor's frame is +x forward, +y left, +z up
		//  flip y & z to publish right handed with z down (x forward, y right, z down)
		// accel.x[accel.samples] = accel_x;
		// accel.y[accel.samples] = (accel_y == INT16_MIN) ? INT16_MAX : -accel_y;
		// accel.z[accel.samples] = (accel_z == INT16_MIN) ? INT16_MAX : -accel_z;
		// 坐标轴对齐
		accel.x[accel.samples] = (accel_y == INT16_MIN) ? INT16_MAX : -accel_y;
		accel.y[accel.samples] = -accel_x;
		accel.z[accel.samples] = (accel_z == INT16_MIN) ? INT16_MAX : -accel_z;
		accel.samples++;
	}

	// _px4_accel.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf) +
				//    perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf));

	if (accel.samples > 0) {
		_px4_accel.updateFIFO(accel);
	}

	return !bad_data;
}

void ICM20689::ProcessGyro(const hrt_abstime &timestamp_sample, const InvenSense_ICM20689::FIFO::DATA fifo[], const uint8_t samples)
{
	sensor_gyro_fifo_s gyro{};
	gyro.timestamp_sample = timestamp_sample;
	gyro.samples = samples;
	gyro.dt = FIFO_SAMPLE_DT;

	for (int i = 0; i < samples; i++) {
		const int16_t gyro_x = combine(fifo[i].GYRO_XOUT_H, fifo[i].GYRO_XOUT_L);
		const int16_t gyro_y = combine(fifo[i].GYRO_YOUT_H, fifo[i].GYRO_YOUT_L);
		const int16_t gyro_z = combine(fifo[i].GYRO_ZOUT_H, fifo[i].GYRO_ZOUT_L);

		// sensor's frame is +x forward, +y left, +z up
		//  flip y & z to publish right handed with z down (x forward, y right, z down)
		// gyro.x[i] = gyro_x;
		// gyro.y[i] = (gyro_y == INT16_MIN) ? INT16_MAX : -gyro_y;
		// gyro.z[i] = (gyro_z == INT16_MIN) ? INT16_MAX : -gyro_z;
		// 坐标轴对齐
		gyro.x[i] = (gyro_y == INT16_MIN) ? INT16_MAX : -gyro_y;
		gyro.y[i] = -gyro_x;
		gyro.z[i] = (gyro_z == INT16_MIN) ? INT16_MAX : -gyro_z;
	}

	// _px4_gyro.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf) +
	// 			  perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf));

	_px4_gyro.updateFIFO(gyro);
}

void ICM20689::UpdateTemperature()
{
	// read current temperature
	uint8_t temperature_buf[3] {};
	temperature_buf[0] = static_cast<uint8_t>(InvenSense_ICM20689::Register::TEMP_OUT_H) | InvenSense_ICM20689::DIR_READ;

	if (transfer(temperature_buf, temperature_buf, sizeof(temperature_buf)) != PX4_OK) {
		// perf_count(_bad_transfer_perf);
		return;
	}

	const int16_t TEMP_OUT = combine(temperature_buf[1], temperature_buf[2]);
	const float TEMP_degC = (TEMP_OUT / InvenSense_ICM20689::TEMPERATURE_SENSITIVITY) + InvenSense_ICM20689::TEMPERATURE_OFFSET;

	if (PX4_ISFINITE(TEMP_degC)) {
		_px4_accel.set_temperature(TEMP_degC);
		_px4_gyro.set_temperature(TEMP_degC);
	}
}
