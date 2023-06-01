#include "ICM42688P.h"
#ifndef TESTMODE
	// #include "config.h"
	// #include "../../msg_def.h"
	#include "systime.h"
class adaptive_delay_typedef icm42688p_delay(0.5,15,400);

#else
	#include "../../px4lib/test_time.h"
	#include "../../px4lib/limit.h"

#endif
#define spi0_path        "/dev/spidev0.0"//前面的0表示SPI1，后面的0表示片选引脚0
// using namespace time_literals;
using namespace math_px4;

/***************using example***************/
/* using example */
class ICM42688P my_icm42688p;

void * thread_icm42688p(void * ptr)
{
    timespec thread_icm42688p_sleep;
    thread_icm42688p_sleep.tv_sec = 0;
    thread_icm42688p_sleep.tv_nsec = 1*1000*1000;//1250us
    core_bind(IMU_CORE);
    rflypilot_config_typedef config;
    rflypilot_config_msg.read(&config);
    thread_msg_typedef _imu_thread;
	#if USING_THREAD_SYNC
	char count_for_att = 0;
	#endif
    while (1)
    {
     //  if(TASK_SCHEDULE_DEBUG)
     //  {
    	// _imu_thread.timestamp = get_time_now();
    	// imu_thread_msg.publish(&_imu_thread);
     //  }

		my_icm42688p.RunImpl();
		#if USING_THREAD_SYNC
		if(count_for_att == IMU_ATT)
		{
			pthread_mutex_lock(&mutex);  
			pthread_cond_signal(&cond);   
			pthread_mutex_unlock(&mutex);
			count_for_att = 0;
		}
		count_for_att++;
		#endif
        // nanosleep(&thread_icm42688p_sleep,NULL);
        delay_us_combined((uint64_t)(1000000.f / config.imu_rate),&scheduler.imu_flag,&icm42688p_delay);

    }
    return NULL;
}
void start_icm42688p(void)
{
    printf("start icm42688p\n");
    int ret = 0;

    ret = my_icm42688p.icm42688p_init();
    ret = my_icm42688p.probe();
    /* thread create */
    int rc;
    pthread_t thr_icm42688p;
    if(rc = pthread_create(&thr_icm42688p, NULL, thread_icm42688p, NULL))
    {
		printf(" thread cretated failed %d \n", rc);
    }
    printf("icm42688p process pid : %d\n", (int)getpid()); 
}


/****************************************************/

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

/* Define PX4_ISFINITE */
#ifdef __cplusplus
constexpr bool PX4_ISFINITE(float x) { return __builtin_isfinite(x); }
constexpr bool PX4_ISFINITE(double x) { return __builtin_isfinite(x); }
#endif /* __cplusplus */

// ICM42688P::ICM42688P(I2CSPIBusOption bus_option, int bus, uint32_t device, enum Rotation rotation, int bus_frequency,
// 		     spi_mode_e spi_mode, spi_drdy_gpio_t drdy_gpio) :
// 	SPI(DRV_IMU_DEVTYPE_ICM42688P, MODULE_NAME, bus, device, spi_mode, bus_frequency),
// 	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
// 	_drdy_gpio(drdy_gpio),
// 	_px4_accel(get_device_id(), rotation),
// 	_px4_gyro(get_device_id(), rotation)
// {
// 	if (drdy_gpio != 0) {
// 		_drdy_missed_perf = perf_alloc(PC_COUNT, MODULE_NAME": DRDY missed");
// 	}

// 	ConfigureSampleRate(_px4_gyro.get_max_rate_hz());
// }
// ICM42688P::ICM42688P()
// {

// }
// ICM42688P::~ICM42688P()
// {
	// perf_free(_bad_register_perf);
	// perf_free(_bad_transfer_perf);
	// perf_free(_fifo_empty_perf);
	// perf_free(_fifo_overflow_perf);
	// perf_free(_fifo_reset_perf);
	// perf_free(_drdy_missed_perf);
// }

int ICM42688P::icm42688p_init()
{
	int ret = SPI::init(spi0_path, SPIDEV_MODE3, 24000000);

	if (ret != PX4_OK) {
		printf("SPI::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool ICM42688P::Reset()
{
	_state = STATE::RESET;
	DataReadyInterruptDisable();
	// ScheduleClear();
	// ScheduleNow();
	return true;
}

void ICM42688P::exit_and_cleanup()
{
	DataReadyInterruptDisable();
	// I2CSPIDriverBase::exit_and_cleanup();
}

// void ICM42688P::print_status()
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

int ICM42688P::probe()
{
	const uint8_t whoami = RegisterRead(InvenSense_ICM42688P::Register::BANK_0::WHO_AM_I);

	if (whoami != WHOAMI) {
		printf("unexpected WHO_AM_I 0x%02x\n", whoami);
		return PX4_ERROR;
	}else{
		printf("icm42688p WHO_AM_I 0x%02x\n", whoami);
	}

	return PX4_OK;
}

void ICM42688P::RunImpl()
{
	const hrt_abstime now = get_time_now();
	// if (_state == STATE::RESET)printf("state is RESET\n");
	// else if (_state == STATE::FIFO_READ)printf("state is FIFO_READ\n");
	// else if (_state == STATE::CONFIGURE)printf("state is CONFIGURE\n");
	// else if (_state == STATE::WAIT_FOR_RESET)printf("state is WAIT_FOR_RESET\n");
	// else printf("state is unknown\n");
	switch (_state) {
	case STATE::RESET:
		// DEVICE_CONFIG: Software reset configuration
		RegisterWrite(InvenSense_ICM42688P::Register::BANK_0::DEVICE_CONFIG, DEVICE_CONFIG_BIT::SOFT_RESET_CONFIG);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		// ScheduleDelayed(1_ms); // wait 1 ms for soft reset to be effective
		usleep(1*1000);
		break;

	case STATE::WAIT_FOR_RESET:
		if ((RegisterRead(InvenSense_ICM42688P::Register::BANK_0::WHO_AM_I) == WHOAMI)
		    && (RegisterRead(InvenSense_ICM42688P::Register::BANK_0::DEVICE_CONFIG) == 0x00)
		    && (RegisterRead(InvenSense_ICM42688P::Register::BANK_0::INT_STATUS) & INT_STATUS_BIT::RESET_DONE_INT)) {

			// Wakeup accel and gyro and schedule remaining configuration
			RegisterWrite(InvenSense_ICM42688P::Register::BANK_0::PWR_MGMT0, PWR_MGMT0_BIT::GYRO_MODE_LOW_NOISE | PWR_MGMT0_BIT::ACCEL_MODE_LOW_NOISE);
			_state = STATE::CONFIGURE;
			// ScheduleDelayed(30_ms); // 30 ms gyro startup time, 10 ms accel from sleep to valid data
			usleep(30*1000);

		} else {
			// RESET not complete
			if (get_time_now()-_reset_timestamp > 1000*1000) {
				printf("ICM42688p: Reset failed, retrying");
				_state = STATE::RESET;
				// ScheduleDelayed(100_ms);
				usleep(100*1000);

			} else {
				printf("ICM42688p: Reset not complete, check again in 10 ms");
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
			if (get_time_now()-(_reset_timestamp) > 1000*1000) {
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
			uint32_t samples = 0;

			if (_data_ready_interrupt_enabled) {
				// scheduled from interrupt if _drdy_fifo_read_samples was set as expected
				if (_drdy_fifo_read_samples.fetch_and(0) != _fifo_gyro_samples) {
					// perf_count(_drdy_missed_perf);

				} else {
					samples = _fifo_gyro_samples;
				}

				// push backup schedule back
				// ScheduleDelayed(_fifo_empty_interval_us * 2);
				usleep(_fifo_empty_interval_us * 2);
			}

			if (samples == 0) {
				// check current FIFO count
				const uint16_t fifo_count = FIFOReadCount();
				// printf("fificont: %d\n",fifo_count);
				if (fifo_count >= FIFO::SIZE) {
					FIFOReset();
					// printf("fifo overflow, reset fifo!\n");
					// perf_count(_fifo_overflow_perf);

				} else if (fifo_count == 0) {
					// perf_count(_fifo_empty_perf);

				} else {
					// FIFO count (size in bytes)
					// sizeof(FIFO::DATA) is 20;
					samples = (fifo_count / sizeof(FIFO::DATA));
					// printf("FIFO_MAX_SAMPLES is %d and samples is %d\n", FIFO_MAX_SAMPLES, samples);
					if (samples > FIFO_MAX_SAMPLES) {
						// not technically an overflow, but more samples than we expected or can publish
						FIFOReset();
						// perf_count(_fifo_overflow_perf);
						// printf("fifo overflow by samples > FIFO_MAX_SAMPLES\n");
						samples = 0;
					}
				}
			}

			bool success = false;

			if (samples >= 1) {
				if (FIFORead(now, samples)) {
					success = true;

					if (_failure_count > 0) {
						_failure_count--;
					}
				}else{
					// printf("fifo read fail\n");
				}
			}else{
				// printf("samples <1 and it is %d\n",samples);
			}

			if (!success) {
				_failure_count++;

				// full reset if things are failing consistently
				if (_failure_count > 10) {
					Reset();
					return;
				}
			}

			// check configuration registers periodically or immediately following any failure
			if (RegisterCheck(_register_bank0_cfg[_checked_register_bank0])
			    && RegisterCheck(_register_bank1_cfg[_checked_register_bank1])
			    && RegisterCheck(_register_bank2_cfg[_checked_register_bank2])
			   ) {
				_last_config_check_timestamp = now;
				_checked_register_bank0 = (_checked_register_bank0 + 1) % size_register_bank0_cfg;
				_checked_register_bank1 = (_checked_register_bank1 + 1) % size_register_bank1_cfg;
				_checked_register_bank2 = (_checked_register_bank2 + 1) % size_register_bank2_cfg;

			} else {
				// register check failed, force reset
				// perf_count(_bad_register_perf);
				printf("register check failed, force reset\n");
				Reset();
			}
		}

		break;
	}
}

void ICM42688P::ConfigureSampleRate(int sample_rate)
{
	// round down to nearest FIFO sample dt
	const float min_interval = FIFO_SAMPLE_DT;
	_fifo_empty_interval_us = math_px4::max<float>(roundf((1e6f / (float)sample_rate) / min_interval) * min_interval, min_interval);

	_fifo_gyro_samples = roundf(math_px4::min((float)_fifo_empty_interval_us / (1e6f / GYRO_RATE), (float)FIFO_MAX_SAMPLES));

	// recompute FIFO empty interval (us) with actual gyro sample limit
	_fifo_empty_interval_us = _fifo_gyro_samples * (1e6f / GYRO_RATE);

	ConfigureFIFOWatermark(_fifo_gyro_samples);
}

void ICM42688P::ConfigureFIFOWatermark(uint8_t samples)
{
	// FIFO watermark threshold in number of bytes
	const uint16_t fifo_watermark_threshold = samples * sizeof(FIFO::DATA);

	for (auto &r : _register_bank0_cfg) {
		if (r.reg == InvenSense_ICM42688P::Register::BANK_0::FIFO_CONFIG2) {
			// FIFO_WM[7:0]  FIFO_CONFIG2
			r.set_bits = fifo_watermark_threshold & 0xFF;

		} else if (r.reg == InvenSense_ICM42688P::Register::BANK_0::FIFO_CONFIG3) {
			// FIFO_WM[11:8] FIFO_CONFIG3
			r.set_bits = (fifo_watermark_threshold >> 8) & 0x0F;
		}
	}
}

void ICM42688P::SelectRegisterBank(enum REG_BANK_SEL_BIT bank)
{
	if (bank != _last_register_bank) {
		// select BANK_0
		uint8_t cmd_bank_sel[2] {};
		cmd_bank_sel[0] = static_cast<uint8_t>(InvenSense_ICM42688P::Register::BANK_0::REG_BANK_SEL);
		cmd_bank_sel[1] = bank;
		transfer(cmd_bank_sel, cmd_bank_sel, sizeof(cmd_bank_sel));

		_last_register_bank = bank;
	}
}

bool ICM42688P::Configure()
{
	// first set and clear all configured register bits
	for (const auto &reg_cfg : _register_bank0_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	for (const auto &reg_cfg : _register_bank1_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	for (const auto &reg_cfg : _register_bank2_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	// now check that all are configured
	bool success = true;

	for (const auto &reg_cfg : _register_bank0_cfg) {
		if (!RegisterCheck(reg_cfg)) {
			success = false;
		}
	}

	for (const auto &reg_cfg : _register_bank1_cfg) {
		if (!RegisterCheck(reg_cfg)) {
			success = false;
		}
	}

	for (const auto &reg_cfg : _register_bank2_cfg) {
		if (!RegisterCheck(reg_cfg)) {
			success = false;
		}
	}

	// 20-bits data format used
	//  the only FSR settings that are operational are ±2000dps for gyroscope and ±16g for accelerometer
	_px4_accel.set_range(16.f * CONSTANTS_ONE_G);
	_px4_gyro.set_range(math_px4::radians(2000.f));

	return success;
}

int ICM42688P::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<ICM42688P *>(arg)->DataReady();
	return 0;
}

void ICM42688P::DataReady()
{
	uint32_t expected = 0;

	if (_drdy_fifo_read_samples.compare_exchange(&expected, _fifo_gyro_samples)) {
		// ScheduleNow();
	}
}

bool ICM42688P::DataReadyInterruptConfigure()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	// Setup data ready on falling edge
	// return px4_arch_gpiosetevent(_drdy_gpio, false, true, true, &DataReadyInterruptCallback, this) == 0;
	return false;
}

bool ICM42688P::DataReadyInterruptDisable()
{
	if (_drdy_gpio == 0) {
		return false;
	}
	//在posix环境下该函数应该始终返回-1, -1!=0, 所以这里返回假false, (分析结果源于PX4 1.12源码, 该函数只在特定平台上生效)
	// return px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
	return false;
}

template <typename T>
bool ICM42688P::RegisterCheck(const T &reg_cfg)
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

template <typename T>
uint8_t ICM42688P::RegisterRead(T reg)
{
	uint8_t cmd[2] {};
	cmd[0] = static_cast<uint8_t>(reg) | DIR_READ;
	SelectRegisterBank(reg);
	transfer(cmd, cmd, sizeof(cmd));
	return cmd[1];
}

template <typename T>
void ICM42688P::RegisterWrite(T reg, uint8_t value)
{
	uint8_t cmd[2] { (uint8_t)reg, value };
	SelectRegisterBank(reg);
	transfer(cmd, cmd, sizeof(cmd));
}

template <typename T>
void ICM42688P::RegisterSetAndClearBits(T reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);

	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}

uint16_t ICM42688P::FIFOReadCount()
{
	// read FIFO count
	uint8_t fifo_count_buf[3] {};
	fifo_count_buf[0] = static_cast<uint8_t>(InvenSense_ICM42688P::Register::BANK_0::FIFO_COUNTH) | DIR_READ;
	SelectRegisterBank(REG_BANK_SEL_BIT::USER_BANK_0);

	if (transfer(fifo_count_buf, fifo_count_buf, sizeof(fifo_count_buf)) != PX4_OK) {
		// perf_count(_bad_transfer_perf);
		return 0;
	}

	return combine(fifo_count_buf[1], fifo_count_buf[2]);
}

bool ICM42688P::FIFORead(const hrt_abstime &timestamp_sample, uint8_t samples)
{
	FIFOTransferBuffer buffer{};
	// !!!! 此处曾报错  故作此修改
// RflyPilot_Project/RflyPilot/src/drivers/imu/icm42688p/ICM42688P.cpp: In member function ‘bool ICM42688P::FIFORead(const hrt_abstime&, uint8_t)’:
// RflyPilot_Project/RflyPilot/src/drivers/imu/icm42688p/ICM42688P.cpp:524:89: error: no matching function for call to ‘min(long unsigned int, const uint32_t&)’
//   const size_t transfer_size = math_px4::min(samples * sizeof(FIFO::DATA) + 4, FIFO::SIZE);
//                                                                                          ^
// In file included from /RflyPilot_Project/RflyPilot/src/drivers/imu/icm42688p/ICM42688P.h:22:0,
//                  from RflyPilot_Project/RflyPilot/src/drivers/imu/icm42688p/ICM42688P.cpp:1:
// RflyPilot_Project/RflyPilot/src/drivers/imu/icm42688p/limit.h:12:16: note: candidate: template<class _Tp> constexpr _Tp math_px4::min(_Tp, _Tp)
//	const size_t transfer_size = math_px4::min(samples * sizeof(FIFO::DATA) + 4, (long unsigned int)(FIFO::SIZE));
	const size_t transfer_size = math_px4::min(samples * sizeof(FIFO::DATA) + 4, (FIFO::SIZE));

	SelectRegisterBank(REG_BANK_SEL_BIT::USER_BANK_0);

	if (transfer((uint8_t *)&buffer, (uint8_t *)&buffer, transfer_size) != PX4_OK) {
		// perf_count(_bad_transfer_perf);
		printf("_bad_transfer_perf\n");
		return false;
	}

	if (buffer.INT_STATUS & INT_STATUS_BIT::FIFO_FULL_INT) {
		// perf_count(_fifo_overflow_perf);
		// printf("_fifo_overflow_perf1\n");
		FIFOReset();
		return false;
	}

	const uint16_t fifo_count_bytes = combine(buffer.FIFO_COUNTH, buffer.FIFO_COUNTL);

	if (fifo_count_bytes >= FIFO::SIZE) {
		// perf_count(_fifo_overflow_perf);
		printf("_fifo_overflow_perf2\n");
		FIFOReset();
		return false;
	}

	const uint8_t fifo_count_samples = fifo_count_bytes / sizeof(FIFO::DATA);

	if (fifo_count_samples == 0) {
		// perf_count(_fifo_empty_perf);
		printf("_fifo_empty_perf\n");
		return false;
	}

	// check FIFO header in every sample
	uint8_t valid_samples = 0;
	// printf("???? is %d\n",math_px4::min(samples, fifo_count_samples));
	for (int i = 0; i < math_px4::min(samples, fifo_count_samples); i++) {
		bool valid = true;

		// With FIFO_ACCEL_EN and FIFO_GYRO_EN header should be 8’b_0110_10xx
		const uint8_t FIFO_HEADER = buffer.f[i].FIFO_Header;

		if (FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_MSG) {
			// FIFO sample empty if HEADER_MSG set
			valid = false;
			// printf("1\n");

		} else if (!(FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_ACCEL)) {
			// accel bit not set
			valid = false;
			// printf("2\n");

		} else if (!(FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_GYRO)) {
			// gyro bit not set
			valid = false;
			// printf("3\n");

		} else if (!(FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_20)) {
			// Packet does not contain a new and valid extended 20-bit data
			valid = false;
			// printf("4\n");

		} else if (FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_ODR_ACCEL) {
			// accel ODR changed
			valid = false;
			// printf("5\n");

		} else if (FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_ODR_GYRO) {
			// gyro ODR changed
			valid = false;
			// printf("6\n");
		}

		if (valid) {
			valid_samples++;

		} else {
			// perf_count(_bad_transfer_perf);
			break;
		}
	}

	if (valid_samples > 0) {
		// printf("valid_samples is %d\n",valid_samples);
		if (ProcessTemperature(buffer.f, valid_samples)) {
			ProcessGyro(timestamp_sample, buffer.f, valid_samples);
			ProcessAccel(timestamp_sample, buffer.f, valid_samples);
			return true;
		}else{
			// printf("ProcessTemperature run fail\n");
		}
	}

	return false;
}

void ICM42688P::FIFOReset()
{
	// perf_count(_fifo_reset_perf);
	
	// SIGNAL_PATH_RESET: FIFO flush
	RegisterSetBits(InvenSense_ICM42688P::Register::BANK_0::SIGNAL_PATH_RESET, SIGNAL_PATH_RESET_BIT::FIFO_FLUSH);

	// reset while FIFO is disabled
	_drdy_fifo_read_samples.store(0);
}

static constexpr int32_t reassemble_20bit(const uint32_t a, const uint32_t b, const uint32_t c)
{
	// 0xXXXAABBC
	uint32_t high   = ((a << 12) & 0x000FF000);
	uint32_t low    = ((b << 4)  & 0x00000FF0);
	uint32_t lowest = (c         & 0x0000000F);

	uint32_t x = high | low | lowest;

	if (a & InvenSense_ICM42688P::Bit7) {
		// sign extend
		x |= 0xFFF00000u;
	}

	return static_cast<int32_t>(x);
}

void ICM42688P::ProcessAccel(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples)
{
	sensor_accel_fifo_s accel{};
	accel.timestamp_sample = timestamp_sample;
	accel.samples = 0;
	accel.dt = FIFO_SAMPLE_DT;

	// 18-bits of accelerometer data
	bool scale_20bit = false;

	// first pass
	for (int i = 0; i < samples; i++) {
		// 20 bit hires mode
		// Sign extension + Accel [19:12] + Accel [11:4] + Accel [3:2] (20 bit extension byte)
		// Accel data is 18 bit ()
		int32_t accel_x = reassemble_20bit(fifo[i].ACCEL_DATA_X1, fifo[i].ACCEL_DATA_X0,
						   fifo[i].Ext_Accel_X_Gyro_X & 0xF0 >> 4);
		int32_t accel_y = reassemble_20bit(fifo[i].ACCEL_DATA_Y1, fifo[i].ACCEL_DATA_Y0,
						   fifo[i].Ext_Accel_Y_Gyro_Y & 0xF0 >> 4);
		int32_t accel_z = reassemble_20bit(fifo[i].ACCEL_DATA_Z1, fifo[i].ACCEL_DATA_Z0,
						   fifo[i].Ext_Accel_Z_Gyro_Z & 0xF0 >> 4);

		// sample invalid if -524288
		if (accel_x != -524288 && accel_y != -524288 && accel_z != -524288) {
			// check if any values are going to exceed int16 limits
			static constexpr int16_t max_accel = INT16_MAX;
			static constexpr int16_t min_accel = INT16_MIN;

			if (accel_x >= max_accel || accel_x <= min_accel) {
				scale_20bit = true;
			}

			if (accel_y >= max_accel || accel_y <= min_accel) {
				scale_20bit = true;
			}

			if (accel_z >= max_accel || accel_z <= min_accel) {
				scale_20bit = true;
			}

			// shift by 2 (2 least significant bits are always 0)
			accel.x[accel.samples] = accel_x / 4;
			accel.y[accel.samples] = accel_y / 4;
			accel.z[accel.samples] = accel_z / 4;
			accel.samples++;
		}
	}

	if (!scale_20bit) {
		// if highres enabled accel data is always 8192 LSB/g
		_px4_accel.set_scale(CONSTANTS_ONE_G / 8192.f);

	} else {
		// 20 bit data scaled to 16 bit (2^4)
		for (int i = 0; i < samples; i++) {
			// 20 bit hires mode
			// Sign extension + Accel [19:12] + Accel [11:4] + Accel [3:2] (20 bit extension byte)
			// Accel data is 18 bit ()
			int16_t accel_x = combine(fifo[i].ACCEL_DATA_X1, fifo[i].ACCEL_DATA_X0);
			int16_t accel_y = combine(fifo[i].ACCEL_DATA_Y1, fifo[i].ACCEL_DATA_Y0);
			int16_t accel_z = combine(fifo[i].ACCEL_DATA_Z1, fifo[i].ACCEL_DATA_Z0);

			accel.x[i] = accel_x;
			accel.y[i] = accel_y;
			accel.z[i] = accel_z;
		}

		_px4_accel.set_scale(CONSTANTS_ONE_G / 2048.f);
	}

	// correct frame for publication
	for (int i = 0; i < accel.samples; i++) {
		// sensor's frame is +x forward, +y left, +z up
		//  flip y & z to publish right handed with z down (x forward, y right, z down)
		accel.x[i] = accel.x[i];
		accel.y[i] = (accel.y[i] == INT16_MIN) ? INT16_MAX : -accel.y[i];
		accel.z[i] = (accel.z[i] == INT16_MIN) ? INT16_MAX : -accel.z[i];
	}

	// _px4_accel.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf) +
				//    perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf));

	if (accel.samples > 0) {
		_px4_accel.updateFIFO(accel);
		// printf("accel x: %d, accel y: %d, accel z: %d\n", accel.x[0], accel.y[0], accel.z[0]);
	}
}

void ICM42688P::ProcessGyro(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples)
{
	sensor_gyro_fifo_s gyro{};
	gyro.timestamp_sample = timestamp_sample;
	gyro.samples = 0;
	gyro.dt = FIFO_SAMPLE_DT;

	// 20-bits of gyroscope data
	bool scale_20bit = false;

	// first pass
	for (int i = 0; i < samples; i++) {
		// 20 bit hires mode
		// Gyro [19:12] + Gyro [11:4] + Gyro [3:0] (bottom 4 bits of 20 bit extension byte)
		int32_t gyro_x = reassemble_20bit(fifo[i].GYRO_DATA_X1, fifo[i].GYRO_DATA_X0, fifo[i].Ext_Accel_X_Gyro_X & 0x0F);
		int32_t gyro_y = reassemble_20bit(fifo[i].GYRO_DATA_Y1, fifo[i].GYRO_DATA_Y0, fifo[i].Ext_Accel_Y_Gyro_Y & 0x0F);
		int32_t gyro_z = reassemble_20bit(fifo[i].GYRO_DATA_Z1, fifo[i].GYRO_DATA_Z0, fifo[i].Ext_Accel_Z_Gyro_Z & 0x0F);

		// check if any values are going to exceed int16 limits
		static constexpr int16_t max_gyro = INT16_MAX;
		static constexpr int16_t min_gyro = INT16_MIN;

		if (gyro_x >= max_gyro || gyro_x <= min_gyro) {
			scale_20bit = true;
		}

		if (gyro_y >= max_gyro || gyro_y <= min_gyro) {
			scale_20bit = true;
		}

		if (gyro_z >= max_gyro || gyro_z <= min_gyro) {
			scale_20bit = true;
		}

		gyro.x[gyro.samples] = gyro_x / 2;
		gyro.y[gyro.samples] = gyro_y / 2;
		gyro.z[gyro.samples] = gyro_z / 2;
		gyro.samples++;
	}

	if (!scale_20bit) {
		// if highres enabled gyro data is always 131 LSB/dps
		_px4_gyro.set_scale(math_px4::radians(1.f / 131.f));

	} else {
		// 20 bit data scaled to 16 bit (2^4)
		for (int i = 0; i < samples; i++) {
			gyro.x[i] = combine(fifo[i].GYRO_DATA_X1, fifo[i].GYRO_DATA_X0);
			gyro.y[i] = combine(fifo[i].GYRO_DATA_Y1, fifo[i].GYRO_DATA_Y0);
			gyro.z[i] = combine(fifo[i].GYRO_DATA_Z1, fifo[i].GYRO_DATA_Z0);
		}

		_px4_gyro.set_scale(math_px4::radians(2000.f / 32768.f));
	}

	// correct frame for publication
	for (int i = 0; i < gyro.samples; i++) {
		// sensor's frame is +x forward, +y left, +z up
		//  flip y & z to publish right handed with z down (x forward, y right, z down)
		gyro.x[i] = gyro.x[i];
		gyro.y[i] = (gyro.y[i] == INT16_MIN) ? INT16_MAX : -gyro.y[i];
		gyro.z[i] = (gyro.z[i] == INT16_MIN) ? INT16_MAX : -gyro.z[i];
	}

	// _px4_gyro.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf) +
	// 			  perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf));

	if (gyro.samples > 0) {
		_px4_gyro.updateFIFO(gyro);
		// printf("gyro x: %d, gyro y: %d, gyro z: %d\n", gyro.x[0], gyro.y[0], gyro.z[0]);
	}
}

bool ICM42688P::ProcessTemperature(const FIFO::DATA fifo[], const uint8_t samples)
{
	int16_t temperature[FIFO_MAX_SAMPLES];
	float temperature_sum{0};

	int valid_samples = 0;

	for (int i = 0; i < samples; i++) {
		const int16_t t = combine(fifo[i].TEMP_DATA1, fifo[i].TEMP_DATA0);

		// sample invalid if -32768
		if (t != -32768) {
			temperature_sum += t;
			temperature[valid_samples] = t;
			valid_samples++;
		}
	}

	if (valid_samples > 0) {
		const float temperature_avg = temperature_sum / valid_samples;

		for (int i = 0; i < valid_samples; i++) {
			// temperature changing wildly is an indication of a transfer error
			if (fabsf(temperature[i] - temperature_avg) > 1000) {
				// perf_count(_bad_transfer_perf);
				return false;
			}
		}

		// use average temperature reading
		const float TEMP_degC = (temperature_avg / TEMPERATURE_SENSITIVITY) + TEMPERATURE_OFFSET;

		if (PX4_ISFINITE(TEMP_degC)) {
			_px4_accel.set_temperature(TEMP_degC);
			_px4_gyro.set_temperature(TEMP_degC);
			return true;

		} else {
			// perf_count(_bad_transfer_perf);
		}
	}

	return false;
}
