/**
 * @file ICM20689.hpp
 *
 * Driver for the Invensense ICM20689 connected via SPI.
 *
 */

#ifndef _ICM20689_H_
#define _ICM20689_H_

#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <sys/time.h>
#include <math.h>
#include "InvenSense_ICM20689_registers.h"
#include "../posix/spi.h"
#include "sensor_accel_fifo.h"
#include "sensor_gyro_fifo.h"
#include "atomic.h"
#include "limit.h"
#include "include.h"
// #include <drivers/drv_hrt.h>
// #include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
// #include <lib/drivers/device/spi.h>
// #include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
// #include <lib/ecl/geo/geo.h>
// #include <lib/perf/perf_counter.h>
// #include <px4_platform_common/atomic.h>
// #include <px4_platform_common/i2c_spi_buses.h>
#include "../accelerometer/PX4Accelerometer.h"
#include "../gyroscope/PX4Gyroscope.h"

using namespace math_px4;

using namespace InvenSense_ICM20689;

class ICM20689 : public SPI
{
public:
	// ICM20689(I2CSPIBusOption bus_option, int bus, uint32_t device, enum Rotation rotation, int bus_frequency,
	// 	 spi_mode_e spi_mode, spi_drdy_gpio_t drdy_gpio);
	// ~ICM20689() override;

	// static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					    //  int runtime_instance);
	// static void print_usage();

	void RunImpl();

	int icm20689_init();

	// int init() override;
	// void print_status() override;

	int probe();

private:
	void exit_and_cleanup();

	// Sensor Configuration
	static constexpr float FIFO_SAMPLE_DT{1e6f / 8000.f};
	static constexpr uint32_t SAMPLES_PER_TRANSFER{2};                   // ensure at least 1 new accel sample per transfer
	static constexpr float GYRO_RATE{1e6f / FIFO_SAMPLE_DT};             // 8000 Hz gyro
	static constexpr float ACCEL_RATE{GYRO_RATE / SAMPLES_PER_TRANSFER}; // 4000 Hz accel

	// maximum FIFO samples per transfer is limited to the size of sensor_accel_fifo/sensor_gyro_fifo
	static constexpr uint32_t FIFO_MAX_SAMPLES{math_px4::min(math_px4::min(InvenSense_ICM20689::FIFO::SIZE / sizeof(InvenSense_ICM20689::FIFO::DATA), sizeof(sensor_gyro_fifo_s::x) / sizeof(sensor_gyro_fifo_s::x[0])), sizeof(sensor_accel_fifo_s::x) / sizeof(sensor_accel_fifo_s::x[0]) * (int)(GYRO_RATE / ACCEL_RATE))};

	// Transfer data
	struct FIFOTransferBuffer {
		uint8_t cmd{static_cast<uint8_t>(InvenSense_ICM20689::Register::FIFO_R_W) | InvenSense_ICM20689::DIR_READ};
		InvenSense_ICM20689::FIFO::DATA f[FIFO_MAX_SAMPLES] {};
	};
	// ensure no struct padding
	static_assert(sizeof(FIFOTransferBuffer) == (1 + FIFO_MAX_SAMPLES *sizeof(InvenSense_ICM20689::FIFO::DATA)));

	struct register_config_t {
		InvenSense_ICM20689::Register reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

	

	bool Reset();

	bool Configure();
	void ConfigureAccel();
	void ConfigureGyro();
	void ConfigureSampleRate(int sample_rate);

	static int DataReadyInterruptCallback(int irq, void *context, void *arg);
	void DataReady();
	bool DataReadyInterruptConfigure();
	bool DataReadyInterruptDisable();

	bool RegisterCheck(const register_config_t &reg_cfg);

	uint8_t RegisterRead(InvenSense_ICM20689::Register reg);
	void RegisterWrite(InvenSense_ICM20689::Register reg, uint8_t value);
	void RegisterSetAndClearBits(InvenSense_ICM20689::Register reg, uint8_t setbits, uint8_t clearbits);

	uint16_t FIFOReadCount();
	bool FIFORead(const hrt_abstime &timestamp_sample, uint8_t samples);
	void FIFOReset();

	bool ProcessAccel(const hrt_abstime &timestamp_sample, const InvenSense_ICM20689::FIFO::DATA fifo[], const uint8_t samples);
	void ProcessGyro(const hrt_abstime &timestamp_sample, const InvenSense_ICM20689::FIFO::DATA fifo[], const uint8_t samples);
	void UpdateTemperature();

	const spi_drdy_gpio_t _drdy_gpio{0};

	PX4Accelerometer _px4_accel;
	PX4Gyroscope _px4_gyro;

	// perf_counter_t _bad_register_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad register")};
	// perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad transfer")};
	// perf_counter_t _fifo_empty_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO empty")};
	// perf_counter_t _fifo_overflow_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO overflow")};
	// perf_counter_t _fifo_reset_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO reset")};
	// perf_counter_t _drdy_missed_perf{nullptr};

	hrt_abstime _reset_timestamp{0};
	hrt_abstime _last_config_check_timestamp{0};
	hrt_abstime _temperature_update_timestamp{0};
	int _failure_count{0};

	px4::atomic<uint32_t> _drdy_fifo_read_samples{0};
	px4::atomic<uint32_t> _drdy_count{0};
	bool _data_ready_interrupt_enabled{false};

	enum class STATE : uint8_t {
		RESET,
		WAIT_FOR_RESET,
		CONFIGURE,
		FIFO_READ,
	};

	STATE _state{STATE::RESET};

	uint16_t _fifo_empty_interval_us{1250}; // default 1250 us / 800 Hz transfer interval
	uint32_t _fifo_gyro_samples{static_cast<uint32_t>(_fifo_empty_interval_us / (1000000 / GYRO_RATE))};

	uint8_t _checked_register{0};
	static constexpr uint8_t size_register_cfg{9};
	register_config_t _register_cfg[size_register_cfg] {
		// Register               | Set bits, Clear bits
		{ InvenSense_ICM20689::Register::CONFIG,        CONFIG_BIT::FIFO_MODE | CONFIG_BIT::DLPF_CFG_BYPASS_DLPF_8KHZ, 0 },
		{ InvenSense_ICM20689::Register::GYRO_CONFIG,   GYRO_CONFIG_BIT::FS_SEL_2000_DPS, GYRO_CONFIG_BIT::FCHOICE_B_8KHZ_BYPASS_DLPF },
		{ InvenSense_ICM20689::Register::ACCEL_CONFIG,  ACCEL_CONFIG_BIT::ACCEL_FS_SEL_16G, 0 },
		{ InvenSense_ICM20689::Register::ACCEL_CONFIG2, ACCEL_CONFIG2_BIT::ACCEL_FCHOICE_B, ACCEL_CONFIG2_BIT::FIFO_SIZE },
		{ InvenSense_ICM20689::Register::FIFO_EN,       FIFO_EN_BIT::XG_FIFO_EN | FIFO_EN_BIT::YG_FIFO_EN | FIFO_EN_BIT::ZG_FIFO_EN | FIFO_EN_BIT::ACCEL_FIFO_EN, FIFO_EN_BIT::TEMP_FIFO_EN },
		{ InvenSense_ICM20689::Register::INT_PIN_CFG,   INT_PIN_CFG_BIT::INT_LEVEL, 0 },
		{ InvenSense_ICM20689::Register::INT_ENABLE,    INT_ENABLE_BIT::DATA_RDY_INT_EN, 0 },
		{ InvenSense_ICM20689::Register::USER_CTRL,     USER_CTRL_BIT::FIFO_EN | USER_CTRL_BIT::I2C_IF_DIS, 0 },
		{ InvenSense_ICM20689::Register::PWR_MGMT_1,    PWR_MGMT_1_BIT::CLKSEL_0, PWR_MGMT_1_BIT::SLEEP },
	};
};

extern class ICM20689 my_icm20689;
void * thread_icm20689_new(void * ptr);
void start_icm20689_new(void);

#endif
