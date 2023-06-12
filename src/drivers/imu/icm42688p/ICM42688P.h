/**
 * @file ICM42688P.hpp
 *
 * Driver for the Invensense ICM42688P connected via SPI.
 *
 */

#ifndef _ICM42688P_H_
#define _ICM42688P_H_

#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <sys/time.h>
#include <math.h>
#include "InvenSense_ICM42688P_registers.h"
#include "../posix/spi.h"
#include "sensor_accel_fifo.h"
#include "sensor_gyro_fifo.h"
#include "atomic.h"
#include "limit.h"
#include "include.h"
// #include <drivers/drv_hrt.h>
// #include <lib/drivers/device/spi.h>
// #include <lib/ecl/geo/geo.h>
// #include <lib/perf/perf_counter.h>
// #include <px4_platform_common/atomic.h>
// #include <px4_platform_common/i2c_spi_buses.h>
#include "../accelerometer/PX4Accelerometer.h"
#include "../gyroscope/PX4Gyroscope.h"
#ifndef TESTMODE
	#include "configure.h"

#else

#endif

typedef uint64_t	hrt_abstime;
typedef uint32_t 	spi_drdy_gpio_t;
using namespace math_px4;
// namespace math
// {
// 	template<typename _Tp>
// 	constexpr _Tp min(_Tp a, _Tp b);

// 	template<typename _Tp>
// 	constexpr _Tp max(_Tp a, _Tp b);

//     template<typename _Tp>
//     constexpr _Tp constrain(_Tp val, _Tp min_val, _Tp max_val);
	
// 	template<typename T>
// 	constexpr T radians(T degrees);
// }



using namespace InvenSense_ICM42688P;

class ICM42688P : public SPI
{
public:
	// ICM42688P(I2CSPIBusOption bus_option, int bus, uint32_t device, enum Rotation rotation, int bus_frequency,
		//   spi_mode_e spi_mode, spi_drdy_gpio_t drdy_gpio);
	// ~ICM42688P() override;

	// static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
	// 				     int runtime_instance);
	// static void print_usage();
	// ICM42688P();
	// ~ICM42688P();

	void RunImpl();

	int icm42688p_init();
	// void print_status() override;
	
	int probe();

private:
	void exit_and_cleanup();

	// Sensor Configuration
	static constexpr float FIFO_SAMPLE_DT{1e6f / 8000.f};     // 8000 Hz accel & gyro ODR configured
	static constexpr float GYRO_RATE{1e6f / FIFO_SAMPLE_DT};
	static constexpr float ACCEL_RATE{1e6f / FIFO_SAMPLE_DT};

	// maximum FIFO samples per transfer is limited to the size of sensor_accel_fifo/sensor_gyro_fifo
	static constexpr uint32_t FIFO_MAX_SAMPLES{math_px4::min<uint32_t>(math_px4::min<uint32_t>(InvenSense_ICM42688P::FIFO::SIZE / sizeof(InvenSense_ICM42688P::FIFO::DATA), sizeof(sensor_gyro_fifo_s::x) / sizeof(sensor_gyro_fifo_s::x[0])), sizeof(sensor_accel_fifo_s::x) / sizeof(sensor_accel_fifo_s::x[0]) * (int)(GYRO_RATE / ACCEL_RATE))};

	// Transfer data
	struct FIFOTransferBuffer {
		uint8_t cmd{static_cast<uint8_t>(InvenSense_ICM42688P::Register::BANK_0::INT_STATUS) | InvenSense_ICM42688P::DIR_READ};
		uint8_t INT_STATUS{0};
		uint8_t FIFO_COUNTH{0};
		uint8_t FIFO_COUNTL{0};
		InvenSense_ICM42688P::FIFO::DATA f[FIFO_MAX_SAMPLES] {};
	};
	// ensure no struct padding
	static_assert(sizeof(FIFOTransferBuffer) == (4 + FIFO_MAX_SAMPLES *sizeof(InvenSense_ICM42688P::FIFO::DATA)));

	struct register_bank0_config_t {
		InvenSense_ICM42688P::Register::BANK_0 reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

	struct register_bank1_config_t {
		InvenSense_ICM42688P::Register::BANK_1 reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

	struct register_bank2_config_t {
		InvenSense_ICM42688P::Register::BANK_2 reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

	bool Reset();

	bool Configure();
	void ConfigureSampleRate(int sample_rate);
	void ConfigureFIFOWatermark(uint8_t samples);

	void SelectRegisterBank(enum REG_BANK_SEL_BIT bank);
	void SelectRegisterBank(InvenSense_ICM42688P::Register::BANK_0 reg) { SelectRegisterBank(REG_BANK_SEL_BIT::USER_BANK_0); }
	void SelectRegisterBank(InvenSense_ICM42688P::Register::BANK_1 reg) { SelectRegisterBank(REG_BANK_SEL_BIT::USER_BANK_1); }
	void SelectRegisterBank(InvenSense_ICM42688P::Register::BANK_2 reg) { SelectRegisterBank(REG_BANK_SEL_BIT::USER_BANK_2); }

	static int DataReadyInterruptCallback(int irq, void *context, void *arg);
	void DataReady();
	bool DataReadyInterruptConfigure();
	bool DataReadyInterruptDisable();

	template <typename T> bool RegisterCheck(const T &reg_cfg);
	template <typename T> uint8_t RegisterRead(T reg);
	template <typename T> void RegisterWrite(T reg, uint8_t value);
	template <typename T> void RegisterSetAndClearBits(T reg, uint8_t setbits, uint8_t clearbits);
	template <typename T> void RegisterSetBits(T reg, uint8_t setbits) { RegisterSetAndClearBits(reg, setbits, 0); }
	template <typename T> void RegisterClearBits(T reg, uint8_t clearbits) { RegisterSetAndClearBits(reg, 0, clearbits); }

	uint16_t FIFOReadCount();
	bool FIFORead(const hrt_abstime &timestamp_sample, uint8_t samples);
	void FIFOReset();

	void ProcessAccel(const hrt_abstime &timestamp_sample, const InvenSense_ICM42688P::FIFO::DATA fifo[], const uint8_t samples);
	void ProcessGyro(const hrt_abstime &timestamp_sample, const InvenSense_ICM42688P::FIFO::DATA fifo[], const uint8_t samples);
	bool ProcessTemperature(const InvenSense_ICM42688P::FIFO::DATA fifo[], const uint8_t samples);

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

	enum REG_BANK_SEL_BIT _last_register_bank {REG_BANK_SEL_BIT::USER_BANK_0};

	px4::atomic<uint32_t> _drdy_fifo_read_samples{0};
	bool _data_ready_interrupt_enabled{false};

	enum class STATE : uint8_t {
		RESET,
		WAIT_FOR_RESET,
		CONFIGURE,
		FIFO_READ,
	} _state{STATE::RESET};

	uint16_t _fifo_empty_interval_us{1250}; // default 1250 us / 800 Hz transfer interval
	uint32_t _fifo_gyro_samples{static_cast<uint32_t>(_fifo_empty_interval_us / (1000000 / GYRO_RATE))};

	uint8_t _checked_register_bank0{0};
	static constexpr uint8_t size_register_bank0_cfg{13};
	register_bank0_config_t _register_bank0_cfg[size_register_bank0_cfg] {
		// Register                              | Set bits, Clear bits
		{ InvenSense_ICM42688P::Register::BANK_0::INT_CONFIG,           INT_CONFIG_BIT::INT1_MODE | INT_CONFIG_BIT::INT1_DRIVE_CIRCUIT, INT_CONFIG_BIT::INT1_POLARITY },
		{ InvenSense_ICM42688P::Register::BANK_0::FIFO_CONFIG,          FIFO_CONFIG_BIT::FIFO_MODE_STOP_ON_FULL, 0 },
		{ InvenSense_ICM42688P::Register::BANK_0::PWR_MGMT0,            PWR_MGMT0_BIT::GYRO_MODE_LOW_NOISE | PWR_MGMT0_BIT::ACCEL_MODE_LOW_NOISE, 0 },
		{ InvenSense_ICM42688P::Register::BANK_0::GYRO_CONFIG0,         GYRO_CONFIG0_BIT::GYRO_ODR_8KHZ_SET, GYRO_CONFIG0_BIT::GYRO_ODR_8KHZ_CLEAR },
		{ InvenSense_ICM42688P::Register::BANK_0::ACCEL_CONFIG0,        ACCEL_CONFIG0_BIT::ACCEL_ODR_8KHZ_SET, ACCEL_CONFIG0_BIT::ACCEL_ODR_8KHZ_CLEAR },
		{ InvenSense_ICM42688P::Register::BANK_0::GYRO_CONFIG1,         0, GYRO_CONFIG1_BIT::GYRO_UI_FILT_ORD },
		{ InvenSense_ICM42688P::Register::BANK_0::GYRO_ACCEL_CONFIG0,   0, GYRO_ACCEL_CONFIG0_BIT::ACCEL_UI_FILT_BW | GYRO_ACCEL_CONFIG0_BIT::GYRO_UI_FILT_BW },
		{ InvenSense_ICM42688P::Register::BANK_0::ACCEL_CONFIG1,        0, ACCEL_CONFIG1_BIT::ACCEL_UI_FILT_ORD },
		{ InvenSense_ICM42688P::Register::BANK_0::FIFO_CONFIG1,         FIFO_CONFIG1_BIT::FIFO_WM_GT_TH | FIFO_CONFIG1_BIT::FIFO_HIRES_EN | FIFO_CONFIG1_BIT::FIFO_TEMP_EN | FIFO_CONFIG1_BIT::FIFO_GYRO_EN | FIFO_CONFIG1_BIT::FIFO_ACCEL_EN, 0 },
		{ InvenSense_ICM42688P::Register::BANK_0::FIFO_CONFIG2,         0, 0 }, // FIFO_WM[7:0] set at runtime
		{ InvenSense_ICM42688P::Register::BANK_0::FIFO_CONFIG3,         0, 0 }, // FIFO_WM[11:8] set at runtime
		{ InvenSense_ICM42688P::Register::BANK_0::INT_CONFIG0,          INT_CONFIG0_BIT::CLEAR_ON_FIFO_READ, 0 },
		{ InvenSense_ICM42688P::Register::BANK_0::INT_SOURCE0,          INT_SOURCE0_BIT::FIFO_THS_INT1_EN, 0 },
	};

	uint8_t _checked_register_bank1{0};
	static constexpr uint8_t size_register_bank1_cfg{1};
	register_bank1_config_t _register_bank1_cfg[size_register_bank1_cfg] {
		// Register                              | Set bits, Clear bits
		{ InvenSense_ICM42688P::Register::BANK_1::GYRO_CONFIG_STATIC2,  0, GYRO_CONFIG_STATIC2_BIT::GYRO_AAF_DIS | GYRO_CONFIG_STATIC2_BIT::GYRO_NF_DIS },
	};

	uint8_t _checked_register_bank2{0};
	static constexpr uint8_t size_register_bank2_cfg{1};
	register_bank2_config_t _register_bank2_cfg[size_register_bank2_cfg] {
		// Register                              | Set bits, Clear bits
		{ InvenSense_ICM42688P::Register::BANK_2::ACCEL_CONFIG_STATIC2, 0, ACCEL_CONFIG_STATIC2_BIT::ACCEL_AAF_DIS },
	};
};

extern class ICM42688P my_icm42688p;
void * thread_icm42688p(void * ptr);
void start_icm42688p(void);

#endif
