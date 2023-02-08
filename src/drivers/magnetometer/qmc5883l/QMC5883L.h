/**
 * @file QMC5883L.hpp
 *
 * Driver for the QMC5883L connected via I2C.
 *
 */

#pragma once

#include <sys/types.h>
#include <unistd.h>
#include <sys/time.h>
#include <pthread.h>
#include <stdio.h>

#include "../../posix/i2c.h"


#include "QST_QMC5883L_registers.h"



// #include <drivers/drv_hrt.h>
// #include <lib/drivers/device/i2c.h>
// #include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
// #include <lib/perf/perf_counter.h>
// #include <px4_platform_common/i2c_spi_buses.h>
#define QMC5883L_DEVICE_BASE_PATH	"/dev/i2c-1"

using namespace QST_QMC5883L;

class QMC5883L
{
public:
	// QMC5883L(I2CSPIBusOption bus_option, int bus, int bus_frequency, enum Rotation rotation = ROTATION_NONE);
	// ~QMC5883L() override;

	// static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
						//  int runtime_instance);
	// static void print_usage();

	void RunImpl();

	int init();
	// void print_status() override;

	int probe();

	bool Configure();

	float magData[3]; 
	uint64_t timestamp[1];
private:
	// Sensor Configuration
	struct register_config_t {
		QST_QMC5883L::Register reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

	bool Reset();

	bool RegisterCheck(const register_config_t &reg_cfg);

	uint8_t RegisterRead(QST_QMC5883L::Register reg);
	void RegisterWrite(QST_QMC5883L::Register reg, uint8_t value);
	void RegisterSetAndClearBits(QST_QMC5883L::Register reg, uint8_t setbits, uint8_t clearbits);

	void set_scale(float scale);
	float measurement_scale{1};
	uint8_t _retries{0};
	uint64_t _reset_timestamp{0};
	uint64_t _last_config_check_timestamp{0};
	// PX4Magnetometer _px4_mag;

	// perf_counter_t _bad_register_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad register")};
	// perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad transfer")};
	// perf_counter_t _reset_perf{perf_alloc(PC_COUNT, MODULE_NAME": reset")};

	// hrt_abstime _reset_timestamp{0};
	// hrt_abstime _last_config_check_timestamp{0};
	int _failure_count{0};

	int16_t _prev_data[3] {};

	enum class STATE : uint8_t {
		RESET,
		WAIT_FOR_RESET,
		CONFIGURE,
		READ,
	} _state{STATE::RESET};

	uint8_t _checked_register{0};
	static constexpr uint8_t size_register_cfg{3};
	register_config_t _register_cfg[size_register_cfg] {
		// Register                   | Set bits, Clear bits
		{ QST_QMC5883L::Register::CNTL1,            QST_QMC5883L::CNTL1_BIT::ODR_50HZ | QST_QMC5883L::CNTL1_BIT::Mode_Continuous, QST_QMC5883L::CNTL1_BIT::OSR_512 | QST_QMC5883L::CNTL1_BIT::RNG_2G},
		{ QST_QMC5883L::Register::CNTL2,            QST_QMC5883L::CNTL2_BIT::ROL_PNT, QST_QMC5883L::CNTL2_BIT::SOFT_RST},
		{ QST_QMC5883L::Register::SET_RESET_PERIOD, SET_RESET_PERIOD_BIT::FBR, 0 },
	};
};


extern class QMC5883L qmc5883l;
void start_qmc5883l(void);
void * thread_qmc5883l(void * ptr);
