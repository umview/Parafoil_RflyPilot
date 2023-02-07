#ifndef _IST8310_H_
#define _IST8310_H_

#include <sys/types.h>
#include <unistd.h>
#include <sys/time.h>
#include <pthread.h>

#include "../posix/i2c.h"
#include "iSentek_IST8310_registers.h"



#define IST8310_DEVICE_BASE_PATH	"/dev/i2c-1"

using namespace iSentek_IST8310;

/* class IST8310 is declared */
class IST8310
{
public:
	// IST8310(I2CSPIBusOption bus_option, int bus, uint8_t addr, int bus_frequency, enum Rotation rotation = ROTATION_NONE);
	// ~IST8310() override;

	// static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
	// 				     int runtime_instance);
	// static void print_usage();
	IST8310();

	void RunImpl();

	int init();
	// void print_status() override;
	/*
	* read mag data straightforwardly
	*/
	int ist8310_read(float *magData, uint64_t *timestamp);

	int probe();

	bool Configure();

	float magData[3]; 
	uint64_t timestamp[1];

private:
	// Sensor Configuration
	struct register_config_t {
		iSentek_IST8310::Register reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

	bool Reset();

	bool RegisterCheck(const register_config_t &reg_cfg);

	uint8_t RegisterRead(iSentek_IST8310::Register reg);
	void RegisterWrite(iSentek_IST8310::Register reg, uint8_t value);
	void RegisterSetAndClearBits(iSentek_IST8310::Register reg, uint8_t setbits, uint8_t clearbits);
	uint64_t elapsed_time(uint64_t base_time);
	void set_scale(float scale);
	float measurement_scale{0};

	// PX4Magnetometer _px4_mag;

	// perf_counter_t _bad_register_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad register")};
	// perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad transfer")};
	// perf_counter_t _reset_perf{perf_alloc(PC_COUNT, MODULE_NAME": reset")};

	uint64_t _reset_timestamp{0};
	uint64_t _last_config_check_timestamp{0};
	int _failure_count{0};

	enum class STATE : uint8_t {
		RESET,
		WAIT_FOR_RESET,
		CONFIGURE,
		MEASURE,
		READ,
	} _state{STATE::RESET};

	uint8_t _checked_register{0};
	static constexpr uint8_t size_register_cfg{4};
	register_config_t _register_cfg[size_register_cfg] {
		// Register               | Set bits, Clear bits
		{ iSentek_IST8310::Register::CNTL2,        0, iSentek_IST8310::CNTL2_BIT::SRST },
		{ iSentek_IST8310::Register::CNTL3,        CNTL3_BIT::Z_16BIT | CNTL3_BIT::Y_16BIT | CNTL3_BIT::X_16BIT, 0 },
		{ iSentek_IST8310::Register::AVGCNTL,      AVGCNTL_BIT::Y_16TIMES_SET | AVGCNTL_BIT::XZ_16TIMES_SET, AVGCNTL_BIT::Y_16TIMES_CLEAR | AVGCNTL_BIT::XZ_16TIMES_CLEAR },
		{ iSentek_IST8310::Register::PDCNTL,       PDCNTL_BIT::PULSE_NORMAL, 0 },
	};
};
extern class IST8310 ist8310;
void start_ist8310(void);
void * thread_ist8310(void * ptr);



#endif