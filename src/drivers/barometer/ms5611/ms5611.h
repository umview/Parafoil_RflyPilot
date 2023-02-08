#ifndef _MS5611_H_
#define _MS5611_H_

#include "posix/i2c.h"
#include <sys/time.h>
#include <sys/errno.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/time.h>
#include <pthread.h>
#ifndef TESTMODE
#include "systime.h"
#else
#include "../../px4lib/test_time.h"
#endif

#define DRV_BARO_DEVTYPE_MS5611		0x3D
#define DRV_BARO_DEVTYPE_MS5607		0x3E
#define ADDR_RESET_CMD			0x1E	/* write to this address to reset chip */
#define ADDR_PROM_SETUP			0xA0	/* address of 8x 2 bytes factory and calibration data */

/* interface ioctls */
#define IOCTL_RESET			2
#define IOCTL_MEASURE			3

#define MS5611_DEVICE_BASE_PATH	"/dev/i2c-1"
#define MS5611_ADDRESS_1		0x76	/* address select pins pulled high (PX4FMU series v1.6+) */
#define MS5611_ADDRESS_2		0x77    /* address select pins pulled low (PX4FMU prototypes) */

enum MS56XX_DEVICE_TYPES {
	MS5611_DEVICE	= 5611,
	MS5607_DEVICE	= 5607,
};

/* helper macro for handling report buffer indices */
#define INCREMENT(_x, _lim)	do { __typeof__(_x) _tmp = _x+1; if (_tmp >= _lim) _tmp = 0; _x = _tmp; } while(0)

/* helper macro for arithmetic - returns the square of the argument */
#define POW2(_x)		((_x) * (_x))

/*
 * MS5611/MS5607 internal constants and data structures.
 */
#define ADDR_CMD_CONVERT_D1_OSR256		0x40	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D1_OSR512		0x42	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D1_OSR1024		0x44	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D1_OSR2048		0x46	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D1_OSR4096		0x48	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D2_OSR256		0x50	/* write to this address to start temperature conversion */
#define ADDR_CMD_CONVERT_D2_OSR512		0x52	/* write to this address to start temperature conversion */
#define ADDR_CMD_CONVERT_D2_OSR1024		0x54	/* write to this address to start temperature conversion */
#define ADDR_CMD_CONVERT_D2_OSR2048		0x56	/* write to this address to start temperature conversion */
#define ADDR_CMD_CONVERT_D2_OSR4096		0x58	/* write to this address to start temperature conversion */

/*
  use an OSR of 1024 to reduce the self-heating effect of the
  sensor. Information from MS tells us that some individual sensors
  are quite sensitive to this effect and that reducing the OSR can
  make a big difference
 */
#define ADDR_CMD_CONVERT_D1			ADDR_CMD_CONVERT_D1_OSR1024
#define ADDR_CMD_CONVERT_D2			ADDR_CMD_CONVERT_D2_OSR1024

/*
 * Maximum internal conversion time for OSR 1024 is 2.28 ms. We set an update
 * rate of 100Hz which is be very safe not to read the ADC before the
 * conversion finished
 */
#define MS5611_CONVERSION_INTERVAL	10000	/* microseconds */
#define MS5611_MEASUREMENT_RATIO	3	/* pressure measurements per temperature measurement */



// namespace ms5611
// {

/**
 * Calibration PROM as reported by the device.
 */
#pragma pack(push,1)
struct prom_s {
	uint16_t factory_setup;
	uint16_t c1_pressure_sens;
	uint16_t c2_pressure_offset;
	uint16_t c3_temp_coeff_pres_sens;
	uint16_t c4_temp_coeff_pres_offset;
	uint16_t c5_reference_temp;
	uint16_t c6_temp_coeff_temp;
	uint16_t serial_and_crc;
};

/**
 * Grody hack for crc4()
 */
union prom_u {
	uint16_t c[8];
	prom_s s;
};
#pragma pack(pop)

extern bool crc4(uint16_t *n_prom);

// } /* namespace */

class MS5611_I2C
{
public:
	// MS5611_I2C(uint8_t bus, ms5611::prom_u &prom_buf, int bus_frequency);
	// ~MS5611_I2C() override = default;
	// MS5611_I2C();
	int	read(unsigned offset, void *data, unsigned count);
	int	ioctl(unsigned operation, unsigned &arg);
	prom_u	_prom;
	int	probe();

protected:
	

private:

	int		_probe_address(uint8_t address);

	/**
	 * Send a reset command to the MS5611.
	 *
	 * This is required after any bus reset.
	 */
	int		_reset();

	/**
	 * Send a measure command to the MS5611.
	 *
	 * @param addr		Which address to use for the measure operation.
	 */
	int		_measure(unsigned addr);

	/**
	 * Read the MS5611 PROM
	 *
	 * @return		PX4_OK if the PROM reads successfully.
	 */
	int		_read_prom();

};



class MS5611
{
public:
	// MS5611(device::Device *interface, ms5611::prom_u &prom_buf, enum MS56XX_DEVICE_TYPES device_type,
	    //    I2CSPIBusOption bus_option, int bus);
	// ~MS5611() override;
	MS5611();
    float _temperature;
    float _pressure;
    uint64_t _timestamp_sample;

	static void print_usage();

	int		init();

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 *
	 * This is the heart of the measurement state machine.  This function
	 * alternately starts a measurement, or collects the data from the
	 * previous measurement.
	 *
	 * When the interval between measurements is greater than the minimum
	 * measurement interval, a gap is inserted between collection
	 * and measurement to provide the most recent measurement possible
	 * at the next interval.
	 */
	void			RunImpl();

protected:
	// void print_status() override;

	// PX4Barometer		_px4_barometer;

	// device::Device		*_interface;
    MS5611_I2C		_interface;

	prom_s		_prom_s;

	enum MS56XX_DEVICE_TYPES _device_type;
	bool			_collect_phase{false};
	unsigned		_measure_phase{false};

	/* intermediate temperature values per MS5611/MS5607 datasheet */
	int64_t			_OFF{0};
	int64_t			_SENS{0};

	/**
	 * Initialize the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void			start();

	/**
	 * Issue a measurement command for the current state.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			measure();

	/**
	 * Collect the result of the most recent measurement.
	 */
	int			collect();
};


/*using example declare*/
extern class MS5611 c_ms5611;
void start_baro(void);
void * thread_baro(void * ptr);

#endif