/****************************************************************************
 *
 *   Copyright (c) 2018-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/


#include "PX4Gyroscope.h"
#ifndef TESTMODE
	#include "systime.h"
	// #include "config.h"
	#include "msg_def.h"
	#include "sensor_calibration.h"
#endif

#ifndef _TIME_H_
	#include "../../px4lib/test_time.h"
#endif
// #include <lib/drivers/device/Device.hpp>
// #include <lib/parameters/param.h>

// using namespace time_literals;
// using matrix::Vector3f;

static constexpr int32_t sum(const int16_t samples[], uint8_t len)
{
	int32_t sum = 0;

	for (int n = 0; n < len; n++) {
		sum += samples[n];
	}

	return sum;
}

// PX4Gyroscope::PX4Gyroscope(uint32_t device_id, enum Rotation rotation) :
// 	_device_id{device_id},
// 	_rotation{rotation}
// {
// 	// advertise immediately to keep instance numbering in sync
// 	_sensor_pub.advertise();

// 	param_get(param_find("IMU_GYRO_RATEMAX"), &_imu_gyro_rate_max);
// }

PX4Gyroscope::PX4Gyroscope()
{
}

PX4Gyroscope::~PX4Gyroscope()
{
	// _sensor_pub.unadvertise();
	// _sensor_fifo_pub.unadvertise();
}

// void PX4Gyroscope::set_device_type(uint8_t devtype)
// {
// 	// current DeviceStructure
// 	union device::Device::DeviceId device_id;
// 	device_id.devid = _device_id;

// 	// update to new device type
// 	device_id.devid_s.devtype = devtype;

// 	// copy back
// 	_device_id = device_id.devid;
// }

void PX4Gyroscope::set_scale(float scale)
{
	if (fabsf(scale - _scale) > FLT_EPSILON) {
		// rescale last sample on scale change
		float rescale = _scale / scale;

		for (auto &s : _last_sample) {
			s = roundf(s * rescale);
		}

		_scale = scale;
	}
}

void PX4Gyroscope::update(const hrt_abstime &timestamp_sample, float x, float y, float z)
{
	// Apply rotation (before scaling)
	// rotate_3f(_rotation, x, y, z);

	sensor_gyro_s report;

	report.timestamp_sample = timestamp_sample;
	report.device_id = _device_id;
	report.temperature = _temperature;
	report.error_count = _error_count;
	report.x = x * _scale;
	report.y = y * _scale;
	report.z = z * _scale;
	report.samples = 1;
	report.timestamp = get_time_now();

	// _sensor_pub.publish(report);
}

void PX4Gyroscope::updateFIFO(sensor_gyro_fifo_s &sample)
{
	// rotate all raw samples and publish fifo
	const uint8_t sampleN = sample.samples;

	// for (int n = 0; n < sampleN; n++) {
	// 	rotate_3i(_rotation, sample.x[n], sample.y[n], sample.z[n]);
	// }

	sample.device_id = _device_id;
	sample.scale = _scale;
	sample.timestamp = get_time_now();
	// _sensor_fifo_pub.publish(sample);


	// trapezoidal integration (equally spaced, scaled by dt later)
	// const Vector3f integral{
	// 	(0.5f * (_last_sample[0] + sample.x[sampleN - 1]) + sum(sample.x, sampleN - 1)),
	// 	(0.5f * (_last_sample[1] + sample.y[sampleN - 1]) + sum(sample.y, sampleN - 1)),
	// 	(0.5f * (_last_sample[2] + sample.z[sampleN - 1]) + sum(sample.z, sampleN - 1)),
	// };

	float integral[3] = {0,};
	integral[0] = (0.5f * (_last_sample[0] + sample.x[sampleN - 1]) + sum(sample.x, sampleN - 1));
	integral[1] = (0.5f * (_last_sample[1] + sample.y[sampleN - 1]) + sum(sample.y, sampleN - 1));
	integral[2] = (0.5f * (_last_sample[2] + sample.z[sampleN - 1]) + sum(sample.z, sampleN - 1));

	_last_sample[0] = sample.x[sampleN - 1];
	_last_sample[1] = sample.y[sampleN - 1];
	_last_sample[2] = sample.z[sampleN - 1];


	const float scale = _scale / sampleN;

	sensor_gyro_s report;

	report.timestamp_sample = sample.timestamp_sample;
	report.device_id = _device_id;
	report.temperature = _temperature;
	report.error_count = _error_count;
	// report.x = integral(0) * scale;
	// report.y = integral(1) * scale;
	// report.z = integral(2) * scale;
	report.x = integral[0] * scale;
	report.y = integral[1] * scale;
	report.z = integral[2] * scale;
	report.samples = sampleN;
	report.timestamp = get_time_now();

	// publish data here
	// _sensor_pub.publish(report);
	#ifndef TESTMODE

    static class iir_lpf2_typedef gyro_lpf[3];
	static rflypilot_config_typedef config;

    static bool initd = false;
    if(!initd)
    {
  		rflypilot_config_msg.read(&config);

	    for(int i = 0; i < 3; i ++)
	    {
	        gyro_lpf[i].set_cutoff_frequency(config.imu_rate,config.gyro_cutoff_hz);
	    }
	    initd = true;
    }



	gyro_raw_typedef _gyro_raw;
    gyro_typedef _gyro;
	_gyro_raw.timestamp = report.timestamp;

	if(USE_RFLYPILOT == 1)
	{
		_gyro_raw.gyro[0] = gyro_lpf[0].apply(-report.y);
		_gyro_raw.gyro[1] = gyro_lpf[1].apply(report.x);
		_gyro_raw.gyro[2] = gyro_lpf[2].apply(report.z);
	}else{
		_gyro_raw.gyro[0] = gyro_lpf[0].apply(-report.x);
		_gyro_raw.gyro[1] = gyro_lpf[1].apply(-report.y);
		_gyro_raw.gyro[2] = gyro_lpf[2].apply(report.z);
	}

	// calibration.apply_accel_calibration(_imu_raw.accel, _imu.accel);
	calibration.apply_gyro_calibration(_gyro_raw.gyro, _gyro.gyro);
	_gyro.timestamp = report.timestamp;
	gyro_raw_msg.publish(&_gyro_raw);
	gyro_msg.publish(&_gyro);
	#else
	printf("gyro x: %f, gyro y: %f, gyro z: %f\n", report.x, report.y, report.z);
	#endif
}
