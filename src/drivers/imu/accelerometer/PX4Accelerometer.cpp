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


#include "PX4Accelerometer.h"
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

static constexpr uint8_t clipping(const int16_t samples[], int16_t clip_limit, uint8_t len)
{
	unsigned clip_count = 0;

	for (int n = 0; n < len; n++) {
		if (abs(samples[n]) >= clip_limit) {
			clip_count++;
		}
	}

	return clip_count;
}

// PX4Accelerometer::PX4Accelerometer(uint32_t device_id, enum Rotation rotation) :
// 	_device_id{device_id},
// 	_rotation{rotation}
// {
// 	// advertise immediately to keep instance numbering in sync
// 	_sensor_pub.advertise();

// 	param_get(param_find("IMU_GYRO_RATEMAX"), &_imu_gyro_rate_max);
// }
PX4Accelerometer::PX4Accelerometer()
{

}

PX4Accelerometer::~PX4Accelerometer()
{
	// _sensor_pub.unadvertise();
	// _sensor_fifo_pub.unadvertise();
}

// void PX4Accelerometer::set_device_type(uint8_t devtype)
// {
// 	// current DeviceStructure
// 	union device::Device::DeviceId device_id;
// 	device_id.devid = _device_id;

// 	// update to new device type
// 	device_id.devid_s.devtype = devtype;

// 	// copy back
// 	_device_id = device_id.devid;
// }

void PX4Accelerometer::set_scale(float scale)
{
	if (fabsf(scale - _scale) > FLT_EPSILON) {
		// rescale last sample on scale change
		float rescale = _scale / scale;

		for (auto &s : _last_sample) {
			s = roundf(s * rescale);
		}

		_scale = scale;

		UpdateClipLimit();
	}
}

void PX4Accelerometer::update(const hrt_abstime &timestamp_sample, float x, float y, float z)
{
	// Apply rotation (before scaling)
	// rotate_3f(_rotation, x, y, z);

	// publish
	sensor_accel_s report;

	report.timestamp_sample = timestamp_sample;
	report.device_id = _device_id;
	report.temperature = _temperature;
	report.error_count = _error_count;
	report.x = x * _scale;
	report.y = y * _scale;
	report.z = z * _scale;
	report.clip_counter[0] = (fabsf(x) >= _clip_limit);
	report.clip_counter[1] = (fabsf(y) >= _clip_limit);
	report.clip_counter[2] = (fabsf(z) >= _clip_limit);
	report.samples = 1;
	report.timestamp = get_time_now();

	// _sensor_pub.publish(report);
}

void PX4Accelerometer::updateFIFO(sensor_accel_fifo_s &sample)
{
	// rotate all raw samples and publish fifo
	const uint8_t sampleN = sample.samples;

	// for (int n = 0; n < sampleN; n++) {
	// 	rotate_3i(_rotation, sample.x[n], sample.y[n], sample.z[n]);
	// }

	sample.device_id = _device_id;
	sample.scale = _scale;
	// sample.timestamp = hrt_absolute_time();
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


	const float scale = _scale / (float)sampleN;

	// publish
	sensor_accel_s report;
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
	report.clip_counter[0] = clipping(sample.x, _clip_limit, sampleN);
	report.clip_counter[1] = clipping(sample.y, _clip_limit, sampleN);
	report.clip_counter[2] = clipping(sample.z, _clip_limit, sampleN);
	report.samples = sampleN;
	// report.timestamp = hrt_absolute_time();
	report.timestamp = get_time_now();

	// _sensor_pub.publish(report);
	// publish data here

	#ifndef TESTMODE

    static class iir_lpf2_typedef accel_lpf[3];
    static bool initd = false;
    if(!initd)
    {
	    for(int i = 0; i < 3; i ++)
	    {
	        accel_lpf[i].set_cutoff_frequency(config.imu_rate,config.accel_cutoff);
	    }
	    initd = true;
    }


	accel_raw_typedef _accel_raw;
    accel_typedef _accel;
	_accel_raw.timestamp = report.timestamp;
	_accel_raw.accel[0] = accel_lpf[0].apply(-report.x);
	_accel_raw.accel[1] = accel_lpf[1].apply(-report.y);
	_accel_raw.accel[2] = accel_lpf[2].apply(report.z);
	// calibration.apply_accel_calibration(_imu_raw.accel, _imu.accel);
	calibration.apply_accel_calibration(_accel_raw.accel, _accel.accel);
	_accel.timestamp = report.timestamp;
	accel_raw_msg.publish(&_accel_raw);
	accel_msg.publish(&_accel);
	#else
	printf("accel x: %f, accel y: %f, accel z: %f\n", report.x, report.y, report.z);
	#endif
}

void PX4Accelerometer::UpdateClipLimit()
{
	// 99.9% of potential max
	_clip_limit = math_px4::constrain((_range / _scale) * 0.999f, 0.f, (float)INT16_MAX);
}
