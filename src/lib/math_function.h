#ifndef _MATH_FUNCTION_
#define _MATH_FUNCTION_
#include "include.h"
class iir_lpf2_typedef{
public:

	iir_lpf2_typedef(float sample_freq, float cutoff_freq)
	{
		// set initial parameters
		set_cutoff_frequency(sample_freq, cutoff_freq);
	}
	iir_lpf2_typedef(void)
	{

	}
	// Change filter parameters
	void set_cutoff_frequency(float sample_freq, float cutoff_freq);
	float apply(float sample);

	// Return the cutoff frequency
	float get_cutoff_freq() const { return _cutoff_freq; }

	// Reset the filter state to this value
	float reset(float sample);

protected:

	float _cutoff_freq;

	float _a1;
	float _a2;

	float _b0;
	float _b1;
	float _b2;

	float _delay_element_1;	// buffered sample -1
	float _delay_element_2;	// buffered sample -2
};
#endif