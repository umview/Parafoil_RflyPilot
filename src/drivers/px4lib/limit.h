#ifndef _LIMIT_H_
#define _LIMIT_H_


#ifndef MATH_PI
#define MATH_PI		3.141592653589793238462643383280
#endif

namespace math_px4
{
	template<typename _Tp>
	constexpr _Tp min(_Tp a, _Tp b)
	{
		return (a < b) ? a : b;
	}

	template<typename _Tp>
	constexpr _Tp max(_Tp a, _Tp b)
	{
		return (a > b) ? a : b;
	}

    template<typename _Tp>
    constexpr _Tp constrain(_Tp val, _Tp min_val, _Tp max_val)
    {
        return (val < min_val) ? min_val : ((val > max_val) ? max_val : val);
    }
	
	template<typename T>
	constexpr T radians(T degrees)
	{
		return degrees * (static_cast<T>(MATH_PI) / static_cast<T>(180));
	}
}

#endif