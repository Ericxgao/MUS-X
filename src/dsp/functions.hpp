#pragma once

#include <rack.hpp>

namespace musx {

using namespace rack;
using simd::float_4;
using simd::int32_4;

template <typename T = float>
inline T fastCos(T x)
{
	// from https://stackoverflow.com/questions/18662261/fastest-implementation-of-sine-cosine-and-square-root-in-c-doesnt-need-to-b
    x *= 1./(2.*M_PI);
    x -= T(.25) + floor(x + T(.25));
    x *= T(16.) * (fabs(x) - T(.5));
    #if EXTRA_PRECISION
    x += T(.225) * x * (fabs(x) - T(1.));
    #endif
    return x;
}

template <typename T = float>
inline T panGetVolL(T pan)
{
	return fastCos((pan + 1.f) * 0.25f * M_PI);
}

template <typename T = float>
inline T panGetVolR(T pan)
{
	return fastCos((pan - 1.f) * 0.25f * M_PI);
}

inline float_4 waveshape(float_4 in)
{
	static const float a = {1.f/8.f};
	static const float b = {1.f/18.f};

	return in - a*in*in - b*in*in*in + a;
}

// y_max = +-1
inline float_4 tanh(float_4 x)
{
	// Pade approximant of tanh
	x = simd::clamp(x, -3.f, 3.f);
	return x * (27 + x * x) / (27 + 9 * x * x);
}

// y_max = +-10
inline float_4 cheapSaturator(float_4 x)
{
	x = simd::clamp(x, -15.f, 15.f);
	return x - 1.f/675.f * x*x*x;
}

/** Antiderivative-antialiased saturator */
// y_max = +-scale
template <typename T = float>
class AntialiasedCheapSaturator {
private:
	static constexpr float epsilon = 1.e-3; // good balance between antialising and stability & noise
	//static constexpr float scale = 13.1713; // deviation from 10*tanh(x/10) < 0.317 in [-23..23]
	static constexpr float scale = 12.; // limit output to +-12V
	T x = 0; // previous input value

	// saturation function
	static T f(T x)
	{
		return x * (fabs(x) + T(2.)) / pow(fabs(x) + T(1.), 2);
	}

	// antiderivative of the saturation function
	static T F(T x)
	{
		x = fabs(x) + T(1.);
		return x + T(1.)/x;
	}

public:
	T process(T in)
	{
		return processBandlimited(in);
	}

	T processBandlimited(T in)
	{
		in /= 20.;
		T bandlimited = (F(in) - F(x)) / (in - x);
		T fallback = T(0.5) * (f(in) + f(x));

		T ret = scale * ifelse(fabs(in - x) < epsilon,
				fallback,
				bandlimited);

		x = in;

		return ret;
	}

	void processBlockBandlimited(T* inBuffer, int oversamplingRate)
	{
		for (int i = 0; i < oversamplingRate; ++i)
		{
			T in = inBuffer[i] / 20.;
			T bandlimited = (F(in) - F(x)) / (in - x);
			T fallback = T(0.5) * (f(in) + f(x));

			inBuffer[i] = scale * ifelse(fabs(in - x) < epsilon,
					fallback,
					bandlimited);

			x = in;
		}
	}

	static T processNonBandlimited(T in)
	{
		in /= T(20.);
		return scale * f(in);
	}
};

/** Limits the derivative of the output by a slew rate, in units/tick. */
template <typename T = float>
struct TSlewLimiter {
	T out = 0.f;
	T slew = 0.f;

	void reset() {
		out = 0.f;
	}

	void setMaxSlew(T slew) {
		this->slew = slew;
	}
	T process(T in) {
		out = simd::clamp(in, out - slew, out + slew);
		return out;
	}
};

static inline float_4 castIntMaskToFloat(int32_4 maskInt)
{
	void* tmp = &maskInt;
	return *(float_4*)tmp;
}

static inline int32_4 castFloatMaskToInt(float_4 maskFloat)
{
	void* tmp = &maskFloat;
	return *(int32_4*)tmp;
}

}
