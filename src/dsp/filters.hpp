#pragma once

#include <rack.hpp>

namespace musx {

using namespace rack;
using simd::float_4;

/**
 * 1 pole lowpass/highpass
 */
template <typename T = float>
struct TOnePole {
	T a = 0.f;
	T b = 0.f;
	T in;
	T tmp;

	TOnePole() {
		reset();
	}

	void reset() {
		in = {0.f};
		tmp = {0.f};
	}

	/** Sets the cutoff frequency.
	`f` is the ratio between the cutoff frequency and sample rate, i.e. f = f_c / f_s
	*/
	void setCutoffFreq(T f) {
		T x;
		f = fmin(f, 0.3);
		x = exp(-2.0f * M_PI * f);
		a = 1.0f - x;
		b = -x;
	}

	void copyCutoffFreq(TOnePole<T> other) {
		a = other.a;
		b = other.b;
	}

	void process(T x) {
		in = x;
		tmp = a*x - b*tmp;
	}

	inline T processLowpass(T x) {
		tmp = a*x - b*tmp;
		return tmp;
	}

	inline void processLowpassBlock(float_4* in, int oversamplingRate)
	{
		for (int i = 0; i < oversamplingRate; ++i)
		{
			process(in[i]);
			in[i] = tmp;
		}
	}

	inline void processHighpassBlock(float_4* in, int oversamplingRate)
	{
		for (int i = 0; i < oversamplingRate; ++i)
		{
			process(in[i]);
			in[i] -= tmp;
		}
	}

	T lowpass()
	{
		return tmp;
	}

	T highpass()
	{
		return in - tmp;
	}
};

/**
 * 1 pole zdf lowpass/highpass
 */
template <typename T = float>
struct TOnePoleZDF {
	T g = 0.f;
	T x;
	T y;
	T z;

	TOnePoleZDF() {
		reset();
	}

	void reset() {
		x = {0.f};
		y = {0.f};
		z = {0.f};
	}

	/** Sets the cutoff frequency.
	`f` is the ratio between the cutoff frequency and sample rate, i.e. f = f_c / f_s
	*/
	void setCutoffFreq(T f) {
		f = M_PI * f;
		g = f / (1. + f);
	}

	void copyCutoffFreq(TOnePoleZDF<T> other) {
		g = other.g;
	}

	void setState(T state, T mask)
	{
		x += mask & (state - x);
		y += mask & (state - y);
		z += mask & (state - z);
	}

	void process(T in) {
		x = in;
		T v = (x - z) * g;
		y = v + z;
		z = y + v;
	}

	// process, but don't update internal state z
	void processDry(T in) {
		x = in;
		T v = (x - z) * g;
		y = v + z;
	}

	inline T processLowpass(T in) {
		process(in);
		return y;
	}

	T lowpass()
	{
		return y;
	}

	T highpass()
	{
		return x - y;
	}
};


/**
 * 4 pole lowpass
 */
template <typename T = float>
struct TFourPole {
	T a = 0.f;
	T b = 0.f;
	T q;
	T tmp4[4];

	TFourPole() {
		reset();
	}

	void reset() {
		for (int i=0; i<4; ++i)
		{
			tmp4[i] = {0.f};
		}
	}

	/** Sets the cutoff frequency.
	`f` is the ratio between the cutoff frequency and sample rate, i.e. f = f_c / f_s
	*/
	void setCutoffFreq(T f) {
		T x;
		f = fmin(f, 0.3);
		x = exp(-2.0f * M_PI * f);
		a = 1.0f - x;
		b = -x;
	}

	void setResonance(T res)
	{
		q = res;
	}

	void process(T x) {
		// resonance
		x -= q * tmp4[3];
		x = simd::clamp(x, -5.f, 5.f);

		T out = a*x - b*tmp4[0];
		tmp4[0] = out;

		out = a*out - b*tmp4[1];
		tmp4[1] = out;

		out = a*out - b*tmp4[2];
		tmp4[2] = out;

		out = a*out - b*tmp4[3];
		tmp4[3] = out;
	}

	T lowpass()
	{
		return lowpass4();
	}

	T lowpass1() {
		return clamp(tmp4[0] * (1.f + q));
	}

	T lowpass2() {
		return clamp(tmp4[1] * (1.f + q));
	}

	T lowpass3() {
		return clamp(tmp4[2] * (1.f + q));
	}

	T lowpass4() {
		return clamp(tmp4[3] * (1.f + q));
	}

	T lowpassN(int order) {
		return clamp(tmp4[order] * (1.f + q));
	}

	T clamp(T in)
	{
		//if (typeid(T) == typeid(float_4))
		//{
			return simd::clamp(in, -5.f, 5.f);
		//}
		//return std::fmin(std::fmax(in, -5.f), 5.f);
	}
};


/**
 * State variable filter
 */
template <typename T = float>
struct TSVF {
	T c = {0.f};
	T q = {1.f};
	T scale = {1.f};

	T lp = {0};
	T hp = {0};
	T bp = {0};

	TSVF() {
		reset();
	}

	void reset() {
		lp = 0.f;
		hp = 0.f;
		bp = 0.f;
	}

	/** Sets the cutoff frequency.
	`f` is the ratio between the cutoff frequency and sample rate, i.e. f = f_c / f_s
	*/
	void setCutoffFreq(T f) {
		f = clamp(f, 0.001, 0.2f);
		c = 2.f * sin(M_PI * f);
	}

	/**
	 * Set resonance between 0 and 1
	 */
	void setResonance(T r) {
		q = clamp(1.f - r, 0.f, 1.f);
		scale = q;
	}

	void process(T x) {
		lp = lp + c * bp;
		hp = scale * x - lp - q * bp;
		bp = c * hp + bp;
		//notch = hp + lp;
	}
	T lowpass() {
		return lp;
	}
	T highpass() {
		return hp;
	}
	T bandpass() {
		return bp;
	}
};


/**
 * (2*O)th order biquad filters to filter out high frequency content between modules
 */
template <typename T = float, size_t O = 2>
struct AliasReductionFilter
{
	dsp::TBiquadFilter<T> filter[O];

	AliasReductionFilter()
	{
		static_assert(O > 0, "Order must be > 0");
		setParameters(0.25);
	}

	/**
	f: normalized frequency (cutoff frequency / sample rate), must be less than 0.5
	Q: quality factor
	*/
	void setParameters(float f, float Q = 0.75f)
	{
		f = clamp(f, 0.f, 0.5f);
		for (size_t i = 0; i < O; i++)
		{
			filter[i].setParameters(dsp::TBiquadFilter<float_4>::LOWPASS, f, Q, 1.f);
		}
	}

	void setCutoffFreq(float f)
	{
		setParameters(f);
	}

	T process(T in)
	{
		T out = in;
		for (size_t i = 0; i < O; i++)
		{
			out = filter[i].process(out);
		}
		return out;
	}

	T processLowpass(T in)
	{
		return process(in);
	}

	void processLowpassBlock(T* in, int oversamplingRate)
	{
		for (int i = 0; i < oversamplingRate; ++i)
		{
			in[i] = processLowpass(in[i]);
		}
	}
};


struct CombFilter
{
	static const int delayLineSize = 2 << 16;
	float_4 delayLine[delayLineSize] = {0};
	int index = 0;

	float_4 freq = 0;
	float_4 feedback = 0;

	// set frequency in Hz
	void setFreq(float_4 f)
	{
		freq = clamp(f, 20.f, 44000.f);
	}

	// [0..5]
	void setFeedback(float_4 f)
	{
		feedback = 1.f - clamp(f, 0.f, 5.f) / 5.f;
		feedback = feedback * feedback;
		feedback = 1.0f - feedback;
	}

	void setNegativeFeedback(float_4 f)
	{
		setFeedback(f);
		feedback *= -1.f;
	}

	void reset()
	{
		std::memset(&delayLine, 0, delayLineSize * sizeof(float_4));
	}

	// dt in seconds
	float_4 process(float_4 in, float_4 dt)
	{
		// read from delay line
		float_4 out = 0.f;
		float_4 fractionalOffset = dt * freq;
		fractionalOffset = fmin(1.f / fractionalOffset, delayLineSize - 1);


		for (size_t i = 0; i < 4; ++i)
		{
			int offsetFloor = fractionalOffset[i];
			int readIndex = index - offsetFloor - 1;
			readIndex += (readIndex < 0) * delayLineSize;

			out[i] = delayLine[readIndex][i];

			int readIndex2 = readIndex + 1;
			readIndex2 &= delayLineSize-1;

			float frac = fractionalOffset[i] - offsetFloor;

			out[i] = crossfade(delayLine[readIndex2][i], delayLine[readIndex][i], frac);
		}

		// write to delay line
		delayLine[index] = clamp(in + feedback * out , -100.f, 100.f);

		// advance index
		++index;
		index &= delayLineSize-1;

		return out;
	}
};

}
