#include "plugin.hpp"

namespace musx {

using namespace rack;
using simd::float_4;
using simd::int32_4;

class LFOBlock {
private:
	const int octaveRange = 10;
	const float minFreq = 2 * std::pow(2, -octaveRange); // Hz
	const float maxFreq = 2 * std::pow(2,  octaveRange); // Hz
	const float logMaxOverMin = std::log(maxFreq/minFreq); // log(maxFreq/minFreq)

	int sampleRateReduction = 1;

	float_4 rand4 = {0.f};

	// integers overflow, so phase resets automatically
	int32_4 phasor = {0};
	int32_4 phaseInc = {0};
	float_4 wave = {0}; // -1..1

	float_4 amp = {1.};

	float_4 reset = {0};

	size_t shape = 0;

public:
	static std::vector<std::string> getShapeLabels()
	{
		std::vector<std::string> labels = {
			"Sine",
			"Triangle",
			"Square",
			"Pulse",
			"Ramp",
			"Saw",
			"Sample & hold",
			"Warped"
		};
		return labels;
	}

	void setRand(float_4 rnd)
	{
		rand4 = rnd;
	}

	void setSampleRateReduction(int s)
	{
		sampleRateReduction = s;
	}

	void setShape(size_t s)
	{
		shape = s;
	}

	// f = -10..10
	void setFrequency(float_4 f, int sampleRate)
	{
		float_4 freq = 2. * dsp::exp2_taylor5(f);
		phaseInc = INT32_MAX / sampleRate * freq * sampleRateReduction;
	}

	void setAmp(float_4 a)
	{
		amp = clamp(a, 0.f, 12.f);
	}

	void setReset(float_4 rst)
	{
		phasor = simd::ifelse(rst > reset + 0.5, -INT32_MAX, phasor);
		reset = rst;
	}

	void process()
	{
		int32_4 lastPhasor = phasor;

		float_4 doSample;

		switch(shape)
		{
			case 0:
				// sine
				phasor += 2*phaseInc;
				wave = -1. * simd::sin((float_4)(phasor/INT32_MAX)*M_PI);
				break;
			case 1:
				// tri
				phasor += 2*phaseInc;
				wave = 2. * simd::ifelse(phasor < 0, (float_4)(phasor/INT32_MAX), -(float_4)(phasor/INT32_MAX)) + 1.;
				break;
			case 2:
				// square
				phasor += 2*phaseInc;
				wave = 2. * (float_4)(phasor > 0 * 2. - 1.) + 1.;
				break;
			case 3:
				// pulse
				phasor += 2*phaseInc;
				wave = 2. * (float_4)(phasor > -INT32_MAX/4 * 2. - 1.) + 1.;
				break;
			case 4:
				// ramp
				phasor += 2*phaseInc;
				wave = (float_4)(phasor/INT32_MAX);
				break;
			case 5:
				// saw
				phasor += 2*phaseInc;
				wave = -(float_4)(phasor/INT32_MAX);
				break;
			case 6:
				// s&h
				phasor += 4*phaseInc;
				doSample = -(lastPhasor > phasor);
				wave -= doSample * wave; // if doSample, set to 0
				wave += doSample * (2. * rand4 - 1.f); // if doSample, set to random value
				break;
			case 7:
				// warped
				phasor += phaseInc;
				wave = 2./3.26 * (simd::sin((float_4)(phasor/INT32_MAX)*M_PI) - simd::sin((float_4)(2*phasor/INT32_MAX)*M_PI + 0.4 * M_PI)) - 0.22;
				break;
		}
	}

	float_4 getUnipolar() const
	{
		return amp * (wave + 1.f);
	}

	float_4 getBipolar() const
	{
		return amp * wave;
	}
};
}
