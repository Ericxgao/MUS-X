#include "plugin.hpp"
#include "../dsp/functions.hpp"

namespace musx {

using namespace rack;
using simd::float_4;
using simd::int32_4;

class LFOBlock {
private:
	int sampleRate = 48000;
	int sampleRateReduction = 1;

	float_4 rand4 = {0.f};

	// integers overflow, so phase resets automatically
	int32_4 phasor = {INT32_MIN};
	int32_4 lastPhasor = {INT32_MIN};
	int32_4 phaseInc = {0};
	float_4 wave = {0}; // -1..1

	float_4 amp = {1.};

	float_4 reset = {0};

	size_t shape = 0;

	int32_4 singleCycle = castFloatMaskToInt(float_4::zero());

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

	// [0..1]
	void setRand(float rnd)
	{
		rand4 = rnd;
	}

	// [0..1]
	void setRand(float_4 rnd)
	{
		rand4 = rnd;
	}

	void setSampleRate(int s)
	{
		sampleRate = s;
	}

	void setSampleRateReduction(int s)
	{
		sampleRateReduction = s;
	}

	void setShape(size_t s)
	{
		shape = s;
	}

	void setSingleCycle(bool s)
	{
		if (s)
		{
			singleCycle = castFloatMaskToInt(float_4::mask());
		}
		else
		{
			singleCycle = castFloatMaskToInt(float_4::zero());
		}
	}

	// 0V = 2Hz
	void setFrequencyVOct(float_4 f)
	{
		float_4 freq = dsp::exp2_taylor5(f);
		phaseInc = INT32_MAX / sampleRate * freq * sampleRateReduction;
	}

	void setAmp(float_4 a)
	{
		amp = clamp(a, 0.f, 10.f);
	}

	void setReset(float_4 rst)
	{
		float_4 mask = rst > (reset + 0.5f);
		phasor += castFloatMaskToInt(mask) & (-phasor + INT32_MIN);
		lastPhasor += castFloatMaskToInt(mask) & (-lastPhasor + INT32_MAX);
		reset = rst;
	}

	void resetPhases()
	{
		phasor = INT32_MIN;
	}

	void process()
	{
		float_4 doSample = -(lastPhasor > phasor);
		float_4 phase;

		lastPhasor = phasor;

		// get phase inc
		int32_4 realPhaseInc = 2*phaseInc;
		switch(shape)
		{
		case 6:
			realPhaseInc = 4*phaseInc;
			break;
		case 7:
			realPhaseInc = phaseInc;
		}
		realPhaseInc -= (singleCycle & (phasor == INT32_MAX)) & realPhaseInc;

		// increment phase
		phasor += 2*realPhaseInc;
		phasor += (singleCycle & (phasor < lastPhasor)) & (-phasor + INT32_MAX);

		switch(shape)
		{
			case 0:
				// sine
				wave = fastCos((float_4)(phasor/INT32_MAX)*M_PI);
				break;
			case 1:
				// tri
				wave = 2. * simd::ifelse(phasor < 0, (float_4)(phasor/INT32_MAX), -(float_4)(phasor/INT32_MAX)) + 1.;
				break;
			case 2:
				// square
				wave = 2. * (float_4)(phasor > 0 * 2. - 1.) + 1.;
				break;
			case 3:
				// pulse
				wave = 2. * (float_4)(phasor > -INT32_MAX/4 * 2. - 1.) + 1.;
				break;
			case 4:
				// ramp
				wave = (float_4)(phasor/INT32_MAX);
				break;
			case 5:
				// saw
				wave = -(float_4)(phasor/INT32_MAX);
				break;
			case 6:
				// s&h
				wave -= doSample * wave; // if doSample, set to 0
				wave += doSample * (2. * rand4 - 1.f); // if doSample, set to random value
				break;
			case 7:
				// warped
				phase = phasor/INT32_MAX * M_PI + 2.701735654f;
				wave = 0.598086124 * (fastCos(phase - 0.5f * M_PI) - fastCos(2.f * phase + 0.1f * M_PI)) - 0.19139f;
				break;
			default:
				wave = 0.f;
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
