#include "plugin.hpp"
#include "../dsp/filters.hpp"

namespace musx {

using namespace rack;
using simd::float_4;

class DriftBlock {
private:
	int sampleRate = 48000;
	int sampleRateReduction = 1;

	const float minFreq = 0.01f; // min freq [Hz]
	const float base = 1000.f/minFreq; // max freq/min freq

	float_4 diverge = {0.};
	musx::TOnePole<float_4> lowpass;

	float driftScale = 1.f;

	float_4 divergeAmount = 1.f;
	float_4 driftAmount = 1.f;

public:
	void randomizeDiverge()
	{
		diverge[0] = rack::random::get<float>() - 0.5f;
		diverge[1] = rack::random::get<float>() - 0.5f;
		diverge[2] = rack::random::get<float>() - 0.5f;
		diverge[3] = rack::random::get<float>() - 0.5f;

		diverge *= 10.f; // +-5V
	}

	void setDiverge(float_4 d)
	{
		diverge = d;
	}

	void setDiverge(float d, size_t i)
	{
		diverge[i] = d;
	}

	float_4 getDiverge()
	{
		return diverge;
	}

	void setSampleRate(int s)
	{
		sampleRate = s;
	}

	void setSampleRateReduction(int s)
	{
		sampleRateReduction = s;
	}

	void setFilterFrequencyV(float f)
	{
		float cutoffFreq = simd::pow(base, f) * minFreq / sampleRate * sampleRateReduction;

		driftScale = std::exp(-1.2f * std::log10(cutoffFreq*48000 / 16.f)) * 128.f + 7.f;

		lowpass.setCutoffFreq(cutoffFreq);
		lowpass.tmp = simd::clamp(lowpass.tmp, -5.f/driftScale, 5.f/driftScale);
	}

	void setDivergeAmount(float_4 d)
	{
		divergeAmount = d;
	}

	void setDriftAmount(float_4 d)
	{
		driftAmount = d;
	}

	float_4 process()
	{
		float_4 rn = {rack::random::get<float>() - 0.5f,
				   	  rack::random::get<float>() - 0.5f,
					  rack::random::get<float>() - 0.5f,
					  rack::random::get<float>() - 0.5f};

		lowpass.process(rn);
		float_4 drift = lowpass.lowpass();

		return simd::clamp(divergeAmount * diverge + driftAmount * driftScale * drift, -10.f, 10.f);
	}
};
}
