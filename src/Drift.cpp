#include "plugin.hpp"
#include "blocks/DriftBlock.hpp"

namespace musx {

using namespace rack;
using simd::float_4;

struct Drift : Module {
	enum ParamId {
		CONST_PARAM,
		RANDOMIZE_PARAM,
		DRIFT_PARAM,
		RATE_PARAM,
		PARAMS_LEN
	};
	enum InputId {
		POLY_INPUT,
		INPUTS_LEN
	};
	enum OutputId {
		OUT_OUTPUT,
		OUTPUTS_LEN
	};
	enum LightId {
		LIGHTS_LEN
	};

	const float minFreq = 0.01f; // min freq [Hz]
	const float base = 1000.f/minFreq; // max freq/min freq
	const int clockDivider = 16;

	int channels = 1;

	dsp::ClockDivider divider;

	musx::DriftBlock driftBlock[4];

	float lastRateParam = -1.f;

	float prevRandomizeValue = 0.f;

	Drift() {
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
		configParam(CONST_PARAM, 0.f, 1.f, 0.f, "Random constant offset", " %", 0.f, 100.f);
		configButton(RANDOMIZE_PARAM, "Randomize constant offsets");
		configParam(DRIFT_PARAM, 0.f, 1.f, 0.f, "Random drift", " %", 0.f, 100.f);
		configParam(RATE_PARAM, 0.f, 1.f, 0.f, "Drift rate", " Hz", base, minFreq);
		getParamQuantity(RATE_PARAM)->smoothEnabled = false;
		configInput(POLY_INPUT, "Polyphony channels");
		configOutput(OUT_OUTPUT, "Signal");

		divider.setDivision(clockDivider);

		for (int c = 0; c < 16; c += 4) {
			driftBlock[c/4].setSampleRateReduction(clockDivider);
		}

		randomizeDiverge();
	}

	void randomizeDiverge()
	{
		for (int c = 0; c < 16; c += 4)
		{
			driftBlock[c/4].randomizeDiverge();
		}
	}

	void process(const ProcessArgs& args) override {
		if (divider.process())
		{
			channels = std::max(1, inputs[POLY_INPUT].getChannels());
			outputs[OUT_OUTPUT].setChannels(channels);

			// randomize
			if (params[RANDOMIZE_PARAM].getValue() && prevRandomizeValue == 0.f)
			{
				randomizeDiverge();
			}
			prevRandomizeValue = params[RANDOMIZE_PARAM].getValue();

			// update filter frequency
			if (params[RATE_PARAM].getValue() != lastRateParam)
			{
				for (int c = 0; c < channels; c += 4) {
					driftBlock[c/4].setFilterFrequencyV(params[RATE_PARAM].getValue());
					lastRateParam = params[RATE_PARAM].getValue();
				}
			}

			for (int c = 0; c < channels; c += 4)
			{
				driftBlock[c/4].setDivergeAmount(params[CONST_PARAM].getValue() * params[CONST_PARAM].getValue());
				driftBlock[c/4].setDriftAmount(params[DRIFT_PARAM].getValue() * params[DRIFT_PARAM].getValue());
				outputs[OUT_OUTPUT].setVoltageSimd(driftBlock[c/4].process(), c);
			}
		}
	}

	void onSampleRateChange(const SampleRateChangeEvent& e) override {
		lastRateParam = -1.f;

		for (int c = 0; c < 16; c += 4) {
			driftBlock[c/4].setSampleRate(e.sampleRate);
		}
	}

	json_t* dataToJson() override {
		json_t* rootJ = json_object();
		json_t* divergeJ = json_array();
		for (int i = 0; i < 16; i++)
		{
			json_array_insert_new(divergeJ, i, json_real(driftBlock[i/4].getDiverge()[i%4]));
		}
		json_object_set_new(rootJ, "diverge", divergeJ);
		return rootJ;
	}

	void dataFromJson(json_t* rootJ) override {
		json_t* divergesJ = json_object_get(rootJ, "diverge");
		if (divergesJ)
		{
			float_4 diverge = {0.f};
			for (int i = 0; i < 16; i++)
			{
				json_t* divergeJ = json_array_get(divergesJ, i);
				if (divergeJ)
				{
					diverge[i%4] = json_real_value(divergeJ);
					driftBlock[i/4].setDiverge(diverge);
				}
			}
		}
	}
};


struct DriftWidget : ModuleWidget {
	DriftWidget(Drift* module) {
		setModule(module);
		setPanel(createPanel(asset::plugin(pluginInstance, "res/Drift.svg"), asset::plugin(pluginInstance, "res/Drift-dark.svg")));

		addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
		addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(7.62, 16.062)), module, Drift::CONST_PARAM));
		addParam(createParamCentered<VCVButton>(mm2px(Vec(7.62, 32.125)), module, Drift::RANDOMIZE_PARAM));
		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(7.62, 48.188)), module, Drift::DRIFT_PARAM));
		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(7.62, 64.251)), module, Drift::RATE_PARAM));

		addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(7.62, 96.375)), module, Drift::POLY_INPUT));

		addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(7.62, 112.438)), module, Drift::OUT_OUTPUT));
	}
};


Model* modelDrift = createModel<Drift, DriftWidget>("Drift");

}
