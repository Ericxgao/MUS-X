#include "plugin.hpp"
#include "blocks/LFOBlock.hpp"

namespace musx {

using namespace rack;
using simd::float_4;
using simd::int32_4;

struct LFO : Module {
	enum ParamId {
		SHAPE_PARAM,
		FREQ_PARAM,
		AMP_PARAM,
		RESET_PARAM,
		PARAMS_LEN
	};
	enum InputId {
		FREQ_INPUT,
		AMP_INPUT,
		RESET_INPUT,
		INPUTS_LEN
	};
	enum OutputId {
		OUT_OUTPUT,
		OUTPUTS_LEN
	};
	enum LightId {
		LIGHTS_LEN
	};

	int channels = 1;

	const int octaveRange = 10;
	musx::LFOBlock lfoBlock[4];
	bool bipolar = true;

	int sampleRateReduction = 1;
	dsp::ClockDivider divider;

	LFO() {
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
		configSwitch(SHAPE_PARAM, 0.f, 7.f, 0.f, "Shape", LFOBlock::getShapeLabels());
		getParamQuantity(SHAPE_PARAM)->snapEnabled = true;
		configParam(FREQ_PARAM, -octaveRange, octaveRange, 0.f, "Frequency", " Hz", 2., 2.);
		configParam(AMP_PARAM, 0.f, 5.f, 5.f, "Amplitude", " V");
		configButton(RESET_PARAM, "Reset phase");
		configInput(FREQ_INPUT, "Frequency CV");
		configInput(AMP_INPUT, "Amplitude CV");
		configInput(RESET_INPUT, "Reset trigger");
		configOutput(OUT_OUTPUT, "LFO");
	}

	void setSampleRateReduction(int arg)
	{
		sampleRateReduction = arg;
		for (int c = 0; c < 16; c += 4)
		{
			lfoBlock[c/4].setSampleRateReduction(sampleRateReduction);
		}
		divider.setDivision(sampleRateReduction);
	}

	void process(const ProcessArgs& args) override {
		if (divider.process())
		{
			//
			// channels
			//
			channels = 1;

			channels = std::max(channels, inputs[FREQ_INPUT].getChannels());
			channels = std::max(channels, inputs[AMP_INPUT].getChannels());
			channels = std::max(channels, inputs[RESET_INPUT].getChannels());

			outputs[OUT_OUTPUT].setChannels(channels);

			float rand = rack::random::uniform(); // 0..1
			float_4 rand4 = {rand, rand, rand, rand};

			for (int c = 0; c < channels; c += 4) {

				lfoBlock[c/4].setRand(rand4);
				lfoBlock[c/4].setShape(params[SHAPE_PARAM].getValue());
				lfoBlock[c/4].setFrequency(params[FREQ_PARAM].getValue() + inputs[FREQ_INPUT].getPolyVoltageSimd<float_4>(c), args.sampleRate);
				lfoBlock[c/4].setAmp(params[AMP_PARAM].getValue() + inputs[AMP_INPUT].getPolyVoltageSimd<float_4>(c));
				lfoBlock[c/4].setReset(params[RESET_PARAM].getValue() + inputs[RESET_INPUT].getPolyVoltageSimd<float_4>(c));

				lfoBlock[c/4].process();

				if (bipolar)
				{
					outputs[OUT_OUTPUT].setVoltageSimd(lfoBlock[c/4].getBipolar(), c);
				}
				else
				{
					outputs[OUT_OUTPUT].setVoltageSimd(lfoBlock[c/4].getUnipolar(), c);
				}
			}

		}
	}

	json_t* dataToJson() override {
		json_t* rootJ = json_object();
		json_object_set_new(rootJ, "sampleRateReduction", json_integer(sampleRateReduction));
		json_object_set_new(rootJ, "bipolar", json_boolean(bipolar));
		return rootJ;
	}

	void dataFromJson(json_t* rootJ) override {
		json_t* sampleRateReductionJ = json_object_get(rootJ, "sampleRateReduction");
		if (sampleRateReductionJ)
		{
			setSampleRateReduction(json_integer_value(sampleRateReductionJ));
		}
		json_t* bipolarJ = json_object_get(rootJ, "bipolar");
		if (bipolarJ)
		{
			bipolar = json_boolean_value(bipolarJ);
		}
	}
};


struct LFOWidget : ModuleWidget {
	LFOWidget(LFO* module) {
		setModule(module);
		setPanel(createPanel(asset::plugin(pluginInstance, "res/LFO.svg"), asset::plugin(pluginInstance, "res/LFO-dark.svg")));

		addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
		addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(7.62, 16.591)), module, LFO::SHAPE_PARAM));
		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(7.62, 33.183)), module, LFO::FREQ_PARAM));
		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(7.62, 59.114)), module, LFO::AMP_PARAM));
		addParam(createParamCentered<VCVButton>(mm2px(Vec(7.62, 84.64)), module, LFO::RESET_PARAM));

		addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(7.62, 43.581)), module, LFO::FREQ_INPUT));
		addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(7.62, 69.635)), module, LFO::AMP_INPUT));
		addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(7.62, 92.671)), module, LFO::RESET_INPUT));

		addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(7.62, 112.438)), module, LFO::OUT_OUTPUT));
	}

	void appendContextMenu(Menu* menu) override {
		LFO* module = getModule<LFO>();

		menu->addChild(new MenuSeparator);

		menu->addChild(createIndexSubmenuItem("Reduce internal sample rate", {"1x", "2x", "4x", "8x", "16x", "32x", "64x", "128x", "256x", "512x", "1024x"},
			[=]() {
				return log2(module->sampleRateReduction);
			},
			[=](int mode) {
				module->setSampleRateReduction(std::pow(2, mode));
			}
		));

		menu->addChild(createBoolMenuItem("Bipolar", "",
			[=]() {
				return module->bipolar;
			},
			[=](int mode) {
				module->bipolar = mode;
			}
		));
	}
};


Model* modelLFO = createModel<LFO, LFOWidget>("LFO");

}
