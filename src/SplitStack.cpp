#include "plugin.hpp"

namespace musx {

using namespace rack;
using simd::float_4;

struct SplitStack : Module {
	enum ParamId {
		STACK_PARAM,
		SPLIT_PARAM,
		SWITCH_PARAM,
		PARAMS_LEN
	};
	enum InputId {
		VOCT_INPUT,
		GATE_INPUT,
		VEL_INPUT,
		AFT_INPUT,
		RETRIG_INPUT,
		INPUTS_LEN
	};
	enum OutputId {
		VOCT_A_OUTPUT,
		VOCT_B_OUTPUT,
		GATE_A_OUTPUT,
		GATE_B_OUTPUT,
		VEL_A_OUTPUT,
		VEL_B_OUTPUT,
		AFT_A_OUTPUT,
		AFT_B_OUTPUT,
		RETRIG_A_OUTPUT,
		RETRIG_B_OUTPUT,
		OUTPUTS_LEN
	};
	enum LightId {
		STACK_LIGHT,
		SPLIT_LIGHT,
		SWITCH_LIGHT,
		LIGHTS_LEN
	};

	float lastSplitParamValue = 0.f;
	bool split = false;
	bool learnedSplitPoint = false;
	float splitPoint = 0.f;
	float_4 oldGates[4] = {0.f};

	SplitStack() {
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
		configSwitch(STACK_PARAM, 0.f, 1.f, 0.f, "Stack A+B", {"off", "on"});
		configSwitch(SPLIT_PARAM, 0.f, 1.f, 0.f, "Split A/B");
		configSwitch(SWITCH_PARAM, 0.f, 1.f, 0.f, "Switch A↔B", {"off", "on"});
		configInput(VOCT_INPUT, "V/Oct");
		configInput(GATE_INPUT, "Gate");
		configInput(VEL_INPUT, "Velocity");
		configInput(AFT_INPUT, "Aftertouch");
		configInput(RETRIG_INPUT, "Retrigger");
		configOutput(VOCT_A_OUTPUT, "V/Oct A");
		configOutput(VOCT_B_OUTPUT, "V/Oct B");
		configOutput(GATE_A_OUTPUT, "Gate A");
		configOutput(GATE_B_OUTPUT, "Gate B");
		configOutput(VEL_A_OUTPUT, "Velocity A");
		configOutput(VEL_B_OUTPUT, "Velocity B");
		configOutput(AFT_A_OUTPUT, "Aftertouch A");
		configOutput(AFT_B_OUTPUT, "Aftertouch B");
		configOutput(RETRIG_A_OUTPUT, "Retrigger A");
		configOutput(RETRIG_B_OUTPUT, "Retrigger B");
	}

	void process(const ProcessArgs& args) override {
		int channels = inputs[VOCT_INPUT].getChannels();

		lights[SWITCH_LIGHT].setBrightness(params[SWITCH_PARAM].getValue());

		if (params[STACK_PARAM].getValue())
		{
			// stack
			lights[STACK_LIGHT].setBrightness(1.f);
			lights[SPLIT_LIGHT].setBrightness(0.f);
			split = false;

			for (int c = 0; c < channels; c += 4) {
				outputs[VOCT_A_OUTPUT].channels = channels;
				outputs[VOCT_A_OUTPUT].setVoltageSimd(inputs[VOCT_INPUT].getPolyVoltageSimd<float_4>(c), c);
				outputs[VOCT_B_OUTPUT].channels = channels;
				outputs[VOCT_B_OUTPUT].setVoltageSimd(inputs[VOCT_INPUT].getPolyVoltageSimd<float_4>(c), c);

				outputs[GATE_A_OUTPUT].channels = channels;
				outputs[GATE_A_OUTPUT].setVoltageSimd(inputs[GATE_INPUT].getPolyVoltageSimd<float_4>(c), c);
				outputs[GATE_B_OUTPUT].channels = channels;
				outputs[GATE_B_OUTPUT].setVoltageSimd(inputs[GATE_INPUT].getPolyVoltageSimd<float_4>(c), c);

				outputs[VEL_A_OUTPUT].channels = channels;
				outputs[VEL_A_OUTPUT].setVoltageSimd(inputs[VEL_INPUT].getPolyVoltageSimd<float_4>(c), c);
				outputs[VEL_B_OUTPUT].channels = channels;
				outputs[VEL_B_OUTPUT].setVoltageSimd(inputs[VEL_INPUT].getPolyVoltageSimd<float_4>(c), c);

				outputs[AFT_A_OUTPUT].channels = channels;
				outputs[AFT_A_OUTPUT].setVoltageSimd(inputs[AFT_INPUT].getPolyVoltageSimd<float_4>(c), c);
				outputs[AFT_B_OUTPUT].channels = channels;
				outputs[AFT_B_OUTPUT].setVoltageSimd(inputs[AFT_INPUT].getPolyVoltageSimd<float_4>(c), c);

				outputs[RETRIG_A_OUTPUT].channels = channels;
				outputs[RETRIG_A_OUTPUT].setVoltageSimd(inputs[RETRIG_INPUT].getPolyVoltageSimd<float_4>(c), c);
				outputs[RETRIG_B_OUTPUT].channels = channels;
				outputs[RETRIG_B_OUTPUT].setVoltageSimd(inputs[RETRIG_INPUT].getPolyVoltageSimd<float_4>(c), c);
			}
		}

		// turn split on or off
		if (lastSplitParamValue && ! params[SPLIT_PARAM].getValue())
		{
			if (!learnedSplitPoint)
			{
				split = !split;
				lights[SPLIT_LIGHT].setBrightness(split);
				if (split)
				{
					lights[STACK_LIGHT].setBrightness(0.f);
					params[STACK_PARAM].setValue(0.f);
				}
			}
			learnedSplitPoint = false;
		}

		lastSplitParamValue = params[SPLIT_PARAM].getValue();

		paramQuantities[SPLIT_PARAM]->description = "off";
		if (split)
		{
			paramQuantities[SPLIT_PARAM]->description = "on";
		}

		// learn split point
		if (split && params[SPLIT_PARAM].getValue())
		{
			paramQuantities[SPLIT_PARAM]->description = "learn split point";
			for (int c = 0; c < channels; c += 1) {
				float gate = inputs[GATE_INPUT].getVoltage(c);

				if (gate > oldGates[c/4][c%4] + 1.f)
				{
					splitPoint = inputs[VOCT_INPUT].getVoltage(c);
					learnedSplitPoint = true;
					break;
				}
			}
		}

		if (split)
		{
			// split
			for (int c = 0; c < channels; c += 4) {
				float_4 mask = params[SWITCH_PARAM].getValue() ?
						inputs[VOCT_INPUT].getPolyVoltageSimd<float_4>(c) < splitPoint :
						inputs[VOCT_INPUT].getPolyVoltageSimd<float_4>(c) >= splitPoint;

				outputs[VOCT_A_OUTPUT].channels = channels;
				outputs[VOCT_A_OUTPUT].setVoltageSimd(~mask & inputs[VOCT_INPUT].getPolyVoltageSimd<float_4>(c), c);
				outputs[VOCT_B_OUTPUT].channels = channels;
				outputs[VOCT_B_OUTPUT].setVoltageSimd(mask & inputs[VOCT_INPUT].getPolyVoltageSimd<float_4>(c), c);

				outputs[GATE_A_OUTPUT].channels = channels;
				outputs[GATE_A_OUTPUT].setVoltageSimd(~mask & inputs[GATE_INPUT].getPolyVoltageSimd<float_4>(c), c);
				outputs[GATE_B_OUTPUT].channels = channels;
				outputs[GATE_B_OUTPUT].setVoltageSimd(mask & inputs[GATE_INPUT].getPolyVoltageSimd<float_4>(c), c);

				outputs[VEL_A_OUTPUT].channels = channels;
				outputs[VEL_A_OUTPUT].setVoltageSimd(~mask & inputs[VEL_INPUT].getPolyVoltageSimd<float_4>(c), c);
				outputs[VEL_B_OUTPUT].channels = channels;
				outputs[VEL_B_OUTPUT].setVoltageSimd(mask & inputs[VEL_INPUT].getPolyVoltageSimd<float_4>(c), c);

				outputs[AFT_A_OUTPUT].channels = channels;
				outputs[AFT_A_OUTPUT].setVoltageSimd(~mask & inputs[AFT_INPUT].getPolyVoltageSimd<float_4>(c), c);
				outputs[AFT_B_OUTPUT].channels = channels;
				outputs[AFT_B_OUTPUT].setVoltageSimd(mask & inputs[AFT_INPUT].getPolyVoltageSimd<float_4>(c), c);

				outputs[RETRIG_A_OUTPUT].channels = channels;
				outputs[RETRIG_A_OUTPUT].setVoltageSimd(~mask & inputs[RETRIG_INPUT].getPolyVoltageSimd<float_4>(c), c);
				outputs[RETRIG_B_OUTPUT].channels = channels;
				outputs[RETRIG_B_OUTPUT].setVoltageSimd(mask & inputs[RETRIG_INPUT].getPolyVoltageSimd<float_4>(c), c);

				oldGates[c/4] = inputs[GATE_INPUT].getPolyVoltageSimd<float_4>(c);
			}
		}

		if (!params[STACK_PARAM].getValue() && !split)
		{
			// normal
			lights[STACK_LIGHT].setBrightness(0.f);
			lights[SPLIT_LIGHT].setBrightness(0.f);
			split = false;

			if (params[SWITCH_PARAM].getValue())
			{
				// AB switched
				for (int c = 0; c < channels; c += 4) {
					outputs[VOCT_A_OUTPUT].channels = 0;
					outputs[VOCT_B_OUTPUT].channels = channels;
					outputs[VOCT_B_OUTPUT].setVoltageSimd(inputs[VOCT_INPUT].getPolyVoltageSimd<float_4>(c), c);

					outputs[GATE_A_OUTPUT].channels = 0;
					outputs[GATE_B_OUTPUT].channels = channels;
					outputs[GATE_B_OUTPUT].setVoltageSimd(inputs[GATE_INPUT].getPolyVoltageSimd<float_4>(c), c);

					outputs[VEL_A_OUTPUT].channels = 0;
					outputs[VEL_B_OUTPUT].channels = channels;
					outputs[VEL_B_OUTPUT].setVoltageSimd(inputs[VEL_INPUT].getPolyVoltageSimd<float_4>(c), c);

					outputs[AFT_A_OUTPUT].channels = 0;
					outputs[AFT_B_OUTPUT].channels = channels;
					outputs[AFT_B_OUTPUT].setVoltageSimd(inputs[AFT_INPUT].getPolyVoltageSimd<float_4>(c), c);

					outputs[RETRIG_A_OUTPUT].channels = 0;
					outputs[RETRIG_B_OUTPUT].channels = channels;
					outputs[RETRIG_B_OUTPUT].setVoltageSimd(inputs[RETRIG_INPUT].getPolyVoltageSimd<float_4>(c), c);
				}
			}
			else
			{
				// AB normal
				for (int c = 0; c < channels; c += 4) {
					outputs[VOCT_A_OUTPUT].channels = channels;
					outputs[VOCT_A_OUTPUT].setVoltageSimd(inputs[VOCT_INPUT].getPolyVoltageSimd<float_4>(c), c);
					outputs[VOCT_B_OUTPUT].channels = 0;

					outputs[GATE_A_OUTPUT].channels = channels;
					outputs[GATE_A_OUTPUT].setVoltageSimd(inputs[GATE_INPUT].getPolyVoltageSimd<float_4>(c), c);
					outputs[GATE_B_OUTPUT].channels = 0;

					outputs[VEL_A_OUTPUT].channels = channels;
					outputs[VEL_A_OUTPUT].setVoltageSimd(inputs[VEL_INPUT].getPolyVoltageSimd<float_4>(c), c);
					outputs[VEL_B_OUTPUT].channels = 0;

					outputs[AFT_A_OUTPUT].channels = channels;
					outputs[AFT_A_OUTPUT].setVoltageSimd(inputs[AFT_INPUT].getPolyVoltageSimd<float_4>(c), c);
					outputs[AFT_B_OUTPUT].channels = 0;

					outputs[RETRIG_A_OUTPUT].channels = channels;
					outputs[RETRIG_A_OUTPUT].setVoltageSimd(inputs[RETRIG_INPUT].getPolyVoltageSimd<float_4>(c), c);
					outputs[RETRIG_B_OUTPUT].channels = 0;
				}
			}
		}
	}
};


struct SplitStackWidget : ModuleWidget {
	SplitStackWidget(SplitStack* module) {
		setModule(module);
		setPanel(createPanel(asset::plugin(pluginInstance, "res/SplitStack.svg"), asset::plugin(pluginInstance, "res/SplitStack-dark.svg")));

		addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
		addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		addParam(createLightParamCentered<VCVLightLatch<MediumSimpleLight<WhiteLight>>>(mm2px(Vec(11.43, 16.062)), module, SplitStack::STACK_PARAM, SplitStack::STACK_LIGHT));
		addParam(createLightParamCentered<VCVLightButton<MediumSimpleLight<WhiteLight>>>(mm2px(Vec(34.29, 16.062)), module, SplitStack::SPLIT_PARAM, SplitStack::SPLIT_LIGHT));
		addParam(createLightParamCentered<VCVLightLatch<MediumSimpleLight<WhiteLight>>>(mm2px(Vec(11.43, 28.95)), module, SplitStack::SWITCH_PARAM, SplitStack::SWITCH_LIGHT));

		addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(7.62, 51.456)), module, SplitStack::VOCT_INPUT));
		addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(7.62, 66.46)), module, SplitStack::GATE_INPUT));
		addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(7.62, 81.465)), module, SplitStack::VEL_INPUT));
		addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(7.62, 96.469)), module, SplitStack::AFT_INPUT));
		addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(7.62, 111.473)), module, SplitStack::RETRIG_INPUT));

		addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(22.86, 51.456)), module, SplitStack::VOCT_A_OUTPUT));
		addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(38.1, 51.456)), module, SplitStack::VOCT_B_OUTPUT));
		addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(22.86, 66.46)), module, SplitStack::GATE_A_OUTPUT));
		addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(38.1, 66.46)), module, SplitStack::GATE_B_OUTPUT));
		addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(22.86, 81.465)), module, SplitStack::VEL_A_OUTPUT));
		addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(38.1, 81.465)), module, SplitStack::VEL_B_OUTPUT));
		addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(22.86, 96.469)), module, SplitStack::AFT_A_OUTPUT));
		addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(38.1, 96.469)), module, SplitStack::AFT_B_OUTPUT));
		addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(22.86, 111.473)), module, SplitStack::RETRIG_A_OUTPUT));
		addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(38.1, 111.473)), module, SplitStack::RETRIG_B_OUTPUT));

	}
};


Model* modelSplitStack = createModel<SplitStack, SplitStackWidget>("SplitStack");

}
