#include "plugin.hpp"
#include "components/componentLibrary.hpp"
#include "components/ModuleWithCustomParamContextMenu.hpp"

#include "blocks/ADSRBlock.hpp"
#include "blocks/DriftBlock.hpp"
#include "blocks/FilterBlock.hpp"
#include "blocks/LFOBlock.hpp"
#include "blocks/OscillatorsBlock.hpp"

#include "dsp/decimator.hpp"
#include "dsp/filters.hpp"

#include <array>

namespace musx {

using namespace rack;

struct Synth : ModuleWithCustomParamContextMenu {
	enum ParamId {
		// assign params
		VOCT_ASSIGN_PARAM,
		GATE_ASSIGN_PARAM,
		VELOCITY_ASSIGN_PARAM,
		AFTERTOUCH_ASSIGN_PARAM,
		PITCH_WHEEL_ASSIGN_PARAM,
		MOD_WHEEL_ASSIGN_PARAM,
		EXPRESSION_ASSIGN_PARAM,
		INDIVIDUAL_MOD_1_ASSIGN_PARAM,
		INDIVIDUAL_MOD_2_ASSIGN_PARAM,
		VOICE_NR_ASSIGN_PARAM,
		RANDOM_ASSIGN_PARAM,

		ENV1_ASSIGN_PARAM,
		ENV2_ASSIGN_PARAM,
		LFO1_UNIPOLAR_ASSIGN_PARAM,
		LFO1_BIPOLAR_ASSIGN_PARAM,
		LFO2_UNIPOLAR_ASSIGN_PARAM,
		LFO2_BIPOLAR_ASSIGN_PARAM,
		GLOBAL_LFO_ASSIGN_PARAM,
		DIVERGE_1_ASSIGN_PARAM,
		DIVERGE_2_ASSIGN_PARAM,
		DRIFT_1_ASSIGN_PARAM,
		DRIFT_2_ASSIGN_PARAM,

		// modulatable params
		ENV1_A_PARAM,
		ENV1_D_PARAM,
		ENV1_S_PARAM,
		ENV1_R_PARAM,

		ENV2_A_PARAM,
		ENV2_D_PARAM,
		ENV2_S_PARAM,
		ENV2_R_PARAM,

		LFO1_FREQ_PARAM,
		LFO1_AMOUNT_PARAM,

		LFO2_FREQ_PARAM,
		LFO2_AMOUNT_PARAM,

		GLOBAL_LFO_AMT_PARAM,

		DRIFT_AMOUNT_PARAM,

		INDIVIDUAL_MOD_OUT_1_PARAM,
		INDIVIDUAL_MOD_OUT_2_PARAM,
		INDIVIDUAL_MOD_OUT_3_PARAM,
		INDIVIDUAL_MOD_OUT_4_PARAM,
		INDIVIDUAL_MOD_OUT_5_PARAM,

		OSC1_TUNE_GLIDE_PARAM,
		OSC1_TUNE_SEMI_PARAM,
		OSC1_TUNE_FINE_PARAM,
		OSC1_SHAPE_PARAM,
		OSC1_PW_PARAM,

		OSC2_TUNE_GLIDE_PARAM,
		OSC2_TUNE_SEMI_PARAM,
		OSC2_TUNE_FINE_PARAM,
		OSC2_SHAPE_PARAM,
		OSC2_PW_PARAM,

		OSC_FM_AMOUNT_PARAM,

		FILTER1_CUTOFF_PARAM,
		FILTER1_RESONANCE_PARAM,
		FILTER1_PAN_PARAM,

		FILTER2_CUTOFF_PARAM,
		FILTER2_RESONANCE_PARAM,
		FILTER2_PAN_PARAM,

		FILTER_SERIAL_PARALLEL_PARAM,

		AMP_VOL_PARAM,

		// mix params
		OSC1_VOL_PARAM,
		OSC1_SUB_VOL_PARAM,
		OSC_RM_VOL_PARAM,
		OSC_NOISE_VOL_PARAM,
		OSC2_VOL_PARAM,
		OSC_EXT_VOL_PARAM,

		// non modulatable params
		ENV1_VEL_PARAM,
		ENV2_VEL_PARAM,
		LFO1_SHAPE_PARAM,
		LFO1_MODE_PARAM,
		LFO2_SHAPE_PARAM,
		LFO2_MODE_PARAM,
		GLOBAL_LFO_FREQ_PARAM,
		DRIFT_RATE_PARAM,

		OSC1_TUNE_OCT_PARAM,
		OSC2_TUNE_OCT_PARAM,
		OSC_TUNE_GLIDE_FINGERED_PARAM,
		OSC_MIX_ROUTE_PARAM,
		OSC_SYNC_PARAM,

		FILTER1_MODE_PARAM,
		FILTER2_CUTOFF_MODE_PARAM,
		FILTER2_MODE_PARAM,

		PARAMS_LEN
	};
	enum InputId {
		VOCT_INPUT,
		GATE_INPUT,
		VELOCITY_INPUT,
		AFTERTOUCH_INPUT,
		PITCH_WHEEL_INPUT,
		MOD_WHEEL_INPUT,
		EXPRESSION_INPUT,
		INDIVIDUAL_MOD_1_INPUT,
		INDIVIDUAL_MOD_2_INPUT,
		RETRIGGER_INPUT,
		EXT_INPUT,
		INPUTS_LEN
	};
	enum OutputId {
		INDIVIDUAL_MOD_1_OUTPUT,
		INDIVIDUAL_MOD_2_OUTPUT,
		INDIVIDUAL_MOD_3_OUTPUT,
		INDIVIDUAL_MOD_4_OUTPUT,
		INDIVIDUAL_MOD_5_OUTPUT,
		OUT_L_OUTPUT,
		OUT_R_OUTPUT,
		OUTPUTS_LEN
	};
	enum LightId {
		// assign lights
		VOCT_ASSIGN_LIGHT,
		GATE_ASSIGN_LIGHT,
		VELOCITY_ASSIGN_LIGHT,
		AFTERTOUCH_ASSIGN_LIGHT,
		PITCH_WHEEL_ASSIGN_LIGHT,
		MOD_WHEEL_ASSIGN_LIGHT,
		EXPRESSION_ASSIGN_LIGHT,
		INDIVIDUAL_MOD_1_ASSIGN_LIGHT,
		INDIVIDUAL_MOD_2_ASSIGN_LIGHT,
		VOICE_NR_ASSIGN_LIGHT,
		RANDOM_ASSIGN_LIGHT,

		ENV1_ASSIGN_LIGHT,
		ENV2_ASSIGN_LIGHT,
		LFO1_UNIPOLAR_ASSIGN_LIGHT,
		LFO1_BIPOLAR_ASSIGN_LIGHT,
		LFO2_UNIPOLAR_ASSIGN_LIGHT,
		LFO2_BIPOLAR_ASSIGN_LIGHT,
		GLOBAL_LFO_ASSIGN_LIGHT,
		DIVERGE_1_ASSIGN_LIGHT,
		DIVERGE_2_ASSIGN_LIGHT,
		DRIFT_1_ASSIGN_LIGHT,
		DRIFT_2_ASSIGN_LIGHT,

		// mix route light
		OSC_MIX_ROUTE_LIGHT,
		OSC_TUNE_GLIDE_FINGERED_LIGHT,
		OSC_SYNC_LIGHT,

		LIGHTS_LEN
	};

	//
	int channels = 1;

	ModuleWidget* widget = nullptr;

	// over/-undersampling, quality
	int lockQualitySettings = -1;
	static const size_t maxOversamplingRate = 16;
	size_t oversamplingRate = 8;
	size_t sampleRate = 48000;

	HalfBandDecimatorCascade<float_4> decimator;

	dsp::ClockDivider uiDivider;
	dsp::ClockDivider modDivider;

	Method filterMethod = Method::RK2;
	IntegratorType filterIntegratorType = IntegratorType::Transistor_tanh;

	// mod matrix
	static constexpr size_t nSources = ENV1_A_PARAM + 1; // number of modulation sources, + 1 for base vale
	static constexpr size_t nMixChannels = 6;
	static constexpr size_t nDestinations = ENV1_VEL_PARAM - ENV1_A_PARAM + nMixChannels; // number of modulation destinations, additional 6 for mix to filter balance

	size_t activeSourceAssign = 0; // index of active mod source assign button. 0 = base value / no button active

	bool oscMixRouteActive = false; // is the OSC_MIX_ROUTE_PARAM button pressed?
	float mixLevels[nMixChannels] = {0.};
	float mixFilterBalances[nMixChannels] = {0.};

	float_4 modMatrixInputs[nSources][4] = {{0}};
	float_4 modMatrixOutputs[nDestinations][4] = {{0}};

	float modMatrix[nDestinations][nSources] = {{0}}; // the mod matrix

	bool mustCalculateDestination[nDestinations] = {false}; // false if all but the first entry of the mod matrix column are 0

	// modulation blocks
	float_4 lastGate[4] = {0.f};
	float_4 lastTrigger[4] = {0.f};

	static constexpr float MIN_TIME = 1e-3f;
	static constexpr float MAX_TIME = 10.f;
	static constexpr float LAMBDA_BASE = MAX_TIME / MIN_TIME;
	static constexpr float ATT_TARGET = 1.2f;
	musx::ADSRBlock env1[4] = {musx::ADSRBlock(MIN_TIME, MAX_TIME, ATT_TARGET)};
	musx::ADSRBlock env2[4] = {musx::ADSRBlock(MIN_TIME, MAX_TIME, ATT_TARGET)};

	musx::LFOBlock lfo1[4];
	musx::LFOBlock lfo2[4];
	musx::LFOBlock globalLfo;
	musx::DriftBlock drift1[4];
	musx::DriftBlock drift2[4];

	// audio blocks
	musx::TOnePoleZDF<float_4> glide1[4];
	musx::TOnePoleZDF<float_4> glide2[4];
	musx::OscillatorsBlock<maxOversamplingRate> oscillators[4];

	float_4 noiseVol1[4] = {0.f};
	float_4 noiseVol2[4] = {0.f};

	float_4 lastExtIn[4] = {0.f};
	float_4 extVol1[4] = {0.f};
	float_4 extVol2[4] = {0.f};

	const float filterMinFreq = 20.f; // min freq [Hz]
	const float filterMaxFreq = 20480.f; // max freq [Hz] // must be 10 octaves for 1V/Oct cutoff CV scaling to work!
	const float filterBase = filterMaxFreq/filterMinFreq; // max freq/min freq
	const float filterLogBase = std::log(filterBase);

	musx::TOnePole<float_4> dcBlocker1[4];
	musx::AliasReductionFilter<float_4> aliasFilter1[4];
	musx::FilterBlock filter1[4];
	musx::AntialiasedCheapSaturator<float_4> saturator1[4];
	float_4 delayBuffer1[4][maxOversamplingRate] = {{0.f}};

	int filter2CutoffMode = 0; // 0: individual, 1: offset, 2: space
	float_4 delayBuffer2[4][maxOversamplingRate] = {{0.f}};
	musx::TOnePole<float_4> dcBlocker2[4];
	musx::AliasReductionFilter<float_4> aliasFilter2[4];
	musx::FilterBlock filter2[4];
	musx::AntialiasedCheapSaturator<float_4> saturator2[4];

	// misc
	bool doRandomize = false;
	bool doReset = false;
	bool jsonLoaded = false;

	Synth() {
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
		configParam(ENV1_VEL_PARAM, 0.f, 1.f, 0.f, "Envelope 1 velocity scaling", " %", 0, 100);
		configParam(ENV2_VEL_PARAM, 0.f, 1.f, 0.f, "Envelope 2 velocity scaling", " %", 0, 100);
		configSwitch(LFO1_SHAPE_PARAM, 0, LFOBlock::getShapeLabels().size() - 1, 0, "LFO 1 shape", LFOBlock::getShapeLabels());
		configSwitch(LFO1_MODE_PARAM, 0, 2, 0, "LFO 1 mode", {"free running", "retrigger", "retrigger, single cycle"});
		configSwitch(LFO2_SHAPE_PARAM, 0, LFOBlock::getShapeLabels().size() - 1, 0, "LFO 2 shape", LFOBlock::getShapeLabels());
		configSwitch(LFO2_MODE_PARAM, 0, 2, 0, "LFO 2 mode", {"free running", "retrigger", "retrigger, single cycle"});
		configParam(GLOBAL_LFO_FREQ_PARAM, -5.f, 5.f, 0.f, "Global LFO frequency", " Hz", 2.f, 2.f);
		configParam(DRIFT_RATE_PARAM, 0.f, 1.f, 0.5f, "Drift rate", " %", 0, 100.);
		configParam(OSC1_TUNE_OCT_PARAM, -4, 4, 0, "Oscillator 1 octave");
		getParamQuantity(OSC1_TUNE_OCT_PARAM)->snapEnabled = true;
		getParamQuantity(OSC1_TUNE_OCT_PARAM)->smoothEnabled = false;
		configParam(OSC2_TUNE_OCT_PARAM, -4, 4, 0, "Oscillator 2 octave");
		getParamQuantity(OSC2_TUNE_OCT_PARAM)->snapEnabled = true;
		getParamQuantity(OSC2_TUNE_OCT_PARAM)->smoothEnabled = false;
		configSwitch(OSC_TUNE_GLIDE_FINGERED_PARAM, 0,   1,   0,  "Fingered glide", {"Off", "On"});
		configSwitch(OSC_SYNC_PARAM, 0,   1,   0,  "Sync", {"Off", "Sync oscillator 2 to oscillator 1"});
		SwitchQuantity* sq = configSwitch(OSC_MIX_ROUTE_PARAM, 0, 1, 0, "Adjust mixer routing to filter 1 / filter 2", {"off", "active"});
		sq->ParamQuantity::randomizeEnabled = false;
		sq->ParamQuantity::resetEnabled = false;
		configSwitch(FILTER1_MODE_PARAM, 0, FilterBlock::getModeLabels().size() - 1, 8, "Filter 1 mode", FilterBlock::getModeLabels());
		configSwitch(FILTER2_CUTOFF_MODE_PARAM, 0, 2, 0, "Filter 2 cutoff mode", {"individual", "offset", "space"});
		configSwitch(FILTER2_MODE_PARAM, 0, FilterBlock::getModeLabels().size() - 1, 8, "Filter 2 mode", FilterBlock::getModeLabels());

		configInput(VOCT_INPUT, "V/Oct");
		configInput(GATE_INPUT, "Gate");
		configInput(VELOCITY_INPUT, "Velocity");
		configInput(AFTERTOUCH_INPUT, "Aftertouch");
		configInput(PITCH_WHEEL_INPUT, "Pitch wheel");
		configInput(MOD_WHEEL_INPUT, "Mod wheel");
		configInput(EXPRESSION_INPUT, "Expression");
		configInput(INDIVIDUAL_MOD_1_INPUT, "Indvidual modulation 1");
		configInput(INDIVIDUAL_MOD_2_INPUT, "Indvidual modulation 2");
		configInput(RETRIGGER_INPUT, "Retrigger");
		configInput(EXT_INPUT, "External audio");
		configOutput(INDIVIDUAL_MOD_1_OUTPUT, "Indvidual modulation 1");
		configOutput(INDIVIDUAL_MOD_2_OUTPUT, "Indvidual modulation 2");
		configOutput(INDIVIDUAL_MOD_3_OUTPUT, "Indvidual modulation 3");
		configOutput(INDIVIDUAL_MOD_4_OUTPUT, "Indvidual modulation 4");
		configOutput(INDIVIDUAL_MOD_5_OUTPUT, "Indvidual modulation 5");
		configOutput(OUT_L_OUTPUT, "Left/Mono");
		configOutput(OUT_R_OUTPUT, "Right");

		setOversamplingRate(oversamplingRate);

		configureUi(true);

		uiDivider.setDivision(16);
		modDivider.setDivision(2);

		configureDrift();
	}

	void loadTemplate()
	{
		if (!widget || jsonLoaded)
		{
			return;
		}

		Model* m = getModel();
		if (!m)
		{
			return;
		}

		std::string filename = m->getFactoryPresetDirectory() + "/template.vcvm";

		widget->load(filename);
	}

	static const std::array<std::string, nSources>& getSourceLabels()
	{
		static const std::array<std::string, nSources> sourceLabelMap = {
			"V/Oct",
			"gate",
			"velocity",
			"aftertouch",
			"pitch wheel",
			"mod wheel",
			"expression pedal",
			"indvidual modulation 1",
			"indvidual modulation 2",
			"voice number",
			"random",

			"envelope 1",
			"envelope 2",
			"LFO 1 (unipolar)",
			"LFO 1 (bipolar)",
			"LFO 2 (unipolar)",
			"LFO 2 (bipolar)",
			"global LFO (bipolar, monophonic)",
			"diverge 1",
			"diverge 2",
			"drift 1",
			"drift 2",
		};

		return sourceLabelMap;
	}

	static const std::array<std::string, nDestinations>& getDestinationLabels()
	{
		static const std::array<std::string, nDestinations> destinationLabelMap = {
			"envelope 1 Attack",
			"envelope 1 Decay",
			"envelope 1 Sustain",
			"envelope 1 Release",

			"envelope 2 Attack",
			"envelope 2 Decay",
			"envelope 2 Sustain",
			"envelope 2 Release",

			"LFO 1 frequency",
			"LFO 1 amount",

			"LFO 2 frequency",
			"LFO 2 amount",

			"global LFO amount",

			"diverge & drift amount",

			"individual modulation 1",
			"individual modulation 2",
			"individual modulation 3",
			"individual modulation 4",
			"individual modulation 5",

			"oscillator 1 glide",
			"oscillator 1 semitones",
			"oscillator 1 fine tune",
			"oscillator 1 shape",
			"oscillator 1 triangle phase / pulse width",

			"oscillator 2 glide offset",
			"oscillator 2 semitones",
			"oscillator 2 fine tune",
			"oscillator 2 shape",
			"oscillator 2 triangle phase / pulse width",

			"oscillator 1 to oscillator 2 FM amount",

			"filter 1 cutoff frequency",
			"filter 1 resonance",
			"filter 1 pan",

			"filter 2 cutoff frequency",
			"filter 2 resonance",
			"filter 2 pan",

			"filter routing: serial / parallel",

			"amp volume",

			"oscillator 1",
			"oscillator 1 sub-oscillator",
			"ring modulator",
			"noise",
			"oscillator 2",
			"external audio input / loopback",
		};

		return destinationLabelMap;
	}

	void configureUi(bool initial = false)
	{
		const auto& sourceLabels = getSourceLabels();
		const auto& destinationLabels = getDestinationLabels();

		// bring "Modulates:" labels into correct order
		std::vector<size_t> destIds;
		for (size_t i = 0; i < FILTER1_CUTOFF_PARAM - ENV1_A_PARAM; i++)
		{
			destIds.push_back(i);
		}
		for (size_t i = OSC1_VOL_PARAM - ENV1_A_PARAM; i < OSC1_VOL_PARAM - ENV1_A_PARAM + 2 * nMixChannels; i++)
		{
			destIds.push_back(i);
		}
		for (size_t i = FILTER1_CUTOFF_PARAM - ENV1_A_PARAM; i < OSC1_VOL_PARAM - ENV1_A_PARAM; i++)
		{
			destIds.push_back(i);
		}

		for (size_t iSource = 0; iSource < sourceLabels.size(); iSource++)
		{
			bool isModulating = false;
			std::string modulatesLabel = "\nModulates:\n";
			for (size_t iDest : destIds)
			{
				if (modMatrix[iDest][iSource + 1] != 0.f)
				{
					std::string label = "";
					if (iDest >= nDestinations - nMixChannels)
					{
						label = destinationLabels[iDest - nMixChannels] + " routing (filter 1 / filter 2)";
					}
					else if (iDest >= nDestinations - 2 * nMixChannels)
					{
						label = destinationLabels[iDest] + " volume";
					}
					else
					{
						label = destinationLabels[iDest];
					}
					label += "\n";
					label[0] = toupper(label[0]);
					modulatesLabel += label;
					isModulating = true;
				}
			}

			SwitchQuantity* sw = nullptr;
			if (initial)
			{
				sw = configSwitch(iSource, 0, 1, 0, "Assign " + sourceLabels[iSource], {"off", "active"});
			}
			else
			{
				sw = dynamic_cast<SwitchQuantity*>(getParamQuantity(iSource));
			}
			if (sw)
			{
				sw->randomizeEnabled = false;
				sw->description = "";
				if (isModulating)
				{
					sw->description = modulatesLabel;
				}
			}

			if (iSource + 1 == activeSourceAssign)
			{
				params[iSource].setValue(1);
			}
		}

		for (size_t i = 0; i < nDestinations - 2 * nMixChannels; i++)
		{
			// config knobs when source assign is active
			if (activeSourceAssign)
			{
				std::string sourceLabel = sourceLabels[activeSourceAssign - 1];
				BipolarColorParamQuantity* param;

				switch(ENV1_A_PARAM + i)
				{
				case OSC1_TUNE_SEMI_PARAM:
				case OSC2_TUNE_SEMI_PARAM:
					switch (activeSourceAssign - 1)
					{
					case VOCT_ASSIGN_PARAM:
						param = configParamBipolarColorParamQuantity(initial, ENV1_A_PARAM + i, -10.f, 10.f, 5.f,
								"Assign " + sourceLabel + " to " + destinationLabels[i],
								" %", 0, 20.);
						break;
					case PITCH_WHEEL_ASSIGN_PARAM:
						param = configParamBipolarColorParamQuantity(initial, ENV1_A_PARAM + i, -2.f, 2.f, 0.16666666f,
								"Assign " + sourceLabel + " to " + destinationLabels[i],
								" semitones", 0, 12.);
						break;
					default:
						param = configParamBipolarColorParamQuantity(initial, ENV1_A_PARAM + i, -2.f, 2.f, 0.f,
								"Assign " + sourceLabel + " to " + destinationLabels[i],
								" %", 0, 50.);
					}
					param->snapEnabled = false;
					param->smoothEnabled = true;
					break;
				case FILTER1_CUTOFF_PARAM:
				case FILTER2_CUTOFF_PARAM:
					switch (activeSourceAssign - 1)
					{
					case VOCT_ASSIGN_PARAM:
						param = configParamBipolarColorParamQuantity(initial, ENV1_A_PARAM + i, -2.f, 2.f, 1.f,
								"Assign " + sourceLabel + " to " + destinationLabels[i],
								" %", 0, 100.);
						break;
					case PITCH_WHEEL_ASSIGN_PARAM:
						param = configParamBipolarColorParamQuantity(initial, ENV1_A_PARAM + i, -0.4f, 0.4f, 0.033333333f,
								"Assign " + sourceLabel + " to " + destinationLabels[i],
								" semitones", 0, 60.);
						break;
					default:
						param = configParamBipolarColorParamQuantity(initial, ENV1_A_PARAM + i, -2.f, 2.f, 0.f,
								"Assign " + sourceLabel + " to " + destinationLabels[i],
								" %", 0, 50.);
					}
					break;
				default:
					param = configParamBipolarColorParamQuantity(initial, ENV1_A_PARAM + i, -1.f, 1.f, 0.f,
							"Assign " + sourceLabel + " to " + destinationLabels[i],
							" %", 0, 100.);
				}

				param->bipolar = true;
				param->color = SCHEME_BLUE;

				getParam(ENV1_A_PARAM + i).setValue(modMatrix[i][activeSourceAssign]);
			}
			// config knobs when source assign is off
			else
			{
				std::string destinationLabel = destinationLabels[i];
				destinationLabel[0] = toupper(destinationLabel[0]);
				BipolarColorParamQuantity* param;

				switch(ENV1_A_PARAM + i)
				{
				case ENV1_A_PARAM:
				case ENV1_D_PARAM:
				case ENV1_R_PARAM:
				case ENV2_A_PARAM:
				case ENV2_D_PARAM:
				case ENV2_R_PARAM:
					param = configParamBipolarColorParamQuantity(initial, ENV1_A_PARAM + i, 0.f, 1.f, 0.f,
							destinationLabel,
							" ms", LAMBDA_BASE, MIN_TIME * 1000.0);
					param->bipolar = false;
					getParam(ENV1_A_PARAM + i).setValue(0.1f * modMatrix[i][activeSourceAssign]);
					break;
				case LFO1_FREQ_PARAM:
				case LFO2_FREQ_PARAM:
					param = configParamBipolarColorParamQuantity(initial, ENV1_A_PARAM + i, -5.f, 10.f, 0.f,
							destinationLabel,
							" Hz", 2.f, 2.f);
					param->bipolar = false;
					getParam(ENV1_A_PARAM + i).setValue(modMatrix[i][activeSourceAssign]);
					break;
				case OSC1_TUNE_GLIDE_PARAM:
					param = configParamBipolarColorParamQuantity(initial, ENV1_A_PARAM + i, 0.f, 1.f, 0.f,
							destinationLabel,
							" ms",
							getGlideFreq(0.f, 40000.f)[0] / getGlideFreq(10.f, 40000.f)[0],
							modDivider.getDivision() / (40000.f * getGlideFreq(0.f, 40000.f)[0]) * 183.939720586f);
					param->bipolar = false;
					getParam(ENV1_A_PARAM + i).setValue(0.1f * modMatrix[i][activeSourceAssign]);
					break;
				case OSC1_TUNE_SEMI_PARAM:
				case OSC2_TUNE_SEMI_PARAM:
					param = configParamBipolarColorParamQuantity(initial, ENV1_A_PARAM + i, -12.f, 12.f, 0.f,
							destinationLabel,
							" semitones");
					param->bipolar = true;
					param->snapEnabled = true;
					param->smoothEnabled = false;
					getParam(ENV1_A_PARAM + i).setValue(12.f / 5.f * modMatrix[i][activeSourceAssign]);
					break;
				case OSC1_TUNE_FINE_PARAM:
				case OSC2_TUNE_FINE_PARAM:
					param = configParamBipolarColorParamQuantity(initial, ENV1_A_PARAM + i, -5.f, 5.f, 0.f,
							destinationLabel,
							" cents", 0, 20.f);
					param->bipolar = true;
					getParam(ENV1_A_PARAM + i).setValue(modMatrix[i][activeSourceAssign]);
					break;
				case FILTER1_CUTOFF_PARAM:
				case FILTER2_CUTOFF_PARAM:
					if (ENV1_A_PARAM + i == FILTER2_CUTOFF_PARAM && filter2CutoffMode)
					{
						param = configParamBipolarColorParamQuantity(initial, ENV1_A_PARAM + i, 0.f, 1.f, 0.5f,
								destinationLabel,
								" %", 0, 200.f, -100.f);
						param->bipolar = true;
					}
					else
					{
						param = configParamBipolarColorParamQuantity(initial, ENV1_A_PARAM + i, 0.f, 1.f, 1.f,
								destinationLabel,
								" Hz", filterMaxFreq / filterMinFreq, filterMinFreq);
						param->bipolar = false;
					}
					getParam(ENV1_A_PARAM + i).setValue(0.1f * modMatrix[i][activeSourceAssign]);
					break;
				case INDIVIDUAL_MOD_OUT_1_PARAM:
				case INDIVIDUAL_MOD_OUT_2_PARAM:
				case INDIVIDUAL_MOD_OUT_3_PARAM:
				case INDIVIDUAL_MOD_OUT_4_PARAM:
				case INDIVIDUAL_MOD_OUT_5_PARAM:
				case OSC2_TUNE_GLIDE_PARAM:
				case FILTER1_PAN_PARAM:
				case FILTER2_PAN_PARAM:
					// bipolar
					param = configParamBipolarColorParamQuantity(initial, ENV1_A_PARAM + i, -5.f, 5.f, 0.f,
							destinationLabel,
							" %", 0, 20.f);
					param->bipolar = true;
					getParam(ENV1_A_PARAM + i).setValue(modMatrix[i][activeSourceAssign]);
					break;
				case ENV1_S_PARAM:
				case ENV2_S_PARAM:
					param = configParamBipolarColorParamQuantity(initial, ENV1_A_PARAM + i, 0.f, 10.f, 10.f,
							destinationLabel,
							" %", 0, 10.f);
					param->bipolar = false;
					getParam(ENV1_A_PARAM + i).setValue(modMatrix[i][activeSourceAssign]);
					break;
				case OSC1_SHAPE_PARAM:
				case OSC1_PW_PARAM:
				case OSC2_SHAPE_PARAM:
				case OSC2_PW_PARAM:
				case AMP_VOL_PARAM:
					param = configParamBipolarColorParamQuantity(initial, ENV1_A_PARAM + i, 0.f, 10.f, 5.f,
							destinationLabel,
							" %", 0, 10.f);
					param->bipolar = false;
					getParam(ENV1_A_PARAM + i).setValue(modMatrix[i][activeSourceAssign]);
					break;
				default:
					// unipolar
					param = configParamBipolarColorParamQuantity(initial, ENV1_A_PARAM + i, 0.f, 10.f, 0.f,
							destinationLabel,
							" %", 0, 10.f);
					param->bipolar = false;
					getParam(ENV1_A_PARAM + i).setValue(modMatrix[i][activeSourceAssign]);
				}

				param->color = SCHEME_GREEN;
				param->indicator = mustCalculateDestination[i];
				param->indicatorColor = SCHEME_BLUE;

				param->modulatedByTooltips.clear();
				if (mustCalculateDestination[i])
				{
					for (size_t iSource = 1; iSource < nSources; iSource++)
					{
						if (modMatrix[i][iSource] != 0.f)
						{
							param->modulatedByTooltips.push_back(sourceLabels[iSource - 1]);
						}
					}
				}
			}
		}

		for (size_t i = nDestinations - 2 * nMixChannels; i < nDestinations - nMixChannels; i++)
		{
			if (oscMixRouteActive)
			{
				// config mix route knobs when source assign is active
				if (activeSourceAssign)
				{
					std::string sourceLabel = sourceLabels[activeSourceAssign - 1];

					BipolarColorParamQuantity* param = configParamBipolarColorParamQuantity(initial, ENV1_A_PARAM + i, -1.f, 1.f, 0.f,
							"Assign " + sourceLabel + " to " + destinationLabels[i] + " routing (filter 1 / filter 2)",
							" %", 0, 100.);

					param->bipolar = true;
					param->color = SCHEME_PURPLE;
				}
				// config mix route knobs when source assign is off
				else
				{
					std::string destinationLabel = destinationLabels[i];
					destinationLabel[0] = toupper(destinationLabel[0]);
					BipolarColorParamQuantity* param = configParamBipolarColorParamQuantity(initial, ENV1_A_PARAM + i, -5.f, 5.f, 0.f,
							destinationLabel + " routing (filter 1 / filter 2)",
							" %", 0, 20.);

					param->bipolar = true;
					param->color = SCHEME_RED;
					param->indicator = mustCalculateDestination[nMixChannels + i];
					param->indicatorColor = SCHEME_PURPLE;

					param->modulatedByTooltips.clear();
					if (mustCalculateDestination[nMixChannels + i])
					{
						for (size_t iSource = 1; iSource < nSources; iSource++)
						{
							if (modMatrix[nMixChannels + i][iSource] != 0.f)
							{
								param->modulatedByTooltips.push_back(sourceLabels[iSource - 1]);
							}
						}
					}
				}
				getParam(ENV1_A_PARAM + i).setValue(modMatrix[nMixChannels + i][activeSourceAssign]);
			}
			else
			{
				// config mix volume knobs when source assign is active
				if (activeSourceAssign)
				{
					std::string sourceLabel = sourceLabels[activeSourceAssign - 1];

					BipolarColorParamQuantity* param = configParamBipolarColorParamQuantity(initial, ENV1_A_PARAM + i, -1.f, 1.f, 0.f,
							"Assign " + sourceLabel + " to " + destinationLabels[i] + " volume",
							" %", 0, 100.);

					param->bipolar = true;
					param->color = SCHEME_BLUE;
				}
				// config mix volume knobs when source assign is off
				else
				{
					std::string destinationLabel = destinationLabels[i];
					destinationLabel[0] = toupper(destinationLabel[0]);
					BipolarColorParamQuantity* param = configParamBipolarColorParamQuantity(initial, ENV1_A_PARAM + i, 0.f, 10.f,
							(i == nDestinations - 2 * nMixChannels) ? 5.f : 0.f,
							destinationLabel + " volume",
							" %", 0, 10.);

					param->bipolar = false;
					param->color = SCHEME_GREEN;
					param->indicator = mustCalculateDestination[i];
					param->indicatorColor = SCHEME_BLUE;

					param->modulatedByTooltips.clear();
					if (mustCalculateDestination[i])
					{
						for (size_t iSource = 1; iSource < nSources; iSource++)
						{
							if (modMatrix[i][iSource] != 0.f)
							{
								param->modulatedByTooltips.push_back(sourceLabels[iSource - 1]);
							}
						}
					}
				}
				getParam(ENV1_A_PARAM + i).setValue(modMatrix[i][activeSourceAssign]);
			}
		}

		// set lights
		for (size_t i = 0; i < OSC_MIX_ROUTE_LIGHT; i++)
		{
			bool isModulating = false;

			for (size_t iDest = 0; iDest < nDestinations; iDest++)
			{
				if (modMatrix[iDest][i + 1] != 0.f)
				{
					isModulating = true;
					break;
				}
			}

			if (i == activeSourceAssign - 1)
			{
				lights[i].setBrightness(1.f);
			}
			else if (isModulating)
			{
				lights[i].setBrightness(0.25f);
			}
			else
			{
				lights[i].setBrightness(0.f);
			}
		}

		if (oscMixRouteActive)
		{
			lights[OSC_MIX_ROUTE_LIGHT].setBrightness(1.f);
		}
		else if (activeSourceAssign == 0)
		{
			lights[OSC_MIX_ROUTE_LIGHT].setBrightness(0.f);
		}
		else
		{
			bool isModulatingMixRoute = false;

			for (size_t iDest = nDestinations - nMixChannels; iDest < nDestinations; iDest++)
			{
				if (modMatrix[iDest][activeSourceAssign] != 0.f)
				{
					isModulatingMixRoute = true;
					break;
				}
			}

			if (isModulatingMixRoute)
			{
				lights[OSC_MIX_ROUTE_LIGHT].setBrightness(0.25f);
			}
			else
			{
				lights[OSC_MIX_ROUTE_LIGHT].setBrightness(0.f);
			}
		}

		// force redraw
		if (widget)
		{
			for (ParamWidget* paramWidget : widget->getParams())
			{
				Widget::ChangeEvent e;
				paramWidget->onChange(e);
			}
		}
	}

	BipolarColorParamQuantity* configParamBipolarColorParamQuantity(bool initial, int paramId, float minValue, float maxValue, float defaultValue, std::string name = "", std::string unit = "", float displayBase = 0.f, float displayMultiplier = 1.f, float displayOffset = 0.f)
	{
		// calling configParam when running can lead to SEGFAULTS
		// call configParam only from Synths's constructor (initial = true)
		// otherwise just update pq's internal variables

		BipolarColorParamQuantity* pq = nullptr;
		if (initial)
		{
			pq = configParam<BipolarColorParamQuantity>(paramId, minValue, maxValue, defaultValue, name, unit, displayBase, displayMultiplier, displayOffset);
		}
		else
		{
			pq = dynamic_cast<BipolarColorParamQuantity*>(getParamQuantity(paramId));
		}
		if (pq)
		{
			pq->minValue = minValue;
			pq->maxValue = maxValue;
			pq->defaultValue = defaultValue;
			pq->name = name;
			pq->unit = unit;
			pq->displayBase = displayBase;
			pq->displayMultiplier = displayMultiplier;
			pq->displayOffset = displayOffset;
		}
		return pq;
	}

	void processUi()
	{
		channels = std::max(1, inputs[VOCT_INPUT].getChannels());

		outputs[INDIVIDUAL_MOD_1_OUTPUT].setChannels(channels);
		outputs[INDIVIDUAL_MOD_2_OUTPUT].setChannels(channels);
		outputs[INDIVIDUAL_MOD_3_OUTPUT].setChannels(channels);
		outputs[INDIVIDUAL_MOD_4_OUTPUT].setChannels(channels);
		outputs[INDIVIDUAL_MOD_5_OUTPUT].setChannels(channels);

		// update activeSourceAssign and oscMixRouteActive
		size_t newActiveSourceAssign = 0;
		for (size_t i = 0; i < ENV1_A_PARAM; i++)
		{
			if (params[i].getValue() && i + 1 != activeSourceAssign)
			{
				newActiveSourceAssign = i + 1;
				break;
			}
		}
		if (newActiveSourceAssign == 0 && params[activeSourceAssign - 1].getValue())
		{
			newActiveSourceAssign = activeSourceAssign;
		}

		// switch off other source assign buttons
		for (size_t i = 0; i < ENV1_A_PARAM; i++)
		{
			if (i + 1 != newActiveSourceAssign)
			{
				params[i].setValue(0);
			}
		}


		bool reconfigureUi = false;
		bool newOscMixRouteActive = params[OSC_MIX_ROUTE_PARAM].getValue() > 0.5f;

		// adapt UI if activeSourceAssign or oscMixRouteActive have changed
		if (activeSourceAssign != newActiveSourceAssign ||
				oscMixRouteActive != newOscMixRouteActive ||
				filter2CutoffMode != (int)getParam(FILTER2_CUTOFF_MODE_PARAM).getValue())
		{
			activeSourceAssign = newActiveSourceAssign;
			oscMixRouteActive = newOscMixRouteActive;
			filter2CutoffMode = (int)getParam(FILTER2_CUTOFF_MODE_PARAM).getValue();
			reconfigureUi = true;
			configureUi();
		}

		// reset / randomize when no activeSourceAssign
		if (doReset)
		{
			if (activeSourceAssign == 0)
			{
				ResetEvent e;
				Module::onReset(e);
			}
		}

		if (doRandomize)
		{
			if (activeSourceAssign == 0)
			{
				RandomizeEvent e;
				Module::onRandomize(e);
			}
		}

		// update mod matrix elements
		for (size_t i = 0; i < nDestinations - 2 * nMixChannels; i++)
		{
			if (activeSourceAssign == 0)
			{
				switch (ENV1_A_PARAM + i)
				{
				case ENV1_A_PARAM:
				case ENV1_D_PARAM:
				case ENV1_R_PARAM:
				case ENV2_A_PARAM:
				case ENV2_D_PARAM:
				case ENV2_R_PARAM:
				case OSC1_TUNE_GLIDE_PARAM:
				case FILTER1_CUTOFF_PARAM:
				case FILTER2_CUTOFF_PARAM:
					modMatrix[i][activeSourceAssign] = getParam(ENV1_A_PARAM + i).getValue() * 10.f;
					break;
				case OSC1_TUNE_SEMI_PARAM:
				case OSC2_TUNE_SEMI_PARAM:
					modMatrix[i][activeSourceAssign] = getParam(ENV1_A_PARAM + i).getValue() * 5.f / 12.f;
					break;
				default:
					modMatrix[i][activeSourceAssign] = getParam(ENV1_A_PARAM + i).getValue();
				}
			}
			else
			{
				modMatrix[i][activeSourceAssign] = getParam(ENV1_A_PARAM + i).getValue();
			}
		}

		if (oscMixRouteActive)
		{
			for (size_t i = nDestinations - 2 * nMixChannels; i < nDestinations - nMixChannels; i++)
			{
				modMatrix[i + nMixChannels][activeSourceAssign] = getParam(ENV1_A_PARAM + i).getValue();
			}
		}
		else
		{
			for (size_t i = nDestinations - 2 * nMixChannels; i < nDestinations - nMixChannels; i++)
			{
				modMatrix[i][activeSourceAssign] = getParam(ENV1_A_PARAM + i).getValue();
			}
		}

		// reset / randomize when activeSourceAssign is active
		if (doReset)
		{
			doReset = false;
			reconfigureUi = true;

			if (activeSourceAssign > 0)
			{
				// reset only modulation assignments when activeSourceAssign
				for (size_t iDest = 0; iDest < nDestinations; iDest++)
				{
					modMatrix[iDest][activeSourceAssign] = 0.f;
				}
			}
		}

		if (doRandomize)
		{
			doRandomize = false;
			reconfigureUi = true;

			if (activeSourceAssign > 0)
			{
				// randomize only modulatable params when activeSourceAssign
				for (size_t iDest = 0; iDest < nDestinations; iDest++)
				{
					modMatrix[iDest][activeSourceAssign] = 2.f * rack::random::uniform() - 1.f;
				}
			}
		}

		// update mustCalculateDestination
		for (size_t iDest = 0; iDest < nDestinations; iDest++)
		{
			bool oldMustCalculateDestination = mustCalculateDestination[iDest];
			mustCalculateDestination[iDest] = false;
			for (size_t iSource = 1; iSource < nSources; iSource++)
			{
				if (modMatrix[iDest][iSource] != 0.f)
				{
					mustCalculateDestination[iDest] = true;
					break;
				}
			}

			reconfigureUi |= oldMustCalculateDestination != mustCalculateDestination[iDest];
		}

		if (reconfigureUi)
		{
			// configure UI again to set tooltips
			configureUi();
		}


		// set non-modulatable parameters
		for (int c = 0; c < channels; c += 4) {
			if (channels < 2)
			{
				modMatrixInputs[VOICE_NR_ASSIGN_PARAM + 1][c/4] = 0.f;
			}
			else
			{
				for (int iChannel = c; iChannel < std::min(channels, c + 4); iChannel++)
				{
					modMatrixInputs[VOICE_NR_ASSIGN_PARAM + 1][c/4][iChannel - c] = 10.f * ((iChannel) / (channels - 1.f)) - 5.f;
				}
			}

			env1[c/4].setVelocityScaling(getParam(ENV1_VEL_PARAM).getValue());
			env2[c/4].setVelocityScaling(getParam(ENV2_VEL_PARAM).getValue());

			lfo1[c/4].setShape(getParam(LFO1_SHAPE_PARAM).getValue());
			lfo1[c/4].setSingleCycle(getParam(LFO1_MODE_PARAM).getValue() == 2);
			lfo2[c/4].setShape(getParam(LFO2_SHAPE_PARAM).getValue());
			lfo2[c/4].setSingleCycle(getParam(LFO2_MODE_PARAM).getValue() == 2);

			drift1[c/4].setFilterFrequencyV(getParam(DRIFT_RATE_PARAM).getValue());
			drift2[c/4].setFilterFrequencyV(getParam(DRIFT_RATE_PARAM).getValue());

			oscillators[c/4].setSync(getParam(OSC_SYNC_PARAM).getValue());

			filter1[c/4].setMode(getParam(FILTER1_MODE_PARAM).getValue());
			filter2[c/4].setMode(getParam(FILTER2_MODE_PARAM).getValue());
		}
		globalLfo.setFrequencyVOct(getParam(GLOBAL_LFO_FREQ_PARAM).getValue());


		// lights
		lights[OSC_TUNE_GLIDE_FINGERED_LIGHT].setBrightness(getParam(OSC_TUNE_GLIDE_FINGERED_PARAM).getValue());
		lights[OSC_SYNC_LIGHT].setBrightness(getParam(OSC_SYNC_PARAM).getValue());
	}

	void onSampleRateChange(const SampleRateChangeEvent& e) override {
		sampleRate = e.sampleRate;
		for (int c = 0; c < 16; c += 4) {
			oscillators[c/4].setSampleRate(sampleRate);

			lfo1[c/4].setSampleRate(sampleRate);
			lfo2[c/4].setSampleRate(sampleRate);

			drift1[c/4].setSampleRate(sampleRate);
			drift1[c/4].setFilterFrequencyV(getParam(DRIFT_RATE_PARAM).getValue());
			drift2[c/4].setSampleRate(sampleRate);
			drift2[c/4].setFilterFrequencyV(getParam(DRIFT_RATE_PARAM).getValue());
		}
		globalLfo.setSampleRate(sampleRate);
		setOversamplingRate(oversamplingRate);
	}

	void setOversamplingRate(size_t arg)
	{
		oversamplingRate = arg;

		for (int c = 0; c < 16; c += 4) {
			oscillators[c/4].setOversamplingRate(oversamplingRate);

			dcBlocker1[c/4].setCutoffFreq(20.f/sampleRate/oversamplingRate);
			aliasFilter1[c/4].setCutoffFreq(18000.f/sampleRate/oversamplingRate);

			dcBlocker2[c/4].setCutoffFreq(20.f/sampleRate/oversamplingRate);
			aliasFilter2[c/4].setCutoffFreq(18000.f/sampleRate/oversamplingRate);

			decimator.reset();
		}
	}

	void setModSampleRateReduction(size_t arg)
	{
		modDivider.setDivision(arg);

		for (int c = 0; c < 16; c += 4) {
			lfo1[c/4].setSampleRateReduction(arg);
			lfo2[c/4].setSampleRateReduction(arg);

			drift1[c/4].setSampleRateReduction(arg);
			drift1[c/4].setFilterFrequencyV(getParam(DRIFT_RATE_PARAM).getValue());
			drift2[c/4].setSampleRateReduction(arg);
			drift2[c/4].setFilterFrequencyV(getParam(DRIFT_RATE_PARAM).getValue());
		}
		globalLfo.setSampleRateReduction(arg);
	}

	void configureDrift()
	{
		if (this->getId())
		{
			// use id to get diverge which is unique for every module instance, but stays the same when the patch is loaded again
			rack::random::local().seed(this->getId(), 1103554439654531);
		}

		for (int c = 0; c < 16; c += 4)
		{
			drift1[c/4].randomizeDiverge();
			drift1[c/4].setDivergeAmount(0.f);
			drift1[c/4].setDriftAmount(1.f);

			drift2[c/4].randomizeDiverge();
			drift2[c/4].setDivergeAmount(0.f);
			drift2[c/4].setDriftAmount(1.f);
		}
	}

	void setFilterMethod(Method m)
	{
		filterMethod = m;
		for (int c = 0; c < 16; c += 4)
		{
			filter1[c/4].setMethod(filterMethod);
			filter2[c/4].setMethod(filterMethod);
		}
	}

	void setFilterIntegratorType(IntegratorType t)
	{
		filterIntegratorType = t;
		for (int c = 0; c < 16; c += 4)
		{
			filter1[c/4].setIntegratorType(filterIntegratorType);
			filter2[c/4].setIntegratorType(filterIntegratorType);
		}
	}

	void onReset(const ResetEvent& e) override
	{
		// reset later in audio thread instead of here in UI thread
		doReset = true;
	}

	void onRandomize(const RandomizeEvent& e) override
	{
		// randomize later in audio thread instead of here in UI thread
		doRandomize = true;
	}

	void appendToParamContextMenu(ParamWidget* param, ui::Menu* menu) override
	{
		menu->addChild(new MenuSeparator);
		menu->addChild(createMenuItem(
				"Clear modulations for this destination",
				"",
				[=]() {resetModulationsForDestination(param->paramId);}));
	}

	void resetModulationsForDestination(int paramId)
	{
		if (paramId == -1)
		{
			return;
		}

		size_t iDest = paramId - DRIFT_2_ASSIGN_PARAM - 1;
		if (oscMixRouteActive)
		{
			iDest += nMixChannels;
		}

		for (size_t iSource = 1; iSource < nSources; iSource++)
		{
			modMatrix[iDest][iSource] = 0.f;
		}

		configureUi();
	}

	void appendToSwitchContextMenu(ParamWidget* param, ui::Menu* menu) override
	{
		menu->addChild(new MenuSeparator);
		menu->addChild(createMenuItem(
				"Clear modulations for this source",
				"",
				[=]() {resetModulationsForSource(param->paramId);}));
	}

	void resetModulationsForSource(int paramId)
	{
		if (paramId == -1)
		{
			return;
		}

		size_t iSource = paramId + 1;

		for (size_t iDest = 0; iDest < nDestinations; iDest++)
		{
			modMatrix[iDest][iSource] = 0.f;
		}

		configureUi();
	}

	void process(const ProcessArgs& args) override {

		size_t currentOversamplingRate = oversamplingRate;

		if (uiDivider.process())
		{
			processUi();
		}

		if (modDivider.process())
		{
			float noise1 = rack::random::uniform();
			float noise2 = rack::random::uniform();
			globalLfo.process();
			float_4 globalLfoOut = globalLfo.getBipolar();
			for (int c = 0; c < channels; c += 4) {
				// get modulation inputs
				for (size_t iInput = 0; iInput < INDIVIDUAL_MOD_2_ASSIGN_PARAM; iInput++)
				{
					modMatrixInputs[iInput + 1][c/4] = inputs[iInput].getPolyVoltageSimd<float_4>(c);
				}

				float_4 triggerInput = inputs[GATE_INPUT].getPolyVoltageSimd<float_4>(c) + inputs[RETRIGGER_INPUT].getPolyVoltageSimd<float_4>(c);
				float_4 rnd = {rack::random::uniform(), rack::random::uniform(), rack::random::uniform(), rack::random::uniform()};
				modMatrixInputs[RANDOM_ASSIGN_PARAM + 1][c/4] = ifelse(triggerInput > lastTrigger[c/4] + 0.5f,
						(10.f * rnd) - 5.f,
						modMatrixInputs[RANDOM_ASSIGN_PARAM + 1][c/4]);

				// process modulation blocks
				env1[c/4].setGate(inputs[GATE_INPUT].getPolyVoltageSimd<float_4>(c));
				env1[c/4].setRetrigger(inputs[RETRIGGER_INPUT].getPolyVoltageSimd<float_4>(c));
				env1[c/4].setVelocity(inputs[VELOCITY_INPUT].getPolyVoltageSimd<float_4>(c));
				modMatrixInputs[ENV1_ASSIGN_PARAM + 1][c/4] = env1[c/4].process(args.sampleTime * modDivider.getDivision());

				env2[c/4].setGate(inputs[GATE_INPUT].getPolyVoltageSimd<float_4>(c));
				env2[c/4].setRetrigger(inputs[RETRIGGER_INPUT].getPolyVoltageSimd<float_4>(c));
				env2[c/4].setVelocity(inputs[VELOCITY_INPUT].getPolyVoltageSimd<float_4>(c));
				modMatrixInputs[ENV2_ASSIGN_PARAM + 1][c/4] = env2[c/4].process(args.sampleTime * modDivider.getDivision());

				if (getParam(LFO1_MODE_PARAM).getValue() > 0)
				{
					lfo1[c/4].setReset(triggerInput);
				}
				lfo1[c/4].process();
				modMatrixInputs[LFO1_UNIPOLAR_ASSIGN_PARAM + 1][c/4] = lfo1[c/4].getUnipolar();
				modMatrixInputs[LFO1_BIPOLAR_ASSIGN_PARAM + 1][c/4] = lfo1[c/4].getBipolar();

				if (getParam(LFO2_MODE_PARAM).getValue() > 0)
				{
					lfo2[c/4].setReset(triggerInput);
				}
				lfo2[c/4].process();
				modMatrixInputs[LFO2_UNIPOLAR_ASSIGN_PARAM + 1][c/4] = lfo2[c/4].getUnipolar();
				modMatrixInputs[LFO2_BIPOLAR_ASSIGN_PARAM + 1][c/4] = lfo2[c/4].getBipolar();

				modMatrixInputs[GLOBAL_LFO_ASSIGN_PARAM + 1][c/4] = clamp(modMatrixOutputs[GLOBAL_LFO_AMT_PARAM - ENV1_A_PARAM][c/4], 0.f, 10.f) * globalLfoOut;

				modMatrixInputs[DIVERGE_1_ASSIGN_PARAM + 1][c/4] = 0.2f * modMatrixOutputs[DRIFT_AMOUNT_PARAM - ENV1_A_PARAM][c/4] * drift1[c/4].getDiverge();
				modMatrixInputs[DIVERGE_2_ASSIGN_PARAM + 1][c/4] = 0.2f * modMatrixOutputs[DRIFT_AMOUNT_PARAM - ENV1_A_PARAM][c/4] * drift2[c/4].getDiverge();
				modMatrixInputs[DRIFT_1_ASSIGN_PARAM + 1][c/4] = 0.2f * modMatrixOutputs[DRIFT_AMOUNT_PARAM - ENV1_A_PARAM][c/4] * drift1[c/4].process();
				modMatrixInputs[DRIFT_2_ASSIGN_PARAM + 1][c/4] = 0.2f * modMatrixOutputs[DRIFT_AMOUNT_PARAM - ENV1_A_PARAM][c/4] * drift2[c/4].process();

				// matrix multiplication
				for (size_t iDest = 0; iDest < nDestinations; iDest++)
				{
					modMatrixOutputs[iDest][c/4] = modMatrix[iDest][0];
					if (iDest == AMP_VOL_PARAM - ENV1_A_PARAM)
					{
						for (size_t iSource = 1; iSource < nSources; iSource++)
						{
							float_4 mult = 0.1f * (10.f + modMatrix[iDest][iSource] * (modMatrixInputs[iSource][c/4] - sgn(modMatrix[iDest][iSource]) * 10.f));
							mult -= (modMatrix[iDest][iSource] < 0.f) * modMatrix[iDest][iSource];
							mult = clamp(mult, 0.f, 1.f);
							modMatrixOutputs[iDest][c/4] *= mult;
						}
					}
					else if (mustCalculateDestination[iDest])
					{
						for (size_t iSource = 1; iSource < nSources; iSource++)
						{
							modMatrixOutputs[iDest][c/4] += modMatrix[iDest][iSource] * modMatrixInputs[iSource][c/4];
						}
					}
				}

				// calculate further values
				float_4 noiseAmp = clamp(modMatrixOutputs[OSC_NOISE_VOL_PARAM - ENV1_A_PARAM][c/4], 0.f, 10.f);
				float_4 noiseMix = clamp(0.2f * modMatrixOutputs[OSC_NOISE_VOL_PARAM + nMixChannels - ENV1_A_PARAM][c/4], -1.f, 1.f);
				noiseVol1[c/4] = 0.5f - 0.5f * noiseMix;
				noiseVol2[c/4] = 1.f - noiseVol1[c/4];
				noiseVol1[c/4] *= noiseAmp;
				noiseVol2[c/4] *= noiseAmp;
				// add -90db noise to bootstrap filter self oscillation
				noiseVol1[c/4] = fmax(noiseVol1[c/4], 3.e-5f);
				noiseVol2[c/4] = fmax(noiseVol2[c/4], 3.e-5f);

				float_4 extAmp = clamp(0.1f * modMatrixOutputs[OSC_EXT_VOL_PARAM - ENV1_A_PARAM][c/4], 0.f, 1.f);
				float_4 extMix = clamp(0.2f * modMatrixOutputs[OSC_EXT_VOL_PARAM + nMixChannels - ENV1_A_PARAM][c/4], -1.f, 1.f);
				extVol1[c/4] = 0.5f - 0.5f * extMix;
				extVol2[c/4] = 1.f - extVol1[c/4];
				extVol1[c/4] *= extAmp;
				extVol2[c/4] *= extAmp;

				// set modulated parameters
				env1[c/4].setAttackTime(0.1f * modMatrixOutputs[ENV1_A_PARAM - ENV1_A_PARAM][c/4]);
				env1[c/4].setDecayTime(0.1f * modMatrixOutputs[ENV1_D_PARAM - ENV1_A_PARAM][c/4]);
				env1[c/4].setSustainLevel(0.1f * modMatrixOutputs[ENV1_S_PARAM - ENV1_A_PARAM][c/4]);
				env1[c/4].setReleaseTime(0.1f * modMatrixOutputs[ENV1_R_PARAM - ENV1_A_PARAM][c/4]);

				env2[c/4].setAttackTime(0.1f * modMatrixOutputs[ENV2_A_PARAM - ENV1_A_PARAM][c/4]);
				env2[c/4].setDecayTime(0.1f * modMatrixOutputs[ENV2_D_PARAM - ENV1_A_PARAM][c/4]);
				env2[c/4].setSustainLevel(0.1f * modMatrixOutputs[ENV2_S_PARAM - ENV1_A_PARAM][c/4]);
				env2[c/4].setReleaseTime(0.1f * modMatrixOutputs[ENV2_R_PARAM - ENV1_A_PARAM][c/4]);

				lfo1[c/4].setRand(noise1);
				lfo1[c/4].setFrequencyVOct(modMatrixOutputs[LFO1_FREQ_PARAM - ENV1_A_PARAM][c/4]);
				lfo1[c/4].setAmp(modMatrixOutputs[LFO1_AMOUNT_PARAM - ENV1_A_PARAM][c/4]);

				lfo2[c/4].setRand(noise2);
				lfo2[c/4].setFrequencyVOct(modMatrixOutputs[LFO2_FREQ_PARAM - ENV1_A_PARAM][c/4]);
				lfo2[c/4].setAmp(modMatrixOutputs[LFO2_AMOUNT_PARAM - ENV1_A_PARAM][c/4]);

				outputs[INDIVIDUAL_MOD_1_OUTPUT].setVoltageSimd(modMatrixOutputs[INDIVIDUAL_MOD_OUT_1_PARAM - ENV1_A_PARAM][c/4], c);
				outputs[INDIVIDUAL_MOD_2_OUTPUT].setVoltageSimd(modMatrixOutputs[INDIVIDUAL_MOD_OUT_2_PARAM - ENV1_A_PARAM][c/4], c);
				outputs[INDIVIDUAL_MOD_3_OUTPUT].setVoltageSimd(modMatrixOutputs[INDIVIDUAL_MOD_OUT_3_PARAM - ENV1_A_PARAM][c/4], c);
				outputs[INDIVIDUAL_MOD_4_OUTPUT].setVoltageSimd(modMatrixOutputs[INDIVIDUAL_MOD_OUT_4_PARAM - ENV1_A_PARAM][c/4], c);
				outputs[INDIVIDUAL_MOD_5_OUTPUT].setVoltageSimd(modMatrixOutputs[INDIVIDUAL_MOD_OUT_5_PARAM - ENV1_A_PARAM][c/4], c);

				// glide only on V/Oct
				float_4 vOctInput = inputs[VOCT_INPUT].getPolyVoltageSimd<float_4>(c);
				float_4 gateInput = inputs[GATE_INPUT].getPolyVoltageSimd<float_4>(c);
				float_4 osc1FreqVOct = getParam(OSC1_TUNE_OCT_PARAM).getValue() +
						modMatrixOutputs[OSC1_TUNE_SEMI_PARAM - ENV1_A_PARAM][c/4] / 5.f -
						modMatrix[OSC1_TUNE_SEMI_PARAM - ENV1_A_PARAM][VOCT_ASSIGN_PARAM + 1] * vOctInput / 5.f +
						modMatrixOutputs[OSC1_TUNE_FINE_PARAM - ENV1_A_PARAM][c/4] / 5.f / 12.f;
				glide1[c/4].setCutoffFreq(getGlideFreq(modMatrixOutputs[OSC1_TUNE_GLIDE_PARAM - ENV1_A_PARAM][c/4], args.sampleRate));
				if (getParam(OSC_TUNE_GLIDE_FINGERED_PARAM).getValue())
				{
					glide1[c/4].setState(vOctInput, gateInput > lastGate[c/4] + 0.5f);
				}
				oscillators[c/4].setOsc1FreqVOct(osc1FreqVOct +
						modMatrix[OSC1_TUNE_SEMI_PARAM - ENV1_A_PARAM][VOCT_ASSIGN_PARAM + 1] / 5.f * glide1[c/4].processLowpass(vOctInput));

				oscillators[c/4].setOsc1Shape(0.2f * modMatrixOutputs[OSC1_SHAPE_PARAM - ENV1_A_PARAM][c/4] - 1.f);
				oscillators[c/4].setOsc1PW(0.2f * modMatrixOutputs[OSC1_PW_PARAM - ENV1_A_PARAM][c/4] - 1.f);
				oscillators[c/4].setOsc1Vol(0.1f * modMatrixOutputs[OSC1_VOL_PARAM - ENV1_A_PARAM][c/4]);
				oscillators[c/4].setOsc1Pan(0.2f * modMatrixOutputs[OSC1_VOL_PARAM + nMixChannels - ENV1_A_PARAM][c/4]);
				oscillators[c/4].setOsc1Subvol(0.1f * modMatrixOutputs[OSC1_SUB_VOL_PARAM - ENV1_A_PARAM][c/4]);
				oscillators[c/4].setOsc1SubPan(0.2f * modMatrixOutputs[OSC1_SUB_VOL_PARAM + nMixChannels - ENV1_A_PARAM][c/4]);

				float_4 osc2FreqVOct = getParam(OSC2_TUNE_OCT_PARAM).getValue() +
						modMatrixOutputs[OSC2_TUNE_SEMI_PARAM - ENV1_A_PARAM][c/4] / 5.f -
						modMatrix[OSC2_TUNE_SEMI_PARAM - ENV1_A_PARAM][VOCT_ASSIGN_PARAM + 1] * vOctInput / 5.f +
						modMatrixOutputs[OSC2_TUNE_FINE_PARAM - ENV1_A_PARAM][c/4] / 5.f / 12.f;
				glide2[c/4].setCutoffFreq(getGlideFreq(modMatrixOutputs[OSC1_TUNE_GLIDE_PARAM - ENV1_A_PARAM][c/4] + modMatrixOutputs[OSC2_TUNE_GLIDE_PARAM - ENV1_A_PARAM][c/4], args.sampleRate));
				if (getParam(OSC_TUNE_GLIDE_FINGERED_PARAM).getValue())
				{
					glide2[c/4].setState(vOctInput, gateInput > lastGate[c/4] + 0.5f);
				}
				oscillators[c/4].setOsc2FreqVOct(osc2FreqVOct +
						modMatrix[OSC2_TUNE_SEMI_PARAM - ENV1_A_PARAM][VOCT_ASSIGN_PARAM + 1] / 5.f * glide2[c/4].processLowpass(vOctInput));

				oscillators[c/4].setOsc2Shape(0.2f * modMatrixOutputs[OSC2_SHAPE_PARAM - ENV1_A_PARAM][c/4] - 1.f);
				oscillators[c/4].setOsc2PW(0.2f * modMatrixOutputs[OSC2_PW_PARAM - ENV1_A_PARAM][c/4] - 1.f);
				oscillators[c/4].setOsc2Vol(0.1f * modMatrixOutputs[OSC2_VOL_PARAM - ENV1_A_PARAM][c/4]);
				oscillators[c/4].setOsc2Pan(0.2f * modMatrixOutputs[OSC2_VOL_PARAM + nMixChannels - ENV1_A_PARAM][c/4]);

				oscillators[c/4].setFmAmount(0.1f * modMatrixOutputs[OSC_FM_AMOUNT_PARAM - ENV1_A_PARAM][c/4]);
				oscillators[c/4].setRingmodVol(0.1f * modMatrixOutputs[OSC_RM_VOL_PARAM - ENV1_A_PARAM][c/4]);
				oscillators[c/4].setRingmodPan(0.2f * modMatrixOutputs[OSC_RM_VOL_PARAM + nMixChannels - ENV1_A_PARAM][c/4]);


				// cutoff mode
				float_4 filterFrequency;
				switch ((int)getParam(FILTER2_CUTOFF_MODE_PARAM).getValue())
				{
				case 0: // individual
					filterFrequency = simd::exp(filterLogBase * 0.1f * modMatrixOutputs[FILTER1_CUTOFF_PARAM - ENV1_A_PARAM][c/4]) * filterMinFreq;
					filterFrequency = simd::clamp(filterFrequency, filterMinFreq, simd::fmin(2.f * filterMaxFreq, args.sampleRate * currentOversamplingRate * 0.18f));
					filter1[c/4].setCutoffFrequencyAndResonance(
							filterFrequency,
							0.5f * modMatrixOutputs[FILTER1_RESONANCE_PARAM - ENV1_A_PARAM][c/4]);

					filterFrequency = simd::exp(filterLogBase * 0.1f * modMatrixOutputs[FILTER2_CUTOFF_PARAM - ENV1_A_PARAM][c/4]) * filterMinFreq;
					filterFrequency = simd::clamp(filterFrequency, filterMinFreq, simd::fmin(2.f * filterMaxFreq, args.sampleRate * currentOversamplingRate * 0.18f));
					filter2[c/4].setCutoffFrequencyAndResonance(
							filterFrequency,
							0.5f * modMatrixOutputs[FILTER2_RESONANCE_PARAM - ENV1_A_PARAM][c/4]);
					break;
				case 1: // offset
					filterFrequency = simd::exp(filterLogBase * 0.1f * modMatrixOutputs[FILTER1_CUTOFF_PARAM - ENV1_A_PARAM][c/4]) * filterMinFreq;
					filterFrequency = simd::clamp(filterFrequency, filterMinFreq, simd::fmin(2.f * filterMaxFreq, args.sampleRate * currentOversamplingRate * 0.18f));
					filter1[c/4].setCutoffFrequencyAndResonance(
							filterFrequency,
							0.5f * modMatrixOutputs[FILTER1_RESONANCE_PARAM - ENV1_A_PARAM][c/4]);

					filterFrequency = modMatrixOutputs[FILTER1_CUTOFF_PARAM - ENV1_A_PARAM][c/4] + (modMatrixOutputs[FILTER2_CUTOFF_PARAM - ENV1_A_PARAM][c/4]) - 5.f;
					filterFrequency = simd::exp(filterLogBase * 0.1f * filterFrequency) * filterMinFreq;
					filterFrequency = simd::clamp(filterFrequency, filterMinFreq, simd::fmin(2.f * filterMaxFreq, args.sampleRate * currentOversamplingRate * 0.18f));
					filter2[c/4].setCutoffFrequencyAndResonance(
							filterFrequency,
							0.5f * modMatrixOutputs[FILTER2_RESONANCE_PARAM - ENV1_A_PARAM][c/4]);
					break;
				case 2: // space
					filterFrequency = modMatrixOutputs[FILTER1_CUTOFF_PARAM - ENV1_A_PARAM][c/4] - (modMatrixOutputs[FILTER2_CUTOFF_PARAM - ENV1_A_PARAM][c/4] - 5.f);
					filterFrequency = simd::exp(filterLogBase * 0.1f * filterFrequency) * filterMinFreq;
					filterFrequency = simd::clamp(filterFrequency, filterMinFreq, simd::fmin(2.f * filterMaxFreq, args.sampleRate * currentOversamplingRate * 0.18f));
					filter1[c/4].setCutoffFrequencyAndResonance(
							filterFrequency,
							0.5f * modMatrixOutputs[FILTER1_RESONANCE_PARAM - ENV1_A_PARAM][c/4]);

					filterFrequency = modMatrixOutputs[FILTER1_CUTOFF_PARAM - ENV1_A_PARAM][c/4] + (modMatrixOutputs[FILTER2_CUTOFF_PARAM - ENV1_A_PARAM][c/4] - 5.f);
					filterFrequency = simd::exp(filterLogBase * 0.1f * filterFrequency) * filterMinFreq;
					filterFrequency = simd::clamp(filterFrequency, filterMinFreq, simd::fmin(2.f * filterMaxFreq, args.sampleRate * currentOversamplingRate * 0.18f));
					filter2[c/4].setCutoffFrequencyAndResonance(
							filterFrequency,
							0.5f * modMatrixOutputs[FILTER2_RESONANCE_PARAM - ENV1_A_PARAM][c/4]);
				}

				lastGate[c/4] = gateInput;
				lastTrigger[c/4] = triggerInput;
			}
		}

		//
		// process audio
		//

		float_4* bufferLR = decimator.getInputArray(currentOversamplingRate);
		std::memset(bufferLR, 0, currentOversamplingRate * sizeof(float_4));

		for (int c = 0; c < channels; c += 4)
		{
			float_4 buffer1[currentOversamplingRate];
			float_4 buffer2[currentOversamplingRate];

			// oscillators
			oscillators[c/4].processBandlimited(buffer1, buffer2);

			// external input/loopback & noise
			float_4 noise = random::normal();
			if (inputs[EXT_INPUT].isConnected())
			{
				float_4 extIn = inputs[EXT_INPUT].getPolyVoltageSimd<float_4>(c);
				for (size_t iSample = 0; iSample < currentOversamplingRate; iSample++)
				{
					// linear interpolation upsampling
					buffer1[iSample] += extVol1[c/4] * crossfade(lastExtIn[c/4], extIn, (iSample + 1.f)/currentOversamplingRate);
					buffer2[iSample] += extVol2[c/4] * crossfade(lastExtIn[c/4], extIn, (iSample + 1.f)/currentOversamplingRate);

					buffer1[iSample] *= 2.f;
					buffer2[iSample] *= 2.f;

					buffer1[iSample] += noiseVol1[c/4] * noise;
					buffer2[iSample] += noiseVol2[c/4] * noise;
				}
				lastExtIn[c/4] = extIn;
			}
			else
			{
				for (size_t iSample = 0; iSample < currentOversamplingRate; iSample++)
				{
					buffer1[iSample] += extVol1[c/4] * (delayBuffer1[c/4][iSample] + delayBuffer2[c/4][iSample]);
					buffer2[iSample] += extVol2[c/4] * (delayBuffer1[c/4][iSample] + delayBuffer2[c/4][iSample]);

					buffer1[iSample] += noiseVol1[c/4] * noise;
					buffer2[iSample] += noiseVol2[c/4] * noise;
				}
			}

			// process filter 1
			dcBlocker1[c/4].processHighpassBlock(buffer1, currentOversamplingRate);
			aliasFilter1[c/4].processLowpassBlock(buffer1, currentOversamplingRate);
			filter1[c/4].processBlock(buffer1, args.sampleTime / currentOversamplingRate, currentOversamplingRate);
			saturator1[c/4].processBlockBandlimited(buffer1, currentOversamplingRate);

			// serial routing
			float_4 serPar = clamp(0.2f * modMatrixOutputs[FILTER_SERIAL_PARALLEL_PARAM - ENV1_A_PARAM][c/4] - 1.f, -1.f, 1.f);
			float_4 serial = 0.5f - 0.5f * serPar; // [1..0]
			for (size_t iSample = 0; iSample < currentOversamplingRate; iSample++)
			{
				delayBuffer2[c/4][iSample] += serial * buffer1[iSample];
			}

			// process filter 2
			dcBlocker2[c/4].processHighpassBlock(delayBuffer2[c/4], currentOversamplingRate);
			aliasFilter2[c/4].processLowpassBlock(delayBuffer2[c/4], currentOversamplingRate);
			filter2[c/4].processBlock(delayBuffer2[c/4], args.sampleTime / currentOversamplingRate, currentOversamplingRate);
			saturator2[c/4].processBlockBandlimited(delayBuffer2[c/4], currentOversamplingRate);

			// parallel routing
			float_4 parallel = 0.5f + 0.5f * serPar; // [0..1]
			for (size_t iSample = 0; iSample < currentOversamplingRate; iSample++)
			{
				buffer1[iSample] *= parallel;
			}

			// pan, amp
			float_4 pan1 = clamp(0.2f * modMatrixOutputs[FILTER1_PAN_PARAM - ENV1_A_PARAM][c/4], -1.f, 1.f);
			float_4 pan2 = clamp(0.2f * modMatrixOutputs[FILTER2_PAN_PARAM - ENV1_A_PARAM][c/4], -1.f, 1.f);
			// constant power pan law
			float_4 vol1L = panGetVolL<float_4>(pan1);
			float_4 vol1R = panGetVolR<float_4>(pan1);
			float_4 vol2L = panGetVolL<float_4>(pan2);
			float_4 vol2R = panGetVolR<float_4>(pan2);
			for (size_t iSample = 0; iSample < currentOversamplingRate; iSample++)
			{
				// amp
				delayBuffer1[c/4][iSample] *= 0.1f * modMatrixOutputs[AMP_VOL_PARAM - ENV1_A_PARAM][c/4];
				delayBuffer2[c/4][iSample] *= 0.1f * modMatrixOutputs[AMP_VOL_PARAM - ENV1_A_PARAM][c/4];

				// sum to stereo
				for (int j = 0; j < std::min(channels - c, 4); j++)
				{
					// L
					bufferLR[iSample][0] += vol1L[j] * delayBuffer1[c/4][iSample][j] + vol2L[j] * delayBuffer2[c/4][iSample][j];
					// R
					bufferLR[iSample][1] += vol1R[j] * delayBuffer1[c/4][iSample][j] + vol2R[j] * delayBuffer2[c/4][iSample][j];
				}
			}

			// delay buffers to bring filter 1 and 2 in phase also with serial routing
			std::memcpy(&delayBuffer1[c/4], &buffer1, sizeof(buffer1));
			std::memcpy(&delayBuffer2[c/4], &buffer2, sizeof(buffer2));
		}

		// downsampling
		float_4 outLR = decimator.process(currentOversamplingRate);

		outputs[OUT_L_OUTPUT].setVoltage(outLR[0]);
		outputs[OUT_R_OUTPUT].setVoltage(outLR[1]);
	}

	json_t* dataToJson() override {
		json_t* rootJ = json_object();
		json_t* entryJ;

		json_t* modMatrixJ = json_array();
		for (size_t iDest = 0; iDest < nDestinations; iDest++)
		{
			for (size_t iSource = 0; iSource < nSources; iSource++)
			{
				entryJ = json_real(modMatrix[iDest][iSource]);
				json_array_insert_new(modMatrixJ, iDest * nSources + iSource, entryJ);
			}
		}
		json_object_set_new(rootJ, "modMatrix", modMatrixJ);

		json_t* mixLevelsJ = json_array();
		for (size_t i = 0; i < nMixChannels; i++)
		{
			json_t* entryJ = json_real(mixLevels[i]);
			json_array_insert_new(mixLevelsJ, i, entryJ);
		}
		json_object_set_new(rootJ, "mixLevels", mixLevelsJ);

		json_t* mixFilterBalancesJ = json_array();
		for (size_t i = 0; i < nMixChannels; i++)
		{
			json_t* entryJ = json_real(mixFilterBalances[i]);
			json_array_insert_new(mixFilterBalancesJ, i, entryJ);
		}
		json_object_set_new(rootJ, "mixFilterBalances", mixFilterBalancesJ);

		std::vector<std::string> labels = FilterBlock::getModeLabels();
		json_object_set_new(rootJ, "filter1Mode", json_string(labels[params[FILTER1_MODE_PARAM].getValue()].c_str()));
		json_object_set_new(rootJ, "filter2Mode", json_string(labels[params[FILTER2_MODE_PARAM].getValue()].c_str()));

		json_object_set_new(rootJ, "oversamplingRate", json_integer(oversamplingRate));
		json_object_set_new(rootJ, "modSampleRateReduction", json_integer(modDivider.getDivision()));
		json_object_set_new(rootJ, "uiSampleRateReduction", json_integer(uiDivider.getDivision()));
		json_object_set_new(rootJ, "filterMethod", json_integer((int)filterMethod));
		json_object_set_new(rootJ, "lockQualitySettings", json_boolean(lockQualitySettings));

		json_object_set_new(rootJ, "filterIntegratorType", json_integer((int)filterIntegratorType));

		return rootJ;
	}

	void dataFromJson(json_t* rootJ) override {
		jsonLoaded = true;

		json_t* entryJ;

		json_t* modMatrixJ = json_object_get(rootJ, "modMatrix");
		if (modMatrixJ)
		{
			for (size_t iDest = 0; iDest < nDestinations; iDest++)
			{
				for (size_t iSource = 0; iSource < nSources; iSource++)
				{
					entryJ = json_array_get(modMatrixJ, iDest * nSources + iSource);
					if (entryJ)
					{
						modMatrix[iDest][iSource] = json_real_value(entryJ);
					}
				}
			}
		}

		json_t* mixLevelsJ = json_object_get(rootJ, "mixLevels");
		if (mixLevelsJ)
		{
			for (size_t i = 0; i < nMixChannels; i++)
			{
				json_t* entryJ = json_array_get(mixLevelsJ, i);
				if (entryJ)
				{
					mixLevels[i] = json_real_value(entryJ);
				}
			}
		}

		json_t* mixFilterBalancesJ = json_object_get(rootJ, "mixFilterBalances");
		if (mixFilterBalancesJ)
		{
			for (size_t i = 0; i < nMixChannels; i++)
			{
				json_t* entryJ = json_array_get(mixFilterBalancesJ, i);
				if (entryJ)
				{
					mixFilterBalances[i] = json_real_value(entryJ);
				}
			}
		}

		// read filter mode from label string, allows adding filter modes in future releases without breaking patches
		std::vector<std::string> labels = FilterBlock::getModeLabels();
		json_t* filter1ModeJ = json_object_get(rootJ, "filter1Mode");
		if (filter1ModeJ)
		{
			auto it = std::find(labels.begin(), labels.end(), json_string_value(filter1ModeJ));
			if (it != labels.end())
			{
				params[FILTER1_MODE_PARAM].setValue(std::distance(labels.begin(), it));
			}
		}
		json_t* filter2ModeJ = json_object_get(rootJ, "filter2Mode");
		if (filter2ModeJ)
		{
			auto it = std::find(labels.begin(), labels.end(), json_string_value(filter2ModeJ));
			if (it != labels.end())
			{
				params[FILTER2_MODE_PARAM].setValue(std::distance(labels.begin(), it));
			}
		}

		if (lockQualitySettings != 1)
		{
			json_t* oversamplingRateJ = json_object_get(rootJ, "oversamplingRate");
			if (oversamplingRateJ)
			{
				setOversamplingRate(json_integer_value(oversamplingRateJ));
			}

			json_t* modSampleRateReductionJ = json_object_get(rootJ, "modSampleRateReduction");
			if (modSampleRateReductionJ)
			{
				setModSampleRateReduction(json_integer_value(modSampleRateReductionJ));
			}

			json_t* uiSampleRateReductionJ = json_object_get(rootJ, "uiSampleRateReduction");
			if (uiSampleRateReductionJ)
			{
				uiDivider.setDivision(json_integer_value(uiSampleRateReductionJ));
			}

			json_t* filterMethodJ = json_object_get(rootJ, "filterMethod");
			if (filterMethodJ)
			{
				setFilterMethod((Method)json_integer_value(filterMethodJ));
			}
		}

		if (lockQualitySettings == -1)
		{
			// load lockQualitySettings only on initial load
			json_t* lockQualitySettingsJ = json_object_get(rootJ, "lockQualitySettings");
			if (lockQualitySettingsJ)
			{
				lockQualitySettings = json_boolean_value(lockQualitySettingsJ);
			}
		}

		json_t* filterIntegratorTypeJ = json_object_get(rootJ, "filterIntegratorType");
		if (filterIntegratorTypeJ)
		{
			setFilterIntegratorType((IntegratorType)json_integer_value(filterIntegratorTypeJ));
		}

		// diverge
		configureDrift();

		configureUi();

		// reset LFO phases, filters
		globalLfo.resetPhases();
		for (int c = 0; c < 16; c += 4) {
			lfo1[c/4].resetPhases();
			lfo2[c/4].resetPhases();

			filter1[c/4].reset();
			filter2[c/4].reset();
		}
	}

private:
	float_4 getGlideFreq(float_4 glideValue, float sampleRate)
	{
		const float glideScale = 1.5f;
		float_4 glideFreq = clamp(10.f - glideValue, 0.f, 10.f); // 10..0
		glideFreq = dsp::exp2_taylor5(glideScale * glideFreq) / dsp::exp2_taylor5(glideScale * 10.f); // 1..0
		glideFreq *= modDivider.getDivision() / sampleRate * 1000.f;
		return glideFreq;
	}
};


struct SynthWidget : ModuleWidget {
	SynthWidget(Synth* module) {
		setModule(module);
		if (module)
		{
			module->widget = this;
			module->loadTemplate();
		}

		setPanel(createPanel(asset::plugin(pluginInstance, "res/Synth.svg"), asset::plugin(pluginInstance, "res/Synth-dark.svg")));

	    addParam(createLightParamCentered<AssignButton>(mm2px(Vec(27.0, 8.557)), module, Synth::VOCT_ASSIGN_PARAM, Synth::VOCT_ASSIGN_LIGHT));
	    addParam(createLightParamCentered<AssignButton>(mm2px(Vec(27.0, 18.182)), module, Synth::GATE_ASSIGN_PARAM, Synth::GATE_ASSIGN_LIGHT));
	    addParam(createLightParamCentered<AssignButton>(mm2px(Vec(27.0, 27.807)), module, Synth::VELOCITY_ASSIGN_PARAM, Synth::VELOCITY_ASSIGN_LIGHT));
	    addParam(createLightParamCentered<AssignButton>(mm2px(Vec(27.0, 37.433)), module, Synth::AFTERTOUCH_ASSIGN_PARAM, Synth::AFTERTOUCH_ASSIGN_LIGHT));
	    addParam(createLightParamCentered<AssignButton>(mm2px(Vec(27.0, 47.058)), module, Synth::PITCH_WHEEL_ASSIGN_PARAM, Synth::PITCH_WHEEL_ASSIGN_LIGHT));
	    addParam(createLightParamCentered<AssignButton>(mm2px(Vec(27.0, 56.683)), module, Synth::MOD_WHEEL_ASSIGN_PARAM, Synth::MOD_WHEEL_ASSIGN_LIGHT));
	    addParam(createLightParamCentered<AssignButton>(mm2px(Vec(27.0, 66.309)), module, Synth::EXPRESSION_ASSIGN_PARAM, Synth::EXPRESSION_ASSIGN_LIGHT));
	    addParam(createLightParamCentered<AssignButton>(mm2px(Vec(27.0, 75.934)), module, Synth::INDIVIDUAL_MOD_1_ASSIGN_PARAM, Synth::INDIVIDUAL_MOD_1_ASSIGN_LIGHT));
	    addParam(createLightParamCentered<AssignButton>(mm2px(Vec(27.0, 85.56)), module, Synth::INDIVIDUAL_MOD_2_ASSIGN_PARAM, Synth::INDIVIDUAL_MOD_2_ASSIGN_LIGHT));
	    addParam(createLightParamCentered<AssignButton>(mm2px(Vec(27.0, 95.185)), module, Synth::VOICE_NR_ASSIGN_PARAM, Synth::VOICE_NR_ASSIGN_LIGHT));
	    addParam(createLightParamCentered<AssignButton>(mm2px(Vec(27.0, 104.81)), module, Synth::RANDOM_ASSIGN_PARAM, Synth::RANDOM_ASSIGN_LIGHT));

	    addParam(createLightParamCentered<AssignButton>(mm2px(Vec(127.5, 12.088)), module, Synth::ENV1_ASSIGN_PARAM, Synth::ENV1_ASSIGN_LIGHT));
	    addParam(createLightParamCentered<AssignButton>(mm2px(Vec(127.5, 37.188)), module, Synth::ENV2_ASSIGN_PARAM, Synth::ENV2_ASSIGN_LIGHT));
	    addParam(createLightParamCentered<AssignButton>(mm2px(Vec(210.0, 6.101)), module, Synth::LFO1_UNIPOLAR_ASSIGN_PARAM, Synth::LFO1_UNIPOLAR_ASSIGN_LIGHT));
	    addParam(createLightParamCentered<AssignButton>(mm2px(Vec(210.0, 17.662)), module, Synth::LFO1_BIPOLAR_ASSIGN_PARAM, Synth::LFO1_BIPOLAR_ASSIGN_LIGHT));
	    addParam(createLightParamCentered<AssignButton>(mm2px(Vec(210.0, 31.201)), module, Synth::LFO2_UNIPOLAR_ASSIGN_PARAM, Synth::LFO2_UNIPOLAR_ASSIGN_LIGHT));
	    addParam(createLightParamCentered<AssignButton>(mm2px(Vec(210.0, 42.762)), module, Synth::LFO2_BIPOLAR_ASSIGN_PARAM, Synth::LFO2_BIPOLAR_ASSIGN_LIGHT));
	    addParam(createLightParamCentered<AssignButton>(mm2px(Vec(263.0, 12.088)), module, Synth::GLOBAL_LFO_ASSIGN_PARAM, Synth::GLOBAL_LFO_ASSIGN_LIGHT));
	    addParam(createLightParamCentered<AssignButton>(mm2px(Vec(257.179, 31.201)), module, Synth::DIVERGE_1_ASSIGN_PARAM, Synth::DIVERGE_1_ASSIGN_LIGHT));
	    addParam(createLightParamCentered<AssignButton>(mm2px(Vec(265.117, 31.201)), module, Synth::DIVERGE_2_ASSIGN_PARAM, Synth::DIVERGE_2_ASSIGN_LIGHT));
	    addParam(createLightParamCentered<AssignButton>(mm2px(Vec(257.179, 42.762)), module, Synth::DRIFT_1_ASSIGN_PARAM, Synth::DRIFT_1_ASSIGN_LIGHT));
	    addParam(createLightParamCentered<AssignButton>(mm2px(Vec(265.117, 42.762)), module, Synth::DRIFT_2_ASSIGN_PARAM, Synth::DRIFT_2_ASSIGN_LIGHT));

	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(47.029, 12.088)), module, Synth::ENV1_A_PARAM));
	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(64.029, 12.088)), module, Synth::ENV1_D_PARAM));
	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(81.029, 12.088)), module, Synth::ENV1_S_PARAM));
	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(98.029, 12.088)), module, Synth::ENV1_R_PARAM));
	    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(115.029, 12.088)), module, Synth::ENV1_VEL_PARAM));
	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(47.029, 37.188)), module, Synth::ENV2_A_PARAM));
	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(64.029, 37.188)), module, Synth::ENV2_D_PARAM));
	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(81.029, 37.188)), module, Synth::ENV2_S_PARAM));
	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(98.029, 37.188)), module, Synth::ENV2_R_PARAM));
	    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(115.029, 37.188)), module, Synth::ENV2_VEL_PARAM));
	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(149.0, 12.088)), module, Synth::LFO1_FREQ_PARAM));
	    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(166.0, 12.088)), module, Synth::LFO1_SHAPE_PARAM));
	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(183.0, 12.088)), module, Synth::LFO1_AMOUNT_PARAM));
	    addParam(createParamCentered<NKK>(mm2px(Vec(198.0, 12.088)), module, Synth::LFO1_MODE_PARAM));
	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(149.0, 37.188)), module, Synth::LFO2_FREQ_PARAM));
	    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(166.0, 37.188)), module, Synth::LFO2_SHAPE_PARAM));
	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(183.0, 37.188)), module, Synth::LFO2_AMOUNT_PARAM));
	    addParam(createParamCentered<NKK>(mm2px(Vec(198.0, 37.188)), module, Synth::LFO2_MODE_PARAM));
	    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(231.0, 12.088)), module, Synth::GLOBAL_LFO_FREQ_PARAM));
	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(248.0, 12.088)), module, Synth::GLOBAL_LFO_AMT_PARAM));
	    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(229.413, 37.188)), module, Synth::DRIFT_RATE_PARAM));
	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(244.825, 37.188)), module, Synth::DRIFT_AMOUNT_PARAM));
	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(284.578, 12.088)), module, Synth::INDIVIDUAL_MOD_OUT_1_PARAM));
	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(284.578, 37.188)), module, Synth::INDIVIDUAL_MOD_OUT_2_PARAM));
	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(284.578, 62.288)), module, Synth::INDIVIDUAL_MOD_OUT_3_PARAM));
	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(284.577, 87.388)), module, Synth::INDIVIDUAL_MOD_OUT_4_PARAM));
	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(284.577, 112.557)), module, Synth::INDIVIDUAL_MOD_OUT_5_PARAM));
	    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(44.864, 62.288)), module, Synth::OSC1_TUNE_OCT_PARAM));
	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(60.276, 62.288)), module, Synth::OSC1_TUNE_SEMI_PARAM));
	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(77.276, 62.288)), module, Synth::OSC1_TUNE_FINE_PARAM));
	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(98.335, 62.288)), module, Synth::OSC1_SHAPE_PARAM));
	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(119.039, 62.288)), module, Synth::OSC1_PW_PARAM));
	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(140.626, 62.288)), module, Synth::OSC1_VOL_PARAM));
	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(157.626, 62.288)), module, Synth::OSC1_SUB_VOL_PARAM));
	    addParam(createLightParamCentered<VCVLightLatch<MediumSimpleLight<WhiteLight>>>(mm2px(Vec(44.864, 87.388)), module, Synth::OSC_TUNE_GLIDE_FINGERED_PARAM, Synth::OSC_TUNE_GLIDE_FINGERED_LIGHT));
	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(60.276, 87.388)), module, Synth::OSC1_TUNE_GLIDE_PARAM));
	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(77.276, 87.388)), module, Synth::OSC2_TUNE_GLIDE_PARAM));
	    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(44.864, 112.488)), module, Synth::OSC2_TUNE_OCT_PARAM));
	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(60.276, 112.488)), module, Synth::OSC2_TUNE_SEMI_PARAM));
	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(77.276, 112.488)), module, Synth::OSC2_TUNE_FINE_PARAM));
	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(98.335, 112.488)), module, Synth::OSC2_SHAPE_PARAM));
	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(119.039, 112.488)), module, Synth::OSC2_PW_PARAM));
	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(140.626, 112.488)), module, Synth::OSC2_VOL_PARAM));
	    addParam(createLightParamCentered<VCVLightLatch<MediumSimpleLight<WhiteLight>>>(mm2px(Vec(95.441, 87.388)), module, Synth::OSC_SYNC_PARAM, Synth::OSC_SYNC_LIGHT));
	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(109.004, 87.388)), module, Synth::OSC_FM_AMOUNT_PARAM));
	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(140.626, 87.388)), module, Synth::OSC_RM_VOL_PARAM));
	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(157.626, 87.388)), module, Synth::OSC_NOISE_VOL_PARAM));
	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(157.626, 112.488)), module, Synth::OSC_EXT_VOL_PARAM));
	    addParam(createLightParamCentered<VCVLightLatch<MediumSimpleLight<RedLight>>>(mm2px(Vec(126.9, 87.388)), module, Synth::OSC_MIX_ROUTE_PARAM, Synth::OSC_MIX_ROUTE_LIGHT));
	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(194.912, 62.288)), module, Synth::FILTER1_CUTOFF_PARAM));
	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(211.912, 62.288)), module, Synth::FILTER1_RESONANCE_PARAM));
	    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(227.325, 62.288)), module, Synth::FILTER1_MODE_PARAM));
	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(259.737, 62.288)), module, Synth::FILTER1_PAN_PARAM));
	    addParam(createParamCentered<NKK>(mm2px(Vec(180.413, 75.237)), module, Synth::FILTER2_CUTOFF_MODE_PARAM));
	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(194.912, 87.388)), module, Synth::FILTER2_CUTOFF_PARAM));
	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(211.912, 87.388)), module, Synth::FILTER2_RESONANCE_PARAM));
	    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(227.325, 87.388)), module, Synth::FILTER2_MODE_PARAM));
	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(259.737, 87.388)), module, Synth::FILTER2_PAN_PARAM));
	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(242.737, 75.237)), module, Synth::FILTER_SERIAL_PARALLEL_PARAM));
	    addParam(createParamCentered<RoundBlackKnobWithArc>(mm2px(Vec(183.587, 112.488)), module, Synth::AMP_VOL_PARAM));

	    addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(7.0, 8.557)), module, Synth::VOCT_INPUT));
	    addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(7.0, 18.182)), module, Synth::GATE_INPUT));
	    addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(7.0, 27.807)), module, Synth::VELOCITY_INPUT));
	    addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(7.0, 37.433)), module, Synth::AFTERTOUCH_INPUT));
	    addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(7.0, 47.058)), module, Synth::PITCH_WHEEL_INPUT));
	    addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(7.0, 56.683)), module, Synth::MOD_WHEEL_INPUT));
	    addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(7.0, 66.309)), module, Synth::EXPRESSION_INPUT));
	    addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(7.0, 75.934)), module, Synth::INDIVIDUAL_MOD_1_INPUT));
	    addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(7.0, 85.56)), module, Synth::INDIVIDUAL_MOD_2_INPUT));
	    addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(8.983, 116.799)), module, Synth::RETRIGGER_INPUT));
	    addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(24.823, 116.799)), module, Synth::EXT_INPUT));

		addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(298.976, 12.088)), module, Synth::INDIVIDUAL_MOD_1_OUTPUT));
		addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(298.976, 37.188)), module, Synth::INDIVIDUAL_MOD_2_OUTPUT));
		addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(298.976, 62.288)), module, Synth::INDIVIDUAL_MOD_3_OUTPUT));
		addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(298.975, 87.388)), module, Synth::INDIVIDUAL_MOD_4_OUTPUT));
		addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(298.975, 112.557)), module, Synth::INDIVIDUAL_MOD_5_OUTPUT));

		addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(246.638, 112.557)), module, Synth::OUT_L_OUTPUT));
		addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(261.791, 112.557)), module, Synth::OUT_R_OUTPUT));
	}

	void appendContextMenu(Menu* menu) override {
		Synth* module = getModule<Synth>();

		menu->addChild(new MenuSeparator);

		menu->addChild(createIndexSubmenuItem("Audio oversampling rate", {"1x (low CPU)", "2x", "4x", "8x", "16x (best quality)"},
			[=]() {
				return log2((int)module->oversamplingRate);
			},
			[=](int mode) {
				module->setOversamplingRate((size_t)std::pow(2, mode));
			}
		));

		menu->addChild(createIndexSubmenuItem("Modulation sample rate reduction", {"1x (best quality)", "2x", "4x", "8x", "16x (low CPU)"},
			[=]() {
				return log2((int)module->modDivider.getDivision());
			},
			[=](int mode) {
				module->setModSampleRateReduction(std::pow(2, mode));
			}
		));

		menu->addChild(createIndexSubmenuItem("UI sample rate reduction", {"1x (best quality)", "2x", "4x", "8x", "16x", "32x", "64x (low CPU)"},
			[=]() {
				return log2((int)module->uiDivider.getDivision());
			},
			[=](int mode) {
				module->uiDivider.setDivision(std::pow(2, mode));
			}
		));

		menu->addChild(createIndexSubmenuItem("Filter ODE Solver", FilterBlock::getOdeSolverLabels(),
			[=]() {
				return (int)module->filterMethod;
			},
			[=](int mode) {
				module->setFilterMethod((Method)mode);
			}
		));

		menu->addChild(createBoolMenuItem("Lock quality settings", "",
			[=]() {
				return module->lockQualitySettings == 1;
			},
			[=](int mode) {
				module->lockQualitySettings = mode;
			}
		));

		menu->addChild(new MenuSeparator);

		menu->addChild(createIndexSubmenuItem("Filter integrator type", FilterBlock::getIntegratorTypeLabels(),
			[=]() {
				return (int)module->filterIntegratorType;
			},
			[=](int mode) {
				module->setFilterIntegratorType((IntegratorType)mode);
			}
		));

	}
};


Model* modelSynth = createModel<Synth, SynthWidget>("Synth");

}
