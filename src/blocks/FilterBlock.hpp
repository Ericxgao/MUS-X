#include "plugin.hpp"
#include "../dsp/odeFilters.hpp"
#include "../dsp/filters.hpp"

namespace musx {

using namespace rack;
using simd::float_4;

class FilterBlock {
private:
	Filter1Pole<float_4, IntegratorType::Linear> filter1Pole_linear;
	LadderFilter2Pole<float_4, IntegratorType::Linear> ladderFilter2Pole_linear;
	LadderFilter4Pole<float_4, IntegratorType::Linear> ladderFilter4Pole_linear;
	SallenKeyFilterLpBp<float_4, IntegratorType::Linear> sallenKeyFilterLpBp_linear;
	SallenKeyFilterHp<float_4, IntegratorType::Linear> sallenKeyFilterHp_linear;
	DiodeClipper<float_4, IntegratorType::Linear> diodeClipper_linear;
	DiodeClipperAsym<float_4, IntegratorType::Linear> diodeClipperAsym_linear;

	Filter1Pole<float_4, IntegratorType::OTA_tanh> filter1Pole_ota_tanh;
	LadderFilter2Pole<float_4, IntegratorType::OTA_tanh> ladderFilter2Pole_ota_tanh;
	LadderFilter4Pole<float_4, IntegratorType::OTA_tanh> ladderFilter4Pole_ota_tanh;
	SallenKeyFilterLpBp<float_4, IntegratorType::OTA_tanh> sallenKeyFilterLpBp_ota_tanh;
	SallenKeyFilterHp<float_4, IntegratorType::OTA_tanh> sallenKeyFilterHp_ota_tanh;
	DiodeClipper<float_4, IntegratorType::OTA_tanh> diodeClipper_ota_tanh;
	DiodeClipperAsym<float_4, IntegratorType::OTA_tanh> diodeClipperAsym_ota_tanh;

	Filter1Pole<float_4, IntegratorType::OTA_alt> filter1Pole_ota_alt;
	LadderFilter2Pole<float_4, IntegratorType::OTA_alt> ladderFilter2Pole_ota_alt;
	LadderFilter4Pole<float_4, IntegratorType::OTA_alt> ladderFilter4Pole_ota_alt;
	SallenKeyFilterLpBp<float_4, IntegratorType::OTA_alt> sallenKeyFilterLpBp_ota_alt;
	SallenKeyFilterHp<float_4, IntegratorType::OTA_alt> sallenKeyFilterHp_ota_alt;
	DiodeClipper<float_4, IntegratorType::OTA_alt> diodeClipper_ota_alt;
	DiodeClipperAsym<float_4, IntegratorType::OTA_alt> diodeClipperAsym_ota_alt;

	Filter1Pole<float_4, IntegratorType::Transistor_tanh> filter1Pole_transistor_tanh;
	LadderFilter2Pole<float_4, IntegratorType::Transistor_tanh> ladderFilter2Pole_transistor_tanh;
	LadderFilter4Pole<float_4, IntegratorType::Transistor_tanh> ladderFilter4Pole_transistor_tanh;
	SallenKeyFilterLpBp<float_4, IntegratorType::Transistor_tanh> sallenKeyFilterLpBp_transistor_tanh;
	SallenKeyFilterHp<float_4, IntegratorType::Transistor_tanh> sallenKeyFilterHp_transistor_tanh;
	DiodeClipper<float_4, IntegratorType::Transistor_tanh> diodeClipper_transistor_tanh;
	DiodeClipperAsym<float_4, IntegratorType::Transistor_tanh> diodeClipperAsym_transistor_tanh;

	Filter1Pole<float_4, IntegratorType::Transistor_alt> filter1Pole_transistor_alt;
	LadderFilter2Pole<float_4, IntegratorType::Transistor_alt> ladderFilter2Pole_transistor_alt;
	LadderFilter4Pole<float_4, IntegratorType::Transistor_alt> ladderFilter4Pole_transistor_alt;
	SallenKeyFilterLpBp<float_4, IntegratorType::Transistor_alt> sallenKeyFilterLpBp_transistor_alt;
	SallenKeyFilterHp<float_4, IntegratorType::Transistor_alt> sallenKeyFilterHp_transistor_alt;
	DiodeClipper<float_4, IntegratorType::Transistor_alt> diodeClipper_transistor_alt;
	DiodeClipperAsym<float_4, IntegratorType::Transistor_alt> diodeClipperAsym_transistor_alt;

	CombFilter combFilter;

	Method method = Method::RK4;
	IntegratorType integratorType = IntegratorType::Transistor_tanh;
	int mode = 8;
	int switchValue = 0;

public:
	static std::vector<std::string> getModeLabels()
	{
		// do not change existing labels! Filter modes are stored and loaded from JSON with these labels.
		// This allows to add new filter types to the list at random positions (in future releases)
		std::vector<std::string> labels = {
			"1-pole lowpass, 6 dB/Oct (non-resonant)",
			"1-pole highpass, 6 dB/Oct (non-resonant)",
			"2-pole ladder lowpass, 12 dB/Oct",
			"2-pole ladder bandpass, 6 dB/Oct",
			"4-pole ladder lowpass, 6 dB/Oct",
			"4-pole ladder lowpass, 12 dB/Oct",
			"4-pole ladder lowpass, 18 dB/Oct",
			"4-pole ladder lowpass, 24 dB/Oct",
			"2-pole Sallen-Key lowpass, 12 dB/Oct",
			"2-pole Sallen-Key bandpass, 6 dB/Oct",
			"2-pole Sallen-Key highpass, 6 dB/Oct",
			"2-pole Sallen-Key highpass, 12 dB/Oct",
			"Comb Filter (positive feedback)",
			"Comb Filter (negative feedback)",
			"Diode Clipper (Symmetric)",
			"Diode Clipper (Asymmetric)",
			"Bypass",
			"Mute"
		};
		return labels;
	}

	static std::vector<std::string> getOdeSolverLabels()
	{
		std::vector<std::string> labels = {
			"1st order Euler (low CPU)",
			"2nd order Runge-Kutta",
			"4th order Runge-Kutta (best quality)"
		};
		return labels;
	}

	static std::vector<std::string> getIntegratorTypeLabels()
	{
		std::vector<std::string> labels = {
			"Linear",
			"OTA with tanh",
			"OTA with alternate saturator",
			"Transistor with tanh",
			"Transistor with alternate saturator"
		};
		return labels;
	}

	void setMethod(Method m)
	{
		method = m;
		calcOffset();
	}

	void setIntegratorType(IntegratorType t)
	{
		integratorType = t;
		calcOffset();
	}

	void setMode(int m)
	{
		mode = m;
		calcOffset();
	}

	void calcOffset()
	{
		switchValue = mode * 100 + (int)integratorType * 10 + (int)method;
	}

	void reset()
	{
		filter1Pole_linear.reset();
		ladderFilter2Pole_linear.reset();
		ladderFilter4Pole_linear.reset();
		sallenKeyFilterLpBp_linear.reset();
		sallenKeyFilterHp_linear.reset();
		diodeClipper_linear.reset();
		diodeClipperAsym_linear.reset();

		filter1Pole_ota_tanh.reset();
		ladderFilter2Pole_ota_tanh.reset();
		ladderFilter4Pole_ota_tanh.reset();
		sallenKeyFilterLpBp_ota_tanh.reset();
		sallenKeyFilterHp_ota_tanh.reset();
		diodeClipper_ota_tanh.reset();
		diodeClipperAsym_ota_tanh.reset();

		filter1Pole_ota_alt.reset();
		ladderFilter2Pole_ota_alt.reset();
		ladderFilter4Pole_ota_alt.reset();
		sallenKeyFilterLpBp_ota_alt.reset();
		sallenKeyFilterHp_ota_alt.reset();
		diodeClipper_ota_alt.reset();
		diodeClipperAsym_ota_alt.reset();

		filter1Pole_transistor_tanh.reset();
		ladderFilter2Pole_transistor_tanh.reset();
		ladderFilter4Pole_transistor_tanh.reset();
		sallenKeyFilterLpBp_transistor_tanh.reset();
		sallenKeyFilterHp_transistor_tanh.reset();
		diodeClipper_transistor_tanh.reset();
		diodeClipperAsym_transistor_tanh.reset();

		filter1Pole_transistor_alt.reset();
		ladderFilter2Pole_transistor_alt.reset();
		ladderFilter4Pole_transistor_alt.reset();
		sallenKeyFilterLpBp_transistor_alt.reset();
		sallenKeyFilterHp_transistor_alt.reset();
		diodeClipper_transistor_alt.reset();
		diodeClipperAsym_transistor_alt.reset();

		combFilter.reset();
	}

	/**
	 * the following code is generated by scripts/filterCodeGen.py
	 */
	void setCutoffFrequencyAndResonance(float_4 frequency, float_4 resonance)
	{
		switch (switchValue)
		{
		case   0:
		case   1:
		case   2:
		case 100:
		case 101:
		case 102:
			filter1Pole_linear.setCutoffFreq(frequency);
			break;
		case  10:
		case  11:
		case  12:
		case 110:
		case 111:
		case 112:
			filter1Pole_ota_tanh.setCutoffFreq(frequency);
			break;
		case  20:
		case  21:
		case  22:
		case 120:
		case 121:
		case 122:
			filter1Pole_ota_alt.setCutoffFreq(frequency);
			break;
		case  30:
		case  31:
		case  32:
		case 130:
		case 131:
		case 132:
			filter1Pole_transistor_tanh.setCutoffFreq(frequency);
			break;
		case  40:
		case  41:
		case  42:
		case 140:
		case 141:
		case 142:
			filter1Pole_transistor_alt.setCutoffFreq(frequency);
			break;
		case 200:
		case 201:
		case 202:
		case 300:
		case 301:
		case 302:
			ladderFilter2Pole_linear.setCutoffFreq(frequency);
			ladderFilter2Pole_linear.setResonance(resonance);
			break;
		case 210:
		case 211:
		case 212:
		case 310:
		case 311:
		case 312:
			ladderFilter2Pole_ota_tanh.setCutoffFreq(frequency);
			ladderFilter2Pole_ota_tanh.setResonance(resonance);
			break;
		case 220:
		case 221:
		case 222:
		case 320:
		case 321:
		case 322:
			ladderFilter2Pole_ota_alt.setCutoffFreq(frequency);
			ladderFilter2Pole_ota_alt.setResonance(resonance);
			break;
		case 230:
		case 231:
		case 232:
		case 330:
		case 331:
		case 332:
			ladderFilter2Pole_transistor_tanh.setCutoffFreq(frequency);
			ladderFilter2Pole_transistor_tanh.setResonance(resonance);
			break;
		case 240:
		case 241:
		case 242:
		case 340:
		case 341:
		case 342:
			ladderFilter2Pole_transistor_alt.setCutoffFreq(frequency);
			ladderFilter2Pole_transistor_alt.setResonance(resonance);
			break;
		case 400:
		case 401:
		case 402:
		case 500:
		case 501:
		case 502:
		case 600:
		case 601:
		case 602:
			ladderFilter4Pole_linear.setCutoffFreq(frequency);
			ladderFilter4Pole_linear.setResonance(resonance);
			break;
		case 410:
		case 411:
		case 412:
		case 510:
		case 511:
		case 512:
		case 610:
		case 611:
		case 612:
			ladderFilter4Pole_ota_tanh.setCutoffFreq(frequency);
			ladderFilter4Pole_ota_tanh.setResonance(resonance);
			break;
		case 420:
		case 421:
		case 422:
		case 520:
		case 521:
		case 522:
		case 620:
		case 621:
		case 622:
		case 720:
		case 721:
		case 722:
			ladderFilter4Pole_ota_alt.setCutoffFreq(frequency);
			ladderFilter4Pole_ota_alt.setResonance(resonance);
			break;
		case 430:
		case 431:
		case 432:
		case 530:
		case 531:
		case 532:
		case 630:
		case 631:
		case 632:
		case 730:
		case 731:
		case 732:
			ladderFilter4Pole_transistor_tanh.setCutoffFreq(frequency);
			ladderFilter4Pole_transistor_tanh.setResonance(resonance);
			break;
		case 440:
		case 441:
		case 442:
		case 540:
		case 541:
		case 542:
		case 640:
		case 641:
		case 642:
		case 740:
		case 741:
		case 742:
			ladderFilter4Pole_transistor_alt.setCutoffFreq(frequency);
			ladderFilter4Pole_transistor_alt.setResonance(resonance);
			break;
		case 700:
		case 701:
		case 702:
			ladderFilter4Pole_linear.setCutoffFreq(frequency);
			ladderFilter4Pole_linear.setResonance(resonance);
			break;
		case 710:
		case 711:
		case 712:
			ladderFilter4Pole_ota_tanh.setCutoffFreq(frequency);
			ladderFilter4Pole_ota_tanh.setResonance(resonance);
			break;
		case 800:
		case 801:
		case 802:
		case 900:
		case 901:
		case 902:
			sallenKeyFilterLpBp_linear.setCutoffFreq(frequency);
			sallenKeyFilterLpBp_linear.setResonance(resonance);
			break;
		case 810:
		case 811:
		case 812:
		case 910:
		case 911:
		case 912:
			sallenKeyFilterLpBp_ota_tanh.setCutoffFreq(frequency);
			sallenKeyFilterLpBp_ota_tanh.setResonance(resonance);
			break;
		case 820:
		case 821:
		case 822:
		case 920:
		case 921:
		case 922:
			sallenKeyFilterLpBp_ota_alt.setCutoffFreq(frequency);
			sallenKeyFilterLpBp_ota_alt.setResonance(resonance);
			break;
		case 830:
		case 831:
		case 832:
		case 930:
		case 931:
		case 932:
			sallenKeyFilterLpBp_transistor_tanh.setCutoffFreq(frequency);
			sallenKeyFilterLpBp_transistor_tanh.setResonance(resonance);
			break;
		case 840:
		case 841:
		case 842:
		case 940:
		case 941:
		case 942:
			sallenKeyFilterLpBp_transistor_alt.setCutoffFreq(frequency);
			sallenKeyFilterLpBp_transistor_alt.setResonance(resonance);
			break;
		case 1000:
		case 1001:
		case 1002:
		case 1100:
		case 1101:
		case 1102:
			sallenKeyFilterHp_linear.setCutoffFreq(frequency);
			sallenKeyFilterHp_linear.setResonance(resonance);
			break;
		case 1010:
		case 1011:
		case 1012:
		case 1110:
		case 1111:
		case 1112:
			sallenKeyFilterHp_ota_tanh.setCutoffFreq(frequency);
			sallenKeyFilterHp_ota_tanh.setResonance(resonance);
			break;
		case 1020:
		case 1021:
		case 1022:
		case 1120:
		case 1121:
		case 1122:
			sallenKeyFilterHp_ota_alt.setCutoffFreq(frequency);
			sallenKeyFilterHp_ota_alt.setResonance(resonance);
			break;
		case 1030:
		case 1031:
		case 1032:
		case 1130:
		case 1131:
		case 1132:
			sallenKeyFilterHp_transistor_tanh.setCutoffFreq(frequency);
			sallenKeyFilterHp_transistor_tanh.setResonance(resonance);
			break;
		case 1040:
		case 1041:
		case 1042:
		case 1140:
		case 1141:
		case 1142:
			sallenKeyFilterHp_transistor_alt.setCutoffFreq(frequency);
			sallenKeyFilterHp_transistor_alt.setResonance(resonance);
			break;
		case 1200:
		case 1201:
		case 1202:
		case 1210:
		case 1211:
		case 1212:
		case 1220:
		case 1221:
		case 1222:
		case 1230:
		case 1231:
		case 1232:
		case 1240:
		case 1241:
		case 1242:
			combFilter.setFreq(frequency);
			combFilter.setFeedback(resonance);
			break;
		case 1300:
		case 1301:
		case 1302:
		case 1310:
		case 1311:
		case 1312:
		case 1320:
		case 1321:
		case 1322:
		case 1330:
		case 1331:
		case 1332:
		case 1340:
		case 1341:
		case 1342:
			combFilter.setFreq(2.f * frequency);
			combFilter.setNegativeFeedback(resonance);
			break;
		case 1400:
		case 1401:
		case 1402:
			diodeClipper_linear.setCutoffFreq(frequency);
			diodeClipper_linear.setResonance(resonance);
			break;
		case 1410:
		case 1411:
		case 1412:
			diodeClipper_ota_tanh.setCutoffFreq(frequency);
			diodeClipper_ota_tanh.setResonance(resonance);
			break;
		case 1420:
		case 1421:
		case 1422:
			diodeClipper_ota_alt.setCutoffFreq(frequency);
			diodeClipper_ota_alt.setResonance(resonance);
			break;
		case 1430:
		case 1431:
		case 1432:
			diodeClipper_transistor_tanh.setCutoffFreq(frequency);
			diodeClipper_transistor_tanh.setResonance(resonance);
			break;
		case 1440:
		case 1441:
		case 1442:
			diodeClipper_transistor_alt.setCutoffFreq(frequency);
			diodeClipper_transistor_alt.setResonance(resonance);
			break;
		case 1500:
		case 1501:
		case 1502:
			diodeClipperAsym_linear.setCutoffFreq(frequency);
			diodeClipperAsym_linear.setResonance(resonance);
			break;
		case 1510:
		case 1511:
		case 1512:
			diodeClipperAsym_ota_tanh.setCutoffFreq(frequency);
			diodeClipperAsym_ota_tanh.setResonance(resonance);
			break;
		case 1520:
		case 1521:
		case 1522:
			diodeClipperAsym_ota_alt.setCutoffFreq(frequency);
			diodeClipperAsym_ota_alt.setResonance(resonance);
			break;
		case 1530:
		case 1531:
		case 1532:
			diodeClipperAsym_transistor_tanh.setCutoffFreq(frequency);
			diodeClipperAsym_transistor_tanh.setResonance(resonance);
			break;
		case 1540:
		case 1541:
		case 1542:
			diodeClipperAsym_transistor_alt.setCutoffFreq(frequency);
			diodeClipperAsym_transistor_alt.setResonance(resonance);
			break;
		}
	}

	float_4 process(float_4 in, float_4 dt)
	{
		switch (switchValue)
		{
		case 0:
			filter1Pole_linear.processEuler(in, dt);
			return filter1Pole_linear.lowpass();
		case 1:
			filter1Pole_linear.processRK2(in, dt);
			return filter1Pole_linear.lowpass();
		case 2:
			filter1Pole_linear.processRK4(in, dt);
			return filter1Pole_linear.lowpass();
		case 10:
			filter1Pole_ota_tanh.processEuler(in, dt);
			return filter1Pole_ota_tanh.lowpass();
		case 11:
			filter1Pole_ota_tanh.processRK2(in, dt);
			return filter1Pole_ota_tanh.lowpass();
		case 12:
			filter1Pole_ota_tanh.processRK4(in, dt);
			return filter1Pole_ota_tanh.lowpass();
		case 20:
			filter1Pole_ota_alt.processEuler(in, dt);
			return filter1Pole_ota_alt.lowpass();
		case 21:
			filter1Pole_ota_alt.processRK2(in, dt);
			return filter1Pole_ota_alt.lowpass();
		case 22:
			filter1Pole_ota_alt.processRK4(in, dt);
			return filter1Pole_ota_alt.lowpass();
		case 30:
			filter1Pole_transistor_tanh.processEuler(in, dt);
			return filter1Pole_transistor_tanh.lowpass();
		case 31:
			filter1Pole_transistor_tanh.processRK2(in, dt);
			return filter1Pole_transistor_tanh.lowpass();
		case 32:
			filter1Pole_transistor_tanh.processRK4(in, dt);
			return filter1Pole_transistor_tanh.lowpass();
		case 40:
			filter1Pole_transistor_alt.processEuler(in, dt);
			return filter1Pole_transistor_alt.lowpass();
		case 41:
			filter1Pole_transistor_alt.processRK2(in, dt);
			return filter1Pole_transistor_alt.lowpass();
		case 42:
			filter1Pole_transistor_alt.processRK4(in, dt);
			return filter1Pole_transistor_alt.lowpass();
		case 100:
			filter1Pole_linear.processEuler(in, dt);
			return filter1Pole_linear.highpass();
		case 101:
			filter1Pole_linear.processRK2(in, dt);
			return filter1Pole_linear.highpass();
		case 102:
			filter1Pole_linear.processRK4(in, dt);
			return filter1Pole_linear.highpass();
		case 110:
			filter1Pole_ota_tanh.processEuler(in, dt);
			return filter1Pole_ota_tanh.highpass();
		case 111:
			filter1Pole_ota_tanh.processRK2(in, dt);
			return filter1Pole_ota_tanh.highpass();
		case 112:
			filter1Pole_ota_tanh.processRK4(in, dt);
			return filter1Pole_ota_tanh.highpass();
		case 120:
			filter1Pole_ota_alt.processEuler(in, dt);
			return filter1Pole_ota_alt.highpass();
		case 121:
			filter1Pole_ota_alt.processRK2(in, dt);
			return filter1Pole_ota_alt.highpass();
		case 122:
			filter1Pole_ota_alt.processRK4(in, dt);
			return filter1Pole_ota_alt.highpass();
		case 130:
			filter1Pole_transistor_tanh.processEuler(in, dt);
			return filter1Pole_transistor_tanh.highpass();
		case 131:
			filter1Pole_transistor_tanh.processRK2(in, dt);
			return filter1Pole_transistor_tanh.highpass();
		case 132:
			filter1Pole_transistor_tanh.processRK4(in, dt);
			return filter1Pole_transistor_tanh.highpass();
		case 140:
			filter1Pole_transistor_alt.processEuler(in, dt);
			return filter1Pole_transistor_alt.highpass();
		case 141:
			filter1Pole_transistor_alt.processRK2(in, dt);
			return filter1Pole_transistor_alt.highpass();
		case 142:
			filter1Pole_transistor_alt.processRK4(in, dt);
			return filter1Pole_transistor_alt.highpass();
		case 200:
			ladderFilter2Pole_linear.processEuler(in, dt);
			return ladderFilter2Pole_linear.lowpass();
		case 201:
			ladderFilter2Pole_linear.processRK2(in, dt);
			return ladderFilter2Pole_linear.lowpass();
		case 202:
			ladderFilter2Pole_linear.processRK4(in, dt);
			return ladderFilter2Pole_linear.lowpass();
		case 210:
			ladderFilter2Pole_ota_tanh.processEuler(in, dt);
			return ladderFilter2Pole_ota_tanh.lowpass();
		case 211:
			ladderFilter2Pole_ota_tanh.processRK2(in, dt);
			return ladderFilter2Pole_ota_tanh.lowpass();
		case 212:
			ladderFilter2Pole_ota_tanh.processRK4(in, dt);
			return ladderFilter2Pole_ota_tanh.lowpass();
		case 220:
			ladderFilter2Pole_ota_alt.processEuler(in, dt);
			return ladderFilter2Pole_ota_alt.lowpass();
		case 221:
			ladderFilter2Pole_ota_alt.processRK2(in, dt);
			return ladderFilter2Pole_ota_alt.lowpass();
		case 222:
			ladderFilter2Pole_ota_alt.processRK4(in, dt);
			return ladderFilter2Pole_ota_alt.lowpass();
		case 230:
			ladderFilter2Pole_transistor_tanh.processEuler(in, dt);
			return ladderFilter2Pole_transistor_tanh.lowpass();
		case 231:
			ladderFilter2Pole_transistor_tanh.processRK2(in, dt);
			return ladderFilter2Pole_transistor_tanh.lowpass();
		case 232:
			ladderFilter2Pole_transistor_tanh.processRK4(in, dt);
			return ladderFilter2Pole_transistor_tanh.lowpass();
		case 240:
			ladderFilter2Pole_transistor_alt.processEuler(in, dt);
			return ladderFilter2Pole_transistor_alt.lowpass();
		case 241:
			ladderFilter2Pole_transistor_alt.processRK2(in, dt);
			return ladderFilter2Pole_transistor_alt.lowpass();
		case 242:
			ladderFilter2Pole_transistor_alt.processRK4(in, dt);
			return ladderFilter2Pole_transistor_alt.lowpass();
		case 300:
			ladderFilter2Pole_linear.processEuler(in, dt);
			return ladderFilter2Pole_linear.bandpass();
		case 301:
			ladderFilter2Pole_linear.processRK2(in, dt);
			return ladderFilter2Pole_linear.bandpass();
		case 302:
			ladderFilter2Pole_linear.processRK4(in, dt);
			return ladderFilter2Pole_linear.bandpass();
		case 310:
			ladderFilter2Pole_ota_tanh.processEuler(in, dt);
			return ladderFilter2Pole_ota_tanh.bandpass();
		case 311:
			ladderFilter2Pole_ota_tanh.processRK2(in, dt);
			return ladderFilter2Pole_ota_tanh.bandpass();
		case 312:
			ladderFilter2Pole_ota_tanh.processRK4(in, dt);
			return ladderFilter2Pole_ota_tanh.bandpass();
		case 320:
			ladderFilter2Pole_ota_alt.processEuler(in, dt);
			return ladderFilter2Pole_ota_alt.bandpass();
		case 321:
			ladderFilter2Pole_ota_alt.processRK2(in, dt);
			return ladderFilter2Pole_ota_alt.bandpass();
		case 322:
			ladderFilter2Pole_ota_alt.processRK4(in, dt);
			return ladderFilter2Pole_ota_alt.bandpass();
		case 330:
			ladderFilter2Pole_transistor_tanh.processEuler(in, dt);
			return ladderFilter2Pole_transistor_tanh.bandpass();
		case 331:
			ladderFilter2Pole_transistor_tanh.processRK2(in, dt);
			return ladderFilter2Pole_transistor_tanh.bandpass();
		case 332:
			ladderFilter2Pole_transistor_tanh.processRK4(in, dt);
			return ladderFilter2Pole_transistor_tanh.bandpass();
		case 340:
			ladderFilter2Pole_transistor_alt.processEuler(in, dt);
			return ladderFilter2Pole_transistor_alt.bandpass();
		case 341:
			ladderFilter2Pole_transistor_alt.processRK2(in, dt);
			return ladderFilter2Pole_transistor_alt.bandpass();
		case 342:
			ladderFilter2Pole_transistor_alt.processRK4(in, dt);
			return ladderFilter2Pole_transistor_alt.bandpass();
		case 400:
			ladderFilter4Pole_linear.processEuler(in, dt);
			return ladderFilter4Pole_linear.lowpass6();
		case 401:
			ladderFilter4Pole_linear.processRK2(in, dt);
			return ladderFilter4Pole_linear.lowpass6();
		case 402:
			ladderFilter4Pole_linear.processRK4(in, dt);
			return ladderFilter4Pole_linear.lowpass6();
		case 410:
			ladderFilter4Pole_ota_tanh.processEuler(in, dt);
			return ladderFilter4Pole_ota_tanh.lowpass6();
		case 411:
			ladderFilter4Pole_ota_tanh.processRK2(in, dt);
			return ladderFilter4Pole_ota_tanh.lowpass6();
		case 412:
			ladderFilter4Pole_ota_tanh.processRK4(in, dt);
			return ladderFilter4Pole_ota_tanh.lowpass6();
		case 420:
			ladderFilter4Pole_ota_alt.processEuler(in, dt);
			return ladderFilter4Pole_ota_alt.lowpass6();
		case 421:
			ladderFilter4Pole_ota_alt.processRK2(in, dt);
			return ladderFilter4Pole_ota_alt.lowpass6();
		case 422:
			ladderFilter4Pole_ota_alt.processRK4(in, dt);
			return ladderFilter4Pole_ota_alt.lowpass6();
		case 430:
			ladderFilter4Pole_transistor_tanh.processEuler(in, dt);
			return ladderFilter4Pole_transistor_tanh.lowpass6();
		case 431:
			ladderFilter4Pole_transistor_tanh.processRK2(in, dt);
			return ladderFilter4Pole_transistor_tanh.lowpass6();
		case 432:
			ladderFilter4Pole_transistor_tanh.processRK4(in, dt);
			return ladderFilter4Pole_transistor_tanh.lowpass6();
		case 440:
			ladderFilter4Pole_transistor_alt.processEuler(in, dt);
			return ladderFilter4Pole_transistor_alt.lowpass6();
		case 441:
			ladderFilter4Pole_transistor_alt.processRK2(in, dt);
			return ladderFilter4Pole_transistor_alt.lowpass6();
		case 442:
			ladderFilter4Pole_transistor_alt.processRK4(in, dt);
			return ladderFilter4Pole_transistor_alt.lowpass6();
		case 500:
			ladderFilter4Pole_linear.processEuler(in, dt);
			return ladderFilter4Pole_linear.lowpass12();
		case 501:
			ladderFilter4Pole_linear.processRK2(in, dt);
			return ladderFilter4Pole_linear.lowpass12();
		case 502:
			ladderFilter4Pole_linear.processRK4(in, dt);
			return ladderFilter4Pole_linear.lowpass12();
		case 510:
			ladderFilter4Pole_ota_tanh.processEuler(in, dt);
			return ladderFilter4Pole_ota_tanh.lowpass12();
		case 511:
			ladderFilter4Pole_ota_tanh.processRK2(in, dt);
			return ladderFilter4Pole_ota_tanh.lowpass12();
		case 512:
			ladderFilter4Pole_ota_tanh.processRK4(in, dt);
			return ladderFilter4Pole_ota_tanh.lowpass12();
		case 520:
			ladderFilter4Pole_ota_alt.processEuler(in, dt);
			return ladderFilter4Pole_ota_alt.lowpass12();
		case 521:
			ladderFilter4Pole_ota_alt.processRK2(in, dt);
			return ladderFilter4Pole_ota_alt.lowpass12();
		case 522:
			ladderFilter4Pole_ota_alt.processRK4(in, dt);
			return ladderFilter4Pole_ota_alt.lowpass12();
		case 530:
			ladderFilter4Pole_transistor_tanh.processEuler(in, dt);
			return ladderFilter4Pole_transistor_tanh.lowpass12();
		case 531:
			ladderFilter4Pole_transistor_tanh.processRK2(in, dt);
			return ladderFilter4Pole_transistor_tanh.lowpass12();
		case 532:
			ladderFilter4Pole_transistor_tanh.processRK4(in, dt);
			return ladderFilter4Pole_transistor_tanh.lowpass12();
		case 540:
			ladderFilter4Pole_transistor_alt.processEuler(in, dt);
			return ladderFilter4Pole_transistor_alt.lowpass12();
		case 541:
			ladderFilter4Pole_transistor_alt.processRK2(in, dt);
			return ladderFilter4Pole_transistor_alt.lowpass12();
		case 542:
			ladderFilter4Pole_transistor_alt.processRK4(in, dt);
			return ladderFilter4Pole_transistor_alt.lowpass12();
		case 600:
			ladderFilter4Pole_linear.processEuler(in, dt);
			return ladderFilter4Pole_linear.lowpass18();
		case 601:
			ladderFilter4Pole_linear.processRK2(in, dt);
			return ladderFilter4Pole_linear.lowpass18();
		case 602:
			ladderFilter4Pole_linear.processRK4(in, dt);
			return ladderFilter4Pole_linear.lowpass18();
		case 610:
			ladderFilter4Pole_ota_tanh.processEuler(in, dt);
			return ladderFilter4Pole_ota_tanh.lowpass18();
		case 611:
			ladderFilter4Pole_ota_tanh.processRK2(in, dt);
			return ladderFilter4Pole_ota_tanh.lowpass18();
		case 612:
			ladderFilter4Pole_ota_tanh.processRK4(in, dt);
			return ladderFilter4Pole_ota_tanh.lowpass18();
		case 620:
			ladderFilter4Pole_ota_alt.processEuler(in, dt);
			return ladderFilter4Pole_ota_alt.lowpass18();
		case 621:
			ladderFilter4Pole_ota_alt.processRK2(in, dt);
			return ladderFilter4Pole_ota_alt.lowpass18();
		case 622:
			ladderFilter4Pole_ota_alt.processRK4(in, dt);
			return ladderFilter4Pole_ota_alt.lowpass18();
		case 630:
			ladderFilter4Pole_transistor_tanh.processEuler(in, dt);
			return ladderFilter4Pole_transistor_tanh.lowpass18();
		case 631:
			ladderFilter4Pole_transistor_tanh.processRK2(in, dt);
			return ladderFilter4Pole_transistor_tanh.lowpass18();
		case 632:
			ladderFilter4Pole_transistor_tanh.processRK4(in, dt);
			return ladderFilter4Pole_transistor_tanh.lowpass18();
		case 640:
			ladderFilter4Pole_transistor_alt.processEuler(in, dt);
			return ladderFilter4Pole_transistor_alt.lowpass18();
		case 641:
			ladderFilter4Pole_transistor_alt.processRK2(in, dt);
			return ladderFilter4Pole_transistor_alt.lowpass18();
		case 642:
			ladderFilter4Pole_transistor_alt.processRK4(in, dt);
			return ladderFilter4Pole_transistor_alt.lowpass18();
		case 700:
			ladderFilter4Pole_linear.processEuler(in, dt);
			return ladderFilter4Pole_linear.lowpass24();
		case 701:
			ladderFilter4Pole_linear.processRK2(in, dt);
			return ladderFilter4Pole_linear.lowpass24();
		case 702:
			ladderFilter4Pole_linear.processRK4(in, dt);
			return ladderFilter4Pole_linear.lowpass24();
		case 710:
			ladderFilter4Pole_ota_tanh.processEuler(in, dt);
			return ladderFilter4Pole_ota_tanh.lowpass24();
		case 711:
			ladderFilter4Pole_ota_tanh.processRK2(in, dt);
			return ladderFilter4Pole_ota_tanh.lowpass24();
		case 712:
			ladderFilter4Pole_ota_tanh.processRK4(in, dt);
			return ladderFilter4Pole_ota_tanh.lowpass24();
		case 720:
			ladderFilter4Pole_ota_alt.processEuler(in, dt);
			return ladderFilter4Pole_ota_alt.lowpass24();
		case 721:
			ladderFilter4Pole_ota_alt.processRK2(in, dt);
			return ladderFilter4Pole_ota_alt.lowpass24();
		case 722:
			ladderFilter4Pole_ota_alt.processRK4(in, dt);
			return ladderFilter4Pole_ota_alt.lowpass24();
		case 730:
			ladderFilter4Pole_transistor_tanh.processEuler(in, dt);
			return ladderFilter4Pole_transistor_tanh.lowpass24();
		case 731:
			ladderFilter4Pole_transistor_tanh.processRK2(in, dt);
			return ladderFilter4Pole_transistor_tanh.lowpass24();
		case 732:
			ladderFilter4Pole_transistor_tanh.processRK4(in, dt);
			return ladderFilter4Pole_transistor_tanh.lowpass24();
		case 740:
			ladderFilter4Pole_transistor_alt.processEuler(in, dt);
			return ladderFilter4Pole_transistor_alt.lowpass24();
		case 741:
			ladderFilter4Pole_transistor_alt.processRK2(in, dt);
			return ladderFilter4Pole_transistor_alt.lowpass24();
		case 742:
			ladderFilter4Pole_transistor_alt.processRK4(in, dt);
			return ladderFilter4Pole_transistor_alt.lowpass24();
		case 800:
			sallenKeyFilterLpBp_linear.processEuler(in, dt);
			return sallenKeyFilterLpBp_linear.lowpass();
		case 801:
			sallenKeyFilterLpBp_linear.processRK2(in, dt);
			return sallenKeyFilterLpBp_linear.lowpass();
		case 802:
			sallenKeyFilterLpBp_linear.processRK4(in, dt);
			return sallenKeyFilterLpBp_linear.lowpass();
		case 810:
			sallenKeyFilterLpBp_ota_tanh.processEuler(in, dt);
			return sallenKeyFilterLpBp_ota_tanh.lowpass();
		case 811:
			sallenKeyFilterLpBp_ota_tanh.processRK2(in, dt);
			return sallenKeyFilterLpBp_ota_tanh.lowpass();
		case 812:
			sallenKeyFilterLpBp_ota_tanh.processRK4(in, dt);
			return sallenKeyFilterLpBp_ota_tanh.lowpass();
		case 820:
			sallenKeyFilterLpBp_ota_alt.processEuler(in, dt);
			return sallenKeyFilterLpBp_ota_alt.lowpass();
		case 821:
			sallenKeyFilterLpBp_ota_alt.processRK2(in, dt);
			return sallenKeyFilterLpBp_ota_alt.lowpass();
		case 822:
			sallenKeyFilterLpBp_ota_alt.processRK4(in, dt);
			return sallenKeyFilterLpBp_ota_alt.lowpass();
		case 830:
			sallenKeyFilterLpBp_transistor_tanh.processEuler(in, dt);
			return sallenKeyFilterLpBp_transistor_tanh.lowpass();
		case 831:
			sallenKeyFilterLpBp_transistor_tanh.processRK2(in, dt);
			return sallenKeyFilterLpBp_transistor_tanh.lowpass();
		case 832:
			sallenKeyFilterLpBp_transistor_tanh.processRK4(in, dt);
			return sallenKeyFilterLpBp_transistor_tanh.lowpass();
		case 840:
			sallenKeyFilterLpBp_transistor_alt.processEuler(in, dt);
			return sallenKeyFilterLpBp_transistor_alt.lowpass();
		case 841:
			sallenKeyFilterLpBp_transistor_alt.processRK2(in, dt);
			return sallenKeyFilterLpBp_transistor_alt.lowpass();
		case 842:
			sallenKeyFilterLpBp_transistor_alt.processRK4(in, dt);
			return sallenKeyFilterLpBp_transistor_alt.lowpass();
		case 900:
			sallenKeyFilterLpBp_linear.processEuler(in, dt);
			return sallenKeyFilterLpBp_linear.bandpass();
		case 901:
			sallenKeyFilterLpBp_linear.processRK2(in, dt);
			return sallenKeyFilterLpBp_linear.bandpass();
		case 902:
			sallenKeyFilterLpBp_linear.processRK4(in, dt);
			return sallenKeyFilterLpBp_linear.bandpass();
		case 910:
			sallenKeyFilterLpBp_ota_tanh.processEuler(in, dt);
			return sallenKeyFilterLpBp_ota_tanh.bandpass();
		case 911:
			sallenKeyFilterLpBp_ota_tanh.processRK2(in, dt);
			return sallenKeyFilterLpBp_ota_tanh.bandpass();
		case 912:
			sallenKeyFilterLpBp_ota_tanh.processRK4(in, dt);
			return sallenKeyFilterLpBp_ota_tanh.bandpass();
		case 920:
			sallenKeyFilterLpBp_ota_alt.processEuler(in, dt);
			return sallenKeyFilterLpBp_ota_alt.bandpass();
		case 921:
			sallenKeyFilterLpBp_ota_alt.processRK2(in, dt);
			return sallenKeyFilterLpBp_ota_alt.bandpass();
		case 922:
			sallenKeyFilterLpBp_ota_alt.processRK4(in, dt);
			return sallenKeyFilterLpBp_ota_alt.bandpass();
		case 930:
			sallenKeyFilterLpBp_transistor_tanh.processEuler(in, dt);
			return sallenKeyFilterLpBp_transistor_tanh.bandpass();
		case 931:
			sallenKeyFilterLpBp_transistor_tanh.processRK2(in, dt);
			return sallenKeyFilterLpBp_transistor_tanh.bandpass();
		case 932:
			sallenKeyFilterLpBp_transistor_tanh.processRK4(in, dt);
			return sallenKeyFilterLpBp_transistor_tanh.bandpass();
		case 940:
			sallenKeyFilterLpBp_transistor_alt.processEuler(in, dt);
			return sallenKeyFilterLpBp_transistor_alt.bandpass();
		case 941:
			sallenKeyFilterLpBp_transistor_alt.processRK2(in, dt);
			return sallenKeyFilterLpBp_transistor_alt.bandpass();
		case 942:
			sallenKeyFilterLpBp_transistor_alt.processRK4(in, dt);
			return sallenKeyFilterLpBp_transistor_alt.bandpass();
		case 1000:
			sallenKeyFilterHp_linear.processEuler(in, dt);
			return sallenKeyFilterHp_linear.highpass6();
		case 1001:
			sallenKeyFilterHp_linear.processRK2(in, dt);
			return sallenKeyFilterHp_linear.highpass6();
		case 1002:
			sallenKeyFilterHp_linear.processRK4(in, dt);
			return sallenKeyFilterHp_linear.highpass6();
		case 1010:
			sallenKeyFilterHp_ota_tanh.processEuler(in, dt);
			return sallenKeyFilterHp_ota_tanh.highpass6();
		case 1011:
			sallenKeyFilterHp_ota_tanh.processRK2(in, dt);
			return sallenKeyFilterHp_ota_tanh.highpass6();
		case 1012:
			sallenKeyFilterHp_ota_tanh.processRK4(in, dt);
			return sallenKeyFilterHp_ota_tanh.highpass6();
		case 1020:
			sallenKeyFilterHp_ota_alt.processEuler(in, dt);
			return sallenKeyFilterHp_ota_alt.highpass6();
		case 1021:
			sallenKeyFilterHp_ota_alt.processRK2(in, dt);
			return sallenKeyFilterHp_ota_alt.highpass6();
		case 1022:
			sallenKeyFilterHp_ota_alt.processRK4(in, dt);
			return sallenKeyFilterHp_ota_alt.highpass6();
		case 1030:
			sallenKeyFilterHp_transistor_tanh.processEuler(in, dt);
			return sallenKeyFilterHp_transistor_tanh.highpass6();
		case 1031:
			sallenKeyFilterHp_transistor_tanh.processRK2(in, dt);
			return sallenKeyFilterHp_transistor_tanh.highpass6();
		case 1032:
			sallenKeyFilterHp_transistor_tanh.processRK4(in, dt);
			return sallenKeyFilterHp_transistor_tanh.highpass6();
		case 1040:
			sallenKeyFilterHp_transistor_alt.processEuler(in, dt);
			return sallenKeyFilterHp_transistor_alt.highpass6();
		case 1041:
			sallenKeyFilterHp_transistor_alt.processRK2(in, dt);
			return sallenKeyFilterHp_transistor_alt.highpass6();
		case 1042:
			sallenKeyFilterHp_transistor_alt.processRK4(in, dt);
			return sallenKeyFilterHp_transistor_alt.highpass6();
		case 1100:
			sallenKeyFilterHp_linear.processEuler(in, dt);
			return sallenKeyFilterHp_linear.highpass12();
		case 1101:
			sallenKeyFilterHp_linear.processRK2(in, dt);
			return sallenKeyFilterHp_linear.highpass12();
		case 1102:
			sallenKeyFilterHp_linear.processRK4(in, dt);
			return sallenKeyFilterHp_linear.highpass12();
		case 1110:
			sallenKeyFilterHp_ota_tanh.processEuler(in, dt);
			return sallenKeyFilterHp_ota_tanh.highpass12();
		case 1111:
			sallenKeyFilterHp_ota_tanh.processRK2(in, dt);
			return sallenKeyFilterHp_ota_tanh.highpass12();
		case 1112:
			sallenKeyFilterHp_ota_tanh.processRK4(in, dt);
			return sallenKeyFilterHp_ota_tanh.highpass12();
		case 1120:
			sallenKeyFilterHp_ota_alt.processEuler(in, dt);
			return sallenKeyFilterHp_ota_alt.highpass12();
		case 1121:
			sallenKeyFilterHp_ota_alt.processRK2(in, dt);
			return sallenKeyFilterHp_ota_alt.highpass12();
		case 1122:
			sallenKeyFilterHp_ota_alt.processRK4(in, dt);
			return sallenKeyFilterHp_ota_alt.highpass12();
		case 1130:
			sallenKeyFilterHp_transistor_tanh.processEuler(in, dt);
			return sallenKeyFilterHp_transistor_tanh.highpass12();
		case 1131:
			sallenKeyFilterHp_transistor_tanh.processRK2(in, dt);
			return sallenKeyFilterHp_transistor_tanh.highpass12();
		case 1132:
			sallenKeyFilterHp_transistor_tanh.processRK4(in, dt);
			return sallenKeyFilterHp_transistor_tanh.highpass12();
		case 1140:
			sallenKeyFilterHp_transistor_alt.processEuler(in, dt);
			return sallenKeyFilterHp_transistor_alt.highpass12();
		case 1141:
			sallenKeyFilterHp_transistor_alt.processRK2(in, dt);
			return sallenKeyFilterHp_transistor_alt.highpass12();
		case 1142:
			sallenKeyFilterHp_transistor_alt.processRK4(in, dt);
			return sallenKeyFilterHp_transistor_alt.highpass12();
		case 1200:
		case 1201:
		case 1202:
		case 1210:
		case 1211:
		case 1212:
		case 1220:
		case 1221:
		case 1222:
		case 1230:
		case 1231:
		case 1232:
		case 1240:
		case 1241:
		case 1242:
		case 1300:
		case 1301:
		case 1302:
		case 1310:
		case 1311:
		case 1312:
		case 1320:
		case 1321:
		case 1322:
		case 1330:
		case 1331:
		case 1332:
		case 1340:
		case 1341:
		case 1342:
			return combFilter.process(in, dt);
		case 1400:
			diodeClipper_linear.processEuler(in, dt);
			return diodeClipper_linear.out();
		case 1401:
			diodeClipper_linear.processRK2(in, dt);
			return diodeClipper_linear.out();
		case 1402:
			diodeClipper_linear.processRK4(in, dt);
			return diodeClipper_linear.out();
		case 1410:
			diodeClipper_ota_tanh.processEuler(in, dt);
			return diodeClipper_ota_tanh.out();
		case 1411:
			diodeClipper_ota_tanh.processRK2(in, dt);
			return diodeClipper_ota_tanh.out();
		case 1412:
			diodeClipper_ota_tanh.processRK4(in, dt);
			return diodeClipper_ota_tanh.out();
		case 1420:
			diodeClipper_ota_alt.processEuler(in, dt);
			return diodeClipper_ota_alt.out();
		case 1421:
			diodeClipper_ota_alt.processRK2(in, dt);
			return diodeClipper_ota_alt.out();
		case 1422:
			diodeClipper_ota_alt.processRK4(in, dt);
			return diodeClipper_ota_alt.out();
		case 1430:
			diodeClipper_transistor_tanh.processEuler(in, dt);
			return diodeClipper_transistor_tanh.out();
		case 1431:
			diodeClipper_transistor_tanh.processRK2(in, dt);
			return diodeClipper_transistor_tanh.out();
		case 1432:
			diodeClipper_transistor_tanh.processRK4(in, dt);
			return diodeClipper_transistor_tanh.out();
		case 1440:
			diodeClipper_transistor_alt.processEuler(in, dt);
			return diodeClipper_transistor_alt.out();
		case 1441:
			diodeClipper_transistor_alt.processRK2(in, dt);
			return diodeClipper_transistor_alt.out();
		case 1442:
			diodeClipper_transistor_alt.processRK4(in, dt);
			return diodeClipper_transistor_alt.out();
		case 1500:
			diodeClipperAsym_linear.processEuler(in, dt);
			return diodeClipperAsym_linear.out();
		case 1501:
			diodeClipperAsym_linear.processRK2(in, dt);
			return diodeClipperAsym_linear.out();
		case 1502:
			diodeClipperAsym_linear.processRK4(in, dt);
			return diodeClipperAsym_linear.out();
		case 1510:
			diodeClipperAsym_ota_tanh.processEuler(in, dt);
			return diodeClipperAsym_ota_tanh.out();
		case 1511:
			diodeClipperAsym_ota_tanh.processRK2(in, dt);
			return diodeClipperAsym_ota_tanh.out();
		case 1512:
			diodeClipperAsym_ota_tanh.processRK4(in, dt);
			return diodeClipperAsym_ota_tanh.out();
		case 1520:
			diodeClipperAsym_ota_alt.processEuler(in, dt);
			return diodeClipperAsym_ota_alt.out();
		case 1521:
			diodeClipperAsym_ota_alt.processRK2(in, dt);
			return diodeClipperAsym_ota_alt.out();
		case 1522:
			diodeClipperAsym_ota_alt.processRK4(in, dt);
			return diodeClipperAsym_ota_alt.out();
		case 1530:
			diodeClipperAsym_transistor_tanh.processEuler(in, dt);
			return diodeClipperAsym_transistor_tanh.out();
		case 1531:
			diodeClipperAsym_transistor_tanh.processRK2(in, dt);
			return diodeClipperAsym_transistor_tanh.out();
		case 1532:
			diodeClipperAsym_transistor_tanh.processRK4(in, dt);
			return diodeClipperAsym_transistor_tanh.out();
		case 1540:
			diodeClipperAsym_transistor_alt.processEuler(in, dt);
			return diodeClipperAsym_transistor_alt.out();
		case 1541:
			diodeClipperAsym_transistor_alt.processRK2(in, dt);
			return diodeClipperAsym_transistor_alt.out();
		case 1542:
			diodeClipperAsym_transistor_alt.processRK4(in, dt);
			return diodeClipperAsym_transistor_alt.out();
		default:
			return in;
		}
	}

	void processBlock(float_4* in, float_4 dt, int oversamplingRate)
	{
		switch (switchValue)
		{
		case 0:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				filter1Pole_linear.processEuler(in[i], dt);
				in[i] = filter1Pole_linear.lowpass();
			}
			break;
		case 1:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				filter1Pole_linear.processRK2(in[i], dt);
				in[i] = filter1Pole_linear.lowpass();
			}
			break;
		case 2:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				filter1Pole_linear.processRK4(in[i], dt);
				in[i] = filter1Pole_linear.lowpass();
			}
			break;
		case 10:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				filter1Pole_ota_tanh.processEuler(in[i], dt);
				in[i] = filter1Pole_ota_tanh.lowpass();
			}
			break;
		case 11:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				filter1Pole_ota_tanh.processRK2(in[i], dt);
				in[i] = filter1Pole_ota_tanh.lowpass();
			}
			break;
		case 12:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				filter1Pole_ota_tanh.processRK4(in[i], dt);
				in[i] = filter1Pole_ota_tanh.lowpass();
			}
			break;
		case 20:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				filter1Pole_ota_alt.processEuler(in[i], dt);
				in[i] = filter1Pole_ota_alt.lowpass();
			}
			break;
		case 21:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				filter1Pole_ota_alt.processRK2(in[i], dt);
				in[i] = filter1Pole_ota_alt.lowpass();
			}
			break;
		case 22:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				filter1Pole_ota_alt.processRK4(in[i], dt);
				in[i] = filter1Pole_ota_alt.lowpass();
			}
			break;
		case 30:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				filter1Pole_transistor_tanh.processEuler(in[i], dt);
				in[i] = filter1Pole_transistor_tanh.lowpass();
			}
			break;
		case 31:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				filter1Pole_transistor_tanh.processRK2(in[i], dt);
				in[i] = filter1Pole_transistor_tanh.lowpass();
			}
			break;
		case 32:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				filter1Pole_transistor_tanh.processRK4(in[i], dt);
				in[i] = filter1Pole_transistor_tanh.lowpass();
			}
			break;
		case 40:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				filter1Pole_transistor_alt.processEuler(in[i], dt);
				in[i] = filter1Pole_transistor_alt.lowpass();
			}
			break;
		case 41:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				filter1Pole_transistor_alt.processRK2(in[i], dt);
				in[i] = filter1Pole_transistor_alt.lowpass();
			}
			break;
		case 42:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				filter1Pole_transistor_alt.processRK4(in[i], dt);
				in[i] = filter1Pole_transistor_alt.lowpass();
			}
			break;
		case 100:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				filter1Pole_linear.processEuler(in[i], dt);
				in[i] = filter1Pole_linear.highpass();
			}
			break;
		case 101:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				filter1Pole_linear.processRK2(in[i], dt);
				in[i] = filter1Pole_linear.highpass();
			}
			break;
		case 102:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				filter1Pole_linear.processRK4(in[i], dt);
				in[i] = filter1Pole_linear.highpass();
			}
			break;
		case 110:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				filter1Pole_ota_tanh.processEuler(in[i], dt);
				in[i] = filter1Pole_ota_tanh.highpass();
			}
			break;
		case 111:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				filter1Pole_ota_tanh.processRK2(in[i], dt);
				in[i] = filter1Pole_ota_tanh.highpass();
			}
			break;
		case 112:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				filter1Pole_ota_tanh.processRK4(in[i], dt);
				in[i] = filter1Pole_ota_tanh.highpass();
			}
			break;
		case 120:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				filter1Pole_ota_alt.processEuler(in[i], dt);
				in[i] = filter1Pole_ota_alt.highpass();
			}
			break;
		case 121:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				filter1Pole_ota_alt.processRK2(in[i], dt);
				in[i] = filter1Pole_ota_alt.highpass();
			}
			break;
		case 122:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				filter1Pole_ota_alt.processRK4(in[i], dt);
				in[i] = filter1Pole_ota_alt.highpass();
			}
			break;
		case 130:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				filter1Pole_transistor_tanh.processEuler(in[i], dt);
				in[i] = filter1Pole_transistor_tanh.highpass();
			}
			break;
		case 131:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				filter1Pole_transistor_tanh.processRK2(in[i], dt);
				in[i] = filter1Pole_transistor_tanh.highpass();
			}
			break;
		case 132:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				filter1Pole_transistor_tanh.processRK4(in[i], dt);
				in[i] = filter1Pole_transistor_tanh.highpass();
			}
			break;
		case 140:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				filter1Pole_transistor_alt.processEuler(in[i], dt);
				in[i] = filter1Pole_transistor_alt.highpass();
			}
			break;
		case 141:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				filter1Pole_transistor_alt.processRK2(in[i], dt);
				in[i] = filter1Pole_transistor_alt.highpass();
			}
			break;
		case 142:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				filter1Pole_transistor_alt.processRK4(in[i], dt);
				in[i] = filter1Pole_transistor_alt.highpass();
			}
			break;
		case 200:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter2Pole_linear.processEuler(in[i], dt);
				in[i] = ladderFilter2Pole_linear.lowpass();
			}
			break;
		case 201:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter2Pole_linear.processRK2(in[i], dt);
				in[i] = ladderFilter2Pole_linear.lowpass();
			}
			break;
		case 202:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter2Pole_linear.processRK4(in[i], dt);
				in[i] = ladderFilter2Pole_linear.lowpass();
			}
			break;
		case 210:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter2Pole_ota_tanh.processEuler(in[i], dt);
				in[i] = ladderFilter2Pole_ota_tanh.lowpass();
			}
			break;
		case 211:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter2Pole_ota_tanh.processRK2(in[i], dt);
				in[i] = ladderFilter2Pole_ota_tanh.lowpass();
			}
			break;
		case 212:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter2Pole_ota_tanh.processRK4(in[i], dt);
				in[i] = ladderFilter2Pole_ota_tanh.lowpass();
			}
			break;
		case 220:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter2Pole_ota_alt.processEuler(in[i], dt);
				in[i] = ladderFilter2Pole_ota_alt.lowpass();
			}
			break;
		case 221:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter2Pole_ota_alt.processRK2(in[i], dt);
				in[i] = ladderFilter2Pole_ota_alt.lowpass();
			}
			break;
		case 222:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter2Pole_ota_alt.processRK4(in[i], dt);
				in[i] = ladderFilter2Pole_ota_alt.lowpass();
			}
			break;
		case 230:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter2Pole_transistor_tanh.processEuler(in[i], dt);
				in[i] = ladderFilter2Pole_transistor_tanh.lowpass();
			}
			break;
		case 231:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter2Pole_transistor_tanh.processRK2(in[i], dt);
				in[i] = ladderFilter2Pole_transistor_tanh.lowpass();
			}
			break;
		case 232:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter2Pole_transistor_tanh.processRK4(in[i], dt);
				in[i] = ladderFilter2Pole_transistor_tanh.lowpass();
			}
			break;
		case 240:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter2Pole_transistor_alt.processEuler(in[i], dt);
				in[i] = ladderFilter2Pole_transistor_alt.lowpass();
			}
			break;
		case 241:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter2Pole_transistor_alt.processRK2(in[i], dt);
				in[i] = ladderFilter2Pole_transistor_alt.lowpass();
			}
			break;
		case 242:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter2Pole_transistor_alt.processRK4(in[i], dt);
				in[i] = ladderFilter2Pole_transistor_alt.lowpass();
			}
			break;
		case 300:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter2Pole_linear.processEuler(in[i], dt);
				in[i] = ladderFilter2Pole_linear.bandpass();
			}
			break;
		case 301:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter2Pole_linear.processRK2(in[i], dt);
				in[i] = ladderFilter2Pole_linear.bandpass();
			}
			break;
		case 302:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter2Pole_linear.processRK4(in[i], dt);
				in[i] = ladderFilter2Pole_linear.bandpass();
			}
			break;
		case 310:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter2Pole_ota_tanh.processEuler(in[i], dt);
				in[i] = ladderFilter2Pole_ota_tanh.bandpass();
			}
			break;
		case 311:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter2Pole_ota_tanh.processRK2(in[i], dt);
				in[i] = ladderFilter2Pole_ota_tanh.bandpass();
			}
			break;
		case 312:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter2Pole_ota_tanh.processRK4(in[i], dt);
				in[i] = ladderFilter2Pole_ota_tanh.bandpass();
			}
			break;
		case 320:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter2Pole_ota_alt.processEuler(in[i], dt);
				in[i] = ladderFilter2Pole_ota_alt.bandpass();
			}
			break;
		case 321:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter2Pole_ota_alt.processRK2(in[i], dt);
				in[i] = ladderFilter2Pole_ota_alt.bandpass();
			}
			break;
		case 322:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter2Pole_ota_alt.processRK4(in[i], dt);
				in[i] = ladderFilter2Pole_ota_alt.bandpass();
			}
			break;
		case 330:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter2Pole_transistor_tanh.processEuler(in[i], dt);
				in[i] = ladderFilter2Pole_transistor_tanh.bandpass();
			}
			break;
		case 331:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter2Pole_transistor_tanh.processRK2(in[i], dt);
				in[i] = ladderFilter2Pole_transistor_tanh.bandpass();
			}
			break;
		case 332:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter2Pole_transistor_tanh.processRK4(in[i], dt);
				in[i] = ladderFilter2Pole_transistor_tanh.bandpass();
			}
			break;
		case 340:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter2Pole_transistor_alt.processEuler(in[i], dt);
				in[i] = ladderFilter2Pole_transistor_alt.bandpass();
			}
			break;
		case 341:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter2Pole_transistor_alt.processRK2(in[i], dt);
				in[i] = ladderFilter2Pole_transistor_alt.bandpass();
			}
			break;
		case 342:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter2Pole_transistor_alt.processRK4(in[i], dt);
				in[i] = ladderFilter2Pole_transistor_alt.bandpass();
			}
			break;
		case 400:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_linear.processEuler(in[i], dt);
				in[i] = ladderFilter4Pole_linear.lowpass6();
			}
			break;
		case 401:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_linear.processRK2(in[i], dt);
				in[i] = ladderFilter4Pole_linear.lowpass6();
			}
			break;
		case 402:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_linear.processRK4(in[i], dt);
				in[i] = ladderFilter4Pole_linear.lowpass6();
			}
			break;
		case 410:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_ota_tanh.processEuler(in[i], dt);
				in[i] = ladderFilter4Pole_ota_tanh.lowpass6();
			}
			break;
		case 411:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_ota_tanh.processRK2(in[i], dt);
				in[i] = ladderFilter4Pole_ota_tanh.lowpass6();
			}
			break;
		case 412:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_ota_tanh.processRK4(in[i], dt);
				in[i] = ladderFilter4Pole_ota_tanh.lowpass6();
			}
			break;
		case 420:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_ota_alt.processEuler(in[i], dt);
				in[i] = ladderFilter4Pole_ota_alt.lowpass6();
			}
			break;
		case 421:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_ota_alt.processRK2(in[i], dt);
				in[i] = ladderFilter4Pole_ota_alt.lowpass6();
			}
			break;
		case 422:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_ota_alt.processRK4(in[i], dt);
				in[i] = ladderFilter4Pole_ota_alt.lowpass6();
			}
			break;
		case 430:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_transistor_tanh.processEuler(in[i], dt);
				in[i] = ladderFilter4Pole_transistor_tanh.lowpass6();
			}
			break;
		case 431:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_transistor_tanh.processRK2(in[i], dt);
				in[i] = ladderFilter4Pole_transistor_tanh.lowpass6();
			}
			break;
		case 432:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_transistor_tanh.processRK4(in[i], dt);
				in[i] = ladderFilter4Pole_transistor_tanh.lowpass6();
			}
			break;
		case 440:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_transistor_alt.processEuler(in[i], dt);
				in[i] = ladderFilter4Pole_transistor_alt.lowpass6();
			}
			break;
		case 441:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_transistor_alt.processRK2(in[i], dt);
				in[i] = ladderFilter4Pole_transistor_alt.lowpass6();
			}
			break;
		case 442:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_transistor_alt.processRK4(in[i], dt);
				in[i] = ladderFilter4Pole_transistor_alt.lowpass6();
			}
			break;
		case 500:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_linear.processEuler(in[i], dt);
				in[i] = ladderFilter4Pole_linear.lowpass12();
			}
			break;
		case 501:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_linear.processRK2(in[i], dt);
				in[i] = ladderFilter4Pole_linear.lowpass12();
			}
			break;
		case 502:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_linear.processRK4(in[i], dt);
				in[i] = ladderFilter4Pole_linear.lowpass12();
			}
			break;
		case 510:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_ota_tanh.processEuler(in[i], dt);
				in[i] = ladderFilter4Pole_ota_tanh.lowpass12();
			}
			break;
		case 511:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_ota_tanh.processRK2(in[i], dt);
				in[i] = ladderFilter4Pole_ota_tanh.lowpass12();
			}
			break;
		case 512:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_ota_tanh.processRK4(in[i], dt);
				in[i] = ladderFilter4Pole_ota_tanh.lowpass12();
			}
			break;
		case 520:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_ota_alt.processEuler(in[i], dt);
				in[i] = ladderFilter4Pole_ota_alt.lowpass12();
			}
			break;
		case 521:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_ota_alt.processRK2(in[i], dt);
				in[i] = ladderFilter4Pole_ota_alt.lowpass12();
			}
			break;
		case 522:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_ota_alt.processRK4(in[i], dt);
				in[i] = ladderFilter4Pole_ota_alt.lowpass12();
			}
			break;
		case 530:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_transistor_tanh.processEuler(in[i], dt);
				in[i] = ladderFilter4Pole_transistor_tanh.lowpass12();
			}
			break;
		case 531:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_transistor_tanh.processRK2(in[i], dt);
				in[i] = ladderFilter4Pole_transistor_tanh.lowpass12();
			}
			break;
		case 532:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_transistor_tanh.processRK4(in[i], dt);
				in[i] = ladderFilter4Pole_transistor_tanh.lowpass12();
			}
			break;
		case 540:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_transistor_alt.processEuler(in[i], dt);
				in[i] = ladderFilter4Pole_transistor_alt.lowpass12();
			}
			break;
		case 541:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_transistor_alt.processRK2(in[i], dt);
				in[i] = ladderFilter4Pole_transistor_alt.lowpass12();
			}
			break;
		case 542:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_transistor_alt.processRK4(in[i], dt);
				in[i] = ladderFilter4Pole_transistor_alt.lowpass12();
			}
			break;
		case 600:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_linear.processEuler(in[i], dt);
				in[i] = ladderFilter4Pole_linear.lowpass18();
			}
			break;
		case 601:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_linear.processRK2(in[i], dt);
				in[i] = ladderFilter4Pole_linear.lowpass18();
			}
			break;
		case 602:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_linear.processRK4(in[i], dt);
				in[i] = ladderFilter4Pole_linear.lowpass18();
			}
			break;
		case 610:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_ota_tanh.processEuler(in[i], dt);
				in[i] = ladderFilter4Pole_ota_tanh.lowpass18();
			}
			break;
		case 611:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_ota_tanh.processRK2(in[i], dt);
				in[i] = ladderFilter4Pole_ota_tanh.lowpass18();
			}
			break;
		case 612:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_ota_tanh.processRK4(in[i], dt);
				in[i] = ladderFilter4Pole_ota_tanh.lowpass18();
			}
			break;
		case 620:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_ota_alt.processEuler(in[i], dt);
				in[i] = ladderFilter4Pole_ota_alt.lowpass18();
			}
			break;
		case 621:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_ota_alt.processRK2(in[i], dt);
				in[i] = ladderFilter4Pole_ota_alt.lowpass18();
			}
			break;
		case 622:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_ota_alt.processRK4(in[i], dt);
				in[i] = ladderFilter4Pole_ota_alt.lowpass18();
			}
			break;
		case 630:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_transistor_tanh.processEuler(in[i], dt);
				in[i] = ladderFilter4Pole_transistor_tanh.lowpass18();
			}
			break;
		case 631:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_transistor_tanh.processRK2(in[i], dt);
				in[i] = ladderFilter4Pole_transistor_tanh.lowpass18();
			}
			break;
		case 632:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_transistor_tanh.processRK4(in[i], dt);
				in[i] = ladderFilter4Pole_transistor_tanh.lowpass18();
			}
			break;
		case 640:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_transistor_alt.processEuler(in[i], dt);
				in[i] = ladderFilter4Pole_transistor_alt.lowpass18();
			}
			break;
		case 641:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_transistor_alt.processRK2(in[i], dt);
				in[i] = ladderFilter4Pole_transistor_alt.lowpass18();
			}
			break;
		case 642:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_transistor_alt.processRK4(in[i], dt);
				in[i] = ladderFilter4Pole_transistor_alt.lowpass18();
			}
			break;
		case 700:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_linear.processEuler(in[i], dt);
				in[i] = ladderFilter4Pole_linear.lowpass24();
			}
			break;
		case 701:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_linear.processRK2(in[i], dt);
				in[i] = ladderFilter4Pole_linear.lowpass24();
			}
			break;
		case 702:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_linear.processRK4(in[i], dt);
				in[i] = ladderFilter4Pole_linear.lowpass24();
			}
			break;
		case 710:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_ota_tanh.processEuler(in[i], dt);
				in[i] = ladderFilter4Pole_ota_tanh.lowpass24();
			}
			break;
		case 711:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_ota_tanh.processRK2(in[i], dt);
				in[i] = ladderFilter4Pole_ota_tanh.lowpass24();
			}
			break;
		case 712:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_ota_tanh.processRK4(in[i], dt);
				in[i] = ladderFilter4Pole_ota_tanh.lowpass24();
			}
			break;
		case 720:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_ota_alt.processEuler(in[i], dt);
				in[i] = ladderFilter4Pole_ota_alt.lowpass24();
			}
			break;
		case 721:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_ota_alt.processRK2(in[i], dt);
				in[i] = ladderFilter4Pole_ota_alt.lowpass24();
			}
			break;
		case 722:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_ota_alt.processRK4(in[i], dt);
				in[i] = ladderFilter4Pole_ota_alt.lowpass24();
			}
			break;
		case 730:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_transistor_tanh.processEuler(in[i], dt);
				in[i] = ladderFilter4Pole_transistor_tanh.lowpass24();
			}
			break;
		case 731:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_transistor_tanh.processRK2(in[i], dt);
				in[i] = ladderFilter4Pole_transistor_tanh.lowpass24();
			}
			break;
		case 732:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_transistor_tanh.processRK4(in[i], dt);
				in[i] = ladderFilter4Pole_transistor_tanh.lowpass24();
			}
			break;
		case 740:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_transistor_alt.processEuler(in[i], dt);
				in[i] = ladderFilter4Pole_transistor_alt.lowpass24();
			}
			break;
		case 741:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_transistor_alt.processRK2(in[i], dt);
				in[i] = ladderFilter4Pole_transistor_alt.lowpass24();
			}
			break;
		case 742:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				ladderFilter4Pole_transistor_alt.processRK4(in[i], dt);
				in[i] = ladderFilter4Pole_transistor_alt.lowpass24();
			}
			break;
		case 800:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterLpBp_linear.processEuler(in[i], dt);
				in[i] = sallenKeyFilterLpBp_linear.lowpass();
			}
			break;
		case 801:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterLpBp_linear.processRK2(in[i], dt);
				in[i] = sallenKeyFilterLpBp_linear.lowpass();
			}
			break;
		case 802:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterLpBp_linear.processRK4(in[i], dt);
				in[i] = sallenKeyFilterLpBp_linear.lowpass();
			}
			break;
		case 810:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterLpBp_ota_tanh.processEuler(in[i], dt);
				in[i] = sallenKeyFilterLpBp_ota_tanh.lowpass();
			}
			break;
		case 811:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterLpBp_ota_tanh.processRK2(in[i], dt);
				in[i] = sallenKeyFilterLpBp_ota_tanh.lowpass();
			}
			break;
		case 812:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterLpBp_ota_tanh.processRK4(in[i], dt);
				in[i] = sallenKeyFilterLpBp_ota_tanh.lowpass();
			}
			break;
		case 820:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterLpBp_ota_alt.processEuler(in[i], dt);
				in[i] = sallenKeyFilterLpBp_ota_alt.lowpass();
			}
			break;
		case 821:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterLpBp_ota_alt.processRK2(in[i], dt);
				in[i] = sallenKeyFilterLpBp_ota_alt.lowpass();
			}
			break;
		case 822:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterLpBp_ota_alt.processRK4(in[i], dt);
				in[i] = sallenKeyFilterLpBp_ota_alt.lowpass();
			}
			break;
		case 830:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterLpBp_transistor_tanh.processEuler(in[i], dt);
				in[i] = sallenKeyFilterLpBp_transistor_tanh.lowpass();
			}
			break;
		case 831:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterLpBp_transistor_tanh.processRK2(in[i], dt);
				in[i] = sallenKeyFilterLpBp_transistor_tanh.lowpass();
			}
			break;
		case 832:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterLpBp_transistor_tanh.processRK4(in[i], dt);
				in[i] = sallenKeyFilterLpBp_transistor_tanh.lowpass();
			}
			break;
		case 840:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterLpBp_transistor_alt.processEuler(in[i], dt);
				in[i] = sallenKeyFilterLpBp_transistor_alt.lowpass();
			}
			break;
		case 841:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterLpBp_transistor_alt.processRK2(in[i], dt);
				in[i] = sallenKeyFilterLpBp_transistor_alt.lowpass();
			}
			break;
		case 842:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterLpBp_transistor_alt.processRK4(in[i], dt);
				in[i] = sallenKeyFilterLpBp_transistor_alt.lowpass();
			}
			break;
		case 900:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterLpBp_linear.processEuler(in[i], dt);
				in[i] = sallenKeyFilterLpBp_linear.bandpass();
			}
			break;
		case 901:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterLpBp_linear.processRK2(in[i], dt);
				in[i] = sallenKeyFilterLpBp_linear.bandpass();
			}
			break;
		case 902:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterLpBp_linear.processRK4(in[i], dt);
				in[i] = sallenKeyFilterLpBp_linear.bandpass();
			}
			break;
		case 910:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterLpBp_ota_tanh.processEuler(in[i], dt);
				in[i] = sallenKeyFilterLpBp_ota_tanh.bandpass();
			}
			break;
		case 911:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterLpBp_ota_tanh.processRK2(in[i], dt);
				in[i] = sallenKeyFilterLpBp_ota_tanh.bandpass();
			}
			break;
		case 912:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterLpBp_ota_tanh.processRK4(in[i], dt);
				in[i] = sallenKeyFilterLpBp_ota_tanh.bandpass();
			}
			break;
		case 920:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterLpBp_ota_alt.processEuler(in[i], dt);
				in[i] = sallenKeyFilterLpBp_ota_alt.bandpass();
			}
			break;
		case 921:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterLpBp_ota_alt.processRK2(in[i], dt);
				in[i] = sallenKeyFilterLpBp_ota_alt.bandpass();
			}
			break;
		case 922:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterLpBp_ota_alt.processRK4(in[i], dt);
				in[i] = sallenKeyFilterLpBp_ota_alt.bandpass();
			}
			break;
		case 930:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterLpBp_transistor_tanh.processEuler(in[i], dt);
				in[i] = sallenKeyFilterLpBp_transistor_tanh.bandpass();
			}
			break;
		case 931:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterLpBp_transistor_tanh.processRK2(in[i], dt);
				in[i] = sallenKeyFilterLpBp_transistor_tanh.bandpass();
			}
			break;
		case 932:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterLpBp_transistor_tanh.processRK4(in[i], dt);
				in[i] = sallenKeyFilterLpBp_transistor_tanh.bandpass();
			}
			break;
		case 940:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterLpBp_transistor_alt.processEuler(in[i], dt);
				in[i] = sallenKeyFilterLpBp_transistor_alt.bandpass();
			}
			break;
		case 941:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterLpBp_transistor_alt.processRK2(in[i], dt);
				in[i] = sallenKeyFilterLpBp_transistor_alt.bandpass();
			}
			break;
		case 942:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterLpBp_transistor_alt.processRK4(in[i], dt);
				in[i] = sallenKeyFilterLpBp_transistor_alt.bandpass();
			}
			break;
		case 1000:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterHp_linear.processEuler(in[i], dt);
				in[i] = sallenKeyFilterHp_linear.highpass6();
			}
			break;
		case 1001:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterHp_linear.processRK2(in[i], dt);
				in[i] = sallenKeyFilterHp_linear.highpass6();
			}
			break;
		case 1002:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterHp_linear.processRK4(in[i], dt);
				in[i] = sallenKeyFilterHp_linear.highpass6();
			}
			break;
		case 1010:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterHp_ota_tanh.processEuler(in[i], dt);
				in[i] = sallenKeyFilterHp_ota_tanh.highpass6();
			}
			break;
		case 1011:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterHp_ota_tanh.processRK2(in[i], dt);
				in[i] = sallenKeyFilterHp_ota_tanh.highpass6();
			}
			break;
		case 1012:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterHp_ota_tanh.processRK4(in[i], dt);
				in[i] = sallenKeyFilterHp_ota_tanh.highpass6();
			}
			break;
		case 1020:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterHp_ota_alt.processEuler(in[i], dt);
				in[i] = sallenKeyFilterHp_ota_alt.highpass6();
			}
			break;
		case 1021:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterHp_ota_alt.processRK2(in[i], dt);
				in[i] = sallenKeyFilterHp_ota_alt.highpass6();
			}
			break;
		case 1022:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterHp_ota_alt.processRK4(in[i], dt);
				in[i] = sallenKeyFilterHp_ota_alt.highpass6();
			}
			break;
		case 1030:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterHp_transistor_tanh.processEuler(in[i], dt);
				in[i] = sallenKeyFilterHp_transistor_tanh.highpass6();
			}
			break;
		case 1031:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterHp_transistor_tanh.processRK2(in[i], dt);
				in[i] = sallenKeyFilterHp_transistor_tanh.highpass6();
			}
			break;
		case 1032:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterHp_transistor_tanh.processRK4(in[i], dt);
				in[i] = sallenKeyFilterHp_transistor_tanh.highpass6();
			}
			break;
		case 1040:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterHp_transistor_alt.processEuler(in[i], dt);
				in[i] = sallenKeyFilterHp_transistor_alt.highpass6();
			}
			break;
		case 1041:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterHp_transistor_alt.processRK2(in[i], dt);
				in[i] = sallenKeyFilterHp_transistor_alt.highpass6();
			}
			break;
		case 1042:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterHp_transistor_alt.processRK4(in[i], dt);
				in[i] = sallenKeyFilterHp_transistor_alt.highpass6();
			}
			break;
		case 1100:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterHp_linear.processEuler(in[i], dt);
				in[i] = sallenKeyFilterHp_linear.highpass12();
			}
			break;
		case 1101:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterHp_linear.processRK2(in[i], dt);
				in[i] = sallenKeyFilterHp_linear.highpass12();
			}
			break;
		case 1102:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterHp_linear.processRK4(in[i], dt);
				in[i] = sallenKeyFilterHp_linear.highpass12();
			}
			break;
		case 1110:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterHp_ota_tanh.processEuler(in[i], dt);
				in[i] = sallenKeyFilterHp_ota_tanh.highpass12();
			}
			break;
		case 1111:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterHp_ota_tanh.processRK2(in[i], dt);
				in[i] = sallenKeyFilterHp_ota_tanh.highpass12();
			}
			break;
		case 1112:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterHp_ota_tanh.processRK4(in[i], dt);
				in[i] = sallenKeyFilterHp_ota_tanh.highpass12();
			}
			break;
		case 1120:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterHp_ota_alt.processEuler(in[i], dt);
				in[i] = sallenKeyFilterHp_ota_alt.highpass12();
			}
			break;
		case 1121:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterHp_ota_alt.processRK2(in[i], dt);
				in[i] = sallenKeyFilterHp_ota_alt.highpass12();
			}
			break;
		case 1122:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterHp_ota_alt.processRK4(in[i], dt);
				in[i] = sallenKeyFilterHp_ota_alt.highpass12();
			}
			break;
		case 1130:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterHp_transistor_tanh.processEuler(in[i], dt);
				in[i] = sallenKeyFilterHp_transistor_tanh.highpass12();
			}
			break;
		case 1131:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterHp_transistor_tanh.processRK2(in[i], dt);
				in[i] = sallenKeyFilterHp_transistor_tanh.highpass12();
			}
			break;
		case 1132:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterHp_transistor_tanh.processRK4(in[i], dt);
				in[i] = sallenKeyFilterHp_transistor_tanh.highpass12();
			}
			break;
		case 1140:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterHp_transistor_alt.processEuler(in[i], dt);
				in[i] = sallenKeyFilterHp_transistor_alt.highpass12();
			}
			break;
		case 1141:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterHp_transistor_alt.processRK2(in[i], dt);
				in[i] = sallenKeyFilterHp_transistor_alt.highpass12();
			}
			break;
		case 1142:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				sallenKeyFilterHp_transistor_alt.processRK4(in[i], dt);
				in[i] = sallenKeyFilterHp_transistor_alt.highpass12();
			}
			break;
		case 1200:
		case 1201:
		case 1202:
		case 1210:
		case 1211:
		case 1212:
		case 1220:
		case 1221:
		case 1222:
		case 1230:
		case 1231:
		case 1232:
		case 1240:
		case 1241:
		case 1242:
		case 1300:
		case 1301:
		case 1302:
		case 1310:
		case 1311:
		case 1312:
		case 1320:
		case 1321:
		case 1322:
		case 1330:
		case 1331:
		case 1332:
		case 1340:
		case 1341:
		case 1342:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				in[i] = combFilter.process(in[i], dt);
			}
			break;
		case 1400:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				diodeClipper_linear.processEuler(in[i], dt);
				in[i] = diodeClipper_linear.out();
			}
			break;
		case 1401:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				diodeClipper_linear.processRK2(in[i], dt);
				in[i] = diodeClipper_linear.out();
			}
			break;
		case 1402:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				diodeClipper_linear.processRK4(in[i], dt);
				in[i] = diodeClipper_linear.out();
			}
			break;
		case 1410:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				diodeClipper_ota_tanh.processEuler(in[i], dt);
				in[i] = diodeClipper_ota_tanh.out();
			}
			break;
		case 1411:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				diodeClipper_ota_tanh.processRK2(in[i], dt);
				in[i] = diodeClipper_ota_tanh.out();
			}
			break;
		case 1412:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				diodeClipper_ota_tanh.processRK4(in[i], dt);
				in[i] = diodeClipper_ota_tanh.out();
			}
			break;
		case 1420:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				diodeClipper_ota_alt.processEuler(in[i], dt);
				in[i] = diodeClipper_ota_alt.out();
			}
			break;
		case 1421:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				diodeClipper_ota_alt.processRK2(in[i], dt);
				in[i] = diodeClipper_ota_alt.out();
			}
			break;
		case 1422:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				diodeClipper_ota_alt.processRK4(in[i], dt);
				in[i] = diodeClipper_ota_alt.out();
			}
			break;
		case 1430:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				diodeClipper_transistor_tanh.processEuler(in[i], dt);
				in[i] = diodeClipper_transistor_tanh.out();
			}
			break;
		case 1431:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				diodeClipper_transistor_tanh.processRK2(in[i], dt);
				in[i] = diodeClipper_transistor_tanh.out();
			}
			break;
		case 1432:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				diodeClipper_transistor_tanh.processRK4(in[i], dt);
				in[i] = diodeClipper_transistor_tanh.out();
			}
			break;
		case 1440:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				diodeClipper_transistor_alt.processEuler(in[i], dt);
				in[i] = diodeClipper_transistor_alt.out();
			}
			break;
		case 1441:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				diodeClipper_transistor_alt.processRK2(in[i], dt);
				in[i] = diodeClipper_transistor_alt.out();
			}
			break;
		case 1442:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				diodeClipper_transistor_alt.processRK4(in[i], dt);
				in[i] = diodeClipper_transistor_alt.out();
			}
			break;
		case 1500:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				diodeClipperAsym_linear.processEuler(in[i], dt);
				in[i] = diodeClipperAsym_linear.out();
			}
			break;
		case 1501:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				diodeClipperAsym_linear.processRK2(in[i], dt);
				in[i] = diodeClipperAsym_linear.out();
			}
			break;
		case 1502:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				diodeClipperAsym_linear.processRK4(in[i], dt);
				in[i] = diodeClipperAsym_linear.out();
			}
			break;
		case 1510:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				diodeClipperAsym_ota_tanh.processEuler(in[i], dt);
				in[i] = diodeClipperAsym_ota_tanh.out();
			}
			break;
		case 1511:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				diodeClipperAsym_ota_tanh.processRK2(in[i], dt);
				in[i] = diodeClipperAsym_ota_tanh.out();
			}
			break;
		case 1512:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				diodeClipperAsym_ota_tanh.processRK4(in[i], dt);
				in[i] = diodeClipperAsym_ota_tanh.out();
			}
			break;
		case 1520:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				diodeClipperAsym_ota_alt.processEuler(in[i], dt);
				in[i] = diodeClipperAsym_ota_alt.out();
			}
			break;
		case 1521:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				diodeClipperAsym_ota_alt.processRK2(in[i], dt);
				in[i] = diodeClipperAsym_ota_alt.out();
			}
			break;
		case 1522:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				diodeClipperAsym_ota_alt.processRK4(in[i], dt);
				in[i] = diodeClipperAsym_ota_alt.out();
			}
			break;
		case 1530:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				diodeClipperAsym_transistor_tanh.processEuler(in[i], dt);
				in[i] = diodeClipperAsym_transistor_tanh.out();
			}
			break;
		case 1531:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				diodeClipperAsym_transistor_tanh.processRK2(in[i], dt);
				in[i] = diodeClipperAsym_transistor_tanh.out();
			}
			break;
		case 1532:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				diodeClipperAsym_transistor_tanh.processRK4(in[i], dt);
				in[i] = diodeClipperAsym_transistor_tanh.out();
			}
			break;
		case 1540:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				diodeClipperAsym_transistor_alt.processEuler(in[i], dt);
				in[i] = diodeClipperAsym_transistor_alt.out();
			}
			break;
		case 1541:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				diodeClipperAsym_transistor_alt.processRK2(in[i], dt);
				in[i] = diodeClipperAsym_transistor_alt.out();
			}
			break;
		case 1542:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				diodeClipperAsym_transistor_alt.processRK4(in[i], dt);
				in[i] = diodeClipperAsym_transistor_alt.out();
			}
			break;
		case 1700:
		case 1701:
		case 1702:
		case 1710:
		case 1711:
		case 1712:
		case 1720:
		case 1721:
		case 1722:
		case 1730:
		case 1731:
		case 1732:
		case 1740:
		case 1741:
		case 1742:
			for (int i = 0; i < oversamplingRate; ++i)
			{
				in[i] = 0.f;
			}
		default:
			return;
		}
	}
};

}
