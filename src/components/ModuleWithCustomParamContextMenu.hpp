#pragma once

#include "plugin.hpp"

namespace musx {

using namespace rack;

struct ModuleWithCustomParamContextMenu : Module {
	virtual void appendToParamContextMenu(rack::app::ParamWidget* param, rack::ui::Menu* menu) {};
	virtual void appendToSwitchContextMenu(rack::app::ParamWidget* param, rack::ui::Menu* menu) {};
};

}
