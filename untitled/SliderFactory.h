#pragma once
#include "SliderCellModel.h"
#include <functional>

namespace sketcherUI {

	template <class SliderFrame, class SliderItem>
	class SliderFactory {
	public:

		virtual SliderFrame * CreateSliderFrame(const char * text) = 0;
		virtual SliderItem * CreateSliderItem(SliderCellModel & model);

	protected:
		virtual SliderItem * CreateCheckBox(const char * text, std::function<void(bool)> callBack) = 0;
		virtual SliderItem * CreateDigitalInteger(const char * text, std::function<void(int)> callBack, std::map<std::string, std::string> extraInfo) = 0;
		virtual SliderItem * CreateDigitalDouble(const char * text, std::function<void(double)> callBack, std::map<std::string, std::string> extraInfo) = 0;
		virtual SliderItem * CreateOptions(const char * text, std::function<void(int)> callBack, std::map<std::string, std::string> extraInfo) = 0;
	};

	template <class SliderFrame, class SliderItem>
	SliderItem * SliderFactory<SliderFrame, SliderItem>::CreateSliderItem(SliderCellModel& model) {
		SliderItem * item = nullptr;
		switch (model.type()) {
		case SKSliderCellTypeCheckbox: {
			item = CreateCheckBox(model.text(), model.callBack1());
			break;
		}
		case SKSliderCellTypeDigitalInteger: {
			item = CreateDigitalInteger(model.text(), model.callBack1i(), model.extraInfo());
			break;
		}
		case SKSliderCellTypeDigitalDouble: {
			item = CreateDigitalDouble(model.text(), model.callBack1d(), model.extraInfo());
			break;
		}
		case SKSliderCellTypeOptions: {
			item = CreateOptions(model.text(), model.callBack1i(), model.extraInfo());
			break;
		}
		default:
			break;
		}
		return item;
	}
}