#pragma once
#include "SliderFactory.h"

namespace sketcherUI {
template <class SliderFrame, class SliderItem> class QTSliderFactory;
}

template <class SliderFrame, class SliderItem>
class sketcherUI::QTSliderFactory : public SliderFactory<SliderFrame, SliderItem> {
public:
	sketcherUI::QTSliderFactory<SliderFrame, SliderItem>() {}
  virtual ~QTSliderFactory() {}

  virtual SliderFrame * CreateSliderFrame(const char * text)override;
  //virtual SliderItem * CreateSliderItem(SliderCellModel & model);

protected:
  virtual SliderItem * CreateCheckBox(const char * text, std::function<void(bool)> callBack)override;
  virtual SliderItem * CreateDigitalInteger(const char * text, std::function<void(int)> callBack, std::map<std::string, std::string> extraInfo)override;
  virtual SliderItem * CreateDigitalDouble(const char * text, std::function<void(double)> callBack, std::map<std::string, std::string> extraInfo)override;
  virtual SliderItem * CreateOptions(const char * text, std::function<void(int)> callBack, std::map<std::string, std::string> extraInfo)override;
};
