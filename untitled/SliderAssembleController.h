#pragma once
#include "SliderCellModel.h"
#include "QTSliderFactory.h"
#include <vector>
#include <qdebug.h>
#include <SketchRetopo.hh>

namespace sketcherUI {


  template <class SliderFrame, class SliderItem>
  class SliderAssembleController {
  public:
	  SliderAssembleController() : _modelArray(NULL), _sliderFrame(NULL) {_sliderFactory = new QTSliderFactory<SliderFrame, SliderItem>();}

	virtual ~SliderAssembleController() {}

    SliderFrame * loadSlider();
    std::vector<SliderCellModel> modelArray() {if(_modelArray == NULL) _modelArray = createModelArray();return *_modelArray;}
    SliderFrame * slider() {if (_sliderFrame == NULL) _sliderFrame = loadSlider(); return _sliderFrame;}

  protected:
    SliderFactory<SliderFrame, SliderItem> * _sliderFactory;
  private:
    std::vector<SliderCellModel> *_modelArray;
    SliderFrame * _sliderFrame;

    std::vector<SliderCellModel> * createModelArray();
  };

  template<class SliderFrame, class SliderItem>
  std::vector<SliderCellModel> * SliderAssembleController<SliderFrame, SliderItem>::createModelArray() {
	  std::vector<SliderCellModel> * modelArray = new std::vector<SliderCellModel>;
	  std::map<std::string, std::string> m;
	  m["min"] = "0";
	  m["max"] = "100";
	  modelArray->push_back(SliderCellModel("index", SKSliderCellTypeDigitalInteger, m));
	  modelArray->back().setCallBack(std::function<void(int)>([](int value){qDebug() << value; }));
	  m.clear();
	  //
	  modelArray->push_back(SliderCellModel("show quads", SKSliderCellTypeCheckbox));
	  modelArray->back().setCallBack(std::function<void(bool)>([](bool isShown){SketchRetopo::get_instance().configRender.always_show_quads = isShown; }));
	  //
	  modelArray->push_back(SliderCellModel("basemesh render line", SKSliderCellTypeCheckbox));
	  modelArray->back().setCallBack(std::function<void(bool)>([&](bool isShown){SketchRetopo::get_instance().configRender.basemesh_render_line = isShown; }));
	  //
	  modelArray->push_back(SliderCellModel("show axis", SKSliderCellTypeCheckbox));
	  modelArray->back().setCallBack(std::function<void(bool)>([&](bool isShown){SketchRetopo::get_instance().configRender.show_axis = isShown; }));
	  //
	  modelArray->push_back(SliderCellModel("show bbox", SKSliderCellTypeCheckbox));
	  modelArray->back().setCallBack(std::function<void(bool)>([&](bool isShown){SketchRetopo::get_instance().configRender.show_bbox = isShown; }));
	  //
	  m["0"] = "default";
	  m["1"] = "no basemesh";
	  m["2"] = "basemesh only";
	  modelArray->push_back(SliderCellModel("render_mode", SKSliderCellTypeOptions, m));
	  modelArray->back().setCallBack(std::function<void(int)>([&](int value){ SketchRetopo::get_instance().configRender.mode = (ConfigRender::Mode)value; }));
	  m.clear();


	  return modelArray;
  }

  template<class SliderFrame, class SliderItem>
  SliderFrame * SliderAssembleController<SliderFrame, SliderItem>::loadSlider() {
    if (_sliderFrame != nullptr) {
      delete _sliderFrame;
    }

    std::vector<SliderCellModel> v = modelArray();
    this->_sliderFrame = _sliderFactory->CreateSliderFrame("");

    for (auto &model : v) {
      SliderItem * item = _sliderFactory->CreateSliderItem(model);
      _sliderFrame->addItem(item);
    }

    return _sliderFrame;
  }

}
