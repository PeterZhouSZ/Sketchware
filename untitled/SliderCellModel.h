#pragma once
#include <functional>
#include <map>
#include <string>

namespace sketcherUI {
enum SKSliderCellType {
  SKSliderCellTypeCheckbox = 0,
  SKSliderCellTypeDigitalInteger,
  SKSliderCellTypeDigitalDouble,
  SKSliderCellTypeOptions,
  SKSliderCellTypeNone
};
}

namespace sketcherUI {


  class SliderCellModel {
  public:
     SliderCellModel(const char *  text,  SKSliderCellType type, std::map<std::string, std::string> extraInfo) : _text(text), _type(type),  _extraInfo(extraInfo) {}
	 SliderCellModel(const char *  text, SKSliderCellType type) : SliderCellModel(text, type, std::map<std::string, std::string>()) {}
    virtual ~SliderCellModel() {}

    const char * text()const {return _text;}
    SKSliderCellType type()const {return _type;}
    std::map<std::string, std::string> extraInfo()const {return _extraInfo;}

    std::function<void()> callBack0() {return _callBack0;}
     std::function<void(bool)> callBack1() {return _callBack1;}
    std::function<void(int)> callBack1i() {return _callBack1i;}
    std::function<void(double)> callBack1d() {return _callBack1d;}

    void setCallBack(std::function<void()> callBack) {_callBack0 = callBack;}
    void setCallBack(std::function<void(bool)> callBack) {_callBack1 = callBack;}
    void setCallBack(std::function<void(int)> callBack) {_callBack1i = callBack;}
    void setCallBack(std::function<void(double)> callBack) {_callBack1d = callBack;}

  protected:
    std::map<std::string, std::string> _extraInfo;
  private:
    std::function<void()> _callBack0;
    std::function<void(bool)> _callBack1;
    std::function<void(int)> _callBack1i;
    std::function<void(double)> _callBack1d;
    const char * _text;
    SKSliderCellType _type;
  };
}
