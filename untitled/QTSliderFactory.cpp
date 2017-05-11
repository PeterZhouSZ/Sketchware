#include "SliderCellModel.h"
#include "QTSliderItemDigital.h"
#include "QTSliderItemCheckbox.h"
#include "QTSliderItemOptions.h"
#include <functional>
#include <QWidget>
#include "QTSliderFactory.h"
#include <QVBoxLayout>
#include <QDockWidget>
#include "qtsliderframe.h"

template <>
QWidget * sketcherUI::QTSliderFactory<sketcherUI::QTSliderFrame, QWidget>::CreateCheckBox(const char * text, std::function<void(bool)> callBack) {
  return  new QTSliderItemCheckbox(text, callBack);
}

template <>
sketcherUI::QTSliderFrame * sketcherUI::QTSliderFactory<sketcherUI::QTSliderFrame, QWidget>::CreateSliderFrame(const char * text) {
  QWidget * inner = new QWidget();
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setAlignment(Qt::AlignTop);

  inner->setLayout(layout);
  sketcherUI::QTSliderFrame* dock = new sketcherUI::QTSliderFrame(QString(QObject::tr(text)));
  dock->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable);
  dock->setWidget(inner);

  return dock;
}

template <>
QWidget * sketcherUI::QTSliderFactory<sketcherUI::QTSliderFrame, QWidget>::CreateDigitalInteger(const char * text, std::function<void(int)> callBack, std::map<std::string, std::string> extraInfo) {
  return new QTSliderItemDigital(text, callBack, std::stoi(extraInfo["min"]), std::stoi(extraInfo["max"]));
}

template <>
QWidget * sketcherUI::QTSliderFactory<sketcherUI::QTSliderFrame, QWidget>::CreateDigitalDouble(const char * text, std::function<void(double)> callBack, std::map<std::string, std::string> extraInfo) {
  int precise = 3;
  if (extraInfo.find("precise") != extraInfo.end()) {
    precise = std::stoi(extraInfo["precise"]);
  }
  return new QTSliderItemDigital(text, callBack, std::stod(extraInfo["min"]), std::stod(extraInfo["max"]), precise);
}

template <>
QWidget * sketcherUI::QTSliderFactory<sketcherUI::QTSliderFrame, QWidget>::CreateOptions(const char * text, std::function<void(int)> callBack, std::map<std::string, std::string> extraInfo) {
  std::vector<const char *> v;
  for (unsigned int i = 0; i < extraInfo.size(); i++) {
    v.push_back(extraInfo[std::to_string(i)].c_str());
  }
  return new QTSliderItemOptions(text, v, callBack);
}
