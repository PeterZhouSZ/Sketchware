#include "QTSliderItemDigital.h"
#include <QHBoxLayout>

sketcherUI::QTSliderItemDigital::QTSliderItemDigital(const char * text, std::function<void(int)> callBack, int min, int max) {
  _label = createLabel(QString(QObject::tr(text)));
  _spinBox = createSpinBox(min, max);
  _intCallBack = callBack;

  layoutSubViews();
  QObject::connect(_spinBox, SIGNAL(valueChanged(int)), this, SLOT(integerDigitalChanged(int)));
}

sketcherUI::QTSliderItemDigital::QTSliderItemDigital(const char * text, std::function<void(double)> callBack, double min, double max, int precise) {
  _label = createLabel(QString(QObject::tr(text)));
  _spinBox = createSpinBox(min, max, precise);
  _doubleCallBack = callBack;

  layoutSubViews();
  QObject::connect(_spinBox, SIGNAL(valueChanged(double)), this, SLOT(doubleDigitalChanged(double)));
}

QLabel * sketcherUI::QTSliderItemDigital::createLabel(const QString & text) {
  QLabel * label = new QLabel(text);

  return label;
}

QAbstractSpinBox * sketcherUI::QTSliderItemDigital::createSpinBox(int min, int max) {
  QSpinBox * spinbox = new QSpinBox();
  spinbox->setMinimum(min);
  spinbox->setMaximum(max);

  return spinbox;
}

QAbstractSpinBox * sketcherUI::QTSliderItemDigital::createSpinBox(double min, double max, int precise) {
  QDoubleSpinBox * spinbox = new QDoubleSpinBox();
  spinbox->setMinimum(min);
  spinbox->setMaximum(max);
  spinbox->setDecimals(precise);

  return spinbox;
}

void sketcherUI::QTSliderItemDigital::layoutSubViews() {
  QHBoxLayout * hLayout = new QHBoxLayout;
  hLayout->addWidget(_label);
  hLayout->addWidget(_spinBox);

  setLayout(hLayout);
}

void sketcherUI::QTSliderItemDigital::integerDigitalChanged(int value) {
  _intCallBack(value);
}

void sketcherUI::QTSliderItemDigital::doubleDigitalChanged(double value) {
  _doubleCallBack(value);
}
