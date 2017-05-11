#include "QTSliderItemCheckbox.h"


sketcherUI::QTSliderItemCheckbox::QTSliderItemCheckbox(const char * text, std::function<void(bool)> callBack, QWidget *parent) :
    QCheckBox(QString(QObject::tr(text)), parent) {
  _stateCallBack = callBack;

  QObject::connect(this, SIGNAL(toggled(bool)), this, SLOT(checkStateChanged(bool)));
}
