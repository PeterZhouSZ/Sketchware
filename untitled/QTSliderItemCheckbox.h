#pragma once
#include <qcheckbox.h>
#include <functional>

namespace sketcherUI {

  class QTSliderItemCheckbox : public QCheckBox {
    Q_OBJECT
  public:
    QTSliderItemCheckbox(const char * text, std::function<void(bool)> callBack, QWidget *parent = Q_NULLPTR);

    virtual ~QTSliderItemCheckbox() {}

  private:
    std::function<void(bool)> _stateCallBack;
  private slots:
    void checkStateChanged(bool checked) {_stateCallBack(checked);}
  };
}
