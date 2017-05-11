#pragma once
#include <qwidget.h>
#include <qspinbox.h>
#include <QDoubleSpinBox>
#include <qlabel.h>
#include <functional>

namespace sketcherUI {
  class QTSliderItemDigital : public QWidget {
    Q_OBJECT
  public:
    QTSliderItemDigital(const char * text, std::function<void(int)> callBack, int min = 0, int max = 0);
    QTSliderItemDigital(const char * text, std::function<void(double)> callBack, double min = 0, double max = 0, int precise = 1);

    virtual ~QTSliderItemDigital() {}
  protected:
    QLabel * createLabel(const QString & text);
    QAbstractSpinBox * createSpinBox(int min, int max);
    QAbstractSpinBox * createSpinBox(double min, double max, int precise);
    void layoutSubViews();
  private:
    QLabel * _label;
    QAbstractSpinBox * _spinBox;
    std::function<void(int)> _intCallBack;
    std::function<void(double)> _doubleCallBack;
  private slots:
    void integerDigitalChanged(int value);
    void doubleDigitalChanged(double value);
  };
}
