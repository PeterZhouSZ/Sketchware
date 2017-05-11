#pragma once
#include <qcombobox.h>
#include <qlabel.h>
#include <functional>

namespace sketcherUI {

  class QTSliderItemOptions : public QWidget {
    Q_OBJECT
  public:
    QTSliderItemOptions(const char * text, std::vector<const char *> options, std::function<void(int)> callBack);

    virtual ~QTSliderItemOptions() {}
  protected:
    QComboBox * createComboBox(std::vector<const char *>& options, std::function<void(int)> callBack);
    QLabel * createLabel(const QString &text);
    void layoutSubViews();
  private:
    QComboBox * _comboBox;
    QLabel * _label;
    std::function<void(int)> _callBack;
  private slots:
    void optionChanged(int index);
  };
}
