#include "QTSliderItemOptions.h"
#include <QHBoxLayout>

sketcherUI::QTSliderItemOptions::QTSliderItemOptions(const char * text, std::vector<const char *> options, std::function<void(int)> callBack) {
  _label = createLabel(QString(QObject::tr(text)));
  _comboBox = createComboBox(options, callBack);
  _callBack = callBack;

  layoutSubViews();

  connect(_comboBox, SIGNAL(activated(int)), this, SLOT(optionChanged(int)));
}

QComboBox * sketcherUI::QTSliderItemOptions::createComboBox(std::vector<const char *> &options, std::function<void(int)> callBack) {
    QComboBox * comboBox = new QComboBox();

    //Customize the comboBox
    for (auto &str : options) {
        comboBox->addItem(str);
    }

    return comboBox;
}

QLabel * sketcherUI::QTSliderItemOptions::createLabel(const QString &text) {
  QLabel * textLabel = new QLabel(text);

  return textLabel;
}

void sketcherUI::QTSliderItemOptions::optionChanged(int index) {
  _callBack(index);
}

void sketcherUI::QTSliderItemOptions::layoutSubViews() {
  QHBoxLayout * layout = new QHBoxLayout;
  layout->addWidget(_label);
  layout->addWidget(_comboBox);

  setLayout(layout);
}
