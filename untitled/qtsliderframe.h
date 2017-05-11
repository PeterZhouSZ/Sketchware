#ifndef QTSLIDERFRAME_H
#define QTSLIDERFRAME_H
#include <QDockWidget>
#include <qwidget.h>
#include <qlayout.h>
#include <QVBoxLayout>

namespace sketcherUI {
class QTSliderFrame;
}

class sketcherUI::QTSliderFrame : public QDockWidget
{
	Q_OBJECT
public:
    sketcherUI::QTSliderFrame(const QString & text) :QDockWidget(text) {}

	void addItem(QWidget * item) {
		QLayout * layout = this->widget()->layout();
		layout->addWidget(item);
	}
};

#endif // QTSLIDERFRAME_H
