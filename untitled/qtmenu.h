#ifndef QTMENU_H
#define QTMENU_H

#include <QWidget>
//#include <menu.h>
#include <qmenu.h>
#include <qaction.h>

namespace sketcherUI  {
class QTMenu;
}
class sketcherUI::QTMenu : public QMenu
{
    Q_OBJECT
public:
    explicit  sketcherUI::QTMenu() : QMenu() {}
    sketcherUI:: QTMenu(const QString & text): QMenu(text) {}

	void addItem(QAction * item) { addAction(item); }
    virtual  ~QTMenu() {}

};



#endif // QTMENU_H
