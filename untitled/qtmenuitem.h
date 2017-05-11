#ifndef QTMENUITEM_H
#define QTMENUITEM_H

#include <qaction.h>
#include <functional>
//#include "menuitem.h"

namespace sketcherUI  {
class QTMenuItem;
}

class  sketcherUI::QTMenuItem : public QAction
{
    Q_OBJECT
public:
     sketcherUI::QTMenuItem() : QAction() {}
     sketcherUI::QTMenuItem(const QString &text) : QAction(text) {}

    virtual ~QTMenuItem() {}

private:
   // function<callbackType> _callBack;


};


#endif // QTMENUITEM_H
