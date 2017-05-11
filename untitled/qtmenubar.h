 #ifndef QTMENUBAR_H
#define QTMENUBAR_H
//#include "menubar.h"
#include "qmenubar.h"

namespace sketcherUI {
class QTMenuBar;
}

class  sketcherUI::QTMenuBar : public QMenuBar
{
    Q_OBJECT
public:
     sketcherUI::QTMenuBar() : QMenuBar() {}

    virtual ~QTMenuBar(){}
};



#endif // QTMENUBAR_H
