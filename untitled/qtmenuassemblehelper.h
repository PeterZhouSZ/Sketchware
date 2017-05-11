#ifndef QTMENUASSEMBLEHELPER_H
#define QTMENUASSEMBLEHELPER_H
#include "menuassemblehelper.h"

namespace sketcherUI {
class QTMenuAssembleHelper : public MenuAssembleHelper
{
public:
    QTMenuAssembleHelper() {}

    virtual ~QTMenuAssembleHelper() {}

    virtual void addMenuItemToMenu(MenuItem * menuItem, Menu * menu) override;
    virtual void addMenuToMenubar(Menu * menu, MenuBar * menubar) override;
    virtual void addMenuToMenu(Menu * subMenu, Menu * menu) override;
};
}


#endif // QTMENUASSEMBLEHELPER_H
