#ifndef MENUABSTRACTFACTORY_H
#define MENUABSTRACTFACTORY_H

#include "menuitemmodel.h"
//#include "menuitem.h"
//#include "menubar.h"
//#include "menu.h"

namespace sketcherUI {
template<class MenuItem, class Menu, class MenuBar>
class MenuAbstractFactory
{
public:
    virtual MenuItem * createMenuItem(sketcherUI::MenuItemModel& menuModel) = 0;
    virtual Menu * createMenu(sketcherUI::MenuItemModel& menuModel) = 0;
    virtual MenuBar * createMenubar() = 0;
};
}


#endif // MENUABSTRACTFACTORY_H
