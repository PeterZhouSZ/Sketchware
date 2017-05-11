#ifndef MENUASSEMBLEHELPER_H
#define MENUASSEMBLEHELPER_H

#include <vector>
#include "menuitemmodel.h"
#include "menu.h"
#include "menubar.h"
#include "menuitem.h"

namespace sketcherUI {
class MenuAssembleHelper
{
public:
    //virtual void assembleMenu(vector<MenuItemModel *> modelArray) = 0;
    virtual void addMenuItemToMenu(MenuItem * menuItem, Menu * menu) = 0;
    virtual void addMenuToMenubar(Menu * menu, MenuBar * menubar) = 0;
    virtual void addMenuToMenu(Menu * subMenu, Menu * menu) = 0;
};
}


#endif // MENUASSEMBLEHELPER_H
