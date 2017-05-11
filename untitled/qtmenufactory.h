#ifndef QTMENUFACTORY_H
#define QTMENUFACTORY_H

#include "menuabstractfactory.h"

namespace sketcherUI {
	template <class MenuItem, class Menu, class MenuBar>
	class QTMenuFactory : public MenuAbstractFactory <MenuItem, Menu, MenuBar>
{
public:
    QTMenuFactory() {}

    virtual ~QTMenuFactory() {}

    virtual MenuItem * createMenuItem(sketcherUI::MenuItemModel& menuModel) override ;
    virtual Menu * createMenu(sketcherUI::MenuItemModel& menuModel) override;
    virtual MenuBar * createMenubar() override;


};
}


#endif // QTMENUFACTORY_H
