#include "qtmenufactory.h"
#include "qaction.h"
#include "qtmenu.h"
//#include "menu.h"
//#include "menuitem.h"
#include "menuitemmodel.h"
#include <qstring.h>
#include "qtmenuitem.h"
#include "qtmenubar.h"
#include <functional>
#include <qobject.h>
#include <QtDebug>


//sketcherUI::QTMenutFactory::QTMenutFactory()
//{

//}
template<>
sketcherUI::QTMenu * sketcherUI::QTMenuFactory<sketcherUI::QTMenuItem, sketcherUI::QTMenu, sketcherUI::QTMenuBar>::createMenu(sketcherUI::MenuItemModel & model) {
     sketcherUI::QTMenu  * menu = new sketcherUI::QTMenu(QString(QObject::tr( model.text())));
     qDebug() << "Menu created\n";
     return menu;
 }

template<>
sketcherUI::QTMenuItem * sketcherUI::QTMenuFactory<sketcherUI::QTMenuItem, sketcherUI::QTMenu, sketcherUI::QTMenuBar>::createMenuItem(sketcherUI::MenuItemModel& model) {
     sketcherUI::QTMenuItem * item = new sketcherUI::QTMenuItem();
	 if (model.text() != nullptr)item->setText(QString(QObject::tr(model.text())));

     qDebug() << "MenuItem created\n";
     switch(model.itemType()) {
	 case sketcherUI::MenuItemTypeToggle: {
         QObject::connect(item, &sketcherUI::QTMenuItem::triggered, model.callBack());
         break;
     }
	 case sketcherUI::MenuItemTypeCheckable: {
         QObject::connect(item, &sketcherUI::QTMenuItem::setCheckable, model.callBack());
         break;
     }
	 case sketcherUI::MenuItemTypeSeparator: {
		 item->setSeparator(true);
	 }
     default:
         break;
     }
     qDebug() << "MenuItem connection complete\n";

     return item;

 }

template <>
sketcherUI::QTMenuBar * sketcherUI::QTMenuFactory<sketcherUI::QTMenuItem, sketcherUI::QTMenu, sketcherUI::QTMenuBar>::createMenubar() {
     sketcherUI::QTMenuBar * bar = new sketcherUI::QTMenuBar();
     qDebug() << "MenuBar created\n";
     return bar;
 }
