//#include "menuassemblecontroller.h"
////#include "qtmenufactory.h"
////#include "menu.h"
//#include "mainwindowmanager.h"
//#include <vector>
////#include "qtmenubar.h"



//template <class MenuItem, class Menu, class MenuBar>
//std::vector<sketcherUI::MenuItemModel> sketcherUI::MenuAssembleController<MenuItem, Menu, MenuBar>::modelArray() {
//    if(_modelArray == NULL) {
//        _modelArray = sketcherUI::MenuAssembleController::createModelArray();
//    }

//    return *_modelArray;
//}

//template <class MenuItem, class Menu, class MenuBar>
//std::vector<sketcherUI::MenuItemModel>* sketcherUI::MenuAssembleController<MenuItem, Menu, MenuBar>::createModelArray() {
//    std::vector<sketcherUI::MenuItemModel>* v = new std::vector<sketcherUI::MenuItemModel>;
//    v->push_back(sketcherUI::MenuItemModel("File"));
//    v->push_back(sketcherUI::MenuItemModel("Edit"));

//    return v;
//}

//template <class MenuItem, class Menu, class MenuBar>
//void sketcherUI::MenuAssembleController<MenuItem, Menu, MenuBar>::loadMenu(void) {

//   // std::vector<MenuItemModel> v = modelArray();
//   // this->_menubar = _menuFactory->createMenubar();

//   //for(auto &model : v) {
//   //    Menu * menu = _menuFactory->createMenu(model);
//   //    if (!model.subMenu().empty()) {
//		 //  sketcherUI::MenuAssembleController<MenuItem, Menu, MenuBar>::addItemsToMenu(model.subMenu(), menu);
//   //    }
//	  // _menubar->addMenu(menu);
//   //}

//   ////Add menubar to window
//   //sketcherUI::MenuAssembleController<MenuItem, Menu, MenuBar>::addMenubarToWindow();
//}

//template <class MenuItem, class Menu, class MenuBar>
//void sketcherUI::MenuAssembleController<MenuItem, Menu, MenuBar>::addMenubarToWindow() {
//    MainWindowManager::sharedManager()->mainWindow()->setMenuBar(_menubar);
//}

//template <class MenuItem, class Menu, class MenuBar>
//void sketcherUI::MenuAssembleController<MenuItem, Menu, MenuBar>::addItemsToMenu(std::vector<sketcherUI::MenuItemModel> &modelArray, Menu * menu) {
//    for(auto &model : modelArray) {
//        if(!model.subMenu().empty()) {
//            Menu * subMenu = _menuFactory->createMenu(model);
//            addItemsToMenu(model.subMenu(), subMenu);
//			menu->addMenu(subMenu);
//        } else {
//            MenuItem * item = _menuFactory->createMenuItem(model);
//			menu->addItem(item);
//        }
//    }
//}
