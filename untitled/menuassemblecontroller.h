#ifndef MENUASSEMBLECONTROLLER_H
#define MENUASSEMBLECONTROLLER_H

#include <vector>
#include "menuitemmodel.h"
//#include "menubar.h"
//#include "menuabstractfactory.h"
//#include "menuassemblehelper.h"
#include "qtmenufactory.h"
//#include "qtmenuassemblehelper.h"
#include <vector>
#include <iostream>
#include <qdebug.h>
#include "SketchRetopo.hh"
#include <algorithm>
#include <kt84/graphics/phong_tessellation.hh>
#include <kt84/Clock.hh>
#include <kt84/tinyfd_util.hh>
#include "helper.hh"

using namespace std;
using namespace Eigen;
using namespace kt84;
using namespace vcg::tri::pl;

namespace sketcherUI {
template <class MenuItem, class Menu, class MenuBar>
class MenuAssembleController;
}

template <class MenuItem, class Menu, class MenuBar>
class sketcherUI::MenuAssembleController
{
public:
    MenuAssembleController() : _modelArray(NULL), _menubar(NULL) { _menuFactory = new sketcherUI::QTMenuFactory<MenuItem, Menu, MenuBar>(); }
    virtual ~MenuAssembleController() {}

    MenuBar*  loadMenu(void);
    std::vector<sketcherUI::MenuItemModel>  modelArray();
	MenuBar *  menubar() { if (_menubar == nullptr) _menubar = loadMenu(); return _menubar }
protected:

    //void init();
    void addMenubarToWindow();

    sketcherUI::MenuAbstractFactory<MenuItem, Menu, MenuBar> * _menuFactory;
    //sketcherUI::MenuAssembleHelper * _helper;

private:
    void addItemsToMenu(std::vector<sketcherUI::MenuItemModel> &modelArray, Menu * menu);
    std::vector<sketcherUI::MenuItemModel>* createModelArray();
    std::vector<sketcherUI::MenuItemModel>* _modelArray;

    MenuBar * _menubar;
};

template <class MenuItem, class Menu, class MenuBar>
std::vector<sketcherUI::MenuItemModel> sketcherUI::MenuAssembleController<MenuItem, Menu, MenuBar>::modelArray() {
    if(_modelArray == NULL) {
		_modelArray = sketcherUI::MenuAssembleController<MenuItem, Menu, MenuBar>::createModelArray();
    }

    return *_modelArray;
}

template <class MenuItem, class Menu, class MenuBar>
std::vector<sketcherUI::MenuItemModel>* sketcherUI::MenuAssembleController<MenuItem, Menu, MenuBar>::createModelArray() {
    std::vector<sketcherUI::MenuItemModel>* v = new std::vector<sketcherUI::MenuItemModel>;
	auto &core = SketchRetopo::get_instance();

	//File
	sketcherUI::MenuItemModel fileModel("File");
	MenuItemModel model1("Import base mesh");
	model1.setCallBack(std::function<void(bool)>([&](bool){
		if (core.configTemp.autoSave.unsaved && !tinyfd_messageBox("SketchRetopo", "Current data is not saved yet. Continue?", "yesno", "question", 0))
			return;
		string fname = tinyfd_util::openFileDialog("SketchRetopo - import base mesh", "", { "*.obj", "*.off", "*.ply", "*.stl" }, false);
		if (!fname.empty()) {
			if (!core.import_basemesh(fname))
				tinyfd_messageBox("SketchRetopo - import base mesh", "Error occurred!", "ok", "error", 1);
		}
	}));
	fileModel.addSubMenu(model1);
	fileModel.addSubMenu(sketcherUI::MenuItemModel("Export base mesh"));
	fileModel.addSubMenu(sketcherUI::MenuItemModel("Export retopomesh"));
	fileModel.addSubMenu(sketcherUI::MenuItemModel());
	fileModel.addSubMenu(sketcherUI::MenuItemModel("Save xml"));
	fileModel.addSubMenu(sketcherUI::MenuItemModel("Load xml"));
	fileModel.addSubMenu(sketcherUI::MenuItemModel("Autosave", MenuItemTypeCheckable));
	fileModel.addSubMenu(sketcherUI::MenuItemModel());
	fileModel.addSubMenu(sketcherUI::MenuItemModel("Autosave Interval"));
	fileModel.addSubMenu(sketcherUI::MenuItemModel("Load texture"));
	fileModel.addSubMenu(sketcherUI::MenuItemModel("Load material"));

    v->push_back(fileModel);

	//Edit
	sketcherUI::MenuItemModel editModel("Edit");
	editModel.addSubMenu(sketcherUI::MenuItemModel("Clear"));

    v->push_back(editModel);

    return v;
}

template <class MenuItem, class Menu, class MenuBar>
MenuBar * sketcherUI::MenuAssembleController<MenuItem, Menu, MenuBar>::loadMenu(void) {

	if (_menubar != nullptr) delete _menubar;

    std::vector<MenuItemModel> v = modelArray();
    this->_menubar = _menuFactory->createMenubar();

   for(auto &model : v) {
       Menu * menu = _menuFactory->createMenu(model);
       if (!model.subMenu().empty()) {
           sketcherUI::MenuAssembleController<MenuItem, Menu, MenuBar>::addItemsToMenu(model.subMenu(), menu);
       }
       _menubar->addMenu(menu);
   }

   return _menubar;

   ////Add menubar to window
   //sketcherUI::MenuAssembleController<MenuItem, Menu, MenuBar>::addMenubarToWindow();
}

template <class MenuItem, class Menu, class MenuBar>
void sketcherUI::MenuAssembleController<MenuItem, Menu, MenuBar>::addMenubarToWindow() {
    MainWindowManager::sharedManager()->mainWindow()->setMenuBar(_menubar);
}

template <class MenuItem, class Menu, class MenuBar>
void sketcherUI::MenuAssembleController<MenuItem, Menu, MenuBar>::addItemsToMenu(std::vector<sketcherUI::MenuItemModel> &modelArray, Menu * menu) {
    for(auto &model : modelArray) {
        if(!model.subMenu().empty()) {
            Menu * subMenu = _menuFactory->createMenu(model);
            addItemsToMenu(model.subMenu(), subMenu);
            menu->addMenu(subMenu);
        } else {
            MenuItem * item = _menuFactory->createMenuItem(model);
            menu->addItem(item);
        }
    }
}



#endif // MENUASSEMBLECONTROLLER_H
