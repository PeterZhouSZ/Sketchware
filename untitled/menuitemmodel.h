#ifndef MENUITEMMODEL_H
#define MENUITEMMODEL_H
#include <functional>
#include <vector>

namespace sketcherUI {

enum MenuItemType {
    MenuItemTypeToggle,
    MenuItemTypeCheckable,
    MenuItemTypeWidget,
	MenuItemTypeSeparator,
    MenuItemTypeNone
};

class MenuItemModel
{
public:
    MenuItemModel(const char * text, const char * shortcut = NULL, MenuItemType itemType = MenuItemTypeToggle) :
        _text(const_cast<char*>(text)), _shortcut(shortcut), _itemType(itemType) { _subMenu = std::vector<sketcherUI::MenuItemModel>();}
    MenuItemModel(const char * text, MenuItemType itemType) : MenuItemModel(text, 0, itemType) {}
	MenuItemModel(MenuItemType itemType = MenuItemTypeSeparator) : MenuItemModel(NULL, NULL, itemType) {}

    const  char * text()const {return _text;}
    const char * shortcut() {return _shortcut;}
    MenuItemType itemType(){return _itemType;}

    std::vector<sketcherUI::MenuItemModel> subMenu(){return _subMenu;}

	void addSubMenu(MenuItemModel & model) { _subMenu.push_back(model); }

    /*template <typename CallbackType>
    void setCallBack(CallbackType callBack) { _callBack = new std::function<CallbackType>(callBack);}
    template < typename CallbackType>
    CallbackType& callBack() {return *static_cast<CallbackType*>(_callBack);}*/

	//void setCallBack(std::function<void()> callBack) { _callBack0 = callBack; }
	void setCallBack(std::function<void(bool)> callBack) { _callBack1 = callBack; }
	//std::function<void()> callBack() { return _callBack0; }
	std::function<void(bool)> callBack() { return _callBack1; }

    virtual ~MenuItemModel() {_subMenu.clear();}

private:
    char * _text;
    const char * _shortcut;
    MenuItemType _itemType;

    void * _callBack;
	std::function<void()> _callBack0;
	std::function<void(bool)> _callBack1;
   std::vector<sketcherUI::MenuItemModel> _subMenu;
};
}


#endif // MENUITEMMODEL_H
