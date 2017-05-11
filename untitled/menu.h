#ifndef MENU_H
#define MENU_H

namespace sketcherUI {
class Menu
{
public:
    virtual ~Menu() = 0;
};
}

inline sketcherUI::Menu::~Menu() {}

#endif // MENU_H
