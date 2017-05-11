#ifndef MENUITEM_H
#define MENUITEM_H

namespace sketcherUI {
class MenuItem
{
public:
    MenuItem() {}
    virtual ~MenuItem() = 0;

};
}

inline sketcherUI::MenuItem::~MenuItem() {}


#endif // MENUITEM_H
