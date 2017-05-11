#ifndef MENUBAR_H
#define MENUBAR_H

namespace sketcherUI  {
class MenuBar;
}

class  sketcherUI::MenuBar
{
public:
    virtual ~MenuBar() = 0;
};

inline sketcherUI::MenuBar::~MenuBar() {}



#endif // MENUBAR_H
