#ifndef TEST_H
#define TEST_H

//namespace sketcherUI {
//template<typename A, typename B, typename C> class Test;
//}

template <typename A>
class Test
{
public:
    Test() {}

    virtual ~Test() {}

    void loadMenu();
};

template<typename A>
void Test<A>::loadMenu() {

}


#endif // TEST_H
