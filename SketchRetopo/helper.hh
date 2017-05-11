#pragma once

#include <string>
#include <algorithm>
#include <boost/lexical_cast.hpp>
#include <kt84/glut_clone/font.hh>

// function to decompose fname in the format of <fname_core>.<fname_num>.xml
inline bool decompose_xml_fname(const std::string& fname, std::string& fname_core, int& fname_num) {
    auto pos1 = fname.find_last_of('.');
    if (pos1 == std::string::npos) return false;
    
    auto fname_ext = fname.substr(pos1 + 1, fname.size() - pos1 - 1);
    if (fname_ext != "xml" && fname_ext != "XML") return false;
    
    auto pos2 = fname.find_last_of('.', pos1 - 1);
    if (pos2 == std::string::npos) return false;
    
    auto num_str = fname.substr(pos2 + 1, pos1 - pos2 - 1);
    fname_num = -1;
    try { fname_num = boost::lexical_cast<unsigned int>(num_str); } catch (const boost::bad_lexical_cast&) {}
    if (fname_num == -1) return false;
    
    fname_core = fname.substr(0, pos2);
    
    return true;
};

inline std::string de_stringify (const std::string* p, size_t n) {
    return std::accumulate(p, p + n / sizeof(std::string), std::string());
}

inline void draw_bitmap_string(
    void* fontID,
    const std::string& string,
    int raster_ox, int raster_oy,
    int border_width = 0,
    double border_color_r = 0.0, double border_color_g = 0.0, double border_color_b = 0.0)
{
    if (border_width > 0) {
        glPushAttrib(GL_CURRENT_BIT);
        glColor3d(border_color_r, border_color_g, border_color_b);
        for (int dx = -border_width; dx <= border_width; ++dx)
        for (int dy = -border_width; dy <= border_width; ++dy)
        {
            if (dx == 0 && dy == 0) continue;
            glRasterPos2i(raster_ox + dx, raster_oy + dy);
            kt84::glutBitmapString(fontID, string);
        }
        glPopAttrib();
    }
    glRasterPos2i(raster_ox, raster_oy);
    kt84::glutBitmapString(fontID, string);
}
