#include "SketchRetopo.hh"
#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <GL/glew.h>
#include <kt84/Clock.hh>
#include <kt84/util.hh>
using namespace std;
using namespace Eigen;
using namespace kt84;

namespace {
    auto& core = SketchRetopo::get_instance();

//#ifdef _WIN32
//    inline FILE* popen(const char* command, const char* mode) { return _popen(command, mode); }
//    inline int pclose(FILE* file) { return _pclose(file); }
//#endif
    
    Vector3d scalar_to_color(double t) {
        double r = std::max<double>(0, 2 * t - 1);
        double g = 1 - 2 * std::abs<double>(t - 0.5);
        double b = std::max<double>(0, 1 - 2 * t);
        return Vector3d(r, g, b);
    }
}

void BaseMesh::init() {
    update_normals();
    
#ifndef NDEBUG
    debugInfo_get(true, true, true);
#endif
    
    cotanWeight_compute();
    centerOfMass_compute();
    boundingBox_compute();
    
    invalidate_displist();
}

void BaseMesh::render_fill() {
    displist_fill.render([&] () {
        glBegin(GL_TRIANGLES);
        for (auto f : faces()) {
            for (auto h : fh_range(f)) {
                auto v = to_vertex_handle(h);
                auto p  = point (v);
                auto n  = normal(v);
                auto overlay_uv = o2e(texcoord2D(h));
                glMultiTexCoord2dv(GL_TEXTURE1, overlay_uv.data());
                glNormal3dv  (n.data());
                glVertex3dv  (p.data());
            }
        }
        glEnd();
    });
}
void BaseMesh::render_line() {
    displist_line.render([&] () {
        glBegin(GL_LINES);
        for (auto e : edges()) {
            for (int i = 0; i < 2; ++i) {
                auto v = to_vertex_handle(halfedge_handle(e, i));
                auto p  = point (v);
                auto n  = normal(v);
                glNormal3dv(&n[0]);
                glVertex3dv(&p[0]);
            }
        }
        glEnd();
    });
}
PointNormal BaseMesh::get_pn(VHandle v) const {
    PointNormal pn;
    pn << o2e(point(v)), o2e(normal(v));
    return pn;
}
