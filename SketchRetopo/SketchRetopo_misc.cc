#include "SketchRetopo.hh"
#include "curvenetwork/Circulator.hh"
#include <sstream>
#include <base64/decode.h>
#include <lodepng/lodepng.h>
#include <kt84/math/RBF.hh>
#include <kt84/adjacent_pairs.hh>
#include <kt84/graphics/graphics_util.hh>
using namespace std;
using namespace Eigen;
using namespace kt84;

void SketchRetopo::state_set(EnumState enumState) {
    State* state_new = nullptr;
    if      (enumState == EnumState::Sketch      ) state_new = &stateSketch      ;
    else if (enumState == EnumState::DeformCurve ) state_new = &stateDeformCurve ;
    else if (enumState == EnumState::Laser       ) state_new = &stateLaser       ;
    else if (enumState == EnumState::Cylinder    ) state_new = &stateCylinder    ;
    else if (enumState == EnumState::EditCorner  ) state_new = &stateEditCorner  ;
    else if (enumState == EnumState::EditTopology) state_new = &stateEditTopology;
    else if (enumState == EnumState::QueryDB     ) state_new = &stateQueryDB     ;
    else if (enumState == EnumState::MoveVertex  ) state_new = &stateMoveVertex  ;
    else if (enumState == EnumState::EdgeLoop    ) state_new = &stateEdgeLoop    ;
    
    if (!state_new || state == state_new) return;
    
    state->exit_state();
    state = state_new;
    state->enter_state();
}
SketchRetopo::EnumState SketchRetopo::state_get() const {
    if      (state == &stateSketch      ) return EnumState::Sketch      ;
    else if (state == &stateDeformCurve ) return EnumState::DeformCurve ;
    else if (state == &stateLaser       ) return EnumState::Laser       ;
    else if (state == &stateCylinder    ) return EnumState::Cylinder    ;
    else if (state == &stateEditCorner  ) return EnumState::EditCorner  ;
    else if (state == &stateEditTopology) return EnumState::EditTopology;
    else if (state == &stateQueryDB     ) return EnumState::QueryDB     ;
    else if (state == &stateMoveVertex  ) return EnumState::MoveVertex  ;
    else if (state == &stateEdgeLoop    ) return EnumState::EdgeLoop    ;
    assert(false);
    return static_cast<EnumState>(-1);
}
void SketchRetopo::memento_store() {
    memento.store(curvenetwork);
    configTemp.autoSave.unsaved = true;
}
void SketchRetopo::memento_undo() {
    if (memento.undo(curvenetwork)) {
        curvenetwork.validate_all_ptr();
        for (auto& p : curvenetwork.patches)
            p.invalidate_displist();
        curvenetwork.invalidate_displist();
        state->enter_state();
        configTemp.autoSave.unsaved = true;
    }
}
void SketchRetopo::memento_redo() {
    if (memento.redo(curvenetwork)) {
        curvenetwork.validate_all_ptr();
        for (auto& p : curvenetwork.patches)
            p.invalidate_displist();
        curvenetwork.invalidate_displist();
        state->enter_state();
        configTemp.autoSave.unsaved = true;
    }
}
void SketchRetopo::material_texture_update() {
    // decode PNG from base64 string
    stringstream ss_encoded, ss_decoded;
    ss_encoded.str(material_texture_png_base64);
    base64::decoder D;
	D.decode(ss_encoded, ss_decoded);
    
    // read PNG using awesome lodepng library!
    vector<unsigned char> pixels;   // pixels in RGB
    unsigned int width, height;
    auto str_decoded = ss_decoded.str();
    lodepng::decode(pixels, width, height, reinterpret_cast<unsigned char*>(&str_decoded[0]), str_decoded.size(), LCT_RGB);
    
    // send pixel data to GPU
    material_texture.bind();
    material_texture.allocate(width, height, GL_RGB, GL_RGB);
    material_texture.copy_cpu2gpu(GL_UNSIGNED_BYTE, &pixels[0]);
    material_texture.unbind();
}
