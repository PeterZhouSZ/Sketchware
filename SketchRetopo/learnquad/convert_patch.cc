#include "convert_patch.hh"
#include <kt84/vector_cast.hh>
using namespace std;
using namespace kt84;
using namespace vcg::tri::pl;

bool convert_patch(MT_PatchPriorityLookup<PolyMesh>& lookup, int index, curvenetwork::Patch& result) {
    if (lookup.numberOfPatches() <= index)
        return false;
    Patch<PolyMesh> giorgio_patch;
    lookup.at(index, giorgio_patch);
    if (giorgio_patch.isNull())
        return false;
    // copy data from Giorgio's mesh
    auto& giorgio_mesh = giorgio_patch.getMesh();
    result.clear();
    for (int i = 0; i < giorgio_mesh.vn; ++i) {
        auto v = result.add_vertex({});
        auto& giorgio_v = giorgio_mesh.vert[i];
        result.data(v).pn <<
            vector_cast<3, Eigen::Vector3d>(giorgio_v.P()),
            vector_cast<3, Eigen::Vector3d>(giorgio_v.N());
    }
    for (int i = 0; i < giorgio_mesh.fn; ++i) {
        OpenMesh::VertexHandle fv[4];
        for (int j = 0; j < 4; ++j)
            fv[j] = result.vertex_handle(distance(&giorgio_mesh.vert[0], giorgio_mesh.face[i].V(j)));
        result.add_face(fv[0], fv[1], fv[2], fv[3]);
    }
    // set corner index
    auto firstcorner = result.vertex_handle(distance(&giorgio_mesh.vert[0], giorgio_patch.getStartCorner().V()));
    auto h = result.halfedge_handle(firstcorner);
    auto num_subdiv = result.num_subdiv();
    for (int i = 0; i < num_subdiv.size(); ++i) {
        result.data(result.from_vertex_handle(h)).patchgen.corner_index = i;
        for (int j = 0; j < num_subdiv[i]; ++j)
            h = result.prev_halfedge_handle(h);
    }
    // set misc data
    result.laplaceSolver_init();
    result.set_halfedgeData();
    result.set_boundary_pn();
    return true;
}

