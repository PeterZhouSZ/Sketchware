#include "Contour2DEdge.h"
#include "kt84\graphics\graphics_util.hh"
#include "SketchRetopo.hh"

using namespace std;
using namespace Eigen;
using namespace kt84;
using namespace kt84::graphics_util;

namespace {
	auto & core = SketchRetopo::get_instance();
}

namespace SketcherCore {

Contour2DEdge::~Contour2DEdge()
{
}

OpenMesh::VertexHandle Contour2DEdge::firstPoint_3DPositionHandle()const {
	return core.basemesh.from_vertex_handle(halfEdge());
}

Vector3d Contour2DEdge::firstPoint_3DPosition()const {
	return o2e(core.basemesh.point(firstPoint_3DPositionHandle()));
}

OpenMesh::VertexHandle Contour2DEdge::secondPoint_3DPositionHandle()const {
	return core.basemesh.to_vertex_handle(halfEdge());
}

Vector3d Contour2DEdge::secondPoint_3DPosition()const {
	return o2e(core.basemesh.point(secondPoint_3DPositionHandle()));
}

OpenMesh::HalfedgeHandle Contour2DEdge::halfEdge()const {
	auto edge_handle = core.basemesh.edge_handle(_idx3D);
	return core.basemesh.halfedge_handle(edge_handle, 0);
}

}