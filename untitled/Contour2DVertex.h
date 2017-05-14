#pragma once
#include "Point2D.h"
#include "Contour2DEdge.h"

namespace SketcherCore {

class Contour2DVertex : public Point2D
{
	
public:
	Contour2DVertex() : _idx(-1), _weight(-1) {}
	Contour2DVertex(Eigen::Vector2d point, double weight, int edge_index, int idx = -1) : Point2D(point, edge_index), _idx(idx), _weight(weight) {}
	Contour2DVertex(Eigen::Vector3d point, double weight, int edge_index, int idx = -1) : Point2D(point, edge_index), _idx(idx), _weight(weight) {}
	virtual ~Contour2DVertex();

	int idx3D()const { return _idx; }
	double weight()const { return _weight; }
	void setEdgeIndex(int index) { _edge_idx = index; }
	

private:
	int _idx;
	double _weight; // Use to infer the 3D curve position of the point
};

}