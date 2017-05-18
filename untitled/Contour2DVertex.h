#pragma once
#include "Point2D.h"
#include "Contour2DEdge.h"

namespace SketcherCore {

class Contour2DVertex : public Point2D
{
	
public:
	Contour2DVertex() : _idx3D(-1), _weight(-1) {}
	Contour2DVertex(Eigen::Vector2d point, double weight, int edge_index, int idx, int idx3D = -1) : Point2D(point, idx), _edge_idx(edge_index), _idx3D(idx), _weight(weight) {}
	Contour2DVertex(Eigen::Vector3d point, double weight, int edge_index, int idx, int idx3D = -1) : Contour2DVertex(Eigen::Vector2d(point.x(), point.y()), weight, edge_index, idx, idx3D) {}
	virtual ~Contour2DVertex();

	int idx3D()const { return _idx3D; }
	double weight()const { return _weight; }
	void setEdgeIndex(int index) { _edge_idx = index; }
	int edge_index()const { return _edge_idx; }
	

private:
	int _idx3D; //The 3d position of point
	double _weight; // Use to infer the 3D curve position of the point
	int _edge_idx;
};

}