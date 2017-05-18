#pragma once
#include "Curve2DEdge.h"
#include "BaseMesh.hh"
#include "Eigen\src\Core\Matrix.h"



namespace SketcherCore {

class Contour2DEdge: public Curve2DEdge
{
	
public:

	Contour2DEdge() : _idx3D(-1) {} //Null edge
	Contour2DEdge(std::pair<int, int> edgePoints, double length,  int index) : Curve2DEdge(edgePoints, length), _idx3D(index) {}
	virtual ~Contour2DEdge();

	int idx3D()const { return _idx3D; }

	//@description:		These are not points of this edge. The points are the endpoints belonging to its corresponding 3D edge where this edge is lied. 
	//							It's aimed to get calculate the endpoints 3D position, and with 'weight' to get the 3D position of the points of this edge. 
	//暂时觉得没有好放的地方，就放在这个类里实现了
	OpenMesh::VertexHandle firstPoint_3DPositionHandle()const;
	OpenMesh::VertexHandle secondPoint_3DPositionHandle()const;
	Eigen::Vector3d firstPoint_3DPosition()const;
	Eigen::Vector3d secondPoint_3DPosition()const;

private:
	OpenMesh::HalfedgeHandle halfEdge()const;

	int _idx3D;
};

}