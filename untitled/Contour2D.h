#pragma once

#ifndef CONTOUR2D_H
#define CONTOUR2D_H


#include "Curve2D.h"
#include "Contour2DEdge.h"
#include "Contour2DVertex.h"
#include  "BaseMesh.hh"
#include <map>



namespace SketcherCore {

template <class Point > class Stroke2D;

class Contour2D: public Curve2D <Contour2DVertex, Contour2DEdge>
{
public:
	enum class FaceCondition {
		VISIBLE = 0,
		INVISIBLE,
		BACK
	};

	Contour2D(BaseMesh& bm) : basemesh(bm) {
		_faceCondition = new FaceCondition[basemesh.n_faces()]; generateContour();
	}
	virtual ~Contour2D();

	//void setBasemesh(BaseMesh &bm) { basemesh = bm; generateContour(); }
	void generateContour();
	
	
	std::vector  < Contour2DEdge> &  contourEdgesArray() {return _lines;}
	Contour2DVertex vertex(int vertex_idx)const { return point(vertex_idx); }
	Contour2DEdge edge(int edge_idx)const { return line(edge_idx); }
	std::vector<Eigen::Vector3d> Contour2D::getCorrespondingContour(const Stroke2D<Point2D> & stroke);
	void set_segment_length(double segment_length) { _segmentLength = segment_length; }
	
	virtual void resample_by_length(double segment_length);

protected:
	void generateFaceConditions();
	void Contour2D::addEdge(BaseMesh::EdgeHandle edge_handle, std::map<int, int>& pointsMap, int & edge_start_index);
	//void resample(double segment_length);

private:
	
	FaceCondition* _faceCondition;

	BaseMesh & basemesh;
	double _segmentLength;

	static const double mid_width_divider;
};

}

#endif