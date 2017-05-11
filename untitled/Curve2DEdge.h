#pragma once
#include <utility>

namespace SketcherCore {

class Curve2DEdge
{
public:
	Curve2DEdge() {}
	Curve2DEdge(std::pair<int, int> edgePoints, double length) :_edgePoints(edgePoints), _length(length) {}
	virtual ~Curve2DEdge();

	double length()const { return _length; }
	int firstPoint()const { return _edgePoints.first; }
	int secondPoint()const { return _edgePoints.second; }
	int nextPoint(int idx)const { if (idx == firstPoint()) return secondPoint(); else if (idx == secondPoint()) return firstPoint(); return -1; }
	const std::pair<int, int>& endPoints()const { return _edgePoints; }

protected:
	std::pair<int, int> _edgePoints;
	
private:
	double _length;
};

 }