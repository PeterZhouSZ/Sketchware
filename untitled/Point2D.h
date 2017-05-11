#pragma once
#include <kt84/graphics/graphics_util.hh>
#include <Eigen\core>

namespace SketcherCore {

class Point2D
{
public:
	typedef Eigen::Vector3d Point;

	Point2D() { _point = Point(0); }
	Point2D(Point point, int edge_index) : _point(point), _edge_idx(edge_index) {}
	Point2D(Point2D firstPoint, Point2D secondPoint, double weight, int edge_index) : Point2D(firstPoint.point() * weight + secondPoint.point() * (1 - weight), edge_index) {}
	virtual ~Point2D();
	
	//Getter and setter
	Point point()const { return _point; }
	double x()const {return _point.x(); }
	double y()const {return _point.y();}
	int edge_index()const { return _edge_idx; }

	//Distance
	double distance(const Eigen::Vector2d & p)const{ return std::sqrt(std::pow<double>(2, p.x() - _point.x()) + std::pow<double>(2, p.y() - _point.y())); }
	double distance(const Eigen::Vector3d & p)const{ Eigen::Vector2d v(p.x(), p.y()); return distance(v); }
	double distance(const Point2D & p)const { return distance(p.point()); }

	//Hit
	bool is_on_surface();

	//Operator
	//Point2D operator+(const Point2D& b)const { return Point2D(_point + b.point()); }
	//Point2D operator-(const Point2D& b)const { return Point2D(_point - b.point()); }

protected:
	Point _point;
	int _edge_idx;
};

}