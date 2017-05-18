#pragma once
#include <kt84/graphics/graphics_util.hh>
#include <Eigen\core>

namespace SketcherCore {

class Point2D
{
public:
	typedef Eigen::Vector2d Point;

	Point2D() { _point = Point(NAN, NAN);  _idx = -1; }
	Point2D(double  x, double y, int idx) : Point2D(Eigen::Vector2d(x, y), idx) {}
	Point2D(Point point, int idx) : _point(point), _idx(idx) {}
	Point2D(Eigen::Vector3d point, int idx) : Point2D(Eigen::Vector2d(point.x(), point.y()), idx) {}
	Point2D(const Point2D firstPoint, const Point2D secondPoint, double weight, int idx) : Point2D((firstPoint.point() * weight + secondPoint.point() * (1 - weight)).eval(), idx) {}
	virtual ~Point2D();
	
	//Getter and setter
	Point point()const { return _point; }
	Point position()const { return point(); }
	double x()const {return _point.x(); }
	double y()const {return _point.y();}
	int idx()const { return _idx; }
	

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
	int _idx;
	
};

}