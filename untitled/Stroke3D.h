#pragma once


#include "kt84\geometry\Polyline_PointNormal.hh"
#include "Point2D.h"
#include <qdebug.h>

namespace SketcherCore {

template <class Point> class Stroke2D;

class Stroke3D
{
public:
	typedef Eigen::Matrix<double, 6, 1> PointNormal;      // (position, normal)
	typedef Eigen::Matrix<double, 6, 1> Line;                   // (point, direction vector)
	typedef Eigen::Matrix<double, 6, 1> Plane;                 //(point, normal(normalized))
	typedef Eigen::Vector3d Point;
	typedef Eigen::Vector3d DirectionVector;
	typedef std::vector<Point> Face;
	typedef Eigen::Matrix<double, 6, 1> SegmentLine;

	using iterator = std::vector<PointNormal>::iterator;
	using reverse_iterator = std::vector<PointNormal>::reverse_iterator;
	using const_iterator = std::vector<PointNormal>::const_iterator;
	using const_reverse_iterator = std::vector<PointNormal>::const_reverse_iterator;
	using value_type = std::vector<PointNormal>::value_type;
	using difference_type = std::vector<PointNormal>::difference_type;
	using pointer = std::vector<PointNormal>::pointer;
	using reference = std::vector<PointNormal>::reference;

	int size() const { return static_cast<int>(_points.size()); }
	bool empty() const { return _points.empty(); }
	void resize(int size_)                     { _points.resize(size_); }
	void resize(int size_, const PointNormal& point) { _points.resize(size_, point); }
	
	
	iterator                begin()       { return _points.begin(); }
	iterator                end()       { return _points.end(); }
	reverse_iterator       rbegin()       { return _points.rbegin(); }
	reverse_iterator       rend()       { return _points.rend(); }
	const_iterator          begin() const { return _points.begin(); }
	const_iterator          end() const { return _points.end(); }
	const_reverse_iterator rbegin() const { return _points.rbegin(); }
	const_reverse_iterator rend() const { return _points.rend(); }
	PointNormal& front()       { return _points.front(); }
	const PointNormal& front() const { return _points.front(); }
	PointNormal& back()       { return _points.back(); }
	const PointNormal& back() const { return _points.back(); }
	PointNormal& operator[](int index)       { return _points[index]; }
	const PointNormal& operator[](int index) const { return _points[index]; }

	Stroke3D();
	Stroke3D(std::vector<PointNormal> stroke, bool is_loop = false) : _points(kt84::Polyline_PointNormal(stroke, is_loop)) {}
	Stroke3D(const Stroke2D<Point2D> & stroke2D) {};
	virtual ~Stroke3D() { clear(); }

	virtual kt84::Polyline_PointNormal polyline_pn() { return _points; }
	

	 //Calculation methods
	 static Plane min_skew_plane(const Point & point, const DirectionVector & direction);
	 static Point intersect_point(const Line & line, const Plane & plane);
	 static Point nearest_point_line_to_point(const Line & line, const Point & point);

	 static PointNormal nearest_point_line_to_face(const Line & line, const Face & face);
	 static PointNormal nearest_point_line_to_face_when_parellel(const Line & line, Face face, const DirectionVector & face_normal);
	 static PointNormal nearest_point_line_to_face_when_unparellel(const Line & line, const Face & face, const DirectionVector & face_normal);
	 static Point nearest_point_segment_line_to_line(const SegmentLine & segment_line, const Line & line);
	 static Point nearest_point_segment_line_to_point(const SegmentLine & segment_line, const Point & point);

	 static Eigen::Matrix<double, 6, 1>& merge(const Eigen::Vector3d & a, const Eigen::Vector3d & b) { Eigen::Matrix<double, 6, 1> result; result << a, b; return result; }

protected:
	//
	void clear() { _points.clear(); }
	void push_back(const PointNormal& point) { _points.push_back(point); }
	void pop_back()                   { _points.pop_back(); }

	// Changed const_iterator to iterator to meet "defect" in standard
	iterator  erase(iterator where_) { return _points.erase(where_); }
	iterator  erase(iterator first, iterator last) { return _points.erase(first, last); }
	iterator insert(iterator where_, const PointNormal& val) { return _points.insert(where_, val); }
	void     insert(iterator where_, int count, const PointNormal& val) { _points.insert(where_, count, val); }
	template <class InputIterator>
	void     insert(iterator where_, InputIterator first, InputIterator last) { _points.insert(where_, first, last); }

	//Polyline related methods
	virtual kt84::Polyline_PointNormal polyline_pn(const Stroke2D<Point2D> & stroke) = 0;
	void set_polyline_pn(const Stroke2D<Point2D> & stroke) { _points = polyline_pn(stroke); }
	
	//Subclass common methods
	virtual std::vector<PointNormal> get_points(const Stroke2D<Point2D> & stroke, const Plane & min_skew)const;
	boost::optional<PointNormal> get_end_point_of_the_stroke(const Point2D & stroke_endpoint);

private:
	kt84::Polyline_PointNormal _points;
};

}