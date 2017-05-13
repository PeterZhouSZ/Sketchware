#pragma once
#include "Curve2D.h"
#include "Curve2DEdge.h"
#include "SketchRetopo.hh"

#define MengerCurveThreshold  0.025

namespace SketcherCore {

template <class Point>
class Stroke2D :
	public Curve2D <Point, Curve2DEdge>
{
public:

	//Ctor and dtor
	Stroke2D(std::vector<Point> points) : _points(points) {
		for (int i = 0; i < points.size() - 1; i++) {
			_lines.push_back(Curve2DEdge(std::make_pair<int, int>(i, i+1), points[i].distance(point[i+1])));
		}
		calculate_length();
	}
	virtual ~Stroke2D() {}


	//Length of the stroke
	double length() const { return _length; }
	//Resample
	virtual void resample_by_length(double segment_length);
	void resample(int target_num_points);
	

protected:
	bool check_match(Stroke2D<Point2D> stroke, double segment_length);	//Check match
	bool at_least_one_point_on_the_surface();													//At least one point on the surface
	bool completely_off_the_surface();
	bool tangent_test_2D();																				//Test whether it is a tagent plane curve or not
	bool normal_test_2D();																				//Test whether it is a normal plane curve or not


private:
	void calculate_length();

	double _length;
};







template <class Point>
void Stroke2D<Point>::resample_by_length(double segment_length) {
	resample(std::max<int>(static_cast<int>(length() / segment_length), 3));
}

template <class Point>
void Stroke2D<Point>::calculate_length() {
	double _length = 0;
	for (auto edge : _lines) {
		_length += edge.length();
	}
}

template<class Point>
void Stroke2D<Point>::resample(int target_num_points) {
	int n = _points.size();

	if (n < 2 || target_num_points < 2)
		return;

	/*if (target_num_points == 2) {
	points[1] = back();
	resize(2);
	return;
	}*/

	//polyline_pointnormal result(target_num_points, is_loop);
	std::vector <Point> result;
	std::vector <Curve2DEdge> edges;
	/*if (is_loop) {
	push_back(front());
	++n;
	}*/

	std::vector<double> src_len_acc(n, 0);
	for (int i = 1; i < n; ++i)
		src_len_acc[i] = src_len_acc[i - 1] + _points[i - 1].distance(_points[i]);

	double tgt_len_segment = src_len_acc.back() / (target_num_points - 1.);
	int src_i = 0;
	double tgt_len_acc = 0;
	int index = 0;
	result[index++] = front();
	while (true) {
		while (tgt_len_acc + tgt_len_segment <= src_len_acc[src_i]) {
			tgt_len_acc += tgt_len_segment;
			double w1 = (tgt_len_acc - src_len_acc[src_i - 1]) / (src_len_acc[src_i] - src_len_acc[src_i - 1]);
			double w0 = 1 - w1;
			Point & p0 = _points[src_i - 1];
			Point & p1 = _points[src_i];
			auto p = Point(p0, p1, w0, edges.size());
			result.push_back(p);
			double len = (result[index - 1]).distance(result[index]);
			edges.push_back(Curve2DEdge(std::pair<int, int>(index - 1, index), len));
			index++;
			if (index == target_num_points - 1) break;
		}

		if (index == target_num_points - 1) break;

		while (src_len_acc[src_i] <= tgt_len_acc + tgt_len_segment) {
			++src_i;
		}
	}
	result.push_back(back());
	double len = (result[index - 1]).distance(result[index]);
	edges.push_back(Curve2DEdge(std::pair<int, int>(index - 1, index), len));

	/*if (is_loop)
	result.pop_back();*/

	_lines.clear();
	_points.clear();
	_lines = edges;
	_points = result;
	return;
}

template <class Point>
bool Stroke2D<Point>::check_match(Stroke2D<Point2D> stroke, double segment_length) {
	resample_by_length(segment_length);
	stroke.resample_by_length(segment_length);
	const int len = size() < stroke.size() ? size() : stroke.size();
	const double screen_width = static_cast<double>(SketchRetopo::get_instance().camera->width);

	double distance = 0;
	for (int i = 0; i < len; i++){
		distance += stroke[i].distance(_points[i]);
	}

	//Temporarily  set tolerance as screen_width / 160
	return distance / len > screen_width / 160;
}

template <class Point>
bool Stroke2D<Point>::completely_off_the_surface() {
	for (auto point : _points) {
		if (point.is_on_surface())return false;
	}
	return true;
}

template <class Point>
bool Stroke2D<Point>::at_least_one_point_on_the_surface() {
	return first().is_on_surface() || back().is_on_surface();
}

template <class Point>
bool Stroke2D<Point>::tangent_test_2D() {
	if (size() < 3)return true;
	std::vector<double> edgeLength;
	for (int i = 0; i < size() - 1; i++) {
		auto p = _points[i + 1] - points[i];
		edgeLength.push_back(Eigen::Vector2d(p.x(), p.y()).norm());
	}

	double result = 0;
	for (int i = 1; i < size() - 1; i++) {
		auto x = _points[i].point() - _points[i - 1].point();
		auto y = _points[i + 1].point() - _points[i].point();
		auto z = _points[i + 1].point() - _points[i - 1].point();
		auto x2d = x.head<2>();
		auto y2d = y.head<2>();
		auto z2d = z.head<2>();
		auto sin = std::sqrt(1 - std::pow(x2d.dot(y2d) / x2d.norm() * y2d.norm(), 2));
		result += 2 * sin / z2d;
	}

	return result / (size() - 2) < MengerCurveThreshold;
	return false;
}

template <class Point>
bool Stroke2D<Point>::normal_test_2D() {
	if (size() <= 1) return false;

	auto core = SketchRetopo::get_instance();
	auto face_idx = core.intersect(first().x(), first().y()).id0;
	auto normal = ose(core.basemesh.normal(face_idx));

	//Project the normal to 2D view
	auto view_normal = core.camera->eye_to_center();
	auto projection_of_normal = view_normal.cross(normal).cross(view_normal).normalize();

	//Test
	const double threshold = 0.99;
	for (int i = 0; i < size() - 1; i++) {
		auto cos = (_points[i + 1].point() - _points[i].point()).dot(projection_of_normal) / _points[i].distance(_points[i + 1]);
		if (cos < threshold) return false;
	}

	return true;
}


}