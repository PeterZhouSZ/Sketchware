#pragma once

#include <vector>

namespace SketcherCore {

template<typename C>
using Iterator = typename C::iterator;

template<typename C>
using ConstIterator = typename C::const_iterator;

template<typename C>
using ReverseIterator = typename C::reverse_iterator;

template<typename C>
using ConstReverseIterator = typename C::const_reverse_iterator;

template <class Point, class Edge>
class Curve2D
{
public:
	using PointType = std::vector < Point > ;
	using EdgeType = std::vector < Edge > ;


	int size() const { return static_cast<int>(_points.size()); }
	bool empty() const { return _points.empty(); }
	void clear() { _points.clear(); }
	//void resize(int size_)                     { _points.resize(size_); }
	//void resize(int size_, const Point& point) { _points.resize(size_, point); }
	//void push_back(const Point& point) { _points.push_back(point); }
	//void pop_back()                   { _points.pop_back(); }

	// Changed const_iterator to iterator to meet "defect" in standard
	//iterator  erase(iterator where_) { return _points.erase(where_); }
	//iterator  erase(iterator first, iterator last) { return _points.erase(first, last); }
	//iterator insert(iterator where_, const Point& val) { return _points.insert(where_, val); }
	//void     insert(iterator where_, int count, const Point& val) { _points.insert(where_, count, val); }
	//template <class InputIterator>
	//void     insert(iterator where_, InputIterator first, InputIterator last) { points.insert(where_, first, last); }
	//iterator                begin()       { return _points.begin(); }
	//iterator                end()       { return _points.end(); }
	//reverse_iterator       rbegin()       { return _points.rbegin(); }
	//reverse_iterator       rend()       { return _points.rend(); }

	ConstIterator<PointType>          begin() const { return _points.begin(); }
	ConstIterator<PointType>          end() const { return _points.end(); }
	ConstReverseIterator<PointType> rbegin() const { return _points.rbegin(); }
	ConstReverseIterator<PointType> rend() const { return _points.rend(); }
	Point& front()       { return _points.front(); }
	const Point& front() const { return _points.front(); }
	Point& back()       { return _points.back(); }
	const Point& back() const { return _points.back(); }
	Point& operator[](int index)       { return _points[index]; }
	const Point& operator[](int index) const { return _points[index]; }

	ConstIterator<EdgeType>            edge_begin() const { return _lines.begin(); }
	ConstIterator<EdgeType>				edge_end() const { return _lines.end(); }
	ConstReverseIterator<EdgeType> edge_rbegin() const { return _lines.rbegin(); }
	ConstReverseIterator<EdgeType> edge_rend() const { return _lines.rend(); }

	Curve2D() {}
	virtual ~Curve2D() {
		_lines.clear();
		_points.clear();
	}

	Point point(int vertex_idx)const { if (vertex_idx >= 0 && vertex_idx < _points.size()) return _points[vertex_idx]; return Point(); }
	Edge line(int edge_idx)const { if (edge_idx >= 0 && edge_idx < _lines.size()) return _lines[edge_idx]; return  Edge(); }

	virtual void resample_by_length(double segment_length) = 0;

protected:
	virtual void generateTopoGraph();


	EdgeType  _lines;
	PointType _points;
	std::vector <std::vector<int>> _topoGraph;
};

template <class Point, class Edge>
void Curve2D<Point, Edge>::generateTopoGraph() {
	_topoGraph.clear();
	_topoGraph = vector<vector<int>>(_points.size(), vector<int>());
	for (auto edge : _lines) {
		_topoGraph[edge.firstPoint()].push_back(edge.secondPoint());
		_topoGraph[edge.secondPoint()].push_back(edge.firstPoint());
	}
}

}