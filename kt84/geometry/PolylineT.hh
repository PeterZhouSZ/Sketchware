#pragma once

#include <Eigen/Core>
#include <vector>
#include "../adjacent_pairs.hh"

namespace kt84 {

template <int N>
struct PolylineT {
    using Point = Eigen::Matrix<double, N, 1>;
    using iterator               = typename std::vector<Point>::iterator              ;
    using reverse_iterator       = typename std::vector<Point>::reverse_iterator      ;
    using const_iterator         = typename std::vector<Point>::const_iterator        ;
    using const_reverse_iterator = typename std::vector<Point>::const_reverse_iterator;
    using value_type             = typename std::vector<Point>::value_type            ;
    using difference_type        = typename std::vector<Point>::difference_type       ;
    using pointer                = typename std::vector<Point>::pointer               ;
    using reference              = typename std::vector<Point>::reference             ;
    
    std::vector<Point> points;
    bool is_loop;
    
    PolylineT()
        : is_loop()
    {}
    PolylineT(const std::vector<Point>& points_, bool is_loop_)
        : points (points_ )
        , is_loop(is_loop_)
    {}
    PolylineT(int size_, bool is_loop_)
        : points (size_   )
        , is_loop(is_loop_)
    {}
    
    int size() const { return static_cast<int>(points.size()); }
    bool empty() const { return points.empty(); }
    void resize(int size_)                     { points.resize(size_); }
    void resize(int size_, const Point& point) { points.resize(size_, point); }
    void clear() { points.clear(); is_loop = false; }
    void push_back(const Point& point) { points.push_back(point); }
    void pop_back ()                   { points.pop_back (); }
    iterator erase(iterator where_) { return points.erase(where_); }
    iterator erase(iterator first, iterator last) { return points.erase(first, last); }
    iterator insert(iterator where_, const Point& val) { return points.insert(where_, val); }
    void     insert(iterator where_, int count, const Point& val) { points.insert(where_, count, val); }
    template <class InputIterator>
    void     insert(iterator where_, InputIterator first, InputIterator last) { points.insert(where_, first, last); }
    iterator                begin()       { return points. begin(); }
    iterator                end  ()       { return points. end  (); }
    reverse_iterator       rbegin()       { return points.rbegin(); }
    reverse_iterator       rend  ()       { return points.rend  (); }
    const_iterator          begin() const { return points. begin(); }
    const_iterator          end  () const { return points. end  (); }
    const_reverse_iterator rbegin() const { return points.rbegin(); }
    const_reverse_iterator rend  () const { return points.rend  (); }
          Point& front()       { return points.front(); }
    const Point& front() const { return points.front(); }
          Point& back ()       { return points.back (); }
    const Point& back () const { return points.back (); }
          Point& operator[](int index)       { return points[index]; }
    const Point& operator[](int index) const { return points[index]; }
    
    void reverse() {
        if (is_loop) {
            points.push_back(points.front());
            std::reverse(points.begin(), points.end());
            points.pop_back();
        } else {
            std::reverse(points.begin(), points.end());
        }
    }
    
    void offset_front(const Point& target) {
        Point offset = target - front();
        front() = target;
        for (int i = 1; i < size() - 1; ++i) {
            points[i] += offset * (size() - 1 - i) / (size() - 1.0);
        }
    }
    
    void offset_back (const Point& target) {
        Point offset = target - back();
        back() = target;
        for (int i = 1; i < size() - 1; ++i) {
            points[i] += offset * i / (size() - 1.0);
        }
    }
    
    double length() const {
        double result = 0;
        for (auto p : adjacent_pairs(*this, is_loop))
            result += (p.first - p.second).norm();
        
        return result;
    }
    
    void set_loop_by_distance(double distance_threshold) {
        is_loop = (front() - back()).norm() < distance_threshold;
    }
    void resample() { resample(size()); }
    void resample_by_length(double segment_length) { resample(std::max<int>(static_cast<int>(length() / segment_length), 3)); }
    void resample(int target_num_points) {
        int NN = size();
        
        if (NN < 2 || target_num_points < 2)
            return;
        
        if (target_num_points == 2) {
            points[1] = back();
            resize(2);
            return;
        }
        
        PolylineT result(target_num_points, is_loop);
        
        if (is_loop) {
            push_back(front());
            ++NN;
        }
        
        std::vector<double> src_len_acc(NN, 0);
        for (int i = 1; i < NN; ++i)
            src_len_acc[i] = src_len_acc[i - 1] + (points[i] - points[i - 1]).norm();
        
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
                Point& p0 = points[src_i - 1];
                Point& p1 = points[src_i];
                Point p = p0 * w0 + p1 * w1;
                result[index++] = p;
                if (index == target_num_points - 1) break;
            }
            
            if (index == target_num_points - 1) break;
            
            while (src_len_acc[src_i] <= tgt_len_acc + tgt_len_segment) {
                ++src_i;
            }
        }
        result[index++] = back();
        
        if (is_loop)
            result.pop_back();
        
        *this = result;
        return;
    }
    void insert_points_per_segment(int num_inserted_points_per_segment = 3) {
        auto points_temp = points;
        auto insert_pos = points_temp.begin();
        for (auto i : adjacent_pairs(*this, is_loop)) {
            auto d = (i.second - i.first) / (num_inserted_points_per_segment + 1.0);
            Point p = i.first + d;
            ++insert_pos;
            for (int j = 0; j < num_inserted_points_per_segment; ++j, ++insert_pos, p += d)
                insert_pos = points_temp.insert(insert_pos, p);
        }
        points = points_temp;
    }
    void smooth(int num_iter = 1, double weight_first_order = 1.0, double weight_second_order = 0.0, double damping = 0.5) {
        for (int k = 0; k < num_iter; ++k) {
            auto points_old = points;
            for (int i = 0; i < size(); ++i) {
                if (!is_loop && i == 0 || i == size() - 1)
                    continue;
                
                int i_next  = (i + 1) % size();
                int i_prev  = (i + size() - 1) % size();
                int i_next2 = (i + 2) % size();
                int i_prev2 = (i + size() - 2) % size();
                
                auto p_prev  = points_old[i_prev ];
                auto p_next  = points_old[i_next ];
                auto p_prev2 = points_old[i_prev2];
                auto p_next2 = points_old[i_next2];
                
                Point p_first_order  = (p_prev + p_next) / 2.0;
                Point p_second_order = (-p_prev2 + 4.0 * p_prev + 4.0 * p_next - p_next2) / 6.0;
                
                if (!is_loop && i == 1 || i == size() - 2)
                    p_second_order = p_first_order;         // undefined
                
                Point p_target = (weight_first_order * p_first_order + weight_second_order * p_second_order) / (weight_first_order + weight_second_order);
                
                points[i] = damping * points_old[i] + (1 - damping) * p_target;
            }
        }
    }
    Point point_at(double arc_length_parameter) const {
        if (arc_length_parameter < 0 || 1 < arc_length_parameter)
            // arc length parameter should be between 0 and 1
            return Point::Zero();
        
        if (arc_length_parameter == 0) return front();
        if (arc_length_parameter == 1) return is_loop ? front() : back();
        
        const int NN = size();
        const double target_length = length() * arc_length_parameter;
        
        double length_acc = 0;
        for (int i = 0; i < NN; ++i) {
            if (i == NN - 1 && !is_loop)
                break;
            
            auto& p0 = points[i];
            auto& p1 = points[(i + 1) % NN];
            
            double segment_length = (p1 - p0).norm();
            if (length_acc <= target_length && target_length < length_acc + segment_length) {
                double t = (target_length - length_acc) / segment_length;
                return (1 - t) * p0 + t * p1;
            }
            
            length_acc += segment_length;
        }
        
        // something wrong happened
        return Point::Zero();
    }
};

typedef PolylineT<2> Polyline2d;
typedef PolylineT<3> Polyline3d;

}

