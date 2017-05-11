#pragma once

#include <Eigen/Core>
#include "RealParam.hh"
#include "AutoSave.hh"

struct ConfigTemp {
    // real-valued adjustable parameters
    RealParam snapSize;
    RealParam brushSize_moveVertex;
    RealParam quadSize;
    RealParam segmentSize;
    
    static const double default_quadSize_ratio;
    static const double default_segmentSize_ratio;
    
    AutoSave  autoSave;
    int       cylinder_num_div;                 // default number of curves automatically generated when using cylinder sketching
    
    // threshold for loop orientation detection
    double loop_threshold_normal;
    double loop_threshold_area;
    
    ConfigTemp();
};
