#pragma once
#include <Eigen/Core>

struct ConfigRender {
    enum class Mode {
        DEFAULT = 0,
        QUADMESH_ONLY,
        BASEMESH_ONLY,
    } mode;
    struct Color {
        Color();
        Eigen::Vector3f background_bottom;
        Eigen::Vector3f background_top;
        Eigen::Vector3f active;
        Eigen::Vector3f patch;
        float patch_alpha;
    } color;
    bool always_show_quads;
    bool basemesh_render_line;
    bool show_axis;
    bool show_bbox;
    bool use_ortho;
    bool show_expmap;
    float overlay_alpha;
    bool overlay_v_flipped;
    bool auto_camera_center;
    double corner_radius;
    double edge_width_interior;
    double edge_width_boundary;
    double singularity_radius;
    Eigen::Vector3d patch_color_rgb[7];
    double          patch_color_alpha;
    Eigen::Vector3f light_pos;
    bool quadmesh_only_show_auxiliary;
    bool quadmesh_only_show_left_side;
    bool turntable_mode;
    double turntable_speed;
    
    ConfigRender();
};
