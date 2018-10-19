#ifndef REALTIME_RECONSTRUCTION_RECONSTRUCTIONPLUGIN_H
#define REALTIME_RECONSTRUCTION_RECONSTRUCTIONPLUGIN_H

#include <string>
#include <ostream>

#include <imgui/imgui.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/ViewerPlugin.h>
#include <OpenMVS/MVS.h>

#include "reconstruction/RealtimeReconstructionBuilder.h"
#include "nbv/QualityMeasure.h"

class ReconstructionPlugin : public igl::opengl::glfw::ViewerPlugin {
public:
    struct Parameters {
        // Sparse reconstruction
        int next_image_idx = 0;

        // Reconstruct mesh
        // Minimum distance in pixels between the projection of two 3D points to consider them different (0 - disabled)
        float dist_insert = 0.5f;
        // Exploits the free-space support in order to reconstruct weakly-represented surfaces
        bool use_free_space_support = false;
        unsigned int fix_non_manifold = 4;
        // Multiplier adjusting the minimum thickness considered during visibility weighting
        float thickness_factor = 1.0f;
        // Multiplier adjusting the quality weight considered during graph-cut
        float quality_factor = 1.0f;

        // Clean mesh
        // Decimation factor in range (0..1] to be applied to the reconstructed surface (1 - disabled)
        float decimate_mesh = 1.0f;
        // Spurious factor for removing faces with too long edges or isolated components (0 - disabled)
        float remove_spurious = 10.0f;
        // Flag controlling the removal of spike faces
        bool remove_spikes = true;
        // Try to close small holes in the reconstructed surface (0 - disabled)
        unsigned int close_holes = 10;
        // Number of iterations to smooth the reconstructed surface (0 - disabled)
        unsigned int smooth_mesh = 1;

        // Texture mesh
        // How many times to scale down the images before mesh refinement
        unsigned int texture_resolution_level = 0;
        // Do not scale images lower than this resolution
        unsigned int min_resolution = 640;
        // Threshold used to find and remove outlier face textures (0 - disabled)
        float texture_outlier_treshold = 6e-2f;
        // Ratio used to adjust the preference for more compact patches
        // (1 - best quality/worst compactness, 0 - worst quality/best compactness)
        float cost_smoothness_ratio = 0.1f;
        // Generate uniform texture patches using global seam leveling
        bool global_seam_leveling = true;
        // Generate uniform texture patch borders using local seam leveling
        bool local_seam_leveling = true;
        // Texture size should be a multiple of this value (0 - power of two)
        unsigned int texture_size_multiple = 0;
        // Specify the heuristic used when deciding where to place a new patch
        // (0 - best fit, 3 - good speed, 100 - best speed)
        unsigned int patch_packing_heuristic = 1;
        // Color used for faces not covered by any image (r, g, b, a)
        // uint32_t empty_color = 0x00FF7F27;
        float empty_color[3] = {0xFF / (255.0f), 0x7F / (255.0f), 0x27 / 255.0f};

        // Refine mesh
        // How many times to scale down the images before mesh refinement
        unsigned int refine_resolution_level = 0;
        // Do not scale images lower than this resolution
        unsigned int refine_min_resolution = 640;
        // Maximum number of neighbor images used to refine the mesh
        unsigned int refine_max_views = 8;
        // Decimation factor in range [0..1] to be applied to the input surface before refinement (0 - auto, 1 - disabled)
        float refine_decimate = 0.0f;
        // Try to close small holes in the input surface (0 - disabled)
        unsigned int refine_close_holes = 10;
        // Ensure edge size and improve vertex valence of the input surface (0 - disabled, 1 - auto, 2 - force)
        unsigned int ensure_edge_size = 1;
        // Maximum face area projected in any pair of images that is not subdivided (0 - disabled)
        unsigned int max_face_area = 64;
        // How many iterations to run mesh optimization on multi-scale images
        unsigned int scales = 3;
        // Image scale factor used at each mesh optimization step
        float scale_step = 0.5f;
        // Recompute some data in order to reduce memory requirements
        unsigned int reduce_memory = 1;
        // Refine mesh using an image pair alternatively as reference (0 - both, 1 - alternate, 2 - only left, 3 - only right)
        unsigned int alternative_pair = 0;
        // Scalar regularity weight to balance between photo-consistency and regularization terms during mesh optimization
        float regularity_weight = 0.2f;
        // Scalar ratio used to compute the regularity gradient as a combination of rigidity and elasticity
        float rigidity_elasticity_ratio = 0.9f;
        // Threshold used to remove vertices on planar patches (0 - disabled)
        float planar_vertex_ratio = 0.0f;
        // Gradient step to be used instead (0 - auto)
        float gradient_step = 45.05f;

        // Menu
        int point_size = 3;
        int view_to_delete = 0;
        bool show_labels = true;
        bool show_cameras = true;
        bool show_point_cloud = true;
        bool show_mesh = true;
        bool show_texture = true;
        bool show_wireframe = false;
        char filename_buffer[64] = "filename";
        // bool compute_mesh_ppa = true;
        bool auto_reconstruct = true;
        bool auto_compute_ppa = true;
    };

    ReconstructionPlugin(Parameters parameters,
                         std::string images_path,
                         std::string reconstruction_path,
                         std::shared_ptr<std::vector<std::string>> image_names,
                         std::shared_ptr<theia::RealtimeReconstructionBuilder> reconstruction_builder,
                         std::shared_ptr<MVS::Scene> mvs_scene,
                         std::shared_ptr<QualityMeasure> quality_measure);

    void init(igl::opengl::glfw::Viewer *_viewer) override;
    bool post_draw() override;

    // Accessors
    std::shared_ptr<theia::RealtimeReconstructionBuilder> get_reconstruction_builder();
    std::shared_ptr<MVS::Scene> get_mvs_scene_();
    std::shared_ptr<QualityMeasure> get_quality_measure_();

    // Mouse IO
    bool mouse_down(int button, int modifier) override;
    bool mouse_up(int button, int modifier) override;
    bool mouse_move(int mouse_x, int mouse_y) override;
    bool mouse_scroll(float delta_y) override;

    // Keyboard IO
    bool key_pressed(unsigned int key, int modifiers) override;
    bool key_down(int key, int modifiers) override;
    bool key_up(int key, int modifiers) override;

    // Labels
    void draw_labels_window();
    void draw_labels(const igl::opengl::ViewerData &data);
    void draw_text(Eigen::Vector3d pos, Eigen::Vector3d normal, const std::string &text);

private:
    // Viewer data indices
    unsigned int VIEWER_DATA_CAMERAS;
    unsigned int VIEWER_DATA_POINT_CLOUD;
    unsigned int VIEWER_DATA_MESH;

    // Parameters
    Parameters parameters_;

    // Input and output paths
    std::string images_path_;
    std::string reconstruction_path_;
    std::shared_ptr<std::vector<std::string>> image_names_;

    // Reconstruction
    std::shared_ptr<theia::RealtimeReconstructionBuilder> reconstruction_builder_;
    std::shared_ptr<MVS::Scene> mvs_scene_;

    // Quality measure
    std::shared_ptr<QualityMeasure> quality_measure_;

    // Log
    // std::ostringstream log_stream_;
    std::ostream& log_stream_ = std::cout;

    // Callback functions
    void save_scene_callback();
    void load_scene_callback();
    void reload_mesh_callback();

    void initialize_callback();
    void extend_callback();
    void extend_all_callback();

    void remove_view_callback(int view_id);
    void remove_last_view_callback();
    void reset_reconstruction_callback();

    void reconstruct_mesh_callback();
    void refine_mesh_callback();
    void texture_mesh_callback();

    void pixels_per_area_callback();
    void gsd_callback();
    void dor_callback();
    void fa_callback();

    void center_object_callback();

    // Helper functions
    void set_cameras();
    void show_cameras(bool visible);

    void set_point_cloud();
    void show_point_cloud(bool visible);

    void set_mesh();
    void show_mesh(bool visible);
};


#endif //REALTIME_RECONSTRUCTION_RECONSTRUCTIONPLUGIN_H
