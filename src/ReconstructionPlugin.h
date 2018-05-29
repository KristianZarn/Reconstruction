#ifndef REALTIME_RECONSTRUCTION_RECONSTRUCTIONPLUGIN_H
#define REALTIME_RECONSTRUCTION_RECONSTRUCTIONPLUGIN_H

#include <string>
#include <ostream>

#include <imgui/imgui.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/ViewerPlugin.h>
#include <OpenMVS/MVS.h>

#include "RealtimeReconstructionBuilder.h"

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
        unsigned int resolution_level = 0;
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
        unsigned int patch_packing_heuristic = 3;
        // Color used for faces not covered by any image
        uint32_t empty_color = 0x00FF7F27;


        // Viewer
        int point_size = 3;
        int view_to_delete = 0;
    };

    ReconstructionPlugin(Parameters parameters,
                         std::string images_path,
                         std::vector<std::string> image_names,
                         std::string reconstruction_path,
                         theia::RealtimeReconstructionBuilder::Options options,
                         theia::CameraIntrinsicsPrior intrinsics_prior);

    void init(igl::opengl::glfw::Viewer *_viewer) override;
    bool post_draw() override;

    // Mouse IO
    bool mouse_down(int button, int modifier) override;
    bool mouse_up(int button, int modifier) override;
    bool mouse_move(int mouse_x, int mouse_y) override;
    bool mouse_scroll(float delta_y) override;

    // Keyboard IO
    bool key_pressed(unsigned int key, int modifiers) override;
    bool key_down(int key, int modifiers) override;
    bool key_up(int key, int modifiers) override;

private:
    // Parameters
    Parameters parameters_;

    // Input and output paths
    std::string images_path_;
    std::vector<std::string> image_names_;
    std::string reconstruction_path_;

    // Reconstruction
    theia::RealtimeReconstructionBuilder reconstruction_builder_;
    MVS::Scene mvs_scene_;

    // Log
    // std::ostringstream log_stream_;
    std::ostream& log_stream_ = std::cout;

    // Helper functions
    void show_point_cloud();
    void show_mesh();
    void reset_reconstruction();

    // Callback functions
    void initialize_callback();
    void extend_callback();
    void reconstruct_mesh_callback();
    void dense_reconstruct_mesh_callback();
    void texture_mesh_callback();
};


#endif //REALTIME_RECONSTRUCTION_RECONSTRUCTIONPLUGIN_H
