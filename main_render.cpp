#include <string>
#include <ostream>
#include <vector>
#include <memory>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui/imgui.h>
#include <imgui_impl_glfw_gl3.h>
#include <igl/readOFF.h>
#include <igl/opengl/glfw/Viewer.h>

#include "reconstruction/Helpers.h"
#include "reconstruction/RealtimeReconstructionBuilder.h"
#include "nbv/QualityMeasure.h"
#include "render/Render.h"
#include "plugins/ReconstructionPlugin.h"
#include "plugins/EditMeshPlugin.h"
#include "plugins/NextBestViewPlugin.h"
#include "plugins/RenderPlugin.h"

int main(int argc, char *argv[]) {

    std::string project_path = "/home/kristian/Documents/reconstruction_code/realtime_reconstruction/dataset/render/";

    std::string image_ext = ".png";
    std::string images_path = project_path + "images/";
    std::string reconstruction_path = project_path + "reconstruction/";
    // std::string calibration_file = project_path + "prior_calibration.txt";
    std::string calibration_file = project_path + "posterior_calibration.txt";

    // Initialize the viewer
    igl::opengl::glfw::Viewer viewer;
    viewer.core.is_animating = true;
    viewer.core.set_rotation_type(igl::opengl::ViewerCore::RotationType::ROTATION_TYPE_TRACKBALL);
    viewer.core.trackball_angle = Eigen::Quaternionf(0, -1, 0, 0);
    viewer.data().point_size = 3;

    // Setup viewer callbacks for ImGui
    viewer.callback_init = [] (igl::opengl::glfw::Viewer& v) -> bool {
        // Setup ImGui
        ImGui::CreateContext();
        ImGui_ImplGlfwGL3_Init(v.window, false);
        ImGui::GetIO().IniFilename = nullptr;
        ImGui::StyleColorsDark();
        ImGuiStyle& style = ImGui::GetStyle();
        style.FrameRounding = 5.0f;
        return false;
    };

    viewer.callback_pre_draw = [] (igl::opengl::glfw::Viewer& v) -> bool {
        glfwPollEvents();
        ImGui_ImplGlfwGL3_NewFrame();
        return false;
    };

    viewer.callback_post_draw = [] (igl::opengl::glfw::Viewer& v) -> bool {
        ImGui::Render();
        return false;
    };

    viewer.callback_shutdown = [] (igl::opengl::glfw::Viewer& v) -> bool {
        // ImGui cleanup
        ImGui_ImplGlfwGL3_Shutdown();
        ImGui::DestroyContext();
        return false;
    };

    // Setup reconstruction objects
    theia::CameraIntrinsicsPrior intrinsics = ReadCalibration(calibration_file);
    RealtimeReconstructionBuilder::Options options = SetRealtimeReconstructionBuilderOptions();
    options.reconstruction_estimator_options.intrinsics_to_optimize = theia::OptimizeIntrinsicsType::NONE;
    options.intrinsics_prior = intrinsics;
    auto reconstruction_builder = std::make_shared<RealtimeReconstructionBuilder>(options);
    auto mvs_scene = std::make_shared<MVS::Scene>(options.num_threads);
    auto quality_measure = std::make_shared<QualityMeasure>(mvs_scene);

    // Attach render plugin
    Render::CameraIntrinsic render_intrinsics{static_cast<unsigned int>(intrinsics.image_width),
                                              static_cast<unsigned int>(intrinsics.image_height),
                                              intrinsics.focal_length.value[0]};
    auto render = std::make_shared<Render>();
    RenderPlugin render_plugin(images_path, reconstruction_path, render_intrinsics, render);
    viewer.plugins.push_back(&render_plugin);

    // Attach reconstruction plugin
    ReconstructionPlugin::Parameters reconstruction_parameters;
    std::shared_ptr<std::vector<std::string>> image_names = render_plugin.get_rendered_image_names();
    ReconstructionPlugin reconstruction_plugin(reconstruction_parameters,
                                               images_path,
                                               reconstruction_path,
                                               image_names,
                                               reconstruction_builder,
                                               mvs_scene,
                                               quality_measure);
    viewer.plugins.push_back(&reconstruction_plugin);

    // Attach next best view plugin
    auto next_best_view = std::make_shared<NextBestView>(mvs_scene);
    NextBestViewPlugin nbv_plugin(next_best_view);
    viewer.plugins.push_back(&nbv_plugin);

    // Attach edit mesh plugin
    // EditMeshPlugin::Parameters edit_mesh_parameters;
    // EditMeshPlugin edit_mesh_plugin(mvs_scene);
    // viewer.plugins.push_back(&edit_mesh_plugin);

    // Start viewer
    viewer.launch();
}
