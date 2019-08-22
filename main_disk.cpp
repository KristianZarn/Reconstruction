#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui/imgui.h>
#include <imgui_impl_glfw_gl3.h>
#include <igl/readOFF.h>
#include <igl/opengl/glfw/Viewer.h>

#include "reconstruction/Helpers.h"
#include "reconstruction/RealtimeReconstructionBuilder.h"
#include "nbv/QualityMeasure.h"
#include "plugins/ReconstructionPlugin.h"
#include "plugins/EditMeshPlugin.h"
#include "plugins/NextBestViewPlugin.h"

int main(int argc, char *argv[]) {

    std::string root_folder = "/home/kristian/Documents/reconstruction_code/realtime_reconstruction/dataset/";

    // Dataset settingvaza
    std::string project_folder = root_folder + "timing/";

    int num_images = 100;
    std::string image_ext = ".jpg";
    std::string images_folder = project_folder + "images/";
    std::string reconstruction_folder = project_folder + "reconstruction/";
    std::string calibration_file = project_folder + "prior_calibration.txt";

    // Initialize the viewer
    igl::opengl::glfw::Viewer viewer;
    viewer.core.is_animating = true;
    viewer.core.set_rotation_type(igl::opengl::ViewerCore::RotationType::ROTATION_TYPE_TRACKBALL);
    viewer.core.trackball_angle = Eigen::Quaternionf(0, -1, 0, 0);
    viewer.data().point_size = 3;

    // For better screenshots
    viewer.core.background_color = Eigen::Vector4f(1.0f, 1.0f, 1.0f, 1.0f);

    // Setup viewer callbacks for ImGui
    viewer.callback_init = [] (igl::opengl::glfw::Viewer& v) -> bool {
        // Setup ImGui
        ImGui::CreateContext();
        ImGui_ImplGlfwGL3_Init(v.window, false);
        ImGui::GetIO().IniFilename = nullptr;
        ImGui::StyleColorsDark();
        ImGuiStyle& style = ImGui::GetStyle();
        style.FrameRounding = 5.0f;

        ImFontConfig font_cfg;
        font_cfg.GlyphExtraSpacing.x = 1.1f;
        // ImGui::GetIO().Fonts->AddFontFromFileTTF("/home/kristian/Documents/reconstruction_code/realtime_reconstruction/resources/OpenSans-ExtraBold.ttf", 14.0f, &font_cfg);
        ImGui::GetIO().Fonts->AddFontFromFileTTF("/home/kristian/Documents/reconstruction_code/realtime_reconstruction/resources/Roboto-Bold.ttf", 16.0f, &font_cfg);
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
    theia::CameraIntrinsicsPrior intrinsics_prior = ReadCalibration(calibration_file);
    RealtimeReconstructionBuilder::Options options = SetRealtimeReconstructionBuilderOptions();
    options.intrinsics_prior = intrinsics_prior;
    auto reconstruction_builder = std::make_shared<RealtimeReconstructionBuilder>(options);
    auto mvs_scene = std::make_shared<MVS::Scene>(options.num_threads);
    auto quality_measure = std::make_shared<QualityMeasure>(mvs_scene);

    // Attach reconstruction plugin
    ReconstructionPlugin::Parameters reconstruction_parameters;
    std::shared_ptr<std::vector<std::string>> image_names = std::make_shared<std::vector<std::string>>();
    for (int i = 0; i <= num_images; i++) {
        std::stringstream ss;
        ss << std::setw(3) << std::setfill('0') << std::to_string(i);
        image_names->emplace_back("frame" + ss.str() + image_ext);
    }

    ReconstructionPlugin reconstruction_plugin(reconstruction_parameters,
                                               images_folder,
                                               reconstruction_folder,
                                               image_names,
                                               reconstruction_builder,
                                               mvs_scene,
                                               quality_measure);
    viewer.plugins.push_back(&reconstruction_plugin);

    // Attach edit mesh plugin
    EditMeshPlugin::Parameters edit_mesh_parameters;
    EditMeshPlugin edit_mesh_plugin(mvs_scene);
    viewer.plugins.push_back(&edit_mesh_plugin);

    // Attach next best view plugin
    auto next_best_view = std::make_shared<NextBestView>(mvs_scene);
    NextBestViewPlugin nbv_plugin(next_best_view);
    viewer.plugins.push_back(&nbv_plugin);

    // Start viewer
    viewer.launch();
}