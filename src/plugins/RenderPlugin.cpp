#include "RenderPlugin.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui_impl_glfw_gl3.h>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>

RenderPlugin::RenderPlugin(
        std::string images_path,
        std::string reconstruction_path,
        Render::CameraIntrinsic camera_intrinsics,
        std::shared_ptr<Render> render)
        : images_path_(std::move(images_path)),
          reconstruction_path_(std::move(reconstruction_path)),
          image_names_(std::make_shared<std::vector<std::string>>()),
          camera_intrinsics_(camera_intrinsics),
          render_(std::move(render)) {}

void RenderPlugin::init(igl::opengl::glfw::Viewer* _viewer) {
    ViewerPlugin::init(_viewer);

    // Check for plugins
    for (int i = 0; i < viewer->plugins.size(); i++) {
        // Reconstruction plugin
        if (!reconstruction_plugin_) {
            reconstruction_plugin_ = dynamic_cast<ReconstructionPlugin*>(viewer->plugins[i]);
        }
        // NBV plugin
        if (!nbv_plugin_) {
            nbv_plugin_ = dynamic_cast<NextBestViewPlugin*>(viewer->plugins[i]);
        }
    }

    // Append mesh for camera
    viewer->append_mesh();
    VIEWER_DATA_CAMERA = static_cast<unsigned int>(viewer->data_list.size() - 1);

    // Append mesh for render cameras
    viewer->append_mesh();
    VIEWER_DATA_RENDER_CAMERAS = static_cast<unsigned int>(viewer->data_list.size() - 1);

    // Append mesh for render mesh
    viewer->append_mesh();
    VIEWER_DATA_RENDER_MESH = static_cast<unsigned int>(viewer->data_list.size() - 1);

    // Initial gizmo pose
    camera_gizmo_ = Eigen::Matrix4f::Identity();


    // Eigen::Vector4f s(4.0f, 4.0f, 4.0f, 1.0f);
    // render_cameras_gizmo_ = Eigen::Matrix4f::Identity() * Eigen::Scaling(s);
    render_cameras_gizmo_ <<
         5.56,         0,         0,         0,
            0,  -4.67593,    3.0082, -0.558036,
            0,   -3.0082,  -4.67593,   4.02713,
            0,         0,         0,         1;
}

bool RenderPlugin::pre_draw() {
    ImGuizmo::BeginFrame();
    return false;
}

bool RenderPlugin::post_draw() {
    // Setup window
    float window_width = 350.0f;
    ImGui::SetNextWindowSize(ImVec2(window_width, 0), ImGuiCond_Always);
    ImGui::SetNextWindowPos(ImVec2(700.0f, 0.0f), ImGuiCond_FirstUseEver);
    ImGui::Begin("Render", nullptr, ImGuiWindowFlags_NoSavedSettings);

    // Gizmo setup
    ImGuiIO& io = ImGui::GetIO();
    ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);
    Eigen::Affine3f scale_base_zoom(Eigen::Scaling(1.0f / viewer->core.camera_base_zoom));
    Eigen::Affine3f scale_zoom(Eigen::Scaling(1.0f / viewer->core.camera_zoom));
    Eigen::Matrix4f gizmo_view = scale_base_zoom * scale_zoom * viewer->core.view;


    // Initialization
    if (ImGui::TreeNodeEx("Render scene", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::InputText("Filename", scene_name_, 128, ImGuiInputTextFlags_AutoSelectAll);
        if (ImGui::Button("Load scene (MVS)", ImVec2(-1, 0))) {
            load_scene_callback();
        }
        if (ImGui::Checkbox("Show render mesh", &render_mesh_visible_)) {
            show_render_mesh(render_mesh_visible_);
        }
        if (ImGui::Checkbox("Show render cameras", &render_cameras_visible_)) {
            show_render_cameras(render_cameras_visible_);
        }
        ImGui::TreePop();
    }

    // Generated render poses
    if (ImGui::TreeNodeEx("Generated render poses", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Checkbox("Pose render cameras", &pose_render_cameras_);
        if (pose_render_cameras_) {
            ImGuizmo::Manipulate(gizmo_view.data(),
                                 viewer->core.proj.data(),
                                 gizmo_operation_,
                                 gizmo_mode_,
                                 render_cameras_gizmo_.data());

            ImGui::Text("Transformation options");
            if (ImGui::RadioButton("Translate", gizmo_operation_ == ImGuizmo::TRANSLATE)) {
                gizmo_operation_ = ImGuizmo::TRANSLATE;
            }
            ImGui::SameLine();
            if (ImGui::RadioButton("Rotate", gizmo_operation_ == ImGuizmo::ROTATE)) {
                gizmo_operation_ = ImGuizmo::ROTATE;
            }
            ImGui::SameLine();
            if (ImGui::RadioButton("Scale", gizmo_operation_ == ImGuizmo::SCALE)) {
                gizmo_operation_ = ImGuizmo::SCALE;
            }

            // Update render poses
            update_render_cameras();
        }
        if (ImGui::SliderInt("Camera density", &camera_density_, 10, 40)) {
            update_render_cameras();
        }
        if (ImGui::SliderInt("Pose index", &selected_pose_, 0, generated_poses_.size()-1)) {
            if (selected_pose_ >= 0 && selected_pose_ < generated_poses_.size()) {
                render_pose_world_aligned_ = align_transform_ * glm::inverse(generated_poses_[selected_pose_]);
            }
        }

        if (ImGui::Button("Render all poses", ImVec2(-1, 0))) {
            render_all_poses_callback();
        }

        ImGui::TreePop();
    }

    // Manual camera pose
    if (ImGui::TreeNodeEx("Manual camera pose", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Checkbox("Show render camera", &camera_visible_);
        ImGui::Checkbox("Pose camera", &pose_camera_);
        if (pose_camera_) {
            ImGuizmo::Manipulate(gizmo_view.data(),
                                 viewer->core.proj.data(),
                                 gizmo_operation_,
                                 gizmo_mode_,
                                 camera_gizmo_.data());

            ImGui::Text("Transformation options");
            if (ImGui::RadioButton("Translate", gizmo_operation_ == ImGuizmo::TRANSLATE)) {
                gizmo_operation_ = ImGuizmo::TRANSLATE;
            }
            ImGui::SameLine();
            if (ImGui::RadioButton("Rotate", gizmo_operation_ == ImGuizmo::ROTATE)) {
                gizmo_operation_ = ImGuizmo::ROTATE;
            }
        }

        if (ImGui::Button("Render current pose", ImVec2(-1, 0))) {
            render_callback();
        }
        if (ImGui::Button("Save current render", ImVec2(-1, 0))) {
            save_render_callback();
        }
        ImGui::PushItemWidth(100.0f);
        ImGui::InputInt("Next image index", &next_image_idx_);
        ImGui::PopItemWidth();

        ImGui::TreePop();
    }
    show_camera();

    // Plugin link
    if (ImGui::TreeNodeEx("Plugin link", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (reconstruction_plugin_) {
            if (ImGui::Button("Initialize (generated)", ImVec2(-1, 0))) {
                initialize_generated_callback();
            }
            if (ImGui::Button("Extend (generated)", ImVec2(-1, 0))) {
                extend_generated_callback();
            }
            if (nbv_plugin_) {
                if (ImGui::Button("Extend (NBV)", ImVec2(-1, 0))) {
                    extend_nbv_callback();
                }
            }
            if (ImGui::Button("Extend (manual)", ImVec2(-1, 0))) {
                extend_manual_callback();
            }
            if (ImGui::Button("Align render mesh", ImVec2(-70, 0))) {
                align_callback();
            }
            ImGui::SameLine();
            ImGui::Checkbox("Auto##align", &auto_align_);
        }
        ImGui::TreePop();
    }

    // Debugging
    if (ImGui::TreeNodeEx("Debug")) {
        if (ImGui::Button("Save render stats", ImVec2(-1, 0))) {
            render_stats_.WriteStatsToFile(reconstruction_path_ + "render_stats.txt");
        }
        if (ImGui::Button("Debug", ImVec2(-1, 0))) {
            log_stream_ << "Render: debug button pressed" << std::endl;
            log_stream_ << "Render cameras gizmo: \n" << render_cameras_gizmo_ << std::endl;
        }
        ImGui::TreePop();
    }

    // Set camera transformation
    if (pose_camera_) {
        // Camera gizmo -> render pose
        render_pose_world_aligned_ = glm::make_mat4(camera_gizmo_.data());
    } else {
        // Render pose -> camera gizmo
        camera_gizmo_ = Eigen::Map<Eigen::Matrix4f>(glm::value_ptr(render_pose_world_aligned_));
    }

    ImGui::End();
    return false;
}

std::shared_ptr<std::vector<std::string>> RenderPlugin::get_rendered_image_names() {
    return image_names_;
}

void RenderPlugin::load_scene_callback() {
    log_stream_ << std::endl;

    std::string tmp(scene_name_);
    std::string fullpath = reconstruction_path_ + tmp + ".mvs";
    mvs_scene_.Release();
    mvs_scene_.Load(fullpath);

    render_->Initialize(mvs_scene_);
    set_render_mesh(mvs_scene_);
    show_render_mesh(true);

    update_render_cameras();
    set_render_cameras();
    show_render_cameras(true);

    log_stream_ << "Render: Scene initialized, loaded from: \n\t"
                << fullpath << std::endl;
}

void RenderPlugin::render_callback() {
    glm::mat4 render_pose = glm::inverse(glm::inverse(align_transform_) * render_pose_world_aligned_);
    render_data_ = render_->RenderFromCamera(render_pose, camera_intrinsics_);
}

void RenderPlugin::save_render_callback() {

    // Prepare filename
    std::stringstream ss;
    ss << std::setw(3) << std::setfill('0') << std::to_string(next_image_idx_);
    std::string filename = "frame" + ss.str() + ".png";
    std::string fullname = images_path_ + filename;

    // Save render
    render_->SaveRender(fullname, camera_intrinsics_, render_data_);

    // Succsessful render
    image_names_->push_back(filename);
    next_image_idx_++;
    log_stream_ << "Render: Image saved to: \n\t" + fullname << std::endl;
}

void RenderPlugin::render_all_poses_callback() {
    render_pose_world_aligned_ = align_transform_ * glm::inverse(generated_poses_[selected_pose_]);

    for (const auto& render_pose : generated_poses_) {
        render_pose_world_aligned_ = align_transform_ * glm::inverse(render_pose);
        render_callback();
        save_render_callback();
    }

}

void RenderPlugin::initialize_generated_callback() {
    if (!reconstruction_plugin_) {
        log_stream_ << "Render Error: Reconstruction plugin not present." << std::endl;
        return;
    }

    if (generated_poses_.size() >= 2) {

        glm::mat4 render_pose_0 = generated_poses_[0];
        render_pose_world_aligned_ = align_transform_ * glm::inverse(render_pose_0);
        render_callback();
        save_render_callback();

        glm::mat4 render_pose_1 = generated_poses_[1];
        render_pose_world_aligned_ = align_transform_ * glm::inverse(render_pose_1);
        render_callback();
        save_render_callback();

        reconstruction_plugin_->initialize_callback();

        glm::mat4 render_pose_2 = generated_poses_[2];
        render_pose_world_aligned_ = align_transform_ * glm::inverse(render_pose_2);
        render_callback();
        save_render_callback();

        reconstruction_plugin_->extend_callback();

        // Update render stats
        render_stats_ = RenderStats();
        Eigen::Matrix4f tmp;

        tmp = reconstruction_plugin_->get_view_matrix(0).cast<float>();
        glm::mat4 estimated_pose_0 = glm::make_mat4(tmp.data());

        tmp = reconstruction_plugin_->get_view_matrix(1).cast<float>();
        glm::mat4 estimated_pose_1 = glm::make_mat4(tmp.data());

        tmp = reconstruction_plugin_->get_view_matrix(2).cast<float>();
        glm::mat4 estimated_pose_2 = glm::make_mat4(tmp.data());

        render_stats_.AddPose(render_pose_0, estimated_pose_0, -1);
        render_stats_.AddPose(render_pose_1, estimated_pose_1, -1);
        render_stats_.AddPose(render_pose_2, estimated_pose_2, -1);

        if (auto_align_) {
            align_callback();
        }
    }
}

void RenderPlugin::extend_generated_callback() {
    glm::mat4 render_pose = generated_poses_[next_image_idx_];
    render_pose_world_aligned_ = align_transform_ * glm::inverse(render_pose);
    extend_manual_callback();
}

void RenderPlugin::extend_nbv_callback() {
    if (!reconstruction_plugin_ || !nbv_plugin_) {
        log_stream_ << "Render Error: Reconstruction or NBV plugin not present." << std::endl;
        return;
    }

    // Compute up vector for NBV
    glm::mat4 cameras_transform = glm::make_mat4(render_cameras_gizmo_.data());
    glm::vec4 up = glm::normalize(cameras_transform[1]);
    glm::vec3 up_aligned = align_transform_ * up;

    nbv_plugin_->initialize_callback(up_aligned);
    std::vector<glm::mat4> best_views = nbv_plugin_->get_initial_best_views();
    std::shared_ptr<RealtimeReconstructionBuilder> reconstruction_builder =
            reconstruction_plugin_->get_reconstruction_builder();

    bool success = false;
    int i = 0;
    for (i = 0; i < best_views.size(); i++) {
        // Try to localize suggested view
        render_pose_world_aligned_ = glm::inverse(best_views[i]);
        render_callback();

        // Convert render to theia image
        int width = camera_intrinsics_.image_width;
        int height = camera_intrinsics_.image_height;
        int channels = 3;
        theia::FloatImage image(width, height, channels);
        for (int row = 0; row < height; row++) {
            for (int col = 0; col < width; col++) {
                for (int c = 0; c < channels; c++) {
                    unsigned int val_int = render_data_[c + channels * col  + width * channels * row];
                    float val_float = static_cast<float>(val_int) / std::numeric_limits<unsigned char>::max();
                    image.SetRowCol(height - row - 1, col, c, val_float);
                }
            }
        }

        // Localization
        theia::CalibratedAbsolutePose tmp_pose;
        success = reconstruction_builder->LocalizeImage(image, tmp_pose);
        if (success) break;
    }

    if (success) {
        log_stream_ << "Render: NBV pose set to view number: " << i << std::endl;
        extend_manual_callback();
    } else {
        log_stream_ << "Render: NBV Localization failed." << std::endl;
    }
}

void RenderPlugin::extend_manual_callback() {
    if (!reconstruction_plugin_) {
        log_stream_ << "Render Error: Reconstruction plugin not present." << std::endl;
        return;
    }

    render_callback();
    save_render_callback();
    reconstruction_plugin_->extend_callback();

    // Update render stats
    std::shared_ptr<RealtimeReconstructionBuilder> reconstruction_builder =
            reconstruction_plugin_->get_reconstruction_builder();

    theia::ViewId view_id = reconstruction_builder->GetLastAddedViewId();
    Eigen::Matrix4f est_pose_eig = reconstruction_plugin_->get_view_matrix(view_id).cast<float>();
    glm::mat4 estimated_pose = glm::make_mat4(est_pose_eig.data());

    glm::mat4 render_pose = glm::inverse(glm::inverse(align_transform_) * render_pose_world_aligned_);
    render_stats_.AddPose(render_pose, estimated_pose, -1);

    if (auto_align_) {
        align_callback();
    }
}

void RenderPlugin::align_callback() {
    if (!reconstruction_plugin_) {
        log_stream_ << "Render Error: Reconstruction plugin not present." << std::endl;
        return;
    }

    if (render_stats_.Size() > 0) {
        align_transform_ = render_stats_.ComputeTransformation();
        set_render_mesh(mvs_scene_);
        set_render_cameras();
        log_stream_ << "Render: Mesh aligned." << std::endl;
    }
}

void RenderPlugin::show_camera() {
    viewer->selected_data_index = VIEWER_DATA_CAMERA;
    if (camera_visible_) {

        // Compute camera transformation
        glm::mat4 tmp_glm = render_pose_world_aligned_;
        float model_scale = 1.0f / 2.0f;
        tmp_glm = glm::scale(tmp_glm, glm::vec3(model_scale, model_scale, model_scale));

        Eigen::Matrix4f tmp_eigen;
        tmp_eigen = Eigen::Map<Eigen::Matrix4f>(glm::value_ptr(tmp_glm));
        Eigen::Matrix4d camera_world = tmp_eigen.cast<double>();

        // Vertices
        Eigen::MatrixXd tmp_V(5, 3);
        tmp_V << 0, 0, 0,
                0.75, 0.5, -1,
                -0.75, 0.5, -1,
                -0.75, -0.5, -1,
                0.75, -0.5, -1;

        // Faces
        Eigen::MatrixXi tmp_F(6, 3);
        tmp_F << 0, 1, 2,
                0, 2, 3,
                0, 3, 4,
                0, 4, 1,
                1, 3, 2,
                1, 4, 3;

        // Apply transformation
        tmp_V = (tmp_V.rowwise().homogeneous() * camera_world.transpose()).rowwise().hnormalized();

        // Set viewer data
        viewer->data().clear();
        viewer->data().set_mesh(tmp_V, tmp_F);
        viewer->data().set_face_based(true);
        Eigen::Vector3d color = Eigen::Vector3d(255, 255, 0) / 255.0;
        viewer->data().uniform_colors(color, color, color);
    } else {
        viewer->data().clear();
    }
}

void RenderPlugin::set_render_mesh(const MVS::Scene& mvs_scene) {
    viewer->selected_data_index = VIEWER_DATA_RENDER_MESH;
    viewer->data().clear();

    // Add vertices
    int num_vertices = mvs_scene.mesh.vertices.size();
    Eigen::MatrixXd V(num_vertices, 3);
    for (int i = 0; i < num_vertices; i++) {
        MVS::Mesh::Vertex vertex = mvs_scene.mesh.vertices[i];
        V(i, 0) = vertex[0];
        V(i, 1) = vertex[1];
        V(i, 2) = vertex[2];
    }

    // Transform vertices by alignment matrix
    Eigen::Matrix4f tmp = Eigen::Map<Eigen::Matrix4f>(glm::value_ptr(align_transform_));
    Eigen::Matrix4d align_mat = tmp.cast<double>();
    Eigen::MatrixXd V_aligned = (V.rowwise().homogeneous() * align_mat.transpose()).rowwise().hnormalized();

    // Add faces
    int num_faces = mvs_scene.mesh.faces.size();
    Eigen::MatrixXi F(num_faces, 3);
    for (int i = 0; i < num_faces; i++) {
        MVS::Mesh::Face face = mvs_scene.mesh.faces[i];
        F(i, 0) = face[0];
        F(i, 1) = face[1];
        F(i, 2) = face[2];
    }

    viewer->data().set_mesh(V_aligned, F);
    viewer->data().show_lines = false;
    viewer->data().set_colors(Eigen::RowVector3d(1, 1, 1));
    center_object();
}

void RenderPlugin::show_render_mesh(bool visible) {
    render_mesh_visible_ = visible;
    viewer->selected_data_index = VIEWER_DATA_RENDER_MESH;
    viewer->data().show_faces = visible;
}

void RenderPlugin::center_object() {
    viewer->selected_data_index = VIEWER_DATA_RENDER_MESH;
    Eigen::MatrixXd points;
    points = viewer->data().V;

    if (points.rows() == 0) {
        return;
    }

    Eigen::Vector3d min_point = points.colwise().minCoeff();
    Eigen::Vector3d max_point = points.colwise().maxCoeff();
    Eigen::Vector3d center = points.colwise().mean();
    viewer->core.camera_base_translation = (-center).cast<float>();
    viewer->core.camera_translation.setConstant(0);

    Eigen::Vector3d diff = (max_point - min_point).array().abs();
    viewer->core.camera_base_zoom = static_cast<float>(2.0 / diff.maxCoeff());
    viewer->core.camera_zoom = 1.0;
}

void RenderPlugin::update_render_cameras() {
    glm::mat4 transform = glm::make_mat4(render_cameras_gizmo_.data());
    generated_poses_ = render_->RenderPosesDome(transform, camera_density_);
    set_render_cameras();
}

void RenderPlugin::set_render_cameras() {
    viewer->selected_data_index = VIEWER_DATA_RENDER_CAMERAS;
    viewer->data().clear();

    // Camera in default position
    int num_vertices = 5;
    Eigen::MatrixXd default_V(num_vertices, 3);
    default_V << 0, 0, 0,
            0.75, 0.5, -1,
            -0.75, 0.5, -1,
            -0.75, -0.5, -1,
            0.75, -0.5, -1;

    int num_faces = 6;
    Eigen::MatrixXi default_F(num_faces, 3);
    default_F << 0, 1, 2,
            0, 2, 3,
            0, 3, 4,
            0, 4, 1,
            1, 3, 2,
            1, 4, 3;

    // Add cameras
    int num_views = generated_poses_.size();
    Eigen::MatrixXd cameras_V(num_views * num_vertices, 3);
    Eigen::MatrixXi cameras_F(num_views * num_faces, 3);

    int i = 0;
    for (const auto& cam_view : generated_poses_) {
        // Camera transformation
        Eigen::Affine3f tmp_view(Eigen::Matrix4f::Map(glm::value_ptr(cam_view)));
        Eigen::Affine3d cam_world = tmp_view.inverse().cast<double>();

        // Align
        Eigen::Matrix4f tmp_align(Eigen::Matrix4f::Map(glm::value_ptr(align_transform_)));
        Eigen::Matrix4d align = tmp_align.cast<double>();

        Eigen::Affine3d scale(Eigen::Scaling(1.0 / 2.5));
        Eigen::Matrix4d transform_mat = align * cam_world.matrix() * scale.matrix();

        // Apply transformation
        Eigen::MatrixXd transformed_V = (default_V.rowwise().homogeneous() * transform_mat.transpose()).rowwise().hnormalized();
        Eigen::MatrixXi transformed_F = default_F.array() + i * num_vertices;

        cameras_V.middleRows(i * num_vertices, num_vertices) = transformed_V;
        cameras_F.middleRows(i * num_faces, num_faces) = transformed_F;

        i++;
    }

    // Set viewer data
    viewer->data().set_mesh(cameras_V, cameras_F);
    viewer->data().set_face_based(true);
    Eigen::Vector3d gray_color = Eigen::Vector3d(128, 128, 128) / 255.0;
    viewer->data().uniform_colors(gray_color, gray_color, gray_color);
}

void RenderPlugin::show_render_cameras(bool visible) {
    render_cameras_visible_ = visible;
    viewer->selected_data_index = VIEWER_DATA_RENDER_CAMERAS;
    viewer->data().show_faces = visible;
    viewer->data().show_lines = visible;
}

// Mouse IO
bool RenderPlugin::mouse_down(int button, int modifier) {
    ImGui_ImplGlfwGL3_MouseButtonCallback(viewer->window, button, GLFW_PRESS, modifier);
    return ImGui::GetIO().WantCaptureMouse;
}

bool RenderPlugin::mouse_up(int button, int modifier) {
    return ImGui::GetIO().WantCaptureMouse;
}

bool RenderPlugin::mouse_move(int mouse_x, int mouse_y) {
    return ImGui::GetIO().WantCaptureMouse;
}

bool RenderPlugin::mouse_scroll(float delta_y) {
    ImGui_ImplGlfwGL3_ScrollCallback(viewer->window, 0.f, delta_y);
    return ImGui::GetIO().WantCaptureMouse;
}

// Keyboard IO
bool RenderPlugin::key_pressed(unsigned int key, int modifiers) {
    ImGui_ImplGlfwGL3_CharCallback(nullptr, key);
    return ImGui::GetIO().WantCaptureKeyboard;
}

bool RenderPlugin::key_down(int key, int modifiers) {
    ImGui_ImplGlfwGL3_KeyCallback(viewer->window, key, 0, GLFW_PRESS, modifiers);
    return ImGui::GetIO().WantCaptureKeyboard;
}

bool RenderPlugin::key_up(int key, int modifiers) {
    ImGui_ImplGlfwGL3_KeyCallback(viewer->window, key, 0, GLFW_RELEASE, modifiers);
    return ImGui::GetIO().WantCaptureKeyboard;
}
