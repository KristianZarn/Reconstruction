#include "RenderPlugin.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui_impl_glfw_gl3.h>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>

#include "util/Helpers.h"

RenderPlugin::RenderPlugin(
        std::string images_path,
        std::string reconstruction_path,
        std::string evaluation_folder,
        Render::CameraIntrinsic camera_intrinsics,
        std::shared_ptr<Render> render)
        : images_folder_(std::move(images_path)),
          reconstruction_folder_(std::move(reconstruction_path)),
          evaluation_folder_(std::move(evaluation_folder)),
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


    Eigen::Vector4f s(10.0f, 10.0f, 10.0f, 1.0f);
    render_cameras_gizmo_ = Eigen::Matrix4f::Identity() * Eigen::Scaling(s);
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
        ImGui::InputText("Filename", mesh_name_, 128, ImGuiInputTextFlags_AutoSelectAll);
        if (ImGui::Button("Load mesh (PLY, OBJ)", ImVec2(-1, 0))) {
            load_scene_callback();
        }
        if (ImGui::Checkbox("Show render cameras", &render_cameras_visible_)) {
            show_render_cameras(render_cameras_visible_);
        }
        if (ImGui::Checkbox("Show render mesh", &render_mesh_visible_)) {
            show_render_mesh(render_mesh_visible_);
        }
        if (ImGui::Checkbox("Show texture", &mesh_texture_visible_)) {
            viewer->selected_data_index = VIEWER_DATA_RENDER_MESH;
            viewer->data().show_texture = mesh_texture_visible_;
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
        if (ImGui::Button("Save render cameras pose", ImVec2(-1, 0))) {
            save_render_cameras_gizmo();
        }
        if (ImGui::Button("Load render cameras pose", ImVec2(-1, 0))) {
            load_render_cameras_gizmo();
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
            if (ImGui::Button("Extend all (generated)", ImVec2(-1, 0))) {
                extend_all_generated_callback();
            }
            if (nbv_plugin_) {
                if (ImGui::Button("Extend (NBV)", ImVec2(-1, 0))) {
                    extend_nbv_callback();
                }
            }
            if (nbv_plugin_) {
                ImGui::PushItemWidth(100.0f);
                ImGui::InputInt("##nbvextend", &nbv_extend_count_);
                ImGui::PopItemWidth();
                ImGui::SameLine();
                if (ImGui::Button("Extend many (NBV)", ImVec2(-1, 0))) {
                    extend_many_nbv_callback();
                }
            }
            if (ImGui::Button("Extend (manual)", ImVec2(-1, 0))) {
                extend_manual_callback();
            }

            ImGui::Spacing();

            if (ImGui::Button("Align render mesh", ImVec2(-70, 0))) {
                align_callback();
            }
            ImGui::SameLine();
            ImGui::Checkbox("Auto##align", &auto_align_);

            if (ImGui::Button("Save aligned mesh", ImVec2(-70, 0))) {
                save_aligned_callback();
            }
            ImGui::SameLine();
            ImGui::Checkbox("Auto##savemesh", &auto_save_mesh_);

            if (ImGui::Button("Save mesh quality", ImVec2(-70, 0))) {
                save_quality_callback();
            }
            ImGui::SameLine();
            ImGui::Checkbox("Auto##savequality", &auto_save_quality_);

            if (ImGui::Button("Save render stats", ImVec2(-70, 0))) {
                save_render_stats_callback();
            }
            ImGui::SameLine();
            ImGui::Checkbox("Auto##saverenderstats", &auto_save_render_stats_);
        }
        ImGui::TreePop();
    }

    // Debugging
    if (ImGui::TreeNodeEx("Debug")) {
        if (ImGui::Button("Outout cameras gizmo", ImVec2(-1, 0))) {
            log_stream_ << "render_cameras_gizmo_:\n" << render_cameras_gizmo_ << std::endl;
        }
        if (ImGui::Button("Debug", ImVec2(-1, 0))) {
            log_stream_ << "Render: debug button pressed" << std::endl;
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

    std::string tmp(mesh_name_);
    std::string fullpath = reconstruction_folder_ + tmp;
    mvs_mesh_.Release();
    mvs_mesh_.Load(fullpath);

    render_->Initialize(mvs_mesh_);
    set_render_mesh(mvs_mesh_);
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
    std::string fullname = images_folder_ + filename;

    // Save render
    render_->SaveRender(fullname, camera_intrinsics_, render_data_);

    // Succsessful render
    image_names_->push_back(filename);
    next_image_idx_++;
    log_stream_ << "Render: Image saved to: \n\t" + fullname << std::endl;
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
        if (auto_save_mesh_) {
            save_aligned_callback();
        }
        if (auto_save_quality_) {
            save_quality_callback();
        }
        if (auto_save_render_stats_) {
            save_render_stats_callback();
        }
    }
}

void RenderPlugin::extend_generated_callback() {
    glm::mat4 render_pose = generated_poses_[next_image_idx_];
    render_pose_world_aligned_ = align_transform_ * glm::inverse(render_pose);
    extend_manual_callback();
}

void RenderPlugin::extend_all_generated_callback() {
    while (next_image_idx_ < generated_poses_.size()) {
        extend_generated_callback();
    }
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
        extend_manual_callback(i);
    } else {
        log_stream_ << "Render: NBV Localization failed." << std::endl;
    }
}

void RenderPlugin::extend_many_nbv_callback() {
    if (!reconstruction_plugin_ || !nbv_plugin_) {
        log_stream_ << "Render Error: Reconstruction or NBV plugin not present." << std::endl;
        return;
    }

    for (int i = 0; i < nbv_extend_count_; i++) {

        // Extend NBV
        extend_nbv_callback();

        // Check if all views estimated
        std::shared_ptr<RealtimeReconstructionBuilder> reconstruction_builder =
                reconstruction_plugin_->get_reconstruction_builder();
        if (!reconstruction_builder->AllEstimated()) {
            log_stream_ << "Render: Not all views are estimated" << std::endl;
            break;
        }


        // Check if MSE too big
        glm::mat4 transform = render_stats_.ComputeTransformation();
        double mse = render_stats_.ComputeMSE(transform);
        if (mse > 0.1) {
            log_stream_ << "Render: MSE too big: " << mse << std::endl;
            break;
        }
    }
}

void RenderPlugin::extend_manual_callback(int best_view_pick) {
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

    if (reconstruction_builder->AllEstimated()) {
        theia::ViewId view_id = reconstruction_builder->GetLastAddedViewId();
        Eigen::Matrix4f est_pose_eig = reconstruction_plugin_->get_view_matrix(view_id).cast<float>();
        glm::mat4 estimated_pose = glm::make_mat4(est_pose_eig.data());

        glm::mat4 render_pose = glm::inverse(glm::inverse(align_transform_) * render_pose_world_aligned_);
        render_stats_.AddPose(render_pose, estimated_pose, best_view_pick);

        if (auto_align_) {
            align_callback();
        }
        if (auto_save_mesh_) {
            save_aligned_callback();
        }
        if (auto_save_quality_) {
            save_quality_callback();
        }
        if (auto_save_render_stats_) {
            save_render_stats_callback();
        }
    }
}

void RenderPlugin::align_callback() {
    if (!reconstruction_plugin_) {
        log_stream_ << "Render Error: Reconstruction plugin not present." << std::endl;
        return;
    }

    if (render_stats_.Size() > 0) {
        align_transform_ = render_stats_.ComputeTransformation();
        set_render_mesh(mvs_mesh_);
        set_render_cameras();
        log_stream_ << "Render: Mesh aligned." << std::endl;
    }
}

void RenderPlugin::save_aligned_callback() {
    if (!reconstruction_plugin_) {
        log_stream_ << "Render Error: Reconstruction plugin not present." << std::endl;
        return;
    }

    // Get reconstruction mesh from reconstruction plugin
    std::shared_ptr<MVS::Scene> reconstruction_scene = reconstruction_plugin_->get_mvs_scene_();
    MVS::Mesh reconstruction_mesh = reconstruction_scene->mesh;

    // Align reconstruction mesh to render mesh
    glm::mat4 align_inverse = glm::inverse(align_transform_);

    MVS::Mesh aligned_mesh;
    aligned_mesh.faces = reconstruction_mesh.faces;
    for (const auto& v_mvs : reconstruction_mesh.vertices) {

        // Convert vertex to glm
        glm::vec4 v_glm(v_mvs.x, v_mvs.y, v_mvs.z, 1.0f);

        // Transform vertex
        glm::vec4 v_aligned_glm = align_inverse * v_glm;

        // Convert back to mvs
        aligned_mesh.vertices.AddConstruct(v_aligned_glm[0], v_aligned_glm[1], v_aligned_glm[2]);
    }

    // Prepare filename
    int num_cameras = reconstruction_scene->images.size();
    std::stringstream ss;
    ss << std::setw(3) << std::setfill('0') << std::to_string(num_cameras);
    std::string filename = ss.str() + ".ply";
    std::string fullname = evaluation_folder_ + filename;

    // Save mesh
    aligned_mesh.Save(fullname);
    log_stream_ << "Render: Mesh written to: \n\t" << fullname << std::endl;
}

void RenderPlugin::save_quality_callback() {
    if (!reconstruction_plugin_) {
        log_stream_ << "Render Error: Reconstruction plugin not present." << std::endl;
        return;
    }

    // Update measure internal representation
    auto quality_measure = reconstruction_plugin_->get_quality_measure_();
    quality_measure->updateMesh();

    // Prepare filename number
    std::shared_ptr<MVS::Scene> reconstruction_scene = reconstruction_plugin_->get_mvs_scene_();
    int num_cameras = reconstruction_scene->images.size();
    std::stringstream ss;
    ss << std::setw(3) << std::setfill('0') << std::to_string(num_cameras);
    std::string filename;
    std::string fullname;

    // Save PPA measure
    log_stream_ << "Render: Computing PPA ..." << std::endl;
    std::vector<double> ppa = quality_measure->pixelsPerArea();
    filename = ss.str() + ".ppa";
    fullname = evaluation_folder_ + filename;
    if (writeVectorToFile(fullname, ppa)) {
        log_stream_ << "Render: PPA written to: \n\t" << fullname << std::endl;
    }

    // Save GSD measure
    log_stream_ << "Render: Computing GSD ..." << std::endl;
    std::vector<double> gsd = quality_measure->groundSamplingDistance();
    filename = ss.str() + ".gsd";
    fullname = evaluation_folder_ + filename;
    if (writeVectorToFile(fullname, gsd)) {
        log_stream_ << "Render: GSD written to: \n\t" << fullname << std::endl;
    }

    // Save MPA measure
    log_stream_ << "Render: Computing MPA ..." << std::endl;
    std::vector<double> mpa = quality_measure->meanPixelsPerArea();
    filename = ss.str() + ".mpa";
    fullname = evaluation_folder_ + filename;
    if (writeVectorToFile(fullname, mpa)) {
        log_stream_ << "Render: MPA written to: \n\t" << fullname << std::endl;
    }
}

void RenderPlugin::save_render_stats_callback() {
    std::string filename = images_folder_ + "render_stats.txt";
    render_stats_.WriteStatsToFile(filename);
    log_stream_ << "Render: Render stats saved to: \n\t" << filename << std::endl;
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

void RenderPlugin::set_render_mesh(const MVS::Mesh& mvs_mesh) {
    viewer->selected_data_index = VIEWER_DATA_RENDER_MESH;
    viewer->data().clear();

    // Add vertices
    int num_vertices = mvs_mesh.vertices.size();
    Eigen::MatrixXd V(num_vertices, 3);
    for (int i = 0; i < num_vertices; i++) {
        MVS::Mesh::Vertex vertex = mvs_mesh.vertices[i];
        V(i, 0) = vertex[0];
        V(i, 1) = vertex[1];
        V(i, 2) = vertex[2];
    }

    // Transform vertices by alignment matrix
    Eigen::Matrix4f tmp = Eigen::Map<Eigen::Matrix4f>(glm::value_ptr(align_transform_));
    Eigen::Matrix4d align_mat = tmp.cast<double>();
    Eigen::MatrixXd V_aligned = (V.rowwise().homogeneous() * align_mat.transpose()).rowwise().hnormalized();

    // Add faces
    int num_faces = mvs_mesh.faces.size();
    Eigen::MatrixXi F(num_faces, 3);
    for (int i = 0; i < num_faces; i++) {
        MVS::Mesh::Face face = mvs_mesh.faces[i];
        F(i, 0) = face[0];
        F(i, 1) = face[1];
        F(i, 2) = face[2];
    }
    viewer->data().set_mesh(V_aligned, F);
    viewer->data().show_lines = false;
    viewer->data().set_colors(Eigen::RowVector3d(1, 1, 1));

    // Add texture if available
    if (!mvs_mesh_.faceTexcoords.IsEmpty()) {

        // Set UVs
        int num_texcoords = mvs_mesh_.faceTexcoords.size();
        Eigen::MatrixXd TC(num_texcoords, 2);
        for (int i = 0; i < num_texcoords; i++) {
            MVS::Mesh::TexCoord texcoord = mvs_mesh_.faceTexcoords[i];
            TC(i, 0) = texcoord[0];
            TC(i, 1) = texcoord[1];
        }
        Eigen::MatrixXi FTC(num_faces, 3);
        for (int i = 0; i < num_faces; i++) {
            FTC(i, 0) = i * 3 + 0;
            FTC(i, 1) = i * 3 + 1;
            FTC(i, 2) = i * 3 + 2;
        }
        viewer->data().set_uv(TC, FTC);

        // Set texture
        SEACAVE::Image8U3 img = mvs_mesh_.textureDiffuse;
        int width = img.width();
        int height = img.height();

        Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R(width, height);
        Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> G(width, height);
        Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> B(width, height);

        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                Pixel8U pixel = img.getPixel(j, i);
                R(i, j) = pixel.r;
                G(i, j) = pixel.g;
                B(i, j) = pixel.b;
            }
        }

        viewer->data().set_texture(R.rowwise().reverse(), G.rowwise().reverse(), B.rowwise().reverse());
        viewer->data().show_texture = mesh_texture_visible_;
    }

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
    Eigen::Vector3d gray_color = Eigen::Vector3d(64, 64, 64) / 255.0;
    viewer->data().uniform_colors(gray_color, gray_color, gray_color);
}

void RenderPlugin::show_render_cameras(bool visible) {
    render_cameras_visible_ = visible;
    viewer->selected_data_index = VIEWER_DATA_RENDER_CAMERAS;
    viewer->data().show_faces = visible;
    viewer->data().show_lines = visible;
}

void RenderPlugin::save_render_cameras_gizmo() {
    std::string filename = "render_cameras_gizmo.txt";
    std::string fullname = reconstruction_folder_ + filename;

    // Open file
    std::ofstream outf(fullname);
    if (!outf) {
        log_stream_ << "Render: Could not open file: \n\t" << fullname << std::endl;
        return;
    }

    // Write data to file
    int cols = render_cameras_gizmo_.cols();
    int rows = render_cameras_gizmo_.rows();
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            outf << render_cameras_gizmo_(i, j) << " ";
        }
        outf << "\n";
    }
    log_stream_ << "Render: Render cameras gizmo saved to: \n\t" << fullname << std::endl;
}

void RenderPlugin::load_render_cameras_gizmo() {
    std::string filename = "render_cameras_gizmo.txt";
    std::string fullname = reconstruction_folder_ + filename;

    // Open file
    std::ifstream infile(fullname);
    if (!infile) {
        log_stream_ << "Render: Could not open file: \n\t" << fullname << std::endl;
        return;
    }

    double value;
    int cols = render_cameras_gizmo_.cols();
    int rows = render_cameras_gizmo_.rows();
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            if (infile.eof()) break;
            infile >> value;
            render_cameras_gizmo_(i, j) = value;
        }
    }
    log_stream_ << "Render: Loaded render cameras gizmo from: \n\t" << fullname << std::endl;
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
