#include "NextBestViewPlugin.h"

#include <math.h>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui_impl_glfw_gl3.h>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>
#include <igl/colormap.h>
#include <igl/jet.h>
#include <igl/unproject_onto_mesh.h>

#include "util/Helpers.h"
#include "nbv/HelpersOptim.h"
#include "nelder_mead/nelder_mead.h"

NextBestViewPlugin::NextBestViewPlugin(std::shared_ptr<NextBestView> nbv)
        : next_best_view_(std::move(nbv)) {}

void NextBestViewPlugin::init(igl::opengl::glfw::Viewer *_viewer) {
    ViewerPlugin::init(_viewer);

    // Append mesh for camera
    viewer->append_mesh();
    VIEWER_DATA_CAMERA = static_cast<unsigned int>(viewer->data_list.size() - 1);

    // Append mesh for mesh
    viewer->append_mesh();
    VIEWER_DATA_NBV_MESH = static_cast<unsigned int>(viewer->data_list.size() - 1);

    // Append mesh for bounding box
    viewer->append_mesh();
    VIEWER_DATA_BOUNDING_BOX = static_cast<unsigned int>(viewer->data_list.size() - 1);

    // Initial gizmo pose
    camera_gizmo_ = Eigen::Matrix4f::Identity();
    bounding_box_gizmo_ = Eigen::Matrix4f::Identity();
}

bool NextBestViewPlugin::pre_draw() {
    ImGuizmo::BeginFrame();
    return false;
}

bool NextBestViewPlugin::post_draw() {
    // Setup window
    float window_width = 350.0f;
    ImGui::SetNextWindowSize(ImVec2(window_width, 0), ImGuiCond_Always);
    ImGui::SetNextWindowPos(ImVec2(350.0f, 0.0f), ImGuiCond_FirstUseEver);
    ImGui::Begin("Nacrtovanje naslednjega pogleda", nullptr, ImGuiWindowFlags_NoSavedSettings);

    // Gizmo setup
    ImGuiIO& io = ImGui::GetIO();
    ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);
    Eigen::Affine3f scale_base_zoom(Eigen::Scaling(1.0f / viewer->core.camera_base_zoom));
    Eigen::Affine3f scale_zoom(Eigen::Scaling(1.0f / viewer->core.camera_zoom));
    Eigen::Matrix4f gizmo_view = scale_base_zoom * scale_zoom * viewer->core.view;

    // NBV object and pose optimization
    if (ImGui::TreeNodeEx("Inicializacija", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::Button("Inicializiraj NBV [n]", ImVec2(-1, 0))) {
            initialize_callback();
        }
        if (ImGui::Checkbox("Prikazi NBV model", &nbv_mesh_visible_)) {
            show_nbv_mesh(nbv_mesh_visible_);
        }
        ImGui::Checkbox("Vidna NBV kamera", &camera_visible_);
        ImGui::TreePop();
    }
    show_nbv_camera();

    // Region of interest
    if (ImGui::TreeNodeEx("Obmocje interesa", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Checkbox("Viden kvader", &bounding_box_visible_);
        ImGui::Checkbox("Postavi kvader", &pose_bounding_box_);

        if (pose_bounding_box_) {
            // Show gizmo
            ImGuizmo::Manipulate(gizmo_view.data(),
                                 viewer->core.proj.data(),
                                 gizmo_operation_,
                                 gizmo_mode_,
                                 bounding_box_gizmo_.data());

            ImGui::Text("Moznosti kvadra");
            if (ImGui::RadioButton("Translacija", gizmo_operation_ == ImGuizmo::TRANSLATE)) {
                gizmo_operation_ = ImGuizmo::TRANSLATE;
            }
            ImGui::SameLine();
            if (ImGui::RadioButton("Rotacija", gizmo_operation_ == ImGuizmo::ROTATE)) {
                gizmo_operation_ = ImGuizmo::ROTATE;
            }
            ImGui::SameLine();
            if (ImGui::RadioButton("Skaliranje", gizmo_operation_ == ImGuizmo::SCALE)) {
                gizmo_operation_ = ImGuizmo::SCALE;
            }
        }

        if (ImGui::Button("Uporabi izbiro", ImVec2(-70, 0))) {
            apply_selection_callback();
        }
        ImGui::SameLine();
        ImGui::Checkbox("Auto##applyselection", &auto_apply_selection_);

        std::ostringstream os;
        os << "Odstotek ciljne kvalitete: "
           << "\t" << target_quality_percentage_ * 100.0 << " %";
        ImGui::TextUnformatted(os.str().c_str());
        ImGui::TreePop();
    }
    show_bounding_box();

    // Best view init
    if (ImGui::TreeNodeEx("Izbira naslednjega pogleda", ImGuiTreeNodeFlags_DefaultOpen)) {

        // PPA
        if (ImGui::Button("Prikazi PPA", ImVec2(-1, 0))) {
            set_nbv_mesh_color(pixels_per_area_);
        }
        ImGui::Spacing();

        // Clustering
        if (ImGui::Button("Prikazi gruce", ImVec2(-70, 0))) {
            show_clusters_callback();
        }
        ImGui::SameLine();
        ImGui::Checkbox("Auto##showclusters", &auto_show_clusters_);

        ImGui::InputFloat("q_t", &next_best_view_->target_quality_);
        ImGui::InputFloat("q_p", &next_best_view_->percentage_increment_);
        ImGui::InputFloat("q_i", &next_best_view_->target_quality_increment_);
        ImGui::Spacing();

        ImGui::InputInt("c_min", &next_best_view_->cluster_min_size_);
        ImGui::InputInt("c_max", &next_best_view_->cluster_max_size_);
        ImGui::InputFloat("phi", &next_best_view_->cluster_angle_);
        ImGui::Spacing();

        // Best view
        ImGui::InputFloat("alpha", &next_best_view_->init_alpha_);
        ImGui::InputFloat("beta", &next_best_view_->init_beta_);
        ImGui::InputFloat("gamma", &next_best_view_->dist_alpha_);


        if (ImGui::SliderInt("Izbrani pogled", &selected_view_, 0, best_views_init_.size()-1)) {
            set_nbv_camera_callback(selected_view_);
        }
        ImGui::TreePop();
    }

    // Optimize camera pose
    /*if (ImGui::TreeNodeEx("Camera pose optimization")) {
        if (ImGui::Button("Optimize [t]", ImVec2(-1, 0))) {
            optimize_callback();
        }
        ImGui::InputFloat("Faces thresh", &next_best_view_->visible_faces_tresh_);
        ImGui::InputFloat("Dist thresh", &next_best_view_->distance_tresh_);
        ImGui::InputFloat("Optim Alpha", &next_best_view_->optim_alpha_);
        ImGui::InputFloat("Optim Beta", &next_best_view_->optim_beta_);
        ImGui::Checkbox("Pose camera", &pose_camera_);
        ImGui::InputFloat3("Position", glm::value_ptr(camera_pos_));
        ImGui::InputFloat3("Angles", glm::value_ptr(camera_rot_));

        if (pose_camera_) {
            // Show gizmo
            ImGuizmo::Manipulate(gizmo_view.data(),
                                 viewer->core.proj.data(),
                                 gizmo_operation_,
                                 gizmo_mode_,
                                 camera_gizmo_.data());

            ImGui::Text("Camera options");
            if (ImGui::RadioButton("Translate", gizmo_operation_ == ImGuizmo::TRANSLATE)) {
                gizmo_operation_ = ImGuizmo::TRANSLATE;
            }
            ImGui::SameLine();
            if (ImGui::RadioButton("Rotate", gizmo_operation_ == ImGuizmo::ROTATE)) {
                gizmo_operation_ = ImGuizmo::ROTATE;
            }
        }
        ImGui::TreePop();
    }*/

    // Debugging
    if (ImGui::TreeNodeEx("Debug")) {

        // Other
        if (ImGui::Button("Debug [d]", ImVec2(-1, 0))) {
            debug_callback();
        }
        ImGui::TreePop();
    }

    // Set camera transformation
    if (pose_camera_) {
        // Camera gizmo -> position and rotation
        glm::vec3 scale;
        ImGuizmo::DecomposeMatrixToComponents(
                camera_gizmo_.data(),
                glm::value_ptr(camera_pos_),
                glm::value_ptr(camera_rot_),
                glm::value_ptr(scale));
    } else {
        // Position and rotation -> camera gizmo
        glm::vec3 scale = glm::vec3(1.0, 1.0, 1.0);
        ImGuizmo::RecomposeMatrixFromComponents(
                glm::value_ptr(camera_pos_),
                glm::value_ptr(camera_rot_),
                glm::value_ptr(scale),
                camera_gizmo_.data());
    }

    ImGui::End();
    return false;
}

glm::vec3 NextBestViewPlugin::get_camera_pos() const {
    return camera_pos_;
}

glm::vec3 NextBestViewPlugin::get_camera_rot() const {
    return camera_rot_;
}

std::vector<glm::mat4> NextBestViewPlugin::get_initial_best_views() const {
    return best_views_init_;
}

int NextBestViewPlugin::get_selected_view() const {
    return selected_view_;
}

Eigen::Matrix4f NextBestViewPlugin::get_bounding_box_gizmo() const {
    return bounding_box_gizmo_;
}

void NextBestViewPlugin::initialize_callback(const glm::vec3& up) {
    // Initialize NBV object
    next_best_view_->Initialize();

    // Apply selection
    if (auto_apply_selection_) {
        apply_selection_callback();
    }

    // Compute NBV and other variables
    log_stream_ << std::endl;
    log_stream_ << "NBV: Computing PPA ... " << std::flush;
    pixels_per_area_ = next_best_view_->ppa_;
    target_quality_percentage_ = next_best_view_->TargetPercentage(pixels_per_area_);
    log_stream_ << "DONE" << std::endl;

    log_stream_ << "NBV: Computing clusters ... " << std::flush;
    clusters_ = next_best_view_->FaceClusters(pixels_per_area_);
    log_stream_ << "DONE" << std::endl;

    log_stream_ << "NBV: Computing initial views ... " << std::flush;
    best_views_init_ = next_best_view_->BestViewInit(clusters_, up);
    log_stream_ << "DONE" << std::endl;

    // Set cluster_id for debugging
    cluster_id_.resize(next_best_view_->mvs_scene_->mesh.faces.size());
    std::fill(cluster_id_.begin(), cluster_id_.end(), -1);
    for (int i = 0; i < clusters_.size(); i++) {
        for (const auto& face_id : clusters_[i].first) {
            cluster_id_[face_id] = i;
        }
    }

    // Set NBV mesh
    set_nbv_mesh();

    // Show clusters
    if (auto_show_clusters_) {
        show_clusters_callback();
    }

    // Set NBV camera
    set_nbv_camera_callback(selected_view_);
}

void NextBestViewPlugin::apply_selection_callback() {
    log_stream_ << std::endl;

    // Get bounding box vectors
    viewer->selected_data_index = VIEWER_DATA_BOUNDING_BOX;
    const Eigen::MatrixXd& bbox_V = viewer->data().V;

    if(!(bbox_V.rows() == 8 && bbox_V.cols() == 3)) {
        log_stream_ << "NBV selection: Bounding box is not set." << std::endl;
    } else {
        Eigen::Vector3d u = bbox_V.row(4) - bbox_V.row(0);
        Eigen::Vector3d v = bbox_V.row(2) - bbox_V.row(0);
        Eigen::Vector3d w = bbox_V.row(1) - bbox_V.row(0);

        // Add faces inside bounding box to valid faces
        std::unordered_set<unsigned int> valid_faces;
        for (int i = 0; i < next_best_view_->mvs_scene_->mesh.faces.size(); i++) {
            auto mvs_face = next_best_view_->mvs_scene_->mesh.faces[i];

            bool valid = true;
            for (int j = 0; j < 3; j++) {
                auto mvs_vert = next_best_view_->mvs_scene_->mesh.vertices[mvs_face[j]];
                Eigen::Vector3d x;
                x << mvs_vert[0], mvs_vert[1], mvs_vert[2];

                double ux = u.dot(x);
                double vx = v.dot(x);
                double wx = w.dot(x);

                valid = valid && (ux > u.dot(bbox_V.row(0)) && ux < u.dot(bbox_V.row(4)) &&
                                  vx > v.dot(bbox_V.row(0)) && vx < v.dot(bbox_V.row(2)) &&
                                  wx > w.dot(bbox_V.row(0)) && wx < w.dot(bbox_V.row(1)));
            }

            if (valid) {
                valid_faces.insert(static_cast<unsigned int>(i));
            }
        }

        // Set valid faces in NBV
        if (valid_faces.empty()) {
            log_stream_ << "NBV selection: No faces selected - valid faces not set." << std::endl;
        } else {
            next_best_view_->SetValidFaces(valid_faces);
            log_stream_ << "NBV selection: Number of valid faces: " << valid_faces.size() << std::endl;
        }
    }
}

void NextBestViewPlugin::show_clusters_callback() {

    int num_clusters = clusters_.size();
    int num_faces = next_best_view_->mvs_scene_->mesh.faces.size();

    Eigen::VectorXd tmp = Eigen::VectorXd::Zero(num_faces);
    for (const auto& cluster_cost : clusters_) {
        for (const auto& face_id : cluster_cost.first) {
            tmp(face_id) = cluster_cost.second;
        }
    }

    Eigen::MatrixXd color;
    igl::colormap(igl::ColorMapType::COLOR_MAP_TYPE_JET, tmp, true, color);

    for (int i = 0; i < num_faces; i++) {
        if (cluster_id_[i] < 0) {
            color.row(i) = Eigen::RowVector3d(1.0, 1.0, 1.0);
        }
    }

    // Set default color
    // Eigen::MatrixXd color(num_faces, 3);
    // Eigen::RowVector3d default_color = Eigen::RowVector3d(1.0, 1.0, 1.0);
    // for (int i = 0; i < num_faces; i++) {
    //     color.row(i) = default_color;
    // }

    // Set cluster colors
    // Eigen::MatrixXd color_palette = (Eigen::MatrixXd::Random(num_clusters, 3).array() + 1.0) / 2.0;
    // for (int i = 0; i < clusters_.size(); i++) {
    //     for (const auto& face_id : clusters_[i].first) {
    //         color.row(face_id) = color_palette.row(i);
    //     }
    // }

    // Show colors
    viewer->selected_data_index = VIEWER_DATA_NBV_MESH;
    viewer->data().set_colors(color);
}

void NextBestViewPlugin::set_nbv_camera_callback(int selected_view) {
    if (selected_view >= 0 && selected_view < best_views_init_.size()) {
        glm::mat4 view = best_views_init_[selected_view];
        glm::mat4 view_world = glm::inverse(view);

        glm::vec3 scale;
        ImGuizmo::DecomposeMatrixToComponents(
                glm::value_ptr(view_world),
                glm::value_ptr(camera_pos_),
                glm::value_ptr(camera_rot_),
                glm::value_ptr(scale));

        camera_visible_ = true;
    }
}

void NextBestViewPlugin::optimize_callback() {

    // Initial point
    int n = 6;
    point_t start;
    start.x = (double*) malloc(n * sizeof(double));
    start.x[0] = camera_pos_[0];
    start.x[1] = camera_pos_[1];
    start.x[2] = camera_pos_[2];
    start.x[3] = camera_pos_[0];
    start.x[4] = camera_rot_[1];
    start.x[5] = camera_rot_[2];

    // Optimisation settings
    optimset_t optim_set;
    optim_set.tolx = 0.01;
    optim_set.tolf = 0.1;
    optim_set.max_iter = 50;
    optim_set.max_eval = 1000;
    optim_set.verbose = 1;

    // Cost function parameters
    auto cluster = next_best_view_->ClusterCenterNormal(clusters_[selected_view_]);
    unsigned int image_width = next_best_view_->mvs_scene_->images.front().width;
    unsigned int image_height = next_best_view_->mvs_scene_->images.front().height;
    double focal_y = next_best_view_->mvs_scene_->images.front().camera.K(1, 1);
    OptimData optim_data{next_best_view_.get(), image_width, image_height, focal_y, cluster.first, cluster.second};

    // Run optimization
    auto time_begin = std::chrono::steady_clock::now();

    point_t solution;
    nelder_mead(n, &start, &solution, &optim_function, &optim_data, &optim_set);
    camera_pos_[0] = static_cast<float>(solution.x[0]);
    camera_pos_[1] = static_cast<float>(solution.x[1]);
    camera_pos_[2] = static_cast<float>(solution.x[2]);
    camera_rot_[0] = static_cast<float>(solution.x[3]);
    camera_rot_[1] = static_cast<float>(solution.x[4]);
    camera_rot_[2] = static_cast<float>(solution.x[5]);

    auto time_end = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_elapsed = time_end - time_begin;
    log_stream_ << "Elapsed time: " << time_elapsed.count() << " s" << std::endl;

    // Debug
    // log_stream_ << "Parameters: \n" << param;
}

void NextBestViewPlugin::debug_callback() {
    log_stream_ << "NBV: Debug button pressed" << std::endl;

    unsigned int image_width = next_best_view_->mvs_scene_->images.front().width;
    unsigned int image_height = next_best_view_->mvs_scene_->images.front().height;
    double focal_y = next_best_view_->mvs_scene_->images.front().camera.K(1, 1);

    glm::mat4 view_matrix;
    glm::vec3 scale = glm::vec3(1.0, 1.0, 1.0);
    ImGuizmo::RecomposeMatrixFromComponents(
            glm::value_ptr(camera_pos_),
            glm::value_ptr(camera_rot_),
            glm::value_ptr(scale),
            glm::value_ptr(view_matrix));
    view_matrix = glm::inverse(view_matrix);

    // auto cluster = next_best_view_->ClusterCenterNormal(clusters_[selected_view_]);
    // double cost = next_best_view_->CostFunction(
    //         view_matrix, image_height, focal_y, image_width, cluster.first, cluster.second);
    // log_stream_ << "Cost: " << cost << std::endl;
    double cost = next_best_view_->CostFunctionInit(view_matrix, image_height, focal_y, image_width);
    log_stream_ << "Cost: " << cost << std::endl;

    // auto render_data = next_best_view_->RenderFaceIdFromCamera(view_matrix, image_width, image_height, focal_y);
    // writeBufferToFile("/home/kristian/Documents/reconstruction_code/realtime_reconstruction/resources/render.dat",
    //         image_width, image_height, render_data);
}

void NextBestViewPlugin::pick_face_callback() {
    viewer->selected_data_index = VIEWER_DATA_NBV_MESH;

    int face_id;
    Eigen::Vector3f barycentric;
    double x = viewer->current_mouse_x;
    double y = viewer->core.viewport(3) - viewer->current_mouse_y;

    // Cast a ray
    bool hit = igl::unproject_onto_mesh(
            Eigen::Vector2f(x, y),
            viewer->core.view,
            viewer->core.proj,
            viewer->core.viewport,
            viewer->data().V,
            viewer->data().F,
            face_id,
            barycentric);

    if (hit) {
        // double fa = face_area_[face_id];
        double ppa = pixels_per_area_[face_id];
        int cid = cluster_id_[face_id];

        // Get cluster quality
        double cost = -1;
        if (cid >= 0) {
            cost = clusters_[cid].second;
        }

        log_stream_ << "FID: " << face_id
                    << "\tPPA: " << ppa
                    << "\tCCost: " << cost << std::endl;
    }
}

void NextBestViewPlugin::show_nbv_camera() {
    viewer->selected_data_index = VIEWER_DATA_CAMERA;
    if (camera_visible_) {

        // Compute camera transformation
        Eigen::Matrix4f tmp;
        glm::vec3 scale = glm::vec3(1.0 / 2.0, 1.0 / 2.0, 1.0 / 2.0);
        ImGuizmo::RecomposeMatrixFromComponents(
                glm::value_ptr(camera_pos_),
                glm::value_ptr(camera_rot_),
                glm::value_ptr(scale),
                tmp.data());
        Eigen::Matrix4d camera_mat_world = tmp.cast<double>();

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
        tmp_V = (tmp_V.rowwise().homogeneous() * camera_mat_world.transpose()).rowwise().hnormalized();

        // Set viewer data
        viewer->data().clear();
        viewer->data().set_mesh(tmp_V, tmp_F);
        viewer->data().set_face_based(true);
        Eigen::Vector3d blue_color = Eigen::Vector3d(0, 0, 255) / 255.0;
        viewer->data().uniform_colors(blue_color, blue_color, blue_color);
    } else {
        viewer->data().clear();
    }
}

void NextBestViewPlugin::show_bounding_box() {
    viewer->selected_data_index = VIEWER_DATA_BOUNDING_BOX;
    if (bounding_box_visible_) {

        // Bounding box transformation
        Eigen::Matrix4d bbox_mat_world = bounding_box_gizmo_.cast<double>();

        // Vertices
        Eigen::MatrixXd bbox_V(8, 3);
        bbox_V << 0, 0, 0,
                0, 0, 1,
                0, 1, 0,
                0, 1, 1,
                1, 0, 0,
                1, 0, 1,
                1, 1, 0,
                1, 1, 1;
        bbox_V = bbox_V.array() - 0.5;

        // Faces
        Eigen::MatrixXi bbox_F(12, 3);
        bbox_F << 0, 6, 4,
                0, 2, 6,
                0, 3, 2,
                0, 1, 3,
                2, 7, 6,
                2, 3, 7,
                4, 6, 7,
                4, 7, 5,
                0, 4, 5,
                0, 5, 1,
                1, 5, 7,
                1, 7, 3;

        // Apply transformation
        bbox_V = (bbox_V.rowwise().homogeneous() * bbox_mat_world.transpose()).rowwise().hnormalized();

        // Set viewer data
        viewer->data().clear();
        viewer->data().set_mesh(bbox_V, bbox_F);
        viewer->data().set_face_based(true);
        viewer->data().set_colors(Eigen::RowVector4d(0, 255, 0, 64)/255.0);
        viewer->data().show_lines = false;
        viewer->data().show_faces = true;
    } else {
        viewer->data().show_faces = false;
    }
}

void NextBestViewPlugin::set_nbv_mesh() {
    viewer->selected_data_index = VIEWER_DATA_NBV_MESH;
    viewer->data().clear();

    // Add vertices
    int num_vertices = next_best_view_->mvs_scene_->mesh.vertices.size();
    Eigen::MatrixXd V(num_vertices, 3);
    for (int i = 0; i < num_vertices; i++) {
        MVS::Mesh::Vertex vertex = next_best_view_->mvs_scene_->mesh.vertices[i];
        V(i, 0) = vertex[0];
        V(i, 1) = vertex[1];
        V(i, 2) = vertex[2];
    }

    // Add faces
    int num_faces = next_best_view_->mvs_scene_->mesh.faces.size();
    Eigen::MatrixXi F(num_faces, 3);
    for (int i = 0; i < num_faces; i++) {
        MVS::Mesh::Face face = next_best_view_->mvs_scene_->mesh.faces[i];
        F(i, 0) = face[0];
        F(i, 1) = face[1];
        F(i, 2) = face[2];
    }

    viewer->data().set_mesh(V, F);
    show_nbv_mesh(nbv_mesh_visible_);
}

void NextBestViewPlugin::set_nbv_mesh_color(const std::vector<double>& face_values) {
    viewer->selected_data_index = VIEWER_DATA_NBV_MESH;
    assert(face_values.size() == viewer->data().F.rows());

    int num_faces = face_values.size();
    Eigen::VectorXd tmp(num_faces);
    for (int i = 0; i < num_faces; i++) {
        tmp(i) = face_values[i];
    }

    Eigen::MatrixXd color;
    igl::colormap(igl::ColorMapType::COLOR_MAP_TYPE_JET, tmp, true, color);
    viewer->data().set_colors(color);
}

void NextBestViewPlugin::show_nbv_mesh(bool visible) {
    nbv_mesh_visible_ = visible;
    viewer->selected_data_index = VIEWER_DATA_NBV_MESH;
    viewer->data().show_faces = visible;
    viewer->data().show_lines = visible;
}

// Mouse IO
bool NextBestViewPlugin::mouse_down(int button, int modifier) {
    ImGui_ImplGlfwGL3_MouseButtonCallback(viewer->window, button, GLFW_PRESS, modifier);
    return ImGui::GetIO().WantCaptureMouse;
}

bool NextBestViewPlugin::mouse_up(int button, int modifier) {
    return ImGui::GetIO().WantCaptureMouse;
}

bool NextBestViewPlugin::mouse_move(int mouse_x, int mouse_y) {
    return ImGui::GetIO().WantCaptureMouse;
}

bool NextBestViewPlugin::mouse_scroll(float delta_y) {
    ImGui_ImplGlfwGL3_ScrollCallback(viewer->window, 0.f, delta_y);
    return ImGui::GetIO().WantCaptureMouse;
}

// Keyboard IO
bool NextBestViewPlugin::key_pressed(unsigned int key, int modifiers) {
    ImGui_ImplGlfwGL3_CharCallback(nullptr, key);
    if (!ImGui::GetIO().WantTextInput) {
        switch (key) {
            case 'n':
            {
                initialize_callback();
                return true;
            }
            case 't':
            {
                optimize_callback();
                return true;
            }
            case 'd':
            {
                debug_callback();
                return true;
            }
            case 'p':
            {
                if (!ImGui::GetIO().WantCaptureMouse) {
                    pick_face_callback();
                }
                return true;
            }
            default: break;
        }
    }
    return ImGui::GetIO().WantCaptureKeyboard;
}

bool NextBestViewPlugin::key_down(int key, int modifiers) {
    ImGui_ImplGlfwGL3_KeyCallback(viewer->window, key, 0, GLFW_PRESS, modifiers);
    return ImGui::GetIO().WantCaptureKeyboard;
}

bool NextBestViewPlugin::key_up(int key, int modifiers) {
    ImGui_ImplGlfwGL3_KeyCallback(viewer->window, key, 0, GLFW_RELEASE, modifiers);
    return ImGui::GetIO().WantCaptureKeyboard;
}
