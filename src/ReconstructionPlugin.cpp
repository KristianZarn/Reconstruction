#include "ReconstructionPlugin.h"

#include <sstream>
#include <iomanip>
#include <chrono>
#include <utility>
#include <vector>
#include <algorithm>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui_impl_glfw_gl3.h>
#include <igl/project.h>
#include "theia/sfm/reconstruction.h"
#include <theia/sfm/reconstruction_estimator.h>
#include <theia/sfm/reconstruction_estimator_utils.h>
#include <theia/io/reconstruction_reader.h>
#include <theia/util/filesystem.h>

#include "helpers.h"

ReconstructionPlugin::ReconstructionPlugin(Parameters parameters,
                                           std::string images_path,
                                           const std::vector<std::string>& image_names,
                                           std::string reconstruction_path,
                                           theia::RealtimeReconstructionBuilder::Options options,
                                           theia::CameraIntrinsicsPrior intrinsics_prior)
        : parameters_(parameters),
          images_path_(std::move(images_path)),
          image_names_(image_names),
          reconstruction_path_(std::move(reconstruction_path)),
          reconstruction_builder_(options, intrinsics_prior),
          mvs_scene_(static_cast<unsigned int>(options.num_threads)) {}

void ReconstructionPlugin::init(igl::opengl::glfw::Viewer *_viewer) {
    ViewerPlugin::init(_viewer);

    // First mesh is for cameras
    VIEWER_DATA_CAMERAS = static_cast<unsigned int>(viewer->data_list.size() - 1);

    // Append mesh for point cloud
    viewer->append_mesh();
    VIEWER_DATA_POINT_CLOUD = static_cast<unsigned int>(viewer->data_list.size() - 1);

    // Append mesh for mesh
    viewer->append_mesh();
    VIEWER_DATA_MESH = static_cast<unsigned int>(viewer->data_list.size() - 1);
}

const MVS::Scene& ReconstructionPlugin::get_mvs_scene() {
    return mvs_scene_;
}

bool ReconstructionPlugin::post_draw() {
    // Text labels
    if (parameters_.show_labels) {
        draw_labels_window();
    }

    // Setup window
    float window_width = 300.0f;
    ImGui::SetNextWindowSize(ImVec2(window_width, 0), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiCond_FirstUseEver);

    ImGui::Begin("Reconstruction", nullptr, ImGuiWindowFlags_NoSavedSettings);

    // Sparse reconstruction
    ImGui::Text("Sparse reconstruction:");
    if (ImGui::Button("Initialize [i]", ImVec2(-1,0))) {
        initialize_callback();
    }
    if (ImGui::Button("Extend [e]", ImVec2(-1, 0))) {
        extend_callback();
    }
    ImGui::Spacing();

    // Edit views
    ImGui::Text("Edit views:");
    ImGui::PushItemWidth(100.0f);
    ImGui::InputInt("", &parameters_.view_to_delete);
    ImGui::PopItemWidth();
    ImGui::SameLine();
    if (ImGui::Button("Remove view", ImVec2(-1, 0))) {
        remove_view_callback(parameters_.view_to_delete);
    }
    if (ImGui::Button("Remove last view [b]", ImVec2(-1, 0))) {
        remove_last_view_callback();
    }
    if (ImGui::Button("Reset reconstruction", ImVec2(-1, 0))) {
        reset_reconstruction_callback();
    }
    ImGui::Spacing();

    // Dense reconstruction
    ImGui::Text("Dense reconstruction:");
    if (ImGui::Button("Reconstruct mesh [m]", ImVec2(-1, 0))) {
        reconstruct_mesh_callback();
    }
    if (ImGui::Button("Refine mesh", ImVec2(-1, 0))) {
        refine_mesh_callback();
    }
    if (ImGui::Button("Texture mesh [t]", ImVec2(-1, 0))) {
        texture_mesh_callback();
    }
    ImGui::Spacing();

    // Display options
    ImGui::Text("Display options:");
    if (ImGui::Button("Center object", ImVec2(-1, 0))) {
        center_object_callback();
    }
    ImGui::Checkbox("Show labels", &parameters_.show_labels);
    if (ImGui::Checkbox("Show cameras [1]", &parameters_.show_cameras)) {
        show_cameras(parameters_.show_cameras);
    }
    if (ImGui::Checkbox("Show point cloud [2]", &parameters_.show_point_cloud)) {
        show_point_cloud(parameters_.show_point_cloud);
    }
    if (ImGui::Checkbox("Show mesh [3]", &parameters_.show_mesh)) {
        show_mesh(parameters_.show_mesh);
    }
    if (ImGui::Checkbox("Show texture", &parameters_.show_texture)) {
        viewer->selected_data_index = VIEWER_DATA_MESH;
        viewer->data().show_texture = parameters_.show_texture;
    }
    if (ImGui::Checkbox("Show wireframe", &parameters_.show_wireframe)) {
        viewer->selected_data_index = VIEWER_DATA_MESH;
        viewer->data().show_lines = parameters_.show_wireframe;
    }
    ImGui::SliderInt("Point size", &parameters_.point_size, 1, 10);
    for (auto& viewer_data : viewer->data_list) {
        if (viewer_data.point_size != parameters_.point_size) {
            viewer_data.point_size = parameters_.point_size;
        }
    }
    ImGui::Spacing();

    // Input output
    ImGui::Text("Input / Output:");
    ImGui::InputText("Filename", parameters_.filename_buffer, 64, ImGuiInputTextFlags_AutoSelectAll);
    ImGui::Spacing();
    if (ImGui::Button("Save point cloud", ImVec2(-1, 0))) {
        log_stream_ << std::endl;

        std::string filename = std::string(parameters_.filename_buffer) + ".ply";
        reconstruction_builder_.WritePly(reconstruction_path_ + filename);
        log_stream_ << "Written to: \n\t" << (reconstruction_path_ + filename) << std::endl;
    }
    if (ImGui::Button("Save mesh", ImVec2(-1, 0))) {
        log_stream_ << std::endl;

        std::string filename_mvs = std::string(parameters_.filename_buffer) + ".mvs";
        mvs_scene_.Save(reconstruction_path_ + filename_mvs);
        log_stream_ << "Written to: \n\t" << (reconstruction_path_ + filename_mvs) << std::endl;

        std::string filename_ply = std::string(parameters_.filename_buffer) + ".ply";
        mvs_scene_.mesh.Save(reconstruction_path_ + filename_ply);
        log_stream_ << "Written to: \n\t" << (reconstruction_path_ + filename_ply) << std::endl;
    }
    if (ImGui::Button("Load mesh", ImVec2(-1, 0))) {
        log_stream_ << std::endl;

        std::string filename_mvs = std::string(parameters_.filename_buffer) + ".mvs";
        mvs_scene_.Load(reconstruction_path_ + filename_mvs);
        set_mesh();
        show_mesh(true);
        show_point_cloud(false);
        log_stream_ << "Loaded from: \n\t" << (reconstruction_path_ + filename_mvs) << std::endl;
    }
    std::ostringstream os;
    os << "Mesh info:"
       << "\t" << mvs_scene_.mesh.vertices.GetSize() << " vertices"
       << "\t" << mvs_scene_.mesh.faces.GetSize() << " faces";
    ImGui::TextUnformatted(os.str().c_str());
    ImGui::Spacing();

    // Debug info
    // std::ostringstream debug;
    // debug << "View: \n" << viewer->core.view << std::endl;
    // debug << "Norm: \n" << viewer->core.norm << std::endl;
    // debug << "Proj: \n" << viewer->core.proj << std::endl;

    // Eigen::Quaternionf quat = viewer->core.trackball_angle;
    // debug << "trackball_angle: \n" << quat.w() << ", " << quat.x() << ", " << quat.y() << ", " << quat.z() << std::endl;
    // ImGui::TextUnformatted(os.str().c_str());

    ImGui::End();
    return false;
}

void ReconstructionPlugin::initialize_callback() {
    log_stream_ << std::endl;

    // Images for initial reconstruction
    std::string image1, image2;
    if ((parameters_.next_image_idx + 1) < image_names_.size()) {
        image1 = images_path_ + image_names_[parameters_.next_image_idx];
        parameters_.next_image_idx++;
        image2 = images_path_ + image_names_[parameters_.next_image_idx];
        parameters_.next_image_idx++;
    } else {
        log_stream_ << "Initialization failed:\n"
                    << "\tImages not available (two images needed)." << std::endl;
        return;
    }

    // Initialize reconstruction
    log_stream_ << "Initializing reconstruction ..." << std::endl;
    auto time_begin = std::chrono::steady_clock::now();

    theia::ReconstructionEstimatorSummary summary =
            reconstruction_builder_.InitializeReconstruction(image1, image2);

    auto time_end = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_elapsed = time_end - time_begin;
    log_stream_ << "Initialization time: " << time_elapsed.count() << " s" << std::endl;

    // Reconstruction summary
    if (summary.success) {
        log_stream_ << "Initialization successful: \n";
        reconstruction_builder_.ColorizeReconstruction(images_path_);
    } else {
        log_stream_ << "Initialization failed: \n";
        log_stream_ << "\tMessage = " << summary.message << "\n\n";
    }
    reconstruction_builder_.PrintStatistics(log_stream_);
    set_cameras();
    show_cameras(true);
    set_point_cloud();
    show_point_cloud(true);
    show_mesh(false);
    center_object_callback();
}

void ReconstructionPlugin::extend_callback() {
    log_stream_ << std::endl;

    // Image for extend
    std::string image;
    if (parameters_.next_image_idx < image_names_.size()) {
        image = images_path_ + image_names_[parameters_.next_image_idx];
        parameters_.next_image_idx++;
    } else {
        log_stream_ << "Extend failed:\n"
                    << "\tNext image not available." << std::endl;
        return;
    }

    // Extend reconstruction
    log_stream_ << "Extending reconstruction ..." << std::endl;
    auto time_begin = std::chrono::steady_clock::now();

    theia::ReconstructionEstimatorSummary summary = reconstruction_builder_.ExtendReconstruction(image);

    auto time_end = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_elapsed = time_end - time_begin;
    log_stream_ << "Extend time: " << time_elapsed.count() << " s" << std::endl;

    // Reconstruction summary
    if (summary.success) {
        log_stream_ << "Extend successful: \n";
        reconstruction_builder_.ColorizeReconstruction(images_path_);
    } else {
        log_stream_ << "Extend failed: \n";
        log_stream_ << "\tMessage = " << summary.message << "\n\n";
    }
    reconstruction_builder_.PrintStatistics(log_stream_);
    set_cameras();
    show_cameras(true);
    set_point_cloud();
    show_point_cloud(true);
    show_mesh(false);
}

void ReconstructionPlugin::remove_view_callback(int view_id) {
    log_stream_ << std::endl;

    reconstruction_builder_.RemoveView(static_cast<theia::ViewId>(view_id));
    log_stream_ << "Removed view with id = " << view_id << std::endl;
    reconstruction_builder_.PrintStatistics(log_stream_);

    set_point_cloud();
    show_point_cloud(true);
    set_cameras();
    show_cameras(true);
}

void ReconstructionPlugin::remove_last_view_callback() {
    std::vector<theia::ViewId> view_ids = reconstruction_builder_.GetReconstruction()->ViewIds();
    if (!view_ids.empty()) {
        theia::ViewId view_to_delete = *std::max_element(view_ids.begin(), view_ids.end());
        remove_view_callback(view_to_delete);
    }
}

void ReconstructionPlugin::reset_reconstruction_callback() {
    log_stream_ << std::endl;

    reconstruction_builder_.ResetReconstruction();
    for (auto& viewer_data : viewer->data_list) {
        viewer_data.clear();
    }
    log_stream_ << "Reconstruction reset" << std::endl;
}

void ReconstructionPlugin::reconstruct_mesh_callback() {
    log_stream_ << std::endl;

    // Set paths
    std::string undistoreted_images_folder = images_path_;

    // Convert reconstruction to mvs scene
    mvs_scene_.Release();
    theia_to_mvs(*(reconstruction_builder_.GetReconstruction()), undistoreted_images_folder, mvs_scene_);

    // Select neighbor views
    int i = 0;
    for (auto& image : mvs_scene_.images) {
        image.ReloadImage(0, false);
        image.UpdateCamera(mvs_scene_.platforms);
        if (image.neighbors.IsEmpty()) {
            SEACAVE::IndexArr points;
            mvs_scene_.SelectNeighborViews(static_cast<uint32_t>(i), points);
        }
        i++;
    }

    // Reconstruct mesh
    log_stream_ << "Reconstructing mesh ..." << std::endl;
    auto time_begin = std::chrono::steady_clock::now();

    mvs_scene_.ReconstructMesh(parameters_.dist_insert,
                               parameters_.use_free_space_support,
                               parameters_.fix_non_manifold,
                               parameters_.thickness_factor,
                               parameters_.quality_factor);

    auto time_end = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_elapsed = time_end - time_begin;
    log_stream_ << "Reconstruct mesh time: " << time_elapsed.count() << " s" << std::endl;
    log_stream_ << "Reconstruct mesh result: \n\t"
                << mvs_scene_.mesh.vertices.GetSize() << " vertices, "
                << mvs_scene_.mesh.faces.GetSize() << " faces." << std::endl;

    // Clean the mesh
    mvs_scene_.mesh.Clean(parameters_.decimate_mesh, parameters_.remove_spurious, parameters_.remove_spikes,
                          parameters_.close_holes, parameters_.smooth_mesh, false);
    set_mesh();
    show_mesh(true);
    show_point_cloud(false);
}

void ReconstructionPlugin::refine_mesh_callback() {
    log_stream_ << std::endl;
    if (!mvs_scene_.mesh.IsEmpty()) {
        log_stream_ << "Refining mesh ..." << std::endl;
        auto time_begin = std::chrono::steady_clock::now();

        // mvs_scene_.RefineMeshCUDA(parameters_.refine_resolution_level,
        //                           parameters_.refine_min_resolution,
        //                           parameters_.refine_max_views,
        //                           parameters_.refine_decimate,
        //                           parameters_.refine_close_holes,
        //                           parameters_.ensure_edge_size,
        //                           parameters_.max_face_area,
        //                           parameters_.scales,
        //                           parameters_.scale_step,
        //                           parameters_.alternative_pair,
        //                           parameters_.regularity_weight,
        //                           parameters_.rigidity_elasticity_ratio,
        //                           parameters_.gradient_step);

        mvs_scene_.RefineMesh(parameters_.refine_resolution_level,
                              parameters_.refine_min_resolution,
                              parameters_.refine_max_views,
                              parameters_.refine_decimate,
                              parameters_.refine_close_holes,
                              parameters_.ensure_edge_size,
                              parameters_.max_face_area,
                              parameters_.scales,
                              parameters_.scale_step,
                              parameters_.reduce_memory,
                              parameters_.alternative_pair,
                              parameters_.regularity_weight,
                              parameters_.rigidity_elasticity_ratio,
                              parameters_.planar_vertex_ratio,
                              parameters_.gradient_step);

        auto time_end = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_elapsed = time_end - time_begin;
        log_stream_ << "Refine mesh time: " << time_elapsed.count() << " s" << std::endl;
        log_stream_ << "Refine mesh result: \n\t"
                    << mvs_scene_.mesh.vertices.GetSize() << " vertices, "
                    << mvs_scene_.mesh.faces.GetSize() << " faces." << std::endl;

        set_mesh();
        show_mesh(true);
        show_point_cloud(false);
    } else {
        log_stream_ << "Refine mesh failed: Mesh is empty." << std::endl;
    }
}

void ReconstructionPlugin::texture_mesh_callback() {
    log_stream_ << std::endl;
    if (!mvs_scene_.mesh.IsEmpty()) {
        log_stream_ << "Texturing mesh ..." << std::endl;
        auto time_begin = std::chrono::steady_clock::now();

        mvs_scene_.TextureMesh(parameters_.texture_resolution_level,
                               parameters_.min_resolution,
                               parameters_.texture_outlier_treshold,
                               parameters_.cost_smoothness_ratio,
                               parameters_.global_seam_leveling,
                               parameters_.local_seam_leveling,
                               parameters_.texture_size_multiple,
                               parameters_.patch_packing_heuristic,
                               Pixel8U(parameters_.empty_color));

        auto time_end = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_elapsed = time_end - time_begin;
        log_stream_ << "Texture mesh time: " << time_elapsed.count() << " s" << std::endl;

        set_mesh();
        show_mesh(true);
        show_point_cloud(false);
    } else {
        log_stream_ << "Texture mesh failed: Mesh is empty." << std::endl;
    }
}

void ReconstructionPlugin::center_object_callback() {
    Eigen::MatrixXd points;
    viewer->selected_data_index = VIEWER_DATA_MESH;
    if (points.rows() == 0 && viewer->data().V.rows() > 0) {
        points = viewer->data().V;
    }
    viewer->selected_data_index = VIEWER_DATA_POINT_CLOUD;
    if (points.rows() == 0 && viewer->data().points.rows() > 0) {
        points = viewer->data().points.leftCols(3);
    }
    if (points.rows() == 0) {
        return;
    }

    Eigen::Vector3d min_point = points.colwise().minCoeff();
    Eigen::Vector3d max_point = points.colwise().maxCoeff();
    Eigen::Vector3d center = points.colwise().mean();
    viewer->core.camera_base_translation = -center.cast<float>();
    viewer->core.camera_translation.setConstant(0);

    viewer->core.camera_base_zoom = 2.0 / (max_point-min_point).array().abs().maxCoeff();
    viewer->core.camera_zoom = 1.0;
}

void ReconstructionPlugin::set_cameras() {
    viewer->selected_data_index = VIEWER_DATA_CAMERAS;
    viewer->data().clear();

    theia::Reconstruction* reconstruction = reconstruction_builder_.GetReconstruction();

    // Add cameras
    std::unordered_set<theia::ViewId> view_ids;
    theia::GetEstimatedViewsFromReconstruction(*reconstruction, &view_ids);

    auto num_views = static_cast<int>(view_ids.size());
    Eigen::MatrixXd cameras(num_views, 3);

    int i = 0;
    for (const auto& view_id : view_ids) {
        Eigen::Vector3d position = reconstruction->View(view_id)->Camera().GetPosition();
        cameras(i, 0) = position(0);
        cameras(i, 1) = position(1);
        cameras(i, 2) = position(2);

        // Add camera label
        viewer->data().add_label(position, std::to_string(view_id));
        i++;
    }
    viewer->data().set_points(cameras, Eigen::RowVector3d(0, 1, 0));
}

void ReconstructionPlugin::show_cameras(bool visible) {
    parameters_.show_cameras = visible;
    viewer->selected_data_index = VIEWER_DATA_CAMERAS;
    viewer->data().show_overlay = visible;
}

void ReconstructionPlugin::set_point_cloud() {
    viewer->selected_data_index = VIEWER_DATA_POINT_CLOUD;
    viewer->data().clear();

    theia::Reconstruction* reconstruction = reconstruction_builder_.GetReconstruction();

    // Add points and colors
    std::unordered_set<theia::TrackId> track_ids;
    theia::GetEstimatedTracksFromReconstruction(*reconstruction, &track_ids);

    auto num_points = static_cast<int>(track_ids.size());
    Eigen::MatrixXd points(num_points, 3);
    Eigen::MatrixXd colors(num_points, 3);

    int i = 0;
    for (const auto& track_id : track_ids) {
        const theia::Track* track = reconstruction->Track(track_id);

        Eigen::Vector3d point = track->Point().hnormalized();
        points(i, 0) = point(0);
        points(i, 1) = point(1);
        points(i, 2) = point(2);

        Eigen::Matrix<uint8_t, 3, 1> color = track->Color();
        colors(i, 0) = color(0);
        colors(i, 1) = color(1);
        colors(i, 2) = color(2);

        i++;
    }
    colors = colors / 255.0;
    viewer->data().set_points(points, colors);
}

void ReconstructionPlugin::show_point_cloud(bool visible) {
    parameters_.show_point_cloud = visible;
    viewer->selected_data_index = VIEWER_DATA_POINT_CLOUD;
    viewer->data().show_overlay = visible;
}

void ReconstructionPlugin::set_mesh() {
    viewer->selected_data_index = VIEWER_DATA_MESH;
    viewer->data().clear();

    // Add vertices
    int num_vertices = mvs_scene_.mesh.vertices.size();
    Eigen::MatrixXd V(num_vertices, 3);
    for (int i = 0; i < num_vertices; i++) {
        MVS::Mesh::Vertex vertex = mvs_scene_.mesh.vertices[i];
        V(i, 0) = vertex[0];
        V(i, 1) = vertex[1];
        V(i, 2) = vertex[2];
    }
    // Add faces
    int num_faces = mvs_scene_.mesh.faces.size();
    Eigen::MatrixXi F(num_faces, 3);
    for (int i = 0; i < num_faces; i++) {
        MVS::Mesh::Face face = mvs_scene_.mesh.faces[i];
        F(i, 0) = face[0];
        F(i, 1) = face[1];
        F(i, 2) = face[2];
    }
    viewer->data().set_mesh(V, F);
    viewer->data().show_lines = parameters_.show_wireframe;

    // Add texture if available
    if (!mvs_scene_.mesh.faceTexcoords.IsEmpty()) {

        // Set UVs
        int num_texcoords = mvs_scene_.mesh.faceTexcoords.size();
        Eigen::MatrixXd TC(num_texcoords, 2);
        for (int i = 0; i < num_texcoords; i++) {
            MVS::Mesh::TexCoord texcoord = mvs_scene_.mesh.faceTexcoords[i];
            TC(i, 0) = texcoord[0];
            TC(i, 1) = texcoord[1];
        }
        Eigen::MatrixXi FTC(num_faces, 3);
        for (int i = 0; i < num_faces; i++) {
            FTC(i, 0) = i*3 + 0;
            FTC(i, 1) = i*3 + 1;
            FTC(i, 2) = i*3 + 2;
        }
        viewer->data().set_uv(TC, FTC);

        // Set texture
        SEACAVE::Image8U3 img = mvs_scene_.mesh.textureDiffuse;
        int width = img.width();
        int height = img.height();

        Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic> R(width, height);
        Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic> G(width, height);
        Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic> B(width, height);

        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                Pixel8U pixel = img.getPixel(j, i);
                R(i, j) = pixel.r;
                G(i, j) = pixel.g;
                B(i, j) = pixel.b;
            }
        }

        viewer->data().set_colors(Eigen::RowVector3d(1, 1, 1));
        viewer->data().set_texture(R.rowwise().reverse(), G.rowwise().reverse(), B.rowwise().reverse());
        viewer->data().show_texture = parameters_.show_texture;
    }
}

void ReconstructionPlugin::show_mesh(bool visible) {
    parameters_.show_mesh = visible;
    viewer->selected_data_index = VIEWER_DATA_MESH;
    viewer->data().show_faces = visible;
}

// Mouse IO
bool ReconstructionPlugin::mouse_down(int button, int modifier)
{
    ImGui_ImplGlfwGL3_MouseButtonCallback(viewer->window, button, GLFW_PRESS, modifier);
    return ImGui::GetIO().WantCaptureMouse;
}

bool ReconstructionPlugin::mouse_up(int button, int modifier)
{
    return ImGui::GetIO().WantCaptureMouse;
}

bool ReconstructionPlugin::mouse_move(int mouse_x, int mouse_y)
{
    return ImGui::GetIO().WantCaptureMouse;
}

bool ReconstructionPlugin::mouse_scroll(float delta_y)
{
    ImGui_ImplGlfwGL3_ScrollCallback(viewer->window, 0.f, delta_y);
    return ImGui::GetIO().WantCaptureMouse;
}

// Keyboard IO
bool ReconstructionPlugin::key_pressed(unsigned int key, int modifiers)
{
    ImGui_ImplGlfwGL3_CharCallback(viewer->window, key);
    if (!ImGui::GetIO().WantTextInput) {
        switch (key) {
            case 'i':
            {
                initialize_callback();
                return ImGui::GetIO().WantCaptureKeyboard;
            }
            case 'e':
            {
                extend_callback();
                return ImGui::GetIO().WantCaptureKeyboard;
            }
            case 'b':
            {
                remove_last_view_callback();
                return ImGui::GetIO().WantCaptureKeyboard;
            }
            case 'm':
            {
                reconstruct_mesh_callback();
                return ImGui::GetIO().WantCaptureKeyboard;
            }
            case 't':
            {
                texture_mesh_callback();
                return ImGui::GetIO().WantCaptureKeyboard;
            }
            case '1':
            {
                show_cameras(!parameters_.show_cameras);
                return ImGui::GetIO().WantCaptureKeyboard;
            }
            case '2':
            {
                show_point_cloud(!parameters_.show_point_cloud);
                return ImGui::GetIO().WantCaptureKeyboard;
            }
            case '3':
            {
                show_mesh(!parameters_.show_mesh);
                return ImGui::GetIO().WantCaptureKeyboard;
            }
            default: break;
        }
    }
    return ImGui::GetIO().WantCaptureKeyboard;
}

bool ReconstructionPlugin::key_down(int key, int modifiers)
{
    ImGui_ImplGlfwGL3_KeyCallback(viewer->window, key, 0, GLFW_PRESS, modifiers);
    return ImGui::GetIO().WantCaptureKeyboard;
}

bool ReconstructionPlugin::key_up(int key, int modifiers)
{
    ImGui_ImplGlfwGL3_KeyCallback(viewer->window, key, 0, GLFW_RELEASE, modifiers);
    return ImGui::GetIO().WantCaptureKeyboard;
}

void ReconstructionPlugin::draw_labels_window() {
    // Text labels
    ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiSetCond_Always);
    ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize, ImGuiSetCond_Always);
    bool visible = true;
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0, 0, 0, 0));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0);
    ImGui::Begin("ViewerLabels", &visible,
                 ImGuiWindowFlags_NoTitleBar
                 | ImGuiWindowFlags_NoResize
                 | ImGuiWindowFlags_NoMove
                 | ImGuiWindowFlags_NoScrollbar
                 | ImGuiWindowFlags_NoScrollWithMouse
                 | ImGuiWindowFlags_NoCollapse
                 | ImGuiWindowFlags_NoSavedSettings
                 | ImGuiWindowFlags_NoInputs);
    for (const auto &data : viewer->data_list) {
        draw_labels(data);
    }
    ImGui::End();
    ImGui::PopStyleColor();
    ImGui::PopStyleVar();
}

void ReconstructionPlugin::draw_labels(const igl::opengl::ViewerData &data) {
    if (data.show_vertid) {
        for (int i = 0; i < data.V.rows(); ++i) {
            draw_text(data.V.row(i), data.V_normals.row(i), std::to_string(i));
        }
    }

    if (data.show_faceid) {
        for (int i = 0; i < data.F.rows(); ++i) {
            Eigen::RowVector3d p = Eigen::RowVector3d::Zero();
            for (int j = 0; j < data.F.cols(); ++j) {
                p += data.V.row(data.F(i, j));
            }
            p /= (double) data.F.cols();

            draw_text(p, data.F_normals.row(i), std::to_string(i));
        }
    }

    if (data.labels_positions.rows() > 0) {
        for (int i = 0; i < data.labels_positions.rows(); ++i) {
            draw_text(data.labels_positions.row(i), Eigen::Vector3d(0.0, 0.0, 0.0),
                      data.labels_strings[i]);
        }
    }
}

void ReconstructionPlugin::draw_text(Eigen::Vector3d pos, Eigen::Vector3d normal, const std::string &text) {
    pos += normal * 0.005f * viewer->core.object_scale;
    Eigen::Vector3f coord = igl::project(Eigen::Vector3f(pos.cast<float>()),
                                         viewer->core.view, viewer->core.proj, viewer->core.viewport);

    // Draw text labels slightly bigger than normal text
    ImDrawList* drawList = ImGui::GetWindowDrawList();
    drawList->AddText(ImGui::GetFont(), ImGui::GetFontSize() * 1.5f,
                      ImVec2(coord[0], (viewer->core.viewport[3] - coord[1])),
                      ImGui::GetColorU32(ImVec4(0, 255, 0, 255)),
                      &text[0], &text[0] + text.size());
}