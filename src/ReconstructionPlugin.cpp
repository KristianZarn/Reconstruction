#include "ReconstructionPlugin.h"

#include <sstream>
#include <iomanip>
#include <chrono>
#include <utility>
#include <vector>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui_impl_glfw_gl3.h>
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

    // Add two additional meshes to viewer. First is for cameras,
    // second for point cloud and the third for mesh
    viewer->append_mesh();
    viewer->append_mesh();
}

bool ReconstructionPlugin::post_draw() {
    // Setup window
    float window_width = 270.0f;
    ImGui::SetNextWindowSize(ImVec2(window_width, 0), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(ImVec2(180.0f, 0.0f), ImGuiCond_FirstUseEver);

    ImGui::Begin("Reconstruction", nullptr, ImGuiWindowFlags_NoSavedSettings);

    ImGui::Text("Sparse reconstruction:");
    if (ImGui::Button("Initialize [i]", ImVec2(-1,0))) {
        initialize_callback();
    }
    if (ImGui::Button("Extend [e]", ImVec2(-1, 0))) {
        extend_callback();
    }
    ImGui::Spacing();

    ImGui::Text("Edit views:");
    ImGui::PushItemWidth(100.0f);
    ImGui::InputInt("", &parameters_.view_to_delete);
    ImGui::PopItemWidth();
    ImGui::SameLine();
    if (ImGui::Button("Remove view", ImVec2(-1, 0))) {
        // TODO move to function
        /*log_stream_ << "View with id = " << parameters_.view_to_delete << " removed." << std::endl;
        reconstruction_builder_.RemoveView(static_cast<theia::ViewId>(parameters_.view_to_delete));
        reconstruction_builder_.PrintStatistics(log_stream_);

        set_point_cloud();
        show_point_cloud();
        set_cameras();
        show_cameras();*/
    }
    if (ImGui::Button("Remove last view", ImVec2(-1, 0))) {
        // TODO remove last view
    }
    if (ImGui::Button("Reset reconstruction", ImVec2(-1, 0))) {
        log_stream_ << "Reconstruction is reset" << std::endl;
        reset_reconstruction();
    }
    ImGui::Spacing();

    ImGui::Text("Dense reconstruction:");
    if (ImGui::Button("Reconstruct mesh [m]", ImVec2(-1, 0))) {
        reconstruct_mesh_callback();
    }
    if (ImGui::Button("Texture mesh [t]", ImVec2(-1, 0))) {
        texture_mesh_callback();
    }
    ImGui::Spacing();

    ImGui::Text("Display options:");
    if (ImGui::Checkbox("Show cameras [1]", &parameters_.visible_cameras)) {
        set_cameras_visible(parameters_.visible_cameras);
    }
    if (ImGui::Checkbox("Show point cloud [2]", &parameters_.visible_point_cloud)) {
        set_point_cloud_visible(parameters_.visible_point_cloud);
    }
    if (ImGui::Checkbox("Show mesh [3]", &parameters_.visible_mesh)) {
        set_mesh_visible(parameters_.visible_mesh);
    }
    ImGui::SliderInt("Point size", &parameters_.point_size, 1, 10);
    for (auto& viewer_data : viewer->data_list) {
        if (viewer_data.point_size != parameters_.point_size) {
            viewer_data.point_size = parameters_.point_size;
        }
    }
    ImGui::Spacing();

    ImGui::Text("Output");
    if (ImGui::Button("Save point cloud", ImVec2(-1, 0))) {
        std::string filename = "sparse_point_cloud.ply";
        reconstruction_builder_.WritePly(reconstruction_path_ + filename);
        log_stream_ << "Written to: \n\t" << (reconstruction_path_ + filename) << std::endl;
    }
    if (ImGui::Button("Save mesh", ImVec2(-1, 0))) {
        std::string filename = "mesh.ply";
        mvs_scene_.mesh.Save(reconstruction_path_ + filename);
        log_stream_ << "Written to: \n\t" << (reconstruction_path_ + filename) << std::endl;
    }
    ImGui::Spacing();

    ImGui::End();
    return false;
}

void ReconstructionPlugin::initialize_callback() {
    // Images for initial reconstruction
    std::string image0, image1;
    if (parameters_.next_image_idx < image_names_.size()) {
        image0 = images_path_ + image_names_[parameters_.next_image_idx];
        parameters_.next_image_idx++;
        image1 = images_path_ + image_names_[parameters_.next_image_idx];
        parameters_.next_image_idx++;
    } else {
        log_stream_ << "Next image not available." << std::endl;
        return;
    }

    if (!theia::FileExists(image0)) {
        log_stream_ << "Image: " << image0 << " does not exist." << std::endl;
        return;
    }
    if (!theia::FileExists(image1)) {
        log_stream_ << "Image: " << image1 << " does not exist." << std::endl;
        return;
    }

    // Initialize reconstruction
    log_stream_ << "Starting initialization" << std::endl;
    auto time_begin = std::chrono::steady_clock::now();

    theia::ReconstructionEstimatorSummary summary =
            reconstruction_builder_.InitializeReconstruction(image0, image1);

    auto time_end = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_elapsed = time_end - time_begin;
    log_stream_ << "Initialization time: " << time_elapsed.count() << " s" << std::endl;

    // Reconstruction summary
    if (summary.success) {
        reconstruction_builder_.ColorizeReconstruction(images_path_);
        reconstruction_builder_.PrintStatistics(log_stream_);
        set_cameras();
        set_cameras_visible(true);
        set_point_cloud();
        set_point_cloud_visible(true);
        set_mesh_visible(false);
    } else {
        log_stream_ << "Initialization failed: \n";
        log_stream_ << "\n\tMessage = " << summary.message << "\n\n";

        reset_reconstruction();

        log_stream_ << "Reconstruction is reset" << std::endl;
    }
}

void ReconstructionPlugin::extend_callback() {
    // Image for extend
    std::string image;
    if (parameters_.next_image_idx < image_names_.size()) {
        image = images_path_ + image_names_[parameters_.next_image_idx];
        parameters_.next_image_idx++;
    } else {
        log_stream_ << "Next image not available." << std::endl;
        return;
    }

    if (!theia::FileExists(image)) {
        log_stream_ << "Image: " << image << " does not exist." << std::endl;
        return;
    }

    // Extend reconstruction
    log_stream_ << "Extending reconstruction" << std::endl;
    auto time_begin = std::chrono::steady_clock::now();

    theia::ReconstructionEstimatorSummary summary = reconstruction_builder_.ExtendReconstruction(image);

    auto time_end = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_elapsed = time_end - time_begin;
    log_stream_ << "Extend time: " << time_elapsed.count() << " s" << std::endl;

    // Reconstruction summary
    if (summary.success) {
        reconstruction_builder_.ColorizeReconstruction(images_path_);
        reconstruction_builder_.PrintStatistics(log_stream_);
        set_cameras();
        set_cameras_visible(true);
        set_point_cloud();
        set_point_cloud_visible(true);
        set_mesh_visible(false);
    } else {
        log_stream_ << "Extend failed: \n";
        log_stream_ << "\n\tMessage = " << summary.message << "\n\n";
        // TODO: remove last view
    }
}

void ReconstructionPlugin::reconstruct_mesh_callback() {
    // Set paths
    std::string undistoreted_images_folder = images_path_;

    // Convert reconstruction to mvs scene
    mvs_scene_.Release();
    bool convert_success = theia_to_mvs(
            *(reconstruction_builder_.GetReconstruction()), undistoreted_images_folder, mvs_scene_);

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
    mvs_scene_.ReconstructMesh(parameters_.dist_insert, parameters_.use_free_space_support, 4,
                               parameters_.thickness_factor, parameters_.quality_factor);
    log_stream_ << "Mesh reconstruction completed: "
                << mvs_scene_.mesh.vertices.GetSize() << " vertices, "
                << mvs_scene_.mesh.faces.GetSize() << " faces." << std::endl;

    // Clean the mesh
    mvs_scene_.mesh.Clean(parameters_.decimate_mesh, parameters_.remove_spurious, parameters_.remove_spikes,
                          parameters_.close_holes, parameters_.smooth_mesh, false);
    set_mesh();
    set_mesh_visible(true);
    set_point_cloud_visible(false);
}

void ReconstructionPlugin::dense_reconstruct_mesh_callback() {
    // TODO: densify point cloud
}

void ReconstructionPlugin::texture_mesh_callback() {
    if (!mvs_scene_.mesh.IsEmpty()) {
        mvs_scene_.TextureMesh(parameters_.resolution_level,
                               parameters_.min_resolution,
                               parameters_.texture_outlier_treshold,
                               parameters_.cost_smoothness_ratio,
                               parameters_.global_seam_leveling,
                               parameters_.local_seam_leveling,
                               parameters_.texture_size_multiple,
                               parameters_.patch_packing_heuristic,
                               Pixel8U(parameters_.empty_color));
        set_mesh();
        set_mesh_visible(true);
        set_point_cloud_visible(false);
    } else {
        log_stream_ << "Texturing failed: Mesh is empty." << std::endl;
    }
}

void ReconstructionPlugin::set_cameras() {
    viewer->selected_data_index = static_cast<int>(DataIdx::CAMERAS);
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

void ReconstructionPlugin::set_cameras_visible(bool visible) {
    parameters_.visible_cameras = visible;
    viewer->selected_data_index = static_cast<int>(DataIdx::CAMERAS);
    viewer->data().show_overlay = visible;
}

void ReconstructionPlugin::set_point_cloud() {
    viewer->selected_data_index = static_cast<int>(DataIdx::POINT_CLOUD);
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

    // Center object
    viewer->core.align_camera_center(points);
}

void ReconstructionPlugin::set_point_cloud_visible(bool visible) {
    parameters_.visible_point_cloud = visible;
    viewer->selected_data_index = static_cast<int>(DataIdx::POINT_CLOUD);
    viewer->data().show_overlay = visible;
}

void ReconstructionPlugin::set_mesh() {
    viewer->selected_data_index = static_cast<int>(DataIdx::MESH);
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
        viewer->data().show_texture = true;
    } else {
        viewer->data().show_texture = false;
    }

    // Center object
    viewer->core.align_camera_center(V);
}

void ReconstructionPlugin::set_mesh_visible(bool visible) {
    parameters_.visible_mesh = visible;
    viewer->selected_data_index = static_cast<int>(DataIdx::MESH);
    viewer->data().show_faces = visible;
    viewer->data().show_lines = visible;
}

void ReconstructionPlugin::reset_reconstruction() {
    parameters_.next_image_idx = 0;
    reconstruction_builder_.ResetReconstruction();
    viewer->data().clear();
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
    ImGui_ImplGlfwGL3_CharCallback(nullptr, key);
    switch (key) {
        case 'i':
        {
            initialize_callback();
            return true;
        }
        case 'e':
        {
            extend_callback();
            return true;
        }
        case 'm':
        {
            reconstruct_mesh_callback();
            return true;
        }
        case 't':
        {
            texture_mesh_callback();
            return true;
        }
        case '1':
        {
            set_cameras_visible(!parameters_.visible_cameras);
            return true;
        }
        case '2':
        {
            set_point_cloud_visible(!parameters_.visible_point_cloud);
            return true;
        }
        case '3':
        {
            set_mesh_visible(!parameters_.visible_mesh);
            return true;
        }
        default: break;
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