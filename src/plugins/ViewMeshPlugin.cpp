#include "ViewMeshPlugin.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui_impl_glfw_gl3.h>
#include <igl/jet.h>

ViewMeshPlugin::ViewMeshPlugin(std::string project_folder)
        : project_folder_(std::move(project_folder)) {}

void ViewMeshPlugin::init(igl::opengl::glfw::Viewer* _viewer) {
    ViewerPlugin::init(_viewer);

    // Append mesh for viewer mesh
    viewer->append_mesh();
    VIEWER_DATA_MESH = static_cast<unsigned int>(viewer->data_list.size() - 1);
}

bool ViewMeshPlugin::post_draw() {
    float window_width = 350.0f;
    ImGui::SetNextWindowSize(ImVec2(window_width, 0), ImGuiCond_Always);
    ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiCond_FirstUseEver);
    ImGui::Begin("Prikazovalnik", nullptr, ImGuiWindowFlags_NoSavedSettings);

    if (ImGui::TreeNodeEx("Odpri", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::InputText("Ime modela", mesh_name_, 128, ImGuiInputTextFlags_AutoSelectAll);
        if (ImGui::Button("Odpri (PLY, OBJ)", ImVec2(-1, 0))) {
            load_mesh_callback();
        }

        ImGui::InputText("Ime kvalitete", quality_name_, 128, ImGuiInputTextFlags_AutoSelectAll);
        if (ImGui::Button("Nalozi kvaliteto", ImVec2(-1, 0))) {
            load_quality_callback();
        }
        if(ImGui::Button("Glajenje kvalitete", ImVec2(-1, 0))) {
            smooth_quality_callback();
        }

        if (ImGui::Checkbox("Viden model", &visible_mesh_)) {
            show_mesh(visible_mesh_);
        }
        if (ImGui::Checkbox("Vidna tekstura", &visible_texture_)) {
            viewer->selected_data_index = VIEWER_DATA_MESH;
            viewer->data().show_texture = visible_texture_;
        }
        if (ImGui::Checkbox("Vidni robovi", &visible_wireframe_)) {
            viewer->selected_data_index = VIEWER_DATA_MESH;
            viewer->data().show_lines = visible_wireframe_;
        }
        ImGui::TreePop();
    }

    ImGui::End();
    return false;
}

void ViewMeshPlugin::load_mesh_callback() {
    std::cout << std::endl;

    // Load mesh to MVS format
    std::string tmp(mesh_name_);
    std::string fullpath = project_folder_ + tmp;
    mvs_mesh_.Release();
    mvs_mesh_.Load(fullpath);

    mvs_mesh_.ListIncidenteVertices();
    mvs_mesh_.ListIncidenteFaces();

    // Set viewer mesh
    set_mesh(mvs_mesh_);
    show_mesh(true);

    std::cout << "Viewer: Scene initialized, loaded from: \n\t" << fullpath << std::endl;
}

void ViewMeshPlugin::load_quality_callback() {

    // Open file
    std::string tmp(quality_name_);
    std::string fullpath = project_folder_ + tmp;
    std::ifstream infile(fullpath);

    // Read data
    quality_data_.clear();
    std::string line;
    double number;
    while (std::getline(infile, line)) {
        std::istringstream stream(line);
        while (stream) {
            if (stream >> number) {
                quality_data_.push_back(number);
            }
        }
    }
    infile.close();

    set_mesh_color();
    std::cout << "Viewer: Quality loaded from: \n\t" << fullpath << std::endl;
}

void ViewMeshPlugin::smooth_quality_callback() {

    // Compute face areas
    std::vector<double> fa = this->face_area();

    // Smooth quality data
    std::vector<double> smooth_quality_data_;
    int num_faces = mvs_mesh_.faces.size();
    for (int i = 0; i < num_faces; i++) {
        MVS::Mesh::FaceIdxArr neighbors;
        mvs_mesh_.GetFaceFaces(i, neighbors);

        double neighbors_area_sum = fa[i];
        for (const auto& n : neighbors) {
            neighbors_area_sum += fa[n];
        }

        double smooth_quality = (fa[i] / neighbors_area_sum) * quality_data_[i];
        for (const auto& n : neighbors) {
            smooth_quality += (fa[n] / neighbors_area_sum) * quality_data_[n];
        }

        smooth_quality_data_.push_back(smooth_quality);
    }

    // Set smooth quality color
    quality_data_ = smooth_quality_data_;
    set_mesh_color();
}

std::vector<double> ViewMeshPlugin::face_area() {
    int num_faces = mvs_mesh_.faces.size();
    std::vector<double> fa(num_faces);
    for (int i = 0; i < num_faces; i++) {

        const auto& face = mvs_mesh_.faces[i];
        const auto& a = mvs_mesh_.vertices[face[0]];
        const auto& b = mvs_mesh_.vertices[face[1]];
        const auto& c = mvs_mesh_.vertices[face[2]];

        // Compute face area
        double ab_x = b.x - a.x;
        double ab_y = b.y - a.y;
        double ab_z = b.z - a.z;

        double ac_x = c.x - a.x;
        double ac_y = c.y - a.y;
        double ac_z = c.z - a.z;

        double face_area = 0.5 * sqrt(
                pow(ab_y * ac_z - ab_z * ac_y , 2.0) +
                pow(ab_z * ac_x - ab_x * ac_z , 2.0) +
                pow(ab_x * ac_y - ab_y * ac_x , 2.0));

        fa[i] = face_area;
    }
    return fa;
}

void ViewMeshPlugin::set_mesh(const MVS::Mesh& mvs_mesh) {
    viewer->selected_data_index = VIEWER_DATA_MESH;
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

    // Add faces
    int num_faces = mvs_mesh.faces.size();
    Eigen::MatrixXi F(num_faces, 3);
    for (int i = 0; i < num_faces; i++) {
        MVS::Mesh::Face face = mvs_mesh.faces[i];
        F(i, 0) = face[0];
        F(i, 1) = face[1];
        F(i, 2) = face[2];
    }
    viewer->data().set_mesh(V, F);
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
        viewer->data().show_texture = visible_texture_;
    }

    center_object();
}

void ViewMeshPlugin::show_mesh(bool visible) {
    visible_mesh_ = visible;
    viewer->selected_data_index = VIEWER_DATA_MESH;
    viewer->data().show_faces = visible;
}

void ViewMeshPlugin::center_object() {
    viewer->selected_data_index = VIEWER_DATA_MESH;
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

void ViewMeshPlugin::set_mesh_color() {

    // Set color
    viewer->selected_data_index = VIEWER_DATA_MESH;
    assert(viewer->data().F.rows() == quality_data_.size());
    auto num_faces = viewer->data().F.rows();
    Eigen::VectorXd measure(num_faces);
    for (int i = 0; i < num_faces; i++) {
        measure(i) = quality_data_[i];
    }

    Eigen::MatrixXd color;
    igl::jet(measure, true, color);
    viewer->data().set_colors(color);
}

// Mouse IO
bool ViewMeshPlugin::mouse_down(int button, int modifier) {
    ImGui_ImplGlfwGL3_MouseButtonCallback(viewer->window, button, GLFW_PRESS, modifier);
    return ImGui::GetIO().WantCaptureMouse;
}

bool ViewMeshPlugin::mouse_up(int button, int modifier) {
    return ImGui::GetIO().WantCaptureMouse;
}

bool ViewMeshPlugin::mouse_move(int mouse_x, int mouse_y) {
    return ImGui::GetIO().WantCaptureMouse;
}

bool ViewMeshPlugin::mouse_scroll(float delta_y) {
    ImGui_ImplGlfwGL3_ScrollCallback(viewer->window, 0.f, delta_y);
    return ImGui::GetIO().WantCaptureMouse;
}

// Keyboard IO
bool ViewMeshPlugin::key_pressed(unsigned int key, int modifiers) {
    ImGui_ImplGlfwGL3_CharCallback(nullptr, key);
    return ImGui::GetIO().WantCaptureKeyboard;
}

bool ViewMeshPlugin::key_down(int key, int modifiers) {
    ImGui_ImplGlfwGL3_KeyCallback(viewer->window, key, 0, GLFW_PRESS, modifiers);
    return ImGui::GetIO().WantCaptureKeyboard;
}

bool ViewMeshPlugin::key_up(int key, int modifiers) {
    ImGui_ImplGlfwGL3_KeyCallback(viewer->window, key, 0, GLFW_RELEASE, modifiers);
    return ImGui::GetIO().WantCaptureKeyboard;
}