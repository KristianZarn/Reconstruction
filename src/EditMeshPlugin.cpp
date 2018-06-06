#include "EditMeshPlugin.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui_impl_glfw_gl3.h>

EditMeshPlugin::EditMeshPlugin(Parameters parameters,
                               const MVS::Scene &mvs_scene)
        : parameters_(parameters),
          mvs_scene_(mvs_scene) {}

void EditMeshPlugin::init(igl::opengl::glfw::Viewer *_viewer) {
    ViewerPlugin::init(_viewer);

    // Append mesh for mesh editing
    viewer->append_mesh();
    VIEWER_DATA_MESH_EDIT = static_cast<unsigned int>(viewer->data_list.size() - 1);

    // Append mesh for bounding box
    viewer->append_mesh();
    VIEWER_DATA_BOUNDING_BOX = static_cast<unsigned int>(viewer->data_list.size() - 1);

    // Initial gizmo pose
    bounding_box_gizmo_ = Eigen::Matrix4f::Identity();
}

bool EditMeshPlugin::pre_draw() {
    ImGuizmo::BeginFrame();
    return false;
}

bool EditMeshPlugin::post_draw() {
    // Setup window
    float window_width = 350.0f;
    ImGui::SetNextWindowSize(ImVec2(window_width, 0), ImGuiCond_Always);
    ImGui::SetNextWindowPos(ImVec2(270.0f, 0.0f), ImGuiCond_FirstUseEver);
    ImGui::Begin("Edit mesh", nullptr, ImGuiWindowFlags_NoSavedSettings);

    if (ImGui::Button("Start edit", ImVec2(-1, 0))) {
        start_edit_callback();
    }

    // Show mesh info
    std::ostringstream os;
    os << "Mesh info:"
       << "\t" << mvs_scene_edit_.mesh.vertices.GetSize() << " vertices"
       << "\t" << mvs_scene_edit_.mesh.faces.GetSize() << " faces";
    ImGui::TextUnformatted(os.str().c_str());

    ImGui::Spacing();

    if (parameters_.active_edit) {
        // Bounding box
        if (ImGui::Button("Pose bounding box", ImVec2(-1, 0))) {
            pose_bounding_box_callback();
        }

        if (parameters_.active_bounding_box) {
            if (ImGui::RadioButton("Translate", parameters_.gizmo_operation == ImGuizmo::TRANSLATE))
                parameters_.gizmo_operation = ImGuizmo::TRANSLATE;
            ImGui::SameLine();
            if (ImGui::RadioButton("Rotate", parameters_.gizmo_operation == ImGuizmo::ROTATE))
                parameters_.gizmo_operation = ImGuizmo::ROTATE;
            ImGui::SameLine();
            if (ImGui::RadioButton("Scale", parameters_.gizmo_operation == ImGuizmo::SCALE))
                parameters_.gizmo_operation = ImGuizmo::SCALE;
            float matrixTranslation[3], matrixRotation[3], matrixScale[3];
            ImGuizmo::DecomposeMatrixToComponents(bounding_box_gizmo_.data(), matrixTranslation, matrixRotation, matrixScale);
            ImGui::InputFloat3("Tr", matrixTranslation, 3);
            ImGui::InputFloat3("Rt", matrixRotation, 3);
            ImGui::InputFloat3("Sc", matrixScale, 3);
            ImGuizmo::RecomposeMatrixFromComponents(matrixTranslation, matrixRotation, matrixScale, bounding_box_gizmo_.data());

            if (parameters_.gizmo_operation != ImGuizmo::SCALE)
            {
                if (ImGui::RadioButton("Local", parameters_.gizmo_mode == ImGuizmo::LOCAL))
                    parameters_.gizmo_mode = ImGuizmo::LOCAL;
                ImGui::SameLine();
                if (ImGui::RadioButton("World", parameters_.gizmo_mode == ImGuizmo::WORLD))
                    parameters_.gizmo_mode = ImGuizmo::WORLD;
            }

            // Setup gizmo
            // Eigen::UniformScaling scale = Eigen::Scaling(0.33 * viewer->core.camera_base_zoom);
            // Eigen::Matrix4f view = scale.inverse() * viewer->core.view;
            Eigen::Matrix4f view = Eigen::Scaling(1.0f / viewer->core.camera_zoom) * viewer->core.view;

            ImGuiIO& io = ImGui::GetIO();
            ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);
            ImGuizmo::Manipulate(view.data(),
                                 viewer->core.proj.data(),
                                 parameters_.gizmo_operation,
                                 parameters_.gizmo_mode,
                                 bounding_box_gizmo_.data());
            // Debug info
            std::ostringstream os;
            os << "camera_base_zoom: \n" << viewer->core.camera_base_zoom << std::endl;
            os << "camera_zoom: \n" << viewer->core.camera_zoom << std::endl;
            ImGui::TextUnformatted(os.str().c_str());

            // Transform bounding box
            if (bounding_box_vertices_.rows() > 0) {
                transform_bounding_box();
            }

            if (ImGui::Button("Done posing", ImVec2(-1, 0))) {
                done_bounding_box_callback();
            }
        }
        ImGui::Spacing();
    }

    ImGui::End();
    return false;
}

void EditMeshPlugin::start_edit_callback() {
    log_stream_ << std::endl;

    // Copy mvs scene
    mvs_scene_edit_ = mvs_scene_;

    // Check if mesh empty
    if (mvs_scene_edit_.mesh.vertices.IsEmpty() || mvs_scene_edit_.mesh.faces.IsEmpty()) {
        log_stream_ << "Start edit failed: Mesh is empty" << std::endl;
    } else {
        // Set mesh
        set_mesh();
        show_mesh(true);
        set_bounding_box();
        show_bounding_box(false);
        parameters_.active_edit = true;
        log_stream_ << "Start edit successful: Mesh was copied for editing" << std::endl;
    }
}

void EditMeshPlugin::pose_bounding_box_callback() {
    show_bounding_box(true);
    parameters_.active_bounding_box = true;
}

void EditMeshPlugin::done_bounding_box_callback() {
    show_bounding_box(false);
    parameters_.active_bounding_box = false;
}

void EditMeshPlugin::set_mesh() {
    // Select viewer data
    viewer->selected_data_index = VIEWER_DATA_MESH_EDIT;
    viewer->data().clear();

    // Add vertices
    int num_vertices = mvs_scene_edit_.mesh.vertices.size();
    Eigen::MatrixXd V(num_vertices, 3);
    for (int i = 0; i < num_vertices; i++) {
        MVS::Mesh::Vertex vertex = mvs_scene_edit_.mesh.vertices[i];
        V(i, 0) = vertex[0];
        V(i, 1) = vertex[1];
        V(i, 2) = vertex[2];
    }
    // Add faces
    int num_faces = mvs_scene_edit_.mesh.faces.size();
    Eigen::MatrixXi F(num_faces, 3);
    for (int i = 0; i < num_faces; i++) {
        MVS::Mesh::Face face = mvs_scene_edit_.mesh.faces[i];
        F(i, 0) = face[0];
        F(i, 1) = face[1];
        F(i, 2) = face[2];
    }
    viewer->data().set_mesh(V, F);

    // Add texture if available
    if (!mvs_scene_edit_.mesh.faceTexcoords.IsEmpty()) {

        // Set UVs
        int num_texcoords = mvs_scene_edit_.mesh.faceTexcoords.size();
        Eigen::MatrixXd TC(num_texcoords, 2);
        for (int i = 0; i < num_texcoords; i++) {
            MVS::Mesh::TexCoord texcoord = mvs_scene_edit_.mesh.faceTexcoords[i];
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
        SEACAVE::Image8U3 img = mvs_scene_edit_.mesh.textureDiffuse;
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
}

void EditMeshPlugin::show_mesh(bool visible) {
    viewer->selected_data_index = VIEWER_DATA_MESH_EDIT;
    viewer->data().show_faces = visible;
}

void EditMeshPlugin::set_bounding_box() {
    // Find bounding box limits for the mesh
    viewer->selected_data_index = VIEWER_DATA_MESH_EDIT;
    Eigen::Vector3d min = viewer->data().V.colwise().minCoeff();
    Eigen::Vector3d max = viewer->data().V.colwise().maxCoeff();
    Eigen::Vector3f center = (0.5 * (min.cast<float>() + max.cast<float>()));

    min = min - center.cast<double>();
    max = max - center.cast<double>();

    // Vertices
    Eigen::MatrixXd tmp_V(8, 3);
    tmp_V << min(0), min(1), min(2),
            min(0), min(1), max(2),
            min(0), max(1), min(2),
            min(0), max(1), max(2),
            max(0), min(1), min(2),
            max(0), min(1), max(2),
            max(0), max(1), min(2),
            max(0), max(1), max(2);
    bounding_box_vertices_ = tmp_V;

    // Faces
    Eigen::MatrixXi tmp_F(12, 3);
    tmp_F << 0, 6, 4,
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

    // Center bounding box on object
    bounding_box_gizmo_.col(3).head<3>() = center;

    // Set viewer data
    viewer->selected_data_index = VIEWER_DATA_BOUNDING_BOX;
    viewer->data().clear();
    viewer->data().set_mesh(tmp_V, tmp_F);
    viewer->data().set_face_based(true);
    viewer->data().set_colors(Eigen::RowVector4d(0, 255, 0, 64)/255.0);
    viewer->data().show_lines = false;
}

void EditMeshPlugin::transform_bounding_box() {
    viewer->selected_data_index = VIEWER_DATA_BOUNDING_BOX;
    Eigen::MatrixXd V = bounding_box_vertices_;
    V = (V.rowwise().homogeneous() * bounding_box_gizmo_.cast<double>().transpose()).rowwise().hnormalized();
    viewer->data().set_vertices(V);
}

void EditMeshPlugin::show_bounding_box(bool visible) {
    viewer->selected_data_index = VIEWER_DATA_BOUNDING_BOX;
    viewer->data().show_faces = visible;
}

// Mouse IO
bool EditMeshPlugin::mouse_down(int button, int modifier)
{
    ImGui_ImplGlfwGL3_MouseButtonCallback(viewer->window, button, GLFW_PRESS, modifier);
    return ImGui::GetIO().WantCaptureMouse;
}

bool EditMeshPlugin::mouse_up(int button, int modifier)
{
    return ImGui::GetIO().WantCaptureMouse;
}

bool EditMeshPlugin::mouse_move(int mouse_x, int mouse_y)
{
    return ImGui::GetIO().WantCaptureMouse;
}

bool EditMeshPlugin::mouse_scroll(float delta_y)
{
    ImGui_ImplGlfwGL3_ScrollCallback(viewer->window, 0.f, delta_y);
    return ImGui::GetIO().WantCaptureMouse;
}

// Keyboard IO
bool EditMeshPlugin::key_pressed(unsigned int key, int modifiers)
{
    ImGui_ImplGlfwGL3_CharCallback(nullptr, key);
    return ImGui::GetIO().WantCaptureKeyboard;
}

bool EditMeshPlugin::key_down(int key, int modifiers)
{
    ImGui_ImplGlfwGL3_KeyCallback(viewer->window, key, 0, GLFW_PRESS, modifiers);
    return ImGui::GetIO().WantCaptureKeyboard;
}

bool EditMeshPlugin::key_up(int key, int modifiers)
{
    ImGui_ImplGlfwGL3_KeyCallback(viewer->window, key, 0, GLFW_RELEASE, modifiers);
    return ImGui::GetIO().WantCaptureKeyboard;
}