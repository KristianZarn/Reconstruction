#include "EditMeshPlugin.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui_impl_glfw_gl3.h>
#include <igl/unproject_onto_mesh.h>
#include <igl/qslim.h>
#include <igl/decimate.h>

EditMeshPlugin::EditMeshPlugin(Parameters parameters,
                               std::string reconstruction_path)
        : parameters_(parameters),
          reconstruction_path_(std::move(reconstruction_path)) {}

void EditMeshPlugin::init(igl::opengl::glfw::Viewer *_viewer) {
    ViewerPlugin::init(_viewer);

    // Append mesh for mesh editing
    viewer->append_mesh();
    VIEWER_DATA_MESH_EDIT = static_cast<unsigned int>(viewer->data_list.size() - 1);

    // Append mesh for bounding box
    viewer->append_mesh();
    VIEWER_DATA_BOUNDING_BOX = static_cast<unsigned int>(viewer->data_list.size() - 1);

    // Append mesh for plane
    viewer->append_mesh();
    VIEWER_DATA_PLANE = static_cast<unsigned int>(viewer->data_list.size() - 1);

    // Initial gizmo pose
    bounding_box_gizmo_ = Eigen::Matrix4f::Identity();
    plane_gizmo_ = Eigen::Matrix4f::Identity();
}

bool EditMeshPlugin::pre_draw() {
    ImGuizmo::BeginFrame();
    return false;
}

bool EditMeshPlugin::post_draw() {
    // Setup window
    float window_width = 350.0f;
    ImGui::SetNextWindowSize(ImVec2(window_width, 0), ImGuiCond_Always);
    ImGui::SetNextWindowPos(ImVec2(350.0f, 0.0f), ImGuiCond_FirstUseEver);
    ImGui::Begin("Edit mesh", nullptr, ImGuiWindowFlags_NoSavedSettings);

    // Input output
    if (ImGui::TreeNodeEx("Input / Output", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::InputText("Filename", parameters_.filename_buffer, 64, ImGuiInputTextFlags_AutoSelectAll);
        ImGui::Spacing();
        if (ImGui::Button("Save mesh (MVS)", ImVec2(-1, 0))) {
            log_stream_ << std::endl;

            std::string filename_mvs = std::string(parameters_.filename_buffer) + ".mvs";
            mvs_scene_.Save(reconstruction_path_ + filename_mvs);
            log_stream_ << "Written to: \n\t" << (reconstruction_path_ + filename_mvs) << std::endl;

            std::string filename_ply = std::string(parameters_.filename_buffer) + ".ply";
            mvs_scene_.mesh.Save(reconstruction_path_ + filename_ply);
            log_stream_ << "Written to: \n\t" << (reconstruction_path_ + filename_ply) << std::endl;
        }
        if (ImGui::Button("Load mesh (MVS)", ImVec2(-1, 0))) {
            log_stream_ << std::endl;

            std::string filename_mvs = std::string(parameters_.filename_buffer) + ".mvs";
            mvs_scene_.Load(reconstruction_path_ + filename_mvs);
            set_mesh(mvs_scene_);
            center_object_callback();
            show_mesh(true);
            set_bounding_box();
            set_plane();
            log_stream_ << "Loaded from: \n\t" << (reconstruction_path_ + filename_mvs) << std::endl;
        }
        if (ImGui::Button("Reset mesh", ImVec2(-1, 0))) {
            reset_mesh_callback();
        }
        std::ostringstream os;
        os << "Mesh info:"
           << "\t" << mvs_scene_.mesh.vertices.GetSize() << " vertices"
           << "\t" << mvs_scene_.mesh.faces.GetSize() << " faces";
        ImGui::TextUnformatted(os.str().c_str());
        ImGui::TreePop();
    }

    // Display options
    if (ImGui::TreeNodeEx("Display options", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::Button("Center object", ImVec2(-1, 0))) {
            center_object_callback();
        }
        if (ImGui::Checkbox("Show mesh", &parameters_.show_mesh)) {
            show_mesh(parameters_.show_mesh);
        }
        if (ImGui::Checkbox("Show texture", &parameters_.show_texture)) {
            viewer->selected_data_index = VIEWER_DATA_MESH_EDIT;
            viewer->data().show_texture = parameters_.show_texture;
        }
        if (ImGui::Checkbox("Show wireframe", &parameters_.show_wireframe)) {
            viewer->selected_data_index = VIEWER_DATA_MESH_EDIT;
            viewer->data().show_lines = parameters_.show_wireframe;
        }
        ImGui::TreePop();
    }

    // Selection
    if (ImGui::TreeNodeEx("Selection", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::RadioButton("Pick faces [s]", parameters_.selection_mode == SelectionMode::PICK)) {
            parameters_.selection_mode = SelectionMode::PICK;
        }
        if (ImGui::RadioButton("Bounding box", parameters_.selection_mode == SelectionMode::BOX)) {
            parameters_.selection_mode = SelectionMode::BOX;
        }
        if (ImGui::RadioButton("Plane", parameters_.selection_mode == SelectionMode::PLANE)) {
            parameters_.selection_mode = SelectionMode::PLANE;
        }
        show_bounding_box(parameters_.selection_mode == SelectionMode::BOX);
        show_plane(parameters_.selection_mode == SelectionMode::PLANE);

        // Gizmo setup
        ImGuiIO& io = ImGui::GetIO();
        ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);
        Eigen::Affine3f scale_base_zoom(Eigen::Scaling(1.0f / viewer->core.camera_base_zoom));
        Eigen::Affine3f scale_zoom(Eigen::Scaling(1.0f / viewer->core.camera_zoom));
        Eigen::Matrix4f view = scale_base_zoom * scale_zoom * viewer->core.view;

        // Selection: Bounding box
        if (parameters_.selection_mode == SelectionMode::BOX) {
            ImGui::Text("Bounding box options:");
            gizmo_options();
            if (ImGui::Button("Select inside", ImVec2(-1, 0))) {
                select_inside_callback();
            }
            // Show gizmo
            ImGuizmo::Manipulate(view.data(),
                                 viewer->core.proj.data(),
                                 parameters_.gizmo_operation,
                                 parameters_.gizmo_mode,
                                 bounding_box_gizmo_.data());
            transform_bounding_box();
        }

        // Selection: Plane
        if (parameters_.selection_mode == SelectionMode::PLANE) {
            ImGui::Text("Plane options:");
            gizmo_options();
            if (ImGui::Button("Select below", ImVec2(-1, 0))) {
                select_faces_below_callback();
            }
            // Show gizmo
            ImGuizmo::Manipulate(view.data(),
                                 viewer->core.proj.data(),
                                 parameters_.gizmo_operation,
                                 parameters_.gizmo_mode,
                                 plane_gizmo_.data());
            transform_plane();
        }
        ImGui::TreePop();
    }

    // Modify
    if (ImGui::TreeNodeEx("Modify", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::Button("Invert selection", ImVec2(-1, 0))) {
            invert_selection_callback();
        }
        if (ImGui::Button("Remove selection", ImVec2(-1, 0))) {
            remove_selection_callback();
        }
        if (ImGui::Button("Fit plane to selection", ImVec2(-1, 0))) {
            fit_plane_callback();
        }
        ImGui::PushItemWidth(150.0f);
        ImGui::InputInt("##decimate", &parameters_.decimate_target);
        ImGui::PopItemWidth();
        ImGui::SameLine();
        if (ImGui::Button("Decimate", ImVec2(-1, 0))) {
            decimate_callback();
        }
        ImGui::PushItemWidth(150.0f);
        ImGui::InputInt("##resize", &parameters_.texture_size);
        ImGui::PopItemWidth();
        ImGui::SameLine();
        if (ImGui::Button("Resize texture", ImVec2(-1, 0))) {
            // TODO: resize texture
        }
        ImGui::PushItemWidth(150.0f);
        ImGui::InputInt("##fill", &parameters_.fill_hole_size);
        ImGui::PopItemWidth();
        ImGui::SameLine();
        if (ImGui::Button("Fill holes", ImVec2(-1, 0))) {
            fill_holes_callback();
        }
        ImGui::TreePop();
    }

    // Debug
    if (ImGui::TreeNodeEx("Debug")) {
        if (ImGui::Button("Debug", ImVec2(-1, 0))) {
            std::cout << "Debug button" << std::endl;
            // std::ostringstream debug;
            // ImGui::TextUnformatted(debug.str().c_str());
            viewer->selected_data_index = VIEWER_DATA_MESH_EDIT;
            log_stream_ << "Vertices: " << viewer->data().V.rows() << std::endl;
            log_stream_ << "Faces: " << viewer->data().F.rows() << std::endl;
            log_stream_ << "V_uv: " << viewer->data().V_uv.rows() << " "
                        << mvs_scene_.mesh.faceTexcoords.size() << std::endl;
            log_stream_ << "F_uv: " << viewer->data().F_uv.rows() << std::endl;

            log_stream_ << "test: V_uv == 3 * faces" << std::endl;
            log_stream_ << "test: F_uv == faces" << std::endl;
        }
        ImGui::TreePop();
    }

    ImGui::End();
    return false;
}

void EditMeshPlugin::gizmo_options() {
    if (ImGui::RadioButton("Translate", parameters_.gizmo_operation == ImGuizmo::TRANSLATE))
        parameters_.gizmo_operation = ImGuizmo::TRANSLATE;
    ImGui::SameLine();
    if (ImGui::RadioButton("Rotate", parameters_.gizmo_operation == ImGuizmo::ROTATE))
        parameters_.gizmo_operation = ImGuizmo::ROTATE;
    ImGui::SameLine();
    if (ImGui::RadioButton("Scale", parameters_.gizmo_operation == ImGuizmo::SCALE))
        parameters_.gizmo_operation = ImGuizmo::SCALE;

    if (parameters_.gizmo_operation != ImGuizmo::SCALE) {
        if (ImGui::RadioButton("Local", parameters_.gizmo_mode == ImGuizmo::LOCAL))
            parameters_.gizmo_mode = ImGuizmo::LOCAL;
        ImGui::SameLine();
        if (ImGui::RadioButton("World", parameters_.gizmo_mode == ImGuizmo::WORLD))
            parameters_.gizmo_mode = ImGuizmo::WORLD;
    }
}

void EditMeshPlugin::reset_mesh_callback() {
    log_stream_ << std::endl;
    // Reset reconstruction
    mvs_scene_.Release();
    // Reset viewer data
    viewer->selected_data_index = VIEWER_DATA_MESH_EDIT;
    viewer->data().clear();
    viewer->selected_data_index = VIEWER_DATA_BOUNDING_BOX;
    viewer->data().clear();
    viewer->selected_data_index = VIEWER_DATA_PLANE;
    viewer->data().clear();
    // Reset selection
    selected_faces_idx_.clear();
    // Reset gizmos
    bounding_box_gizmo_ = Eigen::Matrix4f::Identity();
    plane_gizmo_ = Eigen::Matrix4f::Identity();
    log_stream_ << "Mesh reset" << std::endl;
}

void EditMeshPlugin::center_object_callback() {
    viewer->selected_data_index = VIEWER_DATA_MESH_EDIT;
    if (viewer->data().V.rows() > 0) {
        Eigen::Vector3d min_point = viewer->data().V.colwise().minCoeff();
        Eigen::Vector3d max_point = viewer->data().V.colwise().maxCoeff();
        Eigen::Vector3d center = viewer->data().V.colwise().mean();
        viewer->core.camera_base_translation = (-center).cast<float>();
        viewer->core.camera_translation.setConstant(0);

        Eigen::Vector3d diff = (max_point-min_point).array().abs();
        viewer->core.camera_base_zoom = static_cast<float>(2.0 / diff.maxCoeff());
        viewer->core.camera_zoom = 1.0;
    }
}

void EditMeshPlugin::select_inside_callback() {
    viewer->selected_data_index = VIEWER_DATA_BOUNDING_BOX;
    Eigen::MatrixXd V_box = viewer->data().V;
    Eigen::Vector3d u = V_box.row(4) - V_box.row(0);
    Eigen::Vector3d v = V_box.row(2) - V_box.row(0);
    Eigen::Vector3d w = V_box.row(1) - V_box.row(0);

    // Check which points are in bounding box
    viewer->selected_data_index = VIEWER_DATA_MESH_EDIT;
    std::unordered_set<int> selected_vertices_idx;
    for (int i = 0; i < viewer->data().V.rows(); i++) {
        Eigen::Vector3d x = viewer->data().V.row(i);

        double ux = u.dot(x);
        double vx = v.dot(x);
        double wx = w.dot(x);

        if (ux > u.dot(V_box.row(0)) && ux < u.dot(V_box.row(4)) &&
            vx > v.dot(V_box.row(0)) && vx < v.dot(V_box.row(2)) &&
            wx > w.dot(V_box.row(0)) && wx < w.dot(V_box.row(1))) {
            selected_vertices_idx.insert(i);
        }
    }

    // Transform to selected faces
    selected_faces_idx_.clear();
    for (int i = 0; i < viewer->data().F.rows(); i++) {
        Eigen::Vector3i f = viewer->data().F.row(i);

        if (selected_vertices_idx.find(f(0)) != selected_vertices_idx.end() ||
            selected_vertices_idx.find(f(1)) != selected_vertices_idx.end() ||
            selected_vertices_idx.find(f(2)) != selected_vertices_idx.end()) {
            selected_faces_idx_.insert(i);
        }
    }

    // Set colors of selected faces
    color_selection();
}

void EditMeshPlugin::select_faces_below_callback() {
    viewer->selected_data_index = VIEWER_DATA_PLANE;
    Eigen::MatrixXd V_plane = viewer->data().V;
    Eigen::Vector3d u = V_plane.row(3) - V_plane.row(0);
    Eigen::Vector3d v = V_plane.row(1) - V_plane.row(0);

    Eigen::Vector3d plane_normal = u.cross(v).normalized();
    Eigen::Vector3d plane_center = V_plane.colwise().mean();

    // Check which points are below plane
    viewer->selected_data_index = VIEWER_DATA_MESH_EDIT;
    std::unordered_set<int> selected_vertices_idx;
    for (int i = 0; i < viewer->data().V.rows(); i++) {
        Eigen::Vector3d point = viewer->data().V.row(i);

        if (plane_normal.dot(point - plane_center) < 0) {
            selected_vertices_idx.insert(i);
        }
    }

    // Transform to selected faces
    selected_faces_idx_.clear();
    for (int i = 0; i < viewer->data().F.rows(); i++) {
        Eigen::Vector3i f = viewer->data().F.row(i);

        if (selected_vertices_idx.find(f(0)) != selected_vertices_idx.end() ||
            selected_vertices_idx.find(f(1)) != selected_vertices_idx.end() ||
            selected_vertices_idx.find(f(2)) != selected_vertices_idx.end()) {
            selected_faces_idx_.insert(i);
        }
    }

    // Set colors of selected faces
    color_selection();
}

void EditMeshPlugin::invert_selection_callback() {
    viewer->selected_data_index = VIEWER_DATA_MESH_EDIT;
    std::unordered_set<int> selection_inverse;
    for (int i = 0; i < viewer->data().F.rows(); i++) {
        if (selected_faces_idx_.find(i) == selected_faces_idx_.end()) {
            selection_inverse.insert(i);
        }
    }
    selected_faces_idx_ = selection_inverse;
    color_selection();
}

void EditMeshPlugin::remove_selection_callback() {
    // Get vertices from faces
    std::unordered_set<int> selected_vertices_idx;
    for (const auto& i : selected_faces_idx_) {
        MVS::Mesh::Face face = mvs_scene_.mesh.faces[i];
        selected_vertices_idx.insert(face.x);
        selected_vertices_idx.insert(face.y);
        selected_vertices_idx.insert(face.z);
    }

    // Remove faces
    MVS::Mesh::FaceIdxArr faces_to_remove;
    for (const auto& i : selected_faces_idx_) {
        MVS::Mesh::FIndex& tmp = faces_to_remove.AddEmpty();
        tmp = (MVS::Mesh::FIndex) i;
    }
    mvs_scene_.mesh.RemoveFaces(faces_to_remove, false);
    mvs_scene_.mesh.ListIncidenteFaces();

    // Remove vertices without incident faces
    MVS::Mesh::VertexIdxArr vertices_to_remove;
    for (const auto& i : selected_vertices_idx) {
        if (mvs_scene_.mesh.vertexFaces[i].IsEmpty()) {
            MVS::Mesh::VIndex& tmp = vertices_to_remove.AddEmpty();
            tmp = (MVS::Mesh::VIndex) i;
        }
    }
    mvs_scene_.mesh.RemoveVertices(vertices_to_remove, true);
    mvs_scene_.mesh.ListIncidenteFaces();
    selected_faces_idx_.clear();

    // Reset mesh in viewer
    set_mesh(mvs_scene_);
    show_mesh(true);
    set_bounding_box();
    set_plane();
}

void EditMeshPlugin::fit_plane_callback() {
    if (!selected_faces_idx_.empty()) {
        viewer->selected_data_index = VIEWER_DATA_MESH_EDIT;

        // Set vertex mean as plane center
        Eigen::MatrixXd vertices(selected_faces_idx_.size() * 3, 3);
        int i = 0;
        for (const auto& idx : selected_faces_idx_) {
            Eigen::RowVector3i face = viewer->data().F.row(idx);
            vertices.row(i++) = viewer->data().V.row(face(0));
            vertices.row(i++) = viewer->data().V.row(face(1));
            vertices.row(i++) = viewer->data().V.row(face(2));
        }
        Eigen::Vector3d plane_center = vertices.colwise().mean();
        Eigen::Affine3f translation(Eigen::Translation3f(plane_center.cast<float>()));

        // Set face normals mean as plane normal
        Eigen::MatrixXd face_normals(selected_faces_idx_.size(), 3);
        i = 0;
        for (const auto& idx : selected_faces_idx_) {
            face_normals.row(i++) = viewer->data().F_normals.row(idx);
        }
        Eigen::Vector3d plane_normal = face_normals.colwise().mean();
        Eigen::Vector3f initial_normal = Eigen::Vector3f::UnitZ();
        Eigen::Affine3f rotation(Eigen::Quaternionf().setFromTwoVectors(initial_normal, plane_normal.cast<float>()));

        // Apply transform to gizmo
        plane_gizmo_ = (translation * rotation).matrix();
        parameters_.selection_mode = SelectionMode::PLANE;
    } else {
        log_stream_ << std::endl;
        log_stream_ << "Fit plane failed: selection is empty." << std::endl;
    }
}

void EditMeshPlugin::decimate_callback() {
    if (parameters_.decimate_target < mvs_scene_.mesh.faces.size()) {
        viewer->selected_data_index = VIEWER_DATA_MESH_EDIT;
        Eigen::MatrixXd vertices = viewer->data().V;
        Eigen::MatrixXi faces = viewer->data().F;
        Eigen::MatrixXd V_dec;
        Eigen::MatrixXi F_dec;
        Eigen::VectorXi F_idx;
        Eigen::VectorXi V_idx;
        igl::decimate(vertices, faces, static_cast<size_t>(parameters_.decimate_target), V_dec, F_dec, F_idx, V_idx);

        // Set MVS mesh
        MVS::Mesh::VertexArr vertices_dec;
        for (int i = 0; i < V_dec.rows(); i++) {
            Eigen::RowVector3d vertex = V_dec.row(i);
            MVS::Mesh::Vertex& tmp = vertices_dec.AddEmpty();
            tmp.x = static_cast<MVS::Mesh::Type>(vertex(0));
            tmp.y = static_cast<MVS::Mesh::Type>(vertex(1));
            tmp.z = static_cast<MVS::Mesh::Type>(vertex(2));
        }
        mvs_scene_.mesh.vertices.Release();
        mvs_scene_.mesh.vertices = vertices_dec;

        MVS::Mesh::FaceArr faces_dec;
        for (int i = 0; i < F_dec.rows(); i++) {
            Eigen::RowVector3i face = F_dec.row(i);
            MVS::Mesh::Face& tmp = faces_dec.AddEmpty();
            tmp.x = static_cast<MVS::Mesh::VIndex>(face(0));
            tmp.y = static_cast<MVS::Mesh::VIndex>(face(1));
            tmp.z = static_cast<MVS::Mesh::VIndex>(face(2));
        }
        mvs_scene_.mesh.faces.Release();
        mvs_scene_.mesh.faces = faces_dec;

        mvs_scene_.mesh.ListIncidenteFaces();

        // Set UVs
        if (!mvs_scene_.mesh.faceTexcoords.IsEmpty()) {
            Eigen::MatrixXd TC = viewer->data().V_uv;
            MVS::Mesh::TexCoordArr tex_coords;
            for (int i = 0; i < F_idx.size(); i++) {
                MVS::Mesh::TexCoord& tc1 = tex_coords.AddEmpty();
                tc1.x = static_cast<MVS::Mesh::Type>(TC(F_idx(i) * 3, 0));
                tc1.y = static_cast<MVS::Mesh::Type>(TC(F_idx(i) * 3, 1));

                MVS::Mesh::TexCoord& tc2 = tex_coords.AddEmpty();
                tc2.x = static_cast<MVS::Mesh::Type>(TC(F_idx(i) * 3 + 1, 0));
                tc2.y = static_cast<MVS::Mesh::Type>(TC(F_idx(i) * 3 + 1, 1));

                MVS::Mesh::TexCoord& tc3 = tex_coords.AddEmpty();
                tc3.x = static_cast<MVS::Mesh::Type>(TC(F_idx(i) * 3 + 2, 0));
                tc3.y = static_cast<MVS::Mesh::Type>(TC(F_idx(i) * 3 + 2, 1));
            }
            mvs_scene_.mesh.faceTexcoords.Release();
            mvs_scene_.mesh.faceTexcoords = tex_coords;
        }

        set_mesh(mvs_scene_);
        set_bounding_box();
        set_plane();
        log_stream_ << std::endl;
        log_stream_ << "Decimate success: new number of faces is " << mvs_scene_.mesh.faces.size() << "." << std::endl;
    } else {
        log_stream_ << std::endl;
        log_stream_ << "Decimate failed: target number of faces has to be lower than current number." << std::endl;
    }
}

void EditMeshPlugin::fill_holes_callback() {
    // Clean the mesh
    mvs_scene_.mesh.Clean(1.0, 0.0, false, parameters_.fill_hole_size, 0, false);

    // Recompute array of vertices incident to each vertex
    mvs_scene_.mesh.ListIncidenteFaces();

    set_mesh(mvs_scene_);
    show_mesh(true);
}

void EditMeshPlugin::color_selection() {
    viewer->selected_data_index = VIEWER_DATA_MESH_EDIT;
    Eigen::MatrixXd colors(viewer->data().F.rows(), 3);
    for (int i = 0; i < viewer->data().F.rows(); i++) {
        colors.row(i) = parameters_.default_color;
    }
    // Eigen::MatrixXd colors = Eigen::MatrixXd::Constant(viewer->data().F.rows(), 3, 1);
    for (const auto& i : selected_faces_idx_) {
        colors.row(i) = Eigen::RowVector3d(1, 0, 0);
    }
    viewer->data().set_colors(colors);
}

void EditMeshPlugin::set_mesh(const MVS::Scene& mvs_scene) {
    // Select viewer data
    viewer->selected_data_index = VIEWER_DATA_MESH_EDIT;
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
    // Add faces
    int num_faces = mvs_scene.mesh.faces.size();
    Eigen::MatrixXi F(num_faces, 3);
    for (int i = 0; i < num_faces; i++) {
        MVS::Mesh::Face face = mvs_scene.mesh.faces[i];
        F(i, 0) = face[0];
        F(i, 1) = face[1];
        F(i, 2) = face[2];
    }
    viewer->data().set_mesh(V, F);
    viewer->data().set_colors(parameters_.default_color);
    viewer->data().show_lines = parameters_.show_wireframe;

    // Add texture if available
    if (!mvs_scene.mesh.faceTexcoords.IsEmpty()) {

        // Set UVs
        int num_texcoords = mvs_scene.mesh.faceTexcoords.size();
        Eigen::MatrixXd TC(num_texcoords, 2);
        for (int i = 0; i < num_texcoords; i++) {
            MVS::Mesh::TexCoord texcoord = mvs_scene.mesh.faceTexcoords[i];
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
        SEACAVE::Image8U3 img = mvs_scene.mesh.textureDiffuse;
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

void EditMeshPlugin::show_mesh(bool visible) {
    parameters_.show_mesh = visible;
    viewer->selected_data_index = VIEWER_DATA_MESH_EDIT;
    viewer->data().show_faces = visible;
    if (!visible) {
        parameters_.show_wireframe = visible;
        viewer->data().show_lines = parameters_.show_wireframe;
    }
}

void EditMeshPlugin::set_bounding_box() {
    viewer->selected_data_index = VIEWER_DATA_MESH_EDIT;

    // Check if mesh empty
    if (viewer->data().V.rows() < 0) {
        log_stream_ << std::endl;
        log_stream_ << "Set bounding box failed: Mesh is empty" << std::endl;
        return;
    }

    // Find bounding box limits for the mesh
    Eigen::Vector3d min = viewer->data().V.colwise().minCoeff();
    Eigen::Vector3d max = viewer->data().V.colwise().maxCoeff();
    Eigen::Vector3d center = (0.5 * (min + max));

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
    Eigen::Affine3f translation(Eigen::Translation3f(center.cast<float>()));
    bounding_box_gizmo_ = translation * Eigen::Matrix4f::Identity();

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
    if (bounding_box_vertices_.rows() > 0) {
        Eigen::MatrixXd V = bounding_box_vertices_;
        V = (V.rowwise().homogeneous() * bounding_box_gizmo_.cast<double>().transpose()).rowwise().hnormalized();
        viewer->data().set_vertices(V);
    }
}

void EditMeshPlugin::show_bounding_box(bool visible) {
    viewer->selected_data_index = VIEWER_DATA_BOUNDING_BOX;
    viewer->data().show_faces = visible;
}

void EditMeshPlugin::set_plane() {
    viewer->selected_data_index = VIEWER_DATA_MESH_EDIT;

    // Check if mesh empty
    if (viewer->data().V.rows() < 0) {
        log_stream_ << std::endl;
        log_stream_ << "Set plane failed: Mesh is empty" << std::endl;
        return;
    }

    // Find bounding box limits for the mesh
    Eigen::Vector3d min = viewer->data().V.colwise().minCoeff();
    Eigen::Vector3d max = viewer->data().V.colwise().maxCoeff();
    Eigen::Vector3d center = (0.5 * (min + max));

    // Set plane size
    Eigen::Vector3d diff = (max - min).array().abs();
    double size = diff.maxCoeff();

    // Vertices
    Eigen::MatrixXd tmp_V(4, 3);
    tmp_V << -size/2, -size/2, 0.0,
            -size/2, size/2, 0.0,
            size/2, size/2, 0.0,
            size/2, -size/2, 0.0;
    plane_vertices_ = tmp_V;

    // Faces
    Eigen::MatrixXi tmp_F(2, 3);
    tmp_F << 0, 1, 2,
            2, 3, 0;

    // Rotate around x by 180 degrees
    Eigen::Affine3f rotation;
    rotation = Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX()).matrix();
    plane_gizmo_ = rotation * Eigen::Matrix4f::Identity();

    // Center plane on object
    Eigen::Affine3f translation(Eigen::Translation3f(center.cast<float>()));
    plane_gizmo_ = translation * Eigen::Matrix4f::Identity();

    // Set viewer data
    viewer->selected_data_index = VIEWER_DATA_PLANE;
    viewer->data().clear();
    viewer->data().set_mesh(tmp_V, tmp_F);
    viewer->data().set_face_based(true);
    viewer->data().set_colors(Eigen::RowVector4d(0, 255, 0, 64)/255.0);
    viewer->data().show_lines = false;
}

void EditMeshPlugin::transform_plane() {
    viewer->selected_data_index = VIEWER_DATA_PLANE;
    if (plane_vertices_.rows() > 0) {
        Eigen::MatrixXd V = plane_vertices_;
        V = (V.rowwise().homogeneous() * plane_gizmo_.cast<double>().transpose()).rowwise().hnormalized();
        viewer->data().set_vertices(V);
    }
}

void EditMeshPlugin::show_plane(bool visible) {
    viewer->selected_data_index = VIEWER_DATA_PLANE;
    viewer->data().show_faces = visible;
}

// Mouse IO
bool EditMeshPlugin::mouse_down(int button, int modifier) {
    ImGui_ImplGlfwGL3_MouseButtonCallback(viewer->window, button, GLFW_PRESS, modifier);
    return ImGui::GetIO().WantCaptureMouse;
}

bool EditMeshPlugin::mouse_up(int button, int modifier) {
    return ImGui::GetIO().WantCaptureMouse;
}

bool EditMeshPlugin::mouse_move(int mouse_x, int mouse_y) {
    return ImGui::GetIO().WantCaptureMouse;
}

bool EditMeshPlugin::mouse_scroll(float delta_y) {
    ImGui_ImplGlfwGL3_ScrollCallback(viewer->window, 0.f, delta_y);
    return ImGui::GetIO().WantCaptureMouse;
}

// Keyboard IO
bool EditMeshPlugin::key_pressed(unsigned int key, int modifiers) {
    ImGui_ImplGlfwGL3_CharCallback(viewer->window, key);
    if (!ImGui::GetIO().WantTextInput) {
        switch (key) {
            case 's':
            {
                if (!ImGui::GetIO().WantCaptureMouse &&
                    (parameters_.selection_mode == SelectionMode::PICK)) {
                    viewer->selected_data_index = VIEWER_DATA_MESH_EDIT;
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
                        // Toggle face selection
                        if (selected_faces_idx_.find(face_id) == selected_faces_idx_.end()) {
                            selected_faces_idx_.insert(face_id);
                        } else {
                            selected_faces_idx_.erase(face_id);
                        }
                        // Set colors of selected faces
                        color_selection();
                        return true;
                    }
                }
                return ImGui::GetIO().WantCaptureKeyboard;
            }
            default: break;
        }
    }
    return ImGui::GetIO().WantCaptureKeyboard;
}

bool EditMeshPlugin::key_down(int key, int modifiers) {
    ImGui_ImplGlfwGL3_KeyCallback(viewer->window, key, 0, GLFW_PRESS, modifiers);
    return ImGui::GetIO().WantCaptureKeyboard;
}

bool EditMeshPlugin::key_up(int key, int modifiers) {
    ImGui_ImplGlfwGL3_KeyCallback(viewer->window, key, 0, GLFW_RELEASE, modifiers);
    return ImGui::GetIO().WantCaptureKeyboard;
}