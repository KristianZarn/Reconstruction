// Return points (eigen matrix #V x 3)
Eigen::MatrixXd GetReconstructedPoints();

// Return point colors (eigen matrix #V x 3)
Eigen::MatrixXd GetPointColors();

// Return camera positions (eigen matrix #C x 3)
Eigen::MatrixXd GetCameraPositions();

Eigen::MatrixXd RealtimeReconstructionBuilder::GetReconstructedPoints() {
    std::unordered_set<TrackId> track_ids;
    GetEstimatedTracksFromReconstruction(*reconstruction_, &track_ids);

    auto num_points = static_cast<int>(track_ids.size());
    Eigen::MatrixXd points(num_points, 3);

    int i = 0;
    for (const auto& track_id : track_ids) {
        const Track* track = reconstruction_->Track(track_id);
        Eigen::Vector3d point = track->Point().hnormalized();
        points(i, 0) = point(0);
        points(i, 1) = point(1);
        points(i, 2) = point(2);
        i++;
    }
    return points;
}

Eigen::MatrixXd RealtimeReconstructionBuilder::GetPointColors() {
    std::unordered_set<TrackId> track_ids;
    GetEstimatedTracksFromReconstruction(*reconstruction_, &track_ids);

    auto num_points = static_cast<int>(track_ids.size());
    Eigen::MatrixXd colors(num_points, 3);

    int i = 0;
    for (const auto& track_id : track_ids) {
        const Track* track = reconstruction_->Track(track_id);
        Eigen::Matrix<uint8_t, 3, 1> color = track->Color();
        colors(i, 0) = color(0);
        colors(i, 1) = color(1);
        colors(i, 2) = color(2);
        i++;
    }
    return colors;
}

Eigen::MatrixXd RealtimeReconstructionBuilder::GetCameraPositions() {
    std::unordered_set<ViewId> view_ids;
    GetEstimatedViewsFromReconstruction(*reconstruction_, &view_ids);

    auto num_views = static_cast<int>(view_ids.size());
    Eigen::MatrixXd cameras(num_views, 3);

    int i = 0;
    for (const auto& view_id : view_ids) {
        Eigen::Vector3d position = reconstruction_->View(view_id)->Camera().GetPosition();
        cameras(i, 0) = position(0);
        cameras(i, 1) = position(1);
        cameras(i, 2) = position(2);
        i++;
    }
    return cameras;
}

// Read the mesh
Eigen::MatrixXd vertices;
Eigen::MatrixXi faces;
igl::readOFF("../assets/bunny.off", vertices, faces);

// Show mesh in viewer
viewer.data().set_mesh(vertices, faces);
viewer.data().set_vertices(vertices);
viewer.data().add_points(vertices, Eigen::RowVector3d(1,0,0));



std::string ReconstructionPlugin::image_fullpath(int image_idx) {
    std::stringstream ss;
    ss << std::setw(3) << std::setfill('0') << std::to_string(image_idx);
    std::string fullpath = images_path_ + "frame" + ss.str() + ".png";
    return fullpath;
}

// Save the final mesh
mvs_scene_.Save(mvs_scene_mesh_file);
mvs_scene_.mesh.Save(ply_file);

// Extra cleaning trying to close more holes
// mvs_scene.mesh.Clean(1.f, 0.f, remove_spikes, close_holes, 0, false);
// Extra cleaning to remove non-manifold problems created by closing holes
// mvs_scene.mesh.Clean(1.f, 0.f, false, 0, 0, true);

void ReconstructionPlugin::testing_callback() {
    std::string filename = "mesh.obj";
    mvs_scene_.mesh.Save(reconstruction_path_ + filename);

    Eigen::MatrixXd V;
    Eigen::MatrixXd TC;
    Eigen::MatrixXd N;
    Eigen::MatrixXi F;
    Eigen::MatrixXi FTC;
    Eigen::MatrixXi FN;
    igl::readOBJ(reconstruction_path_ + filename, V, TC, N, F, FTC, FN);

    viewer->data().show_texture = true;
    viewer->data().clear();
    viewer->data().set_mesh(V, F);
    viewer->data().set_uv(TC, FTC);

    // DEBUG
    std::cout << "Top of mesh.faceTexcoords " << std::endl;
    for (int i = 0; i < 5; i++) {
        MVS::Mesh::TexCoord tmp = mvs_scene_.mesh.faceTexcoords[i];
        std::cout << tmp[0] << ", " << tmp[1] << std::endl;
    }


    std::cout << "Top of TC and FTC matrices: " << std::endl;
    std::cout << TC.topRows(5) << std::endl;
    std::cout << FTC.topRows(5) << std::endl;

    std::string tex_filename = reconstruction_path_ + "mesh_material_0_map_Kd.jpg";
    theia::FloatImage img(tex_filename);

    int width = img.Width();
    int height = img.Height();
    Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic> R(width, height);
    Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic> G(width, height);
    Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic> B(width, height);

    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            R(i, (height - 1) - j) = static_cast<unsigned char>(img.GetXY(i, j, 0) * 255);
            G(i, (height - 1) - j) = static_cast<unsigned char>(img.GetXY(i, j, 1) * 255);
            B(i, (height - 1) - j) = static_cast<unsigned char>(img.GetXY(i, j, 2) * 255);
        }
    }

    viewer->data().set_colors(Eigen::RowVector3d(1, 1, 1));
    viewer->data().set_texture(R, G, B);
}


// Add cameras
auto num_views = static_cast<int>(mvs_scene_.platforms.front().poses.size());
Eigen::MatrixXd cameras(num_views, 3);

int i = 0;
for (const auto& pose : mvs_scene_.platforms.front().poses) {
Eigen::Vector3d position = pose.C;
cameras(i, 0) = position(0);
cameras(i, 1) = position(1);
cameras(i, 2) = position(2);

// Add camera label
viewer->data().add_label(position, std::to_string(i));
i++;
}
viewer->data().add_points(cameras, Eigen::RowVector3d(0, 1, 0));

// UI
float w = ImGui::GetContentRegionAvailWidth();
float p = ImGui::GetStyle().FramePadding.x;
if (ImGui::Button("Point cloud [1]", ImVec2((w - p) / 2.f, 0))) {
show_point_cloud();
hide_mesh();
}
ImGui::SameLine(0, p);
if (ImGui::Button("Mesh [2]", ImVec2((w - p) / 2.f, 0))) {
show_mesh();
hide_point_cloud();
}

// Debug info
std::ostringstream os;
// os << "View: \n" << viewer->core.view << std::endl;
// os << "Norm: \n" << viewer->core.norm << std::endl;
// os << "Proj: \n" << viewer->core.proj << std::endl;

os << "camera_base_translation: \n" << viewer->core.camera_base_translation << std::endl;
os << "camera_translation: \n" << viewer->core.camera_translation << std::endl;
os << "camera_center: \n" << viewer->core.camera_center << std::endl;

os << "camera_base_zoom: \n" << viewer->core.camera_base_zoom << std::endl;
os << "camera_zoom: \n" << viewer->core.camera_zoom << std::endl;

// Eigen::Quaternionf quat = viewer->core.trackball_angle;
// os << "trackball_angle: \n" << quat.w() << ", " << quat.x() << ", " << quat.y() << ", " << quat.z() << std::endl;
ImGui::TextUnformatted(os.str().c_str());

// Eigen::MatrixXd colors = Eigen::MatrixXd::Constant(viewer->data().F.rows(), 3, 1);

log_stream_ << "Before: " << std::endl;
log_stream_ << "Vertices: " << mvs_scene_.mesh.vertices.size() << std::endl;
log_stream_ << "Faces: " << mvs_scene_.mesh.faces.size() << std::endl;
log_stream_ << "vertex normals: " << mvs_scene_.mesh.vertexNormals.size() << std::endl;
log_stream_ << "vertex vertices: " << mvs_scene_.mesh.vertexVertices.size() << std::endl;
log_stream_ << "vertex faces: " << mvs_scene_.mesh.vertexFaces.size() << std::endl;
log_stream_ << "vertex boundary: " << mvs_scene_.mesh.vertexBoundary.size() << std::endl;
log_stream_ << "face normals: " << mvs_scene_.mesh.faceNormals.size() << std::endl;
log_stream_ << "face texcoords: " << mvs_scene_.mesh.faceTexcoords.size() << std::endl;

/*void EditMeshPlugin::remove_selection_callback() {
    viewer->selected_data_index = VIEWER_DATA_MESH_EDIT;

    // Remove faces
    MVS::Mesh::FaceIdxArr faces;
    for (const auto& i : selected_faces_idx_) {
        MVS::Mesh::FIndex& tmp = faces.AddEmpty();
        tmp = (MVS::Mesh::FIndex) i;
    }
    mvs_scene_.mesh.RemoveFaces(faces, true);
    selected_faces_idx_.clear();

    // Reset mesh in viewer
    set_mesh(mvs_scene_);
    show_mesh(true);
}*/

void EditMeshPlugin::remove_selection_callback() {
    // Get vertices from faces
    viewer->selected_data_index = VIEWER_DATA_MESH_EDIT;
    std::unordered_set<int> selected_vertices_idx;
    for (const auto& i : selected_faces_idx_) {
        Eigen::Vector3i face = viewer->data().F.row(i);
        selected_vertices_idx.insert(face(0));
        selected_vertices_idx.insert(face(1));
        selected_vertices_idx.insert(face(2));
    }

    // Remove vertices
    MVS::Mesh::VertexIdxArr vertices;
    for (const auto& i : selected_vertices_idx) {
        MVS::Mesh::VIndex& tmp = vertices.AddEmpty();
        tmp = (MVS::Mesh::VIndex) i;
    }

    mvs_scene_.mesh.ListIncidenteFaces();
    mvs_scene_.mesh.RemoveVertices(vertices, true);
    mvs_scene_.mesh.ListIncidenteFaces();
    selected_faces_idx_.clear();

    // Reset mesh in viewer
    set_mesh(mvs_scene_);
    show_mesh(true);
    set_bounding_box();
    set_plane();
}

// Debug
if (ImGui::Button("Debug", ImVec2(-1, 0))) {
std::cout << "Debug button" << std::endl;
std::ostringstream debug;
debug << "View: \n" << viewer->core.view << std::endl;
debug << "camera_base_translation: \n" << viewer->core.camera_base_translation << std::endl;
debug << "camera_translation: \n" << viewer->core.camera_translation << std::endl;
debug << "camera_center: \n" << viewer->core.camera_center << std::endl;
debug << "camera_base_zoom: \n" << viewer->core.camera_base_zoom << std::endl;
debug << "camera_zoom: \n" << viewer->core.camera_zoom << std::endl;
debug << "object_scale: \n" << viewer->core.object_scale << std::endl;
ImGui::TextUnformatted(debug.str().c_str());
}

void EditMeshPlugin::decimate_callback() {
    viewer->selected_data_index = VIEWER_DATA_MESH_EDIT;
    Eigen::MatrixXd vertices = viewer->data().V;
    Eigen::MatrixXi faces = viewer->data().F;
    Eigen::MatrixXd V_dec;
    Eigen::MatrixXi F_dec;
    Eigen::VectorXi F_idx;
    Eigen::VectorXi V_idx;
    igl::decimate(vertices, faces, static_cast<const size_t>(parameters_.decimate_mesh + 1), V_dec, F_dec, F_idx, V_idx);

    log_stream_ << "V_dec size: " << V_dec.rows() << std::endl;
    log_stream_ << "V_dec: \n" << V_dec.topRows(10) << std::endl;

    log_stream_ << "F_dec size: " << F_dec.rows() << std::endl;
    log_stream_ << "F_dec: \n" << F_dec.topRows(10) << std::endl;

    log_stream_ << "F_idx size: " << F_idx.rows() << std::endl;
    log_stream_ << "F_idx: \n" << F_idx.topRows(10) << std::endl;

    log_stream_ << "V_idx size: " << V_idx.rows() << std::endl;
    log_stream_ << "V_idx: \n" << V_idx.topRows(10) << std::endl;

    // Set UVs
    Eigen::MatrixXd TC = viewer->data().V_uv;
    Eigen::MatrixXd TC_dec(F_idx.size() * 3, 2);
    for (int i = 0; i < F_idx.size(); i++) {
        std::cout << "F_idx(i): " << F_idx(i) << std::endl;
        std::cout << TC.row(F_idx(i) * 3 + 0) << std::endl;
        TC_dec.row(i * 3 + 0) = TC.row(F_idx(i) * 3 + 0);
        TC_dec.row(i * 3 + 1) = TC.row(F_idx(i) * 3 + 1);
        TC_dec.row(i * 3 + 2) = TC.row(F_idx(i) * 3 + 2);
    }
    Eigen::MatrixXi FTC(F_idx.size(), 3);
    for (int i = 0; i < F_idx.size(); i++) {
        FTC(i, 0) = i*3 + 0;
        FTC(i, 1) = i*3 + 1;
        FTC(i, 2) = i*3 + 2;
    }

    viewer->data().clear();
    viewer->data().set_mesh(V_dec, F_dec);
    viewer->data().set_uv(TC_dec, FTC);

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

// Symetric matching for pose estimation
// Compute matches
ImagePairMatch pair_match;
pair_match.image1 = features_camera.image_name;
pair_match.image2 = features_match.image_name;

std::vector<IndexedFeatureMatch> putative_matches;
const double lowes_ratio = feature_matcher_->options_.lowes_ratio;
feature_matcher_->cascade_hasher_->MatchImages(hashed_camera, features_camera.descriptors,
hashed_match, features_match.descriptors,
lowes_ratio, &putative_matches);



// Symmetric matching
std::vector<IndexedFeatureMatch> backwards_matches;
feature_matcher_->cascade_hasher_->MatchImages(hashed_match, features_match.descriptors,
hashed_camera, features_camera.descriptors,
lowes_ratio, &backwards_matches);
IntersectMatches(backwards_matches, &putative_matches);

// Geometric verification
feature_matcher_->GeometricVerification(features_camera, features_match, putative_matches, &pair_match);

// Get normalized 2D 3D matches
const Camera& camera = reconstruction_->View(match_view_id)->Camera();
std::vector<FeatureCorrespondence2D3D> pose_match;
for (const auto& match_correspondence : pair_match.correspondences) {

const auto match_feature = std::make_pair(match_view_id, match_correspondence.feature2);
TrackId track_id = image_feature_to_track_id_[match_feature];
const Track* track = reconstruction_->Track(track_id);
if (!track->IsEstimated()) {
continue;
}

FeatureCorrespondence2D3D pose_correspondence;
pose_correspondence.feature = camera.PixelToNormalizedCoordinates(match_correspondence.feature1).hnormalized();
pose_correspondence.world_point = track->Point().hnormalized();
pose_match.emplace_back(pose_correspondence);
}

void ReconstructionPlugin::gsd_callback() {
    quality_measure_->updateMesh();

    // Compute measure
    std::vector<double> gsd = quality_measure_->groundSamplingDistance();

    // Set color
    if (!gsd.empty()) {
        viewer->selected_data_index = VIEWER_DATA_MESH;
        assert(viewer->data().F.rows() == gsd.size());
        auto num_faces = viewer->data().F.rows();
        Eigen::VectorXd measure(num_faces);
        for (int i = 0; i < num_faces; i++) {
            measure(i) = gsd[i];
        }

        Eigen::MatrixXd color;
        igl::jet(measure, true, color);
        viewer->data().set_colors(color);
    }
}

void ReconstructionPlugin::dor_callback() {
    quality_measure_->updateMesh();

    // Compute measure
    std::vector<unsigned int> dor = quality_measure_->degreeOfRedundancy();

    // Set color
    if (!dor.empty()) {
        viewer->selected_data_index = VIEWER_DATA_MESH;
        assert(viewer->data().F.rows() == dor.size());
        auto num_faces = viewer->data().F.rows();
        Eigen::VectorXd measure(num_faces);
        for (int i = 0; i < num_faces; i++) {
            measure(i) = dor[i];
        }

        Eigen::MatrixXd color;
        igl::jet(measure, true, color);
        viewer->data().set_colors(color);
    }
}

void ReconstructionPlugin::fa_callback() {
    quality_measure_->updateMesh();

    // Compute measure
    std::vector<double> fa = quality_measure_->faceArea();

    // Set color
    if (!fa.empty()) {
        viewer->selected_data_index = VIEWER_DATA_MESH;
        assert(viewer->data().F.rows() == fa.size());
        auto num_faces = viewer->data().F.rows();
        Eigen::VectorXd measure(num_faces);
        for (int i = 0; i < num_faces; i++) {
            measure(i) = fa[i];
        }

        Eigen::MatrixXd color;
        igl::jet(measure, true, color);
        viewer->data().set_colors(color);
    }
}

std::vector<glm::mat4> NextBestView::BestViews(const std::vector<std::vector<unsigned int>>& clusters,
                                               const std::vector<double>& face_values) {
    // Compute cluster costs
    std::vector<std::pair<int, double>> cluster_costs;
    for (int i = 0; i < clusters.size(); i++) {
        double cluster_cost = 0.0;
        for (const auto& face_id : clusters[i]) {
            cluster_cost += face_values[face_id];
        }
        cluster_costs.emplace_back(i, cluster_cost);
    }

    // Sort descending
    std::sort(cluster_costs.begin(), cluster_costs.end(), [](auto &left, auto &right) {
        return left.second > right.second;
    });

    // Keep only top N clusters
    if (cluster_costs.size() > max_views_) {
        cluster_costs.resize(max_views_);
    }

    // Average centers and normals
    std::vector<glm::vec3> centers;
    std::vector<glm::vec3> normals;
    for (const auto& pair : cluster_costs) {
        const std::vector<unsigned int>& cluster = clusters[pair.first];

        glm::vec3 center_sum(0.0f);
        glm::vec3 normal_sum(0.0f);
        for (const auto& face_id : cluster) {
            center_sum += face_centers_[face_id];
            normal_sum += face_normals_[face_id];
        }

        centers.emplace_back(center_sum / static_cast<float>(cluster.size()));
        normals.emplace_back(normal_sum / static_cast<float>(cluster.size()));
    }

    // Generate view matrix

}

double NextBestView::AverageCameraDistance() {

    int count = 0;
    double distance_sum = 0.0;

    int num_cameras = mvs_scene_->images.size();
    for (int camera_idx = 0; camera_idx < num_cameras; camera_idx++) {

        // Camera parameters
        unsigned int image_width = mvs_scene_->images[camera_idx].width;
        unsigned int image_height = mvs_scene_->images[camera_idx].height;
        double focal_y = mvs_scene_->images[camera_idx].camera.K(1,1);

        // Camera view matrix
        const auto& R = mvs_scene_->images[camera_idx].camera.R; // world to view (view coordinates)
        const auto& T = mvs_scene_->images[camera_idx].camera.C; // world coordinates

        glm::mat3 view_R(R(0,0), R(0,1), R(0,2),
                         -R(1,0), -R(1,1), -R(1,2),
                         -R(2,0), -R(2,1), -R(2,2));
        glm::vec3 view_T(T.x, T.y, T.z);

        glm::mat4 tmp_R = glm::mat4(view_R);
        glm::mat4 tmp_T = glm::translate(glm::mat4(1.0f), view_T);
        glm::mat4 view_matrix = glm::inverse(tmp_T * tmp_R);

        // Visible faces
        double downscale_factor = 8.0;
        auto visible_faces = VisibleFaces(
                view_matrix,
                static_cast<int>(image_width / downscale_factor),
                static_cast<int>(image_height / downscale_factor),
                focal_y / downscale_factor);

        // Average distance
        for (const auto& face_id : visible_faces) {
            glm::vec3 face_center = face_centers_[face_id];
            distance_sum += glm::distance(face_center, view_T);
            count++;
        }
    }

    return (distance_sum / count);
}