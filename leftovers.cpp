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