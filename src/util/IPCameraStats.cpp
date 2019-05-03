#include "IPCameraStats.h"

#include <iostream>
#include <fstream>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>

bool IPCameraStats::Valid() {
    return (pose_ids.size() == camera_positions.size() &&
            pose_ids.size() == camera_rotations.size() &&
            pose_ids.size() == nbv_positions.size() &&
            pose_ids.size() == nbv_rotations.size() &&
            pose_ids.size() == best_view_picks.size());
}

int IPCameraStats::Size() {
    return pose_ids.size();
}

glm::vec3 IPCameraStats::GetCameraPosition(int index) const {
    return camera_positions[index];
}

glm::vec3 IPCameraStats::GetCameraRotation(int index) const {
    return camera_rotations[index];
}

glm::vec3 IPCameraStats::GetNBVPosition(int index) const {
    return nbv_positions[index];
}

glm::vec3 IPCameraStats::GetNBVRotation(int index) const {
    return nbv_rotations[index];
}

void IPCameraStats::AddPose(int pose_id, const glm::vec3& camera_pos, const glm::vec3& camera_rot,
        const glm::vec3& nbv_pos, const glm::vec3& nbv_rot, int best_view_pick) {
    pose_ids.push_back(pose_id);
    camera_positions.push_back(camera_pos);
    camera_rotations.push_back(camera_rot);
    nbv_positions.push_back(nbv_pos);
    nbv_rotations.push_back(nbv_rot);
    best_view_picks.push_back(best_view_pick);
}

double IPCameraStats::ComputeMeanError() {
    auto num_poses = this->Size();
    double error_sum = 0.0;
    for (int i = 0; i < num_poses; i++) {
        glm::vec3 diff = camera_positions[i] - nbv_positions[i];
        error_sum += glm::length(diff);
    }
    return error_sum / num_poses;
}

double IPCameraStats::ComputeMeanCameraDistance() {
    auto num_poses = this->Size();
    double distance_sum = 0.0;
    int distance_count = 0;
    for (int i = 0; i < num_poses; i++) {
        for (int j = i + 1; j < num_poses; j++) {
            glm::vec3 diff = camera_positions[i] - camera_positions[j];
            distance_sum += glm::length(diff);
            distance_count++;
        }
    }
    return (distance_sum / distance_count);
}

void IPCameraStats::WriteStatsToFile(std::string filename) {
    std::ofstream outfile(filename);

    if (!outfile) {
        std::cout << "IPCamera stats: cannot open file " << filename << " for writing." << std::endl;
        return;
    }

    auto num_poses = this->Size();
    for (int i = 0; i < num_poses; i++) {
        outfile << "View id: " << pose_ids[i] << "\n";
        outfile << "Camera pos: " << glm::to_string(camera_positions[i]) << "\n";
        outfile << "Camera rot: " << glm::to_string(camera_rotations[i]) << "\n";
        outfile << "NBV pos: " << glm::to_string(nbv_positions[i]) << "\n";
        outfile << "NBV rot: " << glm::to_string(nbv_rotations[i]) << "\n";
        outfile << "Best view pick: " << best_view_picks[i] << "\n\n";
    }

    double mean_error = ComputeMeanError();
    outfile << "Mean error: " << mean_error << "\n";

    double mean_camera_distance = ComputeMeanCameraDistance();
    outfile << "Mean camera distance: " << mean_camera_distance << "\n";

    double relative_error = mean_error / mean_camera_distance;
    outfile << "Relative error: " << relative_error << "\n";
}
