#include "IPCameraStats.h"

#include <iostream>
#include <fstream>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_access.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <imguizmo/ImGuizmo.h>

bool IPCameraStats::Valid() {
    return (pose_ids.size() == camera_positions.size() &&
            pose_ids.size() == camera_rotations.size() &&
            pose_ids.size() == nbv_positions.size() &&
            pose_ids.size() == nbv_rotations.size() &&
            pose_ids.size() == best_view_picks.size() &&
            pose_ids.size() == pos_errors.size() &&
            pose_ids.size() == rot_errors.size());
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

    // Compute position error
    glm::vec3 diff = camera_pos - nbv_pos;
    double pos_error = glm::length(diff);
    pos_errors.push_back(pos_error);

    // Compute rotation error
    glm::vec3 scale = glm::vec3(1.0, 1.0, 1.0);

    glm::mat4 camera_world;
    ImGuizmo::RecomposeMatrixFromComponents(
            glm::value_ptr(camera_pos),
            glm::value_ptr(camera_rot),
            glm::value_ptr(scale),
            glm::value_ptr(camera_world));

    glm::mat4 nbv_world;
    ImGuizmo::RecomposeMatrixFromComponents(
            glm::value_ptr(nbv_pos),
            glm::value_ptr(nbv_rot),
            glm::value_ptr(scale),
            glm::value_ptr(nbv_world));

    glm::vec3 camera_front = -glm::column(camera_world, 2);
    glm::vec3 nbv_front = -glm::column(nbv_world, 2);
    double rot_error = glm::angle(camera_front, nbv_front);
    rot_errors.push_back(rot_error);
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

double IPCameraStats::ComputeMeanPosError() {
    double error_sum = 0.0;
    for (const auto& tmp : pos_errors) {
        error_sum += tmp;
    }
    return (error_sum / pos_errors.size());
}

double IPCameraStats::ComputeMeanRotError() {
    double error_sum = 0.0;
    for (const auto& tmp : rot_errors) {
        error_sum += tmp;
    }
    return (error_sum / rot_errors.size());
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

double IPCameraStats::ComputeMaxCameraDistance() {
    auto num_poses = this->Size();
    double max_distance = 0.0;
    for (int i = 0; i < num_poses; i++) {
        for (int j = i + 1; j < num_poses; j++) {
            glm::vec3 diff = camera_positions[i] - camera_positions[j];
            double distance = glm::length(diff);
            if (distance > max_distance) {
                max_distance = distance;
            }
        }
    }
    return max_distance;
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
        outfile << "Best view pick: " << best_view_picks[i] << "\n";
        outfile << "Pos error: " << pos_errors[i] << "\n";
        outfile << "Rot error: " << rot_errors[i] << "\n\n";
    }

    double mean_pos_error = ComputeMeanPosError();
    outfile << "Mean position error: " << mean_pos_error << "\n";

    double mean_rot_error = ComputeMeanRotError();
    outfile << "Mean rotation error: " << mean_rot_error << "\n";

    // double max_camera_distance = ComputeMaxCameraDistance();
    // outfile << "Max camera distance: " << max_camera_distance << "\n";
}
