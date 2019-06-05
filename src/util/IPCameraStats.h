#ifndef REALTIME_RECONSTRUCTION_IPCAMERASTATS_H
#define REALTIME_RECONSTRUCTION_IPCAMERASTATS_H

#include <vector>
#include <string>

#include <glm/glm.hpp>

class IPCameraStats {
public:
    // Utility functions
    bool Valid();
    int Size();
    glm::vec3 GetCameraPosition(int index) const;
    glm::vec3 GetCameraRotation(int index) const;

    glm::vec3 GetNBVPosition(int index) const;
    glm::vec3 GetNBVRotation(int index) const;

    // Add new pose pair
    void AddPose(int pose_id, const glm::vec3& camera_pos, const glm::vec3& camera_rot,
                 const glm::vec3& nbv_pos, const glm::vec3& nbv_rot, int best_view_pick);

    // Compute mean error (obsolete)
    double ComputeMeanError();

    // Compute mean pos error
    double ComputeMeanPosError();

    // Compute mean rot error
    double ComputeMeanRotError();

    // Compute mean camera distance
    double ComputeMeanCameraDistance();

    // Compute max camera distance
    double ComputeMaxCameraDistance();

    // Write to file
    void WriteStatsToFile(std::string filename);

private:
    // Poses are in view coordinates
    std::vector<int> pose_ids;

    std::vector<glm::vec3> camera_positions;
    std::vector<glm::vec3> camera_rotations;
    std::vector<glm::vec3> nbv_positions;
    std::vector<glm::vec3> nbv_rotations;

    std::vector<int> best_view_picks;
    std::vector<double> pos_errors;
    std::vector<double> rot_errors;
};

#endif //REALTIME_RECONSTRUCTION_IPCAMERASTATS_H
