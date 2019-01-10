#ifndef REALTIME_RECONSTRUCTION_RENDERSTATS_H
#define REALTIME_RECONSTRUCTION_RENDERSTATS_H

#include <vector>
#include <string>

#include <glm/glm.hpp>

class RenderStats {
public:
    // Utility functions
    int Size();
    glm::mat4 GetRenderPose(int index);
    glm::mat4 GetEstimatedPose(int index);

    // Add new pose pair
    void AddPose(const glm::mat4& render_pose, const glm::mat4& estimated_pose, int best_view_pick);

    // Compute transformation from render to estimated poses in world coordinates
    glm::mat4 ComputeTransformation();

    // Compute mean squared error
    double ComputeMSE(glm::mat4 transform);

    // Write to file
    void WriteStatsToFile(std::string filename);

private:
    // Poses are in view coordinates
    std::vector<glm::mat4> render_poses;
    std::vector<glm::mat4> estimated_poses;
    std::vector<int> best_view_picks;
};


#endif //REALTIME_RECONSTRUCTION_RENDERSTATS_H
