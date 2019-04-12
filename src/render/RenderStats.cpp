#include "RenderStats.h"

#include <iostream>
#include <fstream>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Geometry>

int RenderStats::Size() {
    if (render_poses.size() != estimated_poses.size()) {
        return 0;
    } else {
        return render_poses.size();
    }
}

glm::mat4 RenderStats::GetRenderPose(int index) {
    return render_poses[index];
}

glm::mat4 RenderStats::GetEstimatedPose(int index) {
    return estimated_poses[index];
}

void RenderStats::AddPose(int pose_id, const glm::mat4& render_pose,
        const glm::mat4& estimated_pose, int best_view_pick) {
    pose_ids.push_back(pose_id);
    render_poses.push_back(render_pose);
    estimated_poses.push_back(estimated_pose);
    best_view_picks.push_back(best_view_pick);
}

glm::mat4 RenderStats::ComputeTransformation() {
    assert(render_poses.size() == estimated_poses.size());
    assert(render_poses.size() >= 3);

    auto num_poses = render_poses.size();

    // Convert positions to eigen
    std::vector<Eigen::Vector3f> ren_pos;
    std::vector<Eigen::Vector3f> est_pos;
    for (int i = 0; i < num_poses; i++) {
        glm::mat4 r = glm::inverse(render_poses[i]);
        Eigen::Vector3f r_eig(r[3][0], r[3][1], r[3][2]);

        glm::mat4 e = glm::inverse(estimated_poses[i]);
        Eigen::Vector3f e_eig(e[3][0], e[3][1], e[3][2]);

        ren_pos.push_back(r_eig);
        est_pos.push_back(e_eig);
    }

    // Compute centers
    Eigen::Vector3f ren_center = Eigen::Vector3f::Zero();
    Eigen::Vector3f est_center = Eigen::Vector3f::Zero();
    for(size_t i = 0; i < num_poses; ++ i) {
        ren_center += ren_pos[i];
        est_center += est_pos[i];
    }
    ren_center /= num_poses;
    est_center /= num_poses;

    // Compute covariance matrix
    double ren_sd = 0;
    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for(size_t i = 0; i < num_poses; ++ i) {
        Eigen::Vector3f ren_tmp = ren_pos[i] - ren_center;
        Eigen::Vector3f est_tmp = est_pos[i] - est_center;

        ren_sd += ren_tmp.squaredNorm();

        // Accumulate covariance
        cov.noalias() += ren_tmp * est_tmp.transpose();
    }

    // Compute SVD
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);

    // Compute the rotation
    Eigen::Matrix3f R = svd.matrixV() * svd.matrixU().transpose();

    // Compute determinant of V*U^T to disambiguate rotation sign
    double det = R.determinant();
    Eigen::Vector3f e(1, 1, (det < 0) ? -1 : 1);

    // Recompute the rotation part if the determinant was negative
    if(det < 0) {
        R.noalias() = svd.matrixV() * e.asDiagonal() * svd.matrixU().transpose();
    }

    // Renormalize the rotation (not needed but gives slightly more orthogonal transformations)
    R = Eigen::Quaternionf(R).normalized().toRotationMatrix();

    // Compute final scale, rotation and translation
    double S = svd.singularValues().dot(e) / ren_sd;
    R *= S;
    Eigen::Vector3f t = est_center - (R * ren_center);

    // Combine transformation
    Eigen::Affine3f transform_eig = Eigen::Affine3f::Identity();
    transform_eig.linear() = R;
    transform_eig.translation() = t;
    glm::mat4 transform_glm = glm::make_mat4(transform_eig.matrix().data());

    return transform_glm;
}

double RenderStats::ComputeMSE(glm::mat4 transform) {
    auto num_poses = render_poses.size();
    double mse = 0;
    for (int i = 0; i < num_poses; i++) {
        glm::mat4 ren_world = glm::inverse(render_poses[i]);
        glm::vec4 ren_pos = ren_world[3];
        glm::vec4 ren_pos_new = transform * ren_pos;

        glm::mat4 est_world = glm::inverse(estimated_poses[i]);
        glm::vec4 est_pos = est_world[3];

        glm::vec3 diff = ren_pos_new - est_pos;
        mse += glm::dot(diff, diff);
    }
    mse = mse / num_poses;
    return mse;
}

void RenderStats::WriteStatsToFile(std::string filename) {
    std::ofstream outfile(filename);

    if (!outfile) {
        std::cout << "RenderStats: cannot open file " << filename << " for writing." << std::endl;
        return;
    }

    auto num_poses = render_poses.size();
    for (int i = 0; i < num_poses; i++) {
        outfile << "View id: " << pose_ids[i] << "\n";
        outfile << "Render view mat: \n" << glm::to_string(render_poses[i]) << "\n";
        outfile << "Estimated view mat: \n" << glm::to_string(estimated_poses[i]) << "\n";
        outfile << "Best view pick: " << best_view_picks[i] << "\n\n";
    }

    glm::mat4 transform = ComputeTransformation();
    outfile << "Transformation (render -> estimated in world coordinates): \n" << glm::to_string(transform) << "\n";

    double mse = ComputeMSE(transform);
    outfile << "Mean squared error: " << mse << "\n";
}

