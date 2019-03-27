#include <imguizmo/ImGuizmo.h>
#include <nelder_mead/nelder_mead.h>

struct OptimData {
    NextBestView* nbv;
    unsigned int image_width;
    unsigned int image_height;
    double focal_y;
    glm::vec3 cluster_center;
    glm::vec3 cluster_normal;
};

void optim_function(int n, point_t *point, const void *arg) {
    auto optim_data = static_cast<const OptimData*>(arg);

    // Create view matrix
    auto position_world = glm::vec3(point->x[0], point->x[1], point->x[2]);
    auto euler_angles_world = glm::vec3(point->x[3], point->x[4], point->x[5]);

    glm::mat4 view_matrix;
    glm::vec3 scale = glm::vec3(1.0, 1.0, 1.0);
    ImGuizmo::RecomposeMatrixFromComponents(
            glm::value_ptr(position_world),
            glm::value_ptr(euler_angles_world),
            glm::value_ptr(scale),
            glm::value_ptr(view_matrix));
    view_matrix = glm::inverse(view_matrix);

    // Call cost function
    double cost = optim_data->nbv->CostFunction(
            view_matrix,
            optim_data->image_height,
            optim_data->focal_y,
            optim_data->image_width,
            optim_data->cluster_center,
            optim_data->cluster_normal);

    point->fx = cost;
}
