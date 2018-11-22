#include <optim.hpp>
#include <imguizmo/ImGuizmo.h>

struct OptimData {
    NextBestView* nbv;
    unsigned int image_width;
    unsigned int image_height;
    double focal_y;
};

double optim_function(const arma::vec& vals_inp, arma::vec* grad_out, void* opt_data) {
    auto optim_data = static_cast<OptimData*>(opt_data);

    // Create view matrix
    auto position_world = glm::vec3(vals_inp(0), vals_inp(1), vals_inp(2));
    auto euler_angles_world = glm::vec3(vals_inp(3), vals_inp(4), vals_inp(5));

    glm::mat4 view_matrix;
    glm::vec3 scale = glm::vec3(1.0, 1.0, 1.0);
    ImGuizmo::RecomposeMatrixFromComponents(
            glm::value_ptr(position_world),
            glm::value_ptr(euler_angles_world),
            glm::value_ptr(scale),
            glm::value_ptr(view_matrix));
    view_matrix = glm::inverse(view_matrix);

    // Call cost function
    double cost = optim_data->nbv->CostFunctionPosition(
            view_matrix,
            optim_data->image_width,
            optim_data->image_height,
            optim_data->focal_y);

    return cost;
}

struct OptimPosData {
    NextBestView* nbv;
    glm::vec3 euler_angles;
    unsigned int image_width;
    unsigned int image_height;
    double focal_y;
};

double optim_pos_function(const arma::vec& vals_inp, arma::vec* grad_out, void* opt_data) {
    auto optim_data = static_cast<OptimPosData*>(opt_data);

    // Create view matrix
    auto position_world = glm::vec3(vals_inp(0), vals_inp(1), vals_inp(2));
    auto euler_angles_world = optim_data->euler_angles;

    glm::mat4 view_matrix;
    glm::vec3 scale = glm::vec3(1.0, 1.0, 1.0);
    ImGuizmo::RecomposeMatrixFromComponents(
            glm::value_ptr(position_world),
            glm::value_ptr(euler_angles_world),
            glm::value_ptr(scale),
            glm::value_ptr(view_matrix));
    view_matrix = glm::inverse(view_matrix);

    // Call cost function
    double cost = optim_data->nbv->CostFunctionPosition(
            view_matrix,
            optim_data->image_width,
            optim_data->image_height,
            optim_data->focal_y);

    // Set gradient
    if (grad_out) {
        double h = 0.1;
        double dcost;
        glm::vec3 tmp_param;
        glm::mat4 tmp_view;

        tmp_param = glm::vec3(vals_inp(0) + h, vals_inp(1), vals_inp(2));
        ImGuizmo::RecomposeMatrixFromComponents(
                glm::value_ptr(tmp_param),
                glm::value_ptr(euler_angles_world),
                glm::value_ptr(scale),
                glm::value_ptr(tmp_view));
        tmp_view = glm::inverse(tmp_view);
        dcost = optim_data->nbv->CostFunctionPosition(
                tmp_view,
                optim_data->image_width,
                optim_data->image_height,
                optim_data->focal_y);
        (*grad_out)(0) = (dcost - cost) / h;

        tmp_param = glm::vec3(vals_inp(0), vals_inp(1) + h, vals_inp(2));
        ImGuizmo::RecomposeMatrixFromComponents(
                glm::value_ptr(tmp_param),
                glm::value_ptr(euler_angles_world),
                glm::value_ptr(scale),
                glm::value_ptr(tmp_view));
        tmp_view = glm::inverse(tmp_view);
        dcost = optim_data->nbv->CostFunctionPosition(
                tmp_view,
                optim_data->image_width,
                optim_data->image_height,
                optim_data->focal_y);
        (*grad_out)(1) = (dcost - cost) / h;

        tmp_param = glm::vec3(vals_inp(0), vals_inp(1), vals_inp(2) + h);
        ImGuizmo::RecomposeMatrixFromComponents(
                glm::value_ptr(tmp_param),
                glm::value_ptr(euler_angles_world),
                glm::value_ptr(scale),
                glm::value_ptr(tmp_view));
        tmp_view = glm::inverse(tmp_view);
        dcost = optim_data->nbv->CostFunctionPosition(
                tmp_view,
                optim_data->image_width,
                optim_data->image_height,
                optim_data->focal_y);
        (*grad_out)(2) = (dcost - cost) / h;
    }

    return cost;
}

struct OptimRotData {
    NextBestView* nbv;
    glm::vec3 position;
    unsigned int image_width;
    unsigned int image_height;
    double focal_y;
};

double optim_rot_function(const arma::vec& vals_inp, arma::vec* grad_out, void* opt_data) {
    auto optim_data = static_cast<OptimRotData*>(opt_data);

    // Create view matrix
    auto position_world = optim_data->position;
    auto euler_angles_world = glm::vec3(vals_inp(0), vals_inp(1), vals_inp(2));

    glm::mat4 view_matrix;
    glm::vec3 scale = glm::vec3(1.0, 1.0, 1.0);
    ImGuizmo::RecomposeMatrixFromComponents(
            glm::value_ptr(position_world),
            glm::value_ptr(euler_angles_world),
            glm::value_ptr(scale),
            glm::value_ptr(view_matrix));
    view_matrix = glm::inverse(view_matrix);

    // Call cost function
    double cost = optim_data->nbv->CostFunctionRotation(
            view_matrix,
            optim_data->image_width,
            optim_data->image_height,
            optim_data->focal_y);

    // Set gradient
    if (grad_out) {
        double h = 1; // represents 1 deg
        double dcost;
        glm::vec3 tmp_param;
        glm::mat4 tmp_view;

        tmp_param = glm::vec3(vals_inp(0) + h, vals_inp(1), vals_inp(2));
        ImGuizmo::RecomposeMatrixFromComponents(
                glm::value_ptr(position_world),
                glm::value_ptr(tmp_param),
                glm::value_ptr(scale),
                glm::value_ptr(tmp_view));
        tmp_view = glm::inverse(tmp_view);
        dcost = optim_data->nbv->CostFunctionRotation(
                tmp_view,
                optim_data->image_width,
                optim_data->image_height,
                optim_data->focal_y);
        (*grad_out)(0) = (dcost - cost) / h;

        tmp_param = glm::vec3(vals_inp(0), vals_inp(1) + h, vals_inp(2));
        ImGuizmo::RecomposeMatrixFromComponents(
                glm::value_ptr(position_world),
                glm::value_ptr(tmp_param),
                glm::value_ptr(scale),
                glm::value_ptr(tmp_view));
        tmp_view = glm::inverse(tmp_view);
        dcost = optim_data->nbv->CostFunctionRotation(
                tmp_view,
                optim_data->image_width,
                optim_data->image_height,
                optim_data->focal_y);
        (*grad_out)(1) = (dcost - cost) / h;

        tmp_param = glm::vec3(vals_inp(0), vals_inp(1), vals_inp(2) + h);
        ImGuizmo::RecomposeMatrixFromComponents(
                glm::value_ptr(position_world),
                glm::value_ptr(tmp_param),
                glm::value_ptr(scale),
                glm::value_ptr(tmp_view));
        tmp_view = glm::inverse(tmp_view);
        dcost = optim_data->nbv->CostFunctionRotation(
                tmp_view,
                optim_data->image_width,
                optim_data->image_height,
                optim_data->focal_y);
        (*grad_out)(2) = (dcost - cost) / h;
    }

    return cost;
}
