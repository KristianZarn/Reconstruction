#include <optim.hpp>

glm::mat4 generate_view_matrix(const glm::vec3& position_world, const glm::vec3 euler_angles_world) {

    // Rotation
    glm::mat4 mat_roll = glm::mat4(1.0f);
    glm::mat4 mat_pitch = glm::mat4(1.0f);
    glm::mat4 mat_yaw = glm::mat4(1.0f);
    mat_pitch = glm::rotate(mat_pitch, euler_angles_world[0], glm::vec3(1.0f, 0.0f, 0.0f));
    mat_yaw = glm::rotate(mat_yaw, euler_angles_world[1], glm::vec3(0.0f, 1.0f, 0.0f));
    mat_roll = glm::rotate(mat_roll, euler_angles_world[2], glm::vec3(0.0f, 0.0f, 1.0f));
    glm::mat4 rotate = mat_yaw * mat_pitch * mat_roll;

    // Translation
    glm::mat4 translate = glm::mat4(1.0f);
    translate = glm::translate(translate, position_world);

    // View matrix
    return glm::inverse(translate * rotate);
}

struct OptimData {
    NextBestView* nbv;
    unsigned int image_width;
    unsigned int image_height;
    double focal_y;
};

double optim_function(const arma::vec& vals_inp, arma::vec* grad_out, void* opt_data) {

    // Create view matrix
    auto position_world = glm::vec3(vals_inp(0), vals_inp(1), vals_inp(2));
    auto euler_angles_world = glm::vec3(vals_inp(3), vals_inp(4), vals_inp(5));
    glm::mat4 view_matrix = generate_view_matrix(position_world, euler_angles_world);

    // Call cost function
    auto optim_data = static_cast<OptimData*>(opt_data);
    double cost = optim_data->nbv->CostFunction(
            view_matrix,
            optim_data->image_width,
            optim_data->image_height,
            optim_data->focal_y);

    // Set gradient
    if (grad_out) {
        double h = 0.01;
        double dcost;
        glm::vec3 tmp;

        tmp = glm::vec3(vals_inp(0) + h, vals_inp(1), vals_inp(2));
        dcost = optim_data->nbv->CostFunction(
                generate_view_matrix(tmp, euler_angles_world),
                optim_data->image_width,
                optim_data->image_height,
                optim_data->focal_y);
        (*grad_out)(0) = (dcost - cost) / h;

        tmp = glm::vec3(vals_inp(0), vals_inp(1) + h, vals_inp(2));
        dcost = optim_data->nbv->CostFunction(
                generate_view_matrix(tmp, euler_angles_world),
                optim_data->image_width,
                optim_data->image_height,
                optim_data->focal_y);
        (*grad_out)(1) = (dcost - cost) / h;

        tmp = glm::vec3(vals_inp(0), vals_inp(1), vals_inp(2) + h);
        dcost = optim_data->nbv->CostFunction(
                generate_view_matrix(tmp, euler_angles_world),
                optim_data->image_width,
                optim_data->image_height,
                optim_data->focal_y);
        (*grad_out)(2) = (dcost - cost) / h;

        tmp = glm::vec3(vals_inp(3) + h, vals_inp(4), vals_inp(5));
        dcost = optim_data->nbv->CostFunction(
                generate_view_matrix(position_world, tmp),
                optim_data->image_width,
                optim_data->image_height,
                optim_data->focal_y);
        (*grad_out)(3) = (dcost - cost) / h;

        tmp = glm::vec3(vals_inp(3), vals_inp(4) + h, vals_inp(5));
        dcost = optim_data->nbv->CostFunction(
                generate_view_matrix(position_world, tmp),
                optim_data->image_width,
                optim_data->image_height,
                optim_data->focal_y);
        (*grad_out)(4) = (dcost - cost) / h;

        tmp = glm::vec3(vals_inp(3), vals_inp(4), vals_inp(5) + h);
        dcost = optim_data->nbv->CostFunction(
                generate_view_matrix(position_world, tmp),
                optim_data->image_width,
                optim_data->image_height,
                optim_data->focal_y);
        (*grad_out)(5) = (dcost - cost) / h;
    }

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
    glm::mat4 view_matrix = generate_view_matrix(position_world, euler_angles_world);

    // Call cost function
    double cost = optim_data->nbv->CostFunctionPosition(
            view_matrix,
            optim_data->image_width,
            optim_data->image_height,
            optim_data->focal_y);

    // Set gradient
    if (grad_out) {
        double h = 0.05;
        double dcost;
        glm::vec3 tmp;

        tmp = glm::vec3(vals_inp(0) + h, vals_inp(1), vals_inp(2));
        dcost = optim_data->nbv->CostFunctionPosition(
                generate_view_matrix(tmp, euler_angles_world),
                optim_data->image_width,
                optim_data->image_height,
                optim_data->focal_y);
        (*grad_out)(0) = (dcost - cost) / h;

        tmp = glm::vec3(vals_inp(0), vals_inp(1) + h, vals_inp(2));
        dcost = optim_data->nbv->CostFunctionPosition(
                generate_view_matrix(tmp, euler_angles_world),
                optim_data->image_width,
                optim_data->image_height,
                optim_data->focal_y);
        (*grad_out)(1) = (dcost - cost) / h;

        tmp = glm::vec3(vals_inp(0), vals_inp(1), vals_inp(2) + h);
        dcost = optim_data->nbv->CostFunctionPosition(
                generate_view_matrix(tmp, euler_angles_world),
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
    glm::mat4 view_matrix = generate_view_matrix(position_world, euler_angles_world);

    // Call cost function
    double cost = optim_data->nbv->CostFunctionRotation(
            view_matrix,
            optim_data->image_width,
            optim_data->image_height,
            optim_data->focal_y);

    // Set gradient
    if (grad_out) {
        double h = 0.0175; // 1 deg in radians
        double dcost;
        glm::vec3 tmp;

        tmp = glm::vec3(vals_inp(0) + h, vals_inp(1), vals_inp(2));
        dcost = optim_data->nbv->CostFunctionRotation(
                generate_view_matrix(position_world, tmp),
                optim_data->image_width,
                optim_data->image_height,
                optim_data->focal_y);
        (*grad_out)(0) = (dcost - cost) / h;

        tmp = glm::vec3(vals_inp(0), vals_inp(1) + h, vals_inp(2));
        dcost = optim_data->nbv->CostFunctionRotation(
                generate_view_matrix(position_world, tmp),
                optim_data->image_width,
                optim_data->image_height,
                optim_data->focal_y);
        (*grad_out)(1) = (dcost - cost) / h;

        tmp = glm::vec3(vals_inp(0), vals_inp(1), vals_inp(2) + h);
        dcost = optim_data->nbv->CostFunctionRotation(
                generate_view_matrix(position_world, tmp),
                optim_data->image_width,
                optim_data->image_height,
                optim_data->focal_y);
        (*grad_out)(2) = (dcost - cost) / h;
    }

    return cost;
}
