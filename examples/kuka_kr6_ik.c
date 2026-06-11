#include <stdio.h>

#define CQUIK_IMPLEMENTATION
#include "cquik.h"

static void print_vector(const char *label, const double *values, int count)
{
    int i;

    printf("%s [", label);
    for (i = 0; i < count; ++i) {
        printf("%s% .9f", i == 0 ? "" : ", ", values[i]);
    }
    printf("]\n");
}

static void print_pose_error(const cquik_vec6 *error)
{
    printf("pose error linear:  [% .9f, % .9f, % .9f]\n",
           error->parts.linear.xyz.x,
           error->parts.linear.xyz.y,
           error->parts.linear.xyz.z);
    printf("pose error angular: [% .9f, % .9f, % .9f]\n",
           error->parts.angular.xyz.x,
           error->parts.angular.xyz.y,
           error->parts.angular.xyz.z);
}

static void print_transform(const char *label, const cquik_mat4 *transform)
{
    int row;

    printf("%s\n", label);
    for (row = 0; row < 4; ++row) {
        printf("  [% .9f % .9f % .9f % .9f]\n",
               transform->m[row][0],
               transform->m[row][1],
               transform->m[row][2],
               transform->m[row][3]);
    }
    printf("  position: [% .9f, % .9f, % .9f]\n",
           transform->affine.tx,
           transform->affine.ty,
           transform->affine.tz);
}

int main(void)
{
    const cquik_joint joints[] = {
        {CQUIK_JOINT_REVOLUTE, 0.0, 0.183,  0.025, -CQUIK_PI / 2.0},
        {CQUIK_JOINT_REVOLUTE, 0.0, 0.000, -0.315,  0.0},
        {CQUIK_JOINT_REVOLUTE, 0.0, 0.000, -0.035,  CQUIK_PI / 2.0},
        {CQUIK_JOINT_REVOLUTE, 0.0, 0.365,  0.000, -CQUIK_PI / 2.0},
        {CQUIK_JOINT_REVOLUTE, 0.0, 0.000,  0.000,  CQUIK_PI / 2.0},
        {CQUIK_JOINT_REVOLUTE, 0.0, 0.080,  0.000,  0.0}
    };
    const cquik_mat4 tool = {
        {
             0.0, 0.0, 1.0, 0.150,
             0.0, 1.0, 0.0, 0.250,
            -1.0, 0.0, 0.0, 0.100,
             0.0, 0.0, 0.0, 1.000
        }
    };
    const cquik_chain chain = {
        (int)(sizeof(joints) / sizeof(joints[0])),
        joints,
        tool.v
    };

    /* In an application this target transform would usually come from a
       planner or controller. Here we generate it from a known joint pose. */
    const double q_target[] = {0.20, -0.35, 0.45, 0.30, -0.25, 0.15};
    double q[] = {0.26, -0.29, 0.39, 0.36, -0.19, 0.09};

    cquik_mat4 target;
    cquik_mat4 solved_transform;
    cquik_vec6 pose_error;
    cquik_options options = cquik_default_options();
    cquik_result result;
    cquik_status status;

    status = cquik_forward_kinematics_typed(&chain, q_target, &target);
    if (status != CQUIK_STATUS_OK) {
        fprintf(stderr, "failed to compute target pose: %s\n",
                cquik_status_string(status));
        return 1;
    }

    options.method = CQUIK_METHOD_DQUIK;
    options.tolerance = 1.0e-10;
    options.max_iterations = 32;
    status = cquik_options_set_recommended_saturation(&chain, &options);
    if (status != CQUIK_STATUS_OK) {
        fprintf(stderr, "failed to configure solver options: %s\n",
                cquik_status_string(status));
        return 1;
    }

    print_vector("target q:", q_target, chain.dof);
    print_vector("initial q:", q, chain.dof);
    print_transform("target tool transform:", &target);

    status = cquik_solve_typed(&chain, &target, q, &options, &result);
    if (status != CQUIK_STATUS_OK) {
        fprintf(stderr, "inverse kinematics failed: %s\n",
                cquik_status_string(status));
        fprintf(stderr, "iterations=%d residual=%.12e\n",
                result.iterations, result.residual_norm);
        return 1;
    }

    status = cquik_forward_kinematics_typed(&chain, q, &solved_transform);
    if (status != CQUIK_STATUS_OK) {
        fprintf(stderr, "failed to compute solved pose: %s\n",
                cquik_status_string(status));
        return 1;
    }

    status = cquik_pose_error_typed(&solved_transform, &target, &pose_error);
    if (status != CQUIK_STATUS_OK) {
        fprintf(stderr, "failed to compute solved pose error: %s\n",
                cquik_status_string(status));
        return 1;
    }

    printf("\nsolver: DQuIK\n");
    printf("status: %s\n", cquik_status_string(result.status));
    printf("iterations: %d\n", result.iterations);
    printf("residual: %.12e\n", result.residual_norm);
    print_vector("solved q:", q, chain.dof);
    print_pose_error(&pose_error);
    print_transform("solved tool transform:", &solved_transform);

    return result.residual_norm < options.tolerance ? 0 : 1;
}
