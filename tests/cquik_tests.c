#include <math.h>
#include <string.h>

#include "munit.h"

#define CQUIK_IMPLEMENTATION
#include "cquik.h"

static void assert_near(double actual, double expected, double tolerance)
{
    if (fabs(actual - expected) > tolerance) {
        munit_errorf("expected %.17g to be within %.3g of %.17g",
                     actual, tolerance, expected);
    }
}

static void assert_mat4_near(const double actual[16], const double expected[16], double tolerance)
{
    int i;
    for (i = 0; i < 16; ++i) {
        if (fabs(actual[i] - expected[i]) > tolerance) {
            munit_errorf("matrix[%d]: expected %.17g to be within %.3g of %.17g",
                         i, actual[i], tolerance, expected[i]);
        }
    }
}

static void assert_array_near(const double *actual, const double *expected, int count, double tolerance)
{
    int i;
    for (i = 0; i < count; ++i) {
        if (fabs(actual[i] - expected[i]) > tolerance) {
            munit_errorf("array[%d]: expected %.17g to be within %.3g of %.17g",
                         i, actual[i], tolerance, expected[i]);
        }
    }
}

static double norm6(const double values[6])
{
    int i;
    double sum = 0.0;

    for (i = 0; i < 6; ++i) {
        sum += values[i] * values[i];
    }

    return sqrt(sum);
}

static const cquik_joint planar_joints[] = {
    {CQUIK_JOINT_REVOLUTE, 0.0, 0.0, 1.0, 0.0},
    {CQUIK_JOINT_REVOLUTE, 0.0, 0.0, 1.0, 0.0}
};

static const cquik_chain planar_chain = {
    2,
    planar_joints,
    NULL
};

static const cquik_joint kuka_joints[] = {
    {CQUIK_JOINT_REVOLUTE, 0.0, 0.183,  0.025, -CQUIK_PI / 2.0},
    {CQUIK_JOINT_REVOLUTE, 0.0, 0.000, -0.315,  0.0},
    {CQUIK_JOINT_REVOLUTE, 0.0, 0.000, -0.035,  CQUIK_PI / 2.0},
    {CQUIK_JOINT_REVOLUTE, 0.0, 0.365,  0.000, -CQUIK_PI / 2.0},
    {CQUIK_JOINT_REVOLUTE, 0.0, 0.000,  0.000,  CQUIK_PI / 2.0},
    {CQUIK_JOINT_REVOLUTE, 0.0, 0.080,  0.000,  0.0}
};

static const double kuka_tool[16] = {
    0.0, 0.0, 1.0, 0.150,
    0.0, 1.0, 0.0, 0.250,
   -1.0, 0.0, 0.0, 0.100,
    0.0, 0.0, 0.0, 1.000
};

static const cquik_chain kuka_chain = {
    6,
    kuka_joints,
    kuka_tool
};

static const cquik_joint mixed_prismatic_joints[] = {
    {CQUIK_JOINT_REVOLUTE, 0.0, 0.10, 0.40, 0.0},
    {CQUIK_JOINT_PRISMATIC, 0.10, 0.20, 0.00, CQUIK_PI / 2.0},
    {CQUIK_JOINT_REVOLUTE, -0.20, 0.00, 0.20, 0.0}
};

static const cquik_chain mixed_prismatic_chain = {
    3,
    mixed_prismatic_joints,
    NULL
};

static const cquik_joint iiwa7_joints[] = {
    {CQUIK_JOINT_REVOLUTE, 0.0, 0.34, 0.0, -1.5707963268},
    {CQUIK_JOINT_REVOLUTE, 0.0, 0.00, 0.0,  1.5707963268},
    {CQUIK_JOINT_REVOLUTE, 0.0, 0.40, 0.0, -1.5707963268},
    {CQUIK_JOINT_REVOLUTE, 0.0, 0.00, 0.0,  1.5707963268},
    {CQUIK_JOINT_REVOLUTE, 0.0, 0.40, 0.0, -1.5707963268},
    {CQUIK_JOINT_REVOLUTE, 0.0, 0.40, 0.0,  1.5707963268},
    {CQUIK_JOINT_REVOLUTE, 0.0, 0.00, 0.0,  0.0}
};

static const cquik_chain iiwa7_chain = {
    7,
    iiwa7_joints,
    NULL
};

/*
 * Reference values generated from https://github.com/steffanlloyd/quik
 * commit a9ebd1f21dfd3c9f0869140dfb0a4ae5a538cccc using quik::Robot,
 * quik::IKSolver, and quik::geometry::hgtDiff. Upstream DH order is
 * a, alpha, d, theta; the cquik_joint initializers above store the same
 * values as theta, d, a, alpha.
 */
static const double kuka_reference_q_target[6] = {
    0.20, -0.35, 0.45, 0.30, -0.25, 0.15
};

static const double kuka_reference_q_initial[6] = {
    0.26, -0.29, 0.39, 0.36, -0.19, 0.09
};

static const double kuka_home_target[16] = {
     0.0, 0.0, 1.0, -0.17499999999999996,
     0.0, 1.0, 0.0,  0.25,
    -1.0, 0.0, 0.0,  0.72799999999999998,
     0.0, 0.0, 0.0,  1.0
};

static const double kuka_home_jacobian[36] = {
    -0.25, 0.54499999999999993, 0.54499999999999993, -0.25, 0.17999999999999991, -0.25,
    -0.17499999999999996, -1.224646799147353e-17, 7.0417190950972823e-18, 0.14999999999999999, 9.1848509936051487e-18, 0.14999999999999999,
     0.0, 0.19999999999999996, -0.11500000000000002, 0.0, -0.14999999999999999, 0.0,
     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0, 1.0, 1.0, 0.0, 1.0, 0.0,
     1.0, 6.123233995736766e-17, 6.123233995736766e-17, 1.0, 6.123233995736766e-17, 1.0
};

static const double kuka_reference_target[16] = {
     0.12115847100984833, -0.6026189026829033,   0.78877822170227874, -0.31806703202483971,
     0.099159938437521217, 0.79800506941864913,  0.59443688966214392,  0.2173186749530433,
    -0.98766792572788475,  0.0061941352415401489, 0.15644072736019471,  0.6444527501060846,
     0.0,                  0.0,                   0.0,                  1.0
};

static const double kuka_reference_jacobian[36] = {
    -0.2173186749530433,   0.4522544176319,      0.55811416205307074, -0.26544284925286754,  0.17003883535530304, -0.28758739082800511,
    -0.31806703202483971,  0.091676509057116412, 0.11313534160767096, -0.0737181486728479,   0.095334133916469541, -0.028908462002738614,
     0.0,                  0.29355231187851472, -0.002350092668409659, 0.027571686966445363, -0.081911292561504562, -0.038181061553817647,
     0.0,                 -0.19866933079506122, -0.19866933079506122,  0.097843395007255737, -0.47797859760337413,  -0.12115847100984833,
     0.0,                  0.98006657784124163,  0.98006657784124163,  0.019833838076209882,  0.87787587135202971,  -0.099159938437521217,
     1.0,                  6.123233995736766e-17, 6.123233995736766e-17, 0.99500416527802571,  0.029502791919178335,  0.98766792572788475
};

static const double kuka_reference_q_solved[4][6] = {
    {
        0.20000000000000029, -0.35000000000000037, 0.45000000000000046,
        0.29999999999999383, -0.249999999999998,   0.15000000000000582
    },
    {
        0.19999999999988871, -0.35000000000021242, 0.44999999999988227,
        0.300000000000894,   -0.24999999999966591, 0.14999999999918162
    },
    {
        0.19999999999999984, -0.35000000000000037, 0.45000000000000023,
        0.3000000000000001,  -0.24999999999999986, 0.14999999999999994
    },
    {
        0.19999999999996948, -0.35000000000004877, 0.44999999999996759,
        0.30000000000020527, -0.24999999999991723, 0.14999999999981634
    }
};

static const double kuka_near_singular_q_target[6] = {
    0.0, -0.70, 0.90, 0.20, 1.0e-6, -0.30
};

static const double kuka_near_singular_q_initial[6] = {
    0.02, -0.67, 0.86, 0.23, 0.02, -0.32
};

static const double kuka_near_singular_target[16] = {
    -0.19867029132545905, 0.097843336296312117, 0.97517013740529601, 0.028783693644595126,
    -1.9866933079624866e-07, 0.9950041652779964, -0.099833416646923023, 0.2337760645829402,
    -0.98006638313158045, -0.01983412770565874, -0.19767774794735457, 0.48655091085517432,
     0.0, 0.0, 0.0, 1.0
};

static const double mixed_prismatic_q[3] = {
    0.25, 0.15, -0.35
};

static const double mixed_prismatic_target[16] = {
     0.80083827305595301, 0.49099812021127232, 0.3428978074554514,  0.54773262329544847,
     0.29232878941621104, 0.17922830478528867, -0.93937271284737889, 0.15742734158505139,
    -0.52268722893065922, 0.85252452205950568, 6.123233995736766e-17, 0.34546255421386812,
     0.0, 0.0, 0.0, 1.0
};

static const double mixed_prismatic_jacobian[18] = {
    -0.15742734158505139, 0.0, 0.098199624042254463,
     0.54773262329544847, 0.0, 0.035845660957057726,
     0.0, 1.0, 0.17050490441190108,
     0.0, 0.0, 0.3428978074554514,
     0.0, 0.0, -0.93937271284737889,
     1.0, 0.0, 6.123233995736766e-17
};

static const double iiwa7_q_initial[7] = {
    0.19, -0.28, 0.37, -0.46, 0.28, -0.17, 0.11
};

static const double iiwa7_q_target[7] = {
    0.15, -0.25, 0.35, -0.45, 0.25, -0.15, 0.10
};

static const double iiwa7_target[16] = {
     0.36368286513686859, -0.6566424669336004,  -0.66072342491029745, -0.58044456364871244,
     0.64563239773701775,  0.68897759609171549, -0.32934583506707654,  0.18923733361713696,
     0.67148609858893371, -0.30680701214776079,  0.67451899654403824,  0.94081653635419116,
     0.0, 0.0, 0.0, 1.0
};

static const double pose_error_near_identity[6] = {
    1.0000000000000001e-09, -2.0000000000000001e-09, 3.0e-09,
    0.0, 0.0, 9.9999999999999982e-08
};

static const double pose_error_pi_x[6] = {
    -0.25, 0.5, -0.75, 3.141592653589794, 0.0, 0.0
};

static MunitResult test_default_options(const MunitParameter params[], void *user_data)
{
    cquik_options options;
    const double expected_length =
        sqrt(0.025 * 0.025 + 0.183 * 0.183) +
        0.315 + 0.035 + 0.365 + 0.080;

    (void)params;
    (void)user_data;

    options = cquik_default_options();
    munit_assert_int(options.method, ==, CQUIK_METHOD_DQUIK);
    munit_assert_int(options.max_iterations, >, 0);
    munit_assert_double(options.tolerance, >, 0.0);
    munit_assert_double(options.damping, >, 0.0);
    munit_assert_string_equal(cquik_status_string(CQUIK_STATUS_OK), "ok");
    munit_assert_string_equal(cquik_status_string(CQUIK_STATUS_STALLED), "solver stalled");

    assert_near(cquik_characteristic_length(&kuka_chain), expected_length, 1.0e-15);
    munit_assert_int(cquik_options_set_recommended_saturation(&kuka_chain, &options), ==, CQUIK_STATUS_OK);
    assert_near(options.max_linear_step, 0.33 * expected_length, 1.0e-15);
    assert_near(options.max_angular_step, 1.0, 1.0e-15);

    return MUNIT_OK;
}

static MunitResult test_mat4_inverse_rigid(const MunitParameter params[], void *user_data)
{
    const double transform[16] = {
        0.0, -1.0, 0.0, 1.0,
        1.0,  0.0, 0.0, 2.0,
        0.0,  0.0, 1.0, 3.0,
        0.0,  0.0, 0.0, 1.0
    };
    const double identity[16] = {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
    };
    double inverse[16];
    double product[16];

    (void)params;
    (void)user_data;

    cquik_mat4_inverse_rigid(transform, inverse);
    cquik_mat4_mul(transform, inverse, product);
    assert_mat4_near(product, identity, 1.0e-12);

    cquik_mat4_mul(product, transform, product);
    assert_mat4_near(product, transform, 1.0e-12);

    memcpy(inverse, transform, sizeof(inverse));
    cquik_mat4_inverse_rigid(inverse, inverse);
    cquik_mat4_mul(transform, inverse, product);
    assert_mat4_near(product, identity, 1.0e-12);

    return MUNIT_OK;
}

static MunitResult test_matrix_vector_helpers(const MunitParameter params[], void *user_data)
{
    const double q[2] = {CQUIK_PI / 2.0, 0.0};
    cquik_mat4 identity;
    cquik_mat4 transform;
    cquik_mat4 inverse;
    cquik_mat4 product;
    cquik_mat4 frames[3];
    cquik_vec6 error;

    (void)params;
    (void)user_data;

    munit_assert_size(sizeof(cquik_vec3), ==, 3u * sizeof(double));
    munit_assert_size(sizeof(cquik_vec4), ==, 4u * sizeof(double));
    munit_assert_size(sizeof(cquik_vec6), ==, 6u * sizeof(double));
    munit_assert_size(sizeof(cquik_mat4), ==, 16u * sizeof(double));

    cquik_mat4_identity_typed(&identity);
    assert_near(identity.affine.r00, 1.0, 1.0e-12);
    assert_near(identity.m[1][1], 1.0, 1.0e-12);
    assert_near(identity.rows.r2.xyzw.z, 1.0, 1.0e-12);
    assert_near(identity.v[15], 1.0, 1.0e-12);

    munit_assert_int(cquik_forward_kinematics_typed(&planar_chain, q, &transform), ==, CQUIK_STATUS_OK);
    assert_near(transform.affine.tx, 0.0, 1.0e-12);
    assert_near(transform.affine.ty, 2.0, 1.0e-12);
    assert_near(transform.m[1][3], 2.0, 1.0e-12);
    assert_near(transform.rows.r1.xyzw.w, 2.0, 1.0e-12);

    munit_assert_int(cquik_forward_kinematics_all_typed(&planar_chain, q, frames), ==, CQUIK_STATUS_OK);
    assert_near(frames[0].affine.r00, 1.0, 1.0e-12);
    assert_near(frames[1].affine.ty, 1.0, 1.0e-12);
    assert_near(frames[2].affine.ty, 2.0, 1.0e-12);

    cquik_mat4_inverse_rigid_typed(&transform, &inverse);
    cquik_mat4_mul_typed(&transform, &inverse, &product);
    assert_mat4_near(product.v, identity.v, 1.0e-12);

    munit_assert_int(cquik_pose_error_typed(&transform, &identity, &error), ==, CQUIK_STATUS_OK);
    assert_near(error.pose.x, 0.0, 1.0e-12);
    assert_near(error.pose.y, 2.0, 1.0e-12);
    assert_near(error.parts.angular.xyz.z, CQUIK_PI / 2.0, 1.0e-12);

    return MUNIT_OK;
}

static MunitResult test_pose_error(const MunitParameter params[], void *user_data)
{
    const double identity[16] = {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
    };
    const double theta = 0.25;
    double current[16] = {
        0.0, 0.0, 0.0, 1.0,
        0.0, 0.0, 0.0, 2.0,
        0.0, 0.0, 1.0, 3.0,
        0.0, 0.0, 0.0, 1.0
    };
    const double translated_target[16] = {
        1.0, 0.0, 0.0, 1.0,
        0.0, 1.0, 0.0, 2.0,
        0.0, 0.0, 1.0, 3.0,
        0.0, 0.0, 0.0, 1.0
    };
    double error[6];

    (void)params;
    (void)user_data;

    current[0] = cos(theta);
    current[1] = -sin(theta);
    current[4] = sin(theta);
    current[5] = cos(theta);

    munit_assert_int(cquik_pose_error(identity, identity, error), ==, CQUIK_STATUS_OK);
    assert_near(error[0], 0.0, 1.0e-12);
    assert_near(error[1], 0.0, 1.0e-12);
    assert_near(error[2], 0.0, 1.0e-12);
    assert_near(error[3], 0.0, 1.0e-12);
    assert_near(error[4], 0.0, 1.0e-12);
    assert_near(error[5], 0.0, 1.0e-12);

    munit_assert_int(cquik_pose_error(current, identity, error), ==, CQUIK_STATUS_OK);
    assert_near(error[0], 1.0, 1.0e-12);
    assert_near(error[1], 2.0, 1.0e-12);
    assert_near(error[2], 3.0, 1.0e-12);
    assert_near(error[3], 0.0, 1.0e-12);
    assert_near(error[4], 0.0, 1.0e-12);
    assert_near(error[5], theta, 1.0e-12);

    munit_assert_int(cquik_pose_error(current, translated_target, error), ==, CQUIK_STATUS_OK);
    assert_near(error[0], 0.0, 1.0e-12);
    assert_near(error[1], 0.0, 1.0e-12);
    assert_near(error[2], 0.0, 1.0e-12);
    assert_near(error[3], 0.0, 1.0e-12);
    assert_near(error[4], 0.0, 1.0e-12);
    assert_near(error[5], theta, 1.0e-12);

    return MUNIT_OK;
}

static MunitResult test_forward_kinematics_planar(const MunitParameter params[], void *user_data)
{
    const double q[2] = {CQUIK_PI / 2.0, 0.0};
    const double identity[16] = {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
    };
    const double expected_joint0[16] = {
        0.0, -1.0, 0.0, 0.0,
        1.0,  0.0, 0.0, 1.0,
        0.0,  0.0, 1.0, 0.0,
        0.0,  0.0, 0.0, 1.0
    };
    const double expected[16] = {
        0.0, -1.0, 0.0, 0.0,
        1.0,  0.0, 0.0, 2.0,
        0.0,  0.0, 1.0, 0.0,
        0.0,  0.0, 0.0, 1.0
    };
    double transform[16];
    double transforms[(2 + 1) * 16];

    (void)params;
    (void)user_data;

    munit_assert_int(cquik_forward_kinematics(&planar_chain, q, transform), ==, CQUIK_STATUS_OK);
    assert_mat4_near(transform, expected, 1.0e-12);

    munit_assert_int(cquik_forward_kinematics_all(&planar_chain, q, transforms), ==, CQUIK_STATUS_OK);
    assert_mat4_near(&transforms[0 * 16], identity, 1.0e-12);
    assert_mat4_near(&transforms[1 * 16], expected_joint0, 1.0e-12);
    assert_mat4_near(&transforms[2 * 16], expected, 1.0e-12);

    return MUNIT_OK;
}

static MunitResult test_jacobian_planar(const MunitParameter params[], void *user_data)
{
    const double q[2] = {CQUIK_PI / 2.0, 0.0};
    const double expected[12] = {
        -2.0, -1.0,
         0.0,  0.0,
         0.0,  0.0,
         0.0,  0.0,
         0.0,  0.0,
         1.0,  1.0
    };
    double jacobian[12];
    int i;

    (void)params;
    (void)user_data;

    munit_assert_int(cquik_jacobian(&planar_chain, q, jacobian), ==, CQUIK_STATUS_OK);
    for (i = 0; i < 12; ++i) {
        assert_near(jacobian[i], expected[i], 1.0e-12);
    }

    return MUNIT_OK;
}

static MunitResult test_reference_kuka_fk_jacobian(const MunitParameter params[], void *user_data)
{
    double transform[16];
    double jacobian[36];

    (void)params;
    (void)user_data;

    munit_assert_int(cquik_forward_kinematics(&kuka_chain, kuka_reference_q_target, transform), ==, CQUIK_STATUS_OK);
    assert_mat4_near(transform, kuka_reference_target, 1.0e-14);

    munit_assert_int(cquik_jacobian(&kuka_chain, kuka_reference_q_target, jacobian), ==, CQUIK_STATUS_OK);
    assert_array_near(jacobian, kuka_reference_jacobian, 36, 1.0e-14);

    return MUNIT_OK;
}

static MunitResult test_reference_kuka_home(const MunitParameter params[], void *user_data)
{
    const double q_home[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double transform[16];
    double transforms[(6 + 1) * 16];
    double pretool_times_tool[16];
    double jacobian[36];

    (void)params;
    (void)user_data;

    munit_assert_int(cquik_forward_kinematics(&kuka_chain, q_home, transform), ==, CQUIK_STATUS_OK);
    assert_mat4_near(transform, kuka_home_target, 1.0e-14);

    munit_assert_int(cquik_forward_kinematics_all(&kuka_chain, q_home, transforms), ==, CQUIK_STATUS_OK);
    cquik_mat4_mul(&transforms[6 * 16], kuka_tool, pretool_times_tool);
    assert_mat4_near(pretool_times_tool, transform, 1.0e-14);

    munit_assert_int(cquik_jacobian(&kuka_chain, q_home, jacobian), ==, CQUIK_STATUS_OK);
    assert_array_near(jacobian, kuka_home_jacobian, 36, 1.0e-14);

    return MUNIT_OK;
}

static MunitResult test_reference_kuka_near_singular(const MunitParameter params[], void *user_data)
{
    double transform[16];
    double solved_transform[16];
    double error[6];
    double q[6];
    cquik_options options = cquik_default_options();
    cquik_result result;
    size_t i;

    (void)params;
    (void)user_data;

    munit_assert_int(cquik_forward_kinematics(&kuka_chain, kuka_near_singular_q_target, transform), ==, CQUIK_STATUS_OK);
    assert_mat4_near(transform, kuka_near_singular_target, 1.0e-14);

    for (i = 0; i < sizeof(q) / sizeof(q[0]); ++i) {
        q[i] = kuka_near_singular_q_initial[i];
    }

    options.method = CQUIK_METHOD_DQUIK;
    options.tolerance = 2.0e-8;
    options.max_iterations = 64;
    options.damping = sqrt(1.0e-7);
    options.max_linear_step = 0.0;
    options.max_angular_step = 0.0;

    munit_assert_int(cquik_solve(&kuka_chain, kuka_near_singular_target, q, &options, &result), ==, CQUIK_STATUS_OK);
    munit_assert_double(result.residual_norm, <, options.tolerance);

    munit_assert_int(cquik_forward_kinematics(&kuka_chain, q, solved_transform), ==, CQUIK_STATUS_OK);
    munit_assert_int(cquik_pose_error(solved_transform, kuka_near_singular_target, error), ==, CQUIK_STATUS_OK);
    munit_assert_double(fabs(error[0]) + fabs(error[1]) + fabs(error[2]) +
                        fabs(error[3]) + fabs(error[4]) + fabs(error[5]), <, 8.0e-8);

    return MUNIT_OK;
}

static MunitResult test_reference_mixed_prismatic(const MunitParameter params[], void *user_data)
{
    double transform[16];
    double solved_transform[16];
    double error[6];
    double jacobian[18];
    double q[3] = {0.20, 0.12, -0.30};
    cquik_options options = cquik_default_options();
    cquik_result result;

    (void)params;
    (void)user_data;

    munit_assert_int(cquik_forward_kinematics(&mixed_prismatic_chain, mixed_prismatic_q, transform), ==, CQUIK_STATUS_OK);
    assert_mat4_near(transform, mixed_prismatic_target, 1.0e-14);

    munit_assert_int(cquik_jacobian(&mixed_prismatic_chain, mixed_prismatic_q, jacobian), ==, CQUIK_STATUS_OK);
    assert_array_near(jacobian, mixed_prismatic_jacobian, 18, 1.0e-14);

    options.method = CQUIK_METHOD_DQUIK;
    options.tolerance = 1.0e-10;
    options.max_iterations = 64;
    options.relative_improvement_tolerance = 0.0;
    options.minimum_step_size = 0.0;

    munit_assert_int(cquik_solve(&mixed_prismatic_chain, mixed_prismatic_target, q, &options, &result), ==, CQUIK_STATUS_OK);
    munit_assert_double(result.residual_norm, <, options.tolerance);

    munit_assert_int(cquik_forward_kinematics(&mixed_prismatic_chain, q, solved_transform), ==, CQUIK_STATUS_OK);
    munit_assert_int(cquik_pose_error(solved_transform, mixed_prismatic_target, error), ==, CQUIK_STATUS_OK);
    munit_assert_double(norm6(error), <, 1.0e-10);

    return MUNIT_OK;
}

static MunitResult test_reference_iiwa7_redundant(const MunitParameter params[], void *user_data)
{
    double transform[16];
    double solved_transform[16];
    double error[6];
    double q[7];
    cquik_options options = cquik_default_options();
    cquik_result result;
    size_t i;

    (void)params;
    (void)user_data;

    munit_assert_int(cquik_forward_kinematics(&iiwa7_chain, iiwa7_q_target, transform), ==, CQUIK_STATUS_OK);
    assert_mat4_near(transform, iiwa7_target, 1.0e-10);

    for (i = 0; i < sizeof(q) / sizeof(q[0]); ++i) {
        q[i] = iiwa7_q_initial[i];
    }

    options.method = CQUIK_METHOD_DQUIK;
    options.tolerance = 1.0e-10;
    options.max_iterations = 64;
    options.damping = sqrt(1.0e-7);
    options.max_linear_step = 0.0;
    options.max_angular_step = 0.0;

    munit_assert_int(cquik_solve(&iiwa7_chain, iiwa7_target, q, &options, &result), ==, CQUIK_STATUS_OK);
    munit_assert_double(result.residual_norm, <, options.tolerance);

    munit_assert_int(cquik_forward_kinematics(&iiwa7_chain, q, solved_transform), ==, CQUIK_STATUS_OK);
    munit_assert_int(cquik_pose_error(solved_transform, iiwa7_target, error), ==, CQUIK_STATUS_OK);
    munit_assert_double(fabs(error[0]) + fabs(error[1]) + fabs(error[2]) +
                        fabs(error[3]) + fabs(error[4]) + fabs(error[5]), <, 1.0e-10);

    return MUNIT_OK;
}

static MunitResult test_reference_pose_error_edges(const MunitParameter params[], void *user_data)
{
    const double tiny = 1.0e-7;
    const double identity[16] = {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
    };
    double near_identity[16] = {
        0.0, 0.0, 0.0, 1.0e-9,
        0.0, 0.0, 0.0, -2.0e-9,
        0.0, 0.0, 1.0, 3.0e-9,
        0.0, 0.0, 0.0, 1.0
    };
    const double pi_x[16] = {
        1.0,  0.0,  0.0, -0.25,
        0.0, -1.0,  0.0,  0.5,
        0.0,  0.0, -1.0, -0.75,
        0.0,  0.0,  0.0,  1.0
    };
    double error[6];

    (void)params;
    (void)user_data;

    near_identity[0] = cos(tiny);
    near_identity[1] = -sin(tiny);
    near_identity[4] = sin(tiny);
    near_identity[5] = cos(tiny);

    munit_assert_int(cquik_pose_error(near_identity, identity, error), ==, CQUIK_STATUS_OK);
    assert_array_near(error, pose_error_near_identity, 6, 1.0e-20);

    munit_assert_int(cquik_pose_error(pi_x, identity, error), ==, CQUIK_STATUS_OK);
    assert_array_near(error, pose_error_pi_x, 6, 1.0e-14);

    return MUNIT_OK;
}

static MunitResult test_solver_modes(const MunitParameter params[], void *user_data)
{
    const cquik_method methods[] = {
        CQUIK_METHOD_NR,
        CQUIK_METHOD_DNR,
        CQUIK_METHOD_QUIK,
        CQUIK_METHOD_DQUIK
    };
    size_t i;

    (void)params;
    (void)user_data;

    for (i = 0; i < sizeof(methods) / sizeof(methods[0]); ++i) {
        cquik_options options = cquik_default_options();
        cquik_result result;
        double q[6];
        size_t j;

        for (j = 0; j < sizeof(q) / sizeof(q[0]); ++j) {
            q[j] = kuka_reference_q_initial[j];
        }

        options.method = methods[i];
        options.tolerance = 1.0e-10;
        options.max_iterations = 32;

        munit_assert_int(cquik_solve(&kuka_chain, kuka_reference_target, q, &options, &result), ==, CQUIK_STATUS_OK);
        munit_assert_double(result.residual_norm, <, 1.0e-10);
        for (j = 0; j < sizeof(q) / sizeof(q[0]); ++j) {
            assert_near(q[j], kuka_reference_q_solved[i][j], 1.0e-8);
        }
    }

    return MUNIT_OK;
}

static MunitResult test_kuka_home_start_convergence(const MunitParameter params[], void *user_data)
{
    double q[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    cquik_options options = cquik_default_options();
    cquik_result result;

    (void)params;
    (void)user_data;

    options.method = CQUIK_METHOD_DQUIK;
    options.tolerance = 1.0e-8;
    options.max_iterations = 64;
    munit_assert_int(cquik_options_set_recommended_saturation(&kuka_chain, &options), ==, CQUIK_STATUS_OK);

    munit_assert_int(cquik_solve(&kuka_chain, kuka_reference_target, q, &options, &result), ==, CQUIK_STATUS_OK);
    munit_assert_double(result.residual_norm, <, options.tolerance);
    munit_assert_int(result.iterations, <, 20);

    return MUNIT_OK;
}

static MunitResult test_workspace_apis(const MunitParameter params[], void *user_data)
{
    double jacobian[36];
    double workspace_jacobian[36];
    double jacobian_workspace[42];
    double solve_workspace[168];
    double solved_transform[16];
    double error[6];
    double q[6];
    double q_short_workspace[6];
    cquik_options options = cquik_default_options();
    cquik_result result;
    size_t i;

    (void)params;
    (void)user_data;

    munit_assert_size(cquik_jacobian_workspace_size(&kuka_chain), ==, 42);
    munit_assert_size(cquik_solve_workspace_size(&kuka_chain), ==, 168);

    munit_assert_int(cquik_jacobian(&kuka_chain, kuka_reference_q_target, jacobian), ==, CQUIK_STATUS_OK);
    munit_assert_int(cquik_jacobian_w(
        &kuka_chain,
        kuka_reference_q_target,
        workspace_jacobian,
        jacobian_workspace,
        41), ==, CQUIK_STATUS_OUT_OF_MEMORY);
    munit_assert_int(cquik_jacobian_w(
        &kuka_chain,
        kuka_reference_q_target,
        workspace_jacobian,
        jacobian_workspace,
        42), ==, CQUIK_STATUS_OK);
    assert_array_near(workspace_jacobian, jacobian, 36, 1.0e-14);

    for (i = 0; i < sizeof(q) / sizeof(q[0]); ++i) {
        q[i] = kuka_reference_q_initial[i];
        q_short_workspace[i] = kuka_reference_q_initial[i];
    }

    options.method = CQUIK_METHOD_DQUIK;
    options.tolerance = 1.0e-10;
    options.max_iterations = 32;

    munit_assert_int(cquik_solve_w(
        &kuka_chain,
        kuka_reference_target,
        q_short_workspace,
        &options,
        &result,
        solve_workspace,
        167), ==, CQUIK_STATUS_OUT_OF_MEMORY);
    munit_assert_int(result.status, ==, CQUIK_STATUS_OUT_OF_MEMORY);

    munit_assert_int(cquik_solve_w(
        &kuka_chain,
        kuka_reference_target,
        q,
        &options,
        NULL,
        solve_workspace,
        168), ==, CQUIK_STATUS_OK);

    munit_assert_int(cquik_forward_kinematics(&kuka_chain, q, solved_transform), ==, CQUIK_STATUS_OK);
    munit_assert_int(cquik_pose_error(solved_transform, kuka_reference_target, error), ==, CQUIK_STATUS_OK);
    munit_assert_double(norm6(error), <, options.tolerance);

    return MUNIT_OK;
}

static MunitResult test_low_dof_damped_solver(const MunitParameter params[], void *user_data)
{
    const double q_target[2] = {0.30, -0.50};
    double q_nr[2] = {-0.20, 0.10};
    double q_dnr[2] = {-0.20, 0.10};
    double q_dnr_workspace[2] = {-0.20, 0.10};
    double workspace[27u * 2u + 6u];
    cquik_mat4 target;
    cquik_mat4 solved_transform;
    cquik_vec6 error;
    cquik_options options = cquik_default_options();
    cquik_result result;

    (void)params;
    (void)user_data;

    munit_assert_int(cquik_forward_kinematics_typed(&planar_chain, q_target, &target), ==, CQUIK_STATUS_OK);

    options.method = CQUIK_METHOD_NR;
    options.tolerance = 1.0e-12;
    options.max_iterations = 16;
    options.relative_improvement_tolerance = 0.0;
    options.minimum_step_size = 0.0;
    munit_assert_int(cquik_solve_typed(&planar_chain, &target, q_nr, &options, &result), ==, CQUIK_STATUS_SINGULAR);
    munit_assert_int(result.status, ==, CQUIK_STATUS_SINGULAR);

    options.method = CQUIK_METHOD_DNR;
    options.damping = 1.0e-4;
    options.max_iterations = 128;
    munit_assert_int(cquik_solve_typed(&planar_chain, &target, q_dnr, &options, &result), ==, CQUIK_STATUS_OK);
    munit_assert_double(result.residual_norm, <, options.tolerance);

    munit_assert_int(cquik_solve_w_typed(
        &planar_chain,
        &target,
        q_dnr_workspace,
        &options,
        &result,
        workspace,
        sizeof(workspace) / sizeof(workspace[0])), ==, CQUIK_STATUS_OK);
    munit_assert_double(result.residual_norm, <, options.tolerance);

    munit_assert_int(cquik_forward_kinematics_typed(&planar_chain, q_dnr, &solved_transform), ==, CQUIK_STATUS_OK);
    munit_assert_int(cquik_pose_error_typed(&solved_transform, &target, &error), ==, CQUIK_STATUS_OK);
    munit_assert_double(norm6(error.v), <, 1.0e-11);

    return MUNIT_OK;
}

static MunitResult test_solver_stalled(const MunitParameter params[], void *user_data)
{
    const double q_target[2] = {0.0, 0.0};
    double q[2] = {0.0, 0.0};
    double target[16];
    cquik_options options = cquik_default_options();
    cquik_result result;

    (void)params;
    (void)user_data;

    munit_assert_int(cquik_forward_kinematics(&planar_chain, q_target, target), ==, CQUIK_STATUS_OK);
    target[11] = 1.0;

    options.method = CQUIK_METHOD_DNR;
    options.tolerance = 1.0e-12;
    options.damping = 1.0e-4;
    options.max_iterations = 16;
    options.minimum_step_size = 1.0e-12;
    options.relative_improvement_tolerance = 0.0;

    munit_assert_int(cquik_solve(&planar_chain, target, q, &options, &result), ==, CQUIK_STATUS_STALLED);
    munit_assert_int(result.status, ==, CQUIK_STATUS_STALLED);
    munit_assert_double(result.residual_norm, >, 0.9);

    return MUNIT_OK;
}

static MunitResult test_invalid_arguments(const MunitParameter params[], void *user_data)
{
    cquik_options options = cquik_default_options();
    cquik_result result;
    double q[2] = {0.0, 0.0};
    double transform[16];

    (void)params;
    (void)user_data;

    cquik_mat4_identity(transform);

    munit_assert_int(cquik_forward_kinematics(NULL, q, transform), ==, CQUIK_STATUS_INVALID_ARGUMENT);
    munit_assert_int(cquik_forward_kinematics(&planar_chain, NULL, transform), ==, CQUIK_STATUS_INVALID_ARGUMENT);
    munit_assert_int(cquik_forward_kinematics_all(NULL, q, transform), ==, CQUIK_STATUS_INVALID_ARGUMENT);
    munit_assert_int(cquik_forward_kinematics_all(&planar_chain, NULL, transform), ==, CQUIK_STATUS_INVALID_ARGUMENT);
    munit_assert_int(cquik_forward_kinematics_all(&planar_chain, q, NULL), ==, CQUIK_STATUS_INVALID_ARGUMENT);
    munit_assert_int(cquik_pose_error(NULL, transform, q), ==, CQUIK_STATUS_INVALID_ARGUMENT);
    munit_assert_int(cquik_solve(NULL, transform, q, NULL, &result), ==, CQUIK_STATUS_INVALID_ARGUMENT);
    munit_assert_int(result.status, ==, CQUIK_STATUS_INVALID_ARGUMENT);
    munit_assert_int(result.iterations, ==, 0);

    options.method = (cquik_method)99;
    munit_assert_int(cquik_solve(&planar_chain, transform, q, &options, &result), ==, CQUIK_STATUS_INVALID_ARGUMENT);
    munit_assert_int(result.status, ==, CQUIK_STATUS_INVALID_ARGUMENT);

    return MUNIT_OK;
}

static MunitTest cquik_tests[] = {
    {(char *)"/default-options", test_default_options, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL},
    {(char *)"/mat4/inverse-rigid", test_mat4_inverse_rigid, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL},
    {(char *)"/matrix-vector-helpers", test_matrix_vector_helpers, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL},
    {(char *)"/pose-error", test_pose_error, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL},
    {(char *)"/fk/planar", test_forward_kinematics_planar, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL},
    {(char *)"/jacobian/planar", test_jacobian_planar, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL},
    {(char *)"/reference/kuka-fk-jacobian", test_reference_kuka_fk_jacobian, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL},
    {(char *)"/reference/kuka-home", test_reference_kuka_home, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL},
    {(char *)"/reference/kuka-near-singular", test_reference_kuka_near_singular, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL},
    {(char *)"/reference/mixed-prismatic", test_reference_mixed_prismatic, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL},
    {(char *)"/reference/iiwa7-redundant", test_reference_iiwa7_redundant, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL},
    {(char *)"/reference/pose-error-edges", test_reference_pose_error_edges, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL},
    {(char *)"/solve/modes", test_solver_modes, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL},
    {(char *)"/solve/kuka-home-start-convergence", test_kuka_home_start_convergence, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL},
    {(char *)"/workspace/apis", test_workspace_apis, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL},
    {(char *)"/solve/low-dof-damped", test_low_dof_damped_solver, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL},
    {(char *)"/solve/stalled", test_solver_stalled, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL},
    {(char *)"/invalid-arguments", test_invalid_arguments, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL},
    {NULL, NULL, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL}
};

static const MunitSuite cquik_suite = {
    (char *)"/cquik",
    cquik_tests,
    NULL,
    1,
    MUNIT_SUITE_OPTION_NONE
};

int main(int argc, char *argv[MUNIT_ARRAY_PARAM(argc + 1)])
{
    return munit_suite_main(&cquik_suite, NULL, argc, argv);
}
