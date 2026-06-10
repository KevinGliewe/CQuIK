/*
    CQuIK - single-header C implementation of Quick Inverse Kinematics.

    Define CQUIK_IMPLEMENTATION in exactly one translation unit before
    including this header to emit the implementation.

        #define CQUIK_IMPLEMENTATION
        #include "cquik.h"

    Matrices are 4x4 row-major homogeneous transforms:

        [ r00 r01 r02 px ]
        [ r10 r11 r12 py ]
        [ r20 r21 r22 pz ]
        [  0   0   0  1 ]

    Linear quantities use the same unit as your DH a/d parameters and target
    transforms. Angular quantities use radians.

    The convenience solve/Jacobian functions allocate temporary memory. Define
    CQUIK_MALLOC and CQUIK_FREE before CQUIK_IMPLEMENTATION to override that,
    or call the _w variants with caller-owned workspace in hot paths.

    The library keeps no global mutable state; calls are reentrant as long as
    caller-owned inputs and workspaces are not shared unsafely.
*/
#ifndef CQUIK_H
#define CQUIK_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Library semantic-version major component. */
#define CQUIK_VERSION_MAJOR 0

/** Library semantic-version minor component. */
#define CQUIK_VERSION_MINOR 1

/** Library semantic-version patch component. */
#define CQUIK_VERSION_PATCH 0

#ifndef CQUIK_MAX_DOF
/** Maximum accepted chain degrees of freedom.
 *
 * Override this before including the header if an application needs a
 * different validation cap. The default is intentionally much larger than a
 * practical serial manipulator while still preventing integer overflow in
 * workspace-size arithmetic.
 */
#define CQUIK_MAX_DOF 1024
#endif

#ifndef CQUIK_PI
/** Pi constant used by CQuIK when the host math headers do not expose one. */
#define CQUIK_PI 3.14159265358979323846264338327950288
#endif

#ifndef CQUIK_API
/** Public symbol linkage decoration.
 *
 * Define CQUIK_STATIC before including the header for internal-linkage
 * functions, CQUIK_BUILD_SHARED while building a Windows DLL, or
 * CQUIK_USE_SHARED while consuming that DLL.
 */
#  if defined(CQUIK_STATIC)
#    define CQUIK_API static
#  elif defined(_WIN32) && defined(CQUIK_BUILD_SHARED)
#    define CQUIK_API __declspec(dllexport)
#  elif defined(_WIN32) && defined(CQUIK_USE_SHARED)
#    define CQUIK_API __declspec(dllimport)
#  else
#    define CQUIK_API extern
#  endif
#endif

/** Status code returned by CQuIK API functions. */
typedef enum cquik_status {
    CQUIK_STATUS_OK = 0,               /**< Operation completed successfully. */
    CQUIK_STATUS_MAX_ITERATIONS = 1,   /**< Solver hit max_iterations while still improving. */
    CQUIK_STATUS_STALLED = 2,          /**< Solver progress or step size fell below stall limits. */
    CQUIK_STATUS_INVALID_ARGUMENT = -1, /**< A required pointer, enum value, or chain shape is invalid. */
    CQUIK_STATUS_OUT_OF_MEMORY = -2,   /**< Allocation failed or caller workspace is too small. */
    CQUIK_STATUS_SINGULAR = -3         /**< Undamped pseudoinverse encountered a singular system. */
} cquik_status;

/** Standard-DH joint type. */
typedef enum cquik_joint_type {
    CQUIK_JOINT_REVOLUTE = 0,  /**< q[i] is added to theta. */
    CQUIK_JOINT_PRISMATIC = 1  /**< q[i] is added to d. */
} cquik_joint_type;

/** Numerical inverse-kinematics method. */
typedef enum cquik_method {
    CQUIK_METHOD_NR = 0,    /**< Newton-Raphson; undamped, requires full row rank. */
    CQUIK_METHOD_DNR = 1,   /**< Damped Newton-Raphson; uses options.damping. */
    CQUIK_METHOD_QUIK = 2,  /**< QuIK/Halley update; undamped, requires full row rank. */
    CQUIK_METHOD_DQUIK = 3  /**< Damped QuIK/Halley update; uses options.damping. */
} cquik_method;

/** One standard-DH joint in a serial chain.
 *
 * The joint transform uses the conventional standard-DH order
 * RotZ(theta + q), TransZ(d), TransX(a), RotX(alpha) for revolute joints.
 * For prismatic joints, q is added to d instead of theta.
 */
typedef struct cquik_joint {
    cquik_joint_type type; /**< Revolute or prismatic joint behavior. */
    double theta;          /**< Fixed DH theta offset, in radians. */
    double d;              /**< Fixed DH d offset, in the chain's linear unit. */
    double a;              /**< Fixed DH a link length, in the chain's linear unit. */
    double alpha;          /**< Fixed DH alpha twist, in radians. */
} cquik_joint;

/** Serial kinematic chain description.
 *
 * The pointed-to joints and optional tool transform must remain valid for the
 * duration of each API call. CQuIK never takes ownership of these pointers.
 */
typedef struct cquik_chain {
    int dof;                    /**< Number of joints; must be in [1, CQUIK_MAX_DOF]. */
    const cquik_joint *joints;  /**< Array of dof standard-DH joints. */
    const double *tool;         /**< Optional 4x4 row-major tool transform, or NULL. */
} cquik_chain;

/** Solver configuration.
 *
 * Use cquik_default_options() as a starting point. Linear values use the same
 * unit as the chain DH parameters and target transforms; angular values use
 * radians. Setting saturation or stall fields to zero disables that specific
 * limit.
 */
typedef struct cquik_options {
    cquik_method method;                 /**< Solver method; default is CQUIK_METHOD_DQUIK. */
    int max_iterations;                  /**< Maximum correction updates before returning MAX_ITERATIONS. */
    double tolerance;                    /**< Converged when the 6D pose-error norm is below this value. */
    double damping;                      /**< DNR/DQuIK damping lambda; ignored by NR/QuIK. */
    double max_linear_step;              /**< Linear error saturation limit; 0 disables it. */
    double max_angular_step;             /**< Angular error saturation limit in radians; 0 disables it. */
    double minimum_step_size;            /**< Minimum joint-correction norm; 0 disables this stall check. */
    double relative_improvement_tolerance; /**< Residual improvement threshold; 0 disables residual stall checks. */
    int max_consecutive_stalls;          /**< Consecutive low-improvement iterations before STALLED; 0 disables it. */
    int max_total_stalls;                /**< Total low-improvement iterations before STALLED; 0 disables it. */
} cquik_options;

/** Result details from cquik_solve() or cquik_solve_w(). */
typedef struct cquik_result {
    cquik_status status; /**< Final solve status; mirrors the function return value. */
    int iterations;      /**< Number of correction updates attempted; already-converged guesses report 0. */
    double residual_norm; /**< Final 6D pose-error norm in mixed linear/radian units. */
} cquik_result;

/** Return the default solver options.
 *
 * Defaults are conservative for a header library: DQuIK, 64 iterations,
 * tolerance 1e-8, light damping, stall checks enabled, and step saturation
 * disabled. Call cquik_options_set_recommended_saturation() to enable the
 * paper/upstream-style saturation limits.
 */
CQUIK_API cquik_options cquik_default_options(void);

/** Return a static human-readable string for a status code. */
CQUIK_API const char *cquik_status_string(cquik_status status);

/** Return the chain characteristic length used for recommended saturation.
 *
 * The value is sum_i sqrt(a_i^2 + d_i^2), matching the upstream QuIK
 * reference implementation. Returns 0.0 for an invalid chain.
 *
 * @param chain Chain description.
 * @return Characteristic chain length in the chain's linear unit.
 */
CQUIK_API double cquik_characteristic_length(const cquik_chain *chain);

/** Set paper/upstream-style step saturation limits on an options struct.
 *
 * Sets max_linear_step to 0.33 * cquik_characteristic_length(chain) and
 * max_angular_step to 1 radian. Existing unrelated option fields are left
 * unchanged.
 *
 * @param chain Valid chain description.
 * @param options Options to modify in place.
 * @return CQUIK_STATUS_OK, or CQUIK_STATUS_INVALID_ARGUMENT.
 */
CQUIK_API cquik_status cquik_options_set_recommended_saturation(
    const cquik_chain *chain,
    cquik_options *options);

/** Write a 4x4 row-major identity transform to out. */
CQUIK_API void cquik_mat4_identity(double out[16]);

/** Multiply two 4x4 row-major matrices.
 *
 * The output may alias either input.
 *
 * @param a Left-hand matrix.
 * @param b Right-hand matrix.
 * @param out Destination matrix receiving a * b.
 */
CQUIK_API void cquik_mat4_mul(const double a[16], const double b[16], double out[16]);

/** Invert a rigid 4x4 row-major homogeneous transform.
 *
 * This assumes the top-left 3x3 block is a rotation matrix and the bottom row
 * is [0 0 0 1]. The output may alias the input.
 *
 * @param t Input rigid transform.
 * @param out Destination inverse transform.
 */
CQUIK_API void cquik_mat4_inverse_rigid(const double t[16], double out[16]);

/** Compute forward kinematics for a chain at joint vector q.
 *
 * @param chain Valid serial chain description.
 * @param q Array of chain->dof joint values.
 * @param out_transform Destination 4x4 row-major end-effector transform.
 * @return CQUIK_STATUS_OK or CQUIK_STATUS_INVALID_ARGUMENT.
 */
CQUIK_API cquik_status cquik_forward_kinematics(
    const cquik_chain *chain,
    const double *q,
    double out_transform[16]);

/** Compute the 6D pose error from current to target.
 *
 * The linear part is current position minus target position in world/base
 * coordinates. The angular part is the logarithm vector of R_current *
 * R_target^T in radians. This is the error convention used by cquik_solve().
 *
 * @param current Current 4x4 row-major transform.
 * @param target Desired 4x4 row-major transform.
 * @param out_error Destination array [dx, dy, dz, rx, ry, rz].
 * @return CQUIK_STATUS_OK or CQUIK_STATUS_INVALID_ARGUMENT.
 */
CQUIK_API cquik_status cquik_pose_error(
    const double current[16],
    const double target[16],
    double out_error[6]);

/** Compute the 6-by-dof geometric Jacobian.
 *
 * Rows 0..2 are linear velocity columns in the chain's linear unit per joint
 * unit. Rows 3..5 are angular velocity columns in radians per joint unit.
 * Revolute joint units are radians; prismatic joint units are the chain's
 * linear unit. The output is row-major: out[row * chain->dof + column].
 *
 * This convenience function allocates temporary storage through CQUIK_MALLOC.
 * Use cquik_jacobian_w() to supply workspace explicitly.
 *
 * @param chain Valid serial chain description.
 * @param q Array of chain->dof joint values.
 * @param out_jacobian_6xn Destination row-major 6-by-dof array.
 * @return CQUIK_STATUS_OK, CQUIK_STATUS_INVALID_ARGUMENT, or CQUIK_STATUS_OUT_OF_MEMORY.
 */
CQUIK_API cquik_status cquik_jacobian(
    const cquik_chain *chain,
    const double *q,
    double *out_jacobian_6xn);

/** Return the number of doubles required by cquik_jacobian_w().
 *
 * Returns 0 for an invalid chain.
 */
CQUIK_API size_t cquik_jacobian_workspace_size(const cquik_chain *chain);

/** Compute the 6-by-dof geometric Jacobian using caller-owned workspace.
 *
 * @param chain Valid serial chain description.
 * @param q Array of chain->dof joint values.
 * @param out_jacobian_6xn Destination row-major 6-by-dof array.
 * @param workspace Caller-owned scratch array of doubles.
 * @param workspace_count Number of doubles available in workspace.
 * @return CQUIK_STATUS_OK, CQUIK_STATUS_INVALID_ARGUMENT, or CQUIK_STATUS_OUT_OF_MEMORY.
 */
CQUIK_API cquik_status cquik_jacobian_w(
    const cquik_chain *chain,
    const double *q,
    double *out_jacobian_6xn,
    double *workspace,
    size_t workspace_count);

/** Solve inverse kinematics in place.
 *
 * q_in_out is both the initial guess and the solved joint vector. On success,
 * cquik_pose_error(FK(q_in_out), target) has norm below options->tolerance.
 * On failure, q_in_out contains the last iterate and result, if non-NULL,
 * describes the stopping condition.
 *
 * NR and QuIK are undamped and require a full-row-rank 6-by-dof Jacobian;
 * they normally require at least six independent joints. DNR and DQuIK use
 * options->damping and are better choices for low-DOF or near-singular chains.
 *
 * This convenience function allocates temporary storage through CQUIK_MALLOC.
 * Use cquik_solve_w() to supply workspace explicitly.
 *
 * @param chain Valid serial chain description.
 * @param target Desired 4x4 row-major end-effector transform.
 * @param q_in_out Array of chain->dof joint values, updated in place.
 * @param options Optional solver options; NULL uses cquik_default_options().
 * @param result Optional solve-result details; may be NULL.
 * @return Solver status.
 */
CQUIK_API cquik_status cquik_solve(
    const cquik_chain *chain,
    const double target[16],
    double *q_in_out,
    const cquik_options *options,
    cquik_result *result);

/** Return the number of doubles required by cquik_solve_w().
 *
 * Returns 0 for an invalid chain.
 */
CQUIK_API size_t cquik_solve_workspace_size(const cquik_chain *chain);

/** Solve inverse kinematics in place using caller-owned workspace.
 *
 * See cquik_solve() for solver semantics. The workspace must contain at least
 * cquik_solve_workspace_size(chain) doubles.
 *
 * @param chain Valid serial chain description.
 * @param target Desired 4x4 row-major end-effector transform.
 * @param q_in_out Array of chain->dof joint values, updated in place.
 * @param options Optional solver options; NULL uses cquik_default_options().
 * @param result Optional solve-result details; may be NULL.
 * @param workspace Caller-owned scratch array of doubles.
 * @param workspace_count Number of doubles available in workspace.
 * @return Solver status.
 */
CQUIK_API cquik_status cquik_solve_w(
    const cquik_chain *chain,
    const double target[16],
    double *q_in_out,
    const cquik_options *options,
    cquik_result *result,
    double *workspace,
    size_t workspace_count);

#ifdef __cplusplus
}
#endif

#endif /* CQUIK_H */

#ifdef CQUIK_IMPLEMENTATION
#ifndef CQUIK_IMPLEMENTATION_DONE
#define CQUIK_IMPLEMENTATION_DONE

#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CQUIK_EPSILON
/** Absolute numerical pivot/zero threshold used internally.
 *
 * Override before defining CQUIK_IMPLEMENTATION if the application uses a
 * very different scale and needs a tighter or looser singularity threshold.
 */
#define CQUIK_EPSILON 1.0e-12
#endif

#ifndef CQUIK_MALLOC
/** Allocation hook used by convenience APIs that need temporary workspace. */
#define CQUIK_MALLOC(size) malloc(size)
#endif

#ifndef CQUIK_FREE
/** Deallocation hook matching CQUIK_MALLOC. */
#define CQUIK_FREE(ptr) free(ptr)
#endif

#if defined(CQUIK_STATIC)
#  if defined(__GNUC__) || defined(__clang__)
#    define CQUIK_DEF static __attribute__((unused))
#  else
#    define CQUIK_DEF static
#  endif
#else
#  define CQUIK_DEF CQUIK_API
#endif

static double cquik__dot3(const double a[3], const double b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static double cquik__norm3(const double v[3])
{
    return sqrt(cquik__dot3(v, v));
}

static void cquik__cross3(const double a[3], const double b[3], double out[3])
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

static int cquik__valid_chain(const cquik_chain *chain)
{
    return chain &&
        chain->dof > 0 &&
        chain->dof <= CQUIK_MAX_DOF &&
        chain->joints;
}

static void cquik__joint_transform(const cquik_joint *joint, double q, double out[16])
{
    double theta = joint->theta;
    double d = joint->d;
    const double a = joint->a;
    const double alpha = joint->alpha;
    const double ct = cos(theta + (joint->type == CQUIK_JOINT_REVOLUTE ? q : 0.0));
    const double st = sin(theta + (joint->type == CQUIK_JOINT_REVOLUTE ? q : 0.0));
    const double ca = cos(alpha);
    const double sa = sin(alpha);

    if (joint->type == CQUIK_JOINT_PRISMATIC) {
        d += q;
    }

    out[0] = ct;
    out[1] = -st * ca;
    out[2] = st * sa;
    out[3] = a * ct;

    out[4] = st;
    out[5] = ct * ca;
    out[6] = -ct * sa;
    out[7] = a * st;

    out[8] = 0.0;
    out[9] = sa;
    out[10] = ca;
    out[11] = d;

    out[12] = 0.0;
    out[13] = 0.0;
    out[14] = 0.0;
    out[15] = 1.0;
}

static cquik_status cquik__eval_kinematics(
    const cquik_chain *chain,
    const double *q,
    double *origins_3xnp1,
    double *axes_3xnp1,
    double out_transform[16])
{
    int i;
    double t[16];

    if (!cquik__valid_chain(chain) || !q || !out_transform) {
        return CQUIK_STATUS_INVALID_ARGUMENT;
    }

    /* Forward traversal starts at the base frame. When requested, keep the
       pre-joint origins and z-axes because the geometric Jacobian columns are
       built from those frame quantities. */
    cquik_mat4_identity(t);

    if (origins_3xnp1) {
        origins_3xnp1[0] = 0.0;
        origins_3xnp1[1] = 0.0;
        origins_3xnp1[2] = 0.0;
    }
    if (axes_3xnp1) {
        axes_3xnp1[0] = 0.0;
        axes_3xnp1[1] = 0.0;
        axes_3xnp1[2] = 1.0;
    }

    for (i = 0; i < chain->dof; ++i) {
        double joint_t[16];
        double next[16];

        cquik__joint_transform(&chain->joints[i], q[i], joint_t);
        cquik_mat4_mul(t, joint_t, next);
        memcpy(t, next, sizeof(t));

        if (origins_3xnp1) {
            origins_3xnp1[(i + 1) * 3 + 0] = t[3];
            origins_3xnp1[(i + 1) * 3 + 1] = t[7];
            origins_3xnp1[(i + 1) * 3 + 2] = t[11];
        }
        if (axes_3xnp1) {
            axes_3xnp1[(i + 1) * 3 + 0] = t[2];
            axes_3xnp1[(i + 1) * 3 + 1] = t[6];
            axes_3xnp1[(i + 1) * 3 + 2] = t[10];
        }
    }

    if (chain->tool) {
        cquik_mat4_mul(t, chain->tool, out_transform);
    } else {
        memcpy(out_transform, t, sizeof(t));
    }

    return CQUIK_STATUS_OK;
}

static cquik_status cquik__jacobian_with_workspace(
    const cquik_chain *chain,
    const double *q,
    double *origins,
    double *axes,
    double transform[16],
    double *out_j)
{
    int j;
    cquik_status status;
    double p_end[3];

    if (!out_j) {
        return CQUIK_STATUS_INVALID_ARGUMENT;
    }

    status = cquik__eval_kinematics(chain, q, origins, axes, transform);
    if (status != CQUIK_STATUS_OK) {
        return status;
    }

    p_end[0] = transform[3];
    p_end[1] = transform[7];
    p_end[2] = transform[11];

    /* Geometric Jacobian: revolute columns are [z x (p_end - p_joint); z],
       prismatic columns are [z; 0]. The axes/origins arrays contain the
       frame state before each joint motion is applied. */
    for (j = 0; j < chain->dof; ++j) {
        const double *axis = &axes[j * 3];
        const double *origin = &origins[j * 3];

        if (chain->joints[j].type == CQUIK_JOINT_REVOLUTE) {
            double r[3];
            double jv[3];
            r[0] = p_end[0] - origin[0];
            r[1] = p_end[1] - origin[1];
            r[2] = p_end[2] - origin[2];
            cquik__cross3(axis, r, jv);

            out_j[0 * chain->dof + j] = jv[0];
            out_j[1 * chain->dof + j] = jv[1];
            out_j[2 * chain->dof + j] = jv[2];
            out_j[3 * chain->dof + j] = axis[0];
            out_j[4 * chain->dof + j] = axis[1];
            out_j[5 * chain->dof + j] = axis[2];
        } else {
            out_j[0 * chain->dof + j] = axis[0];
            out_j[1 * chain->dof + j] = axis[1];
            out_j[2 * chain->dof + j] = axis[2];
            out_j[3 * chain->dof + j] = 0.0;
            out_j[4 * chain->dof + j] = 0.0;
            out_j[5 * chain->dof + j] = 0.0;
        }
    }

    return CQUIK_STATUS_OK;
}

static void cquik__saturate3(double *v, double max_norm)
{
    double n;
    if (max_norm <= 0.0) {
        return;
    }
    n = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (n > max_norm && n > CQUIK_EPSILON) {
        const double s = max_norm / n;
        v[0] *= s;
        v[1] *= s;
        v[2] *= s;
    }
}

static double cquik__norm6(const double v[6])
{
    int i;
    double sum = 0.0;
    for (i = 0; i < 6; ++i) {
        sum += v[i] * v[i];
    }
    return sqrt(sum);
}

static cquik_status cquik__solve6(double a[36], const double b[6], double x[6])
{
    int i;
    int j;
    int k;
    double aug[6][7];

    /* Small fixed-size Gauss-Jordan solve for the normal-equation system.
       Partial pivoting gives a clear singular status for undamped rank loss. */
    for (i = 0; i < 6; ++i) {
        for (j = 0; j < 6; ++j) {
            aug[i][j] = a[i * 6 + j];
        }
        aug[i][6] = b[i];
    }

    for (i = 0; i < 6; ++i) {
        int pivot = i;
        double pivot_abs = fabs(aug[i][i]);

        for (j = i + 1; j < 6; ++j) {
            const double candidate = fabs(aug[j][i]);
            if (candidate > pivot_abs) {
                pivot = j;
                pivot_abs = candidate;
            }
        }

        if (pivot_abs < CQUIK_EPSILON) {
            return CQUIK_STATUS_SINGULAR;
        }

        if (pivot != i) {
            for (k = i; k < 7; ++k) {
                const double tmp = aug[i][k];
                aug[i][k] = aug[pivot][k];
                aug[pivot][k] = tmp;
            }
        }

        {
            const double div = aug[i][i];
            for (k = i; k < 7; ++k) {
                aug[i][k] /= div;
            }
        }

        for (j = 0; j < 6; ++j) {
            if (j != i) {
                const double factor = aug[j][i];
                for (k = i; k < 7; ++k) {
                    aug[j][k] -= factor * aug[i][k];
                }
            }
        }
    }

    for (i = 0; i < 6; ++i) {
        x[i] = aug[i][6];
    }

    return CQUIK_STATUS_OK;
}

static cquik_status cquik__pinv_solve_6xn(
    const double *a,
    int n,
    const double b[6],
    double damping,
    double *x)
{
    int r;
    int c;
    int k;
    double aat[36];
    double y[6];
    cquik_status status;
    const double lambda2 = damping * damping;

    if (!a || n <= 0 || !b || !x) {
        return CQUIK_STATUS_INVALID_ARGUMENT;
    }

    /* Solve min ||A*x - b|| with x = A^T * y and
       (A*A^T + lambda^2 I) * y = b. A nonzero damping lambda makes low-rank
       Jacobians usable for DNR/DQuIK. */
    for (r = 0; r < 6; ++r) {
        for (c = 0; c < 6; ++c) {
            double sum = 0.0;
            for (k = 0; k < n; ++k) {
                sum += a[r * n + k] * a[c * n + k];
            }
            if (r == c) {
                sum += lambda2;
            }
            aat[r * 6 + c] = sum;
        }
    }

    status = cquik__solve6(aat, b, y);
    if (status != CQUIK_STATUS_OK) {
        return status;
    }

    for (k = 0; k < n; ++k) {
        double sum = 0.0;
        for (r = 0; r < 6; ++r) {
            sum += a[r * n + k] * y[r];
        }
        x[k] = sum;
    }

    return CQUIK_STATUS_OK;
}

static void cquik__hessian_delta(
    const cquik_chain *chain,
    const double *j,
    const double *delta,
    double *out_6xn)
{
    int row;
    int col;
    int k;
    const int n = chain->dof;

    /* Directional Hessian product H(q) * delta used by the QuIK/Halley
       correction. This avoids materializing the full third-order Hessian. */
    for (row = 0; row < 6; ++row) {
        for (col = 0; col < n; ++col) {
            out_6xn[row * n + col] = 0.0;
        }
    }

    for (k = 0; k < n; ++k) {
        const int k_revolute = chain->joints[k].type == CQUIK_JOINT_REVOLUTE;
        double jw_k[3] = {
            j[3 * n + k],
            j[4 * n + k],
            j[5 * n + k]
        };
        double jv_k[3] = {
            j[0 * n + k],
            j[1 * n + k],
            j[2 * n + k]
        };

        if (delta[k] == 0.0) {
            continue;
        }

        for (col = 0; col < n; ++col) {
            double d_jv[3] = {0.0, 0.0, 0.0};
            double d_jw[3] = {0.0, 0.0, 0.0};

            if (k <= col && k_revolute) {
                double jv_col[3] = {
                    j[0 * n + col],
                    j[1 * n + col],
                    j[2 * n + col]
                };
                cquik__cross3(jw_k, jv_col, d_jv);
            } else if (k > col && chain->joints[col].type == CQUIK_JOINT_REVOLUTE) {
                double jw_col[3] = {
                    j[3 * n + col],
                    j[4 * n + col],
                    j[5 * n + col]
                };
                cquik__cross3(jw_col, jv_k, d_jv);
            }

            if (k < col && k_revolute &&
                chain->joints[col].type == CQUIK_JOINT_REVOLUTE) {
                double jw_col[3] = {
                    j[3 * n + col],
                    j[4 * n + col],
                    j[5 * n + col]
                };
                cquik__cross3(jw_k, jw_col, d_jw);
            }

            out_6xn[0 * n + col] += d_jv[0] * delta[k];
            out_6xn[1 * n + col] += d_jv[1] * delta[k];
            out_6xn[2 * n + col] += d_jv[2] * delta[k];
            out_6xn[3 * n + col] += d_jw[0] * delta[k];
            out_6xn[4 * n + col] += d_jw[1] * delta[k];
            out_6xn[5 * n + col] += d_jw[2] * delta[k];
        }
    }
}

CQUIK_DEF cquik_options cquik_default_options(void)
{
    cquik_options options;
    options.method = CQUIK_METHOD_DQUIK;
    options.max_iterations = 64;
    options.tolerance = 1.0e-8;
    options.damping = 3.162277660168379e-4; /* sqrt(1e-7), the paper's light damping. */
    options.max_linear_step = 0.0;
    options.max_angular_step = 0.0;
    options.minimum_step_size = 1.0e-14;
    options.relative_improvement_tolerance = 0.05;
    options.max_consecutive_stalls = 5;
    options.max_total_stalls = 20;
    return options;
}

CQUIK_DEF const char *cquik_status_string(cquik_status status)
{
    switch (status) {
    case CQUIK_STATUS_OK:
        return "ok";
    case CQUIK_STATUS_MAX_ITERATIONS:
        return "maximum iterations reached";
    case CQUIK_STATUS_STALLED:
        return "solver stalled";
    case CQUIK_STATUS_INVALID_ARGUMENT:
        return "invalid argument";
    case CQUIK_STATUS_OUT_OF_MEMORY:
        return "out of memory";
    case CQUIK_STATUS_SINGULAR:
        return "singular linear system";
    default:
        return "unknown status";
    }
}

CQUIK_DEF double cquik_characteristic_length(const cquik_chain *chain)
{
    int i;
    double length = 0.0;

    if (!cquik__valid_chain(chain)) {
        return 0.0;
    }

    for (i = 0; i < chain->dof; ++i) {
        const double a = chain->joints[i].a;
        const double d = chain->joints[i].d;
        length += sqrt(a * a + d * d);
    }

    return length;
}

CQUIK_DEF cquik_status cquik_options_set_recommended_saturation(
    const cquik_chain *chain,
    cquik_options *options)
{
    double length;

    if (!cquik__valid_chain(chain) || !options) {
        return CQUIK_STATUS_INVALID_ARGUMENT;
    }

    length = cquik_characteristic_length(chain);
    options->max_linear_step = 0.33 * length;
    options->max_angular_step = 1.0;
    return CQUIK_STATUS_OK;
}

CQUIK_DEF void cquik_mat4_identity(double out[16])
{
    int i;
    for (i = 0; i < 16; ++i) {
        out[i] = 0.0;
    }
    out[0] = 1.0;
    out[5] = 1.0;
    out[10] = 1.0;
    out[15] = 1.0;
}

CQUIK_DEF void cquik_mat4_mul(const double a[16], const double b[16], double out[16])
{
    int r;
    int c;
    double tmp[16];

    for (r = 0; r < 4; ++r) {
        for (c = 0; c < 4; ++c) {
            tmp[r * 4 + c] =
                a[r * 4 + 0] * b[0 * 4 + c] +
                a[r * 4 + 1] * b[1 * 4 + c] +
                a[r * 4 + 2] * b[2 * 4 + c] +
                a[r * 4 + 3] * b[3 * 4 + c];
        }
    }

    memcpy(out, tmp, sizeof(tmp));
}

CQUIK_DEF void cquik_mat4_inverse_rigid(const double t[16], double out[16])
{
    double tmp[16];

    tmp[0] = t[0];
    tmp[1] = t[4];
    tmp[2] = t[8];
    tmp[4] = t[1];
    tmp[5] = t[5];
    tmp[6] = t[9];
    tmp[8] = t[2];
    tmp[9] = t[6];
    tmp[10] = t[10];

    tmp[3] = -(tmp[0] * t[3] + tmp[1] * t[7] + tmp[2] * t[11]);
    tmp[7] = -(tmp[4] * t[3] + tmp[5] * t[7] + tmp[6] * t[11]);
    tmp[11] = -(tmp[8] * t[3] + tmp[9] * t[7] + tmp[10] * t[11]);

    tmp[12] = 0.0;
    tmp[13] = 0.0;
    tmp[14] = 0.0;
    tmp[15] = 1.0;

    memcpy(out, tmp, sizeof(tmp));
}

CQUIK_DEF cquik_status cquik_forward_kinematics(
    const cquik_chain *chain,
    const double *q,
    double out_transform[16])
{
    return cquik__eval_kinematics(chain, q, NULL, NULL, out_transform);
}

CQUIK_DEF cquik_status cquik_pose_error(
    const double current[16],
    const double target[16],
    double out_error[6])
{
    double re[9];
    double eps[3];
    double eps_norm;
    double trace;

    if (!current || !target || !out_error) {
        return CQUIK_STATUS_INVALID_ARGUMENT;
    }

    out_error[0] = current[3] - target[3];
    out_error[1] = current[7] - target[7];
    out_error[2] = current[11] - target[11];

    /* Rotation error uses R_current * R_target^T. Computing the 3x3 product
       directly avoids a full homogeneous inverse/multiply and keeps the
       translational error in the world-frame convention expected by J_v. */
    re[0] = current[0] * target[0] + current[1] * target[1] + current[2] * target[2];
    re[1] = current[0] * target[4] + current[1] * target[5] + current[2] * target[6];
    re[2] = current[0] * target[8] + current[1] * target[9] + current[2] * target[10];
    re[3] = current[4] * target[0] + current[5] * target[1] + current[6] * target[2];
    re[4] = current[4] * target[4] + current[5] * target[5] + current[6] * target[6];
    re[5] = current[4] * target[8] + current[5] * target[9] + current[6] * target[10];
    re[6] = current[8] * target[0] + current[9] * target[1] + current[10] * target[2];
    re[7] = current[8] * target[4] + current[9] * target[5] + current[10] * target[6];
    re[8] = current[8] * target[8] + current[9] * target[9] + current[10] * target[10];

    eps[0] = re[7] - re[5];
    eps[1] = re[2] - re[6];
    eps[2] = re[3] - re[1];
    eps_norm = cquik__norm3(eps);
    trace = re[0] + re[4] + re[8];

    /* Log-map branches mirror upstream QuIK: a Taylor form near identity,
       atan2 for the regular case, and a stable diagonal approximation near
       180 degrees where the rotation axis is poorly conditioned. */
    if (trace > -0.99 || eps_norm > 1.0e-10) {
        if (eps_norm < 1.0e-3) {
            const double scale = 0.75 - trace / 12.0;
            out_error[3] = eps[0] * scale;
            out_error[4] = eps[1] * scale;
            out_error[5] = eps[2] * scale;
        } else {
            const double angle = atan2(eps_norm, trace - 1.0);
            const double scale = angle / eps_norm;
            out_error[3] = eps[0] * scale;
            out_error[4] = eps[1] * scale;
            out_error[5] = eps[2] * scale;
        }
    } else {
        out_error[3] = (CQUIK_PI / 2.0) * (re[0] + 1.0);
        out_error[4] = (CQUIK_PI / 2.0) * (re[4] + 1.0);
        out_error[5] = (CQUIK_PI / 2.0) * (re[8] + 1.0);
    }

    return CQUIK_STATUS_OK;
}

CQUIK_DEF cquik_status cquik_jacobian(
    const cquik_chain *chain,
    const double *q,
    double *out_jacobian_6xn)
{
    double *workspace;
    cquik_status status;
    size_t workspace_count;

    workspace_count = cquik_jacobian_workspace_size(chain);
    if (workspace_count == 0 || !q || !out_jacobian_6xn) {
        return CQUIK_STATUS_INVALID_ARGUMENT;
    }

    workspace = (double *)CQUIK_MALLOC(workspace_count * sizeof(double));
    if (!workspace) {
        return CQUIK_STATUS_OUT_OF_MEMORY;
    }

    status = cquik_jacobian_w(chain, q, out_jacobian_6xn, workspace, workspace_count);
    CQUIK_FREE(workspace);
    return status;
}

CQUIK_DEF size_t cquik_jacobian_workspace_size(const cquik_chain *chain)
{
    if (!cquik__valid_chain(chain)) {
        return 0u;
    }

    return 6u * ((size_t)chain->dof + 1u);
}

CQUIK_DEF cquik_status cquik_jacobian_w(
    const cquik_chain *chain,
    const double *q,
    double *out_jacobian_6xn,
    double *workspace,
    size_t workspace_count)
{
    double *origins;
    double *axes;
    double transform[16];
    const size_t needed = cquik_jacobian_workspace_size(chain);

    if (needed == 0u || !q || !out_jacobian_6xn || !workspace) {
        return CQUIK_STATUS_INVALID_ARGUMENT;
    }
    if (workspace_count < needed) {
        return CQUIK_STATUS_OUT_OF_MEMORY;
    }

    origins = workspace;
    axes = origins + ((size_t)chain->dof + 1u) * 3u;

    return cquik__jacobian_with_workspace(chain, q, origins, axes, transform, out_jacobian_6xn);
}

CQUIK_DEF size_t cquik_solve_workspace_size(const cquik_chain *chain)
{
    if (!cquik__valid_chain(chain)) {
        return 0u;
    }

    return 27u * (size_t)chain->dof + 6u;
}

CQUIK_DEF cquik_status cquik_solve(
    const cquik_chain *chain,
    const double target[16],
    double *q_in_out,
    const cquik_options *options,
    cquik_result *result)
{
    double *workspace;
    size_t workspace_count;
    cquik_status status;

    workspace_count = cquik_solve_workspace_size(chain);
    if (workspace_count == 0u || !target || !q_in_out) {
        if (result) {
            result->status = CQUIK_STATUS_INVALID_ARGUMENT;
            result->iterations = 0;
            result->residual_norm = 0.0;
        }
        return CQUIK_STATUS_INVALID_ARGUMENT;
    }

    workspace = (double *)CQUIK_MALLOC(workspace_count * sizeof(double));
    if (!workspace) {
        if (result) {
            result->status = CQUIK_STATUS_OUT_OF_MEMORY;
            result->iterations = 0;
            result->residual_norm = 0.0;
        }
        return CQUIK_STATUS_OUT_OF_MEMORY;
    }

    status = cquik_solve_w(chain, target, q_in_out, options, result, workspace, workspace_count);
    CQUIK_FREE(workspace);
    return status;
}

CQUIK_DEF cquik_status cquik_solve_w(
    const cquik_chain *chain,
    const double target[16],
    double *q_in_out,
    const cquik_options *options,
    cquik_result *result,
    double *scratch,
    size_t scratch_count)
{
    cquik_options local_options;
    cquik_status status = CQUIK_STATUS_OK;
    double *origins;
    double *axes;
    double *j;
    double *a;
    double *h_delta;
    double *nr_correction;
    double *delta;
    double *correction;
    double current[16];
    double error[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double previous_residual = -1.0;
    int iter;
    int consecutive_stalls = 0;
    int total_stalls = 0;
    const int n = chain ? chain->dof : 0;
    size_t cursor = 0;

    if (!cquik__valid_chain(chain) || !target || !q_in_out || !scratch) {
        if (result) {
            result->status = CQUIK_STATUS_INVALID_ARGUMENT;
            result->iterations = 0;
            result->residual_norm = 0.0;
        }
        return CQUIK_STATUS_INVALID_ARGUMENT;
    }

    if (scratch_count < cquik_solve_workspace_size(chain)) {
        if (result) {
            result->status = CQUIK_STATUS_OUT_OF_MEMORY;
            result->iterations = 0;
            result->residual_norm = 0.0;
        }
        return CQUIK_STATUS_OUT_OF_MEMORY;
    }

    local_options = options ? *options : cquik_default_options();
    if (local_options.max_iterations <= 0) {
        local_options.max_iterations = 64;
    }
    if (local_options.tolerance <= 0.0) {
        local_options.tolerance = 1.0e-8;
    }
    if (local_options.damping < 0.0) {
        local_options.damping = -local_options.damping;
    }
    if (local_options.minimum_step_size < 0.0) {
        local_options.minimum_step_size = -local_options.minimum_step_size;
    }
    if (local_options.relative_improvement_tolerance < 0.0) {
        local_options.relative_improvement_tolerance = -local_options.relative_improvement_tolerance;
    }
    if (local_options.method < CQUIK_METHOD_NR ||
        local_options.method > CQUIK_METHOD_DQUIK) {
        if (result) {
            result->status = CQUIK_STATUS_INVALID_ARGUMENT;
            result->iterations = 0;
            result->residual_norm = 0.0;
        }
        return CQUIK_STATUS_INVALID_ARGUMENT;
    }

    /* Split the single caller-provided scratch block into the temporary arrays
       used each iteration. The public workspace-size function is tied to this
       layout, so no allocations are needed in cquik_solve_w(). */
    origins = &scratch[cursor];
    cursor += ((size_t)n + 1u) * 3u;
    axes = &scratch[cursor];
    cursor += ((size_t)n + 1u) * 3u;
    j = &scratch[cursor];
    cursor += (size_t)6 * (size_t)n;
    a = &scratch[cursor];
    cursor += (size_t)6 * (size_t)n;
    h_delta = &scratch[cursor];
    cursor += (size_t)6 * (size_t)n;
    nr_correction = &scratch[cursor];
    cursor += (size_t)n;
    delta = &scratch[cursor];
    cursor += (size_t)n;
    correction = &scratch[cursor];

    for (iter = 0; iter <= local_options.max_iterations; ++iter) {
        double residual;
        double step_damping = 0.0;
        int col;

        status = cquik__jacobian_with_workspace(chain, q_in_out, origins, axes, current, j);
        if (status != CQUIK_STATUS_OK) {
            break;
        }

        status = cquik_pose_error(current, target, error);
        if (status != CQUIK_STATUS_OK) {
            break;
        }

        residual = cquik__norm6(error);
        if (residual < local_options.tolerance) {
            if (result) {
                result->status = CQUIK_STATUS_OK;
                result->iterations = iter;
                result->residual_norm = residual;
            }
            return CQUIK_STATUS_OK;
        }

        if (iter == local_options.max_iterations) {
            if (result) {
                result->status = CQUIK_STATUS_MAX_ITERATIONS;
                result->iterations = iter;
                result->residual_norm = residual;
            }
            return CQUIK_STATUS_MAX_ITERATIONS;
        }

        /* Stall detection distinguishes "still improving but out of budget"
           from "the residual or joint update has stopped making useful
           progress", which is important for retry-from-new-seed callers. */
        if (previous_residual >= 0.0 &&
            local_options.relative_improvement_tolerance > 0.0 &&
            (local_options.max_consecutive_stalls > 0 ||
             local_options.max_total_stalls > 0)) {
            const double improvement =
                previous_residual > CQUIK_EPSILON ?
                (previous_residual - residual) / previous_residual :
                1.0;

            if (improvement < local_options.relative_improvement_tolerance) {
                ++consecutive_stalls;
                ++total_stalls;
                if ((local_options.max_consecutive_stalls > 0 &&
                     consecutive_stalls >= local_options.max_consecutive_stalls) ||
                    (local_options.max_total_stalls > 0 &&
                     total_stalls >= local_options.max_total_stalls)) {
                    if (result) {
                        result->status = CQUIK_STATUS_STALLED;
                        result->iterations = iter;
                        result->residual_norm = residual;
                    }
                    return CQUIK_STATUS_STALLED;
                }
            } else {
                consecutive_stalls = 0;
            }
        }
        previous_residual = residual;

        cquik__saturate3(&error[0], local_options.max_linear_step);
        cquik__saturate3(&error[3], local_options.max_angular_step);

        if (local_options.method == CQUIK_METHOD_DNR ||
            local_options.method == CQUIK_METHOD_DQUIK) {
            step_damping = local_options.damping;
        }

        if (local_options.method == CQUIK_METHOD_NR ||
            local_options.method == CQUIK_METHOD_DNR) {
            status = cquik__pinv_solve_6xn(j, n, error, step_damping, correction);
        } else {
            /* QuIK first computes a Newton correction, uses it as the
               direction for the Hessian product, then solves with
               J + 0.5 * H * delta for the Halley-style update. */
            status = cquik__pinv_solve_6xn(j, n, error, step_damping, nr_correction);
            if (status == CQUIK_STATUS_OK) {
                for (col = 0; col < n; ++col) {
                    delta[col] = -nr_correction[col];
                }

                cquik__hessian_delta(chain, j, delta, h_delta);

                for (col = 0; col < 6 * n; ++col) {
                    a[col] = j[col] + 0.5 * h_delta[col];
                }

                status = cquik__pinv_solve_6xn(a, n, error, step_damping, correction);
            }
        }

        if (status != CQUIK_STATUS_OK) {
            break;
        }

        if (local_options.minimum_step_size > 0.0) {
            double step_norm_squared = 0.0;
            const double minimum_step_squared =
                local_options.minimum_step_size * local_options.minimum_step_size;

            for (col = 0; col < n; ++col) {
                step_norm_squared += correction[col] * correction[col];
            }
            if (step_norm_squared < minimum_step_squared) {
                if (result) {
                    result->status = CQUIK_STATUS_STALLED;
                    result->iterations = iter;
                    result->residual_norm = residual;
                }
                return CQUIK_STATUS_STALLED;
            }
        }

        for (col = 0; col < n; ++col) {
            q_in_out[col] -= correction[col];
        }
    }

    if (result) {
        result->status = status;
        result->iterations = iter;
        result->residual_norm = cquik__norm6(error);
    }
    return status;
}

#undef CQUIK_DEF

#ifdef __cplusplus
}
#endif

#endif /* CQUIK_IMPLEMENTATION_DONE */
#endif /* CQUIK_IMPLEMENTATION */
