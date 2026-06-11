# CQuIK

CQuIK is a single-header C library for serial-chain numerical inverse
kinematics using the QuIK/DQuIK algorithm from:

> Steffan Lloyd, Rishad Irani, and Mojtaba Ahmadi, "Fast and Robust Inverse
> Kinematics of Serial Robots using Halley's Method", IEEE Transactions on
> Robotics, 2022.

The library is intentionally small: include `cquik.h`, define
`CQUIK_IMPLEMENTATION` in one translation unit, and link with the C math
library when your platform requires it.

```c
#define CQUIK_IMPLEMENTATION
#include "cquik.h"
```

## API Shape

The public API exposes:

- `cquik_forward_kinematics` for standard-DH serial chains.
- `cquik_forward_kinematics_all` for cumulative base-to-joint transforms.
- `cquik_mat4`, `cquik_vec3`, `cquik_vec4`, and `cquik_vec6` helper unions
  for friendlier matrix/vector access while staying compatible with raw arrays.
- `cquik_jacobian` for the 6-by-n geometric Jacobian.
- `cquik_pose_error` for the 6D pose error used by the solver.
- `cquik_solve` for `NR`, `DNR`, `QuIK`, and `DQuIK`.
- `_w` workspace variants for Jacobian and solve calls that should avoid
  allocation in hot loops.
- `cquik_options_set_recommended_saturation` for scale-aware linear/angular
  step saturation.

Functions are exported with `CQUIK_API`. Define `CQUIK_STATIC` before
including the header for internal-linkage functions, or define
`CQUIK_BUILD_SHARED` / `CQUIK_USE_SHARED` when building or consuming a Windows
DLL.

`cquik_jacobian` and `cquik_solve` allocate temporary memory through
`CQUIK_MALLOC` and `CQUIK_FREE`, which can be overridden before defining
`CQUIK_IMPLEMENTATION`. Use `cquik_jacobian_workspace_size`,
`cquik_jacobian_w`, `cquik_solve_workspace_size`, and `cquik_solve_w` when the
caller should own all scratch storage.

## Conventions

Transforms are 4-by-4 row-major matrices:

```text
[ r00 r01 r02 px ]
[ r10 r11 r12 py ]
[ r20 r21 r22 pz ]
[  0   0   0  1 ]
```

Joints use standard DH parameters. For revolute joints, `q[i]` is added to
`theta`. For prismatic joints, `q[i]` is added to `d`.

The raw C APIs accept `double[16]` transforms and `double[6]` pose errors. The
helper unions provide named and row/column views over the same storage:

```c
cquik_mat4 transform;

transform.v[7]          /* flat row-major access */
transform.m[1][3]       /* row/column access */
transform.affine.ty     /* named translation access */
transform.rows.r1.xyzw.w
```

Typed wrappers such as `cquik_forward_kinematics_typed` and
`cquik_pose_error_typed` accept these helpers directly.

Use a consistent linear unit across DH `a`/`d`, tool transforms, target
transforms, tolerances, and `max_linear_step`. Angular values are radians.

## Solver Notes

`DQuIK` is the default method. `DNR` and `DQuIK` use `options.damping`; `NR`
and `QuIK` are undamped and ignore it. Undamped methods require a full-row-rank
6-by-n Jacobian and normally need at least six independent joints. For low-DOF,
redundant, or near-singular configurations, prefer the damped methods.

Default options leave step saturation disabled by setting `max_linear_step` and
`max_angular_step` to zero. Call
`cquik_options_set_recommended_saturation(&chain, &options)` when you want a
reasonable starting limit based on the chain's DH length scale.

`cquik_solve` may return `CQUIK_STATUS_STALLED` when residual progress is too
small or the correction step falls below `minimum_step_size` before the
tolerance is reached. The stall checks can be disabled by setting
`relative_improvement_tolerance`, `minimum_step_size`,
`max_consecutive_stalls`, or `max_total_stalls` to zero as appropriate.

`result.iterations` counts correction updates; an initial guess already within
tolerance reports zero iterations. The library has no global mutable state, so
calls are reentrant as long as caller-owned inputs and workspaces are not shared
unsafely.

Chains must have `1 <= dof <= CQUIK_MAX_DOF`; override `CQUIK_MAX_DOF` before
including the header if a different validation cap is needed.

## Meson Build

```sh
meson setup build
meson test -C build
```

`meson test` builds the KUKA KR6 inverse-kinematics example and the µnit-based
unit suite in `tests/cquik_tests.c`.

The unit suite includes reference values generated from `steffanlloyd/quik`
commit `a9ebd1f21dfd3c9f0869140dfb0a4ae5a538cccc`, covering KUKA KR6 nominal,
home, and near-singular poses, a mixed revolute/prismatic chain, a redundant
KUKA iiwa7 chain, and pose-error edge rotations.

Install the header with:

```sh
meson install -C build
```

The Meson project exports `cquik_dep` internally for examples/tests and
installs `cquik.h` as a header-only library.

## SWIG Interface

The SWIG interface is in `bindings/swig/cquik.i`. It wraps the C API
directly and adds two small helpers:

- `cquik_make_joint(...)` creates a `cquik_joint` value.
- `cquik_make_chain(dof, joints, tool)` creates a `cquik_chain` value.

Use SWIG's generated array wrappers for pointer arguments. Arrays passed into a
chain must outlive that chain because CQuIK stores borrowed pointers. The
example below shows the generated Python wrapper; SWIG can target other
languages from the same interface file.

```python
import math
import cquik

joints = cquik.cquik_joint_array(2)
joints[0] = cquik.cquik_make_joint(
    cquik.CQUIK_JOINT_REVOLUTE, 0.0, 0.0, 1.0, 0.0)
joints[1] = cquik.cquik_make_joint(
    cquik.CQUIK_JOINT_REVOLUTE, 0.0, 0.0, 1.0, 0.0)

chain = cquik.cquik_make_chain(2, joints.cast(), None)

q = cquik.cquik_double_array(2)
q[0] = math.pi / 2.0
q[1] = 0.0

transform = cquik.cquik_double_array(16)
status = cquik.cquik_forward_kinematics(chain, q, transform)
if status != cquik.CQUIK_STATUS_OK:
    raise RuntimeError(cquik.cquik_status_string(status))

print(transform[3], transform[7], transform[11])
```

## Examples

```sh
meson compile -C build cquik-kuka-kr6-ik
./build/cquik-kuka-kr6-ik
```

The complete runnable example is in `examples/kuka_kr6_ik.c`. The snippets
below show the same patterns in smaller pieces.

### Include The Implementation

Put the implementation in exactly one C file:

```c
#define CQUIK_IMPLEMENTATION
#include "cquik.h"
```

Other C files should include only the declarations:

```c
#include "cquik.h"
```

### Define A Chain

This KUKA KR6-style chain uses standard DH parameters in meters and radians.
The optional tool transform is applied after the last joint.

```c
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
```

### Forward Kinematics

```c
double q[6] = {0.20, -0.35, 0.45, 0.30, -0.25, 0.15};
cquik_mat4 transform;

cquik_status status = cquik_forward_kinematics_typed(&kuka_chain, q, &transform);
if (status != CQUIK_STATUS_OK) {
    /* Handle invalid chain or input pointers. */
}
```

`transform` is a 4-by-4 row-major homogeneous matrix:

```text
[ r00 r01 r02 px ]
[ r10 r11 r12 py ]
[ r20 r21 r22 pz ]
[  0   0   0  1 ]
```

### Joint Transforms

Use `cquik_forward_kinematics_all` when you need individual joint frames for
debugging, visualization, or drawing links.

```c
double q[6] = {0.20, -0.35, 0.45, 0.30, -0.25, 0.15};
cquik_mat4 joint_frames[6 + 1];
cquik_status status;

status = cquik_forward_kinematics_all_typed(&kuka_chain, q, joint_frames);
if (status != CQUIK_STATUS_OK) {
    return 1;
}

/* frame 0: base identity */
const cquik_mat4 *base = &joint_frames[0];

/* frame i + 1: cumulative transform after joint i, before the optional tool */
const cquik_mat4 *joint_0 = &joint_frames[1];
const cquik_mat4 *joint_5 = &joint_frames[6];

printf("joint 5 position: %f %f %f\n",
       joint_5->affine.tx,
       joint_5->affine.ty,
       joint_5->affine.tz);
```

The optional `chain.tool` transform is intentionally not included in
`joint_frames`. Call `cquik_forward_kinematics` for the final tool pose, or
multiply `joint_frames[chain.dof]` by `chain.tool` yourself when needed.

### Inverse Kinematics

`cquik_solve` updates the initial guess in place. The target can come from a
planner, controller, or another call to `cquik_forward_kinematics`.

```c
cquik_mat4 target;
double q_target[6] = {0.20, -0.35, 0.45, 0.30, -0.25, 0.15};
double q_guess[6] = {0.26, -0.29, 0.39, 0.36, -0.19, 0.09};
cquik_options options = cquik_default_options();
cquik_result result;
cquik_status status;

status = cquik_forward_kinematics_typed(&kuka_chain, q_target, &target);
if (status != CQUIK_STATUS_OK) {
    return 1;
}

options.method = CQUIK_METHOD_DQUIK;
options.tolerance = 1.0e-10;
options.max_iterations = 32;
cquik_options_set_recommended_saturation(&kuka_chain, &options);

status = cquik_solve_typed(&kuka_chain, &target, q_guess, &options, &result);
if (status != CQUIK_STATUS_OK) {
    fprintf(stderr, "IK failed: %s, residual %.12e\n",
            cquik_status_string(status),
            result.residual_norm);
    return 1;
}

/* q_guess now contains the solved joint values. */
```

### Check The Solved Pose

```c
cquik_mat4 solved_transform;
cquik_vec6 error;

cquik_forward_kinematics_typed(&kuka_chain, q_guess, &solved_transform);
cquik_pose_error_typed(&solved_transform, &target, &error);

printf("linear error:  %.12e %.12e %.12e\n",
       error.parts.linear.xyz.x,
       error.parts.linear.xyz.y,
       error.parts.linear.xyz.z);
printf("angular error: %.12e %.12e %.12e\n",
       error.parts.angular.xyz.x,
       error.parts.angular.xyz.y,
       error.parts.angular.xyz.z);
```

The first three error values use the chain's linear unit. The last three are
radians.

### No-Allocation Solve

Use the workspace API in real-time or hot-loop code where per-call heap
allocation is undesirable.

```c
double workspace[27u * 6u + 6u];
double q[6] = {0.26, -0.29, 0.39, 0.36, -0.19, 0.09};
cquik_options options = cquik_default_options();
cquik_result result;

cquik_options_set_recommended_saturation(&kuka_chain, &options);

status = cquik_solve_w_typed(
    &kuka_chain,
    &target,
    q,
    &options,
    &result,
    workspace,
    sizeof(workspace) / sizeof(workspace[0]));
```

For generic code, query the workspace size:

```c
size_t workspace_count = cquik_solve_workspace_size(&kuka_chain);
```

### Meson Consumer

When CQuIK is a subproject, consume it as a header-only dependency and link the
math library on platforms that need it:

```meson
project('robot_app', 'c', default_options: ['c_std=c99'])

cc = meson.get_compiler('c')
cquik_dep = dependency('cquik', fallback: ['cquik', 'cquik_dep'])
math_dep = cc.find_library('m', required: false)

executable(
  'robot_app',
  'main.c',
  dependencies: [cquik_dep, math_dep],
)
```
