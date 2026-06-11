%module cquik

%{
#define CQUIK_IMPLEMENTATION
#include "cquik.h"
%}

%include "carrays.i"

%include "cquik.h"

%array_class(double, cquik_double_array);
%array_class(cquik_joint, cquik_joint_array);

%inline %{
cquik_joint cquik_make_joint(
    cquik_joint_type type,
    double theta,
    double d,
    double a,
    double alpha)
{
    cquik_joint joint;
    joint.type = type;
    joint.theta = theta;
    joint.d = d;
    joint.a = a;
    joint.alpha = alpha;
    return joint;
}

cquik_chain cquik_make_chain(
    int dof,
    const cquik_joint *joints,
    const double *tool)
{
    cquik_chain chain;
    chain.dof = dof;
    chain.joints = joints;
    chain.tool = tool;
    return chain;
}

double cquik_array_get_double(const double *values, int index)
{
    return values[index];
}

void cquik_array_set_double(double *values, int index, double value)
{
    values[index] = value;
}
%}
