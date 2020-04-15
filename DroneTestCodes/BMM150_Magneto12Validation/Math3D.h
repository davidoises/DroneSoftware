#ifndef MATH3D_LIB
#define MATH3D_LIB

#include "math.h"

void ICACHE_FLASH_ATTR  quat_multiply(double* res_quat, double* q1, double* q2)// res can be the same as in
{
    double w = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    double x = q1[0]*q2[1] + q2[0]*q1[1] - q1[3]*q2[2] + q1[2]*q2[3];
    double y = q1[0]*q2[2] + q2[0]*q1[2] + q1[3]*q2[1] - q1[1]*q2[3];
    double z = q1[0]*q2[3] + q2[0]*q1[3] - q1[2]*q2[1] + q1[1]*q2[2];

    res_quat[0] = w;
    res_quat[1] = x;
    res_quat[2] = y;
    res_quat[3] = z;

}

double ICACHE_FLASH_ATTR quat_norm(double* quat)
{
    return sqrt(pow(quat[0], 2) + pow(quat[1], 2) + pow(quat[2], 2) + pow(quat[3], 2));
}

void ICACHE_FLASH_ATTR quat_conjugate(double* res_quat, double* quat)// res can be the same as in
{
    res_quat[0] = quat[0];
    res_quat[1] = -quat[1];
    res_quat[2] = -quat[2];
    res_quat[3] = -quat[3];
}

void ICACHE_FLASH_ATTR quat_inverse(double* res_quat, double* quat)// res can be the same as in
{
    double mag2 = pow(quat_norm(quat),2);
    quat_conjugate(res_quat, quat);
    res_quat[0] = res_quat[0]/mag2;
    res_quat[1] = res_quat[1]/mag2;
    res_quat[2] = res_quat[2]/mag2;
    res_quat[3] = res_quat[3]/mag2;
}

void ICACHE_FLASH_ATTR quat_normalize(double* res_quat, double* quat)// res can be the same as in
{
    double mag = quat_norm(quat);
    if(mag == 0)
    {
        res_quat[0] = 1;
        res_quat[1] = 0;
        res_quat[2] = 0;
        res_quat[3] = 0;
    }
    else
    {
        res_quat[0] = quat[0]/mag;
        res_quat[1] = quat[1]/mag;
        res_quat[2] = quat[2]/mag;
        res_quat[3] = quat[3]/mag;
    }
}

void ICACHE_FLASH_ATTR quat_generate(double* res_quat, double angle, double* axis)
{
    res_quat[0] = cos(angle/2);
    res_quat[1] = sin(angle/2)*axis[0];
    res_quat[2] = sin(angle/2)*axis[1];
    res_quat[3] = sin(angle/2)*axis[2];
}

double ICACHE_FLASH_ATTR vec_norm(double* vec)
{
    return sqrt(pow(vec[0], 2) + pow(vec[1], 2) + pow(vec[2], 2));
}

double ICACHE_FLASH_ATTR vec_normalize(double* res_vec, double* vec)// res can be the same as in
{
    double norm = vec_norm(vec);
    if(norm == 0)
    {
        res_vec[0] = 0;
        res_vec[1] = 0;
        res_vec[2] = 0;
    }
    else
    {
        res_vec[0] = vec[0]/norm;
        res_vec[1] = vec[1]/norm;
        res_vec[2] = vec[2]/norm;
    }
    return norm;
}

#endif
