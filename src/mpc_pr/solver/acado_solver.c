/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int acado_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 2];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 2 + 1];

acadoWorkspace.state[10] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.state[11] = acadoVariables.u[lRun1 * 2 + 1];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1 * 2] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 * 2 + 2];
acadoWorkspace.d[lRun1 * 2 + 1] = acadoWorkspace.state[1] - acadoVariables.x[lRun1 * 2 + 3];

acadoWorkspace.evGx[lRun1 * 4] = acadoWorkspace.state[2];
acadoWorkspace.evGx[lRun1 * 4 + 1] = acadoWorkspace.state[3];
acadoWorkspace.evGx[lRun1 * 4 + 2] = acadoWorkspace.state[4];
acadoWorkspace.evGx[lRun1 * 4 + 3] = acadoWorkspace.state[5];

acadoWorkspace.evGu[lRun1 * 4] = acadoWorkspace.state[6];
acadoWorkspace.evGu[lRun1 * 4 + 1] = acadoWorkspace.state[7];
acadoWorkspace.evGu[lRun1 * 4 + 2] = acadoWorkspace.state[8];
acadoWorkspace.evGu[lRun1 * 4 + 3] = acadoWorkspace.state[9];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 2;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = u[0];
out[3] = u[1];
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 10; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 2];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 2 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.u[runObj * 2];
acadoWorkspace.objValueIn[3] = acadoVariables.u[runObj * 2 + 1];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 4] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 4 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 4 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 4 + 3] = acadoWorkspace.objValueOut[3];

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[20];
acadoWorkspace.objValueIn[1] = acadoVariables.x[21];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];

}

void acado_multGxd( real_t* const dOld, real_t* const Gx1, real_t* const dNew )
{
dNew[0] += + Gx1[0]*dOld[0] + Gx1[1]*dOld[1];
dNew[1] += + Gx1[2]*dOld[0] + Gx1[3]*dOld[1];
}

void acado_moveGxT( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = Gx1[0];
Gx2[1] = Gx1[1];
Gx2[2] = Gx1[2];
Gx2[3] = Gx1[3];
}

void acado_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[2];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[3];
Gx3[2] = + Gx1[2]*Gx2[0] + Gx1[3]*Gx2[2];
Gx3[3] = + Gx1[2]*Gx2[1] + Gx1[3]*Gx2[3];
}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[2];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[3];
Gu2[2] = + Gx1[2]*Gu1[0] + Gx1[3]*Gu1[2];
Gu2[3] = + Gx1[2]*Gu1[1] + Gx1[3]*Gu1[3];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
}

void acado_setBlockH11( int iRow, int iCol, real_t* const Gu1, real_t* const Gu2 )
{
acadoWorkspace.H[(iRow * 40) + (iCol * 2)] += + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2];
acadoWorkspace.H[(iRow * 40) + (iCol * 2 + 1)] += + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3];
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2)] += + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2];
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2 + 1)] += + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3];
}

void acado_setBlockH11_R1( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 40) + (iCol * 2)] = (real_t)2.0000000000000000e+00;
acadoWorkspace.H[(iRow * 40) + (iCol * 2 + 1)] = 0.0;
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2)] = 0.0;
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2 + 1)] = (real_t)2.0000000000000000e+00;
}

void acado_zeroBlockH11( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 40) + (iCol * 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 40) + (iCol * 2 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2 + 1)] = 0.0000000000000000e+00;
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 40) + (iCol * 2)] = acadoWorkspace.H[(iCol * 40) + (iRow * 2)];
acadoWorkspace.H[(iRow * 40) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 40 + 20) + (iRow * 2)];
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2)] = acadoWorkspace.H[(iCol * 40) + (iRow * 2 + 1)];
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 40 + 20) + (iRow * 2 + 1)];
}

void acado_multQ1d( real_t* const dOld, real_t* const dNew )
{
dNew[0] = + (real_t)3.0000000000000000e+00*dOld[0];
dNew[1] = +dOld[1];
}

void acado_multQN1d( real_t* const dOld, real_t* const dNew )
{
dNew[0] = + (real_t)1.0000000000000001e-01*dOld[0];
dNew[1] = + (real_t)1.0000000000000001e-01*dOld[1];
}

void acado_multRDy( real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + (real_t)2.0000000000000000e+00*Dy1[2];
RDy1[1] = + (real_t)2.0000000000000000e+00*Dy1[3];
}

void acado_multQDy( real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + (real_t)3.0000000000000000e+00*Dy1[0];
QDy1[1] = +Dy1[1];
}

void acado_multEQDy( real_t* const E1, real_t* const QDy1, real_t* const U1 )
{
U1[0] += + E1[0]*QDy1[0] + E1[2]*QDy1[1];
U1[1] += + E1[1]*QDy1[0] + E1[3]*QDy1[1];
}

void acado_multQETGx( real_t* const E1, real_t* const Gx1, real_t* const H101 )
{
H101[0] += + E1[0]*Gx1[0] + E1[2]*Gx1[2];
H101[1] += + E1[0]*Gx1[1] + E1[2]*Gx1[3];
H101[2] += + E1[1]*Gx1[0] + E1[3]*Gx1[2];
H101[3] += + E1[1]*Gx1[1] + E1[3]*Gx1[3];
}

void acado_zeroBlockH10( real_t* const H101 )
{
{ int lCopy; for (lCopy = 0; lCopy < 4; lCopy++) H101[ lCopy ] = 0; }
}

void acado_multEDu( real_t* const E1, real_t* const U1, real_t* const dNew )
{
dNew[0] += + E1[0]*U1[0] + E1[1]*U1[1];
dNew[1] += + E1[2]*U1[0] + E1[3]*U1[1];
}

void acado_multQ1Gx( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = + (real_t)3.0000000000000000e+00*Gx1[0];
Gx2[1] = + (real_t)3.0000000000000000e+00*Gx1[1];
Gx2[2] = +Gx1[2];
Gx2[3] = +Gx1[3];
}

void acado_multQN1Gx( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = + (real_t)1.0000000000000001e-01*Gx1[0];
Gx2[1] = + (real_t)1.0000000000000001e-01*Gx1[1];
Gx2[2] = + (real_t)1.0000000000000001e-01*Gx1[2];
Gx2[3] = + (real_t)1.0000000000000001e-01*Gx1[3];
}

void acado_multQ1Gu( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + (real_t)3.0000000000000000e+00*Gu1[0];
Gu2[1] = + (real_t)3.0000000000000000e+00*Gu1[1];
Gu2[2] = +Gu1[2];
Gu2[3] = +Gu1[3];
}

void acado_multQN1Gu( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + (real_t)1.0000000000000001e-01*Gu1[0];
Gu2[1] = + (real_t)1.0000000000000001e-01*Gu1[1];
Gu2[2] = + (real_t)1.0000000000000001e-01*Gu1[2];
Gu2[3] = + (real_t)1.0000000000000001e-01*Gu1[3];
}

void acado_multHxC( real_t* const Hx, real_t* const Gx, real_t* const A01 )
{
A01[0] = + Hx[0]*Gx[0] + Hx[1]*Gx[2];
A01[1] = + Hx[0]*Gx[1] + Hx[1]*Gx[3];
A01[2] = + Hx[2]*Gx[0] + Hx[3]*Gx[2];
A01[3] = + Hx[2]*Gx[1] + Hx[3]*Gx[3];
A01[4] = + Hx[4]*Gx[0] + Hx[5]*Gx[2];
A01[5] = + Hx[4]*Gx[1] + Hx[5]*Gx[3];
A01[6] = + Hx[6]*Gx[0] + Hx[7]*Gx[2];
A01[7] = + Hx[6]*Gx[1] + Hx[7]*Gx[3];
A01[8] = + Hx[8]*Gx[0] + Hx[9]*Gx[2];
A01[9] = + Hx[8]*Gx[1] + Hx[9]*Gx[3];
A01[10] = + Hx[10]*Gx[0] + Hx[11]*Gx[2];
A01[11] = + Hx[10]*Gx[1] + Hx[11]*Gx[3];
A01[12] = + Hx[12]*Gx[0] + Hx[13]*Gx[2];
A01[13] = + Hx[12]*Gx[1] + Hx[13]*Gx[3];
A01[14] = + Hx[14]*Gx[0] + Hx[15]*Gx[2];
A01[15] = + Hx[14]*Gx[1] + Hx[15]*Gx[3];
A01[16] = + Hx[16]*Gx[0] + Hx[17]*Gx[2];
A01[17] = + Hx[16]*Gx[1] + Hx[17]*Gx[3];
A01[18] = + Hx[18]*Gx[0] + Hx[19]*Gx[2];
A01[19] = + Hx[18]*Gx[1] + Hx[19]*Gx[3];
}

void acado_multHxE( real_t* const Hx, real_t* const E, int row, int col )
{
acadoWorkspace.A[(row * 200 + 400) + (col * 2)] = + Hx[0]*E[0] + Hx[1]*E[2];
acadoWorkspace.A[(row * 200 + 400) + (col * 2 + 1)] = + Hx[0]*E[1] + Hx[1]*E[3];
acadoWorkspace.A[(row * 200 + 420) + (col * 2)] = + Hx[2]*E[0] + Hx[3]*E[2];
acadoWorkspace.A[(row * 200 + 420) + (col * 2 + 1)] = + Hx[2]*E[1] + Hx[3]*E[3];
acadoWorkspace.A[(row * 200 + 440) + (col * 2)] = + Hx[4]*E[0] + Hx[5]*E[2];
acadoWorkspace.A[(row * 200 + 440) + (col * 2 + 1)] = + Hx[4]*E[1] + Hx[5]*E[3];
acadoWorkspace.A[(row * 200 + 460) + (col * 2)] = + Hx[6]*E[0] + Hx[7]*E[2];
acadoWorkspace.A[(row * 200 + 460) + (col * 2 + 1)] = + Hx[6]*E[1] + Hx[7]*E[3];
acadoWorkspace.A[(row * 200 + 480) + (col * 2)] = + Hx[8]*E[0] + Hx[9]*E[2];
acadoWorkspace.A[(row * 200 + 480) + (col * 2 + 1)] = + Hx[8]*E[1] + Hx[9]*E[3];
acadoWorkspace.A[(row * 200 + 500) + (col * 2)] = + Hx[10]*E[0] + Hx[11]*E[2];
acadoWorkspace.A[(row * 200 + 500) + (col * 2 + 1)] = + Hx[10]*E[1] + Hx[11]*E[3];
acadoWorkspace.A[(row * 200 + 520) + (col * 2)] = + Hx[12]*E[0] + Hx[13]*E[2];
acadoWorkspace.A[(row * 200 + 520) + (col * 2 + 1)] = + Hx[12]*E[1] + Hx[13]*E[3];
acadoWorkspace.A[(row * 200 + 540) + (col * 2)] = + Hx[14]*E[0] + Hx[15]*E[2];
acadoWorkspace.A[(row * 200 + 540) + (col * 2 + 1)] = + Hx[14]*E[1] + Hx[15]*E[3];
acadoWorkspace.A[(row * 200 + 560) + (col * 2)] = + Hx[16]*E[0] + Hx[17]*E[2];
acadoWorkspace.A[(row * 200 + 560) + (col * 2 + 1)] = + Hx[16]*E[1] + Hx[17]*E[3];
acadoWorkspace.A[(row * 200 + 580) + (col * 2)] = + Hx[18]*E[0] + Hx[19]*E[2];
acadoWorkspace.A[(row * 200 + 580) + (col * 2 + 1)] = + Hx[18]*E[1] + Hx[19]*E[3];
}

void acado_macHxd( real_t* const Hx, real_t* const tmpd, real_t* const lbA, real_t* const ubA )
{
acadoWorkspace.evHxd[0] = + Hx[0]*tmpd[0] + Hx[1]*tmpd[1];
acadoWorkspace.evHxd[1] = + Hx[2]*tmpd[0] + Hx[3]*tmpd[1];
acadoWorkspace.evHxd[2] = + Hx[4]*tmpd[0] + Hx[5]*tmpd[1];
acadoWorkspace.evHxd[3] = + Hx[6]*tmpd[0] + Hx[7]*tmpd[1];
acadoWorkspace.evHxd[4] = + Hx[8]*tmpd[0] + Hx[9]*tmpd[1];
acadoWorkspace.evHxd[5] = + Hx[10]*tmpd[0] + Hx[11]*tmpd[1];
acadoWorkspace.evHxd[6] = + Hx[12]*tmpd[0] + Hx[13]*tmpd[1];
acadoWorkspace.evHxd[7] = + Hx[14]*tmpd[0] + Hx[15]*tmpd[1];
acadoWorkspace.evHxd[8] = + Hx[16]*tmpd[0] + Hx[17]*tmpd[1];
acadoWorkspace.evHxd[9] = + Hx[18]*tmpd[0] + Hx[19]*tmpd[1];
lbA[0] -= acadoWorkspace.evHxd[0];
lbA[1] -= acadoWorkspace.evHxd[1];
lbA[2] -= acadoWorkspace.evHxd[2];
lbA[3] -= acadoWorkspace.evHxd[3];
lbA[4] -= acadoWorkspace.evHxd[4];
lbA[5] -= acadoWorkspace.evHxd[5];
lbA[6] -= acadoWorkspace.evHxd[6];
lbA[7] -= acadoWorkspace.evHxd[7];
lbA[8] -= acadoWorkspace.evHxd[8];
lbA[9] -= acadoWorkspace.evHxd[9];
ubA[0] -= acadoWorkspace.evHxd[0];
ubA[1] -= acadoWorkspace.evHxd[1];
ubA[2] -= acadoWorkspace.evHxd[2];
ubA[3] -= acadoWorkspace.evHxd[3];
ubA[4] -= acadoWorkspace.evHxd[4];
ubA[5] -= acadoWorkspace.evHxd[5];
ubA[6] -= acadoWorkspace.evHxd[6];
ubA[7] -= acadoWorkspace.evHxd[7];
ubA[8] -= acadoWorkspace.evHxd[8];
ubA[9] -= acadoWorkspace.evHxd[9];
}

void acado_evaluatePathConstraints(const real_t* in, real_t* out)
{
const real_t* xd = in;
/* Vector of auxiliary variables; number of elements: 480. */
real_t* a = acadoWorkspace.conAuxVar;

/* Compute intermediate quantities: */
a[0] = (sin(xd[0]));
a[1] = (((real_t)(5.0000000000000000e-01)*a[0])+(real_t)(1.0000000000000001e-01));
a[2] = (sin((xd[0]+xd[1])));
a[3] = ((((real_t)(5.0000000000000000e-01)*a[0])+((real_t)(4.0000000000000002e-01)*a[2]))+(real_t)(1.0000000000000001e-01));
a[4] = (cos(xd[0]));
a[5] = (cos((xd[0]+xd[1])));
a[6] = (real_t)(6.2500000000000000e-02);
a[7] = (((((((real_t)(5.0000000000000000e-01)*a[4])+(((real_t)(4.0000000000000002e-01)*a[5])/(real_t)(8.0000000000000000e+00)))-(real_t)(-5.9999999999999998e-01))*((((real_t)(5.0000000000000000e-01)*a[4])+(((real_t)(4.0000000000000002e-01)*a[5])/(real_t)(8.0000000000000000e+00)))-(real_t)(-5.9999999999999998e-01)))+(((((real_t)(5.0000000000000000e-01)*a[0])+(((real_t)(4.0000000000000002e-01)*a[2])/(real_t)(8.0000000000000000e+00)))-(real_t)(6.9999999999999996e-01))*((((real_t)(5.0000000000000000e-01)*a[0])+(((real_t)(4.0000000000000002e-01)*a[2])/(real_t)(8.0000000000000000e+00)))-(real_t)(6.9999999999999996e-01))))-a[6]);
a[8] = (((((((real_t)(5.0000000000000000e-01)*a[4])+(((real_t)(1.2000000000000002e+00)*a[5])/(real_t)(8.0000000000000000e+00)))-(real_t)(-5.9999999999999998e-01))*((((real_t)(5.0000000000000000e-01)*a[4])+(((real_t)(1.2000000000000002e+00)*a[5])/(real_t)(8.0000000000000000e+00)))-(real_t)(-5.9999999999999998e-01)))+(((((real_t)(5.0000000000000000e-01)*a[0])+(((real_t)(1.2000000000000002e+00)*a[2])/(real_t)(8.0000000000000000e+00)))-(real_t)(6.9999999999999996e-01))*((((real_t)(5.0000000000000000e-01)*a[0])+(((real_t)(1.2000000000000002e+00)*a[2])/(real_t)(8.0000000000000000e+00)))-(real_t)(6.9999999999999996e-01))))-a[6]);
a[9] = (((((((real_t)(5.0000000000000000e-01)*a[4])+(((real_t)(2.0000000000000000e+00)*a[5])/(real_t)(8.0000000000000000e+00)))-(real_t)(-5.9999999999999998e-01))*((((real_t)(5.0000000000000000e-01)*a[4])+(((real_t)(2.0000000000000000e+00)*a[5])/(real_t)(8.0000000000000000e+00)))-(real_t)(-5.9999999999999998e-01)))+(((((real_t)(5.0000000000000000e-01)*a[0])+(((real_t)(2.0000000000000000e+00)*a[2])/(real_t)(8.0000000000000000e+00)))-(real_t)(6.9999999999999996e-01))*((((real_t)(5.0000000000000000e-01)*a[0])+(((real_t)(2.0000000000000000e+00)*a[2])/(real_t)(8.0000000000000000e+00)))-(real_t)(6.9999999999999996e-01))))-a[6]);
a[10] = (((((((real_t)(5.0000000000000000e-01)*a[4])+(((real_t)(2.8000000000000003e+00)*a[5])/(real_t)(8.0000000000000000e+00)))-(real_t)(-5.9999999999999998e-01))*((((real_t)(5.0000000000000000e-01)*a[4])+(((real_t)(2.8000000000000003e+00)*a[5])/(real_t)(8.0000000000000000e+00)))-(real_t)(-5.9999999999999998e-01)))+(((((real_t)(5.0000000000000000e-01)*a[0])+(((real_t)(2.8000000000000003e+00)*a[2])/(real_t)(8.0000000000000000e+00)))-(real_t)(6.9999999999999996e-01))*((((real_t)(5.0000000000000000e-01)*a[0])+(((real_t)(2.8000000000000003e+00)*a[2])/(real_t)(8.0000000000000000e+00)))-(real_t)(6.9999999999999996e-01))))-a[6]);
a[11] = (((((((real_t)(5.0000000000000000e-01)*a[4])+(((real_t)(4.0000000000000002e-01)*a[5])/(real_t)(8.0000000000000000e+00)))-(real_t)(5.9999999999999998e-01))*((((real_t)(5.0000000000000000e-01)*a[4])+(((real_t)(4.0000000000000002e-01)*a[5])/(real_t)(8.0000000000000000e+00)))-(real_t)(5.9999999999999998e-01)))+(((((real_t)(5.0000000000000000e-01)*a[0])+(((real_t)(4.0000000000000002e-01)*a[2])/(real_t)(8.0000000000000000e+00)))-(real_t)(6.9999999999999996e-01))*((((real_t)(5.0000000000000000e-01)*a[0])+(((real_t)(4.0000000000000002e-01)*a[2])/(real_t)(8.0000000000000000e+00)))-(real_t)(6.9999999999999996e-01))))-a[6]);
a[12] = (((((((real_t)(5.0000000000000000e-01)*a[4])+(((real_t)(1.2000000000000002e+00)*a[5])/(real_t)(8.0000000000000000e+00)))-(real_t)(5.9999999999999998e-01))*((((real_t)(5.0000000000000000e-01)*a[4])+(((real_t)(1.2000000000000002e+00)*a[5])/(real_t)(8.0000000000000000e+00)))-(real_t)(5.9999999999999998e-01)))+(((((real_t)(5.0000000000000000e-01)*a[0])+(((real_t)(1.2000000000000002e+00)*a[2])/(real_t)(8.0000000000000000e+00)))-(real_t)(6.9999999999999996e-01))*((((real_t)(5.0000000000000000e-01)*a[0])+(((real_t)(1.2000000000000002e+00)*a[2])/(real_t)(8.0000000000000000e+00)))-(real_t)(6.9999999999999996e-01))))-a[6]);
a[13] = (((((((real_t)(5.0000000000000000e-01)*a[4])+(((real_t)(2.0000000000000000e+00)*a[5])/(real_t)(8.0000000000000000e+00)))-(real_t)(5.9999999999999998e-01))*((((real_t)(5.0000000000000000e-01)*a[4])+(((real_t)(2.0000000000000000e+00)*a[5])/(real_t)(8.0000000000000000e+00)))-(real_t)(5.9999999999999998e-01)))+(((((real_t)(5.0000000000000000e-01)*a[0])+(((real_t)(2.0000000000000000e+00)*a[2])/(real_t)(8.0000000000000000e+00)))-(real_t)(6.9999999999999996e-01))*((((real_t)(5.0000000000000000e-01)*a[0])+(((real_t)(2.0000000000000000e+00)*a[2])/(real_t)(8.0000000000000000e+00)))-(real_t)(6.9999999999999996e-01))))-a[6]);
a[14] = (((((((real_t)(5.0000000000000000e-01)*a[4])+(((real_t)(2.8000000000000003e+00)*a[5])/(real_t)(8.0000000000000000e+00)))-(real_t)(5.9999999999999998e-01))*((((real_t)(5.0000000000000000e-01)*a[4])+(((real_t)(2.8000000000000003e+00)*a[5])/(real_t)(8.0000000000000000e+00)))-(real_t)(5.9999999999999998e-01)))+(((((real_t)(5.0000000000000000e-01)*a[0])+(((real_t)(2.8000000000000003e+00)*a[2])/(real_t)(8.0000000000000000e+00)))-(real_t)(6.9999999999999996e-01))*((((real_t)(5.0000000000000000e-01)*a[0])+(((real_t)(2.8000000000000003e+00)*a[2])/(real_t)(8.0000000000000000e+00)))-(real_t)(6.9999999999999996e-01))))-a[6]);
a[15] = (cos(xd[0]));
a[16] = (real_t)(5.0000000000000000e-01);
a[17] = (a[15]*a[16]);
a[18] = (real_t)(0.0000000000000000e+00);
a[19] = (real_t)(5.0000000000000000e-01);
a[20] = (a[15]*a[19]);
a[21] = (cos((xd[0]+xd[1])));
a[22] = (real_t)(4.0000000000000002e-01);
a[23] = (a[21]*a[22]);
a[24] = (a[20]+a[23]);
a[25] = (a[21]*a[22]);
a[26] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[0])));
a[27] = (real_t)(5.0000000000000000e-01);
a[28] = (a[26]*a[27]);
a[29] = ((real_t)(-1.0000000000000000e+00)*(sin((xd[0]+xd[1]))));
a[30] = (real_t)(4.0000000000000002e-01);
a[31] = (a[29]*a[30]);
a[32] = ((real_t)(1.0000000000000000e+00)/(real_t)(8.0000000000000000e+00));
a[33] = (a[31]*a[32]);
a[34] = (a[28]+a[33]);
a[35] = ((((real_t)(5.0000000000000000e-01)*a[4])+(((real_t)(4.0000000000000002e-01)*a[5])/(real_t)(8.0000000000000000e+00)))-(real_t)(-5.9999999999999998e-01));
a[36] = (a[34]*a[35]);
a[37] = (real_t)(5.0000000000000000e-01);
a[38] = (a[26]*a[37]);
a[39] = (real_t)(4.0000000000000002e-01);
a[40] = (a[29]*a[39]);
a[41] = ((real_t)(1.0000000000000000e+00)/(real_t)(8.0000000000000000e+00));
a[42] = (a[40]*a[41]);
a[43] = (a[38]+a[42]);
a[44] = ((((real_t)(5.0000000000000000e-01)*a[4])+(((real_t)(4.0000000000000002e-01)*a[5])/(real_t)(8.0000000000000000e+00)))-(real_t)(-5.9999999999999998e-01));
a[45] = (a[43]*a[44]);
a[46] = (a[36]+a[45]);
a[47] = (real_t)(5.0000000000000000e-01);
a[48] = (a[15]*a[47]);
a[49] = (real_t)(4.0000000000000002e-01);
a[50] = (a[21]*a[49]);
a[51] = ((real_t)(1.0000000000000000e+00)/(real_t)(8.0000000000000000e+00));
a[52] = (a[50]*a[51]);
a[53] = (a[48]+a[52]);
a[54] = ((((real_t)(5.0000000000000000e-01)*a[0])+(((real_t)(4.0000000000000002e-01)*a[2])/(real_t)(8.0000000000000000e+00)))-(real_t)(6.9999999999999996e-01));
a[55] = (a[53]*a[54]);
a[56] = (real_t)(5.0000000000000000e-01);
a[57] = (a[15]*a[56]);
a[58] = (real_t)(4.0000000000000002e-01);
a[59] = (a[21]*a[58]);
a[60] = ((real_t)(1.0000000000000000e+00)/(real_t)(8.0000000000000000e+00));
a[61] = (a[59]*a[60]);
a[62] = (a[57]+a[61]);
a[63] = ((((real_t)(5.0000000000000000e-01)*a[0])+(((real_t)(4.0000000000000002e-01)*a[2])/(real_t)(8.0000000000000000e+00)))-(real_t)(6.9999999999999996e-01));
a[64] = (a[62]*a[63]);
a[65] = (a[55]+a[64]);
a[66] = (a[46]+a[65]);
a[67] = (a[29]*a[30]);
a[68] = (a[67]*a[32]);
a[69] = (a[68]*a[35]);
a[70] = (a[29]*a[39]);
a[71] = (a[70]*a[41]);
a[72] = (a[71]*a[44]);
a[73] = (a[69]+a[72]);
a[74] = (a[21]*a[49]);
a[75] = (a[74]*a[51]);
a[76] = (a[75]*a[54]);
a[77] = (a[21]*a[58]);
a[78] = (a[77]*a[60]);
a[79] = (a[78]*a[63]);
a[80] = (a[76]+a[79]);
a[81] = (a[73]+a[80]);
a[82] = (real_t)(5.0000000000000000e-01);
a[83] = (a[26]*a[82]);
a[84] = (real_t)(1.2000000000000002e+00);
a[85] = (a[29]*a[84]);
a[86] = ((real_t)(1.0000000000000000e+00)/(real_t)(8.0000000000000000e+00));
a[87] = (a[85]*a[86]);
a[88] = (a[83]+a[87]);
a[89] = ((((real_t)(5.0000000000000000e-01)*a[4])+(((real_t)(1.2000000000000002e+00)*a[5])/(real_t)(8.0000000000000000e+00)))-(real_t)(-5.9999999999999998e-01));
a[90] = (a[88]*a[89]);
a[91] = (real_t)(5.0000000000000000e-01);
a[92] = (a[26]*a[91]);
a[93] = (real_t)(1.2000000000000002e+00);
a[94] = (a[29]*a[93]);
a[95] = ((real_t)(1.0000000000000000e+00)/(real_t)(8.0000000000000000e+00));
a[96] = (a[94]*a[95]);
a[97] = (a[92]+a[96]);
a[98] = ((((real_t)(5.0000000000000000e-01)*a[4])+(((real_t)(1.2000000000000002e+00)*a[5])/(real_t)(8.0000000000000000e+00)))-(real_t)(-5.9999999999999998e-01));
a[99] = (a[97]*a[98]);
a[100] = (a[90]+a[99]);
a[101] = (real_t)(5.0000000000000000e-01);
a[102] = (a[15]*a[101]);
a[103] = (real_t)(1.2000000000000002e+00);
a[104] = (a[21]*a[103]);
a[105] = ((real_t)(1.0000000000000000e+00)/(real_t)(8.0000000000000000e+00));
a[106] = (a[104]*a[105]);
a[107] = (a[102]+a[106]);
a[108] = ((((real_t)(5.0000000000000000e-01)*a[0])+(((real_t)(1.2000000000000002e+00)*a[2])/(real_t)(8.0000000000000000e+00)))-(real_t)(6.9999999999999996e-01));
a[109] = (a[107]*a[108]);
a[110] = (real_t)(5.0000000000000000e-01);
a[111] = (a[15]*a[110]);
a[112] = (real_t)(1.2000000000000002e+00);
a[113] = (a[21]*a[112]);
a[114] = ((real_t)(1.0000000000000000e+00)/(real_t)(8.0000000000000000e+00));
a[115] = (a[113]*a[114]);
a[116] = (a[111]+a[115]);
a[117] = ((((real_t)(5.0000000000000000e-01)*a[0])+(((real_t)(1.2000000000000002e+00)*a[2])/(real_t)(8.0000000000000000e+00)))-(real_t)(6.9999999999999996e-01));
a[118] = (a[116]*a[117]);
a[119] = (a[109]+a[118]);
a[120] = (a[100]+a[119]);
a[121] = (a[29]*a[84]);
a[122] = (a[121]*a[86]);
a[123] = (a[122]*a[89]);
a[124] = (a[29]*a[93]);
a[125] = (a[124]*a[95]);
a[126] = (a[125]*a[98]);
a[127] = (a[123]+a[126]);
a[128] = (a[21]*a[103]);
a[129] = (a[128]*a[105]);
a[130] = (a[129]*a[108]);
a[131] = (a[21]*a[112]);
a[132] = (a[131]*a[114]);
a[133] = (a[132]*a[117]);
a[134] = (a[130]+a[133]);
a[135] = (a[127]+a[134]);
a[136] = (real_t)(5.0000000000000000e-01);
a[137] = (a[26]*a[136]);
a[138] = (real_t)(2.0000000000000000e+00);
a[139] = (a[29]*a[138]);
a[140] = ((real_t)(1.0000000000000000e+00)/(real_t)(8.0000000000000000e+00));
a[141] = (a[139]*a[140]);
a[142] = (a[137]+a[141]);
a[143] = ((((real_t)(5.0000000000000000e-01)*a[4])+(((real_t)(2.0000000000000000e+00)*a[5])/(real_t)(8.0000000000000000e+00)))-(real_t)(-5.9999999999999998e-01));
a[144] = (a[142]*a[143]);
a[145] = (real_t)(5.0000000000000000e-01);
a[146] = (a[26]*a[145]);
a[147] = (real_t)(2.0000000000000000e+00);
a[148] = (a[29]*a[147]);
a[149] = ((real_t)(1.0000000000000000e+00)/(real_t)(8.0000000000000000e+00));
a[150] = (a[148]*a[149]);
a[151] = (a[146]+a[150]);
a[152] = ((((real_t)(5.0000000000000000e-01)*a[4])+(((real_t)(2.0000000000000000e+00)*a[5])/(real_t)(8.0000000000000000e+00)))-(real_t)(-5.9999999999999998e-01));
a[153] = (a[151]*a[152]);
a[154] = (a[144]+a[153]);
a[155] = (real_t)(5.0000000000000000e-01);
a[156] = (a[15]*a[155]);
a[157] = (real_t)(2.0000000000000000e+00);
a[158] = (a[21]*a[157]);
a[159] = ((real_t)(1.0000000000000000e+00)/(real_t)(8.0000000000000000e+00));
a[160] = (a[158]*a[159]);
a[161] = (a[156]+a[160]);
a[162] = ((((real_t)(5.0000000000000000e-01)*a[0])+(((real_t)(2.0000000000000000e+00)*a[2])/(real_t)(8.0000000000000000e+00)))-(real_t)(6.9999999999999996e-01));
a[163] = (a[161]*a[162]);
a[164] = (real_t)(5.0000000000000000e-01);
a[165] = (a[15]*a[164]);
a[166] = (real_t)(2.0000000000000000e+00);
a[167] = (a[21]*a[166]);
a[168] = ((real_t)(1.0000000000000000e+00)/(real_t)(8.0000000000000000e+00));
a[169] = (a[167]*a[168]);
a[170] = (a[165]+a[169]);
a[171] = ((((real_t)(5.0000000000000000e-01)*a[0])+(((real_t)(2.0000000000000000e+00)*a[2])/(real_t)(8.0000000000000000e+00)))-(real_t)(6.9999999999999996e-01));
a[172] = (a[170]*a[171]);
a[173] = (a[163]+a[172]);
a[174] = (a[154]+a[173]);
a[175] = (a[29]*a[138]);
a[176] = (a[175]*a[140]);
a[177] = (a[176]*a[143]);
a[178] = (a[29]*a[147]);
a[179] = (a[178]*a[149]);
a[180] = (a[179]*a[152]);
a[181] = (a[177]+a[180]);
a[182] = (a[21]*a[157]);
a[183] = (a[182]*a[159]);
a[184] = (a[183]*a[162]);
a[185] = (a[21]*a[166]);
a[186] = (a[185]*a[168]);
a[187] = (a[186]*a[171]);
a[188] = (a[184]+a[187]);
a[189] = (a[181]+a[188]);
a[190] = (real_t)(5.0000000000000000e-01);
a[191] = (a[26]*a[190]);
a[192] = (real_t)(2.8000000000000003e+00);
a[193] = (a[29]*a[192]);
a[194] = ((real_t)(1.0000000000000000e+00)/(real_t)(8.0000000000000000e+00));
a[195] = (a[193]*a[194]);
a[196] = (a[191]+a[195]);
a[197] = ((((real_t)(5.0000000000000000e-01)*a[4])+(((real_t)(2.8000000000000003e+00)*a[5])/(real_t)(8.0000000000000000e+00)))-(real_t)(-5.9999999999999998e-01));
a[198] = (a[196]*a[197]);
a[199] = (real_t)(5.0000000000000000e-01);
a[200] = (a[26]*a[199]);
a[201] = (real_t)(2.8000000000000003e+00);
a[202] = (a[29]*a[201]);
a[203] = ((real_t)(1.0000000000000000e+00)/(real_t)(8.0000000000000000e+00));
a[204] = (a[202]*a[203]);
a[205] = (a[200]+a[204]);
a[206] = ((((real_t)(5.0000000000000000e-01)*a[4])+(((real_t)(2.8000000000000003e+00)*a[5])/(real_t)(8.0000000000000000e+00)))-(real_t)(-5.9999999999999998e-01));
a[207] = (a[205]*a[206]);
a[208] = (a[198]+a[207]);
a[209] = (real_t)(5.0000000000000000e-01);
a[210] = (a[15]*a[209]);
a[211] = (real_t)(2.8000000000000003e+00);
a[212] = (a[21]*a[211]);
a[213] = ((real_t)(1.0000000000000000e+00)/(real_t)(8.0000000000000000e+00));
a[214] = (a[212]*a[213]);
a[215] = (a[210]+a[214]);
a[216] = ((((real_t)(5.0000000000000000e-01)*a[0])+(((real_t)(2.8000000000000003e+00)*a[2])/(real_t)(8.0000000000000000e+00)))-(real_t)(6.9999999999999996e-01));
a[217] = (a[215]*a[216]);
a[218] = (real_t)(5.0000000000000000e-01);
a[219] = (a[15]*a[218]);
a[220] = (real_t)(2.8000000000000003e+00);
a[221] = (a[21]*a[220]);
a[222] = ((real_t)(1.0000000000000000e+00)/(real_t)(8.0000000000000000e+00));
a[223] = (a[221]*a[222]);
a[224] = (a[219]+a[223]);
a[225] = ((((real_t)(5.0000000000000000e-01)*a[0])+(((real_t)(2.8000000000000003e+00)*a[2])/(real_t)(8.0000000000000000e+00)))-(real_t)(6.9999999999999996e-01));
a[226] = (a[224]*a[225]);
a[227] = (a[217]+a[226]);
a[228] = (a[208]+a[227]);
a[229] = (a[29]*a[192]);
a[230] = (a[229]*a[194]);
a[231] = (a[230]*a[197]);
a[232] = (a[29]*a[201]);
a[233] = (a[232]*a[203]);
a[234] = (a[233]*a[206]);
a[235] = (a[231]+a[234]);
a[236] = (a[21]*a[211]);
a[237] = (a[236]*a[213]);
a[238] = (a[237]*a[216]);
a[239] = (a[21]*a[220]);
a[240] = (a[239]*a[222]);
a[241] = (a[240]*a[225]);
a[242] = (a[238]+a[241]);
a[243] = (a[235]+a[242]);
a[244] = (real_t)(5.0000000000000000e-01);
a[245] = (a[26]*a[244]);
a[246] = (real_t)(4.0000000000000002e-01);
a[247] = (a[29]*a[246]);
a[248] = ((real_t)(1.0000000000000000e+00)/(real_t)(8.0000000000000000e+00));
a[249] = (a[247]*a[248]);
a[250] = (a[245]+a[249]);
a[251] = ((((real_t)(5.0000000000000000e-01)*a[4])+(((real_t)(4.0000000000000002e-01)*a[5])/(real_t)(8.0000000000000000e+00)))-(real_t)(5.9999999999999998e-01));
a[252] = (a[250]*a[251]);
a[253] = (real_t)(5.0000000000000000e-01);
a[254] = (a[26]*a[253]);
a[255] = (real_t)(4.0000000000000002e-01);
a[256] = (a[29]*a[255]);
a[257] = ((real_t)(1.0000000000000000e+00)/(real_t)(8.0000000000000000e+00));
a[258] = (a[256]*a[257]);
a[259] = (a[254]+a[258]);
a[260] = ((((real_t)(5.0000000000000000e-01)*a[4])+(((real_t)(4.0000000000000002e-01)*a[5])/(real_t)(8.0000000000000000e+00)))-(real_t)(5.9999999999999998e-01));
a[261] = (a[259]*a[260]);
a[262] = (a[252]+a[261]);
a[263] = (real_t)(5.0000000000000000e-01);
a[264] = (a[15]*a[263]);
a[265] = (real_t)(4.0000000000000002e-01);
a[266] = (a[21]*a[265]);
a[267] = ((real_t)(1.0000000000000000e+00)/(real_t)(8.0000000000000000e+00));
a[268] = (a[266]*a[267]);
a[269] = (a[264]+a[268]);
a[270] = ((((real_t)(5.0000000000000000e-01)*a[0])+(((real_t)(4.0000000000000002e-01)*a[2])/(real_t)(8.0000000000000000e+00)))-(real_t)(6.9999999999999996e-01));
a[271] = (a[269]*a[270]);
a[272] = (real_t)(5.0000000000000000e-01);
a[273] = (a[15]*a[272]);
a[274] = (real_t)(4.0000000000000002e-01);
a[275] = (a[21]*a[274]);
a[276] = ((real_t)(1.0000000000000000e+00)/(real_t)(8.0000000000000000e+00));
a[277] = (a[275]*a[276]);
a[278] = (a[273]+a[277]);
a[279] = ((((real_t)(5.0000000000000000e-01)*a[0])+(((real_t)(4.0000000000000002e-01)*a[2])/(real_t)(8.0000000000000000e+00)))-(real_t)(6.9999999999999996e-01));
a[280] = (a[278]*a[279]);
a[281] = (a[271]+a[280]);
a[282] = (a[262]+a[281]);
a[283] = (a[29]*a[246]);
a[284] = (a[283]*a[248]);
a[285] = (a[284]*a[251]);
a[286] = (a[29]*a[255]);
a[287] = (a[286]*a[257]);
a[288] = (a[287]*a[260]);
a[289] = (a[285]+a[288]);
a[290] = (a[21]*a[265]);
a[291] = (a[290]*a[267]);
a[292] = (a[291]*a[270]);
a[293] = (a[21]*a[274]);
a[294] = (a[293]*a[276]);
a[295] = (a[294]*a[279]);
a[296] = (a[292]+a[295]);
a[297] = (a[289]+a[296]);
a[298] = (real_t)(5.0000000000000000e-01);
a[299] = (a[26]*a[298]);
a[300] = (real_t)(1.2000000000000002e+00);
a[301] = (a[29]*a[300]);
a[302] = ((real_t)(1.0000000000000000e+00)/(real_t)(8.0000000000000000e+00));
a[303] = (a[301]*a[302]);
a[304] = (a[299]+a[303]);
a[305] = ((((real_t)(5.0000000000000000e-01)*a[4])+(((real_t)(1.2000000000000002e+00)*a[5])/(real_t)(8.0000000000000000e+00)))-(real_t)(5.9999999999999998e-01));
a[306] = (a[304]*a[305]);
a[307] = (real_t)(5.0000000000000000e-01);
a[308] = (a[26]*a[307]);
a[309] = (real_t)(1.2000000000000002e+00);
a[310] = (a[29]*a[309]);
a[311] = ((real_t)(1.0000000000000000e+00)/(real_t)(8.0000000000000000e+00));
a[312] = (a[310]*a[311]);
a[313] = (a[308]+a[312]);
a[314] = ((((real_t)(5.0000000000000000e-01)*a[4])+(((real_t)(1.2000000000000002e+00)*a[5])/(real_t)(8.0000000000000000e+00)))-(real_t)(5.9999999999999998e-01));
a[315] = (a[313]*a[314]);
a[316] = (a[306]+a[315]);
a[317] = (real_t)(5.0000000000000000e-01);
a[318] = (a[15]*a[317]);
a[319] = (real_t)(1.2000000000000002e+00);
a[320] = (a[21]*a[319]);
a[321] = ((real_t)(1.0000000000000000e+00)/(real_t)(8.0000000000000000e+00));
a[322] = (a[320]*a[321]);
a[323] = (a[318]+a[322]);
a[324] = ((((real_t)(5.0000000000000000e-01)*a[0])+(((real_t)(1.2000000000000002e+00)*a[2])/(real_t)(8.0000000000000000e+00)))-(real_t)(6.9999999999999996e-01));
a[325] = (a[323]*a[324]);
a[326] = (real_t)(5.0000000000000000e-01);
a[327] = (a[15]*a[326]);
a[328] = (real_t)(1.2000000000000002e+00);
a[329] = (a[21]*a[328]);
a[330] = ((real_t)(1.0000000000000000e+00)/(real_t)(8.0000000000000000e+00));
a[331] = (a[329]*a[330]);
a[332] = (a[327]+a[331]);
a[333] = ((((real_t)(5.0000000000000000e-01)*a[0])+(((real_t)(1.2000000000000002e+00)*a[2])/(real_t)(8.0000000000000000e+00)))-(real_t)(6.9999999999999996e-01));
a[334] = (a[332]*a[333]);
a[335] = (a[325]+a[334]);
a[336] = (a[316]+a[335]);
a[337] = (a[29]*a[300]);
a[338] = (a[337]*a[302]);
a[339] = (a[338]*a[305]);
a[340] = (a[29]*a[309]);
a[341] = (a[340]*a[311]);
a[342] = (a[341]*a[314]);
a[343] = (a[339]+a[342]);
a[344] = (a[21]*a[319]);
a[345] = (a[344]*a[321]);
a[346] = (a[345]*a[324]);
a[347] = (a[21]*a[328]);
a[348] = (a[347]*a[330]);
a[349] = (a[348]*a[333]);
a[350] = (a[346]+a[349]);
a[351] = (a[343]+a[350]);
a[352] = (real_t)(5.0000000000000000e-01);
a[353] = (a[26]*a[352]);
a[354] = (real_t)(2.0000000000000000e+00);
a[355] = (a[29]*a[354]);
a[356] = ((real_t)(1.0000000000000000e+00)/(real_t)(8.0000000000000000e+00));
a[357] = (a[355]*a[356]);
a[358] = (a[353]+a[357]);
a[359] = ((((real_t)(5.0000000000000000e-01)*a[4])+(((real_t)(2.0000000000000000e+00)*a[5])/(real_t)(8.0000000000000000e+00)))-(real_t)(5.9999999999999998e-01));
a[360] = (a[358]*a[359]);
a[361] = (real_t)(5.0000000000000000e-01);
a[362] = (a[26]*a[361]);
a[363] = (real_t)(2.0000000000000000e+00);
a[364] = (a[29]*a[363]);
a[365] = ((real_t)(1.0000000000000000e+00)/(real_t)(8.0000000000000000e+00));
a[366] = (a[364]*a[365]);
a[367] = (a[362]+a[366]);
a[368] = ((((real_t)(5.0000000000000000e-01)*a[4])+(((real_t)(2.0000000000000000e+00)*a[5])/(real_t)(8.0000000000000000e+00)))-(real_t)(5.9999999999999998e-01));
a[369] = (a[367]*a[368]);
a[370] = (a[360]+a[369]);
a[371] = (real_t)(5.0000000000000000e-01);
a[372] = (a[15]*a[371]);
a[373] = (real_t)(2.0000000000000000e+00);
a[374] = (a[21]*a[373]);
a[375] = ((real_t)(1.0000000000000000e+00)/(real_t)(8.0000000000000000e+00));
a[376] = (a[374]*a[375]);
a[377] = (a[372]+a[376]);
a[378] = ((((real_t)(5.0000000000000000e-01)*a[0])+(((real_t)(2.0000000000000000e+00)*a[2])/(real_t)(8.0000000000000000e+00)))-(real_t)(6.9999999999999996e-01));
a[379] = (a[377]*a[378]);
a[380] = (real_t)(5.0000000000000000e-01);
a[381] = (a[15]*a[380]);
a[382] = (real_t)(2.0000000000000000e+00);
a[383] = (a[21]*a[382]);
a[384] = ((real_t)(1.0000000000000000e+00)/(real_t)(8.0000000000000000e+00));
a[385] = (a[383]*a[384]);
a[386] = (a[381]+a[385]);
a[387] = ((((real_t)(5.0000000000000000e-01)*a[0])+(((real_t)(2.0000000000000000e+00)*a[2])/(real_t)(8.0000000000000000e+00)))-(real_t)(6.9999999999999996e-01));
a[388] = (a[386]*a[387]);
a[389] = (a[379]+a[388]);
a[390] = (a[370]+a[389]);
a[391] = (a[29]*a[354]);
a[392] = (a[391]*a[356]);
a[393] = (a[392]*a[359]);
a[394] = (a[29]*a[363]);
a[395] = (a[394]*a[365]);
a[396] = (a[395]*a[368]);
a[397] = (a[393]+a[396]);
a[398] = (a[21]*a[373]);
a[399] = (a[398]*a[375]);
a[400] = (a[399]*a[378]);
a[401] = (a[21]*a[382]);
a[402] = (a[401]*a[384]);
a[403] = (a[402]*a[387]);
a[404] = (a[400]+a[403]);
a[405] = (a[397]+a[404]);
a[406] = (real_t)(5.0000000000000000e-01);
a[407] = (a[26]*a[406]);
a[408] = (real_t)(2.8000000000000003e+00);
a[409] = (a[29]*a[408]);
a[410] = ((real_t)(1.0000000000000000e+00)/(real_t)(8.0000000000000000e+00));
a[411] = (a[409]*a[410]);
a[412] = (a[407]+a[411]);
a[413] = ((((real_t)(5.0000000000000000e-01)*a[4])+(((real_t)(2.8000000000000003e+00)*a[5])/(real_t)(8.0000000000000000e+00)))-(real_t)(5.9999999999999998e-01));
a[414] = (a[412]*a[413]);
a[415] = (real_t)(5.0000000000000000e-01);
a[416] = (a[26]*a[415]);
a[417] = (real_t)(2.8000000000000003e+00);
a[418] = (a[29]*a[417]);
a[419] = ((real_t)(1.0000000000000000e+00)/(real_t)(8.0000000000000000e+00));
a[420] = (a[418]*a[419]);
a[421] = (a[416]+a[420]);
a[422] = ((((real_t)(5.0000000000000000e-01)*a[4])+(((real_t)(2.8000000000000003e+00)*a[5])/(real_t)(8.0000000000000000e+00)))-(real_t)(5.9999999999999998e-01));
a[423] = (a[421]*a[422]);
a[424] = (a[414]+a[423]);
a[425] = (real_t)(5.0000000000000000e-01);
a[426] = (a[15]*a[425]);
a[427] = (real_t)(2.8000000000000003e+00);
a[428] = (a[21]*a[427]);
a[429] = ((real_t)(1.0000000000000000e+00)/(real_t)(8.0000000000000000e+00));
a[430] = (a[428]*a[429]);
a[431] = (a[426]+a[430]);
a[432] = ((((real_t)(5.0000000000000000e-01)*a[0])+(((real_t)(2.8000000000000003e+00)*a[2])/(real_t)(8.0000000000000000e+00)))-(real_t)(6.9999999999999996e-01));
a[433] = (a[431]*a[432]);
a[434] = (real_t)(5.0000000000000000e-01);
a[435] = (a[15]*a[434]);
a[436] = (real_t)(2.8000000000000003e+00);
a[437] = (a[21]*a[436]);
a[438] = ((real_t)(1.0000000000000000e+00)/(real_t)(8.0000000000000000e+00));
a[439] = (a[437]*a[438]);
a[440] = (a[435]+a[439]);
a[441] = ((((real_t)(5.0000000000000000e-01)*a[0])+(((real_t)(2.8000000000000003e+00)*a[2])/(real_t)(8.0000000000000000e+00)))-(real_t)(6.9999999999999996e-01));
a[442] = (a[440]*a[441]);
a[443] = (a[433]+a[442]);
a[444] = (a[424]+a[443]);
a[445] = (a[29]*a[408]);
a[446] = (a[445]*a[410]);
a[447] = (a[446]*a[413]);
a[448] = (a[29]*a[417]);
a[449] = (a[448]*a[419]);
a[450] = (a[449]*a[422]);
a[451] = (a[447]+a[450]);
a[452] = (a[21]*a[427]);
a[453] = (a[452]*a[429]);
a[454] = (a[453]*a[432]);
a[455] = (a[21]*a[436]);
a[456] = (a[455]*a[438]);
a[457] = (a[456]*a[441]);
a[458] = (a[454]+a[457]);
a[459] = (a[451]+a[458]);
a[460] = (real_t)(0.0000000000000000e+00);
a[461] = (real_t)(0.0000000000000000e+00);
a[462] = (real_t)(0.0000000000000000e+00);
a[463] = (real_t)(0.0000000000000000e+00);
a[464] = (real_t)(0.0000000000000000e+00);
a[465] = (real_t)(0.0000000000000000e+00);
a[466] = (real_t)(0.0000000000000000e+00);
a[467] = (real_t)(0.0000000000000000e+00);
a[468] = (real_t)(0.0000000000000000e+00);
a[469] = (real_t)(0.0000000000000000e+00);
a[470] = (real_t)(0.0000000000000000e+00);
a[471] = (real_t)(0.0000000000000000e+00);
a[472] = (real_t)(0.0000000000000000e+00);
a[473] = (real_t)(0.0000000000000000e+00);
a[474] = (real_t)(0.0000000000000000e+00);
a[475] = (real_t)(0.0000000000000000e+00);
a[476] = (real_t)(0.0000000000000000e+00);
a[477] = (real_t)(0.0000000000000000e+00);
a[478] = (real_t)(0.0000000000000000e+00);
a[479] = (real_t)(0.0000000000000000e+00);

/* Compute outputs: */
out[0] = a[1];
out[1] = a[3];
out[2] = a[7];
out[3] = a[8];
out[4] = a[9];
out[5] = a[10];
out[6] = a[11];
out[7] = a[12];
out[8] = a[13];
out[9] = a[14];
out[10] = a[17];
out[11] = a[18];
out[12] = a[24];
out[13] = a[25];
out[14] = a[66];
out[15] = a[81];
out[16] = a[120];
out[17] = a[135];
out[18] = a[174];
out[19] = a[189];
out[20] = a[228];
out[21] = a[243];
out[22] = a[282];
out[23] = a[297];
out[24] = a[336];
out[25] = a[351];
out[26] = a[390];
out[27] = a[405];
out[28] = a[444];
out[29] = a[459];
out[30] = a[460];
out[31] = a[461];
out[32] = a[462];
out[33] = a[463];
out[34] = a[464];
out[35] = a[465];
out[36] = a[466];
out[37] = a[467];
out[38] = a[468];
out[39] = a[469];
out[40] = a[470];
out[41] = a[471];
out[42] = a[472];
out[43] = a[473];
out[44] = a[474];
out[45] = a[475];
out[46] = a[476];
out[47] = a[477];
out[48] = a[478];
out[49] = a[479];
}

void acado_macETSlu( real_t* const E0, real_t* const g1 )
{
g1[0] += 0.0;
;
g1[1] += 0.0;
;
}

void acado_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
/** Row vector of size: 20 */
static const int xBoundIndices[ 20 ] = 
{ 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21 };
acado_moveGuE( acadoWorkspace.evGu, acadoWorkspace.E );
acado_moveGxT( &(acadoWorkspace.evGx[ 4 ]), acadoWorkspace.T );
acado_multGxd( acadoWorkspace.d, &(acadoWorkspace.evGx[ 4 ]), &(acadoWorkspace.d[ 2 ]) );
acado_multGxGx( acadoWorkspace.T, acadoWorkspace.evGx, &(acadoWorkspace.evGx[ 4 ]) );

acado_multGxGu( acadoWorkspace.T, acadoWorkspace.E, &(acadoWorkspace.E[ 4 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 4 ]), &(acadoWorkspace.E[ 8 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 8 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 2 ]), &(acadoWorkspace.evGx[ 8 ]), &(acadoWorkspace.d[ 4 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 4 ]), &(acadoWorkspace.evGx[ 8 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 4 ]), &(acadoWorkspace.E[ 12 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 8 ]), &(acadoWorkspace.E[ 16 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 8 ]), &(acadoWorkspace.E[ 20 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 12 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 4 ]), &(acadoWorkspace.evGx[ 12 ]), &(acadoWorkspace.d[ 6 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 8 ]), &(acadoWorkspace.evGx[ 12 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.E[ 24 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 16 ]), &(acadoWorkspace.E[ 28 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 20 ]), &(acadoWorkspace.E[ 32 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 12 ]), &(acadoWorkspace.E[ 36 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 16 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 6 ]), &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.d[ 8 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 12 ]), &(acadoWorkspace.evGx[ 16 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.E[ 40 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 28 ]), &(acadoWorkspace.E[ 44 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 32 ]), &(acadoWorkspace.E[ 48 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.E[ 52 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 16 ]), &(acadoWorkspace.E[ 56 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 20 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 8 ]), &(acadoWorkspace.evGx[ 20 ]), &(acadoWorkspace.d[ 10 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.evGx[ 20 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 40 ]), &(acadoWorkspace.E[ 60 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 44 ]), &(acadoWorkspace.E[ 64 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.E[ 68 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 52 ]), &(acadoWorkspace.E[ 72 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 56 ]), &(acadoWorkspace.E[ 76 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 20 ]), &(acadoWorkspace.E[ 80 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 24 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 10 ]), &(acadoWorkspace.evGx[ 24 ]), &(acadoWorkspace.d[ 12 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 20 ]), &(acadoWorkspace.evGx[ 24 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.E[ 84 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 64 ]), &(acadoWorkspace.E[ 88 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 68 ]), &(acadoWorkspace.E[ 92 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.E[ 96 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 76 ]), &(acadoWorkspace.E[ 100 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.E[ 104 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 24 ]), &(acadoWorkspace.E[ 108 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 28 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 12 ]), &(acadoWorkspace.evGx[ 28 ]), &(acadoWorkspace.d[ 14 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 24 ]), &(acadoWorkspace.evGx[ 28 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.E[ 112 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 88 ]), &(acadoWorkspace.E[ 116 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 92 ]), &(acadoWorkspace.E[ 120 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.E[ 124 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 100 ]), &(acadoWorkspace.E[ 128 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 104 ]), &(acadoWorkspace.E[ 132 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.E[ 136 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 28 ]), &(acadoWorkspace.E[ 140 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 32 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 14 ]), &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.d[ 16 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 28 ]), &(acadoWorkspace.evGx[ 32 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 112 ]), &(acadoWorkspace.E[ 144 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 116 ]), &(acadoWorkspace.E[ 148 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.E[ 152 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 124 ]), &(acadoWorkspace.E[ 156 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 128 ]), &(acadoWorkspace.E[ 160 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.E[ 164 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 136 ]), &(acadoWorkspace.E[ 168 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 140 ]), &(acadoWorkspace.E[ 172 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 32 ]), &(acadoWorkspace.E[ 176 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 36 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 16 ]), &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.d[ 18 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.evGx[ 36 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.E[ 180 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 148 ]), &(acadoWorkspace.E[ 184 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 152 ]), &(acadoWorkspace.E[ 188 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.E[ 192 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.E[ 196 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 164 ]), &(acadoWorkspace.E[ 200 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.E[ 204 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 172 ]), &(acadoWorkspace.E[ 208 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 176 ]), &(acadoWorkspace.E[ 212 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 36 ]), &(acadoWorkspace.E[ 216 ]) );

acado_multQ1Gu( acadoWorkspace.E, acadoWorkspace.QE );
acado_multQ1Gu( &(acadoWorkspace.E[ 4 ]), &(acadoWorkspace.QE[ 4 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 8 ]), &(acadoWorkspace.QE[ 8 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.QE[ 12 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 16 ]), &(acadoWorkspace.QE[ 16 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 20 ]), &(acadoWorkspace.QE[ 20 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.QE[ 24 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 28 ]), &(acadoWorkspace.QE[ 28 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 32 ]), &(acadoWorkspace.QE[ 32 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QE[ 36 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 40 ]), &(acadoWorkspace.QE[ 40 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 44 ]), &(acadoWorkspace.QE[ 44 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 48 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 52 ]), &(acadoWorkspace.QE[ 52 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 56 ]), &(acadoWorkspace.QE[ 56 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 60 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 64 ]), &(acadoWorkspace.QE[ 64 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 68 ]), &(acadoWorkspace.QE[ 68 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 72 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 76 ]), &(acadoWorkspace.QE[ 76 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.QE[ 80 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 84 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 88 ]), &(acadoWorkspace.QE[ 88 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 92 ]), &(acadoWorkspace.QE[ 92 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 100 ]), &(acadoWorkspace.QE[ 100 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 104 ]), &(acadoWorkspace.QE[ 104 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QE[ 108 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 112 ]), &(acadoWorkspace.QE[ 112 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 116 ]), &(acadoWorkspace.QE[ 116 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 124 ]), &(acadoWorkspace.QE[ 124 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 128 ]), &(acadoWorkspace.QE[ 128 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QE[ 132 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 136 ]), &(acadoWorkspace.QE[ 136 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 140 ]), &(acadoWorkspace.QE[ 140 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 148 ]), &(acadoWorkspace.QE[ 148 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 152 ]), &(acadoWorkspace.QE[ 152 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.QE[ 160 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 164 ]), &(acadoWorkspace.QE[ 164 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 168 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 172 ]), &(acadoWorkspace.QE[ 172 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 176 ]), &(acadoWorkspace.QE[ 176 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 180 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 184 ]), &(acadoWorkspace.QE[ 184 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 188 ]), &(acadoWorkspace.QE[ 188 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 196 ]), &(acadoWorkspace.QE[ 196 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 200 ]), &(acadoWorkspace.QE[ 200 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QE[ 204 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 208 ]), &(acadoWorkspace.QE[ 208 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 212 ]), &(acadoWorkspace.QE[ 212 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 216 ]) );

acado_zeroBlockH10( acadoWorkspace.H10 );
acado_multQETGx( acadoWorkspace.QE, acadoWorkspace.evGx, acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 4 ]), &(acadoWorkspace.evGx[ 4 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 12 ]), &(acadoWorkspace.evGx[ 8 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 24 ]), &(acadoWorkspace.evGx[ 12 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 40 ]), &(acadoWorkspace.evGx[ 16 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 60 ]), &(acadoWorkspace.evGx[ 20 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 84 ]), &(acadoWorkspace.evGx[ 24 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 112 ]), &(acadoWorkspace.evGx[ 28 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 144 ]), &(acadoWorkspace.evGx[ 32 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 180 ]), &(acadoWorkspace.evGx[ 36 ]), acadoWorkspace.H10 );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 4 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 8 ]), &(acadoWorkspace.evGx[ 4 ]), &(acadoWorkspace.H10[ 4 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 16 ]), &(acadoWorkspace.evGx[ 8 ]), &(acadoWorkspace.H10[ 4 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 28 ]), &(acadoWorkspace.evGx[ 12 ]), &(acadoWorkspace.H10[ 4 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 44 ]), &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.H10[ 4 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 64 ]), &(acadoWorkspace.evGx[ 20 ]), &(acadoWorkspace.H10[ 4 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 88 ]), &(acadoWorkspace.evGx[ 24 ]), &(acadoWorkspace.H10[ 4 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 116 ]), &(acadoWorkspace.evGx[ 28 ]), &(acadoWorkspace.H10[ 4 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 148 ]), &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.H10[ 4 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 184 ]), &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.H10[ 4 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 8 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 20 ]), &(acadoWorkspace.evGx[ 8 ]), &(acadoWorkspace.H10[ 8 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 32 ]), &(acadoWorkspace.evGx[ 12 ]), &(acadoWorkspace.H10[ 8 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 48 ]), &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.H10[ 8 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 68 ]), &(acadoWorkspace.evGx[ 20 ]), &(acadoWorkspace.H10[ 8 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 92 ]), &(acadoWorkspace.evGx[ 24 ]), &(acadoWorkspace.H10[ 8 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 120 ]), &(acadoWorkspace.evGx[ 28 ]), &(acadoWorkspace.H10[ 8 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 152 ]), &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.H10[ 8 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 188 ]), &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.H10[ 8 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 36 ]), &(acadoWorkspace.evGx[ 12 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 52 ]), &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 72 ]), &(acadoWorkspace.evGx[ 20 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 96 ]), &(acadoWorkspace.evGx[ 24 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 124 ]), &(acadoWorkspace.evGx[ 28 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 156 ]), &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 192 ]), &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 56 ]), &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 76 ]), &(acadoWorkspace.evGx[ 20 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 100 ]), &(acadoWorkspace.evGx[ 24 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 128 ]), &(acadoWorkspace.evGx[ 28 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 160 ]), &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 196 ]), &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 20 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 80 ]), &(acadoWorkspace.evGx[ 20 ]), &(acadoWorkspace.H10[ 20 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 104 ]), &(acadoWorkspace.evGx[ 24 ]), &(acadoWorkspace.H10[ 20 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 132 ]), &(acadoWorkspace.evGx[ 28 ]), &(acadoWorkspace.H10[ 20 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 164 ]), &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.H10[ 20 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 200 ]), &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.H10[ 20 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 108 ]), &(acadoWorkspace.evGx[ 24 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 136 ]), &(acadoWorkspace.evGx[ 28 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 168 ]), &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 204 ]), &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 28 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 140 ]), &(acadoWorkspace.evGx[ 28 ]), &(acadoWorkspace.H10[ 28 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 172 ]), &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.H10[ 28 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 208 ]), &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.H10[ 28 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 176 ]), &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 212 ]), &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 216 ]), &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.H10[ 36 ]) );

acado_setBlockH11_R1( 0, 0 );
acado_setBlockH11( 0, 0, acadoWorkspace.E, acadoWorkspace.QE );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 4 ]), &(acadoWorkspace.QE[ 4 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.QE[ 12 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.QE[ 24 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 40 ]), &(acadoWorkspace.QE[ 40 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 60 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 84 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 112 ]), &(acadoWorkspace.QE[ 112 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 180 ]) );

acado_zeroBlockH11( 0, 1 );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 4 ]), &(acadoWorkspace.QE[ 8 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.QE[ 16 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.QE[ 28 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 40 ]), &(acadoWorkspace.QE[ 44 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 64 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 88 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 112 ]), &(acadoWorkspace.QE[ 116 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 148 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 184 ]) );

acado_zeroBlockH11( 0, 2 );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.QE[ 20 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.QE[ 32 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 40 ]), &(acadoWorkspace.QE[ 48 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 68 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 92 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 112 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 152 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 188 ]) );

acado_zeroBlockH11( 0, 3 );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.QE[ 36 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 40 ]), &(acadoWorkspace.QE[ 52 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 72 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 112 ]), &(acadoWorkspace.QE[ 124 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 192 ]) );

acado_zeroBlockH11( 0, 4 );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 40 ]), &(acadoWorkspace.QE[ 56 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 76 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 100 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 112 ]), &(acadoWorkspace.QE[ 128 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 160 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 196 ]) );

acado_zeroBlockH11( 0, 5 );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 80 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 104 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 112 ]), &(acadoWorkspace.QE[ 132 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 164 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 200 ]) );

acado_zeroBlockH11( 0, 6 );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 108 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 112 ]), &(acadoWorkspace.QE[ 136 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 168 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 204 ]) );

acado_zeroBlockH11( 0, 7 );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 112 ]), &(acadoWorkspace.QE[ 140 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 172 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 208 ]) );

acado_zeroBlockH11( 0, 8 );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 176 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 212 ]) );

acado_zeroBlockH11( 0, 9 );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 216 ]) );

acado_setBlockH11_R1( 1, 1 );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 8 ]), &(acadoWorkspace.QE[ 8 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 16 ]), &(acadoWorkspace.QE[ 16 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 28 ]), &(acadoWorkspace.QE[ 28 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 44 ]), &(acadoWorkspace.QE[ 44 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 64 ]), &(acadoWorkspace.QE[ 64 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 88 ]), &(acadoWorkspace.QE[ 88 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 116 ]), &(acadoWorkspace.QE[ 116 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 148 ]), &(acadoWorkspace.QE[ 148 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 184 ]), &(acadoWorkspace.QE[ 184 ]) );

acado_zeroBlockH11( 1, 2 );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 16 ]), &(acadoWorkspace.QE[ 20 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 28 ]), &(acadoWorkspace.QE[ 32 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 44 ]), &(acadoWorkspace.QE[ 48 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 64 ]), &(acadoWorkspace.QE[ 68 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 88 ]), &(acadoWorkspace.QE[ 92 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 116 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 148 ]), &(acadoWorkspace.QE[ 152 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 184 ]), &(acadoWorkspace.QE[ 188 ]) );

acado_zeroBlockH11( 1, 3 );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 28 ]), &(acadoWorkspace.QE[ 36 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 44 ]), &(acadoWorkspace.QE[ 52 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 64 ]), &(acadoWorkspace.QE[ 72 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 88 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 116 ]), &(acadoWorkspace.QE[ 124 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 148 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 184 ]), &(acadoWorkspace.QE[ 192 ]) );

acado_zeroBlockH11( 1, 4 );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 44 ]), &(acadoWorkspace.QE[ 56 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 64 ]), &(acadoWorkspace.QE[ 76 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 88 ]), &(acadoWorkspace.QE[ 100 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 116 ]), &(acadoWorkspace.QE[ 128 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 148 ]), &(acadoWorkspace.QE[ 160 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 184 ]), &(acadoWorkspace.QE[ 196 ]) );

acado_zeroBlockH11( 1, 5 );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 64 ]), &(acadoWorkspace.QE[ 80 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 88 ]), &(acadoWorkspace.QE[ 104 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 116 ]), &(acadoWorkspace.QE[ 132 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 148 ]), &(acadoWorkspace.QE[ 164 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 184 ]), &(acadoWorkspace.QE[ 200 ]) );

acado_zeroBlockH11( 1, 6 );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 88 ]), &(acadoWorkspace.QE[ 108 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 116 ]), &(acadoWorkspace.QE[ 136 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 148 ]), &(acadoWorkspace.QE[ 168 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 184 ]), &(acadoWorkspace.QE[ 204 ]) );

acado_zeroBlockH11( 1, 7 );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 116 ]), &(acadoWorkspace.QE[ 140 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 148 ]), &(acadoWorkspace.QE[ 172 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 184 ]), &(acadoWorkspace.QE[ 208 ]) );

acado_zeroBlockH11( 1, 8 );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 148 ]), &(acadoWorkspace.QE[ 176 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 184 ]), &(acadoWorkspace.QE[ 212 ]) );

acado_zeroBlockH11( 1, 9 );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 184 ]), &(acadoWorkspace.QE[ 216 ]) );

acado_setBlockH11_R1( 2, 2 );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 20 ]), &(acadoWorkspace.QE[ 20 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 32 ]), &(acadoWorkspace.QE[ 32 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 48 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 68 ]), &(acadoWorkspace.QE[ 68 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 92 ]), &(acadoWorkspace.QE[ 92 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 152 ]), &(acadoWorkspace.QE[ 152 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 188 ]), &(acadoWorkspace.QE[ 188 ]) );

acado_zeroBlockH11( 2, 3 );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 32 ]), &(acadoWorkspace.QE[ 36 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 52 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 68 ]), &(acadoWorkspace.QE[ 72 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 92 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 124 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 152 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 188 ]), &(acadoWorkspace.QE[ 192 ]) );

acado_zeroBlockH11( 2, 4 );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 56 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 68 ]), &(acadoWorkspace.QE[ 76 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 92 ]), &(acadoWorkspace.QE[ 100 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 128 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 152 ]), &(acadoWorkspace.QE[ 160 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 188 ]), &(acadoWorkspace.QE[ 196 ]) );

acado_zeroBlockH11( 2, 5 );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 68 ]), &(acadoWorkspace.QE[ 80 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 92 ]), &(acadoWorkspace.QE[ 104 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 132 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 152 ]), &(acadoWorkspace.QE[ 164 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 188 ]), &(acadoWorkspace.QE[ 200 ]) );

acado_zeroBlockH11( 2, 6 );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 92 ]), &(acadoWorkspace.QE[ 108 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 136 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 152 ]), &(acadoWorkspace.QE[ 168 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 188 ]), &(acadoWorkspace.QE[ 204 ]) );

acado_zeroBlockH11( 2, 7 );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 140 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 152 ]), &(acadoWorkspace.QE[ 172 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 188 ]), &(acadoWorkspace.QE[ 208 ]) );

acado_zeroBlockH11( 2, 8 );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 152 ]), &(acadoWorkspace.QE[ 176 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 188 ]), &(acadoWorkspace.QE[ 212 ]) );

acado_zeroBlockH11( 2, 9 );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 188 ]), &(acadoWorkspace.QE[ 216 ]) );

acado_setBlockH11_R1( 3, 3 );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QE[ 36 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 52 ]), &(acadoWorkspace.QE[ 52 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 72 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 124 ]), &(acadoWorkspace.QE[ 124 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 192 ]) );

acado_zeroBlockH11( 3, 4 );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 52 ]), &(acadoWorkspace.QE[ 56 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 76 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 100 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 124 ]), &(acadoWorkspace.QE[ 128 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.QE[ 160 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 196 ]) );

acado_zeroBlockH11( 3, 5 );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 80 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 104 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 124 ]), &(acadoWorkspace.QE[ 132 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.QE[ 164 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 200 ]) );

acado_zeroBlockH11( 3, 6 );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 108 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 124 ]), &(acadoWorkspace.QE[ 136 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.QE[ 168 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 204 ]) );

acado_zeroBlockH11( 3, 7 );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 124 ]), &(acadoWorkspace.QE[ 140 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.QE[ 172 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 208 ]) );

acado_zeroBlockH11( 3, 8 );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.QE[ 176 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 212 ]) );

acado_zeroBlockH11( 3, 9 );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 216 ]) );

acado_setBlockH11_R1( 4, 4 );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 56 ]), &(acadoWorkspace.QE[ 56 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 76 ]), &(acadoWorkspace.QE[ 76 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 100 ]), &(acadoWorkspace.QE[ 100 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 128 ]), &(acadoWorkspace.QE[ 128 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.QE[ 160 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 196 ]), &(acadoWorkspace.QE[ 196 ]) );

acado_zeroBlockH11( 4, 5 );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 76 ]), &(acadoWorkspace.QE[ 80 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 100 ]), &(acadoWorkspace.QE[ 104 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 128 ]), &(acadoWorkspace.QE[ 132 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.QE[ 164 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 196 ]), &(acadoWorkspace.QE[ 200 ]) );

acado_zeroBlockH11( 4, 6 );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 100 ]), &(acadoWorkspace.QE[ 108 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 128 ]), &(acadoWorkspace.QE[ 136 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.QE[ 168 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 196 ]), &(acadoWorkspace.QE[ 204 ]) );

acado_zeroBlockH11( 4, 7 );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 128 ]), &(acadoWorkspace.QE[ 140 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.QE[ 172 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 196 ]), &(acadoWorkspace.QE[ 208 ]) );

acado_zeroBlockH11( 4, 8 );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.QE[ 176 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 196 ]), &(acadoWorkspace.QE[ 212 ]) );

acado_zeroBlockH11( 4, 9 );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 196 ]), &(acadoWorkspace.QE[ 216 ]) );

acado_setBlockH11_R1( 5, 5 );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.QE[ 80 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 104 ]), &(acadoWorkspace.QE[ 104 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QE[ 132 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 164 ]), &(acadoWorkspace.QE[ 164 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 200 ]), &(acadoWorkspace.QE[ 200 ]) );

acado_zeroBlockH11( 5, 6 );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 104 ]), &(acadoWorkspace.QE[ 108 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QE[ 136 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 164 ]), &(acadoWorkspace.QE[ 168 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 200 ]), &(acadoWorkspace.QE[ 204 ]) );

acado_zeroBlockH11( 5, 7 );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QE[ 140 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 164 ]), &(acadoWorkspace.QE[ 172 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 200 ]), &(acadoWorkspace.QE[ 208 ]) );

acado_zeroBlockH11( 5, 8 );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 164 ]), &(acadoWorkspace.QE[ 176 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 200 ]), &(acadoWorkspace.QE[ 212 ]) );

acado_zeroBlockH11( 5, 9 );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 200 ]), &(acadoWorkspace.QE[ 216 ]) );

acado_setBlockH11_R1( 6, 6 );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QE[ 108 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 136 ]), &(acadoWorkspace.QE[ 136 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 168 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QE[ 204 ]) );

acado_zeroBlockH11( 6, 7 );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 136 ]), &(acadoWorkspace.QE[ 140 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 172 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QE[ 208 ]) );

acado_zeroBlockH11( 6, 8 );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 176 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QE[ 212 ]) );

acado_zeroBlockH11( 6, 9 );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QE[ 216 ]) );

acado_setBlockH11_R1( 7, 7 );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 140 ]), &(acadoWorkspace.QE[ 140 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 172 ]), &(acadoWorkspace.QE[ 172 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 208 ]), &(acadoWorkspace.QE[ 208 ]) );

acado_zeroBlockH11( 7, 8 );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 172 ]), &(acadoWorkspace.QE[ 176 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 208 ]), &(acadoWorkspace.QE[ 212 ]) );

acado_zeroBlockH11( 7, 9 );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 208 ]), &(acadoWorkspace.QE[ 216 ]) );

acado_setBlockH11_R1( 8, 8 );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 176 ]), &(acadoWorkspace.QE[ 176 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 212 ]), &(acadoWorkspace.QE[ 212 ]) );

acado_zeroBlockH11( 8, 9 );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 212 ]), &(acadoWorkspace.QE[ 216 ]) );

acado_setBlockH11_R1( 9, 9 );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 216 ]) );


acado_copyHTH( 1, 0 );
acado_copyHTH( 2, 0 );
acado_copyHTH( 2, 1 );
acado_copyHTH( 3, 0 );
acado_copyHTH( 3, 1 );
acado_copyHTH( 3, 2 );
acado_copyHTH( 4, 0 );
acado_copyHTH( 4, 1 );
acado_copyHTH( 4, 2 );
acado_copyHTH( 4, 3 );
acado_copyHTH( 5, 0 );
acado_copyHTH( 5, 1 );
acado_copyHTH( 5, 2 );
acado_copyHTH( 5, 3 );
acado_copyHTH( 5, 4 );
acado_copyHTH( 6, 0 );
acado_copyHTH( 6, 1 );
acado_copyHTH( 6, 2 );
acado_copyHTH( 6, 3 );
acado_copyHTH( 6, 4 );
acado_copyHTH( 6, 5 );
acado_copyHTH( 7, 0 );
acado_copyHTH( 7, 1 );
acado_copyHTH( 7, 2 );
acado_copyHTH( 7, 3 );
acado_copyHTH( 7, 4 );
acado_copyHTH( 7, 5 );
acado_copyHTH( 7, 6 );
acado_copyHTH( 8, 0 );
acado_copyHTH( 8, 1 );
acado_copyHTH( 8, 2 );
acado_copyHTH( 8, 3 );
acado_copyHTH( 8, 4 );
acado_copyHTH( 8, 5 );
acado_copyHTH( 8, 6 );
acado_copyHTH( 8, 7 );
acado_copyHTH( 9, 0 );
acado_copyHTH( 9, 1 );
acado_copyHTH( 9, 2 );
acado_copyHTH( 9, 3 );
acado_copyHTH( 9, 4 );
acado_copyHTH( 9, 5 );
acado_copyHTH( 9, 6 );
acado_copyHTH( 9, 7 );
acado_copyHTH( 9, 8 );

acado_multQ1d( acadoWorkspace.d, acadoWorkspace.Qd );
acado_multQ1d( &(acadoWorkspace.d[ 2 ]), &(acadoWorkspace.Qd[ 2 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 4 ]), &(acadoWorkspace.Qd[ 4 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 6 ]), &(acadoWorkspace.Qd[ 6 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 8 ]), &(acadoWorkspace.Qd[ 8 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 10 ]), &(acadoWorkspace.Qd[ 10 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 12 ]), &(acadoWorkspace.Qd[ 12 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 14 ]), &(acadoWorkspace.Qd[ 14 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 16 ]), &(acadoWorkspace.Qd[ 16 ]) );
acado_multQN1d( &(acadoWorkspace.d[ 18 ]), &(acadoWorkspace.Qd[ 18 ]) );

acado_macETSlu( acadoWorkspace.QE, acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 4 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 12 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 24 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 40 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 60 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 84 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 112 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 144 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 180 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 8 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 16 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 28 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 44 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 64 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 88 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 116 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 148 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 184 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 20 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 32 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 48 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 68 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 92 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 120 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 152 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 188 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 36 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 52 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 72 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 96 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 124 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 156 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 192 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 56 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 76 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 100 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 128 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 160 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 196 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 80 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 104 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 132 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 164 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 200 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 108 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 136 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 168 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 204 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 140 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 172 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 208 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 176 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 212 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 216 ]), &(acadoWorkspace.g[ 18 ]) );
acadoWorkspace.lb[0] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[0];
acadoWorkspace.lb[1] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[1];
acadoWorkspace.lb[2] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[2];
acadoWorkspace.lb[3] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[3];
acadoWorkspace.lb[4] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[4];
acadoWorkspace.lb[5] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[5];
acadoWorkspace.lb[6] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[6];
acadoWorkspace.lb[7] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[7];
acadoWorkspace.lb[8] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[8];
acadoWorkspace.lb[9] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[9];
acadoWorkspace.lb[10] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[10];
acadoWorkspace.lb[11] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[11];
acadoWorkspace.lb[12] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[12];
acadoWorkspace.lb[13] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[13];
acadoWorkspace.lb[14] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[14];
acadoWorkspace.lb[15] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[15];
acadoWorkspace.lb[16] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[16];
acadoWorkspace.lb[17] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[17];
acadoWorkspace.lb[18] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[18];
acadoWorkspace.lb[19] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[19];
acadoWorkspace.ub[0] = (real_t)1.0000000000000000e+00 - acadoVariables.u[0];
acadoWorkspace.ub[1] = (real_t)1.0000000000000000e+00 - acadoVariables.u[1];
acadoWorkspace.ub[2] = (real_t)1.0000000000000000e+00 - acadoVariables.u[2];
acadoWorkspace.ub[3] = (real_t)1.0000000000000000e+00 - acadoVariables.u[3];
acadoWorkspace.ub[4] = (real_t)1.0000000000000000e+00 - acadoVariables.u[4];
acadoWorkspace.ub[5] = (real_t)1.0000000000000000e+00 - acadoVariables.u[5];
acadoWorkspace.ub[6] = (real_t)1.0000000000000000e+00 - acadoVariables.u[6];
acadoWorkspace.ub[7] = (real_t)1.0000000000000000e+00 - acadoVariables.u[7];
acadoWorkspace.ub[8] = (real_t)1.0000000000000000e+00 - acadoVariables.u[8];
acadoWorkspace.ub[9] = (real_t)1.0000000000000000e+00 - acadoVariables.u[9];
acadoWorkspace.ub[10] = (real_t)1.0000000000000000e+00 - acadoVariables.u[10];
acadoWorkspace.ub[11] = (real_t)1.0000000000000000e+00 - acadoVariables.u[11];
acadoWorkspace.ub[12] = (real_t)1.0000000000000000e+00 - acadoVariables.u[12];
acadoWorkspace.ub[13] = (real_t)1.0000000000000000e+00 - acadoVariables.u[13];
acadoWorkspace.ub[14] = (real_t)1.0000000000000000e+00 - acadoVariables.u[14];
acadoWorkspace.ub[15] = (real_t)1.0000000000000000e+00 - acadoVariables.u[15];
acadoWorkspace.ub[16] = (real_t)1.0000000000000000e+00 - acadoVariables.u[16];
acadoWorkspace.ub[17] = (real_t)1.0000000000000000e+00 - acadoVariables.u[17];
acadoWorkspace.ub[18] = (real_t)1.0000000000000000e+00 - acadoVariables.u[18];
acadoWorkspace.ub[19] = (real_t)1.0000000000000000e+00 - acadoVariables.u[19];

for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
lRun3 = xBoundIndices[ lRun1 ] - 2;
lRun4 = ((lRun3) / (2)) + (1);
for (lRun2 = 0; lRun2 < lRun4; ++lRun2)
{
lRun5 = (((((lRun4) * (lRun4-1)) / (2)) + (lRun2)) * (2)) + ((lRun3) % (2));
acadoWorkspace.A[(lRun1 * 20) + (lRun2 * 2)] = acadoWorkspace.E[lRun5 * 2];
acadoWorkspace.A[(lRun1 * 20) + (lRun2 * 2 + 1)] = acadoWorkspace.E[lRun5 * 2 + 1];
}
}

for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
acadoWorkspace.conValueIn[0] = acadoVariables.x[lRun1 * 2];
acadoWorkspace.conValueIn[1] = acadoVariables.x[lRun1 * 2 + 1];
acadoWorkspace.conValueIn[2] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.conValueIn[3] = acadoVariables.u[lRun1 * 2 + 1];
acado_evaluatePathConstraints( acadoWorkspace.conValueIn, acadoWorkspace.conValueOut );
acadoWorkspace.evH[lRun1 * 10] = acadoWorkspace.conValueOut[0];
acadoWorkspace.evH[lRun1 * 10 + 1] = acadoWorkspace.conValueOut[1];
acadoWorkspace.evH[lRun1 * 10 + 2] = acadoWorkspace.conValueOut[2];
acadoWorkspace.evH[lRun1 * 10 + 3] = acadoWorkspace.conValueOut[3];
acadoWorkspace.evH[lRun1 * 10 + 4] = acadoWorkspace.conValueOut[4];
acadoWorkspace.evH[lRun1 * 10 + 5] = acadoWorkspace.conValueOut[5];
acadoWorkspace.evH[lRun1 * 10 + 6] = acadoWorkspace.conValueOut[6];
acadoWorkspace.evH[lRun1 * 10 + 7] = acadoWorkspace.conValueOut[7];
acadoWorkspace.evH[lRun1 * 10 + 8] = acadoWorkspace.conValueOut[8];
acadoWorkspace.evH[lRun1 * 10 + 9] = acadoWorkspace.conValueOut[9];

acadoWorkspace.evHx[lRun1 * 20] = acadoWorkspace.conValueOut[10];
acadoWorkspace.evHx[lRun1 * 20 + 1] = acadoWorkspace.conValueOut[11];
acadoWorkspace.evHx[lRun1 * 20 + 2] = acadoWorkspace.conValueOut[12];
acadoWorkspace.evHx[lRun1 * 20 + 3] = acadoWorkspace.conValueOut[13];
acadoWorkspace.evHx[lRun1 * 20 + 4] = acadoWorkspace.conValueOut[14];
acadoWorkspace.evHx[lRun1 * 20 + 5] = acadoWorkspace.conValueOut[15];
acadoWorkspace.evHx[lRun1 * 20 + 6] = acadoWorkspace.conValueOut[16];
acadoWorkspace.evHx[lRun1 * 20 + 7] = acadoWorkspace.conValueOut[17];
acadoWorkspace.evHx[lRun1 * 20 + 8] = acadoWorkspace.conValueOut[18];
acadoWorkspace.evHx[lRun1 * 20 + 9] = acadoWorkspace.conValueOut[19];
acadoWorkspace.evHx[lRun1 * 20 + 10] = acadoWorkspace.conValueOut[20];
acadoWorkspace.evHx[lRun1 * 20 + 11] = acadoWorkspace.conValueOut[21];
acadoWorkspace.evHx[lRun1 * 20 + 12] = acadoWorkspace.conValueOut[22];
acadoWorkspace.evHx[lRun1 * 20 + 13] = acadoWorkspace.conValueOut[23];
acadoWorkspace.evHx[lRun1 * 20 + 14] = acadoWorkspace.conValueOut[24];
acadoWorkspace.evHx[lRun1 * 20 + 15] = acadoWorkspace.conValueOut[25];
acadoWorkspace.evHx[lRun1 * 20 + 16] = acadoWorkspace.conValueOut[26];
acadoWorkspace.evHx[lRun1 * 20 + 17] = acadoWorkspace.conValueOut[27];
acadoWorkspace.evHx[lRun1 * 20 + 18] = acadoWorkspace.conValueOut[28];
acadoWorkspace.evHx[lRun1 * 20 + 19] = acadoWorkspace.conValueOut[29];
acadoWorkspace.evHu[lRun1 * 20] = acadoWorkspace.conValueOut[30];
acadoWorkspace.evHu[lRun1 * 20 + 1] = acadoWorkspace.conValueOut[31];
acadoWorkspace.evHu[lRun1 * 20 + 2] = acadoWorkspace.conValueOut[32];
acadoWorkspace.evHu[lRun1 * 20 + 3] = acadoWorkspace.conValueOut[33];
acadoWorkspace.evHu[lRun1 * 20 + 4] = acadoWorkspace.conValueOut[34];
acadoWorkspace.evHu[lRun1 * 20 + 5] = acadoWorkspace.conValueOut[35];
acadoWorkspace.evHu[lRun1 * 20 + 6] = acadoWorkspace.conValueOut[36];
acadoWorkspace.evHu[lRun1 * 20 + 7] = acadoWorkspace.conValueOut[37];
acadoWorkspace.evHu[lRun1 * 20 + 8] = acadoWorkspace.conValueOut[38];
acadoWorkspace.evHu[lRun1 * 20 + 9] = acadoWorkspace.conValueOut[39];
acadoWorkspace.evHu[lRun1 * 20 + 10] = acadoWorkspace.conValueOut[40];
acadoWorkspace.evHu[lRun1 * 20 + 11] = acadoWorkspace.conValueOut[41];
acadoWorkspace.evHu[lRun1 * 20 + 12] = acadoWorkspace.conValueOut[42];
acadoWorkspace.evHu[lRun1 * 20 + 13] = acadoWorkspace.conValueOut[43];
acadoWorkspace.evHu[lRun1 * 20 + 14] = acadoWorkspace.conValueOut[44];
acadoWorkspace.evHu[lRun1 * 20 + 15] = acadoWorkspace.conValueOut[45];
acadoWorkspace.evHu[lRun1 * 20 + 16] = acadoWorkspace.conValueOut[46];
acadoWorkspace.evHu[lRun1 * 20 + 17] = acadoWorkspace.conValueOut[47];
acadoWorkspace.evHu[lRun1 * 20 + 18] = acadoWorkspace.conValueOut[48];
acadoWorkspace.evHu[lRun1 * 20 + 19] = acadoWorkspace.conValueOut[49];
}

acadoWorkspace.A01[0] = acadoWorkspace.evHx[0];
acadoWorkspace.A01[1] = acadoWorkspace.evHx[1];
acadoWorkspace.A01[2] = acadoWorkspace.evHx[2];
acadoWorkspace.A01[3] = acadoWorkspace.evHx[3];
acadoWorkspace.A01[4] = acadoWorkspace.evHx[4];
acadoWorkspace.A01[5] = acadoWorkspace.evHx[5];
acadoWorkspace.A01[6] = acadoWorkspace.evHx[6];
acadoWorkspace.A01[7] = acadoWorkspace.evHx[7];
acadoWorkspace.A01[8] = acadoWorkspace.evHx[8];
acadoWorkspace.A01[9] = acadoWorkspace.evHx[9];
acadoWorkspace.A01[10] = acadoWorkspace.evHx[10];
acadoWorkspace.A01[11] = acadoWorkspace.evHx[11];
acadoWorkspace.A01[12] = acadoWorkspace.evHx[12];
acadoWorkspace.A01[13] = acadoWorkspace.evHx[13];
acadoWorkspace.A01[14] = acadoWorkspace.evHx[14];
acadoWorkspace.A01[15] = acadoWorkspace.evHx[15];
acadoWorkspace.A01[16] = acadoWorkspace.evHx[16];
acadoWorkspace.A01[17] = acadoWorkspace.evHx[17];
acadoWorkspace.A01[18] = acadoWorkspace.evHx[18];
acadoWorkspace.A01[19] = acadoWorkspace.evHx[19];

acado_multHxC( &(acadoWorkspace.evHx[ 20 ]), acadoWorkspace.evGx, &(acadoWorkspace.A01[ 20 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 40 ]), &(acadoWorkspace.evGx[ 4 ]), &(acadoWorkspace.A01[ 40 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.evGx[ 8 ]), &(acadoWorkspace.A01[ 60 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 80 ]), &(acadoWorkspace.evGx[ 12 ]), &(acadoWorkspace.A01[ 80 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 100 ]), &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.A01[ 100 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.evGx[ 20 ]), &(acadoWorkspace.A01[ 120 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.evGx[ 24 ]), &(acadoWorkspace.A01[ 140 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.evGx[ 28 ]), &(acadoWorkspace.A01[ 160 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.A01[ 180 ]) );

acado_multHxE( &(acadoWorkspace.evHx[ 20 ]), acadoWorkspace.E, 1, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 40 ]), &(acadoWorkspace.E[ 4 ]), 2, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 40 ]), &(acadoWorkspace.E[ 8 ]), 2, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.E[ 12 ]), 3, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.E[ 16 ]), 3, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.E[ 20 ]), 3, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 80 ]), &(acadoWorkspace.E[ 24 ]), 4, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 80 ]), &(acadoWorkspace.E[ 28 ]), 4, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 80 ]), &(acadoWorkspace.E[ 32 ]), 4, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 80 ]), &(acadoWorkspace.E[ 36 ]), 4, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 100 ]), &(acadoWorkspace.E[ 40 ]), 5, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 100 ]), &(acadoWorkspace.E[ 44 ]), 5, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 100 ]), &(acadoWorkspace.E[ 48 ]), 5, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 100 ]), &(acadoWorkspace.E[ 52 ]), 5, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 100 ]), &(acadoWorkspace.E[ 56 ]), 5, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 60 ]), 6, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 64 ]), 6, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 68 ]), 6, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 72 ]), 6, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 76 ]), 6, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 80 ]), 6, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.E[ 84 ]), 7, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.E[ 88 ]), 7, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.E[ 92 ]), 7, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.E[ 96 ]), 7, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.E[ 100 ]), 7, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.E[ 104 ]), 7, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.E[ 108 ]), 7, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.E[ 112 ]), 8, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.E[ 116 ]), 8, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.E[ 120 ]), 8, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.E[ 124 ]), 8, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.E[ 128 ]), 8, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.E[ 132 ]), 8, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.E[ 136 ]), 8, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.E[ 140 ]), 8, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 144 ]), 9, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 148 ]), 9, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 152 ]), 9, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 156 ]), 9, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 160 ]), 9, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 164 ]), 9, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 168 ]), 9, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 172 ]), 9, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 176 ]), 9, 8 );

acadoWorkspace.A[400] = acadoWorkspace.evHu[0];
acadoWorkspace.A[401] = acadoWorkspace.evHu[1];
acadoWorkspace.A[420] = acadoWorkspace.evHu[2];
acadoWorkspace.A[421] = acadoWorkspace.evHu[3];
acadoWorkspace.A[440] = acadoWorkspace.evHu[4];
acadoWorkspace.A[441] = acadoWorkspace.evHu[5];
acadoWorkspace.A[460] = acadoWorkspace.evHu[6];
acadoWorkspace.A[461] = acadoWorkspace.evHu[7];
acadoWorkspace.A[480] = acadoWorkspace.evHu[8];
acadoWorkspace.A[481] = acadoWorkspace.evHu[9];
acadoWorkspace.A[500] = acadoWorkspace.evHu[10];
acadoWorkspace.A[501] = acadoWorkspace.evHu[11];
acadoWorkspace.A[520] = acadoWorkspace.evHu[12];
acadoWorkspace.A[521] = acadoWorkspace.evHu[13];
acadoWorkspace.A[540] = acadoWorkspace.evHu[14];
acadoWorkspace.A[541] = acadoWorkspace.evHu[15];
acadoWorkspace.A[560] = acadoWorkspace.evHu[16];
acadoWorkspace.A[561] = acadoWorkspace.evHu[17];
acadoWorkspace.A[580] = acadoWorkspace.evHu[18];
acadoWorkspace.A[581] = acadoWorkspace.evHu[19];
acadoWorkspace.A[602] = acadoWorkspace.evHu[20];
acadoWorkspace.A[603] = acadoWorkspace.evHu[21];
acadoWorkspace.A[622] = acadoWorkspace.evHu[22];
acadoWorkspace.A[623] = acadoWorkspace.evHu[23];
acadoWorkspace.A[642] = acadoWorkspace.evHu[24];
acadoWorkspace.A[643] = acadoWorkspace.evHu[25];
acadoWorkspace.A[662] = acadoWorkspace.evHu[26];
acadoWorkspace.A[663] = acadoWorkspace.evHu[27];
acadoWorkspace.A[682] = acadoWorkspace.evHu[28];
acadoWorkspace.A[683] = acadoWorkspace.evHu[29];
acadoWorkspace.A[702] = acadoWorkspace.evHu[30];
acadoWorkspace.A[703] = acadoWorkspace.evHu[31];
acadoWorkspace.A[722] = acadoWorkspace.evHu[32];
acadoWorkspace.A[723] = acadoWorkspace.evHu[33];
acadoWorkspace.A[742] = acadoWorkspace.evHu[34];
acadoWorkspace.A[743] = acadoWorkspace.evHu[35];
acadoWorkspace.A[762] = acadoWorkspace.evHu[36];
acadoWorkspace.A[763] = acadoWorkspace.evHu[37];
acadoWorkspace.A[782] = acadoWorkspace.evHu[38];
acadoWorkspace.A[783] = acadoWorkspace.evHu[39];
acadoWorkspace.A[804] = acadoWorkspace.evHu[40];
acadoWorkspace.A[805] = acadoWorkspace.evHu[41];
acadoWorkspace.A[824] = acadoWorkspace.evHu[42];
acadoWorkspace.A[825] = acadoWorkspace.evHu[43];
acadoWorkspace.A[844] = acadoWorkspace.evHu[44];
acadoWorkspace.A[845] = acadoWorkspace.evHu[45];
acadoWorkspace.A[864] = acadoWorkspace.evHu[46];
acadoWorkspace.A[865] = acadoWorkspace.evHu[47];
acadoWorkspace.A[884] = acadoWorkspace.evHu[48];
acadoWorkspace.A[885] = acadoWorkspace.evHu[49];
acadoWorkspace.A[904] = acadoWorkspace.evHu[50];
acadoWorkspace.A[905] = acadoWorkspace.evHu[51];
acadoWorkspace.A[924] = acadoWorkspace.evHu[52];
acadoWorkspace.A[925] = acadoWorkspace.evHu[53];
acadoWorkspace.A[944] = acadoWorkspace.evHu[54];
acadoWorkspace.A[945] = acadoWorkspace.evHu[55];
acadoWorkspace.A[964] = acadoWorkspace.evHu[56];
acadoWorkspace.A[965] = acadoWorkspace.evHu[57];
acadoWorkspace.A[984] = acadoWorkspace.evHu[58];
acadoWorkspace.A[985] = acadoWorkspace.evHu[59];
acadoWorkspace.A[1006] = acadoWorkspace.evHu[60];
acadoWorkspace.A[1007] = acadoWorkspace.evHu[61];
acadoWorkspace.A[1026] = acadoWorkspace.evHu[62];
acadoWorkspace.A[1027] = acadoWorkspace.evHu[63];
acadoWorkspace.A[1046] = acadoWorkspace.evHu[64];
acadoWorkspace.A[1047] = acadoWorkspace.evHu[65];
acadoWorkspace.A[1066] = acadoWorkspace.evHu[66];
acadoWorkspace.A[1067] = acadoWorkspace.evHu[67];
acadoWorkspace.A[1086] = acadoWorkspace.evHu[68];
acadoWorkspace.A[1087] = acadoWorkspace.evHu[69];
acadoWorkspace.A[1106] = acadoWorkspace.evHu[70];
acadoWorkspace.A[1107] = acadoWorkspace.evHu[71];
acadoWorkspace.A[1126] = acadoWorkspace.evHu[72];
acadoWorkspace.A[1127] = acadoWorkspace.evHu[73];
acadoWorkspace.A[1146] = acadoWorkspace.evHu[74];
acadoWorkspace.A[1147] = acadoWorkspace.evHu[75];
acadoWorkspace.A[1166] = acadoWorkspace.evHu[76];
acadoWorkspace.A[1167] = acadoWorkspace.evHu[77];
acadoWorkspace.A[1186] = acadoWorkspace.evHu[78];
acadoWorkspace.A[1187] = acadoWorkspace.evHu[79];
acadoWorkspace.A[1208] = acadoWorkspace.evHu[80];
acadoWorkspace.A[1209] = acadoWorkspace.evHu[81];
acadoWorkspace.A[1228] = acadoWorkspace.evHu[82];
acadoWorkspace.A[1229] = acadoWorkspace.evHu[83];
acadoWorkspace.A[1248] = acadoWorkspace.evHu[84];
acadoWorkspace.A[1249] = acadoWorkspace.evHu[85];
acadoWorkspace.A[1268] = acadoWorkspace.evHu[86];
acadoWorkspace.A[1269] = acadoWorkspace.evHu[87];
acadoWorkspace.A[1288] = acadoWorkspace.evHu[88];
acadoWorkspace.A[1289] = acadoWorkspace.evHu[89];
acadoWorkspace.A[1308] = acadoWorkspace.evHu[90];
acadoWorkspace.A[1309] = acadoWorkspace.evHu[91];
acadoWorkspace.A[1328] = acadoWorkspace.evHu[92];
acadoWorkspace.A[1329] = acadoWorkspace.evHu[93];
acadoWorkspace.A[1348] = acadoWorkspace.evHu[94];
acadoWorkspace.A[1349] = acadoWorkspace.evHu[95];
acadoWorkspace.A[1368] = acadoWorkspace.evHu[96];
acadoWorkspace.A[1369] = acadoWorkspace.evHu[97];
acadoWorkspace.A[1388] = acadoWorkspace.evHu[98];
acadoWorkspace.A[1389] = acadoWorkspace.evHu[99];
acadoWorkspace.A[1410] = acadoWorkspace.evHu[100];
acadoWorkspace.A[1411] = acadoWorkspace.evHu[101];
acadoWorkspace.A[1430] = acadoWorkspace.evHu[102];
acadoWorkspace.A[1431] = acadoWorkspace.evHu[103];
acadoWorkspace.A[1450] = acadoWorkspace.evHu[104];
acadoWorkspace.A[1451] = acadoWorkspace.evHu[105];
acadoWorkspace.A[1470] = acadoWorkspace.evHu[106];
acadoWorkspace.A[1471] = acadoWorkspace.evHu[107];
acadoWorkspace.A[1490] = acadoWorkspace.evHu[108];
acadoWorkspace.A[1491] = acadoWorkspace.evHu[109];
acadoWorkspace.A[1510] = acadoWorkspace.evHu[110];
acadoWorkspace.A[1511] = acadoWorkspace.evHu[111];
acadoWorkspace.A[1530] = acadoWorkspace.evHu[112];
acadoWorkspace.A[1531] = acadoWorkspace.evHu[113];
acadoWorkspace.A[1550] = acadoWorkspace.evHu[114];
acadoWorkspace.A[1551] = acadoWorkspace.evHu[115];
acadoWorkspace.A[1570] = acadoWorkspace.evHu[116];
acadoWorkspace.A[1571] = acadoWorkspace.evHu[117];
acadoWorkspace.A[1590] = acadoWorkspace.evHu[118];
acadoWorkspace.A[1591] = acadoWorkspace.evHu[119];
acadoWorkspace.A[1612] = acadoWorkspace.evHu[120];
acadoWorkspace.A[1613] = acadoWorkspace.evHu[121];
acadoWorkspace.A[1632] = acadoWorkspace.evHu[122];
acadoWorkspace.A[1633] = acadoWorkspace.evHu[123];
acadoWorkspace.A[1652] = acadoWorkspace.evHu[124];
acadoWorkspace.A[1653] = acadoWorkspace.evHu[125];
acadoWorkspace.A[1672] = acadoWorkspace.evHu[126];
acadoWorkspace.A[1673] = acadoWorkspace.evHu[127];
acadoWorkspace.A[1692] = acadoWorkspace.evHu[128];
acadoWorkspace.A[1693] = acadoWorkspace.evHu[129];
acadoWorkspace.A[1712] = acadoWorkspace.evHu[130];
acadoWorkspace.A[1713] = acadoWorkspace.evHu[131];
acadoWorkspace.A[1732] = acadoWorkspace.evHu[132];
acadoWorkspace.A[1733] = acadoWorkspace.evHu[133];
acadoWorkspace.A[1752] = acadoWorkspace.evHu[134];
acadoWorkspace.A[1753] = acadoWorkspace.evHu[135];
acadoWorkspace.A[1772] = acadoWorkspace.evHu[136];
acadoWorkspace.A[1773] = acadoWorkspace.evHu[137];
acadoWorkspace.A[1792] = acadoWorkspace.evHu[138];
acadoWorkspace.A[1793] = acadoWorkspace.evHu[139];
acadoWorkspace.A[1814] = acadoWorkspace.evHu[140];
acadoWorkspace.A[1815] = acadoWorkspace.evHu[141];
acadoWorkspace.A[1834] = acadoWorkspace.evHu[142];
acadoWorkspace.A[1835] = acadoWorkspace.evHu[143];
acadoWorkspace.A[1854] = acadoWorkspace.evHu[144];
acadoWorkspace.A[1855] = acadoWorkspace.evHu[145];
acadoWorkspace.A[1874] = acadoWorkspace.evHu[146];
acadoWorkspace.A[1875] = acadoWorkspace.evHu[147];
acadoWorkspace.A[1894] = acadoWorkspace.evHu[148];
acadoWorkspace.A[1895] = acadoWorkspace.evHu[149];
acadoWorkspace.A[1914] = acadoWorkspace.evHu[150];
acadoWorkspace.A[1915] = acadoWorkspace.evHu[151];
acadoWorkspace.A[1934] = acadoWorkspace.evHu[152];
acadoWorkspace.A[1935] = acadoWorkspace.evHu[153];
acadoWorkspace.A[1954] = acadoWorkspace.evHu[154];
acadoWorkspace.A[1955] = acadoWorkspace.evHu[155];
acadoWorkspace.A[1974] = acadoWorkspace.evHu[156];
acadoWorkspace.A[1975] = acadoWorkspace.evHu[157];
acadoWorkspace.A[1994] = acadoWorkspace.evHu[158];
acadoWorkspace.A[1995] = acadoWorkspace.evHu[159];
acadoWorkspace.A[2016] = acadoWorkspace.evHu[160];
acadoWorkspace.A[2017] = acadoWorkspace.evHu[161];
acadoWorkspace.A[2036] = acadoWorkspace.evHu[162];
acadoWorkspace.A[2037] = acadoWorkspace.evHu[163];
acadoWorkspace.A[2056] = acadoWorkspace.evHu[164];
acadoWorkspace.A[2057] = acadoWorkspace.evHu[165];
acadoWorkspace.A[2076] = acadoWorkspace.evHu[166];
acadoWorkspace.A[2077] = acadoWorkspace.evHu[167];
acadoWorkspace.A[2096] = acadoWorkspace.evHu[168];
acadoWorkspace.A[2097] = acadoWorkspace.evHu[169];
acadoWorkspace.A[2116] = acadoWorkspace.evHu[170];
acadoWorkspace.A[2117] = acadoWorkspace.evHu[171];
acadoWorkspace.A[2136] = acadoWorkspace.evHu[172];
acadoWorkspace.A[2137] = acadoWorkspace.evHu[173];
acadoWorkspace.A[2156] = acadoWorkspace.evHu[174];
acadoWorkspace.A[2157] = acadoWorkspace.evHu[175];
acadoWorkspace.A[2176] = acadoWorkspace.evHu[176];
acadoWorkspace.A[2177] = acadoWorkspace.evHu[177];
acadoWorkspace.A[2196] = acadoWorkspace.evHu[178];
acadoWorkspace.A[2197] = acadoWorkspace.evHu[179];
acadoWorkspace.A[2218] = acadoWorkspace.evHu[180];
acadoWorkspace.A[2219] = acadoWorkspace.evHu[181];
acadoWorkspace.A[2238] = acadoWorkspace.evHu[182];
acadoWorkspace.A[2239] = acadoWorkspace.evHu[183];
acadoWorkspace.A[2258] = acadoWorkspace.evHu[184];
acadoWorkspace.A[2259] = acadoWorkspace.evHu[185];
acadoWorkspace.A[2278] = acadoWorkspace.evHu[186];
acadoWorkspace.A[2279] = acadoWorkspace.evHu[187];
acadoWorkspace.A[2298] = acadoWorkspace.evHu[188];
acadoWorkspace.A[2299] = acadoWorkspace.evHu[189];
acadoWorkspace.A[2318] = acadoWorkspace.evHu[190];
acadoWorkspace.A[2319] = acadoWorkspace.evHu[191];
acadoWorkspace.A[2338] = acadoWorkspace.evHu[192];
acadoWorkspace.A[2339] = acadoWorkspace.evHu[193];
acadoWorkspace.A[2358] = acadoWorkspace.evHu[194];
acadoWorkspace.A[2359] = acadoWorkspace.evHu[195];
acadoWorkspace.A[2378] = acadoWorkspace.evHu[196];
acadoWorkspace.A[2379] = acadoWorkspace.evHu[197];
acadoWorkspace.A[2398] = acadoWorkspace.evHu[198];
acadoWorkspace.A[2399] = acadoWorkspace.evHu[199];
acadoWorkspace.lbA[20] = - acadoWorkspace.evH[0];
acadoWorkspace.lbA[21] = - acadoWorkspace.evH[1];
acadoWorkspace.lbA[22] = - acadoWorkspace.evH[2];
acadoWorkspace.lbA[23] = - acadoWorkspace.evH[3];
acadoWorkspace.lbA[24] = - acadoWorkspace.evH[4];
acadoWorkspace.lbA[25] = - acadoWorkspace.evH[5];
acadoWorkspace.lbA[26] = - acadoWorkspace.evH[6];
acadoWorkspace.lbA[27] = - acadoWorkspace.evH[7];
acadoWorkspace.lbA[28] = - acadoWorkspace.evH[8];
acadoWorkspace.lbA[29] = - acadoWorkspace.evH[9];
acadoWorkspace.lbA[30] = - acadoWorkspace.evH[10];
acadoWorkspace.lbA[31] = - acadoWorkspace.evH[11];
acadoWorkspace.lbA[32] = - acadoWorkspace.evH[12];
acadoWorkspace.lbA[33] = - acadoWorkspace.evH[13];
acadoWorkspace.lbA[34] = - acadoWorkspace.evH[14];
acadoWorkspace.lbA[35] = - acadoWorkspace.evH[15];
acadoWorkspace.lbA[36] = - acadoWorkspace.evH[16];
acadoWorkspace.lbA[37] = - acadoWorkspace.evH[17];
acadoWorkspace.lbA[38] = - acadoWorkspace.evH[18];
acadoWorkspace.lbA[39] = - acadoWorkspace.evH[19];
acadoWorkspace.lbA[40] = - acadoWorkspace.evH[20];
acadoWorkspace.lbA[41] = - acadoWorkspace.evH[21];
acadoWorkspace.lbA[42] = - acadoWorkspace.evH[22];
acadoWorkspace.lbA[43] = - acadoWorkspace.evH[23];
acadoWorkspace.lbA[44] = - acadoWorkspace.evH[24];
acadoWorkspace.lbA[45] = - acadoWorkspace.evH[25];
acadoWorkspace.lbA[46] = - acadoWorkspace.evH[26];
acadoWorkspace.lbA[47] = - acadoWorkspace.evH[27];
acadoWorkspace.lbA[48] = - acadoWorkspace.evH[28];
acadoWorkspace.lbA[49] = - acadoWorkspace.evH[29];
acadoWorkspace.lbA[50] = - acadoWorkspace.evH[30];
acadoWorkspace.lbA[51] = - acadoWorkspace.evH[31];
acadoWorkspace.lbA[52] = - acadoWorkspace.evH[32];
acadoWorkspace.lbA[53] = - acadoWorkspace.evH[33];
acadoWorkspace.lbA[54] = - acadoWorkspace.evH[34];
acadoWorkspace.lbA[55] = - acadoWorkspace.evH[35];
acadoWorkspace.lbA[56] = - acadoWorkspace.evH[36];
acadoWorkspace.lbA[57] = - acadoWorkspace.evH[37];
acadoWorkspace.lbA[58] = - acadoWorkspace.evH[38];
acadoWorkspace.lbA[59] = - acadoWorkspace.evH[39];
acadoWorkspace.lbA[60] = - acadoWorkspace.evH[40];
acadoWorkspace.lbA[61] = - acadoWorkspace.evH[41];
acadoWorkspace.lbA[62] = - acadoWorkspace.evH[42];
acadoWorkspace.lbA[63] = - acadoWorkspace.evH[43];
acadoWorkspace.lbA[64] = - acadoWorkspace.evH[44];
acadoWorkspace.lbA[65] = - acadoWorkspace.evH[45];
acadoWorkspace.lbA[66] = - acadoWorkspace.evH[46];
acadoWorkspace.lbA[67] = - acadoWorkspace.evH[47];
acadoWorkspace.lbA[68] = - acadoWorkspace.evH[48];
acadoWorkspace.lbA[69] = - acadoWorkspace.evH[49];
acadoWorkspace.lbA[70] = - acadoWorkspace.evH[50];
acadoWorkspace.lbA[71] = - acadoWorkspace.evH[51];
acadoWorkspace.lbA[72] = - acadoWorkspace.evH[52];
acadoWorkspace.lbA[73] = - acadoWorkspace.evH[53];
acadoWorkspace.lbA[74] = - acadoWorkspace.evH[54];
acadoWorkspace.lbA[75] = - acadoWorkspace.evH[55];
acadoWorkspace.lbA[76] = - acadoWorkspace.evH[56];
acadoWorkspace.lbA[77] = - acadoWorkspace.evH[57];
acadoWorkspace.lbA[78] = - acadoWorkspace.evH[58];
acadoWorkspace.lbA[79] = - acadoWorkspace.evH[59];
acadoWorkspace.lbA[80] = - acadoWorkspace.evH[60];
acadoWorkspace.lbA[81] = - acadoWorkspace.evH[61];
acadoWorkspace.lbA[82] = - acadoWorkspace.evH[62];
acadoWorkspace.lbA[83] = - acadoWorkspace.evH[63];
acadoWorkspace.lbA[84] = - acadoWorkspace.evH[64];
acadoWorkspace.lbA[85] = - acadoWorkspace.evH[65];
acadoWorkspace.lbA[86] = - acadoWorkspace.evH[66];
acadoWorkspace.lbA[87] = - acadoWorkspace.evH[67];
acadoWorkspace.lbA[88] = - acadoWorkspace.evH[68];
acadoWorkspace.lbA[89] = - acadoWorkspace.evH[69];
acadoWorkspace.lbA[90] = - acadoWorkspace.evH[70];
acadoWorkspace.lbA[91] = - acadoWorkspace.evH[71];
acadoWorkspace.lbA[92] = - acadoWorkspace.evH[72];
acadoWorkspace.lbA[93] = - acadoWorkspace.evH[73];
acadoWorkspace.lbA[94] = - acadoWorkspace.evH[74];
acadoWorkspace.lbA[95] = - acadoWorkspace.evH[75];
acadoWorkspace.lbA[96] = - acadoWorkspace.evH[76];
acadoWorkspace.lbA[97] = - acadoWorkspace.evH[77];
acadoWorkspace.lbA[98] = - acadoWorkspace.evH[78];
acadoWorkspace.lbA[99] = - acadoWorkspace.evH[79];
acadoWorkspace.lbA[100] = - acadoWorkspace.evH[80];
acadoWorkspace.lbA[101] = - acadoWorkspace.evH[81];
acadoWorkspace.lbA[102] = - acadoWorkspace.evH[82];
acadoWorkspace.lbA[103] = - acadoWorkspace.evH[83];
acadoWorkspace.lbA[104] = - acadoWorkspace.evH[84];
acadoWorkspace.lbA[105] = - acadoWorkspace.evH[85];
acadoWorkspace.lbA[106] = - acadoWorkspace.evH[86];
acadoWorkspace.lbA[107] = - acadoWorkspace.evH[87];
acadoWorkspace.lbA[108] = - acadoWorkspace.evH[88];
acadoWorkspace.lbA[109] = - acadoWorkspace.evH[89];
acadoWorkspace.lbA[110] = - acadoWorkspace.evH[90];
acadoWorkspace.lbA[111] = - acadoWorkspace.evH[91];
acadoWorkspace.lbA[112] = - acadoWorkspace.evH[92];
acadoWorkspace.lbA[113] = - acadoWorkspace.evH[93];
acadoWorkspace.lbA[114] = - acadoWorkspace.evH[94];
acadoWorkspace.lbA[115] = - acadoWorkspace.evH[95];
acadoWorkspace.lbA[116] = - acadoWorkspace.evH[96];
acadoWorkspace.lbA[117] = - acadoWorkspace.evH[97];
acadoWorkspace.lbA[118] = - acadoWorkspace.evH[98];
acadoWorkspace.lbA[119] = - acadoWorkspace.evH[99];

acadoWorkspace.ubA[20] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[0];
acadoWorkspace.ubA[21] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[1];
acadoWorkspace.ubA[22] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[2];
acadoWorkspace.ubA[23] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[3];
acadoWorkspace.ubA[24] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[4];
acadoWorkspace.ubA[25] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[5];
acadoWorkspace.ubA[26] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[6];
acadoWorkspace.ubA[27] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[7];
acadoWorkspace.ubA[28] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[8];
acadoWorkspace.ubA[29] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[9];
acadoWorkspace.ubA[30] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[10];
acadoWorkspace.ubA[31] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[11];
acadoWorkspace.ubA[32] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[12];
acadoWorkspace.ubA[33] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[13];
acadoWorkspace.ubA[34] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[14];
acadoWorkspace.ubA[35] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[15];
acadoWorkspace.ubA[36] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[16];
acadoWorkspace.ubA[37] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[17];
acadoWorkspace.ubA[38] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[18];
acadoWorkspace.ubA[39] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[19];
acadoWorkspace.ubA[40] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[20];
acadoWorkspace.ubA[41] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[21];
acadoWorkspace.ubA[42] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[22];
acadoWorkspace.ubA[43] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[23];
acadoWorkspace.ubA[44] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[24];
acadoWorkspace.ubA[45] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[25];
acadoWorkspace.ubA[46] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[26];
acadoWorkspace.ubA[47] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[27];
acadoWorkspace.ubA[48] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[28];
acadoWorkspace.ubA[49] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[29];
acadoWorkspace.ubA[50] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[30];
acadoWorkspace.ubA[51] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[31];
acadoWorkspace.ubA[52] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[32];
acadoWorkspace.ubA[53] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[33];
acadoWorkspace.ubA[54] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[34];
acadoWorkspace.ubA[55] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[35];
acadoWorkspace.ubA[56] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[36];
acadoWorkspace.ubA[57] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[37];
acadoWorkspace.ubA[58] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[38];
acadoWorkspace.ubA[59] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[39];
acadoWorkspace.ubA[60] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[40];
acadoWorkspace.ubA[61] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[41];
acadoWorkspace.ubA[62] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[42];
acadoWorkspace.ubA[63] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[43];
acadoWorkspace.ubA[64] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[44];
acadoWorkspace.ubA[65] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[45];
acadoWorkspace.ubA[66] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[46];
acadoWorkspace.ubA[67] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[47];
acadoWorkspace.ubA[68] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[48];
acadoWorkspace.ubA[69] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[49];
acadoWorkspace.ubA[70] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[50];
acadoWorkspace.ubA[71] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[51];
acadoWorkspace.ubA[72] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[52];
acadoWorkspace.ubA[73] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[53];
acadoWorkspace.ubA[74] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[54];
acadoWorkspace.ubA[75] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[55];
acadoWorkspace.ubA[76] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[56];
acadoWorkspace.ubA[77] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[57];
acadoWorkspace.ubA[78] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[58];
acadoWorkspace.ubA[79] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[59];
acadoWorkspace.ubA[80] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[60];
acadoWorkspace.ubA[81] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[61];
acadoWorkspace.ubA[82] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[62];
acadoWorkspace.ubA[83] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[63];
acadoWorkspace.ubA[84] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[64];
acadoWorkspace.ubA[85] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[65];
acadoWorkspace.ubA[86] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[66];
acadoWorkspace.ubA[87] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[67];
acadoWorkspace.ubA[88] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[68];
acadoWorkspace.ubA[89] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[69];
acadoWorkspace.ubA[90] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[70];
acadoWorkspace.ubA[91] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[71];
acadoWorkspace.ubA[92] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[72];
acadoWorkspace.ubA[93] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[73];
acadoWorkspace.ubA[94] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[74];
acadoWorkspace.ubA[95] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[75];
acadoWorkspace.ubA[96] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[76];
acadoWorkspace.ubA[97] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[77];
acadoWorkspace.ubA[98] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[78];
acadoWorkspace.ubA[99] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[79];
acadoWorkspace.ubA[100] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[80];
acadoWorkspace.ubA[101] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[81];
acadoWorkspace.ubA[102] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[82];
acadoWorkspace.ubA[103] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[83];
acadoWorkspace.ubA[104] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[84];
acadoWorkspace.ubA[105] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[85];
acadoWorkspace.ubA[106] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[86];
acadoWorkspace.ubA[107] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[87];
acadoWorkspace.ubA[108] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[88];
acadoWorkspace.ubA[109] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[89];
acadoWorkspace.ubA[110] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[90];
acadoWorkspace.ubA[111] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[91];
acadoWorkspace.ubA[112] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[92];
acadoWorkspace.ubA[113] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[93];
acadoWorkspace.ubA[114] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[94];
acadoWorkspace.ubA[115] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[95];
acadoWorkspace.ubA[116] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[96];
acadoWorkspace.ubA[117] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[97];
acadoWorkspace.ubA[118] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[98];
acadoWorkspace.ubA[119] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[99];

acado_macHxd( &(acadoWorkspace.evHx[ 20 ]), acadoWorkspace.d, &(acadoWorkspace.lbA[ 30 ]), &(acadoWorkspace.ubA[ 30 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 40 ]), &(acadoWorkspace.d[ 2 ]), &(acadoWorkspace.lbA[ 40 ]), &(acadoWorkspace.ubA[ 40 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.d[ 4 ]), &(acadoWorkspace.lbA[ 50 ]), &(acadoWorkspace.ubA[ 50 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 80 ]), &(acadoWorkspace.d[ 6 ]), &(acadoWorkspace.lbA[ 60 ]), &(acadoWorkspace.ubA[ 60 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 100 ]), &(acadoWorkspace.d[ 8 ]), &(acadoWorkspace.lbA[ 70 ]), &(acadoWorkspace.ubA[ 70 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.d[ 10 ]), &(acadoWorkspace.lbA[ 80 ]), &(acadoWorkspace.ubA[ 80 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.d[ 12 ]), &(acadoWorkspace.lbA[ 90 ]), &(acadoWorkspace.ubA[ 90 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.d[ 14 ]), &(acadoWorkspace.lbA[ 100 ]), &(acadoWorkspace.ubA[ 100 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.d[ 16 ]), &(acadoWorkspace.lbA[ 110 ]), &(acadoWorkspace.ubA[ 110 ]) );

}

void acado_condenseFdb(  )
{
real_t tmp;

acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];

acadoWorkspace.Dy[0] -= acadoVariables.y[0];
acadoWorkspace.Dy[1] -= acadoVariables.y[1];
acadoWorkspace.Dy[2] -= acadoVariables.y[2];
acadoWorkspace.Dy[3] -= acadoVariables.y[3];
acadoWorkspace.Dy[4] -= acadoVariables.y[4];
acadoWorkspace.Dy[5] -= acadoVariables.y[5];
acadoWorkspace.Dy[6] -= acadoVariables.y[6];
acadoWorkspace.Dy[7] -= acadoVariables.y[7];
acadoWorkspace.Dy[8] -= acadoVariables.y[8];
acadoWorkspace.Dy[9] -= acadoVariables.y[9];
acadoWorkspace.Dy[10] -= acadoVariables.y[10];
acadoWorkspace.Dy[11] -= acadoVariables.y[11];
acadoWorkspace.Dy[12] -= acadoVariables.y[12];
acadoWorkspace.Dy[13] -= acadoVariables.y[13];
acadoWorkspace.Dy[14] -= acadoVariables.y[14];
acadoWorkspace.Dy[15] -= acadoVariables.y[15];
acadoWorkspace.Dy[16] -= acadoVariables.y[16];
acadoWorkspace.Dy[17] -= acadoVariables.y[17];
acadoWorkspace.Dy[18] -= acadoVariables.y[18];
acadoWorkspace.Dy[19] -= acadoVariables.y[19];
acadoWorkspace.Dy[20] -= acadoVariables.y[20];
acadoWorkspace.Dy[21] -= acadoVariables.y[21];
acadoWorkspace.Dy[22] -= acadoVariables.y[22];
acadoWorkspace.Dy[23] -= acadoVariables.y[23];
acadoWorkspace.Dy[24] -= acadoVariables.y[24];
acadoWorkspace.Dy[25] -= acadoVariables.y[25];
acadoWorkspace.Dy[26] -= acadoVariables.y[26];
acadoWorkspace.Dy[27] -= acadoVariables.y[27];
acadoWorkspace.Dy[28] -= acadoVariables.y[28];
acadoWorkspace.Dy[29] -= acadoVariables.y[29];
acadoWorkspace.Dy[30] -= acadoVariables.y[30];
acadoWorkspace.Dy[31] -= acadoVariables.y[31];
acadoWorkspace.Dy[32] -= acadoVariables.y[32];
acadoWorkspace.Dy[33] -= acadoVariables.y[33];
acadoWorkspace.Dy[34] -= acadoVariables.y[34];
acadoWorkspace.Dy[35] -= acadoVariables.y[35];
acadoWorkspace.Dy[36] -= acadoVariables.y[36];
acadoWorkspace.Dy[37] -= acadoVariables.y[37];
acadoWorkspace.Dy[38] -= acadoVariables.y[38];
acadoWorkspace.Dy[39] -= acadoVariables.y[39];
acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];

acado_multRDy( acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.Dy[ 4 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 8 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 16 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 20 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 28 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 32 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 36 ]), &(acadoWorkspace.g[ 18 ]) );

acado_multQDy( acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Dy[ 4 ]), &(acadoWorkspace.QDy[ 2 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 8 ]), &(acadoWorkspace.QDy[ 4 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.QDy[ 6 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 16 ]), &(acadoWorkspace.QDy[ 8 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 20 ]), &(acadoWorkspace.QDy[ 10 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.QDy[ 12 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 28 ]), &(acadoWorkspace.QDy[ 14 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 32 ]), &(acadoWorkspace.QDy[ 16 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 36 ]), &(acadoWorkspace.QDy[ 18 ]) );

acadoWorkspace.QDy[20] = + (real_t)1.0000000000000001e-01*acadoWorkspace.DyN[0];
acadoWorkspace.QDy[21] = + (real_t)1.0000000000000001e-01*acadoWorkspace.DyN[1];

acadoWorkspace.QDy[2] += acadoWorkspace.Qd[0];
acadoWorkspace.QDy[3] += acadoWorkspace.Qd[1];
acadoWorkspace.QDy[4] += acadoWorkspace.Qd[2];
acadoWorkspace.QDy[5] += acadoWorkspace.Qd[3];
acadoWorkspace.QDy[6] += acadoWorkspace.Qd[4];
acadoWorkspace.QDy[7] += acadoWorkspace.Qd[5];
acadoWorkspace.QDy[8] += acadoWorkspace.Qd[6];
acadoWorkspace.QDy[9] += acadoWorkspace.Qd[7];
acadoWorkspace.QDy[10] += acadoWorkspace.Qd[8];
acadoWorkspace.QDy[11] += acadoWorkspace.Qd[9];
acadoWorkspace.QDy[12] += acadoWorkspace.Qd[10];
acadoWorkspace.QDy[13] += acadoWorkspace.Qd[11];
acadoWorkspace.QDy[14] += acadoWorkspace.Qd[12];
acadoWorkspace.QDy[15] += acadoWorkspace.Qd[13];
acadoWorkspace.QDy[16] += acadoWorkspace.Qd[14];
acadoWorkspace.QDy[17] += acadoWorkspace.Qd[15];
acadoWorkspace.QDy[18] += acadoWorkspace.Qd[16];
acadoWorkspace.QDy[19] += acadoWorkspace.Qd[17];
acadoWorkspace.QDy[20] += acadoWorkspace.Qd[18];
acadoWorkspace.QDy[21] += acadoWorkspace.Qd[19];

acado_multEQDy( acadoWorkspace.E, &(acadoWorkspace.QDy[ 2 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 4 ]), &(acadoWorkspace.QDy[ 4 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.QDy[ 6 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.QDy[ 8 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 40 ]), &(acadoWorkspace.QDy[ 10 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QDy[ 12 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QDy[ 14 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 112 ]), &(acadoWorkspace.QDy[ 16 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QDy[ 18 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QDy[ 20 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 8 ]), &(acadoWorkspace.QDy[ 4 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 16 ]), &(acadoWorkspace.QDy[ 6 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 28 ]), &(acadoWorkspace.QDy[ 8 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 44 ]), &(acadoWorkspace.QDy[ 10 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 64 ]), &(acadoWorkspace.QDy[ 12 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 88 ]), &(acadoWorkspace.QDy[ 14 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 116 ]), &(acadoWorkspace.QDy[ 16 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 148 ]), &(acadoWorkspace.QDy[ 18 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 184 ]), &(acadoWorkspace.QDy[ 20 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 20 ]), &(acadoWorkspace.QDy[ 6 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 32 ]), &(acadoWorkspace.QDy[ 8 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QDy[ 10 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 68 ]), &(acadoWorkspace.QDy[ 12 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 92 ]), &(acadoWorkspace.QDy[ 14 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QDy[ 16 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 152 ]), &(acadoWorkspace.QDy[ 18 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 188 ]), &(acadoWorkspace.QDy[ 20 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QDy[ 8 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 52 ]), &(acadoWorkspace.QDy[ 10 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QDy[ 12 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QDy[ 14 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 124 ]), &(acadoWorkspace.QDy[ 16 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.QDy[ 18 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QDy[ 20 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 56 ]), &(acadoWorkspace.QDy[ 10 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 76 ]), &(acadoWorkspace.QDy[ 12 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 100 ]), &(acadoWorkspace.QDy[ 14 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 128 ]), &(acadoWorkspace.QDy[ 16 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.QDy[ 18 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 196 ]), &(acadoWorkspace.QDy[ 20 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.QDy[ 12 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 104 ]), &(acadoWorkspace.QDy[ 14 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QDy[ 16 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 164 ]), &(acadoWorkspace.QDy[ 18 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 200 ]), &(acadoWorkspace.QDy[ 20 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QDy[ 14 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 136 ]), &(acadoWorkspace.QDy[ 16 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QDy[ 18 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QDy[ 20 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 140 ]), &(acadoWorkspace.QDy[ 16 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 172 ]), &(acadoWorkspace.QDy[ 18 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 208 ]), &(acadoWorkspace.QDy[ 20 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 176 ]), &(acadoWorkspace.QDy[ 18 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 212 ]), &(acadoWorkspace.QDy[ 20 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QDy[ 20 ]), &(acadoWorkspace.g[ 18 ]) );

acadoWorkspace.g[0] += + acadoWorkspace.H10[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1]*acadoWorkspace.Dx0[1];
acadoWorkspace.g[1] += + acadoWorkspace.H10[2]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[3]*acadoWorkspace.Dx0[1];
acadoWorkspace.g[2] += + acadoWorkspace.H10[4]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[5]*acadoWorkspace.Dx0[1];
acadoWorkspace.g[3] += + acadoWorkspace.H10[6]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[7]*acadoWorkspace.Dx0[1];
acadoWorkspace.g[4] += + acadoWorkspace.H10[8]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[9]*acadoWorkspace.Dx0[1];
acadoWorkspace.g[5] += + acadoWorkspace.H10[10]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[11]*acadoWorkspace.Dx0[1];
acadoWorkspace.g[6] += + acadoWorkspace.H10[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[13]*acadoWorkspace.Dx0[1];
acadoWorkspace.g[7] += + acadoWorkspace.H10[14]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[15]*acadoWorkspace.Dx0[1];
acadoWorkspace.g[8] += + acadoWorkspace.H10[16]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[17]*acadoWorkspace.Dx0[1];
acadoWorkspace.g[9] += + acadoWorkspace.H10[18]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[19]*acadoWorkspace.Dx0[1];
acadoWorkspace.g[10] += + acadoWorkspace.H10[20]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[21]*acadoWorkspace.Dx0[1];
acadoWorkspace.g[11] += + acadoWorkspace.H10[22]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[23]*acadoWorkspace.Dx0[1];
acadoWorkspace.g[12] += + acadoWorkspace.H10[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[25]*acadoWorkspace.Dx0[1];
acadoWorkspace.g[13] += + acadoWorkspace.H10[26]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[27]*acadoWorkspace.Dx0[1];
acadoWorkspace.g[14] += + acadoWorkspace.H10[28]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[29]*acadoWorkspace.Dx0[1];
acadoWorkspace.g[15] += + acadoWorkspace.H10[30]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[31]*acadoWorkspace.Dx0[1];
acadoWorkspace.g[16] += + acadoWorkspace.H10[32]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[33]*acadoWorkspace.Dx0[1];
acadoWorkspace.g[17] += + acadoWorkspace.H10[34]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[35]*acadoWorkspace.Dx0[1];
acadoWorkspace.g[18] += + acadoWorkspace.H10[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[37]*acadoWorkspace.Dx0[1];
acadoWorkspace.g[19] += + acadoWorkspace.H10[38]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[39]*acadoWorkspace.Dx0[1];

tmp = + acadoWorkspace.evGx[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1]*acadoWorkspace.Dx0[1] + acadoVariables.x[2];
tmp += acadoWorkspace.d[0];
acadoWorkspace.lbA[0] = (real_t)-3.1400000000000001e+00 - tmp;
acadoWorkspace.ubA[0] = (real_t)3.1400000000000001e+00 - tmp;
tmp = + acadoWorkspace.evGx[2]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[3]*acadoWorkspace.Dx0[1] + acadoVariables.x[3];
tmp += acadoWorkspace.d[1];
acadoWorkspace.lbA[1] = (real_t)-1.5700000000000001e+00 - tmp;
acadoWorkspace.ubA[1] = (real_t)1.5700000000000001e+00 - tmp;
tmp = + acadoWorkspace.evGx[4]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[5]*acadoWorkspace.Dx0[1] + acadoVariables.x[4];
tmp += acadoWorkspace.d[2];
acadoWorkspace.lbA[2] = (real_t)-3.1400000000000001e+00 - tmp;
acadoWorkspace.ubA[2] = (real_t)3.1400000000000001e+00 - tmp;
tmp = + acadoWorkspace.evGx[6]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[7]*acadoWorkspace.Dx0[1] + acadoVariables.x[5];
tmp += acadoWorkspace.d[3];
acadoWorkspace.lbA[3] = (real_t)-1.5700000000000001e+00 - tmp;
acadoWorkspace.ubA[3] = (real_t)1.5700000000000001e+00 - tmp;
tmp = + acadoWorkspace.evGx[8]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[9]*acadoWorkspace.Dx0[1] + acadoVariables.x[6];
tmp += acadoWorkspace.d[4];
acadoWorkspace.lbA[4] = (real_t)-3.1400000000000001e+00 - tmp;
acadoWorkspace.ubA[4] = (real_t)3.1400000000000001e+00 - tmp;
tmp = + acadoWorkspace.evGx[10]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[11]*acadoWorkspace.Dx0[1] + acadoVariables.x[7];
tmp += acadoWorkspace.d[5];
acadoWorkspace.lbA[5] = (real_t)-1.5700000000000001e+00 - tmp;
acadoWorkspace.ubA[5] = (real_t)1.5700000000000001e+00 - tmp;
tmp = + acadoWorkspace.evGx[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[13]*acadoWorkspace.Dx0[1] + acadoVariables.x[8];
tmp += acadoWorkspace.d[6];
acadoWorkspace.lbA[6] = (real_t)-3.1400000000000001e+00 - tmp;
acadoWorkspace.ubA[6] = (real_t)3.1400000000000001e+00 - tmp;
tmp = + acadoWorkspace.evGx[14]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[15]*acadoWorkspace.Dx0[1] + acadoVariables.x[9];
tmp += acadoWorkspace.d[7];
acadoWorkspace.lbA[7] = (real_t)-1.5700000000000001e+00 - tmp;
acadoWorkspace.ubA[7] = (real_t)1.5700000000000001e+00 - tmp;
tmp = + acadoWorkspace.evGx[16]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[17]*acadoWorkspace.Dx0[1] + acadoVariables.x[10];
tmp += acadoWorkspace.d[8];
acadoWorkspace.lbA[8] = (real_t)-3.1400000000000001e+00 - tmp;
acadoWorkspace.ubA[8] = (real_t)3.1400000000000001e+00 - tmp;
tmp = + acadoWorkspace.evGx[18]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[19]*acadoWorkspace.Dx0[1] + acadoVariables.x[11];
tmp += acadoWorkspace.d[9];
acadoWorkspace.lbA[9] = (real_t)-1.5700000000000001e+00 - tmp;
acadoWorkspace.ubA[9] = (real_t)1.5700000000000001e+00 - tmp;
tmp = + acadoWorkspace.evGx[20]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[21]*acadoWorkspace.Dx0[1] + acadoVariables.x[12];
tmp += acadoWorkspace.d[10];
acadoWorkspace.lbA[10] = (real_t)-3.1400000000000001e+00 - tmp;
acadoWorkspace.ubA[10] = (real_t)3.1400000000000001e+00 - tmp;
tmp = + acadoWorkspace.evGx[22]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[23]*acadoWorkspace.Dx0[1] + acadoVariables.x[13];
tmp += acadoWorkspace.d[11];
acadoWorkspace.lbA[11] = (real_t)-1.5700000000000001e+00 - tmp;
acadoWorkspace.ubA[11] = (real_t)1.5700000000000001e+00 - tmp;
tmp = + acadoWorkspace.evGx[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[25]*acadoWorkspace.Dx0[1] + acadoVariables.x[14];
tmp += acadoWorkspace.d[12];
acadoWorkspace.lbA[12] = (real_t)-3.1400000000000001e+00 - tmp;
acadoWorkspace.ubA[12] = (real_t)3.1400000000000001e+00 - tmp;
tmp = + acadoWorkspace.evGx[26]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[27]*acadoWorkspace.Dx0[1] + acadoVariables.x[15];
tmp += acadoWorkspace.d[13];
acadoWorkspace.lbA[13] = (real_t)-1.5700000000000001e+00 - tmp;
acadoWorkspace.ubA[13] = (real_t)1.5700000000000001e+00 - tmp;
tmp = + acadoWorkspace.evGx[28]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[29]*acadoWorkspace.Dx0[1] + acadoVariables.x[16];
tmp += acadoWorkspace.d[14];
acadoWorkspace.lbA[14] = (real_t)-3.1400000000000001e+00 - tmp;
acadoWorkspace.ubA[14] = (real_t)3.1400000000000001e+00 - tmp;
tmp = + acadoWorkspace.evGx[30]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[31]*acadoWorkspace.Dx0[1] + acadoVariables.x[17];
tmp += acadoWorkspace.d[15];
acadoWorkspace.lbA[15] = (real_t)-1.5700000000000001e+00 - tmp;
acadoWorkspace.ubA[15] = (real_t)1.5700000000000001e+00 - tmp;
tmp = + acadoWorkspace.evGx[32]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[33]*acadoWorkspace.Dx0[1] + acadoVariables.x[18];
tmp += acadoWorkspace.d[16];
acadoWorkspace.lbA[16] = (real_t)-3.1400000000000001e+00 - tmp;
acadoWorkspace.ubA[16] = (real_t)3.1400000000000001e+00 - tmp;
tmp = + acadoWorkspace.evGx[34]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[35]*acadoWorkspace.Dx0[1] + acadoVariables.x[19];
tmp += acadoWorkspace.d[17];
acadoWorkspace.lbA[17] = (real_t)-1.5700000000000001e+00 - tmp;
acadoWorkspace.ubA[17] = (real_t)1.5700000000000001e+00 - tmp;
tmp = + acadoWorkspace.evGx[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[37]*acadoWorkspace.Dx0[1] + acadoVariables.x[20];
tmp += acadoWorkspace.d[18];
acadoWorkspace.lbA[18] = (real_t)-3.1400000000000001e+00 - tmp;
acadoWorkspace.ubA[18] = (real_t)3.1400000000000001e+00 - tmp;
tmp = + acadoWorkspace.evGx[38]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[39]*acadoWorkspace.Dx0[1] + acadoVariables.x[21];
tmp += acadoWorkspace.d[19];
acadoWorkspace.lbA[19] = (real_t)-1.5700000000000001e+00 - tmp;
acadoWorkspace.ubA[19] = (real_t)1.5700000000000001e+00 - tmp;

acadoWorkspace.pacA01Dx0[0] = + acadoWorkspace.A01[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[1]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[1] = + acadoWorkspace.A01[2]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[3]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[2] = + acadoWorkspace.A01[4]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[5]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[3] = + acadoWorkspace.A01[6]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[7]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[4] = + acadoWorkspace.A01[8]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[9]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[5] = + acadoWorkspace.A01[10]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[11]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[6] = + acadoWorkspace.A01[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[13]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[7] = + acadoWorkspace.A01[14]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[15]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[8] = + acadoWorkspace.A01[16]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[17]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[9] = + acadoWorkspace.A01[18]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[19]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[10] = + acadoWorkspace.A01[20]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[21]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[11] = + acadoWorkspace.A01[22]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[23]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[12] = + acadoWorkspace.A01[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[25]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[13] = + acadoWorkspace.A01[26]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[27]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[14] = + acadoWorkspace.A01[28]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[29]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[15] = + acadoWorkspace.A01[30]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[31]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[16] = + acadoWorkspace.A01[32]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[33]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[17] = + acadoWorkspace.A01[34]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[35]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[18] = + acadoWorkspace.A01[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[37]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[19] = + acadoWorkspace.A01[38]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[39]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[20] = + acadoWorkspace.A01[40]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[41]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[21] = + acadoWorkspace.A01[42]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[43]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[22] = + acadoWorkspace.A01[44]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[45]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[23] = + acadoWorkspace.A01[46]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[47]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[24] = + acadoWorkspace.A01[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[49]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[25] = + acadoWorkspace.A01[50]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[51]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[26] = + acadoWorkspace.A01[52]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[53]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[27] = + acadoWorkspace.A01[54]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[55]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[28] = + acadoWorkspace.A01[56]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[57]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[29] = + acadoWorkspace.A01[58]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[59]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[30] = + acadoWorkspace.A01[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[61]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[31] = + acadoWorkspace.A01[62]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[63]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[32] = + acadoWorkspace.A01[64]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[65]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[33] = + acadoWorkspace.A01[66]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[67]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[34] = + acadoWorkspace.A01[68]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[69]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[35] = + acadoWorkspace.A01[70]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[71]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[36] = + acadoWorkspace.A01[72]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[73]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[37] = + acadoWorkspace.A01[74]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[75]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[38] = + acadoWorkspace.A01[76]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[77]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[39] = + acadoWorkspace.A01[78]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[79]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[40] = + acadoWorkspace.A01[80]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[81]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[41] = + acadoWorkspace.A01[82]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[83]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[42] = + acadoWorkspace.A01[84]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[85]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[43] = + acadoWorkspace.A01[86]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[87]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[44] = + acadoWorkspace.A01[88]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[89]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[45] = + acadoWorkspace.A01[90]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[91]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[46] = + acadoWorkspace.A01[92]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[93]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[47] = + acadoWorkspace.A01[94]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[95]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[48] = + acadoWorkspace.A01[96]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[97]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[49] = + acadoWorkspace.A01[98]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[99]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[50] = + acadoWorkspace.A01[100]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[101]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[51] = + acadoWorkspace.A01[102]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[103]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[52] = + acadoWorkspace.A01[104]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[105]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[53] = + acadoWorkspace.A01[106]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[107]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[54] = + acadoWorkspace.A01[108]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[109]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[55] = + acadoWorkspace.A01[110]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[111]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[56] = + acadoWorkspace.A01[112]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[113]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[57] = + acadoWorkspace.A01[114]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[115]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[58] = + acadoWorkspace.A01[116]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[117]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[59] = + acadoWorkspace.A01[118]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[119]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[60] = + acadoWorkspace.A01[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[121]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[61] = + acadoWorkspace.A01[122]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[123]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[62] = + acadoWorkspace.A01[124]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[125]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[63] = + acadoWorkspace.A01[126]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[127]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[64] = + acadoWorkspace.A01[128]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[129]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[65] = + acadoWorkspace.A01[130]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[131]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[66] = + acadoWorkspace.A01[132]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[133]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[67] = + acadoWorkspace.A01[134]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[135]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[68] = + acadoWorkspace.A01[136]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[137]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[69] = + acadoWorkspace.A01[138]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[139]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[70] = + acadoWorkspace.A01[140]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[141]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[71] = + acadoWorkspace.A01[142]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[143]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[72] = + acadoWorkspace.A01[144]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[145]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[73] = + acadoWorkspace.A01[146]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[147]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[74] = + acadoWorkspace.A01[148]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[149]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[75] = + acadoWorkspace.A01[150]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[151]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[76] = + acadoWorkspace.A01[152]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[153]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[77] = + acadoWorkspace.A01[154]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[155]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[78] = + acadoWorkspace.A01[156]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[157]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[79] = + acadoWorkspace.A01[158]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[159]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[80] = + acadoWorkspace.A01[160]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[161]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[81] = + acadoWorkspace.A01[162]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[163]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[82] = + acadoWorkspace.A01[164]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[165]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[83] = + acadoWorkspace.A01[166]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[167]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[84] = + acadoWorkspace.A01[168]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[169]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[85] = + acadoWorkspace.A01[170]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[171]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[86] = + acadoWorkspace.A01[172]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[173]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[87] = + acadoWorkspace.A01[174]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[175]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[88] = + acadoWorkspace.A01[176]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[177]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[89] = + acadoWorkspace.A01[178]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[179]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[90] = + acadoWorkspace.A01[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[181]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[91] = + acadoWorkspace.A01[182]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[183]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[92] = + acadoWorkspace.A01[184]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[185]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[93] = + acadoWorkspace.A01[186]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[187]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[94] = + acadoWorkspace.A01[188]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[189]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[95] = + acadoWorkspace.A01[190]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[191]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[96] = + acadoWorkspace.A01[192]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[193]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[97] = + acadoWorkspace.A01[194]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[195]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[98] = + acadoWorkspace.A01[196]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[197]*acadoWorkspace.Dx0[1];
acadoWorkspace.pacA01Dx0[99] = + acadoWorkspace.A01[198]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[199]*acadoWorkspace.Dx0[1];
acadoWorkspace.lbA[20] -= acadoWorkspace.pacA01Dx0[0];
acadoWorkspace.lbA[21] -= acadoWorkspace.pacA01Dx0[1];
acadoWorkspace.lbA[22] -= acadoWorkspace.pacA01Dx0[2];
acadoWorkspace.lbA[23] -= acadoWorkspace.pacA01Dx0[3];
acadoWorkspace.lbA[24] -= acadoWorkspace.pacA01Dx0[4];
acadoWorkspace.lbA[25] -= acadoWorkspace.pacA01Dx0[5];
acadoWorkspace.lbA[26] -= acadoWorkspace.pacA01Dx0[6];
acadoWorkspace.lbA[27] -= acadoWorkspace.pacA01Dx0[7];
acadoWorkspace.lbA[28] -= acadoWorkspace.pacA01Dx0[8];
acadoWorkspace.lbA[29] -= acadoWorkspace.pacA01Dx0[9];
acadoWorkspace.lbA[30] -= acadoWorkspace.pacA01Dx0[10];
acadoWorkspace.lbA[31] -= acadoWorkspace.pacA01Dx0[11];
acadoWorkspace.lbA[32] -= acadoWorkspace.pacA01Dx0[12];
acadoWorkspace.lbA[33] -= acadoWorkspace.pacA01Dx0[13];
acadoWorkspace.lbA[34] -= acadoWorkspace.pacA01Dx0[14];
acadoWorkspace.lbA[35] -= acadoWorkspace.pacA01Dx0[15];
acadoWorkspace.lbA[36] -= acadoWorkspace.pacA01Dx0[16];
acadoWorkspace.lbA[37] -= acadoWorkspace.pacA01Dx0[17];
acadoWorkspace.lbA[38] -= acadoWorkspace.pacA01Dx0[18];
acadoWorkspace.lbA[39] -= acadoWorkspace.pacA01Dx0[19];
acadoWorkspace.lbA[40] -= acadoWorkspace.pacA01Dx0[20];
acadoWorkspace.lbA[41] -= acadoWorkspace.pacA01Dx0[21];
acadoWorkspace.lbA[42] -= acadoWorkspace.pacA01Dx0[22];
acadoWorkspace.lbA[43] -= acadoWorkspace.pacA01Dx0[23];
acadoWorkspace.lbA[44] -= acadoWorkspace.pacA01Dx0[24];
acadoWorkspace.lbA[45] -= acadoWorkspace.pacA01Dx0[25];
acadoWorkspace.lbA[46] -= acadoWorkspace.pacA01Dx0[26];
acadoWorkspace.lbA[47] -= acadoWorkspace.pacA01Dx0[27];
acadoWorkspace.lbA[48] -= acadoWorkspace.pacA01Dx0[28];
acadoWorkspace.lbA[49] -= acadoWorkspace.pacA01Dx0[29];
acadoWorkspace.lbA[50] -= acadoWorkspace.pacA01Dx0[30];
acadoWorkspace.lbA[51] -= acadoWorkspace.pacA01Dx0[31];
acadoWorkspace.lbA[52] -= acadoWorkspace.pacA01Dx0[32];
acadoWorkspace.lbA[53] -= acadoWorkspace.pacA01Dx0[33];
acadoWorkspace.lbA[54] -= acadoWorkspace.pacA01Dx0[34];
acadoWorkspace.lbA[55] -= acadoWorkspace.pacA01Dx0[35];
acadoWorkspace.lbA[56] -= acadoWorkspace.pacA01Dx0[36];
acadoWorkspace.lbA[57] -= acadoWorkspace.pacA01Dx0[37];
acadoWorkspace.lbA[58] -= acadoWorkspace.pacA01Dx0[38];
acadoWorkspace.lbA[59] -= acadoWorkspace.pacA01Dx0[39];
acadoWorkspace.lbA[60] -= acadoWorkspace.pacA01Dx0[40];
acadoWorkspace.lbA[61] -= acadoWorkspace.pacA01Dx0[41];
acadoWorkspace.lbA[62] -= acadoWorkspace.pacA01Dx0[42];
acadoWorkspace.lbA[63] -= acadoWorkspace.pacA01Dx0[43];
acadoWorkspace.lbA[64] -= acadoWorkspace.pacA01Dx0[44];
acadoWorkspace.lbA[65] -= acadoWorkspace.pacA01Dx0[45];
acadoWorkspace.lbA[66] -= acadoWorkspace.pacA01Dx0[46];
acadoWorkspace.lbA[67] -= acadoWorkspace.pacA01Dx0[47];
acadoWorkspace.lbA[68] -= acadoWorkspace.pacA01Dx0[48];
acadoWorkspace.lbA[69] -= acadoWorkspace.pacA01Dx0[49];
acadoWorkspace.lbA[70] -= acadoWorkspace.pacA01Dx0[50];
acadoWorkspace.lbA[71] -= acadoWorkspace.pacA01Dx0[51];
acadoWorkspace.lbA[72] -= acadoWorkspace.pacA01Dx0[52];
acadoWorkspace.lbA[73] -= acadoWorkspace.pacA01Dx0[53];
acadoWorkspace.lbA[74] -= acadoWorkspace.pacA01Dx0[54];
acadoWorkspace.lbA[75] -= acadoWorkspace.pacA01Dx0[55];
acadoWorkspace.lbA[76] -= acadoWorkspace.pacA01Dx0[56];
acadoWorkspace.lbA[77] -= acadoWorkspace.pacA01Dx0[57];
acadoWorkspace.lbA[78] -= acadoWorkspace.pacA01Dx0[58];
acadoWorkspace.lbA[79] -= acadoWorkspace.pacA01Dx0[59];
acadoWorkspace.lbA[80] -= acadoWorkspace.pacA01Dx0[60];
acadoWorkspace.lbA[81] -= acadoWorkspace.pacA01Dx0[61];
acadoWorkspace.lbA[82] -= acadoWorkspace.pacA01Dx0[62];
acadoWorkspace.lbA[83] -= acadoWorkspace.pacA01Dx0[63];
acadoWorkspace.lbA[84] -= acadoWorkspace.pacA01Dx0[64];
acadoWorkspace.lbA[85] -= acadoWorkspace.pacA01Dx0[65];
acadoWorkspace.lbA[86] -= acadoWorkspace.pacA01Dx0[66];
acadoWorkspace.lbA[87] -= acadoWorkspace.pacA01Dx0[67];
acadoWorkspace.lbA[88] -= acadoWorkspace.pacA01Dx0[68];
acadoWorkspace.lbA[89] -= acadoWorkspace.pacA01Dx0[69];
acadoWorkspace.lbA[90] -= acadoWorkspace.pacA01Dx0[70];
acadoWorkspace.lbA[91] -= acadoWorkspace.pacA01Dx0[71];
acadoWorkspace.lbA[92] -= acadoWorkspace.pacA01Dx0[72];
acadoWorkspace.lbA[93] -= acadoWorkspace.pacA01Dx0[73];
acadoWorkspace.lbA[94] -= acadoWorkspace.pacA01Dx0[74];
acadoWorkspace.lbA[95] -= acadoWorkspace.pacA01Dx0[75];
acadoWorkspace.lbA[96] -= acadoWorkspace.pacA01Dx0[76];
acadoWorkspace.lbA[97] -= acadoWorkspace.pacA01Dx0[77];
acadoWorkspace.lbA[98] -= acadoWorkspace.pacA01Dx0[78];
acadoWorkspace.lbA[99] -= acadoWorkspace.pacA01Dx0[79];
acadoWorkspace.lbA[100] -= acadoWorkspace.pacA01Dx0[80];
acadoWorkspace.lbA[101] -= acadoWorkspace.pacA01Dx0[81];
acadoWorkspace.lbA[102] -= acadoWorkspace.pacA01Dx0[82];
acadoWorkspace.lbA[103] -= acadoWorkspace.pacA01Dx0[83];
acadoWorkspace.lbA[104] -= acadoWorkspace.pacA01Dx0[84];
acadoWorkspace.lbA[105] -= acadoWorkspace.pacA01Dx0[85];
acadoWorkspace.lbA[106] -= acadoWorkspace.pacA01Dx0[86];
acadoWorkspace.lbA[107] -= acadoWorkspace.pacA01Dx0[87];
acadoWorkspace.lbA[108] -= acadoWorkspace.pacA01Dx0[88];
acadoWorkspace.lbA[109] -= acadoWorkspace.pacA01Dx0[89];
acadoWorkspace.lbA[110] -= acadoWorkspace.pacA01Dx0[90];
acadoWorkspace.lbA[111] -= acadoWorkspace.pacA01Dx0[91];
acadoWorkspace.lbA[112] -= acadoWorkspace.pacA01Dx0[92];
acadoWorkspace.lbA[113] -= acadoWorkspace.pacA01Dx0[93];
acadoWorkspace.lbA[114] -= acadoWorkspace.pacA01Dx0[94];
acadoWorkspace.lbA[115] -= acadoWorkspace.pacA01Dx0[95];
acadoWorkspace.lbA[116] -= acadoWorkspace.pacA01Dx0[96];
acadoWorkspace.lbA[117] -= acadoWorkspace.pacA01Dx0[97];
acadoWorkspace.lbA[118] -= acadoWorkspace.pacA01Dx0[98];
acadoWorkspace.lbA[119] -= acadoWorkspace.pacA01Dx0[99];

acadoWorkspace.ubA[20] -= acadoWorkspace.pacA01Dx0[0];
acadoWorkspace.ubA[21] -= acadoWorkspace.pacA01Dx0[1];
acadoWorkspace.ubA[22] -= acadoWorkspace.pacA01Dx0[2];
acadoWorkspace.ubA[23] -= acadoWorkspace.pacA01Dx0[3];
acadoWorkspace.ubA[24] -= acadoWorkspace.pacA01Dx0[4];
acadoWorkspace.ubA[25] -= acadoWorkspace.pacA01Dx0[5];
acadoWorkspace.ubA[26] -= acadoWorkspace.pacA01Dx0[6];
acadoWorkspace.ubA[27] -= acadoWorkspace.pacA01Dx0[7];
acadoWorkspace.ubA[28] -= acadoWorkspace.pacA01Dx0[8];
acadoWorkspace.ubA[29] -= acadoWorkspace.pacA01Dx0[9];
acadoWorkspace.ubA[30] -= acadoWorkspace.pacA01Dx0[10];
acadoWorkspace.ubA[31] -= acadoWorkspace.pacA01Dx0[11];
acadoWorkspace.ubA[32] -= acadoWorkspace.pacA01Dx0[12];
acadoWorkspace.ubA[33] -= acadoWorkspace.pacA01Dx0[13];
acadoWorkspace.ubA[34] -= acadoWorkspace.pacA01Dx0[14];
acadoWorkspace.ubA[35] -= acadoWorkspace.pacA01Dx0[15];
acadoWorkspace.ubA[36] -= acadoWorkspace.pacA01Dx0[16];
acadoWorkspace.ubA[37] -= acadoWorkspace.pacA01Dx0[17];
acadoWorkspace.ubA[38] -= acadoWorkspace.pacA01Dx0[18];
acadoWorkspace.ubA[39] -= acadoWorkspace.pacA01Dx0[19];
acadoWorkspace.ubA[40] -= acadoWorkspace.pacA01Dx0[20];
acadoWorkspace.ubA[41] -= acadoWorkspace.pacA01Dx0[21];
acadoWorkspace.ubA[42] -= acadoWorkspace.pacA01Dx0[22];
acadoWorkspace.ubA[43] -= acadoWorkspace.pacA01Dx0[23];
acadoWorkspace.ubA[44] -= acadoWorkspace.pacA01Dx0[24];
acadoWorkspace.ubA[45] -= acadoWorkspace.pacA01Dx0[25];
acadoWorkspace.ubA[46] -= acadoWorkspace.pacA01Dx0[26];
acadoWorkspace.ubA[47] -= acadoWorkspace.pacA01Dx0[27];
acadoWorkspace.ubA[48] -= acadoWorkspace.pacA01Dx0[28];
acadoWorkspace.ubA[49] -= acadoWorkspace.pacA01Dx0[29];
acadoWorkspace.ubA[50] -= acadoWorkspace.pacA01Dx0[30];
acadoWorkspace.ubA[51] -= acadoWorkspace.pacA01Dx0[31];
acadoWorkspace.ubA[52] -= acadoWorkspace.pacA01Dx0[32];
acadoWorkspace.ubA[53] -= acadoWorkspace.pacA01Dx0[33];
acadoWorkspace.ubA[54] -= acadoWorkspace.pacA01Dx0[34];
acadoWorkspace.ubA[55] -= acadoWorkspace.pacA01Dx0[35];
acadoWorkspace.ubA[56] -= acadoWorkspace.pacA01Dx0[36];
acadoWorkspace.ubA[57] -= acadoWorkspace.pacA01Dx0[37];
acadoWorkspace.ubA[58] -= acadoWorkspace.pacA01Dx0[38];
acadoWorkspace.ubA[59] -= acadoWorkspace.pacA01Dx0[39];
acadoWorkspace.ubA[60] -= acadoWorkspace.pacA01Dx0[40];
acadoWorkspace.ubA[61] -= acadoWorkspace.pacA01Dx0[41];
acadoWorkspace.ubA[62] -= acadoWorkspace.pacA01Dx0[42];
acadoWorkspace.ubA[63] -= acadoWorkspace.pacA01Dx0[43];
acadoWorkspace.ubA[64] -= acadoWorkspace.pacA01Dx0[44];
acadoWorkspace.ubA[65] -= acadoWorkspace.pacA01Dx0[45];
acadoWorkspace.ubA[66] -= acadoWorkspace.pacA01Dx0[46];
acadoWorkspace.ubA[67] -= acadoWorkspace.pacA01Dx0[47];
acadoWorkspace.ubA[68] -= acadoWorkspace.pacA01Dx0[48];
acadoWorkspace.ubA[69] -= acadoWorkspace.pacA01Dx0[49];
acadoWorkspace.ubA[70] -= acadoWorkspace.pacA01Dx0[50];
acadoWorkspace.ubA[71] -= acadoWorkspace.pacA01Dx0[51];
acadoWorkspace.ubA[72] -= acadoWorkspace.pacA01Dx0[52];
acadoWorkspace.ubA[73] -= acadoWorkspace.pacA01Dx0[53];
acadoWorkspace.ubA[74] -= acadoWorkspace.pacA01Dx0[54];
acadoWorkspace.ubA[75] -= acadoWorkspace.pacA01Dx0[55];
acadoWorkspace.ubA[76] -= acadoWorkspace.pacA01Dx0[56];
acadoWorkspace.ubA[77] -= acadoWorkspace.pacA01Dx0[57];
acadoWorkspace.ubA[78] -= acadoWorkspace.pacA01Dx0[58];
acadoWorkspace.ubA[79] -= acadoWorkspace.pacA01Dx0[59];
acadoWorkspace.ubA[80] -= acadoWorkspace.pacA01Dx0[60];
acadoWorkspace.ubA[81] -= acadoWorkspace.pacA01Dx0[61];
acadoWorkspace.ubA[82] -= acadoWorkspace.pacA01Dx0[62];
acadoWorkspace.ubA[83] -= acadoWorkspace.pacA01Dx0[63];
acadoWorkspace.ubA[84] -= acadoWorkspace.pacA01Dx0[64];
acadoWorkspace.ubA[85] -= acadoWorkspace.pacA01Dx0[65];
acadoWorkspace.ubA[86] -= acadoWorkspace.pacA01Dx0[66];
acadoWorkspace.ubA[87] -= acadoWorkspace.pacA01Dx0[67];
acadoWorkspace.ubA[88] -= acadoWorkspace.pacA01Dx0[68];
acadoWorkspace.ubA[89] -= acadoWorkspace.pacA01Dx0[69];
acadoWorkspace.ubA[90] -= acadoWorkspace.pacA01Dx0[70];
acadoWorkspace.ubA[91] -= acadoWorkspace.pacA01Dx0[71];
acadoWorkspace.ubA[92] -= acadoWorkspace.pacA01Dx0[72];
acadoWorkspace.ubA[93] -= acadoWorkspace.pacA01Dx0[73];
acadoWorkspace.ubA[94] -= acadoWorkspace.pacA01Dx0[74];
acadoWorkspace.ubA[95] -= acadoWorkspace.pacA01Dx0[75];
acadoWorkspace.ubA[96] -= acadoWorkspace.pacA01Dx0[76];
acadoWorkspace.ubA[97] -= acadoWorkspace.pacA01Dx0[77];
acadoWorkspace.ubA[98] -= acadoWorkspace.pacA01Dx0[78];
acadoWorkspace.ubA[99] -= acadoWorkspace.pacA01Dx0[79];
acadoWorkspace.ubA[100] -= acadoWorkspace.pacA01Dx0[80];
acadoWorkspace.ubA[101] -= acadoWorkspace.pacA01Dx0[81];
acadoWorkspace.ubA[102] -= acadoWorkspace.pacA01Dx0[82];
acadoWorkspace.ubA[103] -= acadoWorkspace.pacA01Dx0[83];
acadoWorkspace.ubA[104] -= acadoWorkspace.pacA01Dx0[84];
acadoWorkspace.ubA[105] -= acadoWorkspace.pacA01Dx0[85];
acadoWorkspace.ubA[106] -= acadoWorkspace.pacA01Dx0[86];
acadoWorkspace.ubA[107] -= acadoWorkspace.pacA01Dx0[87];
acadoWorkspace.ubA[108] -= acadoWorkspace.pacA01Dx0[88];
acadoWorkspace.ubA[109] -= acadoWorkspace.pacA01Dx0[89];
acadoWorkspace.ubA[110] -= acadoWorkspace.pacA01Dx0[90];
acadoWorkspace.ubA[111] -= acadoWorkspace.pacA01Dx0[91];
acadoWorkspace.ubA[112] -= acadoWorkspace.pacA01Dx0[92];
acadoWorkspace.ubA[113] -= acadoWorkspace.pacA01Dx0[93];
acadoWorkspace.ubA[114] -= acadoWorkspace.pacA01Dx0[94];
acadoWorkspace.ubA[115] -= acadoWorkspace.pacA01Dx0[95];
acadoWorkspace.ubA[116] -= acadoWorkspace.pacA01Dx0[96];
acadoWorkspace.ubA[117] -= acadoWorkspace.pacA01Dx0[97];
acadoWorkspace.ubA[118] -= acadoWorkspace.pacA01Dx0[98];
acadoWorkspace.ubA[119] -= acadoWorkspace.pacA01Dx0[99];

}

void acado_expand(  )
{
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];
acadoVariables.u[10] += acadoWorkspace.x[10];
acadoVariables.u[11] += acadoWorkspace.x[11];
acadoVariables.u[12] += acadoWorkspace.x[12];
acadoVariables.u[13] += acadoWorkspace.x[13];
acadoVariables.u[14] += acadoWorkspace.x[14];
acadoVariables.u[15] += acadoWorkspace.x[15];
acadoVariables.u[16] += acadoWorkspace.x[16];
acadoVariables.u[17] += acadoWorkspace.x[17];
acadoVariables.u[18] += acadoWorkspace.x[18];
acadoVariables.u[19] += acadoWorkspace.x[19];

acadoVariables.x[0] += acadoWorkspace.Dx0[0];
acadoVariables.x[1] += acadoWorkspace.Dx0[1];

acadoVariables.x[2] += + acadoWorkspace.evGx[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.d[0];
acadoVariables.x[3] += + acadoWorkspace.evGx[2]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[3]*acadoWorkspace.Dx0[1] + acadoWorkspace.d[1];
acadoVariables.x[4] += + acadoWorkspace.evGx[4]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[5]*acadoWorkspace.Dx0[1] + acadoWorkspace.d[2];
acadoVariables.x[5] += + acadoWorkspace.evGx[6]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[7]*acadoWorkspace.Dx0[1] + acadoWorkspace.d[3];
acadoVariables.x[6] += + acadoWorkspace.evGx[8]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[9]*acadoWorkspace.Dx0[1] + acadoWorkspace.d[4];
acadoVariables.x[7] += + acadoWorkspace.evGx[10]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[11]*acadoWorkspace.Dx0[1] + acadoWorkspace.d[5];
acadoVariables.x[8] += + acadoWorkspace.evGx[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[13]*acadoWorkspace.Dx0[1] + acadoWorkspace.d[6];
acadoVariables.x[9] += + acadoWorkspace.evGx[14]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[15]*acadoWorkspace.Dx0[1] + acadoWorkspace.d[7];
acadoVariables.x[10] += + acadoWorkspace.evGx[16]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[17]*acadoWorkspace.Dx0[1] + acadoWorkspace.d[8];
acadoVariables.x[11] += + acadoWorkspace.evGx[18]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[19]*acadoWorkspace.Dx0[1] + acadoWorkspace.d[9];
acadoVariables.x[12] += + acadoWorkspace.evGx[20]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[21]*acadoWorkspace.Dx0[1] + acadoWorkspace.d[10];
acadoVariables.x[13] += + acadoWorkspace.evGx[22]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[23]*acadoWorkspace.Dx0[1] + acadoWorkspace.d[11];
acadoVariables.x[14] += + acadoWorkspace.evGx[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.d[12];
acadoVariables.x[15] += + acadoWorkspace.evGx[26]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[27]*acadoWorkspace.Dx0[1] + acadoWorkspace.d[13];
acadoVariables.x[16] += + acadoWorkspace.evGx[28]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[29]*acadoWorkspace.Dx0[1] + acadoWorkspace.d[14];
acadoVariables.x[17] += + acadoWorkspace.evGx[30]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[31]*acadoWorkspace.Dx0[1] + acadoWorkspace.d[15];
acadoVariables.x[18] += + acadoWorkspace.evGx[32]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[33]*acadoWorkspace.Dx0[1] + acadoWorkspace.d[16];
acadoVariables.x[19] += + acadoWorkspace.evGx[34]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[35]*acadoWorkspace.Dx0[1] + acadoWorkspace.d[17];
acadoVariables.x[20] += + acadoWorkspace.evGx[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.d[18];
acadoVariables.x[21] += + acadoWorkspace.evGx[38]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[39]*acadoWorkspace.Dx0[1] + acadoWorkspace.d[19];

acado_multEDu( acadoWorkspace.E, acadoWorkspace.x, &(acadoVariables.x[ 2 ]) );
acado_multEDu( &(acadoWorkspace.E[ 4 ]), acadoWorkspace.x, &(acadoVariables.x[ 4 ]) );
acado_multEDu( &(acadoWorkspace.E[ 8 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 4 ]) );
acado_multEDu( &(acadoWorkspace.E[ 12 ]), acadoWorkspace.x, &(acadoVariables.x[ 6 ]) );
acado_multEDu( &(acadoWorkspace.E[ 16 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 6 ]) );
acado_multEDu( &(acadoWorkspace.E[ 20 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 6 ]) );
acado_multEDu( &(acadoWorkspace.E[ 24 ]), acadoWorkspace.x, &(acadoVariables.x[ 8 ]) );
acado_multEDu( &(acadoWorkspace.E[ 28 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 8 ]) );
acado_multEDu( &(acadoWorkspace.E[ 32 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 8 ]) );
acado_multEDu( &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 8 ]) );
acado_multEDu( &(acadoWorkspace.E[ 40 ]), acadoWorkspace.x, &(acadoVariables.x[ 10 ]) );
acado_multEDu( &(acadoWorkspace.E[ 44 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 10 ]) );
acado_multEDu( &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 10 ]) );
acado_multEDu( &(acadoWorkspace.E[ 52 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 10 ]) );
acado_multEDu( &(acadoWorkspace.E[ 56 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 10 ]) );
acado_multEDu( &(acadoWorkspace.E[ 60 ]), acadoWorkspace.x, &(acadoVariables.x[ 12 ]) );
acado_multEDu( &(acadoWorkspace.E[ 64 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 12 ]) );
acado_multEDu( &(acadoWorkspace.E[ 68 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 12 ]) );
acado_multEDu( &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 12 ]) );
acado_multEDu( &(acadoWorkspace.E[ 76 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 12 ]) );
acado_multEDu( &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 12 ]) );
acado_multEDu( &(acadoWorkspace.E[ 84 ]), acadoWorkspace.x, &(acadoVariables.x[ 14 ]) );
acado_multEDu( &(acadoWorkspace.E[ 88 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 14 ]) );
acado_multEDu( &(acadoWorkspace.E[ 92 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 14 ]) );
acado_multEDu( &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 14 ]) );
acado_multEDu( &(acadoWorkspace.E[ 100 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 14 ]) );
acado_multEDu( &(acadoWorkspace.E[ 104 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 14 ]) );
acado_multEDu( &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 14 ]) );
acado_multEDu( &(acadoWorkspace.E[ 112 ]), acadoWorkspace.x, &(acadoVariables.x[ 16 ]) );
acado_multEDu( &(acadoWorkspace.E[ 116 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 16 ]) );
acado_multEDu( &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 16 ]) );
acado_multEDu( &(acadoWorkspace.E[ 124 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 16 ]) );
acado_multEDu( &(acadoWorkspace.E[ 128 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 16 ]) );
acado_multEDu( &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 16 ]) );
acado_multEDu( &(acadoWorkspace.E[ 136 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 16 ]) );
acado_multEDu( &(acadoWorkspace.E[ 140 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 16 ]) );
acado_multEDu( &(acadoWorkspace.E[ 144 ]), acadoWorkspace.x, &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 148 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 152 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 164 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 172 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 176 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 180 ]), acadoWorkspace.x, &(acadoVariables.x[ 20 ]) );
acado_multEDu( &(acadoWorkspace.E[ 184 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 20 ]) );
acado_multEDu( &(acadoWorkspace.E[ 188 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 20 ]) );
acado_multEDu( &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 20 ]) );
acado_multEDu( &(acadoWorkspace.E[ 196 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 20 ]) );
acado_multEDu( &(acadoWorkspace.E[ 200 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 20 ]) );
acado_multEDu( &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 20 ]) );
acado_multEDu( &(acadoWorkspace.E[ 208 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 20 ]) );
acado_multEDu( &(acadoWorkspace.E[ 212 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 20 ]) );
acado_multEDu( &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 20 ]) );
}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 10; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 2];
acadoWorkspace.state[1] = acadoVariables.x[index * 2 + 1];
acadoWorkspace.state[10] = acadoVariables.u[index * 2];
acadoWorkspace.state[11] = acadoVariables.u[index * 2 + 1];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 2 + 2] = acadoWorkspace.state[0];
acadoVariables.x[index * 2 + 3] = acadoWorkspace.state[1];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 10; ++index)
{
acadoVariables.x[index * 2] = acadoVariables.x[index * 2 + 2];
acadoVariables.x[index * 2 + 1] = acadoVariables.x[index * 2 + 3];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[20] = xEnd[0];
acadoVariables.x[21] = xEnd[1];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[20];
acadoWorkspace.state[1] = acadoVariables.x[21];
if (uEnd != 0)
{
acadoWorkspace.state[10] = uEnd[0];
acadoWorkspace.state[11] = uEnd[1];
}
else
{
acadoWorkspace.state[10] = acadoVariables.u[18];
acadoWorkspace.state[11] = acadoVariables.u[19];
}

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[20] = acadoWorkspace.state[0];
acadoVariables.x[21] = acadoWorkspace.state[1];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 9; ++index)
{
acadoVariables.u[index * 2] = acadoVariables.u[index * 2 + 2];
acadoVariables.u[index * 2 + 1] = acadoVariables.u[index * 2 + 3];
}

if (uEnd != 0)
{
acadoVariables.u[18] = uEnd[0];
acadoVariables.u[19] = uEnd[1];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19];
kkt = fabs( kkt );
for (index = 0; index < 20; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 120; ++index)
{
prd = acadoWorkspace.y[index + 20];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lbA[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ubA[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 4 */
real_t tmpDy[ 4 ];

/** Row vector of size: 2 */
real_t tmpDyN[ 2 ];

for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 2];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 2 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.objValueIn[3] = acadoVariables.u[lRun1 * 2 + 1];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 4] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 4];
acadoWorkspace.Dy[lRun1 * 4 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 4 + 1];
acadoWorkspace.Dy[lRun1 * 4 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 4 + 2];
acadoWorkspace.Dy[lRun1 * 4 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 4 + 3];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[20];
acadoWorkspace.objValueIn[1] = acadoVariables.x[21];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 4]*(real_t)3.0000000000000000e+00;
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 4 + 1];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 4 + 2]*(real_t)2.0000000000000000e+00;
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 4 + 3]*(real_t)2.0000000000000000e+00;
objVal += + acadoWorkspace.Dy[lRun1 * 4]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 4 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 4 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 4 + 3]*tmpDy[3];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*(real_t)1.0000000000000001e-01;
tmpDyN[1] = + acadoWorkspace.DyN[1]*(real_t)1.0000000000000001e-01;
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1];

objVal *= 0.5;
return objVal;
}

