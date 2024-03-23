//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: test_sim.cpp
//
// Code generated for Simulink model 'test_sim'.
//
// Model version                  : 1.11
// Simulink Coder version         : 23.2 (R2023b) 01-Aug-2023
// C/C++ source code generated on : Tue Feb  6 19:41:08 2024
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "test_sim.h"
#include "rtwtypes.h"
#include <math.h>
#include "test_sim_private.h"
#include <string.h>

extern "C"
{

#include "rt_nonfinite.h"

}

#include "rt_defines.h"
#include "test_sim_dt.h"

// Named constants for MATLAB Function: '<S28>/optimizer'
const int32_T test_sim_degrees = 13;

// Block signals (default storage)
B_test_sim_T test_sim_B;

// Continuous states
X_test_sim_T test_sim_X;

// Disabled State Vector
XDis_test_sim_T test_sim_XDis;

// Block states (default storage)
DW_test_sim_T test_sim_DW;

// Real-time model
RT_MODEL_test_sim_T test_sim_M_ = RT_MODEL_test_sim_T();
RT_MODEL_test_sim_T *const test_sim_M = &test_sim_M_;

// Forward declaration for local functions
static void test_sim_binary_expand_op(real_T in1[3], const int8_T in2_data[],
  const int32_T in2_size[2], const real_T in3_data[], const real_T in4[4]);
static real_T test_sim_norm(const real_T x[13]);
static real_T test_sim_maximum(const real_T x[13]);
static real_T test_sim_xnrm2(int32_T n, const real_T x[169], int32_T ix0);
static void test_sim_xgemv(int32_T b_m, int32_T n, const real_T b_A[169],
  int32_T ia0, const real_T x[169], int32_T ix0, real_T y[13]);
static void test_sim_xgerc(int32_T b_m, int32_T n, real_T alpha1, int32_T ix0,
  const real_T y[13], real_T b_A[169], int32_T ia0);
static void test_sim_KWIKfactor(const real_T b_Ac[624], const int32_T iC[48],
  int32_T nA, const real_T b_Linv[169], real_T D[169], real_T b_H[169], int32_T
  n, real_T RLinv[169], real_T *Status);
static void test_sim_DropConstraint(int32_T kDrop, boolean_T iA[48], int32_T *nA,
  int32_T iC[48]);
static void test_sim_qpkwik(const real_T b_Linv[169], const real_T b_Hinv[169],
  const real_T f[13], const real_T b_Ac[624], const real_T b[48], boolean_T iA
  [48], int32_T maxiter, real_T FeasTol, real_T x[13], real_T lambda[48],
  int32_T *status);
static void rate_scheduler(void);
int32_T div_nzp_s32_floor(int32_T numerator, int32_T denominator)
{
  uint32_T absDenominator;
  uint32_T absNumerator;
  uint32_T tempAbsQuotient;
  boolean_T quotientNeedsNegation;
  absNumerator = numerator < 0 ? ~static_cast<uint32_T>(numerator) + 1U :
    static_cast<uint32_T>(numerator);
  absDenominator = denominator < 0 ? ~static_cast<uint32_T>(denominator) + 1U :
    static_cast<uint32_T>(denominator);
  quotientNeedsNegation = ((numerator < 0) != (denominator < 0));
  tempAbsQuotient = absNumerator / absDenominator;
  if (quotientNeedsNegation) {
    absNumerator %= absDenominator;
    if (absNumerator > 0U) {
      tempAbsQuotient++;
    }
  }

  return quotientNeedsNegation ? -static_cast<int32_T>(tempAbsQuotient) :
    static_cast<int32_T>(tempAbsQuotient);
}

//
//         This function updates active task flag for each subrate.
//         The function is called at model base rate, hence the
//         generated code self-manages all its subrates.
//
static void rate_scheduler(void)
{
  // Compute which subrates run during the next base time step.  Subrates
  //  are an integer multiple of the base rate counter.  Therefore, the subtask
  //  counter is reset when it reaches its limit (zero means run).

  (test_sim_M->Timing.TaskCounters.TID[2])++;
  if ((test_sim_M->Timing.TaskCounters.TID[2]) > 2) {// Sample time: [0.1s, 0.0s] 
    test_sim_M->Timing.TaskCounters.TID[2] = 0;
  }
}

//
// This function updates continuous states using the ODE3 fixed-step
// solver algorithm
//
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  // Solver Matrices
  static const real_T rt_ODE3_A[3] = {
    1.0/2.0, 3.0/4.0, 1.0
  };

  static const real_T rt_ODE3_B[3][3] = {
    { 1.0/2.0, 0.0, 0.0 },

    { 0.0, 3.0/4.0, 0.0 },

    { 2.0/9.0, 1.0/3.0, 4.0/9.0 }
  };

  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE3_IntgData *id = static_cast<ODE3_IntgData *>(rtsiGetSolverData(si));
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T hB[3];
  int_T i;
  int_T nXc = 4;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  // Save the state values at time t in y, we'll use x as ynew.
  (void) memcpy(y, x,
                static_cast<uint_T>(nXc)*sizeof(real_T));

  // Assumes that rtsiSetT and ModelOutputs are up-to-date
  // f0 = f(t,y)
  rtsiSetdX(si, f0);
  test_sim_derivatives();

  // f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*));
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  test_sim_step();
  test_sim_derivatives();

  // f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*));
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  test_sim_step();
  test_sim_derivatives();

  // tnew = t + hA(3);
  // ynew = y + f*hB(:,3);
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE3_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, tnew);
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    int32_T tmp;
    int32_T tmp_0;
    if (u0 > 0.0) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (u1 > 0.0) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    y = atan2(static_cast<real_T>(tmp), static_cast<real_T>(tmp_0));
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

static void test_sim_binary_expand_op(real_T in1[3], const int8_T in2_data[],
  const int32_T in2_size[2], const real_T in3_data[], const real_T in4[4])
{
  int32_T i;
  int32_T in2_idx_0;

  // MATLAB Function: '<Root>/Conversion'
  in2_idx_0 = in2_size[1];
  for (i = 0; i < in2_idx_0; i++) {
    in1[in2_data[0]] = -in3_data[0] * 2.0 * rt_atan2d_snf(in4[1], in4[0]);
  }

  // End of MATLAB Function: '<Root>/Conversion'
}

real_T rt_roundd_snf(real_T u)
{
  real_T y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

// Function for MATLAB Function: '<S28>/optimizer'
static real_T test_sim_norm(const real_T x[13])
{
  real_T scale;
  real_T y;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  for (int32_T k = 0; k < 13; k++) {
    real_T absxk;
    absxk = fabs(x[k]);
    if (absxk > scale) {
      real_T t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      real_T t;
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrt(y);
}

// Function for MATLAB Function: '<S28>/optimizer'
static real_T test_sim_maximum(const real_T x[13])
{
  real_T ex;
  int32_T idx;
  int32_T k;
  if (!rtIsNaN(x[0])) {
    idx = 1;
  } else {
    boolean_T exitg1;
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 14)) {
      if (!rtIsNaN(x[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    ex = x[0];
  } else {
    ex = x[idx - 1];
    for (k = idx + 1; k < 14; k++) {
      real_T x_0;
      x_0 = x[k - 1];
      if (ex < x_0) {
        ex = x_0;
      }
    }
  }

  return ex;
}

// Function for MATLAB Function: '<S28>/optimizer'
static real_T test_sim_xnrm2(int32_T n, const real_T x[169], int32_T ix0)
{
  real_T y;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x[ix0 - 1]);
    } else {
      int32_T kend;
      test_sim_B.scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (int32_T k = ix0; k <= kend; k++) {
        test_sim_B.absxk = fabs(x[k - 1]);
        if (test_sim_B.absxk > test_sim_B.scale) {
          test_sim_B.t = test_sim_B.scale / test_sim_B.absxk;
          y = y * test_sim_B.t * test_sim_B.t + 1.0;
          test_sim_B.scale = test_sim_B.absxk;
        } else {
          test_sim_B.t = test_sim_B.absxk / test_sim_B.scale;
          y += test_sim_B.t * test_sim_B.t;
        }
      }

      y = test_sim_B.scale * sqrt(y);
    }
  }

  return y;
}

real_T rt_hypotd_snf(real_T u0, real_T u1)
{
  real_T a;
  real_T b;
  real_T y;
  a = fabs(u0);
  b = fabs(u1);
  if (a < b) {
    a /= b;
    y = sqrt(a * a + 1.0) * b;
  } else if (a > b) {
    b /= a;
    y = sqrt(b * b + 1.0) * a;
  } else if (rtIsNaN(b)) {
    y = (rtNaN);
  } else {
    y = a * 1.4142135623730951;
  }

  return y;
}

// Function for MATLAB Function: '<S28>/optimizer'
static void test_sim_xgemv(int32_T b_m, int32_T n, const real_T b_A[169],
  int32_T ia0, const real_T x[169], int32_T ix0, real_T y[13])
{
  if ((b_m != 0) && (n != 0)) {
    int32_T b;
    if (n - 1 >= 0) {
      memset(&y[0], 0, static_cast<uint32_T>(n) * sizeof(real_T));
    }

    b = (n - 1) * 13 + ia0;
    for (int32_T b_iy = ia0; b_iy <= b; b_iy += 13) {
      int32_T d;
      int32_T ia;
      test_sim_B.c = 0.0;
      d = (b_iy + b_m) - 1;
      for (ia = b_iy; ia <= d; ia++) {
        test_sim_B.c += x[((ix0 + ia) - b_iy) - 1] * b_A[ia - 1];
      }

      ia = div_nzp_s32_floor(b_iy - ia0, 13);
      y[ia] += test_sim_B.c;
    }
  }
}

// Function for MATLAB Function: '<S28>/optimizer'
static void test_sim_xgerc(int32_T b_m, int32_T n, real_T alpha1, int32_T ix0,
  const real_T y[13], real_T b_A[169], int32_T ia0)
{
  if (!(alpha1 == 0.0)) {
    int32_T jA;
    jA = ia0;
    for (int32_T j = 0; j < n; j++) {
      test_sim_B.temp = y[j];
      if (test_sim_B.temp != 0.0) {
        int32_T b;
        test_sim_B.temp *= alpha1;
        b = b_m + jA;
        for (int32_T ijA = jA; ijA < b; ijA++) {
          b_A[ijA - 1] += b_A[((ix0 + ijA) - jA) - 1] * test_sim_B.temp;
        }
      }

      jA += 13;
    }
  }
}

// Function for MATLAB Function: '<S28>/optimizer'
static void test_sim_KWIKfactor(const real_T b_Ac[624], const int32_T iC[48],
  int32_T nA, const real_T b_Linv[169], real_T D[169], real_T b_H[169], int32_T
  n, real_T RLinv[169], real_T *Status)
{
  int32_T b_coltop;
  int32_T b_lastv;
  int32_T exitg1;
  int32_T f_tmp;
  int32_T i;
  int32_T ii;
  int32_T knt;
  boolean_T exitg2;
  *Status = 1.0;
  memset(&RLinv[0], 0, 169U * sizeof(real_T));
  for (b_lastv = 0; b_lastv < nA; b_lastv++) {
    knt = iC[b_lastv];
    for (i = 0; i < 13; i++) {
      test_sim_B.beta1 = 0.0;
      for (ii = 0; ii < 13; ii++) {
        test_sim_B.beta1 += b_Ac[(48 * ii + knt) - 1] * b_Linv[13 * ii + i];
      }

      RLinv[i + 13 * b_lastv] = test_sim_B.beta1;
    }
  }

  memcpy(&test_sim_B.b_A[0], &RLinv[0], 169U * sizeof(real_T));
  memset(&test_sim_B.tau[0], 0, 13U * sizeof(real_T));
  memset(&test_sim_B.work[0], 0, 13U * sizeof(real_T));
  for (i = 0; i < 13; i++) {
    ii = i * 13 + i;
    if (i + 1 < 13) {
      test_sim_B.atmp = test_sim_B.b_A[ii];
      b_lastv = ii + 2;
      test_sim_B.tau[i] = 0.0;
      test_sim_B.beta1 = test_sim_xnrm2(12 - i, test_sim_B.b_A, ii + 2);
      if (test_sim_B.beta1 != 0.0) {
        test_sim_B.b_A_p = test_sim_B.b_A[ii];
        test_sim_B.beta1 = rt_hypotd_snf(test_sim_B.b_A_p, test_sim_B.beta1);
        if (test_sim_B.b_A_p >= 0.0) {
          test_sim_B.beta1 = -test_sim_B.beta1;
        }

        if (fabs(test_sim_B.beta1) < 1.0020841800044864E-292) {
          knt = 0;
          f_tmp = (ii - i) + 13;
          do {
            knt++;
            for (b_coltop = b_lastv; b_coltop <= f_tmp; b_coltop++) {
              test_sim_B.b_A[b_coltop - 1] *= 9.9792015476736E+291;
            }

            test_sim_B.beta1 *= 9.9792015476736E+291;
            test_sim_B.atmp *= 9.9792015476736E+291;
          } while ((fabs(test_sim_B.beta1) < 1.0020841800044864E-292) && (knt <
                    20));

          test_sim_B.beta1 = rt_hypotd_snf(test_sim_B.atmp, test_sim_xnrm2(12 -
            i, test_sim_B.b_A, ii + 2));
          if (test_sim_B.atmp >= 0.0) {
            test_sim_B.beta1 = -test_sim_B.beta1;
          }

          test_sim_B.tau[i] = (test_sim_B.beta1 - test_sim_B.atmp) /
            test_sim_B.beta1;
          test_sim_B.atmp = 1.0 / (test_sim_B.atmp - test_sim_B.beta1);
          for (b_coltop = b_lastv; b_coltop <= f_tmp; b_coltop++) {
            test_sim_B.b_A[b_coltop - 1] *= test_sim_B.atmp;
          }

          for (b_lastv = 0; b_lastv < knt; b_lastv++) {
            test_sim_B.beta1 *= 1.0020841800044864E-292;
          }

          test_sim_B.atmp = test_sim_B.beta1;
        } else {
          test_sim_B.tau[i] = (test_sim_B.beta1 - test_sim_B.b_A_p) /
            test_sim_B.beta1;
          test_sim_B.atmp = 1.0 / (test_sim_B.b_A_p - test_sim_B.beta1);
          b_coltop = (ii - i) + 13;
          for (knt = b_lastv; knt <= b_coltop; knt++) {
            test_sim_B.b_A[knt - 1] *= test_sim_B.atmp;
          }

          test_sim_B.atmp = test_sim_B.beta1;
        }
      }

      test_sim_B.b_A[ii] = 1.0;
      if (test_sim_B.tau[i] != 0.0) {
        b_lastv = 13 - i;
        knt = (ii - i) + 12;
        while ((b_lastv > 0) && (test_sim_B.b_A[knt] == 0.0)) {
          b_lastv--;
          knt--;
        }

        knt = 12 - i;
        exitg2 = false;
        while ((!exitg2) && (knt > 0)) {
          b_coltop = ((knt - 1) * 13 + ii) + 13;
          f_tmp = b_coltop;
          do {
            exitg1 = 0;
            if (f_tmp + 1 <= b_coltop + b_lastv) {
              if (test_sim_B.b_A[f_tmp] != 0.0) {
                exitg1 = 1;
              } else {
                f_tmp++;
              }
            } else {
              knt--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        b_lastv = 0;
        knt = 0;
      }

      if (b_lastv > 0) {
        test_sim_xgemv(b_lastv, knt, test_sim_B.b_A, ii + 14, test_sim_B.b_A, ii
                       + 1, test_sim_B.work);
        test_sim_xgerc(b_lastv, knt, -test_sim_B.tau[i], ii + 1, test_sim_B.work,
                       test_sim_B.b_A, ii + 14);
      }

      test_sim_B.b_A[ii] = test_sim_B.atmp;
    } else {
      test_sim_B.tau[12] = 0.0;
    }
  }

  for (i = 0; i < 13; i++) {
    for (ii = 0; ii <= i; ii++) {
      test_sim_B.R[ii + 13 * i] = test_sim_B.b_A[13 * i + ii];
    }

    for (ii = i + 2; ii < 14; ii++) {
      test_sim_B.R[(ii + 13 * i) - 1] = 0.0;
    }

    test_sim_B.work[i] = 0.0;
  }

  for (i = 12; i >= 0; i--) {
    ii = (i * 13 + i) + 14;
    if (i + 1 < 13) {
      test_sim_B.b_A[ii - 14] = 1.0;
      if (test_sim_B.tau[i] != 0.0) {
        b_lastv = 13 - i;
        knt = ii - i;
        while ((b_lastv > 0) && (test_sim_B.b_A[knt - 2] == 0.0)) {
          b_lastv--;
          knt--;
        }

        knt = 12 - i;
        exitg2 = false;
        while ((!exitg2) && (knt > 0)) {
          b_coltop = (knt - 1) * 13 + ii;
          f_tmp = b_coltop;
          do {
            exitg1 = 0;
            if (f_tmp <= (b_coltop + b_lastv) - 1) {
              if (test_sim_B.b_A[f_tmp - 1] != 0.0) {
                exitg1 = 1;
              } else {
                f_tmp++;
              }
            } else {
              knt--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        b_lastv = 0;
        knt = 0;
      }

      if (b_lastv > 0) {
        test_sim_xgemv(b_lastv, knt, test_sim_B.b_A, ii, test_sim_B.b_A, ii - 13,
                       test_sim_B.work);
        test_sim_xgerc(b_lastv, knt, -test_sim_B.tau[i], ii - 13,
                       test_sim_B.work, test_sim_B.b_A, ii);
      }

      knt = (ii - i) - 1;
      for (b_lastv = ii - 12; b_lastv <= knt; b_lastv++) {
        test_sim_B.b_A[b_lastv - 1] *= -test_sim_B.tau[i];
      }
    }

    test_sim_B.b_A[ii - 14] = 1.0 - test_sim_B.tau[i];
    for (b_lastv = 0; b_lastv < i; b_lastv++) {
      test_sim_B.b_A[(ii - b_lastv) - 15] = 0.0;
    }
  }

  i = 0;
  do {
    exitg1 = 0;
    if (i <= nA - 1) {
      if (fabs(test_sim_B.R[13 * i + i]) < 1.0E-12) {
        *Status = -2.0;
        exitg1 = 1;
      } else {
        i++;
      }
    } else {
      for (ii = 0; ii < n; ii++) {
        for (b_lastv = 0; b_lastv < n; b_lastv++) {
          test_sim_B.beta1 = 0.0;
          for (i = 0; i < 13; i++) {
            test_sim_B.beta1 += b_Linv[13 * ii + i] * test_sim_B.b_A[13 *
              b_lastv + i];
          }

          test_sim_B.TL[ii + 13 * b_lastv] = test_sim_B.beta1;
        }
      }

      memset(&RLinv[0], 0, 169U * sizeof(real_T));
      for (i = nA; i >= 1; i--) {
        knt = (i - 1) * 13;
        b_coltop = (i + knt) - 1;
        RLinv[b_coltop] = 1.0;
        for (ii = i; ii <= nA; ii++) {
          f_tmp = ((ii - 1) * 13 + i) - 1;
          RLinv[f_tmp] /= test_sim_B.R[b_coltop];
        }

        if (i > 1) {
          for (ii = 0; ii <= i - 2; ii++) {
            for (b_lastv = i; b_lastv <= nA; b_lastv++) {
              b_coltop = (b_lastv - 1) * 13;
              f_tmp = b_coltop + ii;
              RLinv[f_tmp] -= RLinv[(b_coltop + i) - 1] * test_sim_B.R[knt + ii];
            }
          }
        }
      }

      for (b_lastv = 0; b_lastv < n; b_lastv++) {
        for (knt = b_lastv + 1; knt <= n; knt++) {
          i = (knt - 1) * 13 + b_lastv;
          b_H[i] = 0.0;
          for (b_coltop = nA + 1; b_coltop <= n; b_coltop++) {
            ii = (b_coltop - 1) * 13;
            b_H[i] -= test_sim_B.TL[(ii + knt) - 1] * test_sim_B.TL[ii + b_lastv];
          }

          b_H[(knt + 13 * b_lastv) - 1] = b_H[i];
        }
      }

      for (b_lastv = 0; b_lastv < nA; b_lastv++) {
        for (knt = 0; knt < n; knt++) {
          i = 13 * b_lastv + knt;
          D[i] = 0.0;
          for (b_coltop = b_lastv + 1; b_coltop <= nA; b_coltop++) {
            ii = (b_coltop - 1) * 13;
            D[i] += test_sim_B.TL[ii + knt] * RLinv[ii + b_lastv];
          }
        }
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);
}

// Function for MATLAB Function: '<S28>/optimizer'
static void test_sim_DropConstraint(int32_T kDrop, boolean_T iA[48], int32_T *nA,
  int32_T iC[48])
{
  if (kDrop > 0) {
    iA[iC[kDrop - 1] - 1] = false;
    if (kDrop < *nA) {
      int32_T b;
      b = *nA - 1;
      for (int32_T i = kDrop; i <= b; i++) {
        iC[i - 1] = iC[i];
      }
    }

    iC[*nA - 1] = 0;
    (*nA)--;
  }
}

// Function for MATLAB Function: '<S28>/optimizer'
static void test_sim_qpkwik(const real_T b_Linv[169], const real_T b_Hinv[169],
  const real_T f[13], const real_T b_Ac[624], const real_T b[48], boolean_T iA
  [48], int32_T maxiter, real_T FeasTol, real_T x[13], real_T lambda[48],
  int32_T *status)
{
  int32_T U_tmp;
  int32_T b_exponent;
  int32_T exitg1;
  int32_T exitg3;
  int32_T exponent;
  int32_T i;
  boolean_T ColdReset;
  boolean_T DualFeasible;
  boolean_T cTolComputed;
  boolean_T exitg2;
  boolean_T exitg4;
  boolean_T guard1;
  boolean_T guard2;
  memset(&x[0], 0, 13U * sizeof(real_T));
  memset(&lambda[0], 0, 48U * sizeof(real_T));
  *status = 1;
  memset(&test_sim_B.r[0], 0, 13U * sizeof(real_T));
  test_sim_B.rMin = 0.0;
  cTolComputed = false;
  for (i = 0; i < 48; i++) {
    test_sim_B.cTol[i] = 1.0;
    test_sim_B.iC[i] = 0;
  }

  test_sim_B.nA = 0;
  for (i = 0; i < 48; i++) {
    if (iA[i]) {
      test_sim_B.nA++;
      test_sim_B.iC[test_sim_B.nA - 1] = i + 1;
    }
  }

  guard1 = false;
  if (test_sim_B.nA > 0) {
    memset(&test_sim_B.Opt[0], 0, 26U * sizeof(real_T));
    for (i = 0; i < 13; i++) {
      test_sim_B.Rhs[i] = f[i];
      test_sim_B.Rhs[i + 13] = 0.0;
    }

    DualFeasible = false;
    test_sim_B.tmp = static_cast<int32_T>(rt_roundd_snf(0.3 * static_cast<real_T>
      (test_sim_B.nA)));
    ColdReset = false;
    do {
      exitg3 = 0;
      if ((!DualFeasible) && (test_sim_B.nA > 0) && (*status <= maxiter)) {
        test_sim_KWIKfactor(b_Ac, test_sim_B.iC, test_sim_B.nA, b_Linv,
                            test_sim_B.D, test_sim_B.b_H, test_sim_degrees,
                            test_sim_B.RLinv, &test_sim_B.Xnorm0);
        if (test_sim_B.Xnorm0 < 0.0) {
          if (ColdReset) {
            *status = -2;
            exitg3 = 2;
          } else {
            test_sim_B.nA = 0;
            memset(&test_sim_B.iC[0], 0, 48U * sizeof(int32_T));
            for (i = 0; i < 48; i++) {
              iA[i] = false;
            }

            ColdReset = true;
          }
        } else {
          for (i = 0; i < test_sim_B.nA; i++) {
            test_sim_B.Rhs[i + 13] = b[test_sim_B.iC[i] - 1];
            for (test_sim_B.kDrop = i + 1; test_sim_B.kDrop <= test_sim_B.nA;
                 test_sim_B.kDrop++) {
              U_tmp = (13 * i + test_sim_B.kDrop) - 1;
              test_sim_B.U[U_tmp] = 0.0;
              for (test_sim_B.iSave = 0; test_sim_B.iSave < test_sim_B.nA;
                   test_sim_B.iSave++) {
                test_sim_B.U[U_tmp] += test_sim_B.RLinv[(13 * test_sim_B.iSave +
                  test_sim_B.kDrop) - 1] * test_sim_B.RLinv[13 *
                  test_sim_B.iSave + i];
              }

              test_sim_B.U[i + 13 * (test_sim_B.kDrop - 1)] = test_sim_B.U[U_tmp];
            }
          }

          for (test_sim_B.kDrop = 0; test_sim_B.kDrop < 13; test_sim_B.kDrop++)
          {
            test_sim_B.Xnorm0 = 0.0;
            for (i = 0; i < 13; i++) {
              test_sim_B.Xnorm0 += test_sim_B.b_H[13 * i + test_sim_B.kDrop] *
                test_sim_B.Rhs[i];
            }

            test_sim_B.Opt[test_sim_B.kDrop] = test_sim_B.Xnorm0;
            for (i = 0; i < test_sim_B.nA; i++) {
              test_sim_B.Opt[test_sim_B.kDrop] += test_sim_B.D[13 * i +
                test_sim_B.kDrop] * test_sim_B.Rhs[i + 13];
            }
          }

          test_sim_B.Xnorm0 = -1.0E-12;
          test_sim_B.kDrop = -1;
          for (test_sim_B.iSave = 0; test_sim_B.iSave < test_sim_B.nA;
               test_sim_B.iSave++) {
            test_sim_B.cMin = 0.0;
            for (i = 0; i < 13; i++) {
              test_sim_B.cMin += test_sim_B.D[13 * test_sim_B.iSave + i] *
                test_sim_B.Rhs[i];
            }

            test_sim_B.Opt[test_sim_B.iSave + 13] = test_sim_B.cMin;
            for (i = 0; i < test_sim_B.nA; i++) {
              test_sim_B.Opt[test_sim_B.iSave + 13] += test_sim_B.U[13 * i +
                test_sim_B.iSave] * test_sim_B.Rhs[i + 13];
            }

            test_sim_B.cMin = test_sim_B.Opt[test_sim_B.iSave + 13];
            lambda[test_sim_B.iC[test_sim_B.iSave] - 1] = test_sim_B.cMin;
            if ((test_sim_B.cMin < test_sim_B.Xnorm0) && (test_sim_B.iSave + 1 <=
                 test_sim_B.nA)) {
              test_sim_B.kDrop = test_sim_B.iSave;
              test_sim_B.Xnorm0 = test_sim_B.cMin;
            }
          }

          if (test_sim_B.kDrop + 1 <= 0) {
            DualFeasible = true;
            memcpy(&x[0], &test_sim_B.Opt[0], 13U * sizeof(real_T));
          } else {
            (*status)++;
            if (test_sim_B.tmp <= 5) {
              i = 5;
            } else {
              i = test_sim_B.tmp;
            }

            if (*status > i) {
              test_sim_B.nA = 0;
              memset(&test_sim_B.iC[0], 0, 48U * sizeof(int32_T));
              for (i = 0; i < 48; i++) {
                iA[i] = false;
              }

              ColdReset = true;
            } else {
              lambda[test_sim_B.iC[test_sim_B.kDrop] - 1] = 0.0;
              test_sim_DropConstraint(test_sim_B.kDrop + 1, iA, &test_sim_B.nA,
                test_sim_B.iC);
            }
          }
        }
      } else {
        if (test_sim_B.nA <= 0) {
          memset(&lambda[0], 0, 48U * sizeof(real_T));
          for (test_sim_B.tmp = 0; test_sim_B.tmp < 13; test_sim_B.tmp++) {
            test_sim_B.Xnorm0 = 0.0;
            for (i = 0; i < 13; i++) {
              test_sim_B.Xnorm0 += -b_Hinv[13 * i + test_sim_B.tmp] * f[i];
            }

            x[test_sim_B.tmp] = test_sim_B.Xnorm0;
          }
        }

        exitg3 = 1;
      }
    } while (exitg3 == 0);

    if (exitg3 == 1) {
      guard1 = true;
    }
  } else {
    for (test_sim_B.tmp = 0; test_sim_B.tmp < 13; test_sim_B.tmp++) {
      test_sim_B.Xnorm0 = 0.0;
      for (i = 0; i < 13; i++) {
        test_sim_B.Xnorm0 += -b_Hinv[13 * i + test_sim_B.tmp] * f[i];
      }

      x[test_sim_B.tmp] = test_sim_B.Xnorm0;
    }

    guard1 = true;
  }

  if (guard1) {
    test_sim_B.Xnorm0 = test_sim_norm(x);
    exitg2 = false;
    while ((!exitg2) && (*status <= maxiter)) {
      test_sim_B.cMin = -FeasTol;
      test_sim_B.tmp = -1;
      for (test_sim_B.kDrop = 0; test_sim_B.kDrop < 48; test_sim_B.kDrop++) {
        if (!cTolComputed) {
          for (i = 0; i < 13; i++) {
            test_sim_B.z[i] = fabs(b_Ac[48 * i + test_sim_B.kDrop] * x[i]);
          }

          test_sim_B.cVal = test_sim_maximum(test_sim_B.z);
          if ((test_sim_B.cTol[test_sim_B.kDrop] >= test_sim_B.cVal) || rtIsNaN
              (test_sim_B.cVal)) {
          } else {
            test_sim_B.cTol[test_sim_B.kDrop] = test_sim_B.cVal;
          }
        }

        if (!iA[test_sim_B.kDrop]) {
          test_sim_B.cVal = 0.0;
          for (i = 0; i < 13; i++) {
            test_sim_B.cVal += b_Ac[48 * i + test_sim_B.kDrop] * x[i];
          }

          test_sim_B.cVal = (test_sim_B.cVal - b[test_sim_B.kDrop]) /
            test_sim_B.cTol[test_sim_B.kDrop];
          if (test_sim_B.cVal < test_sim_B.cMin) {
            test_sim_B.cMin = test_sim_B.cVal;
            test_sim_B.tmp = test_sim_B.kDrop;
          }
        }
      }

      cTolComputed = true;
      if (test_sim_B.tmp + 1 <= 0) {
        exitg2 = true;
      } else if (*status == maxiter) {
        *status = 0;
        exitg2 = true;
      } else {
        do {
          exitg1 = 0;
          if ((test_sim_B.tmp + 1 > 0) && (*status <= maxiter)) {
            guard2 = false;
            if (test_sim_B.nA == 0) {
              for (i = 0; i < 13; i++) {
                test_sim_B.cMin = 0.0;
                for (test_sim_B.kDrop = 0; test_sim_B.kDrop < 13;
                     test_sim_B.kDrop++) {
                  test_sim_B.cMin += b_Hinv[13 * test_sim_B.kDrop + i] * b_Ac[48
                    * test_sim_B.kDrop + test_sim_B.tmp];
                }

                test_sim_B.z[i] = test_sim_B.cMin;
              }

              guard2 = true;
            } else {
              test_sim_KWIKfactor(b_Ac, test_sim_B.iC, test_sim_B.nA, b_Linv,
                                  test_sim_B.D, test_sim_B.b_H, test_sim_degrees,
                                  test_sim_B.RLinv, &test_sim_B.cMin);
              if (test_sim_B.cMin <= 0.0) {
                *status = -2;
                exitg1 = 1;
              } else {
                for (i = 0; i < 169; i++) {
                  test_sim_B.U[i] = -test_sim_B.b_H[i];
                }

                for (i = 0; i < 13; i++) {
                  test_sim_B.cMin = 0.0;
                  for (test_sim_B.kDrop = 0; test_sim_B.kDrop < 13;
                       test_sim_B.kDrop++) {
                    test_sim_B.cMin += test_sim_B.U[13 * test_sim_B.kDrop + i] *
                      b_Ac[48 * test_sim_B.kDrop + test_sim_B.tmp];
                  }

                  test_sim_B.z[i] = test_sim_B.cMin;
                }

                for (test_sim_B.kDrop = 0; test_sim_B.kDrop < test_sim_B.nA;
                     test_sim_B.kDrop++) {
                  test_sim_B.cVal = 0.0;
                  for (i = 0; i < 13; i++) {
                    test_sim_B.cVal += b_Ac[48 * i + test_sim_B.tmp] *
                      test_sim_B.D[13 * test_sim_B.kDrop + i];
                  }

                  test_sim_B.r[test_sim_B.kDrop] = test_sim_B.cVal;
                }

                guard2 = true;
              }
            }

            if (guard2) {
              test_sim_B.kDrop = 0;
              test_sim_B.cMin = 0.0;
              DualFeasible = true;
              ColdReset = true;
              if (test_sim_B.nA > 0) {
                i = 0;
                exitg4 = false;
                while ((!exitg4) && (i <= test_sim_B.nA - 1)) {
                  if (test_sim_B.r[i] >= 1.0E-12) {
                    ColdReset = false;
                    exitg4 = true;
                  } else {
                    i++;
                  }
                }
              }

              if ((test_sim_B.nA != 0) && (!ColdReset)) {
                for (i = 0; i < test_sim_B.nA; i++) {
                  test_sim_B.cVal = test_sim_B.r[i];
                  if (test_sim_B.cVal > 1.0E-12) {
                    test_sim_B.cVal = lambda[test_sim_B.iC[i] - 1] /
                      test_sim_B.cVal;
                    if ((test_sim_B.kDrop == 0) || (test_sim_B.cVal <
                         test_sim_B.rMin)) {
                      test_sim_B.rMin = test_sim_B.cVal;
                      test_sim_B.kDrop = i + 1;
                    }
                  }
                }

                if (test_sim_B.kDrop > 0) {
                  test_sim_B.cMin = test_sim_B.rMin;
                  DualFeasible = false;
                }
              }

              test_sim_B.zTa = 0.0;
              for (i = 0; i < 13; i++) {
                test_sim_B.zTa += b_Ac[48 * i + test_sim_B.tmp] * test_sim_B.z[i];
              }

              if (test_sim_B.zTa <= 0.0) {
                test_sim_B.cVal = 0.0;
                ColdReset = true;
              } else {
                test_sim_B.cVal = 0.0;
                for (i = 0; i < 13; i++) {
                  test_sim_B.cVal += b_Ac[48 * i + test_sim_B.tmp] * x[i];
                }

                test_sim_B.cVal = (b[test_sim_B.tmp] - test_sim_B.cVal) /
                  test_sim_B.zTa;
                ColdReset = false;
              }

              if (DualFeasible && ColdReset) {
                *status = -1;
                exitg1 = 1;
              } else {
                if (ColdReset) {
                  test_sim_B.zTa = test_sim_B.cMin;
                } else if (DualFeasible) {
                  test_sim_B.zTa = test_sim_B.cVal;
                } else if (test_sim_B.cMin < test_sim_B.cVal) {
                  test_sim_B.zTa = test_sim_B.cMin;
                } else {
                  test_sim_B.zTa = test_sim_B.cVal;
                }

                for (i = 0; i < test_sim_B.nA; i++) {
                  test_sim_B.iSave = test_sim_B.iC[i];
                  lambda[test_sim_B.iSave - 1] -= test_sim_B.zTa *
                    test_sim_B.r[i];
                  if ((test_sim_B.iSave <= 48) && (lambda[test_sim_B.iSave - 1] <
                       0.0)) {
                    lambda[test_sim_B.iSave - 1] = 0.0;
                  }
                }

                lambda[test_sim_B.tmp] += test_sim_B.zTa;
                frexp(1.0, &exponent);
                if (fabs(test_sim_B.zTa - test_sim_B.cMin) <
                    2.2204460492503131E-16) {
                  test_sim_DropConstraint(test_sim_B.kDrop, iA, &test_sim_B.nA,
                    test_sim_B.iC);
                }

                if (!ColdReset) {
                  for (i = 0; i < 13; i++) {
                    x[i] += test_sim_B.zTa * test_sim_B.z[i];
                  }

                  frexp(1.0, &b_exponent);
                  if (fabs(test_sim_B.zTa - test_sim_B.cVal) <
                      2.2204460492503131E-16) {
                    if (test_sim_B.nA == test_sim_degrees) {
                      *status = -1;
                      exitg1 = 1;
                    } else {
                      test_sim_B.nA++;
                      test_sim_B.iC[test_sim_B.nA - 1] = test_sim_B.tmp + 1;
                      test_sim_B.kDrop = test_sim_B.nA - 1;
                      exitg4 = false;
                      while ((!exitg4) && (test_sim_B.kDrop + 1 > 1)) {
                        i = test_sim_B.iC[test_sim_B.kDrop - 1];
                        if (test_sim_B.iC[test_sim_B.kDrop] > i) {
                          exitg4 = true;
                        } else {
                          test_sim_B.iSave = test_sim_B.iC[test_sim_B.kDrop];
                          test_sim_B.iC[test_sim_B.kDrop] = i;
                          test_sim_B.iC[test_sim_B.kDrop - 1] = test_sim_B.iSave;
                          test_sim_B.kDrop--;
                        }
                      }

                      iA[test_sim_B.tmp] = true;
                      test_sim_B.tmp = -1;
                      (*status)++;
                    }
                  } else {
                    (*status)++;
                  }
                } else {
                  (*status)++;
                }
              }
            }
          } else {
            test_sim_B.cMin = test_sim_norm(x);
            if (fabs(test_sim_B.cMin - test_sim_B.Xnorm0) > 0.001) {
              test_sim_B.Xnorm0 = test_sim_B.cMin;
              for (i = 0; i < 48; i++) {
                test_sim_B.cMin = fabs(b[i]);
                if (test_sim_B.cMin >= 1.0) {
                  test_sim_B.cTol[i] = test_sim_B.cMin;
                } else {
                  test_sim_B.cTol[i] = 1.0;
                }
              }

              cTolComputed = false;
            }

            exitg1 = 2;
          }
        } while (exitg1 == 0);

        if (exitg1 == 1) {
          exitg2 = true;
        }
      }
    }
  }
}

// Model step function
void test_sim_step(void)
{
  if (rtmIsMajorTimeStep(test_sim_M)) {
    // set solver stop time
    rtsiSetSolverStopTime(&test_sim_M->solverInfo,
                          ((test_sim_M->Timing.clockTick0+1)*
      test_sim_M->Timing.stepSize0));
  }                                    // end MajorTimeStep

  // Update absolute time of base rate at minor time step
  if (rtmIsMinorTimeStep(test_sim_M)) {
    test_sim_M->Timing.t[0] = rtsiGetT(&test_sim_M->solverInfo);
  }

  {
    int8_T tmp_data;
    boolean_T mask;
    static const int8_T a[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1
    };

    static const real_T b_a[16] = { 0.095124921972503731, 0.0, 0.0, 0.0, 0.0,
      0.095124921972503731, 0.0, 0.0, 0.0, 0.0, 0.095124921972503731, 0.0, 0.0,
      0.0, 0.0, 0.095124921972503731 };

    static const real_T b_Kr[960] = { -0.1, -0.0, -0.0, -0.0, -0.2, -0.0, -0.0,
      -0.0, -0.30000000000000004, -0.0, -0.0, -0.0, -0.4, -0.0, -0.0, -0.0, -0.5,
      -0.0, -0.0, -0.0, -0.6, -0.0, -0.0, -0.0, -0.7, -0.0, -0.0, -0.0,
      -0.79999999999999993, -0.0, -0.0, -0.0, -0.89999999999999991, -0.0, -0.0,
      -0.0, -0.99999999999999989, -0.0, -0.0, -0.0, -1.0999999999999999, -0.0,
      -0.0, -0.0, -1.2, -0.0, -0.0, -0.0, -1.3, -0.0, -0.0, -0.0,
      -1.4000000000000001, -0.0, -0.0, -0.0, -1.5000000000000002, -0.0, -0.0,
      -0.0, -1.6000000000000003, -0.0, -0.0, -0.0, -1.7000000000000004, -0.0,
      -0.0, -0.0, -1.8000000000000005, -0.0, -0.0, -0.0, -1.9000000000000006,
      -0.0, -0.0, -0.0, -2.0000000000000004, -0.0, -0.0, -0.0, -0.0, -0.1, -0.0,
      -0.0, -0.0, -0.2, -0.0, -0.0, -0.0, -0.30000000000000004, -0.0, -0.0, -0.0,
      -0.4, -0.0, -0.0, -0.0, -0.5, -0.0, -0.0, -0.0, -0.6, -0.0, -0.0, -0.0,
      -0.7, -0.0, -0.0, -0.0, -0.79999999999999993, -0.0, -0.0, -0.0,
      -0.89999999999999991, -0.0, -0.0, -0.0, -0.99999999999999989, -0.0, -0.0,
      -0.0, -1.0999999999999999, -0.0, -0.0, -0.0, -1.2, -0.0, -0.0, -0.0, -1.3,
      -0.0, -0.0, -0.0, -1.4000000000000001, -0.0, -0.0, -0.0,
      -1.5000000000000002, -0.0, -0.0, -0.0, -1.6000000000000003, -0.0, -0.0,
      -0.0, -1.7000000000000004, -0.0, -0.0, -0.0, -1.8000000000000005, -0.0,
      -0.0, -0.0, -1.9000000000000006, -0.0, -0.0, -0.0, -2.0000000000000004,
      -0.0, -0.0, -0.0, -0.0, -0.1, -0.0, -0.0, -0.0, -0.2, -0.0, -0.0, -0.0,
      -0.30000000000000004, -0.0, -0.0, -0.0, -0.4, -0.0, -0.0, -0.0, -0.5, -0.0,
      -0.0, -0.0, -0.6, -0.0, -0.0, -0.0, -0.7, -0.0, -0.0, -0.0,
      -0.79999999999999993, -0.0, -0.0, -0.0, -0.89999999999999991, -0.0, -0.0,
      -0.0, -0.99999999999999989, -0.0, -0.0, -0.0, -1.0999999999999999, -0.0,
      -0.0, -0.0, -1.2, -0.0, -0.0, -0.0, -1.3, -0.0, -0.0, -0.0,
      -1.4000000000000001, -0.0, -0.0, -0.0, -1.5000000000000002, -0.0, -0.0,
      -0.0, -1.6000000000000003, -0.0, -0.0, -0.0, -1.7000000000000004, -0.0,
      -0.0, -0.0, -1.8000000000000005, -0.0, -0.0, -0.0, -1.9000000000000006,
      -0.0, -0.0, -0.0, -2.0000000000000004, -0.0, -0.0, -0.0, -0.0, -0.1, -0.0,
      -0.0, -0.0, -0.2, -0.0, -0.0, -0.0, -0.30000000000000004, -0.0, -0.0, -0.0,
      -0.4, -0.0, -0.0, -0.0, -0.5, -0.0, -0.0, -0.0, -0.6, -0.0, -0.0, -0.0,
      -0.7, -0.0, -0.0, -0.0, -0.79999999999999993, -0.0, -0.0, -0.0,
      -0.89999999999999991, -0.0, -0.0, -0.0, -0.99999999999999989, -0.0, -0.0,
      -0.0, -1.0999999999999999, -0.0, -0.0, -0.0, -1.2, -0.0, -0.0, -0.0, -1.3,
      -0.0, -0.0, -0.0, -1.4000000000000001, -0.0, -0.0, -0.0,
      -1.5000000000000002, -0.0, -0.0, -0.0, -1.6000000000000003, -0.0, -0.0,
      -0.0, -1.7000000000000004, -0.0, -0.0, -0.0, -1.8000000000000005, -0.0,
      -0.0, -0.0, -1.9000000000000006, -0.0, -0.0, -0.0, -2.0000000000000004,
      -0.0, -0.0, -0.0, -0.0, -0.1, -0.0, -0.0, -0.0, -0.2, -0.0, -0.0, -0.0,
      -0.30000000000000004, -0.0, -0.0, -0.0, -0.4, -0.0, -0.0, -0.0, -0.5, -0.0,
      -0.0, -0.0, -0.6, -0.0, -0.0, -0.0, -0.7, -0.0, -0.0, -0.0,
      -0.79999999999999993, -0.0, -0.0, -0.0, -0.89999999999999991, -0.0, -0.0,
      -0.0, -0.99999999999999989, -0.0, -0.0, -0.0, -1.0999999999999999, -0.0,
      -0.0, -0.0, -1.2, -0.0, -0.0, -0.0, -1.3, -0.0, -0.0, -0.0,
      -1.4000000000000001, -0.0, -0.0, -0.0, -1.5000000000000002, -0.0, -0.0,
      -0.0, -1.6000000000000003, -0.0, -0.0, -0.0, -1.7000000000000004, -0.0,
      -0.0, -0.0, -1.8000000000000005, -0.0, -0.0, -0.0, -1.9000000000000006,
      -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.1, -0.0, -0.0, -0.0,
      -0.2, -0.0, -0.0, -0.0, -0.30000000000000004, -0.0, -0.0, -0.0, -0.4, -0.0,
      -0.0, -0.0, -0.5, -0.0, -0.0, -0.0, -0.6, -0.0, -0.0, -0.0, -0.7, -0.0,
      -0.0, -0.0, -0.79999999999999993, -0.0, -0.0, -0.0, -0.89999999999999991,
      -0.0, -0.0, -0.0, -0.99999999999999989, -0.0, -0.0, -0.0,
      -1.0999999999999999, -0.0, -0.0, -0.0, -1.2, -0.0, -0.0, -0.0, -1.3, -0.0,
      -0.0, -0.0, -1.4000000000000001, -0.0, -0.0, -0.0, -1.5000000000000002,
      -0.0, -0.0, -0.0, -1.6000000000000003, -0.0, -0.0, -0.0,
      -1.7000000000000004, -0.0, -0.0, -0.0, -1.8000000000000005, -0.0, -0.0,
      -0.0, -1.9000000000000006, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
      -0.1, -0.0, -0.0, -0.0, -0.2, -0.0, -0.0, -0.0, -0.30000000000000004, -0.0,
      -0.0, -0.0, -0.4, -0.0, -0.0, -0.0, -0.5, -0.0, -0.0, -0.0, -0.6, -0.0,
      -0.0, -0.0, -0.7, -0.0, -0.0, -0.0, -0.79999999999999993, -0.0, -0.0, -0.0,
      -0.89999999999999991, -0.0, -0.0, -0.0, -0.99999999999999989, -0.0, -0.0,
      -0.0, -1.0999999999999999, -0.0, -0.0, -0.0, -1.2, -0.0, -0.0, -0.0, -1.3,
      -0.0, -0.0, -0.0, -1.4000000000000001, -0.0, -0.0, -0.0,
      -1.5000000000000002, -0.0, -0.0, -0.0, -1.6000000000000003, -0.0, -0.0,
      -0.0, -1.7000000000000004, -0.0, -0.0, -0.0, -1.8000000000000005, -0.0,
      -0.0, -0.0, -1.9000000000000006, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
      -0.0, -0.1, -0.0, -0.0, -0.0, -0.2, -0.0, -0.0, -0.0, -0.30000000000000004,
      -0.0, -0.0, -0.0, -0.4, -0.0, -0.0, -0.0, -0.5, -0.0, -0.0, -0.0, -0.6,
      -0.0, -0.0, -0.0, -0.7, -0.0, -0.0, -0.0, -0.79999999999999993, -0.0, -0.0,
      -0.0, -0.89999999999999991, -0.0, -0.0, -0.0, -0.99999999999999989, -0.0,
      -0.0, -0.0, -1.0999999999999999, -0.0, -0.0, -0.0, -1.2, -0.0, -0.0, -0.0,
      -1.3, -0.0, -0.0, -0.0, -1.4000000000000001, -0.0, -0.0, -0.0,
      -1.5000000000000002, -0.0, -0.0, -0.0, -1.6000000000000003, -0.0, -0.0,
      -0.0, -1.7000000000000004, -0.0, -0.0, -0.0, -1.8000000000000005, -0.0,
      -0.0, -0.0, -1.9000000000000006, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
      -0.0, -0.1, -0.0, -0.0, -0.0, -0.2, -0.0, -0.0, -0.0, -0.30000000000000004,
      -0.0, -0.0, -0.0, -0.4, -0.0, -0.0, -0.0, -0.5, -0.0, -0.0, -0.0, -0.6,
      -0.0, -0.0, -0.0, -0.7, -0.0, -0.0, -0.0, -0.79999999999999993, -0.0, -0.0,
      -0.0, -0.89999999999999991, -0.0, -0.0, -0.0, -0.99999999999999989, -0.0,
      -0.0, -0.0, -1.0999999999999999, -0.0, -0.0, -0.0, -1.2, -0.0, -0.0, -0.0,
      -1.3, -0.0, -0.0, -0.0, -1.4000000000000001, -0.0, -0.0, -0.0,
      -1.5000000000000002, -0.0, -0.0, -0.0, -1.6000000000000003, -0.0, -0.0,
      -0.0, -1.7000000000000004, -0.0, -0.0, -0.0, -1.8000000000000005, -0.0,
      -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.1,
      -0.0, -0.0, -0.0, -0.2, -0.0, -0.0, -0.0, -0.30000000000000004, -0.0, -0.0,
      -0.0, -0.4, -0.0, -0.0, -0.0, -0.5, -0.0, -0.0, -0.0, -0.6, -0.0, -0.0,
      -0.0, -0.7, -0.0, -0.0, -0.0, -0.79999999999999993, -0.0, -0.0, -0.0,
      -0.89999999999999991, -0.0, -0.0, -0.0, -0.99999999999999989, -0.0, -0.0,
      -0.0, -1.0999999999999999, -0.0, -0.0, -0.0, -1.2, -0.0, -0.0, -0.0, -1.3,
      -0.0, -0.0, -0.0, -1.4000000000000001, -0.0, -0.0, -0.0,
      -1.5000000000000002, -0.0, -0.0, -0.0, -1.6000000000000003, -0.0, -0.0,
      -0.0, -1.7000000000000004, -0.0, -0.0, -0.0, -1.8000000000000005, -0.0,
      -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.1,
      -0.0, -0.0, -0.0, -0.2, -0.0, -0.0, -0.0, -0.30000000000000004, -0.0, -0.0,
      -0.0, -0.4, -0.0, -0.0, -0.0, -0.5, -0.0, -0.0, -0.0, -0.6, -0.0, -0.0,
      -0.0, -0.7, -0.0, -0.0, -0.0, -0.79999999999999993, -0.0, -0.0, -0.0,
      -0.89999999999999991, -0.0, -0.0, -0.0, -0.99999999999999989, -0.0, -0.0,
      -0.0, -1.0999999999999999, -0.0, -0.0, -0.0, -1.2, -0.0, -0.0, -0.0, -1.3,
      -0.0, -0.0, -0.0, -1.4000000000000001, -0.0, -0.0, -0.0,
      -1.5000000000000002, -0.0, -0.0, -0.0, -1.6000000000000003, -0.0, -0.0,
      -0.0, -1.7000000000000004, -0.0, -0.0, -0.0, -1.8000000000000005, -0.0,
      -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.1,
      -0.0, -0.0, -0.0, -0.2, -0.0, -0.0, -0.0, -0.30000000000000004, -0.0, -0.0,
      -0.0, -0.4, -0.0, -0.0, -0.0, -0.5, -0.0, -0.0, -0.0, -0.6, -0.0, -0.0,
      -0.0, -0.7, -0.0, -0.0, -0.0, -0.79999999999999993, -0.0, -0.0, -0.0,
      -0.89999999999999991, -0.0, -0.0, -0.0, -0.99999999999999989, -0.0, -0.0,
      -0.0, -1.0999999999999999, -0.0, -0.0, -0.0, -1.2, -0.0, -0.0, -0.0, -1.3,
      -0.0, -0.0, -0.0, -1.4000000000000001, -0.0, -0.0, -0.0,
      -1.5000000000000002, -0.0, -0.0, -0.0, -1.6000000000000003, -0.0, -0.0,
      -0.0, -1.7000000000000004, -0.0, -0.0, -0.0, -1.8000000000000005 };

    static const real_T b_Kx[48] = { 21.000000000000004, 0.0, 0.0, 0.0, 0.0,
      21.000000000000004, 0.0, 0.0, 0.0, 0.0, 21.000000000000004, 0.0, 0.0, 0.0,
      0.0, 21.000000000000004, 19.000000000000004, 0.0, 0.0, 0.0, 0.0,
      19.000000000000004, 0.0, 0.0, 0.0, 0.0, 19.000000000000004, 0.0, 0.0, 0.0,
      0.0, 19.000000000000004, 17.1, 0.0, 0.0, 0.0, 0.0, 17.1, 0.0, 0.0, 0.0,
      0.0, 17.1, 0.0, 0.0, 0.0, 0.0, 17.1 };

    static const real_T b_Ku1[48] = { 28.70000000000001, 0.0, 0.0, 0.0, 0.0,
      28.70000000000001, 0.0, 0.0, 0.0, 0.0, 28.70000000000001, 0.0, 0.0, 0.0,
      0.0, 28.70000000000001, 26.600000000000005, 0.0, 0.0, 0.0, 0.0,
      26.600000000000005, 0.0, 0.0, 0.0, 0.0, 26.600000000000005, 0.0, 0.0, 0.0,
      0.0, 26.600000000000005, 24.510000000000005, 0.0, 0.0, 0.0, 0.0,
      24.510000000000005, 0.0, 0.0, 0.0, 0.0, 24.510000000000005, 0.0, 0.0, 0.0,
      0.0, 24.510000000000005 };

    static const real_T b_Linv[169] = { 0.18663083698528471, 0.0, 0.0, 0.0,
      -3.6360549438631788, 0.0, 0.0, 0.0, 2.0706843241079604, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.18663083698528471, 0.0, 0.0, 0.0, -3.6360549438631788, 0.0, 0.0,
      0.0, 2.0706843241079635, 0.0, 0.0, 0.0, 0.0, 0.0, 0.18663083698528471, 0.0,
      0.0, 0.0, -3.6360549438631788, 0.0, 0.0, 0.0, 2.0706843241079707, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.18663083698528471, 0.0, 0.0, 0.0, -3.6360549438631788,
      0.0, 0.0, 0.0, 2.0706843241079635, 0.0, 0.0, 0.0, 0.0, 0.0,
      3.9244788510643569, 0.0, 0.0, 0.0, -6.4796924427883935, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 3.9244788510643569, 0.0, 0.0, 0.0,
      -6.4796924427884033, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      3.9244788510643569, 0.0, 0.0, 0.0, -6.4796924427884255, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 3.9244788510643569, 0.0, 0.0, 0.0,
      -6.4796924427884033, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      4.6067104052644519, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 4.606710405264459, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 4.606710405264475, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.606710405264459, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.003162277660168379 };

    static const real_T b_Hinv[169] = { 17.543460194212134, 0.0, 0.0, 0.0,
      -27.687018294821787, 0.0, 0.0, 0.0, 9.53904302188613, 0.0, 0.0, 0.0, 0.0,
      0.0, 17.543460194212145, 0.0, 0.0, 0.0, -27.687018294821826, 0.0, 0.0, 0.0,
      9.53904302188616, 0.0, 0.0, 0.0, 0.0, 0.0, 17.543460194212177, 0.0, 0.0,
      0.0, -27.687018294821918, 0.0, 0.0, 0.0, 9.5390430218862257, 0.0, 0.0, 0.0,
      0.0, 0.0, 17.543460194212145, 0.0, 0.0, 0.0, -27.687018294821826, 0.0, 0.0,
      0.0, 9.53904302188616, 0.0, -27.687018294821787, 0.0, 0.0, 0.0,
      57.387948405580431, 0.0, 0.0, 0.0, -29.850066599106725, 0.0, 0.0, 0.0, 0.0,
      0.0, -27.687018294821826, 0.0, 0.0, 0.0, 57.387948405580559, 0.0, 0.0, 0.0,
      -29.850066599106817, 0.0, 0.0, 0.0, 0.0, 0.0, -27.687018294821918, 0.0,
      0.0, 0.0, 57.38794840558085, 0.0, 0.0, 0.0, -29.850066599107024, 0.0, 0.0,
      0.0, 0.0, 0.0, -27.687018294821826, 0.0, 0.0, 0.0, 57.387948405580559, 0.0,
      0.0, 0.0, -29.850066599106817, 0.0, 9.53904302188613, 0.0, 0.0, 0.0,
      -29.850066599106725, 0.0, 0.0, 0.0, 21.221780757971771, 0.0, 0.0, 0.0, 0.0,
      0.0, 9.53904302188616, 0.0, 0.0, 0.0, -29.850066599106817, 0.0, 0.0, 0.0,
      21.221780757971835, 0.0, 0.0, 0.0, 0.0, 0.0, 9.5390430218862257, 0.0, 0.0,
      0.0, -29.850066599107024, 0.0, 0.0, 0.0, 21.221780757971985, 0.0, 0.0, 0.0,
      0.0, 0.0, 9.53904302188616, 0.0, 0.0, 0.0, -29.850066599106817, 0.0, 0.0,
      0.0, 21.221780757971835, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 9.9999999999999974E-6 };

    static const real_T b_Ac[624] = { -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0,
      -0.0, -1.0, -0.0, -0.0, -0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
      0.0, 0.0, 0.0, -1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
      -0.0, -0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0,
      0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -0.0, -1.0,
      -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -1.0, -0.0, -0.0,
      -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0, 0.0, 1.0, 0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -0.0,
      -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0,
      -0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
      -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
      0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0,
      -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0,
      0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -1.0,
      -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0,
      -0.0, -0.0, -1.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -0.0,
      -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, -0.0, -0.0,
      -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
      -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 1.0, 0.0, 0.0, 0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
      -1.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0,
      -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
      -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -0.0, -0.0,
      -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
      -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
      -0.0, -0.0, -0.0, -1.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 1.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
      -1.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
      -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -0.0, -0.0,
      -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

    static const int8_T c_a[192] = { -1, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, 1, 0,
      0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, 1, 0,
      0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, 1, 0,
      0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, 1, 0,
      0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0 };

    static const real_T e_a[16] = { 0.1, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
      0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.1 };

    static const real_T f_a[16] = { 0.095124921972503731, 0.0, -0.0, 0.0, -0.0,
      0.095124921972503731, -0.0, 0.0, -0.0, 0.0, 0.095124921972503731, 0.0,
      -0.0, 0.0, -0.0, 0.095124921972503731 };

    // Reset subsysRan breadcrumbs
    srClearBC(test_sim_DW.CommandVelocityPublisher_Subsys);

    // Reset subsysRan breadcrumbs
    srClearBC(test_sim_DW.MPCController_SubsysRanBC);

    // Reset subsysRan breadcrumbs
    srClearBC(test_sim_DW.EnabledSubsystem_SubsysRanBC);

    // FromWorkspace: '<Root>/reference trajectory'
    {
      real_T *pDataValues = (real_T *)
        test_sim_DW.referencetrajectory_PWORK.DataPtr;
      real_T *pTimeValues = (real_T *)
        test_sim_DW.referencetrajectory_PWORK.TimePtr;
      int_T currTimeIndex = test_sim_DW.referencetrajectory_IWORK.PrevIndex;
      real_T t = test_sim_M->Timing.t[0];

      // Get index
      if (t <= pTimeValues[0]) {
        currTimeIndex = 0;
      } else if (t >= pTimeValues[102]) {
        currTimeIndex = 101;
      } else {
        if (t < pTimeValues[currTimeIndex]) {
          while (t < pTimeValues[currTimeIndex]) {
            currTimeIndex--;
          }
        } else {
          while (t >= pTimeValues[currTimeIndex + 1]) {
            currTimeIndex++;
          }
        }
      }

      test_sim_DW.referencetrajectory_IWORK.PrevIndex = currTimeIndex;

      // Post output
      {
        real_T t1 = pTimeValues[currTimeIndex];
        real_T t2 = pTimeValues[currTimeIndex + 1];
        if (t1 == t2) {
          if (t < t1) {
            {
              int_T elIdx;
              for (elIdx = 0; elIdx < 4; ++elIdx) {
                (&test_sim_B.referencetrajectory[0])[elIdx] =
                  pDataValues[currTimeIndex];
                pDataValues += 103;
              }
            }
          } else {
            {
              int_T elIdx;
              for (elIdx = 0; elIdx < 4; ++elIdx) {
                (&test_sim_B.referencetrajectory[0])[elIdx] =
                  pDataValues[currTimeIndex + 1];
                pDataValues += 103;
              }
            }
          }
        } else {
          real_T f1 = (t2 - t) / (t2 - t1);
          real_T f2 = 1.0 - f1;
          real_T d1;
          real_T d2;
          int_T TimeIndex = currTimeIndex;

          {
            int_T elIdx;
            for (elIdx = 0; elIdx < 4; ++elIdx) {
              d1 = pDataValues[TimeIndex];
              d2 = pDataValues[TimeIndex + 1];
              (&test_sim_B.referencetrajectory[0])[elIdx] = (real_T)
                rtInterpolate(d1, d2, f1, f2);
              pDataValues += 103;
            }
          }
        }
      }
    }

    if (rtmIsMajorTimeStep(test_sim_M) &&
        test_sim_M->Timing.TaskCounters.TID[2] == 0) {
      // Outputs for Atomic SubSystem: '<Root>/Subscribe'
      // MATLABSystem: '<S4>/SourceBlock'
      test_sim_B.SourceBlock_o1 = Sub_test_sim_41.getLatestMessage
        (&test_sim_B.b_varargout_2);

      // Outputs for Enabled SubSystem: '<S4>/Enabled Subsystem' incorporates:
      //   EnablePort: '<S30>/Enable'

      if (test_sim_B.SourceBlock_o1) {
        // SignalConversion generated from: '<S30>/In1' incorporates:
        //   MATLABSystem: '<S4>/SourceBlock'
        //
        test_sim_B.In1 = test_sim_B.b_varargout_2;
        srUpdateBC(test_sim_DW.EnabledSubsystem_SubsysRanBC);
      }

      // End of Outputs for SubSystem: '<S4>/Enabled Subsystem'
      // End of Outputs for SubSystem: '<Root>/Subscribe'

      // MATLAB Function: '<Root>/Conversion'
      test_sim_B.aSinInput = 1.0 / sqrt(((test_sim_B.In1.Pose.Orientation.W *
        test_sim_B.In1.Pose.Orientation.W + test_sim_B.In1.Pose.Orientation.X *
        test_sim_B.In1.Pose.Orientation.X) + test_sim_B.In1.Pose.Orientation.Y *
        test_sim_B.In1.Pose.Orientation.Y) + test_sim_B.In1.Pose.Orientation.Z *
        test_sim_B.In1.Pose.Orientation.Z);
      test_sim_B.y_innov[0] = test_sim_B.In1.Pose.Orientation.W *
        test_sim_B.aSinInput;
      test_sim_B.y_innov[1] = test_sim_B.In1.Pose.Orientation.X *
        test_sim_B.aSinInput;
      test_sim_B.y_innov[2] = test_sim_B.In1.Pose.Orientation.Y *
        test_sim_B.aSinInput;
      test_sim_B.y_innov[3] = test_sim_B.In1.Pose.Orientation.Z *
        test_sim_B.aSinInput;
      test_sim_B.aSinInput = (test_sim_B.y_innov[1] * test_sim_B.y_innov[3] -
        test_sim_B.y_innov[0] * test_sim_B.y_innov[2]) * -2.0;
      test_sim_B.b_idx_0 = test_sim_B.aSinInput;
      if (test_sim_B.aSinInput >= 0.99999999999999778) {
        test_sim_B.b_idx_0 = 1.0;
      }

      if (test_sim_B.aSinInput <= -0.99999999999999778) {
        test_sim_B.b_idx_0 = -1.0;
      }

      mask = ((test_sim_B.aSinInput >= 0.99999999999999778) ||
              (test_sim_B.aSinInput <= -0.99999999999999778));
      test_sim_B.aSinInput = test_sim_B.y_innov[0] * test_sim_B.y_innov[0];
      test_sim_B.eul_tmp = test_sim_B.y_innov[1] * test_sim_B.y_innov[1];
      test_sim_B.eul_tmp_c = test_sim_B.y_innov[2] * test_sim_B.y_innov[2];
      test_sim_B.eul_tmp_b = test_sim_B.y_innov[3] * test_sim_B.y_innov[3];
      test_sim_B.eul[0] = rt_atan2d_snf((test_sim_B.y_innov[1] *
        test_sim_B.y_innov[2] + test_sim_B.y_innov[0] * test_sim_B.y_innov[3]) *
        2.0, ((test_sim_B.aSinInput + test_sim_B.eul_tmp) - test_sim_B.eul_tmp_c)
        - test_sim_B.eul_tmp_b);
      test_sim_B.eul[1] = asin(test_sim_B.b_idx_0);
      test_sim_B.eul[2] = rt_atan2d_snf((test_sim_B.y_innov[2] *
        test_sim_B.y_innov[3] + test_sim_B.y_innov[0] * test_sim_B.y_innov[1]) *
        2.0, ((test_sim_B.aSinInput - test_sim_B.eul_tmp) - test_sim_B.eul_tmp_c)
        + test_sim_B.eul_tmp_b);
      test_sim_B.trueCount = 0;

      // MATLAB Function: '<Root>/Conversion'
      if (mask) {
        for (test_sim_B.i = 0; test_sim_B.i < 1; test_sim_B.i++) {
          test_sim_B.trueCount++;
        }
      }

      if (test_sim_B.trueCount - 1 >= 0) {
        test_sim_B.x_data = test_sim_B.b_idx_0;
      }

      test_sim_B.trueCount_c = 0;

      // MATLAB Function: '<Root>/Conversion'
      if (mask) {
        for (test_sim_B.i = 0; test_sim_B.i < 1; test_sim_B.i++) {
          test_sim_B.trueCount_c++;
        }
      }

      for (test_sim_B.i = 0; test_sim_B.i < test_sim_B.trueCount_c; test_sim_B.i
           ++) {
        if (rtIsNaN(test_sim_B.x_data)) {
          test_sim_B.x_data = (rtNaN);
        } else if (test_sim_B.x_data < 0.0) {
          test_sim_B.x_data = -1.0;
        } else {
          test_sim_B.x_data = (test_sim_B.x_data > 0.0);
        }
      }

      test_sim_B.trueCount_c = 0;

      // MATLAB Function: '<Root>/Conversion'
      if (mask) {
        for (test_sim_B.i = 0; test_sim_B.i < 1; test_sim_B.i++) {
          test_sim_B.trueCount_c++;
        }
      }

      test_sim_B.tmp_size[0] = 1;
      test_sim_B.tmp_size[1] = test_sim_B.trueCount_c;
      test_sim_B.trueCount_f = 0;

      // MATLAB Function: '<Root>/Conversion'
      if (mask) {
        tmp_data = 0;
        for (test_sim_B.i = 0; test_sim_B.i < 1; test_sim_B.i++) {
          test_sim_B.trueCount_f++;
        }
      }

      if (test_sim_B.trueCount == test_sim_B.trueCount_f) {
        // MATLAB Function: '<Root>/Conversion'
        if (test_sim_B.trueCount_c - 1 >= 0) {
          test_sim_B.eul[0] = -test_sim_B.x_data * 2.0 * rt_atan2d_snf
            (test_sim_B.y_innov[1], test_sim_B.y_innov[0]);
        }
      } else {
        // MATLAB Function: '<Root>/Conversion'
        test_sim_binary_expand_op(test_sim_B.eul, &tmp_data, test_sim_B.tmp_size,
          &test_sim_B.x_data, test_sim_B.y_innov);
      }

      // MATLAB Function: '<Root>/Conversion'
      test_sim_B.yaw = test_sim_B.eul[0];

      // Outputs for Enabled SubSystem: '<Root>/MPC Controller' incorporates:
      //   EnablePort: '<S3>/Enable'

      if (rtsiIsModeUpdateTimeStep(&test_sim_M->solverInfo)) {
        if (test_sim_B.SourceBlock_o1) {
          if (!test_sim_DW.MPCController_MODE) {
            (void) memset(&(test_sim_XDis.position), 0,
                          4*sizeof(boolean_T));
            test_sim_DW.MPCController_MODE = true;
          }
        } else {
          if (((test_sim_M->Timing.clockTick2) * 0.1) == rtmGetTStart(test_sim_M))
          {
            (void) memset(&(test_sim_XDis.position), 1,
                          4*sizeof(boolean_T));
          }

          if (test_sim_DW.MPCController_MODE) {
            (void) memset(&(test_sim_XDis.position), 1,
                          4*sizeof(boolean_T));
            test_sim_DW.MPCController_MODE = false;
          }
        }
      }

      // End of Outputs for SubSystem: '<Root>/MPC Controller'
    }

    // Outputs for Enabled SubSystem: '<Root>/MPC Controller' incorporates:
    //   EnablePort: '<S3>/Enable'

    if (test_sim_DW.MPCController_MODE) {
      if (rtmIsMajorTimeStep(test_sim_M) &&
          test_sim_M->Timing.TaskCounters.TID[2] == 0) {
        // MATLAB Function: '<S28>/optimizer' incorporates:
        //   Memory: '<S8>/last_x'
        //   UnitDelay: '<S8>/last_mv'

        test_sim_B.aSinInput = test_sim_B.referencetrajectory[0];
        test_sim_B.eul_tmp = test_sim_B.referencetrajectory[1];
        test_sim_B.eul_tmp_c = test_sim_B.referencetrajectory[2];
        test_sim_B.eul_tmp_b = test_sim_B.referencetrajectory[3];
        for (test_sim_B.trueCount = 0; test_sim_B.trueCount < 20;
             test_sim_B.trueCount++) {
          test_sim_B.i = test_sim_B.trueCount << 2;
          test_sim_B.rseq[test_sim_B.i] = test_sim_B.aSinInput;
          test_sim_B.rseq[test_sim_B.i + 1] = test_sim_B.eul_tmp;
          test_sim_B.rseq[test_sim_B.i + 2] = test_sim_B.eul_tmp_c;
          test_sim_B.rseq[test_sim_B.i + 3] = test_sim_B.eul_tmp_b;
        }

        test_sim_B.old_u[0] = test_sim_DW.last_mv_DSTATE[0];
        test_sim_B.xk[0] = test_sim_DW.last_x_PreviousInput[0];
        test_sim_B.old_u[1] = test_sim_DW.last_mv_DSTATE[1];
        test_sim_B.xk[1] = test_sim_DW.last_x_PreviousInput[1];
        test_sim_B.old_u[2] = test_sim_DW.last_mv_DSTATE[2];
        test_sim_B.xk[2] = test_sim_DW.last_x_PreviousInput[2];
        test_sim_B.old_u[3] = test_sim_DW.last_mv_DSTATE[3];
        test_sim_B.xk[3] = test_sim_DW.last_x_PreviousInput[3];

        // SignalConversion generated from: '<S29>/ SFunction ' incorporates:
        //   MATLAB Function: '<S28>/optimizer'

        test_sim_B.xest[0] = test_sim_B.In1.Pose.Position.X;
        test_sim_B.xest[1] = test_sim_B.In1.Pose.Position.Y;
        test_sim_B.xest[2] = test_sim_B.In1.Pose.Position.Z;
        test_sim_B.xest[3] = test_sim_B.yaw;

        // MATLAB Function: '<S28>/optimizer' incorporates:
        //   Memory: '<S8>/Memory'
        //   Memory: '<S8>/last_x'

        test_sim_B.aSinInput = test_sim_DW.last_x_PreviousInput[1];
        test_sim_B.eul_tmp = test_sim_DW.last_x_PreviousInput[0];
        test_sim_B.eul_tmp_c = test_sim_DW.last_x_PreviousInput[2];
        test_sim_B.eul_tmp_b = test_sim_DW.last_x_PreviousInput[3];
        for (test_sim_B.i = 0; test_sim_B.i < 4; test_sim_B.i++) {
          test_sim_B.y_innov[test_sim_B.i] = test_sim_B.xest[test_sim_B.i] - (((
            static_cast<real_T>(a[test_sim_B.i + 4]) * test_sim_B.aSinInput +
            static_cast<real_T>(a[test_sim_B.i]) * test_sim_B.eul_tmp) +
            static_cast<real_T>(a[test_sim_B.i + 8]) * test_sim_B.eul_tmp_c) +
            static_cast<real_T>(a[test_sim_B.i + 12]) * test_sim_B.eul_tmp_b);
        }

        test_sim_B.aSinInput = test_sim_B.y_innov[1];
        test_sim_B.eul_tmp = test_sim_B.y_innov[0];
        test_sim_B.eul_tmp_c = test_sim_B.y_innov[2];
        test_sim_B.eul_tmp_b = test_sim_B.y_innov[3];
        for (test_sim_B.i = 0; test_sim_B.i < 4; test_sim_B.i++) {
          test_sim_B.xest[test_sim_B.i] = (((b_a[test_sim_B.i + 4] *
            test_sim_B.aSinInput + b_a[test_sim_B.i] * test_sim_B.eul_tmp) +
            b_a[test_sim_B.i + 8] * test_sim_B.eul_tmp_c) + b_a[test_sim_B.i +
            12] * test_sim_B.eul_tmp_b) + test_sim_B.xk[test_sim_B.i];
        }

        memset(&test_sim_B.f[0], 0, 13U * sizeof(real_T));
        for (test_sim_B.trueCount = 0; test_sim_B.trueCount < 12;
             test_sim_B.trueCount++) {
          test_sim_B.aSinInput = 0.0;
          for (test_sim_B.i = 0; test_sim_B.i < 80; test_sim_B.i++) {
            test_sim_B.aSinInput += b_Kr[80 * test_sim_B.trueCount +
              test_sim_B.i] * test_sim_B.rseq[test_sim_B.i];
          }

          test_sim_B.i = test_sim_B.trueCount << 2;
          test_sim_B.f[test_sim_B.trueCount] = ((((b_Kx[test_sim_B.i + 1] *
            test_sim_B.xest[1] + b_Kx[test_sim_B.i] * test_sim_B.xest[0]) +
            b_Kx[test_sim_B.i + 2] * test_sim_B.xest[2]) + b_Kx[test_sim_B.i + 3]
            * test_sim_B.xest[3]) + test_sim_B.aSinInput) +
            (((b_Ku1[test_sim_B.i + 1] * test_sim_B.old_u[1] +
               b_Ku1[test_sim_B.i] * test_sim_B.old_u[0]) + b_Ku1[test_sim_B.i +
              2] * test_sim_B.old_u[2]) + b_Ku1[test_sim_B.i + 3] *
             test_sim_B.old_u[3]);
        }

        test_sim_B.aSinInput = test_sim_B.xest[0];
        test_sim_B.eul_tmp = test_sim_B.xest[1];
        test_sim_B.eul_tmp_c = test_sim_B.xest[2];
        test_sim_B.eul_tmp_b = test_sim_B.xest[3];
        test_sim_B.b_idx_0 = test_sim_B.old_u[1];
        test_sim_B.old_u_m = test_sim_B.old_u[0];
        test_sim_B.old_u_c = test_sim_B.old_u[2];
        test_sim_B.old_u_k = test_sim_B.old_u[3];
        for (test_sim_B.i = 0; test_sim_B.i < 48; test_sim_B.i++) {
          test_sim_B.iAout[test_sim_B.i] =
            test_sim_DW.Memory_PreviousInput[test_sim_B.i];
          test_sim_B.dv[test_sim_B.i] = -(((((0.0 * test_sim_B.aSinInput + 0.0 *
            test_sim_B.eul_tmp) + 0.0 * test_sim_B.eul_tmp_c) + 0.0 *
            test_sim_B.eul_tmp_b) + 0.5) + (((static_cast<real_T>
            (c_a[test_sim_B.i + 48]) * test_sim_B.b_idx_0 + static_cast<real_T>
            (c_a[test_sim_B.i]) * test_sim_B.old_u_m) + static_cast<real_T>
            (c_a[test_sim_B.i + 96]) * test_sim_B.old_u_c) + static_cast<real_T>
            (c_a[test_sim_B.i + 144]) * test_sim_B.old_u_k));
        }

        test_sim_qpkwik(b_Linv, b_Hinv, test_sim_B.f, b_Ac, test_sim_B.dv,
                        test_sim_B.iAout, 244, 1.0E-6, test_sim_B.zopt,
                        test_sim_B.a__1, &test_sim_B.trueCount);
        if ((test_sim_B.trueCount < 0) || (test_sim_B.trueCount == 0)) {
          memset(&test_sim_B.zopt[0], 0, 13U * sizeof(real_T));
        }

        test_sim_B.aSinInput = test_sim_DW.last_x_PreviousInput[1];
        test_sim_B.eul_tmp = test_sim_DW.last_x_PreviousInput[0];
        test_sim_B.eul_tmp_c = test_sim_DW.last_x_PreviousInput[2];
        test_sim_B.eul_tmp_b = test_sim_DW.last_x_PreviousInput[3];
        for (test_sim_B.trueCount = 0; test_sim_B.trueCount < 4;
             test_sim_B.trueCount++) {
          test_sim_B.b_idx_0 = test_sim_B.old_u[test_sim_B.trueCount] +
            test_sim_B.zopt[test_sim_B.trueCount];
          test_sim_B.u[test_sim_B.trueCount] = test_sim_B.b_idx_0;
          test_sim_B.xest[test_sim_B.trueCount] = test_sim_B.b_idx_0;
          test_sim_B.xk[test_sim_B.trueCount] = ((static_cast<real_T>
            (a[test_sim_B.trueCount + 4]) * test_sim_B.aSinInput +
            static_cast<real_T>(a[test_sim_B.trueCount]) * test_sim_B.eul_tmp) +
            static_cast<real_T>(a[test_sim_B.trueCount + 8]) *
            test_sim_B.eul_tmp_c) + static_cast<real_T>(a[test_sim_B.trueCount +
            12]) * test_sim_B.eul_tmp_b;
        }

        test_sim_B.b_idx_0 = test_sim_B.xest[1];
        test_sim_B.old_u_m = test_sim_B.xest[0];
        test_sim_B.old_u_c = test_sim_B.xest[2];
        test_sim_B.old_u_k = test_sim_B.xest[3];
        test_sim_B.aSinInput = test_sim_B.y_innov[1];
        test_sim_B.eul_tmp = test_sim_B.y_innov[0];
        test_sim_B.eul_tmp_c = test_sim_B.y_innov[2];
        test_sim_B.eul_tmp_b = test_sim_B.y_innov[3];
        for (test_sim_B.i = 0; test_sim_B.i < 4; test_sim_B.i++) {
          test_sim_B.xk1[test_sim_B.i] = ((((e_a[test_sim_B.i + 4] *
            test_sim_B.b_idx_0 + e_a[test_sim_B.i] * test_sim_B.old_u_m) +
            e_a[test_sim_B.i + 8] * test_sim_B.old_u_c) + e_a[test_sim_B.i + 12]
            * test_sim_B.old_u_k) + test_sim_B.xk[test_sim_B.i]) +
            (((f_a[test_sim_B.i + 4] * test_sim_B.aSinInput + f_a[test_sim_B.i] *
               test_sim_B.eul_tmp) + f_a[test_sim_B.i + 8] *
              test_sim_B.eul_tmp_c) + f_a[test_sim_B.i + 12] *
             test_sim_B.eul_tmp_b);

          // Gain: '<S8>/umin_scale1'
          test_sim_B.umin_scale1[test_sim_B.i] =
            test_sim_P.umin_scale1_Gain[test_sim_B.i] *
            test_sim_B.u[test_sim_B.i];
        }

        // SignalConversion generated from: '<S3>/mv_vel_y'
        test_sim_B.mv_vel_y = test_sim_B.umin_scale1[1];

        // SignalConversion generated from: '<S3>/mv_vel_yaw'
        test_sim_B.mv_vel_yaw = test_sim_B.umin_scale1[3];

        // SignalConversion generated from: '<S3>/mv_vel_z'
        test_sim_B.mv_vel_z = test_sim_B.umin_scale1[2];
      }

      // StateSpace: '<S3>/Plant (State-Space)'
      test_sim_B.measuredoutputs[0] = 0.0;
      test_sim_B.measuredoutputs[1] = 0.0;
      test_sim_B.measuredoutputs[2] = 0.0;
      test_sim_B.measuredoutputs[3] = 0.0;
      test_sim_B.ri = test_sim_P.PlantStateSpace_C_jc[0U];
      while (test_sim_B.ri < test_sim_P.PlantStateSpace_C_jc[1U]) {
        test_sim_B.measuredoutputs[test_sim_P.PlantStateSpace_C_ir[test_sim_B.ri]]
          += test_sim_P.PlantStateSpace_C_pr[test_sim_B.ri] *
          test_sim_X.position[0U];
        test_sim_B.ri++;
      }

      test_sim_B.ri = test_sim_P.PlantStateSpace_C_jc[1U];
      while (test_sim_B.ri < test_sim_P.PlantStateSpace_C_jc[2U]) {
        test_sim_B.measuredoutputs[test_sim_P.PlantStateSpace_C_ir[test_sim_B.ri]]
          += test_sim_P.PlantStateSpace_C_pr[test_sim_B.ri] *
          test_sim_X.position[1U];
        test_sim_B.ri++;
      }

      test_sim_B.ri = test_sim_P.PlantStateSpace_C_jc[2U];
      while (test_sim_B.ri < test_sim_P.PlantStateSpace_C_jc[3U]) {
        test_sim_B.measuredoutputs[test_sim_P.PlantStateSpace_C_ir[test_sim_B.ri]]
          += test_sim_P.PlantStateSpace_C_pr[test_sim_B.ri] *
          test_sim_X.position[2U];
        test_sim_B.ri++;
      }

      test_sim_B.ri = test_sim_P.PlantStateSpace_C_jc[3U];
      while (test_sim_B.ri < test_sim_P.PlantStateSpace_C_jc[4U]) {
        test_sim_B.measuredoutputs[test_sim_P.PlantStateSpace_C_ir[test_sim_B.ri]]
          += test_sim_P.PlantStateSpace_C_pr[test_sim_B.ri] *
          test_sim_X.position[3U];
        test_sim_B.ri++;
      }

      // End of StateSpace: '<S3>/Plant (State-Space)'
      if (rtmIsMajorTimeStep(test_sim_M) &&
          test_sim_M->Timing.TaskCounters.TID[1] == 0) {
      }

      if (rtsiIsModeUpdateTimeStep(&test_sim_M->solverInfo)) {
        srUpdateBC(test_sim_DW.MPCController_SubsysRanBC);
      }
    }

    // End of Outputs for SubSystem: '<Root>/MPC Controller'
    if (rtmIsMajorTimeStep(test_sim_M) &&
        test_sim_M->Timing.TaskCounters.TID[2] == 0) {
      // Outputs for Enabled SubSystem: '<Root>/Command Velocity Publisher' incorporates:
      //   EnablePort: '<S1>/Enable'

      if (test_sim_B.SourceBlock_o1) {
        // BusAssignment: '<S1>/Bus Assignment1' incorporates:
        //   Constant: '<S1>/Coordinate Frame'
        //   Constant: '<S1>/Type Mask'
        //   Constant: '<S5>/Constant'
        //   DataTypeConversion: '<S1>/Data Type Conversion'

        test_sim_B.BusAssignment1 = test_sim_P.Constant_Value;
        test_sim_B.BusAssignment1.CoordinateFrame =
          test_sim_P.CoordinateFrame_Value;
        test_sim_B.BusAssignment1.TypeMask = test_sim_P.TypeMask_Value;
        test_sim_B.BusAssignment1.Velocity.X = test_sim_B.umin_scale1[0];
        test_sim_B.BusAssignment1.Velocity.Y = test_sim_B.mv_vel_y;
        test_sim_B.BusAssignment1.Velocity.Z = test_sim_B.mv_vel_z;
        test_sim_B.BusAssignment1.YawRate = static_cast<real32_T>
          (test_sim_B.mv_vel_yaw);

        // Outputs for Atomic SubSystem: '<S1>/Publish2'
        // MATLABSystem: '<S6>/SinkBlock'
        Pub_test_sim_8.publish(&test_sim_B.BusAssignment1);

        // End of Outputs for SubSystem: '<S1>/Publish2'
        if (rtsiIsModeUpdateTimeStep(&test_sim_M->solverInfo)) {
          srUpdateBC(test_sim_DW.CommandVelocityPublisher_Subsys);
        }
      }

      // End of Outputs for SubSystem: '<Root>/Command Velocity Publisher'
    }

    if (rtmIsMajorTimeStep(test_sim_M) &&
        test_sim_M->Timing.TaskCounters.TID[1] == 0) {
      // S-Function (saeroclockpacer): '<Root>/Simulation Pace'
      //
      //  The Clock Pacer generates no code, it is only active in
      //  interpreted simulation.

    }
  }

  if (rtmIsMajorTimeStep(test_sim_M)) {
    int32_T i;

    // Update for Enabled SubSystem: '<Root>/MPC Controller' incorporates:
    //   EnablePort: '<S3>/Enable'

    if (test_sim_DW.MPCController_MODE && (rtmIsMajorTimeStep(test_sim_M) &&
         test_sim_M->Timing.TaskCounters.TID[2] == 0)) {
      // Update for Memory: '<S8>/Memory'
      for (i = 0; i < 48; i++) {
        test_sim_DW.Memory_PreviousInput[i] = test_sim_B.iAout[i];
      }

      // End of Update for Memory: '<S8>/Memory'

      // Update for UnitDelay: '<S8>/last_mv'
      test_sim_DW.last_mv_DSTATE[0] = test_sim_B.u[0];

      // Update for Memory: '<S8>/last_x'
      test_sim_DW.last_x_PreviousInput[0] = test_sim_B.xk1[0];

      // Update for UnitDelay: '<S8>/last_mv'
      test_sim_DW.last_mv_DSTATE[1] = test_sim_B.u[1];

      // Update for Memory: '<S8>/last_x'
      test_sim_DW.last_x_PreviousInput[1] = test_sim_B.xk1[1];

      // Update for UnitDelay: '<S8>/last_mv'
      test_sim_DW.last_mv_DSTATE[2] = test_sim_B.u[2];

      // Update for Memory: '<S8>/last_x'
      test_sim_DW.last_x_PreviousInput[2] = test_sim_B.xk1[2];

      // Update for UnitDelay: '<S8>/last_mv'
      test_sim_DW.last_mv_DSTATE[3] = test_sim_B.u[3];

      // Update for Memory: '<S8>/last_x'
      test_sim_DW.last_x_PreviousInput[3] = test_sim_B.xk1[3];
    }

    // End of Update for SubSystem: '<Root>/MPC Controller'

    // External mode
    rtExtModeUploadCheckTrigger(3);

    {                                  // Sample time: [0.0s, 0.0s]
      rtExtModeUpload(0, (real_T)test_sim_M->Timing.t[0]);
    }

    if (rtmIsMajorTimeStep(test_sim_M) &&
        test_sim_M->Timing.TaskCounters.TID[1] == 0) {
                                  // Sample time: [0.033333333333333333s, 0.0s]
      rtExtModeUpload(1, (real_T)((test_sim_M->Timing.clockTick1) *
        0.033333333333333333));
    }

    if (rtmIsMajorTimeStep(test_sim_M) &&
        test_sim_M->Timing.TaskCounters.TID[2] == 0) {// Sample time: [0.1s, 0.0s] 
      rtExtModeUpload(2, (real_T)((test_sim_M->Timing.clockTick2) * 0.1));
    }
  }                                    // end MajorTimeStep

  if (rtmIsMajorTimeStep(test_sim_M)) {
    // signal main to stop simulation
    {                                  // Sample time: [0.0s, 0.0s]
      if ((rtmGetTFinal(test_sim_M)!=-1) &&
          !((rtmGetTFinal(test_sim_M)-((test_sim_M->Timing.clockTick1) *
             0.033333333333333333)) > ((test_sim_M->Timing.clockTick1) *
            0.033333333333333333) * (DBL_EPSILON))) {
        rtmSetErrorStatus(test_sim_M, "Simulation finished");
      }

      if (rtmGetStopRequested(test_sim_M)) {
        rtmSetErrorStatus(test_sim_M, "Simulation finished");
      }
    }

    rt_ertODEUpdateContinuousStates(&test_sim_M->solverInfo);

    // Update absolute time for base rate
    // The "clockTick0" counts the number of times the code of this task has
    //  been executed. The absolute time is the multiplication of "clockTick0"
    //  and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
    //  overflow during the application lifespan selected.

    ++test_sim_M->Timing.clockTick0;
    test_sim_M->Timing.t[0] = rtsiGetSolverStopTime(&test_sim_M->solverInfo);

    {
      // Update absolute timer for sample time: [0.033333333333333333s, 0.0s]
      // The "clockTick1" counts the number of times the code of this task has
      //  been executed. The resolution of this integer timer is 0.033333333333333333, which is the step size
      //  of the task. Size of "clockTick1" ensures timer will not overflow during the
      //  application lifespan selected.

      test_sim_M->Timing.clockTick1++;
    }

    if (rtmIsMajorTimeStep(test_sim_M) &&
        test_sim_M->Timing.TaskCounters.TID[2] == 0) {
      // Update absolute timer for sample time: [0.1s, 0.0s]
      // The "clockTick2" counts the number of times the code of this task has
      //  been executed. The resolution of this integer timer is 0.1, which is the step size
      //  of the task. Size of "clockTick2" ensures timer will not overflow during the
      //  application lifespan selected.

      test_sim_M->Timing.clockTick2++;
    }

    rate_scheduler();
  }                                    // end MajorTimeStep
}

// Derivatives for root system: '<Root>'
void test_sim_derivatives(void)
{
  XDot_test_sim_T *_rtXdot;
  uint32_T ri;
  _rtXdot = ((XDot_test_sim_T *) test_sim_M->derivs);

  // Derivatives for Enabled SubSystem: '<Root>/MPC Controller'
  if (test_sim_DW.MPCController_MODE) {
    // Derivatives for StateSpace: '<S3>/Plant (State-Space)'
    _rtXdot->position[0] = 0.0;
    _rtXdot->position[1] = 0.0;
    _rtXdot->position[2] = 0.0;
    _rtXdot->position[3] = 0.0;
    for (ri = test_sim_P.PlantStateSpace_B_jc[0U]; ri <
         test_sim_P.PlantStateSpace_B_jc[1U]; ri++) {
      _rtXdot->position[test_sim_P.PlantStateSpace_B_ir[ri]] +=
        test_sim_P.PlantStateSpace_B_pr[ri] * test_sim_B.umin_scale1[0U];
    }

    for (ri = test_sim_P.PlantStateSpace_B_jc[1U]; ri <
         test_sim_P.PlantStateSpace_B_jc[2U]; ri++) {
      _rtXdot->position[test_sim_P.PlantStateSpace_B_ir[ri]] +=
        test_sim_P.PlantStateSpace_B_pr[ri] * test_sim_B.umin_scale1[1U];
    }

    for (ri = test_sim_P.PlantStateSpace_B_jc[2U]; ri <
         test_sim_P.PlantStateSpace_B_jc[3U]; ri++) {
      _rtXdot->position[test_sim_P.PlantStateSpace_B_ir[ri]] +=
        test_sim_P.PlantStateSpace_B_pr[ri] * test_sim_B.umin_scale1[2U];
    }

    for (ri = test_sim_P.PlantStateSpace_B_jc[3U]; ri <
         test_sim_P.PlantStateSpace_B_jc[4U]; ri++) {
      _rtXdot->position[test_sim_P.PlantStateSpace_B_ir[ri]] +=
        test_sim_P.PlantStateSpace_B_pr[ri] * test_sim_B.umin_scale1[3U];
    }

    // End of Derivatives for StateSpace: '<S3>/Plant (State-Space)'
  } else {
    {
      real_T *dx;
      int_T i;
      dx = &(((XDot_test_sim_T *) test_sim_M->derivs)->position[0]);
      for (i=0; i < 4; i++) {
        dx[i] = 0.0;
      }
    }
  }

  // End of Derivatives for SubSystem: '<Root>/MPC Controller'
}

// Model initialize function
void test_sim_initialize(void)
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  {
    // Setup solver object
    rtsiSetSimTimeStepPtr(&test_sim_M->solverInfo,
                          &test_sim_M->Timing.simTimeStep);
    rtsiSetTPtr(&test_sim_M->solverInfo, &rtmGetTPtr(test_sim_M));
    rtsiSetStepSizePtr(&test_sim_M->solverInfo, &test_sim_M->Timing.stepSize0);
    rtsiSetdXPtr(&test_sim_M->solverInfo, &test_sim_M->derivs);
    rtsiSetContStatesPtr(&test_sim_M->solverInfo, (real_T **)
                         &test_sim_M->contStates);
    rtsiSetNumContStatesPtr(&test_sim_M->solverInfo,
      &test_sim_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&test_sim_M->solverInfo,
      &test_sim_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&test_sim_M->solverInfo,
      &test_sim_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&test_sim_M->solverInfo,
      &test_sim_M->periodicContStateRanges);
    rtsiSetContStateDisabledPtr(&test_sim_M->solverInfo, (boolean_T**)
      &test_sim_M->contStateDisabled);
    rtsiSetErrorStatusPtr(&test_sim_M->solverInfo, (&rtmGetErrorStatus
      (test_sim_M)));
    rtsiSetRTModelPtr(&test_sim_M->solverInfo, test_sim_M);
  }

  rtsiSetSimTimeStep(&test_sim_M->solverInfo, MAJOR_TIME_STEP);
  test_sim_M->intgData.y = test_sim_M->odeY;
  test_sim_M->intgData.f[0] = test_sim_M->odeF[0];
  test_sim_M->intgData.f[1] = test_sim_M->odeF[1];
  test_sim_M->intgData.f[2] = test_sim_M->odeF[2];
  test_sim_M->contStates = ((X_test_sim_T *) &test_sim_X);
  test_sim_M->contStateDisabled = ((XDis_test_sim_T *) &test_sim_XDis);
  test_sim_M->Timing.tStart = (0.0);
  rtsiSetSolverData(&test_sim_M->solverInfo, static_cast<void *>
                    (&test_sim_M->intgData));
  rtsiSetIsMinorTimeStepWithModeChange(&test_sim_M->solverInfo, false);
  rtsiSetSolverName(&test_sim_M->solverInfo,"ode3");
  rtmSetTPtr(test_sim_M, &test_sim_M->Timing.tArray[0]);
  rtmSetTFinal(test_sim_M, -1);
  test_sim_M->Timing.stepSize0 = 0.033333333333333333;

  // External mode info
  test_sim_M->Sizes.checksums[0] = (221771891U);
  test_sim_M->Sizes.checksums[1] = (1924164527U);
  test_sim_M->Sizes.checksums[2] = (3965349428U);
  test_sim_M->Sizes.checksums[3] = (1409060635U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[11];
    test_sim_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    systemRan[1] = (sysRanDType *)&test_sim_DW.CommandVelocityPublisher_Subsys;
    systemRan[2] = (sysRanDType *)&test_sim_DW.CommandVelocityPublisher_Subsys;
    systemRan[3] = (sysRanDType *)&test_sim_DW.CommandVelocityPublisher_Subsys;
    systemRan[4] = (sysRanDType *)&test_sim_DW.CommandVelocityPublisher_Subsys;
    systemRan[5] = &rtAlwaysEnabled;
    systemRan[6] = (sysRanDType *)&test_sim_DW.MPCController_SubsysRanBC;
    systemRan[7] = (sysRanDType *)&test_sim_DW.MPCController_SubsysRanBC;
    systemRan[8] = (sysRanDType *)&test_sim_DW.EnabledSubsystem_SubsysRanBC;
    systemRan[9] = &rtAlwaysEnabled;
    systemRan[10] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(test_sim_M->extModeInfo,
      &test_sim_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(test_sim_M->extModeInfo, test_sim_M->Sizes.checksums);
    rteiSetTPtr(test_sim_M->extModeInfo, rtmGetTPtr(test_sim_M));
  }

  // data type transition information
  {
    static DataTypeTransInfo dtInfo;
    test_sim_M->SpecialInfo.mappingInfo = (&dtInfo);
    dtInfo.numDataTypes = 33;
    dtInfo.dataTypeSizes = &rtDataTypeSizes[0];
    dtInfo.dataTypeNames = &rtDataTypeNames[0];

    // Block I/O transition table
    dtInfo.BTransTable = &rtBTransTable;

    // Parameters transition table
    dtInfo.PTransTable = &rtPTransTable;
  }

  {
    int32_T i;
    char_T b_zeroDelimTopic[28];
    char_T b_zeroDelimTopic_0[27];
    static const char_T b_zeroDelimTopic_1[28] = "/mavros/local_position/pose";
    static const char_T b_zeroDelimTopic_2[27] = "/mavros/setpoint_raw/local";

    // Start for FromWorkspace: '<Root>/reference trajectory'
    {
      static real_T pTimeValues0[] = { 144.047241015, 144.296288545,
        144.546328475, 144.796385733, 145.046420652, 145.296468155,
        145.546498075, 145.796550834, 146.046583258, 146.296630785,
        146.546661224, 146.796710655, 147.046742117, 147.296783707,
        147.546823119, 147.796878635, 148.046909163, 148.296952363,
        148.546979433, 148.797023455, 149.047066848, 149.298108701,
        149.548137883, 149.798180575, 150.04821082, 150.297255868, 150.547285688,
        150.797319569, 151.04735215, 151.297377168, 151.547403519, 151.797431308,
        152.047474204, 152.297504089, 152.547529814, 152.797558106,
        153.047583612, 153.297607564, 153.54763136, 153.797656601, 154.047687689,
        154.297714228, 154.547753001, 154.797777663, 155.04781738, 155.298846176,
        155.548879346, 155.798901783, 156.047937421, 156.297964005,
        156.548003621, 156.798028361, 157.048078237, 157.298101137,
        157.548135238, 157.798159307, 158.048191415, 158.29821666, 158.548252785,
        158.798277339, 159.048305495, 159.298325612, 159.54836081, 159.798400023,
        160.048437772, 160.298458787, 160.548487091, 160.798513445,
        161.048544316, 161.299564757, 161.54959821, 161.799618375, 162.04864992,
        162.298671314, 162.548700961, 162.798720593, 163.04875585, 163.298773727,
        163.5488071, 163.798830357, 164.048860362, 164.298877746, 164.548911894,
        164.798929042, 165.048955342, 165.298972258, 165.548999717,
        165.799017913, 166.049049542, 166.299067522, 166.54909283, 166.799110599,
        167.049138599, 167.300156163, 167.550185366, 167.800201235,
        168.049227147, 168.299244326, 168.549267875, 168.799283372,
        169.049314731, 169.29933155, 169.549352153 } ;

      static real_T pDataValues0[] = { -0.019857294857501984,
        -0.019564099609851837, -0.0193055160343647, -0.019423285499215126,
        -0.019377386197447777, -0.019209882244467735, -0.018965033814311028,
        -0.017974363639950752, -0.016122808679938316, -0.013991611078381538,
        -0.012165898457169533, -0.011055116541683674, -0.010259519331157207,
        -0.0095635866746306419, -0.009131590835750103, -0.0087829176336526871,
        -0.008166944608092308, -0.0072690178640186787, -0.0064367055892944336,
        -0.0055942744947969913, -0.0043896539136767387, -0.0028780782595276833,
        -0.0015253658639267087, -0.00033754541072994471, 0.00097246887162327766,
        0.0024361950345337391, 0.0036950791254639626, 0.0054558156989514828,
        0.0074752508662641048, 0.0094565236940979958, 0.011645916849374771,
        0.013666299171745777, 0.015425466932356358, 0.017053725197911263,
        0.018367854878306389, 0.020032757893204689, 0.021910300478339195,
        0.023681743070483208, 0.024917794391512871, 0.025423789396882057,
        0.025650585070252419, 0.025970125570893288, 0.026223430410027504,
        0.025821840390563011, 0.016955383121967316, 0.0081789419054985046,
        -0.01035118754953146, -0.048362437635660172, -0.0988360345363617,
        -0.15627835690975189, -0.21748599410057068, -0.2804638147354126,
        -0.34435093402862549, -0.40746816992759705, -0.47020736336708069,
        -0.5330536961555481, -0.595484733581543, -0.65747392177581787,
        -0.719372034072876, -0.78186237812042236, -0.84441202878952026,
        -0.906339168548584, -0.96843397617340088, -1.03067147731781,
        -1.093328595161438, -1.1560255289077759, -1.2159368991851807,
        -1.2556898593902588, -1.275795578956604, -1.2851467132568359,
        -1.2883375883102417, -1.2884472608566284, -1.2874851226806641,
        -1.2866787910461426, -1.2865751981735229, -1.2871946096420288,
        -1.2880187034606934, -1.2885274887084961, -1.2891265153884888,
        -1.2897135019302368, -1.2902905941009521, -1.2908527851104736,
        -1.2912043333053589, -1.2911272048950195, -1.2903437614440918,
        -1.2889664173126221, -1.2874513864517212, -1.2859025001525879,
        -1.2841311693191528, -1.2822754383087158, -1.2800401449203491,
        -1.2777742147445679, -1.2755054235458374, -1.27326238155365,
        -1.271633505821228, -1.2699979543685913, -1.2682257890701294,
        -1.2661781311035156, -1.2637056112289429, -1.261491060256958,
        -1.2595921754837036, -1.2576684951782227, -1.2559922933578491,
        0.020600490272045135, 0.020276082679629326, 0.020556069910526276,
        0.0213304590433836, 0.022125916555523872, 0.0233463067561388,
        0.029069144278764725, 0.042706243693828583, 0.055028799921274185,
        0.0856797993183136, 0.13334259390830994, 0.18898198008537292,
        0.24809792637825012, 0.30956539511680603, 0.37182405591011047,
        0.433633953332901, 0.49477666616439819, 0.55495727062225342,
        0.61553037166595459, 0.6768416166305542, 0.73773455619812012,
        0.798874020576477, 0.86075586080551147, 0.9237515926361084,
        0.98679667711257935, 1.0496023893356323, 1.1117613315582275,
        1.1733241081237793, 1.2353992462158203, 1.2977206707000732,
        1.359832763671875, 1.4221682548522949, 1.484398365020752,
        1.5464884042739868, 1.6090420484542847, 1.6723407506942749,
        1.7355808019638062, 1.7985124588012695, 1.8592581748962402,
        1.9011940956115723, 1.9215819835662842, 1.9307799339294434,
        1.9350172281265259, 1.9364718198776245, 1.9366283416748047,
        1.9364503622055054, 1.9369851350784302, 1.9383187294006348,
        1.9398256540298462, 1.9413366317749023, 1.9430267810821533,
        1.9449071884155273, 1.9472368955612183, 1.9500902891159058,
        1.9530327320098877, 1.9558678865432739, 1.9587007761001587,
        1.9610157012939453, 1.9629117250442505, 1.9646973609924316,
        1.9667917490005493, 1.9686814546585083, 1.970451831817627,
        1.9720222949981689, 1.9736059904098511, 1.9761025905609131,
        1.9790371656417847, 1.9812401533126831, 1.9823328256607056,
        1.9826626777648926, 1.9830055236816406, 1.9832427501678467,
        1.9829213619232178, 1.9823006391525269, 1.9886970520019531,
        1.997833251953125, 2.0116443634033203, 2.0464479923248291,
        2.0949459075927734, 2.1506979465484619, 2.2099294662475586,
        2.27099609375, 2.3325662612915039, 2.3949286937713623,
        2.4578642845153809, 2.5205790996551514, 2.5826539993286133,
        2.6446695327758789, 2.7058670520782471, 2.7668914794921875,
        2.8289322853088379, 2.891237735748291, 2.9533731937408447,
        3.0152847766876221, 3.077317476272583, 3.1397721767425537,
        3.2028188705444336, 3.2570598125457764, 3.2870478630065918,
        3.3025107383728027, 3.3096466064453125, 3.3115437030792236,
        3.3106625080108643, 1.9800217151641846, 1.9795643091201782,
        1.9789596796035767, 1.9784351587295532, 1.9780582189559937,
        1.9778209924697876, 1.9777333736419678, 1.9777439832687378,
        1.9777628183364868, 1.9780943393707275, 1.9785575866699219,
        1.978915810585022, 1.9792290925979614, 1.979543924331665,
        1.9798109531402588, 1.9800263643264771, 1.980217456817627,
        1.98013174533844, 1.9795624017715454, 1.9788421392440796,
        1.9781688451766968, 1.9777708053588867, 1.97760808467865,
        1.977515697479248, 1.9774503707885742, 1.9774206876754761,
        1.9773720502853394, 1.9777345657348633, 1.9783257246017456,
        1.9789336919784546, 1.9794069528579712, 1.9797186851501465,
        1.9799258708953857, 1.9800446033477783, 1.9801504611968994,
        1.9801952838897705, 1.9801759719848633, 1.9801298379898071,
        1.9800686836242676, 1.9797115325927734, 1.9792327880859375,
        1.9786447286605835, 1.9780972003936768, 1.9778327941894531,
        1.9778996706008911, 1.9780548810958862, 1.9782116413116455,
        1.9785025119781494, 1.9787341356277466, 1.9788640737533569,
        1.9789849519729614, 1.9791053533554077, 1.9791983366012573,
        1.9793164730072021, 1.979517936706543, 1.9796503782272339,
        1.9796960353851318, 1.9797426462173462, 1.9797192811965942,
        1.9796332120895386, 1.9793764352798462, 1.9790618419647217,
        1.9788029193878174, 1.9785788059234619, 1.978490948677063,
        1.9783759117126465, 1.9783538579940796, 1.9784020185470581,
        1.9784531593322754, 1.9783920049667358, 1.9783514738082886,
        1.9784497022628784, 1.9786176681518555, 1.9788857698440552,
        1.9791886806488037, 1.9794278144836426, 1.979509711265564,
        1.979575514793396, 1.9795917272567749, 1.9795811176300049,
        1.979581356048584, 1.979536771774292, 1.9793722629547119,
        1.9791951179504395, 1.9789770841598511, 1.9787266254425049,
        1.9784880876541138, 1.9783186912536621, 1.9782124757766724,
        1.9781272411346436, 1.9780616760253906, 1.9782924652099609,
        1.9786167144775391, 1.9789260625839233, 1.9791632890701294,
        1.9792567491531372, 1.9793782234191895, 1.9796358346939087,
        1.9798692464828491, 1.9799318313598633, 1.9799098968505859,
        1.979846715927124, 1.9798508882522583, 1.547559474655662,
        1.5474677691843846, 1.5474078590658049, 1.5473764828082661,
        1.5474262806263308, 1.5474954927947862, 1.5474508098003008,
        1.5474279290672879, 1.5474295179036, 1.5474738879737, 1.5474990374085051,
        1.5475477493044476, 1.5475418447193245, 1.5475608548757176,
        1.5475672977652886, 1.5475664949652292, 1.5474667801198103,
        1.5474074883994202, 1.5474411128696541, 1.5475137914207318,
        1.5475101685759165, 1.5473648561772444, 1.5474843448635676,
        1.5474094162371497, 1.5475115338948111, 1.5475321123984198,
        1.54752650211123, 1.5475334181126694, 1.5474278303470947,
        1.5474809157338478, 1.5475289011981825, 1.5475224210557081,
        1.5474681938674788, 1.5474600447949505, 1.5474633118745427,
        1.5474995980646948, 1.5475224545833208, 1.5475464938816168,
        1.5474141156908612, 1.5473489231106381, 1.5473601958390812,
        1.5474164607611038, 1.5474114260312657, 1.5472825924542286,
        1.547346525886331, 1.5472936938193182, 1.5472768536445238,
        1.54718799988297, 1.5471107280489542, 1.5471678497877459,
        1.5473497519877293, 1.5473800926145653, 1.547496267655168,
        1.5475447299566607, 1.547505703815494, 1.5475498242911438,
        1.5474940417942147, 1.5475474307921269, 1.5475686705347635,
        1.5476879841324427, 1.5476073837515454, 1.547546706223164,
        1.5475403713670115, 1.5476487158474068, 1.5476041427489855,
        1.5476455251362662, 1.5475586960699896, 1.5475670928743224,
        1.5475484831866362, 1.5475333138045411, 1.5475621736284833,
        1.5475944197413067, 1.5476283180203776, 1.5475646472212414,
        1.5474695852634055, 1.5474351542678217, 1.5474440148707964,
        1.5474212440338473, 1.5474662082877495, 1.5475118915226798,
        1.5474322410908083, 1.5473995069649555, 1.5474080434676745,
        1.5474616895106177, 1.5474616671588757, 1.5474871872600655,
        1.5475359494474272, 1.5474992572006323, 1.5473938296225409,
        1.5473186588522532, 1.5473613599922995, 1.5473939264800884,
        1.5474762423571684, 1.5474063093450405, 1.5473869098958113,
        1.5473238910224776, 1.5473809159037213, 1.5472840527680258,
        1.5472572139140706, 1.5472265845772366, 1.5472127488490681,
        1.5472985329714157, 1.5473003136601786 } ;

      test_sim_DW.referencetrajectory_PWORK.TimePtr = static_cast<void *>
        (pTimeValues0);
      test_sim_DW.referencetrajectory_PWORK.DataPtr = static_cast<void *>
        (pDataValues0);
      test_sim_DW.referencetrajectory_IWORK.PrevIndex = 0;
    }

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S4>/Enabled Subsystem'
    // SystemInitialize for SignalConversion generated from: '<S30>/In1' incorporates:
    //   Outport: '<S30>/Out1'

    test_sim_B.In1 = test_sim_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S4>/Enabled Subsystem'

    // Start for MATLABSystem: '<S4>/SourceBlock'
    test_sim_DW.obj_l.matlabCodegenIsDeleted = false;
    test_sim_DW.obj_l.isInitialized = 1;
    for (i = 0; i < 28; i++) {
      b_zeroDelimTopic[i] = b_zeroDelimTopic_1[i];
    }

    Sub_test_sim_41.createSubscriber(&b_zeroDelimTopic[0], 51);
    test_sim_DW.obj_l.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S4>/SourceBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe'
    // InitializeConditions for Memory: '<S8>/Memory'
    for (i = 0; i < 48; i++) {
      test_sim_DW.Memory_PreviousInput[i] = test_sim_P.Memory_InitialCondition[i];
    }

    // End of InitializeConditions for Memory: '<S8>/Memory'

    // InitializeConditions for UnitDelay: '<S8>/last_mv'
    test_sim_DW.last_mv_DSTATE[0] = test_sim_P.last_mv_InitialCondition[0];

    // InitializeConditions for Memory: '<S8>/last_x'
    test_sim_DW.last_x_PreviousInput[0] = test_sim_P.last_x_InitialCondition[0];

    // InitializeConditions for StateSpace: '<S3>/Plant (State-Space)'
    test_sim_X.position[0] = test_sim_P.PlantStateSpace_InitialConditio;

    // InitializeConditions for UnitDelay: '<S8>/last_mv'
    test_sim_DW.last_mv_DSTATE[1] = test_sim_P.last_mv_InitialCondition[1];

    // InitializeConditions for Memory: '<S8>/last_x'
    test_sim_DW.last_x_PreviousInput[1] = test_sim_P.last_x_InitialCondition[1];

    // InitializeConditions for StateSpace: '<S3>/Plant (State-Space)'
    test_sim_X.position[1] = test_sim_P.PlantStateSpace_InitialConditio;

    // InitializeConditions for UnitDelay: '<S8>/last_mv'
    test_sim_DW.last_mv_DSTATE[2] = test_sim_P.last_mv_InitialCondition[2];

    // InitializeConditions for Memory: '<S8>/last_x'
    test_sim_DW.last_x_PreviousInput[2] = test_sim_P.last_x_InitialCondition[2];

    // InitializeConditions for StateSpace: '<S3>/Plant (State-Space)'
    test_sim_X.position[2] = test_sim_P.PlantStateSpace_InitialConditio;

    // InitializeConditions for UnitDelay: '<S8>/last_mv'
    test_sim_DW.last_mv_DSTATE[3] = test_sim_P.last_mv_InitialCondition[3];

    // InitializeConditions for Memory: '<S8>/last_x'
    test_sim_DW.last_x_PreviousInput[3] = test_sim_P.last_x_InitialCondition[3];

    // InitializeConditions for StateSpace: '<S3>/Plant (State-Space)'
    test_sim_X.position[3] = test_sim_P.PlantStateSpace_InitialConditio;

    // SystemInitialize for Outport: '<S3>/mv_vel_x' incorporates:
    //   Gain: '<S8>/umin_scale1'

    test_sim_B.umin_scale1[0] = test_sim_P.mv_vel_x_Y0;

    // SystemInitialize for SignalConversion generated from: '<S3>/mv_vel_y' incorporates:
    //   Outport: '<S3>/mv_vel_y'

    test_sim_B.mv_vel_y = test_sim_P.mv_vel_y_Y0;

    // SystemInitialize for SignalConversion generated from: '<S3>/mv_vel_z' incorporates:
    //   Outport: '<S3>/mv_vel_z'

    test_sim_B.mv_vel_z = test_sim_P.mv_vel_z_Y0;

    // SystemInitialize for SignalConversion generated from: '<S3>/mv_vel_yaw' incorporates:
    //   Outport: '<S3>/mv_vel_yaw'

    test_sim_B.mv_vel_yaw = test_sim_P.mv_vel_yaw_Y0;

    // End of SystemInitialize for SubSystem: '<Root>/MPC Controller'

    // SystemInitialize for Enabled SubSystem: '<Root>/Command Velocity Publisher' 
    // SystemInitialize for Atomic SubSystem: '<S1>/Publish2'
    // Start for MATLABSystem: '<S6>/SinkBlock'
    test_sim_DW.obj.matlabCodegenIsDeleted = false;
    test_sim_DW.obj.isInitialized = 1;
    for (i = 0; i < 27; i++) {
      b_zeroDelimTopic_0[i] = b_zeroDelimTopic_2[i];
    }

    Pub_test_sim_8.createPublisher(&b_zeroDelimTopic_0[0], 105);
    test_sim_DW.obj.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S6>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<S1>/Publish2'
    // End of SystemInitialize for SubSystem: '<Root>/Command Velocity Publisher' 
  }
}

// Model terminate function
void test_sim_terminate(void)
{
  // Terminate for Atomic SubSystem: '<Root>/Subscribe'
  // Terminate for MATLABSystem: '<S4>/SourceBlock'
  if (!test_sim_DW.obj_l.matlabCodegenIsDeleted) {
    test_sim_DW.obj_l.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S4>/SourceBlock'
  // End of Terminate for SubSystem: '<Root>/Subscribe'

  // Terminate for Enabled SubSystem: '<Root>/Command Velocity Publisher'
  // Terminate for Atomic SubSystem: '<S1>/Publish2'
  // Terminate for MATLABSystem: '<S6>/SinkBlock'
  if (!test_sim_DW.obj.matlabCodegenIsDeleted) {
    test_sim_DW.obj.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S6>/SinkBlock'
  // End of Terminate for SubSystem: '<S1>/Publish2'
  // End of Terminate for SubSystem: '<Root>/Command Velocity Publisher'
}

//
// File trailer for generated code.
//
// [EOF]
//
