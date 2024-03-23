//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: rough.cpp
//
// Code generated for Simulink model 'rough'.
//
// Model version                  : 1.4
// Simulink Coder version         : 23.2 (R2023b) 01-Aug-2023
// C/C++ source code generated on : Thu Feb  8 02:21:01 2024
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "rough.h"
#include "rtwtypes.h"
#include <string.h>
#include "rough_private.h"
#include <math.h>

extern "C"
{

#include "rt_nonfinite.h"

}

#include "rough_dt.h"

// Named constants for MATLAB Function: '<S27>/optimizer'
const int32_T rough_degrees = 13;

// Block signals (default storage)
B_rough_T rough_B;

// Continuous states
X_rough_T rough_X;

// Disabled State Vector
XDis_rough_T rough_XDis;

// Block states (default storage)
DW_rough_T rough_DW;

// Real-time model
RT_MODEL_rough_T rough_M_ = RT_MODEL_rough_T();
RT_MODEL_rough_T *const rough_M = &rough_M_;

// Forward declaration for local functions
static real_T rough_norm(const real_T x[13]);
static real_T rough_maximum(const real_T x[13]);
static real_T rough_xnrm2(int32_T n, const real_T x[169], int32_T ix0);
static void rough_xgemv(int32_T b_m, int32_T n, const real_T b_A[169], int32_T
  ia0, const real_T x[169], int32_T ix0, real_T y[13]);
static void rough_xgerc(int32_T b_m, int32_T n, real_T alpha1, int32_T ix0,
  const real_T y[13], real_T b_A[169], int32_T ia0);
static void rough_KWIKfactor(const real_T b_Ac[624], const int32_T iC[48],
  int32_T nA, const real_T b_Linv[169], real_T D[169], real_T b_H[169], int32_T
  n, real_T RLinv[169], real_T *Status);
static void rough_DropConstraint(int32_T kDrop, boolean_T iA[48], int32_T *nA,
  int32_T iC[48]);
static void rough_qpkwik(const real_T b_Linv[169], const real_T b_Hinv[169],
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

  (rough_M->Timing.TaskCounters.TID[2])++;
  if ((rough_M->Timing.TaskCounters.TID[2]) > 2) {// Sample time: [0.1s, 0.0s]
    rough_M->Timing.TaskCounters.TID[2] = 0;
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
  rough_derivatives();

  // f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*));
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  rough_step();
  rough_derivatives();

  // f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*));
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  rough_step();
  rough_derivatives();

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

// Function for MATLAB Function: '<S27>/optimizer'
static real_T rough_norm(const real_T x[13])
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

// Function for MATLAB Function: '<S27>/optimizer'
static real_T rough_maximum(const real_T x[13])
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

// Function for MATLAB Function: '<S27>/optimizer'
static real_T rough_xnrm2(int32_T n, const real_T x[169], int32_T ix0)
{
  real_T y;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x[ix0 - 1]);
    } else {
      int32_T kend;
      rough_B.scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (int32_T k = ix0; k <= kend; k++) {
        rough_B.absxk = fabs(x[k - 1]);
        if (rough_B.absxk > rough_B.scale) {
          rough_B.t = rough_B.scale / rough_B.absxk;
          y = y * rough_B.t * rough_B.t + 1.0;
          rough_B.scale = rough_B.absxk;
        } else {
          rough_B.t = rough_B.absxk / rough_B.scale;
          y += rough_B.t * rough_B.t;
        }
      }

      y = rough_B.scale * sqrt(y);
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

// Function for MATLAB Function: '<S27>/optimizer'
static void rough_xgemv(int32_T b_m, int32_T n, const real_T b_A[169], int32_T
  ia0, const real_T x[169], int32_T ix0, real_T y[13])
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
      rough_B.c = 0.0;
      d = (b_iy + b_m) - 1;
      for (ia = b_iy; ia <= d; ia++) {
        rough_B.c += x[((ix0 + ia) - b_iy) - 1] * b_A[ia - 1];
      }

      ia = div_nzp_s32_floor(b_iy - ia0, 13);
      y[ia] += rough_B.c;
    }
  }
}

// Function for MATLAB Function: '<S27>/optimizer'
static void rough_xgerc(int32_T b_m, int32_T n, real_T alpha1, int32_T ix0,
  const real_T y[13], real_T b_A[169], int32_T ia0)
{
  if (!(alpha1 == 0.0)) {
    int32_T jA;
    jA = ia0;
    for (int32_T j = 0; j < n; j++) {
      rough_B.temp = y[j];
      if (rough_B.temp != 0.0) {
        int32_T b;
        rough_B.temp *= alpha1;
        b = b_m + jA;
        for (int32_T ijA = jA; ijA < b; ijA++) {
          b_A[ijA - 1] += b_A[((ix0 + ijA) - jA) - 1] * rough_B.temp;
        }
      }

      jA += 13;
    }
  }
}

// Function for MATLAB Function: '<S27>/optimizer'
static void rough_KWIKfactor(const real_T b_Ac[624], const int32_T iC[48],
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
      rough_B.beta1 = 0.0;
      for (ii = 0; ii < 13; ii++) {
        rough_B.beta1 += b_Ac[(48 * ii + knt) - 1] * b_Linv[13 * ii + i];
      }

      RLinv[i + 13 * b_lastv] = rough_B.beta1;
    }
  }

  memcpy(&rough_B.b_A[0], &RLinv[0], 169U * sizeof(real_T));
  memset(&rough_B.tau[0], 0, 13U * sizeof(real_T));
  memset(&rough_B.work[0], 0, 13U * sizeof(real_T));
  for (i = 0; i < 13; i++) {
    ii = i * 13 + i;
    if (i + 1 < 13) {
      rough_B.atmp = rough_B.b_A[ii];
      b_lastv = ii + 2;
      rough_B.tau[i] = 0.0;
      rough_B.beta1 = rough_xnrm2(12 - i, rough_B.b_A, ii + 2);
      if (rough_B.beta1 != 0.0) {
        rough_B.b_A_g = rough_B.b_A[ii];
        rough_B.beta1 = rt_hypotd_snf(rough_B.b_A_g, rough_B.beta1);
        if (rough_B.b_A_g >= 0.0) {
          rough_B.beta1 = -rough_B.beta1;
        }

        if (fabs(rough_B.beta1) < 1.0020841800044864E-292) {
          knt = 0;
          f_tmp = (ii - i) + 13;
          do {
            knt++;
            for (b_coltop = b_lastv; b_coltop <= f_tmp; b_coltop++) {
              rough_B.b_A[b_coltop - 1] *= 9.9792015476736E+291;
            }

            rough_B.beta1 *= 9.9792015476736E+291;
            rough_B.atmp *= 9.9792015476736E+291;
          } while ((fabs(rough_B.beta1) < 1.0020841800044864E-292) && (knt < 20));

          rough_B.beta1 = rt_hypotd_snf(rough_B.atmp, rough_xnrm2(12 - i,
            rough_B.b_A, ii + 2));
          if (rough_B.atmp >= 0.0) {
            rough_B.beta1 = -rough_B.beta1;
          }

          rough_B.tau[i] = (rough_B.beta1 - rough_B.atmp) / rough_B.beta1;
          rough_B.atmp = 1.0 / (rough_B.atmp - rough_B.beta1);
          for (b_coltop = b_lastv; b_coltop <= f_tmp; b_coltop++) {
            rough_B.b_A[b_coltop - 1] *= rough_B.atmp;
          }

          for (b_lastv = 0; b_lastv < knt; b_lastv++) {
            rough_B.beta1 *= 1.0020841800044864E-292;
          }

          rough_B.atmp = rough_B.beta1;
        } else {
          rough_B.tau[i] = (rough_B.beta1 - rough_B.b_A_g) / rough_B.beta1;
          rough_B.atmp = 1.0 / (rough_B.b_A_g - rough_B.beta1);
          b_coltop = (ii - i) + 13;
          for (knt = b_lastv; knt <= b_coltop; knt++) {
            rough_B.b_A[knt - 1] *= rough_B.atmp;
          }

          rough_B.atmp = rough_B.beta1;
        }
      }

      rough_B.b_A[ii] = 1.0;
      if (rough_B.tau[i] != 0.0) {
        b_lastv = 13 - i;
        knt = (ii - i) + 12;
        while ((b_lastv > 0) && (rough_B.b_A[knt] == 0.0)) {
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
              if (rough_B.b_A[f_tmp] != 0.0) {
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
        rough_xgemv(b_lastv, knt, rough_B.b_A, ii + 14, rough_B.b_A, ii + 1,
                    rough_B.work);
        rough_xgerc(b_lastv, knt, -rough_B.tau[i], ii + 1, rough_B.work,
                    rough_B.b_A, ii + 14);
      }

      rough_B.b_A[ii] = rough_B.atmp;
    } else {
      rough_B.tau[12] = 0.0;
    }
  }

  for (i = 0; i < 13; i++) {
    for (ii = 0; ii <= i; ii++) {
      rough_B.R[ii + 13 * i] = rough_B.b_A[13 * i + ii];
    }

    for (ii = i + 2; ii < 14; ii++) {
      rough_B.R[(ii + 13 * i) - 1] = 0.0;
    }

    rough_B.work[i] = 0.0;
  }

  for (i = 12; i >= 0; i--) {
    ii = (i * 13 + i) + 14;
    if (i + 1 < 13) {
      rough_B.b_A[ii - 14] = 1.0;
      if (rough_B.tau[i] != 0.0) {
        b_lastv = 13 - i;
        knt = ii - i;
        while ((b_lastv > 0) && (rough_B.b_A[knt - 2] == 0.0)) {
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
              if (rough_B.b_A[f_tmp - 1] != 0.0) {
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
        rough_xgemv(b_lastv, knt, rough_B.b_A, ii, rough_B.b_A, ii - 13,
                    rough_B.work);
        rough_xgerc(b_lastv, knt, -rough_B.tau[i], ii - 13, rough_B.work,
                    rough_B.b_A, ii);
      }

      knt = (ii - i) - 1;
      for (b_lastv = ii - 12; b_lastv <= knt; b_lastv++) {
        rough_B.b_A[b_lastv - 1] *= -rough_B.tau[i];
      }
    }

    rough_B.b_A[ii - 14] = 1.0 - rough_B.tau[i];
    for (b_lastv = 0; b_lastv < i; b_lastv++) {
      rough_B.b_A[(ii - b_lastv) - 15] = 0.0;
    }
  }

  i = 0;
  do {
    exitg1 = 0;
    if (i <= nA - 1) {
      if (fabs(rough_B.R[13 * i + i]) < 1.0E-12) {
        *Status = -2.0;
        exitg1 = 1;
      } else {
        i++;
      }
    } else {
      for (ii = 0; ii < n; ii++) {
        for (b_lastv = 0; b_lastv < n; b_lastv++) {
          rough_B.beta1 = 0.0;
          for (i = 0; i < 13; i++) {
            rough_B.beta1 += b_Linv[13 * ii + i] * rough_B.b_A[13 * b_lastv + i];
          }

          rough_B.TL[ii + 13 * b_lastv] = rough_B.beta1;
        }
      }

      memset(&RLinv[0], 0, 169U * sizeof(real_T));
      for (i = nA; i >= 1; i--) {
        knt = (i - 1) * 13;
        b_coltop = (i + knt) - 1;
        RLinv[b_coltop] = 1.0;
        for (ii = i; ii <= nA; ii++) {
          f_tmp = ((ii - 1) * 13 + i) - 1;
          RLinv[f_tmp] /= rough_B.R[b_coltop];
        }

        if (i > 1) {
          for (ii = 0; ii <= i - 2; ii++) {
            for (b_lastv = i; b_lastv <= nA; b_lastv++) {
              b_coltop = (b_lastv - 1) * 13;
              f_tmp = b_coltop + ii;
              RLinv[f_tmp] -= RLinv[(b_coltop + i) - 1] * rough_B.R[knt + ii];
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
            b_H[i] -= rough_B.TL[(ii + knt) - 1] * rough_B.TL[ii + b_lastv];
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
            D[i] += rough_B.TL[ii + knt] * RLinv[ii + b_lastv];
          }
        }
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);
}

// Function for MATLAB Function: '<S27>/optimizer'
static void rough_DropConstraint(int32_T kDrop, boolean_T iA[48], int32_T *nA,
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

// Function for MATLAB Function: '<S27>/optimizer'
static void rough_qpkwik(const real_T b_Linv[169], const real_T b_Hinv[169],
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
  int32_T iSave;
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
  memset(&rough_B.r[0], 0, 13U * sizeof(real_T));
  rough_B.rMin = 0.0;
  cTolComputed = false;
  for (i = 0; i < 48; i++) {
    rough_B.cTol[i] = 1.0;
    rough_B.iC[i] = 0;
  }

  rough_B.nA = 0;
  for (i = 0; i < 48; i++) {
    if (iA[i]) {
      rough_B.nA++;
      rough_B.iC[rough_B.nA - 1] = i + 1;
    }
  }

  guard1 = false;
  if (rough_B.nA > 0) {
    memset(&rough_B.Opt[0], 0, 26U * sizeof(real_T));
    for (i = 0; i < 13; i++) {
      rough_B.Rhs[i] = f[i];
      rough_B.Rhs[i + 13] = 0.0;
    }

    DualFeasible = false;
    rough_B.tmp = static_cast<int32_T>(rt_roundd_snf(0.3 * static_cast<real_T>
      (rough_B.nA)));
    ColdReset = false;
    do {
      exitg3 = 0;
      if ((!DualFeasible) && (rough_B.nA > 0) && (*status <= maxiter)) {
        rough_KWIKfactor(b_Ac, rough_B.iC, rough_B.nA, b_Linv, rough_B.D,
                         rough_B.b_H, rough_degrees, rough_B.RLinv,
                         &rough_B.Xnorm0);
        if (rough_B.Xnorm0 < 0.0) {
          if (ColdReset) {
            *status = -2;
            exitg3 = 2;
          } else {
            rough_B.nA = 0;
            memset(&rough_B.iC[0], 0, 48U * sizeof(int32_T));
            for (i = 0; i < 48; i++) {
              iA[i] = false;
            }

            ColdReset = true;
          }
        } else {
          for (i = 0; i < rough_B.nA; i++) {
            rough_B.Rhs[i + 13] = b[rough_B.iC[i] - 1];
            for (rough_B.kDrop = i + 1; rough_B.kDrop <= rough_B.nA;
                 rough_B.kDrop++) {
              U_tmp = (13 * i + rough_B.kDrop) - 1;
              rough_B.U[U_tmp] = 0.0;
              for (iSave = 0; iSave < rough_B.nA; iSave++) {
                rough_B.U[U_tmp] += rough_B.RLinv[(13 * iSave + rough_B.kDrop) -
                  1] * rough_B.RLinv[13 * iSave + i];
              }

              rough_B.U[i + 13 * (rough_B.kDrop - 1)] = rough_B.U[U_tmp];
            }
          }

          for (rough_B.kDrop = 0; rough_B.kDrop < 13; rough_B.kDrop++) {
            rough_B.Xnorm0 = 0.0;
            for (i = 0; i < 13; i++) {
              rough_B.Xnorm0 += rough_B.b_H[13 * i + rough_B.kDrop] *
                rough_B.Rhs[i];
            }

            rough_B.Opt[rough_B.kDrop] = rough_B.Xnorm0;
            for (i = 0; i < rough_B.nA; i++) {
              rough_B.Opt[rough_B.kDrop] += rough_B.D[13 * i + rough_B.kDrop] *
                rough_B.Rhs[i + 13];
            }
          }

          rough_B.Xnorm0 = -1.0E-12;
          rough_B.kDrop = -1;
          for (iSave = 0; iSave < rough_B.nA; iSave++) {
            rough_B.cMin = 0.0;
            for (i = 0; i < 13; i++) {
              rough_B.cMin += rough_B.D[13 * iSave + i] * rough_B.Rhs[i];
            }

            rough_B.Opt[iSave + 13] = rough_B.cMin;
            for (i = 0; i < rough_B.nA; i++) {
              rough_B.Opt[iSave + 13] += rough_B.U[13 * i + iSave] *
                rough_B.Rhs[i + 13];
            }

            rough_B.cMin = rough_B.Opt[iSave + 13];
            lambda[rough_B.iC[iSave] - 1] = rough_B.cMin;
            if ((rough_B.cMin < rough_B.Xnorm0) && (iSave + 1 <= rough_B.nA)) {
              rough_B.kDrop = iSave;
              rough_B.Xnorm0 = rough_B.cMin;
            }
          }

          if (rough_B.kDrop + 1 <= 0) {
            DualFeasible = true;
            memcpy(&x[0], &rough_B.Opt[0], 13U * sizeof(real_T));
          } else {
            (*status)++;
            if (rough_B.tmp <= 5) {
              i = 5;
            } else {
              i = rough_B.tmp;
            }

            if (*status > i) {
              rough_B.nA = 0;
              memset(&rough_B.iC[0], 0, 48U * sizeof(int32_T));
              for (i = 0; i < 48; i++) {
                iA[i] = false;
              }

              ColdReset = true;
            } else {
              lambda[rough_B.iC[rough_B.kDrop] - 1] = 0.0;
              rough_DropConstraint(rough_B.kDrop + 1, iA, &rough_B.nA,
                                   rough_B.iC);
            }
          }
        }
      } else {
        if (rough_B.nA <= 0) {
          memset(&lambda[0], 0, 48U * sizeof(real_T));
          for (rough_B.tmp = 0; rough_B.tmp < 13; rough_B.tmp++) {
            rough_B.Xnorm0 = 0.0;
            for (i = 0; i < 13; i++) {
              rough_B.Xnorm0 += -b_Hinv[13 * i + rough_B.tmp] * f[i];
            }

            x[rough_B.tmp] = rough_B.Xnorm0;
          }
        }

        exitg3 = 1;
      }
    } while (exitg3 == 0);

    if (exitg3 == 1) {
      guard1 = true;
    }
  } else {
    for (rough_B.tmp = 0; rough_B.tmp < 13; rough_B.tmp++) {
      rough_B.Xnorm0 = 0.0;
      for (i = 0; i < 13; i++) {
        rough_B.Xnorm0 += -b_Hinv[13 * i + rough_B.tmp] * f[i];
      }

      x[rough_B.tmp] = rough_B.Xnorm0;
    }

    guard1 = true;
  }

  if (guard1) {
    rough_B.Xnorm0 = rough_norm(x);
    exitg2 = false;
    while ((!exitg2) && (*status <= maxiter)) {
      rough_B.cMin = -FeasTol;
      rough_B.tmp = -1;
      for (rough_B.kDrop = 0; rough_B.kDrop < 48; rough_B.kDrop++) {
        if (!cTolComputed) {
          for (i = 0; i < 13; i++) {
            rough_B.z[i] = fabs(b_Ac[48 * i + rough_B.kDrop] * x[i]);
          }

          rough_B.cVal = rough_maximum(rough_B.z);
          if ((rough_B.cTol[rough_B.kDrop] >= rough_B.cVal) || rtIsNaN
              (rough_B.cVal)) {
          } else {
            rough_B.cTol[rough_B.kDrop] = rough_B.cVal;
          }
        }

        if (!iA[rough_B.kDrop]) {
          rough_B.cVal = 0.0;
          for (i = 0; i < 13; i++) {
            rough_B.cVal += b_Ac[48 * i + rough_B.kDrop] * x[i];
          }

          rough_B.cVal = (rough_B.cVal - b[rough_B.kDrop]) /
            rough_B.cTol[rough_B.kDrop];
          if (rough_B.cVal < rough_B.cMin) {
            rough_B.cMin = rough_B.cVal;
            rough_B.tmp = rough_B.kDrop;
          }
        }
      }

      cTolComputed = true;
      if (rough_B.tmp + 1 <= 0) {
        exitg2 = true;
      } else if (*status == maxiter) {
        *status = 0;
        exitg2 = true;
      } else {
        do {
          exitg1 = 0;
          if ((rough_B.tmp + 1 > 0) && (*status <= maxiter)) {
            guard2 = false;
            if (rough_B.nA == 0) {
              for (i = 0; i < 13; i++) {
                rough_B.cMin = 0.0;
                for (rough_B.kDrop = 0; rough_B.kDrop < 13; rough_B.kDrop++) {
                  rough_B.cMin += b_Hinv[13 * rough_B.kDrop + i] * b_Ac[48 *
                    rough_B.kDrop + rough_B.tmp];
                }

                rough_B.z[i] = rough_B.cMin;
              }

              guard2 = true;
            } else {
              rough_KWIKfactor(b_Ac, rough_B.iC, rough_B.nA, b_Linv, rough_B.D,
                               rough_B.b_H, rough_degrees, rough_B.RLinv,
                               &rough_B.cMin);
              if (rough_B.cMin <= 0.0) {
                *status = -2;
                exitg1 = 1;
              } else {
                for (i = 0; i < 169; i++) {
                  rough_B.U[i] = -rough_B.b_H[i];
                }

                for (i = 0; i < 13; i++) {
                  rough_B.cMin = 0.0;
                  for (rough_B.kDrop = 0; rough_B.kDrop < 13; rough_B.kDrop++) {
                    rough_B.cMin += rough_B.U[13 * rough_B.kDrop + i] * b_Ac[48 *
                      rough_B.kDrop + rough_B.tmp];
                  }

                  rough_B.z[i] = rough_B.cMin;
                }

                for (rough_B.kDrop = 0; rough_B.kDrop < rough_B.nA;
                     rough_B.kDrop++) {
                  rough_B.cVal = 0.0;
                  for (i = 0; i < 13; i++) {
                    rough_B.cVal += b_Ac[48 * i + rough_B.tmp] * rough_B.D[13 *
                      rough_B.kDrop + i];
                  }

                  rough_B.r[rough_B.kDrop] = rough_B.cVal;
                }

                guard2 = true;
              }
            }

            if (guard2) {
              rough_B.kDrop = 0;
              rough_B.cMin = 0.0;
              DualFeasible = true;
              ColdReset = true;
              if (rough_B.nA > 0) {
                i = 0;
                exitg4 = false;
                while ((!exitg4) && (i <= rough_B.nA - 1)) {
                  if (rough_B.r[i] >= 1.0E-12) {
                    ColdReset = false;
                    exitg4 = true;
                  } else {
                    i++;
                  }
                }
              }

              if ((rough_B.nA != 0) && (!ColdReset)) {
                for (i = 0; i < rough_B.nA; i++) {
                  rough_B.cVal = rough_B.r[i];
                  if (rough_B.cVal > 1.0E-12) {
                    rough_B.cVal = lambda[rough_B.iC[i] - 1] / rough_B.cVal;
                    if ((rough_B.kDrop == 0) || (rough_B.cVal < rough_B.rMin)) {
                      rough_B.rMin = rough_B.cVal;
                      rough_B.kDrop = i + 1;
                    }
                  }
                }

                if (rough_B.kDrop > 0) {
                  rough_B.cMin = rough_B.rMin;
                  DualFeasible = false;
                }
              }

              rough_B.zTa = 0.0;
              for (i = 0; i < 13; i++) {
                rough_B.zTa += b_Ac[48 * i + rough_B.tmp] * rough_B.z[i];
              }

              if (rough_B.zTa <= 0.0) {
                rough_B.cVal = 0.0;
                ColdReset = true;
              } else {
                rough_B.cVal = 0.0;
                for (i = 0; i < 13; i++) {
                  rough_B.cVal += b_Ac[48 * i + rough_B.tmp] * x[i];
                }

                rough_B.cVal = (b[rough_B.tmp] - rough_B.cVal) / rough_B.zTa;
                ColdReset = false;
              }

              if (DualFeasible && ColdReset) {
                *status = -1;
                exitg1 = 1;
              } else {
                if (ColdReset) {
                  rough_B.zTa = rough_B.cMin;
                } else if (DualFeasible) {
                  rough_B.zTa = rough_B.cVal;
                } else if (rough_B.cMin < rough_B.cVal) {
                  rough_B.zTa = rough_B.cMin;
                } else {
                  rough_B.zTa = rough_B.cVal;
                }

                for (i = 0; i < rough_B.nA; i++) {
                  iSave = rough_B.iC[i];
                  lambda[iSave - 1] -= rough_B.zTa * rough_B.r[i];
                  if ((iSave <= 48) && (lambda[iSave - 1] < 0.0)) {
                    lambda[iSave - 1] = 0.0;
                  }
                }

                lambda[rough_B.tmp] += rough_B.zTa;
                frexp(1.0, &exponent);
                if (fabs(rough_B.zTa - rough_B.cMin) < 2.2204460492503131E-16) {
                  rough_DropConstraint(rough_B.kDrop, iA, &rough_B.nA,
                                       rough_B.iC);
                }

                if (!ColdReset) {
                  for (i = 0; i < 13; i++) {
                    x[i] += rough_B.zTa * rough_B.z[i];
                  }

                  frexp(1.0, &b_exponent);
                  if (fabs(rough_B.zTa - rough_B.cVal) < 2.2204460492503131E-16)
                  {
                    if (rough_B.nA == rough_degrees) {
                      *status = -1;
                      exitg1 = 1;
                    } else {
                      rough_B.nA++;
                      rough_B.iC[rough_B.nA - 1] = rough_B.tmp + 1;
                      rough_B.kDrop = rough_B.nA - 1;
                      exitg4 = false;
                      while ((!exitg4) && (rough_B.kDrop + 1 > 1)) {
                        i = rough_B.iC[rough_B.kDrop - 1];
                        if (rough_B.iC[rough_B.kDrop] > i) {
                          exitg4 = true;
                        } else {
                          iSave = rough_B.iC[rough_B.kDrop];
                          rough_B.iC[rough_B.kDrop] = i;
                          rough_B.iC[rough_B.kDrop - 1] = iSave;
                          rough_B.kDrop--;
                        }
                      }

                      iA[rough_B.tmp] = true;
                      rough_B.tmp = -1;
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
            rough_B.cMin = rough_norm(x);
            if (fabs(rough_B.cMin - rough_B.Xnorm0) > 0.001) {
              rough_B.Xnorm0 = rough_B.cMin;
              for (i = 0; i < 48; i++) {
                rough_B.cMin = fabs(b[i]);
                if (rough_B.cMin >= 1.0) {
                  rough_B.cTol[i] = rough_B.cMin;
                } else {
                  rough_B.cTol[i] = 1.0;
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
void rough_step(void)
{
  static const int8_T a[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

  static const real_T b_a[16] = { 0.095124921972503731, 0.0, 0.0, 0.0, 0.0,
    0.095124921972503731, 0.0, 0.0, 0.0, 0.0, 0.095124921972503731, 0.0, 0.0,
    0.0, 0.0, 0.095124921972503731 };

  static const real_T b_Kr[960] = { -0.1, -0.0, -0.0, -0.0, -0.2, -0.0, -0.0,
    -0.0, -0.30000000000000004, -0.0, -0.0, -0.0, -0.4, -0.0, -0.0, -0.0, -0.5,
    -0.0, -0.0, -0.0, -0.6, -0.0, -0.0, -0.0, -0.7, -0.0, -0.0, -0.0,
    -0.79999999999999993, -0.0, -0.0, -0.0, -0.89999999999999991, -0.0, -0.0,
    -0.0, -0.99999999999999989, -0.0, -0.0, -0.0, -1.0999999999999999, -0.0,
    -0.0, -0.0, -1.2, -0.0, -0.0, -0.0, -1.3, -0.0, -0.0, -0.0,
    -1.4000000000000001, -0.0, -0.0, -0.0, -1.5000000000000002, -0.0, -0.0, -0.0,
    -1.6000000000000003, -0.0, -0.0, -0.0, -1.7000000000000004, -0.0, -0.0, -0.0,
    -1.8000000000000005, -0.0, -0.0, -0.0, -1.9000000000000006, -0.0, -0.0, -0.0,
    -2.0000000000000004, -0.0, -0.0, -0.0, -0.0, -0.1, -0.0, -0.0, -0.0, -0.2,
    -0.0, -0.0, -0.0, -0.30000000000000004, -0.0, -0.0, -0.0, -0.4, -0.0, -0.0,
    -0.0, -0.5, -0.0, -0.0, -0.0, -0.6, -0.0, -0.0, -0.0, -0.7, -0.0, -0.0, -0.0,
    -0.79999999999999993, -0.0, -0.0, -0.0, -0.89999999999999991, -0.0, -0.0,
    -0.0, -0.99999999999999989, -0.0, -0.0, -0.0, -1.0999999999999999, -0.0,
    -0.0, -0.0, -1.2, -0.0, -0.0, -0.0, -1.3, -0.0, -0.0, -0.0,
    -1.4000000000000001, -0.0, -0.0, -0.0, -1.5000000000000002, -0.0, -0.0, -0.0,
    -1.6000000000000003, -0.0, -0.0, -0.0, -1.7000000000000004, -0.0, -0.0, -0.0,
    -1.8000000000000005, -0.0, -0.0, -0.0, -1.9000000000000006, -0.0, -0.0, -0.0,
    -2.0000000000000004, -0.0, -0.0, -0.0, -0.0, -0.1, -0.0, -0.0, -0.0, -0.2,
    -0.0, -0.0, -0.0, -0.30000000000000004, -0.0, -0.0, -0.0, -0.4, -0.0, -0.0,
    -0.0, -0.5, -0.0, -0.0, -0.0, -0.6, -0.0, -0.0, -0.0, -0.7, -0.0, -0.0, -0.0,
    -0.79999999999999993, -0.0, -0.0, -0.0, -0.89999999999999991, -0.0, -0.0,
    -0.0, -0.99999999999999989, -0.0, -0.0, -0.0, -1.0999999999999999, -0.0,
    -0.0, -0.0, -1.2, -0.0, -0.0, -0.0, -1.3, -0.0, -0.0, -0.0,
    -1.4000000000000001, -0.0, -0.0, -0.0, -1.5000000000000002, -0.0, -0.0, -0.0,
    -1.6000000000000003, -0.0, -0.0, -0.0, -1.7000000000000004, -0.0, -0.0, -0.0,
    -1.8000000000000005, -0.0, -0.0, -0.0, -1.9000000000000006, -0.0, -0.0, -0.0,
    -2.0000000000000004, -0.0, -0.0, -0.0, -0.0, -0.1, -0.0, -0.0, -0.0, -0.2,
    -0.0, -0.0, -0.0, -0.30000000000000004, -0.0, -0.0, -0.0, -0.4, -0.0, -0.0,
    -0.0, -0.5, -0.0, -0.0, -0.0, -0.6, -0.0, -0.0, -0.0, -0.7, -0.0, -0.0, -0.0,
    -0.79999999999999993, -0.0, -0.0, -0.0, -0.89999999999999991, -0.0, -0.0,
    -0.0, -0.99999999999999989, -0.0, -0.0, -0.0, -1.0999999999999999, -0.0,
    -0.0, -0.0, -1.2, -0.0, -0.0, -0.0, -1.3, -0.0, -0.0, -0.0,
    -1.4000000000000001, -0.0, -0.0, -0.0, -1.5000000000000002, -0.0, -0.0, -0.0,
    -1.6000000000000003, -0.0, -0.0, -0.0, -1.7000000000000004, -0.0, -0.0, -0.0,
    -1.8000000000000005, -0.0, -0.0, -0.0, -1.9000000000000006, -0.0, -0.0, -0.0,
    -2.0000000000000004, -0.0, -0.0, -0.0, -0.0, -0.1, -0.0, -0.0, -0.0, -0.2,
    -0.0, -0.0, -0.0, -0.30000000000000004, -0.0, -0.0, -0.0, -0.4, -0.0, -0.0,
    -0.0, -0.5, -0.0, -0.0, -0.0, -0.6, -0.0, -0.0, -0.0, -0.7, -0.0, -0.0, -0.0,
    -0.79999999999999993, -0.0, -0.0, -0.0, -0.89999999999999991, -0.0, -0.0,
    -0.0, -0.99999999999999989, -0.0, -0.0, -0.0, -1.0999999999999999, -0.0,
    -0.0, -0.0, -1.2, -0.0, -0.0, -0.0, -1.3, -0.0, -0.0, -0.0,
    -1.4000000000000001, -0.0, -0.0, -0.0, -1.5000000000000002, -0.0, -0.0, -0.0,
    -1.6000000000000003, -0.0, -0.0, -0.0, -1.7000000000000004, -0.0, -0.0, -0.0,
    -1.8000000000000005, -0.0, -0.0, -0.0, -1.9000000000000006, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.1, -0.0, -0.0, -0.0, -0.2, -0.0, -0.0, -0.0,
    -0.30000000000000004, -0.0, -0.0, -0.0, -0.4, -0.0, -0.0, -0.0, -0.5, -0.0,
    -0.0, -0.0, -0.6, -0.0, -0.0, -0.0, -0.7, -0.0, -0.0, -0.0,
    -0.79999999999999993, -0.0, -0.0, -0.0, -0.89999999999999991, -0.0, -0.0,
    -0.0, -0.99999999999999989, -0.0, -0.0, -0.0, -1.0999999999999999, -0.0,
    -0.0, -0.0, -1.2, -0.0, -0.0, -0.0, -1.3, -0.0, -0.0, -0.0,
    -1.4000000000000001, -0.0, -0.0, -0.0, -1.5000000000000002, -0.0, -0.0, -0.0,
    -1.6000000000000003, -0.0, -0.0, -0.0, -1.7000000000000004, -0.0, -0.0, -0.0,
    -1.8000000000000005, -0.0, -0.0, -0.0, -1.9000000000000006, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.1, -0.0, -0.0, -0.0, -0.2, -0.0, -0.0, -0.0,
    -0.30000000000000004, -0.0, -0.0, -0.0, -0.4, -0.0, -0.0, -0.0, -0.5, -0.0,
    -0.0, -0.0, -0.6, -0.0, -0.0, -0.0, -0.7, -0.0, -0.0, -0.0,
    -0.79999999999999993, -0.0, -0.0, -0.0, -0.89999999999999991, -0.0, -0.0,
    -0.0, -0.99999999999999989, -0.0, -0.0, -0.0, -1.0999999999999999, -0.0,
    -0.0, -0.0, -1.2, -0.0, -0.0, -0.0, -1.3, -0.0, -0.0, -0.0,
    -1.4000000000000001, -0.0, -0.0, -0.0, -1.5000000000000002, -0.0, -0.0, -0.0,
    -1.6000000000000003, -0.0, -0.0, -0.0, -1.7000000000000004, -0.0, -0.0, -0.0,
    -1.8000000000000005, -0.0, -0.0, -0.0, -1.9000000000000006, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.1, -0.0, -0.0, -0.0, -0.2, -0.0, -0.0, -0.0,
    -0.30000000000000004, -0.0, -0.0, -0.0, -0.4, -0.0, -0.0, -0.0, -0.5, -0.0,
    -0.0, -0.0, -0.6, -0.0, -0.0, -0.0, -0.7, -0.0, -0.0, -0.0,
    -0.79999999999999993, -0.0, -0.0, -0.0, -0.89999999999999991, -0.0, -0.0,
    -0.0, -0.99999999999999989, -0.0, -0.0, -0.0, -1.0999999999999999, -0.0,
    -0.0, -0.0, -1.2, -0.0, -0.0, -0.0, -1.3, -0.0, -0.0, -0.0,
    -1.4000000000000001, -0.0, -0.0, -0.0, -1.5000000000000002, -0.0, -0.0, -0.0,
    -1.6000000000000003, -0.0, -0.0, -0.0, -1.7000000000000004, -0.0, -0.0, -0.0,
    -1.8000000000000005, -0.0, -0.0, -0.0, -1.9000000000000006, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.1, -0.0, -0.0, -0.0, -0.2, -0.0, -0.0, -0.0,
    -0.30000000000000004, -0.0, -0.0, -0.0, -0.4, -0.0, -0.0, -0.0, -0.5, -0.0,
    -0.0, -0.0, -0.6, -0.0, -0.0, -0.0, -0.7, -0.0, -0.0, -0.0,
    -0.79999999999999993, -0.0, -0.0, -0.0, -0.89999999999999991, -0.0, -0.0,
    -0.0, -0.99999999999999989, -0.0, -0.0, -0.0, -1.0999999999999999, -0.0,
    -0.0, -0.0, -1.2, -0.0, -0.0, -0.0, -1.3, -0.0, -0.0, -0.0,
    -1.4000000000000001, -0.0, -0.0, -0.0, -1.5000000000000002, -0.0, -0.0, -0.0,
    -1.6000000000000003, -0.0, -0.0, -0.0, -1.7000000000000004, -0.0, -0.0, -0.0,
    -1.8000000000000005, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.1, -0.0, -0.0, -0.0, -0.2, -0.0, -0.0, -0.0,
    -0.30000000000000004, -0.0, -0.0, -0.0, -0.4, -0.0, -0.0, -0.0, -0.5, -0.0,
    -0.0, -0.0, -0.6, -0.0, -0.0, -0.0, -0.7, -0.0, -0.0, -0.0,
    -0.79999999999999993, -0.0, -0.0, -0.0, -0.89999999999999991, -0.0, -0.0,
    -0.0, -0.99999999999999989, -0.0, -0.0, -0.0, -1.0999999999999999, -0.0,
    -0.0, -0.0, -1.2, -0.0, -0.0, -0.0, -1.3, -0.0, -0.0, -0.0,
    -1.4000000000000001, -0.0, -0.0, -0.0, -1.5000000000000002, -0.0, -0.0, -0.0,
    -1.6000000000000003, -0.0, -0.0, -0.0, -1.7000000000000004, -0.0, -0.0, -0.0,
    -1.8000000000000005, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.1, -0.0, -0.0, -0.0, -0.2, -0.0, -0.0, -0.0,
    -0.30000000000000004, -0.0, -0.0, -0.0, -0.4, -0.0, -0.0, -0.0, -0.5, -0.0,
    -0.0, -0.0, -0.6, -0.0, -0.0, -0.0, -0.7, -0.0, -0.0, -0.0,
    -0.79999999999999993, -0.0, -0.0, -0.0, -0.89999999999999991, -0.0, -0.0,
    -0.0, -0.99999999999999989, -0.0, -0.0, -0.0, -1.0999999999999999, -0.0,
    -0.0, -0.0, -1.2, -0.0, -0.0, -0.0, -1.3, -0.0, -0.0, -0.0,
    -1.4000000000000001, -0.0, -0.0, -0.0, -1.5000000000000002, -0.0, -0.0, -0.0,
    -1.6000000000000003, -0.0, -0.0, -0.0, -1.7000000000000004, -0.0, -0.0, -0.0,
    -1.8000000000000005, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.1, -0.0, -0.0, -0.0, -0.2, -0.0, -0.0, -0.0,
    -0.30000000000000004, -0.0, -0.0, -0.0, -0.4, -0.0, -0.0, -0.0, -0.5, -0.0,
    -0.0, -0.0, -0.6, -0.0, -0.0, -0.0, -0.7, -0.0, -0.0, -0.0,
    -0.79999999999999993, -0.0, -0.0, -0.0, -0.89999999999999991, -0.0, -0.0,
    -0.0, -0.99999999999999989, -0.0, -0.0, -0.0, -1.0999999999999999, -0.0,
    -0.0, -0.0, -1.2, -0.0, -0.0, -0.0, -1.3, -0.0, -0.0, -0.0,
    -1.4000000000000001, -0.0, -0.0, -0.0, -1.5000000000000002, -0.0, -0.0, -0.0,
    -1.6000000000000003, -0.0, -0.0, -0.0, -1.7000000000000004, -0.0, -0.0, -0.0,
    -1.8000000000000005 };

  static const real_T b_Kx[48] = { 21.000000000000004, 0.0, 0.0, 0.0, 0.0,
    21.000000000000004, 0.0, 0.0, 0.0, 0.0, 21.000000000000004, 0.0, 0.0, 0.0,
    0.0, 21.000000000000004, 19.000000000000004, 0.0, 0.0, 0.0, 0.0,
    19.000000000000004, 0.0, 0.0, 0.0, 0.0, 19.000000000000004, 0.0, 0.0, 0.0,
    0.0, 19.000000000000004, 17.1, 0.0, 0.0, 0.0, 0.0, 17.1, 0.0, 0.0, 0.0, 0.0,
    17.1, 0.0, 0.0, 0.0, 0.0, 17.1 };

  static const real_T b_Ku1[48] = { 28.70000000000001, 0.0, 0.0, 0.0, 0.0,
    28.70000000000001, 0.0, 0.0, 0.0, 0.0, 28.70000000000001, 0.0, 0.0, 0.0, 0.0,
    28.70000000000001, 26.600000000000005, 0.0, 0.0, 0.0, 0.0,
    26.600000000000005, 0.0, 0.0, 0.0, 0.0, 26.600000000000005, 0.0, 0.0, 0.0,
    0.0, 26.600000000000005, 24.510000000000005, 0.0, 0.0, 0.0, 0.0,
    24.510000000000005, 0.0, 0.0, 0.0, 0.0, 24.510000000000005, 0.0, 0.0, 0.0,
    0.0, 24.510000000000005 };

  static const real_T b_Linv[169] = { 0.18663083698528471, 0.0, 0.0, 0.0,
    -3.6360549438631788, 0.0, 0.0, 0.0, 2.0706843241079604, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.18663083698528471, 0.0, 0.0, 0.0, -3.6360549438631788, 0.0, 0.0, 0.0,
    2.0706843241079635, 0.0, 0.0, 0.0, 0.0, 0.0, 0.18663083698528471, 0.0, 0.0,
    0.0, -3.6360549438631788, 0.0, 0.0, 0.0, 2.0706843241079707, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.18663083698528471, 0.0, 0.0, 0.0, -3.6360549438631788, 0.0, 0.0,
    0.0, 2.0706843241079635, 0.0, 0.0, 0.0, 0.0, 0.0, 3.9244788510643569, 0.0,
    0.0, 0.0, -6.4796924427883935, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    3.9244788510643569, 0.0, 0.0, 0.0, -6.4796924427884033, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 3.9244788510643569, 0.0, 0.0, 0.0,
    -6.4796924427884255, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    3.9244788510643569, 0.0, 0.0, 0.0, -6.4796924427884033, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 4.6067104052644519, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.606710405264459, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.606710405264475, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.606710405264459,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.003162277660168379 };

  static const real_T b_Hinv[169] = { 17.543460194212134, 0.0, 0.0, 0.0,
    -27.687018294821787, 0.0, 0.0, 0.0, 9.53904302188613, 0.0, 0.0, 0.0, 0.0,
    0.0, 17.543460194212145, 0.0, 0.0, 0.0, -27.687018294821826, 0.0, 0.0, 0.0,
    9.53904302188616, 0.0, 0.0, 0.0, 0.0, 0.0, 17.543460194212177, 0.0, 0.0, 0.0,
    -27.687018294821918, 0.0, 0.0, 0.0, 9.5390430218862257, 0.0, 0.0, 0.0, 0.0,
    0.0, 17.543460194212145, 0.0, 0.0, 0.0, -27.687018294821826, 0.0, 0.0, 0.0,
    9.53904302188616, 0.0, -27.687018294821787, 0.0, 0.0, 0.0,
    57.387948405580431, 0.0, 0.0, 0.0, -29.850066599106725, 0.0, 0.0, 0.0, 0.0,
    0.0, -27.687018294821826, 0.0, 0.0, 0.0, 57.387948405580559, 0.0, 0.0, 0.0,
    -29.850066599106817, 0.0, 0.0, 0.0, 0.0, 0.0, -27.687018294821918, 0.0, 0.0,
    0.0, 57.38794840558085, 0.0, 0.0, 0.0, -29.850066599107024, 0.0, 0.0, 0.0,
    0.0, 0.0, -27.687018294821826, 0.0, 0.0, 0.0, 57.387948405580559, 0.0, 0.0,
    0.0, -29.850066599106817, 0.0, 9.53904302188613, 0.0, 0.0, 0.0,
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
    -0.0, -0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0,
    -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -0.0, -1.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0,
    -0.0, -0.0, -1.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    1.0, 0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0,
    -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, -0.0, -0.0, -0.0, -1.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0,
    -1.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0,
    -0.0, -0.0, -1.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    1.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -1.0, -0.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0,
    -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -1.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0,
    -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  static const int8_T c_a[192] = { -1, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, 1, 0,
    0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, 1, 0, 0, 0,
    1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, 1, 0, 0, 0, 1, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static const real_T e_a[16] = { 0.1, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
    0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.1 };

  static const real_T f_a[16] = { 0.095124921972503731, 0.0, -0.0, 0.0, -0.0,
    0.095124921972503731, -0.0, 0.0, -0.0, 0.0, 0.095124921972503731, 0.0, -0.0,
    0.0, -0.0, 0.095124921972503731 };

  if (rtmIsMajorTimeStep(rough_M)) {
    // set solver stop time
    rtsiSetSolverStopTime(&rough_M->solverInfo,((rough_M->Timing.clockTick0+1)*
      rough_M->Timing.stepSize0));
  }                                    // end MajorTimeStep

  // Update absolute time of base rate at minor time step
  if (rtmIsMinorTimeStep(rough_M)) {
    rough_M->Timing.t[0] = rtsiGetT(&rough_M->solverInfo);
  }

  // Reset subsysRan breadcrumbs
  srClearBC(rough_DW.CommandVelocityPublisher_Subsys);

  // Reset subsysRan breadcrumbs
  srClearBC(rough_DW.MPCController_SubsysRanBC);

  // Outputs for Enabled SubSystem: '<Root>/MPC Controller' incorporates:
  //   EnablePort: '<S2>/Enable'

  if ((rtmIsMajorTimeStep(rough_M) &&
       rough_M->Timing.TaskCounters.TID[1] == 0) && rtsiIsModeUpdateTimeStep
      (&rough_M->solverInfo)) {
    // Constant: '<Root>/Constant1'
    if (rough_P.Constant1_Value_g > 0.0) {
      if (!rough_DW.MPCController_MODE) {
        (void) memset(&(rough_XDis.position), 0,
                      4*sizeof(boolean_T));
        rough_DW.MPCController_MODE = true;
      }
    } else {
      if (((rough_M->Timing.clockTick1) * 0.033333333333333333) == rtmGetTStart
          (rough_M)) {
        (void) memset(&(rough_XDis.position), 1,
                      4*sizeof(boolean_T));
      }

      if (rough_DW.MPCController_MODE) {
        (void) memset(&(rough_XDis.position), 1,
                      4*sizeof(boolean_T));
        rough_DW.MPCController_MODE = false;
      }
    }

    // End of Constant: '<Root>/Constant1'
  }

  if (rough_DW.MPCController_MODE) {
    // StateSpace: '<S2>/Plant (State-Space)'
    rough_B.measuredoutputs[0] = 0.0;
    rough_B.measuredoutputs[1] = 0.0;
    rough_B.measuredoutputs[2] = 0.0;
    rough_B.measuredoutputs[3] = 0.0;
    rough_B.ri = rough_P.PlantStateSpace_C_jc[0U];
    while (rough_B.ri < rough_P.PlantStateSpace_C_jc[1U]) {
      rough_B.measuredoutputs[rough_P.PlantStateSpace_C_ir[rough_B.ri]] +=
        rough_P.PlantStateSpace_C_pr[rough_B.ri] * rough_X.position[0U];
      rough_B.ri++;
    }

    rough_B.ri = rough_P.PlantStateSpace_C_jc[1U];
    while (rough_B.ri < rough_P.PlantStateSpace_C_jc[2U]) {
      rough_B.measuredoutputs[rough_P.PlantStateSpace_C_ir[rough_B.ri]] +=
        rough_P.PlantStateSpace_C_pr[rough_B.ri] * rough_X.position[1U];
      rough_B.ri++;
    }

    rough_B.ri = rough_P.PlantStateSpace_C_jc[2U];
    while (rough_B.ri < rough_P.PlantStateSpace_C_jc[3U]) {
      rough_B.measuredoutputs[rough_P.PlantStateSpace_C_ir[rough_B.ri]] +=
        rough_P.PlantStateSpace_C_pr[rough_B.ri] * rough_X.position[2U];
      rough_B.ri++;
    }

    rough_B.ri = rough_P.PlantStateSpace_C_jc[3U];
    while (rough_B.ri < rough_P.PlantStateSpace_C_jc[4U]) {
      rough_B.measuredoutputs[rough_P.PlantStateSpace_C_ir[rough_B.ri]] +=
        rough_P.PlantStateSpace_C_pr[rough_B.ri] * rough_X.position[3U];
      rough_B.ri++;
    }

    // End of StateSpace: '<S2>/Plant (State-Space)'
    if (rtmIsMajorTimeStep(rough_M) &&
        rough_M->Timing.TaskCounters.TID[2] == 0) {
      // Sum: '<S6>/Output' incorporates:
      //   Step: '<S6>/Step'

      if (rough_M->Timing.t[0] < rough_P.Ramp_start) {
        rough_B.rtb_TmpSignalConversionAtSFun_b = rough_P.Step_Y0;
      } else {
        rough_B.rtb_TmpSignalConversionAtSFun_b = rough_P.Ramp_slope;
      }

      // SignalConversion generated from: '<S28>/ SFunction ' incorporates:
      //   Clock: '<S6>/Clock'
      //   Constant: '<S6>/Constant'
      //   Constant: '<S6>/Constant1'
      //   MATLAB Function: '<S27>/optimizer'
      //   Product: '<S6>/Product'
      //   Step: '<S6>/Step'
      //   Sum: '<S6>/Output'
      //   Sum: '<S6>/Sum'

      rough_B.TmpSignalConversionAtSFunct[0] = (rough_M->Timing.t[0] -
        rough_P.Ramp_start) * rough_B.rtb_TmpSignalConversionAtSFun_b +
        rough_P.Ramp_InitialOutput;

      // MATLAB Function: '<S27>/optimizer' incorporates:
      //   Constant: '<S2>/Constant'
      //   Constant: '<S2>/Constant1'
      //   Constant: '<S2>/Constant2'
      //   Memory: '<S7>/Memory'
      //   Memory: '<S7>/last_x'
      //   UnitDelay: '<S7>/last_mv'

      for (rough_B.i = 0; rough_B.i < 20; rough_B.i++) {
        rough_B.rseq_tmp = rough_B.i << 2;
        rough_B.rseq[rough_B.rseq_tmp] = rough_B.TmpSignalConversionAtSFunct[0];
        rough_B.rseq[rough_B.rseq_tmp + 1] = rough_P.Constant_Value_p;
        rough_B.rseq[rough_B.rseq_tmp + 2] = rough_P.Constant1_Value;
        rough_B.rseq[rough_B.rseq_tmp + 3] = rough_P.Constant2_Value;
      }

      rough_B.old_u[0] = rough_DW.last_mv_DSTATE[0];
      rough_B.TmpSignalConversionAtSFunct[0] = rough_DW.last_x_PreviousInput[0];
      rough_B.old_u[1] = rough_DW.last_mv_DSTATE[1];
      rough_B.TmpSignalConversionAtSFunct[1] = rough_DW.last_x_PreviousInput[1];
      rough_B.old_u[2] = rough_DW.last_mv_DSTATE[2];
      rough_B.TmpSignalConversionAtSFunct[2] = rough_DW.last_x_PreviousInput[2];
      rough_B.old_u[3] = rough_DW.last_mv_DSTATE[3];
      rough_B.TmpSignalConversionAtSFunct[3] = rough_DW.last_x_PreviousInput[3];
      rough_B.rtb_TmpSignalConversionAtSFun_b = rough_DW.last_x_PreviousInput[1];
      rough_B.rtb_TmpSignalConversionAtSFun_p = rough_DW.last_x_PreviousInput[0];
      rough_B.rtb_TmpSignalConversionAtSFun_c = rough_DW.last_x_PreviousInput[2];
      rough_B.rtb_TmpSignalConversionAtSFun_f = rough_DW.last_x_PreviousInput[3];
      for (rough_B.rseq_tmp = 0; rough_B.rseq_tmp < 4; rough_B.rseq_tmp++) {
        rough_B.y_innov[rough_B.rseq_tmp] =
          rough_B.measuredoutputs[rough_B.rseq_tmp] - (((static_cast<real_T>
          (a[rough_B.rseq_tmp + 4]) * rough_B.rtb_TmpSignalConversionAtSFun_b +
          static_cast<real_T>(a[rough_B.rseq_tmp]) *
          rough_B.rtb_TmpSignalConversionAtSFun_p) + static_cast<real_T>
          (a[rough_B.rseq_tmp + 8]) * rough_B.rtb_TmpSignalConversionAtSFun_c) +
          static_cast<real_T>(a[rough_B.rseq_tmp + 12]) *
          rough_B.rtb_TmpSignalConversionAtSFun_f);
      }

      rough_B.rtb_TmpSignalConversionAtSFun_p = rough_B.y_innov[1];
      rough_B.rtb_TmpSignalConversionAtSFun_c = rough_B.y_innov[0];
      rough_B.rtb_TmpSignalConversionAtSFun_f = rough_B.y_innov[2];
      rough_B.y_innov_c = rough_B.y_innov[3];
      for (rough_B.rseq_tmp = 0; rough_B.rseq_tmp < 4; rough_B.rseq_tmp++) {
        rough_B.xest[rough_B.rseq_tmp] = (((b_a[rough_B.rseq_tmp + 4] *
          rough_B.rtb_TmpSignalConversionAtSFun_p + b_a[rough_B.rseq_tmp] *
          rough_B.rtb_TmpSignalConversionAtSFun_c) + b_a[rough_B.rseq_tmp + 8] *
          rough_B.rtb_TmpSignalConversionAtSFun_f) + b_a[rough_B.rseq_tmp + 12] *
          rough_B.y_innov_c) +
          rough_B.TmpSignalConversionAtSFunct[rough_B.rseq_tmp];
      }

      memset(&rough_B.f[0], 0, 13U * sizeof(real_T));
      for (rough_B.i = 0; rough_B.i < 12; rough_B.i++) {
        rough_B.rtb_TmpSignalConversionAtSFun_b = 0.0;
        for (rough_B.rseq_tmp = 0; rough_B.rseq_tmp < 80; rough_B.rseq_tmp++) {
          rough_B.rtb_TmpSignalConversionAtSFun_b += b_Kr[80 * rough_B.i +
            rough_B.rseq_tmp] * rough_B.rseq[rough_B.rseq_tmp];
        }

        rough_B.rseq_tmp = rough_B.i << 2;
        rough_B.f[rough_B.i] = ((((b_Kx[rough_B.rseq_tmp + 1] * rough_B.xest[1]
          + b_Kx[rough_B.rseq_tmp] * rough_B.xest[0]) + b_Kx[rough_B.rseq_tmp +
          2] * rough_B.xest[2]) + b_Kx[rough_B.rseq_tmp + 3] * rough_B.xest[3])
          + rough_B.rtb_TmpSignalConversionAtSFun_b) + (((b_Ku1[rough_B.rseq_tmp
          + 1] * rough_B.old_u[1] + b_Ku1[rough_B.rseq_tmp] * rough_B.old_u[0])
          + b_Ku1[rough_B.rseq_tmp + 2] * rough_B.old_u[2]) +
          b_Ku1[rough_B.rseq_tmp + 3] * rough_B.old_u[3]);
      }

      rough_B.rtb_TmpSignalConversionAtSFun_b = rough_B.xest[0];
      rough_B.rtb_TmpSignalConversionAtSFun_p = rough_B.xest[1];
      rough_B.rtb_TmpSignalConversionAtSFun_c = rough_B.xest[2];
      rough_B.rtb_TmpSignalConversionAtSFun_f = rough_B.xest[3];
      rough_B.y_innov_c = rough_B.old_u[1];
      rough_B.old_u_m = rough_B.old_u[0];
      rough_B.old_u_c = rough_B.old_u[2];
      rough_B.old_u_k = rough_B.old_u[3];
      for (rough_B.i = 0; rough_B.i < 48; rough_B.i++) {
        rough_B.iAout[rough_B.i] = rough_DW.Memory_PreviousInput[rough_B.i];
        rough_B.dv[rough_B.i] = -(((((0.0 *
          rough_B.rtb_TmpSignalConversionAtSFun_b + 0.0 *
          rough_B.rtb_TmpSignalConversionAtSFun_p) + 0.0 *
          rough_B.rtb_TmpSignalConversionAtSFun_c) + 0.0 *
          rough_B.rtb_TmpSignalConversionAtSFun_f) + 1.0) +
          (((static_cast<real_T>(c_a[rough_B.i + 48]) * rough_B.y_innov_c +
             static_cast<real_T>(c_a[rough_B.i]) * rough_B.old_u_m) +
            static_cast<real_T>(c_a[rough_B.i + 96]) * rough_B.old_u_c) +
           static_cast<real_T>(c_a[rough_B.i + 144]) * rough_B.old_u_k));
      }

      rough_qpkwik(b_Linv, b_Hinv, rough_B.f, b_Ac, rough_B.dv, rough_B.iAout,
                   244, 1.0E-6, rough_B.zopt, rough_B.a__1, &rough_B.i);
      if ((rough_B.i < 0) || (rough_B.i == 0)) {
        memset(&rough_B.zopt[0], 0, 13U * sizeof(real_T));
      }

      rough_B.rtb_TmpSignalConversionAtSFun_b = rough_DW.last_x_PreviousInput[1];
      rough_B.rtb_TmpSignalConversionAtSFun_p = rough_DW.last_x_PreviousInput[0];
      rough_B.rtb_TmpSignalConversionAtSFun_c = rough_DW.last_x_PreviousInput[2];
      rough_B.rtb_TmpSignalConversionAtSFun_f = rough_DW.last_x_PreviousInput[3];
      for (rough_B.i = 0; rough_B.i < 4; rough_B.i++) {
        rough_B.y_innov_c = rough_B.old_u[rough_B.i] + rough_B.zopt[rough_B.i];
        rough_B.u[rough_B.i] = rough_B.y_innov_c;
        rough_B.xest[rough_B.i] = rough_B.y_innov_c;
        rough_B.TmpSignalConversionAtSFunct[rough_B.i] = ((static_cast<real_T>
          (a[rough_B.i + 4]) * rough_B.rtb_TmpSignalConversionAtSFun_b +
          static_cast<real_T>(a[rough_B.i]) *
          rough_B.rtb_TmpSignalConversionAtSFun_p) + static_cast<real_T>
          (a[rough_B.i + 8]) * rough_B.rtb_TmpSignalConversionAtSFun_c) +
          static_cast<real_T>(a[rough_B.i + 12]) *
          rough_B.rtb_TmpSignalConversionAtSFun_f;
      }

      rough_B.rtb_TmpSignalConversionAtSFun_b = rough_B.xest[1];
      rough_B.old_u_m = rough_B.xest[0];
      rough_B.old_u_c = rough_B.xest[2];
      rough_B.old_u_k = rough_B.xest[3];
      rough_B.rtb_TmpSignalConversionAtSFun_p = rough_B.y_innov[1];
      rough_B.rtb_TmpSignalConversionAtSFun_c = rough_B.y_innov[0];
      rough_B.rtb_TmpSignalConversionAtSFun_f = rough_B.y_innov[2];
      rough_B.y_innov_c = rough_B.y_innov[3];
      for (rough_B.i = 0; rough_B.i < 4; rough_B.i++) {
        rough_B.xk1[rough_B.i] = ((((e_a[rough_B.i + 4] *
          rough_B.rtb_TmpSignalConversionAtSFun_b + e_a[rough_B.i] *
          rough_B.old_u_m) + e_a[rough_B.i + 8] * rough_B.old_u_c) +
          e_a[rough_B.i + 12] * rough_B.old_u_k) +
          rough_B.TmpSignalConversionAtSFunct[rough_B.i]) + (((f_a[rough_B.i + 4]
          * rough_B.rtb_TmpSignalConversionAtSFun_p + f_a[rough_B.i] *
          rough_B.rtb_TmpSignalConversionAtSFun_c) + f_a[rough_B.i + 8] *
          rough_B.rtb_TmpSignalConversionAtSFun_f) + f_a[rough_B.i + 12] *
          rough_B.y_innov_c);

        // Gain: '<S7>/umin_scale1'
        rough_B.umin_scale1[rough_B.i] = rough_P.umin_scale1_Gain[rough_B.i] *
          rough_B.u[rough_B.i];
      }
    }

    if (rtmIsMajorTimeStep(rough_M) &&
        rough_M->Timing.TaskCounters.TID[1] == 0) {
    }

    if (rtmIsMajorTimeStep(rough_M) &&
        rough_M->Timing.TaskCounters.TID[2] == 0) {
      // SignalConversion generated from: '<S2>/mv_vel_y'
      rough_B.mv_vel_y = rough_B.umin_scale1[1];

      // SignalConversion generated from: '<S2>/mv_vel_yaw'
      rough_B.mv_vel_yaw = rough_B.umin_scale1[3];

      // SignalConversion generated from: '<S2>/mv_vel_z'
      rough_B.mv_vel_z = rough_B.umin_scale1[2];
    }

    if (rtsiIsModeUpdateTimeStep(&rough_M->solverInfo)) {
      srUpdateBC(rough_DW.MPCController_SubsysRanBC);
    }
  }

  // End of Outputs for SubSystem: '<Root>/MPC Controller'
  if (rtmIsMajorTimeStep(rough_M) &&
      rough_M->Timing.TaskCounters.TID[2] == 0) {
    // Outputs for Enabled SubSystem: '<Root>/Command Velocity Publisher' incorporates:
    //   EnablePort: '<S1>/Enable'

    // Constant: '<Root>/Constant'
    if (rough_P.Constant_Value_pj > 0.0) {
      // BusAssignment: '<S1>/Bus Assignment1' incorporates:
      //   Constant: '<S1>/Coordinate Frame'
      //   Constant: '<S1>/Type Mask'
      //   Constant: '<S3>/Constant'
      //   DataTypeConversion: '<S1>/Data Type Conversion'

      rough_B.BusAssignment1 = rough_P.Constant_Value;
      rough_B.BusAssignment1.CoordinateFrame = rough_P.CoordinateFrame_Value;
      rough_B.BusAssignment1.TypeMask = rough_P.TypeMask_Value;
      rough_B.BusAssignment1.Velocity.X = rough_B.umin_scale1[0];
      rough_B.BusAssignment1.Velocity.Y = rough_B.mv_vel_y;
      rough_B.BusAssignment1.Velocity.Z = rough_B.mv_vel_z;
      rough_B.BusAssignment1.YawRate = static_cast<real32_T>(rough_B.mv_vel_yaw);

      // Outputs for Atomic SubSystem: '<S1>/Publish2'
      // MATLABSystem: '<S4>/SinkBlock'
      Pub_rough_12.publish(&rough_B.BusAssignment1);

      // End of Outputs for SubSystem: '<S1>/Publish2'
      if (rtsiIsModeUpdateTimeStep(&rough_M->solverInfo)) {
        srUpdateBC(rough_DW.CommandVelocityPublisher_Subsys);
      }
    }

    // End of Constant: '<Root>/Constant'
    // End of Outputs for SubSystem: '<Root>/Command Velocity Publisher'
  }

  if (rtmIsMajorTimeStep(rough_M) &&
      rough_M->Timing.TaskCounters.TID[1] == 0) {
    // S-Function (saeroclockpacer): '<Root>/Simulation Pace'
    //
    //  The Clock Pacer generates no code, it is only active in
    //  interpreted simulation.

  }

  // Update for Enabled SubSystem: '<Root>/MPC Controller' incorporates:
  //   EnablePort: '<S2>/Enable'

  if (rough_DW.MPCController_MODE && (rtmIsMajorTimeStep(rough_M) &&
       rough_M->Timing.TaskCounters.TID[2] == 0)) {
    // Update for Memory: '<S7>/Memory'
    for (rough_B.i = 0; rough_B.i < 48; rough_B.i++) {
      rough_DW.Memory_PreviousInput[rough_B.i] = rough_B.iAout[rough_B.i];
    }

    // End of Update for Memory: '<S7>/Memory'

    // Update for UnitDelay: '<S7>/last_mv'
    rough_DW.last_mv_DSTATE[0] = rough_B.u[0];

    // Update for Memory: '<S7>/last_x'
    rough_DW.last_x_PreviousInput[0] = rough_B.xk1[0];

    // Update for UnitDelay: '<S7>/last_mv'
    rough_DW.last_mv_DSTATE[1] = rough_B.u[1];

    // Update for Memory: '<S7>/last_x'
    rough_DW.last_x_PreviousInput[1] = rough_B.xk1[1];

    // Update for UnitDelay: '<S7>/last_mv'
    rough_DW.last_mv_DSTATE[2] = rough_B.u[2];

    // Update for Memory: '<S7>/last_x'
    rough_DW.last_x_PreviousInput[2] = rough_B.xk1[2];

    // Update for UnitDelay: '<S7>/last_mv'
    rough_DW.last_mv_DSTATE[3] = rough_B.u[3];

    // Update for Memory: '<S7>/last_x'
    rough_DW.last_x_PreviousInput[3] = rough_B.xk1[3];
  }

  // End of Update for SubSystem: '<Root>/MPC Controller'
  if (rtmIsMajorTimeStep(rough_M)) {
    // External mode
    rtExtModeUploadCheckTrigger(3);

    {                                  // Sample time: [0.0s, 0.0s]
      rtExtModeUpload(0, (real_T)rough_M->Timing.t[0]);
    }

    if (rtmIsMajorTimeStep(rough_M) &&
        rough_M->Timing.TaskCounters.TID[1] == 0) {
                                  // Sample time: [0.033333333333333333s, 0.0s]
      rtExtModeUpload(1, (real_T)((rough_M->Timing.clockTick1) *
        0.033333333333333333));
    }

    if (rtmIsMajorTimeStep(rough_M) &&
        rough_M->Timing.TaskCounters.TID[2] == 0) {// Sample time: [0.1s, 0.0s]
      rtExtModeUpload(2, (real_T)((rough_M->Timing.clockTick2) * 0.1));
    }
  }                                    // end MajorTimeStep

  if (rtmIsMajorTimeStep(rough_M)) {
    // signal main to stop simulation
    {                                  // Sample time: [0.0s, 0.0s]
      if ((rtmGetTFinal(rough_M)!=-1) &&
          !((rtmGetTFinal(rough_M)-((rough_M->Timing.clockTick1) *
             0.033333333333333333)) > ((rough_M->Timing.clockTick1) *
            0.033333333333333333) * (DBL_EPSILON))) {
        rtmSetErrorStatus(rough_M, "Simulation finished");
      }

      if (rtmGetStopRequested(rough_M)) {
        rtmSetErrorStatus(rough_M, "Simulation finished");
      }
    }

    rt_ertODEUpdateContinuousStates(&rough_M->solverInfo);

    // Update absolute time for base rate
    // The "clockTick0" counts the number of times the code of this task has
    //  been executed. The absolute time is the multiplication of "clockTick0"
    //  and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
    //  overflow during the application lifespan selected.

    ++rough_M->Timing.clockTick0;
    rough_M->Timing.t[0] = rtsiGetSolverStopTime(&rough_M->solverInfo);

    {
      // Update absolute timer for sample time: [0.033333333333333333s, 0.0s]
      // The "clockTick1" counts the number of times the code of this task has
      //  been executed. The resolution of this integer timer is 0.033333333333333333, which is the step size
      //  of the task. Size of "clockTick1" ensures timer will not overflow during the
      //  application lifespan selected.

      rough_M->Timing.clockTick1++;
    }

    if (rtmIsMajorTimeStep(rough_M) &&
        rough_M->Timing.TaskCounters.TID[2] == 0) {
      // Update absolute timer for sample time: [0.1s, 0.0s]
      // The "clockTick2" counts the number of times the code of this task has
      //  been executed. The resolution of this integer timer is 0.1, which is the step size
      //  of the task. Size of "clockTick2" ensures timer will not overflow during the
      //  application lifespan selected.

      rough_M->Timing.clockTick2++;
    }

    rate_scheduler();
  }                                    // end MajorTimeStep
}

// Derivatives for root system: '<Root>'
void rough_derivatives(void)
{
  XDot_rough_T *_rtXdot;
  uint32_T ri;
  _rtXdot = ((XDot_rough_T *) rough_M->derivs);

  // Derivatives for Enabled SubSystem: '<Root>/MPC Controller'
  if (rough_DW.MPCController_MODE) {
    // Derivatives for StateSpace: '<S2>/Plant (State-Space)'
    _rtXdot->position[0] = 0.0;
    _rtXdot->position[1] = 0.0;
    _rtXdot->position[2] = 0.0;
    _rtXdot->position[3] = 0.0;
    for (ri = rough_P.PlantStateSpace_B_jc[0U]; ri <
         rough_P.PlantStateSpace_B_jc[1U]; ri++) {
      _rtXdot->position[rough_P.PlantStateSpace_B_ir[ri]] +=
        rough_P.PlantStateSpace_B_pr[ri] * rough_B.umin_scale1[0U];
    }

    for (ri = rough_P.PlantStateSpace_B_jc[1U]; ri <
         rough_P.PlantStateSpace_B_jc[2U]; ri++) {
      _rtXdot->position[rough_P.PlantStateSpace_B_ir[ri]] +=
        rough_P.PlantStateSpace_B_pr[ri] * rough_B.umin_scale1[1U];
    }

    for (ri = rough_P.PlantStateSpace_B_jc[2U]; ri <
         rough_P.PlantStateSpace_B_jc[3U]; ri++) {
      _rtXdot->position[rough_P.PlantStateSpace_B_ir[ri]] +=
        rough_P.PlantStateSpace_B_pr[ri] * rough_B.umin_scale1[2U];
    }

    for (ri = rough_P.PlantStateSpace_B_jc[3U]; ri <
         rough_P.PlantStateSpace_B_jc[4U]; ri++) {
      _rtXdot->position[rough_P.PlantStateSpace_B_ir[ri]] +=
        rough_P.PlantStateSpace_B_pr[ri] * rough_B.umin_scale1[3U];
    }

    // End of Derivatives for StateSpace: '<S2>/Plant (State-Space)'
  } else {
    {
      real_T *dx;
      int_T i;
      dx = &(((XDot_rough_T *) rough_M->derivs)->position[0]);
      for (i=0; i < 4; i++) {
        dx[i] = 0.0;
      }
    }
  }

  // End of Derivatives for SubSystem: '<Root>/MPC Controller'
}

// Model initialize function
void rough_initialize(void)
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  {
    // Setup solver object
    rtsiSetSimTimeStepPtr(&rough_M->solverInfo, &rough_M->Timing.simTimeStep);
    rtsiSetTPtr(&rough_M->solverInfo, &rtmGetTPtr(rough_M));
    rtsiSetStepSizePtr(&rough_M->solverInfo, &rough_M->Timing.stepSize0);
    rtsiSetdXPtr(&rough_M->solverInfo, &rough_M->derivs);
    rtsiSetContStatesPtr(&rough_M->solverInfo, (real_T **) &rough_M->contStates);
    rtsiSetNumContStatesPtr(&rough_M->solverInfo, &rough_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&rough_M->solverInfo,
      &rough_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&rough_M->solverInfo,
      &rough_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&rough_M->solverInfo,
      &rough_M->periodicContStateRanges);
    rtsiSetContStateDisabledPtr(&rough_M->solverInfo, (boolean_T**)
      &rough_M->contStateDisabled);
    rtsiSetErrorStatusPtr(&rough_M->solverInfo, (&rtmGetErrorStatus(rough_M)));
    rtsiSetRTModelPtr(&rough_M->solverInfo, rough_M);
  }

  rtsiSetSimTimeStep(&rough_M->solverInfo, MAJOR_TIME_STEP);
  rough_M->intgData.y = rough_M->odeY;
  rough_M->intgData.f[0] = rough_M->odeF[0];
  rough_M->intgData.f[1] = rough_M->odeF[1];
  rough_M->intgData.f[2] = rough_M->odeF[2];
  rough_M->contStates = ((X_rough_T *) &rough_X);
  rough_M->contStateDisabled = ((XDis_rough_T *) &rough_XDis);
  rough_M->Timing.tStart = (0.0);
  rtsiSetSolverData(&rough_M->solverInfo, static_cast<void *>(&rough_M->intgData));
  rtsiSetIsMinorTimeStepWithModeChange(&rough_M->solverInfo, false);
  rtsiSetSolverName(&rough_M->solverInfo,"ode3");
  rtmSetTPtr(rough_M, &rough_M->Timing.tArray[0]);
  rtmSetTFinal(rough_M, -1);
  rough_M->Timing.stepSize0 = 0.033333333333333333;

  // External mode info
  rough_M->Sizes.checksums[0] = (3294312775U);
  rough_M->Sizes.checksums[1] = (2815512579U);
  rough_M->Sizes.checksums[2] = (1124498443U);
  rough_M->Sizes.checksums[3] = (4117346443U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[7];
    rough_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    systemRan[1] = (sysRanDType *)&rough_DW.CommandVelocityPublisher_Subsys;
    systemRan[2] = (sysRanDType *)&rough_DW.CommandVelocityPublisher_Subsys;
    systemRan[3] = (sysRanDType *)&rough_DW.CommandVelocityPublisher_Subsys;
    systemRan[4] = (sysRanDType *)&rough_DW.CommandVelocityPublisher_Subsys;
    systemRan[5] = (sysRanDType *)&rough_DW.MPCController_SubsysRanBC;
    systemRan[6] = (sysRanDType *)&rough_DW.MPCController_SubsysRanBC;
    rteiSetModelMappingInfoPtr(rough_M->extModeInfo,
      &rough_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(rough_M->extModeInfo, rough_M->Sizes.checksums);
    rteiSetTPtr(rough_M->extModeInfo, rtmGetTPtr(rough_M));
  }

  // data type transition information
  {
    static DataTypeTransInfo dtInfo;
    rough_M->SpecialInfo.mappingInfo = (&dtInfo);
    dtInfo.numDataTypes = 29;
    dtInfo.dataTypeSizes = &rtDataTypeSizes[0];
    dtInfo.dataTypeNames = &rtDataTypeNames[0];

    // Block I/O transition table
    dtInfo.BTransTable = &rtBTransTable;

    // Parameters transition table
    dtInfo.PTransTable = &rtPTransTable;
  }

  {
    int32_T i;
    char_T b_zeroDelimTopic[27];
    static const char_T b_zeroDelimTopic_0[27] = "/mavros/setpoint_raw/local";

    // InitializeConditions for StateSpace: '<S2>/Plant (State-Space)'
    rough_X.position[0] = rough_P.PlantStateSpace_InitialConditio[0];
    rough_X.position[1] = rough_P.PlantStateSpace_InitialConditio[1];
    rough_X.position[2] = rough_P.PlantStateSpace_InitialConditio[2];
    rough_X.position[3] = rough_P.PlantStateSpace_InitialConditio[3];

    // InitializeConditions for Memory: '<S7>/Memory'
    for (i = 0; i < 48; i++) {
      rough_DW.Memory_PreviousInput[i] = rough_P.Memory_InitialCondition[i];
    }

    // End of InitializeConditions for Memory: '<S7>/Memory'

    // InitializeConditions for UnitDelay: '<S7>/last_mv'
    rough_DW.last_mv_DSTATE[0] = rough_P.last_mv_InitialCondition[0];

    // InitializeConditions for Memory: '<S7>/last_x'
    rough_DW.last_x_PreviousInput[0] = rough_P.last_x_InitialCondition[0];

    // InitializeConditions for UnitDelay: '<S7>/last_mv'
    rough_DW.last_mv_DSTATE[1] = rough_P.last_mv_InitialCondition[1];

    // InitializeConditions for Memory: '<S7>/last_x'
    rough_DW.last_x_PreviousInput[1] = rough_P.last_x_InitialCondition[1];

    // InitializeConditions for UnitDelay: '<S7>/last_mv'
    rough_DW.last_mv_DSTATE[2] = rough_P.last_mv_InitialCondition[2];

    // InitializeConditions for Memory: '<S7>/last_x'
    rough_DW.last_x_PreviousInput[2] = rough_P.last_x_InitialCondition[2];

    // InitializeConditions for UnitDelay: '<S7>/last_mv'
    rough_DW.last_mv_DSTATE[3] = rough_P.last_mv_InitialCondition[3];

    // InitializeConditions for Memory: '<S7>/last_x'
    rough_DW.last_x_PreviousInput[3] = rough_P.last_x_InitialCondition[3];

    // SystemInitialize for Outport: '<S2>/mv_vel_x' incorporates:
    //   Gain: '<S7>/umin_scale1'

    rough_B.umin_scale1[0] = rough_P.mv_vel_x_Y0;

    // SystemInitialize for SignalConversion generated from: '<S2>/mv_vel_y' incorporates:
    //   Outport: '<S2>/mv_vel_y'

    rough_B.mv_vel_y = rough_P.mv_vel_y_Y0;

    // SystemInitialize for SignalConversion generated from: '<S2>/mv_vel_z' incorporates:
    //   Outport: '<S2>/mv_vel_z'

    rough_B.mv_vel_z = rough_P.mv_vel_z_Y0;

    // SystemInitialize for SignalConversion generated from: '<S2>/mv_vel_yaw' incorporates:
    //   Outport: '<S2>/mv_vel_yaw'

    rough_B.mv_vel_yaw = rough_P.mv_vel_yaw_Y0;

    // End of SystemInitialize for SubSystem: '<Root>/MPC Controller'

    // SystemInitialize for Enabled SubSystem: '<Root>/Command Velocity Publisher' 
    // SystemInitialize for Atomic SubSystem: '<S1>/Publish2'
    // Start for MATLABSystem: '<S4>/SinkBlock'
    rough_DW.obj.matlabCodegenIsDeleted = false;
    rough_DW.obj.isInitialized = 1;
    for (i = 0; i < 27; i++) {
      b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
    }

    Pub_rough_12.createPublisher(&b_zeroDelimTopic[0], 105);
    rough_DW.obj.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S4>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<S1>/Publish2'
    // End of SystemInitialize for SubSystem: '<Root>/Command Velocity Publisher' 
  }
}

// Model terminate function
void rough_terminate(void)
{
  // Terminate for Enabled SubSystem: '<Root>/Command Velocity Publisher'
  // Terminate for Atomic SubSystem: '<S1>/Publish2'
  // Terminate for MATLABSystem: '<S4>/SinkBlock'
  if (!rough_DW.obj.matlabCodegenIsDeleted) {
    rough_DW.obj.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S4>/SinkBlock'
  // End of Terminate for SubSystem: '<S1>/Publish2'
  // End of Terminate for SubSystem: '<Root>/Command Velocity Publisher'
}

//
// File trailer for generated code.
//
// [EOF]
//
