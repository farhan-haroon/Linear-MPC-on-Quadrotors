//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: untitled.cpp
//
// Code generated for Simulink model 'untitled'.
//
// Model version                  : 1.0
// Simulink Coder version         : 23.2 (R2023b) 01-Aug-2023
// C/C++ source code generated on : Thu Feb  8 01:34:28 2024
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "untitled.h"
#include "rtwtypes.h"
#include <string.h>
#include "untitled_private.h"
#include <math.h>

extern "C"
{

#include "rt_nonfinite.h"

}

#include "untitled_dt.h"

// Named constants for MATLAB Function: '<S27>/optimizer'
const int32_T untitled_degrees = 13;

// Block signals (default storage)
B_untitled_T untitled_B;

// Continuous states
X_untitled_T untitled_X;

// Disabled State Vector
XDis_untitled_T untitled_XDis;

// Block states (default storage)
DW_untitled_T untitled_DW;

// Real-time model
RT_MODEL_untitled_T untitled_M_ = RT_MODEL_untitled_T();
RT_MODEL_untitled_T *const untitled_M = &untitled_M_;

// Forward declaration for local functions
static real_T untitled_norm(const real_T x[13]);
static real_T untitled_maximum(const real_T x[13]);
static real_T untitled_xnrm2(int32_T n, const real_T x[169], int32_T ix0);
static void untitled_xgemv(int32_T b_m, int32_T n, const real_T b_A[169],
  int32_T ia0, const real_T x[169], int32_T ix0, real_T y[13]);
static void untitled_xgerc(int32_T b_m, int32_T n, real_T alpha1, int32_T ix0,
  const real_T y[13], real_T b_A[169], int32_T ia0);
static void untitled_KWIKfactor(const real_T b_Ac[624], const int32_T iC[48],
  int32_T nA, const real_T b_Linv[169], real_T D[169], real_T b_H[169], int32_T
  n, real_T RLinv[169], real_T *Status);
static void untitled_DropConstraint(int32_T kDrop, boolean_T iA[48], int32_T *nA,
  int32_T iC[48]);
static void untitled_qpkwik(const real_T b_Linv[169], const real_T b_Hinv[169],
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

  (untitled_M->Timing.TaskCounters.TID[2])++;
  if ((untitled_M->Timing.TaskCounters.TID[2]) > 2) {// Sample time: [0.1s, 0.0s] 
    untitled_M->Timing.TaskCounters.TID[2] = 0;
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
  untitled_derivatives();

  // f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*));
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  untitled_step();
  untitled_derivatives();

  // f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*));
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  untitled_step();
  untitled_derivatives();

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
static real_T untitled_norm(const real_T x[13])
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
static real_T untitled_maximum(const real_T x[13])
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
static real_T untitled_xnrm2(int32_T n, const real_T x[169], int32_T ix0)
{
  real_T y;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x[ix0 - 1]);
    } else {
      int32_T kend;
      untitled_B.scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (int32_T k = ix0; k <= kend; k++) {
        untitled_B.absxk = fabs(x[k - 1]);
        if (untitled_B.absxk > untitled_B.scale) {
          untitled_B.t = untitled_B.scale / untitled_B.absxk;
          y = y * untitled_B.t * untitled_B.t + 1.0;
          untitled_B.scale = untitled_B.absxk;
        } else {
          untitled_B.t = untitled_B.absxk / untitled_B.scale;
          y += untitled_B.t * untitled_B.t;
        }
      }

      y = untitled_B.scale * sqrt(y);
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
static void untitled_xgemv(int32_T b_m, int32_T n, const real_T b_A[169],
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
      untitled_B.c = 0.0;
      d = (b_iy + b_m) - 1;
      for (ia = b_iy; ia <= d; ia++) {
        untitled_B.c += x[((ix0 + ia) - b_iy) - 1] * b_A[ia - 1];
      }

      ia = div_nzp_s32_floor(b_iy - ia0, 13);
      y[ia] += untitled_B.c;
    }
  }
}

// Function for MATLAB Function: '<S27>/optimizer'
static void untitled_xgerc(int32_T b_m, int32_T n, real_T alpha1, int32_T ix0,
  const real_T y[13], real_T b_A[169], int32_T ia0)
{
  if (!(alpha1 == 0.0)) {
    int32_T jA;
    jA = ia0;
    for (int32_T j = 0; j < n; j++) {
      untitled_B.temp = y[j];
      if (untitled_B.temp != 0.0) {
        int32_T b;
        untitled_B.temp *= alpha1;
        b = b_m + jA;
        for (int32_T ijA = jA; ijA < b; ijA++) {
          b_A[ijA - 1] += b_A[((ix0 + ijA) - jA) - 1] * untitled_B.temp;
        }
      }

      jA += 13;
    }
  }
}

// Function for MATLAB Function: '<S27>/optimizer'
static void untitled_KWIKfactor(const real_T b_Ac[624], const int32_T iC[48],
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
      untitled_B.beta1 = 0.0;
      for (ii = 0; ii < 13; ii++) {
        untitled_B.beta1 += b_Ac[(48 * ii + knt) - 1] * b_Linv[13 * ii + i];
      }

      RLinv[i + 13 * b_lastv] = untitled_B.beta1;
    }
  }

  memcpy(&untitled_B.b_A[0], &RLinv[0], 169U * sizeof(real_T));
  memset(&untitled_B.tau[0], 0, 13U * sizeof(real_T));
  memset(&untitled_B.work[0], 0, 13U * sizeof(real_T));
  for (i = 0; i < 13; i++) {
    ii = i * 13 + i;
    if (i + 1 < 13) {
      untitled_B.atmp = untitled_B.b_A[ii];
      b_lastv = ii + 2;
      untitled_B.tau[i] = 0.0;
      untitled_B.beta1 = untitled_xnrm2(12 - i, untitled_B.b_A, ii + 2);
      if (untitled_B.beta1 != 0.0) {
        untitled_B.b_A_g = untitled_B.b_A[ii];
        untitled_B.beta1 = rt_hypotd_snf(untitled_B.b_A_g, untitled_B.beta1);
        if (untitled_B.b_A_g >= 0.0) {
          untitled_B.beta1 = -untitled_B.beta1;
        }

        if (fabs(untitled_B.beta1) < 1.0020841800044864E-292) {
          knt = 0;
          f_tmp = (ii - i) + 13;
          do {
            knt++;
            for (b_coltop = b_lastv; b_coltop <= f_tmp; b_coltop++) {
              untitled_B.b_A[b_coltop - 1] *= 9.9792015476736E+291;
            }

            untitled_B.beta1 *= 9.9792015476736E+291;
            untitled_B.atmp *= 9.9792015476736E+291;
          } while ((fabs(untitled_B.beta1) < 1.0020841800044864E-292) && (knt <
                    20));

          untitled_B.beta1 = rt_hypotd_snf(untitled_B.atmp, untitled_xnrm2(12 -
            i, untitled_B.b_A, ii + 2));
          if (untitled_B.atmp >= 0.0) {
            untitled_B.beta1 = -untitled_B.beta1;
          }

          untitled_B.tau[i] = (untitled_B.beta1 - untitled_B.atmp) /
            untitled_B.beta1;
          untitled_B.atmp = 1.0 / (untitled_B.atmp - untitled_B.beta1);
          for (b_coltop = b_lastv; b_coltop <= f_tmp; b_coltop++) {
            untitled_B.b_A[b_coltop - 1] *= untitled_B.atmp;
          }

          for (b_lastv = 0; b_lastv < knt; b_lastv++) {
            untitled_B.beta1 *= 1.0020841800044864E-292;
          }

          untitled_B.atmp = untitled_B.beta1;
        } else {
          untitled_B.tau[i] = (untitled_B.beta1 - untitled_B.b_A_g) /
            untitled_B.beta1;
          untitled_B.atmp = 1.0 / (untitled_B.b_A_g - untitled_B.beta1);
          b_coltop = (ii - i) + 13;
          for (knt = b_lastv; knt <= b_coltop; knt++) {
            untitled_B.b_A[knt - 1] *= untitled_B.atmp;
          }

          untitled_B.atmp = untitled_B.beta1;
        }
      }

      untitled_B.b_A[ii] = 1.0;
      if (untitled_B.tau[i] != 0.0) {
        b_lastv = 13 - i;
        knt = (ii - i) + 12;
        while ((b_lastv > 0) && (untitled_B.b_A[knt] == 0.0)) {
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
              if (untitled_B.b_A[f_tmp] != 0.0) {
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
        untitled_xgemv(b_lastv, knt, untitled_B.b_A, ii + 14, untitled_B.b_A, ii
                       + 1, untitled_B.work);
        untitled_xgerc(b_lastv, knt, -untitled_B.tau[i], ii + 1, untitled_B.work,
                       untitled_B.b_A, ii + 14);
      }

      untitled_B.b_A[ii] = untitled_B.atmp;
    } else {
      untitled_B.tau[12] = 0.0;
    }
  }

  for (i = 0; i < 13; i++) {
    for (ii = 0; ii <= i; ii++) {
      untitled_B.R[ii + 13 * i] = untitled_B.b_A[13 * i + ii];
    }

    for (ii = i + 2; ii < 14; ii++) {
      untitled_B.R[(ii + 13 * i) - 1] = 0.0;
    }

    untitled_B.work[i] = 0.0;
  }

  for (i = 12; i >= 0; i--) {
    ii = (i * 13 + i) + 14;
    if (i + 1 < 13) {
      untitled_B.b_A[ii - 14] = 1.0;
      if (untitled_B.tau[i] != 0.0) {
        b_lastv = 13 - i;
        knt = ii - i;
        while ((b_lastv > 0) && (untitled_B.b_A[knt - 2] == 0.0)) {
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
              if (untitled_B.b_A[f_tmp - 1] != 0.0) {
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
        untitled_xgemv(b_lastv, knt, untitled_B.b_A, ii, untitled_B.b_A, ii - 13,
                       untitled_B.work);
        untitled_xgerc(b_lastv, knt, -untitled_B.tau[i], ii - 13,
                       untitled_B.work, untitled_B.b_A, ii);
      }

      knt = (ii - i) - 1;
      for (b_lastv = ii - 12; b_lastv <= knt; b_lastv++) {
        untitled_B.b_A[b_lastv - 1] *= -untitled_B.tau[i];
      }
    }

    untitled_B.b_A[ii - 14] = 1.0 - untitled_B.tau[i];
    for (b_lastv = 0; b_lastv < i; b_lastv++) {
      untitled_B.b_A[(ii - b_lastv) - 15] = 0.0;
    }
  }

  i = 0;
  do {
    exitg1 = 0;
    if (i <= nA - 1) {
      if (fabs(untitled_B.R[13 * i + i]) < 1.0E-12) {
        *Status = -2.0;
        exitg1 = 1;
      } else {
        i++;
      }
    } else {
      for (ii = 0; ii < n; ii++) {
        for (b_lastv = 0; b_lastv < n; b_lastv++) {
          untitled_B.beta1 = 0.0;
          for (i = 0; i < 13; i++) {
            untitled_B.beta1 += b_Linv[13 * ii + i] * untitled_B.b_A[13 *
              b_lastv + i];
          }

          untitled_B.TL[ii + 13 * b_lastv] = untitled_B.beta1;
        }
      }

      memset(&RLinv[0], 0, 169U * sizeof(real_T));
      for (i = nA; i >= 1; i--) {
        knt = (i - 1) * 13;
        b_coltop = (i + knt) - 1;
        RLinv[b_coltop] = 1.0;
        for (ii = i; ii <= nA; ii++) {
          f_tmp = ((ii - 1) * 13 + i) - 1;
          RLinv[f_tmp] /= untitled_B.R[b_coltop];
        }

        if (i > 1) {
          for (ii = 0; ii <= i - 2; ii++) {
            for (b_lastv = i; b_lastv <= nA; b_lastv++) {
              b_coltop = (b_lastv - 1) * 13;
              f_tmp = b_coltop + ii;
              RLinv[f_tmp] -= RLinv[(b_coltop + i) - 1] * untitled_B.R[knt + ii];
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
            b_H[i] -= untitled_B.TL[(ii + knt) - 1] * untitled_B.TL[ii + b_lastv];
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
            D[i] += untitled_B.TL[ii + knt] * RLinv[ii + b_lastv];
          }
        }
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);
}

// Function for MATLAB Function: '<S27>/optimizer'
static void untitled_DropConstraint(int32_T kDrop, boolean_T iA[48], int32_T *nA,
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
static void untitled_qpkwik(const real_T b_Linv[169], const real_T b_Hinv[169],
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
  memset(&untitled_B.r[0], 0, 13U * sizeof(real_T));
  untitled_B.rMin = 0.0;
  cTolComputed = false;
  for (i = 0; i < 48; i++) {
    untitled_B.cTol[i] = 1.0;
    untitled_B.iC[i] = 0;
  }

  untitled_B.nA = 0;
  for (i = 0; i < 48; i++) {
    if (iA[i]) {
      untitled_B.nA++;
      untitled_B.iC[untitled_B.nA - 1] = i + 1;
    }
  }

  guard1 = false;
  if (untitled_B.nA > 0) {
    memset(&untitled_B.Opt[0], 0, 26U * sizeof(real_T));
    for (i = 0; i < 13; i++) {
      untitled_B.Rhs[i] = f[i];
      untitled_B.Rhs[i + 13] = 0.0;
    }

    DualFeasible = false;
    untitled_B.tmp = static_cast<int32_T>(rt_roundd_snf(0.3 * static_cast<real_T>
      (untitled_B.nA)));
    ColdReset = false;
    do {
      exitg3 = 0;
      if ((!DualFeasible) && (untitled_B.nA > 0) && (*status <= maxiter)) {
        untitled_KWIKfactor(b_Ac, untitled_B.iC, untitled_B.nA, b_Linv,
                            untitled_B.D, untitled_B.b_H, untitled_degrees,
                            untitled_B.RLinv, &untitled_B.Xnorm0);
        if (untitled_B.Xnorm0 < 0.0) {
          if (ColdReset) {
            *status = -2;
            exitg3 = 2;
          } else {
            untitled_B.nA = 0;
            memset(&untitled_B.iC[0], 0, 48U * sizeof(int32_T));
            for (i = 0; i < 48; i++) {
              iA[i] = false;
            }

            ColdReset = true;
          }
        } else {
          for (i = 0; i < untitled_B.nA; i++) {
            untitled_B.Rhs[i + 13] = b[untitled_B.iC[i] - 1];
            for (untitled_B.kDrop = i + 1; untitled_B.kDrop <= untitled_B.nA;
                 untitled_B.kDrop++) {
              U_tmp = (13 * i + untitled_B.kDrop) - 1;
              untitled_B.U[U_tmp] = 0.0;
              for (iSave = 0; iSave < untitled_B.nA; iSave++) {
                untitled_B.U[U_tmp] += untitled_B.RLinv[(13 * iSave +
                  untitled_B.kDrop) - 1] * untitled_B.RLinv[13 * iSave + i];
              }

              untitled_B.U[i + 13 * (untitled_B.kDrop - 1)] = untitled_B.U[U_tmp];
            }
          }

          for (untitled_B.kDrop = 0; untitled_B.kDrop < 13; untitled_B.kDrop++)
          {
            untitled_B.Xnorm0 = 0.0;
            for (i = 0; i < 13; i++) {
              untitled_B.Xnorm0 += untitled_B.b_H[13 * i + untitled_B.kDrop] *
                untitled_B.Rhs[i];
            }

            untitled_B.Opt[untitled_B.kDrop] = untitled_B.Xnorm0;
            for (i = 0; i < untitled_B.nA; i++) {
              untitled_B.Opt[untitled_B.kDrop] += untitled_B.D[13 * i +
                untitled_B.kDrop] * untitled_B.Rhs[i + 13];
            }
          }

          untitled_B.Xnorm0 = -1.0E-12;
          untitled_B.kDrop = -1;
          for (iSave = 0; iSave < untitled_B.nA; iSave++) {
            untitled_B.cMin = 0.0;
            for (i = 0; i < 13; i++) {
              untitled_B.cMin += untitled_B.D[13 * iSave + i] * untitled_B.Rhs[i];
            }

            untitled_B.Opt[iSave + 13] = untitled_B.cMin;
            for (i = 0; i < untitled_B.nA; i++) {
              untitled_B.Opt[iSave + 13] += untitled_B.U[13 * i + iSave] *
                untitled_B.Rhs[i + 13];
            }

            untitled_B.cMin = untitled_B.Opt[iSave + 13];
            lambda[untitled_B.iC[iSave] - 1] = untitled_B.cMin;
            if ((untitled_B.cMin < untitled_B.Xnorm0) && (iSave + 1 <=
                 untitled_B.nA)) {
              untitled_B.kDrop = iSave;
              untitled_B.Xnorm0 = untitled_B.cMin;
            }
          }

          if (untitled_B.kDrop + 1 <= 0) {
            DualFeasible = true;
            memcpy(&x[0], &untitled_B.Opt[0], 13U * sizeof(real_T));
          } else {
            (*status)++;
            if (untitled_B.tmp <= 5) {
              i = 5;
            } else {
              i = untitled_B.tmp;
            }

            if (*status > i) {
              untitled_B.nA = 0;
              memset(&untitled_B.iC[0], 0, 48U * sizeof(int32_T));
              for (i = 0; i < 48; i++) {
                iA[i] = false;
              }

              ColdReset = true;
            } else {
              lambda[untitled_B.iC[untitled_B.kDrop] - 1] = 0.0;
              untitled_DropConstraint(untitled_B.kDrop + 1, iA, &untitled_B.nA,
                untitled_B.iC);
            }
          }
        }
      } else {
        if (untitled_B.nA <= 0) {
          memset(&lambda[0], 0, 48U * sizeof(real_T));
          for (untitled_B.tmp = 0; untitled_B.tmp < 13; untitled_B.tmp++) {
            untitled_B.Xnorm0 = 0.0;
            for (i = 0; i < 13; i++) {
              untitled_B.Xnorm0 += -b_Hinv[13 * i + untitled_B.tmp] * f[i];
            }

            x[untitled_B.tmp] = untitled_B.Xnorm0;
          }
        }

        exitg3 = 1;
      }
    } while (exitg3 == 0);

    if (exitg3 == 1) {
      guard1 = true;
    }
  } else {
    for (untitled_B.tmp = 0; untitled_B.tmp < 13; untitled_B.tmp++) {
      untitled_B.Xnorm0 = 0.0;
      for (i = 0; i < 13; i++) {
        untitled_B.Xnorm0 += -b_Hinv[13 * i + untitled_B.tmp] * f[i];
      }

      x[untitled_B.tmp] = untitled_B.Xnorm0;
    }

    guard1 = true;
  }

  if (guard1) {
    untitled_B.Xnorm0 = untitled_norm(x);
    exitg2 = false;
    while ((!exitg2) && (*status <= maxiter)) {
      untitled_B.cMin = -FeasTol;
      untitled_B.tmp = -1;
      for (untitled_B.kDrop = 0; untitled_B.kDrop < 48; untitled_B.kDrop++) {
        if (!cTolComputed) {
          for (i = 0; i < 13; i++) {
            untitled_B.z[i] = fabs(b_Ac[48 * i + untitled_B.kDrop] * x[i]);
          }

          untitled_B.cVal = untitled_maximum(untitled_B.z);
          if ((untitled_B.cTol[untitled_B.kDrop] >= untitled_B.cVal) || rtIsNaN
              (untitled_B.cVal)) {
          } else {
            untitled_B.cTol[untitled_B.kDrop] = untitled_B.cVal;
          }
        }

        if (!iA[untitled_B.kDrop]) {
          untitled_B.cVal = 0.0;
          for (i = 0; i < 13; i++) {
            untitled_B.cVal += b_Ac[48 * i + untitled_B.kDrop] * x[i];
          }

          untitled_B.cVal = (untitled_B.cVal - b[untitled_B.kDrop]) /
            untitled_B.cTol[untitled_B.kDrop];
          if (untitled_B.cVal < untitled_B.cMin) {
            untitled_B.cMin = untitled_B.cVal;
            untitled_B.tmp = untitled_B.kDrop;
          }
        }
      }

      cTolComputed = true;
      if (untitled_B.tmp + 1 <= 0) {
        exitg2 = true;
      } else if (*status == maxiter) {
        *status = 0;
        exitg2 = true;
      } else {
        do {
          exitg1 = 0;
          if ((untitled_B.tmp + 1 > 0) && (*status <= maxiter)) {
            guard2 = false;
            if (untitled_B.nA == 0) {
              for (i = 0; i < 13; i++) {
                untitled_B.cMin = 0.0;
                for (untitled_B.kDrop = 0; untitled_B.kDrop < 13;
                     untitled_B.kDrop++) {
                  untitled_B.cMin += b_Hinv[13 * untitled_B.kDrop + i] * b_Ac[48
                    * untitled_B.kDrop + untitled_B.tmp];
                }

                untitled_B.z[i] = untitled_B.cMin;
              }

              guard2 = true;
            } else {
              untitled_KWIKfactor(b_Ac, untitled_B.iC, untitled_B.nA, b_Linv,
                                  untitled_B.D, untitled_B.b_H, untitled_degrees,
                                  untitled_B.RLinv, &untitled_B.cMin);
              if (untitled_B.cMin <= 0.0) {
                *status = -2;
                exitg1 = 1;
              } else {
                for (i = 0; i < 169; i++) {
                  untitled_B.U[i] = -untitled_B.b_H[i];
                }

                for (i = 0; i < 13; i++) {
                  untitled_B.cMin = 0.0;
                  for (untitled_B.kDrop = 0; untitled_B.kDrop < 13;
                       untitled_B.kDrop++) {
                    untitled_B.cMin += untitled_B.U[13 * untitled_B.kDrop + i] *
                      b_Ac[48 * untitled_B.kDrop + untitled_B.tmp];
                  }

                  untitled_B.z[i] = untitled_B.cMin;
                }

                for (untitled_B.kDrop = 0; untitled_B.kDrop < untitled_B.nA;
                     untitled_B.kDrop++) {
                  untitled_B.cVal = 0.0;
                  for (i = 0; i < 13; i++) {
                    untitled_B.cVal += b_Ac[48 * i + untitled_B.tmp] *
                      untitled_B.D[13 * untitled_B.kDrop + i];
                  }

                  untitled_B.r[untitled_B.kDrop] = untitled_B.cVal;
                }

                guard2 = true;
              }
            }

            if (guard2) {
              untitled_B.kDrop = 0;
              untitled_B.cMin = 0.0;
              DualFeasible = true;
              ColdReset = true;
              if (untitled_B.nA > 0) {
                i = 0;
                exitg4 = false;
                while ((!exitg4) && (i <= untitled_B.nA - 1)) {
                  if (untitled_B.r[i] >= 1.0E-12) {
                    ColdReset = false;
                    exitg4 = true;
                  } else {
                    i++;
                  }
                }
              }

              if ((untitled_B.nA != 0) && (!ColdReset)) {
                for (i = 0; i < untitled_B.nA; i++) {
                  untitled_B.cVal = untitled_B.r[i];
                  if (untitled_B.cVal > 1.0E-12) {
                    untitled_B.cVal = lambda[untitled_B.iC[i] - 1] /
                      untitled_B.cVal;
                    if ((untitled_B.kDrop == 0) || (untitled_B.cVal <
                         untitled_B.rMin)) {
                      untitled_B.rMin = untitled_B.cVal;
                      untitled_B.kDrop = i + 1;
                    }
                  }
                }

                if (untitled_B.kDrop > 0) {
                  untitled_B.cMin = untitled_B.rMin;
                  DualFeasible = false;
                }
              }

              untitled_B.zTa = 0.0;
              for (i = 0; i < 13; i++) {
                untitled_B.zTa += b_Ac[48 * i + untitled_B.tmp] * untitled_B.z[i];
              }

              if (untitled_B.zTa <= 0.0) {
                untitled_B.cVal = 0.0;
                ColdReset = true;
              } else {
                untitled_B.cVal = 0.0;
                for (i = 0; i < 13; i++) {
                  untitled_B.cVal += b_Ac[48 * i + untitled_B.tmp] * x[i];
                }

                untitled_B.cVal = (b[untitled_B.tmp] - untitled_B.cVal) /
                  untitled_B.zTa;
                ColdReset = false;
              }

              if (DualFeasible && ColdReset) {
                *status = -1;
                exitg1 = 1;
              } else {
                if (ColdReset) {
                  untitled_B.zTa = untitled_B.cMin;
                } else if (DualFeasible) {
                  untitled_B.zTa = untitled_B.cVal;
                } else if (untitled_B.cMin < untitled_B.cVal) {
                  untitled_B.zTa = untitled_B.cMin;
                } else {
                  untitled_B.zTa = untitled_B.cVal;
                }

                for (i = 0; i < untitled_B.nA; i++) {
                  iSave = untitled_B.iC[i];
                  lambda[iSave - 1] -= untitled_B.zTa * untitled_B.r[i];
                  if ((iSave <= 48) && (lambda[iSave - 1] < 0.0)) {
                    lambda[iSave - 1] = 0.0;
                  }
                }

                lambda[untitled_B.tmp] += untitled_B.zTa;
                frexp(1.0, &exponent);
                if (fabs(untitled_B.zTa - untitled_B.cMin) <
                    2.2204460492503131E-16) {
                  untitled_DropConstraint(untitled_B.kDrop, iA, &untitled_B.nA,
                    untitled_B.iC);
                }

                if (!ColdReset) {
                  for (i = 0; i < 13; i++) {
                    x[i] += untitled_B.zTa * untitled_B.z[i];
                  }

                  frexp(1.0, &b_exponent);
                  if (fabs(untitled_B.zTa - untitled_B.cVal) <
                      2.2204460492503131E-16) {
                    if (untitled_B.nA == untitled_degrees) {
                      *status = -1;
                      exitg1 = 1;
                    } else {
                      untitled_B.nA++;
                      untitled_B.iC[untitled_B.nA - 1] = untitled_B.tmp + 1;
                      untitled_B.kDrop = untitled_B.nA - 1;
                      exitg4 = false;
                      while ((!exitg4) && (untitled_B.kDrop + 1 > 1)) {
                        i = untitled_B.iC[untitled_B.kDrop - 1];
                        if (untitled_B.iC[untitled_B.kDrop] > i) {
                          exitg4 = true;
                        } else {
                          iSave = untitled_B.iC[untitled_B.kDrop];
                          untitled_B.iC[untitled_B.kDrop] = i;
                          untitled_B.iC[untitled_B.kDrop - 1] = iSave;
                          untitled_B.kDrop--;
                        }
                      }

                      iA[untitled_B.tmp] = true;
                      untitled_B.tmp = -1;
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
            untitled_B.cMin = untitled_norm(x);
            if (fabs(untitled_B.cMin - untitled_B.Xnorm0) > 0.001) {
              untitled_B.Xnorm0 = untitled_B.cMin;
              for (i = 0; i < 48; i++) {
                untitled_B.cMin = fabs(b[i]);
                if (untitled_B.cMin >= 1.0) {
                  untitled_B.cTol[i] = untitled_B.cMin;
                } else {
                  untitled_B.cTol[i] = 1.0;
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
void untitled_step(void)
{
  static const int8_T a[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

  static const real_T b_a[16] = { 0.99019513592784836, 0.0, 0.0, 0.0, 0.0,
    0.99019513592784836, 0.0, 0.0, 0.0, 0.0, 0.99019513592784836, 0.0, 0.0, 0.0,
    0.0, 0.99019513592784825 };

  static const real_T b_Kr[960] = { -5.459815003314425, -0.0, -0.0, -0.0,
    -10.91963000662885, -0.0, -0.0, -0.0, -16.379445009943275, -0.0, -0.0, -0.0,
    -21.8392600132577, -0.0, -0.0, -0.0, -27.299075016572122, -0.0, -0.0, -0.0,
    -32.758890019886543, -0.0, -0.0, -0.0, -38.218705023200968, -0.0, -0.0, -0.0,
    -43.678520026515393, -0.0, -0.0, -0.0, -49.138335029829811, -0.0, -0.0, -0.0,
    -54.598150033144236, -0.0, -0.0, -0.0, -60.057965036458661, -0.0, -0.0, -0.0,
    -65.517780039773086, -0.0, -0.0, -0.0, -70.977595043087518, -0.0, -0.0, -0.0,
    -76.437410046401951, -0.0, -0.0, -0.0, -81.897225049716383, -0.0, -0.0, -0.0,
    -87.3570400530308, -0.0, -0.0, -0.0, -92.816855056345233, -0.0, -0.0, -0.0,
    -98.276670059659665, -0.0, -0.0, -0.0, -103.7364850629741, -0.0, -0.0, -0.0,
    -109.19630006628852, -0.0, -0.0, -0.0, -0.0, -5.459815003314425, -0.0, -0.0,
    -0.0, -10.91963000662885, -0.0, -0.0, -0.0, -16.379445009943275, -0.0, -0.0,
    -0.0, -21.8392600132577, -0.0, -0.0, -0.0, -27.299075016572122, -0.0, -0.0,
    -0.0, -32.758890019886543, -0.0, -0.0, -0.0, -38.218705023200968, -0.0, -0.0,
    -0.0, -43.678520026515393, -0.0, -0.0, -0.0, -49.138335029829811, -0.0, -0.0,
    -0.0, -54.598150033144236, -0.0, -0.0, -0.0, -60.057965036458661, -0.0, -0.0,
    -0.0, -65.517780039773086, -0.0, -0.0, -0.0, -70.977595043087518, -0.0, -0.0,
    -0.0, -76.437410046401951, -0.0, -0.0, -0.0, -81.897225049716383, -0.0, -0.0,
    -0.0, -87.3570400530308, -0.0, -0.0, -0.0, -92.816855056345233, -0.0, -0.0,
    -0.0, -98.276670059659665, -0.0, -0.0, -0.0, -103.7364850629741, -0.0, -0.0,
    -0.0, -109.19630006628852, -0.0, -0.0, -0.0, -0.0, -5.459815003314425, -0.0,
    -0.0, -0.0, -10.91963000662885, -0.0, -0.0, -0.0, -16.379445009943275, -0.0,
    -0.0, -0.0, -21.8392600132577, -0.0, -0.0, -0.0, -27.299075016572122, -0.0,
    -0.0, -0.0, -32.758890019886543, -0.0, -0.0, -0.0, -38.218705023200968, -0.0,
    -0.0, -0.0, -43.678520026515393, -0.0, -0.0, -0.0, -49.138335029829811, -0.0,
    -0.0, -0.0, -54.598150033144236, -0.0, -0.0, -0.0, -60.057965036458661, -0.0,
    -0.0, -0.0, -65.517780039773086, -0.0, -0.0, -0.0, -70.977595043087518, -0.0,
    -0.0, -0.0, -76.437410046401951, -0.0, -0.0, -0.0, -81.897225049716383, -0.0,
    -0.0, -0.0, -87.3570400530308, -0.0, -0.0, -0.0, -92.816855056345233, -0.0,
    -0.0, -0.0, -98.276670059659665, -0.0, -0.0, -0.0, -103.7364850629741, -0.0,
    -0.0, -0.0, -109.19630006628852, -0.0, -0.0, -0.0, -0.0, -5.459815003314425,
    -0.0, -0.0, -0.0, -10.91963000662885, -0.0, -0.0, -0.0, -16.379445009943275,
    -0.0, -0.0, -0.0, -21.8392600132577, -0.0, -0.0, -0.0, -27.299075016572122,
    -0.0, -0.0, -0.0, -32.758890019886543, -0.0, -0.0, -0.0, -38.218705023200968,
    -0.0, -0.0, -0.0, -43.678520026515393, -0.0, -0.0, -0.0, -49.138335029829811,
    -0.0, -0.0, -0.0, -54.598150033144236, -0.0, -0.0, -0.0, -60.057965036458661,
    -0.0, -0.0, -0.0, -65.517780039773086, -0.0, -0.0, -0.0, -70.977595043087518,
    -0.0, -0.0, -0.0, -76.437410046401951, -0.0, -0.0, -0.0, -81.897225049716383,
    -0.0, -0.0, -0.0, -87.3570400530308, -0.0, -0.0, -0.0, -92.816855056345233,
    -0.0, -0.0, -0.0, -98.276670059659665, -0.0, -0.0, -0.0, -103.7364850629741,
    -0.0, -0.0, -0.0, -109.19630006628852, -0.0, -0.0, -0.0, -0.0,
    -5.459815003314425, -0.0, -0.0, -0.0, -10.91963000662885, -0.0, -0.0, -0.0,
    -16.379445009943275, -0.0, -0.0, -0.0, -21.8392600132577, -0.0, -0.0, -0.0,
    -27.299075016572122, -0.0, -0.0, -0.0, -32.758890019886543, -0.0, -0.0, -0.0,
    -38.218705023200968, -0.0, -0.0, -0.0, -43.678520026515393, -0.0, -0.0, -0.0,
    -49.138335029829811, -0.0, -0.0, -0.0, -54.598150033144236, -0.0, -0.0, -0.0,
    -60.057965036458661, -0.0, -0.0, -0.0, -65.517780039773086, -0.0, -0.0, -0.0,
    -70.977595043087518, -0.0, -0.0, -0.0, -76.437410046401951, -0.0, -0.0, -0.0,
    -81.897225049716383, -0.0, -0.0, -0.0, -87.3570400530308, -0.0, -0.0, -0.0,
    -92.816855056345233, -0.0, -0.0, -0.0, -98.276670059659665, -0.0, -0.0, -0.0,
    -103.7364850629741, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -5.459815003314425, -0.0, -0.0, -0.0, -10.91963000662885, -0.0, -0.0, -0.0,
    -16.379445009943275, -0.0, -0.0, -0.0, -21.8392600132577, -0.0, -0.0, -0.0,
    -27.299075016572122, -0.0, -0.0, -0.0, -32.758890019886543, -0.0, -0.0, -0.0,
    -38.218705023200968, -0.0, -0.0, -0.0, -43.678520026515393, -0.0, -0.0, -0.0,
    -49.138335029829811, -0.0, -0.0, -0.0, -54.598150033144236, -0.0, -0.0, -0.0,
    -60.057965036458661, -0.0, -0.0, -0.0, -65.517780039773086, -0.0, -0.0, -0.0,
    -70.977595043087518, -0.0, -0.0, -0.0, -76.437410046401951, -0.0, -0.0, -0.0,
    -81.897225049716383, -0.0, -0.0, -0.0, -87.3570400530308, -0.0, -0.0, -0.0,
    -92.816855056345233, -0.0, -0.0, -0.0, -98.276670059659665, -0.0, -0.0, -0.0,
    -103.7364850629741, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -5.459815003314425, -0.0, -0.0, -0.0, -10.91963000662885, -0.0, -0.0, -0.0,
    -16.379445009943275, -0.0, -0.0, -0.0, -21.8392600132577, -0.0, -0.0, -0.0,
    -27.299075016572122, -0.0, -0.0, -0.0, -32.758890019886543, -0.0, -0.0, -0.0,
    -38.218705023200968, -0.0, -0.0, -0.0, -43.678520026515393, -0.0, -0.0, -0.0,
    -49.138335029829811, -0.0, -0.0, -0.0, -54.598150033144236, -0.0, -0.0, -0.0,
    -60.057965036458661, -0.0, -0.0, -0.0, -65.517780039773086, -0.0, -0.0, -0.0,
    -70.977595043087518, -0.0, -0.0, -0.0, -76.437410046401951, -0.0, -0.0, -0.0,
    -81.897225049716383, -0.0, -0.0, -0.0, -87.3570400530308, -0.0, -0.0, -0.0,
    -92.816855056345233, -0.0, -0.0, -0.0, -98.276670059659665, -0.0, -0.0, -0.0,
    -103.7364850629741, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -5.459815003314425, -0.0, -0.0, -0.0, -10.91963000662885, -0.0, -0.0, -0.0,
    -16.379445009943275, -0.0, -0.0, -0.0, -21.8392600132577, -0.0, -0.0, -0.0,
    -27.299075016572122, -0.0, -0.0, -0.0, -32.758890019886543, -0.0, -0.0, -0.0,
    -38.218705023200968, -0.0, -0.0, -0.0, -43.678520026515393, -0.0, -0.0, -0.0,
    -49.138335029829811, -0.0, -0.0, -0.0, -54.598150033144236, -0.0, -0.0, -0.0,
    -60.057965036458661, -0.0, -0.0, -0.0, -65.517780039773086, -0.0, -0.0, -0.0,
    -70.977595043087518, -0.0, -0.0, -0.0, -76.437410046401951, -0.0, -0.0, -0.0,
    -81.897225049716383, -0.0, -0.0, -0.0, -87.3570400530308, -0.0, -0.0, -0.0,
    -92.816855056345233, -0.0, -0.0, -0.0, -98.276670059659665, -0.0, -0.0, -0.0,
    -103.7364850629741, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -5.459815003314425, -0.0, -0.0, -0.0, -10.91963000662885, -0.0, -0.0, -0.0,
    -16.379445009943275, -0.0, -0.0, -0.0, -21.8392600132577, -0.0, -0.0, -0.0,
    -27.299075016572122, -0.0, -0.0, -0.0, -32.758890019886543, -0.0, -0.0, -0.0,
    -38.218705023200968, -0.0, -0.0, -0.0, -43.678520026515393, -0.0, -0.0, -0.0,
    -49.138335029829811, -0.0, -0.0, -0.0, -54.598150033144236, -0.0, -0.0, -0.0,
    -60.057965036458661, -0.0, -0.0, -0.0, -65.517780039773086, -0.0, -0.0, -0.0,
    -70.977595043087518, -0.0, -0.0, -0.0, -76.437410046401951, -0.0, -0.0, -0.0,
    -81.897225049716383, -0.0, -0.0, -0.0, -87.3570400530308, -0.0, -0.0, -0.0,
    -92.816855056345233, -0.0, -0.0, -0.0, -98.276670059659665, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -5.459815003314425,
    -0.0, -0.0, -0.0, -10.91963000662885, -0.0, -0.0, -0.0, -16.379445009943275,
    -0.0, -0.0, -0.0, -21.8392600132577, -0.0, -0.0, -0.0, -27.299075016572122,
    -0.0, -0.0, -0.0, -32.758890019886543, -0.0, -0.0, -0.0, -38.218705023200968,
    -0.0, -0.0, -0.0, -43.678520026515393, -0.0, -0.0, -0.0, -49.138335029829811,
    -0.0, -0.0, -0.0, -54.598150033144236, -0.0, -0.0, -0.0, -60.057965036458661,
    -0.0, -0.0, -0.0, -65.517780039773086, -0.0, -0.0, -0.0, -70.977595043087518,
    -0.0, -0.0, -0.0, -76.437410046401951, -0.0, -0.0, -0.0, -81.897225049716383,
    -0.0, -0.0, -0.0, -87.3570400530308, -0.0, -0.0, -0.0, -92.816855056345233,
    -0.0, -0.0, -0.0, -98.276670059659665, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -5.459815003314425, -0.0, -0.0, -0.0,
    -10.91963000662885, -0.0, -0.0, -0.0, -16.379445009943275, -0.0, -0.0, -0.0,
    -21.8392600132577, -0.0, -0.0, -0.0, -27.299075016572122, -0.0, -0.0, -0.0,
    -32.758890019886543, -0.0, -0.0, -0.0, -38.218705023200968, -0.0, -0.0, -0.0,
    -43.678520026515393, -0.0, -0.0, -0.0, -49.138335029829811, -0.0, -0.0, -0.0,
    -54.598150033144236, -0.0, -0.0, -0.0, -60.057965036458661, -0.0, -0.0, -0.0,
    -65.517780039773086, -0.0, -0.0, -0.0, -70.977595043087518, -0.0, -0.0, -0.0,
    -76.437410046401951, -0.0, -0.0, -0.0, -81.897225049716383, -0.0, -0.0, -0.0,
    -87.3570400530308, -0.0, -0.0, -0.0, -92.816855056345233, -0.0, -0.0, -0.0,
    -98.276670059659665, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0,
    -0.0, -0.0, -0.0, -5.459815003314425, -0.0, -0.0, -0.0, -10.91963000662885,
    -0.0, -0.0, -0.0, -16.379445009943275, -0.0, -0.0, -0.0, -21.8392600132577,
    -0.0, -0.0, -0.0, -27.299075016572122, -0.0, -0.0, -0.0, -32.758890019886543,
    -0.0, -0.0, -0.0, -38.218705023200968, -0.0, -0.0, -0.0, -43.678520026515393,
    -0.0, -0.0, -0.0, -49.138335029829811, -0.0, -0.0, -0.0, -54.598150033144236,
    -0.0, -0.0, -0.0, -60.057965036458661, -0.0, -0.0, -0.0, -65.517780039773086,
    -0.0, -0.0, -0.0, -70.977595043087518, -0.0, -0.0, -0.0, -76.437410046401951,
    -0.0, -0.0, -0.0, -81.897225049716383, -0.0, -0.0, -0.0, -87.3570400530308,
    -0.0, -0.0, -0.0, -92.816855056345233, -0.0, -0.0, -0.0, -98.276670059659665
  };

  static const real_T b_Kx[48] = { 1146.5611506960292, 0.0, 0.0, 0.0, 0.0,
    1146.5611506960292, 0.0, 0.0, 0.0, 0.0, 1146.5611506960292, 0.0, 0.0, 0.0,
    0.0, 1146.5611506960292, 1037.3648506297407, 0.0, 0.0, 0.0, 0.0,
    1037.3648506297407, 0.0, 0.0, 0.0, 0.0, 1037.3648506297407, 0.0, 0.0, 0.0,
    0.0, 1037.3648506297407, 933.62836556676655, 0.0, 0.0, 0.0, 0.0,
    933.62836556676655, 0.0, 0.0, 0.0, 0.0, 933.62836556676655, 0.0, 0.0, 0.0,
    0.0, 933.62836556676655 };

  static const real_T b_Ku1[48] = { 1566.9669059512405, 0.0, 0.0, 0.0, 0.0,
    1566.9669059512405, 0.0, 0.0, 0.0, 0.0, 1566.9669059512405, 0.0, 0.0, 0.0,
    0.0, 1566.9669059512405, 1452.3107908816373, 0.0, 0.0, 0.0, 0.0,
    1452.3107908816373, 0.0, 0.0, 0.0, 0.0, 1452.3107908816373, 0.0, 0.0, 0.0,
    0.0, 1452.3107908816373, 1338.2006573123658, 0.0, 0.0, 0.0, 0.0,
    1338.2006573123658, 0.0, 0.0, 0.0, 0.0, 1338.2006573123658, 0.0, 0.0, 0.0,
    0.0, 1338.2006573123658 };

  static const real_T b_Linv[169] = { 0.025262135627105945, 0.0, 0.0, 0.0,
    -0.58263508469202074, 0.0, 0.0, 0.0, 1.2197472397371183, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.025262135627105945, 0.0, 0.0, 0.0, -0.58263508469202074, 0.0, 0.0,
    0.0, 1.2197472397373395, 0.0, 0.0, 0.0, 0.0, 0.0, 0.025262135627105945, 0.0,
    0.0, 0.0, -0.58263508469202074, 0.0, 0.0, 0.0, 1.2197472397372178, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.025262135627105945, 0.0, 0.0, 0.0, -0.58263508469202074,
    0.0, 0.0, 0.0, 1.2197472397373395, 0.0, 0.0, 0.0, 0.0, 0.0, 0.62863266485654,
    0.0, 0.0, 0.0, -2.6965742403636104, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.62863266485654, 0.0, 0.0, 0.0, -2.6965742403640993, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.62863266485654, 0.0, 0.0, 0.0,
    -2.6965742403638306, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.62863266485654, 0.0, 0.0, 0.0, -2.6965742403640993, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.4982507107882734, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.4982507107885452, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.4982507107883958, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.4982507107885452,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.003162277660168379 };

  static const real_T b_Hinv[169] = { 1.8278851462568395, 0.0, 0.0, 0.0,
    -3.6554024323585912, 0.0, 0.0, 0.0, 1.827487168918172, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.8278851462573791, 0.0, 0.0, 0.0, -3.655402432359784, 0.0, 0.0, 0.0,
    1.827487168918835, 0.0, 0.0, 0.0, 0.0, 0.0, 1.8278851462570822, 0.0, 0.0,
    0.0, -3.6554024323591281, 0.0, 0.0, 0.0, 1.8274871689184704, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.8278851462573791, 0.0, 0.0, 0.0, -3.655402432359784, 0.0, 0.0,
    0.0, 1.827487168918835, 0.0, -3.6554024323585912, 0.0, 0.0, 0.0,
    7.6666916611172171, 0.0, 0.0, 0.0, -4.0401442723181278, 0.0, 0.0, 0.0, 0.0,
    0.0, -3.655402432359784, 0.0, 0.0, 0.0, 7.6666916611198541, 0.0, 0.0, 0.0,
    -4.0401442723195933, 0.0, 0.0, 0.0, 0.0, 0.0, -3.6554024323591281, 0.0, 0.0,
    0.0, 7.6666916611184055, 0.0, 0.0, 0.0, -4.0401442723187877, 0.0, 0.0, 0.0,
    0.0, 0.0, -3.655402432359784, 0.0, 0.0, 0.0, 7.6666916611198541, 0.0, 0.0,
    0.0, -4.0401442723195933, 0.0, 1.827487168918172, 0.0, 0.0, 0.0,
    -4.0401442723181278, 0.0, 0.0, 0.0, 2.2447551923775664, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.827487168918835, 0.0, 0.0, 0.0, -4.0401442723195933, 0.0, 0.0, 0.0,
    2.2447551923783808, 0.0, 0.0, 0.0, 0.0, 0.0, 1.8274871689184704, 0.0, 0.0,
    0.0, -4.0401442723187877, 0.0, 0.0, 0.0, 2.2447551923779332, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.827487168918835, 0.0, 0.0, 0.0, -4.0401442723195933, 0.0, 0.0,
    0.0, 2.2447551923783808, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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

  static const real_T f_a[16] = { 0.99019513592784736, -0.0, -0.0, -0.0, 0.0,
    0.99019513592784736, -0.0, -0.0, 0.0, 0.0, 0.99019513592784736, -0.0, 0.0,
    0.0, 0.0, 0.99019513592784891 };

  if (rtmIsMajorTimeStep(untitled_M)) {
    // set solver stop time
    rtsiSetSolverStopTime(&untitled_M->solverInfo,
                          ((untitled_M->Timing.clockTick0+1)*
      untitled_M->Timing.stepSize0));
  }                                    // end MajorTimeStep

  // Update absolute time of base rate at minor time step
  if (rtmIsMinorTimeStep(untitled_M)) {
    untitled_M->Timing.t[0] = rtsiGetT(&untitled_M->solverInfo);
  }

  // Reset subsysRan breadcrumbs
  srClearBC(untitled_DW.CommandVelocityPublisher_Subsys);

  // Reset subsysRan breadcrumbs
  srClearBC(untitled_DW.MPCController_SubsysRanBC);

  // Outputs for Enabled SubSystem: '<Root>/MPC Controller' incorporates:
  //   EnablePort: '<S2>/Enable'

  if ((rtmIsMajorTimeStep(untitled_M) &&
       untitled_M->Timing.TaskCounters.TID[1] == 0) && rtsiIsModeUpdateTimeStep(
       &untitled_M->solverInfo)) {
    // Constant: '<Root>/Constant'
    if (untitled_P.Constant_Value_m > 0.0) {
      if (!untitled_DW.MPCController_MODE) {
        (void) memset(&(untitled_XDis.position), 0,
                      4*sizeof(boolean_T));
        untitled_DW.MPCController_MODE = true;
      }
    } else {
      if (((untitled_M->Timing.clockTick1) * 0.033333333333333333) ==
          rtmGetTStart(untitled_M)) {
        (void) memset(&(untitled_XDis.position), 1,
                      4*sizeof(boolean_T));
      }

      if (untitled_DW.MPCController_MODE) {
        (void) memset(&(untitled_XDis.position), 1,
                      4*sizeof(boolean_T));
        untitled_DW.MPCController_MODE = false;
      }
    }

    // End of Constant: '<Root>/Constant'
  }

  if (untitled_DW.MPCController_MODE) {
    // StateSpace: '<S2>/Plant (State-Space)'
    untitled_B.measuredoutputs[0] = 0.0;
    untitled_B.measuredoutputs[1] = 0.0;
    untitled_B.measuredoutputs[2] = 0.0;
    untitled_B.measuredoutputs[3] = 0.0;
    untitled_B.ri = untitled_P.PlantStateSpace_C_jc[0U];
    while (untitled_B.ri < untitled_P.PlantStateSpace_C_jc[1U]) {
      untitled_B.measuredoutputs[untitled_P.PlantStateSpace_C_ir[untitled_B.ri]]
        += untitled_P.PlantStateSpace_C_pr[untitled_B.ri] * untitled_X.position
        [0U];
      untitled_B.ri++;
    }

    untitled_B.ri = untitled_P.PlantStateSpace_C_jc[1U];
    while (untitled_B.ri < untitled_P.PlantStateSpace_C_jc[2U]) {
      untitled_B.measuredoutputs[untitled_P.PlantStateSpace_C_ir[untitled_B.ri]]
        += untitled_P.PlantStateSpace_C_pr[untitled_B.ri] * untitled_X.position
        [1U];
      untitled_B.ri++;
    }

    untitled_B.ri = untitled_P.PlantStateSpace_C_jc[2U];
    while (untitled_B.ri < untitled_P.PlantStateSpace_C_jc[3U]) {
      untitled_B.measuredoutputs[untitled_P.PlantStateSpace_C_ir[untitled_B.ri]]
        += untitled_P.PlantStateSpace_C_pr[untitled_B.ri] * untitled_X.position
        [2U];
      untitled_B.ri++;
    }

    untitled_B.ri = untitled_P.PlantStateSpace_C_jc[3U];
    while (untitled_B.ri < untitled_P.PlantStateSpace_C_jc[4U]) {
      untitled_B.measuredoutputs[untitled_P.PlantStateSpace_C_ir[untitled_B.ri]]
        += untitled_P.PlantStateSpace_C_pr[untitled_B.ri] * untitled_X.position
        [3U];
      untitled_B.ri++;
    }

    // End of StateSpace: '<S2>/Plant (State-Space)'
    if (rtmIsMajorTimeStep(untitled_M) &&
        untitled_M->Timing.TaskCounters.TID[2] == 0) {
      // Sum: '<S6>/Output' incorporates:
      //   Step: '<S6>/Step'

      if (untitled_M->Timing.t[0] < untitled_P.Ramp_start) {
        untitled_B.rtb_TmpSignalConversionAtSFun_b = untitled_P.Step_Y0;
      } else {
        untitled_B.rtb_TmpSignalConversionAtSFun_b = untitled_P.Ramp_slope;
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

      untitled_B.TmpSignalConversionAtSFunct[0] = (untitled_M->Timing.t[0] -
        untitled_P.Ramp_start) * untitled_B.rtb_TmpSignalConversionAtSFun_b +
        untitled_P.Ramp_InitialOutput;

      // MATLAB Function: '<S27>/optimizer' incorporates:
      //   Constant: '<S2>/Constant'
      //   Constant: '<S2>/Constant1'
      //   Constant: '<S2>/Constant2'
      //   Memory: '<S7>/Memory'
      //   Memory: '<S7>/last_x'
      //   UnitDelay: '<S7>/last_mv'

      for (untitled_B.i = 0; untitled_B.i < 20; untitled_B.i++) {
        untitled_B.rseq_tmp = untitled_B.i << 2;
        untitled_B.rseq[untitled_B.rseq_tmp] =
          untitled_B.TmpSignalConversionAtSFunct[0];
        untitled_B.rseq[untitled_B.rseq_tmp + 1] = untitled_P.Constant_Value_a;
        untitled_B.rseq[untitled_B.rseq_tmp + 2] = untitled_P.Constant1_Value;
        untitled_B.rseq[untitled_B.rseq_tmp + 3] = untitled_P.Constant2_Value;
      }

      untitled_B.old_u[0] = untitled_DW.last_mv_DSTATE[0];
      untitled_B.TmpSignalConversionAtSFunct[0] =
        untitled_DW.last_x_PreviousInput[0];
      untitled_B.old_u[1] = untitled_DW.last_mv_DSTATE[1];
      untitled_B.TmpSignalConversionAtSFunct[1] =
        untitled_DW.last_x_PreviousInput[1];
      untitled_B.old_u[2] = untitled_DW.last_mv_DSTATE[2];
      untitled_B.TmpSignalConversionAtSFunct[2] =
        untitled_DW.last_x_PreviousInput[2];
      untitled_B.old_u[3] = untitled_DW.last_mv_DSTATE[3];
      untitled_B.TmpSignalConversionAtSFunct[3] =
        untitled_DW.last_x_PreviousInput[3];
      untitled_B.rtb_TmpSignalConversionAtSFun_b =
        untitled_DW.last_x_PreviousInput[1];
      untitled_B.rtb_TmpSignalConversionAtSFun_p =
        untitled_DW.last_x_PreviousInput[0];
      untitled_B.rtb_TmpSignalConversionAtSFun_c =
        untitled_DW.last_x_PreviousInput[2];
      untitled_B.rtb_TmpSignalConversionAtSFun_f =
        untitled_DW.last_x_PreviousInput[3];
      for (untitled_B.rseq_tmp = 0; untitled_B.rseq_tmp < 4; untitled_B.rseq_tmp
           ++) {
        untitled_B.y_innov[untitled_B.rseq_tmp] =
          untitled_B.measuredoutputs[untitled_B.rseq_tmp] -
          (((static_cast<real_T>(a[untitled_B.rseq_tmp + 4]) *
             untitled_B.rtb_TmpSignalConversionAtSFun_b + static_cast<real_T>
             (a[untitled_B.rseq_tmp]) *
             untitled_B.rtb_TmpSignalConversionAtSFun_p) + static_cast<real_T>
            (a[untitled_B.rseq_tmp + 8]) *
            untitled_B.rtb_TmpSignalConversionAtSFun_c) + static_cast<real_T>
           (a[untitled_B.rseq_tmp + 12]) *
           untitled_B.rtb_TmpSignalConversionAtSFun_f);
      }

      untitled_B.rtb_TmpSignalConversionAtSFun_p = untitled_B.y_innov[1];
      untitled_B.rtb_TmpSignalConversionAtSFun_c = untitled_B.y_innov[0];
      untitled_B.rtb_TmpSignalConversionAtSFun_f = untitled_B.y_innov[2];
      untitled_B.y_innov_c = untitled_B.y_innov[3];
      for (untitled_B.rseq_tmp = 0; untitled_B.rseq_tmp < 4; untitled_B.rseq_tmp
           ++) {
        untitled_B.xest[untitled_B.rseq_tmp] = (((b_a[untitled_B.rseq_tmp + 4] *
          untitled_B.rtb_TmpSignalConversionAtSFun_p + b_a[untitled_B.rseq_tmp] *
          untitled_B.rtb_TmpSignalConversionAtSFun_c) + b_a[untitled_B.rseq_tmp
          + 8] * untitled_B.rtb_TmpSignalConversionAtSFun_f) +
          b_a[untitled_B.rseq_tmp + 12] * untitled_B.y_innov_c) +
          untitled_B.TmpSignalConversionAtSFunct[untitled_B.rseq_tmp];
      }

      memset(&untitled_B.f[0], 0, 13U * sizeof(real_T));
      for (untitled_B.i = 0; untitled_B.i < 12; untitled_B.i++) {
        untitled_B.rtb_TmpSignalConversionAtSFun_b = 0.0;
        for (untitled_B.rseq_tmp = 0; untitled_B.rseq_tmp < 80;
             untitled_B.rseq_tmp++) {
          untitled_B.rtb_TmpSignalConversionAtSFun_b += b_Kr[80 * untitled_B.i +
            untitled_B.rseq_tmp] * untitled_B.rseq[untitled_B.rseq_tmp];
        }

        untitled_B.rseq_tmp = untitled_B.i << 2;
        untitled_B.f[untitled_B.i] = ((((b_Kx[untitled_B.rseq_tmp + 1] *
          untitled_B.xest[1] + b_Kx[untitled_B.rseq_tmp] * untitled_B.xest[0]) +
          b_Kx[untitled_B.rseq_tmp + 2] * untitled_B.xest[2]) +
          b_Kx[untitled_B.rseq_tmp + 3] * untitled_B.xest[3]) +
          untitled_B.rtb_TmpSignalConversionAtSFun_b) +
          (((b_Ku1[untitled_B.rseq_tmp + 1] * untitled_B.old_u[1] +
             b_Ku1[untitled_B.rseq_tmp] * untitled_B.old_u[0]) +
            b_Ku1[untitled_B.rseq_tmp + 2] * untitled_B.old_u[2]) +
           b_Ku1[untitled_B.rseq_tmp + 3] * untitled_B.old_u[3]);
      }

      untitled_B.rtb_TmpSignalConversionAtSFun_b = untitled_B.xest[0];
      untitled_B.rtb_TmpSignalConversionAtSFun_p = untitled_B.xest[1];
      untitled_B.rtb_TmpSignalConversionAtSFun_c = untitled_B.xest[2];
      untitled_B.rtb_TmpSignalConversionAtSFun_f = untitled_B.xest[3];
      untitled_B.y_innov_c = untitled_B.old_u[1];
      untitled_B.old_u_m = untitled_B.old_u[0];
      untitled_B.old_u_c = untitled_B.old_u[2];
      untitled_B.old_u_k = untitled_B.old_u[3];
      for (untitled_B.i = 0; untitled_B.i < 48; untitled_B.i++) {
        untitled_B.iAout[untitled_B.i] =
          untitled_DW.Memory_PreviousInput[untitled_B.i];
        untitled_B.dv[untitled_B.i] = -(((((0.0 *
          untitled_B.rtb_TmpSignalConversionAtSFun_b + 0.0 *
          untitled_B.rtb_TmpSignalConversionAtSFun_p) + 0.0 *
          untitled_B.rtb_TmpSignalConversionAtSFun_c) + 0.0 *
          untitled_B.rtb_TmpSignalConversionAtSFun_f) + 0.5) +
          (((static_cast<real_T>(c_a[untitled_B.i + 48]) * untitled_B.y_innov_c
             + static_cast<real_T>(c_a[untitled_B.i]) * untitled_B.old_u_m) +
            static_cast<real_T>(c_a[untitled_B.i + 96]) * untitled_B.old_u_c) +
           static_cast<real_T>(c_a[untitled_B.i + 144]) * untitled_B.old_u_k));
      }

      untitled_qpkwik(b_Linv, b_Hinv, untitled_B.f, b_Ac, untitled_B.dv,
                      untitled_B.iAout, 244, 1.0E-6, untitled_B.zopt,
                      untitled_B.a__1, &untitled_B.i);
      if ((untitled_B.i < 0) || (untitled_B.i == 0)) {
        memset(&untitled_B.zopt[0], 0, 13U * sizeof(real_T));
      }

      untitled_B.rtb_TmpSignalConversionAtSFun_b =
        untitled_DW.last_x_PreviousInput[1];
      untitled_B.rtb_TmpSignalConversionAtSFun_p =
        untitled_DW.last_x_PreviousInput[0];
      untitled_B.rtb_TmpSignalConversionAtSFun_c =
        untitled_DW.last_x_PreviousInput[2];
      untitled_B.rtb_TmpSignalConversionAtSFun_f =
        untitled_DW.last_x_PreviousInput[3];
      for (untitled_B.i = 0; untitled_B.i < 4; untitled_B.i++) {
        untitled_B.y_innov_c = untitled_B.old_u[untitled_B.i] +
          untitled_B.zopt[untitled_B.i];
        untitled_B.u[untitled_B.i] = untitled_B.y_innov_c;
        untitled_B.xest[untitled_B.i] = untitled_B.y_innov_c;
        untitled_B.TmpSignalConversionAtSFunct[untitled_B.i] =
          ((static_cast<real_T>(a[untitled_B.i + 4]) *
            untitled_B.rtb_TmpSignalConversionAtSFun_b + static_cast<real_T>
            (a[untitled_B.i]) * untitled_B.rtb_TmpSignalConversionAtSFun_p) +
           static_cast<real_T>(a[untitled_B.i + 8]) *
           untitled_B.rtb_TmpSignalConversionAtSFun_c) + static_cast<real_T>
          (a[untitled_B.i + 12]) * untitled_B.rtb_TmpSignalConversionAtSFun_f;
      }

      untitled_B.rtb_TmpSignalConversionAtSFun_b = untitled_B.xest[1];
      untitled_B.old_u_m = untitled_B.xest[0];
      untitled_B.old_u_c = untitled_B.xest[2];
      untitled_B.old_u_k = untitled_B.xest[3];
      untitled_B.rtb_TmpSignalConversionAtSFun_p = untitled_B.y_innov[1];
      untitled_B.rtb_TmpSignalConversionAtSFun_c = untitled_B.y_innov[0];
      untitled_B.rtb_TmpSignalConversionAtSFun_f = untitled_B.y_innov[2];
      untitled_B.y_innov_c = untitled_B.y_innov[3];
      for (untitled_B.i = 0; untitled_B.i < 4; untitled_B.i++) {
        untitled_B.xk1[untitled_B.i] = ((((e_a[untitled_B.i + 4] *
          untitled_B.rtb_TmpSignalConversionAtSFun_b + e_a[untitled_B.i] *
          untitled_B.old_u_m) + e_a[untitled_B.i + 8] * untitled_B.old_u_c) +
          e_a[untitled_B.i + 12] * untitled_B.old_u_k) +
          untitled_B.TmpSignalConversionAtSFunct[untitled_B.i]) +
          (((f_a[untitled_B.i + 4] * untitled_B.rtb_TmpSignalConversionAtSFun_p
             + f_a[untitled_B.i] * untitled_B.rtb_TmpSignalConversionAtSFun_c) +
            f_a[untitled_B.i + 8] * untitled_B.rtb_TmpSignalConversionAtSFun_f)
           + f_a[untitled_B.i + 12] * untitled_B.y_innov_c);

        // Gain: '<S7>/umin_scale1'
        untitled_B.umin_scale1[untitled_B.i] =
          untitled_P.umin_scale1_Gain[untitled_B.i] * untitled_B.u[untitled_B.i];
      }
    }

    if (rtmIsMajorTimeStep(untitled_M) &&
        untitled_M->Timing.TaskCounters.TID[1] == 0) {
    }

    if (rtmIsMajorTimeStep(untitled_M) &&
        untitled_M->Timing.TaskCounters.TID[2] == 0) {
      // SignalConversion generated from: '<S2>/mv_vel_y'
      untitled_B.mv_vel_y = untitled_B.umin_scale1[1];

      // SignalConversion generated from: '<S2>/mv_vel_yaw'
      untitled_B.mv_vel_yaw = untitled_B.umin_scale1[3];

      // SignalConversion generated from: '<S2>/mv_vel_z'
      untitled_B.mv_vel_z = untitled_B.umin_scale1[2];
    }

    if (rtsiIsModeUpdateTimeStep(&untitled_M->solverInfo)) {
      srUpdateBC(untitled_DW.MPCController_SubsysRanBC);
    }
  }

  // End of Outputs for SubSystem: '<Root>/MPC Controller'
  if (rtmIsMajorTimeStep(untitled_M) &&
      untitled_M->Timing.TaskCounters.TID[2] == 0) {
    // Outputs for Enabled SubSystem: '<Root>/Command Velocity Publisher' incorporates:
    //   EnablePort: '<S1>/Enable'

    // Constant: '<Root>/Constant1'
    if (untitled_P.Constant1_Value_o > 0.0) {
      // BusAssignment: '<S1>/Bus Assignment1' incorporates:
      //   Constant: '<S1>/Coordinate Frame'
      //   Constant: '<S1>/Type Mask'
      //   Constant: '<S3>/Constant'
      //   DataTypeConversion: '<S1>/Data Type Conversion'

      untitled_B.BusAssignment1 = untitled_P.Constant_Value;
      untitled_B.BusAssignment1.CoordinateFrame =
        untitled_P.CoordinateFrame_Value;
      untitled_B.BusAssignment1.TypeMask = untitled_P.TypeMask_Value;
      untitled_B.BusAssignment1.Velocity.X = untitled_B.umin_scale1[0];
      untitled_B.BusAssignment1.Velocity.Y = untitled_B.mv_vel_y;
      untitled_B.BusAssignment1.Velocity.Z = untitled_B.mv_vel_z;
      untitled_B.BusAssignment1.YawRate = static_cast<real32_T>
        (untitled_B.mv_vel_yaw);

      // Outputs for Atomic SubSystem: '<S1>/Publish2'
      // MATLABSystem: '<S4>/SinkBlock'
      Pub_untitled_12.publish(&untitled_B.BusAssignment1);

      // End of Outputs for SubSystem: '<S1>/Publish2'
      if (rtsiIsModeUpdateTimeStep(&untitled_M->solverInfo)) {
        srUpdateBC(untitled_DW.CommandVelocityPublisher_Subsys);
      }
    }

    // End of Constant: '<Root>/Constant1'
    // End of Outputs for SubSystem: '<Root>/Command Velocity Publisher'
  }

  if (rtmIsMajorTimeStep(untitled_M) &&
      untitled_M->Timing.TaskCounters.TID[1] == 0) {
    // S-Function (saeroclockpacer): '<Root>/Simulation Pace'
    //
    //  The Clock Pacer generates no code, it is only active in
    //  interpreted simulation.

  }

  // Update for Enabled SubSystem: '<Root>/MPC Controller' incorporates:
  //   EnablePort: '<S2>/Enable'

  if (untitled_DW.MPCController_MODE && (rtmIsMajorTimeStep(untitled_M) &&
       untitled_M->Timing.TaskCounters.TID[2] == 0)) {
    // Update for Memory: '<S7>/Memory'
    for (untitled_B.i = 0; untitled_B.i < 48; untitled_B.i++) {
      untitled_DW.Memory_PreviousInput[untitled_B.i] =
        untitled_B.iAout[untitled_B.i];
    }

    // End of Update for Memory: '<S7>/Memory'

    // Update for UnitDelay: '<S7>/last_mv'
    untitled_DW.last_mv_DSTATE[0] = untitled_B.u[0];

    // Update for Memory: '<S7>/last_x'
    untitled_DW.last_x_PreviousInput[0] = untitled_B.xk1[0];

    // Update for UnitDelay: '<S7>/last_mv'
    untitled_DW.last_mv_DSTATE[1] = untitled_B.u[1];

    // Update for Memory: '<S7>/last_x'
    untitled_DW.last_x_PreviousInput[1] = untitled_B.xk1[1];

    // Update for UnitDelay: '<S7>/last_mv'
    untitled_DW.last_mv_DSTATE[2] = untitled_B.u[2];

    // Update for Memory: '<S7>/last_x'
    untitled_DW.last_x_PreviousInput[2] = untitled_B.xk1[2];

    // Update for UnitDelay: '<S7>/last_mv'
    untitled_DW.last_mv_DSTATE[3] = untitled_B.u[3];

    // Update for Memory: '<S7>/last_x'
    untitled_DW.last_x_PreviousInput[3] = untitled_B.xk1[3];
  }

  // End of Update for SubSystem: '<Root>/MPC Controller'
  if (rtmIsMajorTimeStep(untitled_M)) {
    // External mode
    rtExtModeUploadCheckTrigger(3);

    {                                  // Sample time: [0.0s, 0.0s]
      rtExtModeUpload(0, (real_T)untitled_M->Timing.t[0]);
    }

    if (rtmIsMajorTimeStep(untitled_M) &&
        untitled_M->Timing.TaskCounters.TID[1] == 0) {
                                  // Sample time: [0.033333333333333333s, 0.0s]
      rtExtModeUpload(1, (real_T)((untitled_M->Timing.clockTick1) *
        0.033333333333333333));
    }

    if (rtmIsMajorTimeStep(untitled_M) &&
        untitled_M->Timing.TaskCounters.TID[2] == 0) {// Sample time: [0.1s, 0.0s] 
      rtExtModeUpload(2, (real_T)((untitled_M->Timing.clockTick2) * 0.1));
    }
  }                                    // end MajorTimeStep

  if (rtmIsMajorTimeStep(untitled_M)) {
    // signal main to stop simulation
    {                                  // Sample time: [0.0s, 0.0s]
      if ((rtmGetTFinal(untitled_M)!=-1) &&
          !((rtmGetTFinal(untitled_M)-((untitled_M->Timing.clockTick1) *
             0.033333333333333333)) > ((untitled_M->Timing.clockTick1) *
            0.033333333333333333) * (DBL_EPSILON))) {
        rtmSetErrorStatus(untitled_M, "Simulation finished");
      }

      if (rtmGetStopRequested(untitled_M)) {
        rtmSetErrorStatus(untitled_M, "Simulation finished");
      }
    }

    rt_ertODEUpdateContinuousStates(&untitled_M->solverInfo);

    // Update absolute time for base rate
    // The "clockTick0" counts the number of times the code of this task has
    //  been executed. The absolute time is the multiplication of "clockTick0"
    //  and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
    //  overflow during the application lifespan selected.

    ++untitled_M->Timing.clockTick0;
    untitled_M->Timing.t[0] = rtsiGetSolverStopTime(&untitled_M->solverInfo);

    {
      // Update absolute timer for sample time: [0.033333333333333333s, 0.0s]
      // The "clockTick1" counts the number of times the code of this task has
      //  been executed. The resolution of this integer timer is 0.033333333333333333, which is the step size
      //  of the task. Size of "clockTick1" ensures timer will not overflow during the
      //  application lifespan selected.

      untitled_M->Timing.clockTick1++;
    }

    if (rtmIsMajorTimeStep(untitled_M) &&
        untitled_M->Timing.TaskCounters.TID[2] == 0) {
      // Update absolute timer for sample time: [0.1s, 0.0s]
      // The "clockTick2" counts the number of times the code of this task has
      //  been executed. The resolution of this integer timer is 0.1, which is the step size
      //  of the task. Size of "clockTick2" ensures timer will not overflow during the
      //  application lifespan selected.

      untitled_M->Timing.clockTick2++;
    }

    rate_scheduler();
  }                                    // end MajorTimeStep
}

// Derivatives for root system: '<Root>'
void untitled_derivatives(void)
{
  XDot_untitled_T *_rtXdot;
  uint32_T ri;
  _rtXdot = ((XDot_untitled_T *) untitled_M->derivs);

  // Derivatives for Enabled SubSystem: '<Root>/MPC Controller'
  if (untitled_DW.MPCController_MODE) {
    // Derivatives for StateSpace: '<S2>/Plant (State-Space)'
    _rtXdot->position[0] = 0.0;
    _rtXdot->position[1] = 0.0;
    _rtXdot->position[2] = 0.0;
    _rtXdot->position[3] = 0.0;
    for (ri = untitled_P.PlantStateSpace_B_jc[0U]; ri <
         untitled_P.PlantStateSpace_B_jc[1U]; ri++) {
      _rtXdot->position[untitled_P.PlantStateSpace_B_ir[ri]] +=
        untitled_P.PlantStateSpace_B_pr[ri] * untitled_B.umin_scale1[0U];
    }

    for (ri = untitled_P.PlantStateSpace_B_jc[1U]; ri <
         untitled_P.PlantStateSpace_B_jc[2U]; ri++) {
      _rtXdot->position[untitled_P.PlantStateSpace_B_ir[ri]] +=
        untitled_P.PlantStateSpace_B_pr[ri] * untitled_B.umin_scale1[1U];
    }

    for (ri = untitled_P.PlantStateSpace_B_jc[2U]; ri <
         untitled_P.PlantStateSpace_B_jc[3U]; ri++) {
      _rtXdot->position[untitled_P.PlantStateSpace_B_ir[ri]] +=
        untitled_P.PlantStateSpace_B_pr[ri] * untitled_B.umin_scale1[2U];
    }

    for (ri = untitled_P.PlantStateSpace_B_jc[3U]; ri <
         untitled_P.PlantStateSpace_B_jc[4U]; ri++) {
      _rtXdot->position[untitled_P.PlantStateSpace_B_ir[ri]] +=
        untitled_P.PlantStateSpace_B_pr[ri] * untitled_B.umin_scale1[3U];
    }

    // End of Derivatives for StateSpace: '<S2>/Plant (State-Space)'
  } else {
    {
      real_T *dx;
      int_T i;
      dx = &(((XDot_untitled_T *) untitled_M->derivs)->position[0]);
      for (i=0; i < 4; i++) {
        dx[i] = 0.0;
      }
    }
  }

  // End of Derivatives for SubSystem: '<Root>/MPC Controller'
}

// Model initialize function
void untitled_initialize(void)
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  {
    // Setup solver object
    rtsiSetSimTimeStepPtr(&untitled_M->solverInfo,
                          &untitled_M->Timing.simTimeStep);
    rtsiSetTPtr(&untitled_M->solverInfo, &rtmGetTPtr(untitled_M));
    rtsiSetStepSizePtr(&untitled_M->solverInfo, &untitled_M->Timing.stepSize0);
    rtsiSetdXPtr(&untitled_M->solverInfo, &untitled_M->derivs);
    rtsiSetContStatesPtr(&untitled_M->solverInfo, (real_T **)
                         &untitled_M->contStates);
    rtsiSetNumContStatesPtr(&untitled_M->solverInfo,
      &untitled_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&untitled_M->solverInfo,
      &untitled_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&untitled_M->solverInfo,
      &untitled_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&untitled_M->solverInfo,
      &untitled_M->periodicContStateRanges);
    rtsiSetContStateDisabledPtr(&untitled_M->solverInfo, (boolean_T**)
      &untitled_M->contStateDisabled);
    rtsiSetErrorStatusPtr(&untitled_M->solverInfo, (&rtmGetErrorStatus
      (untitled_M)));
    rtsiSetRTModelPtr(&untitled_M->solverInfo, untitled_M);
  }

  rtsiSetSimTimeStep(&untitled_M->solverInfo, MAJOR_TIME_STEP);
  untitled_M->intgData.y = untitled_M->odeY;
  untitled_M->intgData.f[0] = untitled_M->odeF[0];
  untitled_M->intgData.f[1] = untitled_M->odeF[1];
  untitled_M->intgData.f[2] = untitled_M->odeF[2];
  untitled_M->contStates = ((X_untitled_T *) &untitled_X);
  untitled_M->contStateDisabled = ((XDis_untitled_T *) &untitled_XDis);
  untitled_M->Timing.tStart = (0.0);
  rtsiSetSolverData(&untitled_M->solverInfo, static_cast<void *>
                    (&untitled_M->intgData));
  rtsiSetIsMinorTimeStepWithModeChange(&untitled_M->solverInfo, false);
  rtsiSetSolverName(&untitled_M->solverInfo,"ode3");
  rtmSetTPtr(untitled_M, &untitled_M->Timing.tArray[0]);
  rtmSetTFinal(untitled_M, -1);
  untitled_M->Timing.stepSize0 = 0.033333333333333333;

  // External mode info
  untitled_M->Sizes.checksums[0] = (1104759370U);
  untitled_M->Sizes.checksums[1] = (939934604U);
  untitled_M->Sizes.checksums[2] = (3211798797U);
  untitled_M->Sizes.checksums[3] = (4167976390U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[7];
    untitled_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    systemRan[1] = (sysRanDType *)&untitled_DW.CommandVelocityPublisher_Subsys;
    systemRan[2] = (sysRanDType *)&untitled_DW.CommandVelocityPublisher_Subsys;
    systemRan[3] = (sysRanDType *)&untitled_DW.CommandVelocityPublisher_Subsys;
    systemRan[4] = (sysRanDType *)&untitled_DW.CommandVelocityPublisher_Subsys;
    systemRan[5] = (sysRanDType *)&untitled_DW.MPCController_SubsysRanBC;
    systemRan[6] = (sysRanDType *)&untitled_DW.MPCController_SubsysRanBC;
    rteiSetModelMappingInfoPtr(untitled_M->extModeInfo,
      &untitled_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(untitled_M->extModeInfo, untitled_M->Sizes.checksums);
    rteiSetTPtr(untitled_M->extModeInfo, rtmGetTPtr(untitled_M));
  }

  // data type transition information
  {
    static DataTypeTransInfo dtInfo;
    untitled_M->SpecialInfo.mappingInfo = (&dtInfo);
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
    untitled_X.position[0] = untitled_P.PlantStateSpace_InitialConditio[0];
    untitled_X.position[1] = untitled_P.PlantStateSpace_InitialConditio[1];
    untitled_X.position[2] = untitled_P.PlantStateSpace_InitialConditio[2];
    untitled_X.position[3] = untitled_P.PlantStateSpace_InitialConditio[3];

    // InitializeConditions for Memory: '<S7>/Memory'
    for (i = 0; i < 48; i++) {
      untitled_DW.Memory_PreviousInput[i] = untitled_P.Memory_InitialCondition[i];
    }

    // End of InitializeConditions for Memory: '<S7>/Memory'

    // InitializeConditions for UnitDelay: '<S7>/last_mv'
    untitled_DW.last_mv_DSTATE[0] = untitled_P.last_mv_InitialCondition[0];

    // InitializeConditions for Memory: '<S7>/last_x'
    untitled_DW.last_x_PreviousInput[0] = untitled_P.last_x_InitialCondition[0];

    // InitializeConditions for UnitDelay: '<S7>/last_mv'
    untitled_DW.last_mv_DSTATE[1] = untitled_P.last_mv_InitialCondition[1];

    // InitializeConditions for Memory: '<S7>/last_x'
    untitled_DW.last_x_PreviousInput[1] = untitled_P.last_x_InitialCondition[1];

    // InitializeConditions for UnitDelay: '<S7>/last_mv'
    untitled_DW.last_mv_DSTATE[2] = untitled_P.last_mv_InitialCondition[2];

    // InitializeConditions for Memory: '<S7>/last_x'
    untitled_DW.last_x_PreviousInput[2] = untitled_P.last_x_InitialCondition[2];

    // InitializeConditions for UnitDelay: '<S7>/last_mv'
    untitled_DW.last_mv_DSTATE[3] = untitled_P.last_mv_InitialCondition[3];

    // InitializeConditions for Memory: '<S7>/last_x'
    untitled_DW.last_x_PreviousInput[3] = untitled_P.last_x_InitialCondition[3];

    // SystemInitialize for Outport: '<S2>/mv_vel_x' incorporates:
    //   Gain: '<S7>/umin_scale1'

    untitled_B.umin_scale1[0] = untitled_P.mv_vel_x_Y0;

    // SystemInitialize for SignalConversion generated from: '<S2>/mv_vel_y' incorporates:
    //   Outport: '<S2>/mv_vel_y'

    untitled_B.mv_vel_y = untitled_P.mv_vel_y_Y0;

    // SystemInitialize for SignalConversion generated from: '<S2>/mv_vel_z' incorporates:
    //   Outport: '<S2>/mv_vel_z'

    untitled_B.mv_vel_z = untitled_P.mv_vel_z_Y0;

    // SystemInitialize for SignalConversion generated from: '<S2>/mv_vel_yaw' incorporates:
    //   Outport: '<S2>/mv_vel_yaw'

    untitled_B.mv_vel_yaw = untitled_P.mv_vel_yaw_Y0;

    // End of SystemInitialize for SubSystem: '<Root>/MPC Controller'

    // SystemInitialize for Enabled SubSystem: '<Root>/Command Velocity Publisher' 
    // SystemInitialize for Atomic SubSystem: '<S1>/Publish2'
    // Start for MATLABSystem: '<S4>/SinkBlock'
    untitled_DW.obj.matlabCodegenIsDeleted = false;
    untitled_DW.obj.isInitialized = 1;
    for (i = 0; i < 27; i++) {
      b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
    }

    Pub_untitled_12.createPublisher(&b_zeroDelimTopic[0], 105);
    untitled_DW.obj.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S4>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<S1>/Publish2'
    // End of SystemInitialize for SubSystem: '<Root>/Command Velocity Publisher' 
  }
}

// Model terminate function
void untitled_terminate(void)
{
  // Terminate for Enabled SubSystem: '<Root>/Command Velocity Publisher'
  // Terminate for Atomic SubSystem: '<S1>/Publish2'
  // Terminate for MATLABSystem: '<S4>/SinkBlock'
  if (!untitled_DW.obj.matlabCodegenIsDeleted) {
    untitled_DW.obj.matlabCodegenIsDeleted = true;
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
