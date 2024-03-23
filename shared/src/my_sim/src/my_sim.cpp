//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: my_sim.cpp
//
// Code generated for Simulink model 'my_sim'.
//
// Model version                  : 1.3
// Simulink Coder version         : 23.2 (R2023b) 01-Aug-2023
// C/C++ source code generated on : Thu Feb  8 20:32:14 2024
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "my_sim.h"
#include "rtwtypes.h"
#include <math.h>
#include <string.h>
#include "my_sim_private.h"

extern "C"
{

#include "rt_nonfinite.h"

}

#include "my_sim_dt.h"

// Named constants for MATLAB Function: '<S26>/optimizer'
const int32_T my_sim_degrees = 13;

// Block signals (default storage)
B_my_sim_T my_sim_B;

// Continuous states
X_my_sim_T my_sim_X;

// Disabled State Vector
XDis_my_sim_T my_sim_XDis;

// Block states (default storage)
DW_my_sim_T my_sim_DW;

// Real-time model
RT_MODEL_my_sim_T my_sim_M_ = RT_MODEL_my_sim_T();
RT_MODEL_my_sim_T *const my_sim_M = &my_sim_M_;

// Forward declaration for local functions
static real_T my_sim_norm(const real_T x[13]);
static real_T my_sim_maximum(const real_T x[13]);
static real_T my_sim_xnrm2(int32_T n, const real_T x[169], int32_T ix0);
static void my_sim_xgemv(int32_T b_m, int32_T n, const real_T b_A[169], int32_T
  ia0, const real_T x[169], int32_T ix0, real_T y[13]);
static void my_sim_xgerc(int32_T b_m, int32_T n, real_T alpha1, int32_T ix0,
  const real_T y[13], real_T b_A[169], int32_T ia0);
static void my_sim_KWIKfactor(const real_T b_Ac[624], const int32_T iC[48],
  int32_T nA, const real_T b_Linv[169], real_T D[169], real_T b_H[169], int32_T
  n, real_T RLinv[169], real_T *Status);
static void my_sim_DropConstraint(int32_T kDrop, boolean_T iA[48], int32_T *nA,
  int32_T iC[48]);
static void my_sim_qpkwik(const real_T b_Linv[169], const real_T b_Hinv[169],
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

  (my_sim_M->Timing.TaskCounters.TID[2])++;
  if ((my_sim_M->Timing.TaskCounters.TID[2]) > 2) {// Sample time: [0.1s, 0.0s]
    my_sim_M->Timing.TaskCounters.TID[2] = 0;
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
  my_sim_derivatives();

  // f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*));
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  my_sim_step();
  my_sim_derivatives();

  // f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*));
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  my_sim_step();
  my_sim_derivatives();

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

// Function for MATLAB Function: '<S26>/optimizer'
static real_T my_sim_norm(const real_T x[13])
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

// Function for MATLAB Function: '<S26>/optimizer'
static real_T my_sim_maximum(const real_T x[13])
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

// Function for MATLAB Function: '<S26>/optimizer'
static real_T my_sim_xnrm2(int32_T n, const real_T x[169], int32_T ix0)
{
  real_T y;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x[ix0 - 1]);
    } else {
      int32_T kend;
      my_sim_B.scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (int32_T k = ix0; k <= kend; k++) {
        my_sim_B.absxk = fabs(x[k - 1]);
        if (my_sim_B.absxk > my_sim_B.scale) {
          my_sim_B.t = my_sim_B.scale / my_sim_B.absxk;
          y = y * my_sim_B.t * my_sim_B.t + 1.0;
          my_sim_B.scale = my_sim_B.absxk;
        } else {
          my_sim_B.t = my_sim_B.absxk / my_sim_B.scale;
          y += my_sim_B.t * my_sim_B.t;
        }
      }

      y = my_sim_B.scale * sqrt(y);
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

// Function for MATLAB Function: '<S26>/optimizer'
static void my_sim_xgemv(int32_T b_m, int32_T n, const real_T b_A[169], int32_T
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
      my_sim_B.c = 0.0;
      d = (b_iy + b_m) - 1;
      for (ia = b_iy; ia <= d; ia++) {
        my_sim_B.c += x[((ix0 + ia) - b_iy) - 1] * b_A[ia - 1];
      }

      ia = div_nzp_s32_floor(b_iy - ia0, 13);
      y[ia] += my_sim_B.c;
    }
  }
}

// Function for MATLAB Function: '<S26>/optimizer'
static void my_sim_xgerc(int32_T b_m, int32_T n, real_T alpha1, int32_T ix0,
  const real_T y[13], real_T b_A[169], int32_T ia0)
{
  if (!(alpha1 == 0.0)) {
    int32_T jA;
    jA = ia0;
    for (int32_T j = 0; j < n; j++) {
      my_sim_B.temp = y[j];
      if (my_sim_B.temp != 0.0) {
        int32_T b;
        my_sim_B.temp *= alpha1;
        b = b_m + jA;
        for (int32_T ijA = jA; ijA < b; ijA++) {
          b_A[ijA - 1] += b_A[((ix0 + ijA) - jA) - 1] * my_sim_B.temp;
        }
      }

      jA += 13;
    }
  }
}

// Function for MATLAB Function: '<S26>/optimizer'
static void my_sim_KWIKfactor(const real_T b_Ac[624], const int32_T iC[48],
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
      my_sim_B.beta1 = 0.0;
      for (ii = 0; ii < 13; ii++) {
        my_sim_B.beta1 += b_Ac[(48 * ii + knt) - 1] * b_Linv[13 * ii + i];
      }

      RLinv[i + 13 * b_lastv] = my_sim_B.beta1;
    }
  }

  memcpy(&my_sim_B.b_A[0], &RLinv[0], 169U * sizeof(real_T));
  memset(&my_sim_B.tau[0], 0, 13U * sizeof(real_T));
  memset(&my_sim_B.work[0], 0, 13U * sizeof(real_T));
  for (i = 0; i < 13; i++) {
    ii = i * 13 + i;
    if (i + 1 < 13) {
      my_sim_B.atmp = my_sim_B.b_A[ii];
      b_lastv = ii + 2;
      my_sim_B.tau[i] = 0.0;
      my_sim_B.beta1 = my_sim_xnrm2(12 - i, my_sim_B.b_A, ii + 2);
      if (my_sim_B.beta1 != 0.0) {
        my_sim_B.b_A_g = my_sim_B.b_A[ii];
        my_sim_B.beta1 = rt_hypotd_snf(my_sim_B.b_A_g, my_sim_B.beta1);
        if (my_sim_B.b_A_g >= 0.0) {
          my_sim_B.beta1 = -my_sim_B.beta1;
        }

        if (fabs(my_sim_B.beta1) < 1.0020841800044864E-292) {
          knt = 0;
          f_tmp = (ii - i) + 13;
          do {
            knt++;
            for (b_coltop = b_lastv; b_coltop <= f_tmp; b_coltop++) {
              my_sim_B.b_A[b_coltop - 1] *= 9.9792015476736E+291;
            }

            my_sim_B.beta1 *= 9.9792015476736E+291;
            my_sim_B.atmp *= 9.9792015476736E+291;
          } while ((fabs(my_sim_B.beta1) < 1.0020841800044864E-292) && (knt < 20));

          my_sim_B.beta1 = rt_hypotd_snf(my_sim_B.atmp, my_sim_xnrm2(12 - i,
            my_sim_B.b_A, ii + 2));
          if (my_sim_B.atmp >= 0.0) {
            my_sim_B.beta1 = -my_sim_B.beta1;
          }

          my_sim_B.tau[i] = (my_sim_B.beta1 - my_sim_B.atmp) / my_sim_B.beta1;
          my_sim_B.atmp = 1.0 / (my_sim_B.atmp - my_sim_B.beta1);
          for (b_coltop = b_lastv; b_coltop <= f_tmp; b_coltop++) {
            my_sim_B.b_A[b_coltop - 1] *= my_sim_B.atmp;
          }

          for (b_lastv = 0; b_lastv < knt; b_lastv++) {
            my_sim_B.beta1 *= 1.0020841800044864E-292;
          }

          my_sim_B.atmp = my_sim_B.beta1;
        } else {
          my_sim_B.tau[i] = (my_sim_B.beta1 - my_sim_B.b_A_g) / my_sim_B.beta1;
          my_sim_B.atmp = 1.0 / (my_sim_B.b_A_g - my_sim_B.beta1);
          b_coltop = (ii - i) + 13;
          for (knt = b_lastv; knt <= b_coltop; knt++) {
            my_sim_B.b_A[knt - 1] *= my_sim_B.atmp;
          }

          my_sim_B.atmp = my_sim_B.beta1;
        }
      }

      my_sim_B.b_A[ii] = 1.0;
      if (my_sim_B.tau[i] != 0.0) {
        b_lastv = 13 - i;
        knt = (ii - i) + 12;
        while ((b_lastv > 0) && (my_sim_B.b_A[knt] == 0.0)) {
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
              if (my_sim_B.b_A[f_tmp] != 0.0) {
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
        my_sim_xgemv(b_lastv, knt, my_sim_B.b_A, ii + 14, my_sim_B.b_A, ii + 1,
                     my_sim_B.work);
        my_sim_xgerc(b_lastv, knt, -my_sim_B.tau[i], ii + 1, my_sim_B.work,
                     my_sim_B.b_A, ii + 14);
      }

      my_sim_B.b_A[ii] = my_sim_B.atmp;
    } else {
      my_sim_B.tau[12] = 0.0;
    }
  }

  for (i = 0; i < 13; i++) {
    for (ii = 0; ii <= i; ii++) {
      my_sim_B.R[ii + 13 * i] = my_sim_B.b_A[13 * i + ii];
    }

    for (ii = i + 2; ii < 14; ii++) {
      my_sim_B.R[(ii + 13 * i) - 1] = 0.0;
    }

    my_sim_B.work[i] = 0.0;
  }

  for (i = 12; i >= 0; i--) {
    ii = (i * 13 + i) + 14;
    if (i + 1 < 13) {
      my_sim_B.b_A[ii - 14] = 1.0;
      if (my_sim_B.tau[i] != 0.0) {
        b_lastv = 13 - i;
        knt = ii - i;
        while ((b_lastv > 0) && (my_sim_B.b_A[knt - 2] == 0.0)) {
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
              if (my_sim_B.b_A[f_tmp - 1] != 0.0) {
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
        my_sim_xgemv(b_lastv, knt, my_sim_B.b_A, ii, my_sim_B.b_A, ii - 13,
                     my_sim_B.work);
        my_sim_xgerc(b_lastv, knt, -my_sim_B.tau[i], ii - 13, my_sim_B.work,
                     my_sim_B.b_A, ii);
      }

      knt = (ii - i) - 1;
      for (b_lastv = ii - 12; b_lastv <= knt; b_lastv++) {
        my_sim_B.b_A[b_lastv - 1] *= -my_sim_B.tau[i];
      }
    }

    my_sim_B.b_A[ii - 14] = 1.0 - my_sim_B.tau[i];
    for (b_lastv = 0; b_lastv < i; b_lastv++) {
      my_sim_B.b_A[(ii - b_lastv) - 15] = 0.0;
    }
  }

  i = 0;
  do {
    exitg1 = 0;
    if (i <= nA - 1) {
      if (fabs(my_sim_B.R[13 * i + i]) < 1.0E-12) {
        *Status = -2.0;
        exitg1 = 1;
      } else {
        i++;
      }
    } else {
      for (ii = 0; ii < n; ii++) {
        for (b_lastv = 0; b_lastv < n; b_lastv++) {
          my_sim_B.beta1 = 0.0;
          for (i = 0; i < 13; i++) {
            my_sim_B.beta1 += b_Linv[13 * ii + i] * my_sim_B.b_A[13 * b_lastv +
              i];
          }

          my_sim_B.TL[ii + 13 * b_lastv] = my_sim_B.beta1;
        }
      }

      memset(&RLinv[0], 0, 169U * sizeof(real_T));
      for (i = nA; i >= 1; i--) {
        knt = (i - 1) * 13;
        b_coltop = (i + knt) - 1;
        RLinv[b_coltop] = 1.0;
        for (ii = i; ii <= nA; ii++) {
          f_tmp = ((ii - 1) * 13 + i) - 1;
          RLinv[f_tmp] /= my_sim_B.R[b_coltop];
        }

        if (i > 1) {
          for (ii = 0; ii <= i - 2; ii++) {
            for (b_lastv = i; b_lastv <= nA; b_lastv++) {
              b_coltop = (b_lastv - 1) * 13;
              f_tmp = b_coltop + ii;
              RLinv[f_tmp] -= RLinv[(b_coltop + i) - 1] * my_sim_B.R[knt + ii];
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
            b_H[i] -= my_sim_B.TL[(ii + knt) - 1] * my_sim_B.TL[ii + b_lastv];
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
            D[i] += my_sim_B.TL[ii + knt] * RLinv[ii + b_lastv];
          }
        }
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);
}

// Function for MATLAB Function: '<S26>/optimizer'
static void my_sim_DropConstraint(int32_T kDrop, boolean_T iA[48], int32_T *nA,
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

// Function for MATLAB Function: '<S26>/optimizer'
static void my_sim_qpkwik(const real_T b_Linv[169], const real_T b_Hinv[169],
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
  memset(&my_sim_B.r[0], 0, 13U * sizeof(real_T));
  my_sim_B.rMin = 0.0;
  cTolComputed = false;
  for (i = 0; i < 48; i++) {
    my_sim_B.cTol[i] = 1.0;
    my_sim_B.iC[i] = 0;
  }

  my_sim_B.nA = 0;
  for (i = 0; i < 48; i++) {
    if (iA[i]) {
      my_sim_B.nA++;
      my_sim_B.iC[my_sim_B.nA - 1] = i + 1;
    }
  }

  guard1 = false;
  if (my_sim_B.nA > 0) {
    memset(&my_sim_B.Opt[0], 0, 26U * sizeof(real_T));
    for (i = 0; i < 13; i++) {
      my_sim_B.Rhs[i] = f[i];
      my_sim_B.Rhs[i + 13] = 0.0;
    }

    DualFeasible = false;
    my_sim_B.tmp = static_cast<int32_T>(rt_roundd_snf(0.3 * static_cast<real_T>
      (my_sim_B.nA)));
    ColdReset = false;
    do {
      exitg3 = 0;
      if ((!DualFeasible) && (my_sim_B.nA > 0) && (*status <= maxiter)) {
        my_sim_KWIKfactor(b_Ac, my_sim_B.iC, my_sim_B.nA, b_Linv, my_sim_B.D,
                          my_sim_B.b_H, my_sim_degrees, my_sim_B.RLinv,
                          &my_sim_B.Xnorm0);
        if (my_sim_B.Xnorm0 < 0.0) {
          if (ColdReset) {
            *status = -2;
            exitg3 = 2;
          } else {
            my_sim_B.nA = 0;
            memset(&my_sim_B.iC[0], 0, 48U * sizeof(int32_T));
            for (i = 0; i < 48; i++) {
              iA[i] = false;
            }

            ColdReset = true;
          }
        } else {
          for (i = 0; i < my_sim_B.nA; i++) {
            my_sim_B.Rhs[i + 13] = b[my_sim_B.iC[i] - 1];
            for (my_sim_B.kDrop = i + 1; my_sim_B.kDrop <= my_sim_B.nA;
                 my_sim_B.kDrop++) {
              U_tmp = (13 * i + my_sim_B.kDrop) - 1;
              my_sim_B.U[U_tmp] = 0.0;
              for (iSave = 0; iSave < my_sim_B.nA; iSave++) {
                my_sim_B.U[U_tmp] += my_sim_B.RLinv[(13 * iSave + my_sim_B.kDrop)
                  - 1] * my_sim_B.RLinv[13 * iSave + i];
              }

              my_sim_B.U[i + 13 * (my_sim_B.kDrop - 1)] = my_sim_B.U[U_tmp];
            }
          }

          for (my_sim_B.kDrop = 0; my_sim_B.kDrop < 13; my_sim_B.kDrop++) {
            my_sim_B.Xnorm0 = 0.0;
            for (i = 0; i < 13; i++) {
              my_sim_B.Xnorm0 += my_sim_B.b_H[13 * i + my_sim_B.kDrop] *
                my_sim_B.Rhs[i];
            }

            my_sim_B.Opt[my_sim_B.kDrop] = my_sim_B.Xnorm0;
            for (i = 0; i < my_sim_B.nA; i++) {
              my_sim_B.Opt[my_sim_B.kDrop] += my_sim_B.D[13 * i + my_sim_B.kDrop]
                * my_sim_B.Rhs[i + 13];
            }
          }

          my_sim_B.Xnorm0 = -1.0E-12;
          my_sim_B.kDrop = -1;
          for (iSave = 0; iSave < my_sim_B.nA; iSave++) {
            my_sim_B.cMin = 0.0;
            for (i = 0; i < 13; i++) {
              my_sim_B.cMin += my_sim_B.D[13 * iSave + i] * my_sim_B.Rhs[i];
            }

            my_sim_B.Opt[iSave + 13] = my_sim_B.cMin;
            for (i = 0; i < my_sim_B.nA; i++) {
              my_sim_B.Opt[iSave + 13] += my_sim_B.U[13 * i + iSave] *
                my_sim_B.Rhs[i + 13];
            }

            my_sim_B.cMin = my_sim_B.Opt[iSave + 13];
            lambda[my_sim_B.iC[iSave] - 1] = my_sim_B.cMin;
            if ((my_sim_B.cMin < my_sim_B.Xnorm0) && (iSave + 1 <= my_sim_B.nA))
            {
              my_sim_B.kDrop = iSave;
              my_sim_B.Xnorm0 = my_sim_B.cMin;
            }
          }

          if (my_sim_B.kDrop + 1 <= 0) {
            DualFeasible = true;
            memcpy(&x[0], &my_sim_B.Opt[0], 13U * sizeof(real_T));
          } else {
            (*status)++;
            if (my_sim_B.tmp <= 5) {
              i = 5;
            } else {
              i = my_sim_B.tmp;
            }

            if (*status > i) {
              my_sim_B.nA = 0;
              memset(&my_sim_B.iC[0], 0, 48U * sizeof(int32_T));
              for (i = 0; i < 48; i++) {
                iA[i] = false;
              }

              ColdReset = true;
            } else {
              lambda[my_sim_B.iC[my_sim_B.kDrop] - 1] = 0.0;
              my_sim_DropConstraint(my_sim_B.kDrop + 1, iA, &my_sim_B.nA,
                                    my_sim_B.iC);
            }
          }
        }
      } else {
        if (my_sim_B.nA <= 0) {
          memset(&lambda[0], 0, 48U * sizeof(real_T));
          for (my_sim_B.tmp = 0; my_sim_B.tmp < 13; my_sim_B.tmp++) {
            my_sim_B.Xnorm0 = 0.0;
            for (i = 0; i < 13; i++) {
              my_sim_B.Xnorm0 += -b_Hinv[13 * i + my_sim_B.tmp] * f[i];
            }

            x[my_sim_B.tmp] = my_sim_B.Xnorm0;
          }
        }

        exitg3 = 1;
      }
    } while (exitg3 == 0);

    if (exitg3 == 1) {
      guard1 = true;
    }
  } else {
    for (my_sim_B.tmp = 0; my_sim_B.tmp < 13; my_sim_B.tmp++) {
      my_sim_B.Xnorm0 = 0.0;
      for (i = 0; i < 13; i++) {
        my_sim_B.Xnorm0 += -b_Hinv[13 * i + my_sim_B.tmp] * f[i];
      }

      x[my_sim_B.tmp] = my_sim_B.Xnorm0;
    }

    guard1 = true;
  }

  if (guard1) {
    my_sim_B.Xnorm0 = my_sim_norm(x);
    exitg2 = false;
    while ((!exitg2) && (*status <= maxiter)) {
      my_sim_B.cMin = -FeasTol;
      my_sim_B.tmp = -1;
      for (my_sim_B.kDrop = 0; my_sim_B.kDrop < 48; my_sim_B.kDrop++) {
        if (!cTolComputed) {
          for (i = 0; i < 13; i++) {
            my_sim_B.z[i] = fabs(b_Ac[48 * i + my_sim_B.kDrop] * x[i]);
          }

          my_sim_B.cVal = my_sim_maximum(my_sim_B.z);
          if ((my_sim_B.cTol[my_sim_B.kDrop] >= my_sim_B.cVal) || rtIsNaN
              (my_sim_B.cVal)) {
          } else {
            my_sim_B.cTol[my_sim_B.kDrop] = my_sim_B.cVal;
          }
        }

        if (!iA[my_sim_B.kDrop]) {
          my_sim_B.cVal = 0.0;
          for (i = 0; i < 13; i++) {
            my_sim_B.cVal += b_Ac[48 * i + my_sim_B.kDrop] * x[i];
          }

          my_sim_B.cVal = (my_sim_B.cVal - b[my_sim_B.kDrop]) /
            my_sim_B.cTol[my_sim_B.kDrop];
          if (my_sim_B.cVal < my_sim_B.cMin) {
            my_sim_B.cMin = my_sim_B.cVal;
            my_sim_B.tmp = my_sim_B.kDrop;
          }
        }
      }

      cTolComputed = true;
      if (my_sim_B.tmp + 1 <= 0) {
        exitg2 = true;
      } else if (*status == maxiter) {
        *status = 0;
        exitg2 = true;
      } else {
        do {
          exitg1 = 0;
          if ((my_sim_B.tmp + 1 > 0) && (*status <= maxiter)) {
            guard2 = false;
            if (my_sim_B.nA == 0) {
              for (i = 0; i < 13; i++) {
                my_sim_B.cMin = 0.0;
                for (my_sim_B.kDrop = 0; my_sim_B.kDrop < 13; my_sim_B.kDrop++)
                {
                  my_sim_B.cMin += b_Hinv[13 * my_sim_B.kDrop + i] * b_Ac[48 *
                    my_sim_B.kDrop + my_sim_B.tmp];
                }

                my_sim_B.z[i] = my_sim_B.cMin;
              }

              guard2 = true;
            } else {
              my_sim_KWIKfactor(b_Ac, my_sim_B.iC, my_sim_B.nA, b_Linv,
                                my_sim_B.D, my_sim_B.b_H, my_sim_degrees,
                                my_sim_B.RLinv, &my_sim_B.cMin);
              if (my_sim_B.cMin <= 0.0) {
                *status = -2;
                exitg1 = 1;
              } else {
                for (i = 0; i < 169; i++) {
                  my_sim_B.U[i] = -my_sim_B.b_H[i];
                }

                for (i = 0; i < 13; i++) {
                  my_sim_B.cMin = 0.0;
                  for (my_sim_B.kDrop = 0; my_sim_B.kDrop < 13; my_sim_B.kDrop++)
                  {
                    my_sim_B.cMin += my_sim_B.U[13 * my_sim_B.kDrop + i] * b_Ac
                      [48 * my_sim_B.kDrop + my_sim_B.tmp];
                  }

                  my_sim_B.z[i] = my_sim_B.cMin;
                }

                for (my_sim_B.kDrop = 0; my_sim_B.kDrop < my_sim_B.nA;
                     my_sim_B.kDrop++) {
                  my_sim_B.cVal = 0.0;
                  for (i = 0; i < 13; i++) {
                    my_sim_B.cVal += b_Ac[48 * i + my_sim_B.tmp] * my_sim_B.D[13
                      * my_sim_B.kDrop + i];
                  }

                  my_sim_B.r[my_sim_B.kDrop] = my_sim_B.cVal;
                }

                guard2 = true;
              }
            }

            if (guard2) {
              my_sim_B.kDrop = 0;
              my_sim_B.cMin = 0.0;
              DualFeasible = true;
              ColdReset = true;
              if (my_sim_B.nA > 0) {
                i = 0;
                exitg4 = false;
                while ((!exitg4) && (i <= my_sim_B.nA - 1)) {
                  if (my_sim_B.r[i] >= 1.0E-12) {
                    ColdReset = false;
                    exitg4 = true;
                  } else {
                    i++;
                  }
                }
              }

              if ((my_sim_B.nA != 0) && (!ColdReset)) {
                for (i = 0; i < my_sim_B.nA; i++) {
                  my_sim_B.cVal = my_sim_B.r[i];
                  if (my_sim_B.cVal > 1.0E-12) {
                    my_sim_B.cVal = lambda[my_sim_B.iC[i] - 1] / my_sim_B.cVal;
                    if ((my_sim_B.kDrop == 0) || (my_sim_B.cVal < my_sim_B.rMin))
                    {
                      my_sim_B.rMin = my_sim_B.cVal;
                      my_sim_B.kDrop = i + 1;
                    }
                  }
                }

                if (my_sim_B.kDrop > 0) {
                  my_sim_B.cMin = my_sim_B.rMin;
                  DualFeasible = false;
                }
              }

              my_sim_B.zTa = 0.0;
              for (i = 0; i < 13; i++) {
                my_sim_B.zTa += b_Ac[48 * i + my_sim_B.tmp] * my_sim_B.z[i];
              }

              if (my_sim_B.zTa <= 0.0) {
                my_sim_B.cVal = 0.0;
                ColdReset = true;
              } else {
                my_sim_B.cVal = 0.0;
                for (i = 0; i < 13; i++) {
                  my_sim_B.cVal += b_Ac[48 * i + my_sim_B.tmp] * x[i];
                }

                my_sim_B.cVal = (b[my_sim_B.tmp] - my_sim_B.cVal) / my_sim_B.zTa;
                ColdReset = false;
              }

              if (DualFeasible && ColdReset) {
                *status = -1;
                exitg1 = 1;
              } else {
                if (ColdReset) {
                  my_sim_B.zTa = my_sim_B.cMin;
                } else if (DualFeasible) {
                  my_sim_B.zTa = my_sim_B.cVal;
                } else if (my_sim_B.cMin < my_sim_B.cVal) {
                  my_sim_B.zTa = my_sim_B.cMin;
                } else {
                  my_sim_B.zTa = my_sim_B.cVal;
                }

                for (i = 0; i < my_sim_B.nA; i++) {
                  iSave = my_sim_B.iC[i];
                  lambda[iSave - 1] -= my_sim_B.zTa * my_sim_B.r[i];
                  if ((iSave <= 48) && (lambda[iSave - 1] < 0.0)) {
                    lambda[iSave - 1] = 0.0;
                  }
                }

                lambda[my_sim_B.tmp] += my_sim_B.zTa;
                frexp(1.0, &exponent);
                if (fabs(my_sim_B.zTa - my_sim_B.cMin) < 2.2204460492503131E-16)
                {
                  my_sim_DropConstraint(my_sim_B.kDrop, iA, &my_sim_B.nA,
                                        my_sim_B.iC);
                }

                if (!ColdReset) {
                  for (i = 0; i < 13; i++) {
                    x[i] += my_sim_B.zTa * my_sim_B.z[i];
                  }

                  frexp(1.0, &b_exponent);
                  if (fabs(my_sim_B.zTa - my_sim_B.cVal) <
                      2.2204460492503131E-16) {
                    if (my_sim_B.nA == my_sim_degrees) {
                      *status = -1;
                      exitg1 = 1;
                    } else {
                      my_sim_B.nA++;
                      my_sim_B.iC[my_sim_B.nA - 1] = my_sim_B.tmp + 1;
                      my_sim_B.kDrop = my_sim_B.nA - 1;
                      exitg4 = false;
                      while ((!exitg4) && (my_sim_B.kDrop + 1 > 1)) {
                        i = my_sim_B.iC[my_sim_B.kDrop - 1];
                        if (my_sim_B.iC[my_sim_B.kDrop] > i) {
                          exitg4 = true;
                        } else {
                          iSave = my_sim_B.iC[my_sim_B.kDrop];
                          my_sim_B.iC[my_sim_B.kDrop] = i;
                          my_sim_B.iC[my_sim_B.kDrop - 1] = iSave;
                          my_sim_B.kDrop--;
                        }
                      }

                      iA[my_sim_B.tmp] = true;
                      my_sim_B.tmp = -1;
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
            my_sim_B.cMin = my_sim_norm(x);
            if (fabs(my_sim_B.cMin - my_sim_B.Xnorm0) > 0.001) {
              my_sim_B.Xnorm0 = my_sim_B.cMin;
              for (i = 0; i < 48; i++) {
                my_sim_B.cMin = fabs(b[i]);
                if (my_sim_B.cMin >= 1.0) {
                  my_sim_B.cTol[i] = my_sim_B.cMin;
                } else {
                  my_sim_B.cTol[i] = 1.0;
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
void my_sim_step(void)
{
  if (rtmIsMajorTimeStep(my_sim_M)) {
    // set solver stop time
    rtsiSetSolverStopTime(&my_sim_M->solverInfo,((my_sim_M->Timing.clockTick0+1)*
      my_sim_M->Timing.stepSize0));
  }                                    // end MajorTimeStep

  // Update absolute time of base rate at minor time step
  if (rtmIsMinorTimeStep(my_sim_M)) {
    my_sim_M->Timing.t[0] = rtsiGetT(&my_sim_M->solverInfo);
  }

  {
    static const int8_T a[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1
    };

    static const real_T b_a[16] = { 0.99019513592784836, 0.0, 0.0, 0.0, 0.0,
      0.99019513592784836, 0.0, 0.0, 0.0, 0.0, 0.99019513592784836, 0.0, 0.0,
      0.0, 0.0, 0.99019513592784825 };

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

    static const real_T f_a[16] = { 0.99019513592784736, -0.0, -0.0, -0.0, 0.0,
      0.99019513592784736, -0.0, -0.0, 0.0, 0.0, 0.99019513592784736, -0.0, 0.0,
      0.0, 0.0, 0.99019513592784891 };

    // StateSpace: '<Root>/State-Space'
    my_sim_B.measuredoutput[0] = 0.0;
    my_sim_B.measuredoutput[1] = 0.0;
    my_sim_B.measuredoutput[2] = 0.0;
    my_sim_B.measuredoutput[3] = 0.0;
    my_sim_B.ri = my_sim_P.StateSpace_C_jc[0U];
    while (my_sim_B.ri < my_sim_P.StateSpace_C_jc[1U]) {
      my_sim_B.measuredoutput[my_sim_P.StateSpace_C_ir[my_sim_B.ri]] +=
        my_sim_P.StateSpace_C_pr[my_sim_B.ri] * my_sim_X.position[0U];
      my_sim_B.ri++;
    }

    my_sim_B.ri = my_sim_P.StateSpace_C_jc[1U];
    while (my_sim_B.ri < my_sim_P.StateSpace_C_jc[2U]) {
      my_sim_B.measuredoutput[my_sim_P.StateSpace_C_ir[my_sim_B.ri]] +=
        my_sim_P.StateSpace_C_pr[my_sim_B.ri] * my_sim_X.position[1U];
      my_sim_B.ri++;
    }

    my_sim_B.ri = my_sim_P.StateSpace_C_jc[2U];
    while (my_sim_B.ri < my_sim_P.StateSpace_C_jc[3U]) {
      my_sim_B.measuredoutput[my_sim_P.StateSpace_C_ir[my_sim_B.ri]] +=
        my_sim_P.StateSpace_C_pr[my_sim_B.ri] * my_sim_X.position[2U];
      my_sim_B.ri++;
    }

    my_sim_B.ri = my_sim_P.StateSpace_C_jc[3U];
    while (my_sim_B.ri < my_sim_P.StateSpace_C_jc[4U]) {
      my_sim_B.measuredoutput[my_sim_P.StateSpace_C_ir[my_sim_B.ri]] +=
        my_sim_P.StateSpace_C_pr[my_sim_B.ri] * my_sim_X.position[3U];
      my_sim_B.ri++;
    }

    // End of StateSpace: '<Root>/State-Space'

    // Sin: '<Root>/Sine Wave'
    my_sim_B.x_ref = sin(my_sim_P.SineWave_Freq * my_sim_M->Timing.t[0] +
                         my_sim_P.SineWave_Phase) * my_sim_P.SineWave_Amp +
      my_sim_P.SineWave_Bias;
    if (rtmIsMajorTimeStep(my_sim_M) &&
        my_sim_M->Timing.TaskCounters.TID[1] == 0) {
      // Constant: '<Root>/Constant1'
      my_sim_B.y_ref = my_sim_P.Constant1_Value;

      // Constant: '<Root>/Constant2'
      my_sim_B.z_ref = my_sim_P.Constant2_Value;

      // Constant: '<Root>/Constant3'
      my_sim_B.yaw_ref = my_sim_P.Constant3_Value;
    }

    if (rtmIsMajorTimeStep(my_sim_M) &&
        my_sim_M->Timing.TaskCounters.TID[2] == 0) {
      // MATLAB Function: '<S26>/optimizer' incorporates:
      //   Memory: '<S6>/Memory'
      //   Memory: '<S6>/last_x'
      //   SignalConversion generated from: '<S27>/ SFunction '
      //   UnitDelay: '<S6>/last_mv'

      for (my_sim_B.i = 0; my_sim_B.i < 20; my_sim_B.i++) {
        my_sim_B.rseq_tmp = my_sim_B.i << 2;
        my_sim_B.rseq[my_sim_B.rseq_tmp] = my_sim_B.x_ref;
        my_sim_B.rseq[my_sim_B.rseq_tmp + 1] = my_sim_B.y_ref;
        my_sim_B.rseq[my_sim_B.rseq_tmp + 2] = my_sim_B.z_ref;
        my_sim_B.rseq[my_sim_B.rseq_tmp + 3] = my_sim_B.yaw_ref;
      }

      my_sim_B.old_u[0] = my_sim_DW.last_mv_DSTATE[0];
      my_sim_B.TmpSignalConversionAtSFunct[0] = my_sim_DW.last_x_PreviousInput[0];
      my_sim_B.old_u[1] = my_sim_DW.last_mv_DSTATE[1];
      my_sim_B.TmpSignalConversionAtSFunct[1] = my_sim_DW.last_x_PreviousInput[1];
      my_sim_B.old_u[2] = my_sim_DW.last_mv_DSTATE[2];
      my_sim_B.TmpSignalConversionAtSFunct[2] = my_sim_DW.last_x_PreviousInput[2];
      my_sim_B.old_u[3] = my_sim_DW.last_mv_DSTATE[3];
      my_sim_B.TmpSignalConversionAtSFunct[3] = my_sim_DW.last_x_PreviousInput[3];
      my_sim_B.rtb_TmpSignalConversionAtSFun_b = my_sim_DW.last_x_PreviousInput
        [1];
      my_sim_B.rtb_TmpSignalConversionAtSFun_p = my_sim_DW.last_x_PreviousInput
        [0];
      my_sim_B.rtb_TmpSignalConversionAtSFun_c = my_sim_DW.last_x_PreviousInput
        [2];
      my_sim_B.rtb_TmpSignalConversionAtSFun_f = my_sim_DW.last_x_PreviousInput
        [3];
      for (my_sim_B.rseq_tmp = 0; my_sim_B.rseq_tmp < 4; my_sim_B.rseq_tmp++) {
        my_sim_B.y_innov[my_sim_B.rseq_tmp] =
          my_sim_B.measuredoutput[my_sim_B.rseq_tmp] - (((static_cast<real_T>
          (a[my_sim_B.rseq_tmp + 4]) * my_sim_B.rtb_TmpSignalConversionAtSFun_b
          + static_cast<real_T>(a[my_sim_B.rseq_tmp]) *
          my_sim_B.rtb_TmpSignalConversionAtSFun_p) + static_cast<real_T>
          (a[my_sim_B.rseq_tmp + 8]) * my_sim_B.rtb_TmpSignalConversionAtSFun_c)
          + static_cast<real_T>(a[my_sim_B.rseq_tmp + 12]) *
          my_sim_B.rtb_TmpSignalConversionAtSFun_f);
      }

      my_sim_B.rtb_TmpSignalConversionAtSFun_b = my_sim_B.y_innov[1];
      my_sim_B.rtb_TmpSignalConversionAtSFun_p = my_sim_B.y_innov[0];
      my_sim_B.rtb_TmpSignalConversionAtSFun_c = my_sim_B.y_innov[2];
      my_sim_B.rtb_TmpSignalConversionAtSFun_f = my_sim_B.y_innov[3];
      for (my_sim_B.rseq_tmp = 0; my_sim_B.rseq_tmp < 4; my_sim_B.rseq_tmp++) {
        my_sim_B.xest[my_sim_B.rseq_tmp] = (((b_a[my_sim_B.rseq_tmp + 4] *
          my_sim_B.rtb_TmpSignalConversionAtSFun_b + b_a[my_sim_B.rseq_tmp] *
          my_sim_B.rtb_TmpSignalConversionAtSFun_p) + b_a[my_sim_B.rseq_tmp + 8]
          * my_sim_B.rtb_TmpSignalConversionAtSFun_c) + b_a[my_sim_B.rseq_tmp +
          12] * my_sim_B.rtb_TmpSignalConversionAtSFun_f) +
          my_sim_B.TmpSignalConversionAtSFunct[my_sim_B.rseq_tmp];
      }

      memset(&my_sim_B.f[0], 0, 13U * sizeof(real_T));
      for (my_sim_B.i = 0; my_sim_B.i < 12; my_sim_B.i++) {
        my_sim_B.rtb_TmpSignalConversionAtSFun_b = 0.0;
        for (my_sim_B.rseq_tmp = 0; my_sim_B.rseq_tmp < 80; my_sim_B.rseq_tmp++)
        {
          my_sim_B.rtb_TmpSignalConversionAtSFun_b += b_Kr[80 * my_sim_B.i +
            my_sim_B.rseq_tmp] * my_sim_B.rseq[my_sim_B.rseq_tmp];
        }

        my_sim_B.rseq_tmp = my_sim_B.i << 2;
        my_sim_B.f[my_sim_B.i] = ((((b_Kx[my_sim_B.rseq_tmp + 1] *
          my_sim_B.xest[1] + b_Kx[my_sim_B.rseq_tmp] * my_sim_B.xest[0]) +
          b_Kx[my_sim_B.rseq_tmp + 2] * my_sim_B.xest[2]) +
          b_Kx[my_sim_B.rseq_tmp + 3] * my_sim_B.xest[3]) +
          my_sim_B.rtb_TmpSignalConversionAtSFun_b) + (((b_Ku1[my_sim_B.rseq_tmp
          + 1] * my_sim_B.old_u[1] + b_Ku1[my_sim_B.rseq_tmp] * my_sim_B.old_u[0])
          + b_Ku1[my_sim_B.rseq_tmp + 2] * my_sim_B.old_u[2]) +
          b_Ku1[my_sim_B.rseq_tmp + 3] * my_sim_B.old_u[3]);
      }

      my_sim_B.rtb_TmpSignalConversionAtSFun_b = my_sim_B.xest[0];
      my_sim_B.rtb_TmpSignalConversionAtSFun_p = my_sim_B.xest[1];
      my_sim_B.rtb_TmpSignalConversionAtSFun_c = my_sim_B.xest[2];
      my_sim_B.rtb_TmpSignalConversionAtSFun_f = my_sim_B.xest[3];
      my_sim_B.old_u_m = my_sim_B.old_u[1];
      my_sim_B.old_u_c = my_sim_B.old_u[0];
      my_sim_B.old_u_k = my_sim_B.old_u[2];
      my_sim_B.old_u_cx = my_sim_B.old_u[3];
      for (my_sim_B.i = 0; my_sim_B.i < 48; my_sim_B.i++) {
        my_sim_B.iAout[my_sim_B.i] = my_sim_DW.Memory_PreviousInput[my_sim_B.i];
        my_sim_B.dv[my_sim_B.i] = -(((((0.0 *
          my_sim_B.rtb_TmpSignalConversionAtSFun_b + 0.0 *
          my_sim_B.rtb_TmpSignalConversionAtSFun_p) + 0.0 *
          my_sim_B.rtb_TmpSignalConversionAtSFun_c) + 0.0 *
          my_sim_B.rtb_TmpSignalConversionAtSFun_f) + 1.0) +
          (((static_cast<real_T>(c_a[my_sim_B.i + 48]) * my_sim_B.old_u_m +
             static_cast<real_T>(c_a[my_sim_B.i]) * my_sim_B.old_u_c) +
            static_cast<real_T>(c_a[my_sim_B.i + 96]) * my_sim_B.old_u_k) +
           static_cast<real_T>(c_a[my_sim_B.i + 144]) * my_sim_B.old_u_cx));
      }

      my_sim_qpkwik(b_Linv, b_Hinv, my_sim_B.f, b_Ac, my_sim_B.dv,
                    my_sim_B.iAout, 244, 1.0E-6, my_sim_B.zopt, my_sim_B.a__1,
                    &my_sim_B.i);
      if ((my_sim_B.i < 0) || (my_sim_B.i == 0)) {
        memset(&my_sim_B.zopt[0], 0, 13U * sizeof(real_T));
      }

      my_sim_B.rtb_TmpSignalConversionAtSFun_b = my_sim_DW.last_x_PreviousInput
        [1];
      my_sim_B.rtb_TmpSignalConversionAtSFun_p = my_sim_DW.last_x_PreviousInput
        [0];
      my_sim_B.rtb_TmpSignalConversionAtSFun_c = my_sim_DW.last_x_PreviousInput
        [2];
      my_sim_B.rtb_TmpSignalConversionAtSFun_f = my_sim_DW.last_x_PreviousInput
        [3];
      for (my_sim_B.i = 0; my_sim_B.i < 4; my_sim_B.i++) {
        my_sim_B.old_u_m = my_sim_B.old_u[my_sim_B.i] + my_sim_B.zopt[my_sim_B.i];
        my_sim_B.u[my_sim_B.i] = my_sim_B.old_u_m;
        my_sim_B.xest[my_sim_B.i] = my_sim_B.old_u_m;
        my_sim_B.TmpSignalConversionAtSFunct[my_sim_B.i] = ((static_cast<real_T>
          (a[my_sim_B.i + 4]) * my_sim_B.rtb_TmpSignalConversionAtSFun_b +
          static_cast<real_T>(a[my_sim_B.i]) *
          my_sim_B.rtb_TmpSignalConversionAtSFun_p) + static_cast<real_T>
          (a[my_sim_B.i + 8]) * my_sim_B.rtb_TmpSignalConversionAtSFun_c) +
          static_cast<real_T>(a[my_sim_B.i + 12]) *
          my_sim_B.rtb_TmpSignalConversionAtSFun_f;
      }

      my_sim_B.old_u_m = my_sim_B.xest[1];
      my_sim_B.old_u_c = my_sim_B.xest[0];
      my_sim_B.old_u_k = my_sim_B.xest[2];
      my_sim_B.old_u_cx = my_sim_B.xest[3];
      my_sim_B.rtb_TmpSignalConversionAtSFun_b = my_sim_B.y_innov[1];
      my_sim_B.rtb_TmpSignalConversionAtSFun_p = my_sim_B.y_innov[0];
      my_sim_B.rtb_TmpSignalConversionAtSFun_c = my_sim_B.y_innov[2];
      my_sim_B.rtb_TmpSignalConversionAtSFun_f = my_sim_B.y_innov[3];
      for (my_sim_B.i = 0; my_sim_B.i < 4; my_sim_B.i++) {
        my_sim_B.xk1[my_sim_B.i] = ((((e_a[my_sim_B.i + 4] * my_sim_B.old_u_m +
          e_a[my_sim_B.i] * my_sim_B.old_u_c) + e_a[my_sim_B.i + 8] *
          my_sim_B.old_u_k) + e_a[my_sim_B.i + 12] * my_sim_B.old_u_cx) +
          my_sim_B.TmpSignalConversionAtSFunct[my_sim_B.i]) + (((f_a[my_sim_B.i
          + 4] * my_sim_B.rtb_TmpSignalConversionAtSFun_b + f_a[my_sim_B.i] *
          my_sim_B.rtb_TmpSignalConversionAtSFun_p) + f_a[my_sim_B.i + 8] *
          my_sim_B.rtb_TmpSignalConversionAtSFun_c) + f_a[my_sim_B.i + 12] *
          my_sim_B.rtb_TmpSignalConversionAtSFun_f);

        // Gain: '<S6>/umin_scale1'
        my_sim_B.umin_scale1[my_sim_B.i] = my_sim_P.umin_scale1_Gain[my_sim_B.i]
          * my_sim_B.u[my_sim_B.i];
      }

      // End of MATLAB Function: '<S26>/optimizer'
    }

    // DataTypeConversion: '<Root>/Data Type Conversion'
    my_sim_B.yaw_pos = static_cast<real32_T>(my_sim_B.measuredoutput[3]);
    if (rtmIsMajorTimeStep(my_sim_M) &&
        my_sim_M->Timing.TaskCounters.TID[1] == 0) {
    }

    if (rtmIsMajorTimeStep(my_sim_M) &&
        my_sim_M->Timing.TaskCounters.TID[2] == 0) {
      // BusAssignment: '<Root>/Bus Assignment1' incorporates:
      //   Constant: '<Root>/Constant'
      //   Constant: '<Root>/Constant4'
      //   Constant: '<S2>/Constant'
      //   DataTypeConversion: '<Root>/Data Type Conversion1'

      my_sim_B.BusAssignment1 = my_sim_P.Constant_Value;
      my_sim_B.BusAssignment1.CoordinateFrame = my_sim_P.Constant_Value_d;
      my_sim_B.BusAssignment1.TypeMask = my_sim_P.Constant4_Value;
      my_sim_B.BusAssignment1.Velocity.X = my_sim_B.umin_scale1[0];
      my_sim_B.BusAssignment1.Velocity.Y = my_sim_B.umin_scale1[1];
      my_sim_B.BusAssignment1.Velocity.Z = my_sim_B.umin_scale1[2];
      my_sim_B.BusAssignment1.YawRate = static_cast<real32_T>
        (my_sim_B.umin_scale1[3]);

      // Outputs for Atomic SubSystem: '<Root>/Publish1'
      // MATLABSystem: '<S5>/SinkBlock'
      Pub_my_sim_41.publish(&my_sim_B.BusAssignment1);

      // End of Outputs for SubSystem: '<Root>/Publish1'
    }

    // BusAssignment: '<Root>/Bus Assignment' incorporates:
    //   Constant: '<S1>/Constant'

    my_sim_B.BusAssignment = my_sim_P.Constant_Value_h;
    my_sim_B.BusAssignment.Pose.Position.X = my_sim_B.measuredoutput[0];
    my_sim_B.BusAssignment.Pose.Position.Y = my_sim_B.measuredoutput[1];
    my_sim_B.BusAssignment.Pose.Position.Z = my_sim_B.measuredoutput[2];

    // Outputs for Atomic SubSystem: '<Root>/Publish'
    // MATLABSystem: '<S4>/SinkBlock'
    Pub_my_sim_1.publish(&my_sim_B.BusAssignment);

    // End of Outputs for SubSystem: '<Root>/Publish'
    if (rtmIsMajorTimeStep(my_sim_M) &&
        my_sim_M->Timing.TaskCounters.TID[1] == 0) {
      // S-Function (saeroclockpacer): '<Root>/Simulation Pace'
      //
      //  The Clock Pacer generates no code, it is only active in
      //  interpreted simulation.

    }
  }

  if (rtmIsMajorTimeStep(my_sim_M)) {
    int32_T i;
    if (rtmIsMajorTimeStep(my_sim_M) &&
        my_sim_M->Timing.TaskCounters.TID[2] == 0) {
      // Update for Memory: '<S6>/last_x'
      my_sim_DW.last_x_PreviousInput[0] = my_sim_B.xk1[0];

      // Update for UnitDelay: '<S6>/last_mv'
      my_sim_DW.last_mv_DSTATE[0] = my_sim_B.u[0];

      // Update for Memory: '<S6>/last_x'
      my_sim_DW.last_x_PreviousInput[1] = my_sim_B.xk1[1];

      // Update for UnitDelay: '<S6>/last_mv'
      my_sim_DW.last_mv_DSTATE[1] = my_sim_B.u[1];

      // Update for Memory: '<S6>/last_x'
      my_sim_DW.last_x_PreviousInput[2] = my_sim_B.xk1[2];

      // Update for UnitDelay: '<S6>/last_mv'
      my_sim_DW.last_mv_DSTATE[2] = my_sim_B.u[2];

      // Update for Memory: '<S6>/last_x'
      my_sim_DW.last_x_PreviousInput[3] = my_sim_B.xk1[3];

      // Update for UnitDelay: '<S6>/last_mv'
      my_sim_DW.last_mv_DSTATE[3] = my_sim_B.u[3];

      // Update for Memory: '<S6>/Memory'
      for (i = 0; i < 48; i++) {
        my_sim_DW.Memory_PreviousInput[i] = my_sim_B.iAout[i];
      }

      // End of Update for Memory: '<S6>/Memory'
    }

    // External mode
    rtExtModeUploadCheckTrigger(3);

    {                                  // Sample time: [0.0s, 0.0s]
      rtExtModeUpload(0, (real_T)my_sim_M->Timing.t[0]);
    }

    if (rtmIsMajorTimeStep(my_sim_M) &&
        my_sim_M->Timing.TaskCounters.TID[1] == 0) {
                                  // Sample time: [0.033333333333333333s, 0.0s]
      rtExtModeUpload(1, (real_T)((my_sim_M->Timing.clockTick1) *
        0.033333333333333333));
    }

    if (rtmIsMajorTimeStep(my_sim_M) &&
        my_sim_M->Timing.TaskCounters.TID[2] == 0) {// Sample time: [0.1s, 0.0s] 
      rtExtModeUpload(2, (real_T)((my_sim_M->Timing.clockTick2) * 0.1));
    }
  }                                    // end MajorTimeStep

  if (rtmIsMajorTimeStep(my_sim_M)) {
    // signal main to stop simulation
    {                                  // Sample time: [0.0s, 0.0s]
      if ((rtmGetTFinal(my_sim_M)!=-1) &&
          !((rtmGetTFinal(my_sim_M)-((my_sim_M->Timing.clockTick1) *
             0.033333333333333333)) > ((my_sim_M->Timing.clockTick1) *
            0.033333333333333333) * (DBL_EPSILON))) {
        rtmSetErrorStatus(my_sim_M, "Simulation finished");
      }

      if (rtmGetStopRequested(my_sim_M)) {
        rtmSetErrorStatus(my_sim_M, "Simulation finished");
      }
    }

    rt_ertODEUpdateContinuousStates(&my_sim_M->solverInfo);

    // Update absolute time for base rate
    // The "clockTick0" counts the number of times the code of this task has
    //  been executed. The absolute time is the multiplication of "clockTick0"
    //  and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
    //  overflow during the application lifespan selected.

    ++my_sim_M->Timing.clockTick0;
    my_sim_M->Timing.t[0] = rtsiGetSolverStopTime(&my_sim_M->solverInfo);

    {
      // Update absolute timer for sample time: [0.033333333333333333s, 0.0s]
      // The "clockTick1" counts the number of times the code of this task has
      //  been executed. The resolution of this integer timer is 0.033333333333333333, which is the step size
      //  of the task. Size of "clockTick1" ensures timer will not overflow during the
      //  application lifespan selected.

      my_sim_M->Timing.clockTick1++;
    }

    if (rtmIsMajorTimeStep(my_sim_M) &&
        my_sim_M->Timing.TaskCounters.TID[2] == 0) {
      // Update absolute timer for sample time: [0.1s, 0.0s]
      // The "clockTick2" counts the number of times the code of this task has
      //  been executed. The resolution of this integer timer is 0.1, which is the step size
      //  of the task. Size of "clockTick2" ensures timer will not overflow during the
      //  application lifespan selected.

      my_sim_M->Timing.clockTick2++;
    }

    rate_scheduler();
  }                                    // end MajorTimeStep
}

// Derivatives for root system: '<Root>'
void my_sim_derivatives(void)
{
  XDot_my_sim_T *_rtXdot;
  uint32_T ri;
  _rtXdot = ((XDot_my_sim_T *) my_sim_M->derivs);

  // Derivatives for StateSpace: '<Root>/State-Space'
  _rtXdot->position[0] = 0.0;
  _rtXdot->position[1] = 0.0;
  _rtXdot->position[2] = 0.0;
  _rtXdot->position[3] = 0.0;
  for (ri = my_sim_P.StateSpace_B_jc[0U]; ri < my_sim_P.StateSpace_B_jc[1U]; ri
       ++) {
    _rtXdot->position[my_sim_P.StateSpace_B_ir[ri]] +=
      my_sim_P.StateSpace_B_pr[ri] * my_sim_B.umin_scale1[0U];
  }

  for (ri = my_sim_P.StateSpace_B_jc[1U]; ri < my_sim_P.StateSpace_B_jc[2U]; ri
       ++) {
    _rtXdot->position[my_sim_P.StateSpace_B_ir[ri]] +=
      my_sim_P.StateSpace_B_pr[ri] * my_sim_B.umin_scale1[1U];
  }

  for (ri = my_sim_P.StateSpace_B_jc[2U]; ri < my_sim_P.StateSpace_B_jc[3U]; ri
       ++) {
    _rtXdot->position[my_sim_P.StateSpace_B_ir[ri]] +=
      my_sim_P.StateSpace_B_pr[ri] * my_sim_B.umin_scale1[2U];
  }

  for (ri = my_sim_P.StateSpace_B_jc[3U]; ri < my_sim_P.StateSpace_B_jc[4U]; ri
       ++) {
    _rtXdot->position[my_sim_P.StateSpace_B_ir[ri]] +=
      my_sim_P.StateSpace_B_pr[ri] * my_sim_B.umin_scale1[3U];
  }

  // End of Derivatives for StateSpace: '<Root>/State-Space'
}

// Model initialize function
void my_sim_initialize(void)
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  {
    // Setup solver object
    rtsiSetSimTimeStepPtr(&my_sim_M->solverInfo, &my_sim_M->Timing.simTimeStep);
    rtsiSetTPtr(&my_sim_M->solverInfo, &rtmGetTPtr(my_sim_M));
    rtsiSetStepSizePtr(&my_sim_M->solverInfo, &my_sim_M->Timing.stepSize0);
    rtsiSetdXPtr(&my_sim_M->solverInfo, &my_sim_M->derivs);
    rtsiSetContStatesPtr(&my_sim_M->solverInfo, (real_T **)
                         &my_sim_M->contStates);
    rtsiSetNumContStatesPtr(&my_sim_M->solverInfo,
      &my_sim_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&my_sim_M->solverInfo,
      &my_sim_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&my_sim_M->solverInfo,
      &my_sim_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&my_sim_M->solverInfo,
      &my_sim_M->periodicContStateRanges);
    rtsiSetContStateDisabledPtr(&my_sim_M->solverInfo, (boolean_T**)
      &my_sim_M->contStateDisabled);
    rtsiSetErrorStatusPtr(&my_sim_M->solverInfo, (&rtmGetErrorStatus(my_sim_M)));
    rtsiSetRTModelPtr(&my_sim_M->solverInfo, my_sim_M);
  }

  rtsiSetSimTimeStep(&my_sim_M->solverInfo, MAJOR_TIME_STEP);
  my_sim_M->intgData.y = my_sim_M->odeY;
  my_sim_M->intgData.f[0] = my_sim_M->odeF[0];
  my_sim_M->intgData.f[1] = my_sim_M->odeF[1];
  my_sim_M->intgData.f[2] = my_sim_M->odeF[2];
  my_sim_M->contStates = ((X_my_sim_T *) &my_sim_X);
  my_sim_M->contStateDisabled = ((XDis_my_sim_T *) &my_sim_XDis);
  my_sim_M->Timing.tStart = (0.0);
  rtsiSetSolverData(&my_sim_M->solverInfo, static_cast<void *>
                    (&my_sim_M->intgData));
  rtsiSetIsMinorTimeStepWithModeChange(&my_sim_M->solverInfo, false);
  rtsiSetSolverName(&my_sim_M->solverInfo,"ode3");
  rtmSetTPtr(my_sim_M, &my_sim_M->Timing.tArray[0]);
  rtmSetTFinal(my_sim_M, 10.0);
  my_sim_M->Timing.stepSize0 = 0.033333333333333333;

  // External mode info
  my_sim_M->Sizes.checksums[0] = (738916925U);
  my_sim_M->Sizes.checksums[1] = (3758009617U);
  my_sim_M->Sizes.checksums[2] = (1428962209U);
  my_sim_M->Sizes.checksums[3] = (4079769678U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[8];
    my_sim_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    systemRan[1] = &rtAlwaysEnabled;
    systemRan[2] = &rtAlwaysEnabled;
    systemRan[3] = &rtAlwaysEnabled;
    systemRan[4] = &rtAlwaysEnabled;
    systemRan[5] = &rtAlwaysEnabled;
    systemRan[6] = &rtAlwaysEnabled;
    systemRan[7] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(my_sim_M->extModeInfo,
      &my_sim_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(my_sim_M->extModeInfo, my_sim_M->Sizes.checksums);
    rteiSetTPtr(my_sim_M->extModeInfo, rtmGetTPtr(my_sim_M));
  }

  // data type transition information
  {
    static DataTypeTransInfo dtInfo;
    my_sim_M->SpecialInfo.mappingInfo = (&dtInfo);
    dtInfo.numDataTypes = 32;
    dtInfo.dataTypeSizes = &rtDataTypeSizes[0];
    dtInfo.dataTypeNames = &rtDataTypeNames[0];

    // Block I/O transition table
    dtInfo.BTransTable = &rtBTransTable;

    // Parameters transition table
    dtInfo.PTransTable = &rtPTransTable;
  }

  {
    int32_T i;
    char_T b_zeroDelimTopic_0[28];
    char_T b_zeroDelimTopic[27];
    static const char_T b_zeroDelimTopic_1[27] = "/mavros/setpoint_raw/local";
    static const char_T b_zeroDelimTopic_2[28] = "/mavros/local_position/pose";

    // InitializeConditions for StateSpace: '<Root>/State-Space'
    my_sim_X.position[0] = my_sim_P.StateSpace_InitialCondition[0];

    // InitializeConditions for Memory: '<S6>/last_x'
    my_sim_DW.last_x_PreviousInput[0] = my_sim_P.last_x_InitialCondition[0];

    // InitializeConditions for UnitDelay: '<S6>/last_mv'
    my_sim_DW.last_mv_DSTATE[0] = my_sim_P.last_mv_InitialCondition[0];

    // InitializeConditions for StateSpace: '<Root>/State-Space'
    my_sim_X.position[1] = my_sim_P.StateSpace_InitialCondition[1];

    // InitializeConditions for Memory: '<S6>/last_x'
    my_sim_DW.last_x_PreviousInput[1] = my_sim_P.last_x_InitialCondition[1];

    // InitializeConditions for UnitDelay: '<S6>/last_mv'
    my_sim_DW.last_mv_DSTATE[1] = my_sim_P.last_mv_InitialCondition[1];

    // InitializeConditions for StateSpace: '<Root>/State-Space'
    my_sim_X.position[2] = my_sim_P.StateSpace_InitialCondition[2];

    // InitializeConditions for Memory: '<S6>/last_x'
    my_sim_DW.last_x_PreviousInput[2] = my_sim_P.last_x_InitialCondition[2];

    // InitializeConditions for UnitDelay: '<S6>/last_mv'
    my_sim_DW.last_mv_DSTATE[2] = my_sim_P.last_mv_InitialCondition[2];

    // InitializeConditions for StateSpace: '<Root>/State-Space'
    my_sim_X.position[3] = my_sim_P.StateSpace_InitialCondition[3];

    // InitializeConditions for Memory: '<S6>/last_x'
    my_sim_DW.last_x_PreviousInput[3] = my_sim_P.last_x_InitialCondition[3];

    // InitializeConditions for UnitDelay: '<S6>/last_mv'
    my_sim_DW.last_mv_DSTATE[3] = my_sim_P.last_mv_InitialCondition[3];

    // InitializeConditions for Memory: '<S6>/Memory'
    for (i = 0; i < 48; i++) {
      my_sim_DW.Memory_PreviousInput[i] = my_sim_P.Memory_InitialCondition[i];
    }

    // End of InitializeConditions for Memory: '<S6>/Memory'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish1'
    // Start for MATLABSystem: '<S5>/SinkBlock'
    my_sim_DW.obj.matlabCodegenIsDeleted = false;
    my_sim_DW.obj.isInitialized = 1;
    for (i = 0; i < 27; i++) {
      b_zeroDelimTopic[i] = b_zeroDelimTopic_1[i];
    }

    Pub_my_sim_41.createPublisher(&b_zeroDelimTopic[0], 1);
    my_sim_DW.obj.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S5>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish1'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish'
    // Start for MATLABSystem: '<S4>/SinkBlock'
    my_sim_DW.obj_e.matlabCodegenIsDeleted = false;
    my_sim_DW.obj_e.isInitialized = 1;
    for (i = 0; i < 28; i++) {
      b_zeroDelimTopic_0[i] = b_zeroDelimTopic_2[i];
    }

    Pub_my_sim_1.createPublisher(&b_zeroDelimTopic_0[0], 1);
    my_sim_DW.obj_e.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S4>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish'
  }
}

// Model terminate function
void my_sim_terminate(void)
{
  // Terminate for Atomic SubSystem: '<Root>/Publish1'
  // Terminate for MATLABSystem: '<S5>/SinkBlock'
  if (!my_sim_DW.obj.matlabCodegenIsDeleted) {
    my_sim_DW.obj.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S5>/SinkBlock'
  // End of Terminate for SubSystem: '<Root>/Publish1'

  // Terminate for Atomic SubSystem: '<Root>/Publish'
  // Terminate for MATLABSystem: '<S4>/SinkBlock'
  if (!my_sim_DW.obj_e.matlabCodegenIsDeleted) {
    my_sim_DW.obj_e.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S4>/SinkBlock'
  // End of Terminate for SubSystem: '<Root>/Publish'
}

//
// File trailer for generated code.
//
// [EOF]
//
