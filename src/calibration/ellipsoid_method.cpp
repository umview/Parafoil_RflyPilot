#include <math.h>
#include <string.h>
#include "ellipsoid_method.h"

static void b_sqrt(double *x);
static void b_xaxpy(int n, double a, const double x[49], int ix0, double y[7],
                    int iy0);
static double b_xnrm2(int n, const double x[7], int ix0);
static void c_xaxpy(int n, double a, const double x[7], int ix0, double y[49],
                    int iy0);
static void svd(const double A[49], double U[49], double s[7], double V[49]);
static void xaxpy(int n, double a, int ix0, double y[49], int iy0);
static double xdotc(int n, const double x[49], int ix0, const double y[49], int
                    iy0);
static double xnrm2(int n, const double x[49], int ix0);
static void xrot(double x[49], int ix0, int iy0, double c, double s);
static void xrotg(double *a, double *b, double *c, double *s);
static void xscal(double a, double x[49], int ix0);
static void xswap(double x[49], int ix0, int iy0);
static void b_sqrt(double *x)
{
  *x = sqrt(*x);
}

static void b_xaxpy(int n, double a, const double x[49], int ix0, double y[7],
                    int iy0)
{
  int ix;
  int iy;
  int k;
  if ((n < 1) || (a == 0.0)) {
  } else {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * x[ix];
      ix++;
      iy++;
    }
  }
}

static double b_xnrm2(int n, const double x[7], int ix0)
{
  double y;
  double scale;
  int kend;
  int k;
  double absxk;
  double t;
  y = 0.0;
  if (!(n < 1)) {
    if (n == 1) {
      y = fabs(x[ix0 - 1]);
    } else {
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (k = ix0; k <= kend; k++) {
        absxk = fabs(x[k - 1]);
        if (absxk > scale) {
          t = scale / absxk;
          y = 1.0 + y * t * t;
          scale = absxk;
        } else {
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * sqrt(y);
    }
  }

  return y;
}

static void c_xaxpy(int n, double a, const double x[7], int ix0, double y[49],
                    int iy0)
{
  int ix;
  int iy;
  int k;
  if ((n < 1) || (a == 0.0)) {
  } else {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * x[ix];
      ix++;
      iy++;
    }
  }
}

static void svd(const double A[49], double U[49], double s[7], double V[49])
{
  double b_A[49];
  int i;
  double b_s[7];
  double e[7];
  int q;
  double work[7];
  int m;
  int qq;
  boolean_T apply_transform;
  double nrm;
  int qp1jj;
  int qs;
  double snorm;
  double rt;
  double r;
  int exitg1;
  boolean_T exitg2;
  double f;
  double scale;
  double sqds;
  memcpy(&b_A[0], &A[0], 49U * sizeof(double));
  for (i = 0; i < 7; i++) {
    b_s[i] = 0.0;
    e[i] = 0.0;
    work[i] = 0.0;
  }

  memset(&U[0], 0, 49U * sizeof(double));
  memset(&V[0], 0, 49U * sizeof(double));
  for (q = 0; q < 6; q++) {
    qq = q + 7 * q;
    apply_transform = false;
    nrm = xnrm2(7 - q, b_A, qq + 1);
    if (nrm > 0.0) {
      apply_transform = true;
      if (b_A[qq] < 0.0) {
        b_s[q] = -nrm;
      } else {
        b_s[q] = nrm;
      }

      if (fabs(b_s[q]) >= 1.0020841800044864E-292) {
        nrm = 1.0 / b_s[q];
        i = (qq - q) + 7;
        for (qp1jj = qq; qp1jj < i; qp1jj++) {
          b_A[qp1jj] *= nrm;
        }
      } else {
        i = (qq - q) + 7;
        for (qp1jj = qq; qp1jj < i; qp1jj++) {
          b_A[qp1jj] /= b_s[q];
        }
      }

      b_A[qq]++;
      b_s[q] = -b_s[q];
    } else {
      b_s[q] = 0.0;
    }

    for (qs = q + 1; qs + 1 < 8; qs++) {
      i = q + 7 * qs;
      if (apply_transform) {
        xaxpy(7 - q, -(xdotc(7 - q, b_A, qq + 1, b_A, i + 1) / b_A[q + 7 * q]),
              qq + 1, b_A, i + 1);
      }

      e[qs] = b_A[i];
    }

    for (qp1jj = q; qp1jj + 1 < 8; qp1jj++) {
      U[qp1jj + 7 * q] = b_A[qp1jj + 7 * q];
    }

    if (q + 1 <= 5) {
      nrm = b_xnrm2(6 - q, e, q + 2);
      if (nrm == 0.0) {
        e[q] = 0.0;
      } else {
        if (e[q + 1] < 0.0) {
          e[q] = -nrm;
        } else {
          e[q] = nrm;
        }

        nrm = e[q];
        if (fabs(e[q]) >= 1.0020841800044864E-292) {
          nrm = 1.0 / e[q];
          for (qp1jj = q + 1; qp1jj < 7; qp1jj++) {
            e[qp1jj] *= nrm;
          }
        } else {
          for (qp1jj = q + 1; qp1jj < 7; qp1jj++) {
            e[qp1jj] /= nrm;
          }
        }

        e[q + 1]++;
        e[q] = -e[q];
        for (qp1jj = q + 1; qp1jj + 1 < 8; qp1jj++) {
          work[qp1jj] = 0.0;
        }

        for (qs = q + 1; qs + 1 < 8; qs++) {
          b_xaxpy(6 - q, e[qs], b_A, (q + 7 * qs) + 2, work, q + 2);
        }

        for (qs = q + 1; qs + 1 < 8; qs++) {
          c_xaxpy(6 - q, -e[qs] / e[q + 1], work, q + 2, b_A, (q + 7 * qs) + 2);
        }
      }

      for (qp1jj = q + 1; qp1jj + 1 < 8; qp1jj++) {
        V[qp1jj + 7 * q] = e[qp1jj];
      }
    }
  }

  m = 5;
  b_s[6] = b_A[48];
  e[5] = b_A[47];
  e[6] = 0.0;
  for (qp1jj = 0; qp1jj < 7; qp1jj++) {
    U[42 + qp1jj] = 0.0;
  }

  U[48] = 1.0;
  for (q = 5; q >= 0; q--) {
    qq = q + 7 * q;
    if (b_s[q] != 0.0) {
      for (qs = q + 1; qs + 1 < 8; qs++) {
        i = (q + 7 * qs) + 1;
        xaxpy(7 - q, -(xdotc(7 - q, U, qq + 1, U, i) / U[qq]), qq + 1, U, i);
      }

      for (qp1jj = q; qp1jj + 1 < 8; qp1jj++) {
        U[qp1jj + 7 * q] = -U[qp1jj + 7 * q];
      }

      U[qq]++;
      for (qp1jj = 1; qp1jj <= q; qp1jj++) {
        U[(qp1jj + 7 * q) - 1] = 0.0;
      }
    } else {
      for (qp1jj = 0; qp1jj < 7; qp1jj++) {
        U[qp1jj + 7 * q] = 0.0;
      }

      U[qq] = 1.0;
    }
  }

  for (q = 6; q >= 0; q--) {
    if ((q + 1 <= 5) && (e[q] != 0.0)) {
      i = (q + 7 * q) + 2;
      for (qs = q + 1; qs + 1 < 8; qs++) {
        qp1jj = (q + 7 * qs) + 2;
        xaxpy(6 - q, -(xdotc(6 - q, V, i, V, qp1jj) / V[i - 1]), i, V, qp1jj);
      }
    }

    for (qp1jj = 0; qp1jj < 7; qp1jj++) {
      V[qp1jj + 7 * q] = 0.0;
    }

    V[q + 7 * q] = 1.0;
  }

  for (q = 0; q < 7; q++) {
    nrm = e[q];
    if (b_s[q] != 0.0) {
      rt = fabs(b_s[q]);
      r = b_s[q] / rt;
      b_s[q] = rt;
      if (q + 1 < 7) {
        nrm = e[q] / r;
      }

      xscal(r, U, 1 + 7 * q);
    }

    if ((q + 1 < 7) && (nrm != 0.0)) {
      rt = fabs(nrm);
      r = rt / nrm;
      nrm = rt;
      b_s[q + 1] *= r;
      xscal(r, V, 1 + 7 * (q + 1));
    }

    e[q] = nrm;
  }

  qq = 0;
  snorm = 0.0;
  for (qp1jj = 0; qp1jj < 7; qp1jj++) {
    nrm = fabs(b_s[qp1jj]);
    r = fabs(e[qp1jj]);
    if (nrm > r) {
      r = nrm;
    }

    if (!(snorm > r)) {
      snorm = r;
    }
  }

  while ((m + 2 > 0) && (!(qq >= 75))) {
    qp1jj = m;
    do {
      exitg1 = 0;
      q = qp1jj + 1;
      if (qp1jj + 1 == 0) {
        exitg1 = 1;
      } else {
        nrm = fabs(e[qp1jj]);
        if ((nrm <= 2.2204460492503131E-16 * (fabs(b_s[qp1jj]) + fabs(b_s[qp1jj
               + 1]))) || (nrm <= 1.0020841800044864E-292) || ((qq > 20) && (nrm
              <= 2.2204460492503131E-16 * snorm))) {
          e[qp1jj] = 0.0;
          exitg1 = 1;
        } else {
          qp1jj--;
        }
      }
    } while (exitg1 == 0);

    if (qp1jj + 1 == m + 1) {
      i = 4;
    } else {
      qs = m + 2;
      i = m + 2;
      exitg2 = false;
      while ((!exitg2) && (i >= qp1jj + 1)) {
        qs = i;
        if (i == qp1jj + 1) {
          exitg2 = true;
        } else {
          nrm = 0.0;
          if (i < m + 2) {
            nrm = fabs(e[i - 1]);
          }

          if (i > qp1jj + 2) {
            nrm += fabs(e[i - 2]);
          }

          r = fabs(b_s[i - 1]);
          if ((r <= 2.2204460492503131E-16 * nrm) || (r <=
               1.0020841800044864E-292)) {
            b_s[i - 1] = 0.0;
            exitg2 = true;
          } else {
            i--;
          }
        }
      }

      if (qs == qp1jj + 1) {
        i = 3;
      } else if (qs == m + 2) {
        i = 1;
      } else {
        i = 2;
        q = qs;
      }
    }

    switch (i) {
     case 1:
      f = e[m];
      e[m] = 0.0;
      for (qp1jj = m; qp1jj + 1 >= q + 1; qp1jj--) {
        xrotg(&b_s[qp1jj], &f, &nrm, &r);
        if (qp1jj + 1 > q + 1) {
          f = -r * e[qp1jj - 1];
          e[qp1jj - 1] *= nrm;
        }

        xrot(V, 1 + 7 * qp1jj, 1 + 7 * (m + 1), nrm, r);
      }
      break;

     case 2:
      f = e[q - 1];
      e[q - 1] = 0.0;
      for (qp1jj = q; qp1jj < m + 2; qp1jj++) {
        xrotg(&b_s[qp1jj], &f, &nrm, &r);
        f = -r * e[qp1jj];
        e[qp1jj] *= nrm;
        xrot(U, 1 + 7 * qp1jj, 1 + 7 * (q - 1), nrm, r);
      }
      break;

     case 3:
      scale = fabs(b_s[m + 1]);
      r = fabs(b_s[m]);
      if (!(scale > r)) {
        scale = r;
      }

      r = fabs(e[m]);
      if (!(scale > r)) {
        scale = r;
      }

      r = fabs(b_s[q]);
      if (!(scale > r)) {
        scale = r;
      }

      r = fabs(e[q]);
      if (!(scale > r)) {
        scale = r;
      }

      f = b_s[m + 1] / scale;
      nrm = b_s[m] / scale;
      r = e[m] / scale;
      sqds = b_s[q] / scale;
      rt = ((nrm + f) * (nrm - f) + r * r) / 2.0;
      nrm = f * r;
      nrm *= nrm;
      if ((rt != 0.0) || (nrm != 0.0)) {
        r = rt * rt + nrm;
        b_sqrt(&r);
        if (rt < 0.0) {
          r = -r;
        }

        r = nrm / (rt + r);
      } else {
        r = 0.0;
      }

      f = (sqds + f) * (sqds - f) + r;
      rt = sqds * (e[q] / scale);
      for (qp1jj = q + 1; qp1jj <= m + 1; qp1jj++) {
        xrotg(&f, &rt, &nrm, &r);
        if (qp1jj > q + 1) {
          e[qp1jj - 2] = f;
        }

        f = nrm * b_s[qp1jj - 1] + r * e[qp1jj - 1];
        e[qp1jj - 1] = nrm * e[qp1jj - 1] - r * b_s[qp1jj - 1];
        rt = r * b_s[qp1jj];
        b_s[qp1jj] *= nrm;
        xrot(V, 1 + 7 * (qp1jj - 1), 1 + 7 * qp1jj, nrm, r);
        b_s[qp1jj - 1] = f;
        xrotg(&b_s[qp1jj - 1], &rt, &nrm, &r);
        f = nrm * e[qp1jj - 1] + r * b_s[qp1jj];
        b_s[qp1jj] = -r * e[qp1jj - 1] + nrm * b_s[qp1jj];
        rt = r * e[qp1jj];
        e[qp1jj] *= nrm;
        xrot(U, 1 + 7 * (qp1jj - 1), 1 + 7 * qp1jj, nrm, r);
      }

      e[m] = f;
      qq++;
      break;

     default:
      if (b_s[q] < 0.0) {
        b_s[q] = -b_s[q];
        xscal(-1.0, V, 1 + 7 * q);
      }

      i = q + 1;
      while ((q + 1 < 7) && (b_s[q] < b_s[i])) {
        rt = b_s[q];
        b_s[q] = b_s[i];
        b_s[i] = rt;
        xswap(V, 1 + 7 * q, 1 + 7 * (q + 1));
        xswap(U, 1 + 7 * q, 1 + 7 * (q + 1));
        q = i;
        i++;
      }

      qq = 0;
      m--;
      break;
    }
  }

  for (qp1jj = 0; qp1jj < 7; qp1jj++) {
    s[qp1jj] = b_s[qp1jj];
  }
}

static void xaxpy(int n, double a, int ix0, double y[49], int iy0)
{
  int ix;
  int iy;
  int k;
  if ((n < 1) || (a == 0.0)) {
  } else {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * y[ix];
      ix++;
      iy++;
    }
  }
}

static double xdotc(int n, const double x[49], int ix0, const double y[49], int
                    iy0)
{
  double d;
  int ix;
  int iy;
  int k;
  d = 0.0;
  if (!(n < 1)) {
    ix = ix0;
    iy = iy0;
    for (k = 1; k <= n; k++) {
      d += x[ix - 1] * y[iy - 1];
      ix++;
      iy++;
    }
  }

  return d;
}

static double xnrm2(int n, const double x[49], int ix0)
{
  double y;
  double scale;
  int kend;
  int k;
  double absxk;
  double t;
  y = 0.0;
  if (!(n < 1)) {
    if (n == 1) {
      y = fabs(x[ix0 - 1]);
    } else {
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (k = ix0; k <= kend; k++) {
        absxk = fabs(x[k - 1]);
        if (absxk > scale) {
          t = scale / absxk;
          y = 1.0 + y * t * t;
          scale = absxk;
        } else {
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * sqrt(y);
    }
  }

  return y;
}

static void xrot(double x[49], int ix0, int iy0, double c, double s)
{
  int ix;
  int iy;
  int k;
  double temp;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 0; k < 7; k++) {
    temp = c * x[ix] + s * x[iy];
    x[iy] = c * x[iy] - s * x[ix];
    x[ix] = temp;
    iy++;
    ix++;
  }
}

static void xrotg(double *a, double *b, double *c, double *s)
{
  double roe;
  double absa;
  double absb;
  double scale;
  double ads;
  double bds;
  roe = *b;
  absa = fabs(*a);
  absb = fabs(*b);
  if (absa > absb) {
    roe = *a;
  }

  scale = absa + absb;
  if (scale == 0.0) {
    *s = 0.0;
    *c = 1.0;
    scale = 0.0;
    *b = 0.0;
  } else {
    ads = absa / scale;
    bds = absb / scale;
    scale *= sqrt(ads * ads + bds * bds);
    if (roe < 0.0) {
      scale = -scale;
    }

    *c = *a / scale;
    *s = *b / scale;
    if (absa > absb) {
      *b = *s;
    } else if (*c != 0.0) {
      *b = 1.0 / *c;
    } else {
      *b = 1.0;
    }
  }

  *a = scale;
}

static void xscal(double a, double x[49], int ix0)
{
  int k;
  for (k = ix0; k <= ix0 + 6; k++) {
    x[k - 1] *= a;
  }
}

static void xswap(double x[49], int ix0, int iy0)
{
  int ix;
  int iy;
  int k;
  double temp;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 0; k < 7; k++) {
    temp = x[ix];
    x[ix] = x[iy];
    x[iy] = temp;
    ix++;
    iy++;
  }
}

void ellipsoid_method_initialize(void)
{
}

void ellipsoid_method_step1(double x, double y, double z, double A[49])
{
  double b_x[7];
  double c_x[7];
  int i2;
  int i3;
  b_x[0] = x * x;
  b_x[1] = y * y;
  b_x[2] = z * z;
  b_x[3] = x;
  b_x[4] = y;
  b_x[5] = z;
  b_x[6] = 1.0;
  c_x[0] = x * x;
  c_x[1] = y * y;
  c_x[2] = z * z;
  c_x[3] = x;
  c_x[4] = y;
  c_x[5] = z;
  c_x[6] = 1;//1.0;
  for (i2 = 0; i2 < 7; i2++) {
    for (i3 = 0; i3 < 7; i3++) {
      A[i2 + 7 * i3] += c_x[i2] * b_x[i3];
    }
  }
}

void ellipsoid_method_step2(const double A[49], double *x_scale, double *y_scale,
  double *z_scale, double *x_offset, double *y_offset, double *z_offset)
{
  int i0;
  double b_A[49];
  double U[49];
  double s[7];
  double V[49];
  int i1;
  double k;
  for (i0 = 0; i0 < 7; i0++) {
    for (i1 = 0; i1 < 7; i1++) {
      b_A[i1 + 7 * i0] = A[i0 + 7 * i1];
    }
  }

  svd(b_A, U, s, V);
  for (i0 = 0; i0 < 7; i0++) {
    for (i1 = 0; i1 < 7; i1++) {
      U[i1 + 7 * i0] = V[i0 + 7 * i1];
    }
  }

  k = 4.0 / (((U[27] * U[27] / U[6] + U[34] * U[34] / U[13]) + U[41] * U[41] /
              U[20]) - 4.0 * U[48]);
  *x_scale = sqrt(k * U[6]);
  *y_scale = sqrt(k * U[13]);
  *z_scale = sqrt(k * U[20]);
  *x_offset = k * U[27] / (2.0 * sqrt(k * U[6]));
  *y_offset = k * U[34] / (2.0 * sqrt(k * U[13]));
  *z_offset = k * U[41] / (2.0 * sqrt(k * U[20]));
}

void ellipsoid_method_terminate(void)
{
}
