#include "cadena.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static void fft(float *re, float *im, int n)
{

    for (int i = 1, j = 0; i < n; i++) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) { float t;
            t = re[i]; re[i] = re[j]; re[j] = t;
            t = im[i]; im[i] = im[j]; im[j] = t; }
    }
    for (int len = 2; len <= n; len <<= 1) {
        double ang = -2.0 * M_PI / len;
        float wr = (float)cos(ang), wi = (float)sin(ang);
        for (int i = 0; i < n; i += len) {
            float cr = 1.0f, ci = 0.0f;
            for (int k = 0; k < len / 2; k++) {
                float ur = re[i+k],        ui = im[i+k];
                float vr = re[i+k+len/2] * cr - im[i+k+len/2] * ci;
                float vi = re[i+k+len/2] * ci + im[i+k+len/2] * cr;
                re[i+k]         = ur + vr;  im[i+k]         = ui + vi;
                re[i+k+len/2]   = ur - vr;  im[i+k+len/2]   = ui - vi;
                float nr = cr * wr - ci * wi;
                ci = cr * wi + ci * wr;  cr = nr;
            }
        }
    }
}

void cadena_init(cadena_t *c, float fs, float sens_g_lsb, float n_rpm)
{
    memset(c, 0, sizeof(*c));
    c->fs     = fs;
    c->fs_dec = fs / CAD_D;
    c->df     = c->fs_dec / CAD_N;
    c->sens   = sens_g_lsb;
    c->fr     = n_rpm / 60.0f;

    double s1 = 0.0, s2 = 0.0;
    for (int i = 0; i < CAD_N; i++) {
        c->w[i] = 0.5f * (1.0f - (float)cos(2.0 * M_PI * i / CAD_N));
        s1 += c->w[i];  s2 += (double)c->w[i] * c->w[i];
    }
    c->S1   = (float)s1;
    c->S2   = (float)s2;
    c->enbw = (float)(CAD_N * s2 / (s1 * s1));
    c->fase = CAD_D - 1;
    c->borde = CAD_BORDE;
}

void cadena_reiniciar(cadena_t *c)
{
    memset(c->z, 0, sizeof c->z);
    c->zi       = 0;
    c->fase     = CAD_D - 1;
    c->borde    = CAD_BORDE;
    c->nblk     = 0;
    c->sat_acum = 0;
}

void cadena_set_fs(cadena_t *c, float fs)
{
    c->fs     = fs;
    c->fs_dec = fs / CAD_D;
    c->df     = c->fs_dec / CAD_N;
}

void cadena_dft(const cadena_t *c, float fr, double dt0, double ts_dec,
                float amp[3][CAD_NARM], float fase[3][CAD_NARM])
{
    const double DOS_PI = 2.0 * M_PI;
    for (int e = 0; e < 3; e++) {
        double m0 = 0.0;
        for (int i = 0; i < CAD_N; i++) m0 += c->blk[e][i];
        const float media = (float)(m0 / CAD_N);
        for (int k = 1; k <= CAD_NARM; k++) {
            const double w0 = DOS_PI * k * (double)fr;
            const double paso = w0 * ts_dec;
            const float  cp = (float)cos(paso), sp = (float)sin(paso);
            float re = 0.0f, im = 0.0f, rr = 1.0f, ri = 0.0f;
            for (int m = 0; m < CAD_N; m++) {
                if ((m & 63) == 0) {
                    double th = fmod(w0 * dt0 + paso * m, DOS_PI);
                    rr = (float)cos(th);  ri = (float)-sin(th);
                }
                const float x = c->w[m] * (c->blk[e][m] - media);
                re += x * rr;  im += x * ri;

                const float nr = rr * cp + ri * sp;
                ri = ri * cp - rr * sp;  rr = nr;
            }
            re *= 2.0f / c->S1;  im *= 2.0f / c->S1;
            const float mod_v = sqrtf(re * re + im * im) / (float)(w0) * 1000.0f;
            amp[e][k - 1] = mod_v / sqrtf(2.0f);
            float g = atan2f(im, re) * (float)(180.0 / M_PI) - 90.0f;
            g = fmodf(g, 360.0f);
            if (g < 0.0f) g += 360.0f;
            fase[e][k - 1] = g;
        }
    }
}

static float rms_bins(const float *A, int lo, int hi, float enbw)
{
    double s = 0.0;
    for (int k = lo; k <= hi; k++) s += (double)A[k] * A[k];
    return (float)sqrt(s / (2.0 * enbw));
}

static void procesar_bloque(cadena_t *c)
{
    const int NH = CAD_N / 2;
    float A[CAD_N / 2 + 1];

    for (int e = 0; e < 3; e++) {

        double m = 0.0;
        for (int i = 0; i < CAD_N; i++) m += c->blk[e][i];
        m /= CAD_N;

        for (int i = 0; i < CAD_N; i++) {
            c->re[i] = (float)(c->blk[e][i] - m) * c->w[i];
            c->im[i] = 0.0f;
        }
        fft(c->re, c->im, CAD_N);

        for (int k = 0; k <= NH; k++) {
            float mag = sqrtf(c->re[k] * c->re[k] + c->im[k] * c->im[k]);
            A[k] = 2.0f * mag / c->S1;
        }
        A[0]  *= 0.5f;
        A[NH] *= 0.5f;

        for (int k = 0; k <= NH; k++) {
            float f = k * c->df;
            A[k] = (f < CAD_FHPF) ? 0.0f
                                  : A[k] / (2.0f * (float)M_PI * f) * 1000.0f;
        }

        int lo = (int)ceilf (CAD_ISO_LO / c->df);
        int hi = (int)floorf(CAD_ISO_HI / c->df);
        if (hi > NH) hi = NH;
        c->feat[e * (1 + CAD_NARM)] = rms_bins(A, lo, hi, c->enbw);

        for (int a = 1; a <= CAD_NARM; a++) {
            int i0 = (int)lrintf(a * c->fr / c->df);
            int l  = i0 - CAD_TOL, h = i0 + CAD_TOL;
            if (l < 0)  l = 0;
            if (h > NH) h = NH;
            c->feat[e * (1 + CAD_NARM) + a] = rms_bins(A, l, h, c->enbw);
        }
    }
    c->nfilas++;
}

int cadena_push(cadena_t *c, int16_t x, int16_t y, int16_t z)
{
    const int16_t v[3] = { x, y, z };
    float k = c->sens * CAD_G;

    for (int e = 0; e < 3; e++)
        if (v[e] >= CAD_SAT || v[e] <= -CAD_SAT) { c->sat_acum++; break; }

    for (int e = 0; e < 3; e++) c->z[e][c->zi] = (float)v[e] * k;

    int listo = 0;
    if (++c->fase == CAD_D) {
        c->fase = 0;
        for (int e = 0; e < 3; e++) {

            float acc = 0.0f;
            int idx = c->zi;
            for (int t = 0; t < FIR_NTAPS; t++) {
                acc += FIR_H[t] * c->z[e][idx];
                if (--idx < 0) idx = FIR_NTAPS - 1;
            }
            c->blk[e][c->nblk] = (float)acc;
        }
        if (c->borde > 0) {
            c->borde--;
        } else if (++c->nblk == CAD_N) {
            procesar_bloque(c);
            c->sat_blq   = c->sat_acum;
            c->sat_total += c->sat_acum;
            c->sat_acum  = 0;
            c->nblk = 0;
            listo = 1;
        }
    }
    if (++c->zi == FIR_NTAPS) c->zi = 0;
    return listo;
}

const float *cadena_features(const cadena_t *c) { return c->feat; }

const char *cadena_nombres(void)
{
    return "RMS_X,1X_X,2X_X,3X_X,RMS_Y,1X_Y,2X_Y,3X_Y,RMS_Z,1X_Z,2X_Z,3X_Z";
}
