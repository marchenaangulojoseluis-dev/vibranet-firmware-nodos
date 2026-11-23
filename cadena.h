#ifndef CADENA_H
#define CADENA_H
#include <stdint.h>

#define CAD_D        10
#define CAD_N        2048
#define CAD_NARM      3
#define CAD_TOL       2
#define CAD_FHPF     10.0f
#define CAD_BORDE   200
#define CAD_ISO_LO   10.0f
#define CAD_ISO_HI 1000.0f
#define CAD_G      9.80665f
#define CAD_SAT    32200
#define CAD_NFEAT   (3 * (1 + CAD_NARM))

#include "fir_taps.h"

typedef struct {
    float fs, fs_dec, df, sens, fr;
    float S1, S2, enbw;

    float z[3][FIR_NTAPS];
    int   zi, fase, borde;

    float blk[3][CAD_N];
    int   nblk;

    float w[CAD_N];
    float re[CAD_N], im[CAD_N];

    float feat[CAD_NFEAT];
    uint32_t nfilas;

    uint16_t sat_blq;
    uint16_t sat_acum;
    uint32_t sat_total;
} cadena_t;

void         cadena_init(cadena_t *c, float fs, float sens_g_lsb, float n_rpm);

void         cadena_reiniciar(cadena_t *c);

void         cadena_set_fs(cadena_t *c, float fs);

void         cadena_dft(const cadena_t *c, float fr, double dt0, double ts_dec,
                        float amp[3][CAD_NARM], float fase[3][CAD_NARM]);
int          cadena_push(cadena_t *c, int16_t x, int16_t y, int16_t z);
const float *cadena_features(const cadena_t *c);
const char  *cadena_nombres(void);

static inline uint16_t cadena_saturadas(const cadena_t *c){ return c->sat_blq; }

#endif
