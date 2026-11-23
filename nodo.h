#ifndef NODO_H
#define NODO_H
#include <stdint.h>

void bsp_cdc_write(const char *s, int n);

void nodo_printf(const char *fmt, ...);

void nodo_init(void);

int nodo_on_data(uint8_t sensor_id, const int16_t *ternas, uint32_t n_ternas, double ts);

#endif
