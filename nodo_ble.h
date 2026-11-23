#ifndef NODO_BLE_H
#define NODO_BLE_H

#include <stdint.h>

#define NODO_BLE_FILA_BYTES   90U

void *nodo_ble_init_char(void);

int nodo_ble_post_fila(const uint8_t *fila, uint16_t n);

void nodo_ble_enviar_pendiente(void);

uint32_t nodo_ble_enviadas(void);
uint32_t nodo_ble_descartadas(void);
int      nodo_ble_suscrito(void);
uint16_t nodo_ble_max_char(void);

void nodo_ble_mtu(uint16_t max_char_len);

void nodo_ble_desconectado(void);

#endif
