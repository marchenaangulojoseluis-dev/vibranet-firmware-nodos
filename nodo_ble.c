#include "nodo_ble.h"
#include "ble_manager.h"
#include "ble_manager_common.h"
#include "ble_stream_class.h"
#include "nodo.h"
#include "stm32l4xx_hal.h"
#include "tx_api.h"
#include <string.h>
#include <stdio.h>

#define COPY_NODO_CHAR_UUID(uuid_struct) COPY_UUID_128(uuid_struct, 0x4e, 0x4f, 0x44, 0x4f, 0x00, 0x01, 0x4b, 0x0a, \
                                                       0x9a, 0x1c, 0x2f, 0x5c, 0x7d, 0x3e, 0x1a, 0x01)

static ble_char_object_t ble_char_nodo;
static volatile uint8_t  nodo_suscrito;
static uint8_t           fila_buf[2][NODO_BLE_FILA_BYTES];
static volatile uint8_t  fila_idx;
static volatile uint8_t  fila_pend;
static uint32_t          n_enviadas, n_descartadas;

void set_board_name(void)
{
  uint32_t uid = HAL_GetUIDw0() ^ HAL_GetUIDw1() ^ HAL_GetUIDw2();
  uint8_t  b   = (uint8_t)(uid ^ (uid >> 8) ^ (uid >> 16) ^ (uid >> 24));
  snprintf(ble_stack_value.board_name, sizeof ble_stack_value.board_name, "NODO-%02X", (unsigned) b);
}

static void attr_mod_request_nodo(void *ble_char_pointer, uint16_t attr_handle, uint16_t offset,
                                  uint8_t data_length, uint8_t *att_data)
{
  (void) ble_char_pointer; (void) attr_handle; (void) offset; (void) data_length;
  nodo_suscrito = (att_data[0] == 1U) ? 1U : 0U;

  bsp_cdc_write(nodo_suscrito ? "NODO: BLE notify ON\r\n" : "NODO: BLE notify OFF\r\n", nodo_suscrito ? 21 : 22);
}

void *nodo_ble_init_char(void)
{
  ble_char_object_t *c = &ble_char_nodo;
  memset(c, 0, sizeof *c);
  c->attr_mod_request_cb = attr_mod_request_nodo;
  COPY_NODO_CHAR_UUID(c->uuid);
  c->char_uuid_type       = UUID_TYPE_128;
  c->char_value_length    = NODO_BLE_FILA_BYTES;
  c->char_properties      = (uint8_t) CHAR_PROP_NOTIFY;
  c->security_permissions = ATTR_PERMISSION_NONE;
  c->gatt_evt_mask        = GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP;
  c->enc_key_size         = 16;
  c->is_variable          = 0;
  nodo_printf("NODO: BLE caracteristica registrada (%u B notify)\r\n", (unsigned) NODO_BLE_FILA_BYTES);
  return c;
}

static volatile uint16_t nodo_max_char = 20U;

void nodo_ble_mtu(uint16_t max_char_len)
{
  nodo_max_char = max_char_len;
}

void nodo_ble_desconectado(void)
{
  nodo_max_char = 20U;
  nodo_suscrito = 0U;
  fila_pend     = 0U;
}

int nodo_ble_post_fila(const uint8_t *fila, uint16_t n)
{
  if ((n != NODO_BLE_FILA_BYTES) || (nodo_suscrito == 0U))
  {
    return 0;
  }
  if (nodo_max_char < NODO_BLE_FILA_BYTES)
  {
    n_descartadas++;
    return 0;
  }
  uint8_t i = (uint8_t)(fila_idx ^ 1U);
  memcpy(fila_buf[i], fila, NODO_BLE_FILA_BYTES);
  fila_idx = i;
  if (fila_pend)
  {
    n_descartadas++;
    return 1;
  }
  fila_pend = 1;
  streamMsg_t msg;
  msg.messageId = BLE_ISTREAM_MSG_SEND_NODO;
  msg.streamID  = 0;
  if (ble_stream_msg(&msg) != SYS_NO_ERROR_CODE)
  {
    fila_pend = 0;
    n_descartadas++;
    return 0;
  }
  return 1;
}

void nodo_ble_enviar_pendiente(void)
{
  uint8_t local[NODO_BLE_FILA_BYTES];
  if (fila_pend == 0U)
  {
    return;
  }
  memcpy(local, fila_buf[fila_idx], NODO_BLE_FILA_BYTES);
  fila_pend = 0;
  if ((nodo_suscrito == 0U) || (nodo_max_char < NODO_BLE_FILA_BYTES))
  {
    return;
  }
  for (int intento = 0; intento < 5; intento++)
  {
    ble_status_t ret = ACI_GATT_UPDATE_CHAR_VALUE(&ble_char_nodo, 0, (uint8_t) NODO_BLE_FILA_BYTES, local);
    if (ret == (ble_status_t) BLE_STATUS_SUCCESS)
    {
      n_enviadas++;
      return;
    }
    if (ret != (ble_status_t) BLE_STATUS_INSUFFICIENT_RESOURCES)
    {
      break;
    }
    tx_thread_sleep(1);
  }
  n_descartadas++;
}

uint32_t nodo_ble_enviadas(void)    { return n_enviadas; }
uint32_t nodo_ble_descartadas(void) { return n_descartadas; }
int      nodo_ble_suscrito(void)    { return (int) nodo_suscrito; }
uint16_t nodo_ble_max_char(void)    { return nodo_max_char; }
