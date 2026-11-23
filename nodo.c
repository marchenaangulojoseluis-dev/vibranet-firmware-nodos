#include "nodo.h"
#include "cadena.h"
#include "nodo_ble.h"
#include "usart.h"
#include "tx_api.h"
#include "services/sysinit.h"
#include "SensorManager.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include "services/SysTimestamp.h"
#include "fir_taps.h"

#define FS_HZ        26578.556f
#define SENS_G_LSB   0.000488f
#define N_RPM        1700.0f

#define NODO_RING_TERNAS   8192
#define NODO_STACK_BYTES   8192
#define NODO_ARRANQUE_MS   3000
#define NODO_ESTADO_S      5

static cadena_t   cad;
static uint8_t    iis3dwb_id = 0xFF;

static int16_t    ring[3 * NODO_RING_TERNAS];
static volatile uint32_t ring_head;
static volatile uint32_t ring_tail;
static volatile uint32_t ring_perdidas;
static uint32_t   muestras_total, filas_total;

static TX_THREAD    nodo_thread;
static TX_SEMAPHORE nodo_sem;
static TX_TIMER     nodo_timer;
static ULONG        nodo_stack[NODO_STACK_BYTES / sizeof(ULONG)];

#define TACO_RECHAZO_MS   5u
#define TACO_IRQ_PRIO     6u
#define TACO_LENTO_MS     100.0f
#define TACO_BLOQUES_VIGENCIA 4u

static uint32_t          taco_fclk;
static uint32_t          taco_rechazo_ticks;
static volatile uint32_t taco_pulsos;
static volatile uint32_t taco_rechazados;
static volatile uint32_t taco_sobrecapturas;
static volatile uint32_t taco_ultimo;
static volatile uint32_t taco_periodo;

#define CAPT_N_ISR 64u
static volatile uint32_t capt_isr[CAPT_N_ISR];
static volatile uint32_t capt_head, capt_tail, capt_desbordes;

static void taco_init(void)
{

  taco_fclk = HAL_RCC_GetPCLK1Freq();
  if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
  {
    taco_fclk *= 2u;
  }
  taco_rechazo_ticks = taco_fclk / 1000u * TACO_RECHAZO_MS;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();

  TIM2->CR1   = 0;
  TIM2->PSC   = 0;
  TIM2->ARR   = 0xFFFFFFFFu;
  TIM2->CCER  = 0;
  TIM2->CCMR1 = (1u << TIM_CCMR1_CC1S_Pos)
              | (0xFu << TIM_CCMR1_IC1F_Pos);
  TIM2->CCER  = TIM_CCER_CC1P | TIM_CCER_CC1E;
  TIM2->EGR   = TIM_EGR_UG;
  TIM2->SR    = 0;
  TIM2->DIER  = TIM_DIER_CC1IE;

  GPIO_InitTypeDef g = { 0 };
  g.Pin       = GPIO_PIN_15;
  g.Mode      = GPIO_MODE_AF_PP;
  g.Pull      = GPIO_NOPULL;
  g.Speed     = GPIO_SPEED_FREQ_LOW;
  g.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &g);

  HAL_NVIC_SetPriority(TIM2_IRQn, TACO_IRQ_PRIO, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  TIM2->CR1 = TIM_CR1_CEN;
}

void TIM2_IRQHandler(void)
{
  uint32_t sr = TIM2->SR;
  if (sr & TIM_SR_CC1IF)
  {
    uint32_t cap = TIM2->CCR1;
    if (sr & TIM_SR_CC1OF)
    {
      taco_sobrecapturas++;
      TIM2->SR = ~TIM_SR_CC1OF;
    }
    uint32_t dt = cap - taco_ultimo;
    if ((taco_pulsos != 0u) && (dt < taco_rechazo_ticks))
    {
      taco_rechazados++;
    }
    else
    {
      if (taco_pulsos != 0u)
      {
        taco_periodo = dt;
      }
      taco_ultimo = cap;
      taco_pulsos++;
      if (capt_head - capt_tail >= CAPT_N_ISR)
      {
        capt_desbordes++;
      }
      else
      {
        capt_isr[capt_head % CAPT_N_ISR] = cap;
        capt_head++;
      }
    }
  }
}

void bsp_cdc_write(const char *s, int n)
{

  if (huart2.Instance == NULL)
  {
    MX_USART2_UART_Init();
  }
  if (n > 0)
  {

    HAL_UART_Transmit(&huart2, (const uint8_t *) s, (uint16_t) n, 100);
  }
}

void nodo_printf(const char *fmt, ...)
{
  static char buf[256];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  if (n > (int) sizeof buf - 1)
  {
    n = sizeof buf - 1;
  }
  if (n > 0)
  {
    bsp_cdc_write(buf, n);
  }
}

static uint32_t bsp_epoch(void)
{

  return tx_time_get() / TX_TIMER_TICKS_PER_SECOND;
}

#define SELLOS_RING      64u
#define SELLOS_HIST      192u
#define PULSOS_HIST      64u
#define AJUSTE_MIN_PARES 40u
#define AJUSTE_RES_MAX_US 1000.0
#define AJUSTE_RES_RMS_US 100.0
#define FASE_PULSOS_MAX_DEG 5.0
#define ORIGEN_SILENCIO_S 3u
#define RETARDO_FIR_BRUTAS ((FIR_NTAPS - 1) / 2)
#define BLOQUE_SIN_ORIGEN 0xFFFFFFFFu

typedef struct { uint64_t idx_ini, ticks; uint32_t n; uint8_t disc, con_sello; } sello_t;
typedef struct { uint64_t idx_fin, ticks; } par_t;
typedef struct { uint64_t ticks; uint32_t rev; uint8_t rev_ok; } pulso_t;

static uint32_t          ts_fclk;
static uint8_t           base_comun;

static sello_t           sellos[SELLOS_RING];
static volatile uint32_t sellos_head, sellos_tail;
static uint64_t          n_in;
static uint64_t          ticks_prev_evt;
static uint8_t           disc_pendiente;
static volatile uint32_t sellos_perdidos, discontinuidades;
volatile uint32_t        nodo_fifo_ovr;
static uint32_t          fifo_ovr_visto;

static par_t    hist[SELLOS_HIST];  static uint32_t hist_n, hist_i;
static pulso_t  phist[PULSOS_HIST]; static uint32_t phist_n, phist_i;
static uint64_t n_out;
static uint8_t  hay_tiempo;
static uint64_t silencio_ref, pulso_prev;  static uint8_t hay_pulso_prev, origen_ok;
static uint32_t rev_actual, ovf_vistos, desb_vistos;
static uint64_t ultimo_ahora;

static float    diag_fs, diag_res_rms_us, diag_pulsos_deg;
static uint32_t diag_pares, bloques_fase, bloques_sin;

static void hist_reiniciar(void) { hist_n = 0; hist_i = 0; }

static void hist_poner(uint64_t idx_fin, uint64_t ticks)
{
  hist[hist_i].idx_fin = idx_fin;  hist[hist_i].ticks = ticks;
  hist_i = (hist_i + 1u) % SELLOS_HIST;
  if (hist_n < SELLOS_HIST) hist_n++;
}

static void pulsos_reiniciar(void) { phist_n = 0; phist_i = 0; hay_pulso_prev = 0; origen_ok = 0; }

static void drenar_pulsos(void)
{
  if (!hay_tiempo || !base_comun)
  {
    capt_tail = capt_head;
    return;
  }

  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  uint32_t fin = capt_head;
  uint32_t ovf = taco_sobrecapturas, desb = capt_desbordes;
  uint64_t n5a = SysTsGetTimestampN(SysGetTimestampSrv());
  uint32_t c2  = TIM2->CNT;
  uint64_t n5b = SysTsGetTimestampN(SysGetTimestampSrv());
  __set_PRIMASK(primask);
  __DMB();
  uint64_t ahora = n5a + (n5b - n5a) / 2u;
  if (ahora < ultimo_ahora)
  {

    pulsos_reiniciar();
    capt_tail = fin;
    silencio_ref = ahora;  ultimo_ahora = ahora;
    return;
  }
  ultimo_ahora = ahora;
  if ((ovf != ovf_vistos) || (desb != desb_vistos))
  {

    ovf_vistos = ovf;  desb_vistos = desb;
    for (uint32_t i = 0; i < PULSOS_HIST; i++) phist[i].rev_ok = 0;
    origen_ok = 0;  hay_pulso_prev = 0;  silencio_ref = ahora;
    capt_tail = fin;
    return;
  }
  if (capt_tail == fin) return;
  if (n5b - n5a > (uint64_t) (ts_fclk / 100000u))
  {
    return;
  }
  const uint64_t silencio = (uint64_t) ORIGEN_SILENCIO_S * ts_fclk;
  while (capt_tail != fin)
  {
    uint32_t cap = capt_isr[capt_tail % CAPT_N_ISR];
    capt_tail++;
    uint32_t edad = c2 - cap;
    if ((edad >= 0x80000000u) || ((uint64_t) edad > ahora))
    {
      continue;
    }
    uint64_t t = ahora - edad;
    if (hay_pulso_prev && (t <= pulso_prev))
    {
      continue;
    }
    if (!hay_pulso_prev)
    {
      origen_ok = (t > silencio_ref) && (t - silencio_ref >= silencio);
      rev_actual = 0;
    }
    else if (t - pulso_prev >= silencio)
    {
      origen_ok = 1;  rev_actual = 0;
    }
    else
    {
      rev_actual++;
    }
    pulso_prev = t;  hay_pulso_prev = 1;
    phist[phist_i].ticks = t;  phist[phist_i].rev = rev_actual;  phist[phist_i].rev_ok = origen_ok;
    phist_i = (phist_i + 1u) % PULSOS_HIST;
    if (phist_n < PULSOS_HIST) phist_n++;
  }
}

static void preparar_lote(const sello_t *s)
{
  if (s->disc)
  {
    cadena_reiniciar(&cad);
    hist_reiniciar();
    if (s->disc == 2u)
    {
      pulsos_reiniciar();
      silencio_ref = s->ticks;
    }
  }
  if (!hay_tiempo && s->con_sello)
  {
    hay_tiempo = 1;
    silencio_ref = s->ticks;
  }
  if (s->con_sello && s->n)
  {
    hist_poner(s->idx_ini + s->n - 1u, s->ticks);
  }
}

static int ajustar_tiempo(uint64_t n0, uint64_t lo, uint64_t hi, double *a, double *b, uint64_t *ref)
{
  double sx = 0, sy = 0, sxx = 0, sxy = 0;
  uint32_t k = 0;
  uint64_t r = 0;
  for (uint32_t i = 0; i < hist_n; i++)
  {
    const par_t *p = &hist[i];
    if (p->idx_fin < lo || p->idx_fin > hi) continue;
    if (k == 0) r = p->ticks;
    double x = (double) (int64_t) (p->idx_fin - n0), y = (double) (int64_t) (p->ticks - r);
    sx += x; sy += y; sxx += x * x; sxy += x * y; k++;
  }
  diag_pares = k;
  if (k < AJUSTE_MIN_PARES) return 0;
  double den = k * sxx - sx * sx;
  if (den <= 0.0) return 0;
  *b = (k * sxy - sx * sy) / den;
  *a = (sy - *b * sx) / k;
  *ref = r;
  double s2 = 0, mx = 0;
  for (uint32_t i = 0; i < hist_n; i++)
  {
    const par_t *p = &hist[i];
    if (p->idx_fin < lo || p->idx_fin > hi) continue;
    double x = (double) (int64_t) (p->idx_fin - n0), y = (double) (int64_t) (p->ticks - r);
    double e = y - (*a + *b * x);
    s2 += e * e;  if (fabs(e) > mx) mx = fabs(e);
  }
  double us = 1e6 / ts_fclk;
  diag_res_rms_us = (float) (sqrt(s2 / k) * us);
  diag_fs = (float) (ts_fclk / *b);
  return (mx * us < AJUSTE_RES_MAX_US) && (diag_res_rms_us < AJUSTE_RES_RMS_US)
         && (fabsf(diag_fs / FS_HZ - 1.0f) < 0.01f);
}

static int ajustar_pulsos(uint64_t t0, uint64_t t1, double *tp_rel, double *T, uint64_t *ref,
                          uint32_t *rev, uint8_t *rev_ok)
{
  uint32_t k = 0;
  double sx = 0, sy = 0, sxx = 0, sxy = 0;
  uint64_t r = 0;
  uint32_t ini = (phist_i + PULSOS_HIST - phist_n) % PULSOS_HIST;
  for (uint32_t q = 0; q < phist_n; q++)
  {
    const pulso_t *p = &phist[(ini + q) % PULSOS_HIST];
    if (p->ticks < t0 || p->ticks > t1) continue;
    if (k == 0) { r = p->ticks; *rev = p->rev; *rev_ok = p->rev_ok; }
    double x = k, y = (double) (int64_t) (p->ticks - r);
    sx += x; sy += y; sxx += x * x; sxy += x * y; k++;
  }
  if (k < 2) return 0;
  double den = k * sxx - sx * sx;
  *T = (k * sxy - sx * sy) / den;
  *tp_rel = (sy - *T * sx) / k;
  *ref = r;
  if (*T <= 0.0) return 0;
  double mx = 0;
  k = 0;
  for (uint32_t q = 0; q < phist_n; q++)
  {
    const pulso_t *p = &phist[(ini + q) % PULSOS_HIST];
    if (p->ticks < t0 || p->ticks > t1) continue;
    double e = (double) (int64_t) (p->ticks - r) - (*tp_rel + *T * k);
    if (fabs(e) > mx) mx = fabs(e);
    k++;
  }
  diag_pulsos_deg = (float) (360.0 * mx / *T);
  return diag_pulsos_deg <= FASE_PULSOS_MAX_DEG;
}

static void emitir_fila(uint32_t epoch, uint32_t bloque, float rpm, const float *f,
                        const float fases[6])
{
  static char buf[256];
  int n = snprintf(buf, sizeof buf, "%lu,%lu,%.1f", (unsigned long) epoch,
                   (unsigned long) bloque, (double) rpm);
  for (int i = 0; i < CAD_NFEAT; i++)
  {
    n += snprintf(buf + n, sizeof buf - n, ",%.6e", (double) f[i]);
  }
  n += snprintf(buf + n, sizeof buf - n, ",%u\r\n", cadena_saturadas(&cad));
  bsp_cdc_write(buf, n);

  uint8_t  bin[NODO_BLE_FILA_BYTES];
  uint16_t o = 0;
  uint16_t sat = cadena_saturadas(&cad);
  uint32_t pulsos = taco_pulsos;
  memcpy(bin + o, &epoch,  4); o += 4;
  memcpy(bin + o, &bloque, 4); o += 4;
  memcpy(bin + o, &rpm,    4); o += 4;
  memcpy(bin + o, f, CAD_NFEAT * sizeof(float)); o += CAD_NFEAT * sizeof(float);
  memcpy(bin + o, &sat,    2); o += 2;
  memcpy(bin + o, fases, 6 * sizeof(float)); o += 6 * sizeof(float);
  memcpy(bin + o, &pulsos, 4); o += 4;
  nodo_ble_post_fila(bin, o);
}

static void procesar_bloque(uint32_t epoch, uint64_t n_fin)
{
  float f[CAD_NFEAT];
  memcpy(f, cadena_features(&cad), sizeof f);
  float fases[6] = { NAN, NAN, NAN, NAN, NAN, NAN };
  float rpm = NAN;
  uint32_t bloque = BLOQUE_SIN_ORIGEN;

  drenar_pulsos();
  uint64_t n0 = n_fin - (uint64_t) CAD_D * (CAD_N - 1);
  double a, b; uint64_t ref5;
  uint64_t lo = (n0 >= 2560u) ? n0 - 2560u : 0u;
  int ok_t = base_comun && ajustar_tiempo(n0, lo, n_fin + 2560u, &a, &b, &ref5);
  if (ok_t)
  {
    cadena_set_fs(&cad, diag_fs);

    double r0 = a - b * RETARDO_FIR_BRUTAS;
    uint64_t t_ini = ref5 + (uint64_t) (int64_t) llround(r0);
    uint64_t t_fin = ref5 + (uint64_t) (int64_t) llround(r0 + b * (double) CAD_D * (CAD_N - 1));
    double tp_rel, T; uint64_t refp; uint32_t rev = 0; uint8_t rev_ok = 0;
    if (ajustar_pulsos(t_ini, t_fin, &tp_rel, &T, &refp, &rev, &rev_ok))
    {
      double fr = (double) ts_fclk / T;
      double dt0 = ((double) (int64_t) (t_ini - refp) - tp_rel) / ts_fclk;
      float amp[3][CAD_NARM], fase[3][CAD_NARM];
      cadena_dft(&cad, (float) fr, dt0, (double) CAD_D * b / ts_fclk, amp, fase);
      for (int e = 0; e < 3; e++)
      {
        for (int k = 0; k < CAD_NARM; k++) f[e * (1 + CAD_NARM) + 1 + k] = amp[e][k];
        if (cadena_saturadas(&cad) == 0u)
        {
          fases[e]     = fase[e][0];
          fases[3 + e] = fase[e][1];
        }
      }
      rpm = (float) (60.0 * fr);
      if (rev_ok) bloque = rev;
      bloques_fase++;
    }
    else bloques_sin++;
  }
  else bloques_sin++;
  emitir_fila(epoch, bloque, rpm, f, fases);
}

int nodo_on_data(uint8_t sensor_id, const int16_t *ternas, uint32_t n_ternas, double ts)
{
  if (sensor_id != iis3dwb_id)
  {
    return 0;
  }
  uint64_t ticks = (uint64_t) llround(ts * (double) ts_fclk);
  uint8_t disc = disc_pendiente;
  if (ticks_prev_evt != 0u)
  {
    if (ticks < ticks_prev_evt)
    {
      disc = 2u;
    }
    else if (disc == 0u)
    {

      double esperado = (double) n_ternas / FS_HZ * ts_fclk;
      if ((double) (ticks - ticks_prev_evt) > 1.5 * esperado + 0.0005 * ts_fclk) disc = 1u;
    }
  }
  ticks_prev_evt = ticks;
  uint32_t ovr = nodo_fifo_ovr;
  if (ovr != fifo_ovr_visto)
  {
    fifo_ovr_visto = ovr;
    if (disc == 0u) disc = 1u;
  }

  uint32_t head = ring_head, tail = ring_tail;
  uint32_t sh = sellos_head, st = sellos_tail;
  __DMB();
  uint32_t libres = NODO_RING_TERNAS - 1 - ((head - tail) % NODO_RING_TERNAS);
  uint8_t  cortado = 0;
  if (sh - st >= SELLOS_RING)
  {
    libres = 0;
    sellos_perdidos++;
  }
  if (n_ternas > libres)
  {
    ring_perdidas += n_ternas - libres;
    n_ternas = libres;
    cortado = 1;
  }
  if (n_ternas == 0u)
  {
    if (disc > disc_pendiente) disc_pendiente = disc;
    if (disc_pendiente == 0u) disc_pendiente = 1u;
    return 1;
  }
  for (uint32_t i = 0; i < n_ternas; i++)
  {
    memcpy(&ring[3 * head], &ternas[3 * i], 3 * sizeof(int16_t));
    head = (head + 1) % NODO_RING_TERNAS;
  }
  sello_t *s = &sellos[sellos_head % SELLOS_RING];
  s->idx_ini = n_in;  s->n = n_ternas;  s->ticks = ticks;  s->disc = disc;  s->con_sello = !cortado;
  n_in += n_ternas;
  if (disc) discontinuidades++;
  disc_pendiente = cortado ? 1u : 0u;
  __DMB();
  ring_head = head;
  sellos_head++;
  tx_semaphore_ceiling_put(&nodo_sem, 1);
  return 1;
}

static void nodo_thread_entry(ULONG arg)
{
  (void) arg;
  uint32_t epoch = bsp_epoch();
  uint32_t ultimo_estado = epoch;
  uint32_t pulsos_impresos = 0;

  cadena_init(&cad, FS_HZ, SENS_G_LSB, N_RPM);
  nodo_printf("NODO: tarea lista, esperando datos del IIS3DWB (id=%u)\r\n", iis3dwb_id);

  for (;;)
  {
    tx_semaphore_get(&nodo_sem, TX_WAIT_FOREVER);
    drenar_pulsos();

    while (sellos_tail != sellos_head)
    {
      __DMB();
      sello_t s = sellos[sellos_tail % SELLOS_RING];
      n_out = s.idx_ini;
      preparar_lote(&s);
      uint32_t t = ring_tail;
      for (uint32_t i = 0; i < s.n; i++)
      {
        int16_t x = ring[3 * t], y = ring[3 * t + 1], z = ring[3 * t + 2];
        t = (t + 1) % NODO_RING_TERNAS;
        muestras_total++;
        if (cadena_push(&cad, x, y, z))
        {
          procesar_bloque(epoch, n_out);
          filas_total++;
        }
        n_out++;
      }
      __DMB();
      ring_tail = t;
      sellos_tail++;
    }

    uint32_t pulsos = taco_pulsos;
    if (pulsos != pulsos_impresos)
    {
      float t_ms = (float) taco_periodo * 1000.0f / (float) taco_fclk;
      if ((pulsos - pulsos_impresos == 1u) && (pulsos == 1u || t_ms > TACO_LENTO_MS))
      {
        nodo_printf("# pulso n=%lu T=%.1f ms\r\n", (unsigned long) pulsos,
                    (double) (pulsos == 1u ? 0.0f : t_ms));
      }
      pulsos_impresos = pulsos;
    }

    uint32_t ahora = bsp_epoch();
    if (ahora - ultimo_estado >= NODO_ESTADO_S)
    {
      uint32_t dt = (ahora > epoch) ? (ahora - epoch) : 1;
      ultimo_estado = ahora;
      nodo_printf("# t=%lus muestras=%lu (%.1f/s) filas=%lu perdidas=%lu ble_sub=%d ble_env=%lu ble_desc=%lu\r\n",
                  (unsigned long) ahora, (unsigned long) muestras_total,
                  (double) muestras_total / (double) dt,
                  (unsigned long) filas_total, (unsigned long) ring_perdidas,
                  nodo_ble_suscrito(), (unsigned long) nodo_ble_enviadas(),
                  (unsigned long) nodo_ble_descartadas());
      float t_ms = (float) taco_periodo * 1000.0f / (float) taco_fclk;
      nodo_printf("# taco pulsos=%lu rech=%lu ovf=%lu desb=%lu T=%.3f ms ble_max=%u origen=%u rev=%lu\r\n",
                  (unsigned long) taco_pulsos, (unsigned long) taco_rechazados,
                  (unsigned long) taco_sobrecapturas, (unsigned long) capt_desbordes, (double) t_ms,
                  (unsigned) nodo_ble_max_char(), (unsigned) origen_ok, (unsigned long) rev_actual);
      nodo_printf("# fase bloques_ok=%lu sin=%lu pares=%lu fs=%.2f res=%.1fus pulsos=%.2fdeg disc=%lu sellos_perd=%lu base=%u\r\n",
                  (unsigned long) bloques_fase, (unsigned long) bloques_sin, (unsigned long) diag_pares,
                  (double) diag_fs, (double) diag_res_rms_us, (double) diag_pulsos_deg,
                  (unsigned long) discontinuidades, (unsigned long) sellos_perdidos, (unsigned) base_comun);
    }
  }
}

static void nodo_timer_cb(ULONG arg)
{
  (void) arg;
  SysEvent evt = { .nRawEvent = SYS_PM_MAKE_EVENT(SYS_PM_EVT_SRC_DATALOG, 0) };
  SysPostPowerModeEvent(evt);
}

void nodo_init(void)
{
  nodo_printf("\r\nNODO: cadena.c integrada. D=%d N=%d fs_dec=%.1f Hz\r\n",
              CAD_D, CAD_N, FS_HZ / CAD_D);

  uint16_t n = SMGetNsensor();
  for (uint8_t i = 0; i < n; i++)
  {
    SensorStatus_t st = SMSensorGetStatus(i);
    if (strcmp(st.p_name, "iis3dwb") == 0)
    {
      iis3dwb_id = i;
    }
    else
    {
      SMSensorDisable(i);
    }
  }
  nodo_printf("NODO: %u sensores, iis3dwb id=%u, el resto deshabilitado\r\n", n, iis3dwb_id);
  nodo_printf("epoch,bloque,n_rpm,%s,sat\r\n", cadena_nombres());

  taco_init();
  ts_fclk    = SYS_TS_CFG_TSDRIVER_FREQ_HZ;
  base_comun = (taco_fclk == ts_fclk);
  nodo_printf("NODO: base de tiempo TIM5 %lu Hz, TIM2 %lu Hz -> fases %s\r\n", (unsigned long) ts_fclk,
              (unsigned long) taco_fclk, base_comun ? "ON" : "OFF (relojes distintos)");
  nodo_printf("NODO: tacometro en PA15/TIM2_CH1, flanco de bajada, rechazo < %u ms, fclk=%lu Hz\r\n",
              TACO_RECHAZO_MS, (unsigned long) taco_fclk);

  tx_semaphore_create(&nodo_sem, "NODO_S", 0);
  tx_thread_create(&nodo_thread, "NODO", nodo_thread_entry, 0,
                   nodo_stack, sizeof nodo_stack,
                   20, 20, TX_NO_TIME_SLICE, TX_AUTO_START);
  tx_timer_create(&nodo_timer, "NODO_T", nodo_timer_cb, 0,
                  NODO_ARRANQUE_MS * TX_TIMER_TICKS_PER_SECOND / 1000, 0, TX_AUTO_ACTIVATE);
}
