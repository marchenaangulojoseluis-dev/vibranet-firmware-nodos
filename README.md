# Firmware de los nodos VibraNet (STEVAL-STWINKT1B)

Código propio del nodo, sin comentarios. Va dentro de `Core/Nodo/` del proyecto
DATALOG2 de ST (FP-SNS-DATALOG2 v3.3.0); el resto del proyecto de ST no está aquí.

| Archivo | Qué hace |
|---|---|
| `nodo.c` / `nodo.h` | Lógica principal del nodo y captura del tacómetro (TIM2_CH1, PA15) |
| `nodo_ble.c` / `nodo_ble.h` | Característica BLE propia `4e4f444f-…` (notify), nombre `NODO-xx` |
| `cadena.c` / `cadena.h` / `fir_taps.h` | Cadena de señal: de las cuentas del IIS3DWB a 12 características |
