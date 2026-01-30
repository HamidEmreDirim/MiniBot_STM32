#ifndef GPS_H
#define GPS_H

#include <stdint.h>
#include "main.h"
#include <stdbool.h>




#ifdef __cplusplus
extern "C" {
#endif

// ===== Types =====
typedef struct GpsData{
    uint8_t  first[512];
    uint16_t first_size;

    uint8_t  second[512];
    uint16_t second_size;

    volatile uint8_t head;
} gps_data;

typedef struct {
    /* From RMC */
    bool   rmc_valid;       /* status A/V */
    double lat_deg;
    double lon_deg;
    double speed_kn;        /* knots */
    double course_deg;      /* true course */
    char   date_utc[7];     /* ddmmyy (6 + NUL) */
    char   time_utc[11];    /* hhmmss.sss (10 + NUL) */

    /* From GGA */
    int    fix_quality;     /* 0=no fix,1=GPS,2=DGPS,... */
    int    sats;
    double hdop;
    double alt_m;           /* altitude meters */

    /* From ZDA (optional, richer date) */
    int    zda_day, zda_mon, zda_year; /* if available (>0) */
} gps_summary;

// ===== Globals you already use in main.c (moved here) =====
extern uint8_t  gps_dma_rx[512];
extern volatile uint16_t out_size;
extern volatile uint8_t *poll_gps;
extern volatile uint32_t gps_version;
extern gps_data gps_ring;

// ===== API you already call =====
uint8_t* get_gps_data(gps_data* gps, uint16_t* out_size);

// HAL DMA-to-idle RX callback you already implemented
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size);

#ifdef __cplusplus
}
#endif
#endif // GPS_H
