#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include "gps.h"

/* Ensure a memory barrier macro exists (for non-ARM unit tests) */
#ifndef __DMB
#define __DMB() do { __asm volatile ("" ::: "memory"); } while(0)
#endif

/* UART from CubeMX (declared in usart.c) */
extern UART_HandleTypeDef huart3;

/* ===== Globals as you had them ===== */
uint8_t  gps_dma_rx[512] = {0};
volatile uint16_t out_size;
volatile uint8_t *poll_gps;
volatile uint32_t gps_version = 0;
gps_data gps_ring = { .head = 0 };

/* =========================
   Private helpers (static)
   ========================= */

static bool nmea_checksum_ok(const char* s, size_t n)
{
    if (n < 4 || s[0] != '$') return false;
    const char* star = (const char*)memchr(s, '*', n);
    if (!star || (star + 2) >= (s + n)) return false;

    uint8_t sum = 0;
    for (const char* p = s + 1; p < star; ++p) sum ^= (uint8_t)(*p);

    int hi = toupper((unsigned char)star[1]);
    int lo = toupper((unsigned char)star[2]);
    int hex = ((hi >= 'A') ? (hi - 'A' + 10) : (hi - '0')) * 16
            + ((lo >= 'A') ? (lo - 'A' + 10) : (lo - '0'));

    return (sum == (uint8_t)hex);
}

/* Trim CR/LF in place, return new length */
static size_t rstrip_crlf(char* s, size_t n)
{
    while (n && (s[n-1] == '\r' || s[n-1] == '\n')) --n;
    s[n] = '\0';
    return n;
}

/* Append helpers (no float %f anywhere) */
static uint16_t append_bytes(uint8_t* dst, uint16_t cur, uint16_t cap,
                             const char* s, uint16_t len)
{
    if (cur >= cap) return cur;
    uint16_t room = (uint16_t)(cap - cur);
    uint16_t n = (len <= room) ? len : room;
    memcpy(&dst[cur], s, n);
    return (uint16_t)(cur + n);
}

static uint16_t append_str(uint8_t* dst, uint16_t cur, uint16_t cap, const char* s)
{
    return append_bytes(dst, cur, cap, s, (uint16_t)strlen(s));
}

static uint16_t append_int32(uint8_t* dst, uint16_t cur, uint16_t cap, int32_t v)
{
    char tmp[16];
    char* p = &tmp[15];
    *p = '\0';
    uint32_t x = (v < 0) ? (uint32_t)(-v) : (uint32_t)v;
    do {
        *--p = (char)('0' + (x % 10));
        x /= 10;
    } while (x);
    if (v < 0) *--p = '-';
    return append_str(dst, cur, cap, p);
}

/* Fixed-point float -> string without %f */
static uint16_t append_ftoa_fixed(uint8_t* dst, uint16_t cur, uint16_t cap, double val, int precision)
{
    if (precision < 0) precision = 0;
    if (precision > 9) precision = 9;

    int neg = (val < 0.0) ? 1 : 0;
    if (neg) val = -val;

    int32_t ip = (int32_t)val;

    uint32_t scale = 1;
    for (int i = 0; i < precision; ++i) scale *= 10;

    double frac_d = (val - (double)ip) * (double)scale + 0.5;
    uint32_t frac = (uint32_t)frac_d;
    if (frac >= scale) { frac -= scale; ip += 1; }

    if (neg) cur = append_str(dst, cur, cap, "-");
    cur = append_int32(dst, cur, cap, ip);

    if (precision > 0) {
        cur = append_str(dst, cur, cap, ".");
        char pad[10];
        int pad_needed = precision;
        char rev[10];
        int r = 0;

        if (frac != 0) {
            while (frac && r < 10) {
                rev[r++] = (char)('0' + (frac % 10));
                frac /= 10;
                pad_needed--;
            }
        }
        for (int i = 0; i < pad_needed; ++i) pad[i] = '0';
        cur = append_bytes(dst, cur, cap, pad, (uint16_t)pad_needed);
        while (r--) cur = append_bytes(dst, cur, cap, &rev[r], 1);
    }
    return cur;
}

/* ddmm.mmmm to signed degrees */
static double parse_lat_ddmm(char* f_lat, char* f_ns)
{
    if (!f_lat || !*f_lat) return 0.0;
    double v = atof(f_lat);
    int deg = (int)(v / 100.0);
    double min = v - (deg * 100.0);
    double dd = deg + (min / 60.0);
    if (f_ns && (*f_ns == 'S' || *f_ns == 's')) dd = -dd;
    return dd;
}

/* dddmm.mmmm to signed degrees */
static double parse_lon_dddmm(char* f_lon, char* f_ew)
{
    if (!f_lon || !*f_lon) return 0.0;
    double v = atof(f_lon);
    int deg = (int)(v / 100.0);
    double min = v - (deg * 100.0);
    double dd = deg + (min / 60.0);
    if (f_ew && (*f_ew == 'W' || *f_ew == 'w')) dd = -dd;
    return dd;
}

/* Split fields between first comma and '*' (or end). Returns count. */
static int split_fields(char* line_after_id, char* fields[], int max_fields)
{
    int cnt = 0;
    char* start = line_after_id;
    char* star  = strchr(line_after_id, '*');
    if (star) *star = '\0'; /* stop at '*' */

    while (cnt < max_fields && start && *start) {
        fields[cnt++] = start;
        char* c = strchr(start, ',');
        if (!c) break;
        *c = '\0';
        start = c + 1;
    }

    if (star) *star = '*'; /* restore for safety (not strictly needed) */
    return cnt;
}

/* init aggregated summary */
static void summary_init(gps_summary* g)
{
    memset(g, 0, sizeof(*g));
    g->fix_quality = 0;
    g->sats        = 0;
    g->hdop        = 0.0;
    g->alt_m       = 0.0;
}

/* parse ONE NMEA sentence (line is '\0'-terminated, without CR/LF) and update 'out' */
static void parse_sentence(char* line, gps_summary* out)
{
    size_t n = strlen(line);
    if (!n || line[0] != '$' || !nmea_checksum_ok(line, n)) return;

    /* Identify sentence type by the 6-char talker+id */
    const int is_RMC = (strncmp(line, "$GPRMC", 6) == 0) || (strncmp(line, "$GNRMC", 6) == 0);
    const int is_GGA = (strncmp(line, "$GPGGA", 6) == 0) || (strncmp(line, "$GNGGA", 6) == 0);
    const int is_ZDA = (strncmp(line, "$GPZDA", 6) == 0) || (strncmp(line, "$GNZDA", 6) == 0);

    /* Find first comma after the 6-char id */
    char* first_comma = strchr(line, ',');
    if (!first_comma) return;
    char* fields[20] = {0};
    int nf = split_fields(first_comma + 1, fields, 20);

    if (is_RMC) {
        /* RMC indices (relative to first field after sentence id):
           0 time, 1 status, 2 lat, 3 N/S, 4 lon, 5 E/W, 6 speed(kn), 7 course, 8 date(ddmmyy) */
        char *f_time = (nf > 0) ? fields[0] : NULL;
        char *f_stat = (nf > 1) ? fields[1] : NULL;
        char *f_lat  = (nf > 2) ? fields[2] : NULL;
        char *f_ns   = (nf > 3) ? fields[3] : NULL;
        char *f_lon  = (nf > 4) ? fields[4] : NULL;
        char *f_ew   = (nf > 5) ? fields[5] : NULL;
        char *f_spd  = (nf > 6) ? fields[6] : NULL;
        char *f_cog  = (nf > 7) ? fields[7] : NULL;
        char *f_date = (nf > 8) ? fields[8] : NULL;

        if (f_stat && (*f_stat=='A' || *f_stat=='a')) out->rmc_valid = true;
        if (f_time && *f_time) { strncpy(out->time_utc, f_time, sizeof(out->time_utc)-1); out->time_utc[sizeof(out->time_utc)-1]=0; }
        if (f_date && *f_date) { strncpy(out->date_utc, f_date, sizeof(out->date_utc)-1); out->date_utc[sizeof(out->date_utc)-1]=0; }

        out->lat_deg    = parse_lat_ddmm(f_lat, f_ns);
        out->lon_deg    = parse_lon_dddmm(f_lon, f_ew);
        if (f_spd && *f_spd)  out->speed_kn   = atof(f_spd);
        if (f_cog && *f_cog)  out->course_deg = atof(f_cog);
        return;
    }

    if (is_GGA) {
        /* GGA indices:
           0 time, 1 lat, 2 N/S, 3 lon, 4 E/W, 5 fix, 6 sats, 7 hdop, 8 alt, 9 'M' */
        char *f_time = (nf > 0) ? fields[0] : NULL;
        char *f_lat  = (nf > 1) ? fields[1] : NULL;
        char *f_ns   = (nf > 2) ? fields[2] : NULL;
        char *f_lon  = (nf > 3) ? fields[3] : NULL;
        char *f_ew   = (nf > 4) ? fields[4] : NULL;
        char *f_fix  = (nf > 5) ? fields[5] : NULL;
        char *f_sats = (nf > 6) ? fields[6] : NULL;
        char *f_hdop = (nf > 7) ? fields[7] : NULL;
        char *f_alt  = (nf > 8) ? fields[8] : NULL;
        char *f_altu = (nf > 9) ? fields[9] : NULL;

        (void)f_time; /* unused but harmless */
        out->lat_deg     = parse_lat_ddmm(f_lat, f_ns);
        out->lon_deg     = parse_lon_dddmm(f_lon, f_ew);
        if (f_fix  && *f_fix)  out->fix_quality = atoi(f_fix);
        if (f_sats && *f_sats) out->sats        = atoi(f_sats);
        if (f_hdop && *f_hdop) out->hdop        = atof(f_hdop);
        if (f_alt  && f_altu && (*f_altu=='M' || *f_altu=='m')) out->alt_m = atof(f_alt);
        return;
    }

    if (is_ZDA) {
        /* ZDA indices: 0 time, 1 day, 2 month, 3 year */
        char *f_day  = (nf > 1) ? fields[1] : NULL;
        char *f_mon  = (nf > 2) ? fields[2] : NULL;
        char *f_year = (nf > 3) ? fields[3] : NULL;

        if (f_day  && *f_day)  out->zda_day  = atoi(f_day);
        if (f_mon  && *f_mon)  out->zda_mon  = atoi(f_mon);
        if (f_year && *f_year) out->zda_year = atoi(f_year);
        return;
    }
}

/* Compose parsed JSON-like line into dst; returns number of bytes */
static uint16_t format_summary(uint8_t* dst, uint16_t dst_cap, const gps_summary* g)
{
    if (dst_cap < 32) return 0;

    char iso_date[11] = ""; /* YYYY-MM-DD */
    char iso_time[13] = ""; /* HH:MM:SS(.sss) */

    if (g->zda_year > 0 && g->zda_mon > 0 && g->zda_day > 0) {
        (void)snprintf(iso_date, sizeof(iso_date), "%04d-%02d-%02d",
                       g->zda_year, g->zda_mon, g->zda_day);
    } else if (g->date_utc[0]) {
        int dd = (g->date_utc[0]-'0')*10 + (g->date_utc[1]-'0');
        int mm = (g->date_utc[2]-'0')*10 + (g->date_utc[3]-'0');
        int yy = (g->date_utc[4]-'0')*10 + (g->date_utc[5]-'0');
        (void)snprintf(iso_date, sizeof(iso_date), "20%02d-%02d-%02d", yy, mm, dd);
    }

    if (g->time_utc[0]) {
        char h[3] = "", m[3] = "", s[7] = "";
        size_t L = strlen(g->time_utc);
        if (L >= 6) {
            h[0]=g->time_utc[0]; h[1]=g->time_utc[1];
            m[0]=g->time_utc[2]; m[1]=g->time_utc[3];
            s[0]=g->time_utc[4]; s[1]=g->time_utc[5];
            if (L > 6) strncpy(&s[2], &g->time_utc[6], sizeof(s)-3);
            (void)snprintf(iso_time, sizeof(iso_time), "%s:%s:%s", h, m, s);
        }
    }

    uint16_t cur = 0;
    cur = append_str(dst, cur, dst_cap, "{");

    cur = append_str(dst, cur, dst_cap, "\"valid\":");
    cur = append_str(dst, cur, dst_cap,
        (g->rmc_valid && g->fix_quality > 0) ? "true" : "false");
    cur = append_str(dst, cur, dst_cap, ",");

    cur = append_str(dst, cur, dst_cap, "\"fix\":");
    cur = append_int32(dst, cur, dst_cap, g->fix_quality);
    cur = append_str(dst, cur, dst_cap, ",");

    cur = append_str(dst, cur, dst_cap, "\"lat\":");
    cur = append_ftoa_fixed(dst, cur, dst_cap, g->lat_deg, 8);
    cur = append_str(dst, cur, dst_cap, ",");

    cur = append_str(dst, cur, dst_cap, "\"lon\":");
    cur = append_ftoa_fixed(dst, cur, dst_cap, g->lon_deg, 8);
    cur = append_str(dst, cur, dst_cap, ",");

    cur = append_str(dst, cur, dst_cap, "\"sats\":");
    cur = append_int32(dst, cur, dst_cap, g->sats);
    cur = append_str(dst, cur, dst_cap, ",");

    cur = append_str(dst, cur, dst_cap, "\"hdop\":");
    cur = append_ftoa_fixed(dst, cur, dst_cap, g->hdop, 2);
    cur = append_str(dst, cur, dst_cap, ",");

    cur = append_str(dst, cur, dst_cap, "\"alt_m\":");
    cur = append_ftoa_fixed(dst, cur, dst_cap, g->alt_m, 2);
    cur = append_str(dst, cur, dst_cap, ",");

    cur = append_str(dst, cur, dst_cap, "\"speed_kn\":");
    cur = append_ftoa_fixed(dst, cur, dst_cap, g->speed_kn, 2);
    cur = append_str(dst, cur, dst_cap, ",");

    cur = append_str(dst, cur, dst_cap, "\"course_deg\":");
    cur = append_ftoa_fixed(dst, cur, dst_cap, g->course_deg, 2);
    cur = append_str(dst, cur, dst_cap, ",");

    cur = append_str(dst, cur, dst_cap, "\"date\":\"");
    cur = append_str(dst, cur, dst_cap, iso_date);
    cur = append_str(dst, cur, dst_cap, "\",");

    cur = append_str(dst, cur, dst_cap, "\"time\":\"");
    cur = append_str(dst, cur, dst_cap, iso_time);
    cur = append_str(dst, cur, dst_cap, "\"}");

    cur = append_str(dst, cur, dst_cap, "\n");
    return cur;
}

/* =========================
   Public API
   ========================= */

uint8_t* get_gps_data(gps_data* gps, uint16_t* out_size_ref)
{
    __DMB();

    uint8_t*  buf = (gps->head == 1) ? gps->first  : gps->second;
    uint16_t* psz = (gps->head == 1) ? &gps->first_size : &gps->second_size;

    char local[512];
    uint16_t in_len = *psz;
    if (in_len > sizeof(local)-1) in_len = sizeof(local)-1;
    memcpy(local, buf, in_len);
    local[in_len] = '\0';

    gps_summary agg;
    summary_init(&agg);

    char* p = local;
    while (*p) {
        char* line = p;
        char* nl = strpbrk(p, "\r\n");
        size_t ln = nl ? (size_t)(nl - p) : strlen(p);

        char save = p[ln];
        p[ln] = '\0';

        ln = rstrip_crlf(line, ln);
        if (ln >= 1 && line[0] == '$')
            parse_sentence(line, &agg);

        p[ln] = save;
        p = nl ? (nl + 1) : (p + ln);
        while (*p == '\r' || *p == '\n') ++p;
    }

    uint16_t out_len = format_summary(buf, 512, &agg);
    *psz = out_len;
    *out_size_ref = out_len;
    return buf;
}

/* =========================
   DMA-to-Idle RX callback
   ========================= */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart != &huart3) return;

    gps_version++;

    if (gps_ring.head == 0) {
        memcpy(gps_ring.first, gps_dma_rx, size);
        __DMB();
        gps_ring.head = 1;
        gps_ring.first_size = size;
    } else {
        memcpy(gps_ring.second, gps_dma_rx, size);
        __DMB();
        gps_ring.head = 0;
        gps_ring.second_size = size;
    }

    __DMB();
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, gps_dma_rx, sizeof(gps_dma_rx));
}
