#ifndef PKSDR_MT_H
#define PKSDR_MT_H

#include <stdint.h>
#include <stdio.h>
#include "rtklib.h"
#include "sbf_mt.h"

#define MAXRAWLEN   16384                   /* max length of receiver raw message */
/* cssr */
#define CSSR_MAX_GNSS     16
#define CSSR_MAX_SV_GNSS  40
#define CSSR_MAX_SV       64
#define CSSR_MAX_SIG      16
#define CSSR_MAX_CELLMASK 64
#define CSSR_MAX_NET      32
#define CSSR_MAX_LOCAL_SV 32
#define CSSR_MAX_GP       128
#define CSSR_MAX_NETWORK  32

/* QZS message header */
typedef struct {
    gtime_t         ssr_tow;                /* GPS time of week */
    gtime_t         time;                   /* SBF message time */
    int             prn_ref;                /* Reference satellite prn */
    int             iod;                    /* IOD (epoch) */
    int             iod_ssr;                /* IOD of SSR */
    int             iod_ssr_p;              /* Previous IOD of SSR */
    int             facility_p;             /* Facility in MADOCA */
    int             fcnt;                   /* Frame count */
    int             ngnss;                  /* Number of augmented GNSS */
    SSR_MSG         *qzs_msg;               /* Message content */
    CSSR_MSG        *qzs_cssr;              /* CSSR message */
    int             disp_mdc;               /* display message */
    int             log_mdc;                /* log madoca messages */
}QZS_H;

typedef struct {
    gtime_t         ssr_tow;                /* L6E time */
    QZS_H           *qzs_h;                 /* QZS message header */
    uint8_t         buff[MAXRAWLEN];        /* message buffer */
    double          tow;                    /* time of week */
    int             week;                   /* GPS week */
}PKSDR_Buffer;

extern void decode_qzsmsgL6e(PKSDR_Buffer *pksdr_data);

#endif // PKSDR_MT_H
