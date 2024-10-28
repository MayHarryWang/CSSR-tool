#ifndef SBF_MT_H
#define SBF_MT_H

#include <stdint.h>
#include <stdio.h>
#include "rtklib.h"
#include "cssr.h"

#ifdef WIN_DLL
#define EXPORT __declspec(dllexport) /* for Windows DLL */
#else
#define EXPORT
#endif

#define MAXRAWLEN   16384                   /* max length of receiver raw message */
#define MAXSYS      16

typedef struct {
    int nsig_total, nsig_max;               /* Total signal */
    int nm_idx[MAXSYS];                     /* Navigation message index */
    double dcm[MAXSYS];                     /* Delta clock multiplier */
    int nsat_g[MAXSYS];                     /* Number of SAT for each GNSS ID */
    int sys_n[MAXSAT], sat_n[MAXSAT];       /* GNSS prn and sys for each sat  */
    int sat_n_p[MAXSAT];                    /* Previous GNSS prn */
    int gnss_n[MAXSAT];                     /* GNSS ID for each sat */
    int nsig_n[MAXSAT];                     /* Number of signal code (signal mask) */
    int sig_n[4][MAXCODE];                  /* List of signal code (signal mask) 0: GPS, 1: Galileo */
    int sig_n_p[4][MAXCODE];                /* Previous list of signal code (signal mask) */
    int svsig[MAXSAT][MAXCODE];             /* Signal mask with satellite index */
    int pdi[MAXSAT][MAXCODE];               /* Phase discontinuity indicator */
    int gnss_idx[16];                       /* GNSS index: 0 for GPS, 2 for Galileo */
}CSSR_MSG;

typedef struct {
    int week, tow;                          /* Week and time of week */
    int tow0;                               /* TOW of beginning of the day */
    int subtype;                            /* Message type (MT) */
    int nsat_n;                             /* Number of total masked satellite */
    int nsat_bds, nsat_gps;                 /* Number of masked bds, gps */
    int nsat_gal, nsat_glo;                 /* Number of masked gal, glo */
    int iodp, iodp_p;                       /* IODP and previous IODP */
    int cstat;                              /* Status of CSSR */
    int sat_mask[MAXSAT];                   /* All satellite mask */
    int iode[MAXSAT];                       /* IODE */
    int iod_ssr_c[7];                       /* IOD of SSR of message */
    int iodc_orb[MAXSAT];                   /* IODC of orbit */
    int iodc_clk[MAXSAT];                   /* IODC of clock */
    int iodc_clk_p[MAXSAT];                 /* Previous IODC of clock */
    double cbias[MAXSAT][MAXCODE];          /* Code bias */
    double pbias[MAXSAT][MAXCODE];          /* Phase bias */
    double ura[MAXSAT];                     /* URA information */
    double dclk[MAXSAT][3];                 /* Clock correction (C0, C1, C2) */
    double dclk_p[MAXSAT][3];               /* Previous clock correction (C0, C1, C2) */
    double deph[MAXSAT][3];                 /* Orbit correction */
    gtime_t t0[8];                          /* Received time of message */
}SSR_MSG;

/* Galileo message header */
typedef struct {
    SSR_MSG         *gal_msg;               /* Message content */
    CSSR_MSG        *gal_cssr;              /* CSSR message */
    gtime_t         time;                   /* SBF message time */
    int             mt;                     /* Message type */
    int             mid;                    /* ID of the message */
    int             ms;                     /* Size of the non-encoded message, in number of pages */
    int             pid;                    /* ID of the transmitted HAS Encoded Page */
    int             *has_page;              /* Galileo HAS pages */
    int             mask_id;                /* Indicate GPS or Galileo */
    int             mask_clkid;             /* Clock bias correction mask */
    int             toh;                    /* Time of hour */
    int             iod_ssr;                /* IOD of SSR */
    int             ngnss;                  /* Number of augmented GNSS */
//    uint8_t         has_msg[8192];          /* message buffer */
    int             disp_has;               /* display has message */
    int             log_has;                /* log has messages */
}GAL_H;

/* BDS message header */
typedef struct {
    gtime_t         ssr_tow;                /* GPS time of week */
    int             prn_ref;                /* Reference satellite prn */
    int             iod;                    /* IOD (epoch), 17 bits */
    int             iod_ssr;                /* IOD of SSR */
    SSR_MSG         *bds_msg;               /* Message content */
    int             disp_b2b;               /* display b2b message */
    int             log_b2b;                /* log b2b messages */
}BDS_H;

/* Septentrio block header */
typedef struct {
    char            sync1;                  /* sync field, 0x24, 0x40 */
    char            sync2;
    uint16_t        CRC;                    /* 16-bit CRC */
    uint16_t        ID;                     /* message ID */
    uint16_t        length;                 /* message length */
}SBF_H;

typedef struct {
    SBF_H           *sbf_h;                 /* Septentrio block header */
    BDS_H           *bds_h;                 /* BDS message header */
    GAL_H           *gal_h;                 /* Galileo message header */
//    SSR_MSG         *ssr_msg;               /* Message content */
    char            head_buff[8];           /* header buffer */
    uint8_t         buff[MAXRAWLEN];        /* message buffer */
    gtime_t         time;                   /* SBF message time */
    gtime_t         ssr_tow;                /* B2b time */
    double          tow;                    /* time of week */
    int             week;                   /* GPS week */
    int             svid;                   /* SVID in sbf format */
    uint8_t         out[2000];              /* Out sbf format buff */
}SBF_Buffer;

EXPORT void sbf_processor(FILE *fp_sbf, FILE *fp_mdc, int BDS_ref, int QZS_ref,
                          int disp_b2b, int disp_has, int disp_mdc,
                          int log_b2b,  int log_has,  int log_mdc);

/* CSSR BDS functions */
EXPORT void decode_bdsmsgB2b(SBF_Buffer *sbf_data, uint8_t *buff);
EXPORT void decode_galmsgE6b(SBF_Buffer *sbf_data, uint8_t *buff, int startdecode);
EXPORT int svid2sat(int svid);

/* CSSR Gal functions */
EXPORT void gMatClose(void);
EXPORT void decoder_has(SBF_Buffer  *sbf_data);
EXPORT void cssrmsg_init(CSSR_MSG *cssr_msg);

EXPORT void galois_inverse(uint8_t *A, int rcnt);
#endif // SBF_MT_H
