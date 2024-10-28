//
// Created by User on 2024/1/29.
//
#include "rtklib.h"
#include "sbf_mt.h"
#include "gflib.h"
#define MAXMS   40
#define MAXLEN  13515   /* 255*53 */
#define PAGECNT 53

#define CSSR_TYPE_MASK      1   /* Mask id */
#define CSSR_TYPE_OC        2   /* Orbit */
#define CSSR_TYPE_CC        3   /* Clock correction */
#define CSSR_TYPE_CB        4   /* Code bias */
#define CSSR_TYPE_PB        5   /* Phase bias */
#define CSSR_TYPE_SIGMASK   6   /* Signal mask */

#define CSSR_CTYPE_MASK     0
#define CSSR_CTYPE_OC       1
#define CSSR_CTYPE_CC       2
#define CSSR_CTYPE_CB       3
#define CSSR_CTYPE_PB       4

#define MAX(x,y)    ((x)>(y)?(x):(y))
#define INVALID_VALUE -10000

static int mid_ = -1;
static int  ms_ = -1;
static int rec[MAXMS]={0}, rcnt = 0; /* rec exists bug */
static int has_cnt = 0;
static int mid_decoded[MAXMS] = {0}, mcnt = 0;
static int icnt = 0;

extern const uint8_t RS_matrix[255][32];

const uint8_t gnsstbl[6]={
        SYS_GPS,SYS_GLO,SYS_GAL,SYS_CMP,SYS_QZS,SYS_SBS};

/* SSR signal and tracking mode IDs ------------------------------------------*/
static const uint8_t ssr_sig_gps[16]={
        CODE_L1C,0,0,CODE_L1S,CODE_L1L,CODE_L1X,CODE_L2S,CODE_L2L,
        CODE_L2X,CODE_L2P,0,CODE_L5I, CODE_L5Q, CODE_L5X,0, 0};

static const uint8_t ssr_sig_gal[16]={
        CODE_L1B,CODE_L1C,CODE_L1X, CODE_L5I, CODE_L5Q,CODE_L5X,CODE_L7I,
        CODE_L7Q, CODE_L7X,CODE_L8I,CODE_L8Q,CODE_L8X,CODE_L6B,CODE_L6C,CODE_L6X, 0};

//int *gf_invert_matrix(int *mat, int rows);
//extern void gMatClose(void) {
//    if (fp_gMat) fclose(fp_gMat);
//    fp_gMat = NULL;
//}

static fatalfunc_t *fatalfunc=NULL; /* fatal callback function */

/* fatal error ---------------------------------------------------------------*/
static void fatalerr(const char *format, ...)
{
    char msg[1024];
    va_list ap;
    va_start(ap,format); vsprintf(msg,format,ap); va_end(ap);
    if (fatalfunc) fatalfunc(msg);
    else fprintf(stderr,"%s",msg);
    exit(-9);
}

static void print_stat(GAL_H *gal_h) {
    /* Print the status of MT header */
    double tow = time2gpst(gal_h->time, NULL);
    if (gal_h->disp_has) {
        printf("HAS MT = %d, TOW = %.1f, IOD_SSR = %d\n", gal_h->gal_msg->subtype, tow, gal_h->iod_ssr);
    }
}

static void print_mask_stat(int nsat, int *prn, int disp) {
    /* Print the status of MT1: mask */
    if (disp) {
        for (int i = 0; i < nsat; i++) {
            printf("%d ", prn[i]);
            if (i == nsat - 1) {
                printf("\n");
            }
        }
    }
}

static double decode_sval(unsigned char *buff, int i, int n, double lsb)
{
    int slim=-((1<<(n-1))-1)-1,v;
    v = getbits(buff, i, n);
    return (v==slim) ? INVALID_VALUE:(double)v*lsb;
}

static int findID(int *page, int id, int cnt) {
    int find = 0;
    for (int j = 0; j < cnt; j++) {
        if (page[j] == id) {
            find = 1;
            break;
        }
    }
    return find;
}

static void md2buff(int *md, uint8_t *buff) {
    int idx = 0;
    for (int i = 0; i < rcnt; i++) {
        for (int j = 0; j < PAGECNT; j++) {
            buff[idx] = (uint8_t)md[i+j*rcnt];
            if (md[i+j*rcnt] > 255 || md[i+j*rcnt] < 0) {
                fprintf(stderr, "GF code overflow!\n");
                exit(-1);
            }
            idx++;
        }
    }
}

static int decode_head(GAL_H *gal_h, uint8_t *buff, int *i, int msgType) {
    int ui;
    int tow0 = gal_h->gal_msg->tow0;
    int toh = gal_h->toh;
    int week = gal_h->gal_msg->week;
    if (msgType == CSSR_TYPE_MASK) {
        ui = 0;
    }
    else {
        ui      = getbitu(buff, *i, 4); *i += 4;
    }
    if (tow0 >= 0) {
        gal_h->gal_msg->tow = tow0 + toh;
        if (week >= 0) {
            gal_h->time = gpst2time(week, gal_h->gal_msg->tow);
        }
    }
    return gal_h->iod_ssr;
}

static uint16_t sys2gnssid(int sys) {
    uint16_t gnss_id;
    if (sys == SYS_GPS) {
        gnss_id = 0;
    }
    else if (sys == SYS_GAL) {
        gnss_id = 2;
    }
    else {
        gnss_id = -1;
    }
    return gnss_id;
}

static int gnss2sys(uint16_t gnss_id) {
    int sys;
    if (gnss_id > 5) {
        printf("Invalid GNSS ID: %d\n", gnss_id);
        return 0;
    }
    sys = gnsstbl[gnss_id];
    return sys;
}

static int decode_mask(GAL_H *gal_h, uint64_t mask, int *list, int sys, const int len, int msgType) {
    int n = 0;
    int idx = 0;
    int ofst = msgType==CSSR_TYPE_MASK?1:0;
    for (int i = 0; i < len; i++) {
        if ((mask >> (len- 1 - i)) &1) {
            list[n] = i + ofst;
            if (ofst) { /* Satellite mask */
                idx = satno(sys, list[n]);
                gal_h->gal_msg->sat_mask[idx - 1] = 1;
            }
            else {  /* Signal mask */
                uint8_t *codes = NULL;
                switch (sys) {
                    case SYS_GPS: codes = (uint8_t *)ssr_sig_gps; break;
                    case SYS_GAL: codes = (uint8_t *)ssr_sig_gal; break;
                }
                gal_h->gal_cssr->sig_n[sys==SYS_GPS?0:1][codes[i]] = 1;
            }
            n++;
        }
    }
    return n;
}

static void decode_sat_orb(GAL_H *gal_h, uint8_t *buff, int *i, int prn_idx) {
    int prn;
    int sys = satsys(prn_idx + 1, &prn);
    int n;
    uint16_t  iode;
    double dx, dy, dz;
    if (sys==SYS_GPS) n = 8;
    else if (sys==SYS_GAL) n = 10;

    iode            = getbitu(buff, *i, n);               *i+=  n;
    dx              = decode_sval(buff, *i, 13, 0.0025);    *i+= 13;
    dy              = decode_sval(buff, *i, 12, 0.0080);    *i+= 12;
    dz              = decode_sval(buff, *i, 12, 0.0080);    *i+= 12;

    gal_h->gal_msg->iode[prn_idx]       = iode;
    /* Must * (-1.0)? */
    gal_h->gal_msg->deph[prn_idx][0]    = -1.0 * dx;
    gal_h->gal_msg->deph[prn_idx][1]    = -1.0 * dy;
    gal_h->gal_msg->deph[prn_idx][2]    = -1.0 * dz;

    if (gal_h->disp_has) {
        if (prn_idx < NSATGPS) {
            printf("G%02d, dx = %f, dy = %f, dz = %f, iode = %d\n",
                   prn, dx, dy, dz, iode);
        }
        else {
            printf("E%02d, dx = %f, dy = %f, dz = %f, iode = %d\n",
                   prn, dx, dy, dz, iode);
        }
    }
}

static void decode_sat_cbias(GAL_H *gal_h, uint8_t *buff, int *i, int prn_idx) {
    double cb;
    for (int k = 0; k < MAXCODE; k++) {
        if (gal_h->gal_cssr->svsig[prn_idx][k] == 1) {
            cb  = decode_sval(buff, *i, 11, 0.02);      *i+= 11;
            gal_h->gal_msg->cbias[prn_idx][k] = cb;
        }
    }
}

static void decode_sat_pbias(GAL_H *gal_h, uint8_t *buff, int *i, int prn_idx) {
    double pb;
    int pdi;
    for (int k = 0; k < MAXCODE; k++) {
        if (gal_h->gal_cssr->svsig[prn_idx][k] == 1) {
            pb  = decode_sval(buff, *i, 11, 0.01);      *i+= 11;
            pdi = getbitu(buff, *i, 2);               *i+=  2;
            gal_h->gal_msg->pbias[prn_idx][k] = pb;
            gal_h->gal_cssr->pdi[prn_idx][k]  = pdi;
        }
    }
}

static void decode_sat_clk(GAL_H *gal_h, uint8_t *buff, int *i, int prn_idx, double *dcm) {
    double dclk;
    int prn;
    int sys = satsys(prn_idx + 1, &prn);
    if (dcm == NULL) {
        *dcm = 1.0;
    }
    dclk    = decode_sval(buff, *i, 13, 0.0025);      *i+= 13;
    dclk   *= (*dcm);

    gal_h->gal_msg->dclk[prn_idx][0] = dclk;

    if (gal_h->disp_has) {
        if (prn_idx < NSATGPS) {
            printf("G%02d, dclk = %f\n",
                   prn, dclk);
        }
        else {
            printf("E%02d, dclk = %f\n",
                   prn, dclk);
        }
    }
}

static void decode_cssr_mask(GAL_H *gal_h, uint8_t *buff, int *i) {
    int iod_ssr = decode_head(gal_h, buff, i, CSSR_TYPE_MASK);
    int sys, nsat, nsig, nsig_s, prn_idx, sig_s_idx, j;
//    int satlist[40]={0};
//    int siglist[16]={0}, siglist_s[16]={0};
    uint16_t ngnss, gnss_id, sigM, sigM_s, cmaf;
    uint64_t satM;
    uint8_t *codes = NULL;

    ngnss                       = getbitu(buff, *i, 4); *i += 4;

    gal_h->ngnss                = ngnss;
    gal_h->iod_ssr              = iod_ssr;
    gal_h->gal_msg->nsat_n      = 0;
    cssrmsg_init(gal_h->gal_cssr);
    for (j = 0; j < MAXSAT; j++) {
        gal_h->gal_msg->sat_mask[j] = 0;
    }
    print_stat(gal_h);

    for (j = 0; j < ngnss; j++) {
        int satlist[40]={0};
        int siglist[16]={0}, siglist_s[16]={0};
        gnss_id     = getbitu(buff, *i, 4);                 *i  +=  4;
        satM        = (uint64_t)getbitu(buff,*i, 8)<<32;    *i  +=  8;
        satM        |= (uint64_t)getbitu(buff,*i, 32);      *i  += 32;
        sigM        = getbitu(buff, *i, 16);                *i  += 16;
        cmaf        = getbitu(buff, *i, 1);                 *i  +=  1;
        gal_h->gal_cssr->gnss_idx[j] = gnss_id;

        sys         = gnss2sys(gnss_id);
        nsat        = decode_mask(gal_h, satM, satlist, sys, 40, CSSR_TYPE_MASK);
        nsig        = decode_mask(gal_h, sigM, siglist, sys, 16, CSSR_TYPE_SIGMASK);

        gal_h->gal_cssr->nsat_g[gnss_id]    = nsat;
        gal_h->gal_msg->nsat_n             += nsat;
        gal_h->gal_cssr->nsig_max           = MAX(gal_h->gal_cssr->nsig_max, nsig);

        switch(sys) {
            case SYS_GPS: codes = (uint8_t *)ssr_sig_gps; break;
            case SYS_GAL: codes = (uint8_t *)ssr_sig_gal; break;
        }
        for (int k = 0; k < nsat; k++) {
            if (sys == SYS_QZS) {
                satlist[k] += 192;
            }
            prn_idx = satno(sys, satlist[k]);
            gal_h->gal_cssr->sat_n[prn_idx - 1]     = satlist[k];
            gal_h->gal_cssr->sys_n[prn_idx - 1]     = sys;
            gal_h->gal_cssr->gnss_n[prn_idx - 1]    = gnss_id;
            if (cmaf == 1) {
                /* Subset */
                sigM_s          = getbitu(buff, *i, nsig);  *i += nsig;
                nsig_s          = decode_mask(gal_h, sigM_s, siglist_s, sys, nsig, CSSR_TYPE_SIGMASK);
                for (int l = 0; l < nsig_s; l++) {
                    sig_s_idx = siglist[siglist_s[l]];
                    gal_h->gal_cssr->svsig[prn_idx - 1][codes[sig_s_idx]] = 1;
                }
                gal_h->gal_cssr->nsig_n[prn_idx - 1]    = nsig_s;
                gal_h->gal_cssr->nsig_total += nsig_s;
            }
            else {
                for (int l = 0; l < nsig; l++) {
                    gal_h->gal_cssr->svsig[prn_idx - 1][codes[siglist[l]]] = 1;
                }
                gal_h->gal_cssr->nsig_total += nsig;
                gal_h->gal_cssr->nsig_n[prn_idx - 1]    = nsig;
            }
        }
        gal_h->gal_cssr->nm_idx[gnss_id] = getbitu(buff, *i, 3); *i += 3;
    }

    *i += 6;
    gal_h->gal_msg->cstat |= (1 << CSSR_CTYPE_MASK);
    gal_h->gal_msg->t0[CSSR_CTYPE_MASK] = gal_h->time;
}

static void decode_cssr_orb(GAL_H *gal_h, uint8_t *buff, int *i) {
    int iod_ssr = decode_head(gal_h, buff, i, CSSR_TYPE_OC);
    print_stat(gal_h);

    for (int j = 0; j < MAXSAT; j++) {
        gal_h->gal_msg->iode[j] = 0;
        gal_h->gal_msg->deph[j][0] = gal_h->gal_msg->deph[j][1] = gal_h->gal_msg->deph[j][2] = 0.0;
        if (gal_h->gal_msg->sat_mask[j] == 1) {
            decode_sat_orb(gal_h, buff, i, j);
        }
    }

    gal_h->gal_msg->iod_ssr_c[CSSR_CTYPE_OC] = gal_h->iod_ssr;
    gal_h->gal_msg->cstat |= (1 << CSSR_CTYPE_OC);
    gal_h->gal_msg->t0[CSSR_CTYPE_OC] = gal_h->time;
}

static void decode_cssr_clk(GAL_H *gal_h, uint8_t *buff, int *i) {
    int iod_ssr = decode_head(gal_h, buff, i, CSSR_TYPE_CC);
    int j, gnss_id, sys;
    uint16_t gnss_idx;
    double dcm;
    print_stat(gal_h);

    if ((gal_h->gal_msg->cstat&(1 << CSSR_CTYPE_MASK))!=(1 << CSSR_CTYPE_MASK)) {
        return;
    }

    for (j = 0; j < gal_h->ngnss; j++) {
        gnss_id = gal_h->gal_cssr->gnss_idx[j];
        dcm             = getbitu(buff, *i, 2);     *i += 2;
        dcm            += 1.0;
        gal_h->gal_cssr->dcm[gnss_id]   = dcm;
    }

    for (j = 0; j < MAXSAT; j++) {
        gal_h->gal_msg->dclk[j][0] = gal_h->gal_msg->dclk[j][1] = gal_h->gal_msg->dclk[j][2] = 0.0;
        if (gal_h->gal_msg->sat_mask[j] == 1) {
            sys = satsys(j + 1, NULL);
            gnss_idx = sys2gnssid(sys);
            decode_sat_clk(gal_h, buff, i, j, &gal_h->gal_cssr->dcm[gnss_idx]);
        }
    }

    gal_h->gal_msg->iod_ssr_c[CSSR_CTYPE_CC] = iod_ssr;
    gal_h->gal_msg->cstat |= (1 << CSSR_CTYPE_CC);
    gal_h->gal_msg->t0[CSSR_CTYPE_CC] = gal_h->time;
}

static void decode_cssr_subclk(GAL_H *gal_h, uint8_t *buff, int *i) {
    int iod_ssr = decode_head(gal_h, buff, i, CSSR_TYPE_CC);

}

static void decode_cssr_cbias(GAL_H *gal_h, uint8_t *buff, int *i) {
    int iod_ssr = decode_head(gal_h, buff, i, CSSR_TYPE_CB);
    print_stat(gal_h);

    for (int j = 0; j < MAXSAT; j++) {
        for (int k = 0; k < MAXCODE; k++) {
            gal_h->gal_msg->cbias[j][k] = 0.0;
        }
        if (gal_h->gal_cssr->nsig_n[j] > 0) {
            decode_sat_cbias(gal_h, buff, i, j);
        }
    }

    gal_h->gal_msg->iod_ssr_c[CSSR_CTYPE_CB] = gal_h->iod_ssr;
    gal_h->gal_msg->cstat |= (1 << CSSR_CTYPE_CB);
    gal_h->gal_msg->t0[CSSR_CTYPE_CB] = gal_h->time;
}

static void decode_cssr_pbias(GAL_H *gal_h, uint8_t *buff, int *i) {
    int iod_ssr = decode_head(gal_h, buff, i, CSSR_TYPE_PB);
    print_stat(gal_h);

    for (int j = 0; j < MAXSAT; j++) {
        for (int k = 0; k < MAXCODE; k++) {
            gal_h->gal_msg->pbias[j][k] = 0.0;
            gal_h->gal_cssr->pdi[j][k] = -1;
        }
        if (gal_h->gal_cssr->nsig_n[j] > 0) {
            decode_sat_pbias(gal_h, buff, i, j);
        }
    }

    gal_h->gal_msg->iod_ssr_c[CSSR_CTYPE_PB] = gal_h->iod_ssr;
    gal_h->gal_msg->cstat |= (1 << CSSR_CTYPE_PB);
    gal_h->gal_msg->t0[CSSR_CTYPE_PB] = gal_h->time;
}

static void decode_has_msg(GAL_H *gal_h, uint8_t *buff) {
    if (gal_h->mt != 1) {
        printf("Invalid message type: %d\n", gal_h->mt);
        return;
    }
    uint32_t toh, flag, res, mask, iod_ssr;
    int i = 0;
    toh         = getbitu(buff, i, 12); i   += 12;
    flag        = getbitu(buff, i, 6);  i   +=  6;
    res         = getbitu(buff, i, 4);  i   +=  4;
    mask        = getbitu(buff, i, 5);  i   +=  5;
    iod_ssr     = getbitu(buff, i, 5);  i   +=  5;

    gal_h->toh      = toh;
    gal_h->iod_ssr  = iod_ssr;

    if (toh >= 3600) {
        printf("Invalid time of hour: %d\n", toh);
        return;
    }
    if ((flag >> 5) &1) {   /* Mask block */
        gal_h->gal_msg->subtype = CSSR_TYPE_MASK;
        gal_h->mask_id          = mask;
        decode_cssr_mask(gal_h, buff, &i);
    }
    if ((flag >> 4) &1) {   /* Orbit correction block */
        gal_h->gal_msg->subtype = CSSR_TYPE_OC;
        decode_cssr_orb(gal_h, buff, &i);
    }
    if ((flag >> 3) &1) {   /* Clock full-set correction block */
        gal_h->gal_msg->subtype = CSSR_TYPE_CC;
        gal_h->mask_clkid       = mask;
        decode_cssr_clk(gal_h, buff, &i);
    }
    if ((flag >> 2) &1) {   /* Clock subset correction block */
        decode_cssr_subclk(gal_h, buff, &i);
    }
    if ((flag >> 1) &1) {   /* Code bias block */
        gal_h->gal_msg->subtype = CSSR_TYPE_CB;
        decode_cssr_cbias(gal_h, buff, &i);
    }
    if ((flag >> 0) &1) {   /* Phase bias block */
        gal_h->gal_msg->subtype = CSSR_TYPE_PB;
        decode_cssr_pbias(gal_h, buff, &i);
    }
}

static void decode_has_page(GAL_H *gal_h, uint8_t *buff, int *rec, int m) {
    int i, j, k, r_idx;
    int *Dinv, *Md;
    Dinv    = imat(rcnt, rcnt);
    Md      = imat(PAGECNT, rcnt);
//    uint8_t buff[MAXLEN]={0};
    if (rcnt >= m) {
        int *D      = imat(rcnt, rcnt);
        int *wd     = imat(PAGECNT, rcnt);

        for (i = 0; i < rcnt; i++) {
            r_idx = rec[i];
            for (j = 0; j < PAGECNT; j++) {
                wd[j*rcnt + i] = gal_h->has_page[r_idx + j * 255];
            }
            for (k = 0; k < rcnt; k++) {
                D[k + i * rcnt] = (int)RS_matrix[r_idx][k]; /* Need to implement inverse of GF(256) */
            }
        }
        Dinv    = gf_invert_matrix(D, rcnt); /* inverse index */
        Md      = gf_matrix_multiply(wd,Dinv, PAGECNT, rcnt);
        md2buff(Md, buff);
    }
    free(Dinv);
    free(Md);
}

extern void decoder_has(SBF_Buffer *sbf_data) {
    int j;
    uint8_t buff[MAXLEN]={0};
    if (rcnt >= ms_) {

        decode_has_page(sbf_data->gal_h, buff, rec, ms_);
        decode_has_msg(sbf_data->gal_h, buff);

        mid_decoded[mcnt] = mid_;
        mcnt += 1;
        mid_ = -1;
        for (j = 0; j < MAXMS; j++) rec[j] = 0;
        rcnt = 0;
        if (mcnt > 10) {    /* mcnt check re-initialization */
            int tmp[MAXMS];
            memcpy(tmp,mid_decoded,sizeof(int)*MAXMS);
            for (j = 0; j < MAXMS-1; j++) {
                if (j < 10) {
                    mid_decoded[j] = tmp[j + 1];
                }
                else mid_decoded[j] = 0;
            }
            mcnt = 10;
        }
    }
    else {
        /* May have bug 20240208 */
        icnt += 1;
        if ((icnt >  2*ms_) && mid_ != -1) {
            icnt = 0;
            mid_ = -1;
            for (j = 0; j < MAXMS; j++) rec[j] = 0;
            rcnt = 0;
        }
    }

}

extern void decode_galmsgE6b(SBF_Buffer *sbf_data, uint8_t *buff, int startdecode) {
    int i = 14, j;
    uint32_t dummy_page       = getbitu(buff, i, 24);
    if (dummy_page == 11484099) { /* Hex: 0xaf3bc3 = dummy page */
        return;
    }
    uint32_t hass, res;
    hass                = getbitu(buff, i, 2);  i += 2;
    res                 = getbitu(buff, i, 2);  i += 2;
    if (hass >= 2) {
        return;
    }
    int mt, mid, ms, pid;
    uint32_t page[PAGECNT] = {0};
    mt                 = getbitu(buff, i, 2);  i += 2; /* Message type */
    mid                = getbitu(buff, i, 5);  i += 5; /* ID of the transmitted HAS Encoded Page */
    ms                 = getbitu(buff, i, 5);  i += 5; /* Size of the non-encoded message */
    ms += 1;
    pid                = getbitu(buff, i, 8);  i += 8; /* ID of the transmitted HAS Encoded Page */
    pid -= 1;
    sbf_data->gal_h->mt = mt;

    if (mid_ == -1 && !findID(mid_decoded, mid, mcnt)) {
        mid_ = mid;
        ms_  = ms;
    }
    if (mid == mid_ && !findID(rec, pid, rcnt)) {
        for (j = 0; j < PAGECNT; j++) {
            page[j] = getbitu(buff, i, 8); i += 8;
            sbf_data->gal_h->has_page[pid + j * 255] = page[j];
        }
        rec[rcnt] = pid;
        rcnt++;         /* Might have bug, check size */
    }
}