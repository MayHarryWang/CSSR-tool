//
// Created by User on 2024/2/22.
//
#include "rtklib.h"
#include "sbf_mt.h"
#include "pksdr_mt.h"

#define CSSR_TYPE_MASK      1   /* Mask id */
#define CSSR_TYPE_OC        2   /* Orbit */
#define CSSR_TYPE_CC        3   /* Clock correction */
#define CSSR_TYPE_CB        4   /* Code bias */
#define CSSR_TYPE_PB        5   /* Phase bias */
#define CSSR_TYPE_URA       7   /* URA */

#define CSSR_TYPE_SIGMASK   12  /* Signal mask */

#define CSSR_CTYPE_MASK     0
#define CSSR_CTYPE_OC       1
#define CSSR_CTYPE_CC       2
#define CSSR_CTYPE_CB       3
#define CSSR_CTYPE_PB       4
#define CSSR_CTYPE_URA      7

#define MAX(x,y)    ((x)>(y)?(x):(y))
#define INVALID_VALUE -10000

static const uint8_t gnsstbl[6]={
        SYS_GPS,SYS_GLO,SYS_GAL,SYS_CMP,SYS_QZS,SYS_SBS};

/* SSR signal and tracking mode IDs ------------------------------------------*/
static const uint8_t ssr_sig_gps[16]={
        CODE_L1C,CODE_L1P,CODE_L1W,CODE_L1S,CODE_L1L,CODE_L1X,CODE_L2S,CODE_L2L,
        CODE_L2X,CODE_L2P,CODE_L2W,CODE_L5I, CODE_L5Q, CODE_L5X,0, 0};

/* Need to check */
static const uint8_t ssr_sig_glo[16]={
        CODE_L1C,CODE_L1P,CODE_L2C,CODE_L2P,CODE_L1C,CODE_L1C,CODE_L1C,CODE_L2C,
        CODE_L2C,CODE_L2C,CODE_L3I,CODE_L3Q,CODE_L3X, 0,0, 0};

static const uint8_t ssr_sig_gal[16]={
        CODE_L1B,CODE_L1C,CODE_L1X, CODE_L5I, CODE_L5Q,CODE_L5X,CODE_L7I,
        CODE_L7Q, CODE_L7X,CODE_L8I,CODE_L8Q,CODE_L8X,CODE_L6B,CODE_L6C,0, 0};

static const uint8_t ssr_sig_qzs[16]={
        CODE_L1C,CODE_L1S,CODE_L1L,CODE_L1X,CODE_L2S,CODE_L2L,CODE_L2X,CODE_L5I,
        CODE_L5Q,CODE_L5X,CODE_L6E,CODE_L6E, CODE_L6E, CODE_L1C,0, 0};

static void print_stat(QZS_H *qzs_h) {
    /* Print the status of MT header */
    double tow = time2gpst(qzs_h->time, NULL);
    if (qzs_h->disp_mdc) {
        printf("MADOCA Subtype = %d, TOW = %.1f, IOD_SSR = %d\n", qzs_h->qzs_msg->subtype, tow, qzs_h->iod_ssr);
    }
    else return;
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

static uint16_t sys2gnssid(int sys) {
    uint16_t gnss_id;
    if (sys == SYS_GPS) {
        gnss_id = 0;
    }
    else if (sys == SYS_GLO) {
        gnss_id = 1;
    }
    else if (sys == SYS_GAL) {
        gnss_id = 2;
    }
    else if (sys == SYS_QZS) {
        gnss_id = 4;
    }
    else {
        gnss_id = -1;
    }
    return gnss_id;
}

static double quality_idx(int ura) {
    double y = 0.0;
    if (ura<= 0) return y;
    if (ura>=63) return 5.4665;
    y = (pow(3.0,(ura>>3)&7) * (1.0 + (ura&7) / 4.0) - 1.0) * 1E-3;
    return y;
}

static void qzscssrmsg_init(CSSR_MSG *cssr_msg) {
    /* Not initializing sat_n_p and sig_n_p */
    cssr_msg->nsig_max = cssr_msg->nsig_total = 0;
    int i, j;
    for (i = 0; i < 16; i++) {
        cssr_msg->gnss_idx[i] = -1;
    }
    for (i = 0; i < MAXSYS; i++) {
        cssr_msg->nm_idx[i] = cssr_msg->nsat_g[i] = 0;
        cssr_msg->dcm[i] = 1.0;
    }
    for (i = 0; i < MAXSAT; i++) {
        cssr_msg->sys_n[i] = cssr_msg->sat_n[i] = cssr_msg->gnss_n[i] = -1;
        cssr_msg->nsig_n[i] = 0;
        for (j = 0; j < MAXCODE; j++) {
            cssr_msg->svsig[i][j] = -1;
            cssr_msg->sig_n[0][j] = cssr_msg->sig_n[1][j] = cssr_msg->sig_n[2][j] = cssr_msg->sig_n[3][j] = -1;
            cssr_msg->pdi[i][j] = -1;
        }
    }
}

static double decode_sval(unsigned char *buff, int i, int n, double lsb)
{
    int slim=-((1<<(n-1))-1)-1,v;
    v = getbits(buff, i, n);
    return (v==slim) ? INVALID_VALUE:(double)v*lsb;
}

static int decode_head(QZS_H *qzs_h, uint8_t *buff, int *i, int msgType) {
    int ui, mi, iodssr;
    if (msgType == CSSR_TYPE_MASK) {
        qzs_h->qzs_msg->tow = getbitu(buff, *i, 20);    *i  += 20;
        qzs_h->qzs_msg->tow0= (qzs_h->qzs_msg->tow / 3600) * 3600;
    }
    else {
        int dtow            = getbitu(buff, *i, 12);    *i  += 12;
        if (qzs_h->qzs_msg->tow >= -1) {
            qzs_h->qzs_msg->tow = qzs_h->qzs_msg->tow0 + dtow;
        }
    }
    if (qzs_h->qzs_msg->week >= 0) {
        qzs_h->time      = gpst2time(qzs_h->qzs_msg->week, qzs_h->qzs_msg->tow);
    }
    ui          = getbitu(buff, *i, 4); *i += 4;
    mi          = getbitu(buff, *i, 1); *i += 1;
    iodssr      = getbitu(buff, *i, 4); *i += 4;

    return iodssr;
}

static int decode_mask(QZS_H *qzs_h, uint64_t mask, int *list, int sys, const int len, int msgType) {
    int n = 0;
    int idx = 0;
    int ofst = msgType==CSSR_TYPE_MASK?1:0;
    int sig_idx = 0;
    for (int i = 0; i < len; i++) {
        if ((mask >> (len- 1 - i)) &1) {
            list[n] = i + ofst;
            if (ofst) { /* Satellite mask */
                if (sys == SYS_QZS) {
                    list[n] += 192;
                }
                idx = satno(sys, list[n]);
                qzs_h->qzs_msg->sat_mask[idx - 1] = 1;
            }
            else {  /* Signal mask */
                uint8_t *codes = NULL;
                switch (sys) {
                    case SYS_GPS: codes = (uint8_t *)ssr_sig_gps; sig_idx = 0; break;
                    case SYS_GAL: codes = (uint8_t *)ssr_sig_gal; sig_idx = 1; break;
                    case SYS_GLO: codes = (uint8_t *)ssr_sig_glo; sig_idx = 2; break;
                    case SYS_QZS: codes = (uint8_t *)ssr_sig_qzs; sig_idx = 3; break;
                }
                qzs_h->qzs_cssr->sig_n[sig_idx][codes[i]] = 1;
            }
            n++;
        }
    }
    return n;
}

static void decode_sat_orb(QZS_H *qzs_h, uint8_t *buff, int *i, int prn_idx) {
    int prn;
    int sys = satsys(prn_idx + 1, &prn);
    int n;
    uint16_t  iode;
    double dx, dy, dz;

    if (sys==SYS_GAL) n = 10;
    else n = 8;

    iode            = getbitu(buff, *i, n);               *i+=  n;
    dx              = decode_sval(buff, *i, 15, 0.0016);    *i+= 15;
    dy              = decode_sval(buff, *i, 13, 0.0064);    *i+= 13;
    dz              = decode_sval(buff, *i, 13, 0.0064);    *i+= 13;

    qzs_h->qzs_msg->iode[prn_idx]       = iode;
    /* Must * (-1.0)? */
    qzs_h->qzs_msg->deph[prn_idx][0]    = dx;
    qzs_h->qzs_msg->deph[prn_idx][1]    = dy;
    qzs_h->qzs_msg->deph[prn_idx][2]    = dz;

    char *c;
    switch(sys) {
        case SYS_GPS: c = "G"; break;
        case SYS_GAL: c = "E"; break;
        case SYS_GLO: c = "R"; break;
        case SYS_QZS: c = "J"; break;
    }
    if (qzs_h->disp_mdc) {
        printf("%s%02d, dx = %f, dy = %f, dz = %f, iode = %d\n",
               c, prn, dx, dy, dz, iode);
    }
}

static void decode_sat_cbias(QZS_H *qzs_h, uint8_t *buff, int *i, int prn_idx) {
    double cb;
    for (int k = 0; k < MAXCODE; k++) {
        if (qzs_h->qzs_cssr->svsig[prn_idx][k] == 1) {
            cb  = decode_sval(buff, *i, 11, 0.02);      *i+= 11;
            qzs_h->qzs_msg->cbias[prn_idx][k] = cb;
        }
    }
}

static void decode_sat_pbias(QZS_H *qzs_h, uint8_t *buff, int *i, int prn_idx) {
    double pb;
    int pdi;
    for (int k = 0; k < MAXCODE; k++) {
        if (qzs_h->qzs_cssr->svsig[prn_idx][k] == 1) {
            pb  = decode_sval(buff, *i, 15, 0.001);       *i+= 15;
            pdi = getbitu(buff, *i, 2);                 *i+=  2;
            qzs_h->qzs_msg->pbias[prn_idx][k] = pb;
            qzs_h->qzs_cssr->pdi[prn_idx][k]  = pdi;
        }
    }
}

static void decode_sat_clk(QZS_H *qzs_h, uint8_t *buff, int *i, int prn_idx) {
    double dclk;
    int prn;
    int sys = satsys(prn_idx + 1, &prn);

    dclk    = decode_sval(buff, *i, 15, 0.0016);      *i+= 15;

    qzs_h->qzs_msg->dclk[prn_idx][0] = dclk;

    char *c;
    switch(sys) {
        case SYS_GPS: c = "G"; break;
        case SYS_GAL: c = "E"; break;
        case SYS_GLO: c = "R"; break;
        case SYS_QZS: c = "J"; break;
    }
    if (qzs_h->disp_mdc) {
        printf("%s%02d, dclk = %f\n",
               c, prn, dclk);
    }
}

static void decode_cssr_mask(QZS_H *qzs_h, uint8_t *buff, int *i) {
    int iod_ssr = decode_head(qzs_h, buff, i, CSSR_TYPE_MASK);
    int sys, nsat, nsig, nsig_s, prn_idx, sig_s_idx, j;

    uint16_t ngnss, gnss_id, sigM, sigM_s, cmaf;
    uint64_t satM;
    uint8_t *codes = NULL;

    ngnss                       = getbitu(buff, *i, 4); *i += 4;

    qzs_h->ngnss                = ngnss;

    if (iod_ssr != qzs_h->iod_ssr) {
        for (j = 0; j < MAXSAT; j++) {
            qzs_h->qzs_cssr->sat_n_p[j] = qzs_h->qzs_cssr->sat_n[j];
        }
        for (j = 0; j < MAXCODE; j++) {
            for (int l = 0; l < 4; l++) {
                qzs_h->qzs_cssr->sig_n_p[l][j] = qzs_h->qzs_cssr->sig_n[l][j];
            }
        }
        qzs_h->iod_ssr_p        = qzs_h->iod_ssr;
    }
    qzs_h->iod_ssr              = iod_ssr;
    qzs_h->qzs_msg->nsat_n      = 0;
    qzscssrmsg_init(qzs_h->qzs_cssr);

    for (j = 0; j < MAXSAT; j++) {
        qzs_h->qzs_msg->sat_mask[j] = 0;
    }
    print_stat(qzs_h);

    for (j = 0; j < ngnss; j++) {
        int satlist[40]={0};
        int siglist[16]={0}, siglist_s[16]={0};
        gnss_id     = getbitu(buff, *i, 4);                 *i  +=  4;
        satM        = (uint64_t)getbitu(buff,*i, 8)<<32;    *i  +=  8;
        satM        |= (uint64_t)getbitu(buff,*i, 32);      *i  += 32;
        sigM        = getbitu(buff, *i, 16);                *i  += 16;
        cmaf        = getbitu(buff, *i, 1);                 *i  +=  1;
        qzs_h->qzs_cssr->gnss_idx[j] = gnss_id;

        sys         = gnss2sys(gnss_id);
        nsat        = decode_mask(qzs_h, satM, satlist, sys, 40, CSSR_TYPE_MASK);
        nsig        = decode_mask(qzs_h, sigM, siglist, sys, 16, CSSR_TYPE_SIGMASK);

        qzs_h->qzs_cssr->nsat_g[gnss_id]    = nsat;
        qzs_h->qzs_msg->nsat_n             += nsat;
        qzs_h->qzs_cssr->nsig_max           = MAX(qzs_h->qzs_cssr->nsig_max, nsig);

        switch(sys) {
            case SYS_GPS: codes = (uint8_t *)ssr_sig_gps; break;
            case SYS_GAL: codes = (uint8_t *)ssr_sig_gal; break;
            case SYS_GLO: codes = (uint8_t *)ssr_sig_glo; break;
            case SYS_QZS: codes = (uint8_t *)ssr_sig_qzs; break;
        }
        for (int k = 0; k < nsat; k++) {
            prn_idx = satno(sys, satlist[k]);
            qzs_h->qzs_cssr->sat_n[prn_idx - 1]     = satlist[k];
            qzs_h->qzs_cssr->sys_n[prn_idx - 1]     = sys;
            qzs_h->qzs_cssr->gnss_n[prn_idx - 1]    = gnss_id;
            if (cmaf == 1) {
                /* Subset */
                sigM_s          = getbitu(buff, *i, nsig);  *i += nsig;
                nsig_s          = decode_mask(qzs_h, sigM_s, siglist_s, sys, nsig, CSSR_TYPE_SIGMASK);
                for (int l = 0; l < nsig_s; l++) {
                    sig_s_idx = siglist[siglist_s[l]];
                    qzs_h->qzs_cssr->svsig[prn_idx - 1][codes[sig_s_idx]] = 1;
                }
                qzs_h->qzs_cssr->nsig_n[prn_idx - 1]    = nsig_s;
                qzs_h->qzs_cssr->nsig_total += nsig_s;
            }
            else {
                for (int l = 0; l < nsig; l++) {
                    qzs_h->qzs_cssr->svsig[prn_idx - 1][codes[siglist[l]]] = 1;
                }
                qzs_h->qzs_cssr->nsig_total += nsig;
                qzs_h->qzs_cssr->nsig_n[prn_idx - 1]    = nsig;
            }
        }
    }

    qzs_h->qzs_msg->cstat |= (1 << CSSR_CTYPE_MASK);
    qzs_h->qzs_msg->t0[CSSR_CTYPE_MASK] = qzs_h->time;
}

static void decode_cssr_orb(QZS_H *qzs_h, uint8_t *buff, int *i) {
    int iod_ssr = decode_head(qzs_h, buff, i, CSSR_TYPE_OC);
    print_stat(qzs_h);

    for (int j = 0; j < MAXSAT; j++) {
        qzs_h->qzs_msg->iode[j] = 0;
        qzs_h->qzs_msg->deph[j][0] = qzs_h->qzs_msg->deph[j][1] = qzs_h->qzs_msg->deph[j][2] = 0.0;
        if (qzs_h->qzs_msg->sat_mask[j] == 1) {
            decode_sat_orb(qzs_h, buff, i, j);
        }
    }

    qzs_h->qzs_msg->iod_ssr_c[CSSR_CTYPE_OC] = iod_ssr;
    qzs_h->qzs_msg->cstat |= (1 << CSSR_CTYPE_OC);
    qzs_h->qzs_msg->t0[CSSR_CTYPE_OC] = qzs_h->time;
}

static void decode_cssr_clk(QZS_H *qzs_h, uint8_t *buff, int *i) {
    int iod_ssr = decode_head(qzs_h, buff, i, CSSR_TYPE_CC);
    int j;
    print_stat(qzs_h);

    if ((qzs_h->qzs_msg->cstat&(1 << CSSR_CTYPE_MASK))!=(1 << CSSR_CTYPE_MASK)) {
        return;
    }

    for (j = 0; j < MAXSAT; j++) {
        qzs_h->qzs_msg->dclk[j][0] = qzs_h->qzs_msg->dclk[j][1] = qzs_h->qzs_msg->dclk[j][2] = 0.0;
        if (qzs_h->qzs_msg->sat_mask[j] == 1) {
            decode_sat_clk(qzs_h, buff, i, j);
        }
    }

    qzs_h->qzs_msg->iod_ssr_c[CSSR_CTYPE_CC] = iod_ssr;
    qzs_h->qzs_msg->cstat |= (1 << CSSR_CTYPE_CC);
    qzs_h->qzs_msg->t0[CSSR_CTYPE_CC] = qzs_h->time;
}

static void decode_cssr_cbias(QZS_H *qzs_h, uint8_t *buff, int *i) {
    int iod_ssr = decode_head(qzs_h, buff, i, CSSR_TYPE_CB);
    print_stat(qzs_h);

    for (int j = 0; j < MAXSAT; j++) {
        for (int k = 0; k < MAXCODE; k++) {
            qzs_h->qzs_msg->cbias[j][k] = 0.0;
        }
        if (qzs_h->qzs_cssr->nsig_n[j] > 0) {
            /* Check code bias */
            decode_sat_cbias(qzs_h, buff, i, j);
        }
    }

    qzs_h->qzs_msg->iod_ssr_c[CSSR_CTYPE_CB] = iod_ssr;
    qzs_h->qzs_msg->cstat |= (1 << CSSR_CTYPE_CB);
    qzs_h->qzs_msg->t0[CSSR_CTYPE_CB] = qzs_h->time;
}

static void decode_cssr_pbias(QZS_H *qzs_h, uint8_t *buff, int *i) {
    int iod_ssr = decode_head(qzs_h, buff, i, CSSR_TYPE_PB);
    print_stat(qzs_h);

    for (int j = 0; j < MAXSAT; j++) {
        for (int k = 0; k < MAXCODE; k++) {
            qzs_h->qzs_msg->pbias[j][k] = 0.0;
            qzs_h->qzs_cssr->pdi[j][k] = -1;
        }
        if (qzs_h->qzs_cssr->nsig_n[j] > 0) {
            decode_sat_pbias(qzs_h, buff, i, j);
        }
    }

    qzs_h->qzs_msg->iod_ssr_c[CSSR_CTYPE_PB] = iod_ssr;
    qzs_h->qzs_msg->cstat |= (1 << CSSR_CTYPE_PB);
    qzs_h->qzs_msg->t0[CSSR_CTYPE_PB] = qzs_h->time;
}

static void decode_cssr_ura(QZS_H *qzs_h, uint8_t *buff, int *i) {
    int iod_ssr = decode_head(qzs_h, buff, i, CSSR_TYPE_URA);
    int j, ura;

    if (qzs_h->iod_ssr != iod_ssr) {
        return;
    }

    for (j = 0; j < MAXSAT; j++) {
        qzs_h->qzs_msg->ura[j] = 0.0;
        if (qzs_h->qzs_msg->sat_mask[j] == 1) {
            ura                     = getbitu(buff, *i, 6);     *i += 6;
            qzs_h->qzs_msg->ura[j]  = quality_idx(ura);
        }
    }

    qzs_h->qzs_msg->cstat |= (1 << CSSR_CTYPE_URA);
    qzs_h->qzs_msg->t0[CSSR_CTYPE_URA] = qzs_h->time;
}

extern void decode_qzsmsgL6e(PKSDR_Buffer *pksdr_data) {
    uint32_t msg = 4073;
    uint8_t subtype;
    int i = 0;
    while (msg == 4073) {
        msg     = getbitu(pksdr_data->buff, i, 12);     i += 12;
        if (msg != 4073) {
            return;
        }
        subtype = getbitu(pksdr_data->buff, i, 4);      i +=  4;
        pksdr_data->qzs_h->qzs_msg->subtype = subtype;
        if (subtype == CSSR_TYPE_MASK) {
            decode_cssr_mask(pksdr_data->qzs_h, pksdr_data->buff, &i);
        }
        else if (subtype == CSSR_TYPE_OC) {
            decode_cssr_orb(pksdr_data->qzs_h, pksdr_data->buff, &i);
        }
        else if (subtype == CSSR_TYPE_CC) {
            decode_cssr_clk(pksdr_data->qzs_h, pksdr_data->buff, &i);
        }
        else if (subtype == CSSR_TYPE_CB) {
            decode_cssr_cbias(pksdr_data->qzs_h, pksdr_data->buff, &i);
        }
        else if (subtype == CSSR_TYPE_PB) {
            decode_cssr_pbias(pksdr_data->qzs_h, pksdr_data->buff, &i);
        }
        else if (subtype == CSSR_TYPE_URA) {
            decode_cssr_ura(pksdr_data->qzs_h, pksdr_data->buff, &i);
        }
        if (i <= 0) {
            return;
        }
    }
}