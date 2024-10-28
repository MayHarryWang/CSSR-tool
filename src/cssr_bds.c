//
// Created by User on 2024/1/22.
//
#include "rtklib.h"
#include "sbf_mt.h"

#define U1(p) (*((uint8_t *)(p)))

#define INVALID_VALUE -10000
//static uint16_t U2(uint8_t *p) {uint16_t a; memcpy(&a,p,2); return a;}
//static uint32_t U4(uint8_t *p) {uint32_t a; memcpy(&a,p,4); return a;}

#define CSSR_TYPE_MASK  1
#define CSSR_TYPE_OC    2
#define CSSR_TYPE_CB    3
#define CSSR_TYPE_CC    4

#define CSSR_CTYPE_MASK     0
#define CSSR_CTYPE_OC       1
#define CSSR_CTYPE_CC       2
#define CSSR_CTYPE_CB       3
#define CSSR_CTYPE_URA      4
#define CSSR_CTYPE_COMB1    5
#define CSSR_CTYPE_COMB2    6

/* SSR signal and tracking mode IDs ------------------------------------------*/
static const uint8_t ssr_sig_gps[16]={
        CODE_L1C,CODE_L1P,0,0,CODE_L1L,CODE_L1X,0,CODE_L2L,
        CODE_L2X,0,0,CODE_L5I, CODE_L5Q, CODE_L5X,0, 0};
static const uint8_t ssr_sig_cmp[16]={
        CODE_L2I,CODE_L1D, CODE_L1P,0,CODE_L5D,CODE_L5P,0,CODE_L7D,
        CODE_L7P,0,0,0,CODE_L6I,0,0,0};
static const uint8_t ssr_sig_glo[16]={
        CODE_L1C,CODE_L1P,CODE_L2C,0,0,0,0,0,
        0,0, 0, 0, 0, 0, 0, 0};
static const uint8_t ssr_sig_gal[16]={
        0,CODE_L1B,CODE_L1C, 0, CODE_L5Q,CODE_L5I,0,
        CODE_L7I,CODE_L7Q,0,0,CODE_L6C,0,0,0, 0};

static void print_stat(SBF_Buffer *sbf_data) {
    /* Print the status of MT header */
    double tow = time2gpst(sbf_data->ssr_tow, &sbf_data->week);
    if (sbf_data->bds_h->disp_b2b) {
        printf("TOW = %.1f, IOD_SSR = %d\n", tow, sbf_data->bds_h->iod_ssr);
    }
}

static void print_mt(uint32_t mt, int disp) {
    if (disp) {
        printf("MT = %d, ", mt);
        if (mt==63) {
            printf("\n");
        }
    }
    else return;
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
    else return;
}

static int slot2prn(const int slot, int *prn_idx) {
    int prn = 0, sys;
    if (slot >= 1 && slot <= 63) {
        /* BDS */
        sys = SYS_CMP;
        prn = slot;
        *prn_idx = satno(sys, prn);
    }
    else if (slot > 63 && slot <= 100) {
        /* GPS */
        sys = SYS_GPS;
        prn = slot - 63;
        *prn_idx = satno(sys, prn);
    }
    else if (slot > 100 && slot <= 137) {
        /* Galileo */
        sys = SYS_GAL;
        prn = slot - 100;
        *prn_idx = satno(sys, prn);
    }
    else if (slot > 137 && slot <= 174) {
        /* GLONASS */
        sys = SYS_GLO;
        prn = slot -137;
        *prn_idx = satno(sys, prn);
    }
    return prn;
}

static double decode_sval(unsigned char *buff, int i, int n, double lsb)
{
    int slim=-((1<<(n-1))-1)-1,v;
    v = getbits(buff, i, n);
    return (v==slim) ? INVALID_VALUE:(double)v*lsb;
}

static double quality_idx(int ura) {
    double y = 0.0;
    if (ura<= 0) return y;
    if (ura>=63) return 5.4665;
    y = (pow(3.0,(ura>>3)&7) * (1.0 + (ura&7) / 4.0) - 1.0) * 1E-3;
    return y;
}

static void ssrmsg_init(SSR_MSG *ssr_msg) {
    ssr_msg->nsat_n = 0;
    for (int i = 0; i < MAXSAT; i++) {
        ssr_msg->iode[i] = ssr_msg->sat_mask[i] = 0;
        ssr_msg->iodc_orb[i] = ssr_msg->iodc_clk[i] = ssr_msg->iodc_clk_p[i] = -1;
        ssr_msg->ura[i] = 0.0;
        ssr_msg->nsat_bds = ssr_msg->nsat_gps = ssr_msg->nsat_gal = ssr_msg->nsat_glo = 0;
        for (int j = 0; j < MAXCODE; j++) {
            ssr_msg->cbias[i][j] = ssr_msg->pbias[i][j] = 0.0;
        }
        for (int j = 0; j < 3; j++) {
            ssr_msg->dclk[i][j] = ssr_msg->deph[i][j] = ssr_msg->dclk_p[i][j] = 0.0;
        }
        for (int j = 0; j < 7; j++) {
            ssr_msg->iod_ssr_c[j] = -1;
        }
    }
}

static void clkidx2satno(SSR_MSG *ssr_msg, int sat_idx, int *prn_idx) {
    *prn_idx = -1;
    int nsat_bds = ssr_msg->nsat_bds;
    int nsat_gps = ssr_msg->nsat_gps;
    int idx = NSATGPS+NSATGLO+NSATGAL+NSATQZS;
    int sys;
    if (sat_idx < nsat_bds) { /* BDS */
        for (int i = idx; i < idx + NSATCMP; i++) {
            if (ssr_msg->sat_mask[i] == 1) {
                if (sat_idx == 0) {
                    sys = satsys(i, NULL);
                    *prn_idx = i;
                    return;
                }
                sat_idx -= 1;
            }
        }
    }
    else if (sat_idx >= nsat_bds && sat_idx < nsat_bds + nsat_gps) { /* GPS */
        int gps_idx = sat_idx - nsat_bds;
        for (int i = 0; i < NSATGPS; i++) {
            if (ssr_msg->sat_mask[i] == 1) {
                if (gps_idx == 0) {
                    sys = satsys(i, NULL);
                    *prn_idx = i;
                    return;
                }
                gps_idx -= 1;
            }
        }
    }
    else return;
}

static int decode_head(SBF_Buffer *sbf_data, uint8_t *buff, int i, int *iodssr, int msgType) {
    gtime_t time = sbf_data->time;
    int sbf_tow = time2gpst(time, &sbf_data->week);
    int tow0 = (sbf_tow / 86400) * 86400;

    int iod     = getbitu(buff, i, 17); i += 21;
    *iodssr     = getbitu(buff, i, 2);  i +=  2;

    int tow = tow0 + iod;
    sbf_data->bds_h->iod = iod;

    if (tow0 >= 0) {
        sbf_data->bds_h->bds_msg->week = sbf_data->week;
        sbf_data->bds_h->bds_msg->tow = tow;
        sbf_data->bds_h->bds_msg->tow0 = tow0;
        sbf_data->ssr_tow = bdt2gpst(bdt2time(sbf_data->week, tow));
    }

    if (msgType == CSSR_TYPE_MASK) {
        sbf_data->bds_h->iod_ssr = *iodssr;
    }
    return i;
}

static int decode_sat_mask(SBF_Buffer *sbf_data, uint64_t mask, int *prn, int sys, const int len) {
    int nsat = 0;
    int prn_idx = 0;
    for (int i = 0; i < len; i++) {
        if ((mask >> (len- 1 - i)) &1) {
            prn[nsat] = i + 1;
            prn_idx = satno(sys, prn[nsat]);
            sbf_data->bds_h->bds_msg->sat_mask[prn_idx - 1] = 1;
            nsat++;
        }
    }
    return nsat;
}

static void decode_sat_orb(SBF_Buffer *sbf_data, uint8_t *buff, int i0) {
    int slot, iodn, iodc, ura, prn, prn_idx, i = i0;
    double dx, dy, dz;
    slot        = getbitu(buff, i, 9);  i += 9;
    prn         = slot2prn(slot, &prn_idx);
    if (prn == 0) {
        return;
    }
    iodn        = getbitu(buff, i, 10); i +=10;
    iodc        = getbitu(buff, i, 3);  i += 3;
    dx          = decode_sval(buff, i, 15, 0.0016); i +=15;
    dy          = decode_sval(buff, i, 13, 0.0064); i +=13;
    dz          = decode_sval(buff, i, 13, 0.0064); i +=13;
    ura         = getbitu(buff, i, 6);  i += 6;


    iodn        = iodn & (0xff);    /* IODC -> IODE */
    double  ura_idx = quality_idx(ura);

    sbf_data->bds_h->bds_msg->iode[prn_idx - 1]        = iodn;
    sbf_data->bds_h->bds_msg->iodc_orb[prn_idx - 1]    = iodc;
    sbf_data->bds_h->bds_msg->deph[prn_idx - 1][0]     = dx;
    sbf_data->bds_h->bds_msg->deph[prn_idx - 1][1]     = dy;
    sbf_data->bds_h->bds_msg->deph[prn_idx - 1][2]     = dz;
    sbf_data->bds_h->bds_msg->ura[prn_idx - 1]         = ura_idx;

    if (sbf_data->bds_h->disp_b2b) {
        if (slot > 63) {
            printf("G%02d, dx = %f, dy = %f, dz = %f, ura = %f, iodn = %d, iodc = %d\n",
                   prn, dx, dy, dz, ura_idx, iodn, iodc);
        }
        else if (slot <= 63) {
            printf("C%02d, dx = %f, dy = %f, dz = %f, ura = %f, iodn = %d, iodc = %d\n",
                   prn, dx, dy, dz, ura_idx, iodn, iodc);
        }
    }
}

static void decode_sat_clk(SBF_Buffer *sbf_data, uint8_t *buff, int sat_idx, int i0) {
    int i = i0, iodc, dclk, prn_idx;
    iodc    = getbitu(buff, i, 3);  i +=  3;
    clkidx2satno(sbf_data->bds_h->bds_msg, sat_idx, &prn_idx);
    if (prn_idx == -1) return;
    if (iodc != sbf_data->bds_h->bds_msg->iodc_clk[prn_idx]) {
        sbf_data->bds_h->bds_msg->iodc_clk_p[prn_idx] = sbf_data->bds_h->bds_msg->iodc_clk[prn_idx];
        sbf_data->bds_h->bds_msg->dclk_p[prn_idx][0] = sbf_data->bds_h->bds_msg->dclk[prn_idx][0];
    }
    sbf_data->bds_h->bds_msg->iodc_clk[prn_idx] = iodc;
    sbf_data->bds_h->bds_msg->dclk[prn_idx][0] = decode_sval(buff, i, 15, 0.0016); i += 15;
}

static void decode_mask(SBF_Buffer *sbf_data, uint8_t *buff, int i0) {
    int iod_ssr;
    int i = decode_head(sbf_data, buff, i0, &iod_ssr, CSSR_TYPE_MASK);
    int iodp    = getbitu(buff, i, 4);  i +=  4;

    sbf_data->bds_h->bds_msg->iodp = iodp;
    if (sbf_data->bds_h->bds_msg->iodp != sbf_data->bds_h->bds_msg->iodp_p) {
        sbf_data->bds_h->bds_msg->iodp_p = sbf_data->bds_h->bds_msg->iodp;
        ssrmsg_init(sbf_data->bds_h->bds_msg);
    }
    sbf_data->bds_h->bds_msg->iod_ssr_c[CSSR_CTYPE_MASK] = iod_ssr;

    print_stat(sbf_data);

    uint64_t bds_mask, gps_mask, gal_mask, glo_mask;
    int nsat_bds, nsat_gps, nsat_gal, nsat_glo;
    int bds_sat[63], gps_sat[37], gal_sat[37], glo_sat[37];

    bds_mask = (uint64_t)getbitu(buff,i, 31)<<32; i+= 31;
    bds_mask|= (uint64_t)getbitu(buff,i, 32);     i+= 32;
    nsat_bds = decode_sat_mask(sbf_data, bds_mask, bds_sat, SYS_CMP, 63);

    gps_mask = (uint64_t)getbitu(buff,i, 5)<<32;  i+=  5;
    gps_mask|= (uint64_t)getbitu(buff,i, 32);     i+= 32;
    nsat_gps = decode_sat_mask(sbf_data, gps_mask, gps_sat, SYS_GPS, 37);

    gal_mask = (uint64_t)getbitu(buff,i, 5)<<32;  i+=  5;
    gal_mask|= (uint64_t)getbitu(buff,i, 32);     i+= 32;
    nsat_gal = decode_sat_mask(sbf_data, gal_mask, gal_sat, SYS_GAL, 37);

    glo_mask = (uint64_t)getbitu(buff,i, 5)<<32;  i+=  5;
    glo_mask|= (uint64_t)getbitu(buff,i, 32);     i+= 32;
    nsat_glo = decode_sat_mask(sbf_data, glo_mask, glo_sat, SYS_GLO, 37);

    sbf_data->bds_h->bds_msg->nsat_n = nsat_bds + nsat_gps + nsat_gal + nsat_glo;
    sbf_data->bds_h->bds_msg->nsat_bds = nsat_bds;
    sbf_data->bds_h->bds_msg->nsat_gps = nsat_gps;
    sbf_data->bds_h->bds_msg->nsat_gal = nsat_gal;
    sbf_data->bds_h->bds_msg->nsat_glo = nsat_glo;
    sbf_data->bds_h->bds_msg->t0[CSSR_CTYPE_MASK] = sbf_data->ssr_tow;
    sbf_data->bds_h->bds_msg->cstat |= (1 << CSSR_CTYPE_MASK);

    if (sbf_data->bds_h->disp_b2b) {
        printf("IODP = %d\n", iodp);
        printf("Number of BDS Mask: %d\n", nsat_bds);
        print_mask_stat(nsat_bds, bds_sat, sbf_data->bds_h->disp_b2b);
        printf("Number of GPS Mask: %d\n", nsat_gps);
        print_mask_stat(nsat_gps, gps_sat, sbf_data->bds_h->disp_b2b);
        printf("Number of Galileo Mask: %d\n", nsat_gal);
        printf("Number of Glonass Mask: %d\n", nsat_glo);
    }
}

static void decode_orb(SBF_Buffer *sbf_data, uint8_t *buff, int i0) {
    int iod_ssr;
    int i = decode_head(sbf_data, buff, i0, &iod_ssr, CSSR_TYPE_OC);
    sbf_data->bds_h->bds_msg->iod_ssr_c[CSSR_CTYPE_OC] = iod_ssr;
    if (sbf_data->bds_h->iod_ssr != iod_ssr) {
        return;
    }
    print_stat(sbf_data);

    for (int k = 0; k < 6; k++) {
        decode_sat_orb(sbf_data, buff, i);   i += 69;
    }
    sbf_data->bds_h->bds_msg->t0[CSSR_CTYPE_OC] = sbf_data->ssr_tow;
    sbf_data->bds_h->bds_msg->cstat |= (1 << CSSR_CTYPE_OC);
    i += 19;
}

static void decode_cbias(SBF_Buffer *sbf_data, uint8_t *buff, int i0) {
    int iod_ssr;
    int i = decode_head(sbf_data, buff, i0, &iod_ssr, CSSR_TYPE_CB);
    if (sbf_data->bds_h->iod_ssr != iod_ssr) {
        return;
    }
    print_stat(sbf_data);
    int n_sat, slot, n_sig, prn, prn_idx, sig_idx, sys;
    double cb;
    n_sat       = getbitu(buff, i, 5);  i +=  5;

    for (int j = 0; j < n_sat; j++) {
        slot        = getbitu(buff, i, 9);  i += 9;
        prn         = slot2prn(slot, &prn_idx);
        sys         = satsys(prn_idx, NULL);
        if (prn == 0) continue;
        n_sig       = getbitu(buff, i, 4);  i += 4;
        for (int k = 0; k < n_sig; k++) {
            sig_idx         = getbitu(buff, i, 4);                i +=  4;
            cb              = decode_sval(buff, i, 12, 0.017);      i += 12;
            switch(sys) {
                case SYS_GPS: sbf_data->bds_h->bds_msg->cbias[prn_idx - 1][ssr_sig_gps[sig_idx]] = cb; break;
                case SYS_CMP: sbf_data->bds_h->bds_msg->cbias[prn_idx - 1][ssr_sig_cmp[sig_idx]] = cb; break;
                case SYS_GAL: sbf_data->bds_h->bds_msg->cbias[prn_idx - 1][ssr_sig_gal[sig_idx]] = cb; break;
                case SYS_GLO: sbf_data->bds_h->bds_msg->cbias[prn_idx - 1][ssr_sig_glo[sig_idx]] = cb; break;
            }
        }
    }
    sbf_data->bds_h->bds_msg->t0[CSSR_CTYPE_CB] = sbf_data->ssr_tow;
    sbf_data->bds_h->bds_msg->cstat |= (1 << CSSR_CTYPE_CB);
}

static void decode_clk(SBF_Buffer *sbf_data, uint8_t *buff, int i0) {
    int iod_ssr;
    int i = decode_head(sbf_data, buff, i0, &iod_ssr, CSSR_TYPE_CC);
    if (sbf_data->bds_h->iod_ssr != iod_ssr) {
        return;
    }

    print_stat(sbf_data);
    int iodp    = getbitu(buff, i, 4);  i +=  4;
    int st1     = getbitu(buff, i, 5);  i +=  5;

    if (iodp != sbf_data->bds_h->bds_msg->iodp) {
        return;
    }

    int sat_idx;
    for (int j = 0; j < 23; j++) {
        sat_idx = st1 * 23 + j;
        if (sat_idx < sbf_data->bds_h->bds_msg->nsat_n) {
            decode_sat_clk(sbf_data, buff, sat_idx, i);
        }
        i += 18;
    }
    sbf_data->bds_h->bds_msg->t0[CSSR_CTYPE_CC] = sbf_data->ssr_tow;
    sbf_data->bds_h->bds_msg->cstat |= (1 << CSSR_CTYPE_CC);
    i += 10;
}

extern void decode_bdsmsgB2b(SBF_Buffer *sbf_data, uint8_t *buff) {
    static int startdecode = FALSE;
    int i = 0, j;
    uint32_t prn, mt;
    prn = getbitu(buff, i, 6);   i += 12;
    mt  = getbitu(buff, i, 6);   i +=  6;
    if (sbf_data->bds_h->prn_ref != prn)
        return;
    print_mt(mt, sbf_data->bds_h->disp_b2b);
    if (mt == 1) {
        /* Satellite mask */
        sbf_data->bds_h->bds_msg->subtype = CSSR_TYPE_MASK;
        decode_mask(sbf_data, buff, i);
    }
    else if (mt == 2) {
        /* Orbit correction */
        sbf_data->bds_h->bds_msg->subtype = CSSR_TYPE_OC;
        decode_orb(sbf_data, buff, i);
    }
    else if (mt == 3) {
        /* Code bias */
        sbf_data->bds_h->bds_msg->subtype = CSSR_TYPE_CB;
        decode_cbias(sbf_data, buff, i);
    }
    else if (mt == 4) {
        /* Clock correction */
        sbf_data->bds_h->bds_msg->subtype = CSSR_TYPE_CC;
        decode_clk(sbf_data, buff, i);
    }
    else if (mt == 5) {
        /* Not broadcast yet */
    }
    else if (mt == 6) {
        /* Not broadcast yet */
    }
    else if (mt == 7) {
        /* Not broadcast yet */
    }
    else if (mt == 63) {
    }
}
