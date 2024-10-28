//
// Created by User on 2024/1/22.
//
#include "rtklib.h"
#include "sbf_mt.h"
#include "pksdr_mt.h"

#define SBF_SYNC1   0x24
#define SBF_SYNC2   0x40

#define SBF_BDSRAWB2b   4242
#define SBF_GALRAWCNAV  4024

#define U1(p) (*((uint8_t *)(p)))

static uint16_t U2(uint8_t *p) {uint16_t a; memcpy(&a,p,2); return a;}
static uint32_t U4(uint8_t *p) {uint32_t a; memcpy(&a,p,4); return a;}

/* set fields (little-endian) ------------------------------------------------*/
static void setU1(uint8_t *p, uint8_t  u) {*p=u;}
static void setU2(uint8_t *p, uint16_t u) {memcpy(p,&u,2);}
static void setU4(uint8_t *p, uint32_t u) {memcpy(p,&u,4);}

static int gal_decode = 0;
static int qzs_decode = 0;


/* checksum lookup table -----------------------------------------------------*/
static const unsigned short CRC_16CCIT_LookUp[256] = {
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
        0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
        0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
        0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
        0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
        0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
        0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
        0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
        0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
        0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
        0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
        0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
        0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
        0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
        0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
        0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
        0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
        0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
        0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
        0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
        0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
        0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
        0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
        0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
        0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
        0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
        0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
        0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
        0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
        0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
        0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
        0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0};

/* SBF checksum calculation --------------------------------------------------*/
static uint16_t sbf_checksum(uint8_t *buff, int len)
{
    int i;
    uint16_t crc = 0;
    for (i=0; i<len; i++) {
        crc = (crc << 8) ^ CRC_16CCIT_LookUp[ (crc >> 8) ^ buff[i] ];
    }
    return crc;
}

// to hex string ---------------------------------------------------------------
static void hex_str(const uint8_t *data, int nbits, char *str)
{
    char *p = str;
    for (int i = 0; i < (nbits + 7) / 8; i++) {
        p += sprintf(p, "%02x", data[i]);
    }
}

/* generate general hex message ----------------------------------------------*/
static int gen_hex(const char *msg, uint8_t *buff)
{
    uint8_t *q=buff;
    char mbuff[1024]="",*args[256],*p;
    uint32_t byte;
    int i,narg=0;

    trace(4,"gen_hex: msg=%s\n",msg);

    strncpy(mbuff,msg,1023);
    for (p=strtok(mbuff," ");p&&narg<256;p=strtok(NULL," ")) {
        args[narg++]=p;
    }
    for (i=0;i<narg;i++) {
        if (sscanf(args[i],"%x",&byte)) *q++=(uint8_t)byte;
    }
    return (int)(q-buff);
}

extern int svid2sat(int svid)
{
    if (svid<= 37) return satno(SYS_GPS,svid);
    if (svid<= 61) return satno(SYS_GLO,svid-37);
    if (svid<= 62) return 0; /* glonass unknown slot */
    if (svid<= 68) return satno(SYS_GLO,svid-38);
    if (svid<= 70) return 0;
    if (svid<=106) return satno(SYS_GAL,svid-70);
    if (svid<=119) return 0;
    if (svid<=140) return satno(SYS_SBS,svid);
    if (svid<=180) return satno(SYS_CMP,svid-140);
    if (svid<=187) return satno(SYS_QZS,svid-180+192);
    if (svid<=190) return 0;
    if (svid<=197) return satno(SYS_IRN,svid-190);
    if (svid<=215) return satno(SYS_SBS,svid-57);
    if (svid<=222) return satno(SYS_IRN,svid-208);
    if (svid<=245) return satno(SYS_CMP,svid-182);
    return 0; /* error */
}

static void sbf_free(SBF_Buffer *sbf) {
    if (!sbf) return;
    free(sbf->bds_h->bds_msg); sbf->bds_h->bds_msg = NULL;
    free(sbf->gal_h->gal_msg); sbf->gal_h->gal_msg = NULL;
    free(sbf->gal_h->has_page); sbf->gal_h->has_page = NULL;
    free(sbf->sbf_h);
    free(sbf->bds_h);
    free(sbf->gal_h);
    free(sbf);
}

static void pksdr_free(PKSDR_Buffer  *pksdr) {
    if (!pksdr) return;
    free(pksdr->qzs_h->qzs_msg); pksdr->qzs_h->qzs_msg = NULL;
    free(pksdr->qzs_h->qzs_cssr); pksdr->qzs_h->qzs_cssr = NULL;
    free(pksdr->qzs_h);
    free(pksdr);
}

static void sbf_header_init(SBF_H *sbf_h) {
    sbf_h->length = sbf_h->CRC = sbf_h->ID = 0;
}

static void bds_header_init(BDS_H *bds_h, int BDS_ref) {
    bds_h->iod = bds_h->iod_ssr = -1;
    bds_h->disp_b2b = 0; bds_h->log_b2b = 0;
    if (BDS_ref != -1) bds_h->prn_ref = BDS_ref;
    else bds_h->prn_ref = -1;
}

static void qzs_header_init(QZS_H *qzs_h, int qzs_ref) {
    qzs_h->iod = qzs_h->iod_ssr = qzs_h->iod_ssr_p = -1;
    qzs_h->facility_p = qzs_h->fcnt = -1;
    qzs_h->ngnss = 0;
    qzs_h->disp_mdc = 0; qzs_h->log_mdc = 0;
    if (qzs_ref != -1) qzs_h->prn_ref = qzs_ref;
    else qzs_h->prn_ref = -1;
}

static void gal_header_init(GAL_H  *gal_h) {
    gal_h->mid = gal_h->ms = gal_h->mt = gal_h->pid = gal_h->mask_clkid = gal_h->mask_id = gal_h->iod_ssr = -1;
    gal_h->disp_has = 0; gal_h->log_has = 0;
    gal_h->toh = gal_h->ngnss = 0;
    gal_h->has_page = imat(255, 53);
}

extern void cssrmsg_init(CSSR_MSG *cssr_msg) {
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
        cssr_msg->sys_n[i] = cssr_msg->sat_n[i] = cssr_msg->sat_n_p[i] = cssr_msg->gnss_n[i] = -1;
        cssr_msg->nsig_n[i] = 0;
        for (j = 0; j < MAXCODE; j++) {
            cssr_msg->svsig[i][j] = -1;
            cssr_msg->sig_n[0][j] = cssr_msg->sig_n[1][j] = cssr_msg->sig_n[2][j] = cssr_msg->sig_n[3][j] = -1;
            cssr_msg->sig_n_p[0][j] = cssr_msg->sig_n_p[1][j] = cssr_msg->sig_n_p[2][j] = cssr_msg->sig_n_p[3][j] = -1;
            cssr_msg->pdi[i][j] = -1;
        }
    }
}

static void ssrmsg_init(SSR_MSG *ssr_msg) {
    ssr_msg->tow = ssr_msg->week = ssr_msg->tow0 = -1;
    ssr_msg->iodp = ssr_msg->iodp_p = -1;
    ssr_msg->subtype = ssr_msg->nsat_n = ssr_msg->cstat = 0;
    ssr_msg->nsat_bds = ssr_msg->nsat_gps = ssr_msg->nsat_gal = ssr_msg->nsat_glo = 0;
    for (int i = 0; i < MAXSAT; i++) {
        ssr_msg->iode[i] = ssr_msg->sat_mask[i] = 0;
        ssr_msg->iodc_orb[i] = ssr_msg->iodc_clk[i] = ssr_msg->iodc_clk_p[i] = -1;
        ssr_msg->ura[i] = 0.0;
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

static uint16_t check_crc(SBF_Buffer *sbf_data) {
    uint16_t crc_ = 0;
    crc_ = rtk_crc16(sbf_data->buff, sbf_data->sbf_h->length - 4);
//    crc_ = sbf_checksum(sbf_data->buff, sbf_data->sbf_h->length - 4);
    return crc_;
}

static void encode_sbf(SBF_Buffer *sbf_data, uint8_t *msg, const int type, int *i) {
    int ch, b=0;
    uint8_t *p = msg;
    uint32_t tow = (int) (sbf_data->tow * 1000);

    setbitu(sbf_data->out, *i, 32, tow);                *i += 32;
    setbitu(sbf_data->out, *i, 16, sbf_data->week);     *i += 16;

    setbitu(sbf_data->out, *i, 8, sbf_data->svid);      *i +=  8;
    setbitu(sbf_data->out, *i, 8, 1);                   *i +=  8; /* CRC must be passed */
    setbitu(sbf_data->out, *i, 8, 0);                   *i +=  8; /* Reserved */

    switch (type) {
        case SBF_BDSRAWB2b:     ch = 34; break;
        case SBF_GALRAWCNAV:    ch = 19; break;
    }
    setbitu(sbf_data->out, *i, 8, ch);                  *i +=  8;
    setbitu(sbf_data->out, *i, 8, 0);                   *i +=  8; /* Reserved */
    setbitu(sbf_data->out, *i, 8, 1);                   *i +=  8; /* Channel Number */

    for (int k = 0; k < 16; k++, p+=4) {
//        if (k == 15) {
//            setU4(sbf_data->out+ (*i), (U4(p)>>12) & 0xffffc );      *i += 32;
//        }
//        else {
//            setU4(sbf_data->out+ (*i), U4(p));      *i += 32;
//        }

        if (k == 15) {
            setbitu(sbf_data->out, *i, 32, (U4(p) >> 12) & 0xffffc);        *i += 32;
        }
        else {
            setbitu(sbf_data->out, *i, 32, getbitu(p, 0, 32)); *i += 32;
        }
    }

    for (int j = *i; *i < sbf_data->sbf_h->length * 8; j++) {
        setbitu(sbf_data->out,*i,1,0);  *i += 1;
    }
}

static int gen_sbf(SBF_Buffer *sbf_data, uint8_t *out_buff, uint8_t *msg, const int type) {
    /* 20240223 Re-generate septentrio binary format  */
    uint16_t crc_ = 0;
    uint8_t *q = out_buff;
    uint8_t *p = msg;
    int i,j=0;
    int ch;
    uint32_t tow = (int) (sbf_data->tow * 1000);

    switch (type) {
        case SBF_BDSRAWB2b:     ch = 34; break;
        case SBF_GALRAWCNAV:    ch = 19; break;
    }

    *q++ = SBF_SYNC1;
    *q++ = SBF_SYNC2;

    setU2(q, (uint16_t) sbf_data->sbf_h->CRC);      q += 2;
    setU2(q, (uint16_t) sbf_data->sbf_h->ID);       q += 2;
    setU2(q, (uint16_t) sbf_data->sbf_h->length);   q += 2;

    setU4(q, tow);                                  q += 4;
    setU2(q, (uint16_t) sbf_data->week);            q += 2;
    setU1(q, (uint8_t) sbf_data->svid);             q += 1;
    setU1(q, (uint8_t) 1);                          q += 1;
    setU1(q, (uint8_t) 0);                          q += 1;

    setU1(q, (uint8_t) ch);                         q += 1;
    setU1(q, (uint8_t) 0);                          q += 1;
    setU1(q, (uint8_t) 1);                          q += 1;

//    setbitu(sbf_data->out, i, 8, SBF_SYNC1);                i +=  8;
//    setbitu(sbf_data->out, i, 8, SBF_SYNC2);                i +=  8;

//    setbitu(sbf_data->out, i, 16,sbf_data->sbf_h->CRC);     i += 16;
//
//    setbitu(sbf_data->out, i, 16, sbf_data->sbf_h->ID);     i += 16;
//    setbitu(sbf_data->out, i, 16, sbf_data->sbf_h->length); i += 16;
    for (i = 0; i < 16; i++, p+=4) {
        if (i == 15) {
            setU4(q, getbitu(p, j, 32)); q += 4;
        }
        else {
            setU4(q, getbitu(p, j, 32)); q += 4;
        }
        j += 32;
    }

//    encode_sbf(sbf_data, msg, type, &i);

    return 1;
}

static int decode_galrawcnav(SBF_Buffer *sbf_data) {
    static int first = 0;
    static uint32_t tow_;
    uint8_t *p = sbf_data->buff + 4, buff[124];
    uint32_t week, tow;
    int i, svid, sat, prn, src, ch;
    tow = U4(p) * 0.001;
    week= U2(p+4);

    if (!first) {
        first = 1;
        tow_ = tow;
    }
    if ((fabs(tow - tow_)) > 0.4) {
        sbf_data->gal_h->gal_msg->week = week;
        sbf_data->gal_h->gal_msg->tow0 = (int) (tow/3600)*3600;
        qzs_decode = 1;
        decoder_has(sbf_data);
        tow_ = tow;
        return 3;   /* QZSS decode */
    }
    tow_ = tow;
    svid = U1(p+6);
    sbf_data->svid = svid;
    if (!(sat=svid2sat(svid))||satsys(sat,&prn)!=SYS_GAL) {
        trace(2,"sbf galrawcnav svid error: svid=%d\n",svid);
        return -1;
    }
    if (!U1(p+7)) {
        trace(3,"sbf galrawcnav parity/crc error: prn=%d\n",prn);
        return -1;
    }
    else {
        src = U1(p+9);
        ch  = U1(p+11);
        for (i=0, p+=12; i<16; i++, p+=4) {
            setbitu(buff, 32*i, 32, U4(p));
        }
        char str[256];
        int blen = (492 + 7) / 8;
        hex_str(buff, 508, str);
        if (sbf_data->gal_h->log_has) {
            ssr_log(3, "%4d\t%6d\tE%2d\t%d\t%3d\t%s",
                    sbf_data->week, (int)sbf_data->tow, prn, src, blen, str);
        }
        decode_galmsgE6b(sbf_data, buff, gal_decode);
        return 1;
    }
}

static int decode_bdsrawB2b(SBF_Buffer *sbf_data) {
    uint8_t *p = sbf_data->buff + 4, buff[200];
    uint32_t week, tow;
    int i, svid, sat, prn, src, ch;
//    tow = U4(p) * 0.001;
//    week= U2(p+4);

    svid = U1(p+6);
    sbf_data->svid  = svid;
    if (!(sat=svid2sat(svid))||satsys(sat,&prn)!=SYS_CMP) {
        trace(2,"sbf bdsraw svid error: svid=%d\n",svid);
        return -1;
    }
    if (!U1(p+7)) {
        trace(3,"sbf bdsraw parity/crc error: prn=%d\n",prn);
        return -1;
    }
    if (prn >= 59) {
        src = U1(p+9);
        ch  = U1(p+11);
        char str[256];
        char *s = str;
        for (i=0, p+=12; i<16; i++, p+=4) {
            if (i == 0) {
                s += sprintf(s, "%05x", U4(p) & 0xfffff);                   /* Check HEX code */
                setbitu(buff, 32*i, 32, U4(p));
            }
            else if (i == 15) {
                s += sprintf(s, "%05x%06x", (U4(p) >> 12) & 0xffffc, 0);    /* Check HEX code */
                setbitu(buff, 32*i, 32, (U4(p) >> 12) & 0xffffc);
            }
            else {
                s += sprintf(s, "%08x", U4(p));                             /* Check HEX code */
                setbitu(buff, 32*i, 32, U4(p));
            }
        }
        if (sbf_data->bds_h->log_b2b) {
            ssr_log(3, "%4d\t%6d\tC%2d\t%d\t%3d\t%s",
                    sbf_data->week, (int)sbf_data->tow, prn, src, 64, str);
        }
        decode_bdsmsgB2b(sbf_data, buff);
        return 1;
    }
    return 0;
}

static int decode_qzsrawL6e(PKSDR_Buffer *pksdr_data, uint8_t *buff) {
    int i = 0;
    uint32_t preamble = getbitu(buff, i, 32); i += 32;
    if (preamble != 449838109u) return 0;

    int prn, vendor, facility, res, sid, alert;
    prn         = getbitu(buff, i, 8);  i += 8;
    vendor      = getbitu(buff, i, 3);  i += 3;
    facility    = getbitu(buff, i, 2);  i += 2;
    res         = getbitu(buff, i, 2);  i += 2;
    sid         = getbitu(buff, i, 1);  i += 1;
    alert       = getbitu(buff, i, 1);  i += 1;

    if (sid == 1) {
        pksdr_data->qzs_h->fcnt = 0;
    }
    if (pksdr_data->qzs_h->facility_p >= 0 && facility != pksdr_data->qzs_h->facility_p) {
        pksdr_data->qzs_h->fcnt = -1;
    }
    pksdr_data->qzs_h->facility_p = facility;
    if (pksdr_data->qzs_h->fcnt < 0) {
        if (pksdr_data->qzs_h->disp_mdc) {
            printf("Facility changed.");
        }
        return -1;
    }

    int j = 1695 * pksdr_data->qzs_h->fcnt;
    int sz;
    uint32_t b;
    for (int k = 0; k < 53; k++) {
        if (k < 52) sz = 32;
        else        sz = 31;
        b   = getbitu(buff, i, sz);                     i  += sz;
        setbitu(pksdr_data->buff, j, sz, b);            j  += sz;
    }

    pksdr_data->qzs_h->fcnt += 1;
    return 1;
}

static int decode(SBF_Buffer *sbf_data) {
    uint8_t *p = sbf_data->buff;
    uint32_t week, tow;
    int type = U2(p)&0x1fff;

    if (sbf_data->sbf_h->length < 14) {
        return -1;
    }
    tow = U4(p+4);
    sbf_data->tow = tow * 0.001;
    week= U2(p+8);
    sbf_data->week = week;
    if (tow == 4294967295u || week == 65535u) {
        trace(2,"sbf tow/week error: type=%d len=%d\n", type, sbf_data->sbf_h->length);
        return -1;
    }

    sbf_data->time=gpst2time(week,tow*0.001);
    switch (type) {
        case SBF_BDSRAWB2b:     return decode_bdsrawB2b(sbf_data);
        case SBF_GALRAWCNAV:    return decode_galrawcnav(sbf_data);
    }
    return 0;
}

static int read_sbf(FILE *fp_sbf, SBF_Buffer *sbf_data, const int len) {

    fseek(fp_sbf, len, SEEK_SET); /* find header pointer */

    if (fread(sbf_data->head_buff, 2, 1, fp_sbf) < 1) {
        return -2;
    }
    if ( (sbf_data->head_buff[0]) == SBF_SYNC1 &&
         (sbf_data->head_buff[1]) == SBF_SYNC2 ) {
        fread(sbf_data->head_buff + 2, 6, 1, fp_sbf);
        memcpy(sbf_data->sbf_h, sbf_data->head_buff, sizeof(SBF_H));

        fseek(fp_sbf, -4, SEEK_CUR);
        fread(sbf_data->buff, sbf_data->sbf_h->length, 1, fp_sbf);

        if (sbf_data->sbf_h->length > MAXRAWLEN) {
            return 0;
        }

        if (check_crc(sbf_data) == sbf_data->sbf_h->CRC) {
            return decode(sbf_data);
        }
    }
    return 0;
}
static void sbfdata_init(SBF_Buffer *sbf_data, int BDS_ref) {
    sbf_data->sbf_h = (SBF_H *) calloc(1, sizeof(SBF_H));
    sbf_data->bds_h = (BDS_H *) calloc(1, sizeof(BDS_H));
    sbf_data->gal_h = (GAL_H *) calloc(1, sizeof(GAL_H));
    sbf_data->bds_h->bds_msg = (SSR_MSG *) calloc(1, sizeof(SSR_MSG));
    sbf_data->gal_h->gal_msg = (SSR_MSG *) calloc(1, sizeof(SSR_MSG));
    sbf_data->gal_h->gal_cssr = (CSSR_MSG *) calloc(1, sizeof(CSSR_MSG));
    sbf_header_init(sbf_data->sbf_h);
    bds_header_init(sbf_data->bds_h, BDS_ref);
    gal_header_init(sbf_data->gal_h);
    ssrmsg_init(sbf_data->bds_h->bds_msg);
    ssrmsg_init(sbf_data->gal_h->gal_msg);
    cssrmsg_init(sbf_data->gal_h->gal_cssr);
}

static void pksdrdata_init(PKSDR_Buffer *pksdr_data, int QZS_ref) {
    pksdr_data->qzs_h           = (QZS_H  *) calloc(1, sizeof(QZS_H));
    pksdr_data->qzs_h->qzs_msg  = (SSR_MSG *) calloc(1, sizeof(SSR_MSG));
    pksdr_data->qzs_h->qzs_cssr = (CSSR_MSG *) calloc(1, sizeof(CSSR_MSG));

    qzs_header_init(pksdr_data->qzs_h, QZS_ref);
    ssrmsg_init(pksdr_data->qzs_h->qzs_msg);
    cssrmsg_init(pksdr_data->qzs_h->qzs_cssr);
}

static uint8_t hex_to_int(char c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    } else if (c >= 'A' && c <= 'F') {
        return 10 + c - 'A';
    } else if (c >= 'a' && c <= 'f') {
        return 10 + c - 'a';
    }
    return 0;
}

static void str_hex(uint8_t *nav_t, const char *nav) {
    int length = 0, j = 0;
    uint8_t hex_high, hex_low;
    while (nav[length] != '\0') {
        length++;
    }

    for (int i = 0; i < length; i += 2) {
        hex_high = hex_to_int(nav[i]);
        hex_low = hex_to_int(nav[i + 1]);
        nav_t[j++] = (hex_high << 4) | hex_low;
    }
}

static int read_pksdr(FILE *fp_pksdr, PKSDR_Buffer *pksdr_data) {
    char buff[1024]="", *label = buff;
    char signal[10], navstr[1024];
    char str[512];

    uint8_t navbuff[1024], hex_high, hex_low;

    double time,ep[6];
    int prn, rev, j=0, week, tow, frm=0;
    gtime_t utc_t, gps_t;

    while (fgets(buff, sizeof(buff), fp_pksdr)) {
        if (strstr(buff, "$TIME")) {
            if (sscanf(buff, "$TIME,%lf,%lf,%lf,%lf,%lf,%lf,%lf,UTC",
                       &time,ep,ep+1,ep+2,ep+3,ep+4,ep+5) < 7) {
                return 0;
            }
            if (ep[0]<100.0) ep[0]+=ep[0]<80.0?2000.0:1900.0;
            utc_t=epoch2time(ep);
            gps_t=utc2gpst(utc_t);
            pksdr_data->tow=time2gpst(gps_t, &week);
        }
        if (strstr(buff, "L6FRM")) {
            if (sscanf(buff, "$L6FRM,%lf,L6E,%d,%d,%s", &time, &prn, &rev, navstr) < 4) {
                return 0;
            }
            if (prn != pksdr_data->qzs_h->prn_ref) continue;

            str_hex(navbuff, navstr);

            if (pksdr_data->qzs_h->log_mdc) {
                ssr_log(3, "%4d\t%6d\tJ0%1d\t%2d\t%3d\t%s",
                        pksdr_data->week, (int)pksdr_data->tow, prn-202, 1, 250, navstr);
            }
            return decode_qzsrawL6e(pksdr_data, navbuff);
//            return 1;
        }
    }
    return -1;
}

extern void sbf_processor(FILE *fp_sbf, FILE *fp_mdc, int BDS_ref, int QZS_ref,
                          int disp_b2b, int disp_has, int disp_mdc,
                          int log_b2b,  int log_has,  int log_mdc) {
    SBF_Buffer *sbf_data        = (SBF_Buffer *) calloc(1, sizeof(SBF_Buffer));
    PKSDR_Buffer *pksdr_data    = (PKSDR_Buffer *) calloc(1, sizeof(PKSDR_Buffer));
    sbfdata_init(sbf_data, BDS_ref);
    pksdrdata_init(pksdr_data, QZS_ref);

    if (disp_b2b) {
        sbf_data->bds_h->disp_b2b = disp_b2b;
    }
    if (disp_has) {
        sbf_data->gal_h->disp_has = disp_has;
    }
    if (disp_mdc) {
        pksdr_data->qzs_h->disp_mdc = disp_mdc;
    }

    if (log_b2b) {
        sbf_data->bds_h->log_b2b = log_b2b;
    }
    if (log_has) {
        sbf_data->gal_h->log_has = log_has;
    }
    if (log_mdc) {
        pksdr_data->qzs_h->log_mdc = log_mdc;
    }

    int len = 0, stat;
    for (int64_t ix = 0 ;; ix++) {
        stat = read_sbf(fp_sbf, sbf_data, len);
        if (stat == -2) break; /* End of file */

//        if (qzs_decode != 1) continue;
        if (stat == 3 && fp_mdc) {
            /* Pocket SDR decode */
            pksdr_data->week    = sbf_data->week;
            pksdr_data->tow     = sbf_data->tow;
            if (read_pksdr(fp_mdc, pksdr_data) > 0 && pksdr_data->qzs_h->fcnt == 5) {
                pksdr_data->qzs_h->qzs_msg->tow0= (int) (pksdr_data->tow / 3600) * 3600;
                pksdr_data->qzs_h->qzs_msg->week= sbf_data->week;
                decode_qzsmsgL6e(pksdr_data);
            }
            qzs_decode = 0;
            continue;
        }
        len += sbf_data->sbf_h->length;
    }
    sbf_free(sbf_data);
    pksdr_free(pksdr_data);
}

