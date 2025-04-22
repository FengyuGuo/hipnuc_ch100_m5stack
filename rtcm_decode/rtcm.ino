/*------------------------------------------------------------------------------
* rtcm.c : rtcm functions
*
*          Copyright (C) 2009-2018 by T.TAKASU, All rights reserved.
*
* references :
*     [1] RTCM Recommended Standards for Differential GNSS (Global Navigation
*         Satellite Systems) Service version 2.3, August 20, 2001
*     [2] RTCM Standard 10403.1 for Differential GNSS (Global Navigation
*         Satellite Systems) Services - Version 3, Octobar 27, 2006
*     [3] RTCM 10403.1-Amendment 3, Amendment 3 to RTCM Standard 10403.1
*     [4] RTCM Paper, April 12, 2010, Proposed SSR Messages for SV Orbit Clock,
*         Code Biases, URA
*     [5] RTCM Paper 012-2009-SC104-528, January 28, 2009 (previous ver of [4])
*     [6] RTCM Paper 012-2009-SC104-582, February 2, 2010 (previous ver of [4])
*     [7] RTCM Standard 10403.1 - Amendment 5, Differential GNSS (Global
*         Navigation Satellite Systems) Services - version 3, July 1, 2011
*     [8] RTCM Paper 019-2012-SC104-689 (draft Galileo ephmeris messages)
*     [9] RTCM Paper 163-2012-SC104-725 (draft QZSS ephemeris message)
*     [10] RTCM Paper 059-2011-SC104-635 (draft Galileo and QZSS ssr messages)
*     [11] RTCM Paper 034-2012-SC104-693 (draft multiple signal messages)
*     [12] RTCM Paper 133-2012-SC104-709 (draft QZSS MSM messages)
*     [13] RTCM Paper 122-2012-SC104-707.r1 (draft MSM messages)
*     [14] RTCM Standard 10403.2, Differential GNSS (Global Navigation Satellite
*          Systems) Services - version 3, February 1, 2013
*     [15] RTCM Standard 10403.2, Differential GNSS (Global Navigation Satellite
*          Systems) Services - version 3, with amendment 1/2, november 7, 2013
*     [16] Proposal of new RTCM SSR Messages (ssr_1_gal_qzss_sbas_dbs_v05)
*          2014/04/17
*     [17] RTCM Standard 10403.2, Differential GNSS (Global Navigation Satellite
*          Systems) Services - version 3, October 7, 2016
*
* version : $Revision:$ $Date:$
* history : 2009/04/10 1.0  new
*           2009/06/29 1.1  support type 1009-1012 to get synchronous-gnss-flag
*           2009/12/04 1.2  support type 1010,1012,1020
*           2010/07/15 1.3  support type 1057-1068 for ssr corrections
*                           support type 1007,1008,1033 for antenna info
*           2010/09/08 1.4  fix problem of ephemeris and ssr sequence upset
*                           (2.4.0_p8)
*           2012/05/11 1.5  comply with RTCM 3 final SSR format (RTCM 3
*                           Amendment 5) (ref [7]) (2.4.1_p6)
*           2012/05/14 1.6  separate rtcm2.c, rtcm3.c
*                           add options to select used codes for msm
*           2013/04/27 1.7  comply with rtcm 3.2 with amendment 1/2 (ref[15])
*           2013/12/06 1.8  support SBAS/BeiDou SSR messages (ref[16])
*           2018/01/29 1.9  support RTCM 3.3 (ref[17])
*                           crc24q() -> rtk_crc24q()
*-----------------------------------------------------------------------------*/
#include "rtklib.h"

static const char rcsid[]="$Id:$";

/* function prototypes -------------------------------------------------------*/
// extern int decode_rtcm2(rtcm_t *rtcm);
extern int decode_rtcm3(rtcm_t *rtcm);
// extern int encode_rtcm3(rtcm_t *rtcm, int type, int sync);

/* constants -----------------------------------------------------------------*/

#define RTCM2PREAMB 0x66        /* rtcm ver.2 frame preamble */
#define RTCM3PREAMB 0xD3        /* rtcm ver.3 frame preamble */
static const unsigned int tbl_CRC24Q[]={
    0x000000,0x864CFB,0x8AD50D,0x0C99F6,0x93E6E1,0x15AA1A,0x1933EC,0x9F7F17,
    0xA18139,0x27CDC2,0x2B5434,0xAD18CF,0x3267D8,0xB42B23,0xB8B2D5,0x3EFE2E,
    0xC54E89,0x430272,0x4F9B84,0xC9D77F,0x56A868,0xD0E493,0xDC7D65,0x5A319E,
    0x64CFB0,0xE2834B,0xEE1ABD,0x685646,0xF72951,0x7165AA,0x7DFC5C,0xFBB0A7,
    0x0CD1E9,0x8A9D12,0x8604E4,0x00481F,0x9F3708,0x197BF3,0x15E205,0x93AEFE,
    0xAD50D0,0x2B1C2B,0x2785DD,0xA1C926,0x3EB631,0xB8FACA,0xB4633C,0x322FC7,
    0xC99F60,0x4FD39B,0x434A6D,0xC50696,0x5A7981,0xDC357A,0xD0AC8C,0x56E077,
    0x681E59,0xEE52A2,0xE2CB54,0x6487AF,0xFBF8B8,0x7DB443,0x712DB5,0xF7614E,
    0x19A3D2,0x9FEF29,0x9376DF,0x153A24,0x8A4533,0x0C09C8,0x00903E,0x86DCC5,
    0xB822EB,0x3E6E10,0x32F7E6,0xB4BB1D,0x2BC40A,0xAD88F1,0xA11107,0x275DFC,
    0xDCED5B,0x5AA1A0,0x563856,0xD074AD,0x4F0BBA,0xC94741,0xC5DEB7,0x43924C,
    0x7D6C62,0xFB2099,0xF7B96F,0x71F594,0xEE8A83,0x68C678,0x645F8E,0xE21375,
    0x15723B,0x933EC0,0x9FA736,0x19EBCD,0x8694DA,0x00D821,0x0C41D7,0x8A0D2C,
    0xB4F302,0x32BFF9,0x3E260F,0xB86AF4,0x2715E3,0xA15918,0xADC0EE,0x2B8C15,
    0xD03CB2,0x567049,0x5AE9BF,0xDCA544,0x43DA53,0xC596A8,0xC90F5E,0x4F43A5,
    0x71BD8B,0xF7F170,0xFB6886,0x7D247D,0xE25B6A,0x641791,0x688E67,0xEEC29C,
    0x3347A4,0xB50B5F,0xB992A9,0x3FDE52,0xA0A145,0x26EDBE,0x2A7448,0xAC38B3,
    0x92C69D,0x148A66,0x181390,0x9E5F6B,0x01207C,0x876C87,0x8BF571,0x0DB98A,
    0xF6092D,0x7045D6,0x7CDC20,0xFA90DB,0x65EFCC,0xE3A337,0xEF3AC1,0x69763A,
    0x578814,0xD1C4EF,0xDD5D19,0x5B11E2,0xC46EF5,0x42220E,0x4EBBF8,0xC8F703,
    0x3F964D,0xB9DAB6,0xB54340,0x330FBB,0xAC70AC,0x2A3C57,0x26A5A1,0xA0E95A,
    0x9E1774,0x185B8F,0x14C279,0x928E82,0x0DF195,0x8BBD6E,0x872498,0x016863,
    0xFAD8C4,0x7C943F,0x700DC9,0xF64132,0x693E25,0xEF72DE,0xE3EB28,0x65A7D3,
    0x5B59FD,0xDD1506,0xD18CF0,0x57C00B,0xC8BF1C,0x4EF3E7,0x426A11,0xC426EA,
    0x2AE476,0xACA88D,0xA0317B,0x267D80,0xB90297,0x3F4E6C,0x33D79A,0xB59B61,
    0x8B654F,0x0D29B4,0x01B042,0x87FCB9,0x1883AE,0x9ECF55,0x9256A3,0x141A58,
    0xEFAAFF,0x69E604,0x657FF2,0xE33309,0x7C4C1E,0xFA00E5,0xF69913,0x70D5E8,
    0x4E2BC6,0xC8673D,0xC4FECB,0x42B230,0xDDCD27,0x5B81DC,0x57182A,0xD154D1,
    0x26359F,0xA07964,0xACE092,0x2AAC69,0xB5D37E,0x339F85,0x3F0673,0xB94A88,
    0x87B4A6,0x01F85D,0x0D61AB,0x8B2D50,0x145247,0x921EBC,0x9E874A,0x18CBB1,
    0xE37B16,0x6537ED,0x69AE1B,0xEFE2E0,0x709DF7,0xF6D10C,0xFA48FA,0x7C0401,
    0x42FA2F,0xC4B6D4,0xC82F22,0x4E63D9,0xD11CCE,0x575035,0x5BC9C3,0xDD8538
};
/* crc-24q parity --------------------------------------------------------------
* compute crc-24q parity for sbas, rtcm3
* args   : unsigned char *buff I data
*          int    len    I      data length (bytes)
* return : crc-24Q parity
* notes  : see reference [2] A.4.3.3 Parity
*-----------------------------------------------------------------------------*/
extern unsigned int rtk_crc24q(const unsigned char *buff, int len)
{
    unsigned int crc=0;
    int i;
    
    trace(4,"crc24q: len=%d\n",len);
    
    for (i=0;i<len;i++) crc=((crc<<8)&0xFFFFFF)^tbl_CRC24Q[(crc>>16)^buff[i]];
    return crc;
}
/* set unsigned/signed bits ----------------------------------------------------
* set unsigned/signed bits to byte data
* args   : unsigned char *buff IO byte data
*          int    pos    I      bit position from start of data (bits)
*          int    len    I      bit length (bits) (len<=32)
*         (unsigned) int I      unsigned/signed data
* return : none
*-----------------------------------------------------------------------------*/
extern void setbitu(unsigned char *buff, int pos, int len, unsigned int data)
{
    unsigned int mask=1u<<(len-1);
    int i;
    if (len<=0||32<len) return;
    for (i=pos;i<pos+len;i++,mask>>=1) {
        if (data&mask) buff[i/8]|=1u<<(7-i%8); else buff[i/8]&=~(1u<<(7-i%8));
    }
}

/* initialize rtcm control -----------------------------------------------------
* initialize rtcm control struct and reallocate memory for observation and
* ephemeris buffer in rtcm control struct
* args   : rtcm_t *raw   IO     rtcm control struct
* return : status (1:ok,0:memory allocation error)
*-----------------------------------------------------------------------------*/
extern int init_rtcm(rtcm_t *rtcm)
{
    gtime_t time0={0};
    obsd_t data0={{0}};
    eph_t  eph0 ={0,-1,-1};
    geph_t geph0={0,-1};
    ssr_t ssr0={{{0}}};
    int i,j;
    
    trace(3,"init_rtcm:\n");
    
    rtcm->staid=rtcm->stah=rtcm->seqno=rtcm->outtype=0;
    rtcm->time=rtcm->time_s=time0;
    rtcm->sta.name[0]=rtcm->sta.marker[0]='\0';
    rtcm->sta.antdes[0]=rtcm->sta.antsno[0]='\0';
    rtcm->sta.rectype[0]=rtcm->sta.recver[0]=rtcm->sta.recsno[0]='\0';
    rtcm->sta.antsetup=rtcm->sta.itrf=rtcm->sta.deltype=0;
    for (i=0;i<3;i++) {
        rtcm->sta.pos[i]=rtcm->sta.del[i]=0.0;
    }
    rtcm->sta.hgt=0.0;
    rtcm->dgps=NULL;
    for (i=0;i<MAXSAT;i++) {
        rtcm->ssr[i]=ssr0;
    }
    rtcm->msg[0]=rtcm->msgtype[0]=rtcm->opt[0]='\0';
    for (i=0;i<6;i++) rtcm->msmtype[i][0]='\0';
    rtcm->obsflag=rtcm->ephsat=0;
    for (i=0;i<MAXSAT;i++) for (j=0;j<NFREQ+NEXOBS;j++) {
        rtcm->cp[i][j]=0.0;
        rtcm->lock[i][j]=rtcm->loss[i][j]=0;
        rtcm->lltime[i][j]=time0;
    }
    rtcm->nbyte=rtcm->nbit=rtcm->len=0;
    rtcm->word=0;
    for (i=0;i<100;i++) rtcm->nmsg2[i]=0;
    for (i=0;i<300;i++) rtcm->nmsg3[i]=0;
    
    rtcm->obs.data=NULL;
    rtcm->nav.eph =NULL;
    rtcm->nav.geph=NULL;
    
    /* reallocate memory for observation and ephemris buffer */
    if (!(rtcm->obs.data=(obsd_t *)malloc(sizeof(obsd_t)*MAXOBS))||
        !(rtcm->nav.eph =(eph_t  *)malloc(sizeof(eph_t )*MAXSAT))||
        !(rtcm->nav.geph=(geph_t *)malloc(sizeof(geph_t)*MAXPRNGLO))) {
        free_rtcm(rtcm);
        return 0;
    }
    rtcm->obs.n=0;
    rtcm->nav.n=MAXSAT;
    rtcm->nav.ng=MAXPRNGLO;
    for (i=0;i<MAXOBS   ;i++) rtcm->obs.data[i]=data0;
    for (i=0;i<MAXSAT   ;i++) rtcm->nav.eph [i]=eph0;
    for (i=0;i<MAXPRNGLO;i++) rtcm->nav.geph[i]=geph0;
    return 1;
}
/* free rtcm control ----------------------------------------------------------
* free observation and ephemris buffer in rtcm control struct
* args   : rtcm_t *raw   IO     rtcm control struct
* return : none
*-----------------------------------------------------------------------------*/
extern void free_rtcm(rtcm_t *rtcm)
{
    trace(3,"free_rtcm:\n");
    
    /* free memory for observation and ephemeris buffer */
    free(rtcm->obs.data); rtcm->obs.data=NULL; rtcm->obs.n=0;
    free(rtcm->nav.eph ); rtcm->nav.eph =NULL; rtcm->nav.n=0;
    free(rtcm->nav.geph); rtcm->nav.geph=NULL; rtcm->nav.ng=0;
}
/* input rtcm 2 message from stream --------------------------------------------
* fetch next rtcm 2 message and input a message from byte stream
* args   : rtcm_t *rtcm IO   rtcm control struct
*          unsigned char data I stream data (1 byte)
* return : status (-1: error message, 0: no message, 1: input observation data,
*                  2: input ephemeris, 5: input station pos/ant parameters,
*                  6: input time parameter, 7: input dgps corrections,
*                  9: input special message)
* notes  : before firstly calling the function, time in rtcm control struct has
*          to be set to the approximate time within 1/2 hour in order to resolve
*          ambiguity of time in rtcm messages.
*          supported msgs RTCM ver.2: 1,3,9,14,16,17,18,19,22
*          refer [1] for RTCM ver.2
*-----------------------------------------------------------------------------*/
// extern int input_rtcm2(rtcm_t *rtcm, unsigned char data)
// {
//     unsigned char preamb;
//     int i;
    
//     trace(5,"input_rtcm2: data=%02x\n",data);
    
//     if ((data&0xC0)!=0x40) return 0; /* ignore if upper 2bit != 01 */
    
//     for (i=0;i<6;i++,data>>=1) { /* decode 6-of-8 form */
//         rtcm->word=(rtcm->word<<1)+(data&1);
        
//         /* synchronize frame */
//         if (rtcm->nbyte==0) {
//             preamb=(unsigned char)(rtcm->word>>22);
//             if (rtcm->word&0x40000000) preamb^=0xFF; /* decode preamble */
//             if (preamb!=RTCM2PREAMB) continue;
            
//             /* check parity */
//             if (!decode_word(rtcm->word,rtcm->buff)) continue;
//             rtcm->nbyte=3; rtcm->nbit=0;
//             continue;
//         }
//         if (++rtcm->nbit<30) continue; else rtcm->nbit=0;
        
//         /* check parity */
//         if (!decode_word(rtcm->word,rtcm->buff+rtcm->nbyte)) {
//             trace(2,"rtcm2 partity error: i=%d word=%08x\n",i,rtcm->word);
//             rtcm->nbyte=0; rtcm->word&=0x3;
//             continue;
//         }
//         rtcm->nbyte+=3;
//         if (rtcm->nbyte==6) rtcm->len=(rtcm->buff[5]>>3)*3+6;
//         if (rtcm->nbyte<rtcm->len) continue;
//         rtcm->nbyte=0; rtcm->word&=0x3;
        
//         /* decode rtcm2 message */
//         return decode_rtcm2(rtcm);
//     }
//     return 0;
// }
/* input rtcm 3 message from stream --------------------------------------------
* fetch next rtcm 3 message and input a message from byte stream
* args   : rtcm_t *rtcm IO   rtcm control struct
*          unsigned char data I stream data (1 byte)
* return : status (-1: error message, 0: no message, 1: input observation data,
*                  2: input ephemeris, 5: input station pos/ant parameters,
*                  10: input ssr messages)
* notes  : before firstly calling the function, time in rtcm control struct has
*          to be set to the approximate time within 1/2 week in order to resolve
*          ambiguity of time in rtcm messages.
*          
*          to specify input options, set rtcm->opt to the following option
*          strings separated by spaces.
*
*          -EPHALL  : input all ephemerides
*          -STA=nnn : input only message with STAID=nnn
*          -GLss    : select signal ss for GPS MSM (ss=1C,1P,...)
*          -RLss    : select signal ss for GLO MSM (ss=1C,1P,...)
*          -ELss    : select signal ss for GAL MSM (ss=1C,1B,...)
*          -JLss    : select signal ss for QZS MSM (ss=1C,2C,...)
*          -CLss    : select signal ss for BDS MSM (ss=2I,7I,...)
*
*          supported RTCM 3 messages
*             (ref [2][3][4][5][6][7][8][9][10][11][12][13][14][15][16][17])
*
*            TYPE       GPS     GLOASS    GALILEO    QZSS     BEIDOU     SBAS
*         ----------------------------------------------------------------------
*          OBS C-L1  : 1001~     1009~       -         -         -         -
*              F-L1  : 1002      1010        -         -         -         -
*              C-L12 : 1003~     1011~       -         -         -         -
*              F-L12 : 1004      1012        -         -         -         -
*
*          NAV       : 1019      1020      1045      1044      1042        -
*                        -         -       1046        -         63*       -
*
*          MSM 1     : 1071~     1081~     1091~     1111~     1121~     1101~
*              2     : 1072~     1082~     1092~     1112~     1122~     1102~
*              3     : 1073~     1083~     1093~     1113~     1123~     1103~
*              4     : 1074      1084      1094      1114      1124      1104
*              5     : 1075      1085      1095      1115      1125      1105
*              6     : 1076      1086      1096      1116      1126      1106
*              7     : 1077      1087      1097      1117      1127      1107
*
*          SSR OBT   : 1057      1063      1240*     1246*     1258*       -
*              CLK   : 1058      1064      1241*     1247*     1259*       -
*              BIAS  : 1059      1065      1242*     1248*     1260*       -
*              OBTCLK: 1060      1066      1243*     1249*     1261*       -
*              URA   : 1061      1067      1244*     1250*     1262*       -
*              HRCLK : 1062      1068      1245*     1251*     1263*       -
*
*          ANT INFO  : 1005 1006 1007 1008 1033
*         ----------------------------------------------------------------------
*                                                    (* draft, ~ only encode)
*
*          for MSM observation data with multiple signals for a frequency,
*          a signal is selected according to internal priority. to select
*          a specified signal, use the input options.
*
*          rtcm3 message format:
*            +----------+--------+-----------+--------------------+----------+
*            | preamble | 000000 |  length   |    data message    |  parity  |
*            +----------+--------+-----------+--------------------+----------+
*            |<-- 8 --->|<- 6 -->|<-- 10 --->|<--- length x 8 --->|<-- 24 -->|
*            
*-----------------------------------------------------------------------------*/
extern int input_rtcm3(rtcm_t *rtcm, unsigned char data)
{
    trace(5,"input_rtcm3: data=%02x\n",data);
    
    /* synchronize frame */
    if (rtcm->nbyte==0) {
        if (data!=RTCM3PREAMB) return 0; //check if frame starts
        rtcm->buff[rtcm->nbyte++]=data;
        return 0;
    }
    rtcm->buff[rtcm->nbyte++]=data; //copy the data to rtcm buffer
    
    if (rtcm->nbyte==3) { //first 3 bytes. length of the frame
        rtcm->len=getbitu(rtcm->buff,14,10)+3; /* length without parity */
    }
    if (rtcm->nbyte<3||rtcm->nbyte<rtcm->len+3) return 0; //frame not complete
    rtcm->nbyte=0;
    
    /* check parity */
    if (rtk_crc24q(rtcm->buff,rtcm->len)!=getbitu(rtcm->buff,rtcm->len*8,24)) {
        trace(2,"rtcm3 parity error: len=%d\n",rtcm->len);
        return 0;
    }
    /* decode rtcm3 message */
    return decode_rtcm3(rtcm);
}
/* input rtcm 2 message from file ----------------------------------------------
* fetch next rtcm 2 message and input a messsage from file
* args   : rtcm_t *rtcm IO   rtcm control struct
*          FILE  *fp    I    file pointer
* return : status (-2: end of file, -1...10: same as above)
* notes  : same as above
*-----------------------------------------------------------------------------*/
// extern int input_rtcm2f(rtcm_t *rtcm, FILE *fp)
// {
//     int i,data=0,ret;
    
//     trace(4,"input_rtcm2f: data=%02x\n",data);
    
//     for (i=0;i<4096;i++) {
//         if ((data=fgetc(fp))==EOF) return -2;
//         if ((ret=input_rtcm2(rtcm,(unsigned char)data))) return ret;
//     }
//     return 0; /* return at every 4k bytes */
// }
/* input rtcm 3 message from file ----------------------------------------------
* fetch next rtcm 3 message and input a messsage from file
* args   : rtcm_t *rtcm IO   rtcm control struct
*          FILE  *fp    I    file pointer
* return : status (-2: end of file, -1...10: same as above)
* notes  : same as above
*-----------------------------------------------------------------------------*/
extern int input_rtcm3f(rtcm_t *rtcm, FILE *fp)
{
    int i,data=0,ret;
    
    trace(4,"input_rtcm3f: data=%02x\n",data);
    
    for (i=0;i<4096;i++) {
        if ((data=fgetc(fp))==EOF) return -2;
        if ((ret=input_rtcm3(rtcm,(unsigned char)data))) return ret;
    }
    return 0; /* return at every 4k bytes */
}
/* generate rtcm 2 message -----------------------------------------------------
* generate rtcm 2 message
* args   : rtcm_t *rtcm   IO rtcm control struct
*          int    type    I  message type
*          int    sync    I  sync flag (1:another message follows)
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
// extern int gen_rtcm2(rtcm_t *rtcm, int type, int sync)
// {
//     trace(4,"gen_rtcm2: type=%d sync=%d\n",type,sync);
    
//     rtcm->nbit=rtcm->len=rtcm->nbyte=0;
    
//     /* not yet implemented */
    
//     return 0;
// }
/* generate rtcm 3 message -----------------------------------------------------
* generate rtcm 3 message
* args   : rtcm_t *rtcm   IO rtcm control struct
*          int    type    I  message type
*          int    sync    I  sync flag (1:another message follows)
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
//extern int gen_rtcm3(rtcm_t *rtcm, int type, int sync)
//{
//    unsigned int crc;
//    int i=0;
//    
//    trace(4,"gen_rtcm3: type=%d sync=%d\n",type,sync);
//    
//    rtcm->nbit=rtcm->len=rtcm->nbyte=0;
//    
//    /* set preamble and reserved */
//    setbitu(rtcm->buff,i, 8,RTCM3PREAMB); i+= 8;
//    setbitu(rtcm->buff,i, 6,0          ); i+= 6;
//    setbitu(rtcm->buff,i,10,0          ); i+=10;
//    
//    /* encode rtcm 3 message body */
//    if (!encode_rtcm3(rtcm,type,sync)) return 0;
//    
//    /* padding to align 8 bit boundary */
//    for (i=rtcm->nbit;i%8;i++) {
//        setbitu(rtcm->buff,i,1,0);
//    }
//    /* message length (header+data) (bytes) */
//    if ((rtcm->len=i/8)>=3+1024) {
//        trace(2,"generate rtcm 3 message length error len=%d\n",rtcm->len-3);
//        rtcm->nbit=rtcm->len=0;
//        return 0;
//    }
//    /* message length without header and parity */
//    setbitu(rtcm->buff,14,10,rtcm->len-3);
//    
//    /* crc-24q */
//    crc=rtk_crc24q(rtcm->buff,rtcm->len);
//    setbitu(rtcm->buff,i,24,crc);
//    
//    /* length total (bytes) */
//    rtcm->nbyte=rtcm->len+3;
//    
//    return 1;
//}
