/*------------------------------------------------------------------------------
* rtcm3.c : rtcm ver.3 message decorder functions
*
*          Copyright (C) 2009-2018 by T.TAKASU, All rights reserved.
*
* references :
*     see rtcm.c
*
* version : $Revision:$ $Date:$
* history : 2012/05/14 1.0  separated from rtcm.c
*           2012/12/12 1.1  support gal/qzs ephemeris, gal/qzs ssr, msm
*                           add station id consistency test for obs data
*           2012/12/25 1.2  change compass msm id table
*           2013/01/31 1.3  change signal id by the latest draft (ref [13])
*           2013/02/23 1.4  change reference for rtcm 3 message (ref [14])
*           2013/05/19 1.5  gpst -> bdt of time-tag in beidou msm message
*           2014/05/02 1.6  fix bug on dropping last field of ssr message
*                           comply with rtcm 3.2 with amendment 1/2 (ref[15])
*                           delete MT 1046 according to ref [15]
*           2014/09/14 1.7  add receiver option -RT_INP
*           2014/12/06 1.8  support SBAS/BeiDou SSR messages (ref [16])
*           2015/03/22 1.9  add handling of iodcrc for beidou/sbas ssr messages
*           2018/01/29 1.10 add support of MT1042, MT1046, MT63
*                           delete support of MT1047
*                           delete compile option SSR_QZSS_DRAFT_V05
*                           update SSR signal and tracking mode IDs
*                           fix bug on L2C code in MT1004 (##131)
*                           fix bug on loss-of-lock detection in MSM 6/7 (##134)
*                           fix bug on ssr 3 message decoding (#321)
*                           fix bug on MT1045 Galileo week rollover
*-----------------------------------------------------------------------------*/
#include "rtklib.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <ctype.h>
#include <dirent.h>

//static const char rcsid[]="$Id:$";

/* constants -----------------------------------------------------------------*/

#define PRUNIT_GPS  299792.458  /* rtcm ver.3 unit of gps pseudorange (m) */
#define PRUNIT_GLO  599584.916  /* rtcm ver.3 unit of glonass pseudorange (m) */
#define RANGE_MS    (CLIGHT*0.001)      /* range in 1 ms */

#define P2_10       0.0009765625          /* 2^-10 */
#define P2_34       5.820766091346740E-11 /* 2^-34 */
#define P2_46       1.421085471520200E-14 /* 2^-46 */
#define P2_59       1.734723475976810E-18 /* 2^-59 */
#define P2_66       1.355252715606880E-20 /* 2^-66 */



/* msm signal id table -------------------------------------------------------*/
const char *msm_sig_gps[32]={
    /* GPS: ref [13] table 3.5-87, ref [14][15] table 3.5-91 */
    ""  ,"1C","1P","1W","1Y","1M",""  ,"2C","2P","2W","2Y","2M", /*  1-12 */
    ""  ,""  ,"2S","2L","2X",""  ,""  ,""  ,""  ,"5I","5Q","5X", /* 13-24 */
    ""  ,""  ,""  ,""  ,""  ,"1S","1L","1X"                      /* 25-32 */
};
const char *msm_sig_glo[32]={
    /* GLONASS: ref [13] table 3.5-93, ref [14][15] table 3.5-97 */
    ""  ,"1C","1P",""  ,""  ,""  ,""  ,"2C","2P",""  ,"3I","3Q",
    "3X",""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,
    ""  ,""  ,""  ,""  ,""  ,""  ,""  ,""
};
const char *msm_sig_gal[32]={
    /* Galileo: ref [15] table 3.5-100 */
    ""  ,"1C","1A","1B","1X","1Z",""  ,"6C","6A","6B","6X","6Z",
    ""  ,"7I","7Q","7X",""  ,"8I","8Q","8X",""  ,"5I","5Q","5X",
    ""  ,""  ,""  ,""  ,""  ,""  ,""  ,""
};
const char *msm_sig_qzs[32]={
    /* QZSS: ref [15] table 3.5-103 */
    ""  ,"1C",""  ,""  ,""  ,""  ,""  ,""  ,"6S","6L","6X",""  ,
    ""  ,""  ,"2S","2L","2X",""  ,""  ,""  ,""  ,"5I","5Q","5X",
    ""  ,""  ,""  ,""  ,""  ,"1S","1L","1X"
};
const char *msm_sig_sbs[32]={
    /* SBAS: ref [13] table 3.5-T+005 */
    ""  ,"1C",""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,
    ""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,"5I","5Q","5X",
    ""  ,""  ,""  ,""  ,""  ,""  ,""  ,""
};
const char *msm_sig_cmp[32]={
    /* BeiDou: ref [15] table 3.5-106 */
    ""  ,"1I","1Q","1X",""  ,""  ,""  ,"6I","6Q","6X",""  ,""  ,
    ""  ,"7I","7Q","7X",""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,
    ""  ,""  ,""  ,""  ,""  ,""  ,""  ,""
};

/* constants -----------------------------------------------------------------*/

#define POLYCRC32   0xEDB88320u /* CRC32 polynomial */
#define POLYCRC24Q  0x1864CFBu  /* CRC24Q polynomial */

const static double gpst0[]={1980,1, 6,0,0,0}; /* gps time reference */
const static double gst0 []={1999,8,22,0,0,0}; /* galileo system time reference */
const static double bdt0 []={2006,1, 1,0,0,0}; /* beidou time reference */

static double leaps[MAXLEAPS+1][7]={ /* leap seconds (y,m,d,h,m,s,utc-gpst) */
    {2017,1,1,0,0,0,-18},
    {2015,7,1,0,0,0,-17},
    {2012,7,1,0,0,0,-16},
    {2009,1,1,0,0,0,-15},
    {2006,1,1,0,0,0,-14},
    {1999,1,1,0,0,0,-13},
    {1997,7,1,0,0,0,-12},
    {1996,1,1,0,0,0,-11},
    {1994,7,1,0,0,0,-10},
    {1993,7,1,0,0,0, -9},
    {1992,7,1,0,0,0, -8},
    {1991,1,1,0,0,0, -7},
    {1990,1,1,0,0,0, -6},
    {1988,1,1,0,0,0, -5},
    {1985,7,1,0,0,0, -4},
    {1983,7,1,0,0,0, -3},
    {1982,7,1,0,0,0, -2},
    {1981,7,1,0,0,0, -1},
    {0}
};
const double chisqr[100]={      /* chi-sqr(n) (alpha=0.001) */
    10.8,13.8,16.3,18.5,20.5,22.5,24.3,26.1,27.9,29.6,
    31.3,32.9,34.5,36.1,37.7,39.3,40.8,42.3,43.8,45.3,
    46.8,48.3,49.7,51.2,52.6,54.1,55.5,56.9,58.3,59.7,
    61.1,62.5,63.9,65.2,66.6,68.0,69.3,70.7,72.1,73.4,
    74.7,76.0,77.3,78.6,80.0,81.3,82.6,84.0,85.4,86.7,
    88.0,89.3,90.6,91.9,93.3,94.7,96.0,97.4,98.7,100 ,
    101 ,102 ,103 ,104 ,105 ,107 ,108 ,109 ,110 ,112 ,
    113 ,114 ,115 ,116 ,118 ,119 ,120 ,122 ,123 ,125 ,
    126 ,127 ,128 ,129 ,131 ,132 ,133 ,134 ,135 ,137 ,
    138 ,139 ,140 ,142 ,143 ,144 ,145 ,147 ,148 ,149
};
const double lam_carr[]={       /* carrier wave length (m) */
    CLIGHT/FREQ1,CLIGHT/FREQ2,CLIGHT/FREQ5,CLIGHT/FREQ6,CLIGHT/FREQ7,CLIGHT/FREQ8
};
const prcopt_t prcopt_default={ /* defaults processing options */
    PMODE_SINGLE,0,2,SYS_GPS,   /* mode,soltype,nf,navsys */
    15.0*D2R,{{0,0}},           /* elmin,snrmask */
    0,1,1,1,                    /* sateph,modear,glomodear,bdsmodear */
    5,0,10,                     /* glomodear,maxout,minlock,minfix */
    0,0,0,0,                    /* estion,esttrop,dynamics,tidecorr */
    1,0,0,0,0,                  /* niter,codesmooth,intpref,sbascorr,sbassatsel */
    0,0,                        /* rovpos,refpos */
    {100.0,100.0},              /* eratio[] */
    {100.0,0.003,0.003,0.0,1.0}, /* err[] */
    {30.0,0.03,0.3},            /* std[] */
    {1E-4,1E-3,1E-4,1E-1,1E-2}, /* prn[] */
    5E-12,                      /* sclkstab */
    {3.0,0.9999,0.20},          /* thresar */
    0.0,0.0,0.05,               /* elmaskar,almaskhold,thresslip */
    30.0,30.0,30.0,             /* maxtdif,maxinno,maxgdop */
    {0},{0},{0},                /* baseline,ru,rb */
    {"",""},                    /* anttype */
    {{0}},{{0}},{0}             /* antdel,pcv,exsats */
};
const solopt_t solopt_default={ /* defaults solution output options */
    SOLF_LLH,TIMES_GPST,1,3,    /* posf,times,timef,timeu */
    0,1,0,0,0,0,                /* degf,outhead,outopt,datum,height,geoid */
    0,0,0,                      /* solstatic,sstat,trace */
    {0.0,0.0},                  /* nmeaintv */
    " ",""                      /* separator/program name */
};
const char *formatstrs[]={      /* stream format strings */
    "RTCM 2",                   /*  0 */
    "RTCM 3",                   /*  1 */
    "NovAtel OEM6",             /*  2 */
    "NovAtel OEM3",             /*  3 */
    "u-blox",                   /*  4 */
    "Superstar II",             /*  5 */
    "Hemisphere",               /*  6 */
    "SkyTraq",                  /*  7 */
    "GW10",                     /*  8 */
    "Javad",                    /*  9 */
    "NVS BINR",                 /* 10 */
    "BINEX",                    /* 11 */
    "Trimble RT17",             /* 12 */
    "LEX Receiver",             /* 13 */
    "Septentrio",               /* 14 */
    "RINEX",                    /* 15 */
    "SP3",                      /* 16 */
    "RINEX CLK",                /* 17 */
    "SBAS",                     /* 18 */
    "NMEA 0183",                /* 19 */
    NULL
};
static char *obscodes[]={       /* observation code strings */
    
    ""  ,"1C","1P","1W","1Y", "1M","1N","1S","1L","1E", /*  0- 9 */
    "1A","1B","1X","1Z","2C", "2D","2S","2L","2X","2P", /* 10-19 */
    "2W","2Y","2M","2N","5I", "5Q","5X","7I","7Q","7X", /* 20-29 */
    "6A","6B","6C","6X","6Z", "6S","6L","8L","8Q","8X", /* 30-39 */
    "2I","2Q","6I","6Q","3I", "3Q","3X","1I","1Q",""    /* 40-49 */
};
static unsigned char obsfreqs[]={ /* 1:L1,2:L2,3:L5,4:L6,5:L7,6:L8,7:L3 */
    
    0, 1, 1, 1, 1,  1, 1, 1, 1, 1, /*  0- 9 */
    1, 1, 1, 1, 2,  2, 2, 2, 2, 2, /* 10-19 */
    2, 2, 2, 2, 3,  3, 3, 5, 5, 5, /* 20-29 */
    4, 4, 4, 4, 4,  4, 4, 6, 6, 6, /* 30-39 */
    2, 2, 4, 4, 3,  3, 3, 1, 1, 0  /* 40-49 */
};
static char codepris[6][MAXFREQ][16]={  /* code priority table */
   
   /* L1,G1E1a   L2,G2,B1     L5,G3,E5a L6,LEX,B3 E5a,B2    E5a+b */
    {"CPYWMNSL","PYWCMNDSLX","IQX"     ,""       ,""       ,""   }, /* GPS */
    {"PC"      ,"PC"        ,"IQX"     ,""       ,""       ,""   }, /* GLO */
    {"CABXZ"   ,""          ,"IQX"     ,"ABCXZ"  ,"IQX"    ,"IQX"}, /* GAL */
    {"CSLXZ"   ,"SLX"       ,"IQX"     ,"SLX"    ,""       ,""   }, /* QZS */
    {"C"       ,""          ,"IQX"     ,""       ,""       ,""   }, /* SBS */
    {"IQX"     ,"IQX"       ,"IQX"     ,"IQX"    ,"IQX"    ,""   }  /* BDS */
};
/* crc tables generated by util/gencrc ---------------------------------------*/
static const unsigned short tbl_CRC16[]={
    0x0000,0x1021,0x2042,0x3063,0x4084,0x50A5,0x60C6,0x70E7,
    0x8108,0x9129,0xA14A,0xB16B,0xC18C,0xD1AD,0xE1CE,0xF1EF,
    0x1231,0x0210,0x3273,0x2252,0x52B5,0x4294,0x72F7,0x62D6,
    0x9339,0x8318,0xB37B,0xA35A,0xD3BD,0xC39C,0xF3FF,0xE3DE,
    0x2462,0x3443,0x0420,0x1401,0x64E6,0x74C7,0x44A4,0x5485,
    0xA56A,0xB54B,0x8528,0x9509,0xE5EE,0xF5CF,0xC5AC,0xD58D,
    0x3653,0x2672,0x1611,0x0630,0x76D7,0x66F6,0x5695,0x46B4,
    0xB75B,0xA77A,0x9719,0x8738,0xF7DF,0xE7FE,0xD79D,0xC7BC,
    0x48C4,0x58E5,0x6886,0x78A7,0x0840,0x1861,0x2802,0x3823,
    0xC9CC,0xD9ED,0xE98E,0xF9AF,0x8948,0x9969,0xA90A,0xB92B,
    0x5AF5,0x4AD4,0x7AB7,0x6A96,0x1A71,0x0A50,0x3A33,0x2A12,
    0xDBFD,0xCBDC,0xFBBF,0xEB9E,0x9B79,0x8B58,0xBB3B,0xAB1A,
    0x6CA6,0x7C87,0x4CE4,0x5CC5,0x2C22,0x3C03,0x0C60,0x1C41,
    0xEDAE,0xFD8F,0xCDEC,0xDDCD,0xAD2A,0xBD0B,0x8D68,0x9D49,
    0x7E97,0x6EB6,0x5ED5,0x4EF4,0x3E13,0x2E32,0x1E51,0x0E70,
    0xFF9F,0xEFBE,0xDFDD,0xCFFC,0xBF1B,0xAF3A,0x9F59,0x8F78,
    0x9188,0x81A9,0xB1CA,0xA1EB,0xD10C,0xC12D,0xF14E,0xE16F,
    0x1080,0x00A1,0x30C2,0x20E3,0x5004,0x4025,0x7046,0x6067,
    0x83B9,0x9398,0xA3FB,0xB3DA,0xC33D,0xD31C,0xE37F,0xF35E,
    0x02B1,0x1290,0x22F3,0x32D2,0x4235,0x5214,0x6277,0x7256,
    0xB5EA,0xA5CB,0x95A8,0x8589,0xF56E,0xE54F,0xD52C,0xC50D,
    0x34E2,0x24C3,0x14A0,0x0481,0x7466,0x6447,0x5424,0x4405,
    0xA7DB,0xB7FA,0x8799,0x97B8,0xE75F,0xF77E,0xC71D,0xD73C,
    0x26D3,0x36F2,0x0691,0x16B0,0x6657,0x7676,0x4615,0x5634,
    0xD94C,0xC96D,0xF90E,0xE92F,0x99C8,0x89E9,0xB98A,0xA9AB,
    0x5844,0x4865,0x7806,0x6827,0x18C0,0x08E1,0x3882,0x28A3,
    0xCB7D,0xDB5C,0xEB3F,0xFB1E,0x8BF9,0x9BD8,0xABBB,0xBB9A,
    0x4A75,0x5A54,0x6A37,0x7A16,0x0AF1,0x1AD0,0x2AB3,0x3A92,
    0xFD2E,0xED0F,0xDD6C,0xCD4D,0xBDAA,0xAD8B,0x9DE8,0x8DC9,
    0x7C26,0x6C07,0x5C64,0x4C45,0x3CA2,0x2C83,0x1CE0,0x0CC1,
    0xEF1F,0xFF3E,0xCF5D,0xDF7C,0xAF9B,0xBFBA,0x8FD9,0x9FF8,
    0x6E17,0x7E36,0x4E55,0x5E74,0x2E93,0x3EB2,0x0ED1,0x1EF0
};
//static const unsigned int tbl_CRC24Q[]={
//    0x000000,0x864CFB,0x8AD50D,0x0C99F6,0x93E6E1,0x15AA1A,0x1933EC,0x9F7F17,
//    0xA18139,0x27CDC2,0x2B5434,0xAD18CF,0x3267D8,0xB42B23,0xB8B2D5,0x3EFE2E,
//    0xC54E89,0x430272,0x4F9B84,0xC9D77F,0x56A868,0xD0E493,0xDC7D65,0x5A319E,
//    0x64CFB0,0xE2834B,0xEE1ABD,0x685646,0xF72951,0x7165AA,0x7DFC5C,0xFBB0A7,
//    0x0CD1E9,0x8A9D12,0x8604E4,0x00481F,0x9F3708,0x197BF3,0x15E205,0x93AEFE,
//    0xAD50D0,0x2B1C2B,0x2785DD,0xA1C926,0x3EB631,0xB8FACA,0xB4633C,0x322FC7,
//    0xC99F60,0x4FD39B,0x434A6D,0xC50696,0x5A7981,0xDC357A,0xD0AC8C,0x56E077,
//    0x681E59,0xEE52A2,0xE2CB54,0x6487AF,0xFBF8B8,0x7DB443,0x712DB5,0xF7614E,
//    0x19A3D2,0x9FEF29,0x9376DF,0x153A24,0x8A4533,0x0C09C8,0x00903E,0x86DCC5,
//    0xB822EB,0x3E6E10,0x32F7E6,0xB4BB1D,0x2BC40A,0xAD88F1,0xA11107,0x275DFC,
//    0xDCED5B,0x5AA1A0,0x563856,0xD074AD,0x4F0BBA,0xC94741,0xC5DEB7,0x43924C,
//    0x7D6C62,0xFB2099,0xF7B96F,0x71F594,0xEE8A83,0x68C678,0x645F8E,0xE21375,
//    0x15723B,0x933EC0,0x9FA736,0x19EBCD,0x8694DA,0x00D821,0x0C41D7,0x8A0D2C,
//    0xB4F302,0x32BFF9,0x3E260F,0xB86AF4,0x2715E3,0xA15918,0xADC0EE,0x2B8C15,
//    0xD03CB2,0x567049,0x5AE9BF,0xDCA544,0x43DA53,0xC596A8,0xC90F5E,0x4F43A5,
//    0x71BD8B,0xF7F170,0xFB6886,0x7D247D,0xE25B6A,0x641791,0x688E67,0xEEC29C,
//    0x3347A4,0xB50B5F,0xB992A9,0x3FDE52,0xA0A145,0x26EDBE,0x2A7448,0xAC38B3,
//    0x92C69D,0x148A66,0x181390,0x9E5F6B,0x01207C,0x876C87,0x8BF571,0x0DB98A,
//    0xF6092D,0x7045D6,0x7CDC20,0xFA90DB,0x65EFCC,0xE3A337,0xEF3AC1,0x69763A,
//    0x578814,0xD1C4EF,0xDD5D19,0x5B11E2,0xC46EF5,0x42220E,0x4EBBF8,0xC8F703,
//    0x3F964D,0xB9DAB6,0xB54340,0x330FBB,0xAC70AC,0x2A3C57,0x26A5A1,0xA0E95A,
//    0x9E1774,0x185B8F,0x14C279,0x928E82,0x0DF195,0x8BBD6E,0x872498,0x016863,
//    0xFAD8C4,0x7C943F,0x700DC9,0xF64132,0x693E25,0xEF72DE,0xE3EB28,0x65A7D3,
//    0x5B59FD,0xDD1506,0xD18CF0,0x57C00B,0xC8BF1C,0x4EF3E7,0x426A11,0xC426EA,
//    0x2AE476,0xACA88D,0xA0317B,0x267D80,0xB90297,0x3F4E6C,0x33D79A,0xB59B61,
//    0x8B654F,0x0D29B4,0x01B042,0x87FCB9,0x1883AE,0x9ECF55,0x9256A3,0x141A58,
//    0xEFAAFF,0x69E604,0x657FF2,0xE33309,0x7C4C1E,0xFA00E5,0xF69913,0x70D5E8,
//    0x4E2BC6,0xC8673D,0xC4FECB,0x42B230,0xDDCD27,0x5B81DC,0x57182A,0xD154D1,
//    0x26359F,0xA07964,0xACE092,0x2AAC69,0xB5D37E,0x339F85,0x3F0673,0xB94A88,
//    0x87B4A6,0x01F85D,0x0D61AB,0x8B2D50,0x145247,0x921EBC,0x9E874A,0x18CBB1,
//    0xE37B16,0x6537ED,0x69AE1B,0xEFE2E0,0x709DF7,0xF6D10C,0xFA48FA,0x7C0401,
//    0x42FA2F,0xC4B6D4,0xC82F22,0x4E63D9,0xD11CCE,0x575035,0x5BC9C3,0xDD8538
//};

/* extract unsigned/signed bits ------------------------------------------------
* extract unsigned/signed bits from byte data
* args   : unsigned char *buff I byte data
*          int    pos    I      bit position from start of data (bits)
*          int    len    I      bit length (bits) (len<=32)
* return : extracted unsigned/signed bits
*-----------------------------------------------------------------------------*/
extern unsigned int getbitu(const unsigned char *buff, int pos, int len)
{
    unsigned int bits=0;
    int i;
    for (i=pos;i<pos+len;i++) bits=(bits<<1)+((buff[i/8]>>(7-i%8))&1u);
    return bits;
}
extern int getbits(const unsigned char *buff, int pos, int len)
{
    unsigned int bits=getbitu(buff,pos,len);
    if (len<=0||32<=len||!(bits&(1u<<(len-1)))) return (int)bits;
    return (int)(bits|(~0u<<len)); /* extend sign */
}
extern void traceopen(const char *file) {}
extern void traceclose(void) {}
extern void tracelevel(int level) {}
extern void trace   (int level, const char *format, ...) {}
extern void tracet  (int level, const char *format, ...) {}
extern void tracemat(int level, const double *A, int n, int m, int p, int q) {}
extern void traceobs(int level, const obsd_t *obs, int n) {}
extern void tracenav(int level, const nav_t *nav) {}
extern void tracegnav(int level, const nav_t *nav) {}
extern void tracehnav(int level, const nav_t *nav) {}
extern void tracepeph(int level, const nav_t *nav) {}
extern void tracepclk(int level, const nav_t *nav) {}
extern void traceb  (int level, const unsigned char *p, int n) {}

/* convert calendar day/time to time -------------------------------------------
* convert calendar day/time to gtime_t struct
* args   : double *ep       I   day/time {year,month,day,hour,min,sec}
* return : gtime_t struct
* notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
*-----------------------------------------------------------------------------*/
extern gtime_t epoch2time(const double *ep)
{
    const int doy[]={1,32,60,91,121,152,182,213,244,274,305,335};
    gtime_t time={0};
    int days,sec,year=(int)ep[0],mon=(int)ep[1],day=(int)ep[2];
    
    if (year<1970||2099<year||mon<1||12<mon) return time;
    
    /* leap year if year%4==0 in 1901-2099 */
    days=(year-1970)*365+(year-1969)/4+doy[mon-1]+day-2+(year%4==0&&mon>=3?1:0);
    sec=(int)floor(ep[5]);
    time.time=(time_t)days*86400+(int)ep[3]*3600+(int)ep[4]*60+sec;
    time.sec=ep[5]-sec;
    return time;
}

/* time to gps time ------------------------------------------------------------
* convert gtime_t struct to week and tow in gps time
* args   : gtime_t t        I   gtime_t struct
*          int    *week     IO  week number in gps time (NULL: no output)
* return : time of week in gps time (s)
*-----------------------------------------------------------------------------*/
extern double time2gpst(gtime_t t, int *week)
{
    gtime_t t0=epoch2time(gpst0);
    time_t sec=t.time-t0.time;
    int w=(int)(sec/(86400*7));
    
    if (week) *week=w;
    return (double)(sec-w*86400*7)+t.sec;
}
/* time difference -------------------------------------------------------------
* difference between gtime_t structs
* args   : gtime_t t1,t2    I   gtime_t structs
* return : time difference (t1-t2) (s)
*-----------------------------------------------------------------------------*/
extern double timediff(gtime_t t1, gtime_t t2)
{
    return difftime(t1.time,t2.time)+t1.sec-t2.sec;
}
/* add time --------------------------------------------------------------------
* add time to gtime_t struct
* args   : gtime_t t        I   gtime_t struct
*          double sec       I   time to add (s)
* return : gtime_t struct (t+sec)
*-----------------------------------------------------------------------------*/
extern gtime_t timeadd(gtime_t t, double sec)
{
    double tt;
    
    t.sec+=sec; tt=floor(t.sec); t.time+=(int)tt; t.sec-=tt;
    return t;
}
/* utc to gpstime --------------------------------------------------------------
* convert utc to gpstime considering leap seconds
* args   : gtime_t t        I   time expressed in utc
* return : time expressed in gpstime
* notes  : ignore slight time offset under 100 ns
*-----------------------------------------------------------------------------*/
extern gtime_t utc2gpst(gtime_t t)
{
    int i;
    
    for (i=0;leaps[i][0]>0;i++) {
        if (timediff(t,epoch2time(leaps[i]))>=0.0) return timeadd(t,-leaps[i][6]);
    }
    return t;
}
/* get current time in utc -----------------------------------------------------
* get current time in utc
* args   : none
* return : current time in utc
*-----------------------------------------------------------------------------*/
static double timeoffset_=0.0;        /* time offset (s) */
extern gtime_t timeget(void)
{
    double ep[6]={0};

    struct timeval tv; // TODO:fix in esp32
    struct tm *tt;
    
    if (!gettimeofday(&tv,NULL)&&(tt=gmtime(&tv.tv_sec))) {
        ep[0]=tt->tm_year+1900; ep[1]=tt->tm_mon+1; ep[2]=tt->tm_mday;
        ep[3]=tt->tm_hour; ep[4]=tt->tm_min; ep[5]=tt->tm_sec+tv.tv_usec*1E-6;
    }

    return timeadd(epoch2time(ep),timeoffset_);
}

/* gps time to time ------------------------------------------------------------
* convert week and tow in gps time to gtime_t struct
* args   : int    week      I   week number in gps time
*          double sec       I   time of week in gps time (s)
* return : gtime_t struct
*-----------------------------------------------------------------------------*/
extern gtime_t gpst2time(int week, double sec)
{
    gtime_t t=epoch2time(gpst0);
    
    if (sec<-1E9||1E9<sec) sec=0.0;
    t.time+=86400*7*week+(int)sec;
    t.sec=sec-(int)sec;
    return t;
}
/* time to string --------------------------------------------------------------
* convert gtime_t struct to string
* args   : gtime_t t        I   gtime_t struct
*          char   *s        O   string ("yyyy/mm/dd hh:mm:ss.ssss")
*          int    n         I   number of decimals
* return : none
*-----------------------------------------------------------------------------*/
extern void time2str(gtime_t t, char *s, int n)
{
    double ep[6];
    
    if (n<0) n=0; else if (n>12) n=12;
    if (1.0-t.sec<0.5/pow(10.0,n)) {t.time++; t.sec=0.0;};
    time2epoch(t,ep);
    sprintf(s,"%04.0f/%02.0f/%02.0f %02.0f:%02.0f:%0*.*f",ep[0],ep[1],ep[2],
            ep[3],ep[4],n<=0?2:n+3,n<=0?0:n,ep[5]);
}
/* get time string -------------------------------------------------------------
* get time string
* args   : gtime_t t        I   gtime_t struct
*          int    n         I   number of decimals
* return : time string
* notes  : not reentrant, do not use multiple in a function
*-----------------------------------------------------------------------------*/
extern char *time_str(gtime_t t, int n)
{
    static char buff[64];
    time2str(t,buff,n);
    return buff;
}

/* satellite system+prn/slot number to satellite number ------------------------
* convert satellite system+prn/slot number to satellite number
* args   : int    sys       I   satellite system (SYS_GPS,SYS_GLO,...)
*          int    prn       I   satellite prn/slot number
* return : satellite number (0:error)
*-----------------------------------------------------------------------------*/
extern int satno(int sys, int prn)
{
    if (prn<=0) return 0;
    switch (sys) {
        case SYS_GPS:
            if (prn<MINPRNGPS||MAXPRNGPS<prn) return 0;
            return prn-MINPRNGPS+1;
        case SYS_GLO:
            if (prn<MINPRNGLO||MAXPRNGLO<prn) return 0;
            return NSATGPS+prn-MINPRNGLO+1;
        case SYS_GAL:
            if (prn<MINPRNGAL||MAXPRNGAL<prn) return 0;
            return NSATGPS+NSATGLO+prn-MINPRNGAL+1;
        case SYS_QZS:
            if (prn<MINPRNQZS||MAXPRNQZS<prn) return 0;
            return NSATGPS+NSATGLO+NSATGAL+prn-MINPRNQZS+1;
        case SYS_CMP:
            if (prn<MINPRNCMP||MAXPRNCMP<prn) return 0;
            return NSATGPS+NSATGLO+NSATGAL+NSATQZS+prn-MINPRNCMP+1;
        case SYS_LEO:
            if (prn<MINPRNLEO||MAXPRNLEO<prn) return 0;
            return NSATGPS+NSATGLO+NSATGAL+NSATQZS+NSATCMP+prn-MINPRNLEO+1;
        case SYS_SBS:
            if (prn<MINPRNSBS||MAXPRNSBS<prn) return 0;
            return NSATGPS+NSATGLO+NSATGAL+NSATQZS+NSATCMP+NSATLEO+prn-MINPRNSBS+1;
    }
    return 0;
}

/* time to calendar day/time ---------------------------------------------------
* convert gtime_t struct to calendar day/time
* args   : gtime_t t        I   gtime_t struct
*          double *ep       O   day/time {year,month,day,hour,min,sec}
* return : none
* notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
*-----------------------------------------------------------------------------*/
extern void time2epoch(gtime_t t, double *ep)
{
    const int mday[]={ /* # of days in a month */
        31,28,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31,
        31,29,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31
    };
    int days,sec,mon,day;
    
    /* leap year if year%4==0 in 1901-2099 */
    days=(int)(t.time/86400);
    sec=(int)(t.time-(time_t)days*86400);
    for (day=days%1461,mon=0;mon<48;mon++) {
        if (day>=mday[mon]) day-=mday[mon]; else break;
    }
    ep[0]=1970+days/1461*4+mon/12; ep[1]=mon%12+1; ep[2]=day+1;
    ep[3]=sec/3600; ep[4]=sec%3600/60; ep[5]=sec%60+t.sec;
}

/* gpstime to utc --------------------------------------------------------------
* convert gpstime to utc considering leap seconds
* args   : gtime_t t        I   time expressed in gpstime
* return : time expressed in utc
* notes  : ignore slight time offset under 100 ns
*-----------------------------------------------------------------------------*/
extern gtime_t gpst2utc(gtime_t t)
{
    gtime_t tu;
    int i;
    
    for (i=0;leaps[i][0]>0;i++) {
        tu=timeadd(t,leaps[i][6]);
        if (timediff(tu,epoch2time(leaps[i]))>=0.0) return tu;
    }
    return t;
}
/* adjust gps week number ------------------------------------------------------
* adjust gps week number using cpu time
* args   : int   week       I   not-adjusted gps week number
* return : adjusted gps week number
*-----------------------------------------------------------------------------*/
extern int adjgpsweek(int week)
{
    int w;
    (void)time2gpst(utc2gpst(timeget()),&w);
    if (w<1560) w=1560; /* use 2009/12/1 if time is earlier than 2009/12/1 */
    return week+(w-week+512)/1024*1024;
}

/* time to beidouo time (bdt) --------------------------------------------------
* convert gtime_t struct to week and tow in beidou time (bdt)
* args   : gtime_t t        I   gtime_t struct
*          int    *week     IO  week number in bdt (NULL: no output)
* return : time of week in bdt (s)
*-----------------------------------------------------------------------------*/
extern double time2bdt(gtime_t t, int *week)
{
    gtime_t t0=epoch2time(bdt0);
    time_t sec=t.time-t0.time;
    int w=(int)(sec/(86400*7));
    
    if (week) *week=w;
    return (double)(sec-w*86400*7)+t.sec;
}
/* gpstime to bdt --------------------------------------------------------------
* convert gpstime to bdt (beidou navigation satellite system time)
* args   : gtime_t t        I   time expressed in gpstime
* return : time expressed in bdt
* notes  : ref [8] 3.3, 2006/1/1 00:00 BDT = 2006/1/1 00:00 UTC
*          no leap seconds in BDT
*          ignore slight time offset under 100 ns
*-----------------------------------------------------------------------------*/
extern gtime_t gpst2bdt(gtime_t t)
{
    return timeadd(t,-14.0);
}
/* obs type string to obs code -------------------------------------------------
* convert obs code type string to obs code
* args   : char   *str   I      obs code string ("1C","1P","1Y",...)
*          int    *freq  IO     frequency (1:L1,2:L2,3:L5,4:L6,5:L7,6:L8,0:err)
*                               (NULL: no output)
* return : obs code (CODE_???)
* notes  : obs codes are based on reference [6] and qzss extension
*-----------------------------------------------------------------------------*/
extern unsigned char obs2code(const char *obs, int *freq)
{
    int i;
    if (freq) *freq=0;
    for (i=1;*obscodes[i];i++) {
        if (strcmp(obscodes[i],obs)) continue;
        if (freq) *freq=obsfreqs[i];
        return (unsigned char)i;
    }
    return CODE_NONE;
}
/* obs code to obs code string -------------------------------------------------
* convert obs code to obs code string
* args   : unsigned char code I obs code (CODE_???)
*          int    *freq  IO     frequency (1:L1,2:L2,3:L5,4:L6,5:L7,6:L8,0:err)
*                               (NULL: no output)
* return : obs code string ("1C","1P","1P",...)
* notes  : obs codes are based on reference [6] and qzss extension
*-----------------------------------------------------------------------------*/
extern char *code2obs(unsigned char code, int *freq)
{
    if (freq) *freq=0;
    if (code<=CODE_NONE||MAXCODE<code) return "";
    if (freq) *freq=obsfreqs[code];
    return obscodes[code];
}
/* get code priority -----------------------------------------------------------
* get code priority for multiple codes in a frequency
* args   : int    sys     I     system (SYS_???)
*          unsigned char code I obs code (CODE_???)
*          char   *opt    I     code options (NULL:no option)
* return : priority (15:highest-1:lowest,0:error)
*-----------------------------------------------------------------------------*/
extern int getcodepri(int sys, unsigned char code, const char *opt)
{
    const char *p,*optstr;
    char *obs,str[8]="";
    int i,j;
    
    switch (sys) {
        case SYS_GPS: i=0; optstr="-GL%2s"; break;
        case SYS_GLO: i=1; optstr="-RL%2s"; break;
        case SYS_GAL: i=2; optstr="-EL%2s"; break;
        case SYS_QZS: i=3; optstr="-JL%2s"; break;
        case SYS_SBS: i=4; optstr="-SL%2s"; break;
        case SYS_CMP: i=5; optstr="-CL%2s"; break;
        default: return 0;
    }
    obs=code2obs(code,&j);
    
    /* parse code options */
    for (p=opt;p&&(p=strchr(p,'-'));p++) {
        if (sscanf(p,optstr,str)<1||str[0]!=obs[0]) continue;
        return str[1]==obs[1]?15:0;
    }
    /* search code priority */
    return (p=strchr(codepris[i][j-1],obs[1]))?14-(int)(p-codepris[i][j-1]):0;
}
/* satellite number to satellite system ----------------------------------------
* convert satellite number to satellite system
* args   : int    sat       I   satellite number (1-MAXSAT)
*          int    *prn      IO  satellite prn/slot number (NULL: no output)
* return : satellite system (SYS_GPS,SYS_GLO,...)
*-----------------------------------------------------------------------------*/
extern int satsys(int sat, int *prn)
{
    int sys=SYS_NONE;
    if (sat<=0||MAXSAT<sat) sat=0;
    else if (sat<=NSATGPS) {
        sys=SYS_GPS; sat+=MINPRNGPS-1;
    }
    else if ((sat-=NSATGPS)<=NSATGLO) {
        sys=SYS_GLO; sat+=MINPRNGLO-1;
    }
    else if ((sat-=NSATGLO)<=NSATGAL) {
        sys=SYS_GAL; sat+=MINPRNGAL-1;
    }
    else if ((sat-=NSATGAL)<=NSATQZS) {
        sys=SYS_QZS; sat+=MINPRNQZS-1; 
    }
    else if ((sat-=NSATQZS)<=NSATCMP) {
        sys=SYS_CMP; sat+=MINPRNCMP-1; 
    }
    else if ((sat-=NSATCMP)<=NSATLEO) {
        sys=SYS_LEO; sat+=MINPRNLEO-1; 
    }
    else if ((sat-=NSATLEO)<=NSATSBS) {
        sys=SYS_SBS; sat+=MINPRNSBS-1; 
    }
    else sat=0;
    if (prn) *prn=sat;
    return sys;
}
/* satellite carrier wave length -----------------------------------------------
* get satellite carrier wave lengths
* args   : int    sat       I   satellite number
*          int    frq       I   frequency index (0:L1,1:L2,2:L5/3,...)
*          nav_t  *nav      I   navigation messages
* return : carrier wave length (m) (0.0: error)
*-----------------------------------------------------------------------------*/
extern double satwavelen(int sat, int frq, const nav_t *nav)
{
    const double freq_glo[]={FREQ1_GLO,FREQ2_GLO,FREQ3_GLO};
    const double dfrq_glo[]={DFRQ1_GLO,DFRQ2_GLO,0.0};
    int i,sys=satsys(sat,NULL);
    
    if (sys==SYS_GLO) {
        if (0<=frq&&frq<=2) {
            for (i=0;i<nav->ng;i++) {
                if (nav->geph[i].sat!=sat) continue;
                return CLIGHT/(freq_glo[frq]+dfrq_glo[frq]*nav->geph[i].frq);
            }
        }
    }
    else if (sys==SYS_CMP) {
        if      (frq==0) return CLIGHT/FREQ1_CMP; /* B1 */
        else if (frq==1) return CLIGHT/FREQ2_CMP; /* B3 */
        else if (frq==2) return CLIGHT/FREQ3_CMP; /* B2 */
    }
    else {
        if      (frq==0) return CLIGHT/FREQ1; /* L1/E1 */
        else if (frq==1) return CLIGHT/FREQ2; /* L2 */
        else if (frq==2) return CLIGHT/FREQ5; /* L5/E5a */
        else if (frq==3) return CLIGHT/FREQ6; /* L6/LEX */
        else if (frq==4) return CLIGHT/FREQ7; /* E5b */
        else if (frq==5) return CLIGHT/FREQ8; /* E5a+b */
    }
    return 0.0;
}
/* bdt to gpstime --------------------------------------------------------------
* convert bdt (beidou navigation satellite system time) to gpstime
* args   : gtime_t t        I   time expressed in bdt
* return : time expressed in gpstime
* notes  : see gpst2bdt()
*-----------------------------------------------------------------------------*/
extern gtime_t bdt2gpst(gtime_t t)
{
    return timeadd(t,14.0);
}
/* beidou time (bdt) to time ---------------------------------------------------
* convert week and tow in beidou time (bdt) to gtime_t struct
* args   : int    week      I   week number in bdt
*          double sec       I   time of week in bdt (s)
* return : gtime_t struct
*-----------------------------------------------------------------------------*/
extern gtime_t bdt2time(int week, double sec)
{
    gtime_t t=epoch2time(bdt0);
    
    if (sec<-1E9||1E9<sec) sec=0.0;
    t.time+=86400*7*week+(int)sec;
    t.sec=sec-(int)sec;
    return t;
}
/*########################################################################### */
/*########################################################################### */
/*########################################################################### */
/*########################################################################### */
/*########################################################################### */
/*########################################################################### */
/*########################################################################### */

/* ssr update intervals ------------------------------------------------------*/
static const double ssrudint[16]={
    1,2,5,10,15,30,60,120,240,300,600,900,1800,3600,7200,10800
};
/* get sign-magnitude bits ---------------------------------------------------*/
static double getbitg(const unsigned char *buff, int pos, int len)
{
    double value=getbitu(buff,pos+1,len-1);
    return getbitu(buff,pos,1)?-value:value;
}
/* adjust weekly rollover of gps time ----------------------------------------*/
static void adjweek(rtcm_t *rtcm, double tow)
{
    double tow_p;
    int week;
    
    /* if no time, get cpu time */
    if (rtcm->time.time==0) rtcm->time=utc2gpst(timeget());
    tow_p=time2gpst(rtcm->time,&week);
    if      (tow<tow_p-302400.0) tow+=604800.0;
    else if (tow>tow_p+302400.0) tow-=604800.0;
    rtcm->time=gpst2time(week,tow);
}
/* adjust weekly rollover of bdt time ----------------------------------------*/
static int adjbdtweek(int week)
{
    int w;
    (void)time2bdt(gpst2bdt(utc2gpst(timeget())),&w);
    if (w<1) w=1; /* use 2006/1/1 if time is earlier than 2006/1/1 */
    return week+(w-week+512)/1024*1024;
}
/* adjust daily rollover of glonass time -------------------------------------*/
static void adjday_glot(rtcm_t *rtcm, double tod)
{
    gtime_t time;
    double tow,tod_p;
    int week;
    
    if (rtcm->time.time==0) rtcm->time=utc2gpst(timeget());
    time=timeadd(gpst2utc(rtcm->time),10800.0); /* glonass time */
    tow=time2gpst(time,&week);
    tod_p=fmod(tow,86400.0); tow-=tod_p;
    if      (tod<tod_p-43200.0) tod+=86400.0;
    else if (tod>tod_p+43200.0) tod-=86400.0;
    time=gpst2time(week,tow+tod);
    rtcm->time=utc2gpst(timeadd(time,-10800.0));
}
/* adjust carrier-phase rollover ---------------------------------------------*/
static double adjcp(rtcm_t *rtcm, int sat, int freq, double cp)
{
    if (rtcm->cp[sat-1][freq]==0.0) ;
    else if (cp<rtcm->cp[sat-1][freq]-750.0) cp+=1500.0;
    else if (cp>rtcm->cp[sat-1][freq]+750.0) cp-=1500.0;
    rtcm->cp[sat-1][freq]=cp;
    return cp;
}
/* loss-of-lock indicator ----------------------------------------------------*/
static int lossoflock(rtcm_t *rtcm, int sat, int freq, int lock)
{
    int lli=(!lock&&!rtcm->lock[sat-1][freq])||lock<rtcm->lock[sat-1][freq];
    rtcm->lock[sat-1][freq]=(unsigned short)lock;
    return lli;
}
/* s/n ratio -----------------------------------------------------------------*/
static unsigned char snratio(double snr)
{
    return (unsigned char)(snr<=0.0||255.5<=snr?0.0:snr*4.0+0.5);
}
/* get observation data index ------------------------------------------------*/
static int obsindex(obs_t *obs, gtime_t time, int sat)
{
    int i,j;
    
    for (i=0;i<obs->n;i++) {
        if (obs->data[i].sat==sat) return i; /* field already exists */
    }
    if (i>=MAXOBS) return -1; /* overflow */
    
    /* add new field */
    obs->data[i].time=time;
    obs->data[i].sat=sat;
    for (j=0;j<NFREQ+NEXOBS;j++) {
        obs->data[i].L[j]=obs->data[i].P[j]=0.0;
        obs->data[i].D[j]=0.0;
        obs->data[i].SNR[j]=obs->data[i].LLI[j]=obs->data[i].code[j]=0;
    }
    obs->n++;
    return i;
}
/* test station id consistency -----------------------------------------------*/
static int test_staid(rtcm_t *rtcm, int staid)
{
    char *p;
    int type,id;
    
    /* test station id option */
    if ((p=strstr(rtcm->opt,"-STA="))&&sscanf(p,"-STA=%d",&id)==1) {
        if (staid!=id) return 0;
    }
    /* save station id */
    if (rtcm->staid==0||rtcm->obsflag) {
        rtcm->staid=staid;
    }
    else if (staid!=rtcm->staid) {
        type=getbitu(rtcm->buff,24,12);
        trace(2,"rtcm3 %d staid invalid id=%d %d\n",type,staid,rtcm->staid);
        
        /* reset station id if station id error */
        rtcm->staid=0;
        return 0;
    }
    return 1;
}
/* decode type 1001-1004 message header --------------------------------------*/
static int decode_head1001(rtcm_t *rtcm, int *sync)
{
    double tow;
    char *msg;
    int i=24,staid,nsat,type;
    
    type=getbitu(rtcm->buff,i,12); i+=12;
    
    if (i+52<=rtcm->len*8) {
        staid=getbitu(rtcm->buff,i,12);       i+=12;
        tow  =getbitu(rtcm->buff,i,30)*0.001; i+=30;
        *sync=getbitu(rtcm->buff,i, 1);       i+= 1;
        nsat =getbitu(rtcm->buff,i, 5);
    }
    else {
        trace(2,"rtcm3 %d length error: len=%d\n",type,rtcm->len);
        return -1;
    }
    /* test station id */
    if (!test_staid(rtcm,staid)) return -1;
    
    adjweek(rtcm,tow);
    
    trace(4,"decode_head1001: time=%s nsat=%d sync=%d\n",time_str(rtcm->time,2),
          nsat,*sync);
    
    if (rtcm->outtype) {
        msg=rtcm->msgtype+strlen(rtcm->msgtype);
        sprintf(msg," staid=%4d %s nsat=%2d sync=%d",staid,
                time_str(rtcm->time,2),nsat,*sync);
    }
    return nsat;
}
/* decode type 1001: L1-only gps rtk observation -----------------------------*/
static int decode_type1001(rtcm_t *rtcm)
{
    int sync;
    if (decode_head1001(rtcm,&sync)<0) return -1;
    rtcm->obsflag=!sync;
    return sync?0:1;
}
/* decode type 1002: extended L1-only gps rtk observables --------------------*/
static int decode_type1002(rtcm_t *rtcm)
{
    double pr1,cnr1,tt,cp1;
    int i=24+64,j,index,nsat,sync,prn,code,sat,ppr1,lock1,amb,sys;
    
    if ((nsat=decode_head1001(rtcm,&sync))<0) return -1;
    
    for (j=0;j<nsat&&rtcm->obs.n<MAXOBS&&i+74<=rtcm->len*8;j++) {
        prn  =getbitu(rtcm->buff,i, 6); i+= 6;
        code =getbitu(rtcm->buff,i, 1); i+= 1;
        pr1  =getbitu(rtcm->buff,i,24); i+=24;
        ppr1 =getbits(rtcm->buff,i,20); i+=20;
        lock1=getbitu(rtcm->buff,i, 7); i+= 7;
        amb  =getbitu(rtcm->buff,i, 8); i+= 8;
        cnr1 =getbitu(rtcm->buff,i, 8); i+= 8;
        if (prn<40) {
            sys=SYS_GPS;
        }
        else {
            sys=SYS_SBS; prn+=80;
        }
        if (!(sat=satno(sys,prn))) {
            trace(2,"rtcm3 1002 satellite number error: prn=%d\n",prn);
            continue;
        }
        tt=timediff(rtcm->obs.data[0].time,rtcm->time);
        if (rtcm->obsflag||fabs(tt)>1E-9) {
            rtcm->obs.n=rtcm->obsflag=0;
        }
        if ((index=obsindex(&rtcm->obs,rtcm->time,sat))<0) continue;
        pr1=pr1*0.02+amb*PRUNIT_GPS;
        if (ppr1!=(int)0xFFF80000) {
            rtcm->obs.data[index].P[0]=pr1;
            cp1=adjcp(rtcm,sat,0,ppr1*0.0005/lam_carr[0]);
            rtcm->obs.data[index].L[0]=pr1/lam_carr[0]+cp1;
        }
        rtcm->obs.data[index].LLI[0]=lossoflock(rtcm,sat,0,lock1);
        rtcm->obs.data[index].SNR[0]=snratio(cnr1*0.25);
        rtcm->obs.data[index].code[0]=code?CODE_L1P:CODE_L1C;
    }
    return sync?0:1;
}
/* decode type 1003: L1&L2 gps rtk observables -------------------------------*/
static int decode_type1003(rtcm_t *rtcm)
{
    int sync;
    if (decode_head1001(rtcm,&sync)<0) return -1;
    rtcm->obsflag=!sync;
    return sync?0:1;
}
/* decode type 1004: extended L1&L2 gps rtk observables ----------------------*/
static int decode_type1004(rtcm_t *rtcm)
{
    const int L2codes[]={CODE_L2X,CODE_L2P,CODE_L2D,CODE_L2W};
    double pr1,cnr1,cnr2,tt,cp1,cp2;
    int i=24+64,j,index,nsat,sync,prn,sat,code1,code2,pr21,ppr1,ppr2;
    int lock1,lock2,amb,sys;
    
    if ((nsat=decode_head1001(rtcm,&sync))<0) return -1;
    
    for (j=0;j<nsat&&rtcm->obs.n<MAXOBS&&i+125<=rtcm->len*8;j++) {
        prn  =getbitu(rtcm->buff,i, 6); i+= 6;
        code1=getbitu(rtcm->buff,i, 1); i+= 1;
        pr1  =getbitu(rtcm->buff,i,24); i+=24;
        ppr1 =getbits(rtcm->buff,i,20); i+=20;
        lock1=getbitu(rtcm->buff,i, 7); i+= 7;
        amb  =getbitu(rtcm->buff,i, 8); i+= 8;
        cnr1 =getbitu(rtcm->buff,i, 8); i+= 8;
        code2=getbitu(rtcm->buff,i, 2); i+= 2;
        pr21 =getbits(rtcm->buff,i,14); i+=14;
        ppr2 =getbits(rtcm->buff,i,20); i+=20;
        lock2=getbitu(rtcm->buff,i, 7); i+= 7;
        cnr2 =getbitu(rtcm->buff,i, 8); i+= 8;
        if (prn<40) {
            sys=SYS_GPS;
        }
        else {
            sys=SYS_SBS; prn+=80;
        }
        if (!(sat=satno(sys,prn))) {
            trace(2,"rtcm3 1004 satellite number error: sys=%d prn=%d\n",sys,prn);
            continue;
        }
        tt=timediff(rtcm->obs.data[0].time,rtcm->time);
        if (rtcm->obsflag||fabs(tt)>1E-9) {
            rtcm->obs.n=rtcm->obsflag=0;
        }
        if ((index=obsindex(&rtcm->obs,rtcm->time,sat))<0) continue;
        pr1=pr1*0.02+amb*PRUNIT_GPS;
        if (ppr1!=(int)0xFFF80000) {
            rtcm->obs.data[index].P[0]=pr1;
            cp1=adjcp(rtcm,sat,0,ppr1*0.0005/lam_carr[0]);
            rtcm->obs.data[index].L[0]=pr1/lam_carr[0]+cp1;
        }
        rtcm->obs.data[index].LLI[0]=lossoflock(rtcm,sat,0,lock1);
        rtcm->obs.data[index].SNR[0]=snratio(cnr1*0.25);
        rtcm->obs.data[index].code[0]=code1?CODE_L1P:CODE_L1C;
        
        if (pr21!=(int)0xFFFFE000) {
            rtcm->obs.data[index].P[1]=pr1+pr21*0.02;
        }
        if (ppr2!=(int)0xFFF80000) {
            cp2=adjcp(rtcm,sat,1,ppr2*0.0005/lam_carr[1]);
            rtcm->obs.data[index].L[1]=pr1/lam_carr[1]+cp2;
        }
        rtcm->obs.data[index].LLI[1]=lossoflock(rtcm,sat,1,lock2);
        rtcm->obs.data[index].SNR[1]=snratio(cnr2*0.25);
        rtcm->obs.data[index].code[1]=L2codes[code2];
    }
    rtcm->obsflag=!sync;
    return sync?0:1;
}
/* get signed 38bit field ----------------------------------------------------*/
static double getbits_38(const unsigned char *buff, int pos)
{
    return (double)getbits(buff,pos,32)*64.0+getbitu(buff,pos+32,6);
}
/* decode type 1005: stationary rtk reference station arp --------------------*/
static int decode_type1005(rtcm_t *rtcm)
{
    double rr[3];
    char *msg;
    int i=24+12,j,staid,itrf;
    
    if (i+140==rtcm->len*8) {
        staid=getbitu(rtcm->buff,i,12); i+=12;
        itrf =getbitu(rtcm->buff,i, 6); i+= 6+4;
        rr[0]=getbits_38(rtcm->buff,i); i+=38+2;
        rr[1]=getbits_38(rtcm->buff,i); i+=38+2;
        rr[2]=getbits_38(rtcm->buff,i);
    }
    else {
        trace(2,"rtcm3 1005 length error: len=%d\n",rtcm->len);
        return -1;
    }
    if (rtcm->outtype) {
        msg=rtcm->msgtype+strlen(rtcm->msgtype);
        sprintf(msg," staid=%4d",staid);
    }
    /* test station id */
    if (!test_staid(rtcm,staid)) return -1;
    
    rtcm->sta.deltype=0; /* xyz */
    for (j=0;j<3;j++) {
        rtcm->sta.pos[j]=rr[j]*0.0001;
        rtcm->sta.del[j]=0.0;
    }
    rtcm->sta.hgt=0.0;
    rtcm->sta.itrf=itrf;
    return 5;
}
/* decode type 1006: stationary rtk reference station arp with height --------*/
static int decode_type1006(rtcm_t *rtcm)
{
    double rr[3],anth;
    char *msg;
    int i=24+12,j,staid,itrf;
    
    if (i+156<=rtcm->len*8) {
        staid=getbitu(rtcm->buff,i,12); i+=12;
        itrf =getbitu(rtcm->buff,i, 6); i+= 6+4;
        rr[0]=getbits_38(rtcm->buff,i); i+=38+2;
        rr[1]=getbits_38(rtcm->buff,i); i+=38+2;
        rr[2]=getbits_38(rtcm->buff,i); i+=38;
        anth =getbitu(rtcm->buff,i,16);
    }
    else {
        trace(2,"rtcm3 1006 length error: len=%d\n",rtcm->len);
        return -1;
    }
    if (rtcm->outtype) {
        msg=rtcm->msgtype+strlen(rtcm->msgtype);
        sprintf(msg," staid=%4d",staid);
    }
    /* test station id */
    if (!test_staid(rtcm,staid)) return -1;
    
    rtcm->sta.deltype=1; /* xyz */
    for (j=0;j<3;j++) {
        rtcm->sta.pos[j]=rr[j]*0.0001;
        rtcm->sta.del[j]=0.0;
    }
    rtcm->sta.hgt=anth*0.0001;
    rtcm->sta.itrf=itrf;
    return 5;
}
/* decode type 1007: antenna descriptor --------------------------------------*/
static int decode_type1007(rtcm_t *rtcm)
{
    char des[32]="";
    char *msg;
    int i=24+12,j,staid,n,setup;
    
    n=getbitu(rtcm->buff,i+12,8);
    
    if (i+28+8*n<=rtcm->len*8) {
        staid=getbitu(rtcm->buff,i,12); i+=12+8;
        for (j=0;j<n&&j<31;j++) {
            des[j]=(char)getbitu(rtcm->buff,i,8); i+=8;
        }
        setup=getbitu(rtcm->buff,i, 8);
    }
    else {
        trace(2,"rtcm3 1007 length error: len=%d\n",rtcm->len);
        return -1;
    }
    if (rtcm->outtype) {
        msg=rtcm->msgtype+strlen(rtcm->msgtype);
        sprintf(msg," staid=%4d",staid);
    }
    /* test station id */
    if (!test_staid(rtcm,staid)) return -1;
    
    strncpy(rtcm->sta.antdes,des,n); rtcm->sta.antdes[n]='\0';
    rtcm->sta.antsetup=setup;
    rtcm->sta.antsno[0]='\0';
    return 5;
}
/* decode type 1008: antenna descriptor & serial number ----------------------*/
static int decode_type1008(rtcm_t *rtcm)
{
    char des[32]="",sno[32]="";
    char *msg;
    int i=24+12,j,staid,n,m,setup;
    
    n=getbitu(rtcm->buff,i+12,8);
    m=getbitu(rtcm->buff,i+28+8*n,8);
    
    if (i+36+8*(n+m)<=rtcm->len*8) {
        staid=getbitu(rtcm->buff,i,12); i+=12+8;
        for (j=0;j<n&&j<31;j++) {
            des[j]=(char)getbitu(rtcm->buff,i,8); i+=8;
        }
        setup=getbitu(rtcm->buff,i, 8); i+=8+8;
        for (j=0;j<m&&j<31;j++) {
            sno[j]=(char)getbitu(rtcm->buff,i,8); i+=8;
        }
    }
    else {
        trace(2,"rtcm3 1008 length error: len=%d\n",rtcm->len);
        return -1;
    }
    if (rtcm->outtype) {
        msg=rtcm->msgtype+strlen(rtcm->msgtype);
        sprintf(msg," staid=%4d",staid);
    }
    /* test station id */
    if (!test_staid(rtcm,staid)) return -1;
    
    strncpy(rtcm->sta.antdes,des,n); rtcm->sta.antdes[n]='\0';
    rtcm->sta.antsetup=setup;
    strncpy(rtcm->sta.antsno,sno,m); rtcm->sta.antsno[m]='\0';
    return 5;
}
/* decode type 1009-1012 message header --------------------------------------*/
static int decode_head1009(rtcm_t *rtcm, int *sync)
{
    double tod;
    char *msg;
    int i=24,staid,nsat,type;
    
    type=getbitu(rtcm->buff,i,12); i+=12;
    
    if (i+49<=rtcm->len*8) {
        staid=getbitu(rtcm->buff,i,12);       i+=12;
        tod  =getbitu(rtcm->buff,i,27)*0.001; i+=27; /* sec in a day */
        *sync=getbitu(rtcm->buff,i, 1);       i+= 1;
        nsat =getbitu(rtcm->buff,i, 5);
    }
    else {
        trace(2,"rtcm3 %d length error: len=%d\n",type,rtcm->len);
        return -1;
    }
    /* test station id */
    if (!test_staid(rtcm,staid)) return -1;
    
    adjday_glot(rtcm,tod);
    
    trace(4,"decode_head1009: time=%s nsat=%d sync=%d\n",time_str(rtcm->time,2),
          nsat,*sync);
    
    if (rtcm->outtype) {
        msg=rtcm->msgtype+strlen(rtcm->msgtype);
        sprintf(msg," staid=%4d %s nsat=%2d sync=%d",staid,
                time_str(rtcm->time,2),nsat,*sync);
    }
    return nsat;
}
/* decode type 1009: L1-only glonass rtk observables -------------------------*/
static int decode_type1009(rtcm_t *rtcm)
{
    int sync;
    if (decode_head1009(rtcm,&sync)<0) return -1;
    rtcm->obsflag=!sync;
    return sync?0:1;
}
/* decode type 1010: extended L1-only glonass rtk observables ----------------*/
static int decode_type1010(rtcm_t *rtcm)
{
    double pr1,cnr1,tt,cp1,lam1;
    int i=24+61,j,index,nsat,sync,prn,sat,code,freq,ppr1,lock1,amb,sys=SYS_GLO;
    
    if ((nsat=decode_head1009(rtcm,&sync))<0) return -1;
    
    for (j=0;j<nsat&&rtcm->obs.n<MAXOBS&&i+79<=rtcm->len*8;j++) {
        prn  =getbitu(rtcm->buff,i, 6); i+= 6;
        code =getbitu(rtcm->buff,i, 1); i+= 1;
        freq =getbitu(rtcm->buff,i, 5); i+= 5;
        pr1  =getbitu(rtcm->buff,i,25); i+=25;
        ppr1 =getbits(rtcm->buff,i,20); i+=20;
        lock1=getbitu(rtcm->buff,i, 7); i+= 7;
        amb  =getbitu(rtcm->buff,i, 7); i+= 7;
        cnr1 =getbitu(rtcm->buff,i, 8); i+= 8;
        if (!(sat=satno(sys,prn))) {
            trace(2,"rtcm3 1010 satellite number error: prn=%d\n",prn);
            continue;
        }
        tt=timediff(rtcm->obs.data[0].time,rtcm->time);
        if (rtcm->obsflag||fabs(tt)>1E-9) {
            rtcm->obs.n=rtcm->obsflag=0;
        }
        if ((index=obsindex(&rtcm->obs,rtcm->time,sat))<0) continue;
        pr1=pr1*0.02+amb*PRUNIT_GLO;
        if (ppr1!=(int)0xFFF80000) {
            rtcm->obs.data[index].P[0]=pr1;
            lam1=CLIGHT/(FREQ1_GLO+DFRQ1_GLO*(freq-7));
            cp1=adjcp(rtcm,sat,0,ppr1*0.0005/lam1);
            rtcm->obs.data[index].L[0]=pr1/lam1+cp1;
        }
        rtcm->obs.data[index].LLI[0]=lossoflock(rtcm,sat,0,lock1);
        rtcm->obs.data[index].SNR[0]=snratio(cnr1*0.25);
        rtcm->obs.data[index].code[0]=code?CODE_L1P:CODE_L1C;
    }
    return sync?0:1;
}
/* decode type 1011: L1&L2 glonass rtk observables ---------------------------*/
static int decode_type1011(rtcm_t *rtcm)
{
    int sync;
    if (decode_head1009(rtcm,&sync)<0) return -1;
    rtcm->obsflag=!sync;
    return sync?0:1;
}
/* decode type 1012: extended L1&L2 glonass rtk observables ------------------*/
static int decode_type1012(rtcm_t *rtcm)
{
    double pr1,cnr1,cnr2,tt,cp1,cp2,lam1,lam2;
    int i=24+61,j,index,nsat,sync,prn,sat,freq,code1,code2,pr21,ppr1,ppr2;
    int lock1,lock2,amb,sys=SYS_GLO;
    
    if ((nsat=decode_head1009(rtcm,&sync))<0) return -1;
    
    for (j=0;j<nsat&&rtcm->obs.n<MAXOBS&&i+130<=rtcm->len*8;j++) {
        prn  =getbitu(rtcm->buff,i, 6); i+= 6;
        code1=getbitu(rtcm->buff,i, 1); i+= 1;
        freq =getbitu(rtcm->buff,i, 5); i+= 5;
        pr1  =getbitu(rtcm->buff,i,25); i+=25;
        ppr1 =getbits(rtcm->buff,i,20); i+=20;
        lock1=getbitu(rtcm->buff,i, 7); i+= 7;
        amb  =getbitu(rtcm->buff,i, 7); i+= 7;
        cnr1 =getbitu(rtcm->buff,i, 8); i+= 8;
        code2=getbitu(rtcm->buff,i, 2); i+= 2;
        pr21 =getbits(rtcm->buff,i,14); i+=14;
        ppr2 =getbits(rtcm->buff,i,20); i+=20;
        lock2=getbitu(rtcm->buff,i, 7); i+= 7;
        cnr2 =getbitu(rtcm->buff,i, 8); i+= 8;
        if (!(sat=satno(sys,prn))) {
            trace(2,"rtcm3 1012 satellite number error: sys=%d prn=%d\n",sys,prn);
            continue;
        }
        tt=timediff(rtcm->obs.data[0].time,rtcm->time);
        if (rtcm->obsflag||fabs(tt)>1E-9) {
            rtcm->obs.n=rtcm->obsflag=0;
        }
        if ((index=obsindex(&rtcm->obs,rtcm->time,sat))<0) continue;
        pr1=pr1*0.02+amb*PRUNIT_GLO;
        if (ppr1!=(int)0xFFF80000) {
            lam1=CLIGHT/(FREQ1_GLO+DFRQ1_GLO*(freq-7));
            rtcm->obs.data[index].P[0]=pr1;
            cp1=adjcp(rtcm,sat,0,ppr1*0.0005/lam1);
            rtcm->obs.data[index].L[0]=pr1/lam1+cp1;
        }
        rtcm->obs.data[index].LLI[0]=lossoflock(rtcm,sat,0,lock1);
        rtcm->obs.data[index].SNR[0]=snratio(cnr1*0.25);
        rtcm->obs.data[index].code[0]=code1?CODE_L1P:CODE_L1C;
        
        if (pr21!=(int)0xFFFFE000) {
            rtcm->obs.data[index].P[1]=pr1+pr21*0.02;
        }
        if (ppr2!=(int)0xFFF80000) {
            lam2=CLIGHT/(FREQ2_GLO+DFRQ2_GLO*(freq-7));
            cp2=adjcp(rtcm,sat,1,ppr2*0.0005/lam2);
            rtcm->obs.data[index].L[1]=pr1/lam2+cp2;
        }
        rtcm->obs.data[index].LLI[1]=lossoflock(rtcm,sat,1,lock2);
        rtcm->obs.data[index].SNR[1]=snratio(cnr2*0.25);
        rtcm->obs.data[index].code[1]=code2?CODE_L2P:CODE_L2C;
    }
    rtcm->obsflag=!sync;
    return sync?0:1;
}
/* decode type 1013: system parameters ---------------------------------------*/
static int decode_type1013(rtcm_t *rtcm)
{
    return 0;
}
/* decode type 1019: gps ephemerides -----------------------------------------*/
static int decode_type1019(rtcm_t *rtcm)
{
    eph_t eph={0};
    double toc,sqrtA;
    char *msg;
    int i=24+12,prn,sat,week,sys=SYS_GPS;
    
    if (i+476<=rtcm->len*8) {
        prn       =getbitu(rtcm->buff,i, 6);              i+= 6;
        week      =getbitu(rtcm->buff,i,10);              i+=10;
        eph.sva   =getbitu(rtcm->buff,i, 4);              i+= 4;
        eph.code  =getbitu(rtcm->buff,i, 2);              i+= 2;
        eph.idot  =getbits(rtcm->buff,i,14)*P2_43*SC2RAD; i+=14;
        eph.iode  =getbitu(rtcm->buff,i, 8);              i+= 8;
        toc       =getbitu(rtcm->buff,i,16)*16.0;         i+=16;
        eph.f2    =getbits(rtcm->buff,i, 8)*P2_55;        i+= 8;
        eph.f1    =getbits(rtcm->buff,i,16)*P2_43;        i+=16;
        eph.f0    =getbits(rtcm->buff,i,22)*P2_31;        i+=22;
        eph.iodc  =getbitu(rtcm->buff,i,10);              i+=10;
        eph.crs   =getbits(rtcm->buff,i,16)*P2_5;         i+=16;
        eph.deln  =getbits(rtcm->buff,i,16)*P2_43*SC2RAD; i+=16;
        eph.M0    =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph.cuc   =getbits(rtcm->buff,i,16)*P2_29;        i+=16;
        eph.e     =getbitu(rtcm->buff,i,32)*P2_33;        i+=32;
        eph.cus   =getbits(rtcm->buff,i,16)*P2_29;        i+=16;
        sqrtA     =getbitu(rtcm->buff,i,32)*P2_19;        i+=32;
        eph.toes  =getbitu(rtcm->buff,i,16)*16.0;         i+=16;
        eph.cic   =getbits(rtcm->buff,i,16)*P2_29;        i+=16;
        eph.OMG0  =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph.cis   =getbits(rtcm->buff,i,16)*P2_29;        i+=16;
        eph.i0    =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph.crc   =getbits(rtcm->buff,i,16)*P2_5;         i+=16;
        eph.omg   =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph.OMGd  =getbits(rtcm->buff,i,24)*P2_43*SC2RAD; i+=24;
        eph.tgd[0]=getbits(rtcm->buff,i, 8)*P2_31;        i+= 8;
        eph.svh   =getbitu(rtcm->buff,i, 6);              i+= 6;
        eph.flag  =getbitu(rtcm->buff,i, 1);              i+= 1;
        eph.fit   =getbitu(rtcm->buff,i, 1)?0.0:4.0; /* 0:4hr,1:>4hr */
    }
    else {
        trace(2,"rtcm3 1019 length error: len=%d\n",rtcm->len);
        return -1;
    }
    if (prn>=40) {
        sys=SYS_SBS; prn+=80;
    }
    trace(4,"decode_type1019: prn=%d iode=%d toe=%.0f\n",prn,eph.iode,eph.toes);
    
    if (rtcm->outtype) {
        msg=rtcm->msgtype+strlen(rtcm->msgtype);
        sprintf(msg," prn=%2d iode=%3d iodc=%3d week=%d toe=%6.0f toc=%6.0f svh=%02X",
                prn,eph.iode,eph.iodc,week,eph.toes,toc,eph.svh);
    }
    if (!(sat=satno(sys,prn))) {
        trace(2,"rtcm3 1019 satellite number error: prn=%d\n",prn);
        return -1;
    }
    eph.sat=sat;
    eph.week=adjgpsweek(week);
    eph.toe=gpst2time(eph.week,eph.toes);
    eph.toc=gpst2time(eph.week,toc);
    eph.ttr=rtcm->time;
    eph.A=sqrtA*sqrtA;
    if (!strstr(rtcm->opt,"-EPHALL")) {
        if (eph.iode==rtcm->nav.eph[sat-1].iode) return 0; /* unchanged */
    }
    rtcm->nav.eph[sat-1]=eph;
    rtcm->ephsat=sat;
    return 2;
}
/* decode type 1020: glonass ephemerides -------------------------------------*/
static int decode_type1020(rtcm_t *rtcm)
{
    geph_t geph={0};
    double tk_h,tk_m,tk_s,toe,tow,tod,tof;
    char *msg;
    int i=24+12,prn,sat,week,tb,bn,sys=SYS_GLO;
    
    if (i+348<=rtcm->len*8) {
        prn        =getbitu(rtcm->buff,i, 6);           i+= 6;
        geph.frq   =getbitu(rtcm->buff,i, 5)-7;         i+= 5+2+2;
        tk_h       =getbitu(rtcm->buff,i, 5);           i+= 5;
        tk_m       =getbitu(rtcm->buff,i, 6);           i+= 6;
        tk_s       =getbitu(rtcm->buff,i, 1)*30.0;      i+= 1;
        bn         =getbitu(rtcm->buff,i, 1);           i+= 1+1;
        tb         =getbitu(rtcm->buff,i, 7);           i+= 7;
        geph.vel[0]=getbitg(rtcm->buff,i,24)*P2_20*1E3; i+=24;
        geph.pos[0]=getbitg(rtcm->buff,i,27)*P2_11*1E3; i+=27;
        geph.acc[0]=getbitg(rtcm->buff,i, 5)*P2_30*1E3; i+= 5;
        geph.vel[1]=getbitg(rtcm->buff,i,24)*P2_20*1E3; i+=24;
        geph.pos[1]=getbitg(rtcm->buff,i,27)*P2_11*1E3; i+=27;
        geph.acc[1]=getbitg(rtcm->buff,i, 5)*P2_30*1E3; i+= 5;
        geph.vel[2]=getbitg(rtcm->buff,i,24)*P2_20*1E3; i+=24;
        geph.pos[2]=getbitg(rtcm->buff,i,27)*P2_11*1E3; i+=27;
        geph.acc[2]=getbitg(rtcm->buff,i, 5)*P2_30*1E3; i+= 5+1;
        geph.gamn  =getbitg(rtcm->buff,i,11)*P2_40;     i+=11+3;
        geph.taun  =getbitg(rtcm->buff,i,22)*P2_30;
    }
    else {
        trace(2,"rtcm3 1020 length error: len=%d\n",rtcm->len);
        return -1;
    }
    if (!(sat=satno(sys,prn))) {
        trace(2,"rtcm3 1020 satellite number error: prn=%d\n",prn);
        return -1;
    }
    trace(4,"decode_type1020: prn=%d tk=%02.0f:%02.0f:%02.0f\n",prn,tk_h,tk_m,tk_s);
    
    if (rtcm->outtype) {
        msg=rtcm->msgtype+strlen(rtcm->msgtype);
        sprintf(msg," prn=%2d tk=%02.0f:%02.0f:%02.0f frq=%2d bn=%d tb=%d",
                prn,tk_h,tk_m,tk_s,geph.frq,bn,tb);
    }
    geph.sat=sat;
    geph.svh=bn;
    geph.iode=tb&0x7F;
    if (rtcm->time.time==0) rtcm->time=utc2gpst(timeget());
    tow=time2gpst(gpst2utc(rtcm->time),&week);
    tod=fmod(tow,86400.0); tow-=tod;
    tof=tk_h*3600.0+tk_m*60.0+tk_s-10800.0; /* lt->utc */
    if      (tof<tod-43200.0) tof+=86400.0;
    else if (tof>tod+43200.0) tof-=86400.0;
    geph.tof=utc2gpst(gpst2time(week,tow+tof));
    toe=tb*900.0-10800.0; /* lt->utc */
    if      (toe<tod-43200.0) toe+=86400.0;
    else if (toe>tod+43200.0) toe-=86400.0;
    geph.toe=utc2gpst(gpst2time(week,tow+toe)); /* utc->gpst */
    
    if (!strstr(rtcm->opt,"-EPHALL")) {
        if (fabs(timediff(geph.toe,rtcm->nav.geph[prn-1].toe))<1.0&&
            geph.svh==rtcm->nav.geph[prn-1].svh) return 0; /* unchanged */
    }
    rtcm->nav.geph[prn-1]=geph;
    rtcm->ephsat=sat;
    return 2;
}
/* decode type 1021: helmert/abridged molodenski -----------------------------*/
static int decode_type1021(rtcm_t *rtcm)
{
    trace(2,"rtcm3 1021: not supported message\n");
    return 0;
}
/* decode type 1022: moledenski-badekas transfromation -----------------------*/
static int decode_type1022(rtcm_t *rtcm)
{
    trace(2,"rtcm3 1022: not supported message\n");
    return 0;
}
/* decode type 1023: residual, ellipoidal grid representation ----------------*/
static int decode_type1023(rtcm_t *rtcm)
{
    trace(2,"rtcm3 1023: not supported message\n");
    return 0;
}
/* decode type 1024: residual, plane grid representation ---------------------*/
static int decode_type1024(rtcm_t *rtcm)
{
    trace(2,"rtcm3 1024: not supported message\n");
    return 0;
}
/* decode type 1025: projection (types except LCC2SP,OM) ---------------------*/
static int decode_type1025(rtcm_t *rtcm)
{
    trace(2,"rtcm3 1025: not supported message\n");
    return 0;
}
/* decode type 1026: projection (LCC2SP - lambert conic conformal (2sp)) -----*/
static int decode_type1026(rtcm_t *rtcm)
{
    trace(2,"rtcm3 1026: not supported message\n");
    return 0;
}
/* decode type 1027: projection (type OM - oblique mercator) -----------------*/
static int decode_type1027(rtcm_t *rtcm)
{
    trace(2,"rtcm3 1027: not supported message\n");
    return 0;
}
/* decode type 1030: network rtk residual ------------------------------------*/
static int decode_type1030(rtcm_t *rtcm)
{
    trace(2,"rtcm3 1030: not supported message\n");
    return 0;
}
/* decode type 1031: glonass network rtk residual ----------------------------*/
static int decode_type1031(rtcm_t *rtcm)
{
    trace(2,"rtcm3 1031: not supported message\n");
    return 0;
}
/* decode type 1032: physical reference station position information ---------*/
static int decode_type1032(rtcm_t *rtcm)
{
    trace(2,"rtcm3 1032: not supported message\n");
    return 0;
}
/* decode type 1033: receiver and antenna descriptor -------------------------*/
static int decode_type1033(rtcm_t *rtcm)
{
    char des[32]="",sno[32]="",rec[32]="",ver[32]="",rsn[32]="";
    char *msg;
    int i=24+12,j,staid,n,m,n1,n2,n3,setup;
    
    n =getbitu(rtcm->buff,i+12,8);
    m =getbitu(rtcm->buff,i+28+8*n,8);
    n1=getbitu(rtcm->buff,i+36+8*(n+m),8);
    n2=getbitu(rtcm->buff,i+44+8*(n+m+n1),8);
    n3=getbitu(rtcm->buff,i+52+8*(n+m+n1+n2),8);
    
    if (i+60+8*(n+m+n1+n2+n3)<=rtcm->len*8) {
        staid=getbitu(rtcm->buff,i,12); i+=12+8;
        for (j=0;j<n&&j<31;j++) {
            des[j]=(char)getbitu(rtcm->buff,i,8); i+=8;
        }
        setup=getbitu(rtcm->buff,i, 8); i+=8+8;
        for (j=0;j<m&&j<31;j++) {
            sno[j]=(char)getbitu(rtcm->buff,i,8); i+=8;
        }
        i+=8;
        for (j=0;j<n1&&j<31;j++) {
            rec[j]=(char)getbitu(rtcm->buff,i,8); i+=8;
        }
        i+=8;
        for (j=0;j<n2&&j<31;j++) {
            ver[j]=(char)getbitu(rtcm->buff,i,8); i+=8;
        }
        i+=8;
        for (j=0;j<n3&&j<31;j++) {
            rsn[j]=(char)getbitu(rtcm->buff,i,8); i+=8;
        }
    }
    else {
        trace(2,"rtcm3 1033 length error: len=%d\n",rtcm->len);
        return -1;
    }
    if (rtcm->outtype) {
        msg=rtcm->msgtype+strlen(rtcm->msgtype);
        sprintf(msg," staid=%4d",staid);
    }
    /* test station id */
    if (!test_staid(rtcm,staid)) return -1;
    
    strncpy(rtcm->sta.antdes, des,n ); rtcm->sta.antdes [n] ='\0';
    rtcm->sta.antsetup=setup;
    strncpy(rtcm->sta.antsno, sno,m ); rtcm->sta.antsno [m] ='\0';
    strncpy(rtcm->sta.rectype,rec,n1); rtcm->sta.rectype[n1]='\0';
    strncpy(rtcm->sta.recver, ver,n2); rtcm->sta.recver [n2]='\0';
    strncpy(rtcm->sta.recsno, rsn,n3); rtcm->sta.recsno [n3]='\0';
    
    trace(3,"rtcm3 1033: ant=%s:%s rec=%s:%s:%s\n",des,sno,rec,ver,rsn);
    return 5;
}
/* decode type 1034: gps network fkp gradient --------------------------------*/
static int decode_type1034(rtcm_t *rtcm)
{
    trace(2,"rtcm3 1034: not supported message\n");
    return 0;
}
/* decode type 1035: glonass network fkp gradient ----------------------------*/
static int decode_type1035(rtcm_t *rtcm)
{
    trace(2,"rtcm3 1035: not supported message\n");
    return 0;
}
/* decode type 1037: glonass network rtk ionospheric correction difference ---*/
static int decode_type1037(rtcm_t *rtcm)
{
    trace(2,"rtcm3 1037: not supported message\n");
    return 0;
}
/* decode type 1038: glonass network rtk geometic correction difference ------*/
static int decode_type1038(rtcm_t *rtcm)
{
    trace(2,"rtcm3 1038: not supported message\n");
    return 0;
}
/* decode type 1039: glonass network rtk combined correction difference ------*/
static int decode_type1039(rtcm_t *rtcm)
{
    trace(2,"rtcm3 1039: not supported message\n");
    return 0;
}
/* decode type 1044: qzss ephemerides (ref [15]) -----------------------------*/
static int decode_type1044(rtcm_t *rtcm)
{
    eph_t eph={0};
    double toc,sqrtA;
    char *msg;
    int i=24+12,prn,sat,week,sys=SYS_QZS;
    
    if (i+473<=rtcm->len*8) {
        prn       =getbitu(rtcm->buff,i, 4)+192;          i+= 4;
        toc       =getbitu(rtcm->buff,i,16)*16.0;         i+=16;
        eph.f2    =getbits(rtcm->buff,i, 8)*P2_55;        i+= 8;
        eph.f1    =getbits(rtcm->buff,i,16)*P2_43;        i+=16;
        eph.f0    =getbits(rtcm->buff,i,22)*P2_31;        i+=22;
        eph.iode  =getbitu(rtcm->buff,i, 8);              i+= 8;
        eph.crs   =getbits(rtcm->buff,i,16)*P2_5;         i+=16;
        eph.deln  =getbits(rtcm->buff,i,16)*P2_43*SC2RAD; i+=16;
        eph.M0    =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph.cuc   =getbits(rtcm->buff,i,16)*P2_29;        i+=16;
        eph.e     =getbitu(rtcm->buff,i,32)*P2_33;        i+=32;
        eph.cus   =getbits(rtcm->buff,i,16)*P2_29;        i+=16;
        sqrtA     =getbitu(rtcm->buff,i,32)*P2_19;        i+=32;
        eph.toes  =getbitu(rtcm->buff,i,16)*16.0;         i+=16;
        eph.cic   =getbits(rtcm->buff,i,16)*P2_29;        i+=16;
        eph.OMG0  =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph.cis   =getbits(rtcm->buff,i,16)*P2_29;        i+=16;
        eph.i0    =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph.crc   =getbits(rtcm->buff,i,16)*P2_5;         i+=16;
        eph.omg   =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph.OMGd  =getbits(rtcm->buff,i,24)*P2_43*SC2RAD; i+=24;
        eph.idot  =getbits(rtcm->buff,i,14)*P2_43*SC2RAD; i+=14;
        eph.code  =getbitu(rtcm->buff,i, 2);              i+= 2;
        week      =getbitu(rtcm->buff,i,10);              i+=10;
        eph.sva   =getbitu(rtcm->buff,i, 4);              i+= 4;
        eph.svh   =getbitu(rtcm->buff,i, 6);              i+= 6;
        eph.tgd[0]=getbits(rtcm->buff,i, 8)*P2_31;        i+= 8;
        eph.iodc  =getbitu(rtcm->buff,i,10);              i+=10;
        eph.fit   =getbitu(rtcm->buff,i, 1)?0.0:2.0; /* 0:2hr,1:>2hr */
    }
    else {
        trace(2,"rtcm3 1044 length error: len=%d\n",rtcm->len);
        return -1;
    }
    trace(4,"decode_type1044: prn=%d iode=%d toe=%.0f\n",prn,eph.iode,eph.toes);
    
    if (rtcm->outtype) {
        msg=rtcm->msgtype+strlen(rtcm->msgtype);
        sprintf(msg," prn=%3d iode=%3d iodc=%3d week=%d toe=%6.0f toc=%6.0f svh=%02X",
                prn,eph.iode,eph.iodc,week,eph.toes,toc,eph.svh);
    }
    if (!(sat=satno(sys,prn))) {
        trace(2,"rtcm3 1044 satellite number error: prn=%d\n",prn);
        return -1;
    }
    eph.sat=sat;
    eph.week=adjgpsweek(week);
    eph.toe=gpst2time(eph.week,eph.toes);
    eph.toc=gpst2time(eph.week,toc);
    eph.ttr=rtcm->time;
    eph.A=sqrtA*sqrtA;
    if (!strstr(rtcm->opt,"-EPHALL")) {
        if (eph.iode==rtcm->nav.eph[sat-1].iode&&
            eph.iodc==rtcm->nav.eph[sat-1].iodc) return 0; /* unchanged */
    }
    rtcm->nav.eph[sat-1]=eph;
    rtcm->ephsat=sat;
    return 2;
}
/* decode type 1045: galileo F/NAV satellite ephemerides (ref [15]) ----------*/
static int decode_type1045(rtcm_t *rtcm)
{
    eph_t eph={0};
    double toc,sqrtA;
    char *msg;
    int i=24+12,prn,sat,week,e5a_hs,e5a_dvs,rsv,sys=SYS_GAL;
    
    if (i+484<=rtcm->len*8) {
        prn       =getbitu(rtcm->buff,i, 6);              i+= 6;
        week      =getbitu(rtcm->buff,i,12);              i+=12; /* gst-week */
        eph.iode  =getbitu(rtcm->buff,i,10);              i+=10;
        eph.sva   =getbitu(rtcm->buff,i, 8);              i+= 8;
        eph.idot  =getbits(rtcm->buff,i,14)*P2_43*SC2RAD; i+=14;
        toc       =getbitu(rtcm->buff,i,14)*60.0;         i+=14;
        eph.f2    =getbits(rtcm->buff,i, 6)*P2_59;        i+= 6;
        eph.f1    =getbits(rtcm->buff,i,21)*P2_46;        i+=21;
        eph.f0    =getbits(rtcm->buff,i,31)*P2_34;        i+=31;
        eph.crs   =getbits(rtcm->buff,i,16)*P2_5;         i+=16;
        eph.deln  =getbits(rtcm->buff,i,16)*P2_43*SC2RAD; i+=16;
        eph.M0    =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph.cuc   =getbits(rtcm->buff,i,16)*P2_29;        i+=16;
        eph.e     =getbitu(rtcm->buff,i,32)*P2_33;        i+=32;
        eph.cus   =getbits(rtcm->buff,i,16)*P2_29;        i+=16;
        sqrtA     =getbitu(rtcm->buff,i,32)*P2_19;        i+=32;
        eph.toes  =getbitu(rtcm->buff,i,14)*60.0;         i+=14;
        eph.cic   =getbits(rtcm->buff,i,16)*P2_29;        i+=16;
        eph.OMG0  =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph.cis   =getbits(rtcm->buff,i,16)*P2_29;        i+=16;
        eph.i0    =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph.crc   =getbits(rtcm->buff,i,16)*P2_5;         i+=16;
        eph.omg   =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph.OMGd  =getbits(rtcm->buff,i,24)*P2_43*SC2RAD; i+=24;
        eph.tgd[0]=getbits(rtcm->buff,i,10)*P2_32;        i+=10; /* E5a/E1 */
        e5a_hs    =getbitu(rtcm->buff,i, 2);              i+= 2; /* OSHS */
        e5a_dvs   =getbitu(rtcm->buff,i, 1);              i+= 1; /* OSDVS */
        rsv       =getbitu(rtcm->buff,i, 7);
    }
    else {
        trace(2,"rtcm3 1045 length error: len=%d\n",rtcm->len);
        return -1;
    }
    trace(4,"decode_type1045: prn=%d iode=%d toe=%.0f\n",prn,eph.iode,eph.toes);
    
    if (rtcm->outtype) {
        msg=rtcm->msgtype+strlen(rtcm->msgtype);
        sprintf(msg," prn=%2d iode=%3d week=%d toe=%6.0f toc=%6.0f hs=%d dvs=%d",
                prn,eph.iode,week,eph.toes,toc,e5a_hs,e5a_dvs);
    }
    if (!(sat=satno(sys,prn))) {
        trace(2,"rtcm3 1045 satellite number error: prn=%d\n",prn);
        return -1;
    }
    eph.sat=sat;
    eph.week=week+1024; /* gal-week = gst-week + 1024 */
    eph.toe=gpst2time(eph.week,eph.toes);
    eph.toc=gpst2time(eph.week,toc);
    eph.ttr=rtcm->time;
    eph.A=sqrtA*sqrtA;
    eph.svh=(e5a_hs<<4)+(e5a_dvs<<3);
    eph.code=1; /* data source = F/NAV */
    if (!strstr(rtcm->opt,"-EPHALL")) {
        if (eph.iode==rtcm->nav.eph[sat-1].iode) return 0; /* unchanged */
    }
    rtcm->nav.eph[sat-1]=eph;
    rtcm->ephsat=sat;
    return 2;
}
/* decode type 1046: galileo I/NAV satellite ephemerides (ref [17]) ----------*/
static int decode_type1046(rtcm_t *rtcm)
{
    eph_t eph={0};
    double toc,sqrtA;
    char *msg;
    int i=24+12,prn,sat,week,e5b_hs,e5b_dvs,e1_hs,e1_dvs,sys=SYS_GAL;
    
    if (i+492<=rtcm->len*8) {
        prn       =getbitu(rtcm->buff,i, 6);              i+= 6;
        week      =getbitu(rtcm->buff,i,12);              i+=12;
        eph.iode  =getbitu(rtcm->buff,i,10);              i+=10;
        eph.sva   =getbitu(rtcm->buff,i, 8);              i+= 8;
        eph.idot  =getbits(rtcm->buff,i,14)*P2_43*SC2RAD; i+=14;
        toc       =getbitu(rtcm->buff,i,14)*60.0;         i+=14;
        eph.f2    =getbits(rtcm->buff,i, 6)*P2_59;        i+= 6;
        eph.f1    =getbits(rtcm->buff,i,21)*P2_46;        i+=21;
        eph.f0    =getbits(rtcm->buff,i,31)*P2_34;        i+=31;
        eph.crs   =getbits(rtcm->buff,i,16)*P2_5;         i+=16;
        eph.deln  =getbits(rtcm->buff,i,16)*P2_43*SC2RAD; i+=16;
        eph.M0    =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph.cuc   =getbits(rtcm->buff,i,16)*P2_29;        i+=16;
        eph.e     =getbitu(rtcm->buff,i,32)*P2_33;        i+=32;
        eph.cus   =getbits(rtcm->buff,i,16)*P2_29;        i+=16;
        sqrtA     =getbitu(rtcm->buff,i,32)*P2_19;        i+=32;
        eph.toes  =getbitu(rtcm->buff,i,14)*60.0;         i+=14;
        eph.cic   =getbits(rtcm->buff,i,16)*P2_29;        i+=16;
        eph.OMG0  =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph.cis   =getbits(rtcm->buff,i,16)*P2_29;        i+=16;
        eph.i0    =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph.crc   =getbits(rtcm->buff,i,16)*P2_5;         i+=16;
        eph.omg   =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph.OMGd  =getbits(rtcm->buff,i,24)*P2_43*SC2RAD; i+=24;
        eph.tgd[0]=getbits(rtcm->buff,i,10)*P2_32;        i+=10; /* E5a/E1 */
        eph.tgd[1]=getbits(rtcm->buff,i,10)*P2_32;        i+=10; /* E5b/E1 */
        e5b_hs    =getbitu(rtcm->buff,i, 2);              i+= 2; /* E5b OSHS */
        e5b_dvs   =getbitu(rtcm->buff,i, 1);              i+= 1; /* E5b OSDVS */
        e1_hs     =getbitu(rtcm->buff,i, 2);              i+= 2; /* E1 OSHS */
        e1_dvs    =getbitu(rtcm->buff,i, 1);              i+= 1; /* E1 OSDVS */
    }
    else {
        trace(2,"rtcm3 1046 length error: len=%d\n",rtcm->len);
        return -1;
    }
    trace(4,"decode_type1046: prn=%d iode=%d toe=%.0f\n",prn,eph.iode,eph.toes);
    
    if (rtcm->outtype) {
        msg=rtcm->msgtype+strlen(rtcm->msgtype);
        sprintf(msg," prn=%2d iode=%3d week=%d toe=%6.0f toc=%6.0f hs=%d %d dvs=%d %d",
                prn,eph.iode,week,eph.toes,toc,e5b_hs,e1_hs,e5b_dvs,e1_dvs);
    }
    if (!(sat=satno(sys,prn))) {
        trace(2,"rtcm3 1046 satellite number error: prn=%d\n",prn);
        return -1;
    }
    eph.sat=sat;
    eph.week=week+1024; /* gal-week = gst-week + 1024 */
    eph.toe=gpst2time(eph.week,eph.toes);
    eph.toc=gpst2time(eph.week,toc);
    eph.ttr=rtcm->time;
    eph.A=sqrtA*sqrtA;
    eph.svh=(e5b_hs<<7)+(e5b_dvs<<6)+(e1_hs<<1)+(e1_dvs<<0);
    eph.code=0; /* data source = I/NAV */
    if (!strstr(rtcm->opt,"-EPHALL")) {
        if (eph.iode==rtcm->nav.eph[sat-1].iode) return 0; /* unchanged */
    }
    rtcm->nav.eph[sat-1]=eph;
    rtcm->ephsat=sat;
    return 2;
}
/* decode type 1042/63: beidou ephemerides -----------------------------------*/
static int decode_type1042(rtcm_t *rtcm)
{
    eph_t eph={0};
    double toc,sqrtA;
    char *msg;
    int i=24+12,prn,sat,week,sys=SYS_CMP;
    
    if (i+499<=rtcm->len*8) {
        prn       =getbitu(rtcm->buff,i, 6);              i+= 6;
        week      =getbitu(rtcm->buff,i,13);              i+=13;
        eph.sva   =getbitu(rtcm->buff,i, 4);              i+= 4;
        eph.idot  =getbits(rtcm->buff,i,14)*P2_43*SC2RAD; i+=14;
        eph.iode  =getbitu(rtcm->buff,i, 5);              i+= 5; /* AODE */
        toc       =getbitu(rtcm->buff,i,17)*8.0;          i+=17;
        eph.f2    =getbits(rtcm->buff,i,11)*P2_66;        i+=11;
        eph.f1    =getbits(rtcm->buff,i,22)*P2_50;        i+=22;
        eph.f0    =getbits(rtcm->buff,i,24)*P2_33;        i+=24;
        eph.iodc  =getbitu(rtcm->buff,i, 5);              i+= 5; /* AODC */
        eph.crs   =getbits(rtcm->buff,i,18)*P2_6;         i+=18;
        eph.deln  =getbits(rtcm->buff,i,16)*P2_43*SC2RAD; i+=16;
        eph.M0    =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph.cuc   =getbits(rtcm->buff,i,18)*P2_31;        i+=18;
        eph.e     =getbitu(rtcm->buff,i,32)*P2_33;        i+=32;
        eph.cus   =getbits(rtcm->buff,i,18)*P2_31;        i+=18;
        sqrtA     =getbitu(rtcm->buff,i,32)*P2_19;        i+=32;
        eph.toes  =getbitu(rtcm->buff,i,17)*8.0;          i+=17;
        eph.cic   =getbits(rtcm->buff,i,18)*P2_31;        i+=18;
        eph.OMG0  =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph.cis   =getbits(rtcm->buff,i,18)*P2_31;        i+=18;
        eph.i0    =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph.crc   =getbits(rtcm->buff,i,18)*P2_6;         i+=18;
        eph.omg   =getbits(rtcm->buff,i,32)*P2_31*SC2RAD; i+=32;
        eph.OMGd  =getbits(rtcm->buff,i,24)*P2_43*SC2RAD; i+=24;
        eph.tgd[0]=getbits(rtcm->buff,i,10)*1E-10;        i+=10;
        eph.tgd[1]=getbits(rtcm->buff,i,10)*1E-10;        i+=10;
        eph.svh   =getbitu(rtcm->buff,i, 1);              i+= 1;
    }
    else {
        trace(2,"rtcm3 1042 length error: len=%d\n",rtcm->len);
        return -1;
    }
    trace(4,"decode_type1042: prn=%d iode=%d toe=%.0f\n",prn,eph.iode,eph.toes);
    
    if (rtcm->outtype) {
        msg=rtcm->msgtype+strlen(rtcm->msgtype);
        sprintf(msg," prn=%2d iode=%3d iodc=%3d week=%d toe=%6.0f toc=%6.0f svh=%02X",
                prn,eph.iode,eph.iodc,week,eph.toes,toc,eph.svh);
    }
    if (!(sat=satno(sys,prn))) {
        trace(2,"rtcm3 1042 satellite number error: prn=%d\n",prn);
        return -1;
    }
    eph.sat=sat;
    eph.week=adjbdtweek(week);
    eph.toe=bdt2gpst(bdt2time(eph.week,eph.toes)); /* bdt -> gpst */
    eph.toc=bdt2gpst(bdt2time(eph.week,toc));      /* bdt -> gpst */
    eph.ttr=rtcm->time;
    eph.A=sqrtA*sqrtA;
    if (!strstr(rtcm->opt,"-EPHALL")) {
        if (timediff(eph.toe,rtcm->nav.eph[sat-1].toe)==0.0&&
            eph.iode==rtcm->nav.eph[sat-1].iode&&
            eph.iodc==rtcm->nav.eph[sat-1].iodc) return 0; /* unchanged */
    }
    rtcm->nav.eph[sat-1]=eph;
    rtcm->ephsat=sat;
    return 2;
}
/* decode ssr 1,4 message header ---------------------------------------------*/
static int decode_ssr1_head(rtcm_t *rtcm, int sys, int *sync, int *iod,
                            double *udint, int *refd, int *hsize)
{
    double tod,tow;
    char *msg;
    int i=24+12,nsat,udi,provid=0,solid=0,ns;
    
    ns=sys==SYS_QZS?4:6;
    
    if (i+(sys==SYS_GLO?53:50+ns)>rtcm->len*8) return -1;
    
    if (sys==SYS_GLO) {
        tod=getbitu(rtcm->buff,i,17); i+=17;
        adjday_glot(rtcm,tod);
    }
    else {
        tow=getbitu(rtcm->buff,i,20); i+=20;
        adjweek(rtcm,tow);
    }
    udi   =getbitu(rtcm->buff,i, 4); i+= 4;
    *sync =getbitu(rtcm->buff,i, 1); i+= 1;
    *refd =getbitu(rtcm->buff,i, 1); i+= 1; /* satellite ref datum */
    *iod  =getbitu(rtcm->buff,i, 4); i+= 4; /* iod */
    provid=getbitu(rtcm->buff,i,16); i+=16; /* provider id */
    solid =getbitu(rtcm->buff,i, 4); i+= 4; /* solution id */
    nsat  =getbitu(rtcm->buff,i,ns); i+=ns;
    *udint=ssrudint[udi];
    
    trace(4,"decode_ssr1_head: time=%s sys=%d nsat=%d sync=%d iod=%d provid=%d solid=%d\n",
          time_str(rtcm->time,2),sys,nsat,*sync,*iod,provid,solid);
    
    if (rtcm->outtype) {
        msg=rtcm->msgtype+strlen(rtcm->msgtype);
        sprintf(msg," %s nsat=%2d iod=%2d udi=%2d sync=%d",
                time_str(rtcm->time,2),nsat,*iod,udi,*sync);
    }
    *hsize=i;
    return nsat;
}
/* decode ssr 2,3,5,6 message header -----------------------------------------*/
static int decode_ssr2_head(rtcm_t *rtcm, int sys, int *sync, int *iod,
                            double *udint, int *hsize)
{
    double tod,tow;
    char *msg;
    int i=24+12,nsat,udi,provid=0,solid=0,ns;
    
    ns=sys==SYS_QZS?4:6;
    
    if (i+(sys==SYS_GLO?52:49+ns)>rtcm->len*8) return -1;
    
    if (sys==SYS_GLO) {
        tod=getbitu(rtcm->buff,i,17); i+=17;
        adjday_glot(rtcm,tod);
    }
    else {
        tow=getbitu(rtcm->buff,i,20); i+=20;
        adjweek(rtcm,tow);
    }
    udi   =getbitu(rtcm->buff,i, 4); i+= 4;
    *sync =getbitu(rtcm->buff,i, 1); i+= 1;
    *iod  =getbitu(rtcm->buff,i, 4); i+= 4;
    provid=getbitu(rtcm->buff,i,16); i+=16; /* provider id */
    solid =getbitu(rtcm->buff,i, 4); i+= 4; /* solution id */
    nsat  =getbitu(rtcm->buff,i,ns); i+=ns;
    *udint=ssrudint[udi];
    
    trace(4,"decode_ssr2_head: time=%s sys=%d nsat=%d sync=%d iod=%d provid=%d solid=%d\n",
          time_str(rtcm->time,2),sys,nsat,*sync,*iod,provid,solid);
    
    if (rtcm->outtype) {
        msg=rtcm->msgtype+strlen(rtcm->msgtype);
        sprintf(msg," %s nsat=%2d iod=%2d udi=%2d sync=%d",
                time_str(rtcm->time,2),nsat,*iod,udi,*sync);
    }
    *hsize=i;
    return nsat;
}
/* ssr signal and tracking mode ids ------------------------------------------*/
static const int codes_gps[]={
    CODE_L1C,CODE_L1P,CODE_L1W,CODE_L1Y,CODE_L1M,CODE_L2C,CODE_L2D,CODE_L2S,
    CODE_L2L,CODE_L2X,CODE_L2P,CODE_L2W,CODE_L2Y,CODE_L2M,CODE_L5I,CODE_L5Q,
    CODE_L5X
};
static const int codes_glo[]={
    CODE_L1C,CODE_L1P,CODE_L2C,CODE_L2P
};
static const int codes_gal[]={
    CODE_L1A,CODE_L1B,CODE_L1C,CODE_L1X,CODE_L1Z,CODE_L5I,CODE_L5Q,CODE_L5X,
    CODE_L7I,CODE_L7Q,CODE_L7X,CODE_L8I,CODE_L8Q,CODE_L8X,CODE_L6A,CODE_L6B,
    CODE_L6C,CODE_L6X,CODE_L6Z
};
static const int codes_qzs[]={
    CODE_L1C,CODE_L1S,CODE_L1L,CODE_L2S,CODE_L2L,CODE_L2X,CODE_L5I,CODE_L5Q,
    CODE_L5X,CODE_L6S,CODE_L6L,CODE_L6X,CODE_L1X
};
static const int codes_bds[]={
    CODE_L1I,CODE_L1Q,CODE_L1X,CODE_L7I,CODE_L7Q,CODE_L7X,CODE_L6I,CODE_L6Q,
    CODE_L6X
};
static const int codes_sbs[]={
    CODE_L1C,CODE_L5I,CODE_L5Q,CODE_L5X
};
/* decode ssr 1: orbit corrections -------------------------------------------*/
static int decode_ssr1(rtcm_t *rtcm, int sys)
{
    double udint,deph[3],ddeph[3];
    int i,j,k,type,sync,iod,nsat,prn,sat,iode,iodcrc,refd=0,np,ni,nj,offp;
    
    type=getbitu(rtcm->buff,24,12);
    
    if ((nsat=decode_ssr1_head(rtcm,sys,&sync,&iod,&udint,&refd,&i))<0) {
        trace(2,"rtcm3 %d length error: len=%d\n",type,rtcm->len);
        return -1;
    }
    switch (sys) {
        case SYS_GPS: np=6; ni= 8; nj= 0; offp=  0; break;
        case SYS_GLO: np=5; ni= 8; nj= 0; offp=  0; break;
        case SYS_GAL: np=6; ni=10; nj= 0; offp=  0; break;
        case SYS_QZS: np=4; ni= 8; nj= 0; offp=192; break;
        case SYS_CMP: np=6; ni=10; nj=24; offp=  1; break;
        case SYS_SBS: np=6; ni= 9; nj=24; offp=120; break;
        default: return sync?0:10;
    }
    for (j=0;j<nsat&&i+121+np+ni+nj<=rtcm->len*8;j++) {
        prn     =getbitu(rtcm->buff,i,np)+offp; i+=np;
        iode    =getbitu(rtcm->buff,i,ni);      i+=ni;
        iodcrc  =getbitu(rtcm->buff,i,nj);      i+=nj;
        deph [0]=getbits(rtcm->buff,i,22)*1E-4; i+=22;
        deph [1]=getbits(rtcm->buff,i,20)*4E-4; i+=20;
        deph [2]=getbits(rtcm->buff,i,20)*4E-4; i+=20;
        ddeph[0]=getbits(rtcm->buff,i,21)*1E-6; i+=21;
        ddeph[1]=getbits(rtcm->buff,i,19)*4E-6; i+=19;
        ddeph[2]=getbits(rtcm->buff,i,19)*4E-6; i+=19;
        
        if (!(sat=satno(sys,prn))) {
            trace(2,"rtcm3 %d satellite number error: prn=%d\n",type,prn);
            continue;
        }
        rtcm->ssr[sat-1].t0 [0]=rtcm->time;
        rtcm->ssr[sat-1].udi[0]=udint;
        rtcm->ssr[sat-1].iod[0]=iod;
        rtcm->ssr[sat-1].iode=iode;     /* sbas/bds: toe/t0 modulo */
        rtcm->ssr[sat-1].iodcrc=iodcrc; /* sbas/bds: iod crc */
        rtcm->ssr[sat-1].refd=refd;
        
        for (k=0;k<3;k++) {
            rtcm->ssr[sat-1].deph [k]=deph [k];
            rtcm->ssr[sat-1].ddeph[k]=ddeph[k];
        }
        rtcm->ssr[sat-1].update=1;
    }
    return sync?0:10;
}
/* decode ssr 2: clock corrections -------------------------------------------*/
static int decode_ssr2(rtcm_t *rtcm, int sys)
{
    double udint,dclk[3];
    int i,j,k,type,sync,iod,nsat,prn,sat,np,offp;
    
    type=getbitu(rtcm->buff,24,12);
    
    if ((nsat=decode_ssr2_head(rtcm,sys,&sync,&iod,&udint,&i))<0) {
        trace(2,"rtcm3 %d length error: len=%d\n",type,rtcm->len);
        return -1;
    }
    switch (sys) {
        case SYS_GPS: np=6; offp=  0; break;
        case SYS_GLO: np=5; offp=  0; break;
        case SYS_GAL: np=6; offp=  0; break;
        case SYS_QZS: np=4; offp=192; break;
        case SYS_CMP: np=6; offp=  1; break;
        case SYS_SBS: np=6; offp=120; break;
        default: return sync?0:10;
    }
    for (j=0;j<nsat&&i+70+np<=rtcm->len*8;j++) {
        prn    =getbitu(rtcm->buff,i,np)+offp; i+=np;
        dclk[0]=getbits(rtcm->buff,i,22)*1E-4; i+=22;
        dclk[1]=getbits(rtcm->buff,i,21)*1E-6; i+=21;
        dclk[2]=getbits(rtcm->buff,i,27)*2E-8; i+=27;
        
        if (!(sat=satno(sys,prn))) {
            trace(2,"rtcm3 %d satellite number error: prn=%d\n",type,prn);
            continue;
        }
        rtcm->ssr[sat-1].t0 [1]=rtcm->time;
        rtcm->ssr[sat-1].udi[1]=udint;
        rtcm->ssr[sat-1].iod[1]=iod;
        
        for (k=0;k<3;k++) {
            rtcm->ssr[sat-1].dclk[k]=dclk[k];
        }
        rtcm->ssr[sat-1].update=1;
    }
    return sync?0:10;
}
/* decode ssr 3: satellite code biases ---------------------------------------*/
static int decode_ssr3(rtcm_t *rtcm, int sys)
{
    const int *codes;
    double udint,bias,cbias[MAXCODE];
    int i,j,k,type,mode,sync,iod,nsat,prn,sat,nbias,np,offp,ncode;
    
    type=getbitu(rtcm->buff,24,12);
    
    if ((nsat=decode_ssr2_head(rtcm,sys,&sync,&iod,&udint,&i))<0) {
        trace(2,"rtcm3 %d length error: len=%d\n",type,rtcm->len);
        return -1;
    }
    switch (sys) {
        case SYS_GPS: np=6; offp=  0; codes=codes_gps; ncode=17; break;
        case SYS_GLO: np=5; offp=  0; codes=codes_glo; ncode= 4; break;
        case SYS_GAL: np=6; offp=  0; codes=codes_gal; ncode=19; break;
        case SYS_QZS: np=4; offp=192; codes=codes_qzs; ncode=13; break;
        case SYS_CMP: np=6; offp=  1; codes=codes_bds; ncode= 9; break;
        case SYS_SBS: np=6; offp=120; codes=codes_sbs; ncode= 4; break;
        default: return sync?0:10;
    }
    for (j=0;j<nsat&&i+5+np<=rtcm->len*8;j++) {
        prn  =getbitu(rtcm->buff,i,np)+offp; i+=np;
        nbias=getbitu(rtcm->buff,i, 5);      i+= 5;
        
        for (k=0;k<MAXCODE;k++) cbias[k]=0.0;
        for (k=0;k<nbias&&i+19<=rtcm->len*8;k++) {
            mode=getbitu(rtcm->buff,i, 5);      i+= 5;
            bias=getbits(rtcm->buff,i,14)*0.01; i+=14;
            if (mode<=ncode) {
                cbias[codes[mode]-1]=(float)bias;
            }
            else {
                trace(2,"rtcm3 %d not supported mode: mode=%d\n",type,mode);
            }
        }
        if (!(sat=satno(sys,prn))) {
            trace(2,"rtcm3 %d satellite number error: prn=%d\n",type,prn);
            continue;
        }
        rtcm->ssr[sat-1].t0 [4]=rtcm->time;
        rtcm->ssr[sat-1].udi[4]=udint;
        rtcm->ssr[sat-1].iod[4]=iod;
        
        for (k=0;k<MAXCODE;k++) {
            rtcm->ssr[sat-1].cbias[k]=(float)cbias[k];
        }
        rtcm->ssr[sat-1].update=1;
    }
    return sync?0:10;
}
/* decode ssr 4: combined orbit and clock corrections ------------------------*/
static int decode_ssr4(rtcm_t *rtcm, int sys)
{
    double udint,deph[3],ddeph[3],dclk[3];
    int i,j,k,type,nsat,sync,iod,prn,sat,iode,iodcrc,refd=0,np,ni,nj,offp;
    
    type=getbitu(rtcm->buff,24,12);
    
    if ((nsat=decode_ssr1_head(rtcm,sys,&sync,&iod,&udint,&refd,&i))<0) {
        trace(2,"rtcm3 %d length error: len=%d\n",type,rtcm->len);
        return -1;
    }
    switch (sys) {
        case SYS_GPS: np=6; ni= 8; nj= 0; offp=  0; break;
        case SYS_GLO: np=5; ni= 8; nj= 0; offp=  0; break;
        case SYS_GAL: np=6; ni=10; nj= 0; offp=  0; break;
        case SYS_QZS: np=4; ni= 8; nj= 0; offp=192; break;
        case SYS_CMP: np=6; ni=10; nj=24; offp=  1; break;
        case SYS_SBS: np=6; ni= 9; nj=24; offp=120; break;
        default: return sync?0:10;
    }
    for (j=0;j<nsat&&i+191+np+ni+nj<=rtcm->len*8;j++) {
        prn     =getbitu(rtcm->buff,i,np)+offp; i+=np;
        iode    =getbitu(rtcm->buff,i,ni);      i+=ni;
        iodcrc  =getbitu(rtcm->buff,i,nj);      i+=nj;
        deph [0]=getbits(rtcm->buff,i,22)*1E-4; i+=22;
        deph [1]=getbits(rtcm->buff,i,20)*4E-4; i+=20;
        deph [2]=getbits(rtcm->buff,i,20)*4E-4; i+=20;
        ddeph[0]=getbits(rtcm->buff,i,21)*1E-6; i+=21;
        ddeph[1]=getbits(rtcm->buff,i,19)*4E-6; i+=19;
        ddeph[2]=getbits(rtcm->buff,i,19)*4E-6; i+=19;
        
        dclk [0]=getbits(rtcm->buff,i,22)*1E-4; i+=22;
        dclk [1]=getbits(rtcm->buff,i,21)*1E-6; i+=21;
        dclk [2]=getbits(rtcm->buff,i,27)*2E-8; i+=27;
        
        if (!(sat=satno(sys,prn))) {
            trace(2,"rtcm3 %d satellite number error: prn=%d\n",type,prn);
            continue;
        }
        rtcm->ssr[sat-1].t0 [0]=rtcm->ssr[sat-1].t0 [1]=rtcm->time;
        rtcm->ssr[sat-1].udi[0]=rtcm->ssr[sat-1].udi[1]=udint;
        rtcm->ssr[sat-1].iod[0]=rtcm->ssr[sat-1].iod[1]=iod;
        rtcm->ssr[sat-1].iode=iode;
        rtcm->ssr[sat-1].iodcrc=iodcrc;
        rtcm->ssr[sat-1].refd=refd;
        
        for (k=0;k<3;k++) {
            rtcm->ssr[sat-1].deph [k]=deph [k];
            rtcm->ssr[sat-1].ddeph[k]=ddeph[k];
            rtcm->ssr[sat-1].dclk [k]=dclk [k];
        }
        rtcm->ssr[sat-1].update=1;
    }
    return sync?0:10;
}
/* decode ssr 5: ura ---------------------------------------------------------*/
static int decode_ssr5(rtcm_t *rtcm, int sys)
{
    double udint;
    int i,j,type,nsat,sync,iod,prn,sat,ura,np,offp;
    
    type=getbitu(rtcm->buff,24,12);
    
    if ((nsat=decode_ssr2_head(rtcm,sys,&sync,&iod,&udint,&i))<0) {
        trace(2,"rtcm3 %d length error: len=%d\n",type,rtcm->len);
        return -1;
    }
    switch (sys) {
        case SYS_GPS: np=6; offp=  0; break;
        case SYS_GLO: np=5; offp=  0; break;
        case SYS_GAL: np=6; offp=  0; break;
        case SYS_QZS: np=4; offp=192; break;
        case SYS_CMP: np=6; offp=  1; break;
        case SYS_SBS: np=6; offp=120; break;
        default: return sync?0:10;
    }
    for (j=0;j<nsat&&i+6+np<=rtcm->len*8;j++) {
        prn=getbitu(rtcm->buff,i,np)+offp; i+=np;
        ura=getbitu(rtcm->buff,i, 6);      i+= 6;
        
        if (!(sat=satno(sys,prn))) {
            trace(2,"rtcm3 %d satellite number error: prn=%d\n",type,prn);
            continue;
        }
        rtcm->ssr[sat-1].t0 [3]=rtcm->time;
        rtcm->ssr[sat-1].udi[3]=udint;
        rtcm->ssr[sat-1].iod[3]=iod;
        rtcm->ssr[sat-1].ura=ura;
        rtcm->ssr[sat-1].update=1;
    }
    return sync?0:10;
}
/* decode ssr 6: high rate clock correction ----------------------------------*/
static int decode_ssr6(rtcm_t *rtcm, int sys)
{
    double udint,hrclk;
    int i,j,type,nsat,sync,iod,prn,sat,np,offp;
    
    type=getbitu(rtcm->buff,24,12);
    
    if ((nsat=decode_ssr2_head(rtcm,sys,&sync,&iod,&udint,&i))<0) {
        trace(2,"rtcm3 %d length error: len=%d\n",type,rtcm->len);
        return -1;
    }
    switch (sys) {
        case SYS_GPS: np=6; offp=  0; break;
        case SYS_GLO: np=5; offp=  0; break;
        case SYS_GAL: np=6; offp=  0; break;
        case SYS_QZS: np=4; offp=192; break;
        case SYS_CMP: np=6; offp=  1; break;
        case SYS_SBS: np=6; offp=120; break;
        default: return sync?0:10;
    }
    for (j=0;j<nsat&&i+22+np<=rtcm->len*8;j++) {
        prn  =getbitu(rtcm->buff,i,np)+offp; i+=np;
        hrclk=getbits(rtcm->buff,i,22)*1E-4; i+=22;
        
        if (!(sat=satno(sys,prn))) {
            trace(2,"rtcm3 %d satellite number error: prn=%d\n",type,prn);
            continue;
        }
        rtcm->ssr[sat-1].t0 [2]=rtcm->time;
        rtcm->ssr[sat-1].udi[2]=udint;
        rtcm->ssr[sat-1].iod[2]=iod;
        rtcm->ssr[sat-1].hrclk=hrclk;
        rtcm->ssr[sat-1].update=1;
    }
    return sync?0:10;
}
/* get signal index ----------------------------------------------------------*/
static void sigindex(int sys, const unsigned char *code, const int *freq, int n,
                     const char *opt, int *ind)
{
    int i,nex,pri,pri_h[8]={0},index[8]={0},ex[32]={0};
    
    /* test code priority */
    for (i=0;i<n;i++) {
        if (!code[i]) continue;
        
        if (freq[i]>NFREQ) { /* save as extended signal if freq > NFREQ */
            ex[i]=1;
            continue;
        }
        /* code priority */
        pri=getcodepri(sys,code[i],opt);
        
        /* select highest priority signal */
        if (pri>pri_h[freq[i]-1]) {
            if (index[freq[i]-1]) ex[index[freq[i]-1]-1]=1;
            pri_h[freq[i]-1]=pri;
            index[freq[i]-1]=i+1;
        }
        else ex[i]=1;
    }
    /* signal index in obs data */
    for (i=nex=0;i<n;i++) {
        if (ex[i]==0) ind[i]=freq[i]-1;
        else if (nex<NEXOBS) ind[i]=NFREQ+nex++;
        else { /* no space in obs data */
            trace(2,"rtcm msm: no space in obs data sys=%d code=%d\n",sys,code[i]);
            ind[i]=-1;
        }
#if 0
        trace(2,"sig pos: sys=%d code=%d ex=%d ind=%d\n",sys,code[i],ex[i],ind[i]);
#endif
    }
}
/* save obs data in msm message ----------------------------------------------*/
static void save_msm_obs(rtcm_t *rtcm, int sys, msm_h_t *h, const double *r,
                         const double *pr, const double *cp, const double *rr,
                         const double *rrf, const double *cnr, const int *lock,
                         const int *ex, const int *half)
{
    const char *sig[32];
    double tt,wl;
    unsigned char code[32];
    char *msm_type="",*q=NULL;
    int i,j,k,type,prn,sat,fn,index=0,freq[32],ind[32];
    
    type=getbitu(rtcm->buff,24,12);
    
    switch (sys) {
        case SYS_GPS: msm_type=q=rtcm->msmtype[0]; break;
        case SYS_GLO: msm_type=q=rtcm->msmtype[1]; break;
        case SYS_GAL: msm_type=q=rtcm->msmtype[2]; break;
        case SYS_QZS: msm_type=q=rtcm->msmtype[3]; break;
        case SYS_SBS: msm_type=q=rtcm->msmtype[4]; break;
        case SYS_CMP: msm_type=q=rtcm->msmtype[5]; break;
    }
    /* id to signal */
    for (i=0;i<h->nsig;i++) {
        switch (sys) {
            case SYS_GPS: sig[i]=msm_sig_gps[h->sigs[i]-1]; break;
            case SYS_GLO: sig[i]=msm_sig_glo[h->sigs[i]-1]; break;
            case SYS_GAL: sig[i]=msm_sig_gal[h->sigs[i]-1]; break;
            case SYS_QZS: sig[i]=msm_sig_qzs[h->sigs[i]-1]; break;
            case SYS_SBS: sig[i]=msm_sig_sbs[h->sigs[i]-1]; break;
            case SYS_CMP: sig[i]=msm_sig_cmp[h->sigs[i]-1]; break;
            default: sig[i]=""; break;
        }
        /* signal to rinex obs type */
        code[i]=obs2code(sig[i],freq+i);
        
        /* freqency index for beidou */
        if (sys==SYS_CMP) {
            if      (freq[i]==5) freq[i]=2; /* B2 */
            else if (freq[i]==4) freq[i]=3; /* B3 */
        }
        if (code[i]!=CODE_NONE) {
            if (q) q+=sprintf(q,"L%s%s",sig[i],i<h->nsig-1?",":"");
        }
        else {
            if (q) q+=sprintf(q,"(%d)%s",h->sigs[i],i<h->nsig-1?",":"");
            
            trace(2,"rtcm3 %d: unknown signal id=%2d\n",type,h->sigs[i]);
        }
    }
    trace(3,"rtcm3 %d: signals=%s\n",type,msm_type);
    
    /* get signal index */
    sigindex(sys,code,freq,h->nsig,rtcm->opt,ind);
    
    for (i=j=0;i<h->nsat;i++) {
        
        prn=h->sats[i];
        if      (sys==SYS_QZS) prn+=MINPRNQZS-1;
        else if (sys==SYS_SBS) prn+=MINPRNSBS-1;
        
        if ((sat=satno(sys,prn))) {
            tt=timediff(rtcm->obs.data[0].time,rtcm->time);
            if (rtcm->obsflag||fabs(tt)>1E-9) {
                rtcm->obs.n=rtcm->obsflag=0;
            }
            index=obsindex(&rtcm->obs,rtcm->time,sat);
        }
        else {
            trace(2,"rtcm3 %d satellite error: prn=%d\n",type,prn);
        }
        for (k=0;k<h->nsig;k++) {
            if (!h->cellmask[k+i*h->nsig]) continue;
            
            if (sat&&index>=0&&ind[k]>=0) {
                
                /* satellite carrier wave length */
                wl=satwavelen(sat,freq[k]-1,&rtcm->nav);
                
                /* glonass wave length by extended info */
                if (sys==SYS_GLO&&ex&&ex[i]<=13) {
                    fn=ex[i]-7;
                    wl=CLIGHT/((freq[k]==2?FREQ2_GLO:FREQ1_GLO)+
                               (freq[k]==2?DFRQ2_GLO:DFRQ1_GLO)*fn);
                }
                /* pseudorange (m) */
                if (r[i]!=0.0&&pr[j]>-1E12) {
                    rtcm->obs.data[index].P[ind[k]]=r[i]+pr[j];
                }
                /* carrier-phase (cycle) */
                if (r[i]!=0.0&&cp[j]>-1E12&&wl>0.0) {
                    rtcm->obs.data[index].L[ind[k]]=(r[i]+cp[j])/wl;
                }
                /* doppler (hz) */
                if (rr&&rrf&&rrf[j]>-1E12&&wl>0.0) {
                    rtcm->obs.data[index].D[ind[k]]=(float)(-(rr[i]+rrf[j])/wl);
                }
                rtcm->obs.data[index].LLI[ind[k]]=
                    lossoflock(rtcm,sat,ind[k],lock[j])+(half[j]?3:0);
                rtcm->obs.data[index].SNR [ind[k]]=(unsigned char)(cnr[j]*4.0);
                rtcm->obs.data[index].code[ind[k]]=code[k];
            }
            j++;
        }
    }
}
/* decode type msm message header --------------------------------------------*/
static int decode_msm_head(rtcm_t *rtcm, int sys, int *sync, int *iod,
                           msm_h_t *h, int *hsize)
{
    msm_h_t h0={0};
    double tow,tod;
    char *msg;
    int i=24,j,dow,mask,staid,type,ncell=0;
    
    type=getbitu(rtcm->buff,i,12); i+=12;
    
    *h=h0;
    if (i+157<=rtcm->len*8) {
        staid     =getbitu(rtcm->buff,i,12);       i+=12;
        
        if (sys==SYS_GLO) {
            dow   =getbitu(rtcm->buff,i, 3);       i+= 3;
            tod   =getbitu(rtcm->buff,i,27)*0.001; i+=27;
            adjday_glot(rtcm,tod);
        }
        else if (sys==SYS_CMP) {
            tow   =getbitu(rtcm->buff,i,30)*0.001; i+=30;
            tow+=14.0; /* BDT -> GPST */
            adjweek(rtcm,tow);
        }
        else {
            tow   =getbitu(rtcm->buff,i,30)*0.001; i+=30;
            adjweek(rtcm,tow);
        }
        *sync     =getbitu(rtcm->buff,i, 1);       i+= 1;
        *iod      =getbitu(rtcm->buff,i, 3);       i+= 3;
        h->time_s =getbitu(rtcm->buff,i, 7);       i+= 7;
        h->clk_str=getbitu(rtcm->buff,i, 2);       i+= 2;
        h->clk_ext=getbitu(rtcm->buff,i, 2);       i+= 2;
        h->smooth =getbitu(rtcm->buff,i, 1);       i+= 1;
        h->tint_s =getbitu(rtcm->buff,i, 3);       i+= 3;
        for (j=1;j<=64;j++) {
            mask=getbitu(rtcm->buff,i,1); i+=1;
            if (mask) h->sats[h->nsat++]=j;
        }
        for (j=1;j<=32;j++) {
            mask=getbitu(rtcm->buff,i,1); i+=1;
            if (mask) h->sigs[h->nsig++]=j;
        }
    }
    else {
        trace(2,"rtcm3 %d length error: len=%d\n",type,rtcm->len);
        return -1;
    }
    /* test station id */
    if (!test_staid(rtcm,staid)) return -1;
    
    if (h->nsat*h->nsig>64) {
        trace(2,"rtcm3 %d number of sats and sigs error: nsat=%d nsig=%d\n",
              type,h->nsat,h->nsig);
        return -1;
    }
    if (i+h->nsat*h->nsig>rtcm->len*8) {
        trace(2,"rtcm3 %d length error: len=%d nsat=%d nsig=%d\n",type,
              rtcm->len,h->nsat,h->nsig);
        return -1;
    }
    for (j=0;j<h->nsat*h->nsig;j++) {
        h->cellmask[j]=getbitu(rtcm->buff,i,1); i+=1;
        if (h->cellmask[j]) ncell++;
    }
    *hsize=i;
    
    trace(4,"decode_head_msm: time=%s sys=%d staid=%d nsat=%d nsig=%d sync=%d iod=%d ncell=%d\n",
          time_str(rtcm->time,2),sys,staid,h->nsat,h->nsig,*sync,*iod,ncell);
    
    if (rtcm->outtype) {
        msg=rtcm->msgtype+strlen(rtcm->msgtype);
        sprintf(msg," staid=%4d %s nsat=%2d nsig=%2d iod=%2d ncell=%2d sync=%d",
                staid,time_str(rtcm->time,2),h->nsat,h->nsig,*iod,ncell,*sync);
    }
    return ncell;
}
/* decode unsupported msm message --------------------------------------------*/
static int decode_msm0(rtcm_t *rtcm, int sys)
{
    msm_h_t h={0};
    int i,sync,iod;
    if (decode_msm_head(rtcm,sys,&sync,&iod,&h,&i)<0) return -1;
    rtcm->obsflag=!sync;
    return sync?0:1;
}
/* decode msm 4: full pseudorange and phaserange plus cnr --------------------*/
static int decode_msm4(rtcm_t *rtcm, int sys)
{
    msm_h_t h={0};
    double r[64],pr[64],cp[64],cnr[64];
    int i,j,type,sync,iod,ncell,rng,rng_m,prv,cpv,lock[64],half[64];
    
    type=getbitu(rtcm->buff,24,12);
    
    /* decode msm header */
    if ((ncell=decode_msm_head(rtcm,sys,&sync,&iod,&h,&i))<0) return -1;
    
    if (i+h.nsat*18+ncell*48>rtcm->len*8) {
        trace(2,"rtcm3 %d length error: nsat=%d ncell=%d len=%d\n",type,h.nsat,
              ncell,rtcm->len);
        return -1;
    }
    for (j=0;j<h.nsat;j++) r[j]=0.0;
    for (j=0;j<ncell;j++) pr[j]=cp[j]=-1E16;
    
    /* decode satellite data */
    for (j=0;j<h.nsat;j++) { /* range */
        rng  =getbitu(rtcm->buff,i, 8); i+= 8;
        if (rng!=255) r[j]=rng*RANGE_MS;
    }
    for (j=0;j<h.nsat;j++) {
        rng_m=getbitu(rtcm->buff,i,10); i+=10;
        if (r[j]!=0.0) r[j]+=rng_m*P2_10*RANGE_MS;
    }
    /* decode signal data */
    for (j=0;j<ncell;j++) { /* pseudorange */
        prv=getbits(rtcm->buff,i,15); i+=15;
        if (prv!=-16384) pr[j]=prv*P2_24*RANGE_MS;
    }
    for (j=0;j<ncell;j++) { /* phaserange */
        cpv=getbits(rtcm->buff,i,22); i+=22;
        if (cpv!=-2097152) cp[j]=cpv*P2_29*RANGE_MS;
    }
    for (j=0;j<ncell;j++) { /* lock time */
        lock[j]=getbitu(rtcm->buff,i,4); i+=4;
    }
    for (j=0;j<ncell;j++) { /* half-cycle ambiguity */
        half[j]=getbitu(rtcm->buff,i,1); i+=1;
    }
    for (j=0;j<ncell;j++) { /* cnr */
        cnr[j]=getbitu(rtcm->buff,i,6)*1.0; i+=6;
    }
    /* save obs data in msm message */
    save_msm_obs(rtcm,sys,&h,r,pr,cp,NULL,NULL,cnr,lock,NULL,half);
    
    rtcm->obsflag=!sync;
    return sync?0:1;
}
/* decode msm 5: full pseudorange, phaserange, phaserangerate and cnr --------*/
static int decode_msm5(rtcm_t *rtcm, int sys)
{
    msm_h_t h={0};
    double r[64],rr[64],pr[64],cp[64],rrf[64],cnr[64];
    int i,j,type,sync,iod,ncell,rng,rng_m,rate,prv,cpv,rrv,lock[64];
    int ex[64],half[64];
    
    type=getbitu(rtcm->buff,24,12);
    
    /* decode msm header */
    if ((ncell=decode_msm_head(rtcm,sys,&sync,&iod,&h,&i))<0) return -1;
    
    if (i+h.nsat*36+ncell*63>rtcm->len*8) {
        trace(2,"rtcm3 %d length error: nsat=%d ncell=%d len=%d\n",type,h.nsat,
              ncell,rtcm->len);
        return -1;
    }
    for (j=0;j<h.nsat;j++) {
        r[j]=rr[j]=0.0; ex[j]=15;
    }
    for (j=0;j<ncell;j++) pr[j]=cp[j]=rrf[j]=-1E16;
    
    /* decode satellite data */
    for (j=0;j<h.nsat;j++) { /* range */
        rng  =getbitu(rtcm->buff,i, 8); i+= 8;
        if (rng!=255) r[j]=rng*RANGE_MS;
    }
    for (j=0;j<h.nsat;j++) { /* extended info */
        ex[j]=getbitu(rtcm->buff,i, 4); i+= 4;
    }
    for (j=0;j<h.nsat;j++) {
        rng_m=getbitu(rtcm->buff,i,10); i+=10;
        if (r[j]!=0.0) r[j]+=rng_m*P2_10*RANGE_MS;
    }
    for (j=0;j<h.nsat;j++) { /* phaserangerate */
        rate =getbits(rtcm->buff,i,14); i+=14;
        if (rate!=-8192) rr[j]=rate*1.0;
    }
    /* decode signal data */
    for (j=0;j<ncell;j++) { /* pseudorange */
        prv=getbits(rtcm->buff,i,15); i+=15;
        if (prv!=-16384) pr[j]=prv*P2_24*RANGE_MS;
    }
    for (j=0;j<ncell;j++) { /* phaserange */
        cpv=getbits(rtcm->buff,i,22); i+=22;
        if (cpv!=-2097152) cp[j]=cpv*P2_29*RANGE_MS;
    }
    for (j=0;j<ncell;j++) { /* lock time */
        lock[j]=getbitu(rtcm->buff,i,4); i+=4;
    }
    for (j=0;j<ncell;j++) { /* half-cycle ambiguity */
        half[j]=getbitu(rtcm->buff,i,1); i+=1;
    }
    for (j=0;j<ncell;j++) { /* cnr */
        cnr[j]=getbitu(rtcm->buff,i,6)*1.0; i+=6;
    }
    for (j=0;j<ncell;j++) { /* phaserangerate */
        rrv=getbits(rtcm->buff,i,15); i+=15;
        if (rrv!=-16384) rrf[j]=rrv*0.0001;
    }
    /* save obs data in msm message */
    save_msm_obs(rtcm,sys,&h,r,pr,cp,rr,rrf,cnr,lock,ex,half);
    
    rtcm->obsflag=!sync;
    return sync?0:1;
}
/* decode msm 6: full pseudorange and phaserange plus cnr (high-res) ---------*/
static int decode_msm6(rtcm_t *rtcm, int sys)
{
    msm_h_t h={0};
    double r[64],pr[64],cp[64],cnr[64];
    int i,j,type,sync,iod,ncell,rng,rng_m,prv,cpv,lock[64],half[64];
    
    type=getbitu(rtcm->buff,24,12);
    
    /* decode msm header */
    if ((ncell=decode_msm_head(rtcm,sys,&sync,&iod,&h,&i))<0) return -1;
    
    if (i+h.nsat*18+ncell*65>rtcm->len*8) {
        trace(2,"rtcm3 %d length error: nsat=%d ncell=%d len=%d\n",type,h.nsat,
              ncell,rtcm->len);
        return -1;
    }
    for (j=0;j<h.nsat;j++) r[j]=0.0;
    for (j=0;j<ncell;j++) pr[j]=cp[j]=-1E16;
    
    /* decode satellite data */
    for (j=0;j<h.nsat;j++) { /* range */
        rng  =getbitu(rtcm->buff,i, 8); i+= 8;
        if (rng!=255) r[j]=rng*RANGE_MS;
    }
    for (j=0;j<h.nsat;j++) {
        rng_m=getbitu(rtcm->buff,i,10); i+=10;
        if (r[j]!=0.0) r[j]+=rng_m*P2_10*RANGE_MS;
    }
    /* decode signal data */
    for (j=0;j<ncell;j++) { /* pseudorange */
        prv=getbits(rtcm->buff,i,20); i+=20;
        if (prv!=-524288) pr[j]=prv*P2_29*RANGE_MS;
    }
    for (j=0;j<ncell;j++) { /* phaserange */
        cpv=getbits(rtcm->buff,i,24); i+=24;
        if (cpv!=-8388608) cp[j]=cpv*P2_31*RANGE_MS;
    }
    for (j=0;j<ncell;j++) { /* lock time */
        lock[j]=getbitu(rtcm->buff,i,10); i+=10;
    }
    for (j=0;j<ncell;j++) { /* half-cycle ambiguity */
        half[j]=getbitu(rtcm->buff,i,1); i+=1;
    }
    for (j=0;j<ncell;j++) { /* cnr */
        cnr[j]=getbitu(rtcm->buff,i,10)*0.0625; i+=10;
    }
    /* save obs data in msm message */
    save_msm_obs(rtcm,sys,&h,r,pr,cp,NULL,NULL,cnr,lock,NULL,half);
    
    rtcm->obsflag=!sync;
    return sync?0:1;
}
/* decode msm 7: full pseudorange, phaserange, phaserangerate and cnr (h-res) */
static int decode_msm7(rtcm_t *rtcm, int sys)
{
    msm_h_t h={0};
    double r[64],rr[64],pr[64],cp[64],rrf[64],cnr[64];
    int i,j,type,sync,iod,ncell,rng,rng_m,rate,prv,cpv,rrv,lock[64];
    int ex[64],half[64];
    
    type=getbitu(rtcm->buff,24,12);
    
    /* decode msm header */
    if ((ncell=decode_msm_head(rtcm,sys,&sync,&iod,&h,&i))<0) return -1;
    
    if (i+h.nsat*36+ncell*80>rtcm->len*8) {
        trace(2,"rtcm3 %d length error: nsat=%d ncell=%d len=%d\n",type,h.nsat,
              ncell,rtcm->len);
        return -1;
    }
    for (j=0;j<h.nsat;j++) {
        r[j]=rr[j]=0.0; ex[j]=15;
    }
    for (j=0;j<ncell;j++) pr[j]=cp[j]=rrf[j]=-1E16;
    
    /* decode satellite data */
    for (j=0;j<h.nsat;j++) { /* range */
        rng  =getbitu(rtcm->buff,i, 8); i+= 8;
        if (rng!=255) r[j]=rng*RANGE_MS;
    }
    for (j=0;j<h.nsat;j++) { /* extended info */
        ex[j]=getbitu(rtcm->buff,i, 4); i+= 4;
    }
    for (j=0;j<h.nsat;j++) {
        rng_m=getbitu(rtcm->buff,i,10); i+=10;
        if (r[j]!=0.0) r[j]+=rng_m*P2_10*RANGE_MS;
    }
    for (j=0;j<h.nsat;j++) { /* phaserangerate */
        rate =getbits(rtcm->buff,i,14); i+=14;
        if (rate!=-8192) rr[j]=rate*1.0;
    }
    /* decode signal data */
    for (j=0;j<ncell;j++) { /* pseudorange */
        prv=getbits(rtcm->buff,i,20); i+=20;
        if (prv!=-524288) pr[j]=prv*P2_29*RANGE_MS;
    }
    for (j=0;j<ncell;j++) { /* phaserange */
        cpv=getbits(rtcm->buff,i,24); i+=24;
        if (cpv!=-8388608) cp[j]=cpv*P2_31*RANGE_MS;
    }
    for (j=0;j<ncell;j++) { /* lock time */
        lock[j]=getbitu(rtcm->buff,i,10); i+=10;
    }
    for (j=0;j<ncell;j++) { /* half-cycle amiguity */
        half[j]=getbitu(rtcm->buff,i,1); i+=1;
    }
    for (j=0;j<ncell;j++) { /* cnr */
        cnr[j]=getbitu(rtcm->buff,i,10)*0.0625; i+=10;
    }
    for (j=0;j<ncell;j++) { /* phaserangerate */
        rrv=getbits(rtcm->buff,i,15); i+=15;
        if (rrv!=-16384) rrf[j]=rrv*0.0001;
    }
    /* save obs data in msm message */
    save_msm_obs(rtcm,sys,&h,r,pr,cp,rr,rrf,cnr,lock,ex,half);
    
    rtcm->obsflag=!sync;
    return sync?0:1;
}
/* decode type 1230: glonass L1 and L2 code-phase biases ---------------------*/
static int decode_type1230(rtcm_t *rtcm)
{
    trace(2,"rtcm3 1230: not supported message\n");
    return 0;
}
/* decode rtcm ver.3 message -------------------------------------------------*/
extern int decode_rtcm3(rtcm_t *rtcm)
{
    double tow;
    int ret=0,type=getbitu(rtcm->buff,24,12),week;
    
    trace(3,"decode_rtcm3: len=%3d type=%d\n",rtcm->len,type);
    
    if (rtcm->outtype) {
        sprintf(rtcm->msgtype,"RTCM %4d (%4d):",type,rtcm->len);
    }
    /* real-time input option */
    if (strstr(rtcm->opt,"-RT_INP")) {
        tow=time2gpst(utc2gpst(timeget()),&week);
        rtcm->time=gpst2time(week,floor(tow));
    }
    switch (type) {
        case 1001: ret=decode_type1001(rtcm); break; /* not supported */
        case 1002: ret=decode_type1002(rtcm); break;
        case 1003: ret=decode_type1003(rtcm); break; /* not supported */
        case 1004: ret=decode_type1004(rtcm); break;
        case 1005: ret=decode_type1005(rtcm); break;
        case 1006: ret=decode_type1006(rtcm); break;
        case 1007: ret=decode_type1007(rtcm); break;
        case 1008: ret=decode_type1008(rtcm); break;
        case 1009: ret=decode_type1009(rtcm); break; /* not supported */
        case 1010: ret=decode_type1010(rtcm); break;
        case 1011: ret=decode_type1011(rtcm); break; /* not supported */
        case 1012: ret=decode_type1012(rtcm); break;
        case 1013: ret=decode_type1013(rtcm); break; /* not supported */
        case 1019: ret=decode_type1019(rtcm); break;
        case 1020: ret=decode_type1020(rtcm); break;
        case 1021: ret=decode_type1021(rtcm); break; /* not supported */
        case 1022: ret=decode_type1022(rtcm); break; /* not supported */
        case 1023: ret=decode_type1023(rtcm); break; /* not supported */
        case 1024: ret=decode_type1024(rtcm); break; /* not supported */
        case 1025: ret=decode_type1025(rtcm); break; /* not supported */
        case 1026: ret=decode_type1026(rtcm); break; /* not supported */
        case 1027: ret=decode_type1027(rtcm); break; /* not supported */
        case 1030: ret=decode_type1030(rtcm); break; /* not supported */
        case 1031: ret=decode_type1031(rtcm); break; /* not supported */
        case 1032: ret=decode_type1032(rtcm); break; /* not supported */
        case 1033: ret=decode_type1033(rtcm); break;
        case 1034: ret=decode_type1034(rtcm); break; /* not supported */
        case 1035: ret=decode_type1035(rtcm); break; /* not supported */
        case 1037: ret=decode_type1037(rtcm); break; /* not supported */
        case 1038: ret=decode_type1038(rtcm); break; /* not supported */
        case 1039: ret=decode_type1039(rtcm); break; /* not supported */
        case 1044: ret=decode_type1044(rtcm); break;
        case 1045: ret=decode_type1045(rtcm); break;
        case 1046: ret=decode_type1046(rtcm); break;
        case   63: ret=decode_type1042(rtcm); break; /* RTCM draft */
        case 1042: ret=decode_type1042(rtcm); break;
        case 1057: ret=decode_ssr1(rtcm,SYS_GPS); break;
        case 1058: ret=decode_ssr2(rtcm,SYS_GPS); break;
        case 1059: ret=decode_ssr3(rtcm,SYS_GPS); break;
        case 1060: ret=decode_ssr4(rtcm,SYS_GPS); break;
        case 1061: ret=decode_ssr5(rtcm,SYS_GPS); break;
        case 1062: ret=decode_ssr6(rtcm,SYS_GPS); break;
        case 1063: ret=decode_ssr1(rtcm,SYS_GLO); break;
        case 1064: ret=decode_ssr2(rtcm,SYS_GLO); break;
        case 1065: ret=decode_ssr3(rtcm,SYS_GLO); break;
        case 1066: ret=decode_ssr4(rtcm,SYS_GLO); break;
        case 1067: ret=decode_ssr5(rtcm,SYS_GLO); break;
        case 1068: ret=decode_ssr6(rtcm,SYS_GLO); break;
        case 1071: ret=decode_msm0(rtcm,SYS_GPS); break; /* not supported */
        case 1072: ret=decode_msm0(rtcm,SYS_GPS); break; /* not supported */
        case 1073: ret=decode_msm0(rtcm,SYS_GPS); break; /* not supported */
        case 1074: ret=decode_msm4(rtcm,SYS_GPS); break;
        case 1075: ret=decode_msm5(rtcm,SYS_GPS); break;
        case 1076: ret=decode_msm6(rtcm,SYS_GPS); break;
        case 1077: ret=decode_msm7(rtcm,SYS_GPS); break;
        case 1081: ret=decode_msm0(rtcm,SYS_GLO); break; /* not supported */
        case 1082: ret=decode_msm0(rtcm,SYS_GLO); break; /* not supported */
        case 1083: ret=decode_msm0(rtcm,SYS_GLO); break; /* not supported */
        case 1084: ret=decode_msm4(rtcm,SYS_GLO); break;
        case 1085: ret=decode_msm5(rtcm,SYS_GLO); break;
        case 1086: ret=decode_msm6(rtcm,SYS_GLO); break;
        case 1087: ret=decode_msm7(rtcm,SYS_GLO); break;
        case 1091: ret=decode_msm0(rtcm,SYS_GAL); break; /* not supported */
        case 1092: ret=decode_msm0(rtcm,SYS_GAL); break; /* not supported */
        case 1093: ret=decode_msm0(rtcm,SYS_GAL); break; /* not supported */
        case 1094: ret=decode_msm4(rtcm,SYS_GAL); break;
        case 1095: ret=decode_msm5(rtcm,SYS_GAL); break;
        case 1096: ret=decode_msm6(rtcm,SYS_GAL); break;
        case 1097: ret=decode_msm7(rtcm,SYS_GAL); break;
        case 1101: ret=decode_msm0(rtcm,SYS_SBS); break; /* not supported */
        case 1102: ret=decode_msm0(rtcm,SYS_SBS); break; /* not supported */
        case 1103: ret=decode_msm0(rtcm,SYS_SBS); break; /* not supported */
        case 1104: ret=decode_msm4(rtcm,SYS_SBS); break;
        case 1105: ret=decode_msm5(rtcm,SYS_SBS); break;
        case 1106: ret=decode_msm6(rtcm,SYS_SBS); break;
        case 1107: ret=decode_msm7(rtcm,SYS_SBS); break;
        case 1111: ret=decode_msm0(rtcm,SYS_QZS); break; /* not supported */
        case 1112: ret=decode_msm0(rtcm,SYS_QZS); break; /* not supported */
        case 1113: ret=decode_msm0(rtcm,SYS_QZS); break; /* not supported */
        case 1114: ret=decode_msm4(rtcm,SYS_QZS); break;
        case 1115: ret=decode_msm5(rtcm,SYS_QZS); break;
        case 1116: ret=decode_msm6(rtcm,SYS_QZS); break;
        case 1117: ret=decode_msm7(rtcm,SYS_QZS); break;
        case 1121: ret=decode_msm0(rtcm,SYS_CMP); break; /* not supported */
        case 1122: ret=decode_msm0(rtcm,SYS_CMP); break; /* not supported */
        case 1123: ret=decode_msm0(rtcm,SYS_CMP); break; /* not supported */
        case 1124: ret=decode_msm4(rtcm,SYS_CMP); break;
        case 1125: ret=decode_msm5(rtcm,SYS_CMP); break;
        case 1126: ret=decode_msm6(rtcm,SYS_CMP); break;
        case 1127: ret=decode_msm7(rtcm,SYS_CMP); break;
        case 1230: ret=decode_type1230(rtcm);     break; /* not supported */
        case 1240: ret=decode_ssr1(rtcm,SYS_GAL); break;
        case 1241: ret=decode_ssr2(rtcm,SYS_GAL); break;
        case 1242: ret=decode_ssr3(rtcm,SYS_GAL); break;
        case 1243: ret=decode_ssr4(rtcm,SYS_GAL); break;
        case 1244: ret=decode_ssr5(rtcm,SYS_GAL); break;
        case 1245: ret=decode_ssr6(rtcm,SYS_GAL); break;
        case 1246: ret=decode_ssr1(rtcm,SYS_QZS); break;
        case 1247: ret=decode_ssr2(rtcm,SYS_QZS); break;
        case 1248: ret=decode_ssr3(rtcm,SYS_QZS); break;
        case 1249: ret=decode_ssr4(rtcm,SYS_QZS); break;
        case 1250: ret=decode_ssr5(rtcm,SYS_QZS); break;
        case 1251: ret=decode_ssr6(rtcm,SYS_QZS); break;
        case 1252: ret=decode_ssr1(rtcm,SYS_SBS); break;
        case 1253: ret=decode_ssr2(rtcm,SYS_SBS); break;
        case 1254: ret=decode_ssr3(rtcm,SYS_SBS); break;
        case 1255: ret=decode_ssr4(rtcm,SYS_SBS); break;
        case 1256: ret=decode_ssr5(rtcm,SYS_SBS); break;
        case 1257: ret=decode_ssr6(rtcm,SYS_SBS); break;
        case 1258: ret=decode_ssr1(rtcm,SYS_CMP); break;
        case 1259: ret=decode_ssr2(rtcm,SYS_CMP); break;
        case 1260: ret=decode_ssr3(rtcm,SYS_CMP); break;
        case 1261: ret=decode_ssr4(rtcm,SYS_CMP); break;
        case 1262: ret=decode_ssr5(rtcm,SYS_CMP); break;
        case 1263: ret=decode_ssr6(rtcm,SYS_CMP); break;
    }
    if (ret>=0) {
        type-=1000;
        if (1<=type&&type<=299) rtcm->nmsg3[type]++; else rtcm->nmsg3[0]++;
    }
    return ret;
}
