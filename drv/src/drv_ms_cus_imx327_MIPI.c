/* Copyright (c) 2018-2019 Sigmastar Technology Corp.
 All rights reserved.

 Unless otherwise stipulated in writing, any and all information contained
herein regardless in any format shall remain the sole proprietary of
Sigmastar Technology Corp. and be kept in strict confidence
(Sigmastar Confidential Information) by the recipient.
Any unauthorized act including without limitation unauthorized disclosure,
copying, use, reproduction, sale, distribution, modification, disassembling,
reverse engineering and compiling of the contents of Sigmastar Confidential
Information is unlawful and strictly prohibited. Sigmastar hereby reserves the
rights to any and all damages, losses, costs and expenses resulting therefrom.
*/

#ifdef __cplusplus
extern "C"
{
#endif

#include <drv_sensor_common.h>
#include <sensor_i2c_api.h>
#include <drv_sensor.h>

#ifdef __cplusplus
}
#endif

SENSOR_DRV_ENTRY_IMPL_BEGIN_EX(IMX327_HDR);

#define SENSOR_CHANNEL_NUM (0)
#define SENSOR_CHANNEL_MODE_LINEAR CUS_SENSOR_CHANNEL_MODE_REALTIME_NORMAL
#define SENSOR_CHANNEL_MODE_SONY_DOL CUS_SENSOR_CHANNEL_MODE_RAW_STORE_HDR

//============================================
//MIPI config begin.
#define SENSOR_MIPI_LANE_NUM (4)
//#define SENSOR_MIPI_HDR_MODE (1) //0: Non-HDR mode. 1:Sony DOL mode
//MIPI config end.
//============================================

#define LOGD //(...)

//c11 extern int usleep(u32 usec);
//int usleep(u32 usec);

#define R_GAIN_REG 1
#define G_GAIN_REG 2
#define B_GAIN_REG 3

#undef SENSOR_DBG
#define SENSOR_DBG 0

///////////////////////////////////////////////////////////////
//          @@@                                                                                       //
//       @   @@      ==  S t a r t * H e r e ==                                            //
//            @@      ==  S t a r t * H e r e  ==                                            //
//            @@      ==  S t a r t * H e r e  ==                                           //
//         @@@@                                                                                  //
//                                                                                                     //
//      Start Step 1 --  show preview on LCM                                         //
//                                                                                                    ���?//
//  Fill these #define value and table with correct settings                        //
//      camera can work and show preview on LCM                                 //
//                                                                                                       //
///////////////////////////////////////////////////////////////

#define SENSOR_ISP_TYPE     ISP_EXT                   //ISP_EXT, ISP_SOC
#define F_number  22                                  // CFG, demo module
//#define SENSOR_DATAFMT      CUS_DATAFMT_BAYER        //CUS_DATAFMT_YUV, CUS_DATAFMT_BAYER
#define SENSOR_IFBUS_TYPE   CUS_SENIF_BUS_MIPI      //CFG //CUS_SENIF_BUS_PARL, CUS_SENIF_BUS_MIPI
#define SENSOR_MIPI_HSYNC_MODE PACKET_HEADER_EDGE1
#define SENSOR_MIPI_HSYNC_MODE_HDR_DOL PACKET_FOOTER_EDGE
#define SENSOR_DATAPREC     CUS_DATAPRECISION_12    //CFG //CUS_DATAPRECISION_8, CUS_DATAPRECISION_10
#define SENSOR_DATAMODE     CUS_SEN_10TO12_9098     //CFG
#define SENSOR_BAYERID      CUS_BAYER_GR//CUS_BAYER_RG            //CFG //CUS_BAYER_GB, CUS_BAYER_GR, CUS_BAYER_BG, CUS_BAYER_RG
#define SENSOR_BAYERID_HDR_DOL      CUS_BAYER_BG//CUS_BAYER_GR
#define SENSOR_RGBIRID      CUS_RGBIR_NONE
#define SENSOR_ORIT         CUS_ORIT_M0F0           //CUS_ORIT_M0F0, CUS_ORIT_M1F0, CUS_ORIT_M0F1, CUS_ORIT_M1F1,
//#define SENSOR_YCORDER      CUS_SEN_YCODR_YC       //CUS_SEN_YCODR_YC, CUS_SEN_YCODR_CY
#define lane_number 2
#define vc0_hs_mode 3   //0: packet header edge  1: line end edge 2: line start edge 3: packet footer edge
#define long_packet_type_enable 0x00 //UD1~UD8 (user define)
#define SENSOR_MAX_GAIN     (2818 * 1024)                  // max sensor again, a-gain * conversion-gain*d-gain
#define SENSOR_MIN_GAIN      (1 * 1024)
#define SENSOR_GAIN_DELAY_FRAME_COUNT      (2)
#define SENSOR_SHUTTER_DELAY_FRAME_COUNT      (2)

#define Preview_MCLK_SPEED  CUS_CMU_CLK_36MHZ//CUS_CMU_SCL_CLK_37P125M        //CFG //CUS_CMU_CLK_12M, CUS_CMU_CLK_16M, CUS_CMU_CLK_24M, CUS_CMU_CLK_27M
//#define Preview_line_period 30000                  ////HTS/PCLK=4455 pixels/148.5MHZ=30usec @MCLK=36MHz
//#define vts_30fps 1125//1346,1616                 //for 29.1fps @ MCLK=36MHz
#define Preview_line_period 30556//30580                  //(36M/37.125M)*30fps=29.091fps(34.375msec), hts=34.375/1125=30556,
#define Preview_line_period_HDR_DOL 14842//30535//30580   10^9 / (1158 * 2) / 29.091   //(36M/37.125M)*30.02fps=29.11fps(34.352msec), hts=34.352/1125=30556,
//#define Line_per_second     32727
#define vts_30fps  1125
#define vts_30fps_HDR_DOL  1158//1125//1090                              //for 29.091fps @ MCLK=36MHz //VMAX
#define Prv_Max_line_number 1080                    //maximum exposure line munber of sensor when preview
#define Preview_WIDTH       1920                    //resolution Width when preview
#define Preview_HEIGHT      1080                    //resolution Height when preview
//#define Preview_HEIGHT_HDR_DOL      1042                    //resolution Height when preview
#define Preview_MAX_FPS     30                     //fastest preview FPS
#define Preview_MIN_FPS     3                      //slowest preview FPS
#define Preview_CROP_START_X     1                      //CROP_START_X
#define Preview_CROP_START_Y     1                      //CROP_START_Y

#define SENSOR_I2C_ADDR    0x34                   //I2C slave address
#define SENSOR_I2C_SPEED   200000//200000 //300000// 240000                  //I2C speed, 60000~320000
#define SENSOR_I2C_CHANNEL 	1							//I2C Channel
#define SENSOR_I2C_PAD_MODE 2							//Pad/Mode Number

#define SENSOR_I2C_LEGACY  I2C_NORMAL_MODE     //usally set CUS_I2C_NORMAL_MODE,  if use old OVT I2C protocol=> set CUS_I2C_LEGACY_MODE
#define SENSOR_I2C_FMT     I2C_FMT_A16D8        //CUS_I2C_FMT_A8D8, CUS_I2C_FMT_A8D16, CUS_I2C_FMT_A16D8, CUS_I2C_FMT_A16D16

#define SENSOR_PWDN_POL     CUS_CLK_POL_POS        // if PWDN pin High can makes sensor in power down, set CUS_CLK_POL_POS
#define SENSOR_RST_POL      CUS_CLK_POL_NEG        // if RESET pin High can makes sensor in reset state, set CUS_CLK_POL_NEG

// VSYNC/HSYNC POL can be found in data sheet timing diagram,
// Notice: the initial setting may contain VSYNC/HSYNC POL inverse settings so that condition is different.

#define SENSOR_VSYNC_POL    CUS_CLK_POL_NEG        // if VSYNC pin High and data bus have data, set CUS_CLK_POL_POS
#define SENSOR_HSYNC_POL    CUS_CLK_POL_POS        // if HSYNC pin High and data bus have data, set CUS_CLK_POL_POS
#define SENSOR_PCLK_POL     CUS_CLK_POL_POS        // depend on sensor setting, sometimes need to try CUS_CLK_POL_POS or CUS_CLK_POL_NEG
//static int  drv_Fnumber = 22;

#if SENSOR_DBG == 1
//#define SENSOR_DMSG(args...) LOGD(args)
//#define SENSOR_DMSG(args...) LOGE(args)
//#define SENSOR_DMSG(args...) printf(args)
#elif SENSOR_DBG == 0
//#define SENSOR_DMSG(args...)
#endif

#undef SENSOR_NAME
#define SENSOR_NAME IMX327

int cus_camsensor_release_handle(ms_cus_sensor *handle);
static int pCus_SetAEGain(ms_cus_sensor *handle, u32 gain);
static int pCus_SetAEUSecs(ms_cus_sensor *handle, u32 us);
static int pCus_SetFPS(ms_cus_sensor *handle, u32 fps);

CUS_CAMSENSOR_CAP sensor_cap = {
    .length = sizeof(CUS_CAMSENSOR_CAP),
    .version = 0x0001,
};

typedef struct {
    struct {
        u16 pre_div0;
        u16 div124;
        u16 div_cnt7b;
        u16 sdiv0;
        u16 mipi_div0;
        u16 r_divp;
        u16 sdiv1;
        u16 r_seld5;
        u16 r_sclk_dac;
        u16 sys_sel;
        u16 pdac_sel;
        u16 adac_sel;
        u16 pre_div_sp;
        u16 r_div_sp;
        u16 div_cnt5b;
        u16 sdiv_sp;
        u16 div12_sp;
        u16 mipi_lane_sel;
        u16 div_dac;
    } clk_tree;
    struct {
        bool bVideoMode;
        u16 res_idx;
        //        bool binning;
        //        bool scaling;
        CUS_CAMSENSOR_ORIT  orit;
    } res;
    struct {
        float sclk;
        u32 hts;
        u32 vts;
        u32 ho;
        u32 xinc;
        u32 line_freq;
        u32 us_per_line;
        u32 final_us;
        u32 final_gain;
        u32 back_pv_us;
        u32 fps;
    } expo;

    int sen_init;
    int still_min_fps;
    int video_min_fps;
    bool dirty;
    bool change;
    I2C_ARRAY tVts_reg[3];
    I2C_ARRAY tGain_reg[3];
    I2C_ARRAY tExpo_reg[3];
    I2C_ARRAY tShs2_reg[3];
    I2C_ARRAY tRhs1_reg[3];
    I2C_ARRAY tGain_hdr_dol_lef_reg[1];
    I2C_ARRAY tGain_hdr_dol_sef_reg[1];
} imx327_params;
// set sensor ID address and data,

const I2C_ARRAY Sensor_id_table[] =
{
    {0x3004, 0x10},      // {address of ID, ID },
    {0x3009, 0x01},      // {address of ID, ID },
};

#if 1 //exposure ratio 12 times mode.
const I2C_ARRAY Sensor_init_table_HDR_DOL_4lane[] =
{
    //DOL 2frame 1080p 12bit
    {0x3002,    0x00},
    {0x3405,    0x10},

    //FRSEL
    {0x3009,    0x01},//{0x3009,    0x01},
    {0x300a,    0xf0},//{0x300a,    0x3c},//{0x300a,    0xf0},

    //WDMODE, WDSEL[1:0]
    {0x300c,    0x11},
    {0x3011,    0x02},

    //VMAX
    {0x3018,    0x86},//{0x3018,    0xca}, //{0x3018,    0x65}, //VMAX
    {0x3019,    0x04},//{0x3019,    0x08}, //{0x3019,    0x04},
    {0x301a,    0x00},

    //HMAX
    {0x301c,    0x58}, //HMAX
    {0x301d,    0x08},

    {0x3129,    0x00},
    {0x313b,    0x61},
    {0x3444,    0x20},
    {0x3445,    0x25},

    //ODBIT[1:0]
    {0x3046,    0x01},
    {0x3446,    0x57},
    {0x3448,    0x37},
    {0x344a,    0x1f},
    {0x344c,    0x1f},
    {0x344e,    0x1f},
    {0x3450,    0x77},
    {0x3452,    0x1f},
    {0x3454,    0x17},
    {0x305c,    0x18},
    {0x305d,    0x03},
    {0x305e,    0x20},
    {0x315e,    0x1a},
    {0x3164,    0x1a},
    {0x317c,    0x00},
    {0x3480,    0x49},
    {0x309e,    0x4a},
    {0x309f,    0x4a},
    {0x30d2,    0x19},//?��?????�蕭??30D2h?????�蕭?�??��?19h.
    {0x30d7,    0x03},
    {0x31ec,    0x0e},

    //FPGC, GAIN [7:0]
    {0x3010, 0x61},
    {0x3014, 0x00},//{0x3014, 0x28},

    //FPGC_1 ,GAIN1 [7:0]
    {0x30F0, 0x64},
    {0x30F2, 0x00},//{0x30F2, 0x64},//{0x30F2, 0x80},

    //FPGC_2 ,GAIN2 [7:0]
    {0x30F4, 0x64},
    {0x30F6, 0x00},//{0x30F6, 0x20},

    //XVSMSKCNT, XVSMSKCNT_INT
    {0x3106, 0x11},

    //ADBIT
    {0x3005, 0x01},

    //WINMODE[2:0]
    {0x3007, 0x04},//M0F0

    //SHS1
    {0x3020,    0x02},//{0x3020,    0x02},
    {0x3021,    0x00},
    {0x3022,    0x00},

    //SHS2
    {0x3024,    0x13},//{0x3024,    0x40},//{0x3024,    0xa0},//{0x3024,    0x40},//{0x3024,    0x9c},
    {0x3025,    0x04},//{0x3025,    0x0f},//{0x3025,    0x07},//{0x3025,    0x0f},//{0x3025,    0x07},
    {0x3026,    0x00},

    //SHS3
    {0x3028,    0x00},
    {0x3029,    0x00},
    {0x302a,    0x00},

    //RHS1
    {0x3030,    0x6d},
    //{0x3030,    0x39/*n = 26, rhs1 = 26 * 2 + 5 = 57*/},//{0x3030,    0x6d},//{0x3030,    0x0b},
    //{0x3030,    0x1F/*n = 13, rhs1 = 13 * 2 + 5 = 31*/},//{0x3030,    0x6d},//{0x3030,    0x0b},
    {0x3031,    0x00},
    {0x3032,    0x00},

    //RHS2
    {0x3034,    0x00},
    {0x3035,    0x00},
    {0x3036,    0x00},

    //HINFOEN
    {0x3045,    0x05},

    //NULL0_SIZE_V
    {0x3415,    0x00},//{0x3415,    0x00},

    //y-out size
    {0x3418,    0x7A},
    {0x3419,    0x09},

    //x-out size
    {0x3472, 0xa0},
    {0x3473, 0x07},

    //MIF_SYNC_TIM0
    {0x347b, 0x23},

    //OPB_SIZE_V[5:0]
    {0x3414, 0x00},

    //WINWV_OB[3:0]
    {0x303A, 0x08},

    //WINPV[10:0]
    {0x303C, 0x04},
    {0x303D, 0x00},

    //WINWV[10:0]
    {0x303E, 0x41},
    {0x303F, 0x04},

#if 1
    {0x3204, 0x4A},
    {0x3209, 0xF0},
    {0x320A, 0x23},
    {0x3344, 0x38},
#endif

    {0x3000, 0x00},   // operating
};
#endif

#if 0  //exposure ratio 32 times mode.
const I2C_ARRAY Sensor_init_table_HDR_DOL_4lane[] =
{
    //DOL 2frame 1080p 12bit
    {0x3002,    0x00},
    {0x3405,    0x10},
    {0x3009,    0x11},//{0x3009,    0x01},
    {0x300a,    0xf0},//{0x300a,    0x3c},//{0x300a,    0xf0},
    {0x300c,    0x11},
    {0x3011,    0x02},
    {0x3018,    0x65},//{0x3018,    0xca}, //{0x3018,    0x65}, //VMAX
    {0x3019,    0x04},//{0x3019,    0x08}, //{0x3019,    0x04},
    {0x301a,    0x00},
    {0x301c,    0x98}, //HMAX
    {0x301d,    0x08},
    {0x3129,    0x00},
    {0x313b,    0x61},
    {0x3444,    0x20},
    {0x3445,    0x25},
    {0x3046,    0x01},
    {0x3446,    0x57},
    {0x3448,    0x37},
    {0x344a,    0x1f},
    {0x344c,    0x1f},
    {0x344e,    0x1f},
    {0x3450,    0x77},
    {0x3452,    0x1f},
    {0x3454,    0x17},
    {0x305c,    0x18},
    {0x305d,    0x03},
    {0x305e,    0x20},
    {0x315e,    0x1a},
    {0x3164,    0x1a},
    {0x317c,    0x00},
    {0x3480,    0x49},
    {0x309e,    0x4a},
    {0x309f,    0x4a},
    {0x30d2,    0x19},//?��?30D2h也�?要設19h.
    {0x30d7,    0x03},
    {0x31ec,    0x0e},

    //FPGC, GAIN [7:0]
    {0x3010, 0x61},
    {0x3014, 0x10},//{0x3014, 0x28},

    //FPGC_1 ,GAIN1 [7:0]
    {0x30F0, 0x64},
    {0x30F2, 0x32},//{0x30F2, 0x64},//{0x30F2, 0x80},

    //FPGC_2 ,GAIN2 [7:0]
    {0x30F4, 0x64},
    {0x30F6, 0x00},//{0x30F6, 0x20},

    //XVSMSKCNT, XVSMSKCNT_INT
    {0x3106, 0x11},

    //ADBIT
    {0x3005, 0x01},

    {0x3007, 0x00},//M0F0

    //SHS1
    {0x3020,    0x02},
    {0x3021,    0x00},
    {0x3022,    0x00},

    //SHS2
    {0x3024,    0xc9},//{0x3024,    0x40},//{0x3024,    0xa0},//{0x3024,    0x40},//{0x3024,    0x9c},
    {0x3025,    0x07},//{0x3025,    0x0f},//{0x3025,    0x07},//{0x3025,    0x0f},//{0x3025,    0x07},
    {0x3026,    0x00},

    //SHS3
    {0x3028,    0x00},
    {0x3029,    0x00},
    {0x302a,    0x00},

    //RHS1
    {0x3030,    0x0b},//{0x3030,    0x0b},
    {0x3031,    0x00},
    {0x3032,    0x00},

    //RHS2
    {0x3034,    0x00},
    {0x3035,    0x00},
    {0x3036,    0x00},

    //HINFOEN
    {0x3045,    0x05},

    //NULL0_SIZE_V
    {0x3415,    0x01},//{0x3415,    0x00},

    //y-out size
    {0x3418,    0x9C},
    {0x3419,    0x08},

    //x-out size
    {0x3472, 0xa0},
    {0x3473, 0x07},

    //MIF_SYNC_TIM0
    {0x347b, 0x23},

    {0x3000, 0x00},   // operating
};
#endif

const I2C_ARRAY Sensor_init_table_4lane[] =
{

    {0x3002, 0x01},   //Master mode stop
    {0xffff, 0x14},//delay
    {0x3000, 0x01},   // standby
    {0xffff, 0x14},//delay
    {0x3005, 0x01},
    {0x3007, 0x00},//mirror/flip
    {0x3009, 0x02},//FRSEL
  //  {0x300A, 0xF0},
  //  {0x300F, 0x00},
  //  {0x3010, 0x21},
    {0x3011, 0x0A},
    {0x3012, 0x64},
    {0x3013, 0x00},
    //{0x3016, 0x08},//yc modify
    {0x3018, 0x65},//VMAX
    {0x3019, 0x04},
    {0x301a, 0x00},
    {0x301c, 0x30},//0x1167 HMAX,for 25fps
    {0x301d, 0x11},

    {0x3046, 0x01},
    {0x304B, 0x0a},//arbitary value
    {0x305C, 0x18},//INCK
    {0x305D, 0x03},
    {0x305E, 0x20},
    {0x305F, 0x01},

    //{0x3070, 0x02},
   // {0x3071, 0x11},
   // {0x309B, 0x10},
   // {0x309C, 0x22},
    {0x309E, 0x4A},
    {0x309F, 0x4A},
 //   {0x30A0, 0x02},//new add
 //   {0x30A2, 0x02},
 //   {0x30A6, 0x20},
 //   {0x30A8, 0x20},
  //  {0x30AA, 0x20},
 //   {0x30AC, 0x20},
  //  {0x30B0, 0x43},

   // {0x3119, 0x9E},
    {0x311C, 0x0E},
  //  {0x311E, 0x08},
    {0x3128, 0x04},
    {0x3129, 0x00},
    {0x313B, 0x41},
 //   {0x313D, 0x83},
   // {0x3150, 0x02},
    {0x315E, 0x1A},//INCKSEL5
    {0x3164, 0x1A},//INCKSEL6
    {0x317C, 0x00},//ADBIT2
    {0x317E, 0x00},
    {0x31EC, 0x0E},


    {0x3405, 0x20},
    {0x3407, 0x03},  // 4 lane for phy
  //  {0x3414, 0x0A},
    {0x3418, 0x49},//Y-out
    {0x3419, 0x04},
  //  {0x342C, 0x47},
  //  {0x342D, 0x00},
   // {0x3430, 0x0F},
  //  {0x3431, 0x00},
    {0x3441, 0x0C},
    {0x3442, 0x0C},
    {0x3443, 0x03},  // 4 lane
    {0x3444, 0x20},
    {0x3445, 0x25},
    {0x3446, 0x47},
    {0x3447, 0x00},
    {0x3448, 0x1F},
    {0x3449, 0x00},
    {0x344A, 0x17},
    {0x344B, 0x00},
    {0x344C, 0x0F},
    {0x344D, 0x00},
    {0x344E, 0x17},
    {0x344F, 0x00},
    {0x3450, 0x47},
    {0x3451, 0x00},
    {0x3452, 0x0F},
    {0x3453, 0x00},
    {0x3454, 0x0F},
    {0x3455, 0x00},
    {0x3472, 0x9c},//x-out size
    {0x3473, 0x07},
    {0x3480, 0x49},//0x9c

    {0x3000, 0x00},   // operating
    //{0xffff, 0x14},

    {0x3002, 0x00},   //Master mode start

};

const I2C_ARRAY Sensor_init_table_2lane[] =
{
#if 1
    {0x3002, 0x01},   //Master mode stop
    {0xffff, 0x14},//delay
    {0x3000, 0x01},   // standby
    {0xffff, 0x14},//delay
    {0x3005, 0x01},
    {0x3007, 0x00},//mirror/flip
    {0x3009, 0x02},//FRSEL
  //  {0x300A, 0xF0},
  //  {0x300F, 0x00},
  //  {0x3010, 0x21},
    {0x3011, 0x0A},
  //  {0x3012, 0x64},
  //  {0x3013, 0x00},
    {0x3016, 0x08},//yc modify
    {0x3018, 0x65},//VMAX
    {0x3019, 0x04},
    {0x301c, 0x30},//0x1167 HMAX,for 25fps
    {0x301d, 0x11},

    //{0x3046, 0x01},
    {0x304B, 0x0a},//arbitary value
    {0x305C, 0x18},//INCK
    {0x305D, 0x03},
    {0x305E, 0x20},
    //{0x305F, 0x01},

    //{0x3070, 0x02},
   // {0x3071, 0x11},
   // {0x309B, 0x10},
   // {0x309C, 0x22},
    {0x309E, 0x4A},
    {0x309F, 0x4A},
 //   {0x30A0, 0x02},//new add
 //   {0x30A2, 0x02},
 //   {0x30A6, 0x20},
 //   {0x30A8, 0x20},
  //  {0x30AA, 0x20},
 //   {0x30AC, 0x20},
  //  {0x30B0, 0x43},

   // {0x3119, 0x9E},
    {0x311C, 0x0E},
  //  {0x311E, 0x08},
    {0x3128, 0x04},
    {0x3129, 0x00},
    {0x313B, 0x41},
 //   {0x313D, 0x83},
   // {0x3150, 0x02},
    {0x315E, 0x1A},//INCKSEL5
    {0x3164, 0x1A},//INCKSEL6
    {0x317C, 0x00},//ADBIT2
    {0x317E, 0x00},
    {0x31EC, 0x0E},


    {0x3405, 0x10},
    {0x3407, 0x01},  // 2 lane for phy
  //  {0x3414, 0x0A},
  //  {0x3418, 0x49},//Y-out
  //  {0x3419, 0x04},
  //  {0x342C, 0x47},
  //  {0x342D, 0x00},
   // {0x3430, 0x0F},
  //  {0x3431, 0x00},
  //  {0x3441, 0x0C},
  //  {0x3442, 0x0C},
    {0x3443, 0x01},  // 2 lane
    //{0x3444, 0x40},
  //  {0x3445, 0x4A},
    {0x3446, 0x57},
  //  {0x3447, 0x00},
    {0x3448, 0x37},
   // {0x3449, 0x00},
    {0x344A, 0x1F},
   // {0x344B, 0x00},
    {0x344C, 0x1F},
  //  {0x344D, 0x00},
  //  {0x344E, 0x17},
   // {0x344F, 0x00},
    {0x3450, 0x77},
  //  {0x3451, 0x00},
    {0x3452, 0x1F},
  //  {0x3453, 0x00},
    {0x3454, 0x17},
   // {0x3472, 0x9c},//x-out size
   // {0x3473, 0x07},
    {0x3480, 0x49},//0x9c

    {0x3000, 0x00},   // operating
    {0xffff, 0x14},

#endif
#if 0
    {0x3002, 0x01},   //Master mode stop
    {0xffff, 0x14},//delay
    {0x3000, 0x01},   // standby
    {0xffff, 0x14},//delay
    {0x3005, 0x01},
    {0x3007, 0x00},//mirror/flip
    {0x3009, 0x02},//FRSEL
    {0x300A, 0xF0},
    {0x300F, 0x00},
    {0x3010, 0x21},
    {0x3012, 0x64},
    {0x3013, 0x00},
    {0x3016, 0x08},//yc modify
    {0x3018, 0x65},//VMAX
    {0x3019, 0x04},
    {0x301c, 0x30},//0x1167 HMAX,for 25fps
    {0x301d, 0x11},

    {0x3046, 0x01},
    {0x304B, 0x0a},
    {0x305C, 0x18},//INCK
    {0x305D, 0x03},
    {0x305E, 0x20},
    {0x305F, 0x01},

    {0x3070, 0x02},
    {0x3071, 0x11},
    {0x309B, 0x10},
    {0x309C, 0x22},
 //   {0x30A0, 0x02},//new add
    {0x30A2, 0x02},
    {0x30A6, 0x20},
    {0x30A8, 0x20},
    {0x30AA, 0x20},
    {0x30AC, 0x20},
    {0x30B0, 0x43},

    {0x3119, 0x9E},
    {0x311C, 0x1E},
    {0x311E, 0x08},
    {0x3128, 0x05},
    {0x3129, 0x00},
    {0x313D, 0x83},
    {0x3150, 0x03},
    {0x315E, 0x1A},//INCKSEL5
    {0x3164, 0x1A},//INCKSEL6
    {0x317C, 0x00},//ADBIT2
    {0x317E, 0x00},
    {0x31EC, 0x0E},

    {0x32b8, 0x50},
    {0x32b9, 0x10},
    {0x32ba, 0x00},
    {0x32bb, 0x04},
    {0x32C8, 0x50},
    {0x32C9, 0x10},
    {0x32CA, 0x00},
    {0x32CB, 0x04},

    {0x332c, 0xD3},
    {0x332d, 0x10},
    {0x332e, 0x0D},
    {0x3358, 0x06},
    {0x3359, 0xE1},
    {0x335A, 0x11},
    {0x3360, 0x1E},
    {0x3361, 0x61},
    {0x3362, 0x10},
    {0x33B0, 0x50},
    {0x33B2, 0x1A},
    {0x33B3, 0x04},

    {0x3405, 0x10},
    {0x3407, 0x01},  // 2 lane for phy
    {0x3414, 0x0A},
    {0x3418, 0x49},//Y-out
    {0x3419, 0x04},
    {0x342C, 0x47},
    {0x342D, 0x00},
    {0x3430, 0x0F},
    {0x3431, 0x00},
    {0x3441, 0x0C},
    {0x3442, 0x0C},
    {0x3443, 0x01},  // 2 lane
    {0x3444, 0x20},
    {0x3445, 0x25},
    {0x3446, 0x57},
    {0x3447, 0x00},
    {0x3448, 0x37},
    {0x3449, 0x00},
    {0x344A, 0x1F},
    {0x344B, 0x00},
    {0x344C, 0x1F},
    {0x344D, 0x00},
    {0x344E, 0x1F},
    {0x344F, 0x00},
    {0x3450, 0x77},
    {0x3451, 0x00},
    {0x3452, 0x1F},
    {0x3453, 0x00},
    {0x3454, 0x17},
    {0x3472, 0x9c},//x-out size
    {0x3473, 0x07},
    {0x3480, 0x49},//0x9c

    {0x3000, 0x00},   // operating
    {0xffff, 0x14},
#endif

   // {0x3002, 0x00},   //Master mode start
};

const I2C_ARRAY TriggerStartTbl[] = {
    {0x3002,0x00},//Master mode start
};

I2C_ARRAY PatternTbl[] = {
    {0x308c,0x20}, //colorbar pattern , bit 0 to enable
};

I2C_ARRAY Current_Mirror_Flip_Tbl[] = {
{0x3007, 0x00},//M0F0

};

/////////////////////////////////////////////////////////////////
//       @@@@@@                                                                                    //
//                 @@                                                                                    //
//             @@@                                                                                      //
//       @       @@                                                                                    //
//         @@@@                                                                                        //
//                                                                                                          //
//      Step 3 --  complete camera features                                              //
//                                                                                                         //
//                                                                                                         //
//  camera set EV, MWB, orientation, contrast, sharpness                          //
//   , saturation, and Denoise can work correctly.                                     //
//                                                                                                          //
/////////////////////////////////////////////////////////////////


const I2C_ARRAY mirr_flip_table[] =
{
    {0x3007, 0x00},//M0F0
    {0x3007, 0x02},//M1F0
    {0x3007, 0x01},//M0F1
    {0x3007, 0x03},//M1F1

};

typedef struct {
    short reg;
    char startbit;
    char stopbit;
} COLLECT_REG_SET;



const static I2C_ARRAY gain_reg[] =
{
    //{0x350A, 0x00},//bit0, high bit
    {0x3014, 0x00},//low bit
    {0x3009, 0x01},//hcg mode,bit 4
    {0x3016, 0x08},//
};

const static I2C_ARRAY gain_HDR_DOL_LEF_reg[] =
{
    {0x3014, 0x00},
};

const static I2C_ARRAY gain_HDR_DOL_SEF_reg[] =
{
    {0x30F2, 0x00},
};
#if 0
static I2C_ARRAY frame_drop_reg[] =
{
    //{0x350A, 0x00},//bit0, high bit
    {0x4202, 0x01}, //frame mask
    {0x3000, 0x23}, //timing reset
    {0x3000, 0xa0}, //low bit
};
#endif
//static int g_sensor_ae_min_gain = 1024;
static CUS_GAIN_GAP_ARRAY gain_gap_compensate[16] =
{  //compensate  gain gap
        {0, 0},
        {0, 0},
        {0, 0},
        {0, 0},
        {0, 0},
        {0, 0},
        {0, 0},
        {0, 0},
        {0, 0},
        {0, 0},
        {0, 0},
        {0, 0},
        {0, 0},
        {0, 0},
        {0, 0},
        {0, 0}
};

const I2C_ARRAY expo_reg[] =
{ //SHS1
    {0x3022, 0x00},
    {0x3021, 0x00},
    {0x3020, 0x02},
};

const I2C_ARRAY expo_SHS2_reg[] =
{
#if 1 //decreasing exposure ratio version.
    {0x3026, 0x00},
    {0x3025, 0x04},
    {0x3024, 0x13},
#else
    {0x3026, 0x00},
    {0x3025, 0x07},
    {0x3024, 0xc9},
#endif
};

const I2C_ARRAY expo_RHS1_reg[] =
{
#if 1 //decreasing exposure ratio version.
    {0x3032, 0x00},
    {0x3031, 0x00},
    {0x3030, 0x65}, /*101*/
#else
    {0x3032, 0x00},
    {0x3031, 0x00},
    {0x3030, 0x0b},
#endif
};

const I2C_ARRAY vts_reg[] =
{
#if 1 //decreasing exposure ratio version.
    {0x301a, 0x00},
    {0x3019, 0x04},
    {0x3018, 0x86},
#else
    {0x301a, 0x00},
    {0x3019, 0x04},
    {0x3018, 0x65},
#endif
};

const I2C_ARRAY dummy_line[] =
{
    {0x4701, 0x00},
    {0x4702, 0x00},
    {0x4703, 0x00},
};

CUS_INT_TASK_ORDER def_order = {
        .RunLength = 9,
        .Orders = {
                CUS_INT_TASK_AE|CUS_INT_TASK_VDOS|CUS_INT_TASK_AF,
                CUS_INT_TASK_AWB|CUS_INT_TASK_VDOS|CUS_INT_TASK_AF,
                CUS_INT_TASK_VDOS|CUS_INT_TASK_AF,
                CUS_INT_TASK_AE|CUS_INT_TASK_VDOS|CUS_INT_TASK_AF,
                CUS_INT_TASK_AWB|CUS_INT_TASK_VDOS|CUS_INT_TASK_AF,
                CUS_INT_TASK_VDOS|CUS_INT_TASK_AF,
                CUS_INT_TASK_AE|CUS_INT_TASK_VDOS|CUS_INT_TASK_AF,
                CUS_INT_TASK_AWB|CUS_INT_TASK_VDOS|CUS_INT_TASK_AF,
                CUS_INT_TASK_VDOS|CUS_INT_TASK_AF,
        },
};

/////////// function definition ///////////////////
#define SensorReg_Read(_reg,_data)     (handle->i2c_bus->i2c_rx(handle->i2c_bus,handle->i2c_cfg,_reg,_data))
#define SensorReg_Write(_reg,_data)    (handle->i2c_bus->i2c_tx(handle->i2c_bus,handle->i2c_cfg,_reg,_data))
#define SensorRegArrayW(_reg,_len)  (handle->i2c_bus->i2c_array_tx(handle->i2c_bus, handle->i2c_cfg,(_reg),(_len)))
#define SensorRegArrayR(_reg,_len)  (handle->i2c_bus->i2c_array_rx(handle->i2c_bus, handle->i2c_cfg,(_reg),(_len)))

/////////////////// sensor hardware dependent //////////////
#if 0
static int ISP_config_io(ms_cus_sensor *handle) {

    ISensorIfAPI *sensor_if = &handle->sensor_if_api;

    SENSOR_DMSG("[%s]", __FUNCTION__);

    sensor_if->HsyncPol(handle, handle->HSYNC_POLARITY);
    sensor_if->VsyncPol(handle, handle->VSYNC_POLARITY);
    sensor_if->ClkPol(handle, handle->PCLK_POLARITY);
    sensor_if->BayerFmt(handle, handle->bayer_id);
    sensor_if->DataBus(handle, handle->sif_bus);

    sensor_if->DataPrecision(handle, handle->data_prec);
    sensor_if->FmtConv(handle,  handle->data_mode);

    return SUCCESS;
}

static int pCus_poweron(ms_cus_sensor *handle, u32 idx)
{
    ISensorIfAPI *sensor_if = &handle->sensor_if_api;
    SENSOR_DMSG("[%s] ", __FUNCTION__);

    //Sensor power on sequence

    sensor_if->SetIOPad(idx, handle->sif_bus, handle->interface_attr.attr_mipi.mipi_lane_num);
    sensor_if->SetCSI_Lane(idx, handle->interface_attr.attr_mipi.mipi_lane_num, 1);
    sensor_if->SetCSI_LongPacketType(idx, 0, 0x1C00, 0);

    sensor_if->MCLK(idx, 1, handle->mclk);
    SENSOR_DMSG("[%s] reset low\n", __FUNCTION__);
    sensor_if->Reset(idx, handle->reset_POLARITY);//sensor_if->Reset(handle, 1,handle->reset_POLARITY);
    SENSOR_MSLEEP(1);
    SENSOR_DMSG("[%s] power low\n", __FUNCTION__);
    sensor_if->PowerOff(idx, handle->pwdn_POLARITY);//sensor_if->PowerOff(handle, 1, handle->pwdn_POLARITY);
    SENSOR_MSLEEP(1);

    // power -> high, reset -> high
    SENSOR_DMSG("[%s] power high\n", __FUNCTION__);
    sensor_if->PowerOff(idx, !handle->pwdn_POLARITY );//sensor_if->PowerOff(handle, 1, !handle->pwdn_POLARITY);
    SENSOR_MSLEEP(1);
    SENSOR_DMSG("[%s] reset high\n", __FUNCTION__);
    sensor_if->Reset(idx, !handle->reset_POLARITY );//sensor_if->Reset(handle, 1, !handle->reset_POLARITY);
    SENSOR_MSLEEP(1);

    //sensor_if->Set3ATaskOrder(handle, def_order);
    // pure power on
    //ISP_config_io(handle);
    sensor_if->PowerOff(idx, !handle->pwdn_POLARITY );//sensor_if->PowerOff(handle, 1, !handle->pwdn_POLARITY);
    SENSOR_MSLEEP(1);
    //handle->i2c_bus->i2c_open(handle->i2c_bus,&handle->i2c_cfg);
    return SUCCESS;
}

static int pCus_poweroff(ms_cus_sensor *handle, u32 idx)
{
    // power/reset low
    ISensorIfAPI *sensor_if = &handle->sensor_if_api;
    SENSOR_DMSG("[%s] power low\n", __FUNCTION__);
    sensor_if->PowerOff(idx, handle->pwdn_POLARITY );//sensor_if->PowerOff(handle, 1, handle->pwdn_POLARITY);
    //handle->i2c_bus->i2c_close(handle->i2c_bus);
    SENSOR_MSLEEP(1);
    //Set_csi_if(0, 0);
    //sensor_if->SetCSI_Clk( CUS_CSI_CLK_DISABLE );
    sensor_if->MCLK(idx, 0, handle->mclk );
#if 0
#if (SENSOR_MIPI_LANE_NUM == 2)
	sensor_if->SetCSI_Lane(SENSOR_PAD_GROUP_SET, 2, 0);
#endif
#if (SENSOR_MIPI_LANE_NUM == 4)
	sensor_if->SetCSI_Lane(SENSOR_PAD_GROUP_SET, 4, 0);
#endif
#endif
    return SUCCESS;
}

/////////////////// image function /////////////////////////
//Get and check sensor ID
//if i2c error or sensor id does not match then return FAIL
static int pCus_GetSensorID(ms_cus_sensor *handle, u32 *id)
{
    int i,n;
    int table_length= ARRAY_SIZE(Sensor_id_table);
    I2C_ARRAY id_from_sensor[ARRAY_SIZE(Sensor_id_table)];

    for(n=0;n<table_length;++n)
    {
      id_from_sensor[n].reg = Sensor_id_table[n].reg;
      id_from_sensor[n].data = 0;
    }

    *id =0;
    if(table_length>8) table_length=8;

    SENSOR_DMSG("\n\n[%s]", __FUNCTION__);

    for(n=0;n<4;++n) //retry , until I2C success
    {
      if(n>2) return FAIL;

      if( SensorRegArrayR((I2C_ARRAY*)id_from_sensor,table_length) == SUCCESS) //read sensor ID from I2C
		  break;
      else
		  SENSOR_MSLEEP(1);
    }

    //convert sensor id to u32 format
    for(i=0;i<table_length;++i)
    {
      if( id_from_sensor[i].data != Sensor_id_table[i].data )
        return FAIL;
      *id = id_from_sensor[i].data;
    }

    SENSOR_DMSG("[%s]IMX327 sensor ,Read sensor id, get 0x%x Success\n", __FUNCTION__, (int)*id);
    return SUCCESS;
}
#endif
static int imx327_SetPatternMode(ms_cus_sensor *handle,u32 mode)
{
    int i;
  switch(mode)
  {
  case 1:
    PatternTbl[0].data = 0x21; //enable
  break;
  case 0:
    PatternTbl[0].data &= 0xFE; //disable
  break;
  default:
    PatternTbl[0].data &= 0xFE; //disable
  break;
  }

  for(i=0;i< ARRAY_SIZE(PatternTbl);i++)
  {
    if(SensorReg_Write(PatternTbl[i].reg,PatternTbl[i].data) != SUCCESS)
    {
      //MSG("[%s:%d]Sensor init fail!!\n", __FUNCTION__, __LINE__);
      return FAIL;
    }
  }

  return SUCCESS;
}
#if 0//defined(__MV5_FPGA__)
static int pCus_I2CWrite(ms_cus_sensor *handle, unsigned short usreg, unsigned short usval)
{
    //unsigned short sen_data;

    SensorReg_Write(usreg, usval);
    //SensorReg_Read(usreg, &sen_data);
    //UartSendTrace("imx327_MIPI reg: 0x%x, data: 0x%x, read: 0x%x.\n", usreg, usval, sen_data);

    return SUCCESS;
}

static int pCus_I2CRead(ms_cus_sensor *handle, unsigned short usreg, unsigned short* pusval)
{
    unsigned short usread_data;

    //SensorReg_Read(usreg, &usread_data);
    *pusval = usread_data;
    //UartSendTrace("imx327_MIPI reg: 0x%x, data: 0x%x\n", usreg, usread_data);

    return SUCCESS;
}
#endif
#if 0
static int pCus_init_mipi2lane_linear(ms_cus_sensor *handle)
{
    //SENSOR_DMSG("\n\n[%s]", __FUNCTION__);
	int i,cnt=0;
    ISensorIfAPI *sensor_if = &(handle->sensor_if_api);
	short sen_data;
    sensor_if->PCLK( CUS_SNR_PCLK_MIPI_TOP );//sensor_if->PCLK(NULL,CUS_PCLK_MIPI_TOP);

    UartSendTrace("IMX327 Sensor_init_table_2lane\n");
    for(i=0;i< ARRAY_SIZE(Sensor_init_table_2lane);i++)
    {
		if(Sensor_init_table_2lane[i].reg==0xffff)
		{
			MsSleep(RTK_MS_TO_TICK(1));//usleep(1000*Sensor_init_table_2lane[i].data);
		}
		else
		{
			cnt = 0;
			while(SensorReg_Write(Sensor_init_table_2lane[i].reg,Sensor_init_table_2lane[i].data) != SUCCESS)
			{
				cnt++;
				//printf("Sensor_init_table_2lane -> Retry %d...\n",cnt);
				if(cnt>=10)
				{
					//printf("[%s:%d]Sensor init fail!!\n", __FUNCTION__, __LINE__);
					return FAIL;
				}
				//usleep(10*1000);
			}
			SensorReg_Read( Sensor_init_table_2lane[i].reg, &sen_data );
			UartSendTrace("IMX327 reg: 0x%x, data: 0x%x, read: 0x%x.\n",Sensor_init_table_2lane[i].reg, Sensor_init_table_2lane[i].data, sen_data);
		}
	}

    for(i=0;i< ARRAY_SIZE(PatternTbl);i++)
    {
        if(SensorReg_Write(PatternTbl[i].reg,PatternTbl[i].data) != SUCCESS)
        {
            //MSG("[%s:%d]Sensor init fail!!\n", __FUNCTION__, __LINE__);
            return FAIL;
        }
    }

    for(i=0;i< ARRAY_SIZE(TriggerStartTbl);i++)
    {
        if(SensorReg_Write(TriggerStartTbl[i].reg,TriggerStartTbl[i].data) != SUCCESS)
        {
            //MSG("[%s:%d]Sensor init fail!!\n", __FUNCTION__, __LINE__);
            return FAIL;
        }
    }

    for(i=0;i< ARRAY_SIZE(Current_Mirror_Flip_Tbl); i++)
    {
        if(SensorReg_Write(Current_Mirror_Flip_Tbl[i].reg,Current_Mirror_Flip_Tbl[i].data) != SUCCESS)
        {
            //MSG("[%s:%d]Sensor init fail!!\n", __FUNCTION__, __LINE__);
            return FAIL;
        }

    }

    pCus_SetAEGain(handle,2048);
    pCus_SetAEUSecs(handle, 25000);
    pCus_SetFPS(handle, 25000);

    return SUCCESS;
}
#endif
static int pCus_init_mipi4lane_linear(ms_cus_sensor *handle)
{
    //SENSOR_DMSG("\n\n[%s]", __FUNCTION__);
    int i,cnt=0;
    //ISensorIfAPI *sensor_if = &(handle->sensor_if_api);
    //short sen_data;
    //sensor_if->PCLK( CUS_SNR_PCLK_MIPI_TOP );//sensor_if->PCLK(NULL,CUS_PCLK_MIPI_TOP);

    //UartSendTrace("IMX327 Sensor_init_table_4lane\n");
    for(i=0;i< ARRAY_SIZE(Sensor_init_table_4lane);i++)
    {
        if(Sensor_init_table_4lane[i].reg==0xffff)
        {
            //MsSleep(RTK_MS_TO_TICK(1));//usleep(1000*Sensor_init_table_2lane[i].data);
            SENSOR_MSLEEP(Sensor_init_table_4lane[i].data);
        }
        else
        {
            cnt = 0;
            while(SensorReg_Write(Sensor_init_table_4lane[i].reg,Sensor_init_table_4lane[i].data) != SUCCESS)
            {
                cnt++;
                //printf("Sensor_init_table_2lane -> Retry %d...\n",cnt);
                if(cnt>=10)
                {
                    //printf("[%s:%d]Sensor init fail!!\n", __FUNCTION__, __LINE__);
                    return FAIL;
                }
                //usleep(10*1000);
            }
            //SensorReg_Read( Sensor_init_table_4lane[i].reg, &sen_data );
            //UartSendTrace("IMX327 reg: 0x%x, data: 0x%x, read: 0x%x.\n",Sensor_init_table_4lane[i].reg, Sensor_init_table_4lane[i].data, sen_data);
        }
    }

    return SUCCESS;
}

static int pCus_init_mipi4lane_HDR_DOL(ms_cus_sensor *handle)
{
    //SENSOR_DMSG("\n\n[%s]", __FUNCTION__);
    int i,cnt=0;
    //ISensorIfAPI *sensor_if = &(handle->sensor_if_api);
    //short sen_data;
    //sensor_if->PCLK( CUS_SNR_PCLK_MIPI_TOP );//sensor_if->PCLK(NULL,CUS_PCLK_MIPI_TOP);

    //UartSendTrace("IMX327 Sensor_init_table_HDR_DOL_4lane\n");
    for(i=0;i< ARRAY_SIZE(Sensor_init_table_HDR_DOL_4lane);i++)
    {
        if(Sensor_init_table_HDR_DOL_4lane[i].reg==0xffff)
        {
            //MsSleep(RTK_MS_TO_TICK(1));//usleep(1000*Sensor_init_table_HDR_DOL_4lane[i].data);
            SENSOR_MSLEEP(Sensor_init_table_HDR_DOL_4lane[i].data);
        }
        else
        {
            cnt = 0;
            pr_err("reg =  %x, data = %x\n", Sensor_init_table_HDR_DOL_4lane[i].reg, Sensor_init_table_HDR_DOL_4lane[i].data);
            while(SensorReg_Write(Sensor_init_table_HDR_DOL_4lane[i].reg,Sensor_init_table_HDR_DOL_4lane[i].data) != SUCCESS)
            {
                cnt++;
                 pr_err("Sensor_init_table_HDR_DOL_4lane -> Retry %d...\n",cnt);
                if(cnt>=10)
                {
                    //printf("[%s:%d]Sensor init fail!!\n", __FUNCTION__, __LINE__);
                    return FAIL;
                }
                //usleep(10*1000);
            }
            //SensorReg_Read( Sensor_init_table_HDR_DOL_4lane[i].reg, &sen_data );
            //UartSendTrace("IMX327 reg: 0x%x, data: 0x%x, read: 0x%x.\n",Sensor_init_table_HDR_DOL_4lane[i].reg, Sensor_init_table_HDR_DOL_4lane[i].data, sen_data);
        }
    }

    //MSG("[%s:%d]Sensor init success!!\n", __FUNCTION__, __LINE__);
    return SUCCESS;
}

/*
int pCus_release(ms_cus_sensor *handle)
{
    ISensorIfAPI *sensor_if = &handle->sensor_if_api;
    sensor_if->PCLK(NULL,CUS_PCLK_OFF);
    return SUCCESS;
}
*/

static int pCus_GetVideoResNum( ms_cus_sensor *handle, u32 *ulres_num)
{
    *ulres_num = handle->video_res_supported.num_res;
    return SUCCESS;
}

static int pCus_GetVideoRes(ms_cus_sensor *handle, u32 res_idx, cus_camsensor_res **res)
{
    //return current resolution
    *res = &handle->video_res_supported.res[res_idx];
    return SUCCESS;
}

static int pCus_SetVideoRes(ms_cus_sensor *handle, u32 res)
{
    handle->video_res_supported.ulcur_res = 0; //TBD

    return SUCCESS;
}

static int pCus_GetOrien(ms_cus_sensor *handle, CUS_CAMSENSOR_ORIT *orit)
{
    short sen_data;

    SensorReg_Read(0x3007, &sen_data);//always success now

    //LOGD("mirror:%x\r\n", sen_data & 0x03);
    switch(sen_data & 0x03)
    {
        case 0x00:
            *orit = CUS_ORIT_M0F0;
        break;
        case 0x02:
            *orit = CUS_ORIT_M1F0;
        break;
        case 0x01:
            *orit = CUS_ORIT_M0F1;
        break;
        case 0x03:
            *orit = CUS_ORIT_M1F1;
        break;
    }
    return SUCCESS;
}

static int pCus_SetOrien(ms_cus_sensor *handle, CUS_CAMSENSOR_ORIT orit) {
    int table_length = ARRAY_SIZE(mirr_flip_table);
    int seg_length=table_length/4;
    int i,j;

    SENSOR_DMSG("\n\n[%s]", __FUNCTION__);

    switch(orit)
    {
        case CUS_ORIT_M0F0:
            for(i=0,j=0;i<seg_length;i++,j++){
                SensorReg_Write(mirr_flip_table[i].reg,mirr_flip_table[i].data);
                Current_Mirror_Flip_Tbl[j].reg = mirr_flip_table[i].reg;
                Current_Mirror_Flip_Tbl[j].data = mirr_flip_table[i].data;
            }
	       //  handle->bayer_id=	CUS_BAYER_BG;
            break;

        case CUS_ORIT_M1F0:
            for(i=seg_length,j=0;i<seg_length*2;i++,j++){
                SensorReg_Write(mirr_flip_table[i].reg,mirr_flip_table[i].data);
                Current_Mirror_Flip_Tbl[j].reg = mirr_flip_table[i].reg;
                Current_Mirror_Flip_Tbl[j].data = mirr_flip_table[i].data;
            }
		//  handle->bayer_id=	CUS_BAYER_BG;
            break;

        case CUS_ORIT_M0F1:
            for(i=seg_length*2,j=0;i<seg_length*3;i++,j++){
                SensorReg_Write(mirr_flip_table[i].reg,mirr_flip_table[i].data);
                Current_Mirror_Flip_Tbl[j].reg = mirr_flip_table[i].reg;
                Current_Mirror_Flip_Tbl[j].data = mirr_flip_table[i].data;
            }
		 // handle->bayer_id=	CUS_BAYER_GR;
            break;

        case CUS_ORIT_M1F1:
            for(i=seg_length*3,j=0;i<seg_length*4;i++,j++){
                SensorReg_Write(mirr_flip_table[i].reg,mirr_flip_table[i].data);
                Current_Mirror_Flip_Tbl[j].reg = mirr_flip_table[i].reg;
                Current_Mirror_Flip_Tbl[j].data = mirr_flip_table[i].data;
            }
		 // handle->bayer_id=	CUS_BAYER_GR;
            break;

        default :
            for(i=0,j=0;i<seg_length;i++,j++){
                SensorReg_Write(mirr_flip_table[i].reg,mirr_flip_table[i].data);
                Current_Mirror_Flip_Tbl[j].reg = mirr_flip_table[i].reg;
                Current_Mirror_Flip_Tbl[j].data = mirr_flip_table[i].data;
            }
		//  handle->bayer_id=	CUS_BAYER_BG;
            break;
    }
    return SUCCESS;
}

static int pCus_GetFPS(ms_cus_sensor *handle)
{
    imx327_params *params = (imx327_params *)handle->private_data;
    //SENSOR_DMSG("[%s]", __FUNCTION__);

    return  params->expo.fps;
}

static int pCus_SetFPS(ms_cus_sensor *handle, u32 fps)
{
    imx327_params *params = (imx327_params *)handle->private_data;
    SENSOR_DMSG("\n\n[%s]", __FUNCTION__);

    if(fps>=3 && fps <= 30){
      fps = fps>29?29:fps;//limit fps at 29 fps due to MCLK=36MHz
      params->expo.fps = fps;
      params->expo.vts=  (vts_30fps*29)/fps;
      params->tVts_reg[0].data = (params->expo.vts>> 16) & 0x0003;
      params->tVts_reg[1].data = (params->expo.vts >> 8) & 0x00ff;
      params->tVts_reg[2].data = (params->expo.vts >> 0) & 0x00ff;
      params->dirty = true;
      return SUCCESS;
    }else if(fps>=3000 && fps <= 30000){
        fps = fps>29091?29091:fps;//limit fps at 29.091 fps due to MCLK=36MHz
        params->expo.fps = fps;
        params->expo.vts=  (vts_30fps*29091)/fps;
        params->tVts_reg[0].data = (params->expo.vts>> 16) & 0x0003;
        params->tVts_reg[1].data = (params->expo.vts >> 8) & 0x00ff;
        params->tVts_reg[2].data = (params->expo.vts >> 0) & 0x00ff;
        params->dirty = true;
        return SUCCESS;
    }else{
      //params->expo.vts=vts_30fps;
      //params->expo.fps=30;
      SENSOR_DMSG("[%s] FPS %d out of range.\n",__FUNCTION__,fps);
      return FAIL;
    }
}

static int pCus_SetFPS_HDR_DOL_SEF1(ms_cus_sensor *handle, u32 fps)
{
    //imx327_params *params = (imx327_params *)handle->private_data;
    //SENSOR_DMSG("\n\n[%s]", __FUNCTION__);

    if(fps>=3 && fps <= 30){
      return SUCCESS;
    }else if(fps>=3000 && fps <= 30000){
        return SUCCESS;
    }else{
      //params->expo.vts=vts_30fps;
      //params->expo.fps=30;
      SENSOR_DMSG("[%s] FPS %d out of range.\n",__FUNCTION__,fps);
      return FAIL;
    }
}

#if 0
static int pCus_GetSensorCap(ms_cus_sensor *handle, CUS_CAMSENSOR_CAP *cap)
{
    if (cap)
        memcpy(cap, &sensor_cap, sizeof(CUS_CAMSENSOR_CAP));
    else     return FAIL;
    return SUCCESS;
}
#endif

///////////////////////////////////////////////////////////////////////
// auto exposure
///////////////////////////////////////////////////////////////////////
// unit: micro seconds
//AE status notification
static int pCus_AEStatusNotify(ms_cus_sensor *handle, CUS_CAMSENSOR_AE_STATUS_NOTIFY status)
{
    imx327_params *params = (imx327_params *)handle->private_data;
    //ISensorIfAPI2 *sensor_if1 = handle->sensor_if_api2;
    switch(status)
    {
        case CUS_FRAME_INACTIVE:
             //SensorReg_Write(0x3001,0);
             break;
        case CUS_FRAME_ACTIVE:
            if(params->dirty)
            {
/*
                if(params->change){

                    // sensor_if1->SetSkipFrame(handle,3);
                     params->change = false;

                    }
*/
                //SensorReg_Write(0x3001,1);
             //   SensorRegArrayW((I2C_ARRAY*)params->tExpo_reg, ARRAY_SIZE(expo_reg));
             //   SensorRegArrayW((I2C_ARRAY*)gain_reg, ARRAY_SIZE(gain_reg));
             //   SensorRegArrayW((I2C_ARRAY*)params->tVts_reg, ARRAY_SIZE(vts_reg));
                //SensorReg_Write(0x3001,0);
               // printf("0x3009=0x%x,0x3014=0x%x,0x3016=0x%x,0x3020=0x%x,0x3021=0x%x\n", gain_reg[1].data,gain_reg[0].data,gain_reg[2].data,params->tExpo_reg[2].data,params->tExpo_reg[1].data);
                params->dirty = false;


            }
            break;
        default :
             break;
    }
    return SUCCESS;
}

static int pCus_AEStatusNotifyHDR_DOL_LEF(ms_cus_sensor *handle, CUS_CAMSENSOR_AE_STATUS_NOTIFY status)
{
    imx327_params *params = (imx327_params *)handle->private_data;
    //ISensorIfAPI2 *sensor_if1 = handle->sensor_if_api2;
    switch(status)
    {
        case CUS_FRAME_INACTIVE:
             //SensorReg_Write(0x3001,0);
             break;
        case CUS_FRAME_ACTIVE:
            if(params->dirty)
            {
                SensorRegArrayW((I2C_ARRAY*)params->tShs2_reg, ARRAY_SIZE(expo_SHS2_reg));
                SensorRegArrayW((I2C_ARRAY*)params->tVts_reg, ARRAY_SIZE(vts_reg));
                SensorRegArrayW((I2C_ARRAY*)params->tExpo_reg, ARRAY_SIZE(expo_reg));
                SensorRegArrayW((I2C_ARRAY*)params->tRhs1_reg, ARRAY_SIZE(expo_RHS1_reg));
                SensorRegArrayW((I2C_ARRAY*)params->tGain_hdr_dol_lef_reg, ARRAY_SIZE(gain_HDR_DOL_LEF_reg));
                SensorRegArrayW((I2C_ARRAY*)params->tGain_hdr_dol_sef_reg, ARRAY_SIZE(gain_HDR_DOL_SEF_reg));

/*
                if(params->change){

                    // sensor_if1->SetSkipFrame(handle,3);
                     params->change = false;

                    }
*/
                //SensorReg_Write(0x3001,1);
             //   SensorRegArrayW((I2C_ARRAY*)params->tExpo_reg, ARRAY_SIZE(expo_reg));
             //   SensorRegArrayW((I2C_ARRAY*)gain_reg, ARRAY_SIZE(gain_reg));
             //   SensorRegArrayW((I2C_ARRAY*)params->tVts_reg, ARRAY_SIZE(vts_reg));
                //SensorReg_Write(0x3001,0);
               // printf("0x3009=0x%x,0x3014=0x%x,0x3016=0x%x,0x3020=0x%x,0x3021=0x%x\n", gain_reg[1].data,gain_reg[0].data,gain_reg[2].data,params->tExpo_reg[2].data,params->tExpo_reg[1].data);
                params->dirty = false;
            }
            break;
        default :
             break;
    }
    return SUCCESS;
}

static int pCus_AEStatusNotifyHDR_DOL_SEF1(ms_cus_sensor *handle, CUS_CAMSENSOR_AE_STATUS_NOTIFY status)
{
   // imx327_params *params = (imx327_params *)handle->private_data;
    switch(status)
    {
        case CUS_FRAME_INACTIVE:
             //SensorReg_Write(0x3001,0);
             break;
        case CUS_FRAME_ACTIVE:
            break;
        default :
             break;
    }
    return SUCCESS;
}

static int pCus_SetAEUSecsHDR_DOL_LEF(ms_cus_sensor *handle, u32 us)
{
    u32 lines = 0, vts = 0, fsc = 0, shs2 = 0;
    imx327_params *params = (imx327_params *)handle->private_data;


    // us = 1000000/30;
    //lines = us/Preview_line_period_HDR_DOL;
    lines = (1000 * us) / Preview_line_period_HDR_DOL;
    if (lines > params->expo.vts - 2) {
        vts = lines + 2;
    }
    else{
      vts = params->expo.vts;
    }

   // lines=us/Preview_line_period_HDR_DOL;
    SENSOR_DMSG("[%s] us %ld, lines %ld, vts %ld\n", __FUNCTION__,
                us,
                lines,
                params->expo.vts
                );

    fsc = vts * 2;
    if(lines < 1)
        lines = 1;
    if(lines > fsc - 104)
        lines = fsc - 104;

    shs2 = fsc - lines - 1;
    params->tShs2_reg[0].data = (shs2 >> 16) & 0x0003;
    params->tShs2_reg[1].data = (shs2 >> 8) & 0x00ff;
    params->tShs2_reg[2].data = (shs2 >> 0) & 0x00ff;


    params->tVts_reg[0].data = (vts >> 16) & 0x0003;
    params->tVts_reg[1].data = (vts >> 8) & 0x00ff;
    params->tVts_reg[2].data = (vts >> 0) & 0x00ff;
    //SensorReg_Write(0x3001,1);
    //SensorRegArrayW((I2C_ARRAY*)params->tShs2_reg, ARRAY_SIZE(expo_SHS2_reg));
    //SensorRegArrayW((I2C_ARRAY*)gain_reg, ARRAY_SIZE(gain_reg));
    //SensorRegArrayW((I2C_ARRAY*)params->tVts_reg, ARRAY_SIZE(vts_reg));
    //UartSendTrace("[LEF, us:%d, lines:%d, ", us, lines);
    //UartSendTrace("vts:%d, shs2:%d]\n", vts, shs2);

    params->dirty = true;
    return SUCCESS;
}

static int pCus_SetAEUSecsHDR_DOL_SEF1(ms_cus_sensor *handle, u32 us)
{
    u32 lines = 0, vts = 0, fsc = 0;
    u32 /*n = 0, */rhs1 = 0, shs1 = 0, shs2 = 0;
    u32 max_rhs1 = 0;
    imx327_params *params = (imx327_params *)handle->private_data;

    // us = 1000000/30;
    //lines = us/Preview_line_period_HDR_DOL;
    lines = (1000 * us) / Preview_line_period_HDR_DOL;
    vts = (params->tVts_reg[0].data << 16) | (params->tVts_reg[1].data << 8) | (params->tVts_reg[2].data << 0);
    shs2 = (params->tShs2_reg[0].data << 16) | (params->tShs2_reg[1].data << 8) | (params->tShs2_reg[2].data << 0);
    fsc = vts * 2;
    //max_rhs1 = fsc - (1109/*BRL HD1080p*/ * 2) - 21;
    max_rhs1 = fsc - (1097/*BRL HD1080p*/ * 2) - 21; //exposure time 12 times
    max_rhs1 = 101;// 0x6d;
#if 1
    //Fix rhs1 as 101//0x6d, exposure line limitation = 1 ~ rhs - 3 -> 98*14842ns = 1/687 s
    rhs1 = max_rhs1;//(params->tRhs1_reg[0].data << 16) | (params->tRhs1_reg[1].data << 8) | (params->tRhs1_reg[2].data << 0);

    //line = rhs1 - (shs1 + 1) = rhs1 - 1 - (2 ~ rhs1 - 2) = 1~ rhs1 - 3
    //Check boundary
    if(lines < 1)
        lines = 1;
    if(lines > rhs1- 3)
        lines = rhs1 - 3;


    if((rhs1 - 1 - 2) < lines){
        shs1 = 2;
    }
    else if((rhs1 <= max_rhs1) && (rhs1 <= shs2 - 2)){
        shs1 = rhs1 - 1 - lines;
        if((shs1 < 2) || (shs1 > (rhs1 - 2))){ //Check boundary
            //shs1 = 0;
            //UartSendTrace("\n\n[SEF1 NG1]\n");
        }
    }
    else{
        //UartSendTrace("\n\n[SEF1 NG2]\n");
    }

#else
    n = 0;
    do{
        rhs1 = 2 * n + 5;
        if(((rhs1 - 1) > lines) && (rhs1 < max_rhs1) && (rhs1 <= shs2 - 2)){
            shs1 = rhs1 - 1 - lines;
            if((shs1 < 2) || (shs1 > (rhs1 - 2))){ //Check boundary
                shs1 = 0;
            }
        }

        if(rhs1 > max_rhs1){
            rhs1 = max_rhs1;
            shs1 = (rhs1 - 2 - 1);//shs1 = rhs1 - 1 - lines;
        }

        n++;
    }while(shs1 == 0);
#endif

    //UartSendTrace("[SEF1, us:%d, lines:%d, ", us, lines);
    //UartSendTrace("vts:%d, shs2:%d, ", vts, shs2);
    //UartSendTrace("rhs1:%d, shs1:%d]\n", rhs1, shs1);

    params->tExpo_reg[0].data = (shs1 >> 16) & 0x0003;
    params->tExpo_reg[1].data = (shs1 >> 8) & 0x00ff;
    params->tExpo_reg[2].data = (shs1 >> 0) & 0x00ff;

    params->tRhs1_reg[0].data = (rhs1 >> 16) & 0x0003;
    params->tRhs1_reg[1].data = (rhs1 >> 8) & 0x00ff;
    params->tRhs1_reg[2].data = (rhs1 >> 0) & 0x00ff;

    return SUCCESS;
}

static void pCus_SetAEGainHDR_DOL_Calculate(u32 gain, u16 *gain_reg)
{
    u32 gain_double;

    if(gain < SENSOR_MIN_GAIN){
      gain = SENSOR_MIN_GAIN;
    }
    else if(gain >= SENSOR_MAX_GAIN){
      gain = SENSOR_MAX_GAIN;
    }

    gain_double = 20*(intlog10(gain)-intlog10(1024));  //gain to db
    *gain_reg = (u16)(((gain_double*10) >>24)/3); //db to sensor reg
}

static int pCus_SetAEGainHDR_DOL_LEF(ms_cus_sensor *handle, u32 gain)
{
    imx327_params *params = (imx327_params *)handle->private_data;
    u16 gain_reg = 0;


    pCus_SetAEGainHDR_DOL_Calculate(gain, &gain_reg);
    params->tGain_hdr_dol_lef_reg[0].data = gain_reg;

    SENSOR_DMSG("[%s] set gain/reg=%ld/0x%x\n", __FUNCTION__, gain, params->tGain_hdr_dol_lef_reg[0].data);

    params->dirty = true;
    return SUCCESS;
}

static int pCus_SetAEGainHDR_DOL_SEF1(ms_cus_sensor *handle, u32 gain)
{
    imx327_params *params = (imx327_params *)handle->private_data;
    u16 gain_reg = 0;


    pCus_SetAEGainHDR_DOL_Calculate(gain, &gain_reg);
    params->tGain_hdr_dol_sef_reg[0].data = gain_reg;

    SENSOR_DMSG("[%s] set gain/reg=%ld/0x%x\n", __FUNCTION__, gain, params->tGain_hdr_dol_sef_reg[0].data);

    params->dirty = true;
    return SUCCESS;
}

static int pCus_GetAEUSecs(ms_cus_sensor *handle, u32 *us)
{
    //int rc;
    u32 lines = 0;
    imx327_params *params = (imx327_params *)handle->private_data;
    //int rc = SensorRegArrayR((I2C_ARRAY*)params->tExpo_reg, ARRAY_SIZE(expo_reg));

    lines |= (u32)(params->tExpo_reg[0].data&0xff)<<16;
    lines |= (u32)(params->tExpo_reg[1].data&0xff)<<8;
    lines |= (u32)(params->tExpo_reg[2].data&0xff)<<0;
   // lines >>= 4;
   // *us = (lines+dummy) * params->expo.us_per_line;
    //*us = lines;//(lines*Preview_line_period);
    *us = (lines*Preview_line_period)/1000;
    SENSOR_DMSG("[%s] sensor expo lines/us %ld,%ld us\n", __FUNCTION__, lines, *us);
    //return rc;
    return SUCCESS;
}

static int pCus_SetAEUSecs(ms_cus_sensor *handle, u32 us)
{
    u32 lines = 0, vts = 0;
    imx327_params *params = (imx327_params *)handle->private_data;

    //return SUCCESS; //TBD

    // us = 1000000/30;
    //lines = us/Preview_line_period;
    lines = (1000*us)/Preview_line_period;
    if (lines >params->expo.vts-2) {
        vts = lines +2;
    }
    else
      vts=params->expo.vts;

   // lines=us/Preview_line_period;
    SENSOR_DMSG("[%s] us %ld, lines %ld, vts %ld\n", __FUNCTION__,
                us,
                lines,
                params->expo.vts
                );
    lines=vts-lines-1;
    params->tExpo_reg[0].data = (lines>>16) & 0x0003;
    params->tExpo_reg[1].data = (lines>>8) & 0x00ff;
    params->tExpo_reg[2].data = (lines>>0) & 0x00ff;


    params->tVts_reg[0].data = (vts >> 16) & 0x0003;
    params->tVts_reg[1].data = (vts >> 8) & 0x00ff;
    params->tVts_reg[2].data = (vts >> 0) & 0x00ff;
    //SensorReg_Write(0x3001,1);
    SensorRegArrayW((I2C_ARRAY*)params->tExpo_reg, ARRAY_SIZE(expo_reg));
    //SensorRegArrayW((I2C_ARRAY*)gain_reg, ARRAY_SIZE(gain_reg));
    SensorRegArrayW((I2C_ARRAY*)params->tVts_reg, ARRAY_SIZE(vts_reg));
    params->dirty = true;
    return SUCCESS;
}

// Gain: 1x = 1024
static int pCus_GetAEGain(ms_cus_sensor *handle, u32* gain)
{
    imx327_params *params = (imx327_params *)handle->private_data;
    //int rc = SensorRegArrayR((I2C_ARRAY*)params->tGain_reg, ARRAY_SIZE(gain_reg));
    unsigned short temp_gain;
  //  *gain=params->expo.final_gain;
    temp_gain=params->tGain_reg[0].data;

    *gain=(u32)(10^((temp_gain*3)/200))*1024;
    if (params->tGain_reg[1].data & 0x10)
       *gain = (*gain) * 2;

    SENSOR_DMSG("[%s] get gain/reg (1024=1X)= %ld/0x%x\n", __FUNCTION__, *gain,params->tGain_reg[0].data);
    //return rc;
    return SUCCESS;
}

static int pCus_SetAEGain_cal(ms_cus_sensor *handle, u32 gain)
{
    imx327_params *params = (imx327_params *)handle->private_data;
    u32 gain_double;
    params->expo.final_gain = gain;

    if(gain<1024)
       gain=1024;
    else if(gain>=3980*1024)
        gain=3980*1024;

    gain_double = 20*(intlog10(gain)-intlog10(1024));
    params->tGain_reg[0].data=(u16)(((gain_double*10) >> 24)/3);

    SENSOR_DMSG("[%s] set gain/reg=%ld/0x%x\n", __FUNCTION__, gain,params->tGain_reg[0].data);

    //return CusHW_i2c_array_tx(handle, handle->i2c_cfg, params->tGain_reg, sizeof(gain_reg)/sizeof(CUS_I2C_ARRAY));
    //return SensorRegArrayW((I2C_ARRAY*)params->tGain_reg, sizeof(gain_reg)/sizeof(I2C_ARRAY));
    params->dirty = true;
    return SUCCESS;
}

static int pCus_SetAEGain(ms_cus_sensor *handle, u32 gain)
{
    //extern DBG_ITEM Dbg_Items[DBG_TAG_MAX];
    imx327_params *params = (imx327_params *)handle->private_data;
    u32 i;//, gain_before=0;
    CUS_GAIN_GAP_ARRAY* Sensor_Gain_Linearity;
    u32 gain_double;

    return SUCCESS; //TBD

   // u32 times = log2((double)gain/1024.0f)/log(2);
    params->expo.final_gain = gain;
    if(gain<1024)
        gain=1024;
    else if(gain>=2810*1024)
        gain=2810*1024;
    Sensor_Gain_Linearity = gain_gap_compensate;

    for(i = 0; i < sizeof(gain_gap_compensate)/sizeof(CUS_GAIN_GAP_ARRAY); i++){
        //LOGD("GAP:%x %x\r\n",Sensor_Gain_Linearity[i].gain, Sensor_Gain_Linearity[i].offset);

        if (Sensor_Gain_Linearity[i].gain == 0)
            break;
        if((gain>Sensor_Gain_Linearity[i].gain) && (gain < (Sensor_Gain_Linearity[i].gain + Sensor_Gain_Linearity[i].offset))){
              gain=Sensor_Gain_Linearity[i].gain;
              break;
        }
    }

    if(gain>=22925)//if gain exceed 2x , enable high conversion gain, >27DB, enable HCG
    {

           if(params->tGain_reg[1].data==0x02){
           // params->change = true;
            // params->tGain_reg[2].data=0x08;
            }
           else{
           // params->change = false;
          //  params->tGain_reg[2].data=0x09;
            }
            //gain_before=gain;
            params->tGain_reg[1].data |= 0x10;
           // params->tGain_reg[2].data=0x08;
            gain /= 2;

    }
    else{

           if(params->tGain_reg[1].data==0x12){
           // params->change = true;
           //  params->tGain_reg[2].data=0x08;
            }
           else{
           // params->change = false;
           // params->tGain_reg[2].data=0x09;
            }
          //  printf("[%s] params->change=%d\n", __FUNCTION__, params->change);

           // gain_before=gain;
            params->tGain_reg[1].data &= ~0x10;
            //params->tGain_reg[2].data=0x09;

        }

    gain_double = 20*(intlog10(gain)-intlog10(1024));
    params->tGain_reg[0].data=(u16)(((gain_double*10)>> 24)/3);

    SensorRegArrayW((I2C_ARRAY*)params->tGain_reg, ARRAY_SIZE(gain_reg));

//LOGD("s:%x %x\r\n", params->tGain_reg[0].data, params->tGain_reg[1].data);
    SENSOR_DMSG("[%s] set gain/reg=%ld/0x%x\n", __FUNCTION__, gain,params->tGain_reg[0].data);
    params->dirty = true;
    return SUCCESS;
    //return CusHW_i2c_array_tx(handle, handle->i2c_cfg, params->tGain_reg, sizeof(gain_reg)/sizeof(CUS_I2C_ARRAY));
   // return SensorRegArrayW((I2C_ARRAY*)params->tGain_reg, sizeof(gain_reg)/sizeof(I2C_ARRAY));
}

static int pCus_GetAEMinMaxUSecs(ms_cus_sensor *handle, u32 *min, u32 *max)
{
    *min = 150;
    *max = 33332; // <- max shutter (30fps: 33332, 60fps:16666)
    return SUCCESS;
}

static int pCus_GetAEMinMaxGain(ms_cus_sensor *handle, u32 *min, u32 *max)
{
    *min = SENSOR_MIN_GAIN;//handle->sat_mingain;
    *max = SENSOR_MAX_GAIN;//3980*1024;
    return SUCCESS;
}
#if 0
static int pCus_GetDGainRemainder(ms_cus_sensor *handle, u32 *dgain_remainder)
{
    *dgain_remainder = handle->dgain_remainder;

    return SUCCESS;
}
#endif
static int IMX327_GetShutterInfo(struct __ms_cus_sensor* handle,CUS_SHUTTER_INFO *info)
{
    info->max = 1000000000/Preview_MIN_FPS;
    info->min = (Preview_line_period * 5);
    info->step = Preview_line_period;
    return SUCCESS;
}

static int pCus_setCaliData_gain_linearity(ms_cus_sensor* handle, CUS_GAIN_GAP_ARRAY* pArray, u32 num) {
    u32 i, j;

    for(i=0,j=0;i< num ;i++,j+=2){
        gain_gap_compensate[i].gain=pArray[i].gain;
        gain_gap_compensate[i].offset=pArray[i].offset;
    }

    //LOGD("[%s]%d, %d, %d, %d\n", __FUNCTION__, num, pArray[0].gain, pArray[1].gain, pArray[num-1].offset);

    return SUCCESS;
}

int cus_camsensor_init_handle_linear(ms_cus_sensor* drv_handle)
{
    ms_cus_sensor *handle = drv_handle;
    imx327_params *params;
    if (!handle) {
        SENSOR_DMSG("[%s] not enough memory!\n", __FUNCTION__);
        return FAIL;
    }
    SENSOR_DMSG("[%s]", __FUNCTION__);
    //private data allocation & init
    if (handle->private_data == NULL) {
        SENSOR_EMSG("[%s] Private data is empty!\n", __FUNCTION__);
        return FAIL;
    }
    params = (imx327_params *)handle->private_data;
    memcpy(params->tGain_reg, gain_reg, sizeof(gain_reg));
    memcpy(params->tExpo_reg, expo_reg, sizeof(expo_reg));
    memcpy(params->tVts_reg, vts_reg, sizeof(vts_reg));
    memcpy(params->tRhs1_reg, expo_RHS1_reg, sizeof(expo_RHS1_reg));
    memcpy(params->tShs2_reg, expo_SHS2_reg, sizeof(expo_SHS2_reg));
    memcpy(params->tGain_hdr_dol_lef_reg, gain_HDR_DOL_LEF_reg, sizeof(gain_HDR_DOL_LEF_reg));
    memcpy(params->tGain_hdr_dol_sef_reg, gain_HDR_DOL_SEF_reg, sizeof(gain_HDR_DOL_SEF_reg));


    ////////////////////////////////////
    //    sensor model ID                           //
    ////////////////////////////////////
    sprintf(handle->model_id,"IMX327_MIPI");

    ////////////////////////////////////
    //    sensor interface info       //
    ////////////////////////////////////
    //SENSOR_DMSG("[%s] entering function with id %d\n", __FUNCTION__, id);
    handle->isp_type    = SENSOR_ISP_TYPE;  //ISP_SOC;
    //handle->data_fmt    = SENSOR_DATAFMT;   //CUS_DATAFMT_YUV;
    handle->sif_bus     = SENSOR_IFBUS_TYPE;//CUS_SENIF_BUS_PARL;
    handle->data_prec   = SENSOR_DATAPREC;  //CUS_DATAPRECISION_8;
    handle->data_mode   = SENSOR_DATAMODE;
    handle->bayer_id    = SENSOR_BAYERID;   //CUS_BAYER_GB;
    handle->RGBIR_id    = SENSOR_RGBIRID;
    handle->orient      = SENSOR_ORIT;      //CUS_ORIT_M1F1;
    //handle->YC_ODER     = SENSOR_YCORDER;   //CUS_SEN_YCODR_CY;
    handle->interface_attr.attr_mipi.mipi_lane_num = SENSOR_MIPI_LANE_NUM;
    handle->interface_attr.attr_mipi.mipi_data_format = CUS_SEN_INPUT_FORMAT_RGB; // RGB pattern.
    handle->interface_attr.attr_mipi.mipi_yuv_order = 0; //don't care in RGB pattern.
    handle->interface_attr.attr_mipi.mipi_hsync_mode = SENSOR_MIPI_HSYNC_MODE;
    handle->interface_attr.attr_mipi.mipi_hdr_mode = CUS_HDR_MODE_NONE;
    handle->interface_attr.attr_mipi.mipi_hdr_virtual_channel_num = 1; //Short frame

    ////////////////////////////////////
    //    resolution capability       //
    ////////////////////////////////////

    handle->video_res_supported.num_res = 1;
    handle->video_res_supported.ulcur_res = 0; //default resolution index is 0.
    handle->video_res_supported.res[0].width = Preview_WIDTH;
    handle->video_res_supported.res[0].height = Preview_HEIGHT;
    handle->video_res_supported.res[0].max_fps= Preview_MAX_FPS;
    handle->video_res_supported.res[0].min_fps= Preview_MIN_FPS;
    handle->video_res_supported.res[0].crop_start_x= Preview_CROP_START_X;
    handle->video_res_supported.res[0].crop_start_y= Preview_CROP_START_Y;
    handle->video_res_supported.res[0].nOutputWidth = 0x79C;//Preview_WIDTH;
    handle->video_res_supported.res[0].nOutputHeight = 0x449;//DOL HDR //Preview_HEIGHT;

    // i2c
    handle->i2c_cfg.mode                = SENSOR_I2C_LEGACY;    //(CUS_ISP_I2C_MODE) FALSE;
    handle->i2c_cfg.fmt                 = SENSOR_I2C_FMT;       //CUS_I2C_FMT_A16D16;
    handle->i2c_cfg.address             = SENSOR_I2C_ADDR;      //0x5a;
    handle->i2c_cfg.speed               = SENSOR_I2C_SPEED;     //320000;

    // mclk
    handle->mclk                        = Preview_MCLK_SPEED;

    //polarity
    /////////////////////////////////////////////////////
    handle->pwdn_POLARITY               = SENSOR_PWDN_POL;  //CUS_CLK_POL_NEG;
    handle->reset_POLARITY              = SENSOR_RST_POL;   //CUS_CLK_POL_NEG;
    //handle->VSYNC_POLARITY              = SENSOR_VSYNC_POL; //CUS_CLK_POL_POS;
    //handle->HSYNC_POLARITY              = SENSOR_HSYNC_POL; //CUS_CLK_POL_POS;
    //handle->PCLK_POLARITY               = SENSOR_PCLK_POL;  //CUS_CLK_POL_POS);    // use '!' to clear board latch error
    /////////////////////////////////////////////////////



    ////////////////////////////////////////////////////
    // AE parameters
    ////////////////////////////////////////////////////
    handle->ae_gain_delay       = SENSOR_GAIN_DELAY_FRAME_COUNT;//1;//0;//1;
    handle->ae_shutter_delay    = SENSOR_SHUTTER_DELAY_FRAME_COUNT;//2;//1;//2;

    handle->ae_gain_ctrl_num = 1;
    handle->ae_shutter_ctrl_num = 1;

    ///calibration
    handle->sat_mingain = SENSOR_MIN_GAIN;//g_sensor_ae_min_gain;
    //handle->dgain_remainder = 0;

    //LOGD("[%s:%d]\n", __FUNCTION__, __LINE__);
    handle->pCus_sensor_release     = cus_camsensor_release_handle;
#if (SENSOR_MIPI_LANE_NUM == 2)
    handle->pCus_sensor_init        = pCus_init_mipi2lane_linear;
#endif
#if (SENSOR_MIPI_LANE_NUM == 4)
    handle->pCus_sensor_init        = pCus_init_mipi4lane_linear;
#endif

    //handle->pCus_sensor_powerupseq  = pCus_powerupseq   ;
    //handle->pCus_sensor_poweron     = pCus_poweron ;
    //handle->pCus_sensor_poweroff    = pCus_poweroff;

    // Normal
    //handle->pCus_sensor_GetSensorID       = pCus_GetSensorID   ;
    //handle->pCus_sensor_GetStillResCap    = pCus_GetStillResCap;
    //handle->pCus_sensor_GetStillRes       = pCus_GetStillRes   ;
    //handle->pCus_sensor_SetStillRes       = pCus_SetStillRes   ;
    //handle->pCus_sensor_GetVideoResCap    = pCus_GetVideoResCap;
    handle->pCus_sensor_GetVideoResNum = pCus_GetVideoResNum;
    handle->pCus_sensor_GetVideoRes       = pCus_GetVideoRes   ;
    handle->pCus_sensor_SetVideoRes       = pCus_SetVideoRes   ;
    handle->pCus_sensor_GetOrien          = pCus_GetOrien      ;
    handle->pCus_sensor_SetOrien          = pCus_SetOrien      ;
    handle->pCus_sensor_GetFPS          = pCus_GetFPS      ;
    handle->pCus_sensor_SetFPS          = pCus_SetFPS      ;
    //handle->pCus_sensor_GetSensorCap    = pCus_GetSensorCap;
    handle->pCus_sensor_SetPatternMode = imx327_SetPatternMode;
    ///////////////////////////////////////////////////////
    // AE
    ///////////////////////////////////////////////////////
    // unit: micro seconds
    //handle->pCus_sensor_GetAETrigger_mode      = pCus_GetAETrigger_mode;
    //handle->pCus_sensor_SetAETrigger_mode      = pCus_SetAETrigger_mode;
    handle->pCus_sensor_AEStatusNotify = pCus_AEStatusNotify;
    handle->pCus_sensor_GetAEUSecs      = pCus_GetAEUSecs;
    handle->pCus_sensor_SetAEUSecs      = pCus_SetAEUSecs;
    handle->pCus_sensor_GetAEGain       = pCus_GetAEGain;
    //handle->pCus_sensor_SetAEGain       = pCus_SetAEGain;
    handle->pCus_sensor_SetAEGain       = pCus_SetAEGain;

    handle->pCus_sensor_GetAEMinMaxGain = pCus_GetAEMinMaxGain;
    handle->pCus_sensor_GetAEMinMaxUSecs= pCus_GetAEMinMaxUSecs;

     //sensor calibration
    handle->pCus_sensor_SetAEGain_cal   = pCus_SetAEGain_cal;
    handle->pCus_sensor_setCaliData_gain_linearity=pCus_setCaliData_gain_linearity;
    handle->pCus_sensor_GetShutterInfo = IMX327_GetShutterInfo;
#if 0//defined(__MV5_FPGA__)
	handle->pCus_sensor_I2CWrite = pCus_I2CWrite; //Andy Liu
	handle->pCus_sensor_I2CRead = pCus_I2CRead; //Andy Liu
#endif
    params->expo.vts = vts_30fps;
    params->expo.fps = 29;
    params->dirty = false;


	handle->channel_num = SENSOR_CHANNEL_NUM;
	handle->channel_mode = SENSOR_CHANNEL_MODE_LINEAR;

    return SUCCESS;
}

static int IMX327_GetShutterInfo_hdr_dol_lef(struct __ms_cus_sensor* handle,CUS_SHUTTER_INFO *info)
{
    info->max = 1000000000/Preview_MIN_FPS;
    info->min = (Preview_line_period_HDR_DOL * 5);
    info->step = Preview_line_period_HDR_DOL;
    return SUCCESS;
}

int cus_camsensor_init_handle_hdr_dol_sef1(ms_cus_sensor* drv_handle)
{
    ms_cus_sensor *handle = drv_handle;
    imx327_params *params = NULL;

    cus_camsensor_init_handle_linear(drv_handle);
    params = (imx327_params *)handle->private_data;

    handle->bayer_id    = SENSOR_BAYERID_HDR_DOL;
    handle->RGBIR_id    = SENSOR_RGBIRID;

    handle->interface_attr.attr_mipi.mipi_hsync_mode = SENSOR_MIPI_HSYNC_MODE_HDR_DOL;
    handle->interface_attr.attr_mipi.mipi_hdr_mode = CUS_HDR_MODE_SONY_DOL;
    handle->video_res_supported.res[0].width = Preview_WIDTH;
    handle->video_res_supported.res[0].height = Preview_HEIGHT; //TBD. Workaround for Sony DOL HDR mode.
    handle->video_res_supported.res[0].nOutputWidth = 1948;//0x7A0;//Preview_WIDTH;
    handle->video_res_supported.res[0].nOutputHeight = (0x97A >> 1) - 1;//(0x89C >> 1);//0x449;//DOL HDR //Preview_HEIGHT;

#if (SENSOR_MIPI_LANE_NUM == 2)
#endif
#if (SENSOR_MIPI_LANE_NUM == 4)
    handle->pCus_sensor_init        = pCus_init_mipi4lane_HDR_DOL;
#endif

    handle->pCus_sensor_SetFPS          = pCus_SetFPS_HDR_DOL_SEF1; //TBD

    handle->pCus_sensor_AEStatusNotify = pCus_AEStatusNotifyHDR_DOL_SEF1;
    handle->pCus_sensor_GetAEUSecs      = pCus_GetAEUSecs;
    handle->pCus_sensor_SetAEUSecs      = pCus_SetAEUSecsHDR_DOL_SEF1;
    handle->pCus_sensor_GetAEGain       = pCus_GetAEGain;
    //handle->pCus_sensor_SetAEGain       = pCus_SetAEGain;
    handle->pCus_sensor_SetAEGain       = pCus_SetAEGainHDR_DOL_SEF1;
    handle->pCus_sensor_GetShutterInfo = IMX327_GetShutterInfo_hdr_dol_lef;
    params->expo.vts = vts_30fps_HDR_DOL;

    handle->channel_mode = SENSOR_CHANNEL_MODE_SONY_DOL;

    handle->ae_gain_ctrl_num = 2;
    handle->ae_shutter_ctrl_num = 2;

    return SUCCESS;
}

int cus_camsensor_release_handle(ms_cus_sensor *handle)
{
    return SUCCESS;
}

static int pCus_poweron_hdr_dol_lef(ms_cus_sensor *handle, u32 idx)
{
    return SUCCESS;
}

static int pCus_poweroff_hdr_dol_lef(ms_cus_sensor *handle, u32 idx)
{
    return SUCCESS;
}

/////////////////// image function /////////////////////////
//Get and check sensor ID
//if i2c error or sensor id does not match then return FAIL
static int pCus_GetSensorID_hdr_dol_lef(ms_cus_sensor *handle, u32 *id)
{
    *id = 0;
     return SUCCESS;
}

static int imx327_SetPatternMode_hdr_dol_lef(ms_cus_sensor *handle,u32 mode)
{
    return SUCCESS;
}
#if 0
static int pCus_I2CWrite_hdr_dol_lef(ms_cus_sensor *handle, unsigned short usreg, unsigned short usval)
{
    return SUCCESS;
}

static int pCus_I2CRead_hdr_dol_lef(ms_cus_sensor *handle, unsigned short usreg)
{
    return SUCCESS;
}
#endif
static int pCus_init_hdr_dol_lef(ms_cus_sensor *handle)
{
    return SUCCESS;
}

static int pCus_GetVideoRes_hdr_dol_lef( ms_cus_sensor *handle, u32 res_idx, cus_camsensor_res **res )
{
    *res = &handle->video_res_supported.res[res_idx];
    return SUCCESS;
}

static int pCus_SetVideoRes_hdr_dol_lef( ms_cus_sensor *handle, u32 res )
{
    handle->video_res_supported.ulcur_res = 0; //TBD
    return SUCCESS;
}

static int pCus_GetOrien_hdr_dol_lef(ms_cus_sensor *handle, CUS_CAMSENSOR_ORIT *orit)
{
    *orit = CUS_ORIT_M0F0;
    return SUCCESS;
}

static int pCus_SetOrien_hdr_dol_lef(ms_cus_sensor *handle, CUS_CAMSENSOR_ORIT orit)
{
    return SUCCESS;
}

static int pCus_GetFPS_hdr_dol_lef(ms_cus_sensor *handle)
{
    imx327_params *params = (imx327_params *)handle->private_data;
    //SENSOR_DMSG("[%s]", __FUNCTION__);

    return  params->expo.fps;
}
static int pCus_SetFPS_hdr_dol_lef(ms_cus_sensor *handle, u32 fps)
{
    return SUCCESS;
}
#if 0
static int pCus_GetSensorCap_hdr_dol_lef(ms_cus_sensor *handle, CUS_CAMSENSOR_CAP *cap)
{
    if (cap)
        memcpy(cap, &sensor_cap, sizeof(CUS_CAMSENSOR_CAP));
    else     return FAIL;
    return SUCCESS;
}
#endif

///////////////////////////////////////////////////////////////////////
// auto exposure
///////////////////////////////////////////////////////////////////////
// unit: micro seconds
//AE status notification
#if 0
static int pCus_AEStatusNotify_hdr_dol_lef(ms_cus_sensor *handle, CUS_CAMSENSOR_AE_STATUS_NOTIFY status)
{
    return SUCCESS;
}
#endif
static int pCus_GetAEUSecs_hdr_dol_lef(ms_cus_sensor *handle, u32 *us)
{
    *us = 0;
    return SUCCESS;
}
#if 0
static int pCus_SetAEUSecs_hdr_dol_lef(ms_cus_sensor *handle, u32 us)
{
    return SUCCESS;
}
#endif
// Gain: 1x = 1024
static int pCus_GetAEGain_hdr_dol_lef(ms_cus_sensor *handle, u32* gain)
{
    *gain = 0;
    return SUCCESS;
}

static int pCus_SetAEGain_cal_hdr_dol_lef(ms_cus_sensor *handle, u32 gain)
{
    return SUCCESS;
}
#if 0
static int pCus_SetAEGain_hdr_dol_lef(ms_cus_sensor *handle, u32 gain)
{
    return SUCCESS;
}
#endif
static int pCus_GetAEMinMaxUSecs_hdr_dol_lef(ms_cus_sensor *handle, u32 *min, u32 *max)
{
    *min = 1;
    *max = 1000000/Preview_MIN_FPS;
    return SUCCESS;
}

static int pCus_GetAEMinMaxGain_hdr_dol_lef(ms_cus_sensor *handle, u32 *min, u32 *max)
{
    *min = handle->sat_mingain;
    *max = 3980*1024;
    return SUCCESS;
}

static int pCus_setCaliData_gain_linearity_hdr_dol_lef(ms_cus_sensor* handle, CUS_GAIN_GAP_ARRAY* pArray, u32 num)
{
    return SUCCESS;
}

int cus_camsensor_init_handle_hdr_dol_lef(ms_cus_sensor* drv_handle)
{
    ms_cus_sensor *handle = drv_handle;
    imx327_params *params;
    if (!handle) {
        SENSOR_DMSG("[%s] not enough memory!\n", __FUNCTION__);
        return FAIL;
    }
    SENSOR_DMSG("[%s]", __FUNCTION__);
    //private data allocation & init
    if (handle->private_data == NULL) {
        SENSOR_EMSG("[%s] Private data is empty!\n", __FUNCTION__);
        return FAIL;
    }
    params = (imx327_params *)handle->private_data;
    memcpy(params->tGain_reg, gain_reg, sizeof(gain_reg));
    memcpy(params->tExpo_reg, expo_reg, sizeof(expo_reg));
    memcpy(params->tVts_reg, vts_reg, sizeof(vts_reg));
    memcpy(params->tRhs1_reg, expo_RHS1_reg, sizeof(expo_RHS1_reg));
    memcpy(params->tShs2_reg, expo_SHS2_reg, sizeof(expo_SHS2_reg));
    memcpy(params->tGain_hdr_dol_lef_reg, gain_HDR_DOL_LEF_reg, sizeof(gain_HDR_DOL_LEF_reg));
    memcpy(params->tGain_hdr_dol_sef_reg, gain_HDR_DOL_SEF_reg, sizeof(gain_HDR_DOL_SEF_reg));


    ////////////////////////////////////
    //    sensor model ID                           //
    ////////////////////////////////////
    sprintf(handle->model_id,"IMX327_MIPI_HDR");

    ////////////////////////////////////
    //    sensor interface info       //
    ////////////////////////////////////
    //SENSOR_DMSG("[%s] entering function with id %d\n", __FUNCTION__, id);
    handle->isp_type    = SENSOR_ISP_TYPE;  //ISP_SOC;
    //handle->data_fmt    = SENSOR_DATAFMT;   //CUS_DATAFMT_YUV;
    handle->sif_bus     = SENSOR_IFBUS_TYPE;//CUS_SENIF_BUS_PARL;
    handle->data_prec   = SENSOR_DATAPREC;  //CUS_DATAPRECISION_8;
    handle->data_mode   = SENSOR_DATAMODE;
    handle->bayer_id    = SENSOR_BAYERID_HDR_DOL;   //CUS_BAYER_GB;
    handle->RGBIR_id    = SENSOR_RGBIRID;
    handle->orient      = SENSOR_ORIT;      //CUS_ORIT_M1F1;
    //handle->YC_ODER     = SENSOR_YCORDER;   //CUS_SEN_YCODR_CY;
    handle->interface_attr.attr_mipi.mipi_lane_num = SENSOR_MIPI_LANE_NUM;
    handle->interface_attr.attr_mipi.mipi_data_format = CUS_SEN_INPUT_FORMAT_RGB; // RGB pattern.
    handle->interface_attr.attr_mipi.mipi_yuv_order = 0; //don't care in RGB pattern.
    handle->interface_attr.attr_mipi.mipi_hsync_mode = SENSOR_MIPI_HSYNC_MODE_HDR_DOL;
    handle->interface_attr.attr_mipi.mipi_hdr_mode = CUS_HDR_MODE_SONY_DOL;
    handle->interface_attr.attr_mipi.mipi_hdr_virtual_channel_num =  0; //Long frame

    ////////////////////////////////////
    //    resolution capability       //
    ////////////////////////////////////

    handle->video_res_supported.num_res = 1;
    handle->video_res_supported.ulcur_res = 0; //default resolution index is 0.
    handle->video_res_supported.res[0].width = Preview_WIDTH;
    handle->video_res_supported.res[0].height = Preview_HEIGHT; //TBD. Workaround for Sony DOL HDR mode
    handle->video_res_supported.res[0].max_fps= Preview_MAX_FPS;
    handle->video_res_supported.res[0].min_fps= Preview_MIN_FPS;
    handle->video_res_supported.res[0].crop_start_x= Preview_CROP_START_X;
    handle->video_res_supported.res[0].crop_start_y= Preview_CROP_START_Y;
    handle->video_res_supported.res[0].nOutputWidth = 1948;//0x7A0;//Preview_WIDTH;
    handle->video_res_supported.res[0].nOutputHeight = (0x97A >> 1) - 1;//(0x89C >> 1);//0x449;//DOL HDR //Preview_HEIGHT;

    // i2c
    handle->i2c_cfg.mode                = SENSOR_I2C_LEGACY;    //(CUS_ISP_I2C_MODE) FALSE;
    handle->i2c_cfg.fmt                 = SENSOR_I2C_FMT;       //CUS_I2C_FMT_A16D16;
    handle->i2c_cfg.address             = SENSOR_I2C_ADDR;      //0x5a;
    handle->i2c_cfg.speed               = SENSOR_I2C_SPEED;     //320000;

    // mclk
    handle->mclk                        = Preview_MCLK_SPEED;

    //polarity
    /////////////////////////////////////////////////////
    handle->pwdn_POLARITY               = SENSOR_PWDN_POL;  //CUS_CLK_POL_NEG;
    handle->reset_POLARITY              = SENSOR_RST_POL;   //CUS_CLK_POL_NEG;
    //handle->VSYNC_POLARITY              = SENSOR_VSYNC_POL; //CUS_CLK_POL_POS;
    //handle->HSYNC_POLARITY              = SENSOR_HSYNC_POL; //CUS_CLK_POL_POS;
    //handle->PCLK_POLARITY               = SENSOR_PCLK_POL;  //CUS_CLK_POL_POS);    // use '!' to clear board latch error
    /////////////////////////////////////////////////////



    ////////////////////////////////////////////////////
    // AE parameters
    ////////////////////////////////////////////////////
    handle->ae_gain_delay       = SENSOR_GAIN_DELAY_FRAME_COUNT;//1;//0;//1;
    handle->ae_shutter_delay    = SENSOR_SHUTTER_DELAY_FRAME_COUNT;//2;//1;//2;

    handle->ae_gain_ctrl_num = 2;
    handle->ae_shutter_ctrl_num = 2;

    ///calibration
    handle->sat_mingain = SENSOR_MIN_GAIN;//g_sensor_ae_min_gain;
    //handle->dgain_remainder = 0;

    //LOGD("[%s:%d]\n", __FUNCTION__, __LINE__);
    handle->pCus_sensor_release     = cus_camsensor_release_handle;
    handle->pCus_sensor_init        = pCus_init_hdr_dol_lef;
    //handle->pCus_sensor_powerupseq  = pCus_powerupseq   ;
    handle->pCus_sensor_poweron     = pCus_poweron_hdr_dol_lef;
    handle->pCus_sensor_poweroff    = pCus_poweroff_hdr_dol_lef;

    // Normal
    handle->pCus_sensor_GetSensorID       = pCus_GetSensorID_hdr_dol_lef;
    //handle->pCus_sensor_GetStillResCap    = pCus_GetStillResCap;
    //handle->pCus_sensor_GetStillRes       = pCus_GetStillRes   ;
    //handle->pCus_sensor_SetStillRes       = pCus_SetStillRes   ;
    //handle->pCus_sensor_GetVideoResCap    = pCus_GetVideoResCap;
    handle->pCus_sensor_GetVideoResNum = pCus_GetVideoResNum;
    handle->pCus_sensor_GetVideoRes       = pCus_GetVideoRes_hdr_dol_lef;
    handle->pCus_sensor_SetVideoRes       = pCus_SetVideoRes_hdr_dol_lef;
    handle->pCus_sensor_GetOrien          = pCus_GetOrien_hdr_dol_lef;
    handle->pCus_sensor_SetOrien          = pCus_SetOrien_hdr_dol_lef;
    handle->pCus_sensor_GetFPS          = pCus_GetFPS_hdr_dol_lef;
    handle->pCus_sensor_SetFPS          = pCus_SetFPS_hdr_dol_lef;
    //handle->pCus_sensor_GetSensorCap    = pCus_GetSensorCap_hdr_dol_lef;
    handle->pCus_sensor_SetPatternMode = imx327_SetPatternMode_hdr_dol_lef;
    ///////////////////////////////////////////////////////
    // AE
    ///////////////////////////////////////////////////////
    // unit: micro seconds
    //handle->pCus_sensor_GetAETrigger_mode      = pCus_GetAETrigger_mode;
    //handle->pCus_sensor_SetAETrigger_mode      = pCus_SetAETrigger_mode;
    handle->pCus_sensor_AEStatusNotify = pCus_AEStatusNotifyHDR_DOL_LEF;
    handle->pCus_sensor_GetAEUSecs      = pCus_GetAEUSecs_hdr_dol_lef;
    handle->pCus_sensor_SetAEUSecs      = pCus_SetAEUSecsHDR_DOL_LEF;
    handle->pCus_sensor_GetAEGain       = pCus_GetAEGain_hdr_dol_lef;
    //handle->pCus_sensor_SetAEGain       = pCus_SetAEGain_hdr_dol_lef;
    handle->pCus_sensor_SetAEGain       = pCus_SetAEGainHDR_DOL_LEF;

    handle->pCus_sensor_GetAEMinMaxGain = pCus_GetAEMinMaxGain_hdr_dol_lef;
    handle->pCus_sensor_GetAEMinMaxUSecs= pCus_GetAEMinMaxUSecs_hdr_dol_lef;

     //sensor calibration
    handle->pCus_sensor_SetAEGain_cal   = pCus_SetAEGain_cal_hdr_dol_lef;
    handle->pCus_sensor_setCaliData_gain_linearity=pCus_setCaliData_gain_linearity_hdr_dol_lef;
    handle->pCus_sensor_GetShutterInfo = IMX327_GetShutterInfo_hdr_dol_lef;
#if 0//defined(__MV5_FPGA__)
    handle->pCus_sensor_I2CWrite = pCus_I2CWrite_hdr_dol_lef; //Andy Liu
    handle->pCus_sensor_I2CRead = pCus_I2CRead_hdr_dol_lef; //Andy Liu
#endif
    params->expo.vts = vts_30fps_HDR_DOL;
    params->expo.fps = 29;
    params->dirty = false;


    handle->channel_num = SENSOR_CHANNEL_NUM + 4;
    handle->channel_mode = SENSOR_CHANNEL_MODE_SONY_DOL;

    return SUCCESS;
}


SENSOR_DRV_ENTRY_IMPL_END_EX(  IMX327_HDR,
                            cus_camsensor_init_handle_linear,
                            cus_camsensor_init_handle_hdr_dol_sef1,
                            cus_camsensor_init_handle_hdr_dol_lef,
                            imx327_params
                         );

