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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>

#include <cam_os_wrapper.h>
#include <linux/kernel.h>
#include <sensor_i2c_api.h>
#include <linux/delay.h>
#include <drv_ms_cus_sensor.h>
#include <drv_sensor.h>
#ifdef __cplusplus
}
#endif

//#define _DEBUG_
//c11 extern int usleep(u32 usec);
//int usleep(u32 usec);

#define SENSOR_CHANNEL_NUM (0)
#define SENSOR_CHANNEL_MODE_LINEAR CUS_SENSOR_CHANNEL_MODE_REALTIME_NORMAL
#define SENSOR_CHANNEL_MODE_SONY_DOL CUS_SENSOR_CHANNEL_MODE_RAW_STORE_HDR

//============================================
//MIPI config begin.
#define SENSOR_MIPI_LANE_NUM (2)
//#define SENSOR_MIPI_HDR_MODE (1) //0: Non-HDR mode. 1:Sony DOL mode
//MIPI config end.
//============================================

#define R_GAIN_REG 1
#define G_GAIN_REG 2
#define B_GAIN_REG 3


//#undef SENSOR_DBG
#define SENSOR_DBG 0

///////////////////////////////////////////////////////////////
//          @@@                                                                                       //
//       @   @@      ==  S t a r t * H e r e ==                                            //
//            @@      ==  S t a r t * H e r e  ==                                            //
//            @@      ==  S t a r t * H e r e  ==                                           //
//         @@@@                                                                                  //
//                                                                                                     //
//      Start Step 1 --  show preview on LCM                                         //
//                                                                                                    �@//
//  Fill these #define value and table with correct settings                        //
//      camera can work and show preview on LCM                                 //
//                                                                                                       //
///////////////////////////////////////////////////////////////

#define SENSOR_ISP_TYPE     ISP_EXT                   //ISP_EXT, ISP_SOC
#define F_number  22                                  // CFG, demo module
//#define SENSOR_DATAFMT      CUS_DATAFMT_BAYER        //CUS_DATAFMT_YUV, CUS_DATAFMT_BAYER
#define SENSOR_IFBUS_TYPE   CUS_SENIF_BUS_MIPI      //CFG //CUS_SENIF_BUS_PARL, CUS_SENIF_BUS_MIPI
#define SENSOR_MIPI_HSYNC_MODE PACKET_HEADER_EDGE1
#define SENSOR_DATAPREC     CUS_DATAPRECISION_10    //CFG //CUS_DATAPRECISION_8, CUS_DATAPRECISION_10
#define SENSOR_DATAMODE     CUS_SEN_10TO12_9000     //CFG
#define SENSOR_MAXGAIN      ((155*31)/10)
#define SENSOR_BAYERID      CUS_BAYER_BG            //CFG //CUS_BAYER_GB, CUS_BAYER_GR, CUS_BAYER_BG, CUS_BAYER_RG
#define SENSOR_RGBIRID      CUS_RGBIR_NONE
#define SENSOR_ORIT         CUS_ORIT_M0F0           //CUS_ORIT_M0F0, CUS_ORIT_M1F0, CUS_ORIT_M0F1, CUS_ORIT_M1F1,
#define SENSOR_MAX_GAIN     80                 // max sensor again, a-gain
//#define SENSOR_YCORDER      CUS_SEN_YCODR_YC       //CUS_SEN_YCODR_YC, CUS_SEN_YCODR_CY
#define lane_number 2
#define vc0_hs_mode 3   //0: packet header edge  1: line end edge 2: line start edge 3: packet footer edge
#define long_packet_type_enable 0x00 //UD1~UD8 (user define)

#define Preview_MCLK_SPEED  CUS_CMU_CLK_27MHZ        //CFG //CUS_CMU_CLK_12M, CUS_CMU_CLK_16M, CUS_CMU_CLK_24M, CUS_CMU_CLK_27M
//#define Preview_line_period 30000                  ////HTS/PCLK=4455 pixels/148.5MHZ=30usec @MCLK=36MHz
//#define vts_30fps 1125//1346,1616                 //for 29.1fps @ MCLK=36MHz
#define Preview_line_period 22222//30580                  //(36M/37.125M)*30fps=29.091fps(34.375msec), hts=34.375/1125=30556,
//#define Line_per_second     32727
#define vts_30fps  1500//1090                              //for 29.091fps @ MCLK=36MHz
#define Prv_Max_line_number 1440                    //maximum exposure line munber of sensor when preview
#define Preview_WIDTH       2560                    //resolution Width when preview
#define Preview_HEIGHT      1440                    //resolution Height when preview
#define Preview_MAX_FPS     30                     //fastest preview FPS
#define Preview_MIN_FPS     3                      //slowest preview FPS
#define Preview_CROP_START_X     0                      //CROP_START_X
#define Preview_CROP_START_Y     0                      //CROP_START_Y

#define SENSOR_I2C_ADDR    0x60                   //I2C slave address
#define SENSOR_I2C_SPEED   200000 //300000// 240000                  //I2C speed, 60000~320000

#define SENSOR_I2C_LEGACY  I2C_NORMAL_MODE     //usally set CUS_I2C_NORMAL_MODE,  if use old OVT I2C protocol=> set CUS_I2C_LEGACY_MODE
#define SENSOR_I2C_FMT     I2C_FMT_A16D8        //CUS_I2C_FMT_A8D8, CUS_I2C_FMT_A8D16, CUS_I2C_FMT_A16D8, CUS_I2C_FMT_A16D16

#define SENSOR_PWDN_POL     CUS_CLK_POL_NEG        // if PWDN pin High can makes sensor in power down, set CUS_CLK_POL_POS
#define SENSOR_RST_POL      CUS_CLK_POL_NEG        // if RESET pin High can makes sensor in reset state, set CUS_CLK_POL_NEG

// VSYNC/HSYNC POL can be found in data sheet timing diagram,
// Notice: the initial setting may contain VSYNC/HSYNC POL inverse settings so that condition is different.

#define SENSOR_VSYNC_POL    CUS_CLK_POL_POS        // if VSYNC pin High and data bus have data, set CUS_CLK_POL_POS
#define SENSOR_HSYNC_POL    CUS_CLK_POL_NEG        // if HSYNC pin High and data bus have data, set CUS_CLK_POL_POS
#define SENSOR_PCLK_POL     CUS_CLK_POL_POS        // depend on sensor setting, sometimes need to try CUS_CLK_POL_POS or CUS_CLK_POL_NEG
//static int  drv_Fnumber = 22;
#define ENABLE_ver 1
#define SGHD_LS_FOR_TBL (164282)        // 65536*0.068*(32*6)^2/1000
#define SGHD_HS_FOR_TBL (362388)    // 65536*0.15*(32*6)^2/1000
unsigned int u32Temp_Manual = 0, u32GainReport = 1024, u32LRUB = 3; // Davis 20181029
u16 gu16DgainUB = 0, gu16DgainLB = 0;
u16 gu16DgainUBH = 0, gu16DgainUBL = 0;
u16 gu16DgainLBH = 0, gu16DgainLBL = 0, u16GainMin = 0;
//static int  drv_Fnumber = 22;
static volatile long long framecount=0;
static volatile int FirstTime=1;
static volatile int fps_delay=5;
static volatile int ver=1;
static int pCus_SetAEGain(ms_cus_sensor *handle, u32 gain);
static int pCus_SetAEUSecs(ms_cus_sensor *handle, u32 us);
static int pCus_SetFPS(ms_cus_sensor *handle, u32 fps);
static int g_sensor_ae_min_gain = 1024;

CUS_MCLK_FREQ UseParaMclk(void);

CUS_CAMSENSOR_CAP sensor_cap = {
    .length = sizeof(CUS_CAMSENSOR_CAP),
    .version = 0x0001,
};

typedef struct {
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
    bool reg_mf;
    bool reg_dirty;
} sc4238_params;
// set sensor ID address and data,

typedef struct {
    u64 gain;
    u8 fine_gain_reg;
} FINE_GAIN;

I2C_ARRAY Sensor_id_table[] =
{
    {0x3107, 0x42},
    {0x3108, 0x35},
};

I2C_ARRAY Sensor_init_table[] =
{
		{0x0103,0x01},
		{0x0100,0x00},
		{0x3638,0x2a},
		{0x3304,0x30},
		{0x331e,0x29},
		{0x3635,0x40},
		{0x363b,0x03},
		{0x3352,0x02},
		{0x3356,0x1f},
		{0x33e0,0xc0},
		{0x33e1,0x08},
		{0x33e2,0x10},
		{0x33e3,0x10},
		{0x33e4,0x20},
		{0x33e5,0x08},
		{0x33e6,0x08},
		{0x33e7,0x10},
		{0x33e8,0x10},
		{0x33e9,0x10},
		{0x33ea,0x20},
		{0x33eb,0x04},
		{0x33ec,0x04},
		{0x33ed,0x10},
		{0x336d,0x01},
		{0x337f,0x2d},
		{0x3320,0x09},
		{0x3e14,0xb1},
		{0x3253,0x06},
		{0x391b,0x80},
		{0x391c,0x0f},
		{0x3905,0xd8},
		{0x3908,0x11},
		{0x4501,0xb4},
		{0x3637,0x22},
		{0x3e09,0x20},
		{0x3106,0x81},
		{0x3634,0x34},
		{0x3630,0xa8},
		{0x3631,0x80},
		{0x4509,0x20},
		{0x3200,0x00},
		{0x3201,0x48},
		{0x3204,0x0a},
		{0x3205,0x4f},
		{0x3202,0x00},
		{0x3203,0x28},
		{0x3206,0x05},
		{0x3207,0xcf},
		{0x3208,0x0a},
		{0x3209,0x00},
		{0x320a,0x05},
		{0x320b,0xa0},
		{0x3211,0x04},
		{0x3213,0x04},
		{0x320c,0x05},
		{0x320d,0x46},
		{0x320e,0x05},
		{0x320f,0xdc},
		{0x36e9,0x20},
		{0x36ea,0x37},
		{0x36eb,0x0c},
		{0x36ec,0x05},
		{0x36ed,0x24},
		{0x4837,0x20},
		{0x36f9,0x57},
		{0x36fa,0x25},
		{0x36fb,0x18},
		{0x36fc,0x01},
		{0x36fd,0x04},
		{0x3308,0x10},
		{0x3306,0x70},
		{0x330b,0xd0},
		{0x3309,0x40},
		{0x331f,0x39},
		{0x3e01,0xba},
		{0x3e02,0xe0},
		{0x3018,0x32},
		{0x3031,0x0a},
		{0x3037,0x20},
		{0x3038,0x22},
		{0x3366,0x92},
		{0x337a,0x08},
		{0x337b,0x10},
		{0x33a3,0x0c},
		{0x3314,0x94},
		{0x330e,0x14},
		{0x334c,0x10},
		{0x3633,0x43},
		{0x3622,0xee},
		{0x363a,0x80},
		{0x3364,0x1e},
		{0x3301,0x30},
		{0x3393,0x30},
		{0x3394,0x30},
		{0x3395,0x30},
		{0x3390,0x08},
		{0x3391,0x08},
		{0x3392,0x08},
		{0x3670,0x48},
		{0x366e,0x04},
		{0x3690,0x43},
		{0x3691,0x43},
		{0x3692,0x43},
		{0x369c,0x08},
		{0x369d,0x08},
		{0x3699,0x80},
		{0x369a,0x9f},
		{0x369b,0x9f},
		{0x36a2,0x08},
		{0x36a3,0x08},
		{0x360f,0x05},
		{0x3671,0xee},
		{0x3672,0x0e},
		{0x3673,0x0e},
		{0x367a,0x08},
		{0x367b,0x08},
		//add
		{0x33f4,0x10},
		{0x33f5,0x10},
		{0x33f6,0x20},
		{0x33f7,0x08},
		{0x33f8,0x08},
		{0x33f9,0x10},
		{0x33fa,0x10},
		{0x33fb,0x10},
		{0x33fc,0x20},
		{0x33fd,0x04},
		{0x33fe,0x04},
		{0x33ff,0x10},
		{0x3399,0xff},
		{0x0100,0x01},
};

FINE_GAIN fine_again[] = {
//gain map update for 1/32 precision
	{100000, 0x20},
	{103125, 0x21},
	{106250, 0x22},
	{109375, 0x23},
	{112500, 0x24},
	{115625, 0x25},
	{118750, 0x26},
	{121875, 0x27},
	{125000, 0x28},
	{128125, 0x29},
	{131250, 0x2a},
	{134375, 0x2b},
	{137500, 0x2c},
	{140625, 0x2d},
	{143750, 0x2e},
	{146875, 0x2f},
	{150000, 0x30},
	{153125, 0x31},
	{156250, 0x32},
	{159375, 0x33},
	{162500, 0x34},
	{165625, 0x35},
	{168750, 0x36},
	{171875, 0x37},
	{175000, 0x38},
	{178125, 0x39},
	{181250, 0x3a},
	{184375, 0x3b},
	{187500, 0x3c},
	{190625, 0x3d},
	{193750, 0x3e},
	{196875, 0x3f},
};


FINE_GAIN fine_dgain[] = {
    {10000, 0x80},
    {10625, 0x88},
    {11250, 0x90},
    {11875, 0x98},
    {12500, 0xa0},
    {13125, 0xa8},
    {13750, 0xb0},
    {14375, 0xb8},
    {15000, 0xc0},
    {15625, 0xc8},
    {16250, 0xd0},
    {16875, 0xd8},
    {17500, 0xe0},
    {18125, 0xe8},
    {18750, 0xf0},
    {19375, 0xf8},
};

I2C_ARRAY TriggerStartTbl[] = {
//{0x30f4,0x00},//Master mode start
};

I2C_ARRAY PatternTbl[] = {
    //pattern mode
};

I2C_ARRAY Current_Mirror_Flip_Tbl[] = {
    {0x3221, 0x00}, // mirror[2:1], flip[6:5]
    {0x3213, 0x04}, // crop for bayer
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


I2C_ARRAY mirr_flip_table[] =
{
    {0x3221, 0x00}, // mirror[2:1], flip[6:5]
    {0x3213, 0x04}, // crop for bayer
};

typedef struct {
    short reg;
    char startbit;
    char stopbit;
} COLLECT_REG_SET;

static I2C_ARRAY gain_reg[] = {
    {0x3e06,0x00},
    {0x3e07, 0x00},
    {0x3e08, 0x00|0x03},
    {0x3e09, 0x10}, //low bit, 0x10 - 0x3e0, step 1/16

};

I2C_ARRAY expo_reg[] = {

	{0x3e00, 0x00}, //expo [20:17]
    {0x3e01, 0x30}, // expo[16:8]
    {0x3e02, 0x00}, // expo[7:0], [3:0] fraction of line
};

I2C_ARRAY vts_reg[] = {
    {0x320e, 0x05},
    {0x320f, 0xdc}
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
#if SENSOR_DBG == 1
//#define SENSOR_DMSG(args...) SENSOR_DMSG(args)
//#define SENSOR_DMSG(args...) LOGE(args)
#define SENSOR_DMSG(args...) SENSOR_DMSG(args)
#elif SENSOR_DBG == 0
//#define SENSOR_DMSG(args...)
#endif
#undef SENSOR_NAME
#define SENSOR_NAME sc4238


#define SensorReg_Read(_reg,_data)     (handle->i2c_bus->i2c_rx(handle->i2c_bus,handle->i2c_cfg,_reg,_data))
#define SensorReg_Write(_reg,_data)    (handle->i2c_bus->i2c_tx(handle->i2c_bus,handle->i2c_cfg,_reg,_data))
#define SensorRegArrayW(_reg,_len)  (handle->i2c_bus->i2c_array_tx(handle->i2c_bus, handle->i2c_cfg,(_reg),(_len)))
#define SensorRegArrayR(_reg,_len)  (handle->i2c_bus->i2c_array_rx(handle->i2c_bus, handle->i2c_cfg,(_reg),(_len)))

int cus_camsensor_release_handle(ms_cus_sensor *handle);

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
#endif
static int pCus_poweron(ms_cus_sensor *handle, u32 idx)
{
    ISensorIfAPI *sensor_if = &handle->sensor_if_api;
    SENSOR_DMSG("[%s] ", __FUNCTION__);

    //Sensor power on sequence
    sensor_if->MCLK(idx, 1, handle->mclk);

    sensor_if->SetIOPad(idx, handle->sif_bus, handle->interface_attr.attr_mipi.mipi_lane_num);
    sensor_if->SetCSI_Clk(idx, CUS_CSI_CLK_216M);
    sensor_if->SetCSI_Lane(idx, handle->interface_attr.attr_mipi.mipi_lane_num, 1);
    sensor_if->SetCSI_LongPacketType(idx, 0, 0x1C00, 0);

    SENSOR_DMSG("[%s] reset low\n", __FUNCTION__);
    sensor_if->Reset(idx, handle->reset_POLARITY);
    CamOsMsSleep(1);
    SENSOR_DMSG("[%s] power low\n", __FUNCTION__);
    sensor_if->PowerOff(idx, handle->pwdn_POLARITY);
    CamOsMsSleep(1);

    // power -> high, reset -> high
    SENSOR_DMSG("[%s] power high\n", __FUNCTION__);
    sensor_if->PowerOff(idx, !handle->pwdn_POLARITY);
    CamOsMsSleep(1);
    SENSOR_DMSG("[%s] reset high\n", __FUNCTION__);
    sensor_if->Reset(idx, !handle->reset_POLARITY);
    CamOsMsSleep(1);

    //sensor_if->Set3ATaskOrder(handle, def_order);
    // pure power on
    //ISP_config_io(handle);
    sensor_if->PowerOff(idx, !handle->pwdn_POLARITY);
    CamOsMsSleep(5);
    //handle->i2c_bus->i2c_open(handle->i2c_bus,&handle->i2c_cfg);

    return SUCCESS;
}

static int pCus_poweroff(ms_cus_sensor *handle, u32 idx)
{
    // power/reset low
    ISensorIfAPI *sensor_if = &handle->sensor_if_api;
    SENSOR_DMSG("[%s] power low\n", __FUNCTION__);
    sensor_if->PowerOff(idx, handle->pwdn_POLARITY);
    //handle->i2c_bus->i2c_close(handle->i2c_bus);
    CamOsMsSleep(1);
    //Set_csi_if(0, 0);
    sensor_if->SetCSI_Clk(idx, CUS_CSI_CLK_DISABLE);
    sensor_if->MCLK(idx, 0, handle->mclk);

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

	SensorReg_Write(0xef,0x00);

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
		  continue;
    }

    //convert sensor id to u32 format
    for(i=0;i<table_length;++i)
    {
      if( id_from_sensor[i].data != Sensor_id_table[i].data )
        return FAIL;
      //*id = id_from_sensor[i].data;
      *id = ((*id)+ id_from_sensor[i].data)<<8;
    }

    *id >>= 8;
    SENSOR_DMSG("[%s]sc4238 Read sensor id, get 0x%x Success\n", __FUNCTION__, (int)*id);
    //SENSOR_DMSG("[%s]Read sensor id, get 0x%x Success\n", __FUNCTION__, (int)*id);

    return SUCCESS;
}

static int sc4238_SetPatternMode(ms_cus_sensor *handle,u32 mode)
{

    SENSOR_DMSG("\n\n[%s], mode=%d \n", __FUNCTION__, mode);

    return SUCCESS;
}
static int pCus_SetFPS(ms_cus_sensor *handle, u32 fps);
static int pCus_SetAEGain_cal(ms_cus_sensor *handle, u32 gain);
static int pCus_AEStatusNotify(ms_cus_sensor *handle, CUS_CAMSENSOR_AE_STATUS_NOTIFY status);
static int pCus_init(ms_cus_sensor *handle)
{
	int i,cnt=0;
	//ISensorIfAPI *sensor_if = &handle->sensor_if_api;
        SENSOR_DMSG("\n\n[%s]", __FUNCTION__);
    //ISensorIfAPI *sensor_if = &(handle->sensor_if_api);
    //sensor_if->PCLK(NULL,CUS_PCLK_MIPI_TOP);


        //sensor_if->SetCSI_Clk(0, CUS_CSI_CLK_216M);
        //sensor_if->SetCSI_Lane(0, handle->interface_attr.attr_mipi.mipi_lane_num, 1);
        //sensor_if->SetCSI_LongPacketType(0, 0, 0x1C00, 0);

        for(i=0;i< ARRAY_SIZE(Sensor_init_table);i++)
        {
		    if(Sensor_init_table[i].reg==0xffff)
		    {
			    SENSOR_MSLEEP(Sensor_init_table[i].data);
		    }
		    else
		    {
			    cnt = 0;
			    while(SensorReg_Write(Sensor_init_table[i].reg,Sensor_init_table[i].data) != SUCCESS)
			    {
				    cnt++;
				    //SENSOR_DMSG("Sensor_init_table -> Retry %d...\n",cnt);
				    if(cnt>=10)
				    {
					    //SENSOR_DMSG("[%s:%d]Sensor init fail!!\n", __FUNCTION__, __LINE__);
					    return FAIL;
			 	    }
				    SENSOR_MSLEEP(10);
			    }
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


    FirstTime=1;

    pCus_SetAEGain(handle,1024); //Set sensor gain = 1x
    pCus_SetAEUSecs(handle, 30000);
    pCus_AEStatusNotify(handle,CUS_FRAME_ACTIVE);

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
    u32 num_res = handle->video_res_supported.num_res;

    if (res_idx >= num_res) {
        return FAIL;
    }

    *res = &handle->video_res_supported.res[res_idx];

    return SUCCESS;
}

static int pCus_GetCurVideoRes(ms_cus_sensor *handle, u32 *cur_idx, cus_camsensor_res **res)
{
    u32 num_res = handle->video_res_supported.num_res;

    *cur_idx = handle->video_res_supported.ulcur_res;

    if (*cur_idx >= num_res) {
        return FAIL;
    }

    *res = &handle->video_res_supported.res[*cur_idx];

    return SUCCESS;
}

static int pCus_SetVideoRes(ms_cus_sensor *handle, u32 res_idx)
{
    u32 num_res = handle->video_res_supported.num_res;
    if (res_idx >= num_res) {
        return FAIL;
    }
    switch (res_idx) {
        case 0:
            handle->video_res_supported.ulcur_res = 0;
            handle->pCus_sensor_init = pCus_init;
            break;

        default:
            break;
    }

    return SUCCESS;
}

static int pCus_GetOrien(ms_cus_sensor *handle, CUS_CAMSENSOR_ORIT *orit) {
    char sen_data;
    sen_data = mirr_flip_table[0].data;
    SENSOR_DMSG("mirror:%x\r\n", sen_data);
    switch(sen_data) {
      case 0x00:
        *orit = CUS_ORIT_M0F0;
        break;
      case 0x06:
        *orit = CUS_ORIT_M1F0;
        break;
      case 0x60:
        *orit = CUS_ORIT_M0F1;
        break;
      case 0x66:
        *orit = CUS_ORIT_M1F1;
        break;
      }
      return SUCCESS;
}

static int pCus_SetOrien(ms_cus_sensor *handle, CUS_CAMSENSOR_ORIT orit) {
   sc4238_params *params = (sc4238_params *)handle->private_data;
   switch(orit) {
     case CUS_ORIT_M0F0:
       if (mirr_flip_table[0].data) {
    	   mirr_flip_table[0].data = 0;
    	   mirr_flip_table[1].data = 2;
           params->reg_mf = true;
       }
       break;
     case CUS_ORIT_M1F0:
       if (mirr_flip_table[0].data!=6) {
    	   mirr_flip_table[0].data = 6;
    	   mirr_flip_table[1].data = 2;
           params->reg_mf = true;
       }
       break;
     case CUS_ORIT_M0F1:
       if (mirr_flip_table[0].data!=0x60) {
    	   mirr_flip_table[0].data = 0x60;
    	   mirr_flip_table[1].data = 2;
           params->reg_mf = true;
       }
       break;
     case CUS_ORIT_M1F1:
       if (mirr_flip_table[0].data!=0x66) {
    	   mirr_flip_table[0].data = 0x66;
           mirr_flip_table[1].data = 2;
           params->reg_mf = true;
       }
       break;
   }
     return SUCCESS;
}

static int pCus_GetFPS(ms_cus_sensor *handle)
{
    sc4238_params *params = (sc4238_params *)handle->private_data;
    //SENSOR_DMSG("[%s]", __FUNCTION__);

    return  params->expo.fps;
}
static int pCus_SetFPS(ms_cus_sensor *handle, u32 fps)
{
	sc4238_params *params = (sc4238_params *)handle->private_data;
    SENSOR_DMSG("\n\n ****************  [%s], fps=%d  **************** \n", __FUNCTION__, fps);
    if(fps>=3 && fps <= 30) {
        params->expo.fps = fps;
        params->expo.vts=  (vts_30fps*30)/fps;
        vts_reg[0].data = (params->expo.vts >> 8) & 0x00ff;
        vts_reg[1].data = (params->expo.vts >> 0) & 0x00ff;
        params->reg_dirty = true;
        return SUCCESS;
    }

    if(fps>=3000 && fps <= 30000) {
        params->expo.fps = fps;
        params->expo.vts=  (vts_30fps*30000)/fps;
        vts_reg[0].data = (params->expo.vts >> 8) & 0x00ff;
        vts_reg[1].data = (params->expo.vts >> 0) & 0x00ff;
        params->reg_dirty = true;
        return SUCCESS;
    }else{
        SENSOR_DMSG("[%s] FPS %d out of range.\n",__FUNCTION__,fps);
        return FAIL;
    }

}

#if 0
static int pCus_GetSensorCap(ms_cus_sensor *handle, CUS_CAMSENSOR_CAP *cap) {
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
static int pCus_AEStatusNotify(ms_cus_sensor *handle, CUS_CAMSENSOR_AE_STATUS_NOTIFY status){

   sc4238_params *params = (sc4238_params *)handle->private_data;

   switch(status)
   {
       case CUS_FRAME_INACTIVE:
       if(params->reg_mf)
       {
           //sensor_if->SetSkipFrame(handle,2);  //to do skip frame method
           SensorRegArrayW((I2C_ARRAY*)mirr_flip_table, sizeof(mirr_flip_table)/sizeof(I2C_ARRAY));
           params->reg_mf = false;
       }
       break;
       case CUS_FRAME_ACTIVE:
       if(params->reg_dirty)
       {
          SensorRegArrayW((I2C_ARRAY*)expo_reg, sizeof(expo_reg)/sizeof(I2C_ARRAY));
          SensorRegArrayW((I2C_ARRAY*)gain_reg, sizeof(gain_reg)/sizeof(I2C_ARRAY));
          SensorRegArrayW((I2C_ARRAY*)vts_reg, sizeof(vts_reg)/sizeof(I2C_ARRAY));
          params->reg_dirty = false;
       }
       break;
       default :
       break;
   }
   return SUCCESS;
}

static int pCus_GetAEUSecs(ms_cus_sensor *handle, u32 *us) {
  int rc=0;
  u32 lines = 0;
  lines |= (u32)(expo_reg[0].data&0x0f)<<16;
  lines |= (u32)(expo_reg[1].data&0xff)<<8;
  lines |= (u32)(expo_reg[2].data&0xf0)<<0;
  lines >>= 4;
  *us = (lines*Preview_line_period)/1000/2; //return us

  SENSOR_DMSG("[%s] sensor expo lines/us %d, %dus\n", __FUNCTION__, lines, *us);
  return rc;
}

static int pCus_SetAEUSecs(ms_cus_sensor *handle, u32 us) {
    int i;
    u32 half_lines = 0,vts = 0;
    sc4238_params *params = (sc4238_params *)handle->private_data;
    I2C_ARRAY expo_reg_temp[] = {  // max expo line vts-4!
    {0x3e00, 0x00},//expo [20:17]
    {0x3e01, 0x00}, // expo[16:8]
    {0x3e02, 0x10}, // expo[7:0], [3:0] fraction of line
    };
    memcpy(expo_reg_temp, expo_reg, sizeof(expo_reg));

    half_lines = (1000*us*2)/Preview_line_period; // Preview_line_period in ns
    if(half_lines<3) half_lines=3;
    if (half_lines >  2 * (params->expo.vts)-10) {
        half_lines = 2 * (params->expo.vts)-10;
    }
    else
     vts=params->expo.vts;
    SENSOR_DMSG("[%s] us %ld, half_lines %ld, vts %ld\n", __FUNCTION__, us, half_lines, params->expo.vts);

    half_lines = half_lines<<4;
//	printf("===================================================================\n");
//	printf("us = %d  half_lines = %x params->expo.vts = %x\n",us, half_lines, params->expo.vts);
//	printf("===================================================================\n");
    expo_reg[0].data = (half_lines>>16) & 0x0f;
    expo_reg[1].data =  (half_lines>>8) & 0xff;
    expo_reg[2].data = (half_lines>>0) & 0xf0;

    for (i = 0; i < sizeof(expo_reg)/sizeof(I2C_ARRAY); i++)
    {
      if (expo_reg[i].data != expo_reg_temp[i].data)
      {
        params->reg_dirty = true;
        break;
      }
     }
    return SUCCESS;
}

// Gain: 1x = 1024
static int pCus_GetAEGain(ms_cus_sensor *handle, u32* gain) {
    int rc = 0;
    u8 Dgain = 1,  Coarse_gain = 1;

    Coarse_gain = ((gain_reg[2].data&0x1C)>>2) +1;
    Dgain = ((gain_reg[0].data&0x0f) + 1);

    *gain = (Coarse_gain*Dgain*(gain_reg[1].data)*(gain_reg[3].data))/2;


    //SENSOR_DMSG("[%s] gain/reg = %d, 0x%x,  0x%x  0x%x 0x%x\n", __FUNCTION__, *gain,gain_reg[0].data,gain_reg[1].data,gain_reg[2].data,gain_reg[3].data);
    //SENSOR_DMSG("[%s] gain/reg = %d, %f ,%d, %d %f %f\n", __FUNCTION__, *gain,temp_gain,Coarse_gain,Dgain,Fine_again,Fine_dgain);
    return rc;
}

static int pCus_SetAEGain_cal(ms_cus_sensor *handle, u32 gain) {

    return SUCCESS;
}

static int pCus_SetAEGain(ms_cus_sensor *handle, u32 gain) {
    sc4238_params *params = (sc4238_params *)handle->private_data;
    u8 i=0 ,Dgain = 1,  Coarse_gain = 1;
    u64 Fine_again = 100000,Fine_dgain = 10000;
    u8 Dgain_reg = 0, Coarse_gain_reg = 0, Fine_again_reg= 0x10,Fine_dgain_reg= 0x80;

    I2C_ARRAY gain_reg_temp[] = {
        {0x3e06, 0x00},
        {0x3e07, 0x00},
        {0x3e08, (0x00|0x03)},
        {0x3e09, 0x10},
    };
    memcpy(gain_reg_temp, gain_reg, sizeof(gain_reg));

    if (gain < 1024) {
        gain = 1024;
    } else if (gain > SENSOR_MAXGAIN*1024) {
        gain = SENSOR_MAXGAIN*1024;
    }

    if (gain < 2 * 1024)
    {
        Dgain = 1;      Fine_dgain = 10000;         Coarse_gain = 1;
        Dgain_reg = 0;  Fine_dgain_reg = 0x80;  Coarse_gain_reg = 0;
    }
    else if (gain <  4 * 1024)
    {
        Dgain = 1;      Fine_dgain = 10000;         Coarse_gain = 2;
        Dgain_reg = 0;  Fine_dgain_reg = 0x80;  Coarse_gain_reg = 1;
    }
    else if (gain <  8 * 1024)
    {
        Dgain = 1;      Fine_dgain = 10000;         Coarse_gain = 4;
        Dgain_reg = 0;  Fine_dgain_reg = 0x80;  Coarse_gain_reg = 3;
    }
    else if (gain <=  (1575/100) * 1024)
    {
        Dgain = 1;      Fine_dgain = 10000;         Coarse_gain = 8;
        Dgain_reg = 0;  Fine_dgain_reg = 0x80;  Coarse_gain_reg = 7;
    }
    else if (gain <  (3150/100) * 1024)
    {
        Dgain = 1;      Fine_again = 196875;    Coarse_gain = 8;
        Dgain_reg = 0;  Fine_again_reg = 0x1f;  Coarse_gain_reg = 7;
    }
    else if (gain <  (6300/100) * 1024)
    {
        Dgain = 2;      Fine_again = 196875;    Coarse_gain = 8;
        Dgain_reg = 1;  Fine_again_reg = 0x1f;  Coarse_gain_reg = 7;
    }
     else if (gain < (12600/100) * 1024)
    {
        Dgain = 4;      Fine_again = 196875;    Coarse_gain = 8;
        Dgain_reg = 3;  Fine_again_reg = 0x1f;  Coarse_gain_reg = 7;
    }
        else if (gain < SENSOR_MAXGAIN * 1024)
    {
        Dgain = 8;      Fine_again = 196875;    Coarse_gain = 8;
        Dgain_reg = 7;  Fine_again_reg = 0x1f;  Coarse_gain_reg = 7;
    }

    if (gain <= (1575/100) * 1024)
    {
        Fine_again = gain*10000 / (Dgain * Coarse_gain * Fine_dgain)*100000/ 1024;
        for(i = 1; i< sizeof(fine_again)/sizeof(FINE_GAIN);i++)
        {
            if(Fine_again >= fine_again[i-1].gain && Fine_again <= fine_again[i].gain)
            {
                Fine_again_reg = (fine_again[i].gain - Fine_again) > (Fine_again - fine_again[i-1].gain) ? fine_again[i-1].fine_gain_reg:fine_again[i].fine_gain_reg;
                break;
            }
            else if(Fine_again > fine_again[31].gain)
            {
                Fine_again_reg = 0x3f;
                break;
            }
        }
    }
    else
    {
        Fine_dgain = gain*100000 / (Dgain * Coarse_gain * Fine_again)*10000/ 1024;
        for(i = 1; i< sizeof(fine_dgain)/sizeof(FINE_GAIN);i++)
        {
            if(Fine_dgain >= fine_dgain[i-1].gain && Fine_dgain <= fine_dgain[i].gain)
            {
                Fine_dgain_reg = (fine_dgain[i].gain - Fine_dgain) > (Fine_dgain - fine_dgain[i-1].gain) ? fine_dgain[i-1].fine_gain_reg:fine_dgain[i].fine_gain_reg;
                break;
            }
            else if(Fine_dgain > fine_dgain[15].gain)
            {
                Fine_dgain_reg = 0xf8;
                break;
            }
        }
    }

    SENSOR_DMSG("[%s]  gain : %f ,%f ,%d , %d\n\n", __FUNCTION__,Fine_again,Fine_dgain,Dgain,Coarse_gain);
    SENSOR_DMSG("[%s]  gain : %x ,%x ,%x , %x\n\n", __FUNCTION__,Dgain_reg,Fine_dgain_reg,Fine_again_reg,Coarse_gain_reg);
    gain_reg[3].data = Fine_again_reg;
    gain_reg[2].data = ((Coarse_gain_reg<<2) & 0x1C) | 0x03;
    gain_reg[1].data = Fine_dgain_reg;
    gain_reg[0].data = Dgain_reg & 0xF;

    for (i = 0; i < sizeof(gain_reg)/sizeof(I2C_ARRAY); i++)
    {
      if (gain_reg[i].data != gain_reg_temp[i].data)
      {
        params->reg_dirty = true;
        break;
      }
    }


    return SUCCESS;
}

static int pCus_GetAEMinMaxUSecs(ms_cus_sensor *handle, u32 *min, u32 *max) {
    *min = 30;
    *max = 1000000/Preview_MIN_FPS;
    return SUCCESS;
}

static int pCus_GetAEMinMaxGain(ms_cus_sensor *handle, u32 *min, u32 *max) {
  *min = 1024;
  *max = SENSOR_MAXGAIN*1024;
  return SUCCESS;
}

static int sc4238_GetShutterInfo(struct __ms_cus_sensor* handle,CUS_SHUTTER_INFO *info)
{
    info->max = 1000000000/Preview_MIN_FPS;
    info->min = Preview_line_period * 3;
    info->step = Preview_line_period;
    return SUCCESS;
}

static int pCus_setCaliData_gain_linearity(ms_cus_sensor* handle, CUS_GAIN_GAP_ARRAY* pArray, u32 num) {

    return SUCCESS;
}

int cus_camsensor_init_handle(ms_cus_sensor* drv_handle) {
   ms_cus_sensor *handle = drv_handle;
    sc4238_params *params;
    if (!handle) {
        SENSOR_DMSG("[%s] not enough memory!\n", __FUNCTION__);
        return FAIL;
    }
    SENSOR_DMSG("[%s]", __FUNCTION__);
    //private data allocation & init
    handle->private_data = CamOsMemCalloc(1, sizeof(sc4238_params));
    params = (sc4238_params *)handle->private_data;

    ////////////////////////////////////
    //    sensor model ID                           //
    ////////////////////////////////////
    strcpy(handle->model_id,"sc4238_MIPI");

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
    handle->interface_attr.attr_mipi.mipi_hdr_virtual_channel_num = 0; //Short frame

    ////////////////////////////////////
    //    resolution capability       //
    ////////////////////////////////////

    handle->video_res_supported.num_res = 1;
    handle->video_res_supported.ulcur_res = 0;
    handle->video_res_supported.res[0].width = Preview_WIDTH;
    handle->video_res_supported.res[0].height = Preview_HEIGHT;
    handle->video_res_supported.res[0].max_fps= Preview_MAX_FPS;
    handle->video_res_supported.res[0].min_fps= Preview_MIN_FPS;
    handle->video_res_supported.res[0].crop_start_x= 0;
    handle->video_res_supported.res[0].crop_start_y= 0;
    handle->video_res_supported.res[0].nOutputWidth= 0;
    handle->video_res_supported.res[0].nOutputHeight= 0;
    strcpy(handle->video_res_supported.res[0].strResDesc, "2560x1440@30fps");

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
    handle->VSYNC_POLARITY              = SENSOR_VSYNC_POL; //CUS_CLK_POL_POS;
    handle->HSYNC_POLARITY              = SENSOR_HSYNC_POL; //CUS_CLK_POL_POS;
    handle->PCLK_POLARITY               = SENSOR_PCLK_POL;  //CUS_CLK_POL_POS);    // use '!' to clear board latch error
    /////////////////////////////////////////////////////

    ////////////////////////////////////////////////////
    // AE parameters
    ////////////////////////////////////////////////////
    handle->ae_gain_delay       = 2;
    handle->ae_shutter_delay    = 2;

    handle->ae_gain_ctrl_num = 1;
    handle->ae_shutter_ctrl_num = 1;

    ///calibration
    handle->sat_mingain=g_sensor_ae_min_gain;


    handle->pCus_sensor_release     = cus_camsensor_release_handle;
    handle->pCus_sensor_init        = pCus_init    ;

    handle->pCus_sensor_poweron     = pCus_poweron ;
    handle->pCus_sensor_poweroff    = pCus_poweroff;

    // Normal
    handle->pCus_sensor_GetSensorID       = pCus_GetSensorID   ;

    handle->pCus_sensor_GetVideoResNum = pCus_GetVideoResNum;
    handle->pCus_sensor_GetVideoRes       = pCus_GetVideoRes;
    handle->pCus_sensor_GetCurVideoRes  = pCus_GetCurVideoRes;
    handle->pCus_sensor_SetVideoRes       = pCus_SetVideoRes;

    handle->pCus_sensor_GetOrien          = pCus_GetOrien      ;
    handle->pCus_sensor_SetOrien          = pCus_SetOrien      ;
    handle->pCus_sensor_GetFPS          = pCus_GetFPS      ;
    handle->pCus_sensor_SetFPS          = pCus_SetFPS      ;
    //handle->pCus_sensor_GetSensorCap    = pCus_GetSensorCap;
    handle->pCus_sensor_SetPatternMode = sc4238_SetPatternMode;
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

    handle->pCus_sensor_SetAEGain       = pCus_SetAEGain;

    handle->pCus_sensor_GetAEMinMaxGain = pCus_GetAEMinMaxGain;
    handle->pCus_sensor_GetAEMinMaxUSecs= pCus_GetAEMinMaxUSecs;

     //sensor calibration
    handle->pCus_sensor_SetAEGain_cal   = pCus_SetAEGain_cal;
    handle->pCus_sensor_setCaliData_gain_linearity=pCus_setCaliData_gain_linearity;
    handle->pCus_sensor_GetShutterInfo = sc4238_GetShutterInfo;
    params->expo.vts=vts_30fps;
    params->expo.fps = 30;
    params->reg_dirty = false;
    params->reg_mf = false;
    return SUCCESS;
}

int cus_camsensor_release_handle(ms_cus_sensor *handle) {
    //ISensorIfAPI *sensor_if = &handle->sensor_if_api;
    //sensor_if->PCLK(NULL,CUS_PCLK_OFF);
    //sensor_if->SetCSI_Clk(handle,CUS_CSI_CLK_DISABLE);
    if (handle && handle->private_data) {
        SENSOR_DMSG("[%s] release handle, handle %x, private data %x",
                __FUNCTION__,
                (int)handle,
                (int)handle->private_data);
        CamOsMemRelease(handle->private_data);
        handle->private_data = NULL;
    }
    return SUCCESS;
}

/******************** Linux kernel library *******************/
int chmap = 0;
module_param(chmap, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(chmap, "VIF channel mapping");

static int __init sigmastar_sc4238_init_driver(void)
{
    int nCamID=0;
    for(nCamID=0;nCamID<4;++nCamID)
    {
        if((chmap>>nCamID)&0x1) //if bitmap bit is 1, register sensor
        {
            DrvRegisterSensorDriver(nCamID, cus_camsensor_init_handle);
            pr_info("Connect %s linear to sensor pad %d\n",__FUNCTION__, nCamID);
        }
    }
    return 0;
}
static void __exit sigmastar_sc4238_exit_driver(void)
{
    pr_info("sensordrv exit");
}

subsys_initcall(sigmastar_sc4238_init_driver);
module_exit(sigmastar_sc4238_exit_driver);

MODULE_DESCRIPTION("Sensor_SC4238");
MODULE_AUTHOR("SigmaStar");
MODULE_LICENSE("GPL");
