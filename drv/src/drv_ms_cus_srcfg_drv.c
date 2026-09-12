#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/init.h>
#include <linux/version.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/semaphore.h>
#include <linux/init.h>
#include <linux/mutex.h> 
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/random.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/proc_fs.h>
#include "srcfg_drv.h"

#define DEVICE_NAME "srcfg"
#define DRV_VERSION "1.0"

static struct semaphore sem;
static unsigned char gsrcfg[64] = {0,};

//==============================================================================
//
//                              MACRO DEFINE
//
//==============================================================================

#define MAX_SUPPORT_SNR                     (2)
#define INVALID_PAD_SEL                     (0xFF)

#define CHIPTOP_REG_BASE                    (0xFD203C00)    // 0x101E
#define CLKGEN_REG_BASE                     (0xFD207000)    // 0x1038
#define VIFTOP_REG_BASE                     (0xFD263200)    // 0x1319

#define CHIPTOP_TO_SR0_BT656_REG_OFST       (0x15)
#define CHIPTOP_TO_SR0_BT656_REG_MASK       (0x0070)
#define CHIPTOP_TO_SR0_MIPI_REG_OFST        (0x15)
#define CHIPTOP_TO_SR0_MIPI_REG_MASK        (0x0380)
#define CHIPTOP_TO_SR0_PAR_REG_OFST         (0x15)
#define CHIPTOP_TO_SR0_PAR_REG_MASK         (0x1C00)

#define CHIPTOP_TO_SR_MODE_REG_OFST         (0x06)
#define CHIPTOP_TO_SR_MODE_REG_MASK         (0x0007)

#define CHIPTOP_TO_CCIR_MODE_REG_OFST         (0x0f)
#define CHIPTOP_TO_CCIR_MODE_REG_MASK         (0x0030)

#define CHIPTOP_TO_SR_MCLK_MODE_REG_OFST    (0x06)
#define CHIPTOP_TO_SR_MCLK_MODE_REG_MASK    (0x0080)

#define CHIPTOP_TO_SR_PDN_MODE_REG_OFST    (0x06)
#define CHIPTOP_TO_SR_PDN_MODE_REG_MASK    (0x0600)

#define CHIPTOP_TO_SR_RST_MODE_REG_OFST    (0x06)
#define CHIPTOP_TO_SR_RST_MODE_REG_MASK    (0x1800)


#define CLKGEN_TO_CLK_SR0_REG_OFST          (0x62)
#define CLKGEN_TO_CLK_SR0_REG_MASK          (0xFF00)

#define VIFTOP_TO_SR0_RST_REG_OFST          (0x00)
#define VIFTOP_TO_SR0_RST_REG_MASK          (0x0004)
#define VIFTOP_TO_SR0_POWER_DOWN_REG_OFST   (0x00)
#define VIFTOP_TO_SR0_POWER_DOWN_REG_MASK   (0x0008)


#define REG_WORD(base, idx)                 (*(((volatile unsigned short*)(base))+2*(idx)))
#define REG_WRITE(base, idx, val)           REG_WORD(base, idx) = (val)
#define REG_READ(base, idx)                 REG_WORD(base, idx)

//==============================================================================
//
//                              GLOBAL VARIABLES
//
//==============================================================================

static int m_iSnrBusType[MAX_SUPPORT_SNR];
//static int m_iSnrPadSel[MAX_SUPPORT_SNR];
static int m_iSnrMclkSel[MAX_SUPPORT_SNR];

//==============================================================================
//
//                              FUNCTIONS
//
//==============================================================================

void Set_SNR_BusType(int iSnrId, int iBusType)
{
    m_iSnrBusType[iSnrId] = iBusType;
}

void Set_SNR_PdnPad(int mode)
{
    u16 u16RegReadVal = 0;

    u16RegReadVal = REG_READ(CHIPTOP_REG_BASE, CHIPTOP_TO_SR_PDN_MODE_REG_OFST);
    u16RegReadVal &= ~(CHIPTOP_TO_SR_PDN_MODE_REG_MASK);
    u16RegReadVal |= ( (mode << 9) & CHIPTOP_TO_SR_PDN_MODE_REG_MASK);
    REG_WRITE(CHIPTOP_REG_BASE, CHIPTOP_TO_SR_PDN_MODE_REG_OFST, u16RegReadVal);

    return;
}

void Set_SNR_RstPad(int mode)
{
    u16 u16RegReadVal = 0;


    u16RegReadVal = REG_READ(CHIPTOP_REG_BASE, CHIPTOP_TO_SR_RST_MODE_REG_OFST);
    u16RegReadVal &= ~(CHIPTOP_TO_SR_RST_MODE_REG_MASK);
    u16RegReadVal |= ( (mode << 11) & CHIPTOP_TO_SR_RST_MODE_REG_MASK);
    REG_WRITE(CHIPTOP_REG_BASE, CHIPTOP_TO_SR_RST_MODE_REG_OFST, u16RegReadVal);

    return;
}

void Set_SNR_IOPad(int mode)
{
    u16 u16RegReadVal = 0;


    u16RegReadVal = REG_READ(CHIPTOP_REG_BASE, CHIPTOP_TO_SR_MCLK_MODE_REG_OFST); //sr mode always be 0
    u16RegReadVal &= ~(CHIPTOP_TO_SR_MCLK_MODE_REG_MASK);
    u16RegReadVal |= ( (1 << 7) & CHIPTOP_TO_SR_MCLK_MODE_REG_MASK);
    REG_WRITE(CHIPTOP_REG_BASE, CHIPTOP_TO_SR_MCLK_MODE_REG_OFST, u16RegReadVal); //mclk mode always be 1


    u16RegReadVal = REG_READ(CHIPTOP_REG_BASE, CHIPTOP_TO_SR_MODE_REG_OFST); //sr mode always be 0
    u16RegReadVal &= ~(CHIPTOP_TO_SR_MODE_REG_MASK);
    u16RegReadVal |= ( (0 << 0) & CHIPTOP_TO_SR_MODE_REG_MASK);
    REG_WRITE(CHIPTOP_REG_BASE, CHIPTOP_TO_SR_MODE_REG_OFST, u16RegReadVal);


    u16RegReadVal = REG_READ(CHIPTOP_REG_BASE, CHIPTOP_TO_CCIR_MODE_REG_OFST); //ccir mode always be 0
    u16RegReadVal &= ~(CHIPTOP_TO_CCIR_MODE_REG_MASK);
    u16RegReadVal |= ( (0 << 0) & CHIPTOP_TO_CCIR_MODE_REG_MASK);
    REG_WRITE(CHIPTOP_REG_BASE, CHIPTOP_TO_CCIR_MODE_REG_OFST, u16RegReadVal);

}

void Set_SNR_MCLK(int iSnrId, int iEnable, int iSpeedIdx)
{
    u16 u16RegReadVal = 0;
    tSensorConfig stSnrCfg;
        
    m_iSnrMclkSel[iSnrId] = iSpeedIdx;
    
    if (iEnable == 0)
        stSnrCfg.tRegSnrClkCfg.reg_ckg_sr_mclk_disable_clock = 1;
    else
        stSnrCfg.tRegSnrClkCfg.reg_ckg_sr_mclk_disable_clock = 0;
    
    stSnrCfg.tRegSnrClkCfg.reg_ckg_sr_mclk_invert_clock = 0;
    
    switch(iSpeedIdx) {
    case SNR_MCLK_27M:
    case SNR_MCLK_72M:
    case SNR_MCLK_61P7M:
    case SNR_MCLK_54M:
    case SNR_MCLK_48M:
    case SNR_MCLK_43P2M:
    case SNR_MCLK_36M:
    case SNR_MCLK_24M:
    case SNR_MCLK_21P6M:
    case SNR_MCLK_12M:
    case SNR_MCLK_5P4M:
    case SNR_MCLK_LPLL:
    case SNR_MCLK_LPLL_DIV2:
    case SNR_MCLK_LPLL_DIV4:
    case SNR_MCLK_LPLL_DIV8:
    case SNR_MCLK_ARMPLL_37P125:
        stSnrCfg.tRegSnrClkCfg.reg_ckg_sr_mclk_select_clock_source = iSpeedIdx;
        break;
    default:
        break;
    }


    u16RegReadVal = REG_READ(CLKGEN_REG_BASE, CLKGEN_TO_CLK_SR0_REG_OFST);
    u16RegReadVal &= ~(CLKGEN_TO_CLK_SR0_REG_MASK);
    u16RegReadVal |= ((u16)stSnrCfg.nRegSnrClkCfg & CLKGEN_TO_CLK_SR0_REG_MASK);
    REG_WRITE(CLKGEN_REG_BASE, CLKGEN_TO_CLK_SR0_REG_OFST, u16RegReadVal);


}

void SNR_PowerDown(int iSnrId, int iVal)
{
    u16 u16RegReadVal = 0;
    tSensorConfig stSnrCfg;
    
    switch(iSnrId)
    {
    case 0: 
        stSnrCfg.tRegVifSnrCtl.reg_vif_ch_sensor_pwrdn = iVal;
        u16RegReadVal = REG_READ(VIFTOP_REG_BASE, VIFTOP_TO_SR0_POWER_DOWN_REG_OFST);
        u16RegReadVal &= ~(VIFTOP_TO_SR0_POWER_DOWN_REG_MASK);
        u16RegReadVal |= ((u16)stSnrCfg.nRegVifSnrCtl & VIFTOP_TO_SR0_POWER_DOWN_REG_MASK);
        REG_WRITE(VIFTOP_REG_BASE, VIFTOP_TO_SR0_POWER_DOWN_REG_OFST, u16RegReadVal);
        break;
    default:
        break;
    }
}

void SNR_Reset(int iSnrId, int iVal)
{
    u16 u16RegReadVal = 0;
    tSensorConfig stSnrCfg;
    
    switch(iSnrId)
    {
    case 0:
        stSnrCfg.tRegVifSnrCtl.reg_vif_ch_sensor_rst = iVal;
        u16RegReadVal = REG_READ(VIFTOP_REG_BASE, VIFTOP_TO_SR0_RST_REG_OFST);
        u16RegReadVal &= ~(VIFTOP_TO_SR0_RST_REG_MASK);
        u16RegReadVal |= ((u16)stSnrCfg.nRegVifSnrCtl & VIFTOP_TO_SR0_RST_REG_MASK);
        REG_WRITE(VIFTOP_REG_BASE, VIFTOP_TO_SR0_RST_REG_OFST, u16RegReadVal);
        break;
    default:
        break;
    }
}

int do_sensor_cfg(int PdnPad, int RstPad, int MclkIdx, int Pdwn, int Rst)
{
    Set_SNR_IOPad(0);
    Set_SNR_PdnPad(PdnPad);
    Set_SNR_RstPad(RstPad);
    Set_SNR_MCLK(0, MclkIdx>=999?0:1, MclkIdx);

    SNR_PowerDown(0, Pdwn);
    SNR_Reset(0, Rst);

    printk("\n Sensor configuration, PdnPad=%u, RstPad=%u, MCLK=%u, PWD_PIN=%u, RST_PIN=%u \n\n",
            PdnPad, RstPad, MclkIdx, Pdwn, Rst);
    return 0;
}

static int srcfg_open(struct inode *inode, struct file *file)
{
    return 0;
}

static int srcfg_close(struct inode *inode, struct file *file)
{
    return 0;
}

static long srcfg_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    return 0;
}

ssize_t srcfg_read(struct file *filp, char __user *buff, size_t count, loff_t *offp)
{
    int ret = 0;

    if(down_interruptible(&sem))
    {
        return -ERESTARTSYS;
    }

    if(strlen(gsrcfg))
        printk("%s", gsrcfg);

    up(&sem);
    return ret;
}

ssize_t srcfg_write(struct file *filp, const char __user *buff, size_t count, loff_t *ppos)
{
    int ret = 0;
    void __user *argp = NULL;
    int help = 0;
    int PdnPad = -1;
    int RstPad = -1;
    int MclkIdx = -1;
    int PwdPin = -1;
    int RstPin = -1;
    char srcfg[64] = {0,};

    if(down_interruptible(&sem))
    {
        return -ERESTARTSYS;
    }
    argp = (void __user *)buff;
    memset(gsrcfg, 0, sizeof(gsrcfg));
    if(copy_from_user(gsrcfg, argp, count)){
        ret = -EFAULT;
    }else{
        *ppos += count;
        ret = count;
    }

    if(memcmp(gsrcfg, "help", 4) == 0)
        help = 1;
    else{
        memset(srcfg, 0, sizeof(srcfg));
        sscanf(gsrcfg, "%s %d %d %d %d %d",srcfg,&PdnPad,&RstPad,&MclkIdx,&PwdPin,&RstPin);
    }
    up(&sem);

    if(help){
       printk(  "sensor pin and mclk configuration:\n"
                "srcfg PdnPad RstPad MclkIdx PwdPin RstPin\n"
			    " -PdnPad   Select sensor power down pad  1:PAD_SR_IO12, 2:PAD_PWM0\n"
			    " -RstPad   Select sensor reset pad       1:PAD_SR_IO13, 2:PAD_PWM1\n"
			    " -MclkIdx 0:27MHz, 1:72MHz, 2:61.7MHz, 3:54MHz, 4:48MHz, 5:43.2MHz, \
			               6:36MHz, 7:24MHz, 8:21.6MHz, 9:12MHz, 10:5.4MHz, 999:MCLK off\n"
			    " -PwdPin sensor power down pin 0:low 1:high\n"
			    " -RstPin sensor reset pin 0:low 1:high\n");
    }else{
         do_sensor_cfg(PdnPad, RstPad, MclkIdx, PwdPin, RstPin);
    }

    return ret;
}

static struct file_operations srcfg_fops =
{
    .owner              = THIS_MODULE,
    .read               = srcfg_read,
    .write              = srcfg_write,
    .open               = srcfg_open,
    .release            = srcfg_close,
    .unlocked_ioctl     = srcfg_ioctl,
};

static struct miscdevice srcfg_miscdev =
{
    .minor  = MISC_DYNAMIC_MINOR,
    .name	= DEVICE_NAME,
    .fops	= &srcfg_fops,
};

static int __init srcfg_init(void)
{
    int ret = 0;
    
    sema_init(&sem, 1);
    ret = misc_register(&srcfg_miscdev);
    if (ret < 0) 
    {
        printk("%s driver can't register!\n",DEVICE_NAME);
        return ret; 
    }

    printk("%s driver %s register success!\n",DEVICE_NAME,DRV_VERSION);
    return 0;
}

static void __exit srcfg_exit(void)
{
    misc_deregister(&srcfg_miscdev);
    printk("%s driver %s exit success\n",DEVICE_NAME,DRV_VERSION);
}

module_init(srcfg_init);
module_exit(srcfg_exit);

MODULE_DESCRIPTION("srcfg driver");
MODULE_AUTHOR("SigmaStar");
MODULE_LICENSE("GPL");

