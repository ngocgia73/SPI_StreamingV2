/*
 * @file : spi_controller_driver.c
 * @author: Daniel Nguyen <daniel.nguyen0105@gmail.com>
 * @date: 2019-10-05
 * @des: driver  for spi-controller device which belong to GM8135/8136 chipset
 *       which kind of controller driver done by chip provider usually
 */
// must have
#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/spi/spi.h>
#include <asm/sizes.h>

#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/spi/flash.h>
#include <linux/sched.h>
#include <linux/io.h>
#include <mach/ftpmu010.h>
#include "spi_driver_controller.h"
#include <mach/hardware.h>
#include <linux/gpio.h>

#include <linux/mutex.h>
#if CONFIG_FTSPI010_USE_AHBDMA
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <mach/ftdmac020.h>
#endif
// ==================================================
// =================start define macro ==============
// ==================================================
#define DRV_NAME 	"ssp-spi"

#define module_device_driver(__device, __device_register_func, __device_unregister_func, \
		                     __driver, __driver_register_func, __driver_unregister_func, \
		                     __spi_board_info, __spi_register_board_info_func) \
static int __init sspc_init(void) \
{\
    int ret = -1; \
	ret = __device_register_func(&(__device)); \
    if(ret < 0) \
        return ret; \
	ret = __spi_register_board_info_func(__spi_board_info, ARRAY_SIZE(__spi_board_info)); \
    if(ret < 0) \
        return ret; \
	ret = __driver_register_func(&(__driver)); \
    return ret; \
}\
static void __exit sspc_exit(void) \
{\
	__driver_unregister_func(&(__driver)); \
	__device_unregister_func(&(__device)); \
}\
module_init(sspc_init); \
module_exit(sspc_exit);

#define module_platform_device_driver(__platform_driver, __platform_device, __spi_board_info) \
	module_device_driver(__platform_device, platform_device_register, platform_device_unregister, \
		             __platform_driver, platform_driver_register, platform_driver_unregister, \
			     __spi_board_info, spi_register_board_info)

#define GPIO_CS // SSP have only one cs pin so if we want to control more spi device then need to use GPIO as cs pin

#ifdef GPIO_CS
#define GPIO_pin_cs0    14
#define GPIO_pin_cs1    18
#define GPIO_pin_cs2    19
#define GPIO_pin_cs3    26
#endif


#define MODEBITS  	(SPI_CPOL | SPI_CPHA | SPI_LSB_FIRST) // MODE 3 + LSB TRANSFER FIRST 

#define MOSI_DMA_EN 1
#define MISO_DMA_EN 1

#define MSB_SPI 1

// giann debug
// #define DEBUG_SSPC_EN
// #define DEBUG_TIME 
// ====================================================
// =============== end define macro====================
// ====================================================

// ====================================================
// ==============start define global variable =========
// ====================================================
static DEFINE_MUTEX(cs_mutex);
#ifdef GPIO_CS
static pmuReg_t  ssp1_pmu_reg[] = {
    /* off, bitmask,   lockbit,    init val,   init mask */
    {0x28, (0x3 << 13), (0x3 << 13), (0x2 << 13), (0x3 << 13)},   /* SSP1 CLK from PLL2 */
    {0x54, (0xFC << 14), (0xFC << 14), (0xFC << 14), (0xFC << 14)},   /* pinMux *///justin
    {0x74, (0x3F << 8), (0x3F << 8), (0x31 << 8), (0x3F << 8)},   /* SSP1 clock divided value */
    {0x7C, (0x1 << 29), (0x1 << 29), (0x1 << 29), (0x1 << 29)},   /* SSP1 source select external */
    {0xB8, (0x1 << 5), (0x1 << 5), (0x0 << 5), (0x1 << 5)},   /* apb mclk on */
};

static pmuRegInfo_t	spi_1_clk_pinmux_info = {
    "SSP010_1",
    ARRAY_SIZE(ssp1_pmu_reg),
    ATTR_TYPE_NONE,
    ssp1_pmu_reg
};
#endif

// ====================================================
// =============end define global variable=============
// ====================================================

//***keep original function from chip provider
// ====================================================
// =============start define internal function ========
// ====================================================
#ifdef GPIO_CS
static void spi_1_pmu_init(struct ftssp010_spi_hw_platform *hw_platform)
{
    u32 ssp1clk_pvalue = 0, CLK;
    int spi_1_fd = -1;

    spi_1_fd = ftpmu010_register_reg(&spi_1_clk_pinmux_info);
    if (unlikely(spi_1_fd < 0)){
        printk("In %s: SPI 1 registers to PMU fail! \n", __func__);
    }

    // read current SPI1 working clock, NOTE: the working of SSP on 8126 can not be over 81MHz due to HW limitation
    ssp1clk_pvalue = (ftpmu010_read_reg(0x74) >> 8) & 0x7F;
	ssp1clk_pvalue &= ~0x7F;
	ssp1clk_pvalue |= 0x06;
#ifdef CONFIG_PLATFORM_GM8139
    CLK = ftpmu010_get_attr(ATTR_TYPE_PLL3);
#else    
    CLK = ftpmu010_get_attr(ATTR_TYPE_PLL2);
#endif
    // working_clk : clock IP is working now
    // note that don't set speed of SPI out of IP's capability
    hw_platform->working_clk = (CLK / (ssp1clk_pvalue + 1));

    //printk("ssp1clk_pvalue = %d\n", ssp1clk_pvalue);
    FTSSP010_SPI_PRINT("SSP1 source clock = %d\n", hw_platform->working_clk);
}

struct ftssp010_spi_hw_platform *ftssp010_spi_get_hw_platform(u8 controller_id)
{
    struct ftssp010_spi_hw_platform *hw_platform = NULL;

    hw_platform = kzalloc(sizeof(struct ftssp010_spi_hw_platform), GFP_KERNEL);
    if (!hw_platform) {
        printk("Error => In %s: alloc fail.\n", __func__);
    }

    // currently, only SSP0 and SSP1 can use SPI driver
    if (controller_id == 0) {
        //spi_hw_init_0(hw_platform);
        printk("SSP0 be used for audio\n");
    } else if (controller_id == 1) {
        spi_1_pmu_init(hw_platform);
    }

    return hw_platform;
}
#endif // GPIO_CS

// control register 2 (offset 0x08)
// datasheet page 390
static inline void ftssp010_clear_fifo(void __iomem *base)
{
    u32 cr2 = inl(base + FTSSP010_OFFSET_CR2);

    cr2 |= FTSSP010_CR2_TXFCLR | FTSSP010_CR2_RXFCLR;
    outl(cr2, base + FTSSP010_OFFSET_CR2);
}

static void ftssp010_set_bits_per_word(void __iomem *base, int bpw)
{
    u32 cr1 = inl(base + FTSSP010_OFFSET_CR1);

    cr1 &= ~FTSSP010_CR1_SDL_MASK;

    if (unlikely(((bpw - 1) < 0) || ((bpw - 1) > 32))) {
        FTSSP010_SPI_PRINT("%s fails: bpw - 1 = %d\n", __func__, bpw - 1);
        return;
    } else {
        cr1 |= FTSSP010_CR1_SDL(bpw - 1);
    }

    outl(cr1, base + FTSSP010_OFFSET_CR1);
}

static inline unsigned int ftssp010_read_status(void __iomem *base)
{
    u32 data = inl(base + FTSSP010_OFFSET_STATUS);
    return data;
}

static inline int ftssp010_rxfifo_valid_entries(void __iomem *base)
{
    u32 data = ftssp010_read_status(base);
    return FTSSP010_STATUS_GET_RFVE(data);
}

/**
 * Note: due to PMU and IP clock limitation, speed_hz may be not just the same as real bclk, it is likely higher
 */
static inline int ftssp010_set_speed(struct ftssp010_spi *ftssp010_spi, u32 speed_hz)
{
    u32 scldiv = 0, cr1 = 0;   

    cr1 = inl(ftssp010_spi->base + FTSSP010_OFFSET_CR1);
    cr1 &= ~0xFFFF;
    // working clk now is ~85Mhz
    scldiv = ftssp010_spi->hw_platform->working_clk / 2;
#ifdef DEBUG_SSPC_EN
    printk(KERN_INFO "======start debug=======\n");
    printk(KERN_INFO "speed hz is : 0x%x %d\n", speed_hz, speed_hz);
    printk(KERN_INFO "before dodiv : scldiv = 0x%x : %d\n", scldiv, scldiv);
    printk(KERN_INFO "working clock is : 0x%x : %d\n",ftssp010_spi->hw_platform->working_clk, ftssp010_spi->hw_platform->working_clk);
#endif
    do_div(scldiv, speed_hz);
#ifdef DEBUG_SSPC_EN
    printk(KERN_INFO "after dodiv : scldiv = 0x%x : %d\n", scldiv, scldiv);
#endif   
    if(scldiv > 0xFFFF)
        printk(KERN_ERR "SSP speed too fast, can't be div to %dHz\n",speed_hz);

    //cr1 |= ((scldiv - 1)>>1);
    cr1 |= 0x0a;

#ifdef DEBUG_SSPC_EN
    printk(KERN_INFO "cr1 is : 0x%x: %d\n", cr1, (cr1 & 0xffff));
    printk(KERN_INFO "======end debug=======\n");
#endif
    outl(cr1, ftssp010_spi->base + FTSSP010_OFFSET_CR1);

    return 0;
}

static inline void ftssp010_cs_high(struct ftssp010_spi *ftssp010_spi, u8 cs)
{
#ifdef GPIO_CS
	switch (cs) {
	  case 0:
        gpio_direction_output(GPIO_pin_cs0, 1);
	    break;
	  case 1:
        gpio_direction_output(GPIO_pin_cs1, 1);
	    break;
	  case 2:
        gpio_direction_output(GPIO_pin_cs2, 1);
	    break;
	  case 3:
        gpio_direction_output(GPIO_pin_cs3, 1);
	    break;	    	    
	  default:
	    printk("Not support this cs value = %d\n", cs);
	    break;
	}
#endif 
}

static inline void ftssp010_cs_low(struct ftssp010_spi *ftssp010_spi, u8 cs)
{
#ifdef GPIO_CS
	switch (cs) {
	  case 0:
        gpio_direction_output(GPIO_pin_cs0, 0);
	    break;
	  case 1:
        gpio_direction_output(GPIO_pin_cs1, 0);
	    break;
	  case 2:
        gpio_direction_output(GPIO_pin_cs2, 0);
	    break;
	  case 3:
        gpio_direction_output(GPIO_pin_cs3, 0);
	    break;	    	    
	  default:
	    printk("Not support this cs value = %d\n", cs);
	    break;
	}
#endif    
}

static inline void ftssp010_enable(void __iomem *base)
{
    u32 cr2 = inl(base + FTSSP010_OFFSET_CR2);

    cr2 |= (FTSSP010_CR2_SSPEN | FTSSP010_CR2_TXDOE | FTSSP010_CR2_RXEN | FTSSP010_CR2_TXEN);
    outl(cr2, base + FTSSP010_OFFSET_CR2);
}

static inline void ftssp010_disable(void __iomem *base)
{
    u32 cr2 = inl(base + FTSSP010_OFFSET_CR2);

    cr2 &= ~(FTSSP010_CR2_SSPEN | FTSSP010_CR2_RXEN | FTSSP010_CR2_TXEN);
    outl(cr2, base + FTSSP010_OFFSET_CR2);
}

/** 
 * @brief set proper controller's state based on specific device
 * 
 * @param ftssp010_spi alias of controller
 * @param mode device's specific mode
 * 
 * @return always return 0 to indicate set up done
 */
static int ftssp010_spi_master_setup_mode(struct ftssp010_spi
                                          *ftssp010_spi, u8 mode)
{
    u32 cr0 = 0;
    
    if (mode & ~MODEBITS) {
        FTSSP010_SPI_PRINT("%s fails: some SPI mode not support currently\n", __func__);
        return -EINVAL;
    }

    cr0 |= FTSSP010_CR0_FFMT_SPI | FTSSP010_CR0_OPM_MASTER_STEREO;

    if (mode & SPI_CPOL) {
        cr0 |= FTSSP010_CR0_SCLKPO;
    }

    if (mode & SPI_CPHA) {
        cr0 |= FTSSP010_CR0_SCLKPH;
    }

    if (mode & SPI_LOOP) {
        cr0 |= FTSSP010_CR0_LBM;
    }

    if (mode & SPI_LSB_FIRST) {
        cr0 |= FTSSP010_CR0_LSB;
    }
    cr0 |= 0x3;
    outl(cr0, ftssp010_spi->base + FTSSP010_OFFSET_CR0);

    return 0;
}


/** 
 * @brief init specific spi divice's setting, this should be given by device. Or, we will give normal value.
 *        
 * @param spi specific device
 * 
 * @return return 0 when everything goes fine else return a negative to indicate error
 */
static int ftssp010_spi_master_setup(struct spi_device *spi)
{
    struct ftssp010_spi *ftssp010_spi = NULL;
    unsigned int bpw = spi->bits_per_word;
    int ret = 0;
    
    ftssp010_spi = spi_master_get_devdata(spi->master);

    if (spi->chip_select > spi->master->num_chipselect) {
        FTSSP010_SPI_PRINT
            ("%s fails: invalid chipselect %u (%u defined)\n",
             __func__, spi->chip_select, spi->master->num_chipselect);
        return -EINVAL;
    }

    /* check bits per word */
    if (bpw == 0) {
        FTSSP010_SPI_PRINT("In %s: bpw == 0, use 8 bits by default\n", __func__);
        bpw = 8;
    }

    if (bpw == 0 || bpw > 32) {
        FTSSP010_SPI_PRINT("%s fails: invalid bpw%u (1 to 32)\n", __func__, bpw);
        return -EINVAL;
    }

    spi->bits_per_word = bpw;
	//printk(" bpw = %d\n", bpw);
    /* check mode */
    spi->mode |= SPI_LSB_FIRST;    
    //printk("Setup device %s, chip select %d, mode = 0x%x, speed = %d\n", spi->modalias, spi->chip_select, spi->mode, spi->max_speed_hz);

    if ((ret = ftssp010_spi_master_setup_mode(ftssp010_spi, spi->mode)) < 0) {
        FTSSP010_SPI_PRINT("%s fails: ftssp010_spi_master_setup_mode not OK\n", __func__);
        return ret;
    }

    /* check speed */
    if (!spi->max_speed_hz) {
        FTSSP010_SPI_PRINT("%s fails: max speed not specified\n", __func__);
        return -EINVAL;
    }
   
    if (spi->max_speed_hz > ftssp010_spi->hw_platform->working_clk) {
        printk("Error => In %s: spi->max_speed_hz(%dHz) is out of current IP's capability(%d).\n", 
               __func__, spi->max_speed_hz, ftssp010_spi->hw_platform->working_clk);
    }
	
    ftssp010_set_bits_per_word(ftssp010_spi->base, bpw);

    /* init */
    mutex_lock(&cs_mutex);
    ftssp010_cs_high(ftssp010_spi, spi->chip_select);
    mutex_unlock(&cs_mutex);
    ftssp010_clear_fifo(ftssp010_spi->base);

    return 0;
}

static void ftssp010_spi_master_cleanup(struct spi_device *spi)
{
    if (!spi->controller_state) {
        return;
    }
}

static inline unsigned int ftssp010_read_feature(void __iomem *base)
{
    return inl(base + FTSSP010_OFFSET_FEATURE);
}


static inline int ftssp010_rxfifo_depth(void __iomem *base)
{
    return FTSSP010_FEATURE_RXFIFO_DEPTH(ftssp010_read_feature(base)) + 1;
}

/** 
 * @brief ISR for SPI controller get something wrong
 * 
 * @param irq spi's irq number
 * @param dev_id specific spi info
 * 
 * @return always return IRQ_HANDLED to indicate we got the irq
 */
static irqreturn_t ftssp010_spi_interrupt(int irq, void *dev_id)
{
    struct spi_master *master = dev_id;
    struct ftssp010_spi *ftssp010_spi = NULL;
    u32 isr = 0;

    ftssp010_spi = spi_master_get_devdata(master);

    isr = inl(ftssp010_spi->base + FTSSP010_OFFSET_ISR);

    if (isr & FTSSP010_ISR_RFOR) {
        outl(FTSSP010_ISR_RFOR, ftssp010_spi->base + FTSSP010_OFFSET_ISR);
        FTSSP010_SPI_PRINT("In %s: RX Overrun\n", __func__);
    }

    wake_up(&ftssp010_spi->waitq);

    return IRQ_HANDLED;
}

#if CONFIG_FTSPI010_USE_AHBDMA
static void ftspi010_dma_enable(void __iomem * base)
{
	unsigned int	tmp = inl(base + FTSSP010_OFFSET_ICR);

	//Tx
	//0x10 Transmit FIFO threshold=1
	//0x10 Transmit DMA request enable= 1
	tmp|= FTSSP010_ICR_TFTHOD(0x6);
	tmp |= FTSSP010_ICR_TFDMA;
	//Rx
	tmp|= FTSSP010_ICR_RFTHOD(0x6);
	tmp |= FTSSP010_ICR_RFDMA;
	
	outl(tmp, base + FTSSP010_OFFSET_ICR);
	
}

static int ftspi010_dma_wait(void)
{
    int rc = 0;

    rc = wait_event_timeout(spi010_queue, trigger_flag== 1, 5 * HZ);
    if (rc == 0) {
    	printk("spi010 dma queue wake up timeout signal arrived\n");
    	return -1;
    }
    trigger_flag = 0;
	
    return 0;
}

void ftspi010_dma_callback(void *param)
{
    trigger_flag = 1;
    wake_up(&spi010_queue);
    return;
}


static int ftspi010_setup_dma(struct ftssp010_spi *ctrl, size_t len, int direct)
{
    struct dma_slave_config *common;

    ctrl->dma_slave_config.id = -1;
    if(direct == DMA_DEV_TO_MEM)
        ctrl->dma_slave_config.handshake = SPI010_RxREQ; // receiving config
    else
        ctrl->dma_slave_config.handshake = SPI010_TxREQ; // transmiting config

    common = &ctrl->dma_slave_config.common;

    //ctrl->dma_slave_config.src_size = FTDMAC020_BURST_SZ_4;//64x4=256 //RichardLin(***)
    //ctrl->dma_slave_config.src_size = FTDMAC020_BURST_SZ_1;//pass
    if(len == 256)
        ctrl->dma_slave_config.src_size = FTDMAC020_BURST_SZ_16;
    else
        ctrl->dma_slave_config.src_size = FTDMAC020_BURST_SZ_8;//64x4=256 //RichardLin(***)


    if(direct == DMA_DEV_TO_MEM){
        ctrl->dma_slave_config.src_sel = AHBMASTER_R_SRC;
        ctrl->dma_slave_config.dst_sel = AHBMASTER_R_DST;
    }else{
        ctrl->dma_slave_config.src_sel = AHBMASTER_W_SRC;
        ctrl->dma_slave_config.dst_sel = AHBMASTER_W_DST;
    }
	
    if(direct == DMA_MEM_TO_DEV){
     	common->src_addr = ctrl->mem_dmaaddr;
    	common->dst_addr = ctrl->spi_dmaaddr;
        
        common->dst_addr_width = 1;
        common->src_addr_width = 1;
    }else{
     	common->src_addr = ctrl->spi_dmaaddr;
    	common->dst_addr = ctrl->mem_dmaaddr;

        common->dst_addr_width = 1;
        common->src_addr_width = 1;
    }

    /* SPI kernel maybe send len = 2011, so can't div 4 */
    //common->dst_addr_width 1;
    //common->src_addr_width = 1;

    common->direction = direct;

    return dmaengine_slave_config(ctrl->dma_chan, common);//step 2
}

static int spi010_dma_start(struct ftssp010_spi *ctrl, size_t len, int direct)
{
	int ret;

	enum dma_ctrl_flags flags;
	struct dma_async_tx_descriptor *desc;

	ret = ftspi010_setup_dma(ctrl, len, direct);
	if (ret)
	    return ret;

	flags = DMA_PREP_INTERRUPT | DMA_CTRL_ACK | DMA_COMPL_SKIP_SRC_UNMAP | DMA_COMPL_SKIP_DEST_UNMAP;

	desc = dmaengine_prep_slave_single(ctrl->dma_chan, (void *)ctrl->sg_dmabuf, len, direct, flags);//step 3

	if (!desc)
	    return ret;

	desc->callback = ftspi010_dma_callback;
	desc->callback_param = &ctrl;
	ctrl->cookie = dmaengine_submit(desc);	//step 4
	dma_async_issue_pending(ctrl->dma_chan);//step 5

	return 0;
}
#endif

// *** keep original function
// ====================================================
// =============end define internal function===========
// ====================================================

// define function to handle normal tranfer use PIO
static int _ftssp010_spi_pio_transfer(struct ftssp010_spi *ftssp010_spi,
                                       struct spi_device *spi, struct spi_transfer *t, int wsize)
{
    unsigned long flags;
    int len = t->len;
    u8 *tx_buf = (void *)t->tx_buf;
    u32 *rx_buf = t->rx_buf;
	u32 tmp=0;
	//int tx_len =  t->len;
	int idx=0, count=0;
    
    // proceed clean tx & rx fifo buff before every transaction 
	ftssp010_clear_fifo(ftssp010_spi->base);

    // === sequence indicator (MSB) === //
    tmp= inl(ftssp010_spi->base+ 0x00);

#if (MSB_SPI)
    tmp &= ~FTSSP010_CR0_LSB;
#else
    tmp |= FTSSP010_CR0_LSB;
#endif
    // === end sequence indicator=== //

    // === SPI set mode=== //
    // SPI mode setting
    tmp |= FTSSP010_CR0_SCLKPO;  // Clock poll
    tmp &= ~FTSSP010_CR0_SCLKPH; // Clock phase

    outl(tmp, ftssp010_spi->base+ 0x00);
    // === end SPI set mode ===//

    while (len > 0)
    {
		spin_lock_irqsave(&ftssp010_spi->lock, flags);
#if (MSB_SPI)
        //printk(KERN_INFO "data of tx_buf = 0x%x\n",*(const u32 *)tx_buf);
		tmp=0x0;
		idx = count * 4;
		tmp |= (*(tx_buf + idx) << 24);
		//printk("===>tx1 is 0x%x\n",tmp);
		tmp |= (*(tx_buf+ idx + 1) << 16);
		//printk("===>tx2 is 0x%x\n",tmp);
		tmp |= (*(tx_buf+ idx + 2) << 8);
		//printk("===>tx3 is 0x%x\n",tmp);
		tmp |= (*(tx_buf+ idx + 3));
        //printk(KERN_INFO "data of tmp = 0x%x\n",(u32)tmp);
#else
        tmp = *(const u32 *)(tx_buf + count);
#endif

		udelay(2);
        // proceed send 4 byte 
		outl(tmp, ftssp010_spi->base+ 0x18);
        // send done 
		while(true)
        {
            // wait to get back data
			tmp= inl(ftssp010_spi->base+ 0x0C);
			tmp&= 0x3F0;
			if(tmp!= 0){
                // have data to read
				break;
			}
		}
        // put data to rx buff
		*rx_buf = inl(ftssp010_spi->base+ 0x18);
		len -= 4;
		count++;

        // proceed clean tx & rx fifo buff after every transaction 
		ftssp010_clear_fifo(ftssp010_spi->base);

		spin_unlock_irqrestore(&ftssp010_spi->lock, flags);
    }
    
    return t->len - len;
}

// define function to handle read data use DMA
static int _ftssp010_spi_DMA_read(struct ftssp010_spi *ftssp010_spi,
                                       struct spi_device *spi, struct spi_transfer *t, int rsize)
{
    u8 *rx_buf = t->rx_buf;
    u32 tmp=0;
    int access_byte=t->len, ret;

    ftssp010_clear_fifo(ftssp010_spi->base);
    tmp= inl(ftssp010_spi->base+ 0x00);
    tmp &= ~FTSSP010_CR0_LSB;

    // tmp |= FTSSP010_CR0_FFMT_SPI | FTSSP010_CR0_OPM_MASTER_STEREO;
    // SPI mode setting
    // tmp |= FTSSP010_CR0_SPIFSPO;
    tmp |= FTSSP010_CR0_SCLKPO;  // Cloock poll
    tmp &= ~FTSSP010_CR0_SCLKPH; // Clock phase

    outl(tmp, ftssp010_spi->base+ 0x00);

#if MISO_DMA_EN
    ftssp010_set_bits_per_word(ftssp010_spi->base, 8);
    // DMA MISO
    // Receive function enable
    tmp= inl(ftssp010_spi->base+ 0x08);
    tmp&= ~(0x80);
    tmp|= 0x80;
    outl(tmp, ftssp010_spi->base+ 0x08);

    // Transmit function is disabled.
    tmp= inl(ftssp010_spi->base+ 0x08);
    tmp&= ~(0x100);
    tmp|= 0x000;
    outl(tmp, ftssp010_spi->base+ 0x08);

    // disable transfer DMA request
    // bit 4  -> 1
    // bit 5  -> 0
    // bit[11:7] : 16
    tmp = inl(ftssp010_spi->base+ 0x10);
    tmp&= 0xFFFFFFDF;
    tmp|= 0x00000710; // orig
    //tmp|= 0x00000810;
    outl(tmp, ftssp010_spi->base+ 0x10); 


    udelay(5);

    memset(ftssp010_spi->mem_dmabuf_2, '\0', access_byte);

    // wait until rxfifo is valid
    while (true)
    {
        tmp=ftssp010_rxfifo_valid_entries(ftssp010_spi->base);
        if(tmp != 0x0)
        {
            printk(KERN_INFO "rxfifo valid\n");
            break;
        }
    }

    ret = spi010_dma_start(ftssp010_spi, access_byte , DMA_DEV_TO_MEM);

    if (ret < 0) {
        printk("spi010 dma read fail\n");
        goto out;
    }

    ftspi010_dma_wait();

    memcpy(rx_buf, ftssp010_spi->mem_dmabuf_2, access_byte);
    memset(ftssp010_spi->mem_dmabuf_2, '\0', sizeof(ftssp010_spi->mem_dmabuf_2));
    ftssp010_clear_fifo(ftssp010_spi->base);
    //wait for next fifo MISO
    for(;;)
    {
        tmp= inl(ftssp010_spi->base+ 0x0C);
        tmp&= 0x3F0;
        if(tmp!= 0){
            break;
        }
    }
#endif

#if CONFIG_FTSPI010_USE_AHBDMA
out:
#endif 
    return access_byte;
}

// define function to handle write data use DMA
/** 
 * @brief real spi transfer work is done here
 * 
 * @param ftssp010_spi alias of controller
 * @param spi specific spi device
 * @param t one spi transfer
 * @param wsize bytes per word, one byte is 8 bits
 * 
 * @return return the number of transfered words 
 */
static int _ftssp010_spi_DMA_transfer(struct ftssp010_spi *ftssp010_spi,
                                       struct spi_device *spi, struct spi_transfer *t, int wsize)
{
    u8 *tx_buf = (void *)t->tx_buf;
    u32 tmp=0;
    int tx_len =  t->len;
    int count=0;
    int access_byte=128, ret;
    // check input
    if(!ftssp010_spi || !spi || !t)
    {
        printk(KERN_ERR "invalid input\n");
        goto __FAIL;
    }
    //printk("tx_len = %d, tx[0]= 0x%x, tx[1]= 0x%x,tx[2]= 0x%x, tx[3]= 0x%x\n", tx_len, *tx_buf, *(tx_buf+1),*(tx_buf+2), *(tx_buf+3));

    // === sequence indicator (MSB) === 
    tmp= inl(ftssp010_spi->base+ 0x00);
#if (1)
    tmp &= ~FTSSP010_CR0_LSB;
#else
    tmp |= FTSSP010_CR0_LSB;
#endif

    // === set SPI mode==== 
    tmp |= FTSSP010_CR0_SCLKPO;  // Cloock poll
    tmp &= ~FTSSP010_CR0_SCLKPH; // Clock phase
    outl(tmp, ftssp010_spi->base+ 0x00);
    // === end set mdoe & sequence indicator ===

#if MOSI_DMA_EN
	ftssp010_set_bits_per_word(ftssp010_spi->base, 8);
	// DMA MOSI
	// Receive function is disabled.
	tmp= inl(ftssp010_spi->base+ 0x08);
	tmp&= ~(0x80);
	tmp|= 0x00;
	outl(tmp, ftssp010_spi->base+ 0x08);
    
    // disable receive DMA request
    // bit 4  -> 0
    // bit 5  -> 1
    // bit[11:7] : 16
	tmp = inl(ftssp010_spi->base+ 0x10);
	tmp&= 0xFFFFFFEF;
	tmp|= 0x00000020; // orig
    //tmp|= 0x00000820;
	outl(tmp, ftssp010_spi->base+ 0x10);

	ftssp010_clear_fifo(ftssp010_spi->base);

	while(tx_len >0)
    {	
	    // memset(ftssp010_spi->mem_dmabuf_2, '\0', access_byte);

	    if(tx_len < 128)
	    {
		    memset(ftssp010_spi->mem_dmabuf_2, '\0', access_byte);
		    memcpy(ftssp010_spi->mem_dmabuf_2, tx_buf + count*128, tx_len);
	    }
	    else
        {
		    memcpy(ftssp010_spi->mem_dmabuf_2, tx_buf + count*128, access_byte);
        }
        //printk(KERN_INFO "len : %d count = %d  access_byte = %d\n",tx_len,count,access_byte);

	    ret = spi010_dma_start(ftssp010_spi, access_byte, DMA_MEM_TO_DEV);
	    if (ret < 0) 
        {
		    printk(KERN_ERR "spi010 dma write fail\n");
		    goto out;
	    }
	    ftspi010_dma_wait();

	    ftssp010_clear_fifo(ftssp010_spi->base);
	    count++;
	    tx_len -= 128;
	} // end while

#endif // MOSI_DMA_EN
#if CONFIG_FTSPI010_USE_AHBDMA
out:
#endif

    return t->len;
__FAIL:
    return 0;
}

// handle message function 
static int ftssp010_spi_handle_message(struct ftssp010_spi *ftssp010_spi, struct spi_device *spi, struct spi_transfer *t)
{
	unsigned int bpw = 0;
	unsigned int wsize = 0;     /* word size */
#ifdef DEBUG_TIME
    struct timeval tv_b;
    struct timeval tv_a;
#endif
	int ret = 0;

	bpw = t->bits_per_word ? t->bits_per_word : spi->bits_per_word;
	if (bpw == 0 || bpw > 32)
	{
		return -EINVAL;
	}
#ifdef DEBUG_SSPC_EN
    printk(KERN_INFO "step 3\n");
    printk(KERN_INFO "t->len = %d\n",t->len);
    printk(KERN_INFO "t->bpw = %d\n",t->bits_per_word);
    printk(KERN_INFO "t->speed_hz = %d\n",t->speed_hz);
#endif
	// do set bpw
	ftssp010_set_bits_per_word(ftssp010_spi->base, bpw);
#if (0)
    tmp= inl(ftssp010_spi->base+ 0x04);
    tmp&= ~(0x7F0003);
    tmp|= 0x1F0001;// Serial Data Length=SDL[22:16]+1, SDL=31
    outl(tmp, ftssp010_spi->base+ 0x04);
#endif
	// check speed
	if ((t->speed_hz == 0) || (t->speed_hz > spi->max_speed_hz))
	{
		//use default device's clk
		t->speed_hz = spi->max_speed_hz;
	}
	// do set speed
	ftssp010_set_speed(ftssp010_spi, t->speed_hz);

	mutex_lock(&cs_mutex);
	ftssp010_cs_low(ftssp010_spi,0);
#if CONFIG_FTSPI010_USE_AHBDMA
	if(t->len <= 8)
	{
#endif
		ret = _ftssp010_spi_pio_transfer(ftssp010_spi, spi, t, wsize); // transfer use PIO
#if CONFIG_FTSPI010_USE_AHBDMA
	}
	else if(t->tx_buf == NULL) // only require reading
	{
#ifdef DEBUG_TIME
        do_gettimeofday(&tv_b);
#endif
		ret = _ftssp010_spi_DMA_read(ftssp010_spi, spi, t, wsize); // transfer use DMA -> just care only rx buff
        
#ifdef DEBUG_TIME
        do_gettimeofday(&tv_a);
        printk(KERN_INFO "excute read data time: %ld ms\n",(tv_a.tv_sec*1000*1000 + tv_a.tv_usec) - (tv_b.tv_sec*1000*1000 + tv_b.tv_usec));
#endif
	}
	else
	{
#ifdef DEBUG_TIME
        do_gettimeofday(&tv_b);
#endif
		ret = _ftssp010_spi_DMA_transfer(ftssp010_spi, spi, t, wsize); // transfer use DMA 

#ifdef DEBUG_TIME
        do_gettimeofday(&tv_a);
        printk(KERN_INFO "excute transfer data time: %ld\n",(tv_a.tv_sec*1000*1000 + tv_a.tv_usec) - (tv_b.tv_sec*1000*1000 + tv_b.tv_usec));
#endif
	}
#endif
	ftssp010_cs_high(ftssp010_spi,0);
	mutex_unlock(&cs_mutex);

	if(t->delay_usecs)
	{
		udelay(t->delay_usecs);
	}
	return ret;
}


/*
 * @brief work until all spi messages requested by a specific device done
 *
 * @param work one requested work to be done
 */
static void ftssp010_spi_work(struct work_struct *work)
{
	struct ftssp010_spi *ftssp010_spi = NULL;
	struct spi_device *spi = NULL;
	struct spi_transfer *t = NULL;
	unsigned long flags = 0;
	int ret = 0;
	ftssp010_spi = container_of(work, struct ftssp010_spi, work);
	spin_lock_irqsave(&ftssp010_spi->lock, flags);
	// query from list
    // 0 -> list not empty 
    // 1 -> lis is empty
	while(!list_empty(&ftssp010_spi->message_queue))
	{
		struct spi_message *m = NULL;
		m = container_of(ftssp010_spi->message_queue.next, struct spi_message, queue);
		if(!m)
			continue;
		list_del_init(&m->queue);
		spin_unlock_irqrestore(&ftssp010_spi->lock, flags);
		// handle message
		// ===start===
		spi = m->spi;
		m->status = 0;
		m->actual_length = 0;
        // @TODO: consider to remove this one <-- daniel commented
		ftssp010_enable(ftssp010_spi->base);
		list_for_each_entry(t, &m->transfers, transfer_list)
		{
#ifdef DEBUG_SSPC_EN
            printk(KERN_INFO "step 2:\n");
            printk(KERN_INFO "t->len = %d\n",t->len);
            printk(KERN_INFO "t->bpw = %d\n",t->bits_per_word);
            printk(KERN_INFO "t->speed_hz = %d\n",t->speed_hz);
#endif
			// get pkg message from queue then handle it
			ret = ftssp010_spi_handle_message(ftssp010_spi, spi, t);
			if(ret < 0)
			{
				m->status = ret;
				break;
			}
			m->actual_length += ret;
#ifdef DEBUG_SSPC_EN
            printk(KERN_INFO "m->actual_length = %d   : ret = %d\n",m->actual_length, ret);
#endif
		}
        // @TODO : consider to remove this one <-- daniel commented
		mutex_lock(&cs_mutex);
		ftssp010_cs_high(ftssp010_spi, 0);
		mutex_unlock(&cs_mutex);

		m->complete(m->context);
		// ===end===
		spin_lock_irqsave(&ftssp010_spi->lock, flags);
	} // end while
	spin_unlock_irqrestore(&ftssp010_spi->lock, flags);
}

/*
 * send a complete spi message 
 */

static int ftssp010_spi_master_transfer(struct spi_device *spi, struct spi_message *m)
{
	struct ftssp010_spi *ftssp010_spi = NULL;
	struct spi_transfer *t = NULL;
	unsigned long flags = 0;
    // check input valid
    if(!spi || !m)
    {
        printk(KERN_ERR "ftssp010_spi_master_transfer : invalid input\n");
        return -EINVAL;
    }
    if(unlikely(list_empty(&(m->transfers))))
    {
        return -EINVAL;
    }
	ftssp010_spi = spi_master_get_devdata(spi->master);
	if(!ftssp010_spi)
	{
		printk(KERN_ERR "ftssp010_spi is null\n");
		return -EINVAL;
	}
	list_for_each_entry(t, &(m->transfers), transfer_list)
	{
		if((t->len) && !(t->rx_buf || t->tx_buf))
		{
			printk(KERN_ERR "missing rx or tx buffer\n");
			return -EINVAL;
		}
	}
#ifdef DEBUG_SSPC_EN
    printk(KERN_INFO "step 1:\n");
    printk(KERN_INFO "t->len = %d\n",t->len);
    printk(KERN_INFO "t->bpw = %d\n",t->bits_per_word);
    printk(KERN_INFO "t->speed_hz = %d\n",t->speed_hz);
#endif

	m->status = -EINPROGRESS;
	m->actual_length = 0;

	spin_lock_irqsave(&ftssp010_spi->lock, flags);

	// add data to queue
	// struct list_head message_queue;
	// add new one into message_queue
	list_add_tail(&m->queue, &ftssp010_spi->message_queue);
	// send work to work queue
	queue_work(ftssp010_spi->workqueue, &ftssp010_spi->work);

	spin_unlock_irqrestore(&ftssp010_spi->lock, flags);
	if (unlikely(list_empty(&m->transfers)))
	{
		return -EINVAL;
	}
	return 0;
}


// internal function for controller device
static int ssp_spi_probe(struct platform_device *pdev)
{
	// TODO: probe fucntion
	// will be called once maching device happen
    printk(KERN_INFO "===PROBE SPI CONTROLLER DRIVER ===\n");
	struct resource *res = NULL;
	struct spi_master *master = NULL;
	// for private data
	struct ftssp010_spi *ftssp010_spi = NULL;

    struct ftssp010_spi_hw_platform *spi_hw_platform = NULL; 

	void __iomem *base = NULL;
	int irq = 0;
	int ret = -1;
    
    // setup to use CS with GPIO 
    spi_hw_platform = ftssp010_spi_get_hw_platform(pdev->id);
    if (unlikely(spi_hw_platform == NULL)) 
    {
        printk(KERN_ERR "Error => %s: SSP(%d) spi_hw_platform is NULL.\n", __func__, pdev->id);
    }

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(!res)
	{
		printk(KERN_ERR "get resource failed\n");
		return -ENXIO;
	}
	
	// get device irq
	irq = platform_get_irq(pdev, 0);
	if(irq < 0)
	{
		printk(KERN_ERR "unable to get irq\n");
		return irq;
	}
	
	// setup spi core . call spi_alloc_master()
	// NOTE1
	master = spi_alloc_master(&(pdev->dev), sizeof(struct ftssp010_spi));
	if(!master)
	{
		printk(KERN_ERR "spi_alloc_master failed\n");
		goto __ERR_DEALLOC;
	}

#ifdef GPIO_CS
	if(gpio_request(GPIO_pin_cs0, "gpio_cs0") < 0) 
	{
		printk(KERN_ERR "gpio#%d request failed!\n", GPIO_pin_cs0);
		ret = -1;
		goto __ERR_DEALLOC;
	}
#endif

	// do setup
	master->bus_num = pdev->id;
	master->mode_bits = MODEBITS;
	master->num_chipselect = 4; // can adjust
	// setup function poiter
	master->setup = ftssp010_spi_master_setup;
	master->transfer = ftssp010_spi_master_transfer;
	master->cleanup = ftssp010_spi_master_cleanup;
	// save it : 
	// dev_set_drvdata(&pdev->dev, data);
	platform_set_drvdata(pdev, master);	

	// why return ftssp010_spi : see NOTE1
	ftssp010_spi = spi_master_get_devdata(master);
	if(!ftssp010_spi)
	{
		printk(KERN_ERR "spi_master_get_devdata failed\n");
		goto __ERR_DEALLOC;
	}
	// setup master private data
	spin_lock_init(&ftssp010_spi->lock);
	INIT_LIST_HEAD(&ftssp010_spi->message_queue);
    // create work item
	INIT_WORK(&(ftssp010_spi->work), ftssp010_spi_work);

	// init waitqueue_head : this one work for transfer msg
	init_waitqueue_head(&ftssp010_spi->waitq);
	// remap phyadd of ssp to viradd
	base = ioremap_nocache(res->start, (res->end - res->start +1));
	if(!base)
	{
		printk(KERN_ERR "ioremap_nocache mem of ssp failed\n");
		ret = -ENOMEM;
		goto __ERR_DEALLOC;
	}

	// handle error for ssp controller
	ret = request_irq(irq, ftssp010_spi_interrupt, 0, dev_name(&(pdev->dev)), master);
	if(ret)
	{
		printk(KERN_ERR "request_irq failed\n");
		goto __ERR_UNMAP;
	}

	ftssp010_spi->hw_platform = spi_hw_platform; 
	ftssp010_spi->irq = irq;
	ftssp010_spi->pbase = res->start;
	ftssp010_spi->base = base;
	ftssp010_spi->master = master;
	ftssp010_spi->rxfifo_depth = ftssp010_rxfifo_depth(base);
	
    // create a workqueue
	ftssp010_spi->workqueue = create_singlethread_workqueue(dev_name(&(pdev->dev)));
	if(!ftssp010_spi->workqueue)
	{
		printk(KERN_ERR "create_singlethread_workqueue failed\n");
		goto __ERR_FREE_IRQ;
	}

	// Initialize the hardware 
	outl(FTSSP010_ICR_RFTHOD(1), base + FTSSP010_OFFSET_ICR);

	ret = spi_register_master(master);
	if(ret)
	{
		printk(KERN_ERR "register master failed\n");
		goto __ERR_DESTROY_WORKQUEUE;
	}

#if CONFIG_FTSPI010_USE_AHBDMA
	// do configure for DMA TRANSFER
	
	ftssp010_spi->spi_dmaaddr = res->start + 0x18; // this one point to head of data register of ssp controller
	ftspi010_dma_enable(base);
	dma_cap_set(DMA_SLAVE, ftssp010_spi->cap_mask);
	{
		struct ftdmac020_dma_slave slave;
		memset(&slave, 0, sizeof(slave));
		// ftdmac020_chan_filter : provide by DMAC driver
		// refer to https://www.kernel.org/doc/Documentation/dmaengine/client.txt
		// DMAC as DMA master
		// SSPC as DMA slave
		ftssp010_spi->dma_chan = dma_request_channel(ftssp010_spi->cap_mask, ftdmac020_chan_filter, (void *)&slave); //step 1
	}
	if(!ftssp010_spi->dma_chan)
	{
		printk(KERN_ERR "request dma chanel failed\n");
		ret = -ENODEV;
		goto __OUT_FREE_CHAN;
	}
	printk(KERN_ERR "SSP get DMA chan : %d\n",ftssp010_spi->dma_chan->chan_id);

	// allocate mem for DMA operation
	// we will get two output 
	// 1st : return from function : addr use by cpu
	// 2nd : 3rd parameter : -> use for dma operation
	ftssp010_spi->mem_dmabuf_2 = dma_alloc_coherent(&(pdev->dev), FTSPI010_DMA_BUF_SIZE, &ftssp010_spi->mem_dmaaddr, GFP_KERNEL);
	if(!ftssp010_spi->mem_dmabuf_2)
	{
		printk(KERN_ERR "allocate mem for DMA operation failed\n");
		ret = -ENOMEM;
		goto __OUT_FREE_DMA;
	}
	ftssp010_spi->sg_dmabuf = dma_to_virt(&pdev->dev, ftssp010_spi->mem_dmaaddr);
	printk(KERN_INFO "sg mem pa = 0x%x, va = 0x%x mem_dmabuf_2 = 0x%x\n", (u32)ftssp010_spi->mem_dmaaddr, (u32)ftssp010_spi->sg_dmabuf, (u32)ftssp010_spi->mem_dmabuf_2);
	// init waitqueue_head : this one work in this driver
	init_waitqueue_head(&spi010_queue);
#endif

	// reach here mean do probe successfully
	return 0;

	// handle exit 
__ERR_DESTROY_WORKQUEUE:
#if CONFIG_FTSPI010_USE_AHBDMA
__OUT_FREE_DMA:
	if(ftssp010_spi->dma_chan)
		dma_release_channel(ftssp010_spi->dma_chan);
__OUT_FREE_CHAN:
#endif
	destroy_workqueue(ftssp010_spi->workqueue);
__ERR_FREE_IRQ:
	free_irq(irq, master);
__ERR_UNMAP:
	if(base)
		iounmap(base);
__ERR_DEALLOC:
	spi_master_put(master);
	return ret;
}// END PROBE FUNCTION

static int __devexit ssp_spi_remove(struct platform_device *pdev)
{
	// TODO : remove function
	struct spi_master *master = NULL;
	struct ftssp010_spi *ftssp010_spi = NULL;
	struct spi_message *m = NULL;
	// get master 	
	master = platform_get_drvdata(pdev);
	ftssp010_spi = spi_master_get_devdata(master);
	// terminate remaining queued transfer
	list_for_each_entry(m, &ftssp010_spi->message_queue, queue)
	{
		m->status = -ESHUTDOWN;
		m->complete(m->context);
	}
	destroy_workqueue(ftssp010_spi->workqueue);
	free_irq(ftssp010_spi->irq, master);
	iounmap(ftssp010_spi->base);
	spi_unregister_master(master);
#if CONFIG_FTSPI010_USE_AHBDMA

	if(ftssp010_spi->mem_dmabuf_2)
		dma_free_coherent(&pdev->dev, FTSPI010_DMA_BUF_SIZE, ftssp010_spi->mem_dmabuf_2, ftssp010_spi->mem_dmaaddr);
	if(ftssp010_spi->dma_chan)
		dma_release_channel(ftssp010_spi->dma_chan);

#endif
	return 0;
} // END REMOVE FUNCTION

// define remove function

// for define struct platform driver
static struct platform_driver ssp_spi_driver = {
	.probe  = ssp_spi_probe,
	.remove = __devexit_p(ssp_spi_remove),
	//.suspend= ssp_spi_suspend,
	//.resume = ssp_spi_resume,
	.driver = {
	       		.name = DRV_NAME, // need have the same name to platform device to maching process happen	
			    .owner= THIS_MODULE,
	          },
};

static void ssp_device_release(struct device *dev)
{
	printk(KERN_INFO "entering %s",__func__);
}
// define struct resource
// arch/arm/mach-GM/include/mach/platform-GM8136/platform_io.h
static struct resource ssp_1_resource[] = {
	{
		.start 	= 	SSP_FTSSP010_1_PA_BASE, // base register address of ssp device
		.end 	= 	SSP_FTSSP010_1_PA_LIMIT,// end register address of ssp device
		.flags 	= 	IORESOURCE_MEM, // inform that this is related to memory
	},
	{
		.start 	= 	SSP_FTSSP010_1_IRQ,
		.end 	= 	SSP_FTSSP010_1_IRQ,
		.flags 	= 	IORESOURCE_IRQ, // inform that this is related to IRQ
	}
};

// for define struct platform device
static struct platform_device ssp_spi_device = {
	.name 		    = 	DRV_NAME,
	.id 		    = 	1, // bus 1
	.num_resources 	= 	ARRAY_SIZE(ssp_1_resource),
	.resource 	    = 	ssp_1_resource,
	.dev 		    = 	{
		.coherent_dma_mask 	= 	DMA_BIT_MASK(32),
		.release 		= 	ssp_device_release,
	},
};

// for struct spi_board_info
static struct spi_board_info spi_devs_info[] __initdata = {
	{
		.modalias 	    = 	"spidev", // refer to spi protocol driver
		.max_speed_hz 	= 	10000000, // 10Mhz
		.bus_num 	    = 	1, // use controller 1: probaly have many controller 
		.chip_select 	= 	0, // use chip select 0
		.mode 		    = 	SPI_MODE_3,
	},
};

module_platform_device_driver(ssp_spi_driver, ssp_spi_device, spi_devs_info);
MODULE_AUTHOR("Daniel Nguyen<daniel.nguyen0105@gmail.com>");
MODULE_DESCRIPTION("SSP SPI controller driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
