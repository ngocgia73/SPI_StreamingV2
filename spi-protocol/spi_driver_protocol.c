#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <asm/uaccess.h>

#include <linux/kthread.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/kfifo.h>
#include <mach/irqs.h>
#include <asm/io.h>
#include <linux/version.h>
#include <linux/cdev.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/irq.h>
#include <asm/gpio.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/spi/spi_driver_protocol.h>

#include <linux/proc_fs.h>
#include <linux/seq_file.h>
/*========================================================================*/
/*=======================START DEFINE MACRO===============================*/
/*========================================================================*/

//#define DEBUG_SPI_P
//#define USE_IRQ
//#define WAIT_TO_TRANSFER

#define DEV_NAME 	"spidev"

#define SIZE_BUFF   (4*1024)

#define COMMAND_LENGTH          4

#define GPIO_INT_PIN            13
// #define GPIO_RESET_PIN          35   // GPIO1[3]  // ES1
#define GPIO_RESET_PIN          58      // GPIO1[26] // ES2
#define GPIO_CE_PIN             59      // GPIO1[27]

 #define MS_TO_TICKS(x)         ((x)*HZ/1000 + 1)


#define VIDEO_QUEUE_SIZE        (4096*16)
#define AUDIO_QUEUE_SIZE        (4096*2)
#define TALKBACK_QUEUE_SIZE     (4096*2)


#ifdef REVERSING_RECEIVE_ORDER_PLATFORM // AIT
#define IO_COMMAND_BYTE_MASK  0x000000FF
#define IO_ADDRESS_BYTE_MASK  0x0000FF00
#define DEVICE_ADDRESS_MASK   0x00FF0000
#define COMMAND_BYTE_MASK     0xFF000000
#define UART_READ_BIT         0x00000020
#define VIDEO_FULL_MASK       0x01000000
#define AUDIO_FULL_MASK       0x02000000
#define TALKBACK_FULL_MASK    0x04000000
#define J_FULL_MASK           0x01000000
#define U_CTS_MASK            0x40000000
#else // Nuvoton, GM, Anyka
#define IO_COMMAND_BYTE_MASK  0xFF000000
#define IO_ADDRESS_BYTE_MASK  0x00FF0000
#define DEVICE_ADDRESS_MASK   0x0000FF00
#define COMMAND_BYTE_MASK     0x000000FF
#define UART_READ_BIT         0x20000000
#define VIDEO_FULL_MASK       0x00000001
#define AUDIO_FULL_MASK       0x00000002
#define TALKBACK_FULL_MASK    0x00000004
#define J_FULL_MASK           0x00000001
#define U_CTS_MASK            0x00000040
#endif


#define BLE_FULL_MASK         TALKBACK_FULL_MASK

/*========================================================================*/
/*=========================END DEFINE MACRO ==============================*/
/*========================================================================*/

/*========================================================================*/
/*========================START DEFINE GLOBAL VARIABLE====================*/
/*========================================================================*/

static dev_t dev_num;
static struct class *cls = NULL;
static struct device *dev = NULL;
static struct cdev *my_cdev = NULL;
// names used to display in /sys/class
const char * class_name = DEV_NAME;

// init list head
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_mutex_lock);

// for thread maintask
static struct task_struct *maintask = NULL;

static unsigned int ble_buff_max_size = 4096;

#ifdef USE_IRQ
static DECLARE_COMPLETION(spi_done);
#endif

module_param(ble_buff_max_size, uint, S_IRUGO);
MODULE_PARM_DESC(ble_buff_max_size, "data bytes in biggest supported SPI message");
/*========================================================================*/
/*============================END DEFINE GLOBAL VARIABLE==================*/
/*========================================================================*/



/*========================================================================*/
/*========================START DEFINE INTERNAL FUNCTION==================*/
/*========================================================================*/

#ifdef USE_IRQ
// define irq function handle
static irqreturn_t gpio_irq_handle(int irq, void *param)
{
    printk(KERN_DEBUG "irq trigger\n");
    gpio_interrupt_clear(irq);
    complete(&spi_done);
    return IRQ_HANDLED;
}
#endif

static int socspi_gpio_init(void)
{
    int retval = 0;
#ifdef USE_IRQ
    int irq;
#endif
#if 0
    struct gpio_interrupt_mode mode = {
        .trigger_method     =   GPIO_INT_TRIGGER_EDGE,
        .trigger_method     =   GPIO_INT_SINGLE_EDGE,
        .trigger_rise_neg   =   GPIO_INT_FALLING
    };
#endif
    // hardreset pin
#if 0
    retval = gpio_request(GPIO_RESET_PIN, "socspi_reset");
    if(retval < 0)
    {
        printk(KERN_ERR "gpio_request reset pin failed ret = %d\n",retval);
        //return retval;
    }
#endif
    gpio_direction_output(GPIO_RESET_PIN, 1);
    
    // chip select 
#if 0
    retval = gpio_request(GPIO_CE_PIN, "socspi_ce");
    if(retval < 0)
    {
        printk(KERN_ERR "gpio_request ce pin failed ret = %d\n",retval);
        //return retval;
    }
#endif
    gpio_direction_input(GPIO_CE_PIN);

    // interrupt pin    
#if 0
    retval = gpio_request(GPIO_INT_PIN, "socspi_irq");
    if(retval < 0)
    {
        printk(KERN_ERR "gpio_request irq pin failed ret = %d\n",retval);
        //return retval;
    }
#endif

#ifdef USE_IRQ
    gpio_direction_input(GPIO_INT_PIN);
    irq = gpio_to_irq(GPIO_INT_PIN);
    gpio_interrupt_enable(irq);
#if 0
    gpio_interrupt_setup(irq, &mode);
#endif

    // IRQ_TYPE_LEVEL_LOW,
    // IRQ_TYPE_EDGE_FALLING,
    retval = request_irq(irq, gpio_irq_handle, IRQ_TYPE_LEVEL_LOW, "socspi_irq", NULL);
    if(retval)
    {
        printk(KERN_ERR "request_irq  failed ret = %d\n",retval);
        return -EFAULT;
    }
#endif
    return retval;
}

static void socspi_gpio_deinit(void)
{
    //int irq;
    //irq = gpio_to_irq(GPIO_INT_PIN);

    //free_irq(irq, NULL);
    gpio_free(GPIO_INT_PIN);
    gpio_free(GPIO_RESET_PIN);
    gpio_free(GPIO_CE_PIN);
}

#ifdef WAIT_TO_TRANSFER
static inline bool socspi_wait_to_transfer(void)
{
    //long timeout = 0;
    if(__gpio_get_value(GPIO_INT_PIN) == 0)
    {
        printk(KERN_DEBUG "interrupt pin is at low level\n");
        return true;
    }
#ifdef USE_IRQ
    timeout = wait_for_completion_killable_timeout(&spi_done, MS_TO_TICKS(200));
    if(timeout == 0)
    {
        printk(KERN_DEBUG "wait for int pin at low level timeout\n");
    }
    else if(timeout == -ERESTARTSYS)
    {
        printk(KERN_DEBUG "SPI interrupted\n");
    }
#endif
    return false; 
}
#endif

// this function will be called when transfer completed

/*
 * We can't use the standard synchronous wrappers for file I/O; we
 * need to protect against async removal of the underlying spi_device.
 */
static void socspi_complete(void *arg)
{
    complete(arg);
}

// define socspi_send_command() func
// return negative if error otherwise return the number of byte sent
static int socspi_send_command(struct socspi_data *p_socspi,
                               struct spi_message *p_msg,
                               char *p_tx,
                               int len,
                               void *p_rx,
                               bool is_user_cmd)
{
    int retval = -1;
    int retry = 3;
    //printk(KERN_DEBUG "socspi_send_command: p_socspi = 0x%x\n",(unsigned int)p_socspi);
    struct spi_transfer transfer = {
        .cs_change = 1,
        .bits_per_word = p_socspi->spi->bits_per_word,
        .delay_usecs = 0,
        .speed_hz = p_socspi->spi->max_speed_hz,
    };
#ifdef WAIT_TO_TRANSFER
    if( !socspi_wait_to_transfer() )
        return -EFAULT;
#endif

__SEND_COMMAND_LOOP:
    // config for spi_transfer
    transfer.len = len;
    transfer.tx_buf = p_tx;
    transfer.rx_buf = p_rx;
#ifdef DEBUG_SPI_P 
    //if(p_tx)
    //    printk(KERN_DEBUG "CMD tx: 0x%x\n",*((int *)p_tx));
#endif
    //
    spi_message_init(p_msg);
    
    spi_message_add_tail(&transfer, p_msg);
    
    // take note : dont change <-- Daniel commented on 20191108  
    // ==============================
    DECLARE_COMPLETION_ONSTACK(done);
    //config for spi_message
    p_msg->complete = socspi_complete;
    p_msg->context = &done;
    // ==============================

    spin_lock_irq(&p_socspi->spi_lock);
    if(p_socspi->spi == NULL)
        retval = -ESHUTDOWN;
    else
    {
#if 0
	    printk(KERN_DEBUG "spi->bits_per_word = %d\n",p_socspi->spi->bits_per_word);
	    printk(KERN_DEBUG "spi->max_speed_hz = %d\n",p_socspi->spi->max_speed_hz);
	    printk(KERN_DEBUG "spi->chip_select = %d\n", p_socspi->spi->chip_select);
	    printk(KERN_DEBUG "spi->mode = %d\n", p_socspi->spi->mode);
#endif
        retval = spi_async(p_socspi->spi, p_msg);
    }
    spin_unlock_irq(&p_socspi->spi_lock);

    if(!retval)
    {
        wait_for_completion(&done);
        retval = p_msg->status;
        if(!retval)
        {
            retval = p_msg->actual_length;
            //printk(KERN_DEBUG "p_msg->actual_length = %d\n",p_msg->actual_length);
        }
    }

    // handle error 
    if(retval < 0 && retry > 0)
    {
        retry --;
        if(kthread_should_stop())
        {
            retval = -EFAULT;
        }
        else
        {
            goto __SEND_COMMAND_LOOP;
        }
    }
    else if(is_user_cmd)
    {
        mutex_lock(&p_socspi->command_lock);
        // clear ccommand
        p_socspi->command = 0x00;
        mutex_unlock(&p_socspi->command_lock);
    }
#ifdef DEBUG_SPI_P
    //if(p_rx)
    //    printk(KERN_DEBUG "CMD rx: 0x%x\n",*((int *)p_rx));
#endif
    return retval;
} 

//define hard_reset func
static void socspi_hard_reset_module(void)
{
    printk(KERN_INFO "start hard reset module\n");
    __gpio_set_value(GPIO_RESET_PIN, 0);
    msleep(100);
    __gpio_set_value(GPIO_RESET_PIN, 1);
}

// define socspi_check_command_status 
static bool socspi_check_command_status(unsigned int rx_status)
{
#ifdef REVERSING_RECEIVE_ORDER_PLATFORM // AIT
    if ((rx_status & 0xFF) > 0 &&
        (rx_status & 0xFF) < 0x70 &&
        (rx_status & 0xFF) != 0x05)
    {
    	return true;
    }
#else
    unsigned char *rx = (unsigned char*)&rx_status;
    if (((rx[3] & 0xFF) > 0) &&
        ((rx[3] & 0xFF) < 0x70) &&
        ((rx[3] & 0xFF) != 0x05))
    {
    	return true;
    }
#endif
    return false;
}

// define socspi maintask
static int socspi_maintask(void *data)
{
    struct spi_message msg;
    struct socspi_data *p_socspi = NULL;
    unsigned int rx_status = 0;
    int retval = 0;
    unsigned int command = 0;
    int len = 0;

    unsigned int audio_busy_jiff = jiffies;
    unsigned int video_busy_jiff = jiffies;

    p_socspi = (struct socspi_data *)data;
    if(IS_ERR(p_socspi))
        return -EFAULT;
    //printk(KERN_INFO "maintask: p_socspi = 0x%x\n",(unsigned int)p_socspi);
    do
    {
        // BLE pairing 
        if(p_socspi->is_set_ble)
        {
            command = (!p_socspi->is_ble_on)? SPI_CMD_BLE_MODE(5) : SPI_CMD_BLE_MODE(6);
            printk(KERN_DEBUG "drv_maintask: into is_set_ble : ble is %s\n",p_socspi->is_ble_on ? "ON" : "OFF");
            retval = socspi_send_command(p_socspi, &msg, (char *)&command, COMMAND_LENGTH, &rx_status, false);
            if(retval < 0)
            {
                printk(KERN_ERR "drv_maintask: socspi_send_command set ble failed\n");
                //p_socspi->is_ble_on = (p_socspi->is_ble_on) ? false : true; 
                goto __CONTINUE_LOOP;
            }
            else
            {
                p_socspi->is_set_ble = false;
                wake_up_interruptible(&p_socspi->ble_wq);
            }
        }
        /*** @TODO : consider to move current one to ioctl entrypoint to reduce time transfer audio,video***/
        // receive flush queue buff cmd
        // user space : will be flush queue buff before and after pairing process
        if(p_socspi->is_flush_all)
        {
#ifdef DEBUG_SPI_P
            printk(KERN_DEBUG "drv_maintask: into is_flush_all\n");
#endif
            printk(KERN_INFO "start clean queue\n");
            kfifo_reset(&p_socspi->talkback_fifo);
            kfifo_reset(&p_socspi->video_fifo);
            kfifo_reset(&p_socspi->audio_fifo);
            
            //printk(KERN_INFO "***size of video queue : %d", kfifo_len(&p_socspi->video_fifo));
            //printk(KERN_INFO "***size of audio queue : %d", kfifo_len(&p_socspi->audio_fifo));

            wake_up_interruptible(&p_socspi->audio_wq);
            wake_up_interruptible(&p_socspi->video_wq);

            mutex_lock(&p_socspi->command_lock);
            p_socspi->command = 0x00;
            mutex_unlock(&p_socspi->command_lock);
            wake_up_interruptible(&p_socspi->command_wq);

            p_socspi->is_flush_all = false;
            wake_up_interruptible(&p_socspi->flush_all_wq);
        } 
        // receive flush audio buff cmd
        // user will flush audio talkback process 
        // to avoid previous buffer data send back to handset
        if(p_socspi->is_flush_audio)
        {
#ifdef DEBUG_SPI_P
            printk(KERN_DEBUG "drv_maintask: into is_flush_audio\n");
#endif
            kfifo_reset(&p_socspi->audio_fifo);
            p_socspi->is_flush_audio = false;
            wake_up_interruptible(&p_socspi->audio_wq);
        }

        // recevie normal cmd from user
        if(p_socspi->command != 0) // 0x0000ffff
        {
#ifdef DEBUG_SPI_P
            printk(KERN_DEBUG "drv_maintask: into serve command user\n");
#endif
            // ***COMMAND_TRANSFER : STEP 2***
            mutex_lock(&p_socspi->command_lock);
            command = p_socspi->command;
            mutex_unlock(&p_socspi->command_lock);

            // call interface send function
            retval = socspi_send_command(p_socspi, &msg, (char *)&command, COMMAND_LENGTH, &rx_status, true);

	        if((retval >= 0) && ((rx_status & DEVICE_ADDRESS_MASK) == DEVICE_ADDRESS_MASK))
            {
                if(socspi_check_command_status(rx_status))
                {
                    //printk(KERN_DEBUG "drv_maintask: into serve command user -> update command_status ok\n");
                    mutex_lock(&p_socspi->command_lock);
                    p_socspi->command_status = rx_status; // ***STATUS_TRANSFER : STEP 2***
                    mutex_unlock(&p_socspi->command_lock);
                    // signal to permit handle next command
                    wake_up_interruptible(&p_socspi->command_wq);
                }
                else
                {
                    //printk(KERN_DEBUG "drv_maintask: into serve command user -> data respone invalid\n");
                }
                // if handset request talkback -> servere it
                if(rx_status & TALKBACK_FULL_MASK)
                {

                    // ***TALKBACK_TRANSFER : STEP 1***
                    p_socspi->is_talkback = true;
                    goto __TALKBACK_READING;
                }
            }
            else
                goto __CONTINUE_LOOP;
        }
        // receive dummy cmd from user
        // purpose is send dummy to spidevice slave then check respone data 
        // if respone data indicate that have BLE data then will be pick up it  
        // @TODO :need push is_dummy_on variable into socspi_data struct
        if(p_socspi->is_dummy_on)
        {
#ifdef DEBUG_SPI_P
            //printk(KERN_DEBUG "drv_maintask: into is_dummy_on\n");
#endif
            command = SPI_CMD_SEND_DUMMY;
            retval = socspi_send_command(p_socspi, &msg, (char *)&command, COMMAND_LENGTH, &rx_status, false);
            if((retval >= 0) && ((rx_status & DEVICE_ADDRESS_MASK) == DEVICE_ADDRESS_MASK))
            {
                if(socspi_check_command_status(rx_status))
                {
                    p_socspi ->command_status = rx_status;
                    wake_up_interruptible(&p_socspi->command_wq);
                }
                if(rx_status & BLE_FULL_MASK) // @TODO : need redefine TALKBACK_FULL_MASK = BLE_FULL_MASK
                {
                    p_socspi->is_talkback = true;
                    goto __TALKBACK_READING;
                }
            }
            else
            {
                msleep(10);
                goto __CONTINUE_LOOP;
            }
            //msleep(10);
        }

__TALKBACK_READING:

        // take note : --> Daniel Nguyen commented
        // *** just receive talkback data from handset 
        // or send audio data to handset at the same time ***

        // talkback data is available or handset send data for BLE setup 
        if(p_socspi->is_talkback)
        {
#ifdef DEBUG_SPI_P
            printk(KERN_INFO "drv_maintask: into is_talkback\n");
#endif
            // ***TALKBACK_TRANSFER : STEP 2***
            retval = socspi_send_command(p_socspi, &msg, NULL, 
                    p_socspi->is_ble_on ? BLE_TRANSFER_SIZE : TALKBACK_TRANSFER_SIZE, p_socspi->talkback_buffer, false);
            if(retval >= 0)
            {
                if(kfifo_avail(&p_socspi->talkback_fifo) < retval)
                {
                    printk(KERN_DEBUG "talkback fifo is overun. resetting...\n");
                    mutex_lock(&p_socspi->talkback_lock);
                    kfifo_reset(&p_socspi->talkback_fifo);
                    mutex_unlock(&p_socspi->talkback_lock);
                }
                mutex_lock(&p_socspi->talkback_lock);
                kfifo_in(&(p_socspi->talkback_fifo), p_socspi->talkback_buffer, retval);
                mutex_unlock(&p_socspi->talkback_lock);
                if(p_socspi->talkback_blocked)
                {
                    wake_up_interruptible(&p_socspi->talkback_wq);
                }
                p_socspi->is_talkback = false;
            }
            else
            {
                goto __CONTINUE_LOOP;
            }
        }
        // If audio buffer is available and the last audio send is more than 40ms 
        // ***AUDIO_TRANSFER : STEP 2***
        // @TODO: need  check talkback is active or not 
        else if(kfifo_len(&p_socspi->audio_fifo) >= AUDIO_TRANSFER_SIZE &&
                (jiffies - audio_busy_jiff) > MS_TO_TICKS(20) ) // @TODO: need file time here
        {
#ifdef DEBUG_SPI_P
            //printk(KERN_DEBUG "drv_maintask: into send audio  : len = %d\n",kfifo_len(&p_socspi->audio_fifo));
#endif
	        command = SPI_CMD_AUDIO(AUDIO_TRANSFER_SIZE);
            //printk(KERN_DEBUG "audio CMD : 0x%x\n",command);
	        retval = socspi_send_command(p_socspi, &msg, (char *)&command, COMMAND_LENGTH, &rx_status, false);
	        if((retval >= 0) && ((rx_status & DEVICE_ADDRESS_MASK) == DEVICE_ADDRESS_MASK))
	        {
                // check command status
	            if(socspi_check_command_status(rx_status))
	            {
	                p_socspi->command_status = rx_status;
	        	    wake_up_interruptible(&p_socspi->command_wq);
	            }

	            // check talkback
	            if(rx_status & TALKBACK_FULL_MASK)
	            {

                    // ***TALKBACK_TRANSFER : STEP 1***
                    p_socspi->is_talkback = true;
	        	    goto __TALKBACK_READING;
	            }
	         	if(!(rx_status & AUDIO_FULL_MASK))
	            {
	        	    mutex_lock(&p_socspi->audio_lock);
	        	    retval = kfifo_out_peek(&p_socspi->audio_fifo, p_socspi->audio_buffer, AUDIO_TRANSFER_SIZE);
	        	    mutex_unlock(&p_socspi->audio_lock);
	        	    retval = socspi_send_command(p_socspi, &msg, p_socspi->audio_buffer, AUDIO_TRANSFER_SIZE, p_socspi->buffer, false);
	        	    if(retval >= 0 && retval == AUDIO_TRANSFER_SIZE)
	        	    {
	        	        
	        	        mutex_lock(&p_socspi->audio_lock);
	        	        retval = kfifo_out(&p_socspi->audio_fifo, p_socspi->audio_buffer, AUDIO_TRANSFER_SIZE);
	        	        mutex_unlock(&p_socspi->audio_lock);
		    	        if((p_socspi->audio_blocked) && (kfifo_avail(&p_socspi->audio_fifo) >= p_socspi->audio_req_len))
		    	        {
		    	            wake_up_interruptible(&p_socspi->audio_wq);
		    	        }
	        	    }
		            else
		            {
	                    audio_busy_jiff = jiffies;
		    	        printk(KERN_DEBUG "lost audio data\n");
		            }
	            }
		        else
		        {
                    audio_busy_jiff = jiffies;
	                //printk(KERN_DEBUG "audio is busy\n");
		        }
	        }
	        else
	        {

	            goto __CONTINUE_LOOP;
	        }
        }
//__VIDEO_SENDING:
        // video is availble 
        // if video buff is full : --> don't permit send video
        // ***VIDEO_TRANSFER : STEP 2***
        //if((kfifo_len(&p_socspi->video_fifo) >= VIDEO_TRANSFER_SIZE) && socspi_permit_send_video())
        if((kfifo_len(&p_socspi->video_fifo) >= VIDEO_TRANSFER_SIZE) && ((jiffies - video_busy_jiff) > MS_TO_TICKS(40)))
        {
#ifdef DEBUG_SPI_P
            //printk(KERN_DEBUG "drv_maintask: into send video : %d\n", kfifo_len(&p_socspi->video_fifo));
#endif
            // @TODO:
            command = SPI_CMD_VIDEO(VIDEO_TRANSFER_SIZE);
            retval = socspi_send_command(p_socspi, &msg, (char *)&command, COMMAND_LENGTH, &rx_status, false);
            if(retval >= 0 && ((rx_status & DEVICE_ADDRESS_MASK) == DEVICE_ADDRESS_MASK))
            {
                // check command status
	            if(socspi_check_command_status(rx_status))
	            {
	                p_socspi->command_status = rx_status;
	    	        wake_up_interruptible(&p_socspi->command_wq);
	            }

                // check talkback
                if(rx_status & TALKBACK_FULL_MASK)
                {

                    // ***TALKBACK_TRANSFER : STEP 1***
                    p_socspi->is_talkback = true;
                    goto __TALKBACK_READING;
                }
                if(!(rx_status & VIDEO_FULL_MASK))
                {
                    mutex_lock(&p_socspi->video_lock);
                    retval = kfifo_out_peek(&p_socspi->video_fifo, p_socspi->video_buffer, VIDEO_TRANSFER_SIZE);
                    mutex_unlock(&p_socspi->video_lock); 
                    len = 0;
                    while(len < VIDEO_TRANSFER_SIZE)
                    {
                        retval = socspi_send_command(p_socspi, &msg, p_socspi->video_buffer + len, BULK_TRANSFER_SIZE, p_socspi->buffer, false);
                        if(retval >= 0 && (retval == BULK_TRANSFER_SIZE))
                        {
                            len += BULK_TRANSFER_SIZE;
                        }
                    }
                    if(len == VIDEO_TRANSFER_SIZE)
                    {
                        mutex_lock(&p_socspi->video_lock);
                        retval = kfifo_out(&p_socspi->video_fifo, p_socspi->video_buffer, VIDEO_TRANSFER_SIZE);
                        mutex_unlock(&p_socspi->video_lock);
                        if(p_socspi->video_blocked && (kfifo_avail(&p_socspi->video_fifo) >= p_socspi->video_req_len))
                        {
                            wake_up_interruptible(&p_socspi->video_wq);
                        }
                    }
                    else
                    {
                        printk(KERN_DEBUG "lost video data\n");
                        video_busy_jiff = jiffies;
                        // maybe need skip some ms (2ms or 3ms)
                    }
                }
                else
                {
                    // maybe need skip some ms (2ms or 3ms)
                    //printk(KERN_DEBUG "video is busy \n");
                    video_busy_jiff = jiffies;
                    retval = -EBUSY;
                }
            }
            else
            {
                goto __CONTINUE_LOOP;
            }
        }

        // BLE pairing 
        // use to send response data after recevie and handle
        if(p_socspi->jcmd_length > 0) // have respone data which sent from user land
        {
#ifdef DEBUG_SPI_P
            printk(KERN_INFO "drv_maintask: into jcmd\n");
#endif
            command = SPI_CMD_J_WRITE(p_socspi->jcmd_length);
            retval = socspi_send_command(p_socspi, &msg, (char *)&command, COMMAND_LENGTH, &rx_status, false);
            if((retval >= 0) && ((rx_status & DEVICE_ADDRESS_MASK) == DEVICE_ADDRESS_MASK))
            {
                if(socspi_check_command_status(rx_status))
                {
                    p_socspi->command_status = rx_status;
                    wake_up_interruptible(&p_socspi->command_wq);
                }
                if(rx_status & BLE_FULL_MASK)
                {
                    p_socspi->is_talkback = 1;
                    goto __TALKBACK_READING;
                }

                if(!(rx_status & J_FULL_MASK))
                {
                    len = 0;
                    while(len < p_socspi->jcmd_length)
                    {
                        retval = socspi_send_command(p_socspi, &msg, p_socspi->jcmd_buffer + len, BULK_TRANSFER_SIZE, p_socspi->buffer, false);
                        if(retval < 0)
                        {
                            printk(KERN_INFO "send ble data failed\n");
                            //msleep(40);
                            goto __CONTINUE_LOOP;
                        }
                        if((p_socspi->jcmd_length - len) >= BULK_TRANSFER_SIZE)
                            len += BULK_TRANSFER_SIZE;
                        else
                            len += (p_socspi->jcmd_length - len);
                    }
                    // re-check
                    if((len == p_socspi->jcmd_length) || (len == BULK_TRANSFER_SIZE))
                    {
#ifdef DEBUG_SPI_P
                        printk(KERN_INFO "J cmd sent %d byte\n",p_socspi->jcmd_length);
#endif
                        p_socspi->jcmd_length = 0;
                        wake_up_interruptible(&p_socspi->jcmd_wq);
                    }
                    else
                    {
                        retval = -EFAULT;
                        printk(KERN_INFO "lost ble response data : %d -- %d\n",p_socspi->jcmd_length, len);
                    }
                }
                else
                {
                    retval = -EBUSY;
                    //msleep(20);
                    printk(KERN_INFO "Jcmd is busy\n");
                }
            }
            else
            {
                printk(KERN_INFO "precheck send ble data failed\n");
                //msleep(20);
                goto __CONTINUE_LOOP;
            }
        }
__CONTINUE_LOOP:
        //printk(KERN_DEBUG "need sleep here to reduce cpu resource\n");
        usleep_range(500, 1000);

    }
    while(!kthread_should_stop());
    printk(KERN_INFO "***maintask stopped***\n");
    return retval;
}

/*========================================================================*/
/*======================END DEFINE INTERNAL FUNCTION======================*/
/*========================================================================*/



/*========================================================================*/
/*=================START DEFINE SEQ_OPERATIONS STUFF======================*/
/*========================================================================*/

// define seq start
static void *socspi_seq_start(struct seq_file *s, loff_t *f_pos)
{
    printk(KERN_DEBUG "socspi_seq_start called\n");
    char *seq_msg = NULL;
    unsigned int command;
    struct spi_message msg;
    struct socspi_data *p_socspi = NULL;
    unsigned int rx_status;
    int retval = -1;

    seq_msg = vmalloc(sizeof(int));
    if(IS_ERR(seq_msg))
    {
        printk(KERN_ERR "unable to allocate memory for seq_msg\n");
        return NULL;
    }

    // @TODO: need push data into msg 

    list_for_each_entry(p_socspi, &device_list, device_entry)
    {
        //printk(KERN_INFO "open : 0x%x\n",(unsigned int)p_socspi);
        if(p_socspi)
            break;
    }
    if(IS_ERR(p_socspi))
    {
        printk(KERN_ERR "p_socspi is null\n");
        return NULL;
    }

	command = SPI_CMD_AUDIO(AUDIO_TRANSFER_SIZE);

	retval = socspi_send_command(p_socspi, &msg, (char *)&command, COMMAND_LENGTH, &rx_status, false);

	if((retval >= 0) && ((rx_status & DEVICE_ADDRESS_MASK) == DEVICE_ADDRESS_MASK))
    {

        // check command status
	    if(socspi_check_command_status(rx_status))
	    {
	        p_socspi->command_status = rx_status;
	        wake_up_interruptible(&p_socspi->command_wq);
	    }

	    if(!(rx_status & AUDIO_FULL_MASK))
        {
            snprintf(seq_msg, sizeof(int), "%s", "on");
            return seq_msg;
        }
    }

    snprintf(seq_msg, sizeof(int), "%s", "off");
    return seq_msg;
}

// define seq show
static int socspi_seq_show(struct seq_file *s, void *p_data)
{
    printk(KERN_DEBUG "socspi_seq_show called\n");
    char *msg = (char *)p_data;

    // write msg into buffer of struct seq_file
    seq_printf(s, "%s\n", msg);
    return 0;
}

// define seq next
static void *socspi_seq_next(struct seq_file *s, void *p_data, loff_t *f_pos)
{
    //131072
    *f_pos = *f_pos + 3;
    // return null indicate that don't have any  message need to send
    return NULL;
}

// define seq stop
static void socspi_seq_stop(struct seq_file *s, void *p_data)
{
    printk(KERN_DEBUG "socspi_seq_stop called\n");
    if(p_data)
        vfree(p_data);
}

static struct seq_operations seq_fops = {
    .start      =   socspi_seq_start,
    .next       =   socspi_seq_next,
    .stop       =   socspi_seq_stop,
    .show       =   socspi_seq_show
};


/*========================================================================*/
/*==================END DEFINE SEQ_OPERATIONS STUFF ======================*/
/*========================================================================*/



/*========================================================================*/
/*======================START DEFINE ENTRYPOIN  FUNCTION==================*/
/*========================================================================*/
// define socspi open function
static int socspi_open(struct inode *inode, struct file *filp)
{
    struct socspi_data  *p_socspi = NULL;
    // how to get socspi_data which we created in probe function
    //  solution -> use linked list  
    mutex_lock(&device_mutex_lock);
    list_for_each_entry(p_socspi, &device_list, device_entry)
    {
        //printk(KERN_INFO "open : 0x%x\n",(unsigned int)p_socspi);
        if(p_socspi)
            break;
    }
    if(IS_ERR(p_socspi))
    {
        printk(KERN_ERR "p_socspi is null\n");
        return -EFAULT;
    }

    filp->private_data = p_socspi;
    
    // allocate buff
    // for read talkback data     : buff is a parameter as tx_buff
    // for send 128 byte audio    : buff is a parameter as rx_buff
    // for send 512 byte video    : buff is a paremeter as rx_buff
    // @TODO: consider to remove it : unnecessary  --> Daniel.Nguyen commented
    p_socspi->buffer = vmalloc(SIZE_BUFF);
    if(IS_ERR(p_socspi->buffer))
    {
        printk(KERN_ERR "unable to allocate mem\n");
        return -ENOMEM;
    }
    // init flag
    p_socspi->video_req_len = 0;
    p_socspi->audio_req_len = 0;
    p_socspi->video_blocked = false;
    p_socspi->audio_blocked = false;
    p_socspi->talkback_blocked = false;
    p_socspi->is_talkback = false;
    p_socspi->is_set_ble = false;
    p_socspi->is_flush_all   = false;
    p_socspi->is_flush_audio = false;
    p_socspi->jcmd_length = 0x00;

    p_socspi->is_dummy_on = false;

    // init command
    p_socspi->command = 0x00;
    p_socspi->command_status = 0x00;


    // create main thread
    maintask = kthread_run(socspi_maintask, (void*)p_socspi, "socspi");
    printk(KERN_INFO "kernel thread: %s\n", maintask->comm);
    mutex_unlock(&device_mutex_lock);
    return 0;
}

// define socspi release function
static int socspi_release(struct inode *inode, struct file *filp)
{
    struct socspi_data *p_socspi = NULL;
    mutex_lock(&device_mutex_lock);
    p_socspi = filp->private_data;
    if(IS_ERR(filp->private_data))
    {
        return -1;
    }
    // stop kthread
    if(maintask)
    {
        kthread_stop(maintask);
        maintask = NULL;
        printk(KERN_INFO "stopped main task\n");
    }
    // free buff
    // @TODO : consider to remove this one --> Daniel commented
    vfree(p_socspi->buffer);
    p_socspi->buffer = NULL;
    mutex_unlock(&device_mutex_lock);
    return 0;

}

// define  socspi ioctl function
static long socspi_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct socspi_data *p_socspi = NULL;
    struct spi_device *spi = NULL;
    struct socspi_transfer req;
    int retval = 0;
    int copied = 0;

    // get p_socspi
    p_socspi = filp->private_data;
    // get spi_device
    spin_lock_irq(&p_socspi->spi_lock);
    spi = spi_dev_get(p_socspi->spi);
    spin_unlock_irq(&p_socspi->spi_lock);


    if(IS_ERR(spi))
        return -ESHUTDOWN;
    //@ TODO : need support multi process
    //mutex_lock(&p_socspi->buf_lock);
    switch(cmd) {
        // read request
        case SPI_IOC_RD_MODE:
            mutex_lock(&p_socspi->buf_lock);
            retval = __put_user(spi->mode & SPI_MODE_MASK, (__u8 __user *)arg);
            mutex_unlock(&p_socspi->buf_lock);
            break;
        case SPI_IOC_RD_LSB_FIRST:
            mutex_lock(&p_socspi->buf_lock);
            retval = __put_user((spi->mode & SPI_LSB_FIRST) ? 1 : 0, (
                        __u8 __user *)arg);
            mutex_unlock(&p_socspi->buf_lock);
            break;
        case SPI_IOC_RD_BITS_PER_WORD:
            mutex_lock(&p_socspi->buf_lock);
            retval = __put_user(spi->bits_per_word, (__u8 __user *)arg);
            mutex_unlock(&p_socspi->buf_lock);
            break;
        case SPI_IOC_RD_MAX_SPEED_HZ:
            mutex_lock(&p_socspi->buf_lock);
            retval = __put_user(spi->max_speed_hz, (__u8 __user *)arg);
            mutex_unlock(&p_socspi->buf_lock);
            break;
        // write request
        case SPI_IOC_WR_MODE:
            mutex_lock(&p_socspi->buf_lock);
            retval = spi_setup(spi);
            mutex_unlock(&p_socspi->buf_lock);
            break;
        case SPI_IOC_WR_LSB_FIRST:
            mutex_lock(&p_socspi->buf_lock);
            retval = spi_setup(spi);
            mutex_unlock(&p_socspi->buf_lock);
            break;
        case SPI_IOC_WR_BITS_PER_WORD:
            mutex_lock(&p_socspi->buf_lock);
            retval = spi_setup(spi);
            mutex_unlock(&p_socspi->buf_lock);
            break;
        case SPI_IOC_WR_MAX_SPEED_HZ:
            mutex_lock(&p_socspi->buf_lock);
            retval = spi_setup(spi);
            mutex_unlock(&p_socspi->buf_lock);
            break;
        // processing our own command
        case SOCSPI_VIDEO_TRANSFER:
        case SOCSPI_H264_TRANSFER:
#ifdef DEBUG_SPI_P
            //printk(KERN_DEBUG "drv ioctl: into send video\n");
#endif
            // ***VIDEO_TRANSFER : STEP 1***
            if(copy_from_user(&req, (void __user *)arg, sizeof(struct socspi_transfer)))
            {
                retval=  -EFAULT;
                break;
            }
            if(kfifo_avail(&p_socspi->video_fifo) < req.len && !p_socspi->is_ble_on)
            {
                p_socspi->video_req_len = req.len;
                p_socspi->video_blocked = true;
                retval = wait_event_interruptible_timeout(p_socspi->video_wq, 
                        (kfifo_avail(&p_socspi->video_fifo) >= req.len)
                        || (p_socspi->is_ble_on), HZ);
                p_socspi->video_blocked = false;
                if(retval <= 0)
                {
                    //printk(KERN_DEBUG "video wait to push data to queue timeout\n");
                    retval = -ETIMEDOUT;
                    break;
                }
            }
            if(p_socspi->is_ble_on)
            {
                kfifo_reset(&p_socspi->video_fifo);
                retval= -EBUSY;
                break;
            }
            // handle only for h264 type
            if(cmd == SOCSPI_H264_TRANSFER)
            {
               // retval = socspi_handle_h264_frame(socspi, &req);
               // if(retval != 0)
               //     break;
            }
            retval = kfifo_from_user(&p_socspi->video_fifo, (void __user *)req.data, req.len, &copied);
            if(!retval && (copied == req.len))
            {
                retval = copied;
            }
            else
            {
                printk(KERN_ERR "copy video data into video fifo failed\n");
                retval =-EFAULT;
            }
            break;
        case SOCSPI_AUDIO_TRANSFER:
#ifdef DEBUG_SPI_P
            //printk(KERN_DEBUG "drv ioctl: into send audio\n");
#endif
            // @TODO: need  check talkback is active or not 
            // ***AUDIO_TRANSFER : STEP 1***
	        if(copy_from_user(&req, (void __user *)arg, sizeof(struct socspi_transfer)))
	        {
		        printk(KERN_ERR "send audio failed . unable to copy data from user land to kernel land\n");
		        retval =  -EFAULT;
		        break;
	        }
	        // put audio data into audio buff until full
	        if( (kfifo_avail(&p_socspi->audio_fifo) < req.len) && !(p_socspi->is_ble_on))
	        {
		        p_socspi->audio_req_len = req.len;
		        p_socspi->audio_blocked = true;
		        retval= wait_event_interruptible_timeout(p_socspi->audio_wq, 
		    		    (kfifo_avail(&p_socspi->audio_fifo) >= req.len) || p_socspi->is_ble_on, HZ);
		        p_socspi->audio_blocked = false;
		        if(retval <= 0)
		        {
		    	    //printk(KERN_DEBUG "audio wait to push data to queue timeout \n");
		    	    retval = -ETIMEDOUT;
		    	    break;
		        }
	        }
	        // check talkback and ble  is woring or not
	        if(p_socspi->is_talkback || p_socspi->is_ble_on)
	        {
	                kfifo_reset(&p_socspi->audio_fifo);
	                retval = -EBUSY;
	                break;
	        }
	        retval = kfifo_from_user(&p_socspi->audio_fifo, (void __user *)req.data, req.len, &copied);
	        if(!retval && (copied == req.len))
	        {
		        retval = copied;
	        }
	        else
	        {
		        printk(KERN_ERR "copy audio data into audio_fifo failed\n");
		        retval = -EFAULT;
	        }
	    
            break;
        // don't care about response data
        case SOCSPI_COMMAND_TRANSFER:
#ifdef DEBUG_SPI_P
            printk(KERN_DEBUG "drv_ioctl: send cmd\n");
#endif
            // ***COMMAND_TRANSFER : STEP 1***
            if(copy_from_user(&req, (void __user *)arg, sizeof(struct socspi_transfer)))
            {
                retval = -EFAULT;
                break;
            }
            // for second times : need wait and make sure previous command handled already
            retval = wait_event_interruptible_timeout(p_socspi->command_wq, p_socspi->command == 0, HZ);
            if(retval > 0)
            {
                // previous cmd handled already
                mutex_lock(&p_socspi->command_lock);
                p_socspi->command = (unsigned int)req.data;
                mutex_unlock(&p_socspi->command_lock);

                // pre-check command 
                if((p_socspi->command & 0x0000FFFF) != 0x0000FFFF) // @TODO: just check 2 bytes
                {
#ifdef DEBUG_SPI_P
                    printk(KERN_DEBUG "cmd ready to send : 0x%08X\n",p_socspi->command);
#endif
                }
                // return to user space
                retval = req.len;
            }
            else
            {
		        printk(KERN_DEBUG "wait to send cmd timout\n");
                retval = -ETIMEDOUT;
            }
            break;
        // care respone data from slave device
        case SOCSPI_STATUS_TRANSFER:
#ifdef DEBUG_SPI_P
            printk(KERN_DEBUG "drv_ioctl: get status\n");
#endif
            // ***STATUS_TRANSFER : STEP 1***
	        retval = wait_event_interruptible_timeout(p_socspi->command_wq, p_socspi->command_status != 0, HZ);
	        if(retval > 0)
	        {
                mutex_lock(&p_socspi->command_lock);
                printk(KERN_DEBUG "status data --> 0x%x\n",p_socspi->command_status);
		        req.data =(void *) p_socspi->command_status;
		        p_socspi->command_status = 0x00;
                mutex_unlock(&p_socspi->command_lock);

		        if(copy_to_user((void __user *)arg, &req, sizeof(struct socspi_transfer)))
		        {
		    	    retval = -EFAULT;
		        }
		        else
		    	    retval = 0;
	        }
	        else
	        {
		        printk(KERN_DEBUG "wait to get status data timout\n");
		        retval = -ETIMEDOUT;
	        }
                break;
            /*
             * remove unnecessary command --> Daniel commented 20191024
            case SOCSPI_U_READ:
                break;
            case SOCSPI_U_WRITE:
                break;
            */
        case SOCSPI_J_TRANSFER:
#ifdef DEBUG_SPI_P
            printk(KERN_INFO "drv ioctl: into j transfer\n");
#endif
            if(copy_from_user(&req, (void __user*)arg, sizeof(struct socspi_transfer)))
            {
                retval = -EFAULT;
                break;
            }
            retval = ((req.len/BULK_TRANSFER_SIZE) + 1)*BULK_TRANSFER_SIZE;
            if(retval > ble_buff_max_size)
            {
                retval = -EFAULT;
                break;
            }
            retval = wait_event_interruptible_timeout(p_socspi->jcmd_wq, p_socspi->jcmd_length == 0, HZ);
            if(retval > 0)
            {
                if(copy_from_user(p_socspi->jcmd_buffer, (const u8 __user*)(uintptr_t) req.data, req.len))
                {
                    retval = -EFAULT;
                    break;
                }
                p_socspi->jcmd_length = req.len;
                retval = req.len;
            }
            else
            {
                retval = -ETIMEDOUT;
            }
            break;
        // when we use flush data
        // -> when BU into pairing mode . whatever pair to handset or mobile device
        case SOCSPI_FLUSH_DATA:
#ifdef DEBUG_SPI_P
            printk(KERN_DEBUG "drv ioctl: into flush all data\n");
#endif
            p_socspi->is_flush_all = true;
            retval = wait_event_interruptible_timeout(p_socspi->flush_all_wq, p_socspi->is_flush_all == false, HZ);
            p_socspi->is_flush_all = false;
            if(retval <= 0)
                retval = -ETIMEDOUT;
            break;
        // when we use flush audio
        // --> after close talkback session 
        case SOCSPI_FLUSH_AUDIO:
#ifdef DEBUG_SPI_P
            printk(KERN_DEBUG "drv ioctl: into flush audio data\n");
#endif
            p_socspi->is_flush_audio = true;
            retval = wait_event_interruptible_timeout(p_socspi->audio_wq, p_socspi->is_flush_audio == false, HZ);
            p_socspi->is_flush_audio = false;
            if(retval <= 0)
                retval = -ETIMEDOUT;
            break;
        case SOCSPI_SWITCH_ON_BLE:
            mutex_lock(&p_socspi->buf_lock);
#ifdef DEBUG_SPI_P
            printk(KERN_INFO "drv ioctl : ble switch on\n");
#endif
            // don't care data from user land
            p_socspi->is_ble_on = true;
            p_socspi->is_set_ble = true;

            retval = wait_event_interruptible_timeout(p_socspi->ble_wq, p_socspi->is_set_ble == false, HZ);
            // prevent loop forever 
            p_socspi->is_set_ble = false;
            if(retval <= 0)
            {
                retval = -ETIMEDOUT;
            }
            mutex_unlock(&p_socspi->buf_lock);
            break;
        case SOCSPI_SWITCH_OFF_BLE:
            mutex_lock(&p_socspi->buf_lock);
#ifdef DEBUG_SPI_P
            printk(KERN_INFO "drv ioctl : ble switch off\n");
#endif
            // don't care data from user land
            p_socspi->is_ble_on = false;
            p_socspi->is_set_ble = true;

            retval = wait_event_interruptible_timeout(p_socspi->ble_wq, p_socspi->is_set_ble == false, HZ);
            // prevent loop forever 
            p_socspi->is_set_ble = false;
            if(retval <= 0)
                retval = -ETIMEDOUT;
            else
                p_socspi->is_dummy_on = false;
            mutex_unlock(&p_socspi->buf_lock);
            break;
        case SOCSPI_DUMMY_ON:
#ifdef DEBUG_SPI_P
            printk(KERN_INFO "drv ioctl: into dummy on\n");
#endif
            // inform that we are in BLE pairing mode
            p_socspi->is_dummy_on = true;
            break;
        case SOCSPI_DUMMY_OFF:
#ifdef DEBUG_SPI_P
            printk(KERN_INFO "drv ioctl: into dummy off\n");
#endif
            p_socspi->is_dummy_on = false;
            break;
        case SOCSPI_HARD_RESET:
#ifdef DEBUG_SPI_P
#endif
            mutex_lock(&p_socspi->buf_lock);
            socspi_hard_reset_module();
            mutex_unlock(&p_socspi->buf_lock);
            break;
        // to check status of handset device is on or off
        case SOCSPI_PU_STATUS:
#ifdef DEBUG_SPI_P
#endif
            break;
        case SOCSPI_SET_DEBUG:
            break;
        default:
            break;
    }
    //mutex_unlock(&p_socspi->buf_lock);
    spi_dev_put(spi);
    return retval;
}

// function socspi_read
// NOTE: this entry poin use for two purpose 
// 1st : read talkback data
// 2nd : read data which sent when set up use BLE
static ssize_t socspi_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{

    // ***TALKBACK_TRANSFER : STEP 3***
    int retval = 0;
    // use to read talkback data from handset device
    struct socspi_data *p_socspi = NULL;
    unsigned int tb_len = 0; // length of tb data
    unsigned int copied = 0;
#ifdef DEBUG_SPI_P
    printk(KERN_INFO "into socspi_read\n");
#endif
    p_socspi = (struct socspi_data *)filp->private_data;
    if(!p_socspi)
    {
        printk(KERN_ERR "invalid input\n");
        return -EFAULT;
    }

    tb_len = kfifo_len(&p_socspi->talkback_fifo);
    
    if(filp->f_flags & O_NONBLOCK)
    {
        if(tb_len == 0)
            return -EAGAIN;
    }
    else if(tb_len == 0)
    {
        // @TODO : consider to remove talkback_blocked flag <-- Daniel commented on 20191108
        p_socspi->talkback_blocked = true;
        retval = wait_event_interruptible(p_socspi->talkback_wq, kfifo_len(&p_socspi->talkback_fifo) > 0);
        if(retval != 0)
            return -ERESTARTSYS;
        p_socspi->talkback_blocked = false;
    }
    tb_len = (tb_len < count)? tb_len : count;
    //put data to user land
    if(!kfifo_to_user(&p_socspi->talkback_fifo, (char __user*)buf, tb_len, &copied))
    {
        retval = copied;
    }
    else
    {
        retval = -EFAULT;
    }
    return retval;
}

// define socspi_write
static ssize_t socspi_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
    return count;
}


// define pool function
// use this entry point to know when talkback data is availble to read
static unsigned __bitwise  socspi_pool(struct file *filp, struct poll_table_struct *pwait)
{
    unsigned __bitwise mask = 0;
    struct socspi_data *p_socspi = NULL;
#ifdef DEBUG_SPI_P
    printk(KERN_DEBUG "into socspi_pool\n");
#endif
    p_socspi = (struct socspi_data *)filp->private_data;
    if(IS_ERR(p_socspi))
    {
        return -1;
    }
    
    p_socspi->talkback_blocked = true;
    poll_wait(filp, &p_socspi->talkback_wq, pwait);

    if(kfifo_len(&p_socspi->talkback_fifo) > 0)
    {
        mask |= POLLIN | POLLRDNORM;
    }
    return mask;
}


// =========================FOR PROC FILE SYSTEM==========================//
static int proc_open(struct inode *inode, struct file *filp)
{
    printk(KERN_DEBUG "proc_open func called \n");
    return seq_open(filp, &seq_fops);
}

static int proc_release(struct inode *inode, struct file *filp)
{
    printk(KERN_DEBUG "proc_release func called\n");
    return seq_release(inode, filp);
}

static ssize_t proc_read(struct file *filp, char __user *buff, size_t count, loff_t *f_pos)
{
    // @TODO: do something to put data to user land
    printk(KERN_DEBUG "proc_read func called\n");
    if((*f_pos) > 0)
        return 0;

    return seq_read(filp, buff, count, f_pos);
}

/*========================================================================*/
/*========================END DEFINE ENTRYPOIN  FUNCTION==================*/
/*========================================================================*/



/*========================================================================*/
/*==================START DEFINE STRUCT FILE_OPERATIONS===================*/
/*========================================================================*/

static const struct file_operations socspi_fops = {
    .owner              =   THIS_MODULE,
    .write              =   socspi_write,
    .read               =   socspi_read,
    .unlocked_ioctl     =   socspi_ioctl,
    .open               =   socspi_open,
    .release            =   socspi_release,
    .poll               =   socspi_pool,
};


static const struct file_operations proc_fops = {
    .open               =  proc_open,
    .release            =  proc_release,
    .read               =  proc_read, 
};

/*========================================================================*/
/*=================== END DEFINE STRUCT FILE_OPERATIONS===================*/
/*========================================================================*/



/*========================================================================*/
/*================ START DEFINE PROBE & REMOVE FUNCTION===================*/
/*========================================================================*/

static int __devinit socspi_probe(struct spi_device *spi)
{
	struct socspi_data *socspi = NULL;
	int ret = -1;
	printk(KERN_INFO "probe spi protocol called\n");
	
	if(IS_ERR(spi))
	{
		printk(KERN_ERR "spidev not initialize yet\n");
		return -ENODEV;
	}
    // hardcode spi setting
    spi->bits_per_word = 8;
    spi->max_speed_hz = 1500000;
    spi->chip_select = 0;
    spi->mode =  SPI_MODE_3;
#ifdef 	DEBUG_SPI_P
	printk(KERN_DEBUG "spi->bits_per_word = %d\n",spi->bits_per_word);
	printk(KERN_DEBUG "spi->max_speed_hz = %d\n",spi->max_speed_hz);
	printk(KERN_DEBUG "spi->chip_select = %d\n", spi->chip_select);
	printk(KERN_DEBUG "spi->mode = %d\n", spi->mode);
#endif
	ret = alloc_chrdev_region(&dev_num, 0, 1, DEV_NAME);
	if(ret < 0)
	{
		printk(KERN_ERR "failed to alloc_chrdev_region %d\n",ret);
		goto __FAILED_REGISTER_DEVNUM;
	}
	cls = class_create(THIS_MODULE, class_name);
	if(IS_ERR(cls))
	{
		printk(KERN_ERR "failed to create class device\n");
		goto __FAILED_CREATE_CLASS_DEVICE;
	}
	dev = device_create(cls, NULL, dev_num, NULL, DEV_NAME"%d.%d",
			spi->master->bus_num, spi->chip_select);
	if(IS_ERR(dev))
	{
		printk(KERN_ERR "failed to create device\n");
		goto __FAILED_CREATE_DEVICE;
	}
	my_cdev = cdev_alloc();
	if(IS_ERR(my_cdev))
	{
		printk(KERN_ERR "failed to alloc cdev\n");
		goto __FAILED_CDEV_ALLOC;
	}
	cdev_init(my_cdev, &socspi_fops);
	ret = cdev_add(my_cdev, dev_num, 1);
	if(ret < 0)
	{
		printk(KERN_ERR "failed to cdev_add\n");
		goto __FAILED_CDEV_ADD;
	}
	socspi = kzalloc(sizeof(*socspi), GFP_KERNEL);
	if(!socspi)
	{
		printk(KERN_ERR "allocate mem for private data failed\n");
		goto __FAILED_CDEV_ADD;
	}
	socspi->spi = spi;

	// create queue for video
	ret = kfifo_alloc(&socspi->video_fifo, VIDEO_QUEUE_SIZE, GFP_KERNEL);
	if(ret != 0)
	{
		printk(KERN_ERR "alloc video queue buff failed\n");
		goto __FAILED_ALLOC_VIDEO_BUFF;
	}

	// create queue for audio
	ret = kfifo_alloc(&socspi->audio_fifo, AUDIO_QUEUE_SIZE, GFP_KERNEL);
	if(ret != 0)
	{
		printk(KERN_ERR "alloc audio queue buff failed\n");
		goto __FAILED_ALLOC_AUDIO_BUFF;
	}
	
	// create queue for talkback
	ret = kfifo_alloc(&socspi->talkback_fifo, TALKBACK_QUEUE_SIZE, GFP_KERNEL);
	if(ret != 0)
	{
		printk(KERN_ERR "alloc talkback queue buff failed\n");
		goto __FAILED_ALLOC_TALKBACK_BUFF;
	}

    // init waitwueue
    // @TODO : need change name of feild data folowing
    // command_wq_list -> command_wq
    // command_video_list -> video_wq
    // command_audio_list -> audio_wq
    // command_talkback_list -> talkback_wq
    // jcmd_wq_list -> jcmd_wq
    init_waitqueue_head(&socspi->command_wq);
    init_waitqueue_head(&socspi->video_wq);
    init_waitqueue_head(&socspi->audio_wq);
    init_waitqueue_head(&socspi->talkback_wq);
    init_waitqueue_head(&socspi->jcmd_wq);

    init_waitqueue_head(&socspi->flush_all_wq);
    init_waitqueue_head(&socspi->ble_wq);

    // init mutext
    mutex_init(&socspi->audio_lock);
    mutex_init(&socspi->video_lock);
    mutex_init(&socspi->talkback_lock);
    mutex_init(&socspi->command_lock);
    // for ioctl cmd
    mutex_init(&socspi->buf_lock);

    //init spin lock
    spin_lock_init(&socspi->spi_lock);

    // init list
    INIT_LIST_HEAD(&socspi->device_entry);
    // add list device_entry into device_list
    list_add(&socspi->device_entry, &device_list);

	// init gpio
	ret = socspi_gpio_init();
    if(ret < 0)
    {
        printk(KERN_ERR "socspi_gpio_init failed\n");
        goto __FAILED_ALLOC_TALKBACK_BUFF;
    }
	// store socspi  into filed struct device of struct spi_device
    spin_lock_irq(&socspi->spi_lock);
	spi_set_drvdata(spi, socspi);
    spin_unlock_irq(&socspi->spi_lock);
    //printk(KERN_INFO "probe : 0x%x\n",(unsigned int)socspi);
    
    // create procfs to debug pu streamming status
    if(NULL == proc_create("pu_status", 0444, NULL, &proc_fops))
    {
        printk(KERN_ERR "failed to create pu_status file in procfs\n");
        goto __FAILED_ALLOC_TALKBACK_BUFF;
    }

	printk(KERN_INFO "spidev probe successfully\n");
	return ret;
__FAILED_ALLOC_TALKBACK_BUFF:
	kfifo_free(&socspi->talkback_fifo);
__FAILED_ALLOC_AUDIO_BUFF:
	kfifo_free(&socspi->video_fifo);
__FAILED_ALLOC_VIDEO_BUFF:
	kfree(socspi);
__FAILED_CDEV_ADD:
	cdev_del(my_cdev);
__FAILED_CDEV_ALLOC:
	device_destroy(cls, dev_num);
__FAILED_CREATE_CLASS_DEVICE:
	class_destroy(cls);
__FAILED_CREATE_DEVICE:
	unregister_chrdev_region(dev_num, 1);
__FAILED_REGISTER_DEVNUM:
	return ret;
}

// function remove
static int __devexit socspi_remove(struct spi_device *spi)
{
	struct socspi_data *socspi = NULL;
	if(IS_ERR(spi))
		return -ENODEV;
    spin_lock_irq(&socspi->spi_lock);
	socspi = spi_get_drvdata(spi);
    spin_unlock_irq(&socspi->spi_lock);
	if(IS_ERR(socspi))
		return -ENODEV;
	// deinit gpio
	socspi_gpio_deinit();

	kfifo_free(&socspi->video_fifo);
	kfifo_free(&socspi->audio_fifo);
	kfifo_free(&socspi->talkback_fifo);

	kfree(socspi);
	spi_set_drvdata(spi, NULL);

    list_del(&socspi->device_entry);
	cdev_del(my_cdev);
	device_destroy(cls, dev_num);
	class_destroy(cls);
	unregister_chrdev_region(dev_num, 1);

    // remove file in procfs
    remove_proc_entry("pu_status", NULL);

    printk(KERN_INFO "socspi remove success\n");
    return 0;
}

/*========================================================================*/
/*================== END DEFINE PROBE & REMOVE FUNCTION===================*/
/*========================================================================*/

static struct spi_driver socspi_driver = {
	.driver = {
		.name 	= DEV_NAME,
		.owner 	= THIS_MODULE,
	},
	.probe = socspi_probe,
	.remove = __devexit_p(socspi_remove),
};

static int __init socspi_init(void)
{
	int ret = -1;
	ret = spi_register_driver(&socspi_driver);
	if(ret < 0)
	{
		printk(KERN_ERR "spi_register_driver failed\n");
	}
	return ret;
}

static void __exit socspi_exit(void)
{
	spi_unregister_driver(&socspi_driver);
}


module_init(socspi_init);
module_exit(socspi_exit);

MODULE_AUTHOR("Daniel Nguyen <ngocgia73@gmail.com>");
MODULE_DESCRIPTION("spi protocol driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:cv2055spi");
