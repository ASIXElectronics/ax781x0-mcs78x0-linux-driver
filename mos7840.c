/*	
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Originally based on drivers/usb/serial
 *        Copyright (C) 2001-2002 Greg Kroah-Hartman <greg@kroah.com>
 *
 */


/*************************************************************************
 *** --------------------------------------------------------------------
 ***
 *** Project Name: ASIX
 ***
 *** Module Name: mos7840
 ***
 *** File: mos7840.c 
 ***		
 *** File Revision: 1.0
 ***
 *** Revision Date: 27/10/11 
 ***
 *** Purpose	  : It gives an interface between USB to 2/4 Serial 
 ***                and serves as a Serial Driver for the high 
 ***		    level layers /applications.
 ***
 *** Change History:
 ***
 *** LEGEND	  :
 ***
 *** Author	: ravikanth G.
 *** 
 *** DPRINTK - Code inserted due to as part of debugging
 ***
 *** DPRINTK - Debug Print statement
 ***
 *************************************************************************/

/* all file inclusion goes here */



#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/module.h>
#include <linux/serial.h>
#include <linux/usb.h>
#include <linux/wait.h>
#include <asm/uaccess.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,15)
#include <linux/usb/serial.h>
#else
#include <../drivers/usb/serial/usb-serial.h>
#endif
#include "mos7840.h"            /* mos7840 Defines    */
#include "mos7840_16C50.h"	/* 16C50 UART defines */

#include <linux/ioctl.h>
#include "ioctl.h"
#ifdef AUTO_SWITCH
#include "gpioi2c.c"
#endif

#if LINUX_VERSION_CODE > KERNEL_VERSION(4,11,0)
#include <linux/sched/signal.h>
#endif
/* all defines goes here */

/*
 * Debug related defines 
 */

/* 1: Enables the debugging -- 0: Disable the debugging */

#define MOS_DEBUG	0

#if MOS_DEBUG
	#define DPRINTK(fmt, args...) printk("%s: " fmt, __FUNCTION__ , ## args)
#else
	#define DPRINTK(fmt, args...)
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18)
#include <linux/config.h>
#endif

/*****************************************
*     threshold	and Flow Control	 *
*****************************************/
/* Parameter default setting*/
static int threshold_en = 0;
static int threshold_val = 0x40;
static int hw_auto_fc = 2;	// 0: Disable HW auto RTS/CTS flow control
				// 1: Always enable HW auto RTS/CTS flow control
				// 2: Enable HW auto RTS/CTS flow control when cflag & CRTSCTS

module_param(hw_auto_fc, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(hw_auto_fc, "HW auto RTS/CTS FC support, 0:none, 1:force, 2:auto(default)");


/*****************************************
*     		parallel port	         *
*****************************************/
#include <linux/parport.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0))
#include <linux/kthread.h>
#endif
#define MOS_ECR_MODE_INIT	0x25
#define MOS_MAX_PORT    0x01


static int restore_state_thread(void *);




#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0))
	typedef pid_t _thread_hdl_;
#else
	typedef struct task_struct * _thread_hdl_;
#endif

int start_kthread(_thread_hdl_ *t_hdl, int (*threadfn)(void *data), void *data, const char *name)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0))
	*t_hdl = kernel_thread(threadfn, data, CLONE_FS|CLONE_FILES);
	if(*t_hdl < 0)
#else
	*t_hdl = kthread_run(threadfn, data, name);
	if(IS_ERR(*t_hdl))
#endif
		return 0;
	
	return -1;
}

/*****************************************
*     	parallel port----end	         *
*****************************************/

static struct usb_serial* mos7840_get_usb_serial (struct usb_serial_port *port, const char *function);
static int mos7840_serial_paranoia_check (struct usb_serial *serial, const char *function);
static int mos7840_port_paranoia_check (struct usb_serial_port *port, const char *function);

/* setting and get register values */
static int mos7840_set_reg_sync(struct usb_serial_port *port, __u16 reg, __u16 val);
static int mos7840_get_reg_sync(struct usb_serial_port *port, __u16 reg, __u16 * val);
static int mos7840_set_Uart_Reg(struct usb_serial_port *port, __u16 reg, __u16 val);
static int mos7840_get_Uart_Reg(struct usb_serial_port *port, __u16 reg, __u16 * val);

void mos7840_Dump_serial_port(struct moschip_port *mos7840_port);

/************************************************************************/
/************************************************************************/
/*             I N T E R F A C E   F U N C T I O N S			*/
/*             I N T E R F A C E   F U N C T I O N S			*/
/************************************************************************/
/************************************************************************/

static inline void mos7840_set_serial_private(struct usb_serial *serial, struct moschip_serial *data)
{
		usb_set_serial_data(serial, (void *)data );
}

static inline struct moschip_serial * mos7840_get_serial_private(struct usb_serial *serial)
{
		return (struct moschip_serial*) usb_get_serial_data(serial);
}

static inline void mos7840_set_port_private(struct usb_serial_port *port, struct moschip_port *data)
{
		usb_set_serial_port_data(port, (void*)data );
}

static inline struct moschip_port * mos7840_get_port_private(struct usb_serial_port *port)
{
	return (struct moschip_port*) usb_get_serial_port_data(port);
}

/*
 * Description:- To set the Control register by calling usb_fill_control_urb function by passing usb_sndctrlpipe function as parameter.
 * 
 * Input Parameters:
 * usb_serial_port:  Data Structure usb_serialport correponding to that seril port.
 * Reg: Register Address
 * Val:  Value to set in the Register.
 */
static int mos7840_set_reg_sync(struct usb_serial_port *port, __u16 reg, __u16 val)
{
        struct usb_device *dev = port->serial->dev;

	val = val & 0x00ff;

        return usb_control_msg(dev, usb_sndctrlpipe(dev, 0), MCS_WRREQ,
                        MCS_WR_RTYPE, val, reg, NULL, 0, MOS_WDR_TIMEOUT);
}

/*
 * Description:- To set the Uart register by calling usb_fill_control_urb function by passing usb_rcvctrlpipe function as parameter.
 * 
 * Input Parameters:
 * usb_serial_port:  Data Structure usb_serialport correponding to that seril port.
 * Reg: Register Address
 * Val:  Value to receive from the Register.
 */

static int mos7840_get_reg_sync(struct usb_serial_port *port, __u16 reg, __u16 *val)
{
        struct usb_device *dev = port->serial->dev;
        int 	ret = 0;
	u8 * 	pData = mos7840_get_port_private(port)->reg_buffer;

	if (pData == NULL)
		return -1;
		
	ret = usb_control_msg(dev, usb_rcvctrlpipe(dev, 0), MCS_RDREQ,
                        MCS_RD_RTYPE, 0, reg, pData, VENDOR_READ_LENGTH, MOS_WDR_TIMEOUT);
	*val = *pData;
DPRINTK("mos7840_get_reg_sync offset is %x, return val %x, status %d\n", reg, *val, ret);
	*val = (*val) & 0x00ff;
	
        return ret;
}

/*
 * Description:- To set the Uart register by calling usb_fill_control_urb function by passing usb_sndctrlpipe function as parameter.
 *
 * Input Parameters:
 * usb_serial_port:  Data Structure usb_serialport correponding to that seril port.
 * Reg: Register Address
 * Val:  Value to set in the Register.
 */
static int mos7840_set_Uart_Reg(struct usb_serial_port *port, __u16 reg, __u16 val)
{
	struct usb_device *dev = port->serial->dev;
	struct moschip_serial *mos7840_serial;
	int 	tmp = 0;        
	int 	device_type = 0;

	mos7840_serial = mos7840_get_serial_private(port->serial);
	if (mos7840_serial == NULL)
		return -1;
	device_type = mos7840_serial->device_type;

	val = val & 0x00ff;
        // For the UART control registers, the application number need to be Or'ed
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,11,0)
	tmp = ((__u16)port->number - (__u16)(port->serial->minor));
#else
	tmp = (__u16)port->port_number;
#endif
	device_type = mos7840_serial->device_type;
	if ((mos7840_serial->parallelport || device_type == MOSCHIP_DEVICE_ID_7820) && tmp > 0 ) 
		val |= (tmp + 2) << 8;
	else 
		val |= (tmp + 1) << 8;
	
DPRINTK("mos7840_set_Uart_Reg application number is %x\n", val);
        return usb_control_msg(dev, usb_sndctrlpipe(dev, 0), MCS_WRREQ,
                        MCS_WR_RTYPE, val, reg, NULL, 0, MOS_WDR_TIMEOUT);
}


/*
 * Description:- To set the Control register by calling usb_fill_control_urb function by passing usb_rcvctrlpipe function as parameter.
 *
 * Input Parameters:
 * usb_serial_port:  Data Structure usb_serialport correponding to that seril port.
 * Reg: Register Address
 * Val:  Value to receive from the Register.
 */
static int mos7840_get_Uart_Reg(struct usb_serial_port *port, __u16 reg, __u16 *val)
{
	struct usb_device *dev = port->serial->dev;
	struct moschip_serial *mos7840_serial;
	int 	tmp = 0;        
        int 	ret=0;
        __u16 	Wval;	
	u8 *	pData = mos7840_get_port_private(port)->reg_buffer;
	int 	device_type = 0;

	if (pData == NULL)
		return -1;

        mos7840_serial = mos7840_get_serial_private(port->serial);
	if (mos7840_serial == NULL)
		return -1;			
	device_type = mos7840_serial->device_type;

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,11,0)
	tmp = ((__u16)port->number - (__u16)(port->serial->minor));
#else
	tmp = (__u16)port->port_number;
#endif	
	/*Wval  is same as application number*/
	if ((mos7840_serial->parallelport || device_type == MOSCHIP_DEVICE_ID_7820) && tmp > 0 )
		Wval = (tmp + 2) << 8;	
	else 
		Wval = (tmp + 1) << 8;
	
DPRINTK("mos7840_set_Uart_Reg application number is %x\n", val);
	ret = usb_control_msg(dev, usb_rcvctrlpipe(dev, 0), MCS_RDREQ,
                        MCS_RD_RTYPE, Wval, reg, pData, VENDOR_READ_LENGTH, MOS_WDR_TIMEOUT);
	*val = *pData;
	*val = (*val) & 0x00ff;

        return ret;
}

void mos7840_Dump_serial_port(struct moschip_port *mos7840_port)
{
	DPRINTK("***************************************\n");
	DPRINTK("Application number is %4x\n",mos7840_port->AppNum);
	DPRINTK("SpRegOffset is %2x\n",mos7840_port->SpRegOffset);
	DPRINTK("ControlRegOffset is %2x \n",mos7840_port->ControlRegOffset);	
	DPRINTK("DCRRegOffset is %2x \n",mos7840_port->DcrRegOffset);	
	DPRINTK("***************************************\n");
}

/* all structre defination goes here */
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,5,0) 
static struct usb_driver io_driver = {
        .name =         DRIVER_DESC,
        .probe =        usb_serial_probe,
        .disconnect =   usb_serial_disconnect,
        .id_table =     id_table_combined,
}; 
#endif

/****************************************************************************
 * moschip7840_4port_device
 *              Structure defining AX78140/AX78120/MCS7840/MCS7820/MCS7810, usb serial device
 ****************************************************************************/
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,15) 
static struct usb_serial_driver moschip7840_4port_device = {
	.driver			= {
					.owner  = THIS_MODULE,
					.name   = DRIVER_DESC,
				},
	.description		= DRIVER_DESC,
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,15)
static struct usb_serial_device_type moschip7840_4port_device = {
	.owner			= THIS_MODULE,
	.name			= DRIVER_DESC,
	.short_name		= "AX781x0/MCS78x0",
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,21) && LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0) 
	.usb_driver 		= &io_driver, 
#endif

	.id_table		= moschip_port_id_table,

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)
	.num_interrupt_in	= 1,
#endif
	#ifdef check
	.num_bulk_in		= 4,
	.num_bulk_out		= 4,
	.num_ports		= 4,
	#endif
	.open			= mos7840_open,
	.close			= mos7840_close,
	.write			= mos7840_write,
	.write_room		= mos7840_write_room,
	.chars_in_buffer	= mos7840_chars_in_buffer,
	.throttle		= mos7840_throttle,
	.unthrottle		= mos7840_unthrottle,
	.calc_num_ports		= mos7840_calc_num_ports,

#ifdef MCSSerialProbe
	.probe			= mos7840_serial_probe,
#endif
	.ioctl			= mos7840_ioctl,
	.set_termios		= mos7840_set_termios,
	.break_ctl		= mos7840_break,
	.tiocmget		= mos7840_tiocmget,
	.tiocmset		= mos7840_tiocmset,
	.attach			= mos7840_startup,
	.release		= mos7840_release,
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,30) 
	.shutdown		= mos7840_shutdown,
#else
	.disconnect		= mos7840_shutdown,
#endif
	.read_bulk_callback	= mos7840_bulk_in_callback, 
	.read_int_callback	= mos7840_interrupt_callback, 
};

/************************************************************************/
/************************************************************************/
/*            U S B  C A L L B A C K   F U N C T I O N S                */
/*            U S B  C A L L B A C K   F U N C T I O N S                */
/************************************************************************/
/************************************************************************/

/*****************************************************************************
 * mos7840_interrupt_callback
 *	this is the callback function for when we have received data on the 
 *	interrupt endpoint.
 * Input : 1 Input
 *			pointer to the URB packet,
 *
 *****************************************************************************/
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,19)
static void mos7840_interrupt_callback (struct urb *urb)
#else
static void mos7840_interrupt_callback (struct urb *urb,struct pt_regs *regs)
#endif
{
	struct moschip_port   	*mos7840_port = NULL;
	struct moschip_serial 	*mos7840_serial = NULL;
	struct usb_serial 	*serial = NULL;
	int 	tmp = 0;
	int 	result;
	int 	length;	
	__u16 	Data;
	u8 *	data;
	__u8 	sp[5],st;	//sp: serial port status
	int 	i;
	__u16 	wval;
	int 	device_type;
	
DPRINTK("%s"," : Entering\n");

	mos7840_serial = (struct moschip_serial *)urb->context;
	if (mos7840_serial == NULL) {
		DPRINTK("%s - Invalid Pointer !!!!:\n", __FUNCTION__);
		return;
	}
	device_type = mos7840_serial->device_type;

	switch (urb->status) {
	case 0:
		/* success */
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		/* this urb is terminated, clean up */
		DPRINTK("%s - urb shutting down with status: %d", __FUNCTION__, urb->status);
		return;
	default:
		DPRINTK("%s - nonzero urb status received: %d", __FUNCTION__, urb->status);
		goto exit;
	}

	length 	= urb->actual_length;
	data 	= urb->transfer_buffer;
	serial 	= mos7840_serial->serial;

	/* AX78140/AX78120/MCS7840/MCS7820/MCS7810 get 5 bytes 
	 * Byte 1 IIR Port 1 (port.number is 0)
	 * Byte 2 IIR Port 2 (port.number is 1)
	 * Byte 3 IIR Port 3 (port.number is 2)
	 * Byte 4 IIR Port 4 (port.number is 3)
	 * Byte 5 FIFO status for both */

	if (length && length > 5) {
		DPRINTK("%s \n","Wrong data !!!");
		return;
	}

	/* MATRIX */
	sp[0] 	= (__u8)data[0];	
	sp[1] 	= (__u8)data[1];	
	sp[2] 	= (__u8)data[2];	
	sp[3] 	= (__u8)data[3];	
	st 	= (__u8)data[4];

	for (i = 0; i < serial->num_ports; i++) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,11,0)
		tmp = ((__u16)serial->port[i]->number - (__u16)(serial->minor));
#else
		tmp = (__u16)serial->port[i]->port_number;
#endif
		mos7840_port = mos7840_get_port_private(serial->port[i]);
		if (mos7840_port == NULL)
			return;
			
		if ((mos7840_serial->parallelport || device_type == MOSCHIP_DEVICE_ID_7820) && tmp > 0 )
			wval = (tmp + 2) << 8;	
		else
			wval = (tmp + 1) << 8;	
		
		if (mos7840_port->open != FALSE) {
			if (sp[i] & 0x01) {
				DPRINTK("SP%d No Interrupt !!!\n",i);
			} else {
				switch(sp[i] & 0x0f){
				case SERIAL_IIR_RLS: 
					DPRINTK("Serial Port %d: Receiver status error or ",i);
					DPRINTK("address bit detected in 9-bit mode\n");
			     		mos7840_port->MsrLsr = 1;
			     		mos7840_get_reg(mos7840_port, wval, LINE_STATUS_REGISTER, &Data);
			     		break;
            			case SERIAL_IIR_MS:  
			     		DPRINTK("Serial Port %d: Modem status change\n", i);
			     		mos7840_port->MsrLsr = 0;	
			     		mos7840_get_reg(mos7840_port, wval, MODEM_STATUS_REGISTER, &Data);	
			     		break;
				}
			}
		}
	}
exit:
	if ( mos7840_serial->status_polling_started == FALSE )
		return;

	result = usb_submit_urb (urb, GFP_ATOMIC);
	if (result) {
		dev_err(&urb->dev->dev, "%s - Error %d submitting interrupt urb\n", __FUNCTION__, result);
	}

	return;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,19)
static void mos7840_control_callback(struct urb *urb)
#else
static void mos7840_control_callback(struct urb *urb, struct pt_regs *regs)
#endif
{
	struct moschip_port *mos7840_port = NULL;
	u8 *	data;	
	__u8 	regval = 0;

	if (!urb) {
               DPRINTK("%s - Invalid Pointer !!!!:\n", __FUNCTION__);
               return;
        }

        switch (urb->status){
	case 0:
		/* success */
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		/* this urb is terminated, clean up */
		DPRINTK("%s - urb shutting down with status: %d", __FUNCTION__, urb->status);
		return;
	default:
		DPRINTK("%s - nonzero urb status received: %d", __FUNCTION__, urb->status);
		goto exit;
        }

	mos7840_port = (struct moschip_port *)urb->context;
	
DPRINTK("%s urb buffer size is %d\n", __FUNCTION__, urb->actual_length);
DPRINTK("%s mos7840_port->MsrLsr is %d port %d\n", __FUNCTION__, mos7840_port->MsrLsr, mos7840_port->port_num);
	data = urb->transfer_buffer;

	regval = (__u8)data[0];
	mos7840_port->shadowMSR = regval;
DPRINTK("%s data is %x\n", __FUNCTION__, regval);

	if (mos7840_port->MsrLsr == 0)
		handle_newMsr(mos7840_port, regval);
	else if (mos7840_port->MsrLsr == 1)
		handle_newLsr(mos7840_port, regval);
exit:
	return;
}

int handle_newMsr(struct moschip_port *port,__u8 newMsr)
{
	struct moschip_port *mos7840_port = NULL;
	struct async_icount *icount = NULL;

	mos7840_port = port;
	icount = &mos7840_port->icount;
	if (newMsr & (MOS_MSR_DELTA_CTS | MOS_MSR_DELTA_DSR | MOS_MSR_DELTA_RI | MOS_MSR_DELTA_CD)) {
	        icount = &mos7840_port->icount;

		/* update input line counters */
                if (newMsr & MOS_MSR_DELTA_CTS) {
                        icount->cts++;
                }
                if (newMsr & MOS_MSR_DELTA_DSR) {
                        icount->dsr++;
                }
                if (newMsr & MOS_MSR_DELTA_CD) {
                        icount->dcd++;
                }
                if (newMsr & MOS_MSR_DELTA_RI) {
                        icount->rng++;
                }
        }
	if (newMsr & MOS_MSR_CTS) {
		wake_up(&mos7840_port->delta_cts_wait);
		mos7840_port->delta_cts_cond = 1;
	} else {
		mos7840_port->delta_cts_cond = 0;
        }

	return 0;
}

int handle_newLsr(struct moschip_port *port,__u8 newLsr)
{
        struct  async_icount *icount = NULL;

DPRINTK("%s - %02x", __FUNCTION__, newLsr);

        if (newLsr & SERIAL_LSR_BI) {
		//
                // Parity and Framing errors only count if they
                // occur exclusive of a break being
                // received.
                //
                newLsr &= (__u8)(SERIAL_LSR_OE | SERIAL_LSR_BI);
        }

        /* update input line counters */
        icount = &port->icount;
        if (newLsr & SERIAL_LSR_BI) {
                icount->brk++;
        }
	if (newLsr & SERIAL_LSR_OE) {
                icount->overrun++;
        }
        if (newLsr & SERIAL_LSR_PE) {
                icount->parity++;
        }
        if (newLsr & SERIAL_LSR_FE) {
                icount->frame++;
        }

	return 0;
}

static int mos7840_get_reg(struct moschip_port *mcs,__u16 Wval, __u16 reg, __u16 * val)
{
        struct usb_device *dev = mcs->port->serial->dev;
        struct usb_ctrlrequest *dr = NULL;
        __u8 *	buffer = NULL;
        int 	ret = 0;

        buffer = (__u8 *)mcs->ctrl_buf;

	//dr = (struct usb_ctrlrequest *)(buffer);
        dr = (void *)(buffer + 2);
        dr->bRequestType = MCS_RD_RTYPE;
        dr->bRequest = MCS_RDREQ;
        dr->wValue = cpu_to_le16(Wval);//0;
        dr->wIndex = cpu_to_le16(reg);
        dr->wLength = cpu_to_le16(2);

        usb_fill_control_urb(mcs->control_urb, dev, usb_rcvctrlpipe(dev, 0), (unsigned char *)dr, buffer, 2, mos7840_control_callback, mcs);
        mcs->control_urb->transfer_buffer_length = 2;
        ret = usb_submit_urb(mcs->control_urb, GFP_ATOMIC);
        return ret;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,19)
static void mos7840_set_led_callback(struct urb *urb)
#else
static void mos7840_set_led_callback(struct urb *urb, struct pt_regs *regs)
#endif
{
	if (!urb) {
		DPRINTK("%s","Invalid Pointer !!!!:\n");
		return;
        }

        switch (urb->status) {
	case 0:
		/* success */
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		/* this urb is terminated, clean up */
		DPRINTK("%s - urb shutting down with status: %d", __FUNCTION__, urb->status);                        
		return;
	default:
		DPRINTK("%s - nonzero urb status received: %d", __FUNCTION__, urb->status);
		goto exit;
        }
exit:
	return;	
}

static int mos7840_set_led_async(struct moschip_port *mcs, __u16 Wval, __u16 reg)
{
	struct usb_device *dev = mcs->port->serial->dev;
	struct usb_ctrlrequest *dr = NULL;

	if ((dr = kmalloc (sizeof(struct usb_ctrlrequest), GFP_ATOMIC)) == NULL)
		return 0;

	dr->bRequestType = MCS_WR_RTYPE;
	dr->bRequest = MCS_WRREQ;
	dr->wValue = cpu_to_le16(Wval);
	dr->wIndex = cpu_to_le16(reg);
	dr->wLength = cpu_to_le16(0);

	usb_fill_control_urb(mcs->control_urb, dev, usb_sndctrlpipe(dev, 0),
		(unsigned char *)dr, NULL, 0, mos7840_set_led_callback, NULL);
	
	return usb_submit_urb(mcs->control_urb, GFP_ATOMIC);
}

static void mos7840_set_led_sync(struct usb_serial_port *port, __u16 reg, __u16 val)
{
	struct usb_device *dev = port->serial->dev;

	usb_control_msg(dev, usb_sndctrlpipe(dev, 0), MCS_WRREQ, MCS_WR_RTYPE,
			val, reg, NULL, 0, MOS_WDR_TIMEOUT);
}
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
static void mos7840_led_off (unsigned long arg)
#else
static void mos7840_led_off (struct timer_list *t)
#endif
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
	struct moschip_port *mcs = (struct moschip_port *) arg;
#else
	struct moschip_port *mcs = from_timer(mcs, t, led_timer1);
#endif	
	if (!mcs)
		return;

	// Turn off MCS7810 LED 
	mos7840_set_led_async(mcs, 0x0300, MODEM_CONTROL_REGISTER);
	mod_timer(&mcs->led_timer2, jiffies + msecs_to_jiffies(MCS7810_LED_OFF_MS));
}
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
static void mos7840_led_flag_off (unsigned long arg)
#else
static void mos7840_led_flag_off (struct timer_list *t)
#endif
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
	struct moschip_port *mcs = (struct moschip_port *) arg;
#else
	struct moschip_port *mcs = from_timer(mcs, t, led_timer2);
#endif	
	if (!mcs)
		return;

	mcs->led_flag = false;
}

/*****************************************************************************
 * mos7840_bulk_in_callback
 *	this is the callback function for when we have received data on the 
 *	bulk in endpoint.
 * Input : 1 Input
 *			pointer to the URB packet,
 *
 *****************************************************************************/
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,19)
static void mos7840_bulk_in_callback (struct urb *urb)
#else
static void mos7840_bulk_in_callback (struct urb *urb, struct pt_regs *regs)
#endif
{
	int			status;
	unsigned char		*data ;
	struct usb_serial	*serial;
	struct usb_serial_port	*port;
	struct moschip_serial	*mos7840_serial;
	struct moschip_port	*mos7840_port;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,9,0)
	struct tty_port *tty;
#else
	struct tty_struct *tty;
#endif

	#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,15)
	int i;
	#endif
	
	if (!urb) {
		DPRINTK("%s - Invalid Pointer !!!!:\n", __FUNCTION__);
		return;
	}

	if (urb->status) {
		DPRINTK("nonzero read bulk status received: %d",urb->status);
		return;
	}

	mos7840_port = (struct moschip_port*)urb->context;
	if (!mos7840_port) {
		DPRINTK("%s","NULL mos7840_port pointer \n");
		return ;
	}

	port = (struct usb_serial_port *)mos7840_port->port;
	if (mos7840_port_paranoia_check (port, __FUNCTION__)) {
		DPRINTK("%s","Port Paranoia failed \n");
		return;
	}

	serial = mos7840_get_usb_serial(port, __FUNCTION__);	
	if (!serial) {
		DPRINTK("%s\n","Bad serial pointer ");
		return;
	}

	DPRINTK("%s\n","Entering... \n");

	data = urb->transfer_buffer;
	mos7840_serial = mos7840_get_serial_private(serial);	

	if (urb->actual_length) {

		if (urb->actual_length == 1) {
			if (data[0] == 0x13)
                                mos7840_port->xoff_cond = 0;
                        else if (data[0] == 0x11) {
				wake_up(&mos7840_port->xoff_wait);
				mos7840_port->xoff_cond = 1;
			}
                }

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27) &&  LINUX_VERSION_CODE > KERNEL_VERSION(2,6,15)
		// 2.6.17 Block
		tty = mos7840_port->port->tty;
		if (tty) {
			tty_buffer_request_room(tty, urb->actual_length);
			tty_insert_flip_string(tty, data, urb->actual_length);
			DPRINTK(" %s \n",data);
			tty_flip_buffer_push(tty);
		}

#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27) && LINUX_VERSION_CODE < KERNEL_VERSION(3,9,0)

       		tty = mos7840_port->port->port.tty;
                if (tty) {
			tty_buffer_request_room(tty, urb->actual_length);
			tty_insert_flip_string(tty, data, urb->actual_length);
			DPRINTK(" %s \n",data);
			tty_flip_buffer_push(tty);
                }

#elif LINUX_VERSION_CODE >= KERNEL_VERSION(3,9,0)

		tty = mos7840_port->port->port.tty->port;

                if (tty) {
			tty_buffer_request_room(tty, urb->actual_length);
			tty_insert_flip_string(tty, data, urb->actual_length);
      DPRINTK(" %s \n",data);
			tty_flip_buffer_push(tty);
                }
                
#else
		tty = mos7840_port->port->tty;
                if (tty) {
			for (i = 0; i < urb->actual_length; ++i) {
                        	/* if we insert more than TTY_FLIPBUF_SIZE characters, we drop them. */
                                if (tty->flip.count >= TTY_FLIPBUF_SIZE) {
					tty_flip_buffer_push(tty);
                                }
                        	/* this doesn't actually push the data through unless tty->low_latency is set */
                                tty_insert_flip_char(tty, data[i], 0);
				DPRINTK(" %c \n",data[i]);
                        }
			tty_flip_buffer_push(tty);
                }
#endif
		mos7840_port->icount.rx += urb->actual_length;
		DPRINTK("mos7840_port->icount.rx is %d:\n", mos7840_port->icount.rx);
	}

	if (!mos7840_port->read_urb) {
		DPRINTK("%s","URB KILLED !!!\n");
		return;
	}
	/* Turn on MCS7810 LED */
	if (mos7840_port->has_led && !mos7840_port->led_flag){
		mos7840_port->led_flag = true;
		mos7840_set_led_async(mos7840_port, 0x0301, MODEM_CONTROL_REGISTER);
		mod_timer(&mos7840_port->led_timer1, jiffies + msecs_to_jiffies(MCS7810_LED_ON_MS));
	}

	if (mos7840_port->read_urb->status != -EINPROGRESS) {
		mos7840_port->read_urb->dev = serial->dev;

		status = usb_submit_urb(mos7840_port->read_urb, GFP_ATOMIC);

		if (status) 
			DPRINTK(" usb_submit_urb(read bulk) failed, status = %d", status);

	}
}

/*****************************************************************************
 * mos7840_bulk_out_data_callback
 *	this is the callback function for when we have finished sending serial data
 *	on the bulk out endpoint.
 * Input : 1 Input
 *			pointer to the URB packet,
 *
 *****************************************************************************/
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,19)
static void mos7840_bulk_out_data_callback (struct urb *urb)
#else
static void mos7840_bulk_out_data_callback (struct urb *urb, struct pt_regs *regs)
#endif
{
	struct moschip_port *mos7840_port;
	struct tty_struct *tty;
	unsigned long flags;
	int i;

	if (!urb) {
		DPRINTK("%s","Invalid Pointer !!!!:\n");
		return;
	}

	if (urb->status) {
		DPRINTK("nonzero write bulk status received:%d\n", urb->status);
		return;
	}

	mos7840_port = (struct moschip_port *)urb->context;
	if (!mos7840_port) {
		DPRINTK("%s","NULL mos7840_port pointer \n");
		return ;
	}

	if (mos7840_port_paranoia_check (mos7840_port->port, __FUNCTION__)) {
		DPRINTK("%s","Port Paranoia failed \n");
		return;
	}

	DPRINTK("%s \n","Entering .........");

	spin_lock_irqsave(&mos7840_port->pool_lock, flags);
	for (i = 0; i < NUM_URBS; i++) {
		if (urb == mos7840_port->write_urb_pool[i]) {
			mos7840_port->busy[i] = 0;
			break;
		}
	}
	spin_unlock_irqrestore(&mos7840_port->pool_lock, flags);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
	tty = mos7840_port->port->tty;
#else
	tty = mos7840_port->port->port.tty;
#endif
	
	if (tty && mos7840_port->open) {
		/* let the tty driver wakeup if it has a special *
		 * write_wakeup function 			 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
		if ((tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) && tty->ldisc.write_wakeup) {
			(tty->ldisc.write_wakeup)(tty);
#else
	#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,30)
		if ((tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) && tty->ldisc.ops->write_wakeup) {
			(tty->ldisc.ops->write_wakeup)(tty);
	#else
		if ((tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) && tty->ldisc->ops->write_wakeup) {
			(tty->ldisc->ops->write_wakeup)(tty);
	#endif
#endif
		}
		/* tell the tty driver that something has changed */
		wake_up_interruptible(&tty->write_wait);
	}
	/* Release the Write URB */
	mos7840_port->write_in_progress = FALSE;
}

/************************************************************************/
/*       D R I V E R  T T Y  I N T E R F A C E  F U N C T I O N S       */
/************************************************************************/
#ifdef MCSSerialProbe
static int mos7840_serial_probe(struct usb_serial *serial, const struct usb_device_id *id)
{
	/*need to implement the mode_reg reading and updating\
			 structures usb_serial_ device_type\
			(i.e num_ports, num_bulkin,bulkout etc)*/
	/* Also we can update the changes  attach */
	return 1;
}
#endif

/*****************************************************************************
 * SerialOpen
 *	this function is called by the tty driver when a port is opened
 *	If successful, we return 0
 *	Otherwise we return a negative error number.
 *****************************************************************************/
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
	static int mos7840_open (struct tty_struct *tty, struct usb_serial_port *port)
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
	static int mos7840_open (struct usb_serial_port *port, struct file *filp)
#else
	static int mos7840_open (struct tty_struct *tty, struct usb_serial_port *port, struct file *filp)
#endif
{
	int 	tmp = 0;
	int 	response;
	int 	j;	
	int 	device_type;
	__u16 	Data;
	int 	status;
	struct urb 		*urb = NULL;
	struct usb_serial 	*serial = NULL;		
	struct moschip_serial 	*mos7840_serial = NULL;
	struct moschip_port 	*mos7840_port = NULL;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,19)
	struct ktermios tmp_termios;
#else
	struct termios tmp_termios;
#endif

	if (mos7840_port_paranoia_check (port, __FUNCTION__)) {
		DPRINTK("%s","Port Paranoia failed \n");
		return -ENODEV;
	}

	serial = port->serial;

	if (mos7840_serial_paranoia_check (serial, __FUNCTION__)) {
		DPRINTK("%s","Serial Paranoia failed \n");
		return -ENODEV;
	}

	mos7840_port = mos7840_get_port_private(port); 

	if (mos7840_port == NULL)
		return -ENODEV;

	mos7840_serial = mos7840_get_serial_private(serial);
	
	if (mos7840_serial == NULL ) {
		return -ENODEV;
	}	
	device_type = mos7840_serial->device_type;

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,11,0)
	tmp = ((__u16)port->number - (__u16)(port->serial->minor));
#else
	tmp = (__u16)port->port_number;
#endif	

	if (mos7840_serial->parallelport && tmp == 2)
		return -EACCES;

	/* Initialising the write urb pool */
	{
		u8 count = 0;
		for (j = 0; j < NUM_URBS; ++j) {
write_urb_start:
			if (count == 10)
				return -ENODEV;
			urb = usb_alloc_urb(0, GFP_ATOMIC);
															      
			if (urb == NULL) {
				DPRINTK("No more urbs???");
				count++;
				goto write_urb_start;
			}
			mos7840_port->write_urb_pool[j] = urb;
																															    
			urb->transfer_buffer = NULL;
			urb->transfer_buffer = kmalloc (URB_TRANSFER_BUFFER_SIZE, GFP_KERNEL);
			if (!urb->transfer_buffer) {
				DPRINTK("%s-out of memory for urb buffers.", __FUNCTION__);
				continue;
			}
		}
	}


/*****************************************************************************
 * Initialize AX78140/AX78120/MCS7840/MCS7820/MCS7810 -- Write Init values to corresponding Registers
 *
 * Register Index
 * 1 : IER
 * 2 : FCR
 * 3 : LCR
 * 4 : MCR
 *
 * 0x08 : SP1/2 Control Reg
 *****************************************************************************/

//NEED to check the fallowing Block
	status = 0;
	Data = 0x0;
	status = mos7840_get_reg_sync(port, mos7840_port->SpRegOffset, &Data);
	if (status < 0) {
		DPRINTK("Reading Spreg failed\n");
		return -1;
	}

	Data |= 0x80;
	status = mos7840_set_reg_sync(port, mos7840_port->SpRegOffset, Data);
	if (status < 0) {
		DPRINTK("writing Spreg failed\n");
		return -1;
	}
	
	Data &= ~0x80;
	status = mos7840_set_reg_sync(port, mos7840_port->SpRegOffset, Data);
	if (status < 0) {
		DPRINTK("writing Spreg failed\n");
		return -1;
	}

	
//Threshold setting
	if (threshold_en == 0) {
		Data = 0x00;
		status = mos7840_set_reg_sync(port, mos7840_port->SpThresholdOffset, Data);
		if (status < 0) {
			DPRINTK("writing ThresholdReg failed\n");
			return -1;
		}	
		Data = 0x00;
		status = mos7840_set_reg_sync(port, mos7840_port->SpThresholdOffset + 1, Data);
		if (status < 0) {
			DPRINTK("writing ThresholdReg failed\n");
			return -1;
		}
	} else {
		Data = threshold_val;
		status = mos7840_set_reg_sync(port, mos7840_port->SpThresholdOffset, Data);
		if (status < 0) {
			DPRINTK("writing ThresholdReg failed\n");
			return -1;
		}
		Data = 0x80;
		status = mos7840_set_reg_sync(port, mos7840_port->SpThresholdOffset + 1, Data);
		if (status < 0) {
			DPRINTK("writing ThresholdReg failed\n");
			return -1;
		}		
	}

//End of block to be checked	
//**************************CHECK***************************//

#ifdef AUTO_SWITCH
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,11,0)
	auto_switch(serial, mos7840_port->ioctl_rs_setting, port->number - port->serial->minor + 1);
#else
	auto_switch(serial, mos7840_port->ioctl_rs_setting, port->port_number + 1);
#endif
#endif
	
	/* SCRATCH_PAD_REGISTER */
	if (mos7840_port->rs_mode_port[mos7840_port->port_num - 1] != 0) 
		Data = 0xC0;
	else if (mos7840_port->rs_mode_port[mos7840_port->port_num - 1] == 0) 
		Data = 0x00;	
        status = 0;
        status = mos7840_set_Uart_Reg(port, SCRATCH_PAD_REGISTER, Data);
        if (status < 0) {
		DPRINTK("Writing SCRATCH_PAD_REGISTER failed status-0x%x\n", status);
		return -1;
        } else 
		DPRINTK("SCRATCH_PAD_REGISTER Writing success status%d\n", status);

//**************************CHECK***************************//
		
	status = 0;
	Data = 0x0;
	status = mos7840_get_reg_sync(port, mos7840_port->ControlRegOffset, &Data);
	if (status < 0) {
		DPRINTK("Reading Controlreg failed\n");
		return -1;
	}
	Data |= 0x08;//Driver done bit
	Data |= 0x20;//rx_disable

	if (hw_auto_fc == 1) 
		Data |= 0x01;
	else if (hw_auto_fc == 0)
		Data &= ~0x01;
	

	status = 0;
	status = mos7840_set_reg_sync(port, mos7840_port->ControlRegOffset, Data);
	if (status < 0) {
		DPRINTK("writing Controlreg failed\n");
		return -1;
	}

	//do register settings here
	// Set all regs to the device default values.
	////////////////////////////////////
	// First Disable all interrupts.
	////////////////////////////////////
	
	Data = 0x00;
	status = 0;
	status = mos7840_set_Uart_Reg(port, INTERRUPT_ENABLE_REGISTER, Data);
	if (status < 0) {
		DPRINTK("disableing interrupts failed\n");
		return -1;
	}
	 // Set FIFO_CONTROL_REGISTER to the default value 
	Data = 0x00;
	status = 0;
	status = mos7840_set_Uart_Reg(port, FIFO_CONTROL_REGISTER, Data);
	if (status < 0) {
		DPRINTK("Writing FIFO_CONTROL_REGISTER  failed\n");
		return -1;
	}

	Data = 0xCF;
	status = 0;
	status = mos7840_set_Uart_Reg(port, FIFO_CONTROL_REGISTER, Data);
	if (status < 0) {
		DPRINTK("Writing FIFO_CONTROL_REGISTER  failed\n");
		return -1;
	}

	Data = 0x03; //LCR_BITS_8
	status = 0;
	status = mos7840_set_Uart_Reg(port, LINE_CONTROL_REGISTER, Data);
	mos7840_port->shadowLCR=Data;

	Data = 0x0b; // MCR_DTR|MCR_RTS|MCR_MASTER_IE
	status = 0;
	status = mos7840_set_Uart_Reg(port, MODEM_CONTROL_REGISTER, Data);
	mos7840_port->shadowMCR=Data;

#ifdef Check
	Data = 0x00;
	status = 0;
	status = mos7840_get_Uart_Reg(port, LINE_CONTROL_REGISTER, &Data);
	mos7840_port->shadowLCR=Data;

	Data |= SERIAL_LCR_DLAB; //data latch enable in LCR 0x80
	status = 0;
	status = mos7840_set_Uart_Reg(port, LINE_CONTROL_REGISTER, Data);
	
	Data = 0x0c;
	status = 0;
	status = mos7840_set_Uart_Reg(port, DIVISOR_LATCH_LSB, Data);
	
	Data = 0x0;
	status = 0;
	status = mos7840_set_Uart_Reg(port, DIVISOR_LATCH_MSB, Data);

	Data = 0x00;
	status = 0;
	status = mos7840_get_Uart_Reg(port, LINE_CONTROL_REGISTER, &Data);

//	Data = mos7840_port->shadowLCR; //data latch disable
	Data = Data & ~SERIAL_LCR_DLAB;
	status = 0;
	status = mos7840_set_Uart_Reg(port, LINE_CONTROL_REGISTER, Data);
	mos7840_port->shadowLCR=Data;
#endif
	//clearing Bulkin and Bulkout Fifo
	Data = 0x0;
	status = 0;
	status = mos7840_get_reg_sync(port, mos7840_port->SpRegOffset, &Data);
	
	Data = Data | 0x0c;
	status = 0;
        status = mos7840_set_reg_sync(port, mos7840_port->SpRegOffset, Data);
	  
	Data = Data & ~0x0c;
	status = 0;
        status = mos7840_set_reg_sync(port, mos7840_port->SpRegOffset, Data);
	//Finally enable all interrupts
	Data = 0x0;
	Data = 0x0c;
	status = 0;
        status = mos7840_set_Uart_Reg(port, INTERRUPT_ENABLE_REGISTER, Data);

	//clearing rx_disable
	Data = 0x0;
	status = 0;
        status = mos7840_get_reg_sync(port, mos7840_port->ControlRegOffset, &Data);
	Data = Data & ~0x20;
	status = 0;
        status = mos7840_set_reg_sync(port, mos7840_port->ControlRegOffset, Data);

	// rx_negate
	Data = 0x0;
	status = 0;
        status = mos7840_get_reg_sync(port, mos7840_port->ControlRegOffset, &Data);
	Data = Data |0x10;
	status = 0;
        status = mos7840_set_reg_sync(port, mos7840_port->ControlRegOffset, Data);


	/* force low_latency on so that our tty_push actually forces *
	 * the data through,otherwise it is scheduled, and with      *
	 * high data rates (like with OHCI) data can get lost.       */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27) 
	if (port->tty)
		port->tty->low_latency = 1;
#elif LINUX_VERSION_CODE < KERNEL_VERSION(3,9,0)
	if (tty)
		tty->low_latency = 1;
#elif LINUX_VERSION_CODE < KERNEL_VERSION(5,12,0)
	if (tty)
		tty->port->low_latency = 1;
#endif

///////////////////////
    	/* see if we've set up our endpoint info yet   *
	 * (can't set it up in mos7840_startup as the  *
	 * structures were not set up at that time.)   */

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,11,0)
	DPRINTK("port number is %d \n",port->number);
#else
	DPRINTK("port number is %d \n",port->port_number);
#endif	
	//DPRINTK("serial number is %d \n",port->serial->minor);
	DPRINTK("Bulkin endpoint is %d \n",port->bulk_in_endpointAddress);
	DPRINTK("BulkOut endpoint is %d \n",port->bulk_out_endpointAddress);
	DPRINTK("Interrupt endpoint is %d \n",port->interrupt_in_endpointAddress);
	DPRINTK("port's number in the device is %d\n",mos7840_port->port_num);
	mos7840_port->bulk_in_buffer    = port->bulk_in_buffer;
    	mos7840_port->bulk_in_endpoint  = port->bulk_in_endpointAddress;
	mos7840_port->read_urb          = port->read_urb;
	mos7840_port->bulk_out_endpoint = port->bulk_out_endpointAddress;

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,11,0)
	tmp = ((__u16)port->number - (__u16)(port->serial->minor));
#else
	tmp = (__u16)port->port_number;
#endif
	/* set up our bulk in urb */
		
	if ((mos7840_serial->parallelport || device_type == MOSCHIP_DEVICE_ID_7820) && tmp > 0 ) {
        	usb_fill_bulk_urb(
                	mos7840_port->read_urb,serial->dev,\
	        	usb_rcvbulkpipe(serial->dev, (port->bulk_in_endpointAddress + 2)),\
  			port->bulk_in_buffer,\
                	mos7840_port->read_urb->transfer_buffer_length,         \
                	mos7840_bulk_in_callback,mos7840_port);	
	} else {
		usb_fill_bulk_urb(  
			mos7840_port->read_urb, 				\
			serial->dev,						\
			usb_rcvbulkpipe(serial->dev, port->bulk_in_endpointAddress),\
			port->bulk_in_buffer,					\
			mos7840_port->read_urb->transfer_buffer_length,		\
			mos7840_bulk_in_callback,mos7840_port);	
	}
	
	DPRINTK("mos7840_open: bulkin endpoint is %d\n", port->bulk_in_endpointAddress);

	response = usb_submit_urb (mos7840_port->read_urb, GFP_KERNEL);
	if (response)
                DPRINTK("%s - Error %d submitting control urb", __FUNCTION__, response);

        /* initialize our wait queues */
        init_waitqueue_head(&mos7840_port->wait_open);
        init_waitqueue_head(&mos7840_port->wait_chase);
        init_waitqueue_head(&mos7840_port->delta_msr_wait);
        init_waitqueue_head(&mos7840_port->delta_cts_wait);
        mos7840_port->delta_cts_cond = 1;
        init_waitqueue_head(&mos7840_port->xoff_wait);
        mos7840_port->xoff_cond = 1;
        init_waitqueue_head(&mos7840_port->wait_command);

	mos7840_port->flow_ctrl_type = 0;

        /* initialize our icount structure */
        memset (&(mos7840_port->icount), 0x00, sizeof(mos7840_port->icount));

        /* initialize our port settings */
        mos7840_port->shadowMCR  = MCR_MASTER_IE; /* Must set to enable ints! */
        mos7840_port->chaseResponsePending = FALSE; 
        /* send a open port command */
        mos7840_port->openPending = FALSE;
        mos7840_port->open        = TRUE; 
	/* Setup termios */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
        if (port->tty) {
                mos7840_set_termios (port, &tmp_termios);
        }
#else
	if (tty) {
                mos7840_set_termios (tty, port, &tmp_termios);
        }
#endif
        mos7840_port->rxBytesAvail = 0x0;
	mos7840_port->icount.tx = 0;
	mos7840_port->icount.rx = 0;

	//DPRINTK("\n\nusb_serial serial:%x	mos7840_port:%x\nmos7840_serial:%x	usb_serial_port port:%x\n\n",
	//	(unsigned int)serial,(unsigned int)mos7840_port,(unsigned int)mos7840_serial,(unsigned int)port);
	
	        return 0;
}


/*****************************************************************************
 * mos7840_close
 *	this function is called by the tty driver when a port is closed
 *****************************************************************************/
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
static void mos7840_close (struct usb_serial_port *port, struct file * filp)
#elif (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,31))
static void mos7840_close(struct tty_struct *tty,struct usb_serial_port *port, struct file *filp)
#else
static void mos7840_close (struct usb_serial_port *port)
#endif
{
	struct usb_serial 	*serial = NULL;
	struct moschip_serial 	*mos7840_serial = NULL;
	struct moschip_port 	*mos7840_port = NULL;
	int	no_urbs , tmp = 0;
	__u16	Data;

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,30)
	struct tty_struct *tty=port->port.tty;
#endif

	DPRINTK("%s\n","mos7840_close:entering...");

	if (mos7840_port_paranoia_check (port, __FUNCTION__)) {
		DPRINTK("%s","Port Paranoia failed \n");
		return;
	}

	serial = mos7840_get_usb_serial (port, __FUNCTION__);
	if (!serial) {
		DPRINTK("%s","Serial Paranoia failed \n");
		return;
	}

	// take the Adpater and port's private data
	mos7840_serial = mos7840_get_serial_private(serial);
	mos7840_port = mos7840_get_port_private(port);
	
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,11,0)
	tmp = ((__u16)port->number - (__u16)(port->serial->minor));
#else
	tmp = (__u16)port->port_number;
#endif	
	
	if (mos7840_serial->parallelport && tmp == 2)
		return ;
	
	if ((mos7840_serial == NULL) || (mos7840_port == NULL)) {
		printk("%s  mos7840_serial or mos7840_port are Null\n", __FUNCTION__);
		return;
	}

	if (serial->dev) {
		/* flush and block(wait) until tx is empty*/
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
		mos7840_block_until_tx_empty(mos7840_port);
#else
		mos7840_block_until_tx_empty(tty, mos7840_port);
#endif
	}

	// kill the ports URB's
	for (no_urbs = 0; no_urbs < NUM_URBS; no_urbs++)
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,10)
 		usb_kill_urb (mos7840_port->write_urb_pool[no_urbs]);
#else
		usb_unlink_urb (mos7840_port->write_urb_pool[no_urbs]);
#endif
	/* Freeing Write URBs*/
	for (no_urbs = 0; no_urbs< NUM_URBS; ++no_urbs) {
        	if (mos7840_port->write_urb_pool[no_urbs]) {
                	if (mos7840_port->write_urb_pool[no_urbs]->transfer_buffer)
                        	kfree(mos7840_port->write_urb_pool[no_urbs]->transfer_buffer);
                	usb_free_urb (mos7840_port->write_urb_pool[no_urbs]);
                }
        }

	/* While closing port, shutdown all bulk read, write  *
	 * and interrupt read if they exists                  */
	if (serial->dev) {
		if (mos7840_port->write_urb) {
			DPRINTK("%s","Shutdown bulk write\n");
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,10)
			usb_kill_urb (mos7840_port->write_urb);
#else
			usb_unlink_urb (mos7840_port->write_urb);
#endif
		}

		if (mos7840_port->read_urb) {
			DPRINTK("%s","Shutdown bulk read\n");
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,10)
			usb_kill_urb (mos7840_port->read_urb);
#else
			usb_unlink_urb (mos7840_port->read_urb);
#endif
		}

		if ((&mos7840_port->control_urb)) {
			DPRINTK("%s","Shutdown control read\n");
		}
	}

	if (mos7840_port->write_urb) {
		/* if this urb had a transfer buffer already (old tx) free it */
		if (mos7840_port->write_urb->transfer_buffer != NULL) {
			kfree(mos7840_port->write_urb->transfer_buffer);
		}
		usb_free_urb(mos7840_port->write_urb);
	}

	// clear the MCR & IER
	Data = 0x00;
	mos7840_set_Uart_Reg(port, MODEM_CONTROL_REGISTER, Data);
	Data = 0x00;
	mos7840_set_Uart_Reg(port, INTERRUPT_ENABLE_REGISTER, Data);
	
	// mos7840_get_Uart_Reg(port,MODEM_CONTROL_REGISTER,&Data1);
	mos7840_port->open         = FALSE;
	mos7840_port->closePending = FALSE;
	mos7840_port->openPending  = FALSE;
}   


/*****************************************************************************
 * SerialBreak
 *	this function sends a break to the port
 *****************************************************************************/
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
static void mos7840_break (struct usb_serial_port *port, int break_state)
#else
static void mos7840_break(struct tty_struct *tty, int break_state)
#endif
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
	struct usb_serial_port *port = tty->driver_data;
#endif

        unsigned char data;
	struct usb_serial *serial;
	struct moschip_serial *mos7840_serial;
	struct moschip_port *mos7840_port;
	
	DPRINTK("%s \n","Entering ...........");
	DPRINTK("mos7840_break: Start\n");

	if (mos7840_port_paranoia_check (port, __FUNCTION__)) {
		DPRINTK("%s","Port Paranoia failed \n");
		return;
	}
		 
	serial = mos7840_get_usb_serial (port, __FUNCTION__);
	if (!serial) {
		DPRINTK("%s","Serial Paranoia failed \n");
		return;
	}

	mos7840_serial = mos7840_get_serial_private(serial);
	mos7840_port = mos7840_get_port_private(port);
	
	if ((mos7840_serial == NULL) || (mos7840_port == NULL))	{
		return;
	}
	
	/* flush and chase */
	mos7840_port->chaseResponsePending = TRUE;

	if (serial->dev) {
		/* flush and block until tx is empty*/
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)	
		mos7840_block_until_chase_response(mos7840_port);
#else
		mos7840_block_until_chase_response(tty,mos7840_port);
#endif
	}

        if (break_state == -1) 
                data = mos7840_port->shadowLCR | LCR_SET_BREAK;
        else 
                data = mos7840_port->shadowLCR & ~LCR_SET_BREAK;
        

        mos7840_port->shadowLCR  = data;
	DPRINTK("mcs7840_break mos7840_port->shadowLCR is %x\n",mos7840_port->shadowLCR);
	mos7840_set_Uart_Reg(port,LINE_CONTROL_REGISTER,mos7840_port->shadowLCR);

	return;
}


/************************************************************************
 *
 * mos7840_block_until_chase_response
 *
 *	This function will block the close until one of the following:
 *		1. Response to our Chase comes from mos7840
 *		2. A timout of 10 seconds without activity has expired
 *		   (1K of mos7840 data @ 2400 baud ==> 4 sec to empty)
 *
 ************************************************************************/
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
static void mos7840_block_until_chase_response(struct moschip_port *mos7840_port)
#else
static void mos7840_block_until_chase_response(struct tty_struct *tty,struct moschip_port *mos7840_port)
#endif
{
	int timeout = 1*HZ;
	int wait = 10;
	int count ;

	while (1) {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
		if (mos7840_port->port != NULL){
			count = mos7840_chars_in_buffer(mos7840_port->port);
		} else {
			printk("mos7840_port->port == NULL!!!\n");
			return;
		}
#else
		if (tty != NULL){
			count = mos7840_chars_in_buffer(tty);
		} else {
			printk("tty == NULL!!!\n");
			return;
		}
#endif
		/* Check for Buffer status */
		if (count <= 0) {
			mos7840_port->chaseResponsePending = FALSE;
			return;
		}

		/* Block the thread for a while */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,15,0))
		interruptible_sleep_on_timeout(&mos7840_port->wait_chase, timeout);
#else
		wait_event_interruptible_timeout(mos7840_port->wait_chase, 0, timeout);
#endif

                /* No activity.. count down section */
		wait--;
		if (wait == 0) {
			DPRINTK("%s - TIMEOUT", __FUNCTION__);
			return;
		}/* else 	{
	                // Reset timout value back to seconds
			wait = 10;
		}*/
	}
}


/************************************************************************
 *
 * mos7840_block_until_tx_empty
 *
 *	This function will block the close until one of the following:
 *		1. TX count are 0
 *		2. The mos7840 has stopped
 *		3. A timout of 3 seconds without activity has expired
 *
 ************************************************************************/
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
static void mos7840_block_until_tx_empty (struct moschip_port *mos7840_port)
#else
static void mos7840_block_until_tx_empty(struct tty_struct *tty,struct moschip_port *mos7840_port)
#endif
{
	struct usb_serial_port *port = tty->driver_data;
	int timeout = HZ/10;
	int wait = 30;
	int count;
	__u16 	Data;

	while (1) {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
		if (mos7840_port->port != NULL){
			count = mos7840_chars_in_buffer(mos7840_port->port);
		} else {
			printk("mos7840_port->port == NULL!!!\n");
			return;
		}
#else
		if (tty != NULL){
			count = mos7840_chars_in_buffer(tty);
		}else{
			printk("tty == NULL!!!\n");
			return;
		}
#endif

		Data = 0x00;
		mos7840_get_Uart_Reg(port, LINE_STATUS_REGISTER, &Data);

                /* Check for Buffer status */
		if (count <= 0 && ((Data & 0x60) == 0x60)) {
			if (mos7840_port->baudRate == 300)
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,15,0))
				interruptible_sleep_on_timeout(&mos7840_port->wait_chase, timeout);
#else
				wait_event_interruptible_timeout(mos7840_port->wait_chase, 0, HZ/40);
#endif
			return;
		}

                /* Block the thread for a while */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,15,0))
		interruptible_sleep_on_timeout(&mos7840_port->wait_chase, timeout);
#else
		wait_event_interruptible_timeout(mos7840_port->wait_chase, 0, timeout);
#endif


                /* No activity.. count down section */
		wait--;
		if (wait == 0) {
			DPRINTK("%s - TIMEOUT", __FUNCTION__);
			return;
		}/* else {
	                // Reset timout value back to seconds
			wait = 30;
		}*/
	}
}

/*****************************************************************************
 * mos7840_write_room
 *	this function is called by the tty driver when it wants to know how many
 *	bytes of data we can accept for a specific port.
 *	If successful, we return the amount of room that we have for this port
 *	Otherwise we return a negative error number.
 *****************************************************************************/
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
static int mos7840_write_room (struct usb_serial_port *port)
#elif LINUX_VERSION_CODE < KERNEL_VERSION(5,14,0)
static int mos7840_write_room(struct tty_struct *tty)
#else
static unsigned int mos7840_write_room(struct tty_struct *tty)
#endif
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
	struct usb_serial_port *port = tty->driver_data;
#endif
	int i;
	unsigned long flags;
	int room = 0;
	struct moschip_port *mos7840_port;
	
        if (mos7840_port_paranoia_check(port, __FUNCTION__) ) {
                DPRINTK("%s","Invalid port \n");
                DPRINTK("%s \n"," mos7840_write_room:leaving ...........");
                return -1;
        }

        mos7840_port = mos7840_get_port_private(port);
        if (mos7840_port == NULL) {
                DPRINTK("%s \n","mos7840_break:leaving ...........");
                return -1;
        }
                                                                                                                             
        spin_lock_irqsave(&mos7840_port->pool_lock, flags);
	for (i = 0; i < NUM_URBS; ++i) {
		if (!mos7840_port->busy[i])
			room += URB_TRANSFER_BUFFER_SIZE;
	}
	spin_unlock_irqrestore(&mos7840_port->pool_lock, flags);
       
        DPRINTK("%s - returns %d", __FUNCTION__, room);
        return (room);

}


/*****************************************************************************
 * mos7840_chars_in_buffer
 *	this function is called by the tty driver when it wants to know how many
 *	bytes of data we currently have outstanding in the port (data that has
 *	been written, but hasn't made it out the port yet)
 *	If successful, we return the number of bytes left to be written in the 
 *	system, 
 *	Otherwise we return a negative error number.
 *****************************************************************************/
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
static int mos7840_chars_in_buffer (struct usb_serial_port *port)
#elif LINUX_VERSION_CODE < KERNEL_VERSION(5,14,0)
static int mos7840_chars_in_buffer(struct tty_struct *tty)
#else
static unsigned int  mos7840_chars_in_buffer(struct tty_struct *tty)
#endif
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)	
	struct usb_serial_port *port = tty->driver_data;
#endif
		
	int i;
    	int chars = 0;
	unsigned long flags;
 	struct moschip_port *mos7840_port;

	if (mos7840_port_paranoia_check(port, __FUNCTION__) ) {
		DPRINTK("%s","Invalid port \n");
		return -1;
	}
                                                                                                                             
        mos7840_port = mos7840_get_port_private(port);
        if (mos7840_port == NULL) {
                DPRINTK("%s \n","mos7840_break:leaving ...........");
                return -1;
        }
                                                                                                                       
    	spin_lock_irqsave(&mos7840_port->pool_lock, flags);
	for (i = 0; i < NUM_URBS; ++i) {
		if (mos7840_port->busy[i]) {
			struct urb *urb = mos7840_port->write_urb_pool[i];
			chars += urb->transfer_buffer_length;
		}
	}
	spin_unlock_irqrestore(&mos7840_port->pool_lock, flags);
	
	return (chars);
}


/*****************************************************************************
 * SerialWrite
 *	this function is called by the tty driver when data should be written to
 *	the port.
 *	If successful, we return the number of bytes written, otherwise we 
 *      return a negative error number.
 *****************************************************************************/
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27) && LINUX_VERSION_CODE > KERNEL_VERSION(2,6,10)
static int mos7840_write (struct usb_serial_port *port, const unsigned char *data, int count)
#elif LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,10)
static int mos7840_write (struct usb_serial_port *port, int from_user, const unsigned char *data, int count)
#else
static int mos7840_write (struct tty_struct *tty,struct usb_serial_port *port, const unsigned char *data, int count)
#endif
{
	int tmp = 0;
	int status;
	int i;
	int bytes_sent = 0;
	int transfer_size;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,10)
	int from_user=0;
#endif
	struct moschip_port *mos7840_port;
	struct usb_serial *serial;
	struct moschip_serial *mos7840_serial;
	struct urb    *urb;
	int device_type;
	const unsigned char *current_position = data;
	unsigned char * data1;

	DPRINTK("%s \n","entering ...........");

#ifdef NOTMOS7840
	Data = 0x00;
        status = 0;
        status = mos7840_get_Uart_Reg(port,LINE_CONTROL_REGISTER,&Data);
	mos7840_port->shadowLCR = Data;
	DPRINTK("mos7840_write: LINE_CONTROL_REGISTER is %x\n", Data);
	DPRINTK("mos7840_write: mos7840_port->shadowLCR is %x\n", mos7840_port->shadowLCR);
	
	//Data = 0x03;
        //status = mos7840_set_Uart_Reg(port,LINE_CONTROL_REGISTER,Data);
        //mos7840_port->shadowLCR=Data;//Need to add later
	
        Data |= SERIAL_LCR_DLAB; //data latch enable in LCR 0x80
        status = 0;
        status = mos7840_set_Uart_Reg(port, LINE_CONTROL_REGISTER, Data);

	//Data = 0x0c;
        //status = mos7840_set_Uart_Reg(port,DIVISOR_LATCH_LSB,Data);
        Data = 0x00;
        status = 0;
        status = mos7840_get_Uart_Reg(port, DIVISOR_LATCH_LSB, &Data);
	DPRINTK("mos7840_write:DLL value is %x\n",Data);

        Data = 0x0;
        status = 0;
        status = mos7840_get_Uart_Reg(port, DIVISOR_LATCH_MSB, &Data);
	DPRINTK("mos7840_write:DLM value is %x\n",Data);

        Data = Data & ~SERIAL_LCR_DLAB;
	DPRINTK("mos7840_write: mos7840_port->shadowLCR is %x\n",mos7840_port->shadowLCR);
        status = 0;
        status = mos7840_set_Uart_Reg(port, LINE_CONTROL_REGISTER, Data);
#endif
	
	if (mos7840_port_paranoia_check (port, __FUNCTION__)) {
		DPRINTK("%s","Port Paranoia failed \n");
		return -1;
	}

	serial = port->serial;
	if (mos7840_serial_paranoia_check (serial, __FUNCTION__)) {
		DPRINTK("%s","Serial Paranoia failed \n");
		return -1;
	}

	mos7840_port = mos7840_get_port_private(port);
	if (mos7840_port == NULL) {
		DPRINTK("%s","mos7840_port is NULL\n");
		return -1;
	}

        if (mos7840_port->delta_cts_cond == 0 && mos7840_port->flow_ctrl_type == 2) {

                //interruptible_sleep_on(&mos7840_port->delta_cts_wait);
                wait_event_interruptible(mos7840_port->delta_cts_wait, (mos7840_port->delta_cts_cond == 1));
        }

        if (mos7840_port->xoff_cond == 0 && mos7840_port->flow_ctrl_type == 1) {

                //interruptible_sleep_on(&mos7840_port->delta_cts_wait);
                wait_event_interruptible(mos7840_port->xoff_wait, (mos7840_port->xoff_cond == 1));
        }

	mos7840_serial = mos7840_get_serial_private(serial);
	if (mos7840_serial == NULL) {
		DPRINTK("%s","mos7840_serial is NULL \n");
		return -1;
	}	
	device_type = mos7840_serial->device_type;
        /* try to find a free urb in the list */
        urb = NULL;

	for (i = 0; i < NUM_URBS; ++i) {
                if (!mos7840_port->busy[i]) {
			mos7840_port->busy[i] = 1;
                        urb = mos7840_port->write_urb_pool[i];
			DPRINTK("\nURB:%d",i);
                        break;
                }
        }

	if (urb == NULL) {
		DPRINTK("%s - no more free urbs", __FUNCTION__);
		goto exit;
	}

	if (urb->transfer_buffer == NULL) {
		urb->transfer_buffer = kmalloc (URB_TRANSFER_BUFFER_SIZE, GFP_KERNEL);
		if (urb->transfer_buffer == NULL) {
			DPRINTK("%s no more kernel memory...", __FUNCTION__);    
	        	goto exit;
                }
        }                                                                                                                   
        
	transfer_size = min (count, URB_TRANSFER_BUFFER_SIZE);

	if (from_user) {
		if (copy_from_user (urb->transfer_buffer, current_position, transfer_size)) {
			bytes_sent = -EFAULT;
			goto exit;
		}
        } else {
		memcpy (urb->transfer_buffer, current_position, transfer_size);
        }                                                                                                                  
	
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,11,0)
	tmp = ((__u16)port->number - (__u16)(port->serial->minor));
#else
	tmp = (__u16)port->port_number;
#endif

	/* fill urb with data and submit  */
	if ((mos7840_serial->parallelport || device_type == MOSCHIP_DEVICE_ID_7820) && tmp > 0 ) {
		usb_fill_bulk_urb (urb,
			mos7840_serial->serial->dev,
			usb_sndbulkpipe(mos7840_serial->serial->dev,
			(port->bulk_out_endpointAddress) + 2),
			urb->transfer_buffer,
			transfer_size,
			mos7840_bulk_out_data_callback,
			mos7840_port);
	} else {
           	usb_fill_bulk_urb (urb,
			mos7840_serial->serial->dev,
			usb_sndbulkpipe(mos7840_serial->serial->dev,
			port->bulk_out_endpointAddress),
			urb->transfer_buffer, 
			transfer_size,
			mos7840_bulk_out_data_callback,
			mos7840_port);
	}
	
	
	/* Turn on MCS7810 LED */
	if (mos7840_port->has_led && !mos7840_port->led_flag) {
		mos7840_port->led_flag = true;
		mos7840_set_led_sync(port, MODEM_CONTROL_REGISTER, 0x0301);
		mod_timer(&mos7840_port->led_timer1, jiffies + msecs_to_jiffies(MCS7810_LED_ON_MS));
	}

	data1=urb->transfer_buffer;
	DPRINTK("\nbulkout endpoint is %d", port->bulk_out_endpointAddress);

        /* send it down the pipe */
        status = usb_submit_urb(urb, GFP_ATOMIC);
	
        if (status) {
		mos7840_port->busy[i] = 0;
		DPRINTK("\n%s - usb_submit_urb(write bulk) failed with status = %d", __FUNCTION__, status);
		bytes_sent = status;
		goto exit;
	}

        bytes_sent = transfer_size;
	mos7840_port->icount.tx += transfer_size;
	

	DPRINTK("\nmos7840_port->icount.tx is %d:", mos7840_port->icount.tx);
exit:
	return bytes_sent;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,19)
static void mos7840_for_throttle_control_callback(struct urb *urb)
#else
static void mos7840_for_throttle_control_callback(struct urb *urb, struct pt_regs *regs)
#endif
{
	//struct moschip_port *mos7840_port;
	if (!urb) {
                DPRINTK("%s","Invalid Pointer !!!!:\n");
                return;
        }

        switch (urb->status) {
                case 0:
                        /* success */
                        break;
                case -ECONNRESET:
                case -ENOENT:
                case -ESHUTDOWN:
                        /* this urb is terminated, clean up */
                        DPRINTK("%s - urb shutting down with status: %d", __FUNCTION__, urb->status);                        
			return;
                default:
                        DPRINTK("%s - nonzero urb status received: %d", __FUNCTION__, urb->status);
                        goto exit;
        }

exit:
	return;	
}

static int mos7840_set_uart_for_throttle(struct usb_serial_port *port, __u16 reg, __u16 val)
{
	int tmp = 0;
        struct usb_device *dev = port->serial->dev;
	struct moschip_serial *mos7840_serial;
	struct moschip_port *mos7840_port;
	struct usb_ctrlrequest *dr = NULL;
	int ret;
 	int device_type;

	mos7840_serial = mos7840_get_serial_private(port->serial);
	mos7840_port = mos7840_get_port_private(port);

	val = val & 0x00ff;
        // For the UART control registers, the application number need to be Or'ed

	if (mos7840_port == NULL)
		return -ENODEV;
	
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,11,0)
	tmp = ((__u16)port->number - (__u16)(port->serial->minor));
#else
	tmp = (__u16)port->port_number;
#endif

	device_type = mos7840_serial->device_type;
	if ((mos7840_serial->parallelport || device_type == MOSCHIP_DEVICE_ID_7820) && tmp > 0 ) 
		val |= (tmp + 2) << 8;	
	else 
		val |= (tmp + 1) << 8;	
	DPRINTK("mos7840_set_Uart_Reg application number is %x\n", val);
	
	if ((dr = kmalloc (sizeof(struct usb_ctrlrequest), GFP_ATOMIC)) == NULL)
		return 0;

	dr->bRequestType = MCS_WR_RTYPE;
	dr->bRequest = MCS_WRREQ;
	dr->wValue = cpu_to_le16(val);
	dr->wIndex = cpu_to_le16(reg);
	dr->wLength = cpu_to_le16(0);

	usb_fill_control_urb(mos7840_port->control_urb_for_throttle, dev, usb_sndctrlpipe(dev, 0),
		(unsigned char *)dr, NULL, 0, mos7840_for_throttle_control_callback, NULL);
	
	ret = usb_submit_urb(mos7840_port->control_urb_for_throttle, GFP_ATOMIC);
	
	return ret;
}

/*****************************************************************************
 * SerialThrottle
 *	this function is called by the tty driver when it wants to stop the data
 *	being read from the port.
 *****************************************************************************/
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
static void mos7840_throttle (struct usb_serial_port *port)
#else
static void mos7840_throttle(struct tty_struct *tty)
#endif
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
	struct usb_serial_port *port = tty->driver_data;
#else
	struct tty_struct *tty;
#endif
	struct moschip_port *mos7840_port;
	int status;

	if (mos7840_port_paranoia_check(port, __FUNCTION__) ) {
		DPRINTK("%s","Invalid port \n");
		return;
	}

	//DPRINTK("- port %d\n", port->number);

	mos7840_port = mos7840_get_port_private(port); 

	if (mos7840_port == NULL)
		return;

	if (!mos7840_port->open) {
		DPRINTK("%s\n","port not opened");
		return;
	}

	DPRINTK("%s","Entering .......... \n");
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
	tty = port->tty;
#endif
	if (!tty) {
		DPRINTK ("%s - no tty available", __FUNCTION__);
		return;
	}

	/* if we are implementing XON/XOFF, send the stop character */
	if (I_IXOFF(tty)) {
		unsigned char stop_char = STOP_CHAR(tty);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27) && LINUX_VERSION_CODE > KERNEL_VERSION(2,6,10)
		status = mos7840_write (port, &stop_char, 1); //FC4
#elif LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,10)
		status = mos7840_write (port,0, &stop_char, 1);
#else
		status = mos7840_write (tty,port, &stop_char, 1);
#endif
		if (status <= 0) {
			return;
		}
	}

	/* if we are implementing RTS/CTS, toggle that line */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
	if ((tty->termios.c_cflag & CRTSCTS) || (hw_auto_fc == 1)) {
#else
	if ((tty->termios->c_cflag & CRTSCTS) || (hw_auto_fc == 1)) {
#endif
		mos7840_port->shadowMCR &= ~MCR_RTS;
		status = 0;
		status = mos7840_set_uart_for_throttle(port, MODEM_CONTROL_REGISTER, mos7840_port->shadowMCR);
		if (status < 0) 
			return;
	}

	return;
}


/*****************************************************************************
 * mos7840_unthrottle
 *	this function is called by the tty driver when it wants to resume the data
 *	being read from the port (called after SerialThrottle is called)
 *****************************************************************************/
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
static void mos7840_unthrottle (struct usb_serial_port *port)
#else
static void mos7840_unthrottle(struct tty_struct *tty)
#endif
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
        struct usb_serial_port *port = tty->driver_data;
#else
	struct tty_struct *tty;
#endif
	int status;
	struct moschip_port *mos7840_port = mos7840_get_port_private(port); 

	if (mos7840_port_paranoia_check(port, __FUNCTION__) ) {
		DPRINTK("%s","Invalid port \n");
		return;
	}

	if (mos7840_port == NULL)
		return;

	if (!mos7840_port->open) {
		DPRINTK("%s - port not opened", __FUNCTION__);
		return;
	}

	DPRINTK("%s","Entering .......... \n");
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
	tty = port->tty;
#endif
	if (!tty) {
		DPRINTK ("%s - no tty available", __FUNCTION__);
		return;
	}

	/* if we are implementing XON/XOFF, send the start character */
	if (I_IXOFF(tty)) {
		unsigned char start_char = START_CHAR(tty);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27) && LINUX_VERSION_CODE > KERNEL_VERSION(2,6,10)
		status = mos7840_write (port, &start_char, 1); //FC4
#elif LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,10)
		status = mos7840_write (port, 0, &start_char, 1);
#else
		status = mos7840_write (tty, port, &start_char, 1);
#endif
		if (status <= 0) 
			return;
	}

	/* if we are implementing RTS/CTS, toggle that line */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
	if ((tty->termios.c_cflag & CRTSCTS) || (hw_auto_fc == 1)) {
#else
	if ((tty->termios->c_cflag & CRTSCTS) || (hw_auto_fc == 1)) {
#endif
		mos7840_port->shadowMCR |= MCR_RTS;
		status = 0;
		status = mos7840_set_uart_for_throttle(port, MODEM_CONTROL_REGISTER, mos7840_port->shadowMCR);
		if (status < 0) 
			return;
	}

	return;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
static int mos7840_tiocmget(struct usb_serial_port *port, struct file *file)
#elif LINUX_VERSION_CODE > KERNEL_VERSION(2,6,38)
static int mos7840_tiocmget(struct tty_struct *tty)
#else
static int mos7840_tiocmget(struct tty_struct *tty, struct file *file)
#endif
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
 	struct usb_serial_port *port = tty->driver_data;    
#endif
    
	struct moschip_port *mos7840_port;
        unsigned int result;
        __u16 msr;
        __u16 mcr;

	mos7840_port = mos7840_get_port_private(port); 

        //DPRINTK("%s - port %d", __FUNCTION__, port->number);

        if (mos7840_port == NULL)
                return -ENODEV;

	//status=mos7840_get_Uart_Reg(port,MODEM_STATUS_REGISTER,&msr);
	//status=mos7840_get_Uart_Reg(port,MODEM_CONTROL_REGISTER,&mcr);
	mcr = mos7840_port->shadowMCR;
	msr = mos7840_port->shadowMSR;
	// COMMENT2: the Fallowing three line are commented for updating only MSR values
        result = ((mcr & MCR_DTR) ? TIOCM_DTR : 0)
                | ((mcr & MCR_RTS) ? TIOCM_RTS : 0)
                | ((mcr & MCR_LOOPBACK) ? TIOCM_LOOP : 0)
         	| ((msr & MOS7840_MSR_CTS) ? TIOCM_CTS : 0)
                | ((msr & MOS7840_MSR_CD) ? TIOCM_CAR : 0)
                | ((msr & MOS7840_MSR_RI) ? TIOCM_RI : 0)
                | ((msr & MOS7840_MSR_DSR) ? TIOCM_DSR : 0);

        DPRINTK("%s - 0x%04X", __FUNCTION__, result);

        return result;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
static int mos7840_tiocmset(struct usb_serial_port *port, struct file *file, unsigned int set, unsigned int clear)
#elif LINUX_VERSION_CODE > KERNEL_VERSION(2,6,38)
static int mos7840_tiocmset(struct tty_struct *tty, unsigned int set, unsigned int clear)
#else
static int mos7840_tiocmset(struct tty_struct *tty, struct file *file,unsigned int set, unsigned int clear)
#endif
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
	struct usb_serial_port *port = tty->driver_data;
#endif
	struct moschip_port *mos7840_port;
        unsigned int mcr;
	unsigned int status;

        //DPRINTK("%s - port %d", __FUNCTION__, port->number);

	mos7840_port = mos7840_get_port_private(port); 

	if (mos7840_port == NULL)
                return -ENODEV;

	mcr = mos7840_port->shadowMCR;
        if (clear & TIOCM_RTS)
                mcr &= ~MCR_RTS;
        if (clear & TIOCM_DTR)
                mcr &= ~MCR_DTR;
        if (clear & TIOCM_LOOP)
                mcr &= ~MCR_LOOPBACK;
	
        if (set & TIOCM_RTS)
                mcr |= MCR_RTS;
        if (set & TIOCM_DTR)
                mcr |= MCR_DTR;
        if (set & TIOCM_LOOP)
                mcr |= MCR_LOOPBACK;

	mos7840_port->shadowMCR = mcr;

        status = 0;
        status = mos7840_set_Uart_Reg(port, MODEM_CONTROL_REGISTER, mcr);
        if (status <0) {
                DPRINTK("setting MODEM_CONTROL_REGISTER Failed\n");
                return -1;
        }

        return 0;
}

/*****************************************************************************
 * SerialSetTermios
 *	this function is called by the tty driver when it wants to change the termios structure
 *****************************************************************************/
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27) && LINUX_VERSION_CODE > KERNEL_VERSION(2,6,19)
static void mos7840_set_termios (struct usb_serial_port *port, struct ktermios *old_termios)
#elif LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,19)
static void mos7840_set_termios (struct usb_serial_port *port, struct termios *old_termios)
#else
static void mos7840_set_termios(struct tty_struct *tty,struct usb_serial_port *port,struct ktermios *old_termios)
#endif
{
	int status;
	unsigned int cflag;
	struct usb_serial *serial;
	struct moschip_port *mos7840_port;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
	struct tty_struct *tty;
#endif

	if (mos7840_port_paranoia_check(port, __FUNCTION__) ) {
		DPRINTK("%s","Invalid port \n");
		return;
	}

	serial = port->serial;

	if (mos7840_serial_paranoia_check(serial, __FUNCTION__) ) {
		DPRINTK("%s","Invalid Serial \n");
		return;
	}

	mos7840_port = mos7840_get_port_private(port); 

	if (mos7840_port == NULL)
		return;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)

	tty = port->tty;

	if (!port->tty || !port->tty->termios) {
		DPRINTK ("%s - no tty or termios", __FUNCTION__);
		return;
	}

#endif
	if (!mos7840_port->open) {
		DPRINTK("%s - port not opened", __FUNCTION__);
		return;
	}

	DPRINTK("%s\n","setting termios - ");

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
        mos7840_block_until_tx_empty(mos7840_port);
#else
        mos7840_block_until_tx_empty(tty,mos7840_port);
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
	if (tty->termios.c_cflag & CRTSCTS)
#else
	if (tty->termios->c_cflag & CRTSCTS)
#endif
		mos7840_port->flow_ctrl_type = 2;
	else if (I_IXOFF(tty))
		mos7840_port->flow_ctrl_type = 1;
	else
		mos7840_port->flow_ctrl_type = 0;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
	cflag = tty->termios.c_cflag;
#else
	cflag = tty->termios->c_cflag;
#endif

	//DPRINTK("newterminal cflag=%u and iflag=%u\n", cflag,tty->termios->c_iflag);
	DPRINTK("oldterminal cflag=%u and iflag=%u\n", old_termios->c_cflag, old_termios->c_iflag);

	if (!cflag) {
           DPRINTK("%s %s\n", __FUNCTION__, "cflag is NULL");
	   return;
	}            

	/* check that they really want us to change something */
	if (old_termios) {
		if ((cflag == old_termios->c_cflag) &&

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
		    (RELEVANT_IFLAG(tty->termios.c_iflag) == RELEVANT_IFLAG(old_termios->c_iflag))) {
#else
		    (RELEVANT_IFLAG(tty->termios->c_iflag) == RELEVANT_IFLAG(old_termios->c_iflag))) {
#endif

			DPRINTK("%s\n","Nothing to change");
			return;
		}
	}

	//DPRINTK("%s - clfag %08x iflag %08x", __FUNCTION__, tty->termios->c_cflag, RELEVANT_IFLAG(tty->termios->c_iflag));
	
	if (old_termios) {
		DPRINTK("%s - old clfag %08x old iflag %08x", __FUNCTION__,
			old_termios->c_cflag,
			RELEVANT_IFLAG(old_termios->c_iflag));
	}

	//DPRINTK("%s - port %d", __FUNCTION__, port->number);

	/* change the port settings to the new ones specified */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
	mos7840_change_port_settings(mos7840_port, old_termios);
#else
	mos7840_change_port_settings(tty, mos7840_port, old_termios);
#endif

	if (!mos7840_port->read_urb) {
		DPRINTK("%s","URB KILLED !!!!!\n");
		return;
	}

	if (mos7840_port->read_urb->status!=-EINPROGRESS) {
		mos7840_port->read_urb->dev = serial->dev;
	 	status = usb_submit_urb(mos7840_port->read_urb, GFP_ATOMIC);
		if (status) {
			DPRINTK(" usb_submit_urb(read bulk) failed, status = %d", status);
		}
	}
	
	return;
}


/*****************************************************************************
 * get_lsr_info - get line status register info
 *
 * Purpose: Let user call ioctl() to get info when the UART physically
 * 	    is emptied.  On bus types like RS485, the transmitter must
 * 	    release the bus after transmitting. This must be done when
 * 	    the transmit shift register is empty, not be done when the
 * 	    transmit holding register is empty.  This functionality
 * 	    allows an RS485 driver to be written in user space. 
 *****************************************************************************/
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
static int get_lsr_info(struct moschip_port *mos7840_port, unsigned int *value)
#else
static int get_lsr_info(struct tty_struct *tty, unsigned int *value)
#endif
{
	int count;
	unsigned int result = 0;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
        count = mos7840_chars_in_buffer(mos7840_port->port);
#else
	count = mos7840_chars_in_buffer(tty);
#endif        
	if (count == 0) {
		DPRINTK("%s -- Empty", __FUNCTION__);
		result = TIOCSER_TEMT;
	}

	if (copy_to_user(value, &result, sizeof(int)))
		return -EFAULT;
	return 0;
}

/*****************************************************************************
 * get_number_bytes_avail - get number of bytes available
 *
 * Purpose: Let user call ioctl to get the count of number of bytes available.
 *****************************************************************************/
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
static int get_number_bytes_avail(struct moschip_port *mos7840_port, unsigned int *value)
#else
static int get_number_bytes_avail(struct tty_struct *tty,struct moschip_port *mos7840_port, unsigned int *value)
#endif
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)
	return -ENOIOCTLCMD;
#else
	unsigned int result = 0;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
	struct tty_struct *tty = mos7840_port->port->tty;
#endif

	if (!tty)
		return -ENOIOCTLCMD;

	result = tty->read_cnt;

	//DPRINTK("%s(%d) = %d", __FUNCTION__,  mos7840_port->port->number, result);
	if (copy_to_user(value, &result, sizeof(int)))
		return -EFAULT;

	return -ENOIOCTLCMD;
#endif
}


/*****************************************************************************
 * set_modem_info
 *      function to set modem info
 *****************************************************************************/

static int set_modem_info(struct moschip_port *mos7840_port, unsigned int cmd, unsigned int *value)
{
	unsigned int mcr ;
	unsigned int arg;
	__u16 Data;
	int status;
	struct usb_serial_port *port;

	if (mos7840_port == NULL)
		return -1;
	
	port = (struct usb_serial_port*)mos7840_port->port;
	if (mos7840_port_paranoia_check(port,__FUNCTION__) ) {
		DPRINTK("%s","Invalid port \n");
		return -1;
	}

	mcr = mos7840_port->shadowMCR;

	if (copy_from_user(&arg, value, sizeof(int)))
		return -EFAULT;

	switch (cmd) {
	case TIOCMBIS:
		if (arg & TIOCM_RTS)
			mcr |= MCR_RTS;
		if (arg & TIOCM_DTR)
			mcr |= MCR_RTS;
		if (arg & TIOCM_LOOP)
			mcr |= MCR_LOOPBACK;
		break;
	case TIOCMBIC:
		if (arg & TIOCM_RTS)
			mcr &= ~MCR_RTS;
		if (arg & TIOCM_DTR)
			mcr &= ~MCR_RTS;
		if (arg & TIOCM_LOOP)
			mcr &= ~MCR_LOOPBACK;
		break;
	case TIOCMSET:
		/* turn off the RTS and DTR and LOOPBACK 
		* and then only turn on what was asked to */
		mcr &=  ~(MCR_RTS | MCR_DTR | MCR_LOOPBACK);
		mcr |= ((arg & TIOCM_RTS) ? MCR_RTS : 0);
		mcr |= ((arg & TIOCM_DTR) ? MCR_DTR : 0);
		mcr |= ((arg & TIOCM_LOOP) ? MCR_LOOPBACK : 0);
		break;
	}

	mos7840_port->shadowMCR = mcr;

	Data = mos7840_port->shadowMCR;
	status = 0;
	status = mos7840_set_Uart_Reg(port,MODEM_CONTROL_REGISTER,Data);
	if (status <0) {
		DPRINTK("setting MODEM_CONTROL_REGISTER Failed\n");
		return -1;
	}

	return 0;
}

/*****************************************************************************
 * get_modem_info
 *      function to get modem info
 *****************************************************************************/

static int get_modem_info(struct moschip_port *mos7840_port, unsigned int *value)
{
	unsigned int result = 0;
	__u16 msr = 0;
	unsigned int mcr = mos7840_port->shadowMCR;
	int status = 0;
	status = mos7840_get_Uart_Reg(mos7840_port->port,MODEM_STATUS_REGISTER,&msr);
	result = ((mcr & MCR_DTR)	? TIOCM_DTR: 0)	  /* 0x002 */
		  | ((mcr & MCR_RTS)	? TIOCM_RTS: 0)   /* 0x004 */
		  | ((msr & MOS7840_MSR_CTS)	? TIOCM_CTS: 0)   /* 0x020 */
		  | ((msr & MOS7840_MSR_CD)	? TIOCM_CAR: 0)   /* 0x040 */
		  | ((msr & MOS7840_MSR_RI)	? TIOCM_RI:  0)   /* 0x080 */
		  | ((msr & MOS7840_MSR_DSR)	? TIOCM_DSR: 0);  /* 0x100 */


	DPRINTK("%s -- %x", __FUNCTION__, result);

	if (copy_to_user(value, &result, sizeof(int)))
		return -EFAULT;
	return 0;
}

/*****************************************************************************
 * get_serial_info
 *      function to get information about serial port
 *****************************************************************************/

static int get_serial_info(struct moschip_port *mos7840_port, struct serial_struct * retinfo)
{
	struct serial_struct tmp;

	if (mos7840_port == NULL)
		return -1;

	if (!retinfo)
		return -EFAULT;

	memset(&tmp, 0, sizeof(tmp));

	tmp.type		= PORT_16550A;
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,11,0)
	tmp.line		= mos7840_port->port->serial->minor;
	tmp.port		= mos7840_port->port->number;
#else
	tmp.line		= mos7840_port->port->minor;
	tmp.port		= mos7840_port->port->port_number;
#endif
	tmp.irq			= 0;
	tmp.flags		= ASYNC_SKIP_TEST | ASYNC_AUTO_IRQ;
	tmp.xmit_fifo_size	= NUM_URBS * URB_TRANSFER_BUFFER_SIZE;
	tmp.baud_base		= 921600;//9600; //baud_base is taken based upon baud rate setting.
	tmp.close_delay		= 5*HZ;
	tmp.closing_wait	= 30*HZ;


	if (copy_to_user(retinfo, &tmp, sizeof(*retinfo)))
		return -EFAULT;
	return 0;
}

/*****************************************************************************
 * SerialIoctl
 *	this function handles any ioctl calls to the driver
 *****************************************************************************/
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
static int mos7840_ioctl (struct usb_serial_port *port, struct file *file, unsigned int cmd, unsigned long arg)
#elif LINUX_VERSION_CODE > KERNEL_VERSION(2,6,38)
static int mos7840_ioctl(struct tty_struct *tty, unsigned int cmd, unsigned long arg)
#else
static int mos7840_ioctl(struct tty_struct *tty, struct file *file, unsigned int cmd, unsigned long arg)
#endif
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)	
	struct usb_serial_port *port = tty->driver_data;
#else
	struct tty_struct *tty;
#endif
	struct moschip_port *mos7840_port;
	struct moschip_serial *mos7840_serial;
	
	struct async_icount cnow;
	struct async_icount cprev;
	struct serial_icounter_struct icount;
	int mosret=0;

	if (mos7840_port_paranoia_check(port, __FUNCTION__) ) {
		DPRINTK("%s","Invalid port \n");
		return -1;
	}

	mos7840_port = mos7840_get_port_private(port); 
	
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
	tty = mos7840_port->port->tty;
#endif	
	mos7840_serial = mos7840_get_serial_private(port->serial);
	if (mos7840_serial == NULL)
		return -1;
	//DPRINTK("%s - port %d, cmd = 0x%x", __FUNCTION__, port->number, cmd);

	switch (cmd) {
                /* return number of bytes available */
	
	case TIOCINQ:
		//DPRINTK("%s (%d) TIOCINQ", __FUNCTION__, port->number);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
		return get_number_bytes_avail(mos7840_port, (unsigned int *)arg);
#else
		return get_number_bytes_avail(tty,mos7840_port, (unsigned int *)arg);
#endif
		break;

	case TIOCOUTQ:
		//DPRINTK("%s (%d) TIOCOUTQ", __FUNCTION__, port->number);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)
		return put_user(tty->driver->chars_in_buffer ?
				tty->driver->chars_in_buffer(tty) : 0,
				(int __user *) arg);
#else
		return put_user(tty->driver->ops->chars_in_buffer ?
				tty->driver->ops->chars_in_buffer(tty) : 0,
				(int __user *) arg);
#endif
		break;

	case TIOCSERGETLSR:
		//DPRINTK("%s (%d) TIOCSERGETLSR", __FUNCTION__,  port->number);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
		return get_lsr_info(mos7840_port, (unsigned int *)arg);
#else
		return get_lsr_info(tty, (unsigned int *)arg);	
#endif
		return 0;

	case TIOCMBIS:
	case TIOCMBIC:
	case TIOCMSET:
		//DPRINTK("%s (%d) TIOCMSET/TIOCMBIC/TIOCMSET", __FUNCTION__, port->number);
		mosret=set_modem_info(mos7840_port, cmd, (unsigned int *) arg);
		return mosret;

	case TIOCMGET:  
		//DPRINTK("%s (%d) TIOCMGET", __FUNCTION__, port->number);
		return get_modem_info(mos7840_port, (unsigned int *)arg);

	case TIOCGSERIAL:
		//DPRINTK("%s (%d) TIOCGSERIAL", __FUNCTION__, port->number);
		return get_serial_info(mos7840_port, (struct serial_struct *)arg);

	case TIOCSSERIAL:
		//DPRINTK("%s (%d) TIOCSSERIAL", __FUNCTION__, port->number);
		return 0;
		break;

	case TIOCMIWAIT:
		//DPRINTK("%s (%d) TIOCMIWAIT", __FUNCTION__, port->number);
		cprev = mos7840_port->icount;
		while (1) {

			/* see if a signal did it */
			if (signal_pending(current))
				return -ERESTARTSYS;
			cnow = mos7840_port->icount;
			if (cnow.rng == cprev.rng && cnow.dsr == cprev.dsr &&
				cnow.dcd == cprev.dcd && cnow.cts == cprev.cts)
				return -EIO; /* no change => error */
			if (((arg & TIOCM_RNG) && (cnow.rng != cprev.rng)) ||
					((arg & TIOCM_DSR) && (cnow.dsr != cprev.dsr)) ||
					((arg & TIOCM_CD)  && (cnow.dcd != cprev.dcd)) ||
					((arg & TIOCM_CTS) && (cnow.cts != cprev.cts)) ) {
				return 0;
			}
			
			cprev = cnow;
		}
			/* NOTREACHED */
			break;

	case TIOCGICOUNT:
		cnow = mos7840_port->icount;
		icount.cts = cnow.cts;
		icount.dsr = cnow.dsr;
		icount.rng = cnow.rng;
		icount.dcd = cnow.dcd;
		icount.rx = cnow.rx;
		icount.tx = cnow.tx;
		icount.frame = cnow.frame;
		icount.overrun = cnow.overrun;
		icount.parity = cnow.parity;
		icount.brk = cnow.brk;
		icount.buf_overrun = cnow.buf_overrun;

		//DPRINTK("%s (%d) TIOCGICOUNT RX=%d, TX=%d", __FUNCTION__,  port->number, icount.rx, icount.tx );
		if (copy_to_user((void *)arg, &icount, sizeof(icount)))
			return -EFAULT;
		
		return 0;

	case TIOCEXBAUD:
		mos7840_send_cmd_write_baud_rate(mos7840_port, (int)arg);
		return 0;

	case IOCTL_CHECK_AS:

#ifdef AUTO_SWITCH
		__put_user(1, (int __user *)arg);
#else
		__put_user(0, (int __user *)arg);	
#endif
		return 0;

	case IOCTL_SET_RS_MODE:

		mos7840_port->ioctl_rs_setting = arg;

		//printk("---mos7840_ioctl() arg=%X\n", mos7840_port->ioctl_rs_setting);

		if (mos7840_port->ioctl_rs_setting == 0)
			mos7840_port->rs_mode_port[mos7840_port->port_num - 1] = 0;
		else if (mos7840_port->ioctl_rs_setting == 0xc3)
			mos7840_port->rs_mode_port[mos7840_port->port_num - 1] = 2;
		else if (mos7840_port->ioctl_rs_setting == 0xc2)
			mos7840_port->rs_mode_port[mos7840_port->port_num - 1] = 3;

		return 0;
	case IOCTL_CHECK_DEVICE:
	{
		unsigned char 	sig[16];
		int 		device_type = 0;
	
		device_type = mos7840_serial->device_type;
		if (device_type == MOSCHIP_DEVICE_ID_7840) {
			strncpy(sig, AX78140_DRV_NAME, strlen(AX78140_DRV_NAME));
		} else if (device_type == MOSCHIP_DEVICE_ID_7820) {
			strncpy(sig, AX78120_DRV_NAME, strlen(AX78120_DRV_NAME));
		}
		
		if (copy_to_user((void *)arg, sig, strlen(AX78120_DRV_NAME)))
			return -EFAULT;
		
		return 0;
	}
	case IOCTL_GET_MSR:
	{
		__u16 msr;

		//printk("IOCTL_GET_MSR\n");
		if (mos7840_get_Uart_Reg(port,MODEM_STATUS_REGISTER,&msr) < 0)
			return -EFAULT;

		//printk("Get MSR: 0x%04x\n", msr);
		
		if (copy_to_user((void *)arg, &msr, sizeof(__u16)))
			return -EFAULT;
		return 0;
	}
	case IOCTL_SET_MCR:
	{
		__u16 mcr;

		//printk("IOCTL_SET_MCR\n");
		if (copy_from_user(&mcr, (void *)arg, sizeof(__u16)))
			return -EFAULT;	

		//printk("Set MCR: 0x%04x\n", mcr);
	
		if (mos7840_set_Uart_Reg(port,MODEM_CONTROL_REGISTER,mcr) < 0)
			return -EFAULT;

		return 0;
	}
	case IOCTL_GET_MCR:
	{
		__u16 mcr;		
		if (mos7840_get_Uart_Reg(port,MODEM_CONTROL_REGISTER,&mcr) < 0)
			return -EFAULT;

		//printk("Get MCR: 0x%04x\n", mcr);
		
		if (copy_to_user((void *)arg, &mcr, sizeof(__u16)))
			return -EFAULT;

		//printk("IOCTL_GET_MCR\n");
		return 0;
	}
	default:
		break;
	}

	return -ENOIOCTLCMD;
}


/*****************************************************************************
 * mos7840_send_cmd_write_baud_rate
 *	this function sends the proper command to change the baud rate of the
 *	specified port.
 *****************************************************************************/

static int mos7840_send_cmd_write_baud_rate (struct moschip_port *mos7840_port, int baudRate)
{
	int divisor = 0;
	int status;
	__u16 Data;
	__u16 clk_sel_val;
	struct usb_serial_port *port;

	if (mos7840_port == NULL)
		return -1;

	port = (struct usb_serial_port*)mos7840_port->port;
	if (mos7840_port_paranoia_check(port, __FUNCTION__)) {
		DPRINTK("%s","Invalid port \n");
		return -1;
	}

	if (mos7840_serial_paranoia_check(port->serial, __FUNCTION__)) {
		DPRINTK("%s","Invalid Serial \n");
		return -1;
	}

	DPRINTK("%s","Entering .......... \n");

	//DPRINTK("%s - port = %d, baud = %d", __FUNCTION__, mos7840_port->port->number, baudRate);
	//reset clk_uart_sel in spregOffset

	if (1) { //baudRate <= 115200)
		clk_sel_val = 0x0;
		Data = 0x0;
		status = 0;
		status = mos7840_calc_baud_rate_divisor (baudRate, &divisor, &clk_sel_val);
		if (status == 0) {
			Data = 0x0;
			status = 0;
			status = mos7840_get_reg_sync(port, mos7840_port->ClkSelectRegOffset, &Data);
			if (status < 0) {		
				DPRINTK("reading spreg failed in set_serial_baud\n");
				return -1;
			}		
			Data &= (mos7840_port->SpRegOffset == 0x0 || mos7840_port->SpRegOffset == 0xa)?0xF8:0xC7;
			status = mos7840_set_reg_sync(port, mos7840_port->ClkSelectRegOffset, Data);
			if (status < 0) {		
				DPRINTK("Writing spreg failed in set_serial_baud\n");
				return -1;
			}			
			status = mos7840_get_reg_sync(port, mos7840_port->SpRegOffset, &Data);
			if (status < 0) {		
				DPRINTK("reading spreg failed in set_serial_baud\n");
				return -1;
			}
			Data = (Data & 0x8f) | clk_sel_val;
			status = 0;
			status = mos7840_set_reg_sync(port, mos7840_port->SpRegOffset, Data);
			if (status < 0) {		
				DPRINTK("Writing spreg failed in set_serial_baud\n");
				return -1;
			}
			DPRINTK("Baud:%d\n", baudRate);
		} else {
			Data = 0x0;
			status = 0;
			status = mos7840_get_reg_sync(port, mos7840_port->ClkSelectRegOffset, &Data);
			if (status < 0) {		
				DPRINTK("reading spreg failed in set_serial_baud\n");
				return -1;
			}
			Data |= (mos7840_port->SpRegOffset == 0x0 || mos7840_port->SpRegOffset == 0xa)?0x02:0x10;
			status = 0;
			status = mos7840_set_reg_sync(port, mos7840_port->ClkSelectRegOffset, Data);
			if (status < 0) {		
				DPRINTK("Writing spreg failed in set_serial_baud\n");
				return -1;
			}
			DPRINTK("AddedBaud:%d\n", baudRate);
		}
       		 /* Calculate the Divisor */
		
		if (status) {
			DPRINTK("%s - bad baud rate", __FUNCTION__);
			DPRINTK("%s\n","bad baud rate");
			return status;
		}
	        /* Enable access to divisor latch */
	        Data = mos7840_port->shadowLCR | SERIAL_LCR_DLAB;
	        mos7840_port->shadowLCR = Data;
		mos7840_set_Uart_Reg(port, LINE_CONTROL_REGISTER, Data);
	
		/* Write the divisor */
		Data = LOW8 (divisor);//ASIX:  commented to test
		DPRINTK("set_serial_baud Value to write DLL is %x\n", Data);
		mos7840_set_Uart_Reg(port, DIVISOR_LATCH_LSB, Data);
	
		Data = HIGH8 (divisor); //ASIX:  commented to test
		DPRINTK("set_serial_baud Value to write DLM is %x\n", Data);
		mos7840_set_Uart_Reg(port, DIVISOR_LATCH_MSB, Data);
	
	        /* Disable access to divisor latch */
	        Data = mos7840_port->shadowLCR & ~SERIAL_LCR_DLAB;
	        mos7840_port->shadowLCR = Data;
		mos7840_set_Uart_Reg(port, LINE_CONTROL_REGISTER, Data);
	}
		
	return status;
}



/*****************************************************************************
 * mos7840_calc_baud_rate_divisor
 *	this function calculates the proper baud rate divisor for the specified
 *	baud rate.
 *****************************************************************************/
static int mos7840_calc_baud_rate_divisor (int baudRate, int *divisor,__u16 *clk_sel_val)
{
	DPRINTK("%s - %d", __FUNCTION__, baudRate);

	if (baudRate <= 115200) {
		*divisor = 115200/baudRate;
		*clk_sel_val = 0x0;
	}

 	if (baudRate == 230400) {
		*divisor = 230400 / baudRate;	
		*clk_sel_val = 0x10;
	} else if (baudRate == 403200) {
		*divisor = 403200 / baudRate;	
		*clk_sel_val = 0x20;
	} else if (baudRate == 460800) {
		*divisor = 460800 / baudRate;	
		*clk_sel_val = 0x30;
	} else if (baudRate == 806400) {
		*divisor = 806400 / baudRate;	
		*clk_sel_val = 0x40;
	} else if (baudRate == 921600) {
		*divisor = 921600 / baudRate;	
		*clk_sel_val = 0x50;
	} else if (baudRate == 1500000) {
		*divisor = 1500000 / baudRate;	
		*clk_sel_val = 0x60;
	} else if (baudRate == 3000000 || baudRate == 1000000 || baudRate == 600000) {
		*divisor = 3000000 / baudRate;	
		*clk_sel_val = 0x70;
	} else if (baudRate == 6000000 || baudRate == 2000000) {
		*divisor = 6000000 / baudRate;	
		return 1;
	}
	return 0;	
}



/*****************************************************************************
 * mos7840_change_port_settings
 *	This routine is called to set the UART on the device to match 
 *      the specified new settings.
 *****************************************************************************/
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27) && LINUX_VERSION_CODE > KERNEL_VERSION(2,6,19)
static void mos7840_change_port_settings (struct moschip_port *mos7840_port, struct ktermios *old_termios)
#elif LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,19)
static void mos7840_change_port_settings (struct moschip_port *mos7840_port, struct termios *old_termios)
#else
static void mos7840_change_port_settings(struct tty_struct *tty,struct moschip_port *mos7840_port, struct ktermios *old_termios)
#endif
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
	struct tty_struct *tty;
#endif
	int baud;
	unsigned cflag;
	unsigned iflag;
	__u8 mask = 0xff;
	__u8 lData;
	__u8 lParity;
	__u8 lStop;
	int status;
	__u16 Data;
	struct usb_serial_port *port;
	struct usb_serial *serial;

	if (mos7840_port == NULL)
		return ;

	port = (struct usb_serial_port *)mos7840_port->port;

	if (mos7840_port_paranoia_check(port, __FUNCTION__)) {
		DPRINTK("%s","Invalid port \n");
		return ;
	}

	if (mos7840_serial_paranoia_check(port->serial, __FUNCTION__)) {
		DPRINTK("%s","Invalid Serial \n");
		return ;
	}

	serial = port->serial;

	//DPRINTK("%s - port %d", __FUNCTION__, mos7840_port->port->number);

	if ((!mos7840_port->open) && (!mos7840_port->openPending)) {
		DPRINTK("%s - port not opened", __FUNCTION__);
		return;
	}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
	tty = mos7840_port->port->tty;
#endif
	
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
	if (!tty) {
#else
	if ((!tty) || (!tty->termios)) {
#endif
		DPRINTK("%s - no tty structures", __FUNCTION__);
		return;
	}

	DPRINTK("%s","Entering .......... \n");

	lData = LCR_BITS_8;
	lStop = LCR_STOP_1;         
	lParity = LCR_PAR_NONE;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
	cflag = tty->termios.c_cflag;
	iflag = tty->termios.c_iflag;
#else
	cflag = tty->termios->c_cflag;
	iflag = tty->termios->c_iflag;
#endif

	/* Change the number of bits */	

	//COMMENT1: the below Line"if (cflag & CSIZE)" is added for the errors we get for serial loop data test i.e serial_loopback.pl -v
	//if (cflag & CSIZE)
	{
	switch (cflag & CSIZE) {
	case CS5:	
		lData = LCR_BITS_5; 
		mask = 0x1f;    
		DPRINTK("%s - databit = 5\n", __FUNCTION__);
		break;

	case CS6:   	
		lData = LCR_BITS_6; 
		mask = 0x3f; 
		DPRINTK("%s - databit = 6\n", __FUNCTION__);
		break;

	case CS7:   	
		lData = LCR_BITS_7; 
		mask = 0x7f;    
		DPRINTK("%s - databit = 7\n", __FUNCTION__);
		break;
	default:
	case CS8:   	
		lData = LCR_BITS_8;
		DPRINTK("%s - databit = 8\n", __FUNCTION__);
		break;
	}
	}
	/* Change the Parity bit */
	if (cflag & PARENB) {
		if (cflag & PARODD) {
			lParity = LCR_PAR_ODD;
			DPRINTK("%s - parity = odd\n", __FUNCTION__);
		} else {
			lParity = LCR_PAR_EVEN;
			DPRINTK("%s - parity = even\n", __FUNCTION__);
		}

	} else 
		DPRINTK("%s - parity = none\n", __FUNCTION__);

	
	if (cflag & CMSPAR) 
		lParity = lParity | 0x20;


	/* Change the Stop bit */
	if (cflag & CSTOPB) {
		lStop = LCR_STOP_2;
		DPRINTK("%s - stop bits = 2\n", __FUNCTION__);
	} else {
		lStop = LCR_STOP_1;
		DPRINTK("%s - stop bits = 1\n", __FUNCTION__);
	}

	
	/* Update the LCR with the correct value */
	mos7840_port->shadowLCR &= ~(LCR_BITS_MASK | LCR_STOP_MASK | LCR_PAR_MASK);
	mos7840_port->shadowLCR |= (lData | lParity | lStop);

	mos7840_port->validDataMask = mask;
	DPRINTK("mos7840_change_port_settings mos7840_port->shadowLCR is %x\n", mos7840_port->shadowLCR);
	/* Disable Interrupts */
	Data = 0x00;
	mos7840_set_Uart_Reg(port, INTERRUPT_ENABLE_REGISTER, Data);

	Data = 0x00;
	mos7840_set_Uart_Reg(port, FIFO_CONTROL_REGISTER, Data);

	Data = 0xCF;
	mos7840_set_Uart_Reg(port, FIFO_CONTROL_REGISTER, Data);

	/* Send the updated LCR value to the mos7840 */
	Data = mos7840_port->shadowLCR;

	mos7840_set_Uart_Reg(port, LINE_CONTROL_REGISTER, Data);


	Data = 0x0b;
	mos7840_port->shadowMCR = Data;
	mos7840_set_Uart_Reg(port, MODEM_CONTROL_REGISTER, Data);

	
	/* set up the MCR register and send it to the mos7840 */
	
	mos7840_port->shadowMCR = MCR_MASTER_IE;
	if (cflag & CBAUD) {
		mos7840_port->shadowMCR |= (MCR_DTR | MCR_RTS);
	}

	if (cflag & CRTSCTS || hw_auto_fc == 1) {
		mos7840_port->shadowMCR |= (MCR_XON_ANY);

		status = 0;
		Data = 0x0;
		status = mos7840_get_reg_sync(port, mos7840_port->ControlRegOffset, &Data);
		if (status < 0) 
			DPRINTK("Reading Controlreg failed\n");

		if (hw_auto_fc == 2) 
			Data |= 0x01;
		else if (hw_auto_fc == 0) 
			Data &= ~0x01;
		

		status = 0;
		status = mos7840_set_reg_sync(port, mos7840_port->ControlRegOffset, Data);
		if (status < 0) 
			DPRINTK("writing Controlreg failed\n");
	} else
		mos7840_port->shadowMCR &= ~(MCR_XON_ANY); 

	Data = mos7840_port->shadowMCR;
	mos7840_set_Uart_Reg(port, MODEM_CONTROL_REGISTER, Data);

	/* Determine divisor based on baud rate */
	baud = tty_get_baud_rate(tty);

	if (!baud) {
		/* pick a default, any default... */
		DPRINTK("%s\n","Picked default baud...");
		baud = 9600;
	}

	mos7840_port->baudRate = baud;

	status = mos7840_send_cmd_write_baud_rate(mos7840_port, baud);

	/* Enable Interrupts */
	Data = 0x0c;
	mos7840_set_Uart_Reg(port, INTERRUPT_ENABLE_REGISTER, Data);

	if (mos7840_port->read_urb->status != -EINPROGRESS) {
		printk("URB NOT IN PROGRESS\n");
		mos7840_port->read_urb->dev = serial->dev;
		
		status = usb_submit_urb(mos7840_port->read_urb, GFP_ATOMIC);
		if (status) 
			DPRINTK(" usb_submit_urb(read bulk) failed, status = %d", status);
	}
	DPRINTK("mos7840_change_port_settings mos7840_port->shadowLCR is End %x\n", mos7840_port->shadowLCR);

	return;
}



/************************************************************************/
/************************************************************************/
/*            U S B   P A R P O R T   F U N C T I O N S                 */
/*            U S B   P A R P O R T   F U N C T I O N S                 */
/************************************************************************/
/************************************************************************/


/*****************************************************************************
 * ECR modes
 *****************************************************************************/

#define ECR_SPP 00
#define ECR_PS2 01
#define ECR_PPF 02
#define ECR_ECP 03
#define ECR_EPP 04


/*****************************************************************************
 * SendMosCmd
 *	this function will be used for sending command to device
 * Input : 5 Input
 *			pointer to the serial device,
 *			request type
 *			value
 *			register index
 *			pointer to data/buffer
 *****************************************************************************/
static int SendMosCmd(void *pp, __u8 request, __u16 value, __u16 index, void *data)
{
        int timeout;
        int status;
        __u8 requesttype;
        __u16 size;
	unsigned int Pipe;
	struct usb_device *usbdev;
	void *buf = NULL;
	
	if(value) /* value -- 0 == Parport   >0: Serail */ {
		
		usbdev = (struct usb_device *)(( struct usb_serial *)pp)->dev;
	} else {
		struct parport_ax781x0_private *priv =(struct parport_ax781x0_private *)((struct parport *)pp)->private_data;
		usbdev = priv->usbdev;
	}

        size = 0x00;
        timeout = MOS_WDR_TIMEOUT;
	if (value == 0)
		value = 0x0a00;
	else 
		value = value << 8;
	
    	if (request == MOS_WRITE) {
		request = (__u8)MOS_WRITE;
		requesttype = (__u8)0x40;
		value  = value + (__u16)*((unsigned char *)data);
		buf = NULL;
		Pipe = usb_sndctrlpipe(usbdev, 0);
	} else {
		request = (__u8)MOS_READ;
		requesttype = (__u8)0xC0;
		size = 0x01;
		Pipe = usb_rcvctrlpipe(usbdev,0);

		buf = kmalloc(size, GFP_KERNEL);
		if (!buf)
			return -ENOMEM;
    	}

	status = usb_control_msg(                           \
	    usbdev,                                         \
	    Pipe,			                    \
	    request,                                        \
	    requesttype,                                    \
	    value,                                          \
	    index,                                          \
	    buf,                                            \
	    size,                                           \
	    timeout );	
	
	if(status < 0) {
		DPRINTK("Write fail-Val %x index %x-status:%d\n",value,index,status);
		return status; 
	}
	if (request == MOS_READ) {
		if (data)
			memcpy(data, buf, size);
		kfree(buf);
	}

	return status;
}


/*****************************************************************************
 * change_mode_mos
 *	this function will be used to change ECR modes
 * Input : 2 Input
 *			pointer to the parport,
 *		        mode to switch to	
 *****************************************************************************/


static void change_mode_mos(struct parport *pp, int m)
{
	unsigned char data;

	DPRINTK("%s\n","Entering...");

	data = 0x00;
	SendMosCmd(pp,MOS_READ,0,0x0A, &data);

	data = data | (m<<5);	
	SendMosCmd(pp,MOS_WRITE,0,0x0A, &data);
}

/*****************************************************************************
 * clear_epp_timeout
 *	this function will be used to clear TIMEOUT BIT in EPP MODE
 * Input : 1 Input
 *			pointer to the parport,
 *****************************************************************************/
 
static int clear_epp_timeout(struct parport *pp)
{
	unsigned char stat;

	DPRINTK("%s\n","Entering...");

	stat = 0x00;
	SendMosCmd(pp,MOS_READ,0,0x01, &stat);

	return stat & 1;
}


/*****************************************************************************
 * parport_ax781x0_write_data
 *	this function will be used to write data to Reg0 - Data Reg 
 * Input : 2 Input
 *			pointer to the parport,
 *			character to be written
 *****************************************************************************/

static void parport_ax781x0_write_data(struct parport *pp, unsigned char d)
{
	/*DPRINTK("%s\n","Entering...");

	SendMosCmd(pp,MOS_WRITE,0,0x00, &d);

	DPRINTK("%s\n","Leaving...");*/
}

/*****************************************************************************
 * parport_ax781x0_read_data
 *	this function will be used to read data from Reg0 - Data Reg 
 * Input : 1 Input
 *			pointer to the parport
 *****************************************************************************/

static unsigned char parport_ax781x0_read_data(struct parport *pp)
{
	unsigned char ret;

	DPRINTK("%s\n","Entering...");

	SendMosCmd(pp,MOS_READ,0,0x00, &ret);
	
	DPRINTK("%s\n","Leaving...");
	return ret;
}

/*****************************************************************************
 * parport_ax781x0_write_control 
 *	this function will be used to write DCR Reg 
 * Input : 2 Input
 *			pointer to the parport,
 *			value to be written
 *****************************************************************************/

static void parport_ax781x0_write_control(struct parport *pp, unsigned char d)
{
	struct parport_ax781x0_private *priv = pp->private_data;	

	DPRINTK("%s\n","Entering...");

	d = (d & 0xf) | (priv->reg[1] & 0xf0);
        
	SendMosCmd(pp,MOS_WRITE,0,0x02, &d);

	priv->reg[1] = d;

	DPRINTK("%s\n","Leaving...");
}

/*****************************************************************************
 * parport_ax781x0_read_control 
 *	this function will be used to read DCR Reg 
 * Input : 1 Input
 *			pointer to the parport
 *****************************************************************************/

static unsigned char parport_ax781x0_read_control(struct parport *pp)
{
/*	struct parport_ax781x0_private *priv = pp->private_data;	

	DPRINTK("%s\n","Entering...");

	return priv->reg[1] & 0xf; // Use sft cpy 

	DPRINTK("%s\n","Leaving...");
*/
	unsigned char ret;
	unsigned char control_reg;

	DPRINTK("%s\n","Entering...");

	ret = SendMosCmd(pp,MOS_READ,0,0x02, &control_reg);
	DPRINTK("%s\n","Leaving...");

	return control_reg;
}

/*****************************************************************************
 * parport_ax781x0_frob_control 
 *	this function will be used to alter ECR Reg 
 * Input : 3 Input
 *			pointer to the parport,
 *			mask value,
 *			value to be written		
 *****************************************************************************/

static unsigned char parport_ax781x0_frob_control(struct parport *pp, unsigned char mask, unsigned char val)
{
	unsigned char d;
	struct parport_ax781x0_private *priv = pp->private_data;	

	DPRINTK("%s\n","Entering...");

	//DPRINTK("%x	%x\n",(unsigned int)pp,(unsigned int)priv);

	mask &= 0x0f;
	val &= 0x0f;
	d = (priv->reg[1] & (~mask)) ^ val;

	SendMosCmd(pp,MOS_WRITE,0,0x02, &d);
	priv->reg[1] = d;

	DPRINTK("%s\n","Leaving...");

	
	return d & 0xf;
}

/*****************************************************************************
 * parport_ax781x0_read_status 
 *	this function will be used to read DSR Reg
 * Input : 1 Input
 *			pointer to the parport
 *****************************************************************************/

static unsigned char parport_ax781x0_read_status(struct parport *pp)
{
	unsigned char data;

	DPRINTK("%s\n","Entering...");

        SendMosCmd(pp,MOS_READ,0,0x01, &data);
	DPRINTK("%s\n","Leaving...");

	return data & 0xf8;
	//data &= 0xf8;
	//data |= 0x08;
	
	//return data ; 
}


/*****************************************************************************
 * parport_ax781x0_disable_irq - Future Use
 *	this function will be used to disable irq 
 * Input : 1 Input
 *			pointer to the parport
 *****************************************************************************/
static void parport_ax781x0_disable_irq(struct parport *pp)
{
 #if 0
	unsigned char d;
	struct parport_ax781x0_private *priv = pp->private_data;	

	DPRINTK("%s\n","Entering...");
 #endif
}

/*****************************************************************************
 * parport_ax781x0_enable_irq - Future Use
 *	this function will be used to enable irq 
 * Input : 1 Input
 *			pointer to the parport
 *****************************************************************************/

static void parport_ax781x0_enable_irq(struct parport *pp)
{
 #if 0
	unsigned char d;
	struct parport_ax781x0_private *priv = pp->private_data;	

	DPRINTK("%s\n","Entering...");
#endif
}

/*****************************************************************************
 * parport_ax781x0_data_forward
 *	this function will be used to set P[0-7] to O/P mode
 * Input : 1 Input
 *			pointer to the parport
 *****************************************************************************/

static void parport_ax781x0_data_forward (struct parport *pp)
{
	unsigned char d;
	struct parport_ax781x0_private *priv = pp->private_data;	

	DPRINTK("%s\n","Entering...");

	d = priv->reg[1] & ~0x20;

	SendMosCmd(pp,MOS_WRITE,0,0x02, &d);
	priv->reg[1] = d;

	DPRINTK("%s\n","Leaving...");
}

/*****************************************************************************
 * parport_ax781x0_data_reverse
 *	this function will be used to set P[0-7] to I/P mode
 * Input : 1 Input
 *			pointer to the parport
 *****************************************************************************/
static void parport_ax781x0_data_reverse (struct parport *pp)
{
	unsigned char d;
	struct parport_ax781x0_private *priv = pp->private_data;

	DPRINTK("%s\n","Entering...");

	d = priv->reg[1] | 0x20;

	SendMosCmd(pp,MOS_WRITE,0,0x02, &d);

	priv->reg[1] = d;

	DPRINTK("%s\n","Leaving...");
}

/*****************************************************************************
 * parport_ax781x0_init_state
 *	this function will be used to set DCR & ECR to initial values
 * Input : 2 Input
 *			pointer to pardevice
 *			pointer to parport_state
 *****************************************************************************/

static void parport_ax781x0_init_state(struct pardevice *dev, struct parport_state *s)
{
	DPRINTK("%s\n","Entering...");
 
	s->u.pc.ctr = 0x0c | (dev->irq_func ? 0x10 : 0x0);
	//s->u.pc.ctr = 0x0c;
	s->u.pc.ecr = MOS_ECR_MODE_INIT;

	DPRINTK("%s\n","Leaving...");
}

/*****************************************************************************
 * parport_ax781x0_save_state
 *	this function will be used to save DCR & ECR values
 * Input : 2 Input
 *			pointer to parport
 *			pointer to parport_state
 *****************************************************************************/

static void parport_ax781x0_save_state(struct parport *pp, struct parport_state *s)
{
	unsigned char data;	
	struct parport_ax781x0_private *priv = pp->private_data;	

	DPRINTK("%s\n","Entering...");
        
	data = 0x00;
        SendMosCmd(pp,MOS_READ,0,0x02, &data);
	s->u.pc.ctr = data;
	priv->reg[1] = data;

	data = 0x00;
        SendMosCmd(pp,MOS_READ,0,0x0A, &data);
	s->u.pc.ecr = data;
	priv->reg[2] = data; 

	DPRINTK("%s\n","Leaving...");
}

static int restore_state_thread(void *vpp)
{
	struct parport 			*pp = vpp;
	struct parport_ax781x0_private 	*priv = pp->private_data;

	if (priv == NULL) {
		printk("parport_ax781x0_private not exist.\n");
		return -1;
	}
	
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0)
	daemonize("otg_hcd_thread");
#endif

	allow_signal(SIGKILL);

	DPRINTK("%s \n","thread ...\n");
	while ( 1 ) {
		//interruptible_sleep_on(&restore_state_event);
		priv->thread_cond = 0;
		wait_event_interruptible(priv->restore_state_event,(priv->thread_cond == 1));
		//wait_event(restore_state_event,(thread_cond==1));

       		DPRINTK("%s \n","restore_state_thread:Thread Wait...\n");
		if(priv->ThreadState) {
			priv->ThreadState = 0;
          	
			DPRINTK("%s \n","Stopping restore_state_thread thread ...");
			up(&priv->thread_complete);
          		break;
       		}

		SendMosCmd(pp,MOS_WRITE,0,0x02, &priv->dcr);
		SendMosCmd(pp,MOS_WRITE,0,0x0A, &priv->ecr);
	}

	
	return 0;
} 
/*****************************************************************************
 * parport_ax781x0_restore_state
 *	this function will be used to restore DCR & ECR values  
 * Input : 2 Input
 *			pointer to parport
 *			pointer to parport_state
 *****************************************************************************/

static void parport_ax781x0_restore_state(struct parport *pp, struct parport_state *s)
{
	struct parport_ax781x0_private 	*priv = NULL;
	unsigned char data;	

	DPRINTK("%s\n","Entering...");
	
	priv = pp->private_data;
	if (priv == NULL)
		return;

	if(s==NULL)
	  	return;

	data = s->u.pc.ctr;
	DPRINTK("%s %x\n","val restrd DCR=",data);
	priv->dcr = data;

	data = s->u.pc.ecr;
	DPRINTK("%s %x\n","val restrd ECR=",data);
	priv->ecr = data;

	//wake_up_interruptible(&restore_state_event);
	wake_up(&priv->restore_state_event); 
	priv->thread_cond = 1;

	DPRINTK("%s\n","Leaving...");
}

/*****************************************************************************
 * parport_ax781x0_epp_read_data
 *	this function will be used to read data in EPP mode
 * Input : 4 Input
 *			pointer to the parport
 *			buffer
 *			length
 *			flags
 *****************************************************************************/

static size_t parport_ax781x0_epp_read_data(struct parport *pp, void *buf, size_t length, int flags)
{
	size_t got = 0;
	unsigned char data;

	DPRINTK("%s\n","Entering...");

	change_mode_mos(pp, ECR_EPP);

	for (; got < length; got++) {
        	SendMosCmd(pp,MOS_WRITE,0,0x04,(unsigned char *) buf);
		buf++;
        	SendMosCmd(pp,MOS_READ,0,0x01,&data);
		if (data & 0x01) {
			clear_epp_timeout(pp);
			break;
		}
	}
	
	change_mode_mos(pp, ECR_PS2);
	
	DPRINTK("%s\n","Leaving...");
	
	
	return got;
}

/*****************************************************************************
 * parport_ax781x0_epp_write_data
 *	this function will be used to write data in EPP mode
 * Input : 4 Input
 *			pointer to the parport
 *			buffer
 *			length
 *			flags
 *****************************************************************************/

static size_t parport_ax781x0_epp_write_data(struct parport *pp, const void *buf, size_t length, int flags)
{
	int i;
	int rlen;
	struct parport_ax781x0_private *priv = pp->private_data;
	struct usb_device *usbdev = priv->usbdev;

	if (!usbdev)
		return 0;

	DPRINTK("%s\n","Entering...");

	change_mode_mos(pp, ECR_EPP);

	i = usb_bulk_msg(usbdev, usb_sndbulkpipe(usbdev, 4), (void *)buf, length, &rlen, HZ*20);
	if (i)
		printk(KERN_ERR "ax781x0: sendbulk ep 2 buf %p len %Zu rlen %u\n", buf, length, rlen);

	change_mode_mos(pp, ECR_PS2); 

	DPRINTK("%s\n","Leaving...");
	
	
	return rlen;
}

/*****************************************************************************
 * parport_ax781x0_epp_read_addr
 *	this function will be used to read addrs in EPP mode
 * Input : 4 Input
 *			pointer to the parport
 *			buffer
 *			length
 *			flags
 *****************************************************************************/

static size_t parport_ax781x0_epp_read_addr(struct parport *pp, void *buf, size_t length, int flags)
{
	size_t got = 0;
	unsigned char data;

	DPRINTK("%s\n","Entering...");

	change_mode_mos(pp, ECR_EPP);

	for (; got < length; got++) {
        	SendMosCmd(pp,MOS_READ,0,0x03, (unsigned char *)buf);
		buf++;
		data = 0x00;
        	SendMosCmd(pp,MOS_READ,0,0x01, &data);
		if (data & 0x01) {
			clear_epp_timeout(pp);
			break;
		}
	}

	change_mode_mos(pp, ECR_PS2);
	DPRINTK("%s\n","Leaving...");
	

	return got;
}

/*****************************************************************************
 * parport_ax781x0_epp_write_addr
 *	this function will be used to write addrs in EPP mode
 * Input : 4 Input
 *			pointer to the parport
 *			buffer
 *			length
 *			flags
 *****************************************************************************/

static size_t parport_ax781x0_epp_write_addr(struct parport *pp, const void *buf, size_t length, int flags)
{
	unsigned char data;
	size_t written = 0;
	
	DPRINTK("%s\n","Entering...");

	change_mode_mos(pp, ECR_EPP);

	for (; written < length; written++) {
        	SendMosCmd(pp,MOS_WRITE,0,0x03,(unsigned char *) buf);
		buf++;
		data = 0x00;
        	SendMosCmd(pp,MOS_READ,0,0x01, &data);
		if (data & 0x01) {
			clear_epp_timeout(pp);
			break;
		}
	}
	change_mode_mos(pp, ECR_PS2);

	DPRINTK("%s\n","Leaving...");
	
	return written;
}

/*****************************************************************************
 * parport_ax781x0_ecp_write_data
 *	this function will be used to write data in ECP mode
 * Input : 4 Input
 *			pointer to the parport
 *			buffer
 *			length
 *			flags
 *****************************************************************************/

static size_t parport_ax781x0_ecp_write_data(struct parport *pp, const void *buffer, size_t len, int flags)
{
	int i;
	int rlen;
	struct parport_ax781x0_private *priv = pp->private_data;
	struct usb_device *usbdev = priv->usbdev;
	
	if (!usbdev)
		return 0;

	DPRINTK("%s\n","Entering...");

	change_mode_mos(pp, ECR_ECP);

	i = usb_bulk_msg(usbdev, usb_sndbulkpipe(usbdev, 4), (void *)buffer, len, &rlen, HZ*20);
	if (i)
		printk(KERN_ERR "ax781x0: sendbulk ep 2 buf %p len %Zu rlen %u\n", buffer, len, rlen);

	change_mode_mos(pp, ECR_PS2);
	DPRINTK("%s\n","Leaving...");
	
	return rlen;
}

/*****************************************************************************
 * parport_ax781x0_ecp_read_data
 *	this function will be used to read data in ECP mode
 * Input : 4 Input
 *			pointer to the parport
 *			buffer
 *			length
 *			flags
 *****************************************************************************/

static size_t parport_ax781x0_ecp_read_data(struct parport *pp, void *buffer, size_t len, int flags)
{
	int i;
	int rlen;
	struct parport_ax781x0_private *priv = pp->private_data;
	struct usb_device *usbdev = priv->usbdev;
	if (!usbdev)
		return 0;

	DPRINTK("%s\n","Entering...");

	change_mode_mos(pp, ECR_ECP);

	i = usb_bulk_msg(usbdev, usb_rcvbulkpipe(usbdev, 3), buffer, len, &rlen, HZ*20);
	if (i)
		printk(KERN_ERR "ax781x0: recvbulk ep 1 buf %p len %Zu rlen %u\n", buffer, len, rlen);

	change_mode_mos(pp, ECR_PS2);

	DPRINTK("%s\n","Leaving...");
	
	return rlen;
}



/*****************************************************************************
 * parport_ax781x0_ecp_write_addr
 * This function will be used to write addrs in ECP mode
 * Input : 4 Input
 *			pointer to the parport
 *			buffer
 *			length
 *			flags
 *****************************************************************************/

static size_t parport_ax781x0_ecp_write_addr(struct parport *pp, const void *buffer, size_t len, int flags)
{
	size_t written = 0;
	DPRINTK("%s\n","Entering...");

	change_mode_mos(pp, ECR_ECP);

	for (; written < len; written++) {
        	SendMosCmd(pp,MOS_WRITE,0,0x08, (void *)buffer);
		buffer++;
	}

	change_mode_mos(pp, ECR_PS2);

	DPRINTK("%s\n","Leaving...");
	
	return written;
}

/*****************************************************************************
 * parport_ax781x0_write_compat
 *	this function will be used to write data in PPF/CB-FIFO mode
 * Input : 4 Input
 *			pointer to the parport
 *			buffer
 *			length
 *			flags
 *****************************************************************************/
static size_t parport_ax781x0_write_compat(struct parport *pp, const void *buffer, size_t len, int flags)
{
	int i;
	int rlen;
	struct parport_ax781x0_private *priv = pp->private_data;
	struct usb_device *usbdev = priv->usbdev;

	if (!usbdev)
		return 0;

	change_mode_mos(pp, ECR_PPF);

	i = usb_bulk_msg(usbdev, usb_sndbulkpipe(usbdev, 4), (void *)buffer, len, &rlen, HZ*20);

	//DPRINTK("%s %x\n","length = ",len);
	if (i)
		printk(KERN_ERR "ax781x0: sendbulk ep 4 buf %p len %Zu rlen %u\n", buffer, len, rlen);

	change_mode_mos(pp, ECR_PS2);
	DPRINTK("%s\n","Leaving...");
	
	return rlen;
}

/****************************************************************************
 * parport_ax781x0_ops
 *	this structure holds the function names of all the possible 
 *      operations on parport
****************************************************************************/
static struct parport_operations parport_ax781x0_ops = 
{
        .owner =                THIS_MODULE,
        .write_data =           parport_ax781x0_write_data,
        .read_data =            parport_ax781x0_read_data,

        .write_control =        parport_ax781x0_write_control,
        .read_control =         parport_ax781x0_read_control,
        .frob_control =         parport_ax781x0_frob_control,

        .read_status =          parport_ax781x0_read_status,

        .enable_irq =           parport_ax781x0_enable_irq,
        .disable_irq =          parport_ax781x0_disable_irq,

        .data_forward =         parport_ax781x0_data_forward,
        .data_reverse =         parport_ax781x0_data_reverse,

        .init_state =           parport_ax781x0_init_state,
        .save_state =           parport_ax781x0_save_state,
        .restore_state =        parport_ax781x0_restore_state,

        .epp_write_data =       parport_ax781x0_epp_write_data,
        .epp_read_data =        parport_ax781x0_epp_read_data,
        .epp_write_addr =       parport_ax781x0_epp_write_addr,
        .epp_read_addr =        parport_ax781x0_epp_read_addr,

        .ecp_write_data =       parport_ax781x0_ecp_write_data,
        .ecp_read_data =        parport_ax781x0_ecp_read_data,
        .ecp_write_addr =       parport_ax781x0_ecp_write_addr,

        .compat_write_data =    parport_ax781x0_write_compat,
        .nibble_read_data =     parport_ieee1284_read_nibble,
        .byte_read_data =       parport_ieee1284_read_byte,
};


static int mos7810_check(struct usb_serial *serial)
{
	int i, pass_count = 0;
	__u16 *mcr_data = NULL;
	__u16 *data = NULL;
	__u16 test_pattern = 0x55AA;
	

	mcr_data = (__u16 *)kmalloc(4, GFP_KERNEL);
	if (!mcr_data)
		return -ENOMEM;

	data = &mcr_data[1];

	// Store MCR setting 
	usb_control_msg(serial->dev, usb_rcvctrlpipe(serial->dev, 0),
		MCS_RDREQ, MCS_RD_RTYPE, 0x0300, MODEM_CONTROL_REGISTER,
		mcr_data, VENDOR_READ_LENGTH, MOS_WDR_TIMEOUT);

	for (i = 0; i < 16; i++) {
		// Send the 1-bit test pattern out to MCS7810 test pin 
		usb_control_msg(serial->dev, usb_sndctrlpipe(serial->dev, 0),
			MCS_WRREQ, MCS_WR_RTYPE, (0x0300 | (((test_pattern >> i) & 0x0001) << 1)),
			MODEM_CONTROL_REGISTER, NULL, 0, MOS_WDR_TIMEOUT);

		// Read the test pattern back 
		usb_control_msg(serial->dev, usb_rcvctrlpipe(serial->dev, 0),
			MCS_RDREQ, MCS_RD_RTYPE, 0, GPIO_REGISTER, data,
			VENDOR_READ_LENGTH, MOS_WDR_TIMEOUT);

		// If this is a MCS7810 device, both test patterns must match 
		if (((test_pattern >> i) ^ (~(*data) >> 1)) & 0x0001)
			break;

		pass_count++;
	}

	// Restore MCR setting 
	usb_control_msg(serial->dev, usb_sndctrlpipe(serial->dev, 0), MCS_WRREQ,
		MCS_WR_RTYPE, 0x0300 | *mcr_data, MODEM_CONTROL_REGISTER, NULL,
		0, MOS_WDR_TIMEOUT);

	kfree(mcr_data);

	if (pass_count == 16)
		return 1;

	
	return 0;
}


#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,12,0)
static int mos7840_calc_num_ports(struct usb_serial *serial, struct usb_serial_endpoints *epds)
#else
static int mos7840_calc_num_ports(struct usb_serial *serial)
#endif
{
	int mos7840_num_ports = 0;
	unsigned char * ep9CtlRegVal;
	struct moschip_serial *mos7840_serial;
	unsigned char * pData;
	int Device_type = 0;	

	pData = kmalloc(4, GFP_KERNEL);
	if (!pData)
		return -1;
	memset(pData, 0, 4);
	ep9CtlRegVal = &pData[2];

	/* create our private serial structure */
	#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,17)
	mos7840_serial = kzalloc (sizeof(struct moschip_serial), GFP_KERNEL);
	#else
	mos7840_serial = kmalloc (sizeof(struct moschip_serial), GFP_KERNEL);
	#endif
	if (mos7840_serial == NULL) {
		DPRINTK("%s - Out of memory", __FUNCTION__);
		return -ENOMEM;
	}

	/* resetting the private structure field values to zero */
	memset (mos7840_serial, 0, sizeof(struct moschip_serial));

	mos7840_serial->serial = serial;

	mos7840_set_serial_private(serial,mos7840_serial);
	
#ifdef AUTO_SWITCH
	Device_type = MOSCHIP_DEVICE_ID_7840;
#else

	usb_control_msg(serial->dev, usb_rcvctrlpipe(serial->dev, 0),
		MCS_RDREQ, MCS_RD_RTYPE, 0, GPIO_REGISTER, pData,
		VENDOR_READ_LENGTH, MOS_WDR_TIMEOUT);

	if (serial->dev->descriptor.idProduct == MOSCHIP_DEVICE_ID_7810 ||
	    serial->dev->descriptor.idProduct == MOSCHIP_DEVICE_ID_7820) {
		Device_type = serial->dev->descriptor.idProduct;
	} else {
		if ((*pData & 0x01) == 1) {
			Device_type = MOSCHIP_DEVICE_ID_7840;
			usb_control_msg(serial->dev, usb_rcvctrlpipe(serial->dev, 0),
				MCS_RDREQ, MCS_RD_RTYPE, 0, 0x47, ep9CtlRegVal,
				VENDOR_READ_LENGTH, MOS_WDR_TIMEOUT);
			usb_control_msg(serial->dev, usb_sndctrlpipe(serial->dev, 0), 
				MCS_WRREQ,MCS_WR_RTYPE, 0x11, 0x47, NULL,
				0, MOS_WDR_TIMEOUT);
			usb_control_msg(serial->dev, usb_rcvctrlpipe(serial->dev, 0),
				MCS_RDREQ, MCS_RD_RTYPE, 0, 0x47, pData,
				VENDOR_READ_LENGTH, MOS_WDR_TIMEOUT);
			usb_control_msg(serial->dev, usb_sndctrlpipe(serial->dev, 0), 
				MCS_WRREQ,MCS_WR_RTYPE, *ep9CtlRegVal, 0x47, NULL,
				0, MOS_WDR_TIMEOUT);
			if (*pData == 0x11) {
				usb_control_msg(serial->dev, usb_rcvctrlpipe(serial->dev, 0),
					MCS_RDREQ,MCS_RD_RTYPE, 0, 0x4f, pData,
					VENDOR_READ_LENGTH, MOS_WDR_TIMEOUT);
				mos7840_serial->parallelport = *pData;// set parallel port exist				
				if (mos7840_serial->parallelport) {
					if (serial->dev->descriptor.idProduct == MOSCHIP_DEVICE_ID_7841) {
						Device_type = MOSCHIP_DEVICE_ID_7841;
						mos7840_num_ports = 1;
					} else
						mos7840_num_ports = 2;
					goto finally;
				}
			}	
		}
		else if (mos7810_check(serial))
			Device_type = MOSCHIP_DEVICE_ID_7810;
		else
			Device_type = MOSCHIP_DEVICE_ID_7820;
	}
#endif
	if (serial->dev->descriptor.idProduct == MOSCHIP_DEVICE_ID_7843 &&
	    serial->dev->descriptor.idVendor  == USB_VENDOR_ID_MOSCHIP) {
		Device_type = MOSCHIP_DEVICE_ID_7843;
		mos7840_num_ports = 3;
	} else
		mos7840_num_ports = (Device_type >> 4) & 0x000F;

	if (serial->dev->descriptor.idProduct == MOSCHIP_DEVICE_ID_7842 &&
	    serial->dev->descriptor.idVendor  == USB_VENDOR_ID_MOSCHIP) {
		Device_type = MOSCHIP_DEVICE_ID_7842;
		mos7840_num_ports = 2;
	} else
		mos7840_num_ports = (Device_type >> 4) & 0x000F;
	
finally:
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)
	serial->type->num_bulk_in=mos7840_num_ports;
	serial->type->num_bulk_out=mos7840_num_ports;
	serial->type->num_ports=mos7840_num_ports;
#else
	serial->num_bulk_in = mos7840_num_ports;
	serial->num_bulk_out = mos7840_num_ports;
	serial->num_ports = mos7840_num_ports;
#endif
	mos7840_serial->device_type = Device_type;
	printk("Number of port: %d\n", mos7840_num_ports);
	
	return mos7840_num_ports;
}




/****************************************************************************
 * mos7840_startup
 ****************************************************************************/

static int mos7840_startup (struct usb_serial *serial)
{
	struct moschip_serial *mos7840_serial;
	struct moschip_port *mos7840_port;
	struct usb_device *dev;
	int i, status, response;
	__u16 Data;

	int device_type;

	
	DPRINTK("%s \n"," mos7840_startup :entering..........");
	if (!serial) {
		DPRINTK("%s\n","Invalid Handler");
		return -1;
	}

	dev = serial->dev;
	
	DPRINTK("%s\n","Entering...");

	mos7840_serial = mos7840_get_serial_private(serial);
	
	mos7840_serial->status_polling_started = FALSE;	

	printk("device_type: %x\n", mos7840_serial->device_type);
	device_type = mos7840_serial->device_type;

	for (i = 0; i < serial->num_ports; ++i) {
		mos7840_port = kmalloc(sizeof(struct moschip_port), GFP_KERNEL);
		if (mos7840_port == NULL) {
			DPRINTK("%s - Out of memory", __FUNCTION__);
			mos7840_set_serial_private(serial,NULL);
			kfree(mos7840_serial);
			return -ENOMEM;
		}

		memset(mos7840_port, 0, sizeof(struct moschip_port));

		/* Initialize all port interrupt end point to port 0 int endpoint *
		* Our device has only one interrupt end point comman to all port */

		mos7840_port->port = serial->port[i];
		mos7840_set_port_private(serial->port[i], mos7840_port);

		spin_lock_init(&mos7840_port->pool_lock);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,11,0)
		serial->port[i]->port_number = i;
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,21)
		serial->port[i]->number = i;
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,11,0)
		mos7840_port->port_num=(serial->port[i]->port_number + 1);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
		mos7840_port->port_num=(serial->port[i]->number + 1);
#else
		mos7840_port->port_num=((serial->port[i]->number - \
	  			 (serial->port[i] ->serial->minor)) + 1);
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,11,0)
		mos7840_port->AppNum = (((__u16)serial->port[i]->port_number) + 1) << 8;
#else
		mos7840_port->AppNum = (((__u16)serial->port[i]->number - \
				(__u16)(serial->port[i] ->serial->minor)) + 1) << 8;
#endif	
		/*if (mos7840_serial->parallelport && i == 2) {
			mos7840_port->AppNum = 0x0a00;				
			mos7840_Dump_serial_port(mos7840_port);
			mos7840_set_port_private(serial->port[i], mos7840_port);			
			break ;
		}*/
	
		if (mos7840_port->port_num == 1) {
			mos7840_port->SpRegOffset = 0x0;
			mos7840_port->ControlRegOffset = 0x1;
			mos7840_port->DcrRegOffset = 0x4 ;
			mos7840_port->IcgRegOffset = 0x2C;
			mos7840_port->SpThresholdOffset = THRESHOLD_VAL_SP1_1;
			mos7840_port->ClkSelectRegOffset = 0x13;
		} else if (mos7840_port->port_num == 2 && !mos7840_serial->parallelport && (device_type == MOSCHIP_DEVICE_ID_7840 || device_type == MOSCHIP_DEVICE_ID_7843 || device_type == MOSCHIP_DEVICE_ID_7842)) {
			mos7840_port->SpRegOffset = 0x8;
			mos7840_port->ControlRegOffset = 0x9;
			mos7840_port->DcrRegOffset = 0x16;
			mos7840_port->IcgRegOffset = 0x2D;
			mos7840_port->SpThresholdOffset = THRESHOLD_VAL_SP2_1;
			mos7840_port->ClkSelectRegOffset = 0x13;
		} else if ((mos7840_port->port_num == 3 ) || 
			(mos7840_port->port_num == 2 && (mos7840_serial->parallelport || device_type == MOSCHIP_DEVICE_ID_7820))) {
			mos7840_port->SpRegOffset = 0xa;
			mos7840_port->ControlRegOffset = 0xb;
			mos7840_port->DcrRegOffset = 0x19;
			mos7840_port->IcgRegOffset = 0x2E;
			mos7840_port->SpThresholdOffset = THRESHOLD_VAL_SP3_1;
			mos7840_port->ClkSelectRegOffset = 0x14;
		} else if (mos7840_port->port_num == 4) {
			mos7840_port->SpRegOffset = 0xc;
			mos7840_port->ControlRegOffset = 0xd;
			mos7840_port->DcrRegOffset = 0x1c;
			mos7840_port->IcgRegOffset = 0x2F;
			mos7840_port->SpThresholdOffset = THRESHOLD_VAL_SP4_1;
			mos7840_port->ClkSelectRegOffset = 0x14;
		} else {
			return -1;
		}
			
		mos7840_Dump_serial_port(mos7840_port);
		
		// write the default ICG value = 0
		Data = 0x00;
		status = 0;
		status = mos7840_set_reg_sync(serial->port[i], mos7840_port->IcgRegOffset, Data);
		if (status < 0) {
			DPRINTK("Writing ICG failed status-0x%x\n", status);
			break;
		} else {
			DPRINTK("ICG Writing success status%d\n", status);
		}	
	
		//enable rx_disable bit in control register
		status = mos7840_get_reg_sync(serial->port[i], mos7840_port->ControlRegOffset, &Data);
		if (status < 0) {
			DPRINTK("Reading ControlReg failed status-0x%x\n", status);
			break;
		} else {
			DPRINTK("ControlReg Reading success val is %x, status%d\n", Data, status);
		}
		
		Data |= 0x08;//setting driver done bit
		Data |= 0x04;//sp1_bit to have cts change reflect in modem status reg				
		
		status = 0;
		status = mos7840_set_reg_sync(serial->port[i], mos7840_port->ControlRegOffset, Data);
		if (status < 0) {
			DPRINTK("Writing ControlReg failed(rx_disable) status-0x%x\n", status);
			break;
		} else {
			DPRINTK("ControlReg Writing success(rx_disable) status%d\n", status);
		}
		
		//Write default values in DCR (i.e 0x01 in DCR0, 0x05 in DCR2 and 0x24 in DCR3
		Data = 0x01;
		status = 0;
		status = mos7840_set_reg_sync(serial->port[i], (__u16)(mos7840_port->DcrRegOffset + 0), Data);
		if (status < 0) {
			DPRINTK("Writing DCR0 failed status-0x%x\n", status);
			break;
		} else {
			DPRINTK("DCR0 Writing success status%d\n", status);
		}
		
		Data = 0x05;
		status = 0;
		status = mos7840_set_reg_sync(serial->port[i], (__u16)(mos7840_port->DcrRegOffset + 1), Data);
		if (status < 0) {
			DPRINTK("Writing DCR1 failed status-0x%x\n", status);
			break;
		} else {
			DPRINTK("DCR1 Writing success status%d\n", status);
		}
		
		Data = 0x24;
		status = 0;
		status = mos7840_set_reg_sync(serial->port[i], (__u16)(mos7840_port->DcrRegOffset + 2), Data);
		if (status < 0) {
			DPRINTK("Writing DCR2 failed status-0x%x\n", status);
			break;
		} else {
			DPRINTK("DCR2 Writing success status%d\n", status);
		}
		
		// write values in clkstart0x0 and clkmulti 0x20	
		Data = 0x0;
		status = 0;
		status = mos7840_set_reg_sync(serial->port[i], CLK_START_VALUE_REGISTER, Data);
		if (status < 0) {
			DPRINTK("Writing CLK_START_VALUE_REGISTER failed status-0x%x\n", status);
			break;
		} else {
			DPRINTK("CLK_START_VALUE_REGISTER Writing success status%d\n", status);
		}
	
		Data = 0x20;
		status = 0;
		status = mos7840_set_reg_sync(serial->port[i], CLK_MULTI_REGISTER, Data);
		if (status < 0) {
			DPRINTK("Writing CLK_MULTI_REGISTER failed status-0x%x\n", status);
			break;
		} else {
			DPRINTK("CLK_MULTI_REGISTER Writing success status%d\n", status);
		}
	
		mos7840_port->control_urb = usb_alloc_urb(0, GFP_ATOMIC);
		mos7840_port->control_urb_for_throttle = usb_alloc_urb(0, GFP_ATOMIC);

		mos7840_port->ctrl_buf = kmalloc(16, GFP_KERNEL);
		mos7840_port->has_led = false;
	
		/* Initialize MCS7810 LED timers */
		if (device_type == MOSCHIP_DEVICE_ID_7810){

			mos7840_port->has_led = true;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
			init_timer(&mos7840_port->led_timer1);
			mos7840_port->led_timer1.function = mos7840_led_off;
			mos7840_port->led_timer1.expires = jiffies + msecs_to_jiffies(MCS7810_LED_ON_MS);
			mos7840_port->led_timer1.data = (unsigned long) mos7840_port;

			init_timer(&mos7840_port->led_timer2);
			mos7840_port->led_timer2.function = mos7840_led_flag_off;
			mos7840_port->led_timer2.expires = jiffies + msecs_to_jiffies(MCS7810_LED_OFF_MS);
			mos7840_port->led_timer2.data = (unsigned long) mos7840_port;
#else
			timer_setup(&mos7840_port->led_timer1, mos7840_led_off, 0);
			mos7840_port->led_timer1.expires = jiffies + msecs_to_jiffies(MCS7810_LED_ON_MS);

			timer_setup(&mos7840_port->led_timer2, mos7840_led_flag_off, 0);
			mos7840_port->led_timer2.expires = jiffies + msecs_to_jiffies(MCS7810_LED_OFF_MS);
#endif
			/* MCS7810 LED default turn off */			
			mos7840_port->led_flag = false;
			mos7840_set_led_sync(serial->port[i], MODEM_CONTROL_REGISTER, 0x0300);			
		}
		
		mos7840_port->rs_mode_port[0] = RS_MODE_DEFAULT_PORT1;
		mos7840_port->rs_mode_port[1] = RS_MODE_DEFAULT_PORT2;
		mos7840_port->rs_mode_port[2] = RS_MODE_DEFAULT_PORT3;
		mos7840_port->rs_mode_port[3] = RS_MODE_DEFAULT_PORT4;
	}
	
	//Zero Length flag enable
	Data = 0x0f;
	status = 0;
	status = mos7840_set_reg_sync(serial->port[0], ZLP_REG5, Data);
	if (status < 0) {
		DPRINTK("Writing ZLP_REG5 failed status-0x%x\n", status);
		return -1;
	} else 
		DPRINTK("ZLP_REG5 Writing success status%d\n", status);

	/* setting configuration feature to one */
	usb_control_msg(serial->dev, usb_sndctrlpipe(serial->dev, 0), (__u8)0x03, 0x00,0x01,0x00, 0x00, 0x00, 5*HZ);

	/********** parport **********/
	if (mos7840_serial->parallelport){
		struct parport *pp;
		struct parport_ax781x0_private *priv;
		unsigned char data;
		if (!(priv = kmalloc(sizeof(struct parport_ax781x0_private), GFP_KERNEL)))
			return -ENOMEM;

		/* resetting the private structure field values to zero */
		memset (priv, 0, sizeof(struct parport_ax781x0_private));
		if (!(pp = parport_register_port(0, PARPORT_IRQ_NONE, PARPORT_DMA_NONE, &parport_ax781x0_ops))) {
			printk(KERN_WARNING "Could not register parport\n");
			goto probe_abort;
		}
		
		pp->private_data = priv;
		priv->usbdev = serial->dev;
		pp->modes = PARPORT_MODE_PCSPP | PARPORT_MODE_TRISTATE | PARPORT_MODE_EPP | PARPORT_MODE_ECP | PARPORT_MODE_COMPAT;
		mos7840_serial->parp = pp;
		priv->mos_serial = mos7840_serial;

		/* Initialize the wait event for restore_state_thread */
		init_waitqueue_head(&priv->restore_state_event);

	#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,37)
		init_MUTEX_LOCKED(&priv->thread_complete);
	#else
		sema_init(&priv->thread_complete, 0);
	#endif

	#if LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0)
		priv->thread_pid = kernel_thread(restore_state_thread,pp,CLONE_KERNEL);
		if(priv->thread_pid < 0) {
			printk("HCD thread creation failed \n");
			return -ENODEV;
		}
	#else 
		if(!start_kthread(&priv->thread_pid, restore_state_thread, pp, "pp_thread"))
			printk("HCD thread creation failed \n");
	#endif
		
		data = 0x00;
		SendMosCmd(pp,MOS_READ,0,0x02, &data); 
		data = 0x00;
		SendMosCmd(pp,MOS_WRITE,0,0x02, &data);
		priv->reg[1] = data; /* DCR */
	
		data = 0x00;
		SendMosCmd(pp,MOS_READ,0,0x0A, &data);
		data = MOS_ECR_MODE_INIT;
		SendMosCmd(pp,MOS_WRITE,0,0x0A, &data);
		priv->reg[2] = data;
	
		DPRINTK("%s\n","Writing Registers Over\n");
		
		parport_announce_port(pp);
		return 0;
	probe_abort:
		kfree(priv);
		return -EPERM;
	}

	////////////////////////

/* Check to see if we've set up our endpoint info yet    *
 * (can't set it up in mos7840_startup as the structures *
 * were not set up at that time.)                        */
	mos7840_serial->status_polling_started = TRUE;
	/* If not yet set, Set here */
	mos7840_serial->interrupt_in_buffer = serial->port[0]->interrupt_in_buffer;
	mos7840_serial->interrupt_in_endpoint = serial->port[0]->interrupt_in_endpointAddress;
	mos7840_serial->interrupt_read_urb = serial->port[0]->interrupt_in_urb;

	/* set up interrupt urb */
	usb_fill_int_urb(                                   \
		mos7840_serial->interrupt_read_urb,     \
		serial->dev,                            \
		usb_rcvintpipe(serial->dev,mos7840_serial->interrupt_in_endpoint),    \
		mos7840_serial->interrupt_in_buffer,             \
		mos7840_serial->interrupt_read_urb->transfer_buffer_length,\
		mos7840_interrupt_callback, mos7840_serial,     \
		mos7840_serial->interrupt_read_urb->interval  );

	/* start interrupt read for mos7840               *
		* will continue as long as mos7840 is connected  */

	response = usb_submit_urb(mos7840_serial->interrupt_read_urb, GFP_KERNEL);
	if (response) {
		DPRINTK("%s - Error %d submitting interrupt urb", __FUNCTION__, response);
	}

	return 0;
}

static void mos7840_release(struct usb_serial *serial)
{
	struct moschip_serial *mos7840_serial = mos7840_get_serial_private(serial);

	mos7840_serial->status_polling_started = FALSE;

	if (mos7840_serial->interrupt_read_urb) {
		DPRINTK("%s","Shutdown interrupt_read_urb\n");
		mos7840_serial->interrupt_in_buffer = NULL;
		usb_kill_urb (mos7840_serial->interrupt_read_urb); 
	}
}

/****************************************************************************
 * mos7840_shutdown
 *	This function is called whenever the device is removed from the usb bus.
 ****************************************************************************/

static void mos7840_shutdown (struct usb_serial *serial)
{
	int i;
	struct moschip_port 	*mos7840_port = NULL;
	struct moschip_serial 	*mos7840_serial = NULL;

	DPRINTK("%s \n"," shutdown :entering..........");

	if (!serial) {
		printk("%s","Invalid Handler \n");
		return;
	}
	mos7840_serial = mos7840_get_serial_private(serial);
	if (mos7840_serial == NULL) {
		printk("%s - Invalid Handler \n", __FUNCTION__);
		return;
	}
	/********** parport **********/
	if (mos7840_serial->parallelport) {
		struct parport_ax781x0_private *priv;
		struct parport *pp;

		pp = mos7840_serial->parp;
		priv = pp->private_data;  
		
		priv->ThreadState = 1;
		//wake_up_interruptible(&restore_state_event);
		wake_up(&priv->restore_state_event); 
		priv->thread_cond = 1;
		//wait here till the kernel thread breaks
		i = down_interruptible(&priv->thread_complete);
		
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
		kill_proc(priv->thread_pid, SIGKILL, 1); 
#else
		// Need to invoke the above call		
		kthread_stop(priv->thread_pid);
#endif
		if (pp) {
			//  priv = pp->private_data;
			priv->usbdev = NULL;
			
			pp->modes = 0x0; 
			parport_remove_port(pp);
			parport_put_port(pp);
			kfree(priv);
		}
		mos7840_serial->parp = NULL;
	}
	/* check for the ports to be closed,close the ports and disconnect */

	/* free private structure allocated for serial port  * 
	 * stop reads and writes on all ports */

	for (i = 0; i < serial->num_ports; ++i) {
		mos7840_port = mos7840_get_port_private(serial->port[i]);
		if (mos7840_port == NULL) {
			printk("Invalid mos7840_port %d", i);
			return;
		}
		
		if (mos7840_port->has_led) {
			/* Turn off MCS7810 LED */
			mos7840_set_led_sync(mos7840_port->port, MODEM_CONTROL_REGISTER, 0x0300);
			del_timer_sync(&mos7840_port->led_timer1);
			del_timer_sync(&mos7840_port->led_timer2);
		}

		kfree(mos7840_port->ctrl_buf);
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,10)
		usb_kill_urb(mos7840_port->control_urb);
		usb_kill_urb(mos7840_port->control_urb_for_throttle);
#else
		usb_unlink_urb(mos7840_port->control_urb);
		usb_unlink_urb(mos7840_port->control_urb_for_throttle);
#endif
		kfree(mos7840_port);
		mos7840_set_port_private(serial->port[i], NULL);
	}
	/* free private structure allocated for serial device */
	
	kfree(mos7840_get_serial_private(serial));
	mos7840_set_serial_private(serial, NULL);
	
        DPRINTK("%s\n", "Thank u :: ");
}

/* Inline functions to check the sanity of a pointer that is passed to us */
static int mos7840_serial_paranoia_check (struct usb_serial *serial, const char *function)
{
        if (!serial) {
                DPRINTK("%s - serial == NULL", function);
                return -1;
        }

        if (!serial->type) {
                DPRINTK("%s - serial->type == NULL!", function);
                return -1;
        }

        return 0;
}
static int mos7840_port_paranoia_check (struct usb_serial_port *port, const char *function)
{
        if (!port) {
                DPRINTK("%s - port == NULL", function);
                return -1;
        }

        if (!port->serial) {
                DPRINTK("%s - port->serial == NULL", function);
                return -1;
        }

        return 0;
}
static struct usb_serial* mos7840_get_usb_serial (struct usb_serial_port *port, const char *function) {
        /* if no port was specified, or it fails a paranoia check */
        if (!port ||
                mos7840_port_paranoia_check (port, function) ||
                mos7840_serial_paranoia_check (port->serial, function)) {
     	/* then say that we don't have a valid usb_serial thing, which will                  
	 * end up genrating -ENODEV return values */
                return NULL;
        }

        return port->serial;
}


#if LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)
/****************************************************************************
 * moschip7840_init
 *	This is called by the module subsystem, or on startup to initialize us
 ****************************************************************************/
 int __init moschip7840_init(void)
{
	int retval;

	DPRINTK("%s \n"," mos7840_init :entering..........");
        /* Register with the usb serial */
	retval = usb_serial_register (&moschip7840_4port_device);
	if (retval)
		goto failed_port_device_register;

	DPRINTK("%s\n","Entring...");

	//info(DRIVER_DESC " " DRIVER_VERSION);
	printk(KERN_INFO KBUILD_MODNAME ": " DRIVER_VERSION ":"
               DRIVER_DESC "\n");

 	/* Register with the usb */
	retval = usb_register(&io_driver);

	if (retval) 
		goto failed_usb_register;

	if (retval == 0) {
		DPRINTK("%s\n","Leaving...");
		return 0;
	}

failed_usb_register:
	usb_serial_deregister(&moschip7840_4port_device);

failed_port_device_register:

	return retval;
}

/****************************************************************************
 * moschip7840_exit
 *	Called when the driver is about to be unloaded.
 ****************************************************************************/
void __exit moschip7840_exit (void)
{
	DPRINTK("%s \n"," mos7840_exit :entering..........");

	usb_deregister(&io_driver);
	usb_serial_deregister(&moschip7840_4port_device);

	DPRINTK("%s\n","Entring...");
}

module_init(moschip7840_init);
module_exit(moschip7840_exit);
#elif LINUX_VERSION_CODE < KERNEL_VERSION(3,5,0)
static struct usb_serial_driver * const serial_drivers[] = {
	&moschip7840_4port_device, NULL
};

module_usb_serial_driver(io_driver, serial_drivers);
#else
static struct usb_serial_driver * const serial_drivers[] = {
	&moschip7840_4port_device, NULL
};

module_usb_serial_driver(serial_drivers, moschip_port_id_table);
#endif

/* Module information */
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
