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
#include "mos7840.h"            /* mos7840 Defines    */
#include "mos7840_16C50.h"	/* 16C50 UART defines */
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,15)
#include <linux/usb/serial.h>
#else
#include <../drivers/usb/serial/usb-serial.h>
#endif

/*
 * Defines used for sending commands to port 
 */

#define SER1_DCR0 4
#define SER1_DCR1 5
#define SER1_DCR2 6

#define GPIO_IN		0
#define GPIO_OUT	1

#define GPIO0_IN	0
#define GPIO0_OUT	1

#define GPIO1_IN	0
#define GPIO1_OUT	1

//delay us
#define I2C_START_DELAY		5
#define I2C_STOP_DELAY		5
#define I2C_DATA_SET_TLE	2
#define I2C_CLOCK_HIGH		10
#define I2C_HALF_CLOCK		5
#define I2C_CLOCK_LOW		10
#define I2C_ACK_WAITMIN		5
#define ZERO_DELAY		0

static int SDA;	//GPIO1
static int SCL; //GPIO0




/* ----------AX781x0/MCS78x0 GPIO basic operation---------- */
static void gpio_change_inout_mode(struct usb_serial *serial, int gpio1_mode, int gpio0_mode)
{
	__u16 data = 0x00;

	//printk("---gpio_change_inout_mode()\n");

	if (gpio1_mode == GPIO_OUT)
		data |= 0x08;	

	if (gpio0_mode == GPIO_OUT)
		data |= 0x04;

	usb_control_msg(serial->dev, usb_sndctrlpipe(serial->dev, 0),
		MCS_WRREQ, MCS_WR_RTYPE, data, 
		SER1_DCR0, NULL, 0, MOS_WDR_TIMEOUT);

	/*
	//read for check
	data = 0x00;
	usb_control_msg(serial->dev, usb_rcvctrlpipe(serial->dev, 0),
		MCS_RDREQ, MCS_RD_RTYPE, 0, SER1_DCR0, &data,
		VENDOR_READ_LENGTH, MOS_WDR_TIMEOUT);

	printk("   SER1_DCR0 = 0x%X\n", data);
	*/
}
static void gpio_set_out(struct usb_serial *serial, int us_delay)
{
	__u16 data = 0x00;

	//printk("---gpio_set_out()\n");

	data = (SDA << 1) | SCL;

	usb_control_msg(serial->dev, usb_sndctrlpipe(serial->dev, 0),
		MCS_WRREQ, MCS_WR_RTYPE, data, 
		GPIO_REGISTER, NULL, 0, MOS_WDR_TIMEOUT);

	udelay(us_delay);
}
static void gpio_get_in(struct usb_serial *serial)
{
	
	__u8 * data;
		
	data = kmalloc(2, GFP_KERNEL);
	if (!data)
		return;
	memset(data, 0, 2);

	//printk("---gpio_get_in()\n");

	usb_control_msg(serial->dev, usb_rcvctrlpipe(serial->dev, 0),
		MCS_RDREQ, MCS_RD_RTYPE, 0, GPIO_REGISTER, data,
		VENDOR_READ_LENGTH, MOS_WDR_TIMEOUT);

	SDA = (*data & 0x2) ? 1 : 0; 
	kfree(data);
}





/* ----------I2C basic operation---------- */
static void i2c_start(struct usb_serial *serial)
{
	SDA = 1;
	SCL = 1;
	gpio_set_out(serial, I2C_START_DELAY);

	SDA = 0;
	gpio_set_out(serial, I2C_START_DELAY);

	SCL = 0;
	gpio_set_out(serial, I2C_CLOCK_LOW);
}
static void i2c_stop(struct usb_serial *serial)
{
	SDA = 0;
	SCL = 1;
	gpio_set_out(serial, I2C_STOP_DELAY);

	SDA = 1;
	gpio_set_out(serial, ZERO_DELAY);
}
static void i2c_make_clock(struct usb_serial *serial)
{
	udelay(I2C_DATA_SET_TLE);

	SCL = 1;
	gpio_set_out(serial, I2C_CLOCK_HIGH);

	SCL = 0;
	gpio_set_out(serial, I2C_CLOCK_LOW);
}





/* ----------I2C basic send operation---------- */
static void i2c_send_byte(struct usb_serial *serial, char byte)
{
	char i;

	SDA = 0;
	SCL = 0;
	gpio_set_out(serial, I2C_CLOCK_LOW);

	for (i = 0; i < 8; i++) {
		if ((byte & 0x80) == 0)
			SDA = 0;
		else
			SDA = 1;

		gpio_set_out(serial, ZERO_DELAY);

		byte = byte << 1;

		i2c_make_clock(serial);
	}

	SDA = 1;
	gpio_set_out(serial, ZERO_DELAY);
}/* //Not use this function yet
static void i2c_send_ack(struct usb_serial *serial)
{
	SDA = 0;
	gpio_set_out(serial, I2C_DATA_SET_TLE);

	i2c_make_clock(serial);

	SDA = 1;
	gpio_set_out(serial, I2C_DATA_SET_TLE);
}*/





/* ----------I2C basic receive operation---------- */
static char i2c_recv_ack(struct usb_serial *serial)
{
	//SDA = 0;
	//SCL = 0;
	//gpio_set_out(serial, ZERO_DELAY);
	SDA = 1;
	SCL = 1;
	gpio_set_out(serial, I2C_HALF_CLOCK);

	gpio_change_inout_mode(serial, GPIO1_IN, GPIO0_OUT);

	gpio_get_in(serial);

	if (SDA)
		return 0;	
	
	udelay(I2C_HALF_CLOCK);

	gpio_change_inout_mode(serial, GPIO1_OUT, GPIO0_OUT);

	SCL = 0;
	gpio_set_out(serial, I2C_CLOCK_LOW * 2);

	return 1;
}/* //Not use this function yet
static char i2c_recv_bit(struct usb_serial *serial)
{
	char data = 0;

	udelay(I2C_DATA_SET_TLE);
	
	SCL = 1;
	gpio_set_out(serial, I2C_HALF_CLOCK);

	gpio_get_in(serial);

	if (SDA != 0)
		data = 1;

	udelay(I2C_HALF_CLOCK);

	SCL = 0;
	gpio_set_out(serial, I2C_CLOCK_LOW);

	return data;	

}
static char i2c_recv_byte(struct usb_serial *serial)
{
	char i, byte=0;

	SDA = 0;
	SCL = 0;
	gpio_set_out(serial, I2C_CLOCK_LOW);

	gpio_change_inout_mode(serial, GPIO1_IN, GPIO0_OUT);

	for (i = 0; i < 8; i++) {
		byte = byte << 1;
		
		if (i2c_recv_bit(serial)) {
			byte += 1;
		}
	}

	gpio_change_inout_mode(serial, GPIO1_OUT, GPIO0_OUT);

	return byte;
}*/





/* ----------I2C TX/RX operation---------- */
static char i2c_write(struct usb_serial *serial, char address, char *data, char number)
{
	i2c_start(serial);

	i2c_send_byte(serial, address & 0xFE);

	if (!i2c_recv_ack(serial)) {
		printk("---i2c_write(), no recv ack(1)!\n");
		i2c_stop(serial);
		return 0;
	}

	while (number--) {
		i2c_send_byte(serial, *data);
		
		if (!i2c_recv_ack(serial)) {
			printk("---i2c_write(), no recv ack(2)!\n");
			i2c_stop(serial);
			return 0;
		}
		
		data++;
	}
	
	i2c_stop(serial);

	return 1;
	
}/* //Not use this function yet
static char i2c_read(struct usb_serial *serial, char address, char *data, char number)
{
	i2c_start(serial);

	i2c_send_byte(serial, address | 0x01); //lowest bit = 1 is read

	if (!i2c_recv_ack(serial)) {
		printk("---i2c_read(), no ack! return!");
		i2c_stop(serial);
		return 0;
	}

	while (number--) {
		*data = i2c_recv_byte(serial);
		data++;

		if (number > 0)
			i2c_send_ack(serial);	
	}
	
	i2c_stop(serial);

	return 1;
}*/





/* ----------RS232/RS422/RS485 i2c auto-switch operation---------- */
static void start_program(struct usb_serial *serial, char address, char value)
{
	char data[10];
	char ret;

	SDA = 0;
	SCL = 0;

	gpio_change_inout_mode(serial, GPIO1_OUT, GPIO0_OUT);

	data[0] = 0x04;
	data[1] = 0x01;
	ret = i2c_write(serial, 0x80, data, 2);
	
	data[0] = 0x09;
	data[1] = 0x55;
	data[2] = 0x55;
	data[3] = 0x55;
	data[4] = 0x55;
	data[5] = 0x55;
	data[6] = 0x55;
	ret = i2c_write(serial, 0x80, data, 7);

	data[0] = address;
	data[1] = value;
	ret = i2c_write(serial, 0x80, data, 2);
}
static void mcr_dtr_clear(struct usb_serial *serial, char port)
{
	__u16 value = (port << 8) & 0x0700;

	__u8 * mcr_data;
		
	mcr_data = kmalloc(2, GFP_KERNEL);
	if (!mcr_data)
		return;
	memset(mcr_data, 0, 2);

	//Store MCR setting
	usb_control_msg(serial->dev, usb_rcvctrlpipe(serial->dev, 0),
		MCS_RDREQ, MCS_RD_RTYPE, value, MODEM_CONTROL_REGISTER,
		mcr_data, VENDOR_READ_LENGTH, MOS_WDR_TIMEOUT);

	//Restore MCR setting
	usb_control_msg(serial->dev, usb_sndctrlpipe(serial->dev, 0), MCS_WRREQ,
		MCS_WR_RTYPE, (*mcr_data & 0xFFFE) | value, MODEM_CONTROL_REGISTER, NULL,
		0, MOS_WDR_TIMEOUT);
		
	kfree(mcr_data);
}
static void auto_switch(struct usb_serial *serial, int mode, char port)
{
	char echo_value = 0;

	//printk("---auto_switch(), mode = %X, port = %d\n", mode, port);

	if(mode){	//RS422 or RS485
		switch (mode) {
		case 0x22:
			echo_value = 0;
			mcr_dtr_clear(serial, port);
			break;
		case 0x23:
			echo_value = 0;
			mcr_dtr_clear(serial, port);
			break;
		case 0xC1:
			echo_value = 1;
			break;
		case 0xC2:
			echo_value = 1;
			break;
		case 0xC3:
			echo_value = 0;
			mcr_dtr_clear(serial, port);
			break;
		default:
			break;
		
		}

		switch (port) {
		case 1:
			start_program(serial, 0x2C, 0x01);
			start_program(serial, 0x2D, echo_value);
			break;
		case 2:
			start_program(serial, 0x30, 0x01);
			start_program(serial, 0x31, echo_value);
			break;
		case 3:
			start_program(serial, 0x34, 0x01);
			start_program(serial, 0x35, echo_value);
			break;
		case 4:
			start_program(serial, 0x38, 0x01);
			start_program(serial, 0x39, echo_value);
			break;
		default:
			break;
		}

	} else { 	//RS232
		switch (port) {
		case 1:
			start_program(serial, 0x2C, 0x00);
			start_program(serial, 0x2D, echo_value);
			break;
		case 2:
			start_program(serial, 0x30, 0x00);
			start_program(serial, 0x31, echo_value);
			break;
		case 3:
			start_program(serial, 0x34, 0x00);
			start_program(serial, 0x35, echo_value);
			break;
		case 4:
			start_program(serial, 0x38, 0x00);
			start_program(serial, 0x39, echo_value);
			break;
		default:
			break;
		}
	}
}
