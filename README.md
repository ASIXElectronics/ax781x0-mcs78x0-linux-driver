# ASIX AX781x0 USB to Serial/UART/Parallel Linux driver (`mos7840.ko`)

This is the official **ASIX AX781x0 USB to Serial/UART/Parallel Linux driver** source for AX78140, AX78120, MCS78x0 controllers (module name: `mos7840.ko`), which is suitable for USB 2.0 I/O bridge applications including USB to UART/RS‑232/RS-422/RS‑485 serial converters, USB parallel (LPT) printer adapters, USB Data Acquisition (DAQ) modules, industrial computers, automation systems, measurement instruments, medical devices, and POS terminals. 

---
## ASIX USB 2.0 to Serial/UART/Parallel I/O bridge ICs 
- [AX78140 USB 2.0 to Multi I/O (4S, 2S+1P) Controllers](https://www.asix.com.tw/en/product/Interface/USB_Bridge/AX78140)
- [AX78120 USB 2.0 to Multi I/O (2S, 1S) Controllers](https://www.asix.com.tw/en/product/Interface/USB_Bridge/AX78120)

### Supported USB 2.0 to Multi-I/O (4S, 2S+1P, 2S, 1S) modes
- **AX78140 4S** : USB 2.0 to 4 serial/UART ports
- **AX78140 2S+1P** : USB 2.0 to 2 serial/UART ports + 1 parallel port
- **AX78120 2S** : USB 2.0 to 2 serial/UART ports 
- **AX78120 1S** : USB 2.0 to 1 serial/UART port

### Supported USB 2.0 to Serial/UART/Parallel Interfaces
| IC | USB VID:PID | `bcdDevice` | Interfaces |
|----|-------------|-------------|-----------|
| **AX78140, MCS7840** | `9710:7840` | —        | USB 2.0 to 4S/2S1P Ports |
| **AX78120, MCS7820** | `9710:7820` | —        | USB 2.0 to 2S/1S Ports |

---
Configuration instructions:
-----------------------------

Install & Uninstall
---------------------------
1. Install driver to kernel.
   ex:  Kernel: 4.8.0
	
	$make install
	-----------------------AX78140/AX78120/MCS7840/MCS7820/MCS7810  INSTALL SCRIPT------------------------
	---------------------FOR LINUX 4.8.0-36-generic----------------------
 
	installed
	mos7840 Driver already Installed
 
	-----------------------Thanks for using AX78140/AX78120/MCS7840/MCS7820/MCS7810 Driver-------------------

2. Remove driver from kernel.
   ex: 	Kernel: 4.8.0
       
       	$make uninstall
	-----------------------AX78140/AX78120/MCS7840/MCS7820/MCS7810 UNINSTALL SCRIPT------------------------
	---------------------FOR LINUX 4.8.0-36-generic----------------------
 
	/lib/modules/4.8.0-36-generic/kernel/drivers/usb/serial/mos7840.ko
	MOS_UNINSTALL : mos7840.ko removed 
 
	-----------------------Thanks for using AX78140/AX78120/MCS7840/MCS7820/MCS7810 Driver-------------------


RS232 RS422 RS485 change:
---------------------------
1.Modified the default setting in mos7840.h

  ex: MCS7840 port1 and port2 is RS232, port3 is RS422, port4 is RS485
	//RS_MODE 0:RS232 1:RS422 2:RS485
	#define RS_MODE_DEFAULT_PORT1 0
	#define RS_MODE_DEFAULT_PORT2 0
	#define RS_MODE_DEFAULT_PORT3 1
	#define RS_MODE_DEFAULT_PORT4 2
	
  Re-compile driver and install
	
2.Auto-switch:

  Auto-switch is a transceiver switch method by I2C.
	1. Enable the define of auto-switch in mos7840.h
		#define AUTO_SWITCH
	2. Re-compile driver and install
	3. Execute ./ioctl to switch transceiver

Modem:
--------
(i)Steps to configure modem in FC2 and FC4

	In KPPP configuration select the device as /dev/usb/ttyUSB0.

(ii)Steps to configure modem in FC3

	rm /dev/modem
	ln -sf /dev/ttyUSB0  /dev/modem

	Now /dev/modem can be selected for  KPPP configuration.

(iii)Steps to configure modem from FC5 to FC11
	
	
	In KPPP configuration select the device as /dev/ttyUSB0.

Browsing net:
---------------
	i.Deactivate the eth0 or any other network device.
	ii.Open the net browser,select proxy as direct connection to Internet.   
  	
Serial mouse configuration:
------------------------------
	1.For this we need to edit /etc/X11/xorg.conf file and add serial mouse.

	(i).Add the fallowing Line
		InputDevice    "AX781x0Mouse0" "SendCoreEvents"

	(ii)Also add the fallowing section

		Section "InputDevice"
	        Identifier  "AX781x0Mouse0"
		        Driver      "mouse"
		        Option      "SendCoreEvents" "true"
		        Option      "Protocol" "auto"
		        Option      "Device" "/dev/ttyUSB0"
		        Option      "ZAxisMapping" "4 5"
		        Option      "Emulate3Buttons" "yes"
		EndSection

	2.One can refer to the xrog.conf_example for editing
	3.After editing we have to restart the Xserver.

Restarting Xserver:
----------------------
	i. in FCX, alt+ctrl+Backspace should work.
	ii. in FC4 with our observation, we have to enter in INIT3, 
		Do "service xfs restart" and then "startx".
Note:In FCX- X represents 1 to 11 except 4.

3.Set Band using Ioctl
	ex: ttyUSB0 set 6M Baud.

		char Port[15] ="/dev/ttyUSB0"
		int nDevice = open(Port, O_RDWR | O_NONBLOCK);
		if (ioctl(nDevice, TIOCEXBAUD, 6000000) < 0) 
			printf("IOCTL failed!\n");
