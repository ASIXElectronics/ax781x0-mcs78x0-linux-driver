/* Definition for IOCTL */
#define IOCTL_SET_RS_MODE		_IOW(0xD0, 11, int)
#define IOCTL_CHECK_AS			_IOR(0xD0, 12, int)
#define	TIOCEXBAUD			0x5462
	//MP Tool
#define IOCTL_CHECK_DEVICE		_IOR(0xD0, 15, int)
#define IOCTL_GET_MSR			_IOR(0xD0, 16, int)
#define IOCTL_SET_MCR			_IOR(0xD0, 17, int)
#define IOCTL_GET_MCR			_IOR(0xD0, 18, int)

#define AX78140_DRV_NAME	"AX78140"
#define AX78120_DRV_NAME	"AX78120"
