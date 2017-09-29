/******************************************************************************
 *
 *  Copyright (C) 2013 Eduardo Jos� Tagle
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/

/******************************************************************************
 *
 *  Filename:      hardware.c
 *
 *  Description:   Contains controller-specific initialization functions
 *
 ******************************************************************************/

#define LOG_TAG "bt_hwcfg"

#include <utils/Log.h>
#include <assert.h>   
#include <ctype.h>
#include <cutils/properties.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>   
#include <stdlib.h>
#include <string.h>   
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#ifdef HW_TENDERLOIN
#include <linux/hsuart.h>
#endif

#include "bt_hci_bdroid.h"
#include "bt_vendor_hci.h"
#include "bluetooth.h"
#include "csr.h"
#include "hci.h"
#include "userial.h"
#include "userial_vendor.h"
#include "upio.h"
#include "ubcsp.h"



#ifndef N_HCI
#define N_HCI	15
#endif

#define HCIUARTSETPROTO		_IOW('U', 200, int)
#define HCIUARTGETPROTO		_IOR('U', 201, int)
#define HCIUARTGETDEVICE	_IOR('U', 202, int)
#define HCIUARTSETFLAGS		_IOW('U', 203, int)
#define HCIUARTGETFLAGS		_IOR('U', 204, int)

#define HCI_UART_NONE	-1
#define HCI_UART_H4		 0
#define HCI_UART_BCSP	 1 

#define FLOW_CTL	0x0001
#define EVEN_PARITY	0x0002
#define ODD_PARITY  0x0004

#define HCI_UART_RAW_DEVICE	0
#define HCI_UART_RESET_ON_INIT	1
#define HCI_UART_CREATE_AMP	2  


/******************************************************************************
**  Constants & Macros
******************************************************************************/

#ifndef BTHW_DBG
#define BTHW_DBG TRUE
#endif

#if (BTHW_DBG == TRUE)
#define BTHWDBG(param, ...) {ALOGD(param, ## __VA_ARGS__);}
#else
#define BTHWDBG(param, ...) {}
#endif



#define HCI_CMD_MAX_LEN             258

#define HCI_GRP_INFORMATIONAL_PARAMS    (0x04 << 10)            /* 0x1000 */

#define HCI_RESET                               0x0C03
#define HCI_READ_BUFFER_SIZE            (0x0005 | HCI_GRP_INFORMATIONAL_PARAMS)
#define HCI_VSC_WRITE_UART_CLOCK_SETTING        0xFC45
#define HCI_VSC_UPDATE_BAUDRATE                 0xFC18
#define HCI_READ_LOCAL_NAME                     0x0C14
#define HCI_VSC_DOWNLOAD_MINIDRV                0xFC2E
#define HCI_VSC_WRITE_BD_ADDR                   0xFC01
#define HCI_VSC_WRITE_SLEEP_MODE                0xFC27
#define HCI_VSC_WRITE_SCO_PCM_INT_PARAM         0xFC1C
#define HCI_VSC_WRITE_PCM_DATA_FORMAT_PARAM     0xFC1E
#define HCI_VSC_WRITE_I2SPCM_INTERFACE_PARAM    0xFC6D
#define HCI_VSC_LAUNCH_RAM                      0xFC4E
#define HCI_READ_LOCAL_BDADDR                   0x1009

#define HCI_EVT_CMD_CMPL_STATUS_RET_BYTE        5
#define HCI_EVT_CMD_CMPL_LOCAL_NAME_STRING      6
#define HCI_EVT_CMD_CMPL_LOCAL_BDADDR_ARRAY     6
#define HCI_EVT_CMD_CMPL_OPCODE                 3
#define LPM_CMD_PARAM_SIZE                      12
#define UPDATE_BAUDRATE_CMD_PARAM_SIZE          6
#define HCI_CMD_PREAMBLE_SIZE                   3
#define HCD_REC_PAYLOAD_LEN_BYTE                2
#define BD_ADDR_LEN                             6
#define LOCAL_NAME_BUFFER_LEN                   32
#define LOCAL_BDADDR_PATH_BUFFER_LEN            256

#define STREAM_TO_UINT16(u16, p) {u16 = ((uint16_t)(*(p)) + (((uint16_t)(*((p) + 1))) << 8)); (p) += 2;}
#define UINT16_TO_STREAM(p, u16) {*(p)++ = (uint8_t)(u16); *(p)++ = (uint8_t)((u16) >> 8);}
#define UINT32_TO_STREAM(p, u32) {*(p)++ = (uint8_t)(u32); *(p)++ = (uint8_t)((u32) >> 8); *(p)++ = (uint8_t)((u32) >> 16); *(p)++ = (uint8_t)((u32) >> 24);}

/******************************************************************************
**  Local type definitions
******************************************************************************/

/* Hardware Configuration State */
enum {
    HW_CFG_START = 1,
	HW_CFG_RBS,
    HW_CFG_END
};

/* h/w config control block */
typedef struct
{
    uint8_t state;                          /* Hardware configuration state */
    char    local_chip_name[LOCAL_NAME_BUFFER_LEN];
} bt_hw_cfg_cb_t;


void hw_config_cback(void *p_evt_buf);
extern uint8_t vnd_local_bd_addr[BD_ADDR_LEN];

/******************************************************************************
**  Static functions
******************************************************************************/

static bt_hw_cfg_cb_t hw_cfg_cb;

/*******************************************************************************
**
** Function        ms_delay
**
** Description     sleep unconditionally for timeout milliseconds
**
** Returns         None
**
*******************************************************************************/
void ms_delay (uint32_t timeout)
{
    struct timespec delay;
    int err;

    if (timeout == 0)
        return;

    delay.tv_sec = timeout / 1000;
    delay.tv_nsec = 1000 * 1000 * (timeout%1000);

    /* [u]sleep can't be used because it uses SIGALRM */
    do {
        err = nanosleep(&delay, &delay);
    } while (err < 0 && errno ==EINTR);
}


#if 1
void *
__memcpy_chk (void *__restrict__ dest, const void *__restrict__ src,
              size_t len, size_t slen)
{
  memcpy (dest, src, len);
  return 0;
}


char *
__strcpy_chk (char *__restrict__ dest, const char *__restrict__ src,
              size_t slen)
{
  size_t len = strlen (src);
  memcpy (dest, src, len + 1);
  return 0;
}
#endif

typedef enum {
	omUnknown = 0,
	omH4,
	omBCSP,
	omHCI
} hcimode_t;



struct uart_t {
	char *type;
	int  proto;
	int  init_speed;
	int  speed;
	int  flags;
	int  (*init) (void);
};

typedef struct {
	char dev[64];
	char type[32];
	char psrfile[64];
	int  init_speed;
	int  speed;
	int  flags;
	int	 send_break;
	int  raw;
	int  dev_id;
	
	int  sfd;
	int  ufd;
	uint16_t 	seqnum;
	hcimode_t 	hci_mode;
	struct ubcsp_packet send_packet;
	uint8_t 	send_buffer[512];
	struct ubcsp_packet receive_packet;
	uint8_t 	receive_buffer[512];
	
	/* RFKILL support */  
	int rfkill_id;
	char *rfkill_state_path;


} HWCFG_t;

static HWCFG_t cfg;

static int uart_speed(int s)
{
	switch (s) {
	case 9600:
		return B9600;
	case 19200:
		return B19200;
	case 38400:
		return B38400;
	case 57600:
		return B57600;
	case 115200:
		return B115200;
	case 230400:
		return B230400;
	case 460800:
		return B460800;
	case 500000:
		return B500000;
	case 576000:
		return B576000;
	case 921600:
		return B921600;
	case 1000000:
		return B1000000;
	case 1152000:
		return B1152000;
	case 1500000:
		return B1500000;
	case 2000000:
		return B2000000;
#ifdef B2500000
	case 2500000:
		return B2500000;
#endif
#ifdef B3000000
	case 3000000:
		return B3000000;
#endif
#ifdef B3500000
	case 3500000:
		return B3500000;
#endif
#ifdef B4000000
	case 4000000:
		return B4000000;
#endif
	default:
		return B57600;
	}
} 

static int hci_open(int rate, hcimode_t mode )
{
	struct termios ti;
	uint8_t delay, activity = 0x00;
#ifdef HW_TENDERLOIN
	struct hsuart_mode uart_mode;
#endif
	int timeout = 0;
	int ld;

	cfg.hci_mode = mode;
	
	if (mode == omHCI) {
	
		struct sockaddr_hci a;
		struct hci_filter flt;

		ALOGI("Opening hci%d", cfg.dev_id);

		/* Create HCI socket */
		cfg.sfd = socket(AF_BLUETOOTH, SOCK_RAW, BTPROTO_HCI);
		if (cfg.sfd < 0) {
			ALOGE("Unable to open NETLINK socket");
			return -1;
		}

		/* Bind socket to the HCI device */
		memset(&a, 0, sizeof(a));
		a.hci_family = AF_BLUETOOTH;
		a.hci_dev = cfg.dev_id;
		if (bind(cfg.sfd, (struct sockaddr *) &a, sizeof(a)) < 0) {
			ALOGE("Unable to bind to open HCI%d device",cfg.dev_id);
			close(cfg.sfd);
			cfg.sfd = -1;
			return -1;
		}
		
		/* Setup no filter */
		memset(&flt, 0, sizeof(flt));
		if (setsockopt(cfg.sfd, SOL_HCI, HCI_FILTER, &flt, sizeof(flt)) < 0) {
			ALOGE("HCI filter setup failed");
			close(cfg.sfd);
			cfg.sfd = -1;
			return -1;
		} 

		ALOGI("device sfd = %d open", cfg.sfd);
		return 0;
	}

#ifdef HW_TENDERLOIN
	if (mode == omBCSP) {
		cfg.ufd = open(cfg.dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
	} else {
		cfg.ufd = open(cfg.dev, O_RDWR | O_NOCTTY);
	}
#else
	cfg.ufd = open(cfg.dev, O_RDWR | O_NOCTTY);
#endif
	if (cfg.ufd < 0) {
		ALOGE("Can't open serial port %s: %s (%d)", cfg.dev, strerror(errno), errno);
		return -1;
	}
	ALOGI("Opened serial port %s", cfg.dev);

#ifndef HW_TENDERLOIN
	tcflush(cfg.ufd, TCIOFLUSH);

	/* Restore TTY line discipline - Just in case ...*/
	ld = N_TTY;
	if (ioctl(cfg.ufd, TIOCSETD, &ld) < 0) {
		ALOGD("Can't restore line discipline to TTY");
	} 	

	if (tcgetattr(cfg.ufd, &ti) < 0) {
		ALOGE("Can't get port settings: %s (%d)",
						strerror(errno), errno);
		close(cfg.ufd);
		cfg.ufd = -1;
		return -1;
	}

	cfmakeraw(&ti);

	ti.c_cflag |=  CLOCAL;
	
	if (cfg.flags & FLOW_CTL)
		ti.c_cflag |= CRTSCTS;
	else
		ti.c_cflag &= ~CRTSCTS;
		
	if (cfg.flags & (EVEN_PARITY|ODD_PARITY))
		ti.c_cflag |=  PARENB;
	else
		ti.c_cflag &= ~PARENB;
		
	ti.c_cflag &= ~PARODD;
	if (cfg.flags & ODD_PARITY)
		ti.c_cflag |= PARODD;
		
	ti.c_cflag &= ~CSIZE;
	ti.c_cflag |=  CS8;
	ti.c_cflag &= ~CSTOPB;

	ti.c_cc[VMIN] = 1;
	ti.c_cc[VTIME] = 0;

	cfsetispeed(&ti, uart_speed(rate));
	cfsetospeed(&ti, uart_speed(rate));

	if (tcsetattr(cfg.ufd, TCSANOW, &ti) < 0) {
		ALOGE("Can't change port settings: %s (%d)",
						strerror(errno), errno);
		close(cfg.ufd);
		cfg.ufd = -1;
		return -1;
	}

	tcflush(cfg.ufd, TCIOFLUSH);
	
	if (cfg.send_break) {
		tcsendbreak(cfg.ufd, 0);
		usleep(500000);
	}

	if (fcntl(cfg.ufd, F_SETFL, fcntl(cfg.ufd, F_GETFL, 0) | O_NONBLOCK) < 0) {
		ALOGE("Can't set non blocking mode: %s (%d)",
						strerror(errno), errno);
		close(cfg.ufd);
		cfg.ufd = -1;
		return -1;
	}
#else
	// HW_TENDERLOIN
	if (rate == 38400) {
		uart_mode.speed = 0x384000;
	} else {
		uart_mode.speed = 0x1C200;
	}
	if (mode == omBCSP) {
		uart_mode.flags = 0x9;
	} else {
		uart_mode.flags = 0x2;
	}
	if (ioctl(cfg.ufd, HSUART_IOCTL_SET_UARTMODE, &uart_mode) < 0) {
		ALOGE("Failed HSUART_IOCTL_SET_UARTMODE: %s (%d)",
                 strerror(errno), errno);
		close(cfg.ufd);
		cfg.ufd = -1;
		return -1;
	}
	usleep(100);
#endif
	
	if (mode == omBCSP) {

		memset(&cfg.send_packet, 0, sizeof(cfg.send_packet));
		memset(&cfg.receive_packet, 0, sizeof(cfg.receive_packet));

		ubcsp_initialize();

		cfg.send_packet.length = 512;
		cfg.send_packet.payload = cfg.send_buffer;

		cfg.receive_packet.length = 512;
		cfg.receive_packet.payload = cfg.receive_buffer;

		ubcsp_receive_packet(&cfg.receive_packet);

		while (1) {
			delay = ubcsp_poll(&activity);

			if (activity & UBCSP_PACKET_SENT)
				break;

			if (delay) {
				usleep(delay * 100);

				if (timeout++ > 5000) {
					ALOGE("Initialization timed out");
					close(cfg.ufd);
					cfg.ufd = -1;
					return -1;
				}
			}
		}
	}
	
	return 0;
}

static int hci_set_speed(int speed)
{
	struct termios ti;
	
	if (tcgetattr(cfg.ufd, &ti) < 0 ||
		cfsetospeed(&ti, uart_speed(speed)) < 0 ||
		cfsetispeed(&ti, uart_speed(speed)) < 0 ||
		tcsetattr(cfg.ufd, TCSANOW, &ti) < 0) {
		ALOGE("Unable to set speed to %d bauds", speed);
		return -1;
	}

	return 0;
} 


void put_uart(uint8_t ch)
{
	if (write(cfg.ufd, &ch, 1) < 0) {
#ifdef HW_TENDERLOIN
		usleep(1000);
#else
		ALOGE("UART write error");
#endif
	}
}

uint8_t get_uart(uint8_t *ch)
{
	int res = read(cfg.ufd, ch, 1);
	return res > 0 ? res : 0;
}

static int do_bcsp_bccmd(uint16_t command, uint16_t seqnum, uint16_t varid, uint8_t *value, uint16_t length)
{
	uint8_t cp[254], rp[254];
	uint8_t cmd[10];
	uint16_t size;
	uint8_t delay, activity = 0x00;
	int timeout = 0, sent = 0;

	size = (length < 8) ? 9 : ((length + 1) / 2) + 5;

	cmd[0] = command & 0xff;
	cmd[1] = command >> 8;
	cmd[2] = size & 0xff;
	cmd[3] = size >> 8;
	cmd[4] = seqnum & 0xff;
	cmd[5] = seqnum >> 8;
	cmd[6] = varid & 0xff;
	cmd[7] = varid >> 8;
	cmd[8] = 0x00;
	cmd[9] = 0x00;

	memset(cp, 0, sizeof(cp));
	cp[0] = 0x00;
	cp[1] = 0xfc;
	cp[2] = (size * 2) + 1;
	cp[3] = 0xc2;
	memcpy(cp + 4, cmd, sizeof(cmd));
	memcpy(cp + 14, value, length);

	cfg.receive_packet.length = 512;
	ubcsp_receive_packet(&cfg.receive_packet);

	cfg.send_packet.channel  = 5;
	cfg.send_packet.reliable = 1;
	cfg.send_packet.length   = (size * 2) + 4;
	memcpy(cfg.send_packet.payload, cp, (size * 2) + 4);

	ubcsp_send_packet(&cfg.send_packet);

	while (1) {
		delay = ubcsp_poll(&activity);

		if (activity & UBCSP_PACKET_SENT) {
			switch (varid) {
			case CSR_VARID_COLD_RESET:
			case CSR_VARID_WARM_RESET:
			case CSR_VARID_COLD_HALT:
			case CSR_VARID_WARM_HALT:
				return 0;
			}

			sent = 1;
			timeout = 0;
		}

		if (activity & UBCSP_PACKET_RECEIVED) {
			if (sent && cfg.receive_packet.channel == 5 &&
					cfg.receive_packet.payload[0] == 0xff) {
				memcpy(rp, cfg.receive_packet.payload,
							cfg.receive_packet.length);
				break;
			}

			cfg.receive_packet.length = 512;
			ubcsp_receive_packet(&cfg.receive_packet);
			timeout = 0;
		}

		if (delay) {
			usleep(delay * 100);

			if (timeout++ > 5000) {
				ALOGE("Operation timed out");
				errno = ETIMEDOUT;
				return -1;
			}
		}
	}

	if (rp[0] != 0xff || rp[2] != 0xc2) {
		errno = EIO;
		return -1;
	}

	if ((rp[11] + (rp[12] << 8)) != 0) {
		errno = ENXIO;
		return -1;
	}

	memcpy(value, rp + 13, length);

	return 0;
}

static int do_h4_bccmd(uint16_t command, uint16_t seqnum, uint16_t varid, uint8_t *value, uint16_t length)
{
	uint8_t cp[254], rp[254];
	uint8_t cmd[10];
	uint16_t size;
	int len, offset = 3;

	size = (length < 8) ? 9 : ((length + 1) / 2) + 5;

	cmd[0] = command & 0xff;
	cmd[1] = command >> 8;
	cmd[2] = size & 0xff;
	cmd[3] = size >> 8;
	cmd[4] = seqnum & 0xff;
	cmd[5] = seqnum >> 8;
	cmd[6] = varid & 0xff;
	cmd[7] = varid >> 8;
	cmd[8] = 0x00;
	cmd[9] = 0x00;

	memset(cp, 0, sizeof(cp));
	cp[0] = 0x01;
	cp[1] = 0x00;
	cp[2] = 0xfc;
	cp[3] = (size * 2) + 1;
	cp[4] = 0xc2;
	memcpy(cp + 5, cmd, sizeof(cmd));
	memcpy(cp + 15, value, length);

	if (write(cfg.ufd, cp, (size * 2) + 5) < 0)
		return -1;

	switch (varid) {
	case CSR_VARID_COLD_RESET:
	case CSR_VARID_WARM_RESET:
	case CSR_VARID_COLD_HALT:
	case CSR_VARID_WARM_HALT:
		return 0;
	}

	do {
		if (read(cfg.ufd, rp, 1) < 1)
			return -1;
	} while (rp[0] != 0x04);

	if (read(cfg.ufd, rp + 1, 2) < 2)
		return -1;

	do {
		len = read(cfg.ufd, rp + offset, sizeof(rp) - offset);
		offset += len;
	} while (offset < rp[2] + 3);

	if (rp[0] != 0x04 || rp[1] != 0xff || rp[3] != 0xc2) {
		errno = EIO;
		return -1;
	}

	if ((rp[12] + (rp[13] << 8)) != 0) {
		errno = ENXIO;
		return -1;
	}

	memcpy(value, rp + 14, length);

	return 0;
}  

static int csr_bccmd_read(uint16_t varid, uint8_t *value, uint16_t length)
{
	if (cfg.hci_mode == omBCSP)
		return do_bcsp_bccmd(0x0000, cfg.seqnum++, varid, value, length);
	return do_h4_bccmd(0x0000, cfg.seqnum++, varid, value, length);
}

static int csr_bccmd_write(uint16_t varid, uint8_t *value, uint16_t length)
{
	if (cfg.hci_mode == omBCSP)
		return do_bcsp_bccmd(0x0002, cfg.seqnum++, varid, value, length);
	return do_h4_bccmd(0x0002, cfg.seqnum++, varid, value, length);
}

static int do_bcsp_hci(uint16_t command,const uint8_t *payld, uint16_t length)
{
	uint8_t cp[254];
	uint8_t delay, activity = 0x00;
	int timeout = 0;

	memset(cp, 0, sizeof(cp));
	cp[0] = command;
	cp[1] = command >> 8;
	cp[2] = length;
	cp[3] = 0xc2;
	if (length) 
		memcpy(cp + 4, payld, length);

	cfg.receive_packet.length = 512;
	ubcsp_receive_packet(&cfg.receive_packet);

	cfg.send_packet.channel  = 5;
	cfg.send_packet.reliable = 1;
	cfg.send_packet.length   = length + 4;
	memcpy(cfg.send_packet.payload, cp, length + 4);

	ubcsp_send_packet(&cfg.send_packet);

	while (1) {
		delay = ubcsp_poll(&activity);

		if (activity & UBCSP_PACKET_SENT) {
			return 0;
		}

		if (delay) {
			usleep(delay * 100);

			if (timeout++ > 5000) {
				ALOGE("Operation timed out");
				errno = ETIMEDOUT;
				return -1;
			}
		}
	}

	return 0;
}

static int do_h4_hci(uint16_t command,const uint8_t *payld, uint16_t length)
{
	uint8_t cp[254];
	int len, offset = 3;

	memset(cp, 0, sizeof(cp));
	cp[0] = 0x01;
	cp[1] = command;
	cp[2] = command >> 8;
	cp[3] = length;
	cp[4] = 0xc2;
	if (length)
		memcpy(cp + 5, payld, length);

	if (write(cfg.ufd, cp, length + 5) < 0)
		return -1;

	return 0;
}  

static int hci_write(uint16_t command,const uint8_t *payld, uint16_t length)
{
	if (cfg.hci_mode == omBCSP)
		return do_bcsp_hci(command, payld, length);
	return do_h4_hci(command, payld, length);
}

static void hci_close(void)
{
	if (cfg.ufd != -1)
		close(cfg.ufd);
	cfg.ufd = -1;
} 


/*****************************************************************************
**   Bluetooth On/Off Static Functions
*****************************************************************************/
static int init_rfkill()
{
    char path[64];
    char buf[16];
    int rfd, sz, id;


    for (id = 0; ; id++)
    {
        snprintf(path, sizeof(path), "/sys/class/rfkill/rfkill%d/type", id);
        rfd = open(path, O_RDONLY);
        if (rfd < 0) {
            ALOGE("Unable to open Bluetooth rfkill (%s): %s (%d)", \
                 path, strerror(errno), errno);
            return -1;
        }

        sz = read(rfd, &buf, sizeof(buf));
        close(rfd);

        if (sz >= 9 && memcmp(buf, "bluetooth", 9) == 0) {
            cfg.rfkill_id = id;
            break;
        }
    }

    asprintf(&cfg.rfkill_state_path, "/sys/class/rfkill/rfkill%d/state", cfg.rfkill_id);
    return 0;
} 

static int set_bluetooth_power(int on)
{
    int sz;
    int rfd = -1;
    int ret = -1;
    char buffer = '0';
#ifdef HW_TENDERLOIN
	char hwpin[] = "/sys/user_hw/pins/bt/reset/level";
#endif

	buffer = on ? '1' : '0';

#ifdef HW_TENDERLOIN
    rfd = open(hwpin, O_WRONLY);
    if (rfd < 0)
    {
        ALOGE("set_bluetooth_power : open(%s) for write failed: %s (%d)",
            hwpin, strerror(errno), errno);
        return ret;
    }

    sz = write(rfd, &buffer, 1);
    if (sz < 0) {
        ALOGE("set_bluetooth_power : write(%s) failed: %s (%d)",
            hwpin, strerror(errno),errno);
    }
    else {
        ret = 0;
	}

    if (rfd >= 0) close(rfd);
	usleep(1000000);
	if (ret) return ret;
#endif

    /* check if we have rfkill interface */
    if (cfg.rfkill_id == -1)
    {
        if (init_rfkill())
            return ret;
    }

    rfd = open(cfg.rfkill_state_path, O_WRONLY);

    if (rfd < 0)
    {
        ALOGE("set_bluetooth_power : open(%s) for write failed: %s (%d)",
            cfg.rfkill_state_path, strerror(errno), errno);
        return ret;
    }

    sz = write(rfd, &buffer, 1);

    if (sz < 0) {
        ALOGE("set_bluetooth_power : write(%s) failed: %s (%d)",
            cfg.rfkill_state_path, strerror(errno),errno);
    }
    else
        ret = 0;

    if (rfd >= 0)
        close(rfd);

    return ret;
} 
  
static void bt_reset(void)   
{   
	set_bluetooth_power(0);
	set_bluetooth_power(1);
}   

/* ericsson */
/*
 * Ericsson specific initialization
 */
static int ericsson(void)
{
	struct timespec tm = {0, 50000};
	int ret;
	uint8_t cmd[1];
	
	switch (cfg.init_speed) {
	case 57600:
		cmd[0] = 0x03;
		break;
	case 115200:
		cmd[0] = 0x02;
		break;
	case 230400:
		cmd[0] = 0x01;
		break;
	case 460800:
		cmd[0] = 0x00;
		break;
	case 921600:
		cmd[0] = 0x20;
		break;
	case 2000000:
		cmd[0] = 0x25;
		break;
	case 3000000:
		cmd[0] = 0x27;
		break;
	case 4000000:
		cmd[0] = 0x2B;
		break;
	default:
		ALOGE("Invalid speed requested %d", cfg.init_speed);
		return -1;
	}

	/* Send initialization command */
	ret = hci_write(0xFC09, cmd, 1);
	if (ret < 0) {
		ALOGE("Failed to write init command");
		return -1;
	}

	nanosleep(&tm, NULL);
	return 0;
}

/* ----------- Digi --------------- */
/*
 * Digianswer specific initialization
 */
static int digi(void)
{
	struct timespec tm = {0, 50000};
	uint8_t cmd[1];
	int ret;
	
	/* DigiAnswer set baud rate command */
	switch (cfg.init_speed) {
	case 57600:
		cmd[0] = 0x08;
		break;
	case 115200:
		cmd[0] = 0x09;
		break;
	default:
		ALOGE("Invalid speed requested %d", cfg.init_speed);
		return -1;
	}

	/* Send initialization command */
	ret = hci_write(0xFC07, cmd, 1);
	if (ret < 0) {
		ALOGE("Failed to write init command");
		return -1;
	}

	nanosleep(&tm, NULL);
	return 0;
} 

/* ------------ Silicon Wave ------------ */
/*
 * Silicon Wave specific initialization
 * Thomas Moser <thomas.moser@tmoser.ch>
 */
static int swave(void)
{
	struct timespec tm = { 0, 500000 };
	uint8_t cmd[6];
	int ret;

	
	// Silicon Wave set baud rate command
	// see HCI Vendor Specific Interface from Silicon Wave
	// first send a "param access set" command to set the
	// appropriate data fields in RAM. Then send a "HCI Reset
	// Subcommand", e.g. "soft reset" to make the changes effective.
	cmd[0] = 0x01;			// param sub command
	cmd[1] = 0x11;			// tag 17 = 0x11 = HCI Transport Params
	cmd[2] = 0x03;			// length of the parameter following
	cmd[3] = 0x01;			// HCI Transport flow control enable
	cmd[4] = 0x01;			// HCI Transport Type = UART

	switch (cfg.init_speed) {
	case 19200:
		cmd[5] = 0x03;
		break;
	case 38400:
		cmd[5] = 0x02;
		break;
	case 57600:
		cmd[5] = 0x01;
		break;
	case 115200:
		cmd[5] = 0x00;
		break;
	default:
		ALOGE("Invalid speed requested %d", cfg.init_speed);
		return -1;
	}

	/* Send initialization command */
	ret = hci_write(0xFC0B, cmd, 6);
	if (ret < 0) {
		ALOGE("Failed to write init command");
		return -1;
	}
	
	// We should wait for a "GET Event" to confirm the success of
	// the baud rate setting. Wait some time before reading. Better:
	// read with timeout, parse data
	// until correct answer, else error handling ... todo ...

	nanosleep(&tm, NULL);

	// we probably got the reply. Now we must send the "soft reset"
	// which is standard HCI RESET.
	ret = hci_write(0x0C03, cmd, 0);
	if (ret < 0) {
		ALOGE("Failed to write init command");
		return -1;
	}

	nanosleep(&tm, NULL);

	// now the uart baud rate on the silicon wave module is set and effective.
	// change our own baud rate as well. Then there is a reset event comming in
 	// on the *new* baud rate. This is *undocumented*! The packet looks like this:
	// 04 FF 01 0B (which would make that a confirmation of 0x0B = "Param
	// subcommand class". So: change to new baud rate, read with timeout, parse
	// data, error handling. BTW: all param access in Silicon Wave is done this way.
	// Maybe this code would belong in a seperate file, or at least code reuse...

	return 0;
} 

/*
 * ST Microelectronics specific initialization
 * Marcel Holtmann <marcel@holtmann.org>
 */
static int st(void)
{
	struct timespec tm = {0, 50000};
	uint8_t cmd[1];
	int ret;

	/* ST Microelectronics set baud rate command */
	switch (cfg.init_speed) {
	case 9600:
		cmd[0] = 0x09;
		break;
	case 19200:
		cmd[0] = 0x0b;
		break;
	case 38400:
		cmd[0] = 0x0d;
		break;
	case 57600:
		cmd[0] = 0x0e;
		break;
	case 115200:
		cmd[0] = 0x10;
		break;
	case 230400:
		cmd[0] = 0x12;
		break;
	case 460800:
		cmd[0] = 0x13;
		break;
	case 921600:
		cmd[0] = 0x14;
		break;
	default:
		ALOGE("Invalid speed requested %d", cfg.init_speed);
		return -1;
	}

	/* Send initialization command */
	ret = hci_write(0xFC46, cmd, 1);
	if (ret < 0) {
		ALOGE("Failed to write init command");
		return -1;
	}

	nanosleep(&tm, NULL);
	return 0;
} 

/* ---------------------- CSR (BlueCore) chips  --------------------------- */
static struct {
	uint16_t id;
	const char *str;
} csr_map[] = {
	{   66, "HCI 9.8"	},
	{   97, "HCI 10.3"	},
	{  101, "HCI 10.5"	},
	{  111,	"HCI 11.0"	},
	{  112,	"HCI 11.1"	},
	{  114,	"HCI 11.2"	},
	{  115,	"HCI 11.3"	},
	{  117,	"HCI 12.0"	},
	{  119,	"HCI 12.1"	},
	{  133,	"HCI 12.2"	},
	{  134,	"HCI 12.3"	},
	{  162,	"HCI 12.4"	},
	{  165,	"HCI 12.5"	},
	{  169,	"HCI 12.6"	},
	{  188,	"HCI 12.7"	},
	{  218,	"HCI 12.8"	},
	{  283,	"HCI 12.9"	},
	{  203,	"HCI 13.2"	},
	{  204,	"HCI 13.2"	},
	{  210,	"HCI 13.3"	},
	{  211,	"HCI 13.3"	},
	{  213,	"HCI 13.4"	},
	{  214,	"HCI 13.4"	},
	{  225,	"HCI 13.5"	},
	{  226,	"HCI 13.5"	},
	{  237,	"HCI 13.6"	},
	{  238,	"HCI 13.6"	},
	{  242,	"HCI 14.0"	},
	{  243,	"HCI 14.0"	},
	{  244,	"HCI 14.0"	},
	{  245,	"HCI 14.0"	},
	{  254,	"HCI 13.7"	},
	{  255,	"HCI 13.7"	},
	{  264,	"HCI 14.1"	},
	{  265,	"HCI 14.1"	},
	{  267,	"HCI 14.2"	},
	{  268,	"HCI 14.2"	},
	{  272,	"HCI 14.3"	},
	{  273,	"HCI 14.3"	},
	{  274,	"HCI 13.8"	},
	{  275,	"HCI 13.8"	},
	{  286,	"HCI 13.9"	},
	{  287,	"HCI 13.9"	},
	{  309,	"HCI 13.10"	},
	{  310,	"HCI 13.10"	},
	{  313,	"HCI 14.4"	},
	{  314,	"HCI 14.4"	},
	{  323,	"HCI 14.5"	},
	{  324,	"HCI 14.5"	},
	{  336,	"HCI 14.6"	},
	{  337,	"HCI 14.6"	},
	{  351,	"HCI 13.11"	},
	{  352,	"HCI 13.11"	},
	{  362,	"HCI 15.0"	},
	{  363,	"HCI 15.0"	},
	{  364,	"HCI 15.0"	},
	{  365,	"HCI 15.0"	},
	{  373,	"HCI 14.7"	},
	{  374,	"HCI 14.7"	},
	{  379,	"HCI 15.1"	},
	{  380,	"HCI 15.1"	},
	{  381,	"HCI 15.1"	},
	{  382,	"HCI 15.1"	},
	{  392,	"HCI 15.2"	},
	{  393,	"HCI 15.2"	},
	{  394,	"HCI 15.2"	},
	{  395,	"HCI 15.2"	},
	{  436,	"HCI 16.0"	},
	{  437,	"HCI 16.0"	},
	{  438,	"HCI 16.0"	},
	{  439,	"HCI 16.0"	},
	{  443,	"HCI 15.3"	},
	{  444,	"HCI 15.3"	},
	{  465,	"HCI 16.1"	},
	{  466,	"HCI 16.1"	},
	{  467,	"HCI 16.1"	},
	{  468,	"HCI 16.1"	},
	{  487,	"HCI 14.8"	},
	{  488,	"HCI 14.8"	},
	{  492,	"HCI 16.2"	},
	{  493,	"HCI 16.2"	},
	{  495,	"HCI 16.2"	},
	{  496,	"HCI 16.2"	},
	{  502,	"HCI 16.1.1"	},
	{  503,	"HCI 16.1.1"	},
	{  504,	"HCI 16.1.1"	},
	{  505,	"HCI 16.1.1"	},
	{  506,	"HCI 16.1.2"	},
	{  507,	"HCI 16.1.2"	},
	{  508,	"HCI 16.1.2"	},
	{  509,	"HCI 16.1.2"	},
	{  516,	"HCI 16.3"	},
	{  517,	"HCI 16.3"	},
	{  518,	"HCI 16.3"	},
	{  519,	"HCI 16.3"	},
	{  523,	"HCI 16.4"	},
	{  524,	"HCI 16.4"	},
	{  525,	"HCI 16.4"	},
	{  526,	"HCI 16.4"	},
	{  553,	"HCI 15.3"	},
	{  554,	"HCI 15.3"	},
	{  562,	"HCI 16.5"	},
	{  563,	"HCI 16.5"	},
	{  564,	"HCI 16.5"	},
	{  565,	"HCI 16.5"	},
	{  593,	"HCI 17.0"	},
	{  594,	"HCI 17.0"	},
	{  595,	"HCI 17.0"	},
	{  599,	"HCI 17.0"	},
	{  600,	"HCI 17.0"	},
	{  608,	"HCI 13.10.1"	},
	{  609,	"HCI 13.10.1"	},
	{  613,	"HCI 17.1"	},
	{  614,	"HCI 17.1"	},
	{  615,	"HCI 17.1"	},
	{  616,	"HCI 17.1"	},
	{  618,	"HCI 17.1"	},
	{  624,	"HCI 17.2"	},
	{  625,	"HCI 17.2"	},
	{  626,	"HCI 17.2"	},
	{  627,	"HCI 17.2"	},
	{  637,	"HCI 16.6"	},
	{  638,	"HCI 16.6"	},
	{  639,	"HCI 16.6"	},
	{  640,	"HCI 16.6"	},
	{  642,	"HCI 13.10.2"	},
	{  643,	"HCI 13.10.2"	},
	{  644,	"HCI 13.10.3"	},
	{  645,	"HCI 13.10.3"	},
	{  668,	"HCI 13.10.4"	},
	{  669,	"HCI 13.10.4"	},
	{  681,	"HCI 16.7"	},
	{  682,	"HCI 16.7"	},
	{  683,	"HCI 16.7"	},
	{  684,	"HCI 16.7"	},
	{  704,	"HCI 16.8"	},
	{  718,	"HCI 16.4.1"	},
	{  719,	"HCI 16.4.1"	},
	{  720,	"HCI 16.4.1"	},
	{  721,	"HCI 16.4.1"	},
	{  722,	"HCI 16.7.1"	},
	{  723,	"HCI 16.7.1"	},
	{  724,	"HCI 16.7.1"	},
	{  725,	"HCI 16.7.1"	},
	{  731,	"HCI 16.7.2"	},
	{  732,	"HCI 16.7.2"	},
	{  733,	"HCI 16.7.2"	},
	{  734,	"HCI 16.7.2"	},
	{  735,	"HCI 16.4.2"	},
	{  736,	"HCI 16.4.2"	},
	{  737,	"HCI 16.4.2"	},
	{  738,	"HCI 16.4.2"	},
	{  750,	"HCI 16.7.3"	},
	{  751,	"HCI 16.7.3"	},
	{  752,	"HCI 16.7.3"	},
	{  753,	"HCI 16.7.3"	},
	{  760,	"HCI 16.7.4"	},
	{  761,	"HCI 16.7.4"	},
	{  762,	"HCI 16.7.4"	},
	{  763,	"HCI 16.7.4"	},
	{  770,	"HCI 16.9"	},
	{  771,	"HCI 16.9"	},
	{  772,	"HCI 16.9"	},
	{  773,	"HCI 16.9"	},
	{  774,	"HCI 17.3"	},
	{  775,	"HCI 17.3"	},
	{  776,	"HCI 17.3"	},
	{  777,	"HCI 17.3"	},
	{  781,	"HCI 16.7.5"	},
	{  786,	"HCI 16.10"	},
	{  787,	"HCI 16.10"	},
	{  788,	"HCI 16.10"	},
	{  789,	"HCI 16.10"	},
	{  791,	"HCI 16.4.3"	},
	{  792,	"HCI 16.4.3"	},
	{  793,	"HCI 16.4.3"	},
	{  794,	"HCI 16.4.3"	},
	{  798,	"HCI 16.11"	},
	{  799,	"HCI 16.11"	},
	{  800,	"HCI 16.11"	},
	{  801,	"HCI 16.11"	},
	{  806,	"HCI 16.7.5"	},
	{  807,	"HCI 16.12"	},
	{  808,	"HCI 16.12"	},
	{  809,	"HCI 16.12"	},
	{  810,	"HCI 16.12"	},
	{  817,	"HCI 16.13"	},
	{  818,	"HCI 16.13"	},
	{  819,	"HCI 16.13"	},
	{  820,	"HCI 16.13"	},
	{  823,	"HCI 13.10.5"	},
	{  824,	"HCI 13.10.5"	},
	{  826,	"HCI 16.14"	},
	{  827,	"HCI 16.14"	},
	{  828,	"HCI 16.14"	},
	{  829,	"HCI 16.14"	},
	{  843,	"HCI 17.3.1"	},
	{  856,	"HCI 17.3.2"	},
	{  857,	"HCI 17.3.2"	},
	{  858,	"HCI 17.3.2"	},
	{ 1120, "HCI 17.11"	},
	{ 1168, "HCI 18.1"	},
	{ 1169, "HCI 18.1"	},
	{ 1241, "HCI 18.x"	},
	{ 1298, "HCI 18.2"	},
	{ 1354, "HCI 18.2"	},
	{ 1392, "HCI 18.2"	},
	{ 1393, "HCI 18.2"	},
	{ 1501, "HCI 18.2"	},
	{ 1503, "HCI 18.2"	},
	{ 1504, "HCI 18.2"	},
	{ 1505, "HCI 18.2"	},
	{ 1506, "HCI 18.2"	},
	{ 1520, "HCI 18.2"	},
	{ 1586, "HCI 18.2"	},
	{ 1591, "HCI 18.2"	},
	{ 1592, "HCI 18.2"	},
	{ 1593, "HCI 18.2.1"	},
	{ 1733, "HCI 18.3"	},
	{ 1734, "HCI 18.3"	},
	{ 1735, "HCI 18.3"	},
	{ 1737, "HCI 18.3"	},
	{ 1915, "HCI 19.2"	},
	{ 1916, "HCI 19.2"	},
	{ 1958, "HCI 19.2"	},
	{ 1981, "Unified 20a"	},
	{ 1982, "Unified 20a"	},
	{ 1989, "HCI 18.4"	},
	{ 2062, "Unified 20a1"	},
	{ 2063, "Unified 20a1"	},
	{ 2067, "Unified 18f"	},
	{ 2068, "Unified 18f"	},
	{ 2243, "Unified 18e"	},
	{ 2244, "Unified 18e"	},
	{ 2258, "Unified 20d"	},
	{ 2259, "Unified 20d"	},
	{ 2361, "Unified 20e"	},
	{ 2362, "Unified 20e"	},
	{ 2386, "Unified 21a"	},
	{ 2387, "Unified 21a"	},
	{ 2423, "Unified 21a"	},
	{ 2424, "Unified 21a"	},
	{ 2623, "Unified 21c"	},
	{ 2624, "Unified 21c"	},
	{ 2625, "Unified 21c"	},
	{ 2626, "Unified 21c"	},
	{ 2627, "Unified 21c"	},
	{ 2628, "Unified 21c"	},
	{ 2629, "Unified 21c"	},
	{ 2630, "Unified 21c"	},
	{ 2631, "Unified 21c"	},
	{ 2632, "Unified 21c"	},
	{ 2633, "Unified 21c"	},
	{ 2634, "Unified 21c"	},
	{ 2635, "Unified 21c"	},
	{ 2636, "Unified 21c"	},
	{ 2649, "Unified 21c"	},
	{ 2650, "Unified 21c"	},
	{ 2651, "Unified 21c"	},
	{ 2652, "Unified 21c"	},
	{ 2653, "Unified 21c"	},
	{ 2654, "Unified 21c"	},
	{ 2655, "Unified 21c"	},
	{ 2656, "Unified 21c"	},
	{ 2658, "Unified 21c"	},
	{ 3057, "Unified 21d"	},
	{ 3058, "Unified 21d"	},
	{ 3059, "Unified 21d"	},
	{ 3060, "Unified 21d"	},
	{ 3062, "Unified 21d"	},
	{ 3063, "Unified 21d"	},
	{ 3064, "Unified 21d"	},
	{ 3164, "Unified 21e"	},
	{ 3413, "Unified 21f"	},
	{ 3414, "Unified 21f"	},
	{ 3415, "Unified 21f"	},
	{ 3424, "Unified 21f"	},
	{ 3454, "Unified 21f"	},
	{ 3684, "Unified 21f"	},
	{ 3764, "Unified 21f"	},
	{ 4276, "Unified 22b"	},
	{ 4277, "Unified 22b"	},
	{ 4279, "Unified 22b"	},
	{ 4281, "Unified 22b"	},
	{ 4282, "Unified 22b"	},
	{ 4283, "Unified 22b"	},
	{ 4284, "Unified 22b"	},
	{ 4285, "Unified 22b"	},
	{ 4289, "Unified 22b"	},
	{ 4290, "Unified 22b"	},
	{ 4291, "Unified 22b"	},
	{ 4292, "Unified 22b"	},
	{ 4293, "Unified 22b"	},
	{ 4294, "Unified 22b"	},
	{ 4295, "Unified 22b"	},
	{ 4363, "Unified 22c"	},
	{ 4373, "Unified 22c"	},
	{ 4374, "Unified 22c"	},
	{ 4532, "Unified 22d"	},
	{ 4533, "Unified 22d"	},
	{ 4698, "Unified 23c"	},
	{ 4839, "Unified 23c"	},
	{ 4841, "Unified 23c"	},
	{ 4866, "Unified 23c"	},
	{ 4867, "Unified 23c"	},
	{ 4868, "Unified 23c"	},
	{ 4869, "Unified 23c"	},
	{ 4870, "Unified 23c"	},
	{ 4871, "Unified 23c"	},
	{ 4872, "Unified 23c"	},
	{ 4874, "Unified 23c"	},
	{ 4875, "Unified 23c"	},
	{ 4876, "Unified 23c"	},
	{ 4877, "Unified 23c"	},
	{ 2526, "Marcel 1 (2005-09-26)"	},
	{ 2543, "Marcel 2 (2005-09-28)"	},
	{ 2622, "Marcel 3 (2005-10-27)"	},
	{ 3326, "Marcel 4 (2006-06-16)"	},
	{ 3612, "Marcel 5 (2006-10-24)"	},
	{ 4509, "Marcel 6 (2007-06-11)"	},
	{ 5417, "Marcel 7 (2008-08-26)" },
	{  195, "Sniff 1 (2001-11-27)"	},
	{  220, "Sniff 2 (2002-01-03)"	},
	{  269, "Sniff 3 (2002-02-22)"	},
	{  270, "Sniff 4 (2002-02-26)"	},
	{  284, "Sniff 5 (2002-03-12)"	},
	{  292, "Sniff 6 (2002-03-20)"	},
	{  305, "Sniff 7 (2002-04-12)"	},
	{  306, "Sniff 8 (2002-04-12)"	},
	{  343, "Sniff 9 (2002-05-02)"	},
	{  346, "Sniff 10 (2002-05-03)"	},
	{  355, "Sniff 11 (2002-05-16)"	},
	{  256, "Sniff 11 (2002-05-16)"	},
	{  390, "Sniff 12 (2002-06-26)"	},
	{  450, "Sniff 13 (2002-08-16)"	},
	{  451, "Sniff 13 (2002-08-16)"	},
	{  533, "Sniff 14 (2002-10-11)"	},
	{  580, "Sniff 15 (2002-11-14)"	},
	{  623, "Sniff 16 (2002-12-12)"	},
	{  678, "Sniff 17 (2003-01-29)"	},
	{  847, "Sniff 18 (2003-04-17)"	},
	{  876, "Sniff 19 (2003-06-10)"	},
	{  997, "Sniff 22 (2003-09-05)"	},
	{ 1027, "Sniff 23 (2003-10-03)"	},
	{ 1029, "Sniff 24 (2003-10-03)"	},
	{ 1112, "Sniff 25 (2003-12-03)"	},
	{ 1113, "Sniff 25 (2003-12-03)"	},
	{ 1133, "Sniff 26 (2003-12-18)"	},
	{ 1134, "Sniff 26 (2003-12-18)"	},
	{ 1223, "Sniff 27 (2004-03-08)"	},
	{ 1224, "Sniff 27 (2004-03-08)"	},
	{ 1319, "Sniff 31 (2004-04-22)"	},
	{ 1320, "Sniff 31 (2004-04-22)"	},
	{ 1427, "Sniff 34 (2004-06-16)"	},
	{ 1508, "Sniff 35 (2004-07-19)"	},
	{ 1509, "Sniff 35 (2004-07-19)"	},
	{ 1587, "Sniff 36 (2004-08-18)"	},
	{ 1588, "Sniff 36 (2004-08-18)"	},
	{ 1641, "Sniff 37 (2004-09-16)"	},
	{ 1642, "Sniff 37 (2004-09-16)"	},
	{ 1699, "Sniff 38 (2004-10-07)"	},
	{ 1700, "Sniff 38 (2004-10-07)"	},
	{ 1752, "Sniff 39 (2004-11-02)"	},
	{ 1753, "Sniff 39 (2004-11-02)"	},
	{ 1759, "Sniff 40 (2004-11-03)"	},
	{ 1760, "Sniff 40 (2004-11-03)"	},
	{ 1761, "Sniff 40 (2004-11-03)"	},
	{ 2009, "Sniff 41 (2005-04-06)"	},
	{ 2010, "Sniff 41 (2005-04-06)"	},
	{ 2011, "Sniff 41 (2005-04-06)"	},
	{ 2016, "Sniff 42 (2005-04-11)"	},
	{ 2017, "Sniff 42 (2005-04-11)"	},
	{ 2018, "Sniff 42 (2005-04-11)"	},
	{ 2023, "Sniff 43 (2005-04-14)"	},
	{ 2024, "Sniff 43 (2005-04-14)"	},
	{ 2025, "Sniff 43 (2005-04-14)"	},
	{ 2032, "Sniff 44 (2005-04-18)"	},
	{ 2033, "Sniff 44 (2005-04-18)"	},
	{ 2034, "Sniff 44 (2005-04-18)"	},
	{ 2288, "Sniff 45 (2005-07-08)"	},
	{ 2289, "Sniff 45 (2005-07-08)"	},
	{ 2290, "Sniff 45 (2005-07-08)"	},
	{ 2388, "Sniff 46 (2005-08-17)"	},
	{ 2389, "Sniff 46 (2005-08-17)"	},
	{ 2390, "Sniff 46 (2005-08-17)"	},
	{ 2869, "Sniff 47 (2006-02-15)"	},
	{ 2870, "Sniff 47 (2006-02-15)"	},
	{ 2871, "Sniff 47 (2006-02-15)"	},
	{ 3214, "Sniff 48 (2006-05-16)"	},
	{ 3215, "Sniff 48 (2006-05-16)"	},
	{ 3216, "Sniff 48 (2006-05-16)"	},
	{ 3356, "Sniff 49 (2006-07-17)"	},
	{ 3529, "Sniff 50 (2006-09-21)"	},
	{ 3546, "Sniff 51 (2006-09-29)"	},
	{ 3683, "Sniff 52 (2006-11-03)"	},
	{    0, NULL }
};
 
static const char *csr_builddeftostr(uint16_t def)
{
	switch (def) {
	case 0x0000:
		return "NONE";
	case 0x0001:
		return "CHIP_BASE_BC01";
	case 0x0002:
		return "CHIP_BASE_BC02";
	case 0x0003:
		return "CHIP_BC01B";
	case 0x0004:
		return "CHIP_BC02_EXTERNAL";
	case 0x0005:
		return "BUILD_HCI";
	case 0x0006:
		return "BUILD_RFCOMM";
	case 0x0007:
		return "BT_VER_1_1";
	case 0x0008:
		return "TRANSPORT_ALL";
	case 0x0009:
		return "TRANSPORT_BCSP";
	case 0x000a:
		return "TRANSPORT_H4";
	case 0x000b:
		return "TRANSPORT_USB";
	case 0x000c:
		return "MAX_CRYPT_KEY_LEN_56";
	case 0x000d:
		return "MAX_CRYPT_KEY_LEN_128";
	case 0x000e:
		return "TRANSPORT_USER";
	case 0x000f:
		return "CHIP_BC02_KATO";
	case 0x0010:
		return "TRANSPORT_NONE";
	case 0x0012:
		return "REQUIRE_8MBIT";
	case 0x0013:
		return "RADIOTEST";
	case 0x0014:
		return "RADIOTEST_LITE";
	case 0x0015:
		return "INSTALL_FLASH";
	case 0x0016:
		return "INSTALL_EEPROM";
	case 0x0017:
		return "INSTALL_COMBO_DOT11";
	case 0x0018:
		return "LOWPOWER_TX";
	case 0x0019:
		return "TRANSPORT_TWUTL";
	case 0x001a:
		return "COMPILER_GCC";
	case 0x001b:
		return "CHIP_BC02_CLOUSEAU";
	case 0x001c:
		return "CHIP_BC02_TOULOUSE";
	case 0x001d:
		return "CHIP_BASE_BC3";
	case 0x001e:
		return "CHIP_BC3_NICKNACK";
	case 0x001f:
		return "CHIP_BC3_KALIMBA";
	case 0x0020:
		return "INSTALL_HCI_MODULE";
	case 0x0021:
		return "INSTALL_L2CAP_MODULE";
	case 0x0022:
		return "INSTALL_DM_MODULE";
	case 0x0023:
		return "INSTALL_SDP_MODULE";
	case 0x0024:
		return "INSTALL_RFCOMM_MODULE";
	case 0x0025:
		return "INSTALL_HIDIO_MODULE";
	case 0x0026:
		return "INSTALL_PAN_MODULE";
	case 0x0027:
		return "INSTALL_IPV4_MODULE";
	case 0x0028:
		return "INSTALL_IPV6_MODULE";
	case 0x0029:
		return "INSTALL_TCP_MODULE";
	case 0x002a:
		return "BT_VER_1_2";
	case 0x002b:
		return "INSTALL_UDP_MODULE";
	case 0x002c:
		return "REQUIRE_0_WAIT_STATES";
	case 0x002d:
		return "CHIP_BC3_PADDYWACK";
	case 0x002e:
		return "CHIP_BC4_COYOTE";
	case 0x002f:
		return "CHIP_BC4_ODDJOB";
	case 0x0030:
		return "TRANSPORT_H4DS";
	case 0x0031:
		return "CHIP_BASE_BC4";
	default:
		return "UNKNOWN";
	}
}

static const char *csr_buildidtostr(uint16_t id)
{
	static char str[12];
	int i;

	for (i = 0; csr_map[i].id; i++)
		if (csr_map[i].id == id)
			return csr_map[i].str;

	snprintf(str, 11, "Build %d", id);
	return str;
}

static const char *csr_chipvertostr(uint16_t ver, uint16_t rev)
{
	switch (ver) {
	case 0x00:
		return "BlueCore01a";
	case 0x01:
		switch (rev) {
		case 0x64:
			return "BlueCore01b (ES)";
		case 0x65:
		default:
			return "BlueCore01b";
		}
	case 0x02:
		switch (rev) {
		case 0x89:
			return "BlueCore02-External (ES2)";
		case 0x8a:
			return "BlueCore02-External";
		case 0x28:
			return "BlueCore02-ROM/Audio/Flash";
		default:
			return "BlueCore02";
		}
	case 0x03:
		switch (rev) {
		case 0x43:
			return "BlueCore3-MM";
		case 0x15:
			return "BlueCore3-ROM";
		case 0xe2:
			return "BlueCore3-Flash";
		case 0x26:
			return "BlueCore4-External";
		case 0x30:
			return "BlueCore4-ROM";
		default:
			return "BlueCore3 or BlueCore4";
		}
	default:
		return "Unknown";
	}
} 


/*
 * CSR specific initialization extension for Audio(PCM) Interface Configuration
 * of ROM based Chips. This function helps in configuring PCM interface
 * according to requirement of different platforms.
 * Anantha Idapalapati <aidapalapati@nvidia.com>
 */
static int csr_audio_init(void)
{
	uint8_t payload[14];

	uint16_t varid=0x5031;
	
	uint16_t portid		= 1;
	uint16_t syncport	= 0;
	uint16_t iotype		= 5; // 5-I2S Master; 8-I2S Slave.
	uint32_t readrate	= 8000;
	uint32_t writerate	= 8000;

	/* There is some configuration (like PCM interface configuration,
	*  which user may need to be done after WARM_RESET of chip. WARM_RESET
	*  is done as part of initialization. Do the configuration required
	*  for runtime as part of this function.
	*/

	// Payload format
	memset(payload, 0, sizeof(payload));
	payload[0] = portid & 0xff;
	payload[1] = (portid & 0xff00) >> 8 ;
	payload[2] = syncport & 0xff;
	payload[3] = (syncport & 0xff00) >> 8 ;
	payload[4] = iotype & 0xff;
	payload[5] = (iotype  & 0xff00) >> 8;
	payload[6] = readrate & 0xff;
	payload[7] = (readrate & 0xff00) >> 8;
	payload[8] = (readrate & 0xff0000) >> 16;
	payload[9] = readrate >> 24;
	payload[10] = writerate & 0xff;
	payload[11] = (writerate & 0xff00) >> 8;
	payload[12] = (writerate & 0xff0000) >> 16;
	payload[13] = writerate >> 24;
	
	if (csr_bccmd_write(varid, payload, sizeof(payload))) {
		ALOGE("Failed to write command (SET_BCCMD AUDIO_CONFIG)");
		return -1;
	}

	return 0;
} 

  
/* read length of pfkey from the persistent store in words, the key should exist */   
static uint16_t csr_get_pfkey_len(uint16_t pfkey)   
{   
    uint16_t varid, pskey_len, psstore;   
    uint8_t payload[6];   
   
    psstore = CSR_PSSTORE_DEFAULT;   
       
    varid = CSR_VARID_PS_SIZE;           /* varid: PS value size */   
    pskey_len = 0x0000;   
   
    memset(payload, 0, sizeof(payload));   
	
	/* payload */
    payload[0] = pfkey & 0xFF; 
	payload[1] = pfkey >> 8; 
    payload[2] = pskey_len & 0xFF; 
	payload[3] = pskey_len >> 8;   
    payload[4] = psstore & 0xFF; 
	payload[5] = psstore >> 8;   
   
    if (csr_bccmd_read(varid, payload, sizeof(payload))) {
		ALOGE("Failed to read pfkey length");
		return 0;
	}
   
    if ((payload[0] + (payload[1] << 8)) != pfkey)   
        ALOGE("incorrect pfkey");
   
    pskey_len = payload[2] + (payload[3] << 8);   
   
    // ALOGD("PSKey length: %d", pskey_len);   
   
    return pskey_len;   
   
}   
   
   
/* Read/write pfkey.   
 *   
 * Parameters:  
 *  
 * - write: if 0 then read performed, write otherwise.  
 * - pfkey: pfkey  
 * - data:  buffer to store the data to read/write  
 * - data_len: size of the data in buffer to write (in bytes)  
 *  
 * Returns the number of bytes read, or 0 on write.  
*/   
static int csr_pfkey(int write, uint16_t pfkey, uint8_t *data, int data_len)   
{   
    uint16_t bccmd_req, varid, pskey_len, psstore;   
    uint8_t payload[258];   
    int len = 0, chunk = 0;   
   
    if (write)   
    {   
		pskey_len = (data_len + 1) >> 1;   
        psstore = CSR_PSSTORE_PSRAM;    /* write to RAM - We don't want to make it permanent */   
//      psstore = CSR_PSSTORE_DEFAULT;   
   
    }   
    else   
    {   
        pskey_len = csr_get_pfkey_len(pfkey);   
        if (!pskey_len) {
            ALOGE("pfkey has zero length");   
			return -1;
		}
        psstore = CSR_PSSTORE_DEFAULT;   
    }   
   
    varid = CSR_VARID_PS;           /* varid: Read/Write PS key */   
   
    memset(payload, 0, sizeof(payload));   
   
    payload[0] = pfkey & 0xFF; 
	payload[1] = pfkey >> 8;      
    payload[2] = pskey_len & 0xFF; 
	payload[3] = pskey_len >> 8;   
    payload[4] = psstore & 0xFF; 
	payload[5] = psstore >> 8;   
   
    if (write) {
        memcpy(&payload[6], data, data_len);   
		if (csr_bccmd_write(varid, payload, (pskey_len + 3) * 2)) { 
			ALOGE("Failed to write pfkey");
			return -1;
		}
	} else {
		if (csr_bccmd_read(varid, payload, (pskey_len + 3) * 2)) { 
			ALOGE("Failed to read pfkey");
			return -1;
		}
	}
   
   
    if ((payload[0] + (payload[1] << 8)) != pfkey)   
        ALOGE("incorrect pfkey");   
   
    if ((payload[2] + (payload[3] << 8)) != pskey_len)   
        ALOGE("incorrect pfkey length");   
   
    if ((payload[4] + (payload[5] << 8)) != psstore)   
        ALOGE("incorrect psstore");   
   
    if (!write)   
    {   
        memcpy(data, &payload[6], pskey_len * 2);   
        return pskey_len * 2;   
    }   
    else   
        return 0;   
   
}   
   
static int csr_read_pfkey(uint16_t pfkey, uint8_t *data)
{
	return csr_pfkey(0, pfkey, data, 0);
}

static int csr_write_pfkey(uint16_t pfkey, uint8_t *data, int data_len)  
{
	return csr_pfkey(1, pfkey, data, data_len);
}

static int csr_read_pfkey_uint16(uint16_t pfkey, uint16_t *data)
{
	uint8_t buf[2];
	if (csr_pfkey(0, pfkey, buf, 0) != 2) {
		return -1;
	}
	*data = buf[0] + (buf[1] << 8);
	return 0;
}

static int csr_write_pfkey_uint16(uint16_t pfkey, uint16_t data)  
{
	uint8_t buf[2];
	buf[0] = data & 0xFF ; 
	buf[1] = (data >> 8) & 0xFF;    
	return csr_pfkey(1, pfkey, buf, 2) == 2 ? 0 : -1;
}


static int csr_read_pfkey_uint32(uint16_t pfkey, uint32_t *data)
{
	uint8_t buf[4];
	if (csr_pfkey(0, pfkey, buf, 0) != 2) {
		return -1;
	}
	*data = (buf[0] << 16) + (buf[1] << 24) + buf[2] + (buf[3] << 8);
	return 0;
}

static int csr_write_pfkey_uint32(uint16_t pfkey, uint32_t data)  
{
	uint8_t buf[4];
	buf[0] = (data >> 16) & 0xFF; 
	buf[1] = (data >> 24) & 0xFF;    
	buf[2] = data & 0xFF ; 
	buf[3] = (data >> 8) & 0xFF;    
	return csr_pfkey(1, pfkey, buf, 2) == 2 ? 0 : -1;
}

static int csr_read_varid_uint16(uint16_t varid, uint16_t* val)   
{   
    uint8_t payload[2];   
    memset(payload, 0, sizeof(payload));   
   
	if (csr_bccmd_read(varid, payload, 2)) { 
		ALOGE("Failed to read varid 0x%04x", varid);
		return -1;
	}

    *val = payload[0] + (payload[1] << 8);   
	return 0;
}   

static int csr_write_varid_uint16(uint16_t varid, uint16_t val)   
{   
    uint8_t payload[2];   
    memset(payload, 0, sizeof(payload));
	
    payload[0] = val & 0xFF; payload[1] = val >> 8;   

	if (csr_bccmd_write(varid, payload, 2)) {
		ALOGE("Failed to write varid 0x%04x with 0x%04x", varid, val);
		return -1;
	
	}
   
    return 0;
}   

   
/* set UART baud rate, stop bit(s), parity   
 *  
 * The change happens immediately, most probably the response on this command will be received back  
 * after the new UART settings took effect.  
 *   
 * Parameters:  
 *  
 * baud = baud rate, the value is used as a clock divisor, any value can be programmed.  
 *        the common values are: 9600, 19200, ... 115200, 230400, 460800, 921600.  
 * stop = number of stop bits, either 1 or 2  
 * parity = parity, 0 - no parity, 1 - even parity, 2 - odd parity  
 *  
*/   
static int csr_uart_config(int baud, int stop, int parity)   
{   
    uint16_t divisor;   
	int ret;	
      
    divisor = (baud*64+7812)/15625;   
   
    if (stop == 1)   
        divisor |= 0x0000;   
    else if (stop == 2)   
        divisor |= 0x2000;   
    else   
        ALOGE("invalid stop bit number");   
       
    if (parity == 0)   
        divisor |= 0x0000;   
    else if (parity == 1)   
        divisor |= 0xC000;   
    else if (parity == 2)   
        divisor |= 0x4000;   
    else   
        ALOGE("invalid parity bit value");   
   
    ret = csr_write_varid_uint16(CSR_VARID_CONFIG_UART, divisor);   
	if (ret) {
		ALOGE("Failed to set uart speed");
	}
   
    return ret;   
   
}   
   
/* enter DEEP_SLEEP mode  
 *  
 * The change happens approx. 0.5s after sending event back  
 *  
*/   
static int csr_deep_sleep(void)   
{   
    uint16_t testid;   
	int ret;
   
	testid = 10;            /* DEEP_SLEEP */   
    
    ret = csr_write_varid_uint16(CSR_VARID_RADIOTEST, testid);   
	if (ret) {
		ALOGE("Failed to set uart speed");
	}
   
    return ret;   
}   
   
/* Forces the module to reboot immediately  
 *  
 * Parameters:  
 *  
 * warm = 1 (warm reboot, RAM store is preserved) or 0 (cold reboot, the same as   
 *           power-cycling the module)  
 *  
 * It's most probable that response will never be able to make it back, so don't bother  
*/   
static int csr_reboot(int warm)   
{   
    uint16_t varid;   
    uint8_t payload[2];   
	int ret;

    if (warm == 0)   
        varid = CSR_VARID_COLD_RESET;           /* varid: COLD_RESET */   
    else if (warm == 1)   
        varid = CSR_VARID_WARM_RESET;           /* varid: WARM_RESET */   
    else   
        ALOGE("invalid reset type");   
   
    memset(payload, 0, sizeof(payload));   
   
    ret = csr_bccmd_write(varid, payload, 0);   
   
    return ret;   
   
}   

/* read CSR firmware build id/number */   
static uint16_t csr_getbuildid(void)   
{   
    uint16_t buildid = 0;   
	csr_read_varid_uint16(CSR_VARID_BUILDID, &buildid);
    return buildid;   
}   

/* read CSR chip version */   
static uint16_t csr_getchipver(void)   
{   
    uint16_t chipver = 0;   
	csr_read_varid_uint16(CSR_VARID_CHIPVER, &chipver);
    return chipver;   
}   

/* read CSR chip revision */   
static uint16_t csr_getchiprev(void)   
{   
    uint16_t chiprev = 0;   
	csr_read_varid_uint16(CSR_VARID_CHIPREV, &chiprev);
    return chiprev;   
}   

/* read CSR max crypt key len */   
static uint16_t csr_maxcryptkeylen(void)   
{   
    uint16_t maxlen = 0;   
	csr_read_varid_uint16(CSR_VARID_MAX_CRYPT_KEY_LENGTH, &maxlen);
    return maxlen;   
}   

static void csr_print_rev(void)
{
	uint16_t buildid, chipver, chiprev, maxkeylen, mapsco;

	buildid = csr_getbuildid();
	ALOGD("0x%04x (%s)", buildid, csr_buildidtostr(buildid));

	chipver = csr_getchipver();
	chiprev = csr_getchiprev();
	ALOGD("Chip version: %s", csr_chipvertostr(chipver, chiprev));

	maxkeylen = csr_maxcryptkeylen();
	ALOGD("Max key size: %d bit", maxkeylen * 8);

	csr_read_pfkey_uint16(CSR_PSKEY_HOSTIO_MAP_SCO_PCM, &mapsco);
	ALOGD("SCO mapping:  %s", mapsco ? "PCM" : "HCI");
}
 

static int char2dig(char c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'A' && c <= 'F')
		return c + 10 - 'A';
	if (c >= 'a' && c <= 'f')
		return c + 10 - 'a';
	return -1;
}

static int ishexdig(char c)
{
	if (c >= '0' && c <= '9')
		return 1;
	if (c >= 'A' && c <= 'F')
		return 1;
	if (c >= 'a' && c <= 'f')
		return 1;
	return 0;
}
  
static int str2val(char* s, int base)
{
	int res = 0, v;
	while (*s) {
		v = char2dig(*s++);
		if (v >= base || v < 0) {
			return -1;
		}
		res *= base;
		res += v;
	}
	return res;
}

static int csr_load_config(void)
{
	char c;
	char hexc[5];
	int line = 1;
	int v, pos = 0;
	uint16_t pfkey = 0;
	uint8_t val[256];
	
	int cf = open(cfg.psrfile, O_RDONLY);
	if (cf < 0) {
		ALOGE("Unable to open PSR file '%s'", cfg.psrfile);
		return -1;
	}
	
	while (read(cf, &c, 1) == 1) {
		if (c == '\n')
			line++;
	
		if (c <= ' ')
			continue;
			
		// Start of line ?
		if (c != '&') {
			// No, skip to the next line
			while (read(cf, &c, 1) == 1 && c != '\n');
			line++;
			continue;
		}
		
		// Start of line. Process it
		if (read(cf, &hexc, 4) != 4) {
			ALOGE("Error at config file line %d: hex constants must be 4 chars long",line);
			break;
		}
		hexc[4] = 0;
		
		// Convert to decimal
		v = str2val(hexc,16);
		if (v < 0) {
			ALOGE("Error at config file line %d: invalid hex char",line);
			break;
		}
		pfkey = v;
		
		// Skip the " = " and spaces...
		int pos = 0;
		while (1) {
		
			// Skip spaces between lines...
			while (read(cf, &c, 1) == 1 && c != '\n' && !ishexdig(c));

			if (c == '\n') {
				line++;
				break;
			}

			hexc[0] = c;

			// Start of number. Process it
			if (read(cf, &hexc[1], 3) != 3) {
				ALOGE("Error at config file line %d, pos=%d: hex constants must be 4 chars long",line,pos);
				break;
			}
			hexc[4] = 0;
			
			// Convert to decimal
			v = str2val(hexc,16);
			if (v < 0) {
				ALOGE("Error at config file line %d, pos=%d: invalid hex char",line,pos);
				break;
			}
			val[pos++] = v;
			val[pos++] = v >> 8;
		};
		
		// We must override some specific values to make the chip work
		switch (pfkey) {
			case CSR_PSKEY_BDADDR:
				{
					ALOGD("Overriding BDADDR to %02X-%02X-%02X-%02X-%02X-%02X",
						0, vnd_local_bd_addr[1], vnd_local_bd_addr[2],
						vnd_local_bd_addr[3], vnd_local_bd_addr[4],	vnd_local_bd_addr[5]);
					
					val[6] = vnd_local_bd_addr[1]; val[7] = 0;
					val[4] = vnd_local_bd_addr[2]; val[5] = 0;
					val[0] = vnd_local_bd_addr[3]; val[1] = 0;
					val[3] = vnd_local_bd_addr[4];
					val[2] = vnd_local_bd_addr[5];
					pos = 8;
				}
				break;
#ifndef HW_TENDERLOIN
			case CSR_PSKEY_UART_BAUD_RATE:
				{
					uint16_t divisor = (cfg.speed * 64 + 7812) / 15625;   
					
					ALOGD("Overriding baudrate to %d", cfg.speed);
					
					val[0] = divisor;
					val[1] = divisor >> 8;
					pos = 2;
				}
				break;
			case CSR_PSKEY_UART_CONFIG:
				{
					uint16_t uart_conf = 0;   
					
					ALOGD("Overriding UART config");
					
					if ((cfg.flags & EVEN_PARITY)) {
						uart_conf |= 6;		/* one stop bit, even parity */ 
					}
					
					if ((cfg.flags & FLOW_CTL)) {
						uart_conf |= 0x28;	/* CTS/RTS flow control */
					}
					val[0] = uart_conf;
					val[1] = uart_conf >> 8;
					pos = 2;
				}
				break;
			case CSR_PSKEY_HOST_INTERFACE:
				{
					uint16_t host_if = 1; // BCSP mode
					
					ALOGD("Overriding HOST mode to BCSP");
					
					val[0] = host_if;
					val[1] = host_if >> 8;
					pos = 2;
				}
				break;
#endif
		}
		
		ALOGD("PFKEY = 0x%04x", pfkey);
		ALOGD("Values: (count = %d)", pos/2);
		for (v = 0 ; v < pos; v += 2) {
			ALOGD(" [%d] = 0x%04x", v, val[v] + (val[v+1] << 8));
		}
		
		// Send the command!
		if (csr_write_pfkey( pfkey, val, pos)) {
			ALOGD("Failed to write config");
			close(cf);
			return -1;
		}
	};
	
	
	close(cf);
	return 0;
}
 
int bcsp(void)   
{   
    uint8_t buf[258];   
    unsigned long written;   
    uint16_t len;   
    uint16_t divisor, uart_conf, retries, host_if, val16;   
	uint32_t val32;
	
    int status;   
	uint16_t deep_sleep;
	int i;
   
      
	/* Print bluetooth chipset info */
	csr_print_rev();
   
    /* Read bluetooth address */
    len = csr_read_pfkey( CSR_PSKEY_BDADDR, buf);   
    if (len != 8)   
        ALOGE("CSR_PSKEY_BDADDR");   
		
    /* It's hard to make it more akward than that ;) */   
    ALOGD("read:CSR_PSKEY_BDADDR:    %02X-%02X-%02X-%02X-%02X-%02X", 0,   
                                                            buf[6] + (buf[7] << 8),   
                                                            buf[4] + (buf[5] << 8),   
                                                            buf[0] + (buf[1] << 8),   
                                                            buf[3],   
                                                            buf[2]);   
															
	ALOGD("Overriding BDADDR to %02X-%02X-%02X-%02X-%02X-%02X",
			0, vnd_local_bd_addr[1], vnd_local_bd_addr[2],
			vnd_local_bd_addr[3], vnd_local_bd_addr[4],	vnd_local_bd_addr[5]);
					
	buf[6] = vnd_local_bd_addr[1]; buf[7] = 0;
	buf[4] = vnd_local_bd_addr[2]; buf[5] = 0;
	buf[0] = vnd_local_bd_addr[3]; buf[1] = 0;
	buf[3] = vnd_local_bd_addr[4];
	buf[2] = vnd_local_bd_addr[5];
	csr_write_pfkey( CSR_PSKEY_BDADDR, buf, 8);   

    /* Read bluetooth address */
    len = csr_read_pfkey( CSR_PSKEY_BDADDR, buf);   
    if (len != 8)   
        ALOGE("CSR_PSKEY_BDADDR");   
		
    /* It's hard to make it more akward than that ;) */   
    ALOGD("read:CSR_PSKEY_BDADDR:    %02X-%02X-%02X-%02X-%02X-%02X", 0,   
                                                            buf[6] + (buf[7] << 8),   
                                                            buf[4] + (buf[5] << 8),   
                                                            buf[0] + (buf[1] << 8),   
                                                            buf[3],   
                                                            buf[2]);   
	
															
	/* PSKEY structures  
	  
	0. CSR_PSKEY_HOSTIO_UART_PS_BLOCK (0x191): This is a block of 10 keys which are described below. 
		If changing CSR_PSKEY_UART_CONFIG[7], make sure that CSR_PSKEY_HOST_INTERFACE is set accordingly. 
		Both BCSP protocol and BCSP hardware should be disabled or enabled at the same time

	  
	0       1       2       3       4       5       6       7       8       9  
	  
	3B0     A8      FA      14      4       0       4       1E      64      A  
	  
	1. CSR_PSKEY_WD_PERIOD (0x1F8) and CSR_PSKEY_WD_TIMEOUT (0x1F7): These keys control Deep Sleep watchdog 
		timer and specify the time in microseconds. PERIOD defines the interval between kicking 
		the watchdog. TIMEOUT defines the interval after which watchdog reboots BC2. BC2 needs 
		to wake up from Deep Sleep to kick the watchdog first (otherwise it itself gets kicked 
		(rebooted) by the watchdog), thus the PERIOD key limits the maximum amount of time BC2 
		can stay in Deep Sleep continiously. After kicking the watchdog, BC2 enters Deep Sleep 
		again if there's no wake up condition detected. It's not recommended to change these keys, 
		but if you do, make sure that TIMEOUT > PERIOD, otherwise Deep Sleep will become Deep Pain ;)
		
	2. CSR_PSKEY_HOST_INTERFACE (0x1F9): Protocol to use: 1(UART BCSP), 2(USB), 3(UART H4). 
		Set to 1 to use BCSP
		
	3. CSR_PSKEY_MKT_TASK_ACTIVE (0x1FA) and CSR_PSKEY_DEBUG_TASK_PERIOD (0x218). Both of these keys were 
		depreciated in HCI firmware 15.x and later. If you use older firmware you might want to 
		make sure that ACTIVE is set to 0
	  
	4.[0] CSR_PSKEY_UART_BAUD_RATE (0x204)  
	   0x3B0 == 230400 bps  
	=> 0x1D8 == 115200 bps  
	  
	  
	5.[1] CSR_PSKEY_UART_CONFIG (0x205): A bit field that represents various UART settings. The bits are:

		0: 0 (one stop bit)		1 (two stop bits)
		1: 0 (no parity bits)	1 (one parity bit)
		2: 0 (odd parity)		1 (even parity)
		3: 0 (no CTS/RTS flow ctrl)	1 (CTS/RTS flow control)
		4: 0 (no automatic RTS)	1 (automatic RTS)
		5:    RTS position (on which bit turn RTS on)
		6: 0 (no TX zero)		1 (TX zero)
		7: 0 (BCSP hardware)		1 (non-BCSP hardware)
		8:    RX rate delay (LSB)
		9:    RX rate delay (MSB)
   
		Bit [7] controls BCSP hardware, it should be unset for BCSP protocol, and set otherwise.
		Here're a few examples:
   
		0xA8 == CTS/RTS hardware flow control, RTS on 1st bit, non-BCSP hardware (default for H4)
		0x06 == even parity, BCSP hardware (default for BCSP)
	
		=> 6    == one parity bit, even parity (BCSP) 
		=> 168  == no parity, H4, enables hardware flow control
	  
	6. CSR_PSKEY_UART_SLEEP_TIMEOUT (0x222): Defines the time in milliseconds of inactivity after which 
		BC2 enters Deep Sleep mode. One second is default value. If changing it, make sure it's 
		not less than BCSP retransmission timeout (normally 250ms), otherwise BC2 enters Deep 
		Sleep again before getting retransmitted packet
	   
	7. CSR_PSKEY_DEEP_SLEEP_STATE (0x229): Enable/disable Deep Sleep mode. 
		If using BCSP protocol, that key needs to be set to 1 (use Deep Sleep whenever possible) 
		for most power saving. The other possible values are: 0 (Deep Sleep disabled) and 2 
		(use Deep Sleep only when there're no ACL connections)
		
	8. CSR_PSKEY_DEEP_SLEEP_WAKE_CTS (0x23C): Whenever activity on UART CTS line can wake up BC2 from Deep 
		Sleep. Set it to 1 if using CTS line to wake up BC2
	  
	9.[2] CSR_PSKEY_UART_SEQ_TIMEOUT (0x405): Time-out in milliseconds to wait before trying to resend
		BCSP packet. Only relevant when using BCSP protocol.

		0xFA == 250ms (default value)
	  
	10.[3] CSR_PSKEY_UART_SEQ_RETRIES (0x406): The number of retries for BCSP packet retransmission.
		After this number reached, BC2 gives up and marks BCSP link as broken and then reboots.
		Only relevant when using BCSP protocol.
   
		0x14 == 20 retries (default value). To retry indefinitely set it to 0.
	    0x14 == 20 retries.  
		=> 0 == retry indefinitely  
	  
	11.[4] CSR_PSKEY_UART_SEQ_WINSIZE (0x407)  
	  
	   4 == number of unacknowledged packets that can be sent before waiting   
	   for an acknowledgment  
	  
	12.[5] CSR_PSKEY_UART_USE_CRC_ON_TX (0x408)  
	   0 == disabled  
	  
	13.[6] CSR_PSKEY_UART_HOST_INITIAL_STATE (0x409): When BC2 boots up it will judge the state
		of the other end of communication link based on this value:
   
		0 (normal operation)
		1 (requires waking up before starting communication)
		2 (we need to wake it up)
		3 (can doze after waking up and before we send data)
		4 (never sleeps). Since we don't want to wake up Psion, we leave it at default 4.  
	   4 == host never sleeps  
	  
	14.[7] CSR_PSKEY_UART_HOST_ATTENTION_SPAN (0x40A): The time in seconds the other end of 
		communication link falls asleep after receiving the data. Not relevant if 
		CSR_PSKEY_UART_HOST_INITIAL_STATE == 4.
   
		0x1E == 30 seconds  
	  
	15.[8] CSR_PSKEY_UART_HOST_WAKEUP_TIME (0x40B): Time in milliseconds the other end of
		communication link requires to wake up before it gets ready to receive the data.
		Not relevant if CSR_PSKEY_UART_HOST_INITIAL_STATE == 4.
   
		0x64 == 100ms

	  
	16.[10] CSR_PSKEY_UART_HOST_WAKEUP_WAIT (0x40C): Time in milliseconds BC2 needs to wait 
		before sending the data after the other side of communication line has woken up.
		Not relevant if CSR_PSKEY_UART_HOST_INITIAL_STATE == 4.
   
		0xA == 10ms

	  
	*/   

#ifndef HW_TENDERLOIN
    len = csr_read_pfkey( CSR_PSKEY_HOSTIO_UART_PS_BLOCK, buf);   
    if (len != 20) {
        ALOGE("CSR_PSKEY_HOST_INTERFACE"); 

		// Failed to get the block... Retry one by one...
		
		/* ---------------------------------------------------------------------------- */  
		if (csr_read_pfkey_uint16( CSR_PSKEY_UART_BAUD_RATE, &divisor)) {
			ALOGE("CSR_PSKEY_UART_BAUD_RATE");   
		} else {
			ALOGD("read:CSR_PSKEY_UART_BAUD_RATE:      %d (%d bps)", divisor, ((divisor * 15625) - 7812) / 64 );   
		}

		divisor = (cfg.speed * 64 + 7812) / 15625;   
		
		ALOGD("write:CSR_PSKEY_UART_BAUD_RATE:      %d (%d bps)", divisor, ((divisor * 15625) - 7812) / 64 );   
		csr_write_pfkey_uint16( CSR_PSKEY_UART_BAUD_RATE, divisor);   
		if (csr_read_pfkey_uint16( CSR_PSKEY_UART_BAUD_RATE, &divisor)) {
			ALOGE("CSR_PSKEY_UART_BAUD_RATE");   
		} else {
			ALOGD("read:CSR_PSKEY_UART_BAUD_RATE:      %d (%d bps)", divisor, ((divisor * 15625) - 7812) / 64 );   
		}
		
		/* ---------------------------------------------------------------------------- */  
		if (csr_read_pfkey_uint16( CSR_PSKEY_UART_CONFIG, &uart_conf)) {
			ALOGE("CSR_PSKEY_UART_CONFIG");   
		} else {
			ALOGD("read:CSR_PSKEY_UART_CONFIG:      %d", uart_conf);   
		}

		uart_conf = 0;		
		if ((cfg.flags & EVEN_PARITY)) {
			uart_conf |= 6;		/* one stop bit, even parity */ 
		}
		
		if ((cfg.flags & FLOW_CTL)) {
			uart_conf |= 0x28;	/* CTS/RTS flow control */
		}

		
		ALOGD("write:CSR_PSKEY_UART_CONFIG:     %d", uart_conf);   
		csr_write_pfkey_uint16( CSR_PSKEY_UART_CONFIG, uart_conf);   
		if (csr_read_pfkey_uint16( CSR_PSKEY_UART_CONFIG, &uart_conf)) {
			ALOGE("CSR_PSKEY_UART_CONFIG");   
		} else {
			ALOGD("read:CSR_PSKEY_UART_CONFIG:      %d", uart_conf);   
		}

		/* ---------------------------------------------------------------------------- */  
		if (csr_read_pfkey_uint16( CSR_PSKEY_UART_SEQ_RETRIES, &retries)) {
			ALOGE("CSR_PSKEY_UART_SEQ_RETRIES");   
		} else {
			ALOGD("read:CSR_PSKEY_UART_SEQ_RETRIES:      %d", retries);   
		}

		retries = 0; 		/* retry indefinitely on BCSP packet transmission */   
		
		ALOGD("write:CSR_PSKEY_UART_SEQ_RETRIES:     %d", retries);   
		csr_write_pfkey_uint16( CSR_PSKEY_UART_SEQ_RETRIES, retries);   
		if (csr_read_pfkey_uint16( CSR_PSKEY_UART_SEQ_RETRIES, &retries)) {
			ALOGE("CSR_PSKEY_UART_SEQ_RETRIES");   
		} else {
			ALOGD("read:CSR_PSKEY_UART_SEQ_RETRIES:      %d", retries);   
		}
		
		
	} else {
	
		ALOGD("read:CSR_PSKEY_HOSTIO_UART_PS_BLOCK:    ");   
		for (len = 0; len < 20; len = len + 2)   
			ALOGD("%02x ", buf[len] + (buf[len+1] << 8));   
   
		divisor = (cfg.speed * 64 + 7812) / 15625;   
		buf[0] = divisor & 0xFF ; 
		buf[1] = (divisor >> 8) & 0xFF;    
   
		uart_conf = 0;		
		if ((cfg.flags & EVEN_PARITY)) {
			uart_conf |= 6;		/* one stop bit, even parity */ 
		}
		
		if ((cfg.flags & FLOW_CTL)) {
			uart_conf |= 0x28;	/* CTS/RTS flow control */
		}

		buf[2] = uart_conf & 0xFF ; 
		buf[3] = (uart_conf >> 8) & 0xFF;     
    
		buf[6] = retries & 0xFF ; 
		buf[7] = (retries >> 8) & 0xFF;    
   
		ALOGD("write:CSR_PSKEY_HOSTIO_UART_PS_BLOCK:   ");   
		for (len = 0; len < 10*2; len = len + 2)   
			ALOGD("%02x ", buf[len] + (buf[len+1] << 8));   
		ALOGD("\n");   
		csr_write_pfkey( CSR_PSKEY_HOSTIO_UART_PS_BLOCK, buf, 20);   

	
		len = csr_read_pfkey( CSR_PSKEY_HOSTIO_UART_PS_BLOCK, buf);   
		if (len != 20) {
			ALOGE("CSR_PSKEY_HOST_INTERFACE");   
		} else {
			ALOGD("read:CSR_PSKEY_HOSTIO_UART_PS_BLOCK:    ");   
			for (len = 0; len < 20; len = len + 2)   
				ALOGD("%02x ", buf[len] + (buf[len+1] << 8));   
			ALOGD("\n");   
		}
	}
	
	/* ---------------------------------------------------------------------------- */   
    /* CSR_PSKEY_HOST_INTERFACE should be change at the same time as CSR_PSKEY_HOSTIO_UART_PS_BLOCK */   
    if (csr_read_pfkey_uint16( CSR_PSKEY_HOST_INTERFACE, &host_if)) {
        ALOGE("CSR_PSKEY_HOST_INTERFACE");   
	} else {
		ALOGD("read:CSR_PSKEY_HOST_INTERFACE:      %d", host_if);   
	}
	
	host_if = 1; /* UART in BCSP mode */   

    ALOGD("write:CSR_PSKEY_HOST_INTERFACE:     %d", host_if);   
    csr_write_pfkey_uint16( CSR_PSKEY_HOST_INTERFACE, host_if);   
    if (csr_read_pfkey_uint16( CSR_PSKEY_HOST_INTERFACE, &host_if)) {
        ALOGE("CSR_PSKEY_HOST_INTERFACE");   
	} else {
		ALOGD("read:CSR_PSKEY_HOST_INTERFACE:      %d", host_if);   
	}
   
	/* ---------------------------------------------------------------------------- */   
    if (csr_read_pfkey_uint16( CSR_PSKEY_DEEP_SLEEP_STATE, &deep_sleep)) {
        ALOGE("CSR_PSKEY_DEEP_SLEEP_STATE");   
	} else {
		ALOGD("read:CSR_PSKEY_DEEP_SLEEP_STATE:        %d", deep_sleep);   
	}
		
	/* ---------------------------------------------------------------------------- */   
    if (csr_read_pfkey_uint16( CSR_PSKEY_DEEP_SLEEP_WAKE_CTS, &val16)) { 
        ALOGE("CSR_PSKEY_DEEP_SLEEP_WAKE_CTS");   
	} else {
		ALOGD("read:CSR_PSKEY_DEEP_SLEEP_WAKE_CTS:     %d", val16);   
	}
   
	/* ---------------------------------------------------------------------------- */   
    if (csr_read_pfkey_uint16( CSR_PSKEY_UART_SLEEP_TIMEOUT, &val16)) {
        ALOGE("CSR_PSKEY_UART_SLEEP_TIMEOUT");   
	} else {
		ALOGD("read:CSR_PSKEY_UART_SLEEP_TIMEOUT:      %d ms", val16);   
	}
   
	/* ---------------------------------------------------------------------------- */   
    if (csr_read_pfkey_uint32( CSR_PSKEY_WD_PERIOD, &val32)) {
        ALOGE("CSR_PSKEY_WD_PERIOD");   
	} else {
		ALOGD("read:CSR_PSKEY_WD_PERIOD:           %d ms", val32/1000);   
	}
   
	/* ---------------------------------------------------------------------------- */   
    if (csr_read_pfkey_uint32( CSR_PSKEY_WD_TIMEOUT, &val32)) {
        ALOGE("CSR_PSKEY_WD_TIMEOUT");   
	} else {
		ALOGD("read:CSR_PSKEY_WD_TIMEOUT:          %d ms", val32/1000);   
	}
   
	/* ---------------------------------------------------------------------------- */   
    if (csr_read_pfkey_uint32( CSR_PSKEY_HOSTIO_UART_RESET_TIMEOUT, &val32)) {
        ALOGE("CSR_PSKEY_HOSTIO_UART_RESET_TIMEOUT");   
	} else {
		ALOGD("read:CSR_PSKEY_HOSTIO_UART_RESET_TIMEOUT:   %d ms", val32/1000);   
	}
   
	/* ---------------------------------------------------------------------------- */   
    if (csr_read_pfkey_uint32( CSR_PSKEY_HOSTIO_BREAK_POLL_PERIOD, &val32))  { 
        ALOGE("CSR_PSKEY_HOSTIO_BREAK_POLL_PERIOD");   
	} else {
		ALOGD("read:CSR_PSKEY_HOSTIO_BREAK_POLL_PERIOD:    %d ms", val32/1000);   
	}
#endif

	// Load the Bluetooth PSR file into the chip
	if (csr_load_config() < 0) {
		ALOGE("csr_load_config failed");
		hci_close();
		return -1;
	}

    ALOGD("Rebooting device.... Should restart in UART BCSP @ %d,%c,%sflow", cfg.speed, (cfg.flags & EVEN_PARITY) ? 'E': 'N', (cfg.flags & FLOW_CTL) ? "" : "no" );   
   
    /* warm reboot */   
    if (csr_reboot(1) < 0) {
		ALOGE("csr_reboot failed");
		hci_close();
		return -1;
	}
	hci_close();
	
	/* Give time to reboot ... */
	usleep(500000);
	
	/* Reopen in the new mode / new speed and redo the link negotiation */
	if (hci_open(cfg.speed, omH4) < 0) {
        ALOGE("Unable to open Bluetooth [2]");
		return -1;
	}

#ifndef HW_TENDERLOIN
   	/* Print bluetooth chipset info */
	csr_print_rev();
	
	/* Setup bluecore6 chip (again, just in case warm reboot did something to our config */
	csr_load_config();
#endif
	
	ALOGD("Done");
   
    return 0;   
   
}  

static int csr_tegra(void)
{
	// Start with an standard BCSP negotiation
	int ret = bcsp();
	if (ret < 0)
		return ret;
		
	// And now, do the audio configuration (must be done after warm reset)
	return csr_audio_init();
}

static int csr(void) 
{   
    uint8_t buf[258];   
    unsigned long written;   
    uint16_t len;   
    uint16_t divisor, uart_conf, retries, host_if, val16;   
	uint32_t val32;
	
    int status;   
	uint16_t deep_sleep;
	int i;
   
      
	/* Print bluetooth chipset info */
	csr_print_rev();
   
    /* Read bluetooth address */
    len = csr_read_pfkey( CSR_PSKEY_BDADDR, buf);   
    if (len != 8)   
        ALOGE("CSR_PSKEY_BDADDR");   
		
    /* It's hard to make it more akward than that ;) */   
    ALOGD("read:CSR_PSKEY_BDADDR:    %02X-%02X-%02X-%02X-%02X-%02X", 0,   
                                                            buf[6] + (buf[7] << 8),   
                                                            buf[4] + (buf[5] << 8),   
                                                            buf[0] + (buf[1] << 8),   
                                                            buf[3],   
                                                            buf[2]);   

	ALOGD("Overriding BDADDR to %02X-%02X-%02X-%02X-%02X-%02X",
			0, vnd_local_bd_addr[1], vnd_local_bd_addr[2],
			vnd_local_bd_addr[3], vnd_local_bd_addr[4],	vnd_local_bd_addr[5]);
					
	buf[6] = vnd_local_bd_addr[1]; buf[7] = 0;
	buf[4] = vnd_local_bd_addr[2]; buf[5] = 0;
	buf[0] = vnd_local_bd_addr[3]; buf[1] = 0;
	buf[3] = vnd_local_bd_addr[4];
	buf[2] = vnd_local_bd_addr[5];
	csr_write_pfkey( CSR_PSKEY_BDADDR, buf, 8);   

    /* Read bluetooth address */
    len = csr_read_pfkey( CSR_PSKEY_BDADDR, buf);   
    if (len != 8)   
        ALOGE("CSR_PSKEY_BDADDR");   
		
    /* It's hard to make it more akward than that ;) */   
    ALOGD("read:CSR_PSKEY_BDADDR:    %02X-%02X-%02X-%02X-%02X-%02X", 0,   
                                                            buf[6] + (buf[7] << 8),   
                                                            buf[4] + (buf[5] << 8),   
                                                            buf[0] + (buf[1] << 8),   
                                                            buf[3],   
                                                            buf[2]);   
															

#ifndef HW_TENDERLOIN
	/* ---------------------------------------------------------------------------- */  
	if (csr_read_pfkey_uint16( CSR_PSKEY_UART_BAUD_RATE, &divisor)) {
		ALOGE("CSR_PSKEY_UART_BAUD_RATE");   
	} else {
		ALOGD("read:CSR_PSKEY_UART_BAUD_RATE:      %d (%d bps)", divisor, ((divisor * 15625) - 7812) / 64 );   
	}
#endif

	divisor = (cfg.speed * 64 + 7812) / 15625;   
	
	ALOGD("write:CSR_PSKEY_UART_BAUD_RATE:      %d (%d bps)", divisor, ((divisor * 15625) - 7812) / 64 );   
	csr_write_pfkey_uint16( CSR_PSKEY_UART_BAUD_RATE, divisor);   
	if (csr_read_pfkey_uint16( CSR_PSKEY_UART_BAUD_RATE, &divisor)) {
		ALOGE("CSR_PSKEY_UART_BAUD_RATE");   
	} else {
		ALOGD("read:CSR_PSKEY_UART_BAUD_RATE:      %d (%d bps)", divisor, ((divisor * 15625) - 7812) / 64 );   
	}
	
	
    ALOGD("Rebooting device...");   
   
    /* warm reboot */   
    csr_reboot(1);   
	hci_close();
	
	/* Give time to reboot ... */
	usleep(500000);
	
	/* Reopen in the new mode / new speed and redo the link negotiation */
	if (hci_open(cfg.speed, omH4) < 0) {
        ALOGE("Unable to open Bluetooth [2]");
		return -1;
	}
   
   	/* Print bluetooth chipset info */
	csr_print_rev();
	
	ALOGD("Done");
   
    return 0;   
   
}  

// UART definitions
static const struct uart_t uart[] = {
	{ "hci",        HCI_UART_NONE,     -1,     -1, FLOW_CTL, 			NULL     },

	{ "any",        HCI_UART_H4,   115200, 115200, FLOW_CTL, 			NULL     },
	{ "ericsson",   HCI_UART_H4,   57600,  115200, FLOW_CTL, 			ericsson },
	{ "digi",       HCI_UART_H4,   9600,   115200, FLOW_CTL, 			digi     },

	{ "bcsp",       HCI_UART_BCSP, 115200, 115200, 0,        			bcsp     },
    { "csr_tegra",  HCI_UART_BCSP, 921600, 3000000,FLOW_CTL|EVEN_PARITY,csr_tegra },

	/* Xircom PCMCIA cards: Credit Card Adapter and Real Port Adapter */
	{ "xircom",     HCI_UART_H4,   115200, 115200, FLOW_CTL, 			NULL     },

	/* CSR Casira serial adapter or BrainBoxes serial dongle (BL642) */
	{ "csr",        HCI_UART_H4,   115200, 115200, FLOW_CTL, 			csr      },

	/* BrainBoxes PCMCIA card (BL620) */
	{ "bboxes",     HCI_UART_H4,   115200, 460800, FLOW_CTL, 			csr      },  
	
	/* Silicon Wave kits */
	{ "swave",      HCI_UART_H4,   115200, 115200, FLOW_CTL, 			swave    },

	/* ST Microelectronics minikits based on STLC2410/STLC2415 */
	{ "st",         HCI_UART_H4,    57600, 115200, FLOW_CTL, 			st       },

	/* Philips generic Ericsson IP core based */
	{ "philips",    HCI_UART_H4,   115200, 115200, FLOW_CTL, 			NULL     },

	/* Sphinx Electronics PICO Card */
	{ "picocard",   HCI_UART_H4,   115200, 115200, FLOW_CTL, 			NULL     },

	/* Inventel BlueBird Module */
	{ "inventel",   HCI_UART_H4,   115200, 115200, FLOW_CTL, 			NULL     },

	/* COM One Platinium Bluetooth PC Card */
	{ "comone",     HCI_UART_BCSP, 115200, 115200, 0,        			bcsp     },

	/* TDK Bluetooth PC Card and IBM Bluetooth PC Card II */
	{ "tdk",        HCI_UART_BCSP, 115200, 115200, 0,        			bcsp     },

	/* Socket Bluetooth CF Card (Rev G) */
	{ "socket",     HCI_UART_BCSP, 230400, 230400, 0,        			bcsp     },

	/* 3Com Bluetooth Card (Version 3.0) */
	{ "3com",       HCI_UART_H4,   115200, 115200, FLOW_CTL, 			csr      },

	/* AmbiCom BT2000C Bluetooth PC/CF Card */
	{ "bt2000c",    HCI_UART_H4,    57600, 460800, FLOW_CTL, 			csr      },

	/* Zoom Bluetooth PCMCIA Card */
	{ "zoom",       HCI_UART_BCSP, 115200, 115200, 0,        			bcsp     },

	/* Sitecom CN-504 PCMCIA Card */
	{ "sitecom",    HCI_UART_BCSP, 115200, 115200, 0,        			bcsp     },

	/* Billionton PCBTC1 PCMCIA Card */
	{ "billionton", HCI_UART_BCSP, 115200, 115200, 0,        			bcsp     },

	{ NULL, 0, 0, 0, 0, NULL}
};

static const struct uart_t * get_by_type(char *type)
{
	int i;
	for (i = 0; uart[i].type; i++) {
		if (!strcmp(uart[i].type, type))
			return &uart[i];
	}
	return NULL;
}


void hw_init(void)
{
	ALOGE("Enter hw_init");
	strcpy(cfg.dev,"/dev/ttyHS2");
	strcpy(cfg.type,"hci");
	strcpy(cfg.psrfile,"bluecore6.psr");
	cfg.init_speed = -1;
	cfg.speed = -1;
	cfg.flags = -1;
	cfg.send_break = 0;
	cfg.raw = 0;
	cfg.ufd = -1;
	cfg.sfd = -1;
	cfg.seqnum = 0x0000;
	cfg.hci_mode = omUnknown;
	cfg.dev_id = 0;
	
	/* RFKILL support */  
	cfg.rfkill_id = -1;
	cfg.rfkill_state_path = NULL;
	ALOGE("Leaving HW_init and type = '%s'", cfg.type);

}

void hw_close(void)
{
	hci_close();
	free(cfg.rfkill_state_path);
	cfg.rfkill_state_path = NULL;
}

/*******************************************************************************
**
** Function        hw_set_devid
**
** Description     Configure HCI device to use
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int hw_set_devid(char *p_conf_name, char *p_conf_value, int param)
{
    cfg.dev_id = atoi(p_conf_value);
    return 0;
}


/*******************************************************************************
**
** Function        hw_set_device
**
** Description     Configure HCI device to use
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int hw_set_device(char *p_conf_name, char *p_conf_value, int param)
{
    strcpy(cfg.dev, p_conf_value);
    return 0;
} 


/*******************************************************************************
**
** Function        hw_set_type
**
** Description     Configure HCI device type to use
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int hw_set_type(char *p_conf_name, char *p_conf_value, int param)
{
    strcpy(cfg.type, p_conf_value);
    return 0;
} 

/*******************************************************************************
**
** Function        hw_set_psrfile
**
** Description     Configure HCI PSR config file
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int hw_set_psrfile(char *p_conf_name, char *p_conf_value, int param)
{
    strcpy(cfg.psrfile, p_conf_value);
    return 0;
} 

/*******************************************************************************
**
** Function        hw_set_init_speed
**
** Description     Configure UART initial speed
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int hw_set_init_speed(char *p_conf_name, char *p_conf_value, int param)
{
    cfg.init_speed = atoi(p_conf_value);
    return 0;
} 

/*******************************************************************************
**
** Function        hw_set_speed
**
** Description     Configure UART working speed
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int hw_set_speed(char *p_conf_name, char *p_conf_value, int param)
{
    cfg.speed = atoi(p_conf_value);
    return 0;
} 

/*******************************************************************************
**
** Function        hw_set_flow_ctl
**
** Description     Configure UART flow control
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int hw_set_flow_ctl(char *p_conf_name, char *p_conf_value, int param)
{
	if (cfg.flags == -1)
		cfg.flags = 0;
	cfg.flags &= ~FLOW_CTL;
	if (atoi(p_conf_value)) 
		cfg.flags |= FLOW_CTL;
    return 0;
} 

/*******************************************************************************
**
** Function        hw_set_parity
**
** Description     Configure UART parity
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int hw_set_parity(char *p_conf_name, char *p_conf_value, int param)
{
	int parity = atoi(p_conf_value);
	if (cfg.flags == -1)
		cfg.flags = 0;
	cfg.flags &= ~(EVEN_PARITY | ODD_PARITY);
	switch (parity) {
		case 1:
			cfg.flags |= EVEN_PARITY;
			break;
		case 2:
			cfg.flags |= ODD_PARITY;
			break;
	}
	return 0;
} 

/*******************************************************************************
**
** Function        hw_set_send_brk
**
** Description     Configure UART to send a break before init
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int hw_set_send_brk(char *p_conf_name, char *p_conf_value, int param)
{
	cfg.send_break = atoi(p_conf_value);
    return 0;
} 

/*******************************************************************************
**
** Function        hw_set_raw
**
** Description     Configure UART to init as raw
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int hw_set_raw(char *p_conf_name, char *p_conf_value, int param)
{
	cfg.raw = atoi(p_conf_value);
    return 0;
} 


/* Initialize UART driver */
static int init_uart(const struct uart_t* u)
{
	int i, ld;
	unsigned long flags = 0;

	if (cfg.raw)
		flags |= 1 << HCI_UART_RAW_DEVICE;

    /* reset device to known state */   
    ALOGD("Resetting...");
    bt_reset();   

	ALOGD("Opening %s @ %d,%c,%sflow", cfg.dev, cfg.init_speed, (cfg.flags & EVEN_PARITY) ? 'E': 'N', (cfg.flags & FLOW_CTL) ? "" : "no" );

	if (hci_open(cfg.init_speed, u->proto == HCI_UART_H4 ? omH4 : omBCSP) < 0) {
	
		if (cfg.init_speed == cfg.speed)
			return -1;
			
		// Just in case, retry with the destination speed
		if (hci_open(cfg.speed, u->proto == HCI_UART_H4 ? omH4 : omBCSP) < 0) {
			return -1;
		}
	}
	
	if (u->init && u->init() < 0) {
		hci_close();
		return -1;
	}

#if 0
	/* Set actual baudrate */
	if (hci_set_speed(cfg.speed) < 0) {
		ALOGE("Can't set working baud rate");
		hci_close();
		return -1;
	}

	/* Set TTY to N_HCI line discipline */
	i = N_HCI;
	if (ioctl(cfg.ufd, TIOCSETD, &i) < 0) {
		ALOGE("Can't set line discipline");
		hci_close();
		return -1;
	}

	if (flags && ioctl(cfg.ufd, HCIUARTSETFLAGS, flags) < 0) {
		ALOGE("Can't set UART flags");
		hci_close();
		return -1;
	}

	if (ioctl(cfg.ufd, HCIUARTSETPROTO, u->proto) < 0) {
		ALOGE("Can't set device");
		hci_close();
		return -1;
	}
#endif

	return cfg.ufd;
}   

int hw_config(void)
{
	// Get the UART type
	const struct uart_t* uart = get_by_type(cfg.type);
	if (!uart) {
		ALOGE("Undefined Bluetooth type '%s'", cfg.type);
		return -1;
	}
	
	// If no need to do init, don't do it
	if (uart->proto == HCI_UART_NONE)
		return 0;

	// Override values not specifically set by user
	if (cfg.init_speed == -1)
		cfg.init_speed = uart->init_speed;
	if (cfg.speed == -1)
		cfg.speed = uart->speed;
	if (cfg.flags == -1)
		cfg.flags = uart->flags;

	// Initialize the UART and switch it to HCI
	return init_uart(uart);
}

static uint8_t hw_config_set_bdaddr(HC_BT_HDR *p_buf)
{
    uint8_t retval = FALSE;
    uint8_t *p = (uint8_t *) (p_buf + 1);

    ALOGI("Setting local bd addr to %02X:%02X:%02X:%02X:%02X:%02X",
        vnd_local_bd_addr[0], vnd_local_bd_addr[1], vnd_local_bd_addr[2],
        vnd_local_bd_addr[3], vnd_local_bd_addr[4], vnd_local_bd_addr[5]);

    UINT16_TO_STREAM(p, HCI_VSC_WRITE_BD_ADDR);
    *p++ = BD_ADDR_LEN; /* parameter length */
    *p++ = vnd_local_bd_addr[5];
    *p++ = vnd_local_bd_addr[4];
    *p++ = vnd_local_bd_addr[3];
    *p++ = vnd_local_bd_addr[2];
    *p++ = vnd_local_bd_addr[1];
    *p = vnd_local_bd_addr[0];

    p_buf->len = HCI_CMD_PREAMBLE_SIZE + BD_ADDR_LEN;
    hw_cfg_cb.state = HW_CFG_RBS;

    retval = bt_vendor_cbacks->xmit_cb(HCI_VSC_WRITE_BD_ADDR, p_buf, \
                                 hw_config_cback);

    return (retval);
}

void hw_config_cback(void *p_mem)
{
    HC_BT_HDR *p_evt_buf = (HC_BT_HDR *) p_mem;
    char        *p_name, *p_tmp;
    uint8_t     *p, status;
    uint16_t    opcode;
    HC_BT_HDR  *p_buf=NULL;
    uint8_t     is_proceeding = FALSE;
    int         i;
#if (USE_CONTROLLER_BDADDR == TRUE)
    const uint8_t null_bdaddr[BD_ADDR_LEN] = {0,0,0,0,0,0};
#endif

    status = *((uint8_t *)(p_evt_buf + 1) + HCI_EVT_CMD_CMPL_STATUS_RET_BYTE);
    p = (uint8_t *)(p_evt_buf + 1) + HCI_EVT_CMD_CMPL_OPCODE;
    STREAM_TO_UINT16(opcode,p);

    /* Ask a new buffer big enough to hold any HCI commands sent in here */
    if ((status == 0) && bt_vendor_cbacks)
        p_buf = (HC_BT_HDR *) bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + \
                                                       HCI_CMD_MAX_LEN);

    ALOGE("%s: state=%u\n", __func__, hw_cfg_cb.state);

    if (p_buf != NULL)
    {
        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
        p_buf->offset = 0;
        p_buf->len = 0;
        p_buf->layer_specific = 0;

        p = (uint8_t *) (p_buf + 1);

        switch (hw_cfg_cb.state)
        {
			// case HW_CFG_START:
			case 98:
				is_proceeding = hw_config_set_bdaddr(p_buf);
				break;
			// case HW_CFG_RBS:
			case 99:
			case HW_CFG_START:
                /* read local name */
                UINT16_TO_STREAM(p, HCI_READ_BUFFER_SIZE);
                *p = 0; /* parameter length */

                p_buf->len = HCI_CMD_PREAMBLE_SIZE;
                hw_cfg_cb.state = HW_CFG_END;

                is_proceeding = bt_vendor_cbacks->xmit_cb(HCI_READ_BUFFER_SIZE, \
                                                    p_buf, hw_config_cback);
				break;
			case HW_CFG_RBS:
            case HW_CFG_END:
                ALOGI("vendor lib fwcfg completed");
                bt_vendor_cbacks->dealloc(p_buf);
                bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_SUCCESS);

                hw_cfg_cb.state = 0;

                is_proceeding = TRUE;
                break;

        } // switch(hw_cfg_cb.state)
    } // if (p_buf != NULL)

    /* Free the RX event buffer */
    if (bt_vendor_cbacks)
        bt_vendor_cbacks->dealloc(p_evt_buf);

    if (is_proceeding == FALSE)
    {
        ALOGE("vendor lib fwcfg aborted!!!");
        if (bt_vendor_cbacks)
        {
            if (p_buf != NULL)
                bt_vendor_cbacks->dealloc(p_buf);

            bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_FAIL);
        }

        hw_cfg_cb.state = 0;
    }
}

void hw_config_start(void)
{
    HC_BT_HDR  *p_buf = NULL;
    uint8_t     *p;

    hw_cfg_cb.state = 0;

    /* Start from sending HCI_RESET */

    if (bt_vendor_cbacks)
    {
        p_buf = (HC_BT_HDR *) bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + \
                                                       HCI_CMD_PREAMBLE_SIZE);
    }

    if (p_buf)
    {
        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
        p_buf->offset = 0;
        p_buf->layer_specific = 0;
        p_buf->len = HCI_CMD_PREAMBLE_SIZE;

        p = (uint8_t *) (p_buf + 1);
        // UINT16_TO_STREAM(p, HCI_RESET);
        UINT16_TO_STREAM(p, HCI_READ_BUFFER_SIZE);
        *p = 0; /* parameter length */

        hw_cfg_cb.state = HW_CFG_END;

        bt_vendor_cbacks->xmit_cb(HCI_RESET, p_buf, hw_config_cback);
    }
    else
    {
        if (bt_vendor_cbacks)
        {
            ALOGE("vendor lib fw conf aborted [no buffer]");
            bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_FAIL);
        }
    }
}
