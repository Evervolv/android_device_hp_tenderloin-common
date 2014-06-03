/******************************************************************************
 *
 *  Copyright (C) 2009-2012 Broadcom Corporation
 *  Copyright (C) 2012-13 Eduardo José Tagle <ejtagle@hotmail.com>
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
 *  Filename:      userial_vendor.c
 *
 *  Description:   Contains vendor-specific userial functions
 *                 In our case, we route HCI through the linux kernel
 *                 netlink layer.
 *
 ******************************************************************************/

#define LOG_TAG "bt_userial_vendor"

#include <utils/Log.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include "bt_vendor_hci.h"
#include "userial.h"
#include "userial_vendor.h"
#include "hardware.h"
#include "bluetooth.h"
#include "hci.h"

/******************************************************************************
**  Constants & Macros
******************************************************************************/

#ifndef VNDUSERIAL_DBG
#define VNDUSERIAL_DBG FALSE
#endif

#if (VNDUSERIAL_DBG == TRUE)
#define VNDUSERIALDBG(param, ...) {ALOGD(param, ## __VA_ARGS__);}
#else
#define VNDUSERIALDBG(param, ...) {}
#endif

/******************************************************************************
**  Local type definitions
******************************************************************************/

/* vendor serial control block */
typedef struct
{
    int fd;                     /* fd to Bluetooth netlink device */
    int dev_id;
} vnd_netlink_cb_t;

/******************************************************************************
**  Static variables
******************************************************************************/

static vnd_netlink_cb_t vnd_netlink;

/*****************************************************************************
**   Userial Vendor API Functions
*****************************************************************************/

/*******************************************************************************
**
** Function        userial_vendor_init
**
** Description     Initialize userial vendor-specific control block
**
** Returns         None
**
*******************************************************************************/
void userial_vendor_init(void)
{
    vnd_netlink.fd    = -1;
    vnd_netlink.dev_id = 0;
}

/*******************************************************************************
**
** Function        userial_vendor_open
**
** Description     Open the serial port with the given configuration
**
** Returns         device fd
**
*******************************************************************************/
int userial_vendor_open(void)
{
    struct sockaddr_hci a;
    struct hci_filter flt;
    vnd_netlink.fd = -1;

    ALOGI("userial vendor open: opening hci%d", vnd_netlink.dev_id);

    /* Create HCI socket */
    vnd_netlink.fd = socket(AF_BLUETOOTH, SOCK_RAW, BTPROTO_HCI);
    if (vnd_netlink.fd < 0) {
        ALOGE("userial vendor open: Unable to open NETLINK socket");
        return -1;
    }

    // Setup no filter
    memset(&flt, 0, sizeof(flt));
    memset((void *) &flt.type_mask, 0xff, sizeof(flt.type_mask));
    memset((void *) &flt.event_mask, 0xff, sizeof(flt.event_mask));
    if (setsockopt(vnd_netlink.fd, SOL_HCI, HCI_FILTER, &flt, sizeof(flt)) < 0) {
        ALOGE("userial vendor open: HCI filter setup failed");
        close(vnd_netlink.fd);
        return -1;
    }

    // Bind socket to the HCI device */
    memset(&a, 0, sizeof(a));
    a.hci_family = AF_BLUETOOTH;
    a.hci_dev = vnd_netlink.dev_id;
    a.hci_channel = HCI_CHANNEL_RAW;

    if (bind(vnd_netlink.fd, (struct sockaddr *) &a, sizeof(a)) < 0) {
        ALOGE("userial vendor open: Unable to bind to open HCI%d device",vnd_netlink.dev_id);
        close(vnd_netlink.fd);
        return -1;
    }

    /* Set the socket to RAW */
    if (ioctl(vnd_netlink.fd, HCISETRAW, 1) < 0) {
        ALOGE("Can't access device");
        return -1;
    }

    /* bring HCI device up */
    if (ioctl(vnd_netlink.fd, HCIDEVUP, vnd_netlink.dev_id) < 0) {
        ALOGE("Can't bring up device errno=%d", errno);
        return -1;
    }

    ALOGI("device fd = %d open", vnd_netlink.fd);

    return vnd_netlink.fd;
}

/*******************************************************************************
**
** Function        userial_vendor_close
**
** Description     Conduct vendor-specific close work
**
** Returns         None
**
*******************************************************************************/
void userial_vendor_close(void)
{
    int result;

    if (vnd_netlink.fd == -1)
        return;

    ALOGI("device fd = %d close", vnd_netlink.fd);

    if ((result = close(vnd_netlink.fd)) < 0)
        ALOGE( "close(fd:%d) FAILED result:%d", vnd_netlink.fd, result);

    vnd_netlink.fd = -1;
}


/*******************************************************************************
**
** Function        userial_set_dev
**
** Description     Configure HCI device to use
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int userial_set_dev(char *p_conf_name, char *p_conf_value, int param)
{
    vnd_netlink.dev_id = atoi(p_conf_value);
    return hw_set_devid(p_conf_name, p_conf_value, param);
}
