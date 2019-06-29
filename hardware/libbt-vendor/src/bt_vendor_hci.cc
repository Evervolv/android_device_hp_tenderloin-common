/******************************************************************************
 *
 *  Copyright (C) 2009-2012 Broadcom Corporation
 *  Copyright (C) 2012-2013 Eduardo Jos� Tagle <ejtagle@hotmail.com>
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
 *  Filename:      bt_vendor_hci.c
 *
 *  Description:   Netlink vendor specific library implementation
 *
 ******************************************************************************/

#define LOG_TAG "bt_vendor"

#include <utils/Log.h>
#include "bt_vendor_hci.h"
#include "upio.h"
#include "userial_vendor.h"
#include "hardware.h"
#include <string.h>

#ifndef BTVND_DBG
#define BTVND_DBG FALSE
#endif

#if (BTVND_DBG == TRUE)
#define BTVNDDBG(param, ...) {ALOGD(param, ## __VA_ARGS__);}
#else
#define BTVNDDBG(param, ...) {}
#endif

/******************************************************************************
**  Externs
******************************************************************************/

void hw_config_start(void);
void vnd_load_conf(const char *p_path);

/*
 *  Variables
 */

bt_vendor_callbacks_t *bt_vendor_cbacks = NULL;
uint8_t vnd_local_bd_addr[6]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00};


/******************************************************************************
**  Functions
******************************************************************************/

/*****************************************************************************
**
**   BLUETOOTH VENDOR INTERFACE LIBRARY FUNCTIONS
**
*****************************************************************************/

static int init(const bt_vendor_callbacks_t* p_cb, unsigned char *local_bdaddr)
{
    ALOGI("init");

    if (p_cb == NULL) {
        ALOGE("init failed with no user callbacks!");
        return -1;
    }

    userial_vendor_init();
    upio_init();
    hw_init();

    vnd_load_conf(VENDOR_LIB_CONF_FILE);

    /* store reference to user callbacks */
    bt_vendor_cbacks = (bt_vendor_callbacks_t *) p_cb;

    /* This is handed over from the stack */
    memcpy(vnd_local_bd_addr, local_bdaddr, 6);

    return 0;
}


/** Requested operations */
static int op(bt_vendor_opcode_t opcode, void *param)
{
    int retval = 0;

    switch(opcode) {
        case BT_VND_OP_POWER_CTRL:
            {
                int *state = (int *) param;
                if (*state == BT_VND_PWR_OFF) {
                    BTVNDDBG("BT_VND_OP_POWER_CTRL OFF");
                    upio_set_bluetooth_power(UPIO_BT_POWER_OFF);
                }
                else if (*state == BT_VND_PWR_ON) {
                    BTVNDDBG("BT_VND_OP_POWER_CTRL ON");
                    upio_set_bluetooth_power(UPIO_BT_POWER_ON);
                } else {
                    BTVNDDBG("BT_VND_OP_POWER_CTRL unknown=%d", *state);
                }
            }
            break;

        case BT_VND_OP_FW_CFG:
#if 1
            if (bt_vendor_cbacks)
            {
                BTVNDDBG("BT_VND_OP_FW_CFG");
                bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_SUCCESS);
            } else {
                BTVNDDBG("BT_VND_OP_FW_CFG no-callbacks");
            }
#else
            BTVNDDBG("BT_VND_OP_FW_CFG");
			hw_config_start();
#endif
            break;

        case BT_VND_OP_SCO_CFG:
            if (bt_vendor_cbacks)
            {
                BTVNDDBG("BT_VND_OP_SCO_CFG");
                bt_vendor_cbacks->scocfg_cb(BT_VND_OP_RESULT_SUCCESS);
            } else {
                BTVNDDBG("BT_VND_OP_SCO_CFG no-callbacks");
            }
            break;

        case BT_VND_OP_USERIAL_OPEN:
            {
                int (*fd_array)[] = (int (*)[]) param;
                int fd, idx;

                BTVNDDBG("BT_VND_OP_USERIAL_OPEN");
                // Configure the hw and setup the HCI channel
                fd = hw_config();
                if (fd < 0) {
                    ALOGE("BT_VND_OP_USERIAL_OPEN hw_config failed");
                    return FALSE;
                }

                // Open the HCI channel
                // fd = userial_vendor_open();
                if (fd != -1)
                {
                    for (idx=0; idx < CH_MAX; idx++)
                        (*fd_array)[idx] = fd;

                    retval = 1;
                } else {
                    BTVNDDBG("BT_VND_OP_USERIAL_OPEN userial_vendor_open fd=-1");
                }
                /* retval contains numbers of open fd of HCI channels */
            }
            break;

        case BT_VND_OP_USERIAL_CLOSE:
            {
                BTVNDDBG("BT_VND_OP_USERIAL_CLOSE");
                userial_vendor_close();
                hw_close();
            }
            break;

        case BT_VND_OP_GET_LPM_IDLE_TIMEOUT:
            {
                BTVNDDBG("BT_VND_OP_GET_LPM_IDLE_TIMEOUT");
                uint32_t *timeout_ms = (uint32_t *) param;
                *timeout_ms = 250;
            }
            break;

        case BT_VND_OP_LPM_SET_MODE:
            {
                if (bt_vendor_cbacks) {
                    BTVNDDBG("BT_VND_OP_LPM_SET_MODE");
                    bt_vendor_cbacks->lpm_cb(BT_VND_OP_RESULT_SUCCESS);
                } else {
                    BTVNDDBG("BT_VND_OP_LPM_SET_MODE no-callbacks");
                }
            }
            break;

        case BT_VND_OP_LPM_WAKE_SET_STATE:
            // BTVNDDBG("BT_VND_OP_LPM_WAKE_SET_STATE");
            break;
        default:
            BTVNDDBG("UNSUPPORED OP=%d", opcode);
            break;
    }

    return retval;
}

/** Closes the interface */
static void cleanup( void )
{
    BTVNDDBG("cleanup");

    upio_cleanup();

    bt_vendor_cbacks = NULL;
}

// Entry point of DLib
const bt_vendor_interface_t BLUETOOTH_VENDOR_LIB_INTERFACE = {
    sizeof(bt_vendor_interface_t),
    init,
    op,
    cleanup
};

