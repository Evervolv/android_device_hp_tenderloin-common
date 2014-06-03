/******************************************************************************
 *
 *  Copyright (C) 2009-2012 Broadcom Corporation
 *  Copyright (C) 2012-2013 Eduardo José Tagle <ejtagle@hotmail.com>
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
 *  Filename:      hardware.h
 *
 *  Description:   Contains definitions used hw initialization
 *
 ******************************************************************************/

#ifndef HARDWARE_H
#define HARDWARE_H


void hw_init(void);
void hw_close(void);
int hw_config(void);

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
int hw_set_devid(char *p_conf_name, char *p_conf_value, int param);


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
int hw_set_device(char *p_conf_name, char *p_conf_value, int param);

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
int hw_set_type(char *p_conf_name, char *p_conf_value, int param);

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
int hw_set_psrfile(char *p_conf_name, char *p_conf_value, int param);

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
int hw_set_init_speed(char *p_conf_name, char *p_conf_value, int param);

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
int hw_set_speed(char *p_conf_name, char *p_conf_value, int param);

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
int hw_set_flow_ctl(char *p_conf_name, char *p_conf_value, int param);

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
int hw_set_parity(char *p_conf_name, char *p_conf_value, int param);

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
int hw_set_send_brk(char *p_conf_name, char *p_conf_value, int param);

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
int hw_set_raw(char *p_conf_name, char *p_conf_value, int param);


#endif 