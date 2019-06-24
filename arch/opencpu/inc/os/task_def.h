/* Copyright Statement:
 *
 * (C) 2005-2016  MediaTek Inc. All rights reserved.
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. ("MediaTek") and/or its licensors.
 * Without the prior written permission of MediaTek and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 * You may only use, reproduce, modify, or distribute (as applicable) MediaTek Software
 * if you have agreed to and been bound by the applicable license agreement with
 * MediaTek ("License Agreement") and been granted explicit permission to do so within
 * the License Agreement ("Permitted User").  If you are not a Permitted User,
 * please cease any access or use of MediaTek Software immediately.
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT MEDIATEK SOFTWARE RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES
 * ARE PROVIDED TO RECEIVER ON AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL
 * WARRANTIES, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 */

#ifndef __TASK_DEF_H__
#define __TASK_DEF_H__
#include "FreeRTOSConfig.h"

typedef enum {
    TASK_PRIORITY_IDLE = 0,                                 /* lowest, special for idle task */
    TASK_PRIORITY_SYSLOG,                                   /* special for syslog task */

    /* User task priority begin, please define your task priority at this interval */
    TASK_PRIORITY_LOW,                                      /* low */
    TASK_PRIORITY_BELOW_NORMAL,                             /* below normal */
    TASK_PRIORITY_NORMAL,                                   /* normal */
    TASK_PRIORITY_ABOVE_NORMAL,                             /* above normal */
    TASK_PRIORITY_HIGH,                                     /* high */
    TASK_PRIORITY_SOFT_REALTIME,                            /* soft real time */
    TASK_PRIORITY_HARD_REALTIME,                            /* hard real time */
    /* User task priority end */


    /* Don't change below tasks' priority !!!*/
    TASK_PRIORITY_HISR_TASK = (configMAX_PRIORITIES - 2 - 24)- 2,/* AP HISR task priority */
    TASK_PRIORITY_OS_TIMER  = (configMAX_PRIORITIES - 2 - 24)- 1,/* AP OS timer task priority */
    TASK_PRIORITY_MD_BEGIN  = (configMAX_PRIORITIES - 2 - 24),   /* MD tasks priority begin, reserve 25 priorities for MD internal usage */ 
    TASK_PRIORITY_MD_END    = (configMAX_PRIORITIES - 2),        /* MD tasks priority end */
    TASK_PRIORITY_GKI_TIMER = (configMAX_PRIORITIES - 1),        /* highest, MD GKI timer priority */
} task_priority_type_t;

/* part_1: SDK tasks configure infomation, please don't modify */

/*The following is an example to define the XXXX task.
please be careful  to the stack size, in freertos the unit of the stack size is portSTACK_TYPE, which is uint_32 type and is equals to 4 bytes.
#define XXXX_TASK_NAME "XXXX"
#define XXXX_TASK_STACKSIZE 512
#define XXXX_TASK_PRIO TASK_PRIORITY_LOW
#define XXXX_QUEUE_LENGTH  16
*/

/* syslog task definition */
#define SYSLOG_TASK_NAME "SYSLOG"
#define TASK_PRIORITY_SYSLOG 1
/*This definition determines whether the port service feature is enabled. If it is not defined, then the port service feature is not supported.*/
#if defined(MTK_PORT_SERVICE_ENABLE)
#define SYSLOG_TASK_STACKSIZE 1024
#else
#define SYSLOG_TASK_STACKSIZE 192
#endif
#define SYSLOG_TASK_PRIO TASK_PRIORITY_SYSLOG
/*MTK_NO_PSRAM_ENABLE: the definition of MTK_NO_PSRAM_ENABLE determines whether the demo uses PSRAM. If it is defined, means no PSRAM in this project.*/
#if (PRODUCT_VERSION == 7687) || (PRODUCT_VERSION == 7697) || defined(MTK_NO_PSRAM_ENABLE)
#define SYSLOG_QUEUE_LENGTH 8
#elif (PRODUCT_VERSION == 2523) || (PRODUCT_VERSION == 2625)
#define SYSLOG_QUEUE_LENGTH 512
#endif

/* for lwIP task */
#define TCPIP_THREAD_NAME              "lwIP"
#define TCPIP_THREAD_STACKSIZE         (512 * 4)
#define TCPIP_THREAD_PRIO              TASK_PRIORITY_HIGH

/* for nbnetif task */
#define NB_NETIF_TASK_NAME             "nbif"
#define NB_NETIF_TASK_STACKSIZE        (1024 * 4)
#define NB_NETIF_TASK_PRIO             TASK_PRIORITY_HIGH

/* for iperf task */
#define IPERF_TASK_NAME                "iperf"
#define IPERF_TASK_STACKSIZE           (1200 * 4)
#define IPERF_TASK_PRIO                TASK_PRIORITY_NORMAL

/* for ping task */
#define PING_TASK_NAME                 "ping"
#define PING_TASK_STACKSIZE            (512 * 4)
#define PING_TASK_PRIO                 TASK_PRIORITY_NORMAL

/* for nbnetif task */
#define APB_NW_APP_TASK_NAME           "apb_nw"
#define APB_NW_APP_TASK_STACKSIZE      (1024 * 2)
#define APB_NW_APP_TASK_PRIO           TASK_PRIORITY_NORMAL

/* for cis onenet client task */
#define APB_PROXY_ONENET_EXAMPLE_TASK_NAME           "apb_onenet_client"
#define APB_PROXY_ONENET_EXAMPLE_STACKSIZE      (1024 * 2)
#define APB_PROXY_ONENET_EXAMPLE_PRIO           TASK_PRIORITY_NORMAL

/* for tls at cmd task */
#define APB_PROXY_TLS_TASK_NAME           "apb_tls"
#define APB_PROXY_TLS_TASK_STACKSIZE      (2048 * 4)
#define APB_PROXY_TLS_TASK_PRIO           TASK_PRIORITY_NORMAL

#ifdef MTK_USB_DEMO_ENABLED
/* USB */
#define USB_TASK_NAME                   "USB"
#define USB_TASK_STACKSIZE              (1024 * 4)
#undef USB_TASK_PRIO
#define USB_TASK_PRIO                   TASK_PRIORITY_HIGH
#define USB_QUEUE_LENGTH                500
#endif

/* ATCI task definition */
#define ATCI_TASK_NAME              "ATCI"
#ifdef MTK_AUDIO_TUNING_ENABLED
#define ATCI_TASK_STACKSIZE         (5000*4) /*unit byte!*/
#else
#define ATCI_TASK_STACKSIZE         (1024*4) /*unit byte!*/
#endif
#define ATCI_TASK_PRIO              TASK_PRIORITY_NORMAL

/* tel_conn_mgr task */
#define TEL_CONN_MGR_TASK_NAME              ("tel_conn_mgr")
#define TEL_CONN_MGR_TASK_STACKSIZE         (4 * 1024)
#define TEL_CONN_MGR_TASK_PRIO              TASK_PRIORITY_NORMAL

/* for os utilization task */
/*MTK_OS_CPU_UTILIZATION_ENABLE: This definition determines whether enable CPU utilization profiling feature, if enabled, can use AT command on MT25x3 or CLI command on MT76x7 to do CPU utilization profiling.*/
#if defined(MTK_OS_CPU_UTILIZATION_ENABLE)
#define MTK_OS_CPU_UTILIZATION_TASK_NAME "CPU"
#define MTK_OS_CPU_UTILIZATION_STACKSIZE 1024 /*unit byte*/
#define MTK_OS_CPU_UTILIZATION_PRIO      TASK_PRIORITY_SOFT_REALTIME
#endif

/* part_2: Application and customer tasks configure information */
/* currently, only UI task and tasks to show example project function which define in apps/project/src/main.c */

/* for Application task */
#define FREERTOS_EXAMPLE_TASK_NAME       "freertos_example"
#define FREERTOS_EXAMPLE_TASK_STACKSIZE  2048 /*unit byte!*/
#define FREERTOS_EXAMPLE_TASK_PRIO       TASK_PRIORITY_NORMAL

/* atci keypad task*/
#define ATCI_KEYPAD_TASK_NAME "KEYPAD_atci"
#define ATCI_KEYPAD_TASK_STACKSIZE (512*4) /* unit byte!*/
#define ATCI_KEYPAD_TASK_PRIO TASK_PRIORITY_NORMAL
#define ATCI_KEYPAD_QUEUE_LENGTH  100

/* ATCI MSDC task definition */
#define ATCI_MSDC_TASK_NAME              "MSDC_atci"
#define ATCI_MSDC_TASK_STACKSIZE         (200*4) /*unit byte!*/
#define ATCI_MSDC_TASK_PRIO              TASK_PRIORITY_NORMAL

/* FOTA task definition */
#define FOTA_TASK_NAME             "FOTA_TASK"
#define FOTA_TASK_STACKSIZE        (1024 * 4) /*unit is bytes.*/
#define FOTA_TASK_PRIORITY         TASK_PRIORITY_NORMAL
#define FOTA_QUEUE_LENGTH          10

#define GNSS_BRIDGE_TASK_NAME "GNSS_BRIDGE"
#define GNSS_BRIDGE_TASK_STACK_SIZE 1500
#define GNSS_BRIDGE_TASK_PRIO TASK_PRIORITY_NORMAL

/* gnss task definition */
#define GNSS_DEMO_TASK_NAME "gnss_t"
#define GNSS_DEMO_TASK_STACK_SIZE 8000
#define GNSS_DEMO_TASK_PRIO TASK_PRIORITY_HIGH

#define GNSS_BRIDGE_TASK_NAME "GNSS_BRIDGE"
#define GNSS_BRIDGE_TASK_STACK_SIZE 1500
#define GNSS_BRIDGE_TASK_PRIO TASK_PRIORITY_NORMAL

#ifdef __RS_FOTA_SUPPORT__
/* redstone fota task definition */
#define RS_FOTA_TASK_NAME             "REDSTONE_FOTA_TASK"
#define RS_FOTA_TASK_STACKSIZE        (1024 * 4) /*unit is bytes.*/
#define RS_FOTA_TASK_PRIORITY         TASK_PRIORITY_NORMAL
#define RS_FOTA_QUEUE_LENGTH          15
#endif

#endif
