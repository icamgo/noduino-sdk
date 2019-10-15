/* Copyright Statement:
 *
 * (C) 2005-2017 MediaTek Inc. All rights reserved.
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
 * the License Agreement ("Permitted User"). If you are not a Permitted User,
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

#ifndef __RIL_H__
#define __RIL_H__
#include "ril_general_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* Notice: to enable SMS related AT commands (3GPP spec v27.005), please set MTK_RIL_SMS_COMMAND_SUPPORT as y in project feature make file. */


/** @addtogroup RIL
 * @{
 * This section introduces the Radio Interface Layer (RIL) interface APIs including terms and acronyms, supported features, software architecture, 
 * details on how to use this interface, RIL functions, enumerations and structures.
 * RIL is a middleware module that enables communication with Narrow Band Internet of Ting (NB-IoT) modem. In general, RIL is usullay  
 * responsible to manage and share CMUX channels to support multiple users to access modem simultaneously, construct AT 
 * command string by request, send command, receive command response, parse response string, report result codes and 
 * included subparameters to user, receive unsolicited result codes (URC) and broadcast to users waiting for the event.
 * RIL interfaces depend on the NB-IoT AT command set, each AT command almost has a corresponding RIL API to send. Please refer to 
 * MediaTek MT2625_NB-IoT AT Command Specification document for more information about each AT command.
 *
 *
 * @section Terms_Chapter Terms and acronyms
 *
 * |Terms                   |Details                                                                 |
 * |------------------------------|------------------------------------------------------------------------| 
 * |\b AT                       | Abbreivation of ATtention, every command line starts with AT. |
 * |\b ME                       | Mobile Equipment | 
 * |\b MS                       | Mobile Station |
 * |\b MT                       | Mobile Termination |
 * |\b NAS                      | Non-Access-Stratum protocol |
 * |\b MUX                      | MUX (Multiplexer) represents AP-MD adaption layer over CSCI, not 27.101 protocol. |
 * |\b RIL                      | Radio Interface Layer |
 * |\b TA                       | Terminal Adaptor, e.g. a GSM data card equal to DCE Data Circuit terminating Equipement | 
 * |\b TE                       | Terminal Equipment, e.g. a computer. |
 *
 *
 * @section RIL_Architechture_Chapter Software Architecture of RIL
 * This section shows the software architecture of RIL on mt2625 platform.
 * @image html mt2625_ril_arch.png
 */
  
/**
 * @defgroup ril_macro Macro
 * @{
 * RIL related macro definitions.
 */

#define RIL_OMITTED_INTEGER_PARAM    (0x80000000)    /**< Pre-defined, if an integer parameter is omitted. */
#define RIL_OMITTED_STRING_PARAM    (NULL)    /**< Pre-defined, if a string parameter is omitted. */


/* Group mask definitions. */
#define RIL_GROUP_MASK_NONE                    (0x00000000)    /**< A command doesn't belong to any function group. */
#define RIL_GROUP_MASK_POWER_MANAGEMENT        (0x00000001)    /**< Power Management function group. */
#define RIL_GROUP_MASK_MOBILITY                (0x00000002)    /**< Mobility function group. */
#define RIL_GROUP_MASK_SMS                     (0x00000004)    /**< Short message function group. */
#define RIL_GROUP_MASK_PHONEBOOK               (0x00000008)    /**< Phonebook function group. */
#define RIL_GROUP_MASK_SIM                     (0x00000010)    /**< SIM function group. */
#define RIL_GROUP_MASK_PACKET_DATA             (0x00000020)    /**< Packet Data function group. */
#define RIL_GROUP_MASK_ENGINEERING_MODE        (0x00000040)    /**< Engineering Mode function group. */
#define RIL_GROUP_MASK_SIM_TOOLKIT             (0x00000080)    /**< SIM Toolkit function group. */
#define RIL_GROUP_MASK_MUX                     (0x00000100)    /**< MUX function group. */
#define RIL_GROUP_MASK_ALL                     (0xFFFFFFFF)    /**< All function groups included. */

/**
 * @}
 */


/** @defgroup RIL_enum Enum
 * @{
 * RIL related enumuration definitions.
 */

/** @brief This enum defines the AT command mode based on the specification. */
typedef enum {
    RIL_EXECUTE_MODE,    /**< AT command execute or set mode, formatted as AT+XXX=<param1>,<param2>,... */
    RIL_ACTIVE_MODE,     /**< AT command active mode, formatted as AT+XXX */
    RIL_READ_MODE,       /**< AT command read mode, formatted as AT+XXX? */
    RIL_TEST_MODE        /**< AT command test mode, formattedas AT+XXX=? */
} ril_request_mode_t;


/** @brief This enum defines return value of the RIL SDK APIs. */
typedef enum {
    RIL_STATUS_INVALID_PARAM = -2,    /**< Invalid input parameter. */
    RIL_STATUS_FAIL = -1,             /**< The operation has failed. */
    RIL_STATUS_SUCCESS = 0            /**< The operation completed successfully. */
} ril_status_t;


/** @brief This enum defines the result code for the AT command response. */
typedef enum {
    /* RIL module's internal result codes. */
    RIL_RESULT_CODE_UNDEFINED = -4,    /**< Undefined error. */
    RIL_RESULT_CODE_RESPONSE_FORMAT_ERROR = -3,    /**< The response format doesn't match with expectation. */
    RIL_RESULT_CODE_RESPONSE_TIMEOUT = -2,    /**< The AT response didn't arrive on time. */
    RIL_RESULT_CODE_MUX_SEND_ERROR = -1,    /**< Failed to send an AT command by MUX. */

    /* (CME) error codes. */

    START_OF_CME_ERROR_CODES = 0,    /**< Start index. */

    /* specification - General errors. */

    RIL_CME_PHONE_FAILURE = START_OF_CME_ERROR_CODES,    /**< Phone failure. */
    RIL_CME_NO_MS_CONNECTION,    /**< No connection to the phone. */
    RIL_CME_PHONE_LINK_RESERVED,    /**< Phone-adaptor link is reserved. */
    RIL_CME_OPERATION_NOT_ALLOWED,    /**< The operation is not allowed. */
    RIL_CME_OPERATION_NOT_SUPPORTED,    /**< The operation is not supported. */
    RIL_CME_PH_SIM_PIN_REQUIRED,    /**< PH-SIM pin is required. */
    RIL_CME_FSIM_PIN_REQUIRED,    /**< PH-FSIM pin is required. */
    RIL_CME_FSIM_PUK_REQUIRED,    /**< PH-FSIM PUK is required. */
    RIL_CME_SIM_NOT_INSERTED = 10,    /**< The SIM is not inserted. */
    RIL_CME_SIM_PIN_REQUIRED,    /**< The SIM pin is required. */
    RIL_CME_SIM_PUK_REQUIRED,    /**< The SIM PUK is required. */
    RIL_CME_SIM_FAILURE,    /**< The SIM has failed. */
    RIL_CME_SIM_BUSY,    /**< The SIM is busy. */
    RIL_CME_SIM_WRONG,    /**< The SIM is wrong. */
    RIL_CME_INCORRECT_PASSWORD,    /**< Incorrect password. */
    RIL_CME_SIM_PIN2_REQUIRED,    /**< SIM PIN2 is required. */
    RIL_CME_SIM_PUK2_REQUIRED,    /**< SIM PUK2 is required. */
    RIL_CME_MEMORY_FULL = 20,    /**< Memory is full. */
    RIL_CME_INVALID_INDEX,    /**< Invalid index. */
    RIL_CME_NOT_FOUND,    /**< Not found. */
    RIL_CME_MEMORY_FAILURE,    /**< Memory failure. */
    RIL_CME_LONG_TEXT,    /**< The text string is too long. */
    RIL_CME_INVALID_TEXT_CHARS,    /**< Invalid characters in the text string. */
    RIL_CME_LONG_DIALSTRING,    /**< The dial string is too long. */
    RIL_CME_INVALID_DIALSTRING_CHARS,    /**< Invalid characters in the dial string. */
    RIL_CME_NO_NETWORK_SERVICE = 30,    /**< No network service. */
    RIL_CME_NETWORK_TIMEOUT,    /**< Network timeout. */
    RIL_CME_EMERGENCY_ONLY,    /**< Network is not available, emergency calls only. */
    RIL_CME_NET_PIN_REQUIRED = 40,    /**< Network personalization PIN is required. */
    RIL_CME_NET_PUK_REQUIRED,    /**< Network personalization PUK is required. */
    RIL_CME_NETSUB_PIN_REQUIRED,    /**< Network subset personalization PIN is required. */
    RIL_CME_NETSUB_PUK_REQUIRED,    /**< Network subset personalization PUK is required. */
    RIL_CME_NETPROV_PIN_REQUIRED,    /**< Service provider personalization PIN is required. */
    RIL_CME_NETPROV_PUK_REQUIRED,    /**< Service provider personalization PUK is required. */
    RIL_CME_CORPORATE_PIN_REQUIRED,    /**< Corporate personalization PIN is required. */
    RIL_CME_CORPORATE_PUK_REQUIRED,    /**< Corporate personalization PUK is required. */
    RIL_CME_HIDDEN_KEY_REQUIRED,    /**< Hidden key is required. */
    RIL_CME_UNKNOWN = 100,    /**< Unknown. */

    /* PSD errors. */

    RIL_CME_ILLEGAL_MS = 103,    /**< Illegal MS. */
    RIL_CME_ILLEGAL_ME = 106,    /**< Illegal ME. */
    RIL_CME_PSD_SERVICES_NOT_ALLOWED = 107,    /**< PSD services not allowed. */
    RIL_CME_PLMN_NOT_ALLOWED = 111,    /**< PLMN not allowed. */
    RIL_CME_LOCATION_AREA_NOT_ALLOWED,    /**< Location area not allowed. */
    RIL_CME_ROAMING_NOT_ALLOWED,    /**< Roaming not allowed in this location area. */
    RIL_CME_SERVICE_OPTION_NOT_SUPPORTED = 132,    /**< Service option not supported. */
    RIL_CME_SERVICE_OPTION_NOT_SUBSCRIBED,    /**< Requested service option not subscribed. */
    RIL_CME_SERVICE_OPTION_OUT_OF_ORDER,    /**< Service option is temporarily out of order. */
    RIL_CME_UNSPECIFIED_PSD_ERROR = 148,    /**< Unspecified PSD error. */
    RIL_CME_PDP_AUTHENTIFICATION_ERROR = 149,    /**< PDP authentication failure. */
    RIL_CME_INVALID_MOBILE_CLASS = 150,    /**< Invalid mobile class. */

    /* LTE specific errors - ESM.  */
    RIL_CME_LAST_PDN_DISCONNECTION_NOT_ALLOWED = 151,    /**< PSD - last PDN disconnection not allowed. */


    /* Final result codes. */

    START_OF_RESULT_CODES = 10000,    /**< Beginning of the final result codes. */
    RIL_RESULT_CODE_OK = START_OF_RESULT_CODES,    /**< OK. */
    RIL_RESULT_CODE_CONNECT,    /**< CONNECT. */
    RIL_RESULT_CODE_RING,    /**< RING. */
    RIL_RESULT_CODE_NO_CARRIER,    /**< NO CARRIER. */
    RIL_RESULT_CODE_ERROR,    /**< ERROR. */
    RIL_RESULT_CODE_INV,    /**< CONNECT 1200. */
    RIL_RESULT_CODE_INTERMEDIATE,    /**< intermediate response */
    RIL_RESULT_CODE_NO_DIALTONE,    /**< NO DIALTONE. */
    RIL_RESULT_CODE_BUSY,    /**< BUSY. */
    RIL_RESULT_CODE_NO_ANSWER,    /**< NO ANSWER. */
    RIL_RESULT_CODE_PROCEEDING,    /**< PROCEEDING. */
    RIL_RESULT_CODE_FCERROR,    /**< +FCERROR. */
    RIL_RESULT_CODE_CME_ERROR,    /**< +CME ERROR. */
    RIL_RESULT_CODE_CMS_ERROR,    /**< +CMS ERROR. */
    RIL_RESULT_CODE_NULL,    /**< Empty. */
    END_OF_RESULT_CODES,    /**< The end of the final result codes. */

} ril_result_code_t;


/* Do not modify the order of these command IDs. */
typedef enum {
    /* Unique identifiers for the AT commands in 3GPP Technical Specification 27.007. */
    RIL_CMD_ID_CGMI = 1,    /**< AT command +CMGI. */
    RIL_CMD_ID_CGMM,    /**< AT command +CGMM. */
    RIL_CMD_ID_CGMR,    /**< AT command +CGMR. */
    RIL_CMD_ID_CGSN,    /**< AT command +CGSN. */
    RIL_CMD_ID_CSCS,    /**< AT command +CSCS. */
    RIL_CMD_ID_CIMI,    /**< AT command +CIMI. */
    RIL_CMD_ID_CMUX,    /**< AT command +CMUX. */
#ifndef __RIL_CMD_SET_SLIM_ENABLE__
    RIL_CMD_ID_CR,    /**< AT command +CR. */
#endif
    RIL_CMD_ID_CEER,    /**< AT command +CEER. */
    RIL_CMD_ID_CNUM,    /**< AT command +CNUM. */
#ifndef __RIL_CMD_SET_SLIM_ENABLE__
    RIL_CMD_ID_CREG,    /**< AT command +CREG. */
#endif
    RIL_CMD_ID_COPS,    /**< AT command +COPS. */
    RIL_CMD_ID_CLCK,    /**< AT command +CLCK. */
    RIL_CMD_ID_CPWD,    /**< AT command +CPWD. */
    RIL_CMD_ID_CPOL,    /**< AT command +CPOL. */
    RIL_CMD_ID_CPLS,    /**< AT command +CPLS. */
    RIL_CMD_ID_COPN,    /**< AT command +COPN. */
    RIL_CMD_ID_CFUN,    /**< AT command +CFUN. */
    RIL_CMD_ID_CPIN,    /**< AT command +CPIN. */
    RIL_CMD_ID_CSQ,    /**< AT command +CSQ. */
    RIL_CMD_ID_CCLK,    /**< AT command +CCLK. */
    RIL_CMD_ID_CSIM,    /**< AT command +CSIM. */
    RIL_CMD_ID_CRSM,    /**< AT command +CRSM. */
    RIL_CMD_ID_CMAR,    /**< AT command +CMAR. */
    RIL_CMD_ID_CTZU,    /**< AT command +CTZU. */
    RIL_CMD_ID_CTZR,    /**< AT command +CTZR. */
    RIL_CMD_ID_CGPIAF,    /**< AT command +CGPIAF. */
    RIL_CMD_ID_CPINR,    /**< AT command +CPINR. */
#ifndef __RIL_CMD_SET_SLIM_ENABLE__
    RIL_CMD_ID_CSUS,    /**< AT command +CSUS. */
#endif
    RIL_CMD_ID_CESQ,    /**< AT command +CESQ. */
    RIL_CMD_ID_CMEE,    /**< AT command +CMEE. */
    RIL_CMD_ID_CGDCONT,    /**< AT command +CGDCONT. */
    RIL_CMD_ID_CGATT,    /**< AT command +CGATT. */
    RIL_CMD_ID_CGACT,    /**< AT command +CGACT. */
    RIL_CMD_ID_CGDATA,    /**< AT command +CGDATA. */
    RIL_CMD_ID_CGPADDR,    /**< AT command +CGPADDR. */
    RIL_CMD_ID_CGEREP,    /**< AT command +CGEREP. */
#ifndef __RIL_CMD_SET_SLIM_ENABLE__
    RIL_CMD_ID_CGREG,    /**< AT command +CGREG. */
#endif
    RIL_CMD_ID_CEREG,    /**< AT command +CEREG. */
    RIL_CMD_ID_CGCONTRDP,    /**< AT command +CGCONTRDP. */
    RIL_CMD_ID_CGDEL,    /**< AT command +CGDEL. */
    RIL_CMD_ID_CGAUTH,    /**< AT command +CGAUTH. */
    RIL_CMD_ID_CIPCA,    /**< AT command +CIPCA. */
    RIL_CMD_ID_CPSMS,    /**< AT command +CPSMS. */
    RIL_CMD_ID_CCIOTOPT,    /**< AT command +CCIOTOPT. */
    RIL_CMD_ID_CEDRXS,    /**< AT command +CEDRXS. */
    RIL_CMD_ID_CEDRXRDP,    /**< AT command +CEDRXRDP. */
    RIL_CMD_ID_CGAPNRC,    /**< AT command +CGAPNRC. */
    RIL_CMD_ID_CSCON,    /**< AT command +CSCON. */
    RIL_CMD_ID_CCHO,    /**< AT command +CCHO. */
    RIL_CMD_ID_CCHC,    /**< AT command +CCHC. */
    RIL_CMD_ID_CGLA,    /**< AT command +CGLA. */
    RIL_CMD_ID_CRCES,    /**< AT command +CRCES. */

#if defined(__RIL_SMS_COMMAND_SUPPORT__)
    /* Unique identifiers for the AT command in 3GPP Technical Specification 27.005. */
    RIL_CMD_ID_CSMS,    /**< AT command +CSMS. */
    RIL_CMD_ID_CPMS,    /**< AT command +CPMS. */
    RIL_CMD_ID_CMGF,    /**< AT command +CMGF. */
    RIL_CMD_ID_CSCA,    /**< AT command +CSCA. */
    RIL_CMD_ID_CSMP,    /**< AT command +CSMP. */
    RIL_CMD_ID_CSDH,    /**< AT command +CSDH. */
    RIL_CMD_ID_CSAS,    /**< AT command +CSAS. */
    RIL_CMD_ID_CRES,    /**< AT command +CRES. */
    RIL_CMD_ID_CNMI,    /**< AT command +CNMI. */
    RIL_CMD_ID_CMGL_PDU,    /**< AT command +CMGL if PDU mode. */
    RIL_CMD_ID_CMGR_PDU,    /**< AT command +CMGR if PDU mode. */
    RIL_CMD_ID_CMGS_PDU,    /**< AT command +CMGS if PDU mode. */
    RIL_CMD_ID_CMGW_PDU,    /**< AT command +CMGW if PDU mode. */
    RIL_CMD_ID_CMGD,    /**< AT command +CMGD. */
    RIL_CMD_ID_CMGC_PDU,    /**< AT command +CMGC if PDU mode. */
    RIL_CMD_ID_CNMA_PDU,    /**< AT command +CNMA if PDU mode. */
    RIL_CMD_ID_CMSS_PDU,    /**< AT command +CMSS if PDU mode. */
    RIL_CMD_ID_CMGL_TXT,    /**< AT command +CMGL if TEXT mode. */
    RIL_CMD_ID_CMGR_TXT,    /**< AT command +CMGR if TEXT mode. */
    RIL_CMD_ID_CMGS_TXT,    /**< AT command +CMGS if TEXT mode. */
    RIL_CMD_ID_CMGW_TXT,    /**< AT command +CMGW if TEXT mode. */
    RIL_CMD_ID_CMGC_TXT,    /**< AT command +CMGC if TEXT mode. */
    RIL_CMD_ID_CNMA_TXT,    /**< AT command +CNMA if TEXT mode. */
    RIL_CMD_ID_CMSS_TXT,    /**< AT command +CMSS if TEXT mode. */
#endif /* __RIL_SMS_COMMAND_SUPPORT__ */

    /* Commands in technical specification V.250. */
    RIL_CMD_ID_E,    /**< AT command ATE. */
    RIL_CMD_ID_I,    /**< AT command ATI. */
#ifndef __RIL_CMD_SET_SLIM_ENABLE__
    RIL_CMD_ID_O,    /**< AT command ATO. */
    RIL_CMD_ID_Q,    /**< AT command ATQ. */
    RIL_CMD_ID_S3,    /**< AT command ATS3. */
    RIL_CMD_ID_S4,    /**< AT command ATS4. */
    RIL_CMD_ID_S5,    /**< AT command ATS5. */
    RIL_CMD_ID_S7,    /**< AT command ATS7. */
    RIL_CMD_ID_S10,    /**< AT command ATS10. */
    RIL_CMD_ID_V,    /**< AT command ATV. */
    RIL_CMD_ID_X,    /**< AT command ATX. */
    RIL_CMD_ID_Z,    /**< AT command ATZ. */
    RIL_CMD_ID_AF,    /**< AT command AT&F. */
    RIL_CMD_ID_GCAP,    /**< AT command +GCAP. */
    RIL_CMD_ID_S25,    /**< AT command ATS25. */
#endif

    /* Unique identifiers for MediaTek proprietary AT commands. */
    RIL_CMD_ID_MLTS,    /**< AT command *MLTS. */
    RIL_CMD_ID_MSIMINS,    /**< AT command *MSIMINS. */
    RIL_CMD_ID_MSPN,    /**< AT command *MSPN. */
    RIL_CMD_ID_MUNSOL,    /**< AT command *MUNSOL. */
    RIL_CMD_ID_MGCOUNT,    /**< AT command *MGCOUNT. */
#ifndef __RIL_CMD_SET_SLIM_ENABLE__
    RIL_CMD_ID_MGSINK,    /**< AT command *MGSINK. */
    RIL_CMD_ID_MCGDEFCONT,    /**< AT command *MCGDEFCONT. */
    RIL_CMD_ID_MGTCSINK,    /**< AT command *MGTCSINK. */
    RIL_CMD_ID_MSACL,    /**< AT command *MSACL. */
    RIL_CMD_ID_MLACL,    /**< AT command *MLACL. */
    RIL_CMD_ID_MWACL,    /**< AT command *MWACL. */
    RIL_CMD_ID_MDACL,    /**< AT command *MDACL. */
    RIL_CMD_ID_MSMEXTRAINFO,    /**< AT command *MSMEXTRAINFO. */
    RIL_CMD_ID_MSMEXTRAUNSOL,    /**< AT command *MSMEXTRAUNSOL. */
    RIL_CMD_ID_MSMSTATUS,    /**< AT command *MSMSTATUS. */
    RIL_CMD_ID_MMGI,    /**< AT command *MMGI. */
    RIL_CMD_ID_MMGRW,    /**< AT command *MMGRW. */
    RIL_CMD_ID_MMGSC_PDU,    /**< AT command *MMGSC if PDU mode. */
    RIL_CMD_ID_MMGSC_TXT,    /**< AT command *MMGSC if TEXT mode. */
#endif
    RIL_CMD_ID_MUPIN,    /**< AT command *MUPIN. */
    RIL_CMD_ID_MUAPP,    /**< AT command *MUAPP. */
    RIL_CMD_ID_MSST,    /**< AT command *MSST. */
    RIL_CMD_ID_MABORT,    /**< AT command *MABORT. */
#ifndef __RIL_CMD_SET_SLIM_ENABLE__
    RIL_CMD_ID_MCAL,    /**< AT command *MCAL. */
    RIL_CMD_ID_MNON,    /**< AT command *MNON. */
    RIL_CMD_ID_MOPL,    /**< AT command *MOPL. */
    RIL_CMD_ID_MMUX,    /**< AT command *MMUX. */
    RIL_CMD_ID_MROUTEMMI,    /**< AT command *MROUTEMMI. */
    RIL_CMD_ID_MCEERMODE,    /**< AT command *MCEERMOTE. */
    RIL_CMD_ID_MFTRCFG,    /**< AT command *MFTRCFG. */
#endif
    RIL_CMD_ID_HVER,    /**< AT command ^HVER. */
#ifndef __RIL_CMD_SET_SLIM_ENABLE__
    RIL_CMD_ID_MODE,    /**< AT command ^MODE. */
    RIL_CMD_ID_SYSINFO,    /**< AT command ^SYSINFO. */
    RIL_CMD_ID_SYSCONFIG,    /**< AT command ^SYSCONFIG. */
    RIL_CMD_ID_CARDMODE,    /**< AT command ^CARDMOE. */
    RIL_CMD_ID_SPN,    /**< AT command ^SPN. */
    RIL_CMD_ID_MSTLOCK,    /**< AT command *MSTLOCK. */
    RIL_CMD_ID_MSTPD,    /**< AT command *MSTPD. */
    RIL_CMD_ID_MSTMODE,    /**< AT command *MSTMODE. */
    RIL_CMD_ID_MSTICREC,    /**< AT command *MSTICREC. */
    RIL_CMD_ID_MSTICIMG,    /**< AT command *MSTICIMG. */
    RIL_CMD_ID_MSTGC,    /**< AT command *MSTGC. */
    RIL_CMD_ID_MSTCR,    /**< AT command *MSTCR. */
    RIL_CMD_ID_MSTMS,    /**< AT command *MSTMS. */
    RIL_CMD_ID_MSTEV,    /**< AT command *MSTEV. */
#endif
    RIL_CMD_ID_MICCID,    /**< AT command *MICCID. */
    RIL_CMD_ID_MHOMENW,    /**< AT command *MHOMENW. */
#ifndef __RIL_CMD_SET_SLIM_ENABLE__
    RIL_CMD_ID_MCSIMLOCK,    /**< AT command *MCSIMLOCK. */
    RIL_CMD_ID_MEMMREEST,    /**< AT command *MEMMREEST. */
    RIL_CMD_ID_MAPNURI,    /**< AT command *MAPNURI. */
    RIL_CMD_ID_MPLMNURI,    /**< AT command *MPLMNURI. */
    RIL_CMD_ID_MPDI,    /**< AT command *MPDI. */
    RIL_CMD_ID_MNBIOTDT,    /**< AT command *MNBIOTDT. */
    RIL_CMD_ID_MNBIOTRAI,    /**< AT command *MNBIOTRAI. */
    RIL_CMD_ID_MNVMQ,    /**< AT command *MNVMQ. */
    RIL_CMD_ID_MNVMAUTH,    /**< AT command *MNVMAUTH. */
    RIL_CMD_ID_MNVMW,    /**< AT command *MNVMW. */
    RIL_CMD_ID_MNVMR,    /**< AT command *MNVMR. */
    RIL_CMD_ID_MNVMGET,    /**< AT command *MNVMGET. */
    RIL_CMD_ID_MNVMIVD,    /**< AT command *MNVMIVD. */
    RIL_CMD_ID_MNVMRSTONE,    /**< AT command *MNVMSTONE. */
    RIL_CMD_ID_MNVMRST,    /**< AT command *MNVMRST. */
    RIL_CMD_ID_MNVMMDNQ,    /**< AT command *MNVMDNQ. */
    RIL_CMD_ID_MNVMMDR,    /**< AT command *MNVMDR. */
    RIL_CMD_ID_MNVMMDC,    /**< AT command *MNVMDC. */
    RIL_CMD_ID_IDCFREQ,    /**< AT command +IDCFREQ. */
    RIL_CMD_ID_IDCPWRBACKOFF,    /**< AT command +IDCPWRBACKOFF. */
    RIL_CMD_ID_IDCTX2GPS,    /**< AT command +IDCTX2GPS. */
#endif
    RIL_CMD_ID_MCALDEV,    /**< AT command *MCALDEV. */
    RIL_CMD_ID_MATWAKEUP,    /**< AT command *MATWAKEUP. */
    RIL_CMD_ID_MBAND,    /**< AT command *MBAND. */
    RIL_CMD_ID_MENGINFO,    /**< AT command *MENGINFO. */
    RIL_CMD_ID_MFRCLLCK,    /**< AT command *MFRCLLCK. */
    RIL_CMD_ID_MSPCHSC,    /**< AT command *MSPCHSC. */
    RIL_CMD_ID_MDPDNP,    /**< AT command *MDPDNP. */
    RIL_CMD_ID_MEDRXCFG,    /**< AT command *MEDRXCFG. */
    RIL_CMD_ID_MCELLINFO,    /**< AT command *MCELLINFO. */
    RIL_CMD_ID_MCGSN,    /**< AT command: *MCGSN. */
    RIL_CMD_ID_MUPDIR,    /**< AT command *MUPDIR. */
    RIL_CMD_ID_MTC,    /**< AT command: *MTC. */

    RIL_CMD_ID_CUSTOM_CMD,
    RIL_CMD_ID_CUSTOM_CMD_URC,

    RIL_CMD_ID_INVALID    /**< Indicates an invalid identifier. */

} ril_cmd_id_t;


/** @brief This enum defines the event IDs for unsolicited result codes (URC). */
typedef enum {
    RIL_URC_ID_START = 0,    /**< URC event ID for command +CREG. */
    RIL_URC_ID_TEST,

    RIL_URC_ID_CREG,    /**< +CREG. */
    RIL_URC_ID_CTZR,    /**< +CTZR. */
    RIL_URC_ID_CGEV,    /**< +CGEV. */
    RIL_URC_ID_CGREG,    /**< +CGREG. */
    RIL_URC_ID_CEREG,    /**< +CEREG. */
    RIL_URC_ID_CCIOTOPT,    /**< +CCIOTOPT. */
    RIL_URC_ID_CEDRXP,    /**< +CEDRXP. */
    RIL_URC_ID_CSCON,    /**< +CSCON. */
    RIL_URC_ID_CPIN,    /**< +CPIN. */    

#if defined(__RIL_SMS_COMMAND_SUPPORT__)
    /* SMS specifications */
    RIL_URC_ID_CMTI,    /**< +CMTI. */
    RIL_URC_ID_CMT,    /**< +CMT. */
#endif

    /* Proprietary command specifications. */
    RIL_URC_ID_MLTS,    /**< *MLTS. */
    RIL_URC_ID_MSIMINS,    /**< *MSIMINS. */
    RIL_URC_ID_MSQN,    /**< *MSQN. */
    RIL_URC_ID_MGCOUNT,    /**< *MGCOUNT. */
#ifndef __RIL_CMD_SET_SLIM_ENABLE__
    RIL_URC_ID_MSMEXTRAUNSOL,    /**< *MSMEXTRAUNSOL. */
    RIL_URC_ID_MMGI,    /**< *MMGI. */
    RIL_URC_ID_MSMPUKBLKD,    /**< *MSMPUKBLKD. */
    RIL_URC_ID_MODE,    /**< ^MODE. */
    RIL_URC_ID_MSTC,    /**< *MSTC. */
    RIL_URC_ID_MSTUD,    /**< *MSTUD. */
    RIL_URC_ID_MCCST,    /**< *MCCST. */
    RIL_URC_ID_MCSIMLOCK,    /**< *MCSIMLOCK. */
    RIL_URC_ID_MAPNURI,    /**< *MAPNURI. */
    RIL_URC_ID_MPLMNURI,    /**< *MPLMNURI. */
    RIL_URC_ID_MPDI,    /**< *MPDI. */
    RIL_URC_ID_IDCFREQ,    /**< +IDCFREQ. */
#endif
    RIL_URC_ID_MATREADY,    /**< *MATREADY. */
    RIL_URC_ID_MATWAKEUP,    /**< *MATWAKEUP. */
    RIL_URC_ID_MDPDNP,    /**< *MDPDNP. */
    RIL_URC_ID_MUPDIR,    /**< *MUPDIR. */
    RIL_URC_ID_MUPDI,    /**< *MUPDI. */

    RIL_URC_ID_END,    /**< End of the URC event ID enumeration. */
    RIL_URC_ID_INVALID = RIL_URC_ID_END,    /**< Indicates an invalid URC event ID. */
} ril_urc_id_t;

/**
 * @}
 */


/**
 * @addtogroup ril_typedef Typedef
 * @{ 
 * RIL related function pointer prototype definitions.
 */

/**
 * @brief A function pointer prototype for the URC event callback.
 * @param[out] param    A pointer to the data structure with URC parameters.
 * @param[out] param_len    Indicates the length of the reported parameter structure.
 * @return reserved.
 */
typedef int32_t (*ril_event_callback_t)(ril_urc_id_t event_id, void *param, uint32_t param_len);
/**
 * @}
 */

/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief Defines the reporting parameter structure for AT response.
 */
typedef struct {
    ril_cmd_id_t cmd_id;    /**< A unique ID for the AT command. */
    ril_request_mode_t mode;    /**< AT command mode, like execute(set) mode, active mode, read mode and test mode, is the same as which mode user passed. */
    ril_result_code_t res_code;    /**< Final response code, parsed from the raw AT response string. */
    void *cmd_param;    /**< A pointer to the subparameter structure, user should cast it to the right data structure specified for each command. */
    void *user_data;    /**< A pointer to the user data, it's the same data passed to the send API. */
    char *test_mode_str;    /**< A pointer to the response string if under AT command test mode, otherwise it's NULL. */
    uint32_t test_mode_str_len;    /**< The response string length if under AT command test mode, otherwise it's zero. */
} ril_cmd_response_t;
/**
 * @}
 */

/** 
 * @addtogroup ril_typedef Typedef
 * @{
 */
/**
 * @brief Defines the callback fucntion pointer prototype for AT response.
 * @param[out] response    A pointer to the data, command mode, subparameters, user data and more.
 * @return Reserved.
 */
typedef int32_t (*ril_cmd_response_callback_t)(ril_cmd_response_t *response);

/**
 * @brief This defined callback fucntion pointer prototype for customized AT command response
 * @param[out] rsp_str    A pointer to a string that was reported by customized AT command handler. 
 * @param[out] rsp_str_len    indicates the length of response string. 
 * @param[out] user_data    A pointer to the user data.
 */
typedef int32_t (*ril_custom_cmd_response_callback_t)(char *rsp_str, uint32_t rsp_str_len, void *user_data);
/**
 * @}
 */

/**
 * @brief    This function registers a URC event handler.
 * @param[in] group_mask    indicates the group URCs to receive. It may be doing "OR" operation for multiple group bit masks.
 * @param[in] callback    is callback function to register. This callback function is invoked by RIL when URC arrives and need to notify to user.
 * @return #RIL_STATUS_SUCCESS, if the operation completed successfully.
 *             #RIL_STATUS_FAIL, if the operation has failed.
 */
extern ril_status_t ril_register_event_callback(uint32_t group_mask, ril_event_callback_t callback);

/**
 * @brief    This function deregisters URC event handler.
 * @param[in] callback    is the callback function to deregister.
 * @return #RIL_STATUS_SUCCESS, if the operation completed successfully.
 *             #RIL_STATUS_FAIL, if the operation has failed.
 */
extern ril_status_t ril_deregister_event_callback(ril_event_callback_t callback);

/**
 * @brief    This function gets an available data channel ID, note that the channel ID starts from 1.
 * @return    0    if all data channels are busy.
 *                others    if the channel ID is available.
 */
extern uint32_t ril_get_free_data_channel();

/**
 * @brief    This function sets data channel as reserved or not.
 * @param[in] channel_id    is to specify which channel to configure.
 * @param[in] is_reserved    set true to reserve the specified channel.
 *                                        set false to release the channel.
 * @return   if the return value is #RIL_STATUS_SUCCESS, the operation completed successfully.
 *               if the return value is #RIL_STATUS_FAIL, invalid data channel ID.
 */
extern ril_status_t ril_set_data_channel_reserved(uint32_t channel_id, bool is_reserved);

/**
 * @brief    Internal use, used to protect from dirty data received when switching to data mode.
 */
extern ril_status_t ril_channel_forbid_data_receiving(uint32_t channel_id, bool enabled);

/*************************************************
 *                 3GPP Technial Specification v27.007
 *************************************************/

/* AT command: AT+CGMI */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * RIL related data structure definitions.
 * @brief This defines data structure for AT response of "+CGMI".
 */
typedef struct {
    char *manufacturer;    /**< TA returns the manufacturer identification information. */
} ril_manufacturer_identification_rsp_t;
/**
 * @}
 */


/** 
 * @brief This function sends an AT command "+CGMI".
 * @param[in] mode    AT command mode setting.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
                                     For sub-parameters data structure, please refer to #ril_manufacturer_identification_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_manufacturer_identification(ril_request_mode_t mode,
                                                                ril_cmd_response_callback_t callback,
                                                                void *user_data);


/* AT command: AT+CGMM */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CGMM".
 */
typedef struct {
    char *model;    /**< TA returns the product model identification information. */
} ril_model_identification_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CGMM".
 * @param[in] mode    AT command mode setting.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For sub-parameters data structure, please refer to #ril_model_identification_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_model_identification(ril_request_mode_t mode,
        ril_cmd_response_callback_t callback,
        void *user_data);



/* AT command: AT+CGMR */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CGMR".
 */
typedef struct {
    char *revision;    /**< TA returns the product software version identification information. */
} ril_revision_identification_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CGMR".
 * @param[in] mode    AT command mode setting.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For sub-parameters data structure, please refer to #ril_revision_identification_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_revision_identification(ril_request_mode_t mode,
        ril_cmd_response_callback_t callback,
        void *user_data);



/* AT command: AT+CGSN */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CGSN".
 */
typedef struct {
    union {
        char *sn;    /**< Serial number of the device. */
        char *imei;    /**< IMEI. */
        char *imeisv;    /**< Internatinal Mobile Equipment Identity Software Version (IMEISV). */
        char *svn;    /**< Software version number. */
    } value;
} ril_serial_number_rsp_t;
/**
 * @}
 */
/** 
 * @brief This function sends an AT command "+CGSN".
 * @param[in] mode    AT command mode setting.
 * @param[in] sn_type    indicates the sub-parameter value. \n
 *                                  Gets the serial number, if set to 0 or omitted. \n
 *                                  Gets the IMEI, if set to 1. \n
 *                                  Gets the IMEISV, if set to 2. \n
 *                                  Gets the software version number, if set to 3.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For sub-parameters data structure, please refer to #ril_serial_number_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_serial_number(ril_request_mode_t mode,
        int32_t sn_type,
        ril_cmd_response_callback_t callback,
        void *user_data);



/* AT command: AT+CSCS */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CSCS".
 */
typedef struct {
    char *chset;    /**< TA returns the character set in use. */
} ril_select_character_set_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CSCS".
 * @param[in] mode    AT command mode setting.
 * @param[in] chset    A pointer to the character set used by TE.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For sub-parameters data structure, please refer to #ril_select_character_set_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_select_charcter_set(ril_request_mode_t mode,
        char *chset,
        ril_cmd_response_callback_t callback,
        void *user_data);



/* AT command: AT+CIMI */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CIMI".
 */
typedef struct {
    char *imsi;    /**< TA returns the IMSI to identify the SIM card attached to ME. */
} ril_imsi_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CIMI".
 * @param[in] mode    AT command mode setting.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For sub-parameters data structure, please refer to #ril_imsi_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_imsi(ril_request_mode_t mode,
                                     ril_cmd_response_callback_t callback,
                                     void *user_data);


/* AT command: AT+CMUX */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CMUX".
 */
typedef struct {
    int32_t mode;         /**< Multiplexer Transparency Mechanism. */
    int32_t subset;       /**< Multiplexer control channel and setup mechanism. */
    int32_t port_speed;   /**< Transmission rate for each channel.*/
    int32_t N1;           /**< Maximum frame size for each channel.*/
    int32_t T1;           /**< Acknowledgement timer in units of ten milliseconds. */
    int32_t N2;           /**< The maximum number of re-transmissions.*/
    int32_t T2;           /**< Response timer for the multiplexer control channel in units of ten milliseconds.*/
    int32_t T3;           /**< Wake up response timer in seconds.*/
    int32_t k;            /**< Window size for an advanced option with error recovery mode.*/
} ril_cmux_param_t, ril_cmux_param_rsp_t;
/**
 * @}
 */


/** 
 * @brief This function sends an AT command "+CMUX". The AT command sets parameters for the Control Channel. If the parameters
               are left out, the default values are used.
 * @param[in] mode    AT command mode setting.
 * @param[in] cmux_param    indicates the sub-parameter value, please refer to #ril_cmux_param_t.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For sub-parameters data structure, please refer to #ril_cmux_param_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_multiplexing_mode(ril_request_mode_t mode,
        ril_cmux_param_t *cmux_param,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT+CR */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CR".
 */
typedef struct {
    int32_t mode; /**< TA controls whether intermediate result code +CR: <serv> is returned from the TA to the TE. */
} ril_service_reporting_control_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CR".
 * @param[in] mode    AT command mode setting.
 * @param[in] cr_mode is the intermediate result code +CR: <serv> returned from the TA to the TE.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For sub-parameters data structure, please refer to #ril_service_reporting_control_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_service_reporting_control(ril_request_mode_t mode,
        int32_t cr_mode,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT+CEER */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CEER".
 */
typedef struct {
    char *report;  /**< TA returns results from GSM/UMTS network. */
} ril_extended_error_report_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CEER".
 * @param[in] mode    AT command mode setting.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For sub-parameters data structure, please refer to #ril_extended_error_report_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_extended_error_report(ril_request_mode_t mode,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT cmd: AT+CNUM */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for subscriber number.
 */
typedef struct {
    char* alpha;    /**< Optional alphanumeric string associated with <number>; used character set should be the one selected with command. */
    char* number;    /**< String type, phone number of format specified by <type> */
    int32_t type;    /**< Type of address octet in integer format.(refer to 3GPP 24.008 sub-clause 10.5.4.7) */
} ril_subscriber_number_entry_t;
/**
 * @brief This defines data structure for AT response of "+CNUM".
 */
typedef struct {
    int32_t array_num;    /**< The number of elements in the array. */
    ril_subscriber_number_entry_t *entries_array;    /**< A pointer to the array for the list of subscriber number, please refer to #ril_subscriber_number_entry_t, for information of each subscriber number. */
} ril_subscriber_number_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CNUM". This AT command controls the presentation of an unsolicited result code "+CNUM".
 * @param[in] mode    AT command mode setting.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For sub-parameters data structure, please refer to #ril_subscriber_number_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
ril_status_t ril_request_subscriber_number(ril_request_mode_t mode,
        ril_cmd_response_callback_t callback,
        void *user_data);



/* AT command: AT+CREG */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CREG".
 */
typedef struct {
    int32_t stat; /**< Circuit mode registration status of TA. */
    int32_t lac;  /**< Location area code.*/
    int32_t ci;   /**< Cell ID. */
    int32_t act;  /**< The access technology of the serving cell. */
    int32_t cause_type;  /**< The type of reject_cause. */
    int32_t reject_cause; /**< The reason for the failed registration. */
} ril_network_registration_urc_t;

/**
 * @brief This defines data structure for URC notification of "+CREG".
 */
typedef struct {
    int32_t n;  /**< The action of the AT command. */
    int32_t stat; /**< Circuit mode registration status of TA. */
    int32_t lac;  /**< Location area code.*/
    int32_t ci;   /**< Cell ID. */
    int32_t act;  /**< The access technology of the serving cell. */
    int32_t cause_type; /**< The type of reject_cause. */
    int32_t reject_cause; /**< The reason for the failed registration. */
} ril_network_registration_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CREG". This AT command controls the presentation of an unsolicited result code "+CREG".
 * @param[in] mode    AT command mode setting.
 * @param[in] action    indicates the action of the "+CREG" URC. 
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For sub-parameters data structure, please refer to #ril_network_registration_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_network_registration(ril_request_mode_t mode,
        int32_t action,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT+COPS */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+COPS".
 */
typedef struct {
    int32_t mode;    /**< Indicates the mode selected by MT; automatic mode, manual operator mode, manual deregistration, and more. */
    int32_t format;  /**< The format of the field "oper". */
    char *oper;      /**< Location area identification number with an operator code. */
    int32_t act;     /**< Indicates the access technology. */
} ril_plmn_selection_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+COPS".
 * @param[in] mode    AT command mode setting.
 * @param[in] cmode    Indicates the sub-parameter value.
 * @param[in] format    The format of the field "oper".
 * @param[in] oper    Location area identification number with an operator code.
 * @param[in] act    Indicates the access technology.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For sub-parameters data structure, please refer to #ril_plmn_selection_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_plmn_selection(ril_request_mode_t mode,
                                        int32_t cmode,
                                        int32_t format,
                                        char *oper,
                                        int32_t act,
                                        ril_cmd_response_callback_t callback,
                                        void *user_data);


/* AT command: AT+CLCK */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for the lock status.
 */
typedef struct {
    int32_t status; /**< The status of the related class.*/
    int32_t classn; /**< The information of field "class", but it's not required by NB-IoT, so will be ignored. */
} ril_facility_lock_struct_t;
/**
 * @brief This defines data structure for AT response of "+CLCK".
 */
typedef struct {
    ril_facility_lock_struct_t *status_array; /**< A pointer to the array to indicate lock status, please refer to #ril_facility_lock_struct_t, for each element in the array. */
    int32_t array_num; /**< The number of elements in the array.*/
} ril_facility_lock_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CLCK".
 * @param[in] mode    AT command mode setting.
 * @param[in] fac    The network facility.
 * @param[in] cmode    The action on this network facility.
 * @param[in] passwd    The password specified for the facility.
 * @param[in] class_p    The class of information.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For sub-parameters data structure, please refer to #ril_facility_lock_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_facility_lock(ril_request_mode_t mode,
                                       char *fac,
                                       int32_t cmode,
                                       char *passwd,
                                       int32_t class_p,
                                       ril_cmd_response_callback_t callback,
                                       void *user_data);


/* AT command: AT+CPWD */
/** 
 * @brief This function sends an AT command "+CPWD".
 * @param[in] mode    AT command mode setting.
 * @param[in] fac The network facility.
 * @param[in] oldpwd The previous password.
 * @param[in] newpwd The new password.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_change_password(ril_request_mode_t mode,
        char *fac,
        char *oldpwd,
        char *newpwd,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT+CPOL */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This is data structure for each PLMN item.
 */
typedef struct {
    int32_t index; /**< The order number of operator in the SIM/USIM preferred operator list. */
    int32_t format; /**< The format of the field "oper". */
    char *oper; /**< Location area identification number with an operator code. */
    int32_t gsm_act; /**< GSM access technology. */
    int32_t gsm_compact_act; /**< GSM compact access technology. */
    int32_t utran_act;   /**< UTRAN access technology. */
    int32_t e_utran_act; /**<E-UTRAN access technology. */
} ril_plmn_item_struct;
/**
 * @brief This defines data structure for AT response of "+CPOL".
 */
typedef struct {
    ril_plmn_item_struct *plmn_list; /**< A pointer to the array to indicate the PLMN list. */
    int32_t array_num; /**< The number of elements in the array. */
} ril_preferred_plmn_list_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CPOL".
 * @param[in] mode    AT command mode setting.
 * @param[in] plmn_p The edited PLMN selector with Access Technology lists in the SIM card.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For sub-parameters data structure, please refer to #ril_preferred_plmn_list_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_preferred_plmn_list(ril_request_mode_t mode,
        ril_plmn_item_struct *plmn_p,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT+CPLS */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CPLS".
 */
typedef struct {
    int32_t list; /**< Options for PLMN control; user-controlled, operator-controlled and others. */
} ril_selection_of_preferred_plmn_list_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CPLS".
 * @param[in] mode    AT command mode setting.
 * @param[in] list    Options for PLMN control.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For sub-parameters data structure, please refer to #ril_selection_of_preferred_plmn_list_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_selection_of_preferred_plmn_list(ril_request_mode_t mode,
        int32_t list,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT+COPN */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This is data structure for operator name.
 */
typedef struct {
    char *numeric; /**< The operator name in numeric format. */
    char *alpha;  /**< The operator name in long alphanumeric format. */
} ril_operator_names_struct_t;

/**
 * @brief This defines data structure for AT response of "+COPN". The list of operator names.
 */
typedef struct {
    ril_operator_names_struct_t *oper_name; /**< A pointer to the array for the list of operator names, please refer to #ril_operator_names_struct_t, for information of each operator name. */
    int32_t array_num; /**< The number of elements in the array. */
} ril_read_operator_names_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+COPN".
 * @param[in] mode    AT command mode setting.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For sub-parameters data structure, please refer to #ril_read_operator_names_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_read_operator_names(ril_request_mode_t mode,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT+CFUN */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CFUN".
 */
typedef struct {
    int32_t fun; /**< The level of functionality in the Mobile Termination (MT). */
} ril_set_phone_functionality_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CFUN".
 * @param[in] mode    AT command mode setting.
 * @param[in] fun    The level of functionality in the MT.
 * @param[in] rst    Whether to reset the MT before the action.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For sub-parameters data structure, please refer to #ril_set_phone_functionality_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
ril_status_t ril_request_set_phone_functionality(ril_request_mode_t mode,
        int32_t fun,
        int32_t rst,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT+CPIN */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response and the URC of "+CPIN".
 */
typedef struct {
    char *code;  /**< Indicates SIM card status or whether a password is required. */
} ril_enter_pin_rsp_t, ril_enter_pin_urc_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CPIN".
 * @param[in] mode    AT command mode setting.
 * @param[in] pin    password for the previous pin.
 * @param[in] new_pin password for the new pin.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For sub-parameters data structure, please refer to #ril_enter_pin_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_enter_pin(ril_request_mode_t mode,
                                   char *pin,
                                   char *new_pin,
                                   ril_cmd_response_callback_t callback,
                                   void *user_data);


/* AT command: AT+CSQ */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CSQ".
 */
typedef struct {
    int32_t rssi; /**< The received signal strength indication. */
    int32_t ber; /**< The bit error rate. */
} ril_signal_quality_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CSQ".
 * @param[in] mode    AT command mode setting.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For sub-parameters data structure, please refer to #ril_signal_quality_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_signal_quality(ril_request_mode_t mode,
                                        ril_cmd_response_callback_t callback,
                                        void *user_data);


/* AT command: AT+CCLK */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CCLK".
 */
typedef struct {
    char *time; /**< The response time in "yy/MM/dd,hh:mm:sszz" format. */
} ril_clock_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CCLK".
 * @param[in] mode    AT command mode setting.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For sub-parameters data structure, please refer to #ril_clock_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_clock(ril_request_mode_t mode,
                               char *time,
                               ril_cmd_response_callback_t callback,
                               void *user_data);


/* AT command: AT+CSIM */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CSIM".
 */
typedef struct {
    int32_t length; /**< Number of characters in the command sent to TE. */
    char *response; /**< Command response sent from SIM to the MT. */
} ril_generic_sim_access_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CSIM".
 * @param[in] mode    AT command mode setting.
 * @param[in] length    Number of characters sent in a command to the TE.
 * @param[in] command Command passed on by the MT to the SIM.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For sub-parameters data structure, please refer to #ril_generic_sim_access_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_generic_sim_access(ril_request_mode_t mode,
        int32_t length,
        char *command,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT+CRSM */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT command of "+CRSM".
 */
typedef struct {
    int32_t command; /**< The command passed on by the MT to the SIM. */
    int32_t field;  /**< This is the identifier of an elementary datafile on SIM. */
    int32_t p1;  /**< The parameters passed on by the MT to the SIM. */
    int32_t p2;  /**< The parameters passed on by the MT to the SIM. */
    int32_t p3;  /**< The parameters passed on by the MT to the SIM. */
    char *data;  /**< Data written to the SIM. */
    char *pathid; /**< Data file path on the SIM/UICC. */
} ril_restricted_sim_access_req_t;

/**
 * @brief This defines data structure for AT response of "+CRSM".
 */
typedef struct {
    int32_t sw1; /**< The information from the SIM about the execution of the actual command. */
    int32_t sw2; /**< The information from the SIM about the execution of the actual command. */
    char *response; /**< Indicates successful completion of the command issued previously. */
} ril_restricted_sim_access_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CRSM".
 * @param[in] mode    AT command mode setting.
 * @param[in] req    The information of restricted SIM access.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For sub-parameters data structure, please refer to #ril_restricted_sim_access_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_restricted_sim_access(ril_request_mode_t mode,
        ril_restricted_sim_access_req_t *req,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT+CMAR */
/** 
 * @brief This function sends an AT command "+CMAR".
 * @param[in] mode    AT command mode setting.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_master_reset(ril_request_mode_t mode,
                                      char *phone_lock_code,
                                      ril_cmd_response_callback_t callback,
                                      void *user_data);


/* AT command: AT+CTZU */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CTZU".
 */
typedef struct {
    int32_t onoff; /**< Enables or disables automatic timezone update. */
} ril_automatic_time_zone_update_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CTZU".
 * @param[in] mode    AT command mode setting.
 * @param[in] onoff Enables or disables automatic time zone update.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For sub-parameters data structure, please refer to #ril_automatic_time_zone_update_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_automatic_time_zone_update(ril_request_mode_t mode,
        int32_t onoff,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT+CTZR */
/* URC: +CTZR */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for URC of "+CTZR".
 */
typedef struct {
    char *tz;  /**< The local time zone. */
    int32_t dst; /**< The daylight savings adjustment. */
    char *time;  /**< The local time. The format is "YYYY/MM/DD,hh:mm:ss". */
} ril_time_zone_reporting_urc_t;

/**
 * @brief This defines data structure for AT response of "+CTZR".
 */
typedef struct {
    int32_t reporting; /**< Time zone change event status. */
} ril_time_zone_reporting_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CTZR".
 * @param[in] mode    AT command mode setting.
 * @param[in] reporting Enable or disable the time zone change event reporting.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For sub-parameters data structure, please refer to #ril_time_zone_reporting_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_time_zone_reporting(ril_request_mode_t mode,
        int32_t reporting,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT+CGPIAF */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CGPIAF".
 */
typedef struct {
    int32_t ipv6_addr_format;    /**< The IPv6 address format. */
    int32_t ipv6_subnetnotation; /**< The subnet-notation for <remote address and subnetmask>. */
    int32_t ipv6_leadingzeros;   /**< Indicates whether the leading zeros are omitted. */
    int32_t ipv6_compresszeros;  /**< Whether 1-n instances of 16-bit zero-values are replaced by only '::'. */
} ril_printing_ip_address_format_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CGPIAF".
 * @param[in] mode    AT command mode setting.
 * @param[in] ipv6_addr_format the IPv6 address format.
 * @param[in] ipv6_subnetnotation the subnet-notation for <remote address and subnetmask>.
 * @param[in] ipv6_leadingzeros indicates whether leading zeros are omitted.
 * @param[in] ipv6_compresszeros indicates whether 1-n instances of 16-bit zero-values are replaced by only '::'.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For sub-parameters data structure, please refer to #ril_printing_ip_address_format_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_printing_ip_address_format(ril_request_mode_t mode,
        int32_t ipv6_addr_format,
        int32_t ipv6_subnetnotation,
        int32_t ipv6_leadingzeros,
        int32_t ipv6_compresszeros,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT+CPINR */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CPINR".
 */
typedef struct {
    char *code;   /**< Type of PIN. */
    int32_t retries;  /**< Number of remaining retries per PIN. */
    int32_t default_retries; /**< Number of default/initial retries per PIN. */
} ril_remaining_pin_retries_entry_t;

/**
 * @brief This defines data structure for AT response of "+CPINR".
 */
typedef struct {
    ril_remaining_pin_retries_entry_t *retry_entries; /**< A pointer to the array to indicate the pin entry list. */
    int32_t array_num;  /**< The number of pin entries. */
} ril_remaining_pin_retries_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CPINR".
 * @param[in] mode    AT command mode setting.
 * @param[in] sel_code PIN type.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For sub-parameters data structure, please refer to #ril_remaining_pin_retries_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_remaining_pin_retries(ril_request_mode_t mode,
        char *sel_code,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT+CSUS */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CSUS".
 */
typedef struct {
    int32_t card_slot; /**< Slot index for the selected card. */
} ril_set_card_slot_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CSUS".
 * @param[in] mode    AT command mode setting.
 * @param[in] card_slot    Slot index for the selected card.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For sub-parameters data structure, please refer to #ril_set_card_slot_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_set_card_slot(ril_request_mode_t mode,
                                       int32_t card_slot,
                                       ril_cmd_response_callback_t callback,
                                       void *user_data);


/* AT command: AT+CESQ */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CESQ".
 */
typedef struct {
    int32_t rxlev;  /**< The received signal strength level. */
    int32_t ber;   /**< The channel bit error rate. */
    int32_t rscp;  /**< The received signal code power. */
    int32_t ecno;  /**< The ratio of the received energy per PN chip to the total received power spectral density. */
    int32_t rsrq;  /**< The reference signal received quality. */
    int32_t rsrp;  /**< The reference signal received power. */
} ril_extended_signal_quality_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CESQ".
 * @param[in] mode    AT command mode setting.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For sub-parameters data structure, please refer to #ril_extended_signal_quality_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_extended_signal_quality(ril_request_mode_t mode,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT+CMEE */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CMEE".
 */
typedef struct {
    int32_t n;  /**< Disables or enables the use of final result code.*/
} ril_report_mobile_termination_error_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CMEE".
 * @param[in] mode    AT command mode setting.
 * @param[in] n Disables or enables the use of final result code.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For sub-parameters data structure, please refer to #ril_report_mobile_termination_error_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_report_mobile_termination_error(ril_request_mode_t mode,
        int32_t n,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT+CGDCONT */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for data account entry.
 */
typedef struct {
    int32_t cid;    /**< Defines the PDP context. */
    char *pdp_type;  /**< Specifies the type of packet data protocol. */
    char *apn;  /**< A logical name to select the GGSN or the external packet data network. */
    char *pdp_addr;  /**< Identifies the MT in the address space applicable to the PDP. */
    int32_t d_comp;  /**< Controls PDP data compression. */
    int32_t h_comp;  /**< Controls PDP header compression. */
    int32_t ipv4addralloc; /**< Controls MT/TA requests to get the IPv4 address information. */
    int32_t request_type; /**< Indicates the type of PDP context activation request for the PDP context*/
    int32_t pcscf_discovery; /**< Influences the MT/TA requests to get the P-CSCF address. */
    int32_t im_cn_signalling_flag_ind; /**< Indicates to the network whether the PDP context is for IMCN subsystem-related signalling only or not.*/
    int32_t nslpi; /**< Indicates the NAS signalling priority requested for this PDP context. */
    int32_t securepco; /**< Specifies if security protected transmission of PCO is requested or not. */
} ril_pdp_context_entity_t;

/**
 * @brief This defines data structure for AT response of "+CGDCONT".
 */
typedef struct {
    ril_pdp_context_entity_t *pdp_context; /**< A pointer to the array to indicate data account entries list. */
    int32_t array_num;  /**< The number of elements in the array. */
} ril_define_pdp_context_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CGDCONT".
 * @param[in] mode    AT command mode setting.
 * @param[in] req Specifies the PDP context parameter values.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For sub-parameters data structure, please refer to #ril_define_pdp_context_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_define_pdp_context(ril_request_mode_t mode,
        ril_pdp_context_entity_t *req,
        ril_cmd_response_callback_t callback,
        void *user_data,
        int32_t channel_id);


/* AT command: AT+CGATT */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CGATT".
 */
typedef struct {
    int32_t state; /**< Indicates the state of the PS attachment. */
} ril_ps_attach_or_detach_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CGATT".
 * @param[in] mode    AT command mode setting.
 * @param[in] state Indicates the state of the PS attachment.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For sub-parameters data structure, please refer to #ril_ps_attach_or_detach_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_ps_attach_or_detach(ril_request_mode_t mode,
        int32_t state,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT+CGACT */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT command of "+CGACT".
 */
typedef struct {
    int32_t state;  /**< Activate/deactivate the PDP context (s).*/
    int32_t *cid_array; /**< The list of cids (PDP context). */
    int32_t cid_array_len; /**< Total number of cids. */
} ril_pdp_context_activate_or_deactivate_req_t;
/**
 * @brief This is data structure for the state entry.
 */
typedef struct {
    int32_t state; /**< Activate/deactivate the PDP context(s). */
    int32_t cid; /**< Specifies a particular PDP context. */
} ril_cid_state_struct_t;

/**
 * @brief This defines data structure for AT response of "+CGACT".
 */
typedef struct {
    ril_cid_state_struct_t *cid_state;  /**< A pointer to the array to indicate the cids state, please refer to #ril_cid_state_struct_t, for information about each cid. */
    int32_t array_num; /**< The number of the entries. */
} ril_pdp_context_activate_or_deactivate_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CGACT".
 * @param[in] mode    AT command mode setting.
 * @param[in] req Activate or deactivate the specified PDP context (s).
                  Please refer to #ril_pdp_context_activate_or_deactivate_req_t, for the subparameters data structure.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For sub-parameters data structure, please refer to #ril_pdp_context_activate_or_deactivate_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
ril_status_t ril_request_pdp_context_activate_or_deactivate(ril_request_mode_t mode,
        ril_pdp_context_activate_or_deactivate_req_t *req,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT+CGDATA */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT command of "+CGDATA".
 */
typedef struct {
    char *l2p;  /**< Indicates the layer 2 protocol to be used between the TE and MT. */
    int32_t *cid_array; /**< A pointer to the array to indicate PDP contexts. */
    int32_t cid_array_len; /**< Total number of PDP contexts. */
} ril_enter_data_state_req_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CGDATA".
 * Execute this command to establish communication between the TE and the network using one or more Packet Domain PDP types.
 * @param[in] mode    AT command mode setting.
 * @param[in] req    Request parameter. Please refer to #ril_enter_data_state_req_t, for more information about the subparameters data structure.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
ril_status_t ril_request_enter_data_state(ril_request_mode_t mode,
        ril_enter_data_state_req_t *req,
        ril_cmd_response_callback_t callback,
        void *user_data,
        int32_t channel_id);


/* AT command: AT+CGPADDR */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT command of "+CGPADDR".
 */
typedef struct {
    int32_t *cid_array;   /**< A pointer to the array to indicate PDP contexts. */
    int32_t cid_array_len; /**< Total number of PDP contexts. */
} ril_show_pdp_address_req_t;
/**
 * @brief PDP context address entry data structure.
 */
typedef struct {
    int32_t cid;   /**< Identifies the PDP context. */
    char *pdp_addr_1; /**< The address space applicable to the PDP. */
    char *pdp_addr_2; /**< The address space applicable to the PDP. */
} ril_cid_addr_struct_t;
/**
 * @brief This defines data structure for AT response of "+CGPADDR".
 */
typedef struct {
    ril_cid_addr_struct_t *cid_addr;  /**< The PDP context address list. */
    int32_t array_num;  /**< Number of entries in the list.*/
} ril_show_pdp_address_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CGPADDR".
 * @param[in] mode    AT command mode setting.
 * @param[in] req For more information about sub-parameters, please see #ril_show_pdp_address_req_t.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about the sub-parameters,  please refer to #ril_show_pdp_address_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
ril_status_t ril_request_show_pdp_address(ril_request_mode_t mode,
        ril_show_pdp_address_req_t *req,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT+CGEREP */
/* URC: +CGEV*/
/** 
* @addtogroup RIL_enum Enum
* @{
*/
/**
 * RIL related enum definition
 * @brief This defines an enumeration for the packet format of URC "+CGEV".
 */
typedef enum {
    RIL_URC_TYPE_NW_DETACH, /**< The network has forced a PS to detach. */
    RIL_URC_TYPE_ME_DETACH, /**< The mobile termination has forced a PS to detach. */
    RIL_URC_TYPE_NW_CLASS, /**< The network has forced a change of MT class. */
    RIL_URC_TYPE_ME_CLASS, /**< The mobile termination has forced a change of MT class. */
    RIL_URC_TYPE_NW_PDN_ACT, /**< The network has activated a context. */
    RIL_URC_TYPE_ME_PDN_ACT, /**< The mobile termination has activated a context. */
    RIL_URC_TYPE_NW_ACT, /**< The network has activated a context. */
    RIL_URC_TYPE_ME_ACT, /**< The network has responded to an ME initiated context activation. */
    RIL_URC_TYPE_NW_DEACT, /**< The network has forced a context deactivation. */
    RIL_URC_TYPE_ME_DEACT, /**< The mobile termination has forced a context deactivation. */
    RIL_URC_TYPE_NW_PDN_DEACT, /**< The network has deactivated a context. */
    RIL_URC_TYPE_ME_PDN_DEACT, /**< The mobile termination has deactivated a context. */
    RIL_URC_TYPE_NW_MODIFY, /**< The network has modified the context. */
    RIL_URC_TYPE_ME_MODIFY, /**< The mobile termination has modified the context. */
    RIL_URC_TYPE_REJECT, /**< A network request for context activation occurred when the MT was unable to report it to the TE. */
    RIL_URC_TYPE_NW_REACT, /**< The network has requested a context reactivation. */
    RIL_URC_TYPE_MAX
} ril_urc_cgev_type_enum;
/**
 * @}
 */

/** 
* @addtogroup ril_structure Structure
* @{
*/
/**
 * @brief This defines data structure for URC of "+CGEV".
 */
typedef struct {
    ril_urc_cgev_type_enum response_type; /**< The data type of URC "+CGEV". */
    union {
        char *classb; /**< Indicates the mode of operation. */
        int32_t cid; /**< Identifies the PDP context. */
        int32_t p_cid; /**< Indicates the context identifier allocated by MT for an MT initiated context of a second address type. */
        char *pdp_type; /**< Specifies the type of packet data protocol. */
    } response1; /**< The first parameter of the URC "+CGEV". */
    union {
        int32_t reason; /**< Indicates the reason to reject the context activation request for PDP type IPv4v6. */
        int32_t cid; /**< Identifies the PDP context. */
        char *pdp_addr; /** Identifies the MT in the address space applicable to the PDP. */
        int32_t change_reason; /**< Reason for change. */
    } response2; /**< The second parameter of the URC "+CGEV". */
    union {
        int32_t cid_other; /**< Indicates the context identifier allocated by MT for an MT initiated context of a second address type. */
        int32_t event_type; /**< Indicates whether this is an informational event. */
        int32_t cid; /**< Identifies the PDP context. */
    } response3; /**< The third parameter of the URC "+CGEV". */
} ril_cgev_event_reporting_urc_t;

/**
 * @brief This defines data structure for AT response of "+CGEREP".
 */
typedef struct {
    int32_t cmode; /**< Unsolicited result codes, +CGEV from MT to TE.*/
    int32_t bfr; /**< The effect on buffered codes when <mode> 1 or 2 is entered. */
} ril_packet_domain_event_reporting_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CGEREP". This command controls the processing of unsolicited result codes specified within this command.
 * @param[in] mode    AT command mode setting.
 * @param[in] cmode Enables or disables sending of unsolicited result codes, +CGEV from MT to TE.
 * @param[in] bfr Controls the effect on buffered codes when <mode> 1 or 2 is entered.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_packet_domain_event_reporting_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
ril_status_t ril_request_packet_domain_event_reporting(ril_request_mode_t mode,
        int32_t cmode,
        int32_t bfr,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT+CGREG */
/* URC: +CGREG */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CGREG".
 */
typedef struct {
    int32_t n; /**< The presentation of an unsolicited result code "+CGREG". */
    int32_t stat; /**< Indicates the GPRS registration status. */
    int32_t lac; /**< Location area code. */
    int32_t ci; /**< GERAN/UTRAN cell ID. */
    int32_t act; /**< The access technology of the serving cell. */
    int32_t rac; /**< Routing area code. */
    int32_t cause_type; /**< The type of reject_cause. */
    int32_t reject_cause; /**< The cause of the failed registration. */
    int32_t active_time; /**< The Active Time value (T3324) allocated to the UE in GERAN/UTRAN. */
    int32_t periodic_rau; /**< The extended periodic RAU value allocated to the UE in GERAN/UTRAN. */
    int32_t gprs_readytimer; /**< Indicates the GPRS READY timer value allocated to the UE in GERAN/UTRAN. */
} ril_gprs_network_registration_status_rsp_t;
/**
 * @brief This defines data structure for URC of "+CGREG".
 */
typedef struct {
    int32_t stat; /**< Indicates the GPRS registration status. */
    int32_t lac; /**< Location area code. */
    int32_t ci; /**< GERAN/UTRAN cell ID. */
    int32_t act; /**< The access technology of the serving cell. */
    int32_t rac; /**< Routing area code. */
    int32_t cause_type; /**< The type of reject_cause. */
    int32_t reject_cause; /**< The cause of the failed registration. */
    int32_t active_time; /**< The Active Time value (T3324) allocated to the UE in GERAN/UTRAN. */
    int32_t periodic_rau; /**< The extended periodic RAU value allocated to the UE in GERAN/UTRAN. */
    int32_t gprs_readytimer; /**< Indicates the GPRS READY timer value allocated to the UE in GERAN/UTRAN. */
} ril_gprs_network_registration_status_urc_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CGREG".
 * @param[in] mode    AT command mode setting.
 * @param[in] n The presentation of an unsolicited result code "+CGREG".
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_gprs_network_registration_status_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
ril_status_t ril_request_gprs_network_registration_status(ril_request_mode_t mode,
        int32_t n,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT+CEREG */
/* URC: +CEREG */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CEREG".
 */
typedef struct {
    int32_t n;    /**< Display mode cocnfiguration when "+CEREG" unsolicited result code is received. */
    int32_t stat;    /**< Indicates EPS registration status. */
    int32_t tac;    /**< Two bytes tracking area code in hexadecimal format. */
    int32_t ci;    /**< String type, four byte GERAN/UTRAN/E-TRAN cell id in hexadecimal format. */
    int32_t act;    /**< Access technology of the registered network. */
    int32_t rac;    /**< String type, one byte routing area code in hexadecimal format. */
    int32_t cause_type;    /**< Integer type, indicates the type of the field "reject cause". */
    int32_t reject_cause;    /**< Integer type, contains the cause of the failed registration. */
    int32_t active_time;    /**< One byte in an 8 bit format, indicates the Active Time value allocated to the UE in E-UTRAN. */
    int32_t periodic_tau;    /**< One byte in an 8 bit format, indicates the extended periodic TAU value allocated to the UE in E-UTRAN. */
} ril_eps_network_registration_status_rsp_t;
/**
 * @brief This defines data structure for URC of "+CEREG".
 */
typedef struct {
    int32_t stat;    /**< Indicates EPS registration status. */
    int32_t tac;    /**< String type, two bytes location area code in hexadecimal format. */
    int32_t ci;    /**< String type, four byte GERAN/UTRAN/E-TRAN cell ID in hexadecimal format. */
    int32_t act;    /**< Access technology of the registered network. */
    int32_t rac;    /**< String type, one byte routing area code in hexadecimal format. */
    int32_t cause_type;    /**< Integer type, indicates the type of the field "reject cause". */
    int32_t reject_cause;    /**< Integer type, contains the cause of the failed registration. */
    int32_t active_time;    /**< One byte in an 8 bit format, indicates the Active Time value allocated to the UE in E-UTRAN. */
    int32_t periodic_tau;    /**< One byte in an 8 bit format, indicates the extended periodic TAU value allocated to the UE in E-UTRAN. */
} ril_eps_network_registration_status_urc_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CEREG". This command controls the processing of unsolicited result codes specified within this command.
 * @param[in] mode    AT command mode setting.
 * @param[in] n    Display mode for the unsolicited result codes, +CEREG, from MT to TE.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_eps_network_registration_status_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
ril_status_t ril_request_eps_network_registration_status(ril_request_mode_t mode,
        int32_t n,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT+CGCONTRDP */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for PDP context dynamic parameters.
 */
typedef struct {
    int32_t cid; /**< Identifies of the PDP context. */
    int32_t bearer_id; /**< The identifier for the bearer. */
    char *apn; /**< The logical name to select the GGSN or the external packet data network. */
    char *local_addr_and_subnet_mask; /**< The IP address and subnet mask of the MT. */
    char *gw_addr; /**< The Gateway Address of the MT. */
    char *dns_prim_addr; /**< The IP address of the primary DNS server. */
    char *dns_sec_addr; /**< The IP address of the secondary DNS server. */
    char *pcscf_prim_addr; /**< The IP address of the primary P-CSCF server. */
    char *pcscf_sec_addr; /**< The IP address of the secondary P-CSCF server. */
    int32_t im_cn_signalling_flag; /**< Whether the PDP context is for IM CN subsystem-related signalling only. */
    int32_t lipa_indication; /**< Indicates the PDP context provides connectivity using a LIPA PDN connection. */
    int32_t ipv4_mtu; /**< Indicates the IPv4 MTU size in octets. */
    int32_t nonip_mtu; /**< Indicates the Non-IP MTU size in octects. */
    int32_t serving_plmn_rate_control_value;    /**< Indicates the maximum number of uplink messages the UE is allowed to send in a 6-minute interval. */
} ril_pdp_context_entry_struct_t; /**<  */
/**
 * @brief This defines data structure for AT response of "+CGCONTRDP".
 */
typedef struct {
    ril_pdp_context_entry_struct_t *pdp_context; /**< The list of PDP context dynamic parameters. */
    int32_t array_num; /**< The number of entries in the list. */
} ril_pdp_context_read_dynamic_parameters_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CGCONTRDP".
 * @param[in] mode    AT command mode setting.
 * @param[in] cid    Defines the PDP context ID.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_pdp_context_read_dynamic_parameters_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
ril_status_t ril_request_pdp_context_read_dynamic_parameters(ril_request_mode_t mode,
        int32_t cid,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT+CGDEL */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CGDEL".
 */
typedef struct {
    int32_t *cid_array; /**< The list of identifiers for the PDP context. */
    int32_t array_num; /**< The number of entries in the list. */
} ril_delete_non_active_pdp_contexts_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CGDEL".
 * @param[in] mode    AT command mode setting.
 * @param[in] cid     Defines the PDP context ID.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_delete_non_active_pdp_contexts_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
ril_status_t ril_request_delete_non_active_pdp_contexts(ril_request_mode_t mode,
        int32_t cid,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT+CGAUTH */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for PDP context authentication parameters.
 */
typedef struct {
    int32_t cid; /**< Identifies the PDP context. */
    int32_t auth_prot; /**< Authentication protocol used for this PDP context. */
    char *userid; /**< User name to access the IP network. */
    char *password; /**< Password to access the IP network. */
} ril_authentication_pdp_struct_t;
/**
 * @brief This defines data structure for AT response of "+CGAUTH".
 */
typedef struct {
    ril_authentication_pdp_struct_t *cntx_array;  /**< The list of PDP context authentication parameters. */
    int32_t array_num; /**< The number of entries in the list. */
} ril_define_pdp_context_authentication_parameters_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CGAUTH".
 * @param[in] mode    AT command mode setting.
 * @param[in] cid    Defines the PDP context ID.
 * @param[in] auth_port Authentication protocol used for this PDP context.
 * @param[in] userid User name to access the IP network.
 * @param[in] password Password to access the IP network.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_define_pdp_context_authentication_parameters_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
ril_status_t ril_request_define_pdp_context_authentication_parameters(ril_request_mode_t mode,
        int32_t cid,
        int32_t auth_port,
        char *userid,
        char *password,
        ril_cmd_response_callback_t callback,
        void *user_data,
        int32_t channel_id);


/* AT command: AT+CIPCA */

/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CIPCA".
 */
typedef struct {
    int32_t n; /**< Establish an initial PDP context automatically or not. */
    int32_t attach_without_pdn;    /**< Wether EPS attach with PDN connection. */
} ril_initial_pdp_context_activation_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CIPCA".
 * @param[in] mode    AT command mode setting.
 * @param[in] n     Establish an initial PDP context automatically or not.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_initial_pdp_context_activation_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
ril_status_t ril_request_initial_pdp_context_activation(ril_request_mode_t mode,
        int32_t n,
        int32_t attach_without_pdn,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT+CPSMS */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CPSMS".
 */
typedef struct {
    int32_t mode;  /**< Enable or disable the use of PSM in the UE. */
    char *req_prdc_rau;  /**< Request to allocate an extended periodic RAU value for the UE in GERAN/UTRAN. */
    char *req_gprs_rdy_tmr;  /**< Request to allocate a GPRS READY timer value for the UE in GERAN/UTRAN. */
    char *req_prdc_tau;  /**< Request to allocate an extended periodic TAU value for the UE in E-UTRAN. */
    char *req_act_time;  /**< Request to allocate an Active Time value for the UE. */
} ril_power_saving_mode_setting_rsp_t, ril_power_saving_mode_setting_req_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CPSMS".
 * @param[in] mode    AT command mode setting.
 * @param[in] req    The UE power saving mode (PSM) settings.
                              For more information about sub-parameters data structure, please refer to #ril_power_saving_mode_setting_req_t.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_power_saving_mode_setting_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
ril_status_t ril_request_power_saving_mode_setting(ril_request_mode_t mode,
        ril_power_saving_mode_setting_req_t *req,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT+CCIOTOPT */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CCIOTOPT".
 */
typedef struct {
    int32_t n; /**< Enables or disables reporting of unsolicited result code +CCIOTOPTI. */
    int32_t supported_UE_opt; /**< Indicates the UE's support for CIoT EPS optimizations. */
    int32_t preferred_UE_opt; /**< Indicates the UE's preference for CIoT EPS optimizations. */
} ril_ciot_optimisation_configuration_rsp_t;
/**
 * @brief This defines data structure for URC of "+CCIOTOPT".
 */
typedef struct {
    int32_t supported_network_opt; /**< Indicates the Network support for CIoT EPS optimizations. */
} ril_ciot_optimisation_configuration_urc_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CCIOTOPT".
 * @param[in] mode    AT command mode setting.
 * @param[in] n Enables or disables reporting of unsolicited result code +CCIOTOPTI.
 * @param[in] supported_UE_opt    Indicates the UE's support for CIoT EPS optimizations.
 * @param[in] preferred_UE_opt    Indicates the UE's preference for CIoT EPS optimizations.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_ciot_optimisation_configuration_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
ril_status_t ril_request_ciot_optimisation_configuration(ril_request_mode_t mode,
        int32_t n,
        int32_t supported_UE_opt,
        int32_t preferred_UE_opt,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT+CEDRXS */
/* AT URC: +CEDRXP */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for eDRX setting.
 */
typedef struct {
    int32_t act_type; /**< The type of access technology. */
    char *requested_eDRX_value; /**< The eDRX value. */
} ril_eDRX_setting_struct_t;
/**
 * @brief This defines data structure for AT response of "+CEDRXS".
 */
typedef struct {
    int32_t array_num; /**< The number of entries in the array. */
    ril_eDRX_setting_struct_t *eDRX_setting; /**< A pointer to the array for the eDRX settings, please refer to #ril_eDRX_setting_struct_t, for sub-parameter of each element . */
} ril_eDRX_setting_rsp_t;
/**
 * @brief This defines data structure for URC of "+CEDRXP".
 */
typedef struct {
    int32_t act_type; /**< The type of access technology. */
    char *requested_eDRX_value; /**< The eDRX value. */
    char *nw_provided_eDRX_value; /**<  The eDRX value. */
    char *paging_time_window; /**< The paging time window. */
} ril_eDRX_setting_urc_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CEDRXS".
 * @param[in] mode    AT command mode setting.
 * @param[in] eDRX_mode    Enable or disable the use of eDRX in the UE.
 * @param[in] act_type    Indicates the type of access technology.
 * @param[in] requested_eDRX_value    String type, half a byte in a 4-bit format. The eDRX value refers to bit 4 to 1 of octet 3 of the extended DRX parameter.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_eDRX_setting_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
ril_status_t ril_request_eDRX_setting(ril_request_mode_t mode,
                                      int32_t eDRX_mode,
                                      int32_t act_type,
                                      char *requested_eDRX_value,
                                      ril_cmd_response_callback_t callback,
                                      void *user_data);


/* AT command: AT+CEDRXRDP */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CEDRXRDP".
 */
typedef struct {
    int32_t act_type; /**<  The type of access technology.  */
    char *requested_eDRX_value; /**< The eDRX value. */
    char *nw_provided_eDRX_value; /**< The eDRX value. */
    char *paging_time_window; /**< The paging time window. */
} ril_read_eDRX_dynamic_parameters_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CEDRXRDP".
 * @param[in] mode    AT command mode setting.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_read_eDRX_dynamic_parameters_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
ril_status_t ril_request_read_eDRX_dynamic_parameters(ril_request_mode_t mode,
        ril_cmd_response_callback_t callblack,
        void *user_data);



/* AT command: AT+CGAPNRC */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CGAPNRC".
 */
typedef struct {
    int32_t cid; /**< Identifies the PDP context. */
    int32_t additional_exception_reports; /**<  Whether additional exception reports are sent. */
    int32_t uplink_time_unit; /**< The time unit for maximum uplink rate; unrestricted, minutes, hour, day and week. */
    int32_t maximum_uplink_rate; /**< The unit of maximum uplink rate. */
} ril_APN_rate_control_struct_t;
/**
 * @brief This defines data structure for AT response of "+CGAPNRC".
 */
typedef struct {
    int32_t array_num;  /**< The number of entries in the list. */
    ril_APN_rate_control_struct_t *param_list; /**< A pointer to the array for APN rate control list. */
} ril_APN_rate_control_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CGAPNRC".
 * @param[in] mode    AT command mode setting.
 * @param[in] cid    Defines the PDP context ID.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_APN_rate_control_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
ril_status_t ril_request_APN_rate_control(ril_request_mode_t mode,
        int32_t cid,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT cmd: AT+CSCON */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for URC of "+CSCON".
 */
typedef struct {
    int32_t mode;    /**< Indicates whether unsolicited result code reporting enabled. */
} ril_signaling_connection_status_urc_t;
/**
 * @brief This defines data structure for AT response of "+CSCON".
 */
typedef struct {
    int32_t n;    /**< Indicates whether unsolicited result code reporting enabled. */
    int32_t mode;    /**< Indicates signaling connection status. */
} ril_signaling_connection_status_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CSCON".
 * @param[in] mode    AT command mode setting.
 * @param[in] n    Enables unsolicited result code reporting. 
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_signaling_connection_status_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
ril_status_t ril_request_signaling_connection_status(ril_request_mode_t mode,
        int32_t n,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT cmd: AT+CCHO */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for response of "+CCHO".
 */
typedef struct {
    int32_t sessionid;    /**< Indicates a session id to be used a specific application on the smart card using logical channels mechanism. */
} ril_open_uicc_logical_channel_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CCHO".
 * @param[in] mode    AT command mode setting.
 * @param[in] dfname    String type in hexadecimal character format, all selectable applications in the UICC are referenced by a DF name coded on 1 to 16 bytes. 
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_open_uicc_logical_channel_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
ril_status_t ril_request_open_uicc_logical_channel(ril_request_mode_t mode,
        char *dfname,
        ril_cmd_response_callback_t callback,
        void *user_data);



/* AT cmd: AT+CCHC */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for response of "+CCHC".
 */
typedef struct {
    char *response_str;    /**< hope to receive "CCHC" if performed successully. */
} ril_close_uicc_logical_channel_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CCHC".
 * @param[in] mode    AT command mode setting.
 * @param[in] sessionid    The session used to target a specific application on the smart card using logical channels mechanism. 
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_close_uicc_logical_channel_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
ril_status_t ril_request_close_uicc_logical_channel(ril_request_mode_t mode,
        int32_t sessionid,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT cmd: AT+CGLA */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for response of "+CGLA".
 */
typedef struct {
    int32_t length;    /**< The length of the characters that are sent to TE in #command or #response. */
    char *response;    /**< response to the command passed on by the UICC to the MT in the format as described in 3GPP TS 31.101. */
} ril_generic_uicc_logical_channel_access_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CGLA".
 * @param[in] mode    AT command mode setting.
 * @param[in] sessionid    This is the identifier of the session used to send the APDU commands to the UICC.
 * @param[in] length    The length of the characters that are sent to TE in #command.
 * @param[in] command    Command passed on by the MT to the UICC in the format as described in 3GPP TS 31.101.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_generic_uicc_logical_channel_access_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
ril_status_t ril_request_generic_uicc_logical_channel_access(ril_request_mode_t mode,
        int32_t sessionid,
        int32_t length,
        char *command,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT cmd: AT+CRCES */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CRCES".
 */
typedef struct {
    int32_t act;    /**< Indicates access technology of the serving cell. */
    int32_t ce_level;    /**< Indicates Coverage Enhancement (CE) level of the MT in serving cell. */
} ril_read_coverage_enhancement_status_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CRCES".
 * @param[in] mode    AT command mode setting.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_read_coverage_enhancement_status_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
ril_status_t ril_request_read_coverage_enhancement_status(ril_request_mode_t mode,
        ril_cmd_response_callback_t callback,
        void *user_data);


#if defined(__RIL_SMS_COMMAND_SUPPORT__)
/*************************************************
 *                 Technical specification for 3GPP v27.005
 *************************************************/
/* AT command: AT+CSMS */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CSMS".
 */
typedef struct {
    int32_t service;    /**< Current service. */
    int32_t mt;    /**< Support for MT messages. */
    int32_t mo;    /**< Support for mobile originated messages. */
    int32_t bm;    /**< Support for broadcast type messages. */
} ril_select_message_service_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CSMS".
 * @param[in] mode    AT command mode setting.  
 * @param[in] service    Subparameter for this AT command, specifies release assistance information.  
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_select_message_service_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_select_message_service(ril_request_mode_t mode,
        int32_t service,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT+CPMS */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CPMS".
 */
typedef struct {
    char *mem1;    /**< Memory location to read and delete messages from. */
    int32_t used1;    /**< The number of messages in #mem1. */
    int32_t total1;    /**< The available storage allocated for messages in #mem1. */
    char *mem2;    /**< Memory location to write messages or send out messages from. */
    int32_t used2;    /**< The number of messages in #mem2. */
    int32_t total2;    /**< The available storage allocated for messages in #mem2. */
    char *mem3;    /**< Memory location to place the received messages. */
    int32_t used3;    /**< The number of messages in #mem3. */
    int32_t total3;    /**< The available storage allocated for messages in #mem3. */
} ril_preferred_message_storage_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CPMS".
 * @param[in] mode    AT command mode setting.  
 * @param[in] mem1    Subparameter for this AT command, to specifies a storage to read and delete messages from. 
 * @param[in] mem2    Subparameter for this AT command, to specifies a storage to write or send messages. 
 * @param[in] mem3    Subparameter for this AT command, to specifies the storage for received messages.  
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_preferred_message_storage_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_preferred_message_storage(ril_request_mode_t mode,
        char *mem1,
        char *mem2,
        char *mem3,
        ril_cmd_response_callback_t callback,
        void *user_data);



/* AT command: AT+CMGF*/
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CMGF".
 */
typedef struct {
    int32_t message_mode;    /**< 0, PDU mode and 1, text mode. */
} ril_message_format_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CMGF".
 * @param[in] mode    AT command mode setting.  
 * @param[in] message_mode    Subparameter for this AT command, to set PDU mode or text mode. 
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_message_format_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_message_format(ril_request_mode_t mode,
        int32_t message_mode,
        ril_cmd_response_callback_t callback,
        void *user_data);



/* AT command: AT+CSCA */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CSCA".
 */
typedef struct {
    char *sca;    /**< 3GPP 24.011 RP SC address Address-Value field in string format. */
    int32_t tosca;    /**< Service center address format 3GPP 24.011 RP SC address Type-of-Address octet in integer format. */
} ril_service_centre_address_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CSCA".
 * @param[in] mode    AT command mode setting.  
 * @param[in] sca    Subparameter for this AT command, to set 3GPP 24.011 RP SC address Address-Value field in string format. 
 * @param[in] tosca    Subparameter for this AT command, to set Service center address format 3GPP 24.011 RP SC address Type-of-Address octet in integer format.  
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_service_centre_address_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_service_centre_address(ril_request_mode_t mode,
        char *sca,
        int32_t tosca,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT+CSMP */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CSMP".
 */
typedef struct {
    int32_t fo;    /**< First octet of 3GPP 23.040 SMS-DELIVER, SMS-SUBMIT in integer format. */
    int32_t vp;    /**< 3GPP 23.040 TP-Validity-Period in integer format (default 167). */
    int32_t pid;    /**< 3GPP 23.040 TP-Protocoal-Identifier in integer format (default 0). */
    int32_t dcs;    /**< 3GPP 23.038 SMS data coding scheme in integer format (default 0 i.e. 7-bit coding). */
} ril_set_text_mode_parameters_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CSMP".
 * @param[in] mode    AT command mode setting.  
 * @param[in] fo    Subparameter for this AT command, to set first octet of 3GPP 23.040 SMS-DELIVER, SMS-SUBMIT in integer format. 
 * @param[in] vp    Subparameter for this AT command, to set 3GPP 23.040 TP-Validity-Period in integer format. 
 * @param[in] pid  Subparameter for this AT command, to set 3GPP 23.040 TP-Protocoal-Identifier in integer format.
 * @param[in] dcs    Subparameter for this AT command, to set 3GPP 23.038 SMS data coding scheme in integer format.  
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_set_text_mode_parameters_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_set_text_mode_parameters(ril_request_mode_t mode,
        int32_t fo,
        int32_t vp,
        int32_t pid,
        int32_t dcs,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT+CSDH */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CSDH".
 */
typedef struct {
    int32_t show;    /**< Indicates if the header values defined in commands +CSCA and +CSMP are hidden or not. */
} ril_show_text_mode_parameters_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CSDH".
 * @param[in] mode    AT command mode setting.  
 * @param[in] show    Subparameter for this AT command, set this parameter if the header values defined in commands +CSCA and +CSMP are hidden or not.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_show_text_mode_parameters_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_show_text_mode_parameters(ril_request_mode_t mode,
        int32_t show,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT+CSAS */
/** 
 * @brief This function sends an AT command "+CSAS".
 * @param[in] mode    AT command mode setting.  
 * @param[in] profile    Subparameter for this AT command, specifies the manufacturer specific profile number to store settings.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_save_settings(ril_request_mode_t mode,
        int32_t profile,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT+CRES */
/** 
 * @brief This function sends an AT command "+CRES".
 * @param[in] mode    AT command mode setting.  
 * @param[in] profile    Subparameter for this AT command, specifies the manufacturer specific profile number to store settings.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_restore_settings(ril_request_mode_t mode,
        int32_t profile,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT+CNMI */
/* AT URC: +CMTI */
/* AT URC: +CMT */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure to send parameters and AT response of "+CNMI".
 */
typedef struct {
    int32_t mode;    /**< Indication mode for new messages. */
    int32_t mt;    /**< Indicates the rules to store received SMS depending on its data coding scheme. */
    int32_t bm;    /**< Indicates the rules to store received CBMs depending on its data coding scheme. */
    int32_t ds;    /**< Indicates the rules to route SMS-STATUS-REPORTs. */
    int32_t bfr;    /**< Indicates if TA buffer of unsolicited result codes defined within this command is flushed or cleared. */
} ril_new_message_indication_rsp_t, ril_new_message_indication_param_t;
/**
 * @}
 */

/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for URC of "+CMTI".
 */
typedef struct {
    char *mem;    /**< Memory location to place the received messages. */
    int32_t index;    /**< Indicates storage location. */
} ril_new_message_indication_txt_urc_t;
/**
 * @}
 */

/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for URC of "+CMT".
 */
typedef struct {
    int32_t length;    /**< Indicates the length of sms pdu content */
    char *pdu;    /**< Indicates the sms content as pdu format. */
} ril_new_message_indication_pdu_urc_t;
/**
 * @}
 */


/** 
 * @brief This function sends an AT command "+CNMI".
 * @param[in] mode    AT command mode setting.  
 * @param[in] param    A pointer to the sent parameters, for more information, please refer to #ril_new_message_indication_param_t. 
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_new_message_indication_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_new_message_indication(ril_request_mode_t mode,
        ril_new_message_indication_param_t *param,
        ril_cmd_response_callback_t callback,
        void *user_data);

// PDU mode
/* AT command: AT+CMGL */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure as an entry for AT response of "+CMGL".
 */
typedef struct {
    int32_t index;    /**< Indicates a location in the associated memory. */
    int32_t stat;    /**< Indicates message types, including received unread messages/received read messages/stored unsent messages/stored sent messages/all messages. */
    char *alpha;    /**< Alphanumeric representation of "da" or "oa" corresonding to the entry found in MT phonebook. */
    int32_t length;    /**< Indicates the length of the messages in the text mode or the length of the actual TP data unit in octets in PDU mode. */
    char *pdu;    /* A pointer to the PDU data in hexadecimal format. */
} ril_message_entry_pdu_desc_t;
/**
 * @brief This defines data structure for AT response of "+CMGL" in PDU mode.
 */
typedef struct {
    uint32_t message_num;    /**< Number of parameters are included in the AT response. */
    ril_message_entry_pdu_desc_t *entry_list;    /**< A pointer to the parameter list, for more information about each entry, please refer to #ril_message_entry_pdu_desc_t. */
} ril_list_messages_pdu_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CMGL" in PDU mode.
 * @param[in] mode    AT command mode setting.  
 * @param[in] stat    Subparameter for this AT command, specifies message types. 
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_list_messages_pdu_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_list_messages_pdu(ril_request_mode_t mode,
        int32_t stat,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT+CMGR */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CMGR" in PDU mode.
 */
typedef struct {
    int32_t stat;    /**< Specifies which type of messages need to read. */
    int32_t alpha;    /**< Alphanumeric representation of "da" or "oa" corresonding to the entry found in MT phonebook. */
    int32_t length;    /**< Integer type, the value indicating in the text mode the length of the message body in characters or in PDU mode, the length of the actual TP data unit in octets. */
    char *pdu;    /* Point to PDU data in hexadecimal format. */
} ril_read_message_pdu_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CMGR" if PDU mode.
 * @param[in] mode    AT command mode setting.  
 * @param[in] index    Subparameter for this AT command, specifies the message index. 
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_read_message_pdu_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_read_message_pdu(ril_request_mode_t mode,
        int32_t index,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT+CMGS */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CMGS" in PDU mode.
 */
typedef struct {
    int32_t mr;    /**< Indicates 3GPP 23.040 TP-Message-Reference in integer format. */
    char *ackpdu;    /**< PDU format data for acknowledgement. */
} ril_send_message_pdu_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CMGS" in PDU mode.
 * @param[in] mode    AT command mode setting.  
 * @param[in] length    Subparameter for this AT command, to indicate the length of actual TP data unit in octets.  
 * @param[in] pdu    Subparameter for this AT command, point to PDU data. 
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_send_message_pdu_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_send_message_pdu(ril_request_mode_t mode,
        int32_t length,
        char *pdu,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT+CMGW */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CMGW" in PDU mode.
 */
typedef struct {
    int32_t index;    /**< The message index in its associated memory. */
} ril_write_message_pdu_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CMGW" in PDU mode.
 * @param[in] mode    AT command mode setting.  
 * @param[in] length    Subparameter for this AT command, indicates the length of actual TP data unit in octets. 
 * @param[in] stat    Subparameter for this AT command, indicate the message status.  
 * @param[in] pdu    Subparameter for this AT command, a pointer to the PDU data. 
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_write_message_pdu_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_write_message_pdu(ril_request_mode_t mode,
        int32_t length,
        int32_t stat,
        char *pdu,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT+CMGD */
/** 
 * @brief This function sends an AT command "+CMGD" in PDU mode.
 * @param[in] mode    AT command mode setting.  
 * @param[in] index    Subparameter for this AT command, indicates the index of messages in integer format. 
 * @param[in] deflag    Subparameter for this AT command, indicate a message to delete.  
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_delete_message(ril_request_mode_t mode,
        int32_t index,
        int32_t delflag,
        ril_cmd_response_callback_t callback,
        void *user_data);



/* AT command: AT+CMGC */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CMGC" in PDU mode.
 */
typedef struct {
    int32_t mr;    /**< Indicates 3GPP 23.040 TP-Message-Reference in integer format. */
    char *ackpdu;    /**< PDU format data for acknowledge. */
} ril_send_command_pdu_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CMGC" in PDU mode.
 * @param[in] mode    AT command mode setting.  
 * @param[in] length    Subparameter for this AT command, indicates the length of actual TP data unit in octets. 
 * @param[in] pdu    Subparameter for this AT command, a pointer to the PDU data. 
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_send_command_pdu_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_send_command_pdu(ril_request_mode_t mode,
        int32_t length,
        char *pdu,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT+CNMA */
/** 
 * @brief This function sends an AT command "+CNMA" in PDU mode.
 * @param[in] mode    AT command mode setting. 
 * @param[in] n    Subparameter for this AT command, indicates acknowledgement types, postitive/negative/similar in text mode. 
 * @param[in] length    Subparameter for this AT command, indicates the length of optional PDU message. 
 * @param[in] pdu    Subparameter for this AT command, a pointer to the optional PDU data. 
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_new_message_ack_pdu(ril_request_mode_t mode,
        int32_t n,
        int32_t length,
        char *pdu,
        ril_cmd_response_callback_t callback,
        void *user_data);

/* AT command: AT+CMSS */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CMSS" in PDU mode.
 */
typedef struct {
    int32_t mr;    /**< Indicates 3GPP 23.040 TP-Message-Reference in integer format. */
    char *ackpdu;    /**< PDU format data for acknowledgement. */
} ril_send_message_from_storage_pdu_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CMSS" in PDU mode.
 * @param[in] mode    AT command mode setting.  
 * @param[in] index    Subparameter for this AT command, indicates the messages index in its associated storage.
 * @param[in] da    Subparameter for this AT command, 3GPP 23.040 TP-Destination-Address Address-Value field in string format.  
 * @param[in] toda    Subparameter for this AT command, 3GPP 24.011 TP-Destination-Address Type-of-Address octet in integer format. 
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_send_message_from_storage_pdu_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_send_message_from_storage_pdu(ril_request_mode_t mode,
        int32_t index,
        char *da,
        int32_t toda,
        ril_cmd_response_callback_t callback,
        void *user_data);


// text mode
/* AT command: AT+CMGL */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CMGL in TEXT mode.
 */
typedef struct {
    int32_t index;    /**< The location in its associated memory. */
    char *stat;    /**< Indicates the message state, e.g. received unread, received read, stored unsent, stored sent messages. */
    union {
        char *oa;    /**< 3GPP 23.040 TP-Originating-Address Address-Value field in string format. */
        char *da;    /**< 3GPP 23.040 TP-Destination-Address Address-Value field in string format. */
    } str_addr;    /**< Indicates address, alternatively originating address or destination address. */
    char *alpha;    /**< String type alphanumeric representation of "oa" or "da" found in MT phonebook. */
    char *scts;    /**< Service center time stamp. String format: "yy/MM/dd,hh:mm:ss-/+zz" (Year/Month/Dat,Hour:Second:+/Timezone). */
    union {
        int32_t tooa;    /**< 3GPP 23.040 TP-Originating-Address Type-of-Address octet in integer format. */
        int32_t toda;    /**< 3GPP 23.040 TP-Destination-Address Type-of-Address octet in integer format. */
    } int_addr;    /**< Indicates type of address, alternatively "tooa" or "toda". */
    int32_t length;    /**< The message length (integer) in the text mode or the length of the actual TP in octetes in PDU mode.*/
    char *data;    /**< 3GPP 23.040 TP-User-Data in text mode responses. */
    int32_t fo;    /**< The first byte of SMS-DELIVER, SMS-SUBMIT or SMS-REPORT in integer format. */
    int32_t mr;    /**< Message reference, integer format. */
    char *ra;    /**< Recipient address, string format. */
    int32_t tora;    /**< Type of address of "ra". */
    char *dt;    /**< Discharge time. */
    int32_t st;    /**< Status of SMS-STATUS-REPORT, integer format. */
    int32_t ct;
    int32_t sn;    /**< 3GPP TS 23.041 CBM Serial Number in integer format. */
    int32_t mid;    /**< 3GPP 23.041 CBM Message Identifier in integer format. */
    int32_t page;    /**< 3GPP TS 23.041 CBM Page Parameter bits 4-7 in integer format. */
    int32_t pages;    /**< 3GPP TS 23.041 CBM Page Parameter bits 0-3 in integer format. */
} ril_message_entry_txt_desc_t;

/**
 * @brief This defines data structure for AT response of "+CMGL" in TEXT mode.
 */
typedef struct {
    uint32_t message_num;    /**< Indicates the number of parameters included in AT response. */
    ril_message_entry_txt_desc_t *entry_list;    /**< A pointer to the parameter list, for more information about each entry, please refer to #ril_message_entry_txt_desc_t. */
} ril_list_messages_txt_rsp_t;

/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CMGL" in TEXT mode.
 * @param[in] mode    AT command mode setting.  
 * @param[in] stat    Subparameter for this AT command, to indicate the type of messages in selected storage, e.g. received unread, received read, stored unsent, stored sent messages.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_list_messages_txt_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_list_messages_txt(ril_request_mode_t mode,
        char *stat,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT+CMGR */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CMGR in TEXT mode.
 */
typedef struct {
    char *stat;    /**< Indicates the message state, e.g. received unread, received read, stored unsent, stored sent messages. */
    char *oa;    /**< 3GPP 23.040 TP-Originating-Address Address-Value field in string format. */
    char *alpha;    /**< String type alphanumeric represetation of "oa" or "da" in MT phonebook. */
    char *scts;    /**< Service center time stamp. String format: "yy/MM/dd,hh:mm:ss-/+zz" (Year/Month/Dat,Hour:Second:+/Timezone). */
    int32_t tooa;    /**< 3GPP 23.040 TP-Originating-Address Type-of-Address octet in integer format. */
    int32_t fo;    /**< The first byte of SMS-DELIVER, SMS-SUBMIT or SMS-REPORT in integer format. */
    int32_t pid;    /**< 3GPP 23.040 TP-Protocol-Identifier in integer format. */
    int32_t dcs;    /**< Depending on the command or result: 3GPP 23.038 SMS Data Coding Scheme, or Cell Broadcast Data Coding Scheme in integer format. */
    char *sca;    /**< SGPP 24.011 RP SC address Address-Value field in string format. */
    int32_t tosca;    /**< 3GPP 24.011 RP SC address Type-of-Address octet in integer format. */
    int32_t length;    /**< The messages length (integer) in the text mode or the length of the actual TP in octetes in PDU mode.*/
    char *data;    /**< 3GPP 23.040 TP-User-Data in text mode responses. */
    char *da;    /**< 3GPP 23.040 TP-Destination-Address Address-Value field in string format. */
    int32_t toda;    /**< 3GPP 23.040 TP-Destination-Address Type-of-Address octet in integer format. */
    int32_t vp;    /**< Depending on SMS-SUBMIT "fo" setting, 3GPP 23.040 TP-Validity-Period either in integer format. */
    int32_t mr;    /**< Message reference. */
    char *ra;    /**< Recipient address. */
    int32_t tora;    /**< Type of address of "ra". */
    char *dt;    /**< Discharge time. */
    int32_t st;    /**< Status of SMS-STATUS-REPORT, integer format. */
    int32_t ct;
    int32_t mn;
    char *cdata;    
    int32_t sn;    /**< 3GPP TS 23.041 CBM Serial Number in integer format. */
    int32_t mid;    /**< 3GPP 23.041 CBM Message Identifier in integer format. */
    int32_t page;    /**< 3GPP TS 23.041 CBM Page Parameter bits 4-7 in integer format. */
    int32_t pages;    /**< 3GPP TS 23.041 CBM Page Parameter bits 0-3 in integer format. */
} ril_read_message_txt_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CMGR" in TEXT mode.
 * @param[in] mode    AT command mode setting.  
 * @param[in] index    Subparameter for this AT command, indicates a location in its associated memory.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_read_message_txt_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_read_message_txt(ril_request_mode_t mode,
        int32_t index,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT+CMGS */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CMGS in TEXT mode.
 */
typedef struct {
    int32_t mr;    /**< Message reference. */
    char *scts;    /**< Service center time stamp. String format: "yy/MM/dd,hh:mm:ss-/+zz" (Year/Month/Dat,Hour:Second:+/Timezone). */
} ril_send_message_txt_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CMGS" in TEXT mode.
 * @param[in] mode    AT command mode setting.  
 * @param[in] da    Subparameter for this AT command, indicates destination address of the message. 
 * @param[in] toda    Subparameter for this AT command, indicates the address type of the destination address for the message. 
 * @param[in] text    Subparameter for this AT command, indicates the content of the message.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_send_message_txt_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_send_message_txt(ril_request_mode_t mode,
        char *da,
        int32_t toda,
        char *text,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT+CMGW */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CMGW in TEXT mode.
 */
typedef struct {
    int32_t index;    /**< The message index in its associated storage. */
} ril_write_message_txt_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CMGW" in TEXT mode.
 * @param[in] mode    AT command mode setting.  
 * @param[in] addr    Subparameter for this AT command, indicates the address of the message, originating address or destination address.
 * @param[in] to_addr    Subparameter for this AT command, indicates the type of the address for the message. 
 * @param[in] stat    Subparameter for this AT command, indicates the status of the message when stored in memory.
 * @param[in] text    Subparameter for this AT command, indicates the content of the message.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_write_message_txt_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_write_message_txt(ril_request_mode_t mode,
        char *addr,
        int32_t to_addr,
        char *stat,
        char *text,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT+CMGC */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CMGC in TEXT mode.
 */
typedef struct {
    int32_t mr;    /**< Message reference. */
    char *scts;    /**< Service center time stamp. String format: "yy/MM/dd,hh:mm:ss-/+zz" (Year/Month/Dat,Hour:Second:+/Timezone). */
} ril_send_command_txt_rsp_t;
/**
 * @brief This defines data structure to send parameters of "+CMGC in TEXT mode.
 */
typedef struct {
    int32_t fo;    /**< The first octet of 3GPP 23.040 SMS-COMMAND in integer format. */
    int32_t ct;    /**< The first octet of 3GPP 23.040 TP-Command-Type in integer format. */
    int32_t pid;    /**< 3GPP 23.040 TP-Protocol-Identifier in integer format. */    
    int32_t mn;    /**< 3GPP 23.040 TP-Message-Number in integer format. */
    char *da;    /**< 3GPP 23.040 TP-Destination-Address Address-Value field in string format. */
    int32_t toda;    /**< 3GPP 23.040 TP-Destination-Address Type-of-Address octet in integer format. */
    char *cdata;    /**< Indicates the content of SMS command. */
} ril_send_command_param_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CMGC" in TEXT mode.
 * @param[in] mode    AT command mode setting.  
 * @param[in] cmd_param    A pointer to the sent subparameter structure for this AT command, for more information, please refer to #ril_send_command_param_t.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_send_command_txt_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_send_command_txt(ril_request_mode_t mode,
        ril_send_command_param_t *cmd_param,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT+CNMA */
/** 
 * @brief This function sends an AT command "+CNMA" in TEXT mode.
 * @param[in] mode    AT command mode setting.  
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_new_message_ack_txt(ril_request_mode_t mode,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT+CMSS */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "+CMSS in TEXT mode.
 */
typedef struct {
    int32_t mr;    /**< Message reference. */
    char *scts;    /**< Service center time stamp. String format: "yy/MM/dd,hh:mm:ss-/+zz" (Year/Month/Dat,Hour:Second:+/Timezone). */
} ril_send_message_from_storage_txt_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "+CMSS" in TEXT mode.
 * @param[in] mode    AT command mode setting.  
 * @param[in] index    Subparameter for this AT command, indicates a location in its associated memory.
 * @param[in] da    Subparameter for this AT command, indicates the destination address of message, 3GPP 23.040 TP-Destination-Address Address-Value field in string format.
 * @param[in] toda    Subparameter for this AT command, indicates the type of destination address for the message, 3GPP 23.040 TP-Destination-Address Type-of-Address octet in integer format..
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_send_message_from_storage_txt_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_send_message_from_storage_txt(ril_request_mode_t mode,
        int32_t index,
        char *da,
        int32_t toda,
        ril_cmd_response_callback_t callback,
        void *user_data);

#endif /* __RIL_SMS_COMMAND_SUPPORT__ */

/*************************************************
 *                 Technical specification for 3GPP v250
 *************************************************/

// Basic command response with connect text
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defined reporting parameter structure prototype for AT response with connect text.
 */
typedef struct {
    char *connect_text; /**< Basic command connect text. Like "CONNECT XXX\0D\0A".*/
} ril_connect_rsp_t;


/**
 * @brief This defined reporting parameter structure prototype for AT response including an integer data.
 */
typedef struct {
    int32_t value; /**< response data (integer).*/
} ril_normal_int_rsp_t;


/**
 * @brief This defined reporting parameter structure prototype for AT response including a string.
 */
typedef struct {
    char *text; /**< A pointer to the string.*/
} ril_normal_str_rsp_t;
/**
 * @}
 */
 

/** 
 * @brief This function sends an AT command "ATE".
 *   The setting of this parameter determines whether or not the DCE echoes characters received from
 *   the DTE during command state and online command state.
 * @param[in] type  DEC setting parameter.\n
 *   0: DCE does not echo characters during command state and online command state.\n
 *   1: DCE echoes characters during command state and online command state. [Default Value]\n
 * @param[in] callback    Point to response callback function. The result code will be reported in this callback without subparameters of AT response data string. 
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_command_echo(int32_t type, ril_cmd_response_callback_t callback, void *user_data);


/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defined reporting parameter structure prototype for the command "ATI" response.
 */
typedef struct {
    char *identification_info; /**< The identification information response from manufacturer.*/
} ril_identification_info_rsp_t;
/**
 * @}
 */


/** 
 * @brief This function sends an AT command "ATI". Request identification information.
 *    This command causes the DCE to transmit one or more lines of information text, determined by the
 *    manufacturer, followed by a final result code. <value> may optionally be used to select from
 *    among multiple types of identifying information, specified by the manufacturer.
 * @param[in] value The value need get identification information. \n
 *    0: Reports product code.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *    Please see #ril_identification_info_rsp_t for the subparameters data structure.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_identification_info(ril_cmd_response_callback_t callback, void *user_data);


/** 
 * @brief This function sends an AT command "ATO". Return to online data state.
 *    Causes the DCE to return to online data state and issue a CONNECT or CONNECT <text> result
 *    code.
 * @param[in] type  The type need to get online data.\n
 *    0: Return to online data state from online command state. Also used to retrain after a modem-on-hold transaction or to reconnect to a modem that has been placed in
 *        anon-hold state per V.92.\n
 *    other: Reserved for manufacturer proprietary use.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *   Please see #ril_connect_rsp_t for the subparameters data structure.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_return_to_online_data(int32_t type, ril_cmd_response_callback_t callback, void *user_data);


/** 
 * @brief This function sends an AT command "ATQ". Result code suppression.
 *   The setting of this parameter determines whether or not the DCE transmits result codes to the DTE.
 *   When result codes are being suppressed, no portion of any intermediate, final, or unsolicited result
 *   code -header, result text, line terminator, or trailer - is transmitted. Information text transmitted in
 *   response to commands is not affected by the setting of this parameter.
 * @param[in]  type 0: DCE transmits result codes. \n 1: Result codes are suppressed and not transmitted.\n
 * @param[in] callback    Point to response callback function. The result code will be reported in this callback without subparameters of AT response data string. 
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_set_result_suppression_mode(int32_t type, ril_cmd_response_callback_t callback, void *user_data);


/** 
 * @brief This function sends an AT command "ATS3". Command line termination character.
 *    This S-parameter represents the decimal IA5 value of the character recognized by the DCE from the
 *    DTE to terminate an incoming command line.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *    Please see #ril_normal_int_rsp_t for the subparameters data structure.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_command_line_termination_char(ril_cmd_response_callback_t callback, void *user_data);


/** 
 * @brief This function sends an AT command "ATS4". Response formatting character.
 *    This S-parameter represents the decimal IA5 value of the character generated by the DCE as part of
 *    the header, trailer, and terminator for result codes and information text, along with the S3 parameter.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *    Please see #ril_normal_int_rsp_t for the subparameters data structure.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_response_formatting_char(ril_cmd_response_callback_t callback, void *user_data);


/** 
 * @brief This function sends an AT command "ATS5". Command line editing character.
 *    This S-parameter represents the decimal IA5 value of the character recognized by the DCE as a
 *    request to delete from the command line the immediately preceding character.
 * @param[in] mode  AT command mode setting.
 * @param[in] value  Set command line editing character to this value. Range is [0 ~ 127].
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string. 
 *    Please see #ril_normal_int_rsp_t for the subparameters data structure.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_command_line_editing_char(ril_request_mode_t mode, int32_t value, ril_cmd_response_callback_t callback, void *user_data);


/** 
 * @brief This function sends an AT command "ATS7". Connection completion timeout.
 *    This parameter specifies the amount of time, in seconds, that the DCE shall allow between either
 *    answering a call (automatically or by the A command) or completion of signalling of call
 *    addressing information to network (dialling), and establishment of a connection with the remote
 *    DCE.
 * @param[in] mode  AT command mode setting.
 * @param[in] value  Number of seconds in which connection must be established or call will be disconnected. Range is [0 ~ 255].
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *    Please see #ril_normal_int_rsp_t for the subparameters data structure.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_connection_completion_timeout(ril_request_mode_t mode, int32_t value, ril_cmd_response_callback_t callback, void *user_data);

/*ATS10*/
/* ril_normal_int_rsp_t */
/** 
 * @brief This function sends an AT command "ATS10". Automatic disconnect delay.
 *    This parameter specifies the amount of time, in tenths of a second, that the DCE will remain
 *    connected to the line (off-hook) after the DCE has indicated the absence of received line signal. If
 *    the received line signal is once again detected before the time specified in S10 expires, the DCE
 *    remains connected to the line and the call continues.
 * @param[in] mode  AT command mode setting.
 * @param[in] value Number of tenths of a second of delay. Range is [0 ~ 254].
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string. 
 *    Please see #ril_normal_int_rsp_t for the subparameters data structure.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_automatic_disconnect_delay(ril_request_mode_t mode, int32_t value, ril_cmd_response_callback_t callback, void *user_data);


/** 
 * @brief This function sends an AT command "ATX". Result code selection and call progress monitoring control.
 *    The setting of this parameter determines whether or not the DCE transmits particular result codes to the DTE.
 * @param[in] type The values for X parameter.\n
 *    0: CONNECT result code is given upon entering online data state. Dial tone and busy
 *    detection are disabled.\n
 *    1: CONNECT <text> result code is given upon entering online data state. Dial tone and
 *    busy detection are disabled.\n
 *    2: CONNECT <text> result code is given upon entering online data state. Dial tone
 *    detection is enabled, and busy detection is disabled.\n
 *    3: CONNECT <text> result code is given upon entering online data state. Dial tone
 *    detection is disabled, and busy detection is enabled.\n
 *    4: CONNECT <text> result code is given upon entering online data state. Dial tone and
 *    busy detection are both enabled.\n
 * @param[in] callback    Point to response callback function. The result code will be reported in this callback without subparameters of AT response data string. 
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_result_code_selection(int32_t type, ril_cmd_response_callback_t callback, void *user_data);


/** 
 * @brief This function sends an AT command "ATZ". Reset to default configuration.
 *    This command instructs the DCE to set all parameters to their factory defaults as specified by the manufacturer.
 * @param[in] type The values manufacturer-specific.\n
 * @param[in] callback    Point to response callback function. The result code will be reported in this callback without subparameters of AT response data string. 
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_reset_to_default_config(int32_t type, ril_cmd_response_callback_t callback, void *user_data);


/** 
 * @brief This function sends an AT command "AT&F". Set to factory-defined configuration.
 *    This command instructs the DCE to set all parameters to default values specified by the
 *    manufacturer, which may take into consideration hardware configuration switches and other
 *    manufacturer-defined criteria.
 * @param[in] type 0: Set parameters to factory defaults.\n  other: Reserved for manufacturer proprietary use.\n
 * @param[in] callback    Point to response callback function. The result code will be reported in this callback without subparameters of AT response data string. 
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_set_to_factory_defined_config(int32_t type, ril_cmd_response_callback_t callback, void *user_data);


/** 
 * @brief This function sends an AT command "+GCAP". Request complete capabilities list.
 *    This extended-format command causes the DCE to transmit one or more lines of information text in
 *    a specific format. The content is a list of additional capabilities command +<name>s, which is
 *    intended to permit the user of the DCE to identify the overall capabilities of the DCE.
 * @param[in] mode  AT command mode setting.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *    Please see #ril_normal_str_rsp_t for the subparameters data structure.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_complete_capabilities_list(ril_request_mode_t mode, ril_cmd_response_callback_t callback, void *user_data);


/** 
 * @brief This function sends an AT command "ATS25". Delay To DTR.
 *    Sets the length of time that the modem will ignore DTR for taking the action specified by &Dn. Its units are seconds for synchronous modes and one hundredths of a second for other modes.
 * @param[in] mode  AT command mode setting.
 * @param[in] value Number of tenths of a second of delay. Range is [0 ~ 255].
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string. 
 *    Please see #ril_normal_int_rsp_t for the subparameters data structure.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_delay_to_dtr(ril_request_mode_t mode, int32_t value, ril_cmd_response_callback_t callback, void *user_data);


/*************************************************
 *                 MediaTek proprietary commands
 *************************************************/
/* AT command: AT*MLTS */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for the URC of command "*MLTS"
 */
typedef struct {
    char *tds;    /**< Time-string, which indicates year, month, day, hours, minutes, seconds and time zone if present. */
    char *dst;    /**< Daylight saving time string. */
    char *full_name;    /**< String type, long alphanumberic format name received from the NITZ message if present. */
    int32_t coding_scheme_f;    /**< Coding scheme for strings #full_name. */
    char *short_name;    /**< String type, short alphanumberic format name received from the NITZ message if present.*/
    int32_t coding_scheme_s;    /**< Coding scheme for strings #short_name. */
    char *local_time_zone;    /**< String type. Indicates in quarters of an hour, the difference between the local time and GMT if present. E.g. +2 hours will be shown as "+08". */
    char *LSA_identify;    /**< Hex string. LSA identity of the current cell in hex format if present. E.g. "1F3A45". */
} ril_get_local_timestamp_and_network_info_urc_t;
/**
 * @brief This defines data structure for AT response of "*MLTS"
 */
typedef struct {
    int32_t enable;    /**< TA returns current setting to enable or disable MLTS URC events.*/
} ril_get_local_timestamp_and_network_info_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "*MLTS".
 * @param[in] mode    AT command mode setting.
 * @param[in] enable    Subparameter of this AT command, to enable/disable MLTS unsolicited events.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_get_local_timestamp_and_network_info_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_get_local_timestamp_and_network_info(ril_request_mode_t mode,
        int32_t enable,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT*MSIMINS */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for the URC of command "*MSIMINS"
 */
typedef struct {
    int32_t inserted;    /**< Present SIM change status. 0 means inserted -> removed; 1 means removed -> inserted. */
} ril_sim_inserted_status_reporting_urc_t;
/**
 * @brief This defines data structure for AT response of "*MSIMINS"
 */
typedef struct {
    int32_t n;    /**< 0 means disable unsolicited result code; 1 means enable unsolicited result code. */
    int32_t inserted;    /**< Present SIM change status. 0 means inserted -> removed; 1 means removed -> inserted. */
} ril_sim_inserted_status_reporting_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "*MSIMINS".
 * @param[in] mode    AT command mode setting.
 * @param[in] n    Subparameter of this AT command, enables/disables MSIMINS unsolicited result code.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_sim_inserted_status_reporting_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_sim_inserted_status_reporting(ril_request_mode_t mode,
        int32_t n,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT*MSPN */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "*MSPN"
 */
typedef struct {
    char *spn;    /**< service provider name on SIM. */
    int32_t display_mode;    /**< 0 means don't display PLMN, already registered on PLMN; 1 means display PLMN. */
} ril_get_service_provider_name_from_sim_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "*MSPN".
 * @param[in] mode    AT command mode setting.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_get_service_provider_name_from_sim_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_get_service_provider_name_from_sim(ril_request_mode_t mode,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT cmds: AT*MUNSOL */
/** 
 * @brief This function sends an AT command "*MUNSOL".
 * @param[in] mode    AT command mode setting.
 * @param[in] ind    Various value represent to report different unsolicited events, please refer to the NB-IoT proprietary AT command specification.
 * @param[in] op_mode    Means operation mode, including disable, enable and query.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_extra_unsolicited_indications(ril_request_mode_t mode,
        char *ind,
        int32_t op_mode,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT*MCGDEFCONT */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for both sending parameters and AT response of "*MCGDEFCONT".
 */
typedef struct {
    char *pdp_type;    /**< Packet Data Protocol Type, a string parameter which specifies the type of package data protocol. */
    char *apn;    /**< Access Pointer Name, a string parameter that is a logical name that is used to select network. */
    char *username;    /**< Username for the connection to the service provider. */
    char *password;    /**< Password for the connection to the service provider. */
} ril_send_pdn_connection_set_default_psd_attach_req_t, ril_send_pdn_connection_set_default_psd_attach_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "*MCGDEFCONT".
 * @param[in] mode    AT command mode setting.
 * @param[in] config    A pointer to the sent parameters, for more information, please refer to #ril_send_pdn_connection_set_default_psd_attach_req_t.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_send_pdn_connection_set_default_psd_attach_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_pdn_connection_set_default_psd_attach(ril_request_mode_t mode,
        ril_send_pdn_connection_set_default_psd_attach_req_t *config,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT*MSACL */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "*MSACL".
 */
typedef struct {
    int32_t supported;    /**< Indicates if ACL is supported by SIM or not. */
    int32_t enabled;    /**< Indicates if ACL is enabled by user or not. */
} ril_control_ACL_feature_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "*MSACL".
 * @param[in] mode    AT command mode setting.
 * @param[in] acl_mode    Subparameter for this AT command, enables or disables ACL. 
 * @param[in] pin2    Indicates PIN2 string as the second parameter.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_control_ACL_feature_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_control_ACL_feature(ril_request_mode_t mode,
        int32_t acl_mode,
        char *pin2,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT*MLACL */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "*MLACL".
 */
typedef struct {
    int32_t index;    /**< Indicates entry index. */
    char *APN;    /**< Access point name in textual format. */
} ril_display_ACL_list_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "*MLACL".
 * @param[in] mode    AT command mode setting.
 * @param[in] from    Subparameter for this AT command, indicates start index. 
 * @param[in] to    Subparameter for this AT command, indicates end index.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_display_ACL_list_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_display_ACL_list(ril_request_mode_t mode,
        int32_t from,
        int32_t to,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT*MWACL */
/** 
 * @brief This function sends an AT command "*MWACL".
 * @param[in] mode    AT command mode setting.
 * @param[in] index    Subparameter for this AT command, indicates entry index. 
 * @param[in] apn    Subparameter for this AT command, indicates access point name in textual format. 
 * @param[in] pin2    Subparameter for this AT command, indicates PIN2 string.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_write_ACL_entry(ril_request_mode_t mode,
        int32_t index,
        char *APN,
        char *pin2,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT*MDACL */
/** 
 * @brief This function sends an AT command "*MDACL".
 * @param[in] mode    AT command mode setting.
 * @param[in] index    Subparameter for this AT command, indicates entry index. 
 * @param[in] pin2    Subparameter for this AT command, indicates PIN2 string.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_delete_ACL_entry(ril_request_mode_t mode,
        int32_t index,
        char *pin2,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT*MSMEXTRAINFO */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "*MSMEXTRAINFO".
 */
typedef struct {
    int32_t enable;    /**< Indicates if extra fields enabled or not. */
} ril_control_extra_info_on_sms_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "*MSMEXTRAINFO".
 * @param[in] mode    AT command mode setting.
 * @param[in] enable    Subparameter for this AT command, enables or disables extra fields on SMS. 
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_control_extra_info_on_sms_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_control_extra_info_on_sms(ril_request_mode_t mode,
        int32_t enable,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT*MSMEXTRAUNSOL */
/* AT URC: +SMREADY */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for the URC of "+SMREADY".
 */
typedef struct {
    int32_t unread_records;    /**< Indicates the number of unread SMS records on SIM. */
    int32_t used_records;    /**< Indicates the number of used SMS records on SIM. */
    int32_t memory_exceeded;    /**< Indicates if SMS memory excceed or not. */
    int32_t num_sms_records;    /**< Indicates total number of SMS records on SIM. */
} ril_control_extra_unsolicited_messages_urc_t;
/**
 * @brief This defines data structure for AT response of "*MSMEXTRAUNSOL".
 */
typedef struct {
    int32_t enable;    /**< Indicates if extra test unsolicited messages enabled or not. */
} ril_control_extra_unsolicited_messages_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "*MSMEXTRAUNSOL".
 * @param[in] mode    AT command mode setting.
 * @param[in] enable    Subparameter for this AT command, enables or disables extra test unsolicited messages. 
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_control_extra_unsolicited_messages_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_control_extra_unsolicited_messages(ril_request_mode_t mode,
        int32_t enable,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT*MSMSTATUS */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "*MSMSTATUS".
 */
typedef struct {
    int32_t sms_status;    /**< Indicates SMS status. */
    int32_t unread_records;    /**< Indicates number of unread SMS records on SIM. */
    int32_t used_records;    /**< Indicates number of used SMS records on SIM. */
    int32_t memory_exceeded;    /**< Indicates if SMS memory excceed or not. */
    int32_t num_sms_records;    /**< Indicates total number of SMS records on SIM. */
    int32_t first_free_record;    /**< Indicates the first free SMS record on SIM. */
    int32_t num_smsp_records;    /**< Indicates total number of SMSP records on SIM. */
    int32_t service;    /**< A numeric parameter which indicates the service or service preference to be used. */
} ril_obtain_status_of_sms_functionality_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "*MSMSTATUS".
 * @param[in] mode    AT command mode setting.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_obtain_status_of_sms_functionality_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_obtain_status_of_sms_functionality(ril_request_mode_t mode,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT*MMGI */
/* AT URC: *MMGI */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for the URC of "*MMGI".
 */
typedef struct {
    int32_t event_id;    /**< Event indication identifier. */
} ril_control_sms_unsolicited_indication_urc_t;
/**
 * @brief This defines data structure as an entry for AT response of "*MMGI".
 */
typedef struct {
    int32_t event_id;    /**< Event indication identifier. */
    int32_t status;    /**< Event ID status. */
} ril_control_sms_unsolicited_indication_entry_t;
/**
 * @brief This defines data structure for AT response of "*MMGI".
 */
typedef struct {
    int32_t entry_num;    /**< Indicates the number of entries included in AT response. */
    ril_control_sms_unsolicited_indication_entry_t *param_list;    /**< A pointer to the parameter list, for more information about each entry, please refer to #ril_control_sms_unsolicited_indication_entry_t. */
} ril_control_sms_unsolicited_indication_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "*MMGI".
 * @param[in] mode    AT command mode setting. 
 * @param[in] op_mode    Subparameter for this AT command, indicates operation mode. 
 * @param[in] event_id    Subparameter for this AT command, indicates event indication identifier.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_control_sms_unsolicited_indication_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_control_sms_unsolicited_indication(ril_request_mode_t mode,
        int32_t op_mode,
        int32_t event_id,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT URC: *MSMPUKBLKD */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for the URC of "*MSMPUKBLKD".
 */
typedef struct {
    char *urc;    /**< TA reports unsolicited text, usually generated on startup or when PUK enter incorrectly 10 times when the (U)SIM is PUK blocked and therefore unusable. */
} ril_sim_puk_blocked_unsolicited_indication_urc_t;
/**
 * @}
 */


/* AT command: AT*MMGRW */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "*MMGRW".
 */
typedef struct {
    int32_t loc;    /**< Indicates SMS SIM location for writing or rewriting. */
} ril_sms_location_rewrite_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "*MMGRW".
 * @param[in] mode    AT command mode setting. 
 * @param[in] loc    Subparameter for this AT command, indicates SMS SIM location for writing or rewriting. 
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_sms_location_rewrite_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_sms_location_rewrite(ril_request_mode_t mode,
        int32_t loc,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT*MMGSC */
/* if sms is in pdu mode */
/** 
 * @brief This function sends an AT command "*MMGSC" in SMS TEXT mode.
 * @param[in] mode    AT command mode setting.  
 * @param[in] status    Subparameter for this AT command. 
 *                               String type, usually pass parameter like "REC UNREAD"/"REC READ"/"STO UNSENT"/"STO SENT".
 * @param[in] loc    Subparameter for this AT command, means SMS SIM location. 
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_sms_location_status_change_txt(ril_request_mode_t mode,
        char *status,
        int32_t loc,
        ril_cmd_response_callback_t callback,
        void *user_data);
/* if sms is in text mode */
/** 
 * @brief This function sends an AT command "*MMGSC" in SMS PDU mode.
 * @param[in] mode    AT command mode setting.  
 * @param[in] status    Subparameter for this AT command. 
 *                               Integer type, various value means received unread messages/received read messages/stored unsent messages/stored sent messages.
 * @param[in] loc    Subparameter for this AT command, means SMS SIM location. 
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_sms_location_status_change_pdu(ril_request_mode_t mode,
        int32_t status,
        int32_t loc,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT*MUPIN */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure as an entry for AT response of "*MUPIN".
 */
typedef struct {
    char *access_level;    /**< Indicates access condition level. */
    int32_t pin_status;    /**< Indicates PIN status, decimal format. */
    char *pin_verification_state;    /**< Indicates possible PIN verification state in string format. */
    int32_t pin_retry_counter;    /**< Indicates PIN remaining attempts value. */
    int32_t puk_retry_counter;    /**< Indicates unblock PIN remaining attempts value. */
    char *key_reference;    /**< Indicates PIN key reference code value. */
} ril_uicc_pin_information_access_entry_t;
/**
 * @brief This defines data structure for AT response of "*MUPIN".
 */
typedef struct {
    int32_t entry_num;    /* Indicates number of entries included in AT response. */
    ril_uicc_pin_information_access_entry_t *param_list;    /* A pointer to the parameter list, for more information about each entry, pleaser refer to #ril_uicc_pin_information_access_entry_t. */
} ril_uicc_pin_information_access_rsp_t;
/**
 * @brief This defines data structure for sending parameters of "*MUPIN".
 */
typedef struct {
    int32_t mode;    /**< Access mode. */
    char *access_level;    /**< Access condition level. */
    char *pin_or_puk;    /**< personal identification number or personal unblocking key needed to change associated PIN code. */
    char *new_pin;    /**< New personal identification number. */
    char *new_pin_confirm;    /**< New personal identification number confirmation. */
} ril_uicc_pin_information_access_req_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "*MUPIN".
 * @param[in] mode    AT command mode setting. 
 * @param[in] req_param    A pointer to the sent parameter, for more information, please refer to #ril_uicc_pin_information_access_req_t. 
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_uicc_pin_information_access_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_uicc_pin_information_access(ril_request_mode_t mode,
        ril_uicc_pin_information_access_req_t *req_param,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT*MUAPP */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure as an entry for AT response of "*MUAPP".
 */
typedef struct {
    int32_t index;    /**< Possible AID index values. */
    int32_t application_code;    /**< Possible UICC application code defined in the AID. */
    int32_t application_state;    /**< Possible application state values. */
    char *application_label;    /**< Description string for application. */
} ril_uicc_application_list_access_entry_t;
/**
 * @brief This defines data structure for AT response of "*MUAPP".
 */
typedef struct {
    int32_t entry_num;    /* Indicates number of parameter entries included in AT response. */
    ril_uicc_application_list_access_entry_t *param_list;    /* A pointer to the parameter list, for more information about each entry, please refer to #ril_uicc_application_list_access_entry_t. */
} ril_uicc_application_list_access_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "*MUAPP".
 * @param[in] mode    AT command mode setting. 
 * @param[in] access_mode    Subparameter for this AT command, indicates operation mode value. 
 * @param[in] index    Subparameter for this AT command, indicates possible AID index values. 
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_uicc_application_list_access_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_uicc_application_list_access(ril_request_mode_t mode,
        int32_t access_mode,
        int32_t index,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT*MSST */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "*MSST".
 */
typedef struct {
    int32_t stt;    /**< Indicates service type table. */
    char *sst_bitmap_allocated;    /**< Indicates if SIM service allocated or not. */
    char *sst_bitmap_activated;    /**< Indicates if SIM service activated or not. */
    char *ust_bitmap_available;    /**< Indicates if USIM service available or not. */
    char *est_bitmap;    /**< Indicates USIM service enable or not. */
} ril_read_sst_ust_from_usim_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "*MSST".
 * @param[in] mode    AT command mode setting. 
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_read_sst_ust_from_usim(ril_request_mode_t mode,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT*MABORT */
/** 
 * @brief This function sends an AT command "*MABORT".
 * @param[in] mode    AT command mode setting. 
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_abort_mm_related_at_command(ril_request_mode_t mode,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT*MNON */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure as an entry for AT response of "*MNON".
 */
typedef struct {
    int32_t index;    /**< Indicates index in prioritized network list type info in SIM files records. */
    char *operator_name;    /**< Indicates network operator name(Mandatory info with (U)SIMs). */
    char *short_form;    /**< Indicates network operator name short form(Optional info). */
    char *plmn_add_info;    /**< Indicates PLMN additional information(Optional info). */
} ril_network_operator_name_entry_t;
/**
 * @brief This defines data structure for AT response of "*MNON".
 */
typedef struct {
    int32_t entry_num;    /**< Indicates number of parameter entries included in AT response. */
    ril_network_operator_name_entry_t *param_list;    /**< A pointer to the parameter list, for more information about each entry, please refer to #ril_network_operator_name_entry_t. */
} ril_network_operator_name_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "*MNON".
 * @param[in] mode    AT command mode setting.  
 * @param[in] index    Subparameter for this AT command, indicates index in prioritized network list type info in SIM files records. 
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_network_operator_name_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_network_operator_name(ril_request_mode_t mode,
        int32_t index,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT*MOPL */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure as an entry for AT response of "*MOPL".
 */
typedef struct {
    int32_t index;    /**< Indicates index in prioritized network list type info in SIM files records. */
    char *mcc_mnc;    /**< Indicates mobile country and mobile network coded as in 3GPP. */
    char *lac_range;    /**< Indicates location area identity range. */
    int32_t pnn_id;    /**< Indicates PLMN network name record identifier. */
} ril_network_operator_plmn_list_entry_t;
/**
 * @brief This defines data structure for AT response of "*MOPL".
 */
typedef struct {
    int32_t entry_num;    /**< Indicates number of parameter entries included in AT response. */
    ril_network_operator_plmn_list_entry_t *param_list;    /**< A pointer to the parameter list, for more information about each entry, please refer to #ril_network_operator_plmn_list_entry_t. */
} ril_network_operator_plmn_list_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "*MOPL".
 * @param[in] mode    AT command mode setting.  
 * @param[in] index    Subparameter for this AT command, indicates index in prioritized network list type info in SIM files records. 
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string. 
 *                                  For more information about sub-parameters data structure, please refer to #ril_network_operator_plmn_list_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_network_operator_plmn_list(ril_request_mode_t mode,
        int32_t index,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT*MMUX */
/** 
 * @brief This function sends an AT command "*MMUX".
 * @param[in] mode    AT command mode setting.  
 * @param[in] atp_seg_ui    Subparameter for this AT command, indicates AT/PPP segmentation uplink state. 
 * @param[in] seg_ui_27010    Subparameter for this AT command, indicates 27.010 frames segmentation uplink state. 
 * @param[in] esc_mon    Subparameter for this AT command, indicates if escape sequence monitoring is performed or not. 
 * @param[in] fcs_en_27010    Subparameter for this AT command, indicates if 27.010 MUX protocol FCS checking or calculation is performed or not.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string. 
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_mux_configuration(ril_request_mode_t mode,
        int32_t atp_seg_ui,
        int32_t seg_ui_27010,
        int32_t esc_mon,
        int32_t fcs_en_27010,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT*MROUTEMMI */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure as an entry for AT response of "*MROUTEMMI".
 */
typedef struct {
    int32_t option_id;    /**< Indicates route MMI option id applied on channel used. */
    int32_t option_value;    /**< Indicates value of the given #option_id applied to the current AT channel where the *MROUTEMMI command is sent. */
    char *list_of_id;    /**< Indicates the list of MUX channel IDs configured for option id. */
} ril_mmi_at_channel_routing_configuration_entry_t;
/**
 * @brief This defines data structure for AT response of "*MROUTEMMI".
 */
typedef struct {
    int32_t entry_num;    /**< Indicates number of parameter entries included in AT response. */
    ril_mmi_at_channel_routing_configuration_entry_t *param_list;    /**< A pointer to the parameter list, for more information about each entry, please refer to #ril_mmi_at_channel_routing_configuration_entry_t. */
} ril_mmi_at_channel_routing_configuration_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "*MROUTEMMI".
 * @param[in] mode    AT command mode setting.  
 * @param[in] control_mode    Subparameter for this AT command, indicates access mode.
 * @param[in] option_id    Subparameter for this AT command, indicates route MMI option id applied on channel used.  
 * @param[in] option_value    Subparameter for this AT command, indicates value of the given #option_id applied to the current AT channel 
 *                                         where the *MROUTEMMI command is sent. 
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string. 
 *                                  For more information about sub-parameters data structure, please refer to #ril_mmi_at_channel_routing_configuration_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_mmi_at_channel_routing_configuration(ril_request_mode_t mode,
        int32_t control_mode,
        int32_t option_id,
        int32_t option_value,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT*MCEERMODE */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "*MCEERMODE".
 */
typedef struct {
    int32_t mode;    /**< Indicates CEER response mode. */
} ril_ceer_response_mode_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "*MCEERMODE".
 * @param[in] mode    AT command mode setting.  
 * @param[in] mode_setting    Subparameter for this AT command, indicates CEER response mode.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string. 
 *                                  For more information about sub-parameters data structure, please refer to #ril_ceer_response_mode_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_ceer_response_mode(ril_request_mode_t mode,
        int32_t mode_setting,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT*MFTRCFG */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure as an entry for AT response of "*MFTRCFG".
 */
typedef struct {
    int32_t modem_feature;    /**< Indicates modem feature id. */
    int32_t feature_value;    /**< Indicates possible range value per feature. */
    int32_t need_reboot;    /**< Indicates if need modem softrware reboot state or not. */
} ril_modem_feature_configuration_entry_t;
/**
 * @brief This defines data structure for AT response of "*MFTRCFG".
 */
typedef struct {
    int32_t entry_num;    /**< Indicates number of parameter entries included in AT response. */
    ril_modem_feature_configuration_entry_t *param_list;    /**< A pointer to the parameter list, for more information about each entry, please refer to #ril_modem_feature_configuration_entry_t. */
} ril_modem_feature_configuration_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "*MFTRCFG".
 * @param[in] mode    AT command mode setting.  
 * @param[in] access_mode    Subparameter for this AT command, indicates access mode.
 * @param[in] modem_feature    Subparameter for this AT command, indicates modem feature id. 
 * @param[in] feature_value    Subparameter for this AT command, indicates possible range value per feature.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string. 
 *                                  For more information about sub-parameters data structure, please refer to #ril_modem_feature_configuration_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_modem_feature_configuration(ril_request_mode_t mode,
        int32_t access_mode,
        int32_t modem_feature,
        int32_t feature_value,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT*MSTLOCK */
/** 
 * @brief This function sends an AT command "*MSTLOCK".
 * @param[in] mode    AT command mode setting.  
 * @param[in] data    Subparameter for this AT command, decimal digit.
 * @param[in] timeout    Subparameter for this AT command, user response timer - time set in seconds. Maximum timeout value is 3600 seconds.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_STK_registering(ril_request_mode_t mode,
        int32_t data,
        int32_t timeout,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT*MSTPD */
/** 
 * @brief This function sends an AT command "*MSTPD".
 * @param[in] mode    AT command mode setting.  
 * @param[in] length    Subparameter for this AT command, integer type, determines the number of bytes of "hex_data" used for the MMI data for the terminal profile.
 * @param[in] hex_data    Subparameter for this AT command, hexadecimal representation of the "length" number of bytes with bits set by MMI to indicates
                                       which of the parameters in the byte definition table given above are to be enabled in the terminal profile.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_STK_terminal_profile_download(ril_request_mode_t mode,
        int32_t length,
        char *hex_data,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT*MSTMODE */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "*MSTMODE".
 */
typedef struct {
    int32_t output_mode;    /**< Indicates output mode. */
} ril_STK_output_format_setting_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "*MSTMODE".
 * @param[in] mode    AT command mode setting.  
 * @param[in] output_mode    Subparameter for this AT command. 0 means PDU  mode, 1 means Text mode, 2 means CMGF mode(default, will use AT+CMGF setting).
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string. 
 *                                  For more information about sub-parameters data structure, please refer to #ril_STK_output_format_setting_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_STK_output_format_setting(ril_request_mode_t mode,
        int32_t output_mode,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT*MSTICREC */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure as an image instance for AT response of "*MSTICREC".
 */
typedef struct {
    int32_t width;    /**< Indicates width of this image instance expressed in raster image points. */
    int32_t height;    /**< Indicates height of this image instance expressed in raster image points. */
    int32_t cs;    /**< Indicates coding scheme. */
    int32_t efid;    /**< Hexadecimal representation of the image instance file to be read on the SIM/UICC - two bytes of data ordered MSB LSB. */
    int32_t offset;    /**< Indicates number of bytes offset from start of the file to begin reading. */
    int32_t length;    /**< Indicates number of bytes to be read. */
} ril_STK_obtain_icon_record_struct_t;
/**
 * @brief This defines data structure for AT response of "*MSTICREC".
 */
typedef struct {
    int32_t num_rec;    /**< Indicates the number of the icon record to be read from the SIM/UICC. */
    int32_t num_instances;    /**< Indicates the number of the instance data records within this image record. */
    ril_STK_obtain_icon_record_struct_t *instances_list;    /**< A pointer to the instance list, for more information about each instance, please refer to #ril_STK_obtain_icon_record_struct_t. */
} ril_STK_obtain_icon_record_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "*MSTICREC".
 * @param[in] mode    AT command mode setting.  
 * @param[in] num_rec    Subparameter for this AT command, integer, the number of the icon recrd to read from the SIM/UICC.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string. 
 *                                  For more information about sub-parameters data structure, please refer to #ril_STK_obtain_icon_record_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_STK_obtain_icon_record(ril_request_mode_t mode,
        int32_t num_rec,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT*MSTICIMG */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure as an image instance for AT response of "*MSTICIMG".
 */
typedef struct {
    int32_t length;    /**< Indicates the number of bytes of image instance data to read or was read. */
    char *data;    /**< String containing a list of hex values for bytes of image instance data. */
} ril_STK_obtain_icon_image_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "*MSTICIMG".
 * @param[in] mode    AT command mode setting.  
 * @param[in] efid    Subparameter for this AT command, hexadecimal representation of the image instance file to be read on the SIM/UICC.
 * @param[in] offset    Subparameter for this AT command, means number of bytes offset from start of the file to begin reading from. 
 * @param[in] length    Subparameter for this AT command, means number of bytes of image data to read or was read.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string. 
 *                                  For more information about sub-parameters data structure, please refer to #ril_STK_obtain_icon_image_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_STK_obtain_icon_image(ril_request_mode_t mode,
        int32_t efid,
        int32_t offset,
        int32_t length,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* URC: *MSTC */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for the URC of "*MSTC".
 */
typedef struct {
    char *cmdid;    /**< Decimal format of type of command, unique identifier for the current SIM Toolkit proactive command issues by the SIM. */
} ril_STK_proactive_command_indication_urc_t;
/**
 * @}
 */


/* AT command: AT*MSTGC */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "*MSTGC".
 */
typedef struct {
    char *cmdid;    /**< Decimal notation, command type value. */
    char *data;    /**< Indicates proactive command specific data, depend on #cmdid. */
} ril_STK_parameters_associated_with_proactive_command_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "*MSTGC".
 * @param[in] mode    AT command mode setting.  
 * @param[in] cmdid    Subparameter for this AT command, decimal notation, specifies command type value.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string. 
 *                                  For more information about sub-parameters data structure, please refer to #ril_STK_parameters_associated_with_proactive_command_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_STK_parameters_associated_with_proactive_command(ril_request_mode_t mode,
        char *cmdid,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT*MSTCR */
/** 
 * @brief This function sends an AT command "*MSTCR".
 * @param[in] mode    AT command mode setting.  
 * @param[in] cmdid    Subparameter for this AT command, decimal notation, specifies command type value, e.g. 05,08,... 
 * @param[in] result    Subparameter for this AT command, decimal notation, dependent on the command type, are used for setting of the general result parameter
                                  returned to the SIMAT task in the next phase of signaling for building the terminal response comand.                                  
 * @param[in] data    Subparameter for this AT command, is an additional data provided for certain commands, as required for the terminal response returned to the SIM
                                after processing a proactive SIM command. 
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_STK_inform_response_to_proactive_command(ril_request_mode_t mode,
        char *cmdid,
        int32_t result,
        char *data,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT URC: *MSTUD */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for the URC of "*MSTUD".
 */
typedef struct {
    char *cmdid;    /**< decimal format of type of command, unique identifier for the proactive command. */
    char *data;    /**< Indicates proactive command specific data, dependent on #cmdid. */
} ril_STK_unsolicited_data_urc_t;
/**
 * @}
 */


/* AT command: AT*MSTMS */
/** 
 * @brief This function sends an AT command "*MSTMS".
 * @param[in] mode    AT command mode setting.  
 * @param[in] item    Subparameter for this AT command, numeric type identifying an item in the menu items list.                                 
 * @param[in] help    Subparameter for this AT command, numeric type, 0 means no help requested; 1 means help requested.. 
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_STK_menu_selection(ril_request_mode_t mode,
        int32_t item,
        int32_t help,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT*MSTEV */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for sending parameters of "*MSTEV".
 */
typedef struct {
    char *event;    /**< Hex two digits, stands for various event type. */
    char *language;    /**< Two character string representation of the language code. */
    int32_t charsDownDisplay;    /**< Numeric type to indicate how many characters down are displayed. */
    int32_t sizingSupported;    /**< Numeric type 0 or 1 to indicate FALSE/TRUE. */
    int32_t charsAcrossDisplay;    /**< Numeric type to indicate how many characters across are displayed. */
    int32_t variableFontSupport;    /**< Numeric type 0 or 1 to indicate FALSE/TRUE. */
    int32_t displayResize;    /**< Numeric type 0 or 1 to indicate FALSE/TRUE. */
    int32_t textWrapping;    /**< Numeric type 0 or 1 to indicate FALSE/TRUE. */
    int32_t textScrolling;    /**< Numeric type 0 or 1 to indicate FALSE/TRUE. */
    int32_t menuWidthReduction;    /**< Numeric type to indicate the allowed menu size reduction. */
} ril_STK_monitored_event_occurence_param_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "*MSTEV".
 * @param[in] mode    AT command mode setting.  
 * @param[in] param    A pointer to the sent parameters, for more information, please refer to #ril_STK_monitored_event_occurence_param_t. 
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_STK_monitored_event_occurence(ril_request_mode_t mode,
        ril_STK_monitored_event_occurence_param_t *param,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT URC: *MCCST */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for the URC of "*MCCST".
 */
typedef struct {
    int32_t cc_type;    /**< Indicates call control type. */
    int32_t cc_event;    /**< Indicates call control event. */
    char *alpha_id;    /**< String format, using either SMS default alphabet or UCS alpha field coding to inform user of current transaction. */
} ril_STK_unsolicited_response_for_call_control_urc_t;
/**
 * @}
 */


/* AT command: AT*MICCID */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "*MICCID".
 */
typedef struct {
    char *iccid;    /**< International circuit card(ICC) ID of the (U)SIM, formatted as a numeric string without double quotes. */
} ril_read_usim_iccid_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "*MICCID".
 * @param[in] mode    AT command mode setting.  
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_read_usim_iccid_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_read_usim_iccid(ril_request_mode_t mode,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT*MHOMENW */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "*MHOMENW".
 */
typedef struct {
    char *oper_long_alpha;    /**< Indicates home operator in long alphanumeric format. */
    char *oper_short_alpha;    /**< Indicates home operator in short alphanumeric format. */
    int32_t oper_numeric;    /**< Indicates home operator in numeric GSM location area identification number format. */
} ril_display_home_network_information_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "*MHOMENW".
 * @param[in] mode    AT command mode setting.  
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_display_home_network_information_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_display_home_network_information(ril_request_mode_t mode,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT*MCSIMLOCK */
/* AT URC: *MCSIMLOCK */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "*MCSIMLOCK".
 */
typedef struct {
    int32_t lock_state;    /**< Indicates the current CSIM lock state on this channel. */
    int32_t other_lock_state;    /**< Indicates if the CSIM access is locked on any other AT channel. */
} ril_lock_csim_access_rsp_t;
/**
 * @brief This defines data structure for the URC of "*MCSIMLOCK".
 */
typedef struct {
    int32_t state;    /* is equal to zero, unsolicited event is only sent if the CSIM lock guard timer expires and the modem automatically releases the lock. */
} ril_lock_csim_access_urc_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "*MCSIMLOCK".
 * @param[in] mode    AT command mode setting.  
 * @param[in] lock_state    Subparameter for this AT command, to set the current CSIM lock state on this channel.  
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_lock_csim_access_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_lock_csim_access(ril_request_mode_t mode,
        int32_t lock_state,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT*MAPNURI */
/* AT URC: *MAPNURI */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for the URC of "*MAPNURI".
 */
typedef struct {
    int32_t cid;    /**< Specifies a particular active PDP context. */
    int32_t urc_active;    /**< Indicates if uplink rate control is active or not. */
    int32_t additional_exception_reports;    /**< Indicates whether or not addtional exception reports are allowed to be sendt when the maximum uplink rate is reached. */
    int32_t uplink_time_unit;    /**< Specifies the time unit of be used for the maximum uplink rate. */
    int32_t maximum_uplink_rate;    /**< Specifies the maximum number of messages the UE is restricted to send per link time unit. */
} ril_apn_rate_control_indication_urc_t;
/**
 * @brief This defines data structure as an entry for AT response of "*MAPNURI".
 */
typedef struct {
    int32_t mode;    /**< Indicates if the unsolicited result codes are forwarded for APN rate control or not. */
    int32_t cid;    /**< Specifies a particular active PDP context. */
    int32_t urc_active;    /**< Indicates whether or not addtional exception reports are allowed to be sendt when the maximum uplink rate is reached. */
    int32_t additional_exception_reports;    /**< Indicates whether or not addtional exception reports are allowed to be sendt when the maximum uplink rate is reached. */
    int32_t uplink_time_unit;    /**< Specifies the time unit of be used for the maximum uplink rate. */
    int32_t maximum_uplink_rate;    /**< Specifies the maximum number of messages the UE is restricted to send per link time unit. */
} ril_apn_rate_control_indication_entry_t;
/**
 * @brief This defines data structure for AT response of "*MAPNURI".
 */
typedef struct {
    int32_t entry_num;    /**< Indicates number of parameter entries included in AT response. */
    ril_apn_rate_control_indication_entry_t *param_list;    /**< A pointer to the parameter list, for more information about each entry, please refer to #ril_apn_rate_control_indication_entry_t. */
} ril_apn_rate_control_indication_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "*MAPNURI".
 * @param[in] mode    AT command mode setting.  
 * @param[in] control_mode    Subparameter for this AT command, set if forward unsolicited result codes or not.  
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_apn_rate_control_indication_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_apn_rate_control_indication(ril_request_mode_t mode,
        int32_t control_mode,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT*MPLMNURI */
/* AT URC: *PLMNURI */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for the URC of "*PLMNURI".
 */
typedef struct {
    int32_t urc_active;    /**< Indicates if uplink rate control is active or not. */
    int32_t serving_plmn_rate_control_value;    /**< Indicates maximum number of uplink messages the UE is allowed to send in a 6 minute interval. */
} ril_plmn_rate_control_indication_urc_t;
/**
 * @brief This defines data structure for AT response of "*PLMNURI".
 */
typedef struct {
    int32_t mode;    /**< Indicates if forward unsolicited result codes or not. */
    int32_t urc_active;    /**< Indicates if uplink rate control is active or not. */
    int32_t serving_plmn_rate_control_value;    /**< Indicates maximum number of uplink messages the UE is allowed to send in a 6 minute interval. */
} ril_plmn_rate_control_indication_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "*PLMNURI".
 * @param[in] mode    AT command mode setting.  
 * @param[in] control_mode    Subparameter for this AT command, set if forward unsolicited result codes or not.  
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_plmn_rate_control_indication_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_plmn_rate_control_indication(ril_request_mode_t mode,
        int32_t control_mode,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT*MPDI */
/* AT URC: *MPDI */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for the URC of "*MPDI".
 */
typedef struct {
    int32_t status;    /**< Indicates if start packet discard indication or not. */
    int32_t cid;    /**< Specifies a particular PDP context definition. */
    int32_t time;    /**< Indicates the amount of time that the uplink packets will be discarded for(time in units of seconds). */
} ril_packet_discard_indication_urc_t;
/**
 * @brief This defines data structure for AT response of "*MPDI".
 */
typedef struct {
    int32_t mode;    /**< Indicates if forware unsolicited result codes or not. */
} ril_packet_discard_indication_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "*MPDI".
 * @param[in] mode    AT command mode setting.  
 * @param[in] control_mode    Subparameter for this AT command, set if forward unsolicited result codes or not.  
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_packet_discard_indication_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_packet_discard_indication(ril_request_mode_t mode,
        int32_t control_mode,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT*MNBIOTDT */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure as an entry for AT response of "*MNBIOTDT".
 */
typedef struct {
    int32_t cid;    /**< Specifies a particular PDP context definition. */
    int32_t type;    /**< Specifies normal data or exceptional data. */
} ril_nbiot_data_type_entry_t;
/**
 * @brief This defines data structure as an entry for AT response of "*MNBIOTDT".
 */
typedef struct {
    int32_t entry_num;    /**< Indicates number of parameter entries included in AT response. */
    ril_nbiot_data_type_entry_t *param_list;    /**< A pointer to the parameter list, for more information about each entry, please refer to #ril_nbiot_data_type_entry_t. */
} ril_nbiot_data_type_rsp_t;
/**
 * @brief This defines data structure for sending parameters of "*MNBIOTDT".
 */
typedef struct {
    int32_t type;    /**< Specifies normal data or exceptional data. */
    int32_t param_num;    /**< Specifies the number of the cid. */
    int32_t *cid_list;    /**< Point to the cid list. */
} ril_nbiot_data_type_req_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "*MNBIOTDT".
 * @param[in] mode    AT command mode setting.  
 * @param[in] req_param    A pointer to the sent parameters, for more information, please refer to #ril_nbiot_data_type_req_t.  
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_nbiot_data_type_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_nbiot_data_type(ril_request_mode_t mode,
        ril_nbiot_data_type_req_t *req_param,
        ril_cmd_response_callback_t callback,
        void *user_data);



/* AT command: AT*MNBIOTRAI */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "*MNBIOTRAI".
 */
typedef struct {
    int32_t rai;    /**< Specifies release assistance information. */
} ril_nbiot_release_assistance_indication_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "*MNBIOTRAI".
 * @param[in] mode    AT command mode setting.  
 * @param[in] rai    Subparameter for this AT command, specifies release assistance information.  
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                  For more information about sub-parameters data structure, please refer to #ril_nbiot_release_assistance_indication_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_nbiot_release_assistance_indication(ril_request_mode_t mode,
        int32_t rai,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT URC: *MSQN */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for the URC of "*MSQN".
 */
typedef struct {
    int32_t rssi;    /**< Indicates signal strength indication. */
    int32_t ber;    /**< Indicates channel bit error rate. */
} ril_urc_propriteary_signal_quality_report_t;
/**
 * @}
 */


/* AT command: AT*MATWAKEUP */
/** 
 * @brief This function sends an AT command "*MATWAKEUP".
 * @param[in] mode    AT command mode setting.  
 * @param[in] n    Subparameter for this AT command, specifies if enable *MATWAKEUP URC or not.  
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_set_modem_wakeup_indication(ril_request_mode_t mode,
        int32_t n,
        ril_cmd_response_callback_t callback,
        void *user_data);



/* AT command: AT*MBAND */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "*MBAND".
 */
typedef struct {
    int32_t current_band;    /**< Integer value indicating current selected NB-IoT band. */
} ril_query_modem_operating_band_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "*MBAND".
 * @param[in] mode    AT command mode setting.  
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                   For more information about sub-parameters data structure, please refer to #ril_query_modem_operating_band_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_query_modem_operating_band(ril_request_mode_t mode,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT*MENGINFO */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for the serving cell information of "*MENGINFO".
 */
typedef struct {
    int32_t sc_earfcn;    /**< Indicates the EARFCN for serving cell. */
    int32_t sc_earfcn_offset;    /**< Indicates the EARFCN offset for serving cell. */
    int32_t sc_pci;    /**< Indicates the serving cell physical cell ID. */
    char *sc_cellid;    /**< String type, four byte (28 bits) cell ID in hexadecimal format for serving cell. */
    int32_t sc_rsrp;    /**< Indicates serving cell RSRP value in units of dBm. */
    int32_t sc_rsrq;    /**< Indicates serving cell RSRQ value in units of dB. */
    int32_t sc_rssi;    /**< Indicates serving cell RSSI value in units of dBm. */
    int32_t sc_snr;    /**< Indicates last SNR value for serving cell in units of dB. */
    int32_t sc_band;    /**< Indicates current serving cell band. */
    char *sc_tac;    /**< Indicates two byte tracking area code (TAC) in hexadecimal format. */
    int32_t sc_ecl;    /**< Indicates last Enhanced Coverage Level (ECL) value for serving cell. */
    int32_t sc_tx_pwr;    /**< Indicates current UE transmit power, units of cBm Centibels relative to one miliwatt. */
} ril_query_network_state_serving_cell_info_t;
/**
 * @brief This defines data structure for the neighbor cell information of "*MENGINFO".
 */
typedef struct {
    int32_t nc_earfcn;    /**< Indicates the EARFCN for neighbor cell. */
    int32_t nc_earfcn_offset;    /**< Indicates the EARFCN offset for neighbor cell. */
    int32_t nc_pci;    /**< Indicates the neighbor cell physical cell ID. */
    int32_t nc_rsrp;    /**< Indicates neighbor cell RSRP value in units of dBm. */
} ril_query_network_state_neighbor_cell_info_t;
/**
 * @brief This defines data structure for the data transfer information of "*MENGINFO".
 */
typedef struct {
    int32_t RLC_UL_BLER;    /**< Represented in % value (range 0 to 100), UL block error rate in RLC. */
    int32_t RLC_DL_BLER;    /**< Represented in % value (range 0 to 100), DL block error rate in RLC. */
    int32_t MAC_UL_BLER;    /**< Represented in % value (range 0 to 100), UL block error rate in MAC for UL-SCH. */
    int32_t MAC_DL_BLER;    /**< Represented in % value (range 0 to 100), DL block error rate in MAC for UL-SCH. */
    int32_t MAC_UL_total_bytes;    /**< Total number of transport block bytes (re)transmitted on UL-SCH. */
    int32_t MAC_DL_total_bytes;    /**< Total number of transport block bytes (re)transmitted on DL-SCH, excluding BCCH. */
    int32_t MAC_UL_total_HARQ_TX;    /**< Total number of HARQ (re)transmission for transport blocks on UL-SCH. */
    int32_t MAC_DL_total_HARQ_TX;    /**< Total number of HARQ (re)transmission for transport blocks on DL-SCH, excluding BCCH. */
    int32_t MAC_UL_HARQ_re_TX;    /**< The number of HARQ retransmission for transport blocks on UL-SCH. */
    int32_t MAC_DL_HARQ_re_TX;    /**< The number of HARQ retransmission for transport blocks on DL-SCH. */
    int32_t RLC_UL_tput;    /**< RLC uplink throughput. */
    int32_t RLC_DL_tput;    /**< RLC downlink throughput. */
    int32_t MAC_UL_tput;    /**< UL throughput in MAC for UL-SCH. */
    int32_t MAC_DL_tput;    /**< DL throughput in MAC for DL-SCH, excluding BCCH. */
} ril_query_network_state_data_transfer_info_t;
/**
 * @brief This defines data structure for AT response of "*MENGINFO".
 */
typedef struct {
    ril_query_network_state_serving_cell_info_t *sc_info;    /**< A poiner to the structure for serving cell information. */
    int32_t nc_num;    /**< Indicates number of neighbor cells to be retrieved. */
    ril_query_network_state_neighbor_cell_info_t *nc_info;    /**< A pointer to the structure list for neightbor cells information, access way like array, e.g. nc_info[0], nc_info[1], ... */
    ril_query_network_state_data_transfer_info_t *data_transfer_info;    /**< A pointer to the structure for data transfer information. */
} ril_query_network_state_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "*MENGINFO".
 * @param[in] mode    AT command mode setting. 
 * @param[in] display_mode    Subparameter for this AT command, indicates which display mode the modem returns when "*MENGINFO" reporting.  
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                   For more information about sub-parameters data structure, please refer to #ril_query_network_state_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_query_network_state(ril_request_mode_t mode,
        int32_t display_mode,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT*MFRCLLCK */
/** 
 * @brief This function sends an AT command "*MFRCLLCK".
 * @param[in] mode    AT command mode setting. 
 * @param[in] lock      Indicates whether to activate lock or remote lock. 
 * @param[in] earfcn      Indicates requested EARFCN on which to lock. 
 * @param[in] earfcn_offset      Indicates requested EARFCN offset. 
 * @param[in] pci      Indicates physical cell ID, range: 0 ~ 503.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_frequency_and_cell_lock(ril_request_mode_t mode,
        int32_t lock,
        int32_t earfcn,
        int32_t earfcn_offset,
        int32_t pci,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT*MSPCHSC */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for AT response of "*MSPCHSC".
 */
typedef struct {
    int32_t mode;    /**< Indicates whether new algorithm is applied. */
} ril_set_scrambling_algorithm_for_npdsch_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "*MSPCHSC".
 * @param[in] mode    AT command mode setting. 
 * @param[in] algo_mode    Subparameter for this AT command, specifies which algorithm will be applied.  
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                   For more information about sub-parameters data structure, please refer to #ril_set_scrambling_algorithm_for_npdsch_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_set_scrambling_algorithm_for_npdsch(ril_request_mode_t mode,
        int32_t algo_mode,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT*MDPDNP */
/* AT URC: *MDPDNP */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure for the URC of "*MDPDNP".
 */
typedef struct {
    char *apn;    /**< A string parameter to indicate access pointer name. */
    char *pdn_type;    /**< A string parameter which specifies the type of package data protocol, e.g. IP/IPV6/IPV4V6. */
} ril_default_pdn_parameter_urc_t;

/**
 * @brief This defines data structure for AT response of "*MDPDNP".
 */
typedef struct {
    int32_t n;    /**< Indicates whether the unsolicited result code of default PDN parameter is enabled. */
} ril_default_pdn_parameter_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "*MDPDNP".
 * @param[in] mode    AT command mode setting. 
 * @param[in] n    Subparameter for this AT command, specifies whether the unsolicited result code is enabled.  
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                   For more information about sub-parameters data structure, please refer to #ril_default_pdn_parameter_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_default_pdn_parameter(ril_request_mode_t mode,
        int32_t n,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT*MEDRXCFG */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure as an entry for AT response of "*MEDRXCFG".
 */
typedef struct {
    int32_t act_type;    /**< Indicates the type of access technology. */
    char *requested_eDRX_value;    /**< String type, half a byte in 4-bit format. */
    char *requested_paging_time_window_value;    /**< String type, half a byte in 4-bit format. */
} ril_eDRX_configuration_rsp_t;
/**
 * @}
 */

/** 
 * @brief This function sends an AT command "*MEDRXCFG".
 * @param[in] mode    AT command mode setting. 
 * @param[in] cfg_mode    Subparameter for this AT command, indicates enable or disable the use of eDRX in the UE.
 * @param[in] act_type    Subparameter for this AT command, indicates the type of access technology.
 * @param[in] requested_eDRX_value    Subparameter for this AT command, half a byte in 4-bit format.
 * @param[in] requested_paging_time_window_value    Subparameter for this AT command, half a byte in 4-bit format.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                   For more information about sub-parameters data structure, please refer to #ril_eDRX_configuration_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_eDRX_configuration(ril_request_mode_t mode,
        int32_t cfg_mode,
        int32_t act_type,
        char *requested_eDRX_value,
        char *requested_paging_time_window_value,
        ril_cmd_response_callback_t callback,
        void *user_data);


/* AT command: AT*MCELLINFO */
/** 
 * @addtogroup ril_structure Structure
 * @{
 */
/**
 * @brief This defines data structure as an entry for AT response of "*MCELLINFO".
 */
typedef struct {
    int32_t earfcn;    /**< Indicates the EARFCN for serving or neighbor cell. */
    int32_t earfcn_offset;    /**< Indicates the EARFCN offset for serving or neighbor cell. */
    int32_t pci;    /**< Indicates the serving or neightbor cell physical cell ID. */
    int32_t rsrp;    /**< Indicates serving or neighbor cell RSRP value in units of dBm. */
    int32_t rsrq;    /**< Indicates serving or neighbor cell RSRQ value in units of dB. */
    int32_t rssi;    /**< Indicates serving or neighbor cell RSSI value in units of dBm. */
    int32_t sinr;    /**< Indicates last SINR value for serving or neighbor cell in units of dB. */
    int32_t mcc;    /**< Indicates Mobile Country Code. */
    int32_t mnc;    /**< Indicates Mobile Network Code. */
    char *tac;    /**< Indicates two byte tracking area code (TAC) in hexadecimal format. */
    char *cellid;    /**< String type, four byte (28 bits) cell ID in hexadecimal format for serving or neighbor cell. */
} ril_serving_and_neighbor_cell_info_t;

/**
 * @brief This defines data structure for AT response of "*MCELLINFO".
 */
typedef struct {
    ril_serving_and_neighbor_cell_info_t *sc_info;    /**< A poiner to the structure for serving cell information. */
    int32_t nc_num;    /**< Indicates number of neighbor cells to be retrieved. */
    ril_serving_and_neighbor_cell_info_t *nc_info_list;    /**< A pointer to the structure list for neightbor cells information, access way like array, e.g. nc_info[0], nc_info[1], ... */
} ril_serving_and_neighbor_cell_info_rsp_t;

/**
 * @}
 */

/** 
 * @brief This function sends an AT command "*MCELLINFO".
 * @param[in] mode    AT command mode setting. 
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 *                                   For more information about sub-parameters data structure, please refer to #ril_serving_and_neighbor_cell_info_rsp_t.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_serving_and_neighbor_cell_info(ril_request_mode_t mode,
        ril_cmd_response_callback_t callback,
        void *user_data);



/* AT command: AT*MTC */

/** 
 * @brief This function sends an AT command "*MTC".
 * @param[in] mode    AT command mode setting. 
 * @param[in] NFM_active    Subparameter for this AT command, integer value indicating NFM active mode.
 * @param[in] start_timer_active    Subparameter for this AT command, integer value indicating to enable/disable start timer.
 * @param[in] STPar    Subparameter for this AT command, integer value which is the ST time interval in seconds 1 ~ 15360 (default is 60).
 * @param[in] test_mode    Subparameter for this AT command, integer value indicating the test mode.
 * @param[in] callback    is a response callback function, including the result code and sub-parameters of AT response data string.
 * @param[in] user_data    A pointer to the user data.
 * @return #RIL_STATUS_SUCCESS, the operation completed successfully.
 *             #RIL_STATUS_FAIL, the operation has failed.
 *             #RIL_STATUS_INVALID_PARAM, invalid sub-parameter.
 */
extern ril_status_t ril_request_test_config(ril_request_mode_t mode,
        int32_t NFM_active,
        int32_t start_timer_active,
        int32_t STPar,
        int32_t test_mode,
        ril_cmd_response_callback_t callback,
        void *user_data);


#ifdef __cplusplus
}
#endif
#endif /* __RIL_H__ */

