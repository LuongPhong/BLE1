/******************************************************************************

 @file       immediate_alert.c

 @brief This file contains the Immediate Alert service.

 Group: CMCU, SCS
 Target Device: CC2640R2

 Created by Gerrikoio using TI's Service Generator as a template

 ******************************************************************************

 Copyright (c) 2012-2017, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 Release Name: simplelink_cc2640r2_sdk_1_40_00_45
 Release Date: 2017-07-20 17:16:59
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "peripheral.h"
#include "simpleBLEPeripheral.h"
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/UART.h>
#include <string.h>
#include <icall.h>
#include "util.h"
#include "LinkLoss.h"
/* This Header file contains all BLE API and icall structure definition */
//#include "icall_ble_api.h"
#include "att.h"
#include "gatt_profile_uuid.h"
#include <string.h>

#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"
extern UART_Handle uart;
//uint8 blink = 0;
uint8 disconnect = 0;
uint8 mode = 0;
#define START_ADVERTISING_EVT_LINK_LOSS         0x0001  // Start Advertising
extern void print_number (uint16 number);
extern void user_print(char* str);
extern void gapRole_setEvent(uint32_t event);

extern void gapRole_setEvent(uint32_t event);
extern Clock_Struct periodicClock;
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */


/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Immediate Alert service
CONST uint8_t linklossServUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(LINK_LOSS_SERV_UUID), HI_UINT16(IMMEDIATE_ALERT_SERV_UUID)
};

// Alert level Command characteristic
CONST uint8_t Link_lossCommandUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(ALERT_LEVEL_UUID), HI_UINT16(ALERT_LEVEL_UUID)
};

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Application callback.
static LinkLossimmedAlertCBs_t *pAppCBs = NULL;


/*********************************************************************
 * Profile Attributes - variables
 */

// Immediate Alert Service attribute.
static CONST gattAttrType_t linklossService = { ATT_BT_UUID_SIZE, linklossServUUID };

static gattCharCfg_t *linklossClientCharCfg;

// Command Characteristic
static uint8 alertLevelCommandProps = GATT_PROP_WRITE_NO_RSP;
// Characteristic "AlertLevel" Value variable
static uint8_t link_losstLevelVal[1] = {0};



/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t link_lossAttrTbl[] =
{
  // Immediate Alert Service
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8_t *)&linklossService         /* pValue */
  },

  // Immediate Alert Client Characteristic Configuration
   {
     { ATT_BT_UUID_SIZE, clientCharCfgUUID },
     GATT_PERMIT_READ,
     0,
     (uint8 *) &linklossClientCharCfg
   },

   // Command Declaration
  {
    { ATT_BT_UUID_SIZE, characterUUID },
    GATT_PERMIT_READ,
    0,
    &alertLevelCommandProps
  },

    // Command Value
    {
      { ATT_BT_UUID_SIZE,Link_lossCommandUUID },
      GATT_PERMIT_READ | GATT_PERMIT_WRITE,
      0,
      link_losstLevelVal
    }
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t Link_Loss_immedAlert_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                           uint8 *pValue, uint16 *pLen, uint16 offset,
                                           uint16 maxLen, uint8 method );
static bStatus_t Link_Loss_immedAlert_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                            uint8 *pValue, uint16 len, uint16 offset,
                                            uint8 method );
/*********************************************************************
 * PROFILE CALLBACKS
 */
// Immediate Alert Service Callbacks
// Note: When an operation on a characteristic requires authorization and
// pfnAuthorizeAttrCB is not defined for that characteristic's service, the
// Stack will report a status of ATT_ERR_UNLIKELY to the client.  When an
// operation on a characteristic requires authorization the Stack will call
// pfnAuthorizeAttrCB to check a client's authorization prior to calling
// pfnReadAttrCB or pfnWriteAttrCB, so no checks for authorization need to be
// made within these functions.
CONST gattServiceCBs_t Link_Loss_immedAlertCBs =
{
     Link_Loss_immedAlert_ReadAttrCB, // Read callback function pointer
     Link_Loss_immedAlert_WriteAttrCB, // Write callback function pointer
     NULL                   // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      ImmedAlert_AddService
 *
 * @brief   Initializes the Immediate Alert Service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 */



bStatus_t LINK_LOSS_AddService(void)
{
  uint8_t status;

  // Allocate Client Characteristic Configuration table
  linklossClientCharCfg = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) *
                                                          linkDBNumConns );
  if ( linklossClientCharCfg == NULL )
  {
    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, linklossClientCharCfg );

  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService(link_lossAttrTbl,
                                        GATT_NUM_ATTRS(link_lossAttrTbl),
                                        GATT_MAX_ENCRYPT_KEY_SIZE,
                                        &Link_Loss_immedAlertCBs);

  return (status);
}

/*********************************************************************
 * @fn      immedAlert_Register
 *
 * @brief   Register a callback function with the Immediate Alert Service.
 *
 * @param   appCallbacks - pointer to application callbacks.
 *
 * @return  None.
 * Link_loss_Register
 */
bStatus_t Link_Loss_Register( LinkLossimmedAlertCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    pAppCBs = appCallbacks;

    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}

/*********************************************************************
 * @fn      ImmedAlert_SetParameter
 *
 * @brief   Set an Immediate Alert Service parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to write
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t Link_loss_SetParameter( uint8 param, uint8 len, void *value )
{
    bStatus_t ret = SUCCESS;
    switch ( param )
    {
      case LINK_LOSSLEVEL:
        if ( len == LINK_LOSSLEVEL_LEN )
        {
          memcpy(link_losstLevelVal, value, len);
        }
        else
        {
          ret = bleInvalidRange;
        }
        break;

      default:
        ret = INVALIDPARAMETER;
        break;
    }
    return ret;
}

/*********************************************************************
 * @fn      ImmedAlert_GetParameter
 *
 * @brief   Get an Immediate Alert Service parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to get.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 *
 * @return  bStatus_t
 */
bStatus_t Link_loss_GetParameter(uint8_t param, void *value)
{
    bStatus_t ret = SUCCESS;
    switch ( param )
    {
      case LINK_LOSSLEVEL:
        memcpy(value, link_losstLevelVal, LINK_LOSSLEVEL_LEN);
        break;

      default:
        ret = INVALIDPARAMETER;
        break;
    }
    return ret;
}

/*********************************************************************
 * @fn          immedAlert_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t Link_Loss_immedAlert_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                       uint8 *pValue, uint16 *pLen, uint16 offset,
                                       uint16 maxLen, uint8 method )
{
  bStatus_t status = SUCCESS;

  // See if request is regarding the AlertLevel Characteristic Value
if ( ! memcmp(pAttr->type.uuid, Link_lossCommandUUID, pAttr->type.len) )
  {
    if ( offset > LINK_LOSSLEVEL_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, LINK_LOSSLEVEL_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  else
  {
    // If we get here, that means you've forgotten to add an if clause for a
    // characteristic value attribute in the attribute table that has READ permissions.
    *pLen = 0;
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  return status;
}

/*********************************************************************
 * @fn      immedAlert_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t Link_Loss_immedAlert_WriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                 uint8_t *pValue, uint16_t len, uint16_t offset,
                                 uint8 method)
{
    bStatus_t status  = SUCCESS;
    uint8_t   paramID = 0xFF;

    // See if request is regarding a Client Characterisic Configuration
    if ( ! memcmp(pAttr->type.uuid, clientCharCfgUUID, pAttr->type.len) )
    {
      // Allow only notifications.
      status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                               offset, GATT_CLIENT_CFG_NOTIFY);
    }
    // See if request is regarding the AlertLevel Characteristic Value
    else if ( ! memcmp(pAttr->type.uuid, Link_lossCommandUUID, pAttr->type.len) )
    {
      if ( offset + len > LINK_LOSSLEVEL_LEN )
      {
        status = ATT_ERR_INVALID_OFFSET;
      }
      else
      {
        // Copy pValue into the variable we point to from the attribute table.
        memcpy(pAttr->pValue + offset, pValue, len);

        // Only notify application if entire expected value is written
        if ( offset + len == LINK_LOSSLEVEL_LEN)
          paramID = LINK_LOSSLEVEL;
        uint8 *pCurValue = (uint8 *)pAttr->pValue;
        disconnect = *pCurValue;

        if(disconnect ==  0x03)
        {
            Util_startClock(&periodicClock);

          // Disable connectable advertising.
          //
             GAPRole_TerminateConnection();
//             uint8 advertEnabled = TRUE; // Turn on Advertising
//           GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &advertEnabled);
             Util_startClock(&periodicClock);


        }
    }
    }
    else
    {
      // If we get here, that means you've forgotten to add an if clause for a
      // characteristic value attribute in the attribute table that has WRITE permissions.
      status = ATT_ERR_ATTR_NOT_FOUND;
    }

    // Let the application know something changed (if it did) by using the
    // callback it registered earlier (if it did).
    if (paramID != 0xFF)
      if ( pAppCBs && pAppCBs->pfnChangeCb )
        pAppCBs->pfnChangeCb( paramID ); // Call app function from stack task context.

    return status;
}

