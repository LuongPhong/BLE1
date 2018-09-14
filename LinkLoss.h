#ifndef LINKLOSS_H
#define LINKLOSS_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */
#include "bcomdef.h"
//  Characteristic defines
#define LINK_LOSS_SERV_UUID             0x1803  // Link Loss
#define LINK_LOSSLEVEL      5
#define LINK_LOSSLEVEL_LEN  1

// Heart Rate Service Parameters
#define ALERTLEVEL_COMMAND                   0
#define SIMPLEPROFILE_SERVICE_LINK_LOSS               0x00000001
// Value for alert level command characteristic
#define ALERTLEVEL_COMMAND_NO_ALERT        0x00
#define ALERTLEVEL_COMMAND_MILD_ALERT      0x01
#define ALERTLEVEL_COMMAND_HIGH_ALERT      0x02

// ATT Error code
// Control point value not supported
#define ALERTLEVEL_ERR_NOT_SUP               0x80

#define ALERTLEVEL_COMMAND_SET               1

/*********************************************************************
 * TYPEDEFS
 */


/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */
// Callback when a characteristic value has changed
typedef void (*immedalertServiceCB_t)(uint8 event);

typedef struct
{
  immedalertServiceCB_t        pfnChangeCb;  // Called when characteristic value changes
} LinkLossimmedAlertCBs_t;


/*********************************************************************
 * API FUNCTIONS
 */

/*********************************************************************
 * @fn      ImmedAlert_AddService
 *
 * @brief   Initializes the Immediate Alert service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 */
extern bStatus_t LINK_LOSS_AddService(void);

/*********************************************************************
 * @fn      ImmedAlert_Register
 *
 * @brief   Register a callback function with the Immediate Alert Service.
 *
 * @param   pfnServiceCB - Callback function.
 *
 * @return  None.
 */
extern bStatus_t Link_Loss_Register( LinkLossimmedAlertCBs_t *appCallbacks );

/*
 * ImmedAlert_SetParameter - Set a ImmedAlert parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t Link_loss_SetParameter( uint8 param, uint8 len, void *value );

/*
 * ImmedAlert_GetParameter - Get an Immediate Alert parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t Link_loss_GetParameter(uint8 param, void *value);


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* IMMEDIATE_ALERT_H */
