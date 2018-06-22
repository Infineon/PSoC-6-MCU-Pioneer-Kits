/******************************************************************************
* File Name: ble_application.c
*
* Version: 1.10
*
* Description: This file contains functions that handle custom BLE services
*
* Related Document: CE218135_BLE_Proximity.pdf
*
* Hardware Dependency: CY8CKIT-062-BLE PSoC 6 BLE Pioneer Kit
*                      CY8CKIT-028-EPD E-INK Display Shield
*
*******************************************************************************
* Copyright (2017), Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* (“Software”), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software (“EULA”).
*
* If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress’s integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, 
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED 
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress 
* reserves the right to make changes to the Software without notice. Cypress 
* does not assume any liability arising out of the application or use of the 
* Software or any product or circuit described in the Software. Cypress does 
* not authorize its products for use in any products where a malfunction or 
* failure of the Cypress product may reasonably be expected to result in 
* significant property damage, injury or death (“High Risk Product”). By 
* including Cypress’s product in a High Risk Product, the manufacturer of such 
* system or application assumes all risk of such use and in doing so agrees to 
* indemnify Cypress against all liability.
*******************************************************************************/
/******************************************************************************
* This file contains functions that handle stack events and the custom 
* CapSense Proximity service
*******************************************************************************/

/* Header file includes */
#include "ble_application.h"
#include "led.h"
#include "proximity.h"

/* Indexes of a two-byte CCCD array */
#define CCCD_INDEX_0            (uint8_t) (0x00u)
#define CCCD_INDEX_1            (uint8_t) (0x01u)

/* Bit mask for the notification bit in CCCD (Client Characteristic Configuration 
   Descriptor), which is written by the client device */
#define CCCD_NOTIFY_ENABLED     (uint8_t) (0x01u)
#define CCCD_NOTIFY_DISABLED    (uint8_t) (0x00u)

/* Redefinition of long CCCD handles and indexes for better readability */
#define PROXIMITY_CCCD_HANDLE      \
(CY_BLE_CAPSENSE_PROXIMITY_CAPSENSE_PROXIMITY_CLIENT_CHARACTERISTIC_CONFIGURATION_DESC_HANDLE)
#define PROXIMITY_CCCD_INDEX       \
(CY_BLE_CAPSENSE_PROXIMITY_CAPSENSE_PROXIMITY_CLIENT_CHARACTERISTIC_CONFIGURATION_DESC_INDEX)

/* 'connectionHandle' stores the BLE connection parameters */
cy_stc_ble_conn_handle_t static connectionHandle;

/* These flags are enabled when the Central device writes to CCCD (Client 
   Characteristic Configuration Descriptor) of the CapSense Proximity
   Characteristics to enable notifications */
uint8_t static proximityNotificationStatus  = CCCD_NOTIFY_DISABLED;

/* This flag is used to restart advertisements in the main control loop */
bool    static restartBleAdvertisement   = false;

/*******************************************************************************
* Function Name: void UpdateCccdStatusInGattDb
*                (cy_ble_gatt_db_attr_handle_t cccdHandle, uint8_t value)                            
********************************************************************************
* Summary:
*  This function updates the notification status (lower byte of CCCD array) of
*  a characteristic in GATT DB with the provided parameters
*
* Parameters:
*  cccdHandle:	CCCD handle of the service
*  value     :  Notification status. Valid values are CCCD_NOTIFY_DISABLED and
*               CCCD_NOTIFY_ENABLED
*
* Return:
*  void
*
*******************************************************************************/
void UpdateCccdStatusInGattDb(cy_ble_gatt_db_attr_handle_t cccdHandle,
                              uint8_t value)
{
    /* Local variable to store the current CCCD value */
    uint8_t cccdValue[CY_BLE_CCCD_LEN];
    
    /* Load the notification status to the CCCD array */
    cccdValue[CCCD_INDEX_0] = value;
    cccdValue[CCCD_INDEX_1] = CY_BLE_CCCD_DEFAULT;
        
    /* Local variable that stores notification data parameters */
    cy_stc_ble_gatt_handle_value_pair_t  cccdValuePair = 
    {
        .attrHandle = cccdHandle,
        .value.len = CY_BLE_CCCD_LEN,
        .value.val = cccdValue
    };
    
    /* Local variable that stores attribute value */
    cy_stc_ble_gatts_db_attr_val_info_t  cccdAttributeHandle=
    {
        .connHandle = connectionHandle,
        .handleValuePair = cccdValuePair,
        .offset = CY_BLE_CCCD_DEFAULT,
    };
    
    /* Extract flag value from the connection handle - TO BE FIXED*/
    if(connectionHandle.bdHandle == 0u)
    {
        cccdAttributeHandle.flags = CY_BLE_GATT_DB_LOCALLY_INITIATED;
    }
    else
    {
        cccdAttributeHandle.flags = CY_BLE_GATT_DB_PEER_INITIATED;
    }
    
    /* Update the CCCD attribute value per the input parameters */
    Cy_BLE_GATTS_WriteAttributeValueCCCD(&cccdAttributeHandle);
}

/*******************************************************************************
* Function Name: void SendCapSenseProximityNotification(uint8_t proximityData)
********************************************************************************
* Summary:
*  Send CapSense Proximity data as BLE Notifications. This function updates
*  the notification handle with data and triggers the BLE component to send
*  notification
*
* Parameters:
*  proximityData:	CapSense proximity value
*
* Return:
*  void
*
*******************************************************************************/
void SendCapSenseProximityNotification(uint8_t proximityData)
{
    /* Make sure that stack is not busy, then send the notification. Note that 
       the number of buffers in the BLE stack that holds the application data 
       payload are limited, and there are chances that notification might drop 
       a packet if the BLE stack buffers are not available. This error condition
       is not handled in this example project */
    if (Cy_BLE_GATT_GetBusyStatus(connectionHandle.attId) 
                                                     == CY_BLE_STACK_STATE_FREE)
    {
        /* Local variable that stores CapSense notification data parameters */
        cy_stc_ble_gatts_handle_value_ntf_t proximityNotificationHandle =
        {
            .connHandle = connectionHandle,
            .handleValPair.attrHandle = 
                            CY_BLE_CAPSENSE_PROXIMITY_CAPSENSE_PROXIMITY_CHAR_HANDLE,
            .handleValPair.value.val = &proximityData,
            .handleValPair.value.len = CAPSENSE_PROXIMITY_DATA_LEN
        };
        
        /* Send the updated handle as part of attribute for notifications */
        Cy_BLE_GATTS_Notification(&proximityNotificationHandle);
    }
}

/*******************************************************************************
* Function Name: void HandleDisconnectEventforProximity(void)
********************************************************************************
* Summary:
*  This functions handles the 'disconnect' event for the Proximity service
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void HandleDisconnectEventforProximity(void)
{
    /* Reset CapSense proximity notification flag to prevent further notifications
       being sent to Central device after next connection. */
    proximityNotificationStatus = CCCD_NOTIFY_DISABLED;
    
    /* Update the corresponding CCCD value in GATT DB */
    UpdateCccdStatusInGattDb(PROXIMITY_CCCD_HANDLE, proximityNotificationStatus);
}

/*******************************************************************************
* Function Name: void HandleWriteRequestforProximity
*                     (cy_stc_ble_gatts_write_cmd_req_param_t *writeRequest)
********************************************************************************
* Summary:
*  This functions handles the 'write request' event for the Proximity service
*
* Parameters:
*  writeRequest : pointer to the write request parameters from the central        
*
* Return:
*  void
*
*******************************************************************************/
void HandleWriteRequestforProximity(cy_stc_ble_gatts_write_cmd_req_param_t
    *writeRequest)
{
    /* Check the validity and then extract the write value sent by the Client 
       for CapSense Proximity CCCD */
    if ((writeRequest->handleValPair.value.val[PROXIMITY_CCCD_INDEX] 
            == CCCD_NOTIFY_DISABLED)||
        (writeRequest->handleValPair.value.val[PROXIMITY_CCCD_INDEX]
            == CCCD_NOTIFY_ENABLED))
    {
        proximityNotificationStatus = writeRequest->
                                    handleValPair.value.val[PROXIMITY_CCCD_INDEX] ;
        
        /* Update the corresponding CCCD value in GATT DB */
        UpdateCccdStatusInGattDb(PROXIMITY_CCCD_HANDLE, proximityNotificationStatus);
    }
}

/*******************************************************************************
* Function Name: void StackEventHandler(uint32_t event, void *eventParameter)
********************************************************************************
* Summary:
*  Call back event function to handle various events from the BLE stack
*
* Parameters:
*  event            :	event returned
*  eventParameter   :	link to value of the events returned
*
* Return:
*  void
*
*******************************************************************************/
void StackEventHandler(uint32_t event, void *eventParameter)
{
    /* Local variable to store the data received as part of the write request
       events */
    cy_stc_ble_gatts_write_cmd_req_param_t   *writeReqParameter;
    
    /* Take an action based on the current event */
    switch (event)
    {
        /* This event is received when the BLE stack is Started */
        case CY_BLE_EVT_STACK_ON:
            
            /* Set restartBleAdvertisement flag to allow calling advertisement
               API */
            restartBleAdvertisement = true;
            break;
       
        /*~~~~~~~~~~~~~~~~~~~~~~ GATT EVENTS ~~~~~~~~~~~~~~~~~~~~~~~~~~*/
        
        /* This event is received when device is connected over GATT level */    
        case CY_BLE_EVT_GATT_CONNECT_IND:
            
            /* Update attribute handle on GATT Connection */
            connectionHandle = *(cy_stc_ble_conn_handle_t *) eventParameter;
            break;
        
        /* This event is received when device is disconnected */
        case CY_BLE_EVT_GATT_DISCONNECT_IND:

            /* Call the functions that handle the disconnect events the
               proximity custom service */
            HandleDisconnectEventforProximity();
            break;
        
        /* This event is received when Central device sends a Write command
           on an Attribute */
        case CY_BLE_EVT_GATTS_WRITE_REQ:
            
            /* Read the write request parameter */
            writeReqParameter = (cy_stc_ble_gatts_write_cmd_req_param_t *) 
                                eventParameter;

            /* When this event is triggered, the peripheral has received a 
               write command on the custom  characteristic. Check if command
               fits the custom attributes and update the flag for sending 
               notifications by the service */
            if (PROXIMITY_CCCD_HANDLE
                == writeReqParameter->handleValPair.attrHandle)
            {
                HandleWriteRequestforProximity(writeReqParameter);
            }
            
            /* Send the response to the write request received. */
            Cy_BLE_GATTS_WriteRsp(connectionHandle);
            break;
        
        /*~~~~~~~~~~~~~~~~~~~~~~ OTHER EVENTS ~~~~~~~~~~~~~~~~~~~~~~~~~~*/ 
            
        /* Do nothing for all other events */
        default:
            break;
    }
}

/*******************************************************************************
* Function Name: void ProcessBleEvents(void)
********************************************************************************
* Summary:
*  Function that continuously process the BLE events and handles custom BLE 
*  services
*
* Parameters:
*  None
*
* Return:
*  Void
*
*******************************************************************************/
void ProcessBleEvents(void)
{  
    /* Variable used to control status LEDs */
    status_led_indication_t static bleStatusIndication = NO_INDICATION;

    /* Process event callback to handle BLE events. The events generated 
	   and used for this application are inside the 'StackEventHandler' 
       routine */
    Cy_BLE_ProcessEvents();

    /* BLE is connected */
	if(Cy_BLE_GetConnectionState(connectionHandle) == 
        CY_BLE_CONN_STATE_CONNECTED)
	{
        /* Update the status LEDs if they're not indicating the connection
        already */
        if(bleStatusIndication != INDICATE_CONNECTION)
        {
            bleStatusIndication = INDICATE_CONNECTION;
            UpdateStatusLEDs(bleStatusIndication);
        }
        
        /* Check the Proximity service notification is enabled */
        if (proximityNotificationStatus == CCCD_NOTIFY_ENABLED)
        {
            /* Variable used to read CapSense proximity data */
            proximity_data_t* proxData;

            /* Get the current CapSense proximity data */
            proxData = GetProximityData();
        
            /* Check if the proximity data is updated */
            if(proxData->proximityDataUpdated == true)
            {
                /* Send data over proximity notification */
                SendCapSenseProximityNotification(proxData->proximityData);
            }
		}
	}
    /* BLE is advertising */
    else if (Cy_BLE_GetAdvertisementState() == CY_BLE_ADV_STATE_ADVERTISING)
    {
        /* Update the status LEDs if they're not indicating the advertisement
        already */
        if(bleStatusIndication != INDICATE_ADVERTISEMENT)
        {
            bleStatusIndication = INDICATE_ADVERTISEMENT;
            UpdateStatusLEDs(bleStatusIndication);
        }
    }
    else
    {
        /* If previously indication was a connected state, show disconnected 
           state */
        if (bleStatusIndication == INDICATE_CONNECTION)
        {
            bleStatusIndication = INDICATE_DISCONNECTION;
            UpdateStatusLEDs(bleStatusIndication);
        }
        /* Otherwise, no indication required */
        else
        {   
            bleStatusIndication = NO_INDICATION;
            UpdateStatusLEDs(bleStatusIndication);
        }
        
         /* Check if the advertisement needs to be restarted */
    	if(restartBleAdvertisement)
    	{
    		/* Reset 'restartBleAdvertisement' flag */
    		restartBleAdvertisement = false;
    		
    		/* Start Advertisement and enter discoverable mode */
    		Cy_BLE_GAPP_StartAdvertisement
             (CY_BLE_ADVERTISING_FAST,CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);	
    	}
    }
}

/*******************************************************************************
* Function Name: void InitBle(void)
********************************************************************************
*
* Summary:
*  Function that initializes the BLE component with a custom event handler
*
* Parameters:
*  None
*
* Return:
*  None
*
* Side Effects:
*  None
*
*******************************************************************************/
void InitBle(void)
{
    /* Start the BLE component and register the stack event handler */
    Cy_BLE_Start(StackEventHandler);
    
}

/*******************************************************************************
* Function Name: void RestartBleAdvertisement (void)
********************************************************************************
*
* Summary:
*  Function that restarts BLE advertisement after a timeout or a disconnect 
*  event
*
* Parameters:
*  None
*
* Return:
*  None
*
* Side Effects:
*  None
*
*******************************************************************************/
void RestartBleAdvertisement (void)
{
    /* Set the restart advertisement flag */
    restartBleAdvertisement = true;
}

/*******************************************************************************
* Function Name: bool isBLEreadyForLPM(void)
********************************************************************************
*
* Summary:
*  Function to check if the BLE is ready to enter low power mode
*
* Parameters:
*  None
*
* Return:
*  bool     : true if ready to enter low power mode, false otherwise
*
* Side Effects:
*  None
*
*******************************************************************************/
bool  IsBleReadyForLowPowerMode(void)
{
    /* Variable that stores the return flag */
    bool lowPowerModeReady = false;
    
    /* Don't enter low power mode if the stack is not on */
    if(Cy_BLE_GetState() == CY_BLE_STATE_ON)
    {
        /* Don't enter the low power mode if BLE is connected and the proximity
           notification is enabled */
        if((Cy_BLE_GetNumOfActiveConn() != 0u)&& \
           (proximityNotificationStatus == CCCD_NOTIFY_ENABLED))
        {
            lowPowerModeReady=false;
        }
        else
        {
            lowPowerModeReady=true;
        }
    }
    
    /* Return  the low power mode entry readiness */
    return lowPowerModeReady;
}

/* [] END OF FILE */
