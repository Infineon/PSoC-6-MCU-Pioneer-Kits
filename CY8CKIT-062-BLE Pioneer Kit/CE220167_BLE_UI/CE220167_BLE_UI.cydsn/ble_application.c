/******************************************************************************
* File Name: ble_application.c
*
* Version: 1.10
*
* Description: This file contains functions that handle custom BLE services
*
* Related Document: CE220167_BLE_UI.pdf
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
* This file contains functions that handle custom BLE services, which includes
* the CapSense Slider service, CapSense button service and the RGB LED service
*******************************************************************************/

/* Header file includes */
#include "ble_application.h"
#include "led.h"
#include "touch.h"

/* Indexes of a two-byte CCCD array */
#define CCCD_INDEX_0            (uint8_t) (0x00u)
#define CCCD_INDEX_1            (uint8_t) (0x01u)

/* Bit mask for the notification bit in CCCD (Client Characteristic Configuration 
   Descriptor), which is written by the client device */
#define CCCD_NOTIFY_ENABLED     (uint8_t) (0x01u)
#define CCCD_NOTIFY_DISABLED    (uint8_t) (0x00u)

/* Redefinition of long CCCD handles and indexes for better readability */
#define SLIDER_CCCD_HANDLE      \
(CY_BLE_CAPSENSE_SLIDER_CAPSENSE_SLIDER_CLIENT_CHARACTERISTIC_CONFIGURATION_DESC_HANDLE)
#define BUTTON_CCCD_HANDLE      \
(CY_BLE_CAPSENSE_BUTTON_CAPSENSE_BUTTON_CLIENT_CHARACTERISTIC_CONFIGURATION_DESC_HANDLE)
#define RGB_CCCD_HANDLE         \
(CY_BLE_RGB_LED_RGB_LED_CONTROL_CLIENT_CHARACTERISTIC_CONFIGURATION_DESC_HANDLE)
#define SLIDER_CCCD_INDEX       \
(CY_BLE_CAPSENSE_SLIDER_CAPSENSE_SLIDER_CLIENT_CHARACTERISTIC_CONFIGURATION_DESC_INDEX)
#define BUTTON_CCCD_INDEX       \
(CY_BLE_CAPSENSE_BUTTON_CAPSENSE_BUTTON_CLIENT_CHARACTERISTIC_CONFIGURATION_DESC_INDEX)
#define RGB_CCCD_INDEX          \
(CY_BLE_RGB_LED_RGB_LED_CONTROL_CLIENT_CHARACTERISTIC_CONFIGURATION_DESC_INDEX)

/* 'connectionHandle' stores the BLE connection parameters */
cy_stc_ble_conn_handle_t static connectionHandle;

/* Array to store the present RGB LED control data. The four bytes of the array 
   represent {Red, Green, Blue, Intensity} respectively */
uint8_t static rgbData[RGB_DATA_LEN];

/* These flags are enabled when the Central device writes to CCCD (Client 
   Characteristic Configuration Descriptor) of the CapSense and RGB LED 
   Characteristics to enable notifications */
uint8_t static sliderNotificationStatus  = CCCD_NOTIFY_DISABLED;
uint8_t static buttonNotificationStatus  = CCCD_NOTIFY_DISABLED;
uint8_t static rgbNotificationStatus     = CCCD_NOTIFY_DISABLED;

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
* Function Name: void SendCapSenseSliderNotification(uint8_t sliderData)
********************************************************************************
* Summary:
*  Send CapSense Slider data as BLE Notifications. This function updates
*  the notification handle with data and triggers the BLE component to send
*  notification
*
* Parameters:
*  sliderData:	CapSense slider value
*
* Return:
*  void
*
*******************************************************************************/
void SendCapSenseSliderNotification(uint8_t sliderData)
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
        cy_stc_ble_gatts_handle_value_ntf_t sliderNotificationHandle =
        {
            .connHandle = connectionHandle,
            .handleValPair.attrHandle = 
                            CY_BLE_CAPSENSE_SLIDER_CAPSENSE_SLIDER_CHAR_HANDLE,
            .handleValPair.value.val = &sliderData,
            .handleValPair.value.len = CAPSENSE_SLIDER_DATA_LEN
        };
        
        /* Send the updated handle as part of attribute for notifications */
        Cy_BLE_GATTS_Notification(&sliderNotificationHandle);
    }
}

/*******************************************************************************
* Function Name: void SendCapSenseButtonNotification(uint8_t *buttonData)
********************************************************************************
* Summary:
*  Send CapSense Button data as BLE Notifications. This function updates
*  the notification handle with data and triggers the BLE component to send
*  notification
*
* Parameters:
*  buttonData:	CapSense slider value
*
* Return:
*  void
*
*******************************************************************************/
void SendCapSenseButtonNotification(uint8_t *buttonData)
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
        cy_stc_ble_gatts_handle_value_ntf_t buttonNotificationHandle =
        {
            .connHandle = connectionHandle,
            .handleValPair.attrHandle = 
                            CY_BLE_CAPSENSE_BUTTON_CAPSENSE_BUTTON_CHAR_HANDLE,
            .handleValPair.value.val = buttonData,
            .handleValPair.value.len = CAPSENSE_BUTTON_DATA_LEN
        };

        /* Send the updated handle as part of attribute for notifications */
        Cy_BLE_GATTS_Notification(&buttonNotificationHandle);
    }
}

/*******************************************************************************
* Function Name: void SendRgbNotification(uint8_t *rgbData)
********************************************************************************
* Summary:
*  Send RGB LED data as BLE Notifications. This function updates
*  the notification handle with data and triggers the BLE component to send
*  notification
*
* Parameters:
*  rgbData:	pointer to an array containing RGB color and Intensity values
*
* Return:
*  void
*
*******************************************************************************/
void SendRgbNotification(uint8_t *rgbData)
{
    /* Make sure that stack is not busy, then send the notification. Note that 
       the number of buffers in the BLE stack that holds the application data 
       payload are limited, and there are chances that notification might drop 
       a packet if the BLE stack buffers are not available. This error condition
       is not handled in this example project */
    if (Cy_BLE_GATT_GetBusyStatus(connectionHandle.attId) 
                                                     == CY_BLE_STACK_STATE_FREE)
    {
        /* Local variable that stores RGB LED notification data parameters */
        cy_stc_ble_gatts_handle_value_ntf_t rgbNotificationHandle = 
        {
            .connHandle = connectionHandle,
            .handleValPair.attrHandle = 
                                 CY_BLE_RGB_LED_RGB_LED_CONTROL_CHAR_HANDLE,
            .handleValPair.value.val = rgbData,
            .handleValPair.value.len = RGB_DATA_LEN
        };

        /* Send the updated handle as part of attribute for notifications */
        Cy_BLE_GATTS_Notification(&rgbNotificationHandle);
    }
}
/*******************************************************************************
* Function Name: void UpdateRgb(void)
********************************************************************************
* Summary:
*  Receive the new RGB data and update the read characteristic handle so that the
*  next read from the BLE central device gives the current RGB color and intensity 
*  data. This function also calls SetColorRgb() function to set the color of the
*  RGB LED per the current value
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void UpdateRgb(void)
{
    /* Local variable that stores RGB control data parameters */
    cy_stc_ble_gatt_handle_value_pair_t  rgbHandle = 
    {
        .attrHandle = CY_BLE_RGB_LED_RGB_LED_CONTROL_CHAR_HANDLE,
        .value.val  = rgbData,
        .value.len  = RGB_DATA_LEN 
    };
    
    /* Local variable that stores RGB attribute value */
    cy_stc_ble_gatts_db_attr_val_info_t  rgbAttributeValue = 
    {
        .handleValuePair = rgbHandle,
        .offset = CY_BLE_CCCD_DEFAULT,
        .connHandle = connectionHandle,
        .flags = CY_BLE_GATT_DB_LOCALLY_INITIATED
    };

    /* Send updated RGB control handle as an attribute to the central device, 
       so that the central reads the new RGB color data */
    Cy_BLE_GATTS_WriteAttributeValue(&rgbAttributeValue);
    
    /* Set the color of the RGB LED to match the current values */
    SetColorRgb(rgbData);
}

/*******************************************************************************
* Function Name: void HandleDisconnectEventforSlider(void)
********************************************************************************
* Summary:
*  This functions handles the 'disconnect' event for the Slider service
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void HandleDisconnectEventforSlider(void)
{
    /* Reset CapSense slider notification flag to prevent further notifications
       being sent to Central device after next connection. */
    sliderNotificationStatus = CCCD_NOTIFY_DISABLED;
    
    /* Update the corresponding CCCD value in GATT DB */
    UpdateCccdStatusInGattDb(SLIDER_CCCD_HANDLE, sliderNotificationStatus);
}

/*******************************************************************************
* Function Name: void HandleDisconnectEventforButtons(void)
********************************************************************************
* Summary:
*  This functions handles the 'disconnect' event for the Button service
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void HandleDisconnectEventforButtons(void)
{
    /* Reset CapSense button notification flag to prevent further notifications
       being sent to Central device after next connection. */
    buttonNotificationStatus = CCCD_NOTIFY_DISABLED;
    
    /* Update the corresponding CCCD value in GATT DB */
    UpdateCccdStatusInGattDb(BUTTON_CCCD_HANDLE, buttonNotificationStatus);
}

/*******************************************************************************
* Function Name: void HandleDisconnectEventforRgb(void)
********************************************************************************
* Summary:
*  This functions handles the 'disconnect' event for the RGB LED service
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void HandleDisconnectEventforRgb(void)
{
    /* Reset RGB notification flag to prevent further notifications
       being sent to Central device after next connection. */
    rgbNotificationStatus = CCCD_NOTIFY_DISABLED;
    
    /* Update the corresponding CCCD value in GATT DB */
    UpdateCccdStatusInGattDb(RGB_CCCD_HANDLE, rgbNotificationStatus);

    /* Reset the color coordinates */
    rgbData[RED_INDEX]       = RGB_LED_OFF;
    rgbData[GREEN_INDEX]     = RGB_LED_OFF;
    rgbData[BLUE_INDEX]      = RGB_LED_OFF;
    rgbData[INTENSITY_INDEX] = RGB_LED_OFF;
    
    /* Set the color of the RGB LED to match the current values */
    UpdateRgb();
}

/*******************************************************************************
* Function Name: void HandleWriteRequestforSlider
*                     (cy_stc_ble_gatts_write_cmd_req_param_t *writeRequest)
********************************************************************************
* Summary:
*  This functions handles the 'write request' event for the Slider service
*
* Parameters:
*  writeRequest : pointer to the write request parameters from the central        
*
* Return:
*  void
*
*******************************************************************************/
void HandleWriteRequestforSlider(cy_stc_ble_gatts_write_cmd_req_param_t
    *writeRequest)
{
    /* Check the validity and then extract the write value sent by the Client 
       for CapSense Slider CCCD */
    if ((writeRequest->handleValPair.value.val[SLIDER_CCCD_INDEX] 
            == CCCD_NOTIFY_DISABLED)||
        (writeRequest->handleValPair.value.val[SLIDER_CCCD_INDEX]
            == CCCD_NOTIFY_ENABLED))
    {
        sliderNotificationStatus = writeRequest->
                                    handleValPair.value.val[SLIDER_CCCD_INDEX] ;
        
        /* Update the corresponding CCCD value in GATT DB */
        UpdateCccdStatusInGattDb(SLIDER_CCCD_HANDLE, sliderNotificationStatus);
    }
}

/*******************************************************************************
* Function Name: void HandleWriteRequestforButtons
*                     (cy_stc_ble_gatts_write_cmd_req_param_t *writeRequest)
********************************************************************************
* Summary:
*  This functions handles the 'write request' event for the Button service
*
* Parameters:
*  writeRequest : pointer to the write request parameters from the central       
*
* Return:
*  void
*
*******************************************************************************/
void HandleWriteRequestforButtons(cy_stc_ble_gatts_write_cmd_req_param_t
                                   *writeRequest)
{
    /* Check the validity and then extract the write value sent by the Client 
       for CapSense Button CCCD */
    if ((writeRequest->handleValPair.value.val[BUTTON_CCCD_INDEX] 
            == CCCD_NOTIFY_DISABLED)||
        (writeRequest->handleValPair.value.val[BUTTON_CCCD_INDEX]
            == CCCD_NOTIFY_ENABLED))
    {
        buttonNotificationStatus = writeRequest->
                                    handleValPair.value.val[BUTTON_CCCD_INDEX] ;
        
         /* Update the corresponding CCCD value in GATT DB */
        UpdateCccdStatusInGattDb(BUTTON_CCCD_HANDLE, buttonNotificationStatus);
    }
}

/*******************************************************************************
* Function Name: void HandleWriteRequestforRgb
*                     (cy_stc_ble_gatts_write_cmd_req_param_t *writeRequest)
********************************************************************************
* Summary:
*  This functions handles the 'write request' event for the RGB LED service
*
* Parameters:
*  writeRequest : pointer to the write request parameters from the central        
*
* Return:
*  void
*
*******************************************************************************/
void HandleWriteRequestforRgb(cy_stc_ble_gatts_write_cmd_req_param_t 
                              *writeRequest)
{
    /* Check the validity and then extract the write value sent by the Client 
       for RGB LED CCCD */
    if (writeRequest->handleValPair.attrHandle == RGB_CCCD_HANDLE && 
        ((writeRequest->handleValPair.value.val[RGB_CCCD_INDEX] 
            == CCCD_NOTIFY_DISABLED)||
        (writeRequest->handleValPair.value.val[RGB_CCCD_INDEX]
            == CCCD_NOTIFY_ENABLED)))
    {
        rgbNotificationStatus = writeRequest->
                                    handleValPair.value.val[RGB_CCCD_INDEX] ;
        
         /* Update the corresponding CCCD value in GATT DB */
        UpdateCccdStatusInGattDb(RGB_CCCD_HANDLE, rgbNotificationStatus);
    }
    
    /* Check if the notification is enabled for RGB LED service */
    if (rgbNotificationStatus == CCCD_NOTIFY_ENABLED)
    {
        /* Update the RGB LED Notification attribute with new color
           coordinates */
        SendRgbNotification(rgbData);
    }
    
    /* Check if the returned handle is matching to RGB LED Control Write
       Attribute and extract the RGB data*/
    if (writeRequest->handleValPair.attrHandle == 
            CY_BLE_RGB_LED_RGB_LED_CONTROL_CHAR_HANDLE)
    {
        /* Extract the write value sent by the Client for RGB LED Color 
           characteristic */
        rgbData[RED_INDEX] = 
                writeRequest->handleValPair.value.val[RED_INDEX];
        rgbData[GREEN_INDEX] = 
                writeRequest->handleValPair.value.val[GREEN_INDEX];
        rgbData[BLUE_INDEX] = 
                writeRequest->handleValPair.value.val[BLUE_INDEX];
        rgbData[INTENSITY_INDEX] = 
                writeRequest->handleValPair.value.val[INTENSITY_INDEX];

        /* Update the the attribute for RGB LED read characteristics and
           set the color of the LED per the received value */
        UpdateRgb();
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

            /* Call the functions that handle the disconnect events for all 
               three custom services */
            HandleDisconnectEventforSlider();
            HandleDisconnectEventforButtons();
            HandleDisconnectEventforRgb();
            break;
        
        /* This event is received when Central device sends a Write command
           on an Attribute */
        case CY_BLE_EVT_GATTS_WRITE_REQ:
            
            /* Read the write request parameter */
            writeReqParameter = (cy_stc_ble_gatts_write_cmd_req_param_t *) 
                                eventParameter;

            /* When this event is triggered, the peripheral has received a 
               write command on the custom  characteristic. Check if command
               fits any of the custom attributes and update the flag for
               sending notifications by the respective service */
            if (SLIDER_CCCD_HANDLE
                == writeReqParameter->handleValPair.attrHandle)
            {
                HandleWriteRequestforSlider(writeReqParameter);
            }
            if (BUTTON_CCCD_HANDLE
                == writeReqParameter->handleValPair.attrHandle)
            {
                HandleWriteRequestforButtons(writeReqParameter);
            }
            if (RGB_CCCD_HANDLE
                == writeReqParameter->handleValPair.attrHandle ||
                CY_BLE_RGB_LED_RGB_LED_CONTROL_CHAR_HANDLE
                == writeReqParameter->handleValPair.attrHandle)
            {
                HandleWriteRequestforRgb(writeReqParameter);
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
        
        /* Check if any of the BLE CapSense service notifications
           are enabled */
        if ((sliderNotificationStatus == CCCD_NOTIFY_ENABLED)||
            (buttonNotificationStatus == CCCD_NOTIFY_ENABLED))
        {
            /* Variable used to read CapSense touch data */
            touch_data_t* capSenseData;

            /* Get the current CapSense touch data */
            capSenseData = GetcapSenseData();
        
            /* Check if the slider data is updated and slider notifications
               are enabled */
            if((capSenseData->sliderDataUpdated == true)&&
               (sliderNotificationStatus == CCCD_NOTIFY_ENABLED))
            {
                /* Send data over slider notification */
                SendCapSenseSliderNotification(capSenseData->sliderData);
            }
            /* Check if the button data is updated and button notifications
               are enabled */
            if((capSenseData->buttonDataUpdated== true)&&
               (buttonNotificationStatus == CCCD_NOTIFY_ENABLED))
            {
                /* Send data over button notification */
                SendCapSenseButtonNotification((uint8_t*)
                                                capSenseData->buttonData);  
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
        /* Don't enter the low power mode if BLE is connected */
        if(Cy_BLE_GetNumOfActiveConn() != 0u)
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
