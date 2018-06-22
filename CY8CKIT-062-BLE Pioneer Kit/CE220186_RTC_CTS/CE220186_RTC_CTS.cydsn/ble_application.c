/******************************************************************************
* File Name: ble_application.c
*
* Version: 1.10
*
* Description: This file contains functions that handle BLE stack and 
*              the current time service
*
* Related Document: CE220186_RTC_CTS.pdf
*
* Hardware Dependency: CY8CKIT-062-BLE PSoC 6 BLE Pioneer Kit
*                      CY8CKIT-028-EPD EINK Display Shield
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
* This file contains functions that handle BLE stack and the current time service
*******************************************************************************/

/* Header file includes */
#include  "ble_application.h"
#include  "real_time_clock.h"
#include  "led.h"

/* These variable store BLE connection parameters */
cy_stc_ble_conn_handle_t                connectionHandle;
cy_stc_ble_gap_peer_addr_info_t         peerAddrInfo;

/* Structures that store CTS data */
time_and_date_t                         currentTimeAndDate;
cy_stc_ble_cts_local_time_info_t        localTimeInfo;
cy_stc_ble_cts_reference_time_info_t    referenceTime;

/* This is used to restart advertisement after a timeout or disconnection */
bool    restartBleAdvertisement        =   false;

/*******************************************************************************
* Function Name: void CallBackCts(uint32 event, void *eventParam)
********************************************************************************
*
* Summary:
*  This is an event callback function to receive events from the BLE Component,
*  which are specific to Current Time Service.
*
* Parameters:  
*  event:       Event for Current Time Service.
*  eventParams: Event parameter for Current Time Service.
*
* Return: 
*  None
*
*******************************************************************************/
void CallBackCts(uint32 event, void *eventParam)
{
    /* Variables that store CTS parameters */
    cy_stc_ble_cts_char_value_t *timeAttribute;
    cy_stc_ble_cts_char_value_t * ntfValParam;
    
    /* This is a CTS specific event triggered by the BLE component */
    switch(event)
    {
    /* Event to read CTS characteristics */
    case CY_BLE_EVT_CTSC_READ_CHAR_RESPONSE:
        
        /* Read the event parameter */
        timeAttribute = (cy_stc_ble_cts_char_value_t *) eventParam;
        
        /* Copy the received current time from the time server to local data structure
           and then write to the RTC */
        if(timeAttribute->charIndex == CY_BLE_CTS_CURRENT_TIME)
        {
            memcpy(&currentTimeAndDate, timeAttribute->value->val, timeAttribute->value->len);
            SetRtc(&currentTimeAndDate);
        }
         /* Copy the received local time info from the time server to local data structure */
        else if(timeAttribute->charIndex == CY_BLE_CTS_LOCAL_TIME_INFO)
        {
            memcpy(&localTimeInfo, timeAttribute->value->val, timeAttribute->value->len);
        }
        /* Copy the received reference time from the time server to local data structure */
        else if(timeAttribute->charIndex == CY_BLE_CTS_REFERENCE_TIME_INFO)
        {
            memcpy(&referenceTime, timeAttribute->value->val, timeAttribute->value->len);
        }
        break;
        
     /* Event to receive CTS notification */    
    case CY_BLE_EVT_CTSC_NOTIFICATION:
        
        /* Read the event parameter */
        ntfValParam = (cy_stc_ble_cts_char_value_t *) eventParam;
        
        /* Copy the current time received from the time server to local data structure  */
        memcpy(&currentTimeAndDate, ntfValParam->value->val, ntfValParam->value->len);
        SetRtc(&currentTimeAndDate);  
        break;
        
    /* Event to send Read request for Current Time characteristic to the Time Server */       
    case CY_BLE_EVT_CTSC_WRITE_DESCR_RESPONSE:
        
        Cy_BLE_CTSC_GetCharacteristicValue(connectionHandle, CY_BLE_CTS_CURRENT_TIME);
        break;

    default:
        break;
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
void StackEventHandler(uint32_t event, void *eventParam)
{
    /**********************************************************
    *        Variables used by the stack handler
    ***********************************************************/
    uint16_t count;
    cy_stc_ble_gap_auth_info_t *authInfo;
    cy_stc_ble_gap_peer_addr_info_t bondedDeviceInfo[CY_BLE_GAP_MAX_BONDED_DEVICE];
    cy_stc_ble_gap_bonded_device_list_info_t bdList =
    {
        .bdHandleAddrList = bondedDeviceInfo
    };
    static cy_stc_ble_gap_sec_key_info_t keyInfo =
    {
        .localKeysFlag    = CY_BLE_GAP_SMP_INIT_ENC_KEY_DIST | 
                            CY_BLE_GAP_SMP_INIT_IRK_KEY_DIST | 
                            CY_BLE_GAP_SMP_INIT_CSRK_KEY_DIST,
        .exchangeKeysFlag = CY_BLE_GAP_SMP_INIT_ENC_KEY_DIST | 
                            CY_BLE_GAP_SMP_INIT_IRK_KEY_DIST | 
                            CY_BLE_GAP_SMP_INIT_CSRK_KEY_DIST|
                            CY_BLE_GAP_SMP_RESP_ENC_KEY_DIST |
                            CY_BLE_GAP_SMP_RESP_IRK_KEY_DIST |
                            CY_BLE_GAP_SMP_RESP_CSRK_KEY_DIST,
    };
    
    /* Handle the BLE events */
    switch(event)
    {
        /**********************************************************
        *                       Generic and HCI Events
        ***********************************************************/
        case CY_BLE_EVT_STACK_ON: 
            
            /* Generate the keys required for authentication */
            Cy_BLE_GAP_GenerateKeys(&keyInfo);
            
            /* Request to restart advertisement */
            restartBleAdvertisement = true;
            break;
        
        case CY_BLE_EVT_SET_DEVICE_ADDR_COMPLETE:
            
            /* Reads the BD device address from BLE Controller's memory */
            Cy_BLE_GAP_GetBdAddress();
            break;
       
        /**********************************************************
        *                       GAP Events
        ***********************************************************/
        case CY_BLE_EVT_GAP_AUTH_REQ:
            
            /* Handle the GAP authentication request */
            if(cy_ble_configPtr->
               authInfo[CY_BLE_SECURITY_CONFIGURATION_0_INDEX].security 
               == (CY_BLE_GAP_SEC_MODE_1 | CY_BLE_GAP_SEC_LEVEL_1))
            {
                cy_ble_configPtr->authInfo
                [CY_BLE_SECURITY_CONFIGURATION_0_INDEX].authErr 
                = CY_BLE_GAP_AUTH_ERROR_PAIRING_NOT_SUPPORTED;
            }
            
            cy_ble_configPtr->
            authInfo[CY_BLE_SECURITY_CONFIGURATION_0_INDEX].bdHandle 
            = ((cy_stc_ble_gap_auth_info_t *)eventParam)->bdHandle;
            
            if(Cy_BLE_GAPP_AuthReqReply(&cy_ble_configPtr->
               authInfo[CY_BLE_SECURITY_CONFIGURATION_0_INDEX])
               != CY_BLE_SUCCESS)
            {
                if(Cy_BLE_GAP_RemoveOldestDeviceFromBondedList() 
                   == CY_BLE_SUCCESS)
                {
                     Cy_BLE_GAPP_AuthReqReply(&cy_ble_configPtr->
                     authInfo[CY_BLE_SECURITY_CONFIGURATION_0_INDEX]);            
                }
            }
            break;

        case CY_BLE_EVT_GAP_PASSKEY_DISPLAY_REQUEST:
           
            /* Get the BD address of currently connected device and get the list 
               of bounded devices */
            peerAddrInfo.bdHandle = connectionHandle.bdHandle;
            (void) Cy_BLE_GAP_GetPeerBdAddr(&peerAddrInfo);  
            (void) Cy_BLE_GAP_GetBondList(&bdList); 

            for(count = 0u; count < bdList.noOfDevices; count ++)
            {
                /* Check if any address from the bonded devices list equals to
                   currently connected device */
                if(memcmp((const void *) &peerAddrInfo, 
                  (const void *) &bdList.bdHandleAddrList[count],
                   sizeof(cy_stc_ble_gap_bd_addr_t)) == 0u)
                {
                    /* If the matching device address was found then remove the 
                       record associated with current address from the bonded 
                       list, since the bonding information was cleared on the 
                       peer device and it is required to generate new keys. The 
                       bounding record contains stored keys */
                    (void)Cy_BLE_GAP_RemoveDeviceFromBondList
                    (&bdList.bdHandleAddrList[count].bdAddr);
                }
            }                  
            break;

        case CY_BLE_EVT_GAP_AUTH_COMPLETE:
            
            /* Start the discovery if the authentication has been completed */
            authInfo = (cy_stc_ble_gap_auth_info_t *)eventParam;
            (void)authInfo;
            Cy_BLE_GATTC_StartDiscovery(connectionHandle);
            break;

        case CY_BLE_EVT_GAP_DEVICE_CONNECTED:
            
            /* Fetch the security keys */
            keyInfo.SecKeyParam.bdHandle = 
            (*(cy_stc_ble_gap_connected_param_t *)eventParam).bdHandle;
            Cy_BLE_GAP_SetSecurityKeys(&keyInfo);
            
            /* Send authentication request to peer device */
            cy_ble_configPtr->
            authInfo[CY_BLE_SECURITY_CONFIGURATION_0_INDEX].bdHandle 
            = connectionHandle.bdHandle;
            Cy_BLE_GAP_AuthReq(&cy_ble_configPtr->
            authInfo[CY_BLE_SECURITY_CONFIGURATION_0_INDEX]);
            break;

        case CY_BLE_EVT_GAP_KEYS_GEN_COMPLETE:
            
            /* Set the ID address */
            keyInfo.SecKeyParam = (*(cy_stc_ble_gap_sec_key_param_t *)eventParam);
            Cy_BLE_GAP_SetIdAddress(&cy_ble_deviceAddress);
            break;
            
        /**********************************************************
        *                       GATT Events
        ***********************************************************/
        case CY_BLE_EVT_GATT_CONNECT_IND:
            connectionHandle = *(cy_stc_ble_conn_handle_t *)eventParam;
            break;

        case CY_BLE_EVT_GATTS_XCNHG_MTU_REQ:
            { 
                cy_stc_ble_gatt_xchg_mtu_param_t mtu = 
                {
                    .connHandle = ((cy_stc_ble_gatt_xchg_mtu_param_t *)
                                    eventParam)->connHandle
                };
                Cy_BLE_GATT_GetMtuSize(&mtu);
            }
            break;

        case CY_BLE_EVT_GATTC_DISCOVERY_COMPLETE:
            {
                uint32_t discIdx = 
                Cy_BLE_GetDiscoveryIdx(*(cy_stc_ble_conn_handle_t *)eventParam);
                
                if(cy_ble_serverInfo[discIdx][CY_BLE_SRVI_CTS].range.startHandle != 0u)
                {
                    /* Enable Notification for Current Time Characteristic. */
                    uint16_t timeCCCD = CY_BLE_CCCD_NOTIFICATION;
                    Cy_BLE_CTSC_SetCharacteristicDescriptor
                        (connectionHandle,
                         CY_BLE_CTS_CURRENT_TIME,
                         CY_BLE_CTS_CURRENT_TIME_CCCD,
                         CY_BLE_CCCD_LEN,
                         (uint8_t *)&timeCCCD);
                }
                break;
            }
       
    /**********************************************************
    *                       Other Events
    ***********************************************************/
        default:
           
    		break;
    }
}

/*******************************************************************************
* Function Name: void ProcessBleEvents(void)
********************************************************************************
*
* Summary:
*  Function that processes BLE events and restarts advertisement if required 
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
*  Function that initializes the BLE component and registers CTS callback 
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
    
    /* Registers the CTS specific callback handler */
    Cy_BLE_CTS_RegisterAttrCallback(CallBackCts); 
}

/*******************************************************************************
* Function Name: void RestartBleAvertisement(void)
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
void RestartBleAvertisement(void)
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
    
    /* Enter low power mode if the stack on */
    if(Cy_BLE_GetState() == CY_BLE_STATE_ON)
    {
        lowPowerModeReady=true;
    }
    
    /* Return  the low power mode entry readiness */
    return lowPowerModeReady;
}

/* [] END OF FILE */
