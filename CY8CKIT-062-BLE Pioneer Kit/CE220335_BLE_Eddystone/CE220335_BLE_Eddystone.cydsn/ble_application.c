/******************************************************************************
* File Name: ble_application.c
*
* Version: 1.10
*
* Description: This file contains functions that handle BLE services
*
* Related Document: CE220335_BLE_Eddystone.pdf
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
* This file contains functions that handle a custom BLE service which advertises
* Eddystone beacon frames
*******************************************************************************/

/* Header file includes */
#include <string.h>
#include "ble_application.h"
#include "eddystone_config.h"
#include "led.h"
#include "time.h"

/* Macros to access BLE advertisement data and settings */
#define AdvertisementData       cy_ble_discoveryData\
                                [CY_BLE_BROADCASTER_CONFIGURATION_0_INDEX]\
                                .advData
                                
#define AdvertisementSize       cy_ble_discoveryData\
                                [CY_BLE_BROADCASTER_CONFIGURATION_0_INDEX]\
                                .advDataLen
                                
#define AdvertisementTimeOut    cy_ble_discoveryModeInfo\
                                [CY_BLE_BROADCASTER_CONFIGURATION_0_INDEX]\
                                .advTo
                                
/* Number of packets broadcasted during a UID/URL + TLM advertisement.
   This is equal to the total time out in seconds / 100 milliseconds or
   total time out *10 */                      
#define PACKETS_PER_BROADCAST   ((APP_UID_URL_TIMEOUT \
                                 +APP_TLM_TIMEOUT)*10)

/* Variables that store constant URL, UID and TLM packet data. To change 
   the Eddystone packet settings, see the header file eddystone_config.h */
uint8_t const solicitationData[] = SERVICE_SOLICITATION;
uint8_t const serviceDataUID[]   = UID_SERVICE_DATA;
uint8_t const nameSpaceID[]      = NAME_SPACE_ID;
uint8_t const instanceID[]       = INSTANCE_ID;
uint8_t const packetDataURL[]    = URL_PACKET_DATA;
uint8_t const serviceDataTLM[]   = TLM_SERVICE_DATA;
                                
/* Data type that stores advertisement packet count since power-up 
   or reset  */
typedef union
{
    /* 32-bit packet count for mathematical operations */
    uint32_t    count;
    /* Same packet count segmented into four 8-bit fields for copying 
       to the Eddystone frame */
    uint8_t     countByte[4u]; 
}   packet_count_t;

/* Enumerated data type for core Eddystone roles */
typedef enum
{
    /* Eddystone UID adv */
    EDDYSTONE_UID,
    /* Eddystone URL adv */
    EDDYSTONE_URL,
    /* Eddystone TLM adv */
    EDDYSTONE_TLM
}   eddystone_role_t;

/* Variable that stores the current role  (UID/URL or TLM). To change 
   the Eddystone packet settings, see the header file eddystone_config.h */
eddystone_role_t beaconCurrentRole = EDDYSTONE_IMPLEMENTATION;

/* variable that stores advertisement packet count since power-up or reset */
packet_count_t   packetCount;

/*******************************************************************************
* Function Name: void ConfigureAdvPacket(void)
********************************************************************************
* Summary:
*  Function that configures Eddystone packets at run-time
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void ConfigureAdvPacket(void)
{
   /* Note: To change the Eddystone packet settings, see the header file 
      eddystone_config.h */

    /* Variable  that stores the time elapsed since power-up or reset */
    uint8_t* secondCount;
    
    /* Load the service Solicitation data onto the Eddystone advertisement 
       packet */
    memcpy (&AdvertisementData[SOLICITATION_INDEX],
            &solicitationData, 
            (sizeof solicitationData));
    
    /* Check if the current Eddystone role is either UID or URL */
    if( (beaconCurrentRole == EDDYSTONE_UID) ||
        (beaconCurrentRole == EDDYSTONE_URL))
    {
        /* Turn the Red LED on and the Orange LED off */
        TurnOffOrange;
        TurnOnRed;
        
        /* Configure the advertisement timeout */
        AdvertisementTimeOut = APP_UID_URL_TIMEOUT;

        /* If the current role is UID, load the UID specific data onto the
           Eddystone advertisement packet */
        if(beaconCurrentRole == EDDYSTONE_UID)
        {
            /* Load Service Data */
            memcpy (&AdvertisementData[UID_SERVICE_INDEX], 
                    &serviceDataUID, 
                    (sizeof serviceDataUID));
           
            /* Load Name-space ID */
            memcpy (&AdvertisementData[NAME_SPACE_INDEX],
                    &nameSpaceID,
                    (sizeof nameSpaceID));

            /* Load Instance ID  */
             memcpy (&AdvertisementData[INSTANCE_INDEX],
                     &instanceID, 
                     (sizeof instanceID));
            
            /* Load the reserved fields with default values */
            AdvertisementData[RESERVED_INDEX_0] = RESERVED_FIELD_VALUE;              
            AdvertisementData[RESERVED_INDEX_1] = RESERVED_FIELD_VALUE;              
            
            /* Configure advertisement packet length */
            AdvertisementSize = UID_PACKET_SIZE;
        }
        /* If the current role is URL, load the URL specific data onto the
           Eddystone advertisement packet */
        else if(beaconCurrentRole == EDDYSTONE_URL)
        {    
            /* Load URL data */
            memcpy (&AdvertisementData[URL_INDEX],
                     &packetDataURL, 
                     (sizeof packetDataURL));
            
            /* Configure advertisement packet length */
            AdvertisementSize = URL_PACKET_SIZE;
        }
    }
    /* If the current role is TLM, load the TLM specific data onto the
       Eddystone advertisement packet */
    else if(beaconCurrentRole == EDDYSTONE_TLM)
    {
        /* Turn the Orange LED on and the Red LED off */
        TurnOffRed;
        TurnOnOrange;
        
        /* Get the time elapsed since power-up or reset (100 ms per bit) */
        secondCount = getSecondCount();
        
        /* Configure the advertisement timeout */
        AdvertisementTimeOut = APP_TLM_TIMEOUT;

        /* Load Service Data */
        memcpy (&AdvertisementData[TLM_SERVICE_INDEX],
                &serviceDataTLM,    
                (sizeof serviceDataTLM));
        
        /* Load battery voltage in mV (1 mV per bit) */
        AdvertisementData[BATTERY_MSB_INDEX] = BATTERY_VOLTAGE_MSB;     
        AdvertisementData[BATTERY_LSB_INDEX] = BATTERY_VOLTAGE_LSB;     

        /* Load beacon temperature in Celsius (8.8 fixed point notation) */
        AdvertisementData[TEMPERATURE_MSB_INDEX] = BEACON_TEMPERATURE_MSB;     
        AdvertisementData[TEMPERATURE_LSB_INDEX] = BEACON_TEMPERATURE_LSB;     

        /* Load advertising packet count since power-up or reset */
        AdvertisementData[PACKET_COUNT_INDEX_0] = packetCount.countByte
                                                  [TLM_4B_ENDIAN_SWAP_0];     
        AdvertisementData[PACKET_COUNT_INDEX_1] = packetCount.countByte
                                                  [TLM_4B_ENDIAN_SWAP_1];     
        AdvertisementData[PACKET_COUNT_INDEX_2] = packetCount.countByte
                                                  [TLM_4B_ENDIAN_SWAP_2];     
        AdvertisementData[PACKET_COUNT_INDEX_3] = packetCount.countByte
                                                  [TLM_4B_ENDIAN_SWAP_3];     
    
        /* Load time elapsed since power-on or reboot (100 ms per bit) */
        AdvertisementData[SECONDS_INDEX_0] = secondCount
                                             [TLM_4B_ENDIAN_SWAP_0];     
        AdvertisementData[SECONDS_INDEX_1] = secondCount
                                             [TLM_4B_ENDIAN_SWAP_1];     
        AdvertisementData[SECONDS_INDEX_2] = secondCount
                                             [TLM_4B_ENDIAN_SWAP_2];     
        AdvertisementData[SECONDS_INDEX_3] = secondCount
                                             [TLM_4B_ENDIAN_SWAP_3];     
                
        /* Configure advertisement packet length */
        AdvertisementSize = TLM_PACKET_SIZE;
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
*  None
*
*******************************************************************************/
void StackEventHandler(uint32_t event, void *eventParameter)
{
    /* Event parameter is not used in this application */
    (void) eventParameter;
    
    /* Take an action based on the current event */
    switch (event)
    {
        /* BLE stack is Started */
        case CY_BLE_EVT_STACK_ON:
            
            /* Note: To change the Eddystone packet settings, see the  
               header file eddystone_config.h */
            /* Check if the selected Eddystone implementation is valid.
               Halt the CPU otherwise */
            if ((EDDYSTONE_IMPLEMENTATION != EDDYSTONE_UID)&&
                (EDDYSTONE_IMPLEMENTATION != EDDYSTONE_URL))
            {
                CY_ASSERT(0u);
            }
            else
            {
                /* Start with the selected implementation */
                beaconCurrentRole = EDDYSTONE_IMPLEMENTATION;
            }
            
            /* Reset advertisement packet count and start tracking time */
            packetCount.count = 0x00000000u;
            startSecondCount();
            
            /* Configure Eddystone packets for the selected implementation */
            ConfigureAdvPacket();

            /* Start advertisement */
            Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_CUSTOM,
                            CY_BLE_BROADCASTER_CONFIGURATION_0_INDEX);	
            
            break; 

        /* Advertisement started or stopped */
        case CY_BLE_EVT_GAPP_ADVERTISEMENT_START_STOP:
            
            /* Advertisement timeout */
            if (Cy_BLE_GetAdvertisementState() != CY_BLE_ADV_STATE_ADVERTISING)
            {
                /* On advertisement timeout, switch between URI/URL and
                *  TLM packets. */
                if( (beaconCurrentRole == EDDYSTONE_UID) ||
                    (beaconCurrentRole == EDDYSTONE_URL) )
                {
                    beaconCurrentRole = EDDYSTONE_TLM;
                }
                else if(beaconCurrentRole == EDDYSTONE_TLM)
                {
                    beaconCurrentRole = EDDYSTONE_IMPLEMENTATION;
                }
                
                /* Configure Eddystone packets for the selected implementation */
                ConfigureAdvPacket();
                
                /* Restart advertisement */
                Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_CUSTOM,
                                CY_BLE_BROADCASTER_CONFIGURATION_0_INDEX);	       
            }
            /* Advertisement started */
            else
            {
                /* If the beacon role switched to UID/URL, a complete URL/UID + TLM 
                   frames have been broadcasted */
                if( (beaconCurrentRole == EDDYSTONE_UID) ||
                    (beaconCurrentRole == EDDYSTONE_URL) )
                {
                    /* Increment advertisement packet count */
                    packetCount.count+=PACKETS_PER_BROADCAST;
                }    
            }
            break;
            
        /* Disconnect event */
        case CY_BLE_EVT_GAP_DEVICE_DISCONNECTED:
            
            /* Configure Eddystone packets for the default implementation */
            beaconCurrentRole = EDDYSTONE_IMPLEMENTATION;
            ConfigureAdvPacket();
            
            /* Restart advertisement */
            Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_CUSTOM,
                            CY_BLE_BROADCASTER_CONFIGURATION_0_INDEX);
            break;
            
        /* Do nothing for all other events */
        default:
            break;
    }
}

/*******************************************************************************
* Function Name: void ProcessBleEvents(void)
********************************************************************************
* Summary:
*  Function that continuously process the BLE events 
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void ProcessBleEvents(void)
{
    /* Process event callback to handle BLE events. The events generated 
	   and used for this application are inside the 'StackEventHandler' 
       routine */
    Cy_BLE_ProcessEvents();
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
*******************************************************************************/
void InitBle(void)
{
    /* Start the BLE component and register the stack event handler */
    Cy_BLE_Start(StackEventHandler);
}

/* [] END OF FILE */
