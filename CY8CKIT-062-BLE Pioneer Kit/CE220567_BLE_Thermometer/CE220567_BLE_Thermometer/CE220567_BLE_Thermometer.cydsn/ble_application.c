/******************************************************************************
* File Name: ble_application.c
*
* Version: 1.00
*
* Description: This file contains functions that handle BLE stack and 
*              the health thermometer service
*
* Related Document: CE220567_BLE_Thermometer.pdf
*
* Hardware Dependency: CY8CKIT-062-BLE PSoC 6 BLE Pioneer Kit
*                      CY8CKIT-028-EPD E-INK Display Shield
*
******************************************************************************
* Copyright (2017), Cypress Semiconductor Corporation.
******************************************************************************
* This software, including source code, documentation and related materials
* ("Software") is owned by Cypress Semiconductor Corporation (Cypress) and is
* protected by and subject to worldwide patent protection (United States and
* foreign), United States copyright laws and international treaty provisions.
* Cypress hereby grants to licensee a personal, non-exclusive, non-transferable
* license to copy, use, modify, create derivative works of, and compile the
* Cypress source code and derivative works for the sole purpose of creating
* custom software in support of licensee product, such licensee product to be
* used only in conjunction with Cypress's integrated circuit as specified in the
* applicable agreement. Any reproduction, modification, translation, compilation,
* or representation of this Software except as specified above is prohibited
* without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes to the Software without notice.
* Cypress does not assume any liability arising out of the application or use
* of Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use as critical components in any products
* where a malfunction or failure may reasonably be expected to result in
* significant injury or death ("ACTIVE Risk Product"). By including Cypress's
* product in a ACTIVE Risk Product, the manufacturer of such system or application
* assumes all risk of such use and in doing so indemnifies Cypress against all
* liability. Use of this Software may be limited by and subject to the applicable
* Cypress software license agreement.
*****************************************************************************/
/*******************************************************************************
* This file contains functions that handle BLE stack and the Health Thermometer
* Service
*******************************************************************************/

/* Header file includes */
#include <math.h>
#include "ble_application.h"
#include "temperature.h"
#include "led.h"

/* Declaration of Health Thermometer Service (HTS) specific macros.
   GATT specifications of the Health Thermometer  service can be found at:
   https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.service.health_thermometer.xml
*/

/* Total size of the Health Thermometer Service characteristic array */
#define HTS_CHARACTERISTIC_SIZE     (uint8_t) (5u)
/* Size of the temperature value in the Health Thermometer Service 
   characteristic array */
#define HTS_TEMPERATURE_DATA_SIZE   (uint8_t) (4u)
/* Index of the temperature value in the Health Thermometer Service 
   characteristic array */
#define HTS_TEMPERATURE_DATA_INDEX  (uint8_t) (1u)

/* Macros used to convert from IEEE-754 single precision floating 
   point format to IEEE-11073 FLOAT with two decimal digits of precision */
#define IEEE_11073_MANTISSA_SCALER  (uint8_t) (100u)
#define IEEE_11073_EXPONENT_VALUE   (int8_t)  (-2)
#define IEEE_11073_EXPONENT_INDEX   (uint8_t) (3u)

/* Data-type used to store temperature as an IEEE-11073 FLOAT value as well as
   access it as an array of bytes for BLE operations */
typedef union 
{
    int32_t temeratureValue;
    int8_t  temperatureArray[HTS_TEMPERATURE_DATA_SIZE];
} temperature_data_t;

/* Flags used to request the main BLE processing loop of various actions   */
bool    requestBleAdvertisement =   false;
bool    requestHtsIndication    =   false;
bool    requestTemperatureScan  =   false;

/* These variable store BLE connection parameters */
cy_stc_ble_conn_handle_t connectionHandle;

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
    /* Handle the BLE events */
    switch(event)
    {
        /**********************************************************
        *                       Generic and HCI Events
        ***********************************************************/
        case CY_BLE_EVT_STACK_ON: 
            /* Request to start advertisement */
            requestBleAdvertisement = true;
            break;

        /**********************************************************
        *                       GATT Events
        ***********************************************************/
        case CY_BLE_EVT_GATT_CONNECT_IND:
            /* Get the connection handle */
            connectionHandle = *(cy_stc_ble_conn_handle_t *)eventParam;
            break;

        /**********************************************************
        *                       Other Events
        ***********************************************************/
        default:
    		break;
    }
}

/*******************************************************************************
* Function Name: void CallBackHts(uint32 event, void *eventParam)
********************************************************************************
*
* Summary:
*  This is an event callback function to receive events from the BLE Component,
*  which are specific to Health Thermometer Service.
*
* Parameters:  
*  event:       Event for Health Thermometer Service.
*  eventParams: Event parameter for Health Thermometer Service.
*
* Return: 
*  None
*
*******************************************************************************/
void CallBackHts(uint32 event, void *eventParam)
{
    /* Remove warning for unused parameter */
    (void)eventParam;
    
    /* Handle the HTS events */
    switch(event)
    {
        /* This event is received when indication are enabled by the central */
        case CY_BLE_EVT_HTSS_INDICATION_ENABLED:
            /* Set the requestHtsIndication flag */
            requestHtsIndication = true;
            break;

        /* This event is received when indication are enabled by the central */
        case CY_BLE_EVT_HTSS_INDICATION_DISABLED:
            /* Reset the requestHtsIndication flag */
            requestHtsIndication = false;
            break;
        
        /* Do nothing for all other events */
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
    
    /* Temporary array to hold Health Thermometer Characteristic information */
    uint8 valueArray[HTS_CHARACTERISTIC_SIZE];
    float temperature;
    temperature_data_t tempData;

    /* Process event callback to handle BLE events. The events generated 
	   and used for this application are inside the 'StackEventHandler' 
       routine */
    Cy_BLE_ProcessEvents();

    /* BLE is connected */
	if(Cy_BLE_GetConnectionState(connectionHandle) 
       == CY_BLE_CONN_STATE_CONNECTED)
	{
        /* Update the status LEDs if they're not indicating the connection
        already */
        if(bleStatusIndication != INDICATE_CONNECTION)
        {
            bleStatusIndication = INDICATE_CONNECTION;
            UpdateStatusLEDs(bleStatusIndication);
        }
        /* Check if temperature data needs to be read and notification needs to
           be sent */
        if (requestHtsIndication && requestTemperatureScan)
        {
            /* Read the temperature value */
            temperature = GetTemperature();
            
            /* Convert from IEEE-754 single precision floating point format to
               IEEE-11073 FLOAT, which is mandated by the health thermometer
               characteristic */
            tempData.temeratureValue = (int32_t)(roundf(temperature*
                                                IEEE_11073_MANTISSA_SCALER));
            tempData.temperatureArray[IEEE_11073_EXPONENT_INDEX] = 
                                                IEEE_11073_EXPONENT_VALUE;         
            
            /* Read Health Thermometer Characteristic from GATT DB */
            if(CY_BLE_SUCCESS == Cy_BLE_HTSS_GetCharacteristicValue
                                 (CY_BLE_HTS_TEMP_MEASURE,
                                  HTS_CHARACTERISTIC_SIZE, valueArray))
            { 
                /* Update temperature value in the characteristic */
                memcpy(&valueArray[HTS_TEMPERATURE_DATA_INDEX],
                       tempData.temperatureArray, HTS_TEMPERATURE_DATA_SIZE);

                /* Send indication to the central */
                Cy_BLE_HTSS_SendIndication(connectionHandle, 
                                           CY_BLE_HTS_TEMP_MEASURE,
                                           HTS_CHARACTERISTIC_SIZE, valueArray);
            }
            /* Reset the requestTemperatureScan flag */
            requestTemperatureScan = false;
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
        /* Advertisement time out. Turn off all indications */
        else if (bleStatusIndication == INDICATE_ADVERTISEMENT)
        {   
            bleStatusIndication = NO_INDICATION;
            UpdateStatusLEDs(bleStatusIndication);
        }
        else
        {
            bleStatusIndication = NO_INDICATION;
            UpdateStatusLEDs(bleStatusIndication);
        }
        
         /* Check if the advertisement needs to be restarted */
    	if(requestBleAdvertisement)
    	{
    		/* Reset 'requestBleAdvertisement' flag */
    		requestBleAdvertisement = false;
    		
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
    
    /* Register the Health Thermometer Service specific callback handler */
    Cy_BLE_HTS_RegisterAttrCallback(CallBackHts);
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
    requestBleAdvertisement = true;
}

/*******************************************************************************
* Function Name: void RequestTemperatureScan(void)
********************************************************************************
*
* Summary:
*  Function that requests the main BLE loop to read the temperature
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
void RequestTemperatureScan(void)
{
    requestTemperatureScan = true;
}

/*******************************************************************************
* Function Name: bool IsBleconnected(void)
********************************************************************************
*
* Summary:
*  Function to check if the BLE is in a connected state
*
* Parameters:
*  None
*
* Return:
*  bool     : true if connected, false otherwise
*
* Side Effects:
*  None
*
*******************************************************************************/
bool  IsBleconnected(void)
{
    /* Variable that stores the return flag */
    bool connectionState = false;
    
    /* Get the connection state */
    if(Cy_BLE_GetNumOfActiveConn() != 0u)
    {
        connectionState=true;
    }
    
    /* Return  the low power mode entry readiness */
    return connectionState;
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
