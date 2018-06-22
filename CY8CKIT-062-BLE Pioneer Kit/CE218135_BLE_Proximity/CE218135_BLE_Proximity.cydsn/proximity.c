/******************************************************************************
* File Name: proximity.c
*
* Version: 1.10
*
* Description: This file contains functions that handle the initialization and
*              the scanning of the CapSense Proximity and the Buttons
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
* This file contains functions that handle the initialization and the scanning of 
* the CapSense Proximity and the Buttons
*******************************************************************************/

/* Header file includes */
#include <project.h>
#include "proximity.h"

/* Maximum value of CapSense proximity sensor data accepted by the CySmart app */
#define PROX_MAX_VALUE  (uint16_t)(0x00FFu)

/*******************************************************************************
* Function Name: proximity_data_t* proximity_data_t* GetProximityData(void)
********************************************************************************
* Summary:
*  Function that scans CapSense proximity sensor, processes information and then
*  returns the data
*
* Parameters:
*  None
*
* Return:
*  proximity_data_t*  : address of the data-structure that stores proximity 
*                       information
*
*******************************************************************************/
proximity_data_t* GetProximityData(void)
{
    /* Structure that stores the current CapSense proximity information */
    proximity_data_t  static currentProximityData = 
    {    
        /* Initialize the flag that track updates to proximity information */
        .proximityDataUpdated = false,
        /* Initialize the proximity information */
        .proximityData = 0u,
    };
        
    /* Variables used to store the instantaneous proximity information */
    uint8_t  static proximity  = 0u;

    /* Do this only when the CapSense isn't busy with a previous operation */
    if (CapSense_IsBusy() == CapSense_NOT_BUSY)
    {
        /* Process data from the proximity widget */
		CapSense_ProcessAllWidgets();
        
        /* Check if the signal is greater than the finger threshold */
        if(CapSense_PROXIMITY0_SNS0_DIFF_VALUE > 
           CapSense_PROXIMITY0_FINGER_TH_VALUE)
        {
            /*If proximity value is not within the range, cap the value */
    		if((CapSense_PROXIMITY0_SNS0_DIFF_VALUE -
                CapSense_PROXIMITY0_FINGER_TH_VALUE) 
                <= PROX_MAX_VALUE)
    		{
                /* Store the proximity data */
    	         proximity =  (uint8_t)(CapSense_PROXIMITY0_SNS0_DIFF_VALUE - 
                                        CapSense_PROXIMITY0_FINGER_TH_VALUE);
    		}
            /* Store the maximum value otherwise */
            else
            {
                proximity = (uint8_t)PROX_MAX_VALUE;
            }
        }
        /* Clear the value if the signal is less than the finger threshold */
        else
        {
           proximity = 0u; 
        }
        
        /* Start the next CapSense scan */
        CapSense_ScanAllWidgets();
    }
                      
    /* Check if the proximity data has changed */
    if (proximity != currentProximityData.proximityData)
    {
        /* Proximity proximity position */
        currentProximityData.proximityData = proximity;
        /* Proximity data updated */
        currentProximityData.proximityDataUpdated = true;
    }
    else
    {  
       /* Proximity data not updated */
       currentProximityData.proximityDataUpdated = false; 
    }
        
    /* return the proximity information */
    return &currentProximityData;
}

/*******************************************************************************
* Function Name: bool IsCapSenseReadyForLowPowerMode(void)
********************************************************************************
*
* Summary:
*  Function to check if CapSense is ready to enter low power mode
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
bool  IsCapSenseReadyForLowPowerMode(void)
{
    /* Variable that stores the return flag */
    bool lowPowerModeReady;
    
    /* Don't enter low power mode if CapSense is busy with a scan */
    if(CapSense_IsBusy() != CapSense_NOT_BUSY)
    {
        lowPowerModeReady=false;
    }
    else
    {
        lowPowerModeReady=true;
    }
    
    /* Return  the low power mode entry readiness */
    return lowPowerModeReady;
}

/*******************************************************************************
* Function Name: void InitCapSense(void)
********************************************************************************
* Summary:
*  Initializes CapSense proximity sensing
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void InitCapSense(void)
{
    /* Start the CapSense component and initialize the baselines */
    CapSense_Start();

}

/* [] END OF FILE */
