/******************************************************************************
* File Name: touch.c
*
* Version: 1.10
*
* Description: This file contains functions that handle the initialization and
*              the scanning of the CapSense Slider and the Buttons
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
* This file contains functions that handle the initialization and the scanning of 
* the CapSense Slider and the Buttons
*******************************************************************************/

/* Header file includes */
#include <project.h>
#include "touch.h"

/* For more details on the data format of CapSense buttons and slider required
   for Cypress's Custom BLE Profiles, see Cypress CapSense profile specification
   available at:
   http://www.cypress.com/documentation/software-and-drivers/cypresss-custom-ble-profiles-and-services
*/

/* CapSense Button macros */
/* Header (position 0)contains the number of buttons */
#define BUTTON_DATA_HEADER  (uint8_t) (0x02u)
#define BUTTON_HEADER_POS   (uint8_t) (0x00u)
/* Buttons masks store the touch data  */
#define BUTTON0_MASK        (uint8_t) (0x01u)
#define BUTTON1_MASK        (uint8_t) (0x02u)
#define BUTTON_MASK_POS0    (uint8_t) (0x02u)
#define BUTTON_MASK_POS1    (uint8_t) (0x01u)
/* Macro used to clear button masks */
#define BUTTON_MASK_CLEAR   (uint8_t) (0x00u)

/* Slider position value received from the CapSense Component when the slider is 
   not touched */
#define NO_SLIDER_TOUCH   	(uint8_t) (0xFFu)

/*******************************************************************************
* Function Name: touch_data_t* GetcapSenseData(void)
********************************************************************************
* Summary:
*  Function that scans CapSense sensors, processes touch information and then
*  returns the touch information
*
* Parameters:
*  None
*
* Return:
*  touch_data_t*  : address of the data-structure that stores touch 
*                   information
*
*******************************************************************************/
touch_data_t* GetcapSenseData(void)
{
    /* Structure that stores the current CapSense touch information */
    touch_data_t  static currentCapSenseData = 
    {    
        /* Initialize the flags that track updates to touch information */
        .buttonDataUpdated = false,
        .sliderDataUpdated = false,
        /* Initialize the button and slider touch information */
        .sliderData = NO_SLIDER_TOUCH,
        .buttonData[BUTTON_HEADER_POS]  = BUTTON_DATA_HEADER,
        .buttonData[BUTTON_MASK_POS0]   = BUTTON_MASK_CLEAR,
        .buttonData[BUTTON_MASK_POS1]   = BUTTON_MASK_CLEAR
    };
        
    /* Variables used to store the button and slider touch information */
    uint8_t  static buttonMask      = BUTTON_MASK_CLEAR;
    uint8_t  static sliderPosition  = NO_SLIDER_TOUCH;

    /* Do this only when the CapSense isn't busy with a previous operation */
    if (CapSense_IsBusy() == CapSense_NOT_BUSY)
    {
        /* Process data from both the slider and the buttons */
		CapSense_ProcessAllWidgets();
        
        /* Read the finger position on the slider */
        sliderPosition = (uint8_t)CapSense_GetCentroidPos
                                    (CapSense_LINEARSLIDER0_WDGT_ID);
                                               
        /* Clear the button masks to acquire new data */
        buttonMask = BUTTON_MASK_CLEAR;
                            
        /* Get the touch status of button 0 */
        if (CapSense_IsWidgetActive(CapSense_BUTTON0_WDGT_ID))
        {
            buttonMask |= BUTTON0_MASK;
        }          
        /* Get the touch status of button 1 */
        if (CapSense_IsWidgetActive(CapSense_BUTTON1_WDGT_ID))
        {
            buttonMask |= BUTTON1_MASK;
        }                    

        /* Start the next CapSense scan */
        CapSense_ScanAllWidgets();
    }
                      
    /* Check if the finger position on the slider has changed */
    if (sliderPosition != currentCapSenseData.sliderData)
    {
        /* Slider touch position */
        currentCapSenseData.sliderData = (uint8_t)sliderPosition;
        /* Slider data updated */
        currentCapSenseData.sliderDataUpdated = true;
    }
    else
    {  
       /* Slider data not updated */
       currentCapSenseData.sliderDataUpdated = false; 
    }
    /* Check for a change in the button mask */
    if (buttonMask != currentCapSenseData.buttonData[BUTTON_MASK_POS0])
    {
        /* Button mask for BUTTON0 and BUTTON1 */
        currentCapSenseData.buttonData[BUTTON_MASK_POS0] = buttonMask;
        /* Button data updated */
        currentCapSenseData.buttonDataUpdated = true;
    }
    else
    {
        /* Button data not updated */
        currentCapSenseData.buttonDataUpdated = false; 
    }
    
    /* return the touch information */
    return &currentCapSenseData;
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
*  Initializes CapSense touch sensing
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
