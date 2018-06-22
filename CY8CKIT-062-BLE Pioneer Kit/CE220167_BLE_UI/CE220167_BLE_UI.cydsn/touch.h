/******************************************************************************
* File Name: touch.h
*
* Version: 1.10
*
* Description: This file is the public interface of touch.c source file
*
* Related Document: CE220167_BLE_UI.pdf
*
* Hardware Dependency: CY8CKIT-062-BLE PSoC 6 BLE Pioneer Kit
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
* This file contains the declaration of functions provided by the
* touch.c file 
********************************************************************************/

/* Include guard */
#ifndef TOUCH_H
#define TOUCH_H

/* Header file includes */
#include <project.h>
 
/* For more details on the data format of CapSense buttons and slider required
   for Cypress's Custom BLE Profiles, see Cypress CapSense profile specification
   available at:
   http://www.cypress.com/documentation/software-and-drivers/cypresss-custom-ble-profiles-and-services
*/    
/* Data length of CapSense button and slider */
#define CAPSENSE_BUTTON_DATA_LEN    (uint8_t) (0x03u)
#define CAPSENSE_SLIDER_DATA_LEN    (uint8_t) (0x01u)

/* Data-type for storing CapSense touch information */    
typedef struct
{
    /* Flags that track changes to CapSense data */
    bool sliderDataUpdated;
    bool buttonDataUpdated;
    /* Slider and button data fields */
    uint8_t  sliderData;
    uint8_t  buttonData[CAPSENSE_BUTTON_DATA_LEN];
}   touch_data_t;

/* Function that initializes the CapSense */    
void InitCapSense(void);

/* Function that scans CapSense sensors, processes touch information and then
   returns the touch information */
touch_data_t* GetcapSenseData(void);

/* Function to check if CapSense is ready to enter low power mode */
bool  IsCapSenseReadyForLowPowerMode(void);

#endif /* TOUCH_H */
/* [] END OF FILE */
