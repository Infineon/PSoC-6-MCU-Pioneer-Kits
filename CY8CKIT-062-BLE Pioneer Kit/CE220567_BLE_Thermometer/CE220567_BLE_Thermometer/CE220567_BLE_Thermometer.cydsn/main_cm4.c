/******************************************************************************
* File Name: main_cm4.c
*
* Version: 1.00
*
* Description: This code example demonstrates BLE Health Thermometer Service
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
/*****************************************************************************
* This code example demonstrates interfacing PSoC 6 MCU with BLE Connectivity 
* (PSoC 6 MCU) with a thermistor circuit to read temperature information and 
* send the temperature information using BLE Health Thermometer Service 
* indications to a mobile device running CySmart mobile application
*******************************************************************************/

/* Header file includes */
#include "temperature.h"
#include "display.h"
#include "low_power.h"
#include "ble_application.h"

/*******************************************************************************
* Function Name: int main()
********************************************************************************
*
* Summary: 
*  Main function that initializes the system, and continuously processes BLE
*  events. The main function also refreshes the E-INK display and enters low
*  power modes if the conditions permit
*
* Parameters:
*  None
*
* Return:
*  int
*
* Side Effects:
*  None  
*
*******************************************************************************/
int main(void)
{ 
    /* Enable global interrupts */
    __enable_irq();
    
    /* Initialize the E-INK display, temperature sensing, low power, and 
       BLE modules */
    InitDisplay();
    InitTemperature();
    InitLowPower();
    InitBle();
     
    /* Refresh the E-INK display to show the instructions to evaluate this
       code example */
    RefreshDisplay();

    for (;;)
    {
        /* Process the BLE events and send indication when required */
        ProcessBleEvents();
        
        /* Refresh the E-INK display with temperature information at
           regular intervals, if the BLE is not connected */
        RefreshDisplay();
      
        /* Turn off the LEDs and enter low power modes if the conditions 
           permit */
        HandleLowPowerMode();
    }
}

/* [] END OF FILE */
