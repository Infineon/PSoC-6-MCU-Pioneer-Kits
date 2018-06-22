/******************************************************************************
* File Name: main_cm0p.c
*
* Version: 1.10
*
* Description: This code example shows how to create a user-interface 
*              solution using an EINK display and CapSense.
*
* Related Document: CE218133_EINK_CapSense.pdf
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
* This code example demonstrates how to create a user-interface solution
* using EINK display and CapSense sliders and buttons. EINK displays consume 
* no power for image retention. However, during display update, the CPU has to 
* be active for as many as two seconds, which consumes CPU cycles and increases 
* the average power  consumption. PSoC 6 BLE has an ARM Cortex-M0+ companion core 
* that can rectify these issues as it consumes very low power and takes processing 
* overhead away from the main ARM Cortex-M4 CPU. Together with PSoC 6 BLE’s 
* CapSense touch sensing, EINK display can create user interfaces that have 
* “always-on” functionality.
*******************************************************************************/

/* Header file includes */
#include <project.h>
#include "touch.h"
#include "screen.h"

/*******************************************************************************
* Function Name: int main()
********************************************************************************
*
* Summary: 
*  Main function that continuously reads touch information and updates the screen
*  accordingly
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
    /* Variable used to store the touch information */
    touch_data_t touchData;

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize the display, touch detection and low power modules */
    InitScreen();
    InitTouch();

    for (;;)
    {
        /* Read the touch information from the CapSense buttons and the slider */
        touchData = GetTouch();

        /* Update the screen according to the touch input */
        UpdateScreen(touchData);
    }
}

/* [] END OF FILE */
