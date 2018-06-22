/******************************************************************************
* File Name: main_cm4.c
*
* Version: 1.10
*
* Description: This project demonstrates accurate time keeping with PSoC 6 
*              BLE’s real time clock (RTC), which also generates alarms 
*              (interrupts) every one minute to show time information on an 
*              EINK display. In addition, a BLE current time service (CTS) is 
*              used to synchronize time and date with a current time server 
*              such as an iPhone.
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
* This project demonstrates accurate time keeping with PSoC 6 BLE’s real time 
* clock (RTC), which also generates alarms (interrupts) every one minute to 
* show time information on an EINK display. In addition, a BLE current time 
* service (CTS) is used to synchronize time and date with a current time server
* such as an iPhone.
*
* Note: This project requires an iOS device with iOS 8 or later to evaluate. 
*       Android devices do not support current time service. 
*******************************************************************************/

/* Header file includes */
#include "display.h"
#include "low_power.h"
#include "real_time_clock.h"
#include "ble_application.h"

/*******************************************************************************
* Function Name: int main()
********************************************************************************
*
* Summary: 
*  Main function that initializes the system, and continuously processes BLE
*  events. The main function also refreshes the EINK display and enters low
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
    
    /* Initialize the EINK display, RTC, low power, and BLE modules */
    InitDisplay(); 
    InitRtc();
    InitLowPower();
    InitBle();
    
    /* Refresh the EINK display to show the instructions and the default 
       time and date */
    RefreshDisplay();
        
    for (;;)
    {
        /* Process the BLE events */
        ProcessBleEvents();
        
        /* Refresh the EINK display if required */
        RefreshDisplay();
      
        /* Turn off the LEDs and enter low power modes if the conditions 
           permit */
        HandleLowPowerMode();
    }
}

/* [] END OF FILE */
