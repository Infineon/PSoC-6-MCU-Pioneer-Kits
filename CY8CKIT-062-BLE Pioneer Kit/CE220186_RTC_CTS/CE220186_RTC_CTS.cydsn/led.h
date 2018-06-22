/******************************************************************************
* File Name: LED.h
*
* Version: 1.10
*
* Description: This file declares the functions provided by the led.c 
*              source file 
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
* This file declares the functions provided by the LED.c source file 
********************************************************************************/

/* Include guard */
#ifndef LED_H
#define LED_H

/* Header file includes */
#include <project.h>

/* Data-type of status LED indications */
typedef enum
{   
    NO_INDICATION,
    INDICATE_ADVERTISEMENT,
    INDICATE_CONNECTION,
    INDICATE_DISCONNECTION,
}   status_led_indication_t;

/* Function that updates the status LEDs to indicate the BLE state */
void UpdateStatusLEDs(status_led_indication_t indication);

/* Function that services the toggling / timeout of the status LEDs */
void ServiceStatusLEDs (void);

/* Function to check if any of the status LEDs require periodic toggle */
bool IsLEDperiodicToggleActive(void);

#endif /* LED_H */
/* [] END OF FILE */
