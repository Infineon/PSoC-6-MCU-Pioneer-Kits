/******************************************************************************
* File Name: LED.c
*
* Version: 1.00
*
* Description: This file contains functions that handle the initialization and
*              control of the status LEDs
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
* This file contains functions that handle the initialization and control of 
* the status LEDs
*******************************************************************************/

/* Header file includes */
#include <project.h>
#include "led.h"

/* Macros used to control the individual status LEDs */   
#define TurnOnOrange    Cy_GPIO_Write(Pin_LED_Orange_0_PORT,\
                        Pin_LED_Orange_0_NUM, 0)
#define TurnOnRed       Cy_GPIO_Write(Pin_LED_Red_0_PORT,\
                        Pin_LED_Red_0_NUM, 0)
#define TurnOffOrange   Cy_GPIO_Write(Pin_LED_Orange_0_PORT,\
                        Pin_LED_Orange_0_NUM, 1)
#define TurnOffRed      Cy_GPIO_Write(Pin_LED_Red_0_PORT,\
                        Pin_LED_Red_0_NUM, 1)

/* Time for which the disconnect LED turns on = 6*0.5 seconds = 3 seconds */
#define DISCONNECT_DELAY (uint8_t) (0x06u)

/* Flags that control the behavior of status LEDs */
bool static toggleConnectionLED = false;
bool static disconnectLEDActive = false;

/*******************************************************************************
* Function Name: void UpdateStatusLEDs(status_led_indication_t indication)
********************************************************************************
* Summary:
*  Update status LEDs based on the current BLE status
*
* Parameters:
*  indication   : BLE status that needs to be indicated
*
* Return:
*  void
*
*******************************************************************************/
void UpdateStatusLEDs(status_led_indication_t indication)
{    
    /* Update status LEDs based on the BLE indication status */
    switch (indication)
    {   
        /* If BLE is advertising, turn off the red LED that shows disconnection 
           and turn on the orange LED */
        case INDICATE_ADVERTISEMENT:
            TurnOnOrange;
            TurnOffRed;
            /* Disable toggling the orange LED at periodic intervals and disable
               disconnect LED timer (both controlled by the MCWDT 
               interrupt) */
            toggleConnectionLED = false;
            disconnectLEDActive = false;
            break;
        /* If BLE connection is active, continuously toggle the orange LED at 
           periodic intervals (controlled by the MCWDT interrupt), and turn 
           off the disconnect LED */
        case INDICATE_CONNECTION:
            TurnOffOrange;
            TurnOffRed;
            toggleConnectionLED = true;
            disconnectLEDActive = false;
            break;
         /* If BLE connection is active, continuously toggle the orange LED at 
            periodic intervals (controlled by the MCWDT interrupt), and turn 
            off the disconnect LED */
        case INDICATE_DISCONNECTION:
            TurnOffOrange;
            TurnOnRed;
            toggleConnectionLED = false;
            disconnectLEDActive = true;
            break;
        /* Turn off orange LED and its toggling flag. Note that red LED will be
           turned off when the timer expires (controlled by the MCWDT 
           interrupt) */
        case NO_INDICATION:
            TurnOffOrange;
            toggleConnectionLED = false;
            break;
       /* Do nothing for invalid inputs */
       default:
            break;
    }
}

/*******************************************************************************
* Function Name: void ServiceStatusLEDs(void)
********************************************************************************
* Summary:
*  This functions is called every 0.5 second to service the toggling / timeout
*  of the status LEDs
*
* Parameters:
*  None
*
* Return:
*  void
*
*******************************************************************************/
void ServiceStatusLEDs(void)
{
    /* Counter used to calculate the duration during which the disconnect LED
       remains on */
    static uint8 disconnectDelay = 0u;
    
    /* Variable used to periodically toggle the connection LED  */
    static bool toggleLEDFlag = false;
    
    /* Invert the state of the connection LED  */
    if (toggleConnectionLED)
        {
            if (toggleLEDFlag)
            {
                TurnOffOrange;
                toggleLEDFlag = false;
            }
            else
            {
                TurnOnOrange;
                toggleLEDFlag = true;
            }
        }
    /* Calculate the time for which the disconnect LED was on, and turn the LED 
       off if time duration exceeds the specified delay */    
    if (disconnectLEDActive)
        {
            disconnectDelay++;
            if (disconnectDelay>=DISCONNECT_DELAY)
            {
               disconnectDelay =0u;
               disconnectLEDActive = false;
               TurnOffRed;
            }
        }
}

/*******************************************************************************
* Function Name: bool IsLEDperiodicToggleActive(void)
********************************************************************************
*
* Summary:
*  Function to check if any of the status LEDs require periodic toggle
*
* Parameters:
*  None
*
* Return:
*  bool     : true if the periodic toggle is required
*             false otherwise
*
* Side Effects:
*  None
*
*******************************************************************************/
bool IsLEDperiodicToggleActive(void)
{
    /* Return true if disconnect LED or connection LED requires
       periodic toggle */
    return (disconnectLEDActive||toggleConnectionLED);
}

/* [] END OF FILE */
