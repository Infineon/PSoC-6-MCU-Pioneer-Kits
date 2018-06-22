/******************************************************************************
* File Name: led.c
*
* Version: 1.10
*
* Description: This file contains functions that handle the initialization and
*              control of the RGB LED and the status LEDs
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
* This file contains functions that handle the initialization and control of 
* the RGB LED and the status LEDs
*******************************************************************************/

/* Header file includes */
#include <project.h>
#include "led.h"

/* Macros used to turn off the individual RGB LEDs */
#define TurnOffRGBRed   Cy_TCPWM_Counter_SetCompare0(PWM_Red_TCPWM__HW, \
                        PWM_Red_TCPWM__CNT_IDX, RGB_LED_OFF)
#define TurnOffRGBGreen Cy_TCPWM_Counter_SetCompare0(PWM_Green_TCPWM__HW, \
                        PWM_Green_TCPWM__CNT_IDX, RGB_LED_OFF)
#define TurnOffRGBBlue  Cy_TCPWM_Counter_SetCompare0(PWM_Blue_TCPWM__HW, \
                        PWM_Blue_TCPWM__CNT_IDX, RGB_LED_OFF)

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
#define DISCONNECT_DELAY             (uint8_t) (0x06u)

/* Macros used to scale the RGB color component values so that they fall within
   the linear range of Pseudo Random PWM compare values */
/* Pseudo Random PWMs that drive Red and Green LEDs are configured as 16 bit 
   and the Pseudo Random PWM that drives the Blue LED is configured as 32 bit 
   so that they have direct HSIOM connections to the RGB LED pins on the
   Pioneer kit. Therefore, two different scaling values are used */
#define COLOR_COMPONENT_SCALER_RG  (uint8_t) (0x01u)
#define COLOR_COMPONENT_SCALER_B   (uint8_t) (0x10u)
/* Threshold below which RGB color mixing is skipped */
#define LED_NO_COLOR_THRESHOLD     (uint8_t) (0x04u)

/* Flags that control the behavior of status LEDs */
bool static toggleConnectionLED = false;
bool static disconnectLEDActive = false;

/*******************************************************************************
* Function Name: void SetColorRgb(uint8_t *colorInfo)
********************************************************************************
* Summary:
*  Changes the RGB LED color to the specified value
*
* Parameters:
*  uint8_t* colorInfo : pointer to a 4-byte array holding the color information
*
* Return:
*  void
*
*******************************************************************************/
void SetColorRgb(uint8_t* colorInfo)
{
    /* Local variables to calculate the color components from the received RGB
       data*/
    uint32_t    redComponent;
    uint32_t    greenComponent;
    uint32_t    blueComponent;

    /* If the Intensity value sent by the client is below a set threshold, then 
	   turn off the LEDs */
    if (colorInfo[INTENSITY_INDEX] < LED_NO_COLOR_THRESHOLD)
    {
        TurnOffRGBRed;
        TurnOffRGBGreen;
        TurnOffRGBBlue;
    }
    else
    {
        /* If the individual color values of Red, Green and Blue components are 
		   less than the predefined threshold, then turn off the LEDs */
        if ((colorInfo[RED_INDEX] < LED_NO_COLOR_THRESHOLD) &&
            (colorInfo[GREEN_INDEX] < LED_NO_COLOR_THRESHOLD) &&
            (colorInfo[BLUE_INDEX] < LED_NO_COLOR_THRESHOLD))
        {
            TurnOffRGBRed;
            TurnOffRGBGreen;
            TurnOffRGBBlue;
        }
        else
        {
            /* Calculate the intensity of the Red, Green and Blue components 
        	   from the received 4-byte data */
            redComponent   = (colorInfo[RED_INDEX] * colorInfo
                              [INTENSITY_INDEX]) >> COLOR_COMPONENT_SCALER_RG;
            greenComponent = (colorInfo[GREEN_INDEX] * colorInfo
                              [INTENSITY_INDEX]) >> COLOR_COMPONENT_SCALER_RG;
            blueComponent  = (colorInfo[BLUE_INDEX] * colorInfo
                              [INTENSITY_INDEX]) << COLOR_COMPONENT_SCALER_B;
            
            /* Update the compare value of the PWMs for color control */
            Cy_TCPWM_Counter_SetCompare0(PWM_Red_TCPWM__HW,
                                   PWM_Red_TCPWM__CNT_IDX, redComponent);
            Cy_TCPWM_Counter_SetCompare0(PWM_Green_TCPWM__HW,
                                   PWM_Green_TCPWM__CNT_IDX, greenComponent);
            Cy_TCPWM_Counter_SetCompare0(PWM_Blue_TCPWM__HW,
                                   PWM_Blue_TCPWM__CNT_IDX, blueComponent);
        }
    }
}

/*******************************************************************************
* Function Name: void InitRgb(void)
********************************************************************************
* Summary:
*  Initializes the RGB LED by starting the PWM components
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void InitRgb(void)
{
    /* Start the PWM components for LED control*/
    PWM_Red_Start();
    PWM_Green_Start();
    PWM_Blue_Start();
}

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
            if (disconnectDelay >= DISCONNECT_DELAY)
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
