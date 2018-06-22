/******************************************************************************
* File Name: main_cm4.c
*
* Version: 1.10
*
* Description: This file contains the functions to initialize the RTC, set
*              time and date, read the time and date as strings and to 
*              generate alarm interrupts at one minute intervals to refresh
*              the EINK display.
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
* This file contains the functions to initialize the RTC, set time and date, 
* read the time and date as strings and to generate alarm interrupts at one 
* minute intervals to refresh the EINK display.
*******************************************************************************/

/* Header file includes */
#include <project.h>
#include <stdio.h>
#include "real_time_clock.h"
#include "ble_application.h"
#include "display.h"

/* Default time written to RTC at power-up or reset */
#define TIME_AT_RESET           (00u),   /* Seconds    */\
                                (00u),   /* Minutes    */\
                                (00u),   /* Hours      */\
                                (01u),   /* Date       */\
                                (01u),   /* Month      */\
                                (17u)    /* Year 20xx  */
                                
/* Shift by 8 to move a data to the higher byte */
#define SHIFT_TO_HIGHER_BYTE    (8u)

/* Structure that enables alarm interrupts at 1 minute intervals, i.e when the 
   seconds field rolls over to "00". Note that only the seconds field is enabled. 
   Other fields are kept at the default values and not enabled */                                
cy_stc_rtc_alarm_t const alarm = 
{
    .sec            =   00u,
    .secEn          =   CY_RTC_ALARM_ENABLE,
    .min            =   00u,
    .minEn          =   CY_RTC_ALARM_DISABLE,
    .hour           =   00u,
    .hourEn         =   CY_RTC_ALARM_DISABLE,
    .dayOfWeek      =   01u,
    .dayOfWeekEn    =   CY_RTC_ALARM_DISABLE,
    .date           =   01u,
    .dateEn         =   CY_RTC_ALARM_DISABLE,
    .month          =   01u,
    .monthEn        =   CY_RTC_ALARM_DISABLE,
    .almEn          =   CY_RTC_ALARM_ENABLE
};

/*******************************************************************************
* Function Name: void Cy_RTC_Alarm1Interrupt(void)
********************************************************************************
*
* Summary:
*  This function is the handler for the RTC Alarm1 Interrupt. It clears the 
*  interrupt and schedules a display refresh
*
* Parameters:
*  None
*
* Return:
*  None
*
**********************************************************************************/
void Cy_RTC_Alarm1Interrupt(void)
{   
    /* Clear any pending interrupts */
    Cy_RTC_ClearInterrupt(CY_RTC_INTR_ALARM1);
    __NVIC_ClearPendingIRQ(RTC_RTC_IRQ_cfg.intrSrc);
    
    /* Schedule a display refresh at the next available slot */
    EnableDisplayRefresh();
}

/*******************************************************************************
* Function Name: void InitRtc(void)
********************************************************************************
*
* Summary:
*  This function initializes the RTC, loads default time and date values and 
*  enables the alarm interrupt
*
* Parameters:
*  None
*
* Return:
*  None
*
**********************************************************************************/
void InitRtc(void)
{
    /* Start the RTC */
    RTC_Start();  
    
    /* Clear any pending interrupts */
    Cy_RTC_ClearInterrupt(CY_RTC_INTR_ALARM1);
    __NVIC_ClearPendingIRQ(RTC_RTC_IRQ_cfg.intrSrc);
    
    /*Configures the source (Alarm1) that trigger the interrupts */
    Cy_RTC_SetInterruptMask(CY_RTC_INTR_ALARM1);

    /* Set the default date and time and wait till the operation is successful */
    while(Cy_RTC_SetDateAndTimeDirect(TIME_AT_RESET) != CY_RET_SUCCESS);
     
    /* Wait for alarm to be set */
    while(Cy_RTC_SetAlarmDateAndTime(&alarm,CY_RTC_ALARM_1) != CY_RET_SUCCESS);
}

/*******************************************************************************
* Function Name: void SetRtc(time_and_date_t* timeAndDate)
********************************************************************************
*
* Summary:
*  This function writes the time and date specified in the parameter to the RTC
*  and schedules a display refresh
*
* Parameters:
*  time_and_date_t* : structure that contains time and date
*
* Return:
*  None
*
**********************************************************************************/
void SetRtc(time_and_date_t* timeAndDate)
{
    /* Wait until the writing operation is successful */
    while(Cy_RTC_SetDateAndTimeDirect
               (timeAndDate->seconds,
                timeAndDate->minutes,
                timeAndDate->hours,
                timeAndDate->day,
                timeAndDate->month,
                (((uint32_t)timeAndDate->yearHigh 
                            << SHIFT_TO_HIGHER_BYTE | timeAndDate->yearLow)
                            - CY_RTC_TWO_THOUSAND_YEARS))
        != CY_RET_SUCCESS);
    
    /* Schedule a display refresh at the next available slot */
    EnableDisplayRefresh();
}

/*******************************************************************************
* Function Name: time_date_strings_t* GetRtcStrings(void)
********************************************************************************
*
* Summary:
*  This function returns a structure that contains current time and date strings.
*  Date is returned in "mm/dd/yyyy" format and time is in "hh:mm" format
*
* Parameters:
*  time_and_date_t* : structure that contains time and date strings
*
* Return:
*  None
*
**********************************************************************************/
time_date_strings_t* GetRtcStrings(void)
{
    /* Variable that stores date&time read from the RTC */
    cy_stc_rtc_config_t dateTime;
    
    /* Variable that stores converted strings */
    time_date_strings_t   static dateTimeStrings;

    /* Get Date and Time from RTC registers */
    Cy_RTC_GetDateAndTime(&dateTime);

    /* Print Date info in mm/dd/yyyy format */
    sprintf(dateTimeStrings.dateString, "%02u/%02u/20%02u", (uint16_t) dateTime.month,
           (uint16_t) dateTime.date, (uint16_t) dateTime.year);
    
    /* Print Time in hh:mm:ss format */
    sprintf(dateTimeStrings.timeSting, "%02u:%02u", (uint16_t) dateTime.hour, 
            (uint16_t) dateTime.min);
    
    /* Return the converted strings */
    return &dateTimeStrings;
}

/* [] END OF FILE */
