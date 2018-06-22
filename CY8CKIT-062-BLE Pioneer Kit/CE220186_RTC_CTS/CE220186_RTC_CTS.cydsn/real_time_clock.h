/******************************************************************************
* File Name: real_time_clock.h
*
* Version: 1.10
*
* Description: This file declares the functions provided by the 
*              real_time_clock.c source file
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
* This file declares the functions provided by the real_time_clock.c 
* source file
*******************************************************************************/

/* Include Guard */
#ifndef REAL_TIME_CLOCK_H
#define REAL_TIME_CLOCK_H
 
/* Header file includes */
#include <project.h>

/* Character size of a time string in HH:MM format */    
#define TIME_STRING_SIZE    (06u)
/* Character size of a date string in MM/DD/YYYY format */  
#define DATE_STRING_SIZE    (11u)

/* Reuse the BLE CTS time and date structure for RTC APIs */    
typedef cy_stc_ble_cts_current_time_t time_and_date_t; 
    
/* Data-type that stores time and date as strings */
typedef struct 
{
    char timeSting[TIME_STRING_SIZE];
    char dateString[DATE_STRING_SIZE];
}   time_date_strings_t;    

/* Function that initializes the real time clock */
void InitRtc(void);

/* Function that writes times and date to RTC  */
void SetRtc(time_and_date_t* timeAndDate);

/* Function to read current RTC values as strings */
time_date_strings_t*  GetRtcStrings(void);

#endif
/* [] END OF FILE */
