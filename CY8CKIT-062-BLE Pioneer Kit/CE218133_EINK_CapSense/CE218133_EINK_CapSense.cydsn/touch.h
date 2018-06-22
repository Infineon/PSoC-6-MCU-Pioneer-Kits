/******************************************************************************
* File Name: touch.h
*
* Version: 1.10
*
* Description: This file declares the functions provided by the touch.c file
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
* This file declares the functions provided by the touch.c file
*******************************************************************************/

/* Include Guard */
#ifndef TOUCH_H
#define TOUCH_H

/* Header file includes */    
#include <project.h>    

/* Total number of touch types */
#define NUMBER_OF_TOUCH_TYPES (uint8_t)(0x05u)
    
/* Enumerated data type for different types of CapSense touch. Integer values 
   are added so that this data type can be used to as a parameter to function 
   pointers */
typedef enum
{
    NO_TOUCH        = 0x00u,
    SELECT_BUTTON   = 0x01u,
    BACK_BUTTON     = 0x02u,
    SLIDER_LEFT     = 0x03u,
    SLIDER_RIGHT    = 0x04u
}   touch_data_types_t;

/* Data type that stores touch information */
typedef struct
{
    touch_data_types_t  touchType;
    bool                scanBusy;
}   touch_data_t;

/* Function that initializes the CapSense touch sensing */
void       InitTouch(void);

/* Function to read and analyze the touch information from the CapSense sensors */
touch_data_t    GetTouch(void);

#endif
/* [] END OF FILE */
