/******************************************************************************
* File Name: main_cm4.c
*
* Version: 1.10
*
* Description: This project demonstrates a BLE Eddystone beacon that broadcasts
*              the core Eddystone beacon frames (URL, UID and TLM)
*
* Related Document: CE220335_BLE_Eddystone.pdf
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
* This project demonstrates a BLE Eddystone beacon that broadcasts the core 
* Eddystone beacon frames (URL, UID and TLM). 
* 
* The Eddystone-UID frame broadcasts a unique 16-byte Beacon ID composed of a 
* 10-byte name-space and a 6-byte instance.  
*
* The Eddystone-URL frame broadcasts a URL using a compressed encoding format 
* in order to fit more within the limited advertisement packet.
* 
* The Eddystone-TLM frame broadcasts telemetry information about the beacon 
* itself such as battery voltage, device temperature, and counts of broadcast 
* packets.
*
* This project broadcasts the URL frames by default, with interleaved TLM 
* frames. To change the Eddystone packet settings, see the header file
* eddystone_config.h
*******************************************************************************/

/* Header file includes */
#include "ble_application.h"
#include "display.h"

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  System entrance point. This function calls the initializing functions and
*  shows the instructions to use this code example on the E-INK display. The main
*  function then continuously processes BLE events
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main()
{
	/* Enable global interrupts */
	__enable_irq(); 
           
    /* Initialize the components used for E-INK display and BLE */
	InitDisplay();
    InitBle();

    /* Display the instructions to evaluate this code example on the E-INK 
       display */
    DisplayInstructions();
    	
    for(;;)
    {
        /* Continuously process BLE events */
        ProcessBleEvents();
        
        /* Put Cortex-M4 to Deep-Sleep mode. BLE / MCWDT will wake up the CPU if
           processing is required */
        Cy_SysPm_DeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
    }	
}

/* [] END OF FILE */
